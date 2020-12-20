/* INCLUDES */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/stat.h>
#ifdef PERFORMANCE_MEASURE
#include <sys/time.h>
#endif
#include <sys/types.h>

#include <libspe2.h>
#include <malloc_align.h>
#include <free_align.h>

#include "zlib.h"

#include "../minigzip.h"

#define BUF_FREE 0
#define BUF_BUSY 1

#define NUM_READ_BUF ( 8 * SPU_THREADS )/* must be larger than 2 * SPU_THREADS */
#define NUM_WRITE_BUF ( 8 * SPU_THREADS )/* must be larger than SPU_THREADS */

#define DATA_BUF_SIZE ( 64 * 1024 )/* 64KB buffer */

#define MAIN_THREAD_SLEEP_US 1000/* sleep 1 ms when blocked for read or write */
#define FIO_THREAD_SLEEP_US 10000/* sleep 10 ms to avoid busy waiting */

/* EXTERNAL VARIABLES */

extern spe_program_handle_t minigzip_spu_compress;
extern spe_program_handle_t minigzip_spu_decompress;

/* TYPEDEFS */

typedef struct _block_info_t {
	unsigned int crc;
	unsigned int offset;
	unsigned int in_size;
	unsigned int out_size;
	int read_buf_index;
	unsigned int read_buf_size;
	int write_buf_index;/* only for cell gzip format */
	unsigned int write_buf_size;/* only for cell gzip format */
	unsigned int read_done;
	unsigned int write_done;
} block_info_t;

typedef struct _input_args_t {
	unsigned int uncompr;
	char compress_mode;/* for compress */
	char compress_level;/* for compress */
	unsigned int num_threads;
	unsigned int blk_size;/* for compress */
	char a_file_name[MAX_FILE_NAME_LEN];
} input_args_t;

typedef struct _spe_status_t{
	unsigned int status;
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int )];
} spe_status_t;

typedef struct _fio_thread_args_t {
	input_args_t* p_input_args;
	FILE* p_in;
	FILE* p_out;
	unsigned int next_fread_block_no;
	unsigned int num_blocks;
} fio_thread_args_t;

typedef struct _ppu_pthread_data_t {
	spe_context_ptr_t spuid;
	pthread_t pthread;
	void* argp;
} ppu_pthread_data_t;
/* STATIC VARIABLES */

static control_block_t cb;
static control_block_t a_cb[SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

static volatile gz_ret_info_t a_ret_info[SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
static volatile spe_status_t a_spe_status[SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

static block_info_t volatile a_block_info[CELL_GZIP_MAX_BLOCKS];
static void* ap_read_buf[NUM_READ_BUF];
static int a_read_buf_status[NUM_READ_BUF];
static void* ap_write_buf[NUM_WRITE_BUF];
static int a_write_buf_status[NUM_WRITE_BUF];
static sem_t rw_buf_lock;

static unsigned char a_buf[DATA_BUF_SIZE];

/* STATIC FUNCTION PROTOTYPES */

static int parse_input_args( int argc, char* argv[], input_args_t* p_input_args );
static int set_decompress_block( unsigned int header_offset, gzip_header_extra_t* p_header_extra, size_t file_size );
static int set_compress_block( unsigned int blk_size, size_t file_size );
static int init_read_buf( unsigned int buf_size );
static int alloc_read_buf();
static void free_read_buf( int index );
static void term_read_buf();
static int init_write_buf( unsigned int buf_size );
static int alloc_write_buf();
static void free_write_buf( int index );
static void term_write_buf();
static void* fio_read_thread_func( void* p_arg );
static void* fio_write_thread_func( void* p_arg );

static void* ppu_pthread_function( void* p_arg );

/* GLOBAL FUNCTIONS */

int main( int argc, char* argv[] ) {
	ppu_pthread_data_t a_pthread_data[SPU_THREADS];
	spe_program_handle_t* p_spu_prog;
	pthread_t fio_read_thread;
	pthread_t fio_write_thread;
	fio_thread_args_t fio_thread_args;

	input_args_t input_args;
	int header_offset;
	gzip_header_extra_t header_extra;
	unsigned int crc;
	unsigned int in_size;
	unsigned int out_size;
	unsigned int num_blocks;
	unsigned int read_buf_size;
	unsigned int write_buf_size;

	char a_outfile[MAX_FILE_NAME_LEN];
	FILE* p_in;
	FILE* p_out;
	fpos_t pos;
	int fd;
	struct stat file_stat;
	void* p_mmap_addr;
	int len;

	unsigned int spe_completion_mask;
	unsigned int next_block_no;

	int ret;
	unsigned int mbox_data;
	unsigned int i;

#ifdef PERFORMANCE_MEASURE
	struct timeval tv0;
	struct timeval tv1;
	struct timeval tv2;
	struct timeval tv3;
	struct timeval a_tv3_s[SPU_THREADS];
	struct timeval a_tv3_e[SPU_THREADS];
	struct timeval tv4;
	struct timeval tv5;
	struct timeval tv6;
	struct timeval tv7;
	struct timeval tv8;
	unsigned int read_block_count = 0;
	unsigned int write_block_count = 0;
#endif

	argc--, argv++;

	/* init data structure */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv0, NULL );
#endif

	memset( &cb, 0x00, sizeof( control_block_t ) );
	memset( a_cb, 0x00, SPU_THREADS * sizeof( control_block_t ) );

	/* parse argc, argv */

	parse_input_args( argc, argv, &input_args );
	strcpy( cb.a_outmode, "ab  " );
	cb.a_outmode[2] = input_args.compress_level;
	cb.a_outmode[3] = input_args.compress_mode;
	if( strlen( input_args.a_file_name ) + 1 > sizeof( cb.a_file_name ) ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] file name(%s) too long\n", input_args.a_file_name ) );
		exit( 1 );
	}
	strcpy( cb.a_file_name, input_args.a_file_name );

	/* open input file */

	fd = open( cb.a_file_name, O_RDONLY );
	if( fd == -1 ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] open failure\n" ) );
		exit( 1 );
	}

	p_in = fdopen( fd, "rb" );
	if( p_in == NULL ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fopen failure\n" ) );
		exit( 1 );
	}

	/* open output file */

	if( input_args.uncompr == 1 ) {/* decompress */
		if( strlen( input_args.a_file_name ) + 1 > sizeof( a_outfile ) ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] file name(%s) too long\n", input_args.a_file_name ) );
			exit( 1 );
		}
		sprintf( a_outfile, "%s", input_args.a_file_name );
		len = strlen( cb.a_file_name );
		a_outfile[len - SUFFIX_LEN] = ( char )'\0';
	}
	else {/* compress */
		if( strlen( input_args.a_file_name ) + strlen( GZ_SUFFIX ) + 1 > sizeof( a_outfile ) ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] file name(%s) too long\n", input_args.a_file_name ) );
			exit( 1 );
		}
		sprintf( a_outfile, "%s%s", cb.a_file_name, GZ_SUFFIX );
	}
	
	p_out = fopen( a_outfile, "wb" );
	if( p_out == NULL ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fopen(%s) failure, errno=%d\n", a_outfile, errno ) );
		exit( 1 );
	}

	/* mmap file */

	if( fstat( fd, &file_stat ) != 0 ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fstat failure, errno=%d\n", errno ) );
		exit( 1 );
	}

	p_mmap_addr = mmap( 0, file_stat.st_size, PROT_READ, MAP_PRIVATE, fd, 0 );
	if( p_mmap_addr == MAP_FAILED ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] mmap failure\n" ) );
		exit( 1 );
	}

	/* process gzip header */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv1, NULL );
#endif

	header_offset = 0;/* to avoid compiler warning */

	if( input_args.uncompr == 1 ) {/* decompress */
		header_offset = check_header( ( unsigned char* )( p_mmap_addr ), file_stat.st_size, &header_extra );
		if( header_offset == -1 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] check_header failure\n" ) );
			exit( 1 );
		}

		num_blocks = set_decompress_block( header_offset, &header_extra, file_stat.st_size );
	}
	else {/* compress */
		num_blocks = set_compress_block( input_args.blk_size, file_stat.st_size );
		if( input_args.blk_size == 0 ) {
			header_extra.transparent = 0;
			header_extra.extra_block_info_present = 0;
		}
		else {
			header_extra.transparent = 0;
			header_extra.extra_block_info_present = 1;
			header_extra.num_cell_gzip_block = num_blocks;
			/* header_extra.a_cell_gzip_block_size[i] had dummy value, should be updated later */
		}

		header_offset = write_header( a_buf, DATA_BUF_SIZE, &header_extra );
		if( header_offset == -1 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] write_header failure\n" ) );
		}

		fwrite( a_buf, 1, header_offset, p_out );
		if( ferror( p_out ) ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fwrite failure, errno=%d\n", errno ) );
			exit( 1 );
		}
	}
	cb.transparent = header_extra.transparent;

	if( num_blocks > CELL_GZIP_MAX_BLOCKS ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] too many blocks\n" ) );
		exit( 1 );
	}
	else if( input_args.num_threads > num_blocks ) {
		input_args.num_threads = num_blocks;
	}

	/* update cb.cell_gzip */

	cb.cell_gzip = 0;
	if( input_args.uncompr == 1 ) {
		if( header_extra.extra_block_info_present == 1 ) {
			cb.cell_gzip = 1;
		}
	}
	else {
		if( input_args.blk_size != 0 ) {
			cb.cell_gzip = 1;
		}
	}

	/* init input/output buffer */

	write_buf_size = 0;/* to avoid compiler warning */
	if( cb.cell_gzip == 1 ) {
		if( sem_init( &rw_buf_lock, 0, 1 ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] sem_init failure\n" ) );
			exit( 1 );
		}
		if( input_args.uncompr == 1 ) {
			read_buf_size = gzip_bound( CELL_GZIP_MAX_BLK_SIZE );
			read_buf_size += ( 16 - ( read_buf_size & 0xf ) ) & 0xf;/* should be multiple of 16 to avoid DMA error */
			write_buf_size = CELL_GZIP_MAX_BLK_SIZE;
		}
		else {
			read_buf_size = input_args.blk_size;
			write_buf_size = gzip_bound( input_args.blk_size );
			write_buf_size += ( 16 - ( write_buf_size & 0xf ) ) & 0xf;/* should be multiple of 16 to avoid DMA error */
		}
		if( init_read_buf( read_buf_size ) == -1 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] init_read_buf failure\n" ) );
			exit( 1 );
		}
		if( init_write_buf( write_buf_size ) == -1 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] init_write_buf failure\n" ) );
			exit( 1 );
		}
	}

	/* copy cb to a_cb and set trailer return address and last */

	for( i = 0 ; i < input_args.num_threads ; i++ ) {
		a_cb[i] = cb;
		a_cb[i].id = i;
		a_cb[i].addr_ret_info = ( unsigned int )( &( a_ret_info[i] ) );
		a_cb[i].addr_spe_status = ( unsigned int )( &( a_spe_status[i].status ) );
	}

	/* assign buf for initial work and read initial data */

	if( cb.cell_gzip == 1 ) {
		if( input_args.uncompr == 1 ) {
			if( fseek( p_in, header_offset, SEEK_SET ) != 0 ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fseek failure\n" ) );
				exit( 1 );
			}
		}

		for( i = 0 ; i < input_args.num_threads ; i++ ) {
			/* read */

			a_block_info[i].read_buf_index = alloc_read_buf();
			if( a_block_info[i].read_buf_index == -1 ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] alloc_read_buf failure\n" ) );
				exit( 1 );
			}
			if( fread( ap_read_buf[a_block_info[i].read_buf_index], 1, a_block_info[i].in_size, p_in ) != a_block_info[i].in_size ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fread size too small\n" ) );
				exit( 1 );
			}
			else if( ferror( p_in ) ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fread failure, errno=%d\n", errno ) );
				exit( 1 );
			}
			else {
				a_block_info[i].read_done = 1;
			}

			/* write */
		
			a_block_info[i].write_buf_index = alloc_write_buf();
			if( a_block_info[i].write_buf_index == -1 ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] alloc_write_buf failure\n" ) );
				exit( 1 );
			}
		}
	}

	/* assign initial work loads */

	spe_completion_mask = 0;
	next_block_no = 0;
	if( cb.cell_gzip == 1 ) {
		for( i = 0 ; i < input_args.num_threads ; i++ ) {
			spe_completion_mask |= ( 1 << i );
			a_cb[i].block_no = next_block_no;
			a_cb[i].addr_read_data = ( unsigned int )( ap_read_buf[a_block_info[next_block_no].read_buf_index] );
			a_cb[i].read_data_size = a_block_info[next_block_no].in_size;
			a_cb[i].addr_write_buf = ( unsigned int )( ap_write_buf[a_block_info[next_block_no].write_buf_index] );
			a_cb[i].write_buf_size = write_buf_size;
			if( next_block_no == num_blocks - 1 ) {
				a_cb[i].last = 1;
			}
			else {
				a_cb[i].last = 0;
			}
			a_spe_status[i].status = SPE_RUNNABLE;
			next_block_no++;
		}
	}
	else {
		spe_completion_mask = 1;
		a_cb[0].block_no = next_block_no;
		if( input_args.uncompr == 1 ) {
			a_cb[0].addr_read_data = ( unsigned int )( p_mmap_addr + header_offset );
		}
		else {
			a_cb[0].addr_read_data = ( unsigned int )p_mmap_addr;
		}
		a_cb[0].read_data_size = file_stat.st_size;
		a_cb[0].last = 1;
		a_spe_status[0].status = SPE_RUNNABLE;
		next_block_no++;
	}

	/* create file io thread */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv2, NULL );
#endif

	if( cb.cell_gzip == 1 ) {
		fio_thread_args.p_input_args = &input_args;
		fio_thread_args.p_in = p_in;
		fio_thread_args.p_out = p_out;
		fio_thread_args.next_fread_block_no = input_args.num_threads;
		fio_thread_args.num_blocks = num_blocks;

		if( pthread_create( &fio_read_thread, NULL, fio_read_thread_func, ( void* )( &fio_thread_args ) ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] pthread_create failure\n" ) );
			exit( 1 );
		}

		if( pthread_create( &fio_write_thread, NULL, fio_write_thread_func, ( void* )( &fio_thread_args ) ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] pthread_create failure\n" ) );
			exit( 1 );
		}
	}
	else {
		fclose( p_out );
	}

	/* get system info (# of SPEs) */
	
#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv3, NULL );
#endif

	if( ( spe_cpu_info_get( SPE_COUNT_PHYSICAL_SPES, -1 ) < ( int )input_args.num_threads ) || ( spe_cpu_info_get( SPE_COUNT_PHYSICAL_SPES, -1 ) > SPU_THREADS ) ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] not enough availabe SPEs, num_threads=%d\n", input_args.num_threads ) );
		exit( 1 );
	}

	/* create SPE threads */

	if( input_args.uncompr == 1 ) {/* decompress */
		p_spu_prog = &minigzip_spu_decompress;
	}
	else {
		p_spu_prog = &minigzip_spu_compress;
	}

	for( i = 0 ; i < input_args.num_threads ; i++ ) {
#ifdef PERFORMANCE_MEASURE
		gettimeofday( &a_tv3_s[i], NULL );
#endif
		a_pthread_data[i].spuid = spe_context_create( 0, NULL );
		if( a_pthread_data[i].spuid == NULL ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] spe_context_create failure\n" ) );
                        exit( 1 );
                }

		ret = spe_program_load( a_pthread_data[i].spuid, p_spu_prog );
		if( ret != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] spe_program_load failure\n" ) );
			exit( 1 );
		}

		a_pthread_data[i].argp = &( a_cb[i] );

		if( pthread_create( &( a_pthread_data[i].pthread ), NULL, &ppu_pthread_function, &( a_pthread_data[i] ) ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] pthread_create failure\n" ) );
			exit( 1 );
		}
#ifdef PERFORMANCE_MEASURE
		gettimeofday( &a_tv3_e[i], NULL );
#endif
	}

	/* distribute work load to each threads */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv4, NULL );
#endif

	while( spe_completion_mask != 0 ) {
		sched_yield();
		for( i = 0 ; i < input_args.num_threads ; i++ ) {
			if( a_spe_status[i].status == SPE_WAITING ) {/* update finished block info */ 
				__asm__ __volatile__ ("sync" : : : "memory");

				if( a_ret_info[i].success == 0 ) {
					ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] SPE thread %d failure\n", i ) );
					exit( 1 );
				}

				a_block_info[a_ret_info[i].block_no].crc = a_ret_info[i].crc;
				if( a_block_info[a_ret_info[i].block_no].in_size != a_ret_info[i].in_size ) {
					ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] size error\n" ) );
					exit( 1 );
				}
				a_block_info[a_ret_info[i].block_no].out_size = a_ret_info[i].out_size;
				if( cb.cell_gzip == 1 ) {
					if( a_ret_info[i].block_no != a_cb[i].block_no ) {
						fprintf( stderr, "block_no inconsist, %d, %d\n", a_ret_info[i].block_no, a_cb[i].block_no );
						printf( "block_no inconsist, %d, %d\n", a_ret_info[i].block_no, a_cb[i].block_no );
					}
					a_block_info[a_ret_info[i].block_no].write_done = 1;
					free_read_buf( a_block_info[a_ret_info[i].block_no].read_buf_index );
				}
				a_spe_status[i].status = SPE_ASSIGNING_NB;
			}

			if( a_spe_status[i].status == SPE_ASSIGNING_NB ) {/* assign next block */
				if( next_block_no < num_blocks ) {/* if cb.cell_gzip == 0, this condition cannot be met */
					if( ( a_block_info[next_block_no].read_done != 0 ) && ( ( a_block_info[next_block_no].write_buf_index = alloc_write_buf() ) != -1 ) ) {
						a_cb[i].block_no = next_block_no;
						a_cb[i].addr_read_data = ( unsigned int )( ap_read_buf[a_block_info[next_block_no].read_buf_index] );
						a_cb[i].read_data_size = a_block_info[next_block_no].in_size;
						a_cb[i].addr_write_buf = ( unsigned int )( ap_write_buf[a_block_info[next_block_no].write_buf_index] );
						a_cb[i].write_buf_size = write_buf_size;
						if( next_block_no == num_blocks - 1 ) {
							a_cb[i].last = 1;
						}
						else {
							a_cb[i].last = 0;
						}
						a_spe_status[i].status = SPE_RUNNABLE;
						mbox_data = SPE_RUNNABLE;
						spe_in_mbox_write( a_pthread_data[i].spuid, &mbox_data, 1, SPE_MBOX_ANY_NONBLOCKING );

						next_block_no++;
					}
					else {
#ifdef PERFORMANCE_MEASURE
						if( a_block_info[next_block_no].read_done == 0 ) {
							read_block_count++;
						}
						else {
							write_block_count++;
						}
#endif
						usleep( MAIN_THREAD_SLEEP_US );
					}
				}
				else {
					a_spe_status[i].status = SPE_DONE;
					mbox_data = SPE_DONE;
					spe_in_mbox_write( a_pthread_data[i].spuid, &mbox_data, 1, SPE_MBOX_ANY_NONBLOCKING );
					spe_completion_mask &= ~( 1 << i );
				}
			}
		}
	}

	/* wait for SPE threads to complete execution */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv5, NULL );
#endif

	for( i = 0 ; i < input_args.num_threads ; i++ ) {
		if( pthread_join( a_pthread_data[i].pthread, NULL ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] pthread_join failure\n" ) );
			exit( 1 );
		}

		if( spe_context_destroy( a_pthread_data[i].spuid ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] spe_context_destroy failure\n" ) );
			exit( 1 );
		}
	}

	/* wait for file io thread to complete execution */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv6, NULL );
#endif

	if( cb.cell_gzip == 1 ) {
		if( pthread_join( fio_read_thread, NULL ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] pthread_join failure\n" ) );
			exit( 1 );
		}

		if( pthread_join( fio_write_thread, NULL ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] pthread_join failure\n" ) );
			exit( 1 );
		}
	}
	else {
		p_out = fopen( a_outfile, "ab" );
		if( p_out == NULL ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fopen(%s) failure, errno=%d\n", a_outfile, errno ) );
			exit( 1 );
		}
	}

	/* term input/output buffer */

	term_read_buf();
	term_write_buf();

	if( sem_destroy( &rw_buf_lock ) != 0 ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] sem_destroy failure\n" ) );
		exit( 1 );
	}

	/* update gzip header - compress */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv7, NULL );
#endif

	if( ( input_args.uncompr == 0 ) && ( input_args.blk_size != 0 ) ) {
		header_extra.transparent = 0;
		header_extra.extra_block_info_present = 1;
		header_extra.num_cell_gzip_block = num_blocks;
		for( i = 0 ; i < num_blocks ; i++ ) {
			header_extra.a_cell_gzip_block_size[i] = a_block_info[i].out_size;
		}

		if( write_header( a_buf, DATA_BUF_SIZE, &header_extra ) != header_offset ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] write_header returns invalid header_offset\n" ) );
			exit( 1 );
		}

		if( fgetpos( p_out, &pos ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fgetpos failure, errno=%d\n", errno ) );
			exit( 1 );
		}
		if( fseek( p_out, 0, SEEK_SET ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fseek failure, errno=%d\n", errno ) );
			exit( 1 );
		}

		fwrite( a_buf, 1, header_offset, p_out );
		if( ferror( p_out ) ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fwrite failure, errno=%d\n", errno ) );
			exit( 1 );
		}

		if( fsetpos( p_out, &pos ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fseek failure, errno=%d\n", errno ) );
			exit( 1 );
		}
	}

	/* process gzip trailer */

	crc = a_block_info[0].crc;
	in_size = a_block_info[0].in_size;
	out_size = a_block_info[0].out_size;
	for( i = 1 ; i < num_blocks ; i++ ) {
		if( input_args.uncompr == 1 ) {
			crc = crc32_combine( crc, a_block_info[i].crc, a_block_info[i].out_size );
		}
		else {
			crc = crc32_combine( crc, a_block_info[i].crc, a_block_info[i].in_size );
		}
		in_size += a_block_info[i].in_size;
		out_size += a_block_info[i].out_size;
	}

	if( input_args.uncompr == 1 ) {
		if( check_trailer( p_mmap_addr + header_offset + in_size, file_stat.st_size - header_offset - in_size, crc, out_size ) != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] check_trailer failure\n" ) );
			exit( 1 );
		}
	}
	else {
		len = write_trailer( a_buf, DATA_BUF_SIZE, crc, in_size );
		if( len != 0 ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] write_trailer failure\n" ) );
			exit( 1 );
		}
		fwrite( a_buf, 1, GZ_TRAILER_SIZE, p_out );
		if( ferror( p_out ) ) {
			ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] fwrite failure, errno=%d\n", errno ) );
			exit( 1 );
		}
	}

	/* unmap file */

	if( munmap( p_mmap_addr, file_stat.st_size ) != 0 ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] munmap failure\n" ) );
		exit( 1 );
	}

	/* close input file */

	fclose( p_in );

	close( fd );

	/* close output file */

	fclose( p_out );

	/* unlink file */
	
	unlink( cb.a_file_name );

	/* execution completed */

#ifdef PERFORMANCE_MEASURE
	gettimeofday( &tv8, NULL );
#endif

#ifdef PERFORMANCE_MEASURE
	OUTPUT_MESSAGE( ( "init=%ld\n", ( tv1.tv_sec * 1000000 + tv1.tv_usec ) - ( tv0.tv_sec * 1000000 + tv0.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "header processing=%ld\n", ( tv2.tv_sec * 1000000 + tv2.tv_usec ) - ( tv1.tv_sec * 1000000 + tv1.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "fio thred creation=%ld\n", ( tv3.tv_sec * 1000000 + tv3.tv_usec ) - ( tv2.tv_sec * 1000000 + tv2.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "spe thread creation=%ld\n", ( tv4.tv_sec * 1000000 + tv4.tv_usec ) - ( tv3.tv_sec * 1000000 + tv3.tv_usec ) ) );
	for( i = 0 ; i < input_args.num_threads ; i++ ) {
		OUTPUT_MESSAGE( ( "	thread %d create=%ld\n", i, ( a_tv3_e[i].tv_sec * 1000000 + a_tv3_e[i].tv_usec ) - ( a_tv3_s[i].tv_sec * 1000000 + a_tv3_s[i].tv_usec ) ) );
	}
	OUTPUT_MESSAGE( ( "(de)compress=%ld\n", ( tv5.tv_sec * 1000000 + tv5.tv_usec ) - ( tv4.tv_sec * 1000000 + tv4.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "spe thread completion=%ld\n", ( tv6.tv_sec * 1000000 + tv6.tv_usec ) - ( tv5.tv_sec * 1000000 + tv5.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "fio thread completion=%ld\n", ( tv7.tv_sec * 1000000 + tv7.tv_usec ) - ( tv6.tv_sec * 1000000 + tv6.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "term=%ld\n", ( tv8.tv_sec * 1000000 + tv8.tv_usec ) - ( tv7.tv_sec * 1000000 + tv7.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "total=%ld\n", ( tv8.tv_sec * 1000000 + tv8.tv_usec ) - ( tv0.tv_sec * 1000000 + tv0.tv_usec ) ) );
	OUTPUT_MESSAGE( ( "read_block_count=%u, write_block_count=%u\n", read_block_count, write_block_count ) );
#endif

	return 0;
}

static int parse_input_args( int argc, char* argv[], input_args_t* p_input_args ) {
	p_input_args->uncompr = 0;
	p_input_args->compress_mode = ( char )0;
	p_input_args->compress_level = '6';
	p_input_args->num_threads = spe_cpu_info_get( SPE_COUNT_PHYSICAL_SPES, -1 );
	p_input_args->blk_size = CELL_GZIP_MAX_BLK_SIZE;
	p_input_args->a_file_name[0] = '\0';
	
	while( argc > 0 ) {
		if( strcmp( *argv, "-d" ) == 0 ) {
			p_input_args->uncompr = 1;
		}
		else if( strcmp( *argv, "-f" ) == 0 ) {
			p_input_args->compress_mode = 'f';
		}
		else if( strcmp( *argv, "-h" ) == 0 ) {
			p_input_args->compress_mode = 'h';
		}
		else if (strcmp(*argv, "-r") == 0) {
			p_input_args->compress_mode = 'R';
		}
		else if( ( *argv )[0] == '-' && ( *argv )[1] >= '1' && ( *argv )[1] <= '9' && ( *argv )[2] == 0 ) {
			p_input_args->compress_level = ( *argv )[1];
		}
		else if( ( *argv )[0] == '-' && ( *argv )[1] == 't' ) {
			p_input_args->num_threads = atoi( ( argv[0] + 2 ) );
			if( p_input_args->num_threads < 1 || p_input_args->num_threads > SPU_THREADS ) {
				return -1;
			}
		}
		else if( ( *argv )[0] == '-' && ( *argv )[1] == 'b' ) {
			p_input_args->blk_size = atoi( ( argv[0] + 2 ) ) * CELL_GZIP_MIN_BLK_SIZE;
			if( ( p_input_args->blk_size != 0 ) && ( p_input_args->blk_size < CELL_GZIP_MIN_BLK_SIZE || p_input_args->blk_size > CELL_GZIP_MAX_BLK_SIZE ) ) {
				return -1;
			}
		}
		else {
			break;
		}
		argc--, argv++;
	}

	if( argc == 0 ) {
		return -1;
	}
	else if( strlen( *argv ) + 1 > sizeof( p_input_args->a_file_name ) ) {
		return -1;
	}

	strcpy( p_input_args->a_file_name, *argv );

	return 0;
}

static int set_decompress_block( unsigned int header_offset, gzip_header_extra_t* p_header_extra, size_t file_size ) {
	unsigned int num_blocks;
	unsigned int i;

	if( p_header_extra->transparent == 1 ) {
		a_block_info[0].offset = 0;
		a_block_info[0].in_size = file_size;
		a_block_info[0].read_done = 0;
		a_block_info[0].write_done = 0;
		return 1;
	}
	else if( p_header_extra->extra_block_info_present == 0 ) {
		a_block_info[0].offset = header_offset;
		a_block_info[0].in_size = file_size -header_offset - GZ_TRAILER_SIZE;
		a_block_info[0].read_done = 0;
		a_block_info[0].write_done = 0;
		return 1;
	}	

	num_blocks = p_header_extra->num_cell_gzip_block;
	for( i = 0 ; i < num_blocks ; i++ ) {
		if( i == 0 ) {
			a_block_info[0].offset = header_offset;
		}
		else {
			a_block_info[i].offset = a_block_info[i - 1].offset + a_block_info[i - 1].in_size;
		}
		a_block_info[i].in_size = p_header_extra->a_cell_gzip_block_size[i];
		a_block_info[i].read_done = 0;
		a_block_info[i].write_done = 0;
	}

	return num_blocks;
}

static int set_compress_block( unsigned int blk_size, size_t file_size ) {
	unsigned int remaining_bytes;
	unsigned int offset;
	int num_blocks;

	remaining_bytes = file_size;
	offset = 0;
	num_blocks = 0;

	if( blk_size == 0 ) {
		a_block_info[0].offset = 0;
		a_block_info[0].in_size = file_size;
		a_block_info[0].read_done = 0;
		a_block_info[0].write_done = 0;
		return 1;
	}

	while( remaining_bytes > 0 ) {
		a_block_info[num_blocks].offset = offset;
		if( remaining_bytes > blk_size ) {
			a_block_info[num_blocks].in_size = blk_size;
		}
		else {
			a_block_info[num_blocks].in_size = remaining_bytes;
		}
		a_block_info[num_blocks].read_done = 0;
		a_block_info[num_blocks].write_done = 0;
		remaining_bytes -= a_block_info[num_blocks].in_size;
		offset += a_block_info[num_blocks].in_size;
		num_blocks++;
	}

	return num_blocks;
}

static int init_read_buf( unsigned int buf_size ) {
	int i;

	for( i = 0 ; i < NUM_READ_BUF ; i++ ) {
		ap_read_buf[i] = _malloc_align( buf_size, CACHE_LINE_OFFSET_BITS );
		if( ap_read_buf[i] == NULL ) {
			return -1;
		}
		a_read_buf_status[i] = BUF_FREE;
	}

	return 0;
}

static int alloc_read_buf() {
	int i;

	sem_wait( &rw_buf_lock );
	for( i = 0 ; i < NUM_READ_BUF ; i++ ) {
		if( a_read_buf_status[i] == BUF_FREE ) {
			a_read_buf_status[i] = BUF_BUSY;
			sem_post( &rw_buf_lock );
			return i;
		}
	}
	sem_post( &rw_buf_lock );
	return -1;
}

static void free_read_buf( int index ) {
	sem_wait( &rw_buf_lock );
	a_read_buf_status[index] = BUF_FREE;
	sem_post( &rw_buf_lock );
}

static void term_read_buf() {
	int i;

	for( i = 0 ; i < NUM_READ_BUF ; i++ ) {
		_free_align( ap_read_buf[i] );
	}

	return;
}

static int init_write_buf( unsigned int buf_size ) {
	int i;

	for( i = 0 ; i < NUM_WRITE_BUF ; i++ ) {
		ap_write_buf[i] = _malloc_align( buf_size, CACHE_LINE_OFFSET_BITS );
		if( ap_write_buf[i] == NULL ) {
			return -1;
		}
		a_write_buf_status[i] = BUF_FREE;
	}

	return 0;
}

static int alloc_write_buf() {
	int i;

	sem_wait( &rw_buf_lock );
	for( i = 0 ; i < NUM_WRITE_BUF ; i++ ) {
		if( a_write_buf_status[i] == BUF_FREE ) {
			a_write_buf_status[i] = BUF_BUSY;
			sem_post( &rw_buf_lock );
			return i;
		}
	}
	sem_post( &rw_buf_lock );
	return -1;
}

static void free_write_buf( int index ) {
	sem_wait( &rw_buf_lock );
	a_write_buf_status[index] = BUF_FREE;
	sem_post( &rw_buf_lock );
}

static void term_write_buf() {
	int i;

	for( i = 0 ; i < NUM_WRITE_BUF ; i++ ) {
		_free_align( ap_write_buf[i] );
	}

	return;
}

static void* fio_read_thread_func( void* p_arg ) {
	fio_thread_args_t* p_fio_thread_args;
	unsigned int next_fread_block_no;
	int read_buf_index;

	p_fio_thread_args = ( fio_thread_args_t* )p_arg;
	next_fread_block_no = p_fio_thread_args->next_fread_block_no;

	while( 1 ) {
		usleep( FIO_THREAD_SLEEP_US );

		/* try pre-read */

		while( ( ( read_buf_index = alloc_read_buf() ) != -1 ) && ( next_fread_block_no != p_fio_thread_args->num_blocks ) ) {
			a_block_info[next_fread_block_no].read_buf_index = read_buf_index;
			if( fread( ap_read_buf[a_block_info[next_fread_block_no].read_buf_index], 1, a_block_info[next_fread_block_no].in_size, p_fio_thread_args->p_in ) != a_block_info[next_fread_block_no].in_size ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:fio_thread_fucn()] fread size too small\n" ) );
				exit( 1 );
			}
			else if( ferror( p_fio_thread_args->p_in ) ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:fio_thread_fucn()] fread failure, errno=%d\n", errno ) );
				exit( 1 );
			}
			else {
				a_block_info[next_fread_block_no].read_done = 1;
			}
			next_fread_block_no++;
			sched_yield();
		}

		if( next_fread_block_no == p_fio_thread_args->num_blocks ) {
			break;
		}
	}

	return NULL;
}

static void* fio_write_thread_func( void* p_arg ) {
	fio_thread_args_t* p_fio_thread_args;
	unsigned int next_fwrite_block_no;

	p_fio_thread_args = ( fio_thread_args_t* )p_arg;
	next_fwrite_block_no = 0;

	while( 1 ) {
		usleep( FIO_THREAD_SLEEP_US );

		/* try write to file */

		while( ( a_block_info[next_fwrite_block_no].write_done == 1 ) && ( next_fwrite_block_no != p_fio_thread_args->num_blocks ) ) {
			fwrite( ap_write_buf[a_block_info[next_fwrite_block_no].write_buf_index], 1, a_block_info[next_fwrite_block_no].out_size, p_fio_thread_args->p_out );
			if( ferror( p_fio_thread_args->p_out ) ) {
				ERROR_MESSAGE( ( "[minigzip_ppu.c:fio_write_thread_func()] fwrite failure, errno=%d\n", errno ) );
				exit( 1 );
			}
			free_write_buf( a_block_info[next_fwrite_block_no].write_buf_index );
			next_fwrite_block_no++;
			sched_yield();
		}

		if( next_fwrite_block_no == p_fio_thread_args->num_blocks ) {
			break;
		}
	}

	return NULL;
}

static void* ppu_pthread_function( void* p_arg ) {
        ppu_pthread_data_t* p_data = ( ppu_pthread_data_t* )p_arg;
        unsigned int entry;

        entry = SPE_DEFAULT_ENTRY;

        if( spe_context_run( p_data->spuid, &entry, 0, p_data->argp, NULL, NULL ) < 0 ) {
		ERROR_MESSAGE( ( "[minigzip_ppu.c:main()] spe_context_run failure\n" ) );
                exit( 1 );
        }

        pthread_exit( NULL );
}

