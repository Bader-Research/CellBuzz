#ifndef __minigzip_h__
#define __minigzip_h__

#include <stdio.h>

#define SPU_THREADS 16
#define CACHE_LINE_SIZE 128
#define CACHE_LINE_OFFSET_BITS 7/* log2( CACHE_LINE_SIZE ) */

#if 1
#define ERROR_MESSAGE( x ) printf x
#else
#define ERROR_MESSAGE( x )
#endif

#if 1
#define OUTPUT_MESSAGE( x ) printf x
#else
#define OUTPUT_MESSAGE( x )
#endif

#define GZ_SUFFIX ".gz"
#define SUFFIX_LEN ( sizeof( GZ_SUFFIX ) - 1 )
#define MAX_FILE_NAME_LEN CACHE_LINE_SIZE
#define GZ_TRAILER_SIZE 8

#define CELL_GZIP_MAX_BLOCKS 65535
#define CELL_GZIP_MAX_BLOCK_NO_DIGITS 5
#define CELL_GZIP_MIN_BLK_SIZE ( 100 * 1024 )/* 100 KB */
#define CELL_GZIP_MAX_BLK_SIZE ( 900 * 1024 )/* 900 KB */

#define SPE_RUNNABLE 0
#define SPE_WAITING 1
#define SPE_ASSIGNING_NB 2
#define SPE_DONE 3

typedef struct _control_block_t {
	unsigned int id;
	unsigned int block_no;
	unsigned int cell_gzip;
	unsigned int last;
	unsigned int transparent;
	unsigned int addr_read_data;
	unsigned int read_data_size;
	unsigned int addr_write_buf;/* only for cell gzip format */
	unsigned int write_buf_size;/* only for cell gzip format */
	unsigned int addr_ret_info;
	unsigned int addr_spe_status;
	char a_outmode[20];
	char a_file_name[MAX_FILE_NAME_LEN];
	char a_pad[CACHE_LINE_SIZE - ( sizeof( unsigned int ) * 11 ) - 20];/* pad to a full cache line */
} control_block_t;

typedef struct _gz_ret_info_t {
	unsigned int block_no;
	unsigned int success;
	unsigned int crc;
	unsigned int in_size;
	unsigned int out_size;
	char a_pad[CACHE_LINE_SIZE - 5 * sizeof( unsigned int )];/* pad to a full cache line */
} gz_ret_info_t;

#endif/* __minigzip_h__ */

