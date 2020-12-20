#include <stdio.h>
#include <sys/time.h>
#include <errno.h>
#include <sched.h>
#include <libspe2.h>
#include <malloc_align.h>
#include <free_align.h>
#include <pthread.h>
#include <dirent.h>

#include "../rc5.h"

#define LE2BE( le ) ( ( ( ( le ) & 0xff ) << 24 ) | ( ( ( le ) & 0xff00 ) << 8 ) | ( ( ( le ) & 0xff0000 ) >> 8 ) | ( ( ( le ) & 0xff000000 ) >> 24 ) )
#define BE2LE( be ) ( ( ( ( be ) & 0xff ) << 24 ) | ( ( ( be ) & 0xff00 ) << 8 ) | ( ( ( be ) & 0xff0000 ) >> 8 ) | ( ( ( be ) & 0xff000000 ) >> 24 ) )
#define ROTL(x,y) (((x)<<(y&(w-1))) | ((x)>>(w-(y&(w-1)))))


extern spe_program_handle_t rc5_encrypt;
extern spe_program_handle_t rc5_decrypt;


static control_block cb[NO_SPU] __attribute__ ((aligned(128)));

typedef struct ppu_pthread_data {
  spe_context_ptr_t spe_ctx;
  pthread_t pthread;
  void *argp;
} ppu_pthread_data_t;

//using namespace std;

unsigned int expanded_key_size;
//spe_gid_t gid;
//speid_t speids[NO_SPU];
//int status[NO_SPU];

unsigned char* key;
unsigned int* S;

void *ppu_pthread_function(void *arg) {
  ppu_pthread_data_t *datap = (ppu_pthread_data_t *)arg;
  unsigned int entry = SPE_DEFAULT_ENTRY;
  if (spe_context_run(datap->spe_ctx, &entry, 0, datap->argp, NULL, NULL) < 0) {
    perror ("Failed running context");
    exit (1);
  }
  pthread_exit(NULL);
}


int rc5_Init(unsigned int DATASIZE) {

  unsigned int a_L[b/wb];
  unsigned int A,B;
  unsigned int temp = 0;
  unsigned int sentinel;
  int i,j;
  

  expanded_key_size = t * sizeof( unsigned int );
  if( ( expanded_key_size % 16 ) != 0 ) {
    expanded_key_size += 16 - (expanded_key_size % 16);
  }
  S = ( unsigned int* )_malloc_align( expanded_key_size, 7 );
  if( S == NULL ) {
    printf("malloc failure\n");exit(0);
  }
  
  key = (unsigned char *)malloc(b);
  if(key==NULL) {
    printf("malloc failure\n");
    return -1;
  }
  for(i=0 ;i<b;i++) {
    if((i%4)==0) {
      temp = (unsigned int)rand();
    }
    key[i] = (unsigned char)( temp & 0xff );
    temp >>= 8;
  }
  
  /* converting the secret key from bytes to words */
  
  memset(a_L,0x00,b);
  
  for(i=0;i<b;i++) {
    temp = (key[i]&0xff) << (8*(3-(i%4)));
    a_L[i / wb] = a_L[i / wb] + temp;
  }
  
  /* initializing the array S */
  
  S[0] = BE2LE( P );
  for( i = 1 ; i < 2 * r + 1 ; i++ ) {
    S[i] = BE2LE( LE2BE( S[i-1] ) + Q );
  }
  
  /* mixing in the secret key */
  
  if( 2 * r + 1 > b / wb ) {
    sentinel = 3*t;
  }
  else {
    sentinel = 3*(b/wb);
  }
  
  i = 0;
  j = 0;
  A = 0;
  B = 0;
  
  for( temp = 0 ; temp < sentinel ; temp++ ) {
    S[i] = BE2LE( ROTL( ( LE2BE( S[i] ) + LE2BE( A ) + LE2BE( B ) ), 3 ) );
    A = S[i];
    a_L[j] = BE2LE( ROTL( ( LE2BE( a_L[j] ) + LE2BE( A ) + LE2BE( B ) ), ( LE2BE( A ) + LE2BE( B ) ) ) );
    B = a_L[j];
    i = ( i + 1 ) % ( 2 * r + 1 );
    j = ( j + 1 ) % ( b / wb );
  }
  
  return 0;
}


void rc5_Decrypt(unsigned int* encrypted_data, unsigned int * decrypted_data, unsigned int DATASIZE) {
  unsigned int chunksize;
  int i;
  ppu_pthread_data_t data[NO_SPU];

  chunksize = DATASIZE/NO_SPU;

  for(i=0;i<NO_SPU;i++) {
    cb[i].addrinputdata     = (unsigned int) &(encrypted_data[i*chunksize]);
    cb[i].addroutputdata     = (unsigned int) &(decrypted_data[i*chunksize]);
    cb[i].addrexpandedtable    = (unsigned int) S;
  }	
  

   /* allocate SPE tasks */
   for (i = 0; i < NO_SPU; i++) {
    if ((data[i].spe_ctx = spe_context_create (0, NULL)) == NULL) {
      perror ("Failed creating context");
      exit (1);
    }
    if (spe_program_load (data[i].spe_ctx, &rc5_decrypt))  {
      perror ("Failed loading program");
      exit (1);
    }
    data[i].argp = &cb[i];
   }
 
//   gettimeofday(&timev1, NULL);

   for(i=0;i<NO_SPU;i++) {
    if (pthread_create (&data[i].pthread, NULL, &ppu_pthread_function, &data[i])) {
      perror ("Failed creating thread");
      exit (1);
    }

   }
   
   for(i=0;i<NO_SPU;i++) {
     if (pthread_join (data[i].pthread, NULL)) {
       perror ("Failed joining thread\n");
       exit (1);
     }
   }
 
 //  gettimeofday(&timev2, NULL);

   __asm__ __volatile__ ("sync" : : : "memory");

}

void rc5_Encrypt(unsigned int* input_data, unsigned int * encrypted_data, unsigned int DATASIZE) {
  unsigned int chunksize;
  int i;
  ppu_pthread_data_t data[NO_SPU];

  /* create SPE threads - encryption */
  
  chunksize = DATASIZE/NO_SPU;
  
  for(i=0;i<NO_SPU;i++) {
    cb[i].addrinputdata     = (unsigned int) &(input_data[i*chunksize]);
    cb[i].addroutputdata     = (unsigned int) &(encrypted_data[i*chunksize]);
    cb[i].addrexpandedtable    = (unsigned int) S;
    cb[i].DATASIZE = DATASIZE;
  }	
  
   /* allocate SPE tasks */
   for (i = 0; i < NO_SPU; i++) {
    if ((data[i].spe_ctx = spe_context_create (0, NULL)) == NULL) {
      perror ("Failed creating context");
      exit (1);
    }
    if (spe_program_load (data[i].spe_ctx, &rc5_encrypt))  {
      perror ("Failed loading program");
      exit (1);
    }
    data[i].argp = &cb[i];
   }
 
//   gettimeofday(&timev1, NULL);

   for(i=0;i<NO_SPU;i++) {
    if (pthread_create (&data[i].pthread, NULL, &ppu_pthread_function, &data[i])) {
      perror ("Failed creating thread");
      exit (1);
    }

   }
   
   for(i=0;i<NO_SPU;i++) {
     if (pthread_join (data[i].pthread, NULL)) {
       perror ("Failed joining thread\n");
       exit (1);
     }
   }
 
 //  gettimeofday(&timev2, NULL);

   __asm__ __volatile__ ("sync" : : : "memory");
  
}

int rc5_Check(unsigned int* input_data,unsigned int * decrypted_data, unsigned int DATASIZE) {
  int num_errors;
  int i;
  
  num_errors = 0;
  
  for( i = 0 ; i < DATASIZE ; i++ ) {
    if( input_data[i] != decrypted_data[i] ) {
      num_errors++;
    }
  }
  
  if( num_errors == 0 )
    printf( "Correct results\n");
  else 
    printf("Incorrect results\n");  
}


