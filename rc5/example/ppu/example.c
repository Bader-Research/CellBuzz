#include <stdio.h>
#include "../../src/rc5.h"
#include <malloc_align.h>
#include <free_align.h>

int main() {
  unsigned int* input_data;
  unsigned int* encrypted_data;
  unsigned int* decrypted_data;
  unsigned int DATASIZE = 2097152;
  int i;
  printf("Initialize ...\n");

  //initialize rc5
  if(rc5_Init(DATASIZE) == -1 ) {
    printf("initialize failure\n");exit(0);
  }

  
  //load input data
  encrypted_data = ( unsigned int* )_malloc_align( DATASIZE * sizeof( unsigned int ), 7 );
  if( encrypted_data == NULL ) {
    printf("malloc failure\n");exit(0);
  }
  decrypted_data = ( unsigned int* )_malloc_align( DATASIZE * sizeof( unsigned int ), 7 ); 
  if( decrypted_data == NULL ) {
    printf("malloc failure\n");exit(0);
  }

  input_data = ( unsigned int* )_malloc_align( DATASIZE * sizeof( unsigned int ), 7 ); 
  if( input_data == NULL ) {
    printf("malloc failure\n");exit(0);
  }
  for(i=0;i<DATASIZE;i++) {
    input_data[i] = ( unsigned int )rand();
  }
  printf("Data Encrypt ...\n");
  rc5_Encrypt(input_data,encrypted_data, DATASIZE);

  printf("\nData Decrypt ...\n");
  rc5_Decrypt(encrypted_data, decrypted_data,DATASIZE);
  /* force memory bus flush */
  
  printf("Check Correctness ... \n");
  /* check correctness */
  rc5_Check(input_data,decrypted_data, DATASIZE);

  _free_align(input_data);
  _free_align(encrypted_data);
  _free_align(decrypted_data);

  
  return 0;
}

