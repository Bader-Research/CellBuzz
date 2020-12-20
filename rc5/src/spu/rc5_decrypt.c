#include <stdlib.h>
#include <string.h>
#include <spu_mfcio.h>
#include <malloc_align.h>
#include <free_align.h>
#include <spu_internals.h>

#include "../rc5.h"
#include "../rc5_spu.h"

#define BUFFERSIZE 1024
//#define CHUNKSIZE (DATASIZE/NO_SPU)
unsigned int CHUNKSIZE;

#define ROTATE_L_V(x,y) spu_rl(x,(vector signed int)spu_and(y,(0x1f))) 
#define ROTATE_R_V(x,y) spu_rl(x,spu_sub(0,(vector signed int)spu_and(y,(0x1f)))) 

#define LE2BE(le) ((((le)&0xff)<<24)|(((le)&0xff00)<<8)|(((le)&0xff0000)>>8)|(((le)&0xff000000)>>24))
#define BE2LE(be) ((((be)&0xff)<<24)|(((be)&0xff00)<<8)|(((be)&0xff0000)>>8)|(((be)&0xff000000)>>24))

#define LE2BE_V(le) spu_shuffle(le,le,((vector unsigned char){3,2,1,0,7,6,5,4,11,10,9,8,15,14,13,12}))
#define BE2LE_V(be) spu_shuffle(be,be,((vector unsigned char){3,2,1,0,7,6,5,4,11,10,9,8,15,14,13,12}))

static control_block cb __attribute__ ( ( aligned(128 ) ) );

static unsigned int inputdata[2][BUFFERSIZE * sizeof(unsigned int)] __attribute__ ( ( aligned( 128 ) ) );
static unsigned int decrypteddata[2][BUFFERSIZE * sizeof(unsigned int)] __attribute__ ( ( aligned( 128 ) ) );
static unsigned int a_temp[BUFFERSIZE * sizeof( unsigned int )];
static unsigned int* S;

void rc5_decrypt( vector unsigned int* inputtext, vector unsigned int* outputtext, unsigned int size ) {
  int offset;
  unsigned int i,j;
  unsigned int shiftj;
  vector unsigned int A,B;
  offset  = size >> 3;
  for(i=0;i<size/(2*4);i++) {
    A = outputtext[i];
    B = outputtext[offset+i];
    for(j=r;j>=1;j-=4) {
      shiftj = (j<<1);
      B=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(B),spu_splats(S[shiftj+1])),LE2BE_V(A)),LE2BE_V(A)));
      A=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(A),spu_splats(S[shiftj])),LE2BE_V(B)),LE2BE_V(B))); 
      
      B=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(B),spu_splats(S[shiftj-1])),LE2BE_V(A)),LE2BE_V(A))); 
      A=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(A),spu_splats(S[shiftj-2])),LE2BE_V(B)),LE2BE_V(B)));
      
      B=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(B),spu_splats(S[shiftj-3])),LE2BE_V(A)),LE2BE_V(A))); 
      A=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(A),spu_splats(S[shiftj-4])),LE2BE_V(B)),LE2BE_V(B))); 
      
      B=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(B),spu_splats(S[shiftj-5])),LE2BE_V(A)),LE2BE_V(A))); 
      A=BE2LE_V(spu_xor(ROTATE_R_V(spu_sub(LE2BE_V(A),spu_splats(S[shiftj-6])),LE2BE_V(B)),LE2BE_V(B))); 
    }
    outputtext[i] = A;
    outputtext[offset+i] = B;
    inputtext[offset+i]=BE2LE_V(spu_sub(LE2BE_V(outputtext[offset+i]),spu_splats((S[1]))));
    inputtext[i]=BE2LE_V(spu_sub(LE2BE_V(outputtext[i]),spu_splats((S[0]))));
  }
}

int main( unsigned long long speid, unsigned long long argp, unsigned long long envp ) {
  unsigned int i,j;
  unsigned int key_size;
  unsigned int buffercount;
  unsigned int inpchar = 0;
#ifdef PROFILING
  unsigned int tstart;
  int kk;
  unsigned int tstop;
#endif

  mfc_get(&cb, argp, sizeof(control_block), 31, 0, 0);
  mfc_write_tag_mask(1 << 31);
  mfc_read_tag_status_all(); 
  
  CHUNKSIZE = cb.DATASIZE/NO_SPU;
  buffercount  = CHUNKSIZE/BUFFERSIZE;
  key_size = t * sizeof(unsigned int);
  if((key_size%16)!=0) {
    key_size += 16 - (key_size%16);
  }
  S = (unsigned int *)_malloc_align(key_size, 7);
  if(S == 0) {
    printf("malloc failure\n");exit(0);
  }
  
  mfc_get(S, cb.addrexpandedtable, key_size, 20, 0, 0 );
  mfc_write_tag_mask(1 << 20);
  mfc_read_tag_status_all(); 
  
  for(i = 0; i < t; i++) {
    S[i] = LE2BE(S[i]);
  }
#ifdef PROFILING
  spu_write_decrementer(0x7fffffff);
  tstart = spu_read_decrementer();
  ///////////LOOPOVER
  for(kk=0;kk<2000;kk++) {    
#endif  
  if(buffercount == 0) {
    mfc_get( inputdata[0], cb.addrinputdata, CHUNKSIZE * sizeof( unsigned int ), 20, 0, 0 );
    mfc_write_tag_mask( 1 << 20 );
    mfc_read_tag_status_all(); 
    
    data_shuffle( inputdata[0], a_temp, CHUNKSIZE);
    
    rc5_decrypt( ( vector unsigned int* )decrypteddata[0], ( vector unsigned int* )inputdata[0], CHUNKSIZE);
    
    data_restore( decrypteddata[0], a_temp, CHUNKSIZE );
    
    mfc_put( decrypteddata[0], cb.addroutputdata, CHUNKSIZE * sizeof( unsigned int ), 20, 0, 0 );
    mfc_write_tag_mask( 1 << 20 );
    mfc_read_tag_status_all(); 
  }	

  else {								
    mfc_get(inputdata[inpchar],cb.addrinputdata,BUFFERSIZE*sizeof(unsigned int),inpchar,0,0);
    j=0;
    for(;j<(buffercount-1);j++) {
      inpchar = (inpchar+1)%2;
      mfc_get(inputdata[inpchar],cb.addrinputdata+((j+1)*sizeof(unsigned int)*BUFFERSIZE),BUFFERSIZE*sizeof(unsigned int),inpchar,0,0);
      inpchar = (inpchar+1)%2;
      mfc_write_tag_mask(1<<inpchar);
      mfc_read_tag_status_all();

      data_shuffle( inputdata[inpchar], a_temp, BUFFERSIZE);
      rc5_decrypt( ( vector unsigned int* )decrypteddata[inpchar], ( vector unsigned int* )inputdata[inpchar], BUFFERSIZE);
      data_restore( decrypteddata[inpchar], a_temp, BUFFERSIZE ); 

      mfc_put( decrypteddata[inpchar], cb.addroutputdata+(j*sizeof(unsigned int)*BUFFERSIZE), BUFFERSIZE * sizeof( unsigned int ), inpchar, 0, 0 );
      inpchar = (inpchar+1)%2;
    }
    mfc_write_tag_mask(1<<inpchar);
    mfc_read_tag_status_all();
    data_shuffle( inputdata[inpchar], a_temp, BUFFERSIZE);
    rc5_decrypt( ( vector unsigned int* )decrypteddata[inpchar], ( vector unsigned int* )inputdata[inpchar], BUFFERSIZE);
    data_restore( decrypteddata[inpchar], a_temp, BUFFERSIZE ); 
    mfc_put( decrypteddata[inpchar], cb.addroutputdata+(j*sizeof(unsigned int)*BUFFERSIZE), BUFFERSIZE * sizeof( unsigned int ), inpchar, 0, 0 ); 
    mfc_write_tag_mask(1<<inpchar);
    mfc_read_tag_status_all();
    inpchar = (inpchar+1)%2;
    mfc_write_tag_mask(1<<inpchar);
    mfc_read_tag_status_all();
  }
#ifdef PROFILING
  }
  tstop = spu_read_decrementer();
  fprintf(stdout,"Decrementer Cycle Counter: %u, Seconds: %f\n",(tstart-tstop)/2000,((tstart-tstop)/(14318000.0*2000)));
  fflush(stdout);
#endif	
  _free_align( S ); 
  
  return 0;
}

