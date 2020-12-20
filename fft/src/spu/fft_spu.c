#include "../fft.h"
//#include <cbe_mfcio.h>
#include <spu_mfcio.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <spu_internals.h>
//#include <profile.h>

/*
Using multi buffering, to fetch all the elements at once but in chunks using multiple buffers - this will help reduce latency, and issuing dma fetch for all the buffers at once is possible because LS has enough space for holding 1/8 of the input data. 
fetching 128 bytes in one DMA transfer .. <10k elements that is 10k/8 elements for each spu .. means .. that <16 buffers in total .. 2 k elements .. => 8kbytes
*/
//no of elements fetched in a single DMA transfer call
//#define BUFFER_SIZE 256
#define MAX_ARRAY_SIZE 2048
#define PI 3.1416
vector unsigned char R_PERM_VECTOR1=(vector unsigned char){0,1,2,3,16,17,18,19,4,5,6,7,20,21,22,23};

vector unsigned char R_PERM_VECTOR2=(vector unsigned char){8,9,10,11,24,25,26,27,12,13,14,15,28,29,30,31};


//real r .. imaginary i .. .2 buffers
float rdatabuffer[2][MAX_ARRAY_SIZE] __attribute__ ((aligned (128))); //real fetch from array in MM
float idatabuffer[2][MAX_ARRAY_SIZE] __attribute__ ((aligned (128))); //imaginary fetch from array in MM
float routdatabuffer[2*MAX_ARRAY_SIZE] __attribute__ ((aligned (128)));//real output data buffer
float ioutdatabuffer[2*MAX_ARRAY_SIZE] __attribute__ ((aligned (128)));

//real part stored in wcos .. imaginary in wsin
float *wcos;
float *wsin;

/* data pointers to different parts of the databuffer array */
// real and imaginary
float *rdata[2][2];
float *idata[2][2];
float *routdata[2];
float *ioutdata[2];

/* number of buffers that are used to store DATA */
int buffer_count; 

/* control structure */
control_block cb __attribute__ ((aligned (128)));
sync_control_block sync_cb __attribute__ ((aligned (128)));


int synchronize_spes(int speno,int message) {
  volatile int signal[4] __attribute__ ((aligned (128)));
  unsigned long ea;
  char* ls;  
  int spenop1 = speno + 1;
  int tag_id = 1;
  int toggle = 0;
  
  signal[3] = message;
  ls = ((char*)&signal[0])+ 12;  
  
  if(spenop1 == 1) {
    ea =  (unsigned long)sync_cb.sig1[1] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    spu_read_signal1();
    spu_read_signal2();
    
    ea =  (unsigned long)sync_cb.sig2[3] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    ea =  (unsigned long)sync_cb.sig2[7] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    spu_read_signal2();
    
  }
  
  if(spenop1 == 2) {
    spu_read_signal1();
    
    ea =  (unsigned long)sync_cb.sig2[3] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    signal[3] = spu_read_signal1();
    
    ea =  (unsigned long)sync_cb.sig2[0] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
  }
  
  if(spenop1 == 3) {
    ea =  (unsigned long)sync_cb.sig1[3] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    signal[3] = spu_read_signal2();
    
  }
  
  if(spenop1 == 4) {
    spu_read_signal1();
    
    spu_read_signal2();
    
    ea =  (unsigned long)sync_cb.sig1[0] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    signal[3] = spu_read_signal2();
    
    ea =  (unsigned long)sync_cb.sig1[1] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    ea =  (unsigned long)sync_cb.sig2[2] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
  }
  
  if(spenop1 == 5) {
    ea =  (unsigned long)sync_cb.sig1[5] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    signal[3] = spu_read_signal2();
    
  }
  
  if(spenop1 == 6) {
    spu_read_signal1();
    
    ea =  (unsigned long)sync_cb.sig2[7] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    signal[3] = spu_read_signal1();
    
    ea =  (unsigned long)sync_cb.sig2[4] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
  }
  
  if(spenop1 == 7) {
    ea =  (unsigned long)sync_cb.sig1[7] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    signal[3] = spu_read_signal2();
    
  }
  
  if(spenop1 == 8) {
    spu_read_signal1();
    
    spu_read_signal2();
    
    ea =  (unsigned long)sync_cb.sig2[0] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    signal[3] = spu_read_signal2();
    
    ea =  (unsigned long)sync_cb.sig1[5] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
    
    ea =  (unsigned long)sync_cb.sig2[6] + 12;
    mfc_sndsig(ls , ea, tag_id, 0,0);
    mfc_read_tag_status_all();
  }
  return (signal[3]);
}

/* using singleton method for computing twiddle factors */
// simdization required
void compute_twiddle_factors() {
  int size = cb.n;
  unsigned int L;
  int k;
  float theta = 2*PI/size;
  float S = sin(theta); 
  float C = 1 - 2*sin(theta/2)*sin(theta/2);
  wcos[0] = 1; wsin[0] = 0;
  for(k=0;k<size/8-1;k++) {
    wcos[k+1] = C*wcos[k] - S*wsin[k];
    wsin[k+1] = S*wcos[k] + C*wsin[k];
  }
  L = size/8;
  wcos[L] = wsin[L] = sqrt(2)/2;
  for(k=1;k<size/8;k++) {
    wcos[L+k] = wsin[L-k];
    wsin[L+k] = wcos[L-k];
  }
  L = size/4;
  wcos[L] = 0 ;
  wsin[L] = 1 ;
  for(k=1;k<size/4;k++) {
    wcos[L+k] = -wcos[L-k];
    wsin[L+k] = wsin[L-k];
  }
}

/* here is the location where the SPE begins execution, once its thread is created */
int main(unsigned long long speid, addr64 argp, addr64 envp) {
  int array_size, chunk_size, i, j, k, no_of_problems, dist, problem_size, my_array_pos, j_initial, j_twiddle, j_first, k_initial, temp;
  int MYTHREAD, j_first_initial;
  //array address  (main memory) of the get and put arrays
  unsigned int addrfetchr,addrfetchi,addrputr,addrputi; 
//  float *fetchr,*fetchi, *putr, *puti;
  float Wcos, Wsin;
  float tempr, tempi;
  unsigned int addrswap;
  ///////////////////////////////
  int lll;
  unsigned int mb_value = 1;
  int loopover;
//  unsigned int tstart;
//  unsigned int tstop;
  unsigned int sync_cb_addr;
  int BUFFER_SIZE;
  vector float *vroutdatabuffer,*vioutdatabuffer,*vrdatabuffer1,*vrdatabuffer2,*vidatabuffer1,*vidatabuffer2;
  vector float *vroutdata1,*vroutdata2,*vioutdata1,*vioutdata2;
  vector float vtempr,vtempi;
  vector float vtemp1r,vtemp1i;
  vector float vtemp2r,vtemp2i;
  vector float vtemp3r,vtemp3i;
  vector float vWcos,vWsin;
  vector float vWcos1,vWsin1;
  vector float vWcos2,vWsin2;
  vector float *vwcos, *vwsin;
  int message,p;
  //////////////////////////////
  /* DMA control block information from system memory. */
  mfc_get(&cb, argp.ui[1], sizeof(cb), 31, 0, 0);
  mfc_write_tag_mask(1<<31);
  mfc_read_tag_status_all();
  
  /*********************SYNC INITIALIZE**************/
  sync_cb_addr = spu_read_in_mbox();
  mfc_get(&sync_cb, sync_cb_addr, sizeof(sync_cb), 5, 0, 0);
  mfc_write_tag_mask(1 << 5);
  mfc_read_tag_status_all();
  //  for(i=0;i<NO_SPU;i++) {
  //    printf("AREA : %u \n",sync_cb.spu_control[i]);
  //  }
  /*************************************************/
  //initialize parameters
  array_size = cb.n;
  wcos = (float *)malloc(sizeof(float)*array_size/2);
  wsin = (float *)malloc(sizeof(float)*array_size/2);
  compute_twiddle_factors();
  BUFFER_SIZE = array_size/(NO_SPU*4);

  chunk_size = array_size/(NO_SPU*2); //chunk_size is the largest continuous chunk this SPE will fetch
  buffer_count = 2;//chunk_size/BUFFER_SIZE;//buffer count is the no of buffers required to fetch this chunk
  MYTHREAD = cb.threadno;
  //  printf("Chunk Size : %d, buffer count : %d, Mythread: %d\n",chunk_size, buffer_count, MYTHREAD);
  //initialize buffer pointers
  for(i = 0;i<2/*MAX_ARRAY_SIZE/BUFFER_SIZE*/;i++) {//MAX array size 
    rdata[0][i] = &(rdatabuffer[0][i*BUFFER_SIZE]);
    idata[0][i] = &(idatabuffer[0][i*BUFFER_SIZE]);
    rdata[1][i] = &(rdatabuffer[1][i*BUFFER_SIZE]);
    idata[1][i] = &(idatabuffer[1][i*BUFFER_SIZE]);
  }
  routdata[0] = &(routdatabuffer[0]);
  routdata[1] = &(routdatabuffer[MAX_ARRAY_SIZE]);
  ioutdata[0] = &(ioutdatabuffer[0]);
  ioutdata[1] = &(ioutdatabuffer[MAX_ARRAY_SIZE]);

  /*********************************************************************/
  /*******************LOOPOVER******************************************/
  //  spu_write_decrementer(0x7fffffff);
  //  tstart = spu_read_decrementer();

  //  prof_clear();
  //  prof_start();
while(1) {
  message=2;
  if(MYTHREAD==0) {
    message = spu_read_in_mbox();
//        printf("STARTING %d\n",p++);
  }
  
  message = synchronize_spes(MYTHREAD,message);
  if(message==3) return 0;
#ifdef PROFILING
for(lll=0;lll<100000;lll++) {
#endif
  addrfetchr = (unsigned int)cb.addrDataAr;
  addrfetchi = (unsigned int)cb.addrDataAi;
  addrputr = (unsigned int)cb.addrDataBr;
  addrputi = (unsigned int)cb.addrDataBi;

  no_of_problems = 1;
  dist = 1;
  problem_size = array_size;

  my_array_pos = chunk_size * MYTHREAD;
  /**************************STAGE 0****************************/

  vrdatabuffer1 = (vector float *)&(rdatabuffer[0][0]);
  vrdatabuffer2 = (vector float *)&(rdatabuffer[1][0]);
  vidatabuffer1 = (vector float *)&(idatabuffer[0][0]);
  vidatabuffer2 = (vector float *)&(idatabuffer[1][0]);
  vwcos = (vector float *)wcos;
  vwsin = (vector float *)wsin;
  vroutdatabuffer = (vector float *)&(routdatabuffer[0]);
  vioutdatabuffer = (vector float *)&(ioutdatabuffer[0]);

  
  spu_mfcdma32(rdata[0][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
  spu_mfcdma32(idata[0][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
  spu_mfcdma32(rdata[1][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD); 
  spu_mfcdma32(idata[1][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
  
  k_initial = (int)my_array_pos;
  j_first_initial = 0;
  j_initial = 2*k_initial;
  j_twiddle = k_initial;
  
  for(j=0,k=0 ;  j<2*chunk_size;) {
    temp = j/(BUFFER_SIZE*2);
    if(temp<(buffer_count-1)) {
	  spu_mfcdma32(rdata[0][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
	  spu_mfcdma32(idata[0][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
	  spu_mfcdma32(rdata[1][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD); 
	  spu_mfcdma32(idata[1][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
    }
    mfc_write_tag_mask(1<<temp);
    mfc_read_tag_status_all();
    for(i=0;  i<BUFFER_SIZE;  j+=16,k+=8,j_twiddle+=8,i+=8) {
      vWcos = vwcos[j_twiddle/4];
      vWsin = vwsin[j_twiddle/4];

      vtempr = spu_add(vrdatabuffer1[k/4], vrdatabuffer2[k/4]);
      vtempi = spu_add(vidatabuffer1[k/4], vidatabuffer2[k/4]);
     
      vtemp1r = spu_sub(vrdatabuffer1[k/4], vrdatabuffer2[k/4]);
      vtemp1i = spu_sub(vidatabuffer1[k/4], vidatabuffer2[k/4]);
      vtemp2r = spu_sub(spu_mul(vtemp1r,vWcos), spu_mul(vtemp1i,vWsin));
      vtemp2i = spu_add(spu_mul(vtemp1r,vWsin), spu_mul(vtemp1i,vWcos));
      
      vtemp3r = spu_shuffle(vtempr, vtemp2r, R_PERM_VECTOR1);
      vroutdatabuffer[j/4] = vtemp3r;
      vtemp3i = spu_shuffle(vtempi, vtemp2i, R_PERM_VECTOR1);
      vioutdatabuffer[j/4] = vtemp3i;

      vtemp3r = spu_shuffle(vtempr,vtemp2r, R_PERM_VECTOR2);
      vroutdatabuffer[j/4+1] = vtemp3r;
      vtemp3i = spu_shuffle(vtempi,vtemp2i, R_PERM_VECTOR2);
      vioutdatabuffer[j/4+1] = vtemp3i;

      //unroll
      vWcos = vwcos[j_twiddle/4+1];
      vWsin = vwsin[j_twiddle/4+1];

      vtempr = spu_add(vrdatabuffer1[k/4+1], vrdatabuffer2[k/4+1]);
      vtempi = spu_add(vidatabuffer1[k/4+1], vidatabuffer2[k/4+1]);
     
      vtemp1r = spu_sub(vrdatabuffer1[k/4+1], vrdatabuffer2[k/4+1]);
      vtemp1i = spu_sub(vidatabuffer1[k/4+1], vidatabuffer2[k/4+1]);
      vtemp2r = spu_sub(spu_mul(vtemp1r,vWcos), spu_mul(vtemp1i,vWsin));
      vtemp2i = spu_add(spu_mul(vtemp1r,vWsin), spu_mul(vtemp1i,vWcos));
      
      vtemp3r = spu_shuffle(vtempr, vtemp2r, R_PERM_VECTOR1);
      vtemp3i = spu_shuffle(vtempi, vtemp2i, R_PERM_VECTOR1);
      vroutdatabuffer[j/4+2] = vtemp3r;
      vioutdatabuffer[j/4+2] = vtemp3i;
      vtemp3r = spu_shuffle(vtempr,vtemp2r, R_PERM_VECTOR2);
      vtemp3i = spu_shuffle(vtempi,vtemp2i, R_PERM_VECTOR2);
      vroutdatabuffer[j/4+3] = vtemp3r;
      vioutdatabuffer[j/4+3] = vtemp3i;
    }
    spu_mfcdma32(&(routdatabuffer[temp*BUFFER_SIZE*2]), addrputr+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
    spu_mfcdma32(&(ioutdatabuffer[temp*BUFFER_SIZE*2]), addrputi+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
  }
  addrswap = addrfetchr;
  addrfetchr = addrputr;
  addrputr = addrswap;
  addrswap = addrfetchi;
  addrfetchi = addrputi;
  addrputi = addrswap;
  no_of_problems = 2;
  problem_size /= 2;
  dist = 2;
  mfc_write_tag_mask(1<<0);
  mfc_read_tag_status_all();

  synchronize_spes(MYTHREAD,2);
  /********************************END STAGE 0**************************/
  
  /********************************STAGE 1******************************/

    spu_mfcdma32(rdata[0][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    spu_mfcdma32(idata[0][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    spu_mfcdma32(rdata[1][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD); 
    spu_mfcdma32(idata[1][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
      
    
  k_initial = (int)(my_array_pos/2);
  k_initial *= 2;
  j_first_initial = my_array_pos - k_initial;
  j_initial = 2*k_initial;
  j_twiddle = k_initial;
  
  for(j=0,k=0 ;  j<2*chunk_size;) {
    temp = j/(BUFFER_SIZE*2);
    if(temp<(buffer_count-1)) {
      spu_mfcdma32(rdata[0][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
      spu_mfcdma32(idata[0][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
      spu_mfcdma32(rdata[1][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD); 
      spu_mfcdma32(idata[1][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
    }
    mfc_write_tag_mask(1<<temp);
    mfc_read_tag_status_all();

    for(i=0;  i<BUFFER_SIZE;  j+=8,k+=4,j_twiddle+=4,i+=4) {
      vWcos1 = spu_splats(wcos[j_twiddle]);
      vWsin1 = spu_splats(wsin[j_twiddle]);
      vWcos2 = spu_splats(wcos[j_twiddle+2]);
      vWsin2 = spu_splats(wsin[j_twiddle+2]);

      vWcos = spu_shuffle(vWcos1,vWcos2,((vector unsigned char){0,1,2,3,0,1,2,3,16,17,18,19,16,17,18,19}));
      vWsin = spu_shuffle(vWsin1,vWsin2,((vector unsigned char){0,1,2,3,0,1,2,3,16,17,18,19,16,17,18,19}));

      vtempr = spu_add(vrdatabuffer1[k/4], vrdatabuffer2[k/4]);
      vtempi = spu_add(vidatabuffer1[k/4], vidatabuffer2[k/4]);
     
      vtemp1r = spu_sub(vrdatabuffer1[k/4], vrdatabuffer2[k/4]);
      vtemp1i = spu_sub(vidatabuffer1[k/4], vidatabuffer2[k/4]);
      vtemp2r = spu_sub(spu_mul(vtemp1r,vWcos), spu_mul(vtemp1i,vWsin));
      vtemp2i = spu_add(spu_mul(vtemp1r,vWsin), spu_mul(vtemp1i,vWcos));
      
      vtemp3r = spu_shuffle(vtempr, vtemp2r, ((vector unsigned char){0,1,2,3,4,5,6,7,16,17,18,19,20,21,22,23}));
      vtemp3i = spu_shuffle(vtempi, vtemp2i, ((vector unsigned char){0,1,2,3,4,5,6,7,16,17,18,19,20,21,22,23}));
      vroutdatabuffer[j/4] = vtemp3r;
      vioutdatabuffer[j/4] = vtemp3i;
      vtemp3r = spu_shuffle(vtempr,vtemp2r, ((vector unsigned char){8,9,10,11,12,13,14,15,24,25,26,27,28,29,30,31}));
      vtemp3i = spu_shuffle(vtempi,vtemp2i, ((vector unsigned char){8,9,10,11,12,13,14,15,24,25,26,27,28,29,30,31}));
      vroutdatabuffer[j/4+1] = vtemp3r;
      vioutdatabuffer[j/4+1] = vtemp3i;

    }
    spu_mfcdma32(&(routdatabuffer[temp*BUFFER_SIZE*2]), addrputr+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
    spu_mfcdma32(&(ioutdatabuffer[temp*BUFFER_SIZE*2]), addrputi+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
  }
  addrswap = addrfetchr;
  addrfetchr = addrputr;
  addrputr = addrswap;
  addrswap = addrfetchi;
  addrfetchi = addrputi;
  addrputi = addrswap;
  no_of_problems *= 2;
  problem_size /= 2;
  dist *= 2;
  mfc_write_tag_mask(1<<0);
  mfc_read_tag_status_all();
  
  synchronize_spes(MYTHREAD,2);

  
  /********************************END STAGE 1*************************/
  


 /*STAGE 2 */
 //initiate DMA get transfer for all buffers
    spu_mfcdma32(rdata[0][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    spu_mfcdma32(idata[0][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    spu_mfcdma32(rdata[1][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD); 
    spu_mfcdma32(idata[1][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    
    k_initial = (int)(my_array_pos/no_of_problems);
    k_initial *= no_of_problems;
    j_first_initial = my_array_pos - k_initial;
    j_initial = 2*k_initial;
    j_twiddle = k_initial;
    
    //for this case  .. jfirstinitial=0 .. assuming chunksize > BUFFER_SIZE
    for(j=0,k=0 ;  j<2*(chunk_size/4);) {
      temp = (j*4)/(BUFFER_SIZE*2);
      if(temp<(buffer_count-1)) {
	spu_mfcdma32(rdata[0][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
	spu_mfcdma32(idata[0][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
	spu_mfcdma32(rdata[1][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD); 
	spu_mfcdma32(idata[1][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
      }
      mfc_write_tag_mask(1<<temp);
      mfc_read_tag_status_all();
    //      mfc_write_tag_mask(1<<temp);
    //      mfc_read_tag_status_all();
for(i=0;  i<BUFFER_SIZE/no_of_problems;  j+=2*(no_of_problems/4),k+=(no_of_problems/4),j_twiddle+=no_of_problems,i++) {
	Wcos = wcos[j_twiddle];
	Wsin = wsin[j_twiddle];
	for(j_first=0;  j_first<(no_of_problems/4);  j_first++) {
	  vroutdatabuffer[j+j_first] = spu_add(vrdatabuffer1[k+j_first],vrdatabuffer2[k+j_first]);
	  vioutdatabuffer[j+j_first] = spu_add(vidatabuffer1[k+j_first],vidatabuffer2[k+j_first]);
	  vtempr = spu_sub(vrdatabuffer1[k+j_first],vrdatabuffer2[k+j_first]);
	  vtempi = spu_sub(vidatabuffer1[k+j_first],vidatabuffer2[k+j_first]);
	  vroutdatabuffer[j+(no_of_problems/4)+j_first] = spu_sub(spu_mul(vtempr,spu_splats(Wcos)),spu_mul(vtempi,spu_splats(Wsin)));
	  vioutdatabuffer[j+(no_of_problems/4)+j_first] = spu_add(spu_mul(vtempr,spu_splats(Wsin)),spu_mul(vtempi,spu_splats(Wcos)));	  
	  
	}
      }
      spu_mfcdma32(&(routdatabuffer[temp*BUFFER_SIZE*2]), addrputr+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
	      spu_mfcdma32(&(ioutdatabuffer[temp*BUFFER_SIZE*2]), addrputi+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
      //write back the currently computed results
    }
    addrswap = addrfetchr;
    addrfetchr = addrputr;
    addrputr = addrswap;
    addrswap = addrfetchi;
    addrfetchi = addrputi;
    addrputi = addrswap;
    no_of_problems *= 2;
    problem_size /= 2;
    dist *= 2;
    mfc_write_tag_mask(1<<0);
    mfc_read_tag_status_all();
    
    /********************************Synchronize**************************/
    synchronize_spes(MYTHREAD,2);
    /********************************************************/

  while(no_of_problems < BUFFER_SIZE && problem_size > 1) {
    //initiate DMA get transfer for all buffers
    spu_mfcdma32(rdata[0][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    spu_mfcdma32(idata[0][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD,sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    spu_mfcdma32(rdata[1][0],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD); 
    spu_mfcdma32(idata[1][0],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    
    k_initial = (int)(my_array_pos/no_of_problems);
    k_initial *= no_of_problems;
    j_first_initial = my_array_pos - k_initial;
    j_initial = 2*k_initial;
    j_twiddle = k_initial;
    
    //for this case  .. jfirstinitial=0 .. assuming chunksize > BUFFER_SIZE
    for(j=0,k=0 ;  j<2*(chunk_size/4);) {
      temp = (j*4)/(BUFFER_SIZE*2);
      if(temp<(buffer_count-1)) {
	spu_mfcdma32(rdata[0][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
	spu_mfcdma32(idata[0][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(temp+1)*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
	spu_mfcdma32(rdata[1][temp+1],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD); 
	spu_mfcdma32(idata[1][temp+1],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+((temp+1)*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,temp+1,MFC_GET_CMD);
      }
      mfc_write_tag_mask(1<<temp);
      mfc_read_tag_status_all();
    //      mfc_write_tag_mask(1<<temp);
    //      mfc_read_tag_status_all();
      for(i=0;  i<BUFFER_SIZE/no_of_problems;  j+=2*(no_of_problems/4),k+=(no_of_problems/4),j_twiddle+=no_of_problems,i++) {
	Wcos = wcos[j_twiddle];
	Wsin = wsin[j_twiddle];
	for(j_first=0;  j_first<(no_of_problems/4);  j_first+=2) {
	  vroutdatabuffer[j+j_first] = spu_add(vrdatabuffer1[k+j_first],vrdatabuffer2[k+j_first]);
	  vioutdatabuffer[j+j_first] = spu_add(vidatabuffer1[k+j_first],vidatabuffer2[k+j_first]);
	  vtempr = spu_sub(vrdatabuffer1[k+j_first],vrdatabuffer2[k+j_first]);
	  vtempi = spu_sub(vidatabuffer1[k+j_first],vidatabuffer2[k+j_first]);
	  vroutdatabuffer[j+(no_of_problems/4)+j_first] = spu_sub(spu_mul(vtempr,spu_splats(Wcos)),spu_mul(vtempi,spu_splats(Wsin)));
	  vioutdatabuffer[j+(no_of_problems/4)+j_first] = spu_add(spu_mul(vtempr,spu_splats(Wsin)),spu_mul(vtempi,spu_splats(Wcos)));

	  vroutdatabuffer[j+j_first+1] = spu_add(vrdatabuffer1[k+j_first+1],vrdatabuffer2[k+j_first+1]);
	  vioutdatabuffer[j+j_first+1] = spu_add(vidatabuffer1[k+j_first+1],vidatabuffer2[k+j_first+1]);
	  vtemp1r = spu_sub(vrdatabuffer1[k+j_first+1],vrdatabuffer2[k+j_first+1]);
	  vtemp1i = spu_sub(vidatabuffer1[k+j_first+1],vidatabuffer2[k+j_first+1]);
	  vroutdatabuffer[j+(no_of_problems/4)+j_first+1] = spu_sub(spu_mul(vtemp1r,spu_splats(Wcos)),spu_mul(vtemp1i,spu_splats(Wsin)));
	  vioutdatabuffer[j+(no_of_problems/4)+j_first+1] = spu_add(spu_mul(vtemp1r,spu_splats(Wsin)),spu_mul(vtemp1i,spu_splats(Wcos)));
	  
	}
      }
      spu_mfcdma32(&(routdatabuffer[temp*BUFFER_SIZE*2]), addrputr+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
	      spu_mfcdma32(&(ioutdatabuffer[temp*BUFFER_SIZE*2]), addrputi+(j_initial+temp*BUFFER_SIZE*2)*sizeof(float),sizeof(float)*BUFFER_SIZE*2,0,MFC_PUT_CMD);
      //write back the currently computed results
    }
    addrswap = addrfetchr;
    addrfetchr = addrputr;
    addrputr = addrswap;
    addrswap = addrfetchi;
    addrfetchi = addrputi;
    addrputi = addrswap;
    no_of_problems *= 2;
    problem_size /= 2;
    dist *= 2;
    mfc_write_tag_mask(1<<0);
    mfc_read_tag_status_all();
    
    /********************************Synchronize**************************/
    synchronize_spes(MYTHREAD,2);
    /********************************Synchronize**************************/
  }
  //    prof_stop();

  vroutdata1 = (vector float *)routdata[0];
  vroutdata2 = (vector float *)routdata[1];
  vioutdata1 = (vector float *)ioutdata[0];
  vioutdata2 = (vector float *)ioutdata[1];
  while(problem_size > 1) {
    //the no of problems have increased beyond the buffer size .. we need to worry about the boundary cases now of where the chunk boundary would lie
    //initiate DMA get transfer for all buffers
    //    prof_clear();
    //    prof_start();
    for(i=0;i<buffer_count;i++) {
      //initialize multiple DMA transfers with same tag
      spu_mfcdma32(rdata[0][i],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+i*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
      spu_mfcdma32(idata[0][i],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+i*BUFFER_SIZE*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
      spu_mfcdma32(rdata[1][i],addrfetchr+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(i*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD); 
      spu_mfcdma32(idata[1][i],addrfetchi+(unsigned int)sizeof(float)*chunk_size*MYTHREAD+(i*BUFFER_SIZE+array_size/2)*sizeof(float),sizeof(float)*BUFFER_SIZE,0,MFC_GET_CMD);
    }

    
    mfc_write_tag_mask(1<<0);
    mfc_read_tag_status_all();
    //    prof_stop();

    k_initial = (int)(my_array_pos/no_of_problems);
    k_initial *= no_of_problems;
    j_first_initial = my_array_pos - k_initial;
    j_initial = 2*k_initial;
    j_twiddle = k_initial;
    
    //for this case  .. it is possible that j_first_initial != 0
    for(j=0,k=0 ;  k<(chunk_size/4); j+=(2*(no_of_problems/4)),k+=((no_of_problems-j_first_initial)/4),j_twiddle+=no_of_problems) {
      if(j != 0) {
	j_first_initial = 0;
      }
      Wcos = wcos[j_twiddle];
      Wsin = wsin[j_twiddle];
      
      for(j_first=j_first_initial/4; (j_first<no_of_problems/4)&&((k+j_first-j_first_initial/4)<chunk_size/4); j_first++) {
	vroutdata1[k+j_first-j_first_initial/4] = spu_add(vrdatabuffer1[k+j_first-j_first_initial/4], vrdatabuffer2[k+j_first-j_first_initial/4]);
	vioutdata1[k+j_first-j_first_initial/4] = spu_add(vidatabuffer1[k+j_first-j_first_initial/4], vidatabuffer2[k+j_first-j_first_initial/4]);
	vtempr = spu_sub(vrdatabuffer1[k+j_first-j_first_initial/4],vrdatabuffer2[k+j_first-j_first_initial/4]);
	vtempi = spu_sub(vidatabuffer1[k+j_first-j_first_initial/4],vidatabuffer2[k+j_first-j_first_initial/4]);
	vroutdata2[k+j_first-j_first_initial/4] = spu_sub(spu_mul(vtempr,spu_splats(Wcos)), spu_mul(vtempi,spu_splats(Wsin)));
	vioutdata2[k+j_first-j_first_initial/4] = spu_add(spu_mul(vtempr,spu_splats(Wsin)), spu_mul(vtempi,spu_splats(Wcos)));

	//complex multiplication
      }
      spu_mfcdma32(&(routdata[0][k*4]), addrputr+(j*4+j_initial+j_first_initial)*sizeof(float),sizeof(float)*(j_first*4-j_first_initial),1,MFC_PUT_CMD);
      spu_mfcdma32(&(ioutdata[0][k*4]), addrputi+(j*4+j_initial+j_first_initial)*sizeof(float),sizeof(float)*(j_first*4-j_first_initial),1,MFC_PUT_CMD);
      spu_mfcdma32(&(routdata[1][k*4]), addrputr+(j*4+j_initial+no_of_problems+j_first_initial)*sizeof(float),sizeof(float)*(j_first*4-j_first_initial),1,MFC_PUT_CMD);
      spu_mfcdma32(&(ioutdata[1][k*4]), addrputi+(j*4+j_initial+no_of_problems+j_first_initial)*sizeof(float),sizeof(float)*(j_first*4-j_first_initial),1,MFC_PUT_CMD);
	//      }
    }
    addrswap = addrfetchr;
    addrfetchr = addrputr;
    addrputr = addrswap;
    addrswap = addrfetchi;
    addrfetchi = addrputi;
    addrputi = addrswap;
    no_of_problems *= 2;
    problem_size /= 2;
    dist *= 2;
    mfc_write_tag_mask(1<<1);
    mfc_read_tag_status_all();
    
    /********************************Synchronize**************************/
    synchronize_spes(MYTHREAD,2);
    /********************************Synchronize**************************/
  }
#ifdef PROFILING
}
#endif
  if(MYTHREAD == 0) {
//        printf("ENDING\n");
    spu_write_out_mbox(1);
  }
 }
// prof_stop();
 //  tstop = spu_read_decrementer();
 //  fprintf(stdout,"Decrementer Cycle Counter: %u \n",tstart-tstop);
 
 return 0;
}
