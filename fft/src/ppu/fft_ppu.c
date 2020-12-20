#include "../fft.h"
#include <sched.h>
#include <libspe2.h>
#include <stdio.h>
#include <errno.h>
#include <malloc_align.h>
#include <stdlib.h> 
#include <sys/time.h>
#include <math.h>
#include <dirent.h>
#include <pthread.h>

#define PI 3.1416

/* eight control blocks, one for each SPE. */
control_block cb[NO_SPU] __attribute__ ((aligned (128)));
sync_control_block sync_cb __attribute__ ((aligned (128)));

/* pointer to the SPE code, used at thread creation time */
extern spe_program_handle_t fft_spu; //for calculating head


typedef struct ppu_pthread_data {
  spe_context_ptr_t spe_ctx;
  pthread_t pthread;
  unsigned int flags;
  void *argp;
} ppu_pthread_data_t;

ppu_pthread_data_t data[NO_SPU];

//2 data arrays - A & B 
float *dataAr;
float *dataAi;
float *dataBr;
float *dataBi;
#ifdef CHECK_C
float *CdataAr;
float *CdataAi;
float *CdataBr;
float *CdataBi;

#endif
unsigned int chunk_size, array_size, log_array_size;
int no_of_stages;
int i,j,ii;
unsigned int mb_status;
unsigned int new;
unsigned int mb_value;
void *ps;
unsigned int usec;
#ifdef CHECK_C
unsigned int NP, ProblemSize, Dist;
float *getr,*geti,*putr,*puti,*swapr,*swapi;
unsigned int J,Jtwiddle,K;
float *wcos, *wsin,Wcos,Wsin;
unsigned int Jfirst;
float tempr,tempi;
unsigned int stage;
unsigned int loopover;
int flag = 1;
#endif

#ifdef CHECK_C
void compute_twiddle_factors(float *wcos,float *wsin,float size) {
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
#endif


void *ppu_pthread_function(void *arg) {
  ppu_pthread_data_t *datap = (ppu_pthread_data_t *)arg;
  unsigned int entry = SPE_DEFAULT_ENTRY;
  if (spe_context_run(datap->spe_ctx, &entry, datap->flags, datap->argp, NULL, NULL) < 0) {
    perror ("Failed running context");
    exit (1);
  }
  pthread_exit(NULL);
}


void load_fft_data(unsigned long array_size) {
  unsigned long i;
  srand(2);
  for (i=0; i<array_size; i++) {
    dataAr[i] = (float)rand()/ RAND_MAX;
    dataAi[i] = 0.0;
#ifdef CHECK_C
    CdataAr[i] = dataAr[i];
    CdataAi[i] = dataAi[i];
#endif
  }
}

float* initialize(int log_array_size) {
  
  if (log_array_size < 10 || log_array_size > 15) {
    printf("usage: fft <log of #elements in big array (10 <= x <= 15) >\n");
    return 0;
  }
  
  array_size = 1 << log_array_size;
  
  /* dividing task into the p spes, dividing array size by 2*p (chunk i and chunk i+p for spe i)*/
  chunk_size = array_size / (NO_SPU*2);

  /* the big array needs to be aligned on a 128-byte cache line */
  if ((dataAr = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  } 
  if ((dataAi = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  }

  if ((dataBr = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  }  
  
  if ((dataBi = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  }

#ifdef CHECK_C
  if ((CdataAr = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  }  if ((CdataAi = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  }  if ((CdataBr = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  }  if ((CdataBi = (float *)_malloc_align(array_size * sizeof(float), 7)) == NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    return 0;
  }
  wcos = (float *)malloc(sizeof(float)*array_size/2);
  wsin = (float *)malloc(sizeof(float)*array_size/2);
  
#endif
  
  
  fprintf(stderr, "ready to call (create) SPE threads\n"); 
  fflush(stderr);
  
  
  /* load the control blocks for each SPE with data, giving the address for start of data for each SPE*/
  for (i = 0; i < NO_SPU; i++) {
    cb[i].n = array_size; //fft of size n
    cb[i].threadno = i;
    //    cb[i].chunk_size = chunk_size * sizeof(float); /* convert to units of bytes */
    cb[i].addrDataAr     = (unsigned int) dataAr;
    cb[i].addrDataAi     = (unsigned int) dataAi;
    cb[i].addrDataBr     = (unsigned int) dataBr;
    cb[i].addrDataBi     = (unsigned int) dataBi;
  }
  /* allocate SPE tasks */
  for (i = 0; i < NO_SPU; i++) {
    if ((data[i].spe_ctx = spe_context_create (SPE_MAP_PS, NULL)) == NULL) {
      perror ("Failed creating context");
      exit (1);
    }
    if (spe_program_load (data[i].spe_ctx, &fft_spu))  {
      perror ("Failed loading program");
      exit (1);
    }
    data[i].argp = &cb[i];
    data[i].flags = SPE_MAP_PS;
  }
  /*************************************************************/  
  for(i=0;i<NO_SPU;i++) {
    sync_cb.rank = 1;
    if((sync_cb.sig1[i] = spe_ps_area_get(data[i].spe_ctx, SPE_SIG_NOTIFY_1_AREA))== NULL){
            printf("Failed call to spe_ps_get_area(%d)\n", i);
            return 0;
    }
    if((sync_cb.sig2[i] = spe_ps_area_get(data[i].spe_ctx, SPE_SIG_NOTIFY_2_AREA))== NULL){
            printf("Failed call to spe_get_ps_area(%d)\n", i);
            return 0;
    }
    
  }

  for(i=0;i<NO_SPU;i++) {
    if (pthread_create (&data[i].pthread, NULL, &ppu_pthread_function, &data[i])) {
      perror ("Failed creating thread");
      exit (1);
    }
  }

  unsigned int temp = (unsigned int)(&sync_cb);
  for(i=0;i<NO_SPU;i++) {
    if(spe_in_mbox_write(data[i].spe_ctx, &temp, 1, SPE_MBOX_ANY_NONBLOCKING) < 0) { 
      fprintf(stderr, "Failed writing messages to spe %d\n", i);
      exit(1);
    }
  }
  return (dataAr);
}

float* fft() {
  unsigned int temp = 2;
  if(spe_in_mbox_write(data[0].spe_ctx, &temp, 1, SPE_MBOX_ANY_NONBLOCKING) < 0) { 
    fprintf(stderr, "Failed writing messages to spe %d\n", i);
    exit(1);
  }

  while(!spe_out_mbox_status(data[0].spe_ctx)) {}
  spe_out_mbox_read(data[0].spe_ctx, &mb_value, 1);
}

int finalize() {
  //waiting for spes to execute last stage and exit
  struct timeval start, finish;
  unsigned int temp = 3;
  if(spe_in_mbox_write(data[0].spe_ctx, &temp, 1, SPE_MBOX_ANY_NONBLOCKING) < 0) { 
    fprintf(stderr, "Failed writing messages to spe %0\n", i);
    exit(1);
  }

  for(i=0;i<NO_SPU;i++) {
    if (pthread_join (data[i].pthread, NULL)) {
      perror ("Failed joining thread\n");
      exit (1);
    }
  }

  printf("SPEs task done.  Now checking results...\n"); fflush(stdout);


#ifdef CHECK_C
  printf("Checking for correctness ... \n");
  gettimeofday(&start,NULL);

  compute_twiddle_factors(wcos,wsin,array_size);
  getr = CdataAr;
  geti = CdataAi;
  putr = CdataBr;
  puti = CdataBi;
  NP = 1;
  ProblemSize = array_size;
  Dist = 1;
  stage = 0;
  while(ProblemSize > 1) {
    for(J=0,Jtwiddle=0,K=0;J<(array_size-1);J+=2*NP,K+=NP,Jtwiddle+=NP) {
      Wcos = wcos[Jtwiddle];
      Wsin = wsin[Jtwiddle];
      for(Jfirst=0;Jfirst<NP;Jfirst++) {
	putr[J+Jfirst] = getr[K+Jfirst] + getr[K+Jfirst+array_size/2];
	puti[J+Jfirst] = geti[K+Jfirst] + geti[K+Jfirst+array_size/2];
	tempr = getr[K+Jfirst] - getr[K+Jfirst+array_size/2];
	tempi = geti[K+Jfirst] - geti[K+Jfirst+array_size/2];
	putr[J+Jfirst+Dist] = tempr*Wcos-tempi*Wsin;
	puti[J+Jfirst+Dist] = tempr*Wsin+tempi*Wcos;
      }
    }
    stage++;
    NP *= 2;
    ProblemSize /= 2;
    Dist *= 2;
    swapr = getr;
    swapi = geti;
    getr = putr;
    geti = puti;
    putr = swapr;
    puti = swapi;
  }
  
  for(ii=0;ii<array_size;ii++) {
    if((fabs(dataAr[ii]-CdataAr[ii]) > 0.1)  || (fabs(dataAi[ii]-CdataAi[ii])>0.1)) 
      flag = 0;
  }
  if(flag) 
    printf("Correct Results !\n");
  else 
    printf("InCorrect Results !\n");
  gettimeofday(&finish,NULL);
  usec = finish.tv_sec*1000*1000 + finish.tv_usec;
  usec -= (start.tv_sec*1000*1000 + start.tv_usec);
  printf("USEC :  %u\n",usec);;
#endif

}


int main(int argc, char *argv[]) {
register unsigned int start,stop,diff;
  int iii,size,logsize;
  float *input, *output;
  logsize = 13;
  size = 1<<logsize;
  input = initialize(logsize);
  /* load the array with initial values (random for now) with imaginary values 0.0 */
  load_fft_data(size);

#ifdef PROFILING
    __asm__ volatile ("mftb %0, 268":"=r" (start));
#endif
    output = fft();
#ifdef PROFILING
    __asm__ volatile ("mftb %0, 268":"=r" (stop));
    diff = stop - start;
    printf ("usecs  = %f\n", (float) (diff) / 1431800.0f);
#endif
  finalize();
  /* Issue a sync, just to be safe. */
  __asm__ __volatile__ ("sync" : : : "memory");

  return 0;
}
