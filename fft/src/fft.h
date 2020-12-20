
#ifndef __fft_h__
#define __fft_h__


/*No of SPUs */
#define NO_SPU 8
#define LOG_NO_SPU 3

typedef struct _control_block {

  unsigned int  n;
  unsigned int threadno;
  //  unsigned int  chunk_size; /* size, in bytes, of each of these array pieces */
  unsigned int  addrDataAr;     /* address to be filled by double-buffered DMA */
  unsigned int  addrDataAi;     /* address to be filled by double-buffered DMA */
  unsigned int  addrDataBr;     /* address to be filled by double-buffered DMA */
  unsigned int  addrDataBi;     /* address to be filled by double-buffered DMA */
  unsigned char pad[104];   /* pad to a full cache line (128 bytes) */
} control_block;

typedef struct _sync_control_block {
  int rank;
  void* sig1[NO_SPU];
  void* sig2[NO_SPU];
  unsigned char pad[128-4*(2*NO_SPU+1)];   /* pad to a full cache line (128 bytes) */
} sync_control_block;


typedef union
{
  unsigned long long ull;
  unsigned int ui[2];
}
addr64;


#endif
