#include <stdio.h>
#include <stdlib.h>

#include <spu_mfcio.h>

#include "global_spu.h"

void Initialize_Buffer()
{
  spu_ldData->Incnt = 0;
  spu_ldData->Rdptr = &(spu_ldData->Rdbfr[0]) + RDBFR_SIZE;
  spu_ldData->Rdmax = spu_ldData->Rdptr;

#ifdef VERIFY
  /*  only the verifier uses this particular bit counter 
   *  Bitcnt keeps track of the current parser position with respect
   *  to the video elementary stream being decoded, regardless 
   *  of whether or not it is wrapped within a systems layer stream 
   */
  spu_ld->Bitcnt = 0;
#endif

  spu_ldData->Bfr = 0;

  Flush_Buffer(0); /* fills valid data into bfr */
}

void Fill_Buffer()
{
  /* This transfers another chunk of data that was captured from
   * the file to decode
   */
  mfc_get(&(spu_ldData->Rdbfr[0]), 
	  jobData.bitstreamBuffer + bitstreamPos, 
	  RDBFR_SIZE * sizeof(char), 
	  RDBFR_GET_TAG, 0, 0);

  bitstreamPos += RDBFR_SIZE;
  //jobData.ReadBufferSize = jobData.ReadBufferSize - Buffer_Level;

  mfc_write_tag_mask(1 << RDBFR_GET_TAG);
  mfc_read_tag_status_all();

  spu_ldData->Rdptr = &(spu_ldData->Rdbfr[0]);
}

/* Trace_Flag output */
void Print_Bits(code,bits,len)
int code,bits,len;
{
  int i;
  for (i=0; i<len; i++)
    printf("%d",(code>>(bits-1-i))&1);
}

/* return next n bits (right adjusted) without advancing */

unsigned int Show_Bits(int N)

{
  return spu_ldData->Bfr >> (32-N);
}

void Flush_Buffer(int N)

{
  int Incnt;

  spu_ldData->Bfr <<= N;

  Incnt = spu_ldData->Incnt -= N;

  if (Incnt <= 24)
  {
    if (spu_ldData->Rdptr < &(spu_ldData->Rdbfr[0])+(RDBFR_SIZE-4))
    {
      do
      {
        spu_ldData->Bfr |= *spu_ldData->Rdptr++ << (24 - Incnt);
        Incnt += 8;
      }
      while (Incnt <= 24);
    }
    else
    {
      do
      {
        if (spu_ldData->Rdptr >= &(spu_ldData->Rdbfr[0])+RDBFR_SIZE)
          Fill_Buffer();
        spu_ldData->Bfr |= *spu_ldData->Rdptr++ << (24 - Incnt);
        Incnt += 8;
      }
      while (Incnt <= 24);
    }
    spu_ldData->Incnt = Incnt;
  }

#ifdef VERIFY 
  spu_ld->Bitcnt += N;
#endif /* VERIFY */

}


/* return next n bits (right adjusted) */

unsigned int Get_Bits(int N)

{
  unsigned int Val;

  Val = Show_Bits(N);
  Flush_Buffer(N);

  return Val;
}

/* return next bit (could be made faster than Get_Bits(1)) */

unsigned int Get_Bits1()
{
  return Get_Bits(1);
}
