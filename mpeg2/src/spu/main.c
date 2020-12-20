#include <spu_mfcio.h>
#include <stdio.h>
#include <stdlib.h>

#include <libmisc.h>

#define GLOBAL
#include "global_spu.h"
#undef GLOBAL

/* IBM CELL: SPE start point */
int main(unsigned long long speid, addr64 argp, addr64 envp)
{
  int i;
  int bx, by;

  int jobDataAddr;
  int firstTime = 1;

  Initialize_Fast_IDCT();

  /* This buffer is universal and only needs to be created
   * once per lifetime
   */
  if (!(SPU_Clip=(unsigned char *)malloc(1024)))
  {
    fprintf(stderr, "Error: malloc Clip buffer on SPU\n");
    return 0;
  }

  SPU_Clip += 384;

  for (i=-384; i<640; i++)
    SPU_Clip[i] = (i<0) ? 0 : ((i>255) ? 255 : i);

  while(1)
  {
    /* Wait for another operation */
    jobDataAddr = spu_read_in_mbox();

    if( jobDataAddr == SPU_OP_WORK )
      jobDataAddr = spu_read_in_mbox();
    else if( jobDataAddr == SPU_OP_SYNC )
      continue;
    else if( jobDataAddr == SPU_OP_EXIT )
      break;

    mfc_get(&jobData, 
	    jobDataAddr, 
	    sizeof(struct spuJobData), 
	    JOBDATA_GET_TAG, 0, 0);

    mfc_write_tag_mask(1 << JOBDATA_GET_TAG);
    mfc_read_tag_status_all();

    bitstreamPos = 0;
    base = &(jobData.sliceData);
    spu_ld = base;
    spu_ldData = &baseData;

    chromSliceSize = jobData.Chroma_Width * (jobData.chroma_format == CHROMA420 ? 8 : 16);
    lumaSliceSize = jobData.Coded_Picture_Width * 16;

    if( __builtin_expect(firstTime, 0) )
    {
      forwardref_frame[0][0] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Coded_Picture_Width<<1)*16, 4);
      forwardref_frame[0][1] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Chroma_Width<<1)*16, 4);
      forwardref_frame[0][2] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Chroma_Width<<1)*16, 4);

      forwardref_frame[1][0] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Coded_Picture_Width<<1)*16, 4);
      forwardref_frame[1][1] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Chroma_Width<<1)*16, 4);
      forwardref_frame[1][2] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Chroma_Width<<1)*16, 4);

      backref_frame[0] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Coded_Picture_Width<<1) * 16, 4);
      backref_frame[1] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Chroma_Width<<1)*16, 4);
      backref_frame[2] = (unsigned char*)malloc_align(sizeof(char) * (jobData.Chroma_Width<<1)*16, 4);

      current_frame[0] = (unsigned char*)malloc_align(sizeof(char) * lumaSliceSize, 4);
      current_frame[1] = (unsigned char*)malloc_align(sizeof(char) * chromSliceSize, 4);
      current_frame[2] = (unsigned char*)malloc_align(sizeof(char) * chromSliceSize, 4);

      firstTime = 0;
    }  

    Initialize_Buffer();

    Fault_Flag = 0;

    /* decode slice header (may change quantizer_scale) */
    int slice_vert_pos_ext;
    slice_vert_pos_ext = slice_header();

    /* decode macroblock address increment */
    jobData.MBAinc = Get_macroblock_address_increment();

    if (Fault_Flag) 
    {
      printf("start_of_slice(): MBAinc unsuccessful\n");
      //return(0);   /* trigger: go to next slice */
      continue;
    }

    /* set current location */
    /* NOTE: the arithmetic used to derive macroblock_address below is
     *       equivalent to ISO/IEC 13818-2 section 6.3.17: Macroblock
     */
    jobData.MBA = ((slice_vert_pos_ext<<7) + (jobData.code&255) - 1)*jobData.mb_width + jobData.MBAinc - 1;
    jobData.MBAinc = 1; /* first macroblock in slice: not skipped */

    bx = (jobData.MBA%jobData.mb_width);
    by = (jobData.MBA/jobData.mb_width);

    /* Don't care about slice return value  */
    slice(jobData.framenum, 
	  jobData.MBAmax, jobData.MBA, jobData.MBAinc, 
	  jobData.dc_dct_pred, 
	  jobData.PMV);
    mfc_put(current_frame[0], 
	    jobData.current_frame[0] + (lumaSliceSize * by),// + bx,
	    lumaSliceSize,
	    SLICE_PUT_TAG, 0, 0);
    mfc_put(current_frame[1], 
	    jobData.current_frame[1] + (chromSliceSize * by),// + bx,
	    chromSliceSize,
	    SLICE_PUT_TAG, 0, 0);
    mfc_put(current_frame[2], 
	    jobData.current_frame[2] + (chromSliceSize * by),// + bx,
	    chromSliceSize,
	    SLICE_PUT_TAG, 0, 0);

    mfc_write_tag_mask(1<<SLICE_PUT_TAG);
    mfc_read_tag_status_all();
  }

  free_align(current_frame[0]);
  free_align(current_frame[1]);
  free_align(current_frame[2]);

  free_align(forwardref_frame[0][0]);
  free_align(forwardref_frame[0][1]);
  free_align(forwardref_frame[0][2]);

  free_align(forwardref_frame[1][0]);
  free_align(forwardref_frame[1][1]);
  free_align(forwardref_frame[1][2]);

  free_align(backref_frame[0]);
  free_align(backref_frame[1]);
  free_align(backref_frame[2]);
  
  return 0;
}
