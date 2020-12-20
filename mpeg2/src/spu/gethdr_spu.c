#include "global_spu.h"

static int  extra_bit_information _ANSI_ARGS_((void));

/* ISO/IEC 13818-2 section 5.3 */
/* Purpose: this function is mainly designed to aid in bitstream conformance
   testing.  A simple Flush_Buffer(1) would do */
void marker_bit(char *text)
{
  int marker;

  marker = Get_Bits(1);

#ifdef VERIFY  
  if(!marker)
    printf("ERROR: %s--marker_bit set to 0",text);
#endif
}

/* decode slice header */

/* ISO/IEC 13818-2 section 6.2.4 */
int slice_header()
{
  int slice_vertical_position_extension;
  int quantizer_scale_code;
  int pos;
  int slice_picture_id_enable = 0;
  int slice_picture_id = 0;
  int extra_information_slice = 0;

  pos = spu_ld->Bitcnt;

  slice_vertical_position_extension =
    (spu_ld->MPEG2_Flag && jobData.vertical_size>2800) ? Get_Bits(3) : 0;

  if (spu_ld->scalable_mode==SC_DP)
    spu_ld->priority_breakpoint = Get_Bits(7);

  quantizer_scale_code = Get_Bits(5);
  spu_ld->quantizer_scale =
    spu_ld->MPEG2_Flag ? (spu_ld->q_scale_type ? Non_Linear_quantizer_scale[quantizer_scale_code] : quantizer_scale_code<<1) : quantizer_scale_code;

  /* slice_id introduced in March 1995 as part of the video corridendum
     (after the IS was drafted in November 1994) */
  if (Get_Bits(1))
  {
    spu_ld->intra_slice = Get_Bits(1);

    slice_picture_id_enable = Get_Bits(1);
	slice_picture_id = Get_Bits(6);

    extra_information_slice = extra_bit_information();
  }
  else
    spu_ld->intra_slice = 0;

#ifdef VERBOSE
  if (Verbose_Flag>PICTURE_LAYER)
  {
    printf("slice header (byte %d)\n",(pos>>3)-4);
    if (Verbose_Flag>SLICE_LAYER)
    {
      if (spu_ld->MPEG2_Flag && jobData.vertical_size>2800)
        printf("  slice_vertical_position_extension=%d\n",slice_vertical_position_extension);
  
      if (spu_ld->scalable_mode==SC_DP)
        printf("  priority_breakpoint=%d\n",spu_ld->priority_breakpoint);

      printf("  quantizer_scale_code=%d\n",quantizer_scale_code);

      printf("  slice_picture_id_enable = %d\n", slice_picture_id_enable);

      if(slice_picture_id_enable)
        printf("  slice_picture_id = %d\n", slice_picture_id);

    }
  }
#endif /* VERBOSE */

#ifdef VERIFY
  verify_slice_header++;
#endif /* VERIFY */


  return slice_vertical_position_extension;
}

/* decode extra bit information */
/* ISO/IEC 13818-2 section 6.2.3.4. */
static int extra_bit_information()
{
  int Byte_Count = 0;

  while (Get_Bits1())
  {
    Flush_Buffer(8);
    Byte_Count++;
  }

  return(Byte_Count);
}
