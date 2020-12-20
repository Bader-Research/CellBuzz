#include <stdio.h>

#include "global_spu.h"

static void motion_compensation _ANSI_ARGS_((int MBA, int macroblock_type, 
 int motion_type, int PMV[2][2][2], int motion_vertical_field_select[2][2], 
 int dmvector[2], int stwtype, int dct_type));

static void skipped_macroblock _ANSI_ARGS_((int dc_dct_pred[3], 
  int PMV[2][2][2], int *motion_type, int motion_vertical_field_select[2][2],
  int *stwtype, int *macroblock_type));

static void Add_Block _ANSI_ARGS_((int comp, int bx, int by,
  int dct_type, int addflag));
static void Clear_Block _ANSI_ARGS_((int comp));
//static void Sum_Block _ANSI_ARGS_((int comp));
static void Saturate _ANSI_ARGS_((short *bp));

static int decode_macroblock _ANSI_ARGS_((int *macroblock_type, 
  int *stwtype, int *stwclass, int *motion_type, int *dct_type,
  int PMV[2][2][2], int dc_dct_pred[3], 
  int motion_vertical_field_select[2][2], int dmvector[2]));

static void macroblock_modes _ANSI_ARGS_((int *pmacroblock_type, int *pstwtype,
  int *pstwclass, int *pmotion_type, int *pmotion_vector_count, int *pmv_format, int *pdmv,
  int *pmvscale, int *pdct_type));

/* decode all macroblocks of the current picture */
/* ISO/IEC 13818-2 section 6.3.16 */
int slice(int framenum, int MBAmax, int MBA, int MBAinc, int dc_dct_pred[3], int PMV[2][2][2])
{
  int macroblock_type, motion_type, dct_type;
  int motion_vertical_field_select[2][2];
  int dmvector[2];
  int stwtype, stwclass;
  //int SNRMBA, SNRMBAinc;
  int ret;

  //if (Two_Streams && enhan.scalable_mode==SC_SNR)
  //{
  //  SNRMBA=0;
  //  SNRMBAinc=0;
  //}

  Fault_Flag=0;

  for (;;)
  {

    /* this is how we properly exit out of picture */
    if (MBA>=MBAmax)
      return(-1); /* all macroblocks decoded */

    if( Show_Bits(24) == 0x01L )
      return 0;

#ifdef TRACE
    if (jobData.Trace_Flag)
      printf("frame %d, MB %d\n",framenum,MBA);
#endif /* TRACE */

#ifdef DISPLAY
    /* Display should not be defined on the SPU side - can safely remove
       these, but leaving as a comparison to PPU reference */
    if (!progressive_frame && picture_structure==FRAME_PICTURE 
      && MBA==(MBAmax>>1) && framenum!=0 && Output_Type==T_X11 
       && !Display_Progressive_Flag)
    {
      Display_Second_Field();
    }
#endif

    //spu_ld = base;
    //spu_ldData = &baseData;

    if (MBAinc==0)
    {
      //if (base->scalable_mode==SC_DP && base->priority_breakpoint==1)
      //    ld = &enhan;

      if (!Show_Bits(23) || Fault_Flag) /* next_start_code or fault */
      {
resync: /* if Fault_Flag: resynchronize to next next_start_code */
        Fault_Flag = 0;
        return(0);     /* trigger: go to next slice */
      }
      else /* neither next_start_code nor Fault_Flag */
      {
        //if (base->scalable_mode==SC_DP && base->priority_breakpoint==1)
        //  ld = &enhan;

        /* decode macroblock address increment */
        MBAinc = Get_macroblock_address_increment();

        if (Fault_Flag) goto resync;
      }
    }

    if (MBA>=MBAmax)
    {
      /* MBAinc points beyond picture dimensions */
#ifdef TRACE
      if (!jobData.Quiet_Flag)
        printf("Too many macroblocks in picture\n");
#endif
      return(-1);
    }

    if (MBAinc==1) /* not skipped */
    {
      ret = decode_macroblock(&macroblock_type, &stwtype, &stwclass,
              &motion_type, &dct_type, PMV, dc_dct_pred, 
              motion_vertical_field_select, dmvector);

      if(ret==-1)
        return(-1);
   
      if(ret==0)
        goto resync;

    }
    else /* MBAinc!=1: skipped macroblock */
    {      
      /* ISO/IEC 13818-2 section 7.6.6 */
      skipped_macroblock(dc_dct_pred, PMV, &motion_type, 
        motion_vertical_field_select, &stwtype, &macroblock_type);
    }

    /* SCALABILITY: SNR */
    /* ISO/IEC 13818-2 section 7.8 */
    /* NOTE: we currently ignore faults encountered in this routine */
    //if (Two_Streams && enhan.scalable_mode==SC_SNR)
    //  Decode_SNR_Macroblock(&SNRMBA, &SNRMBAinc, MBA, MBAmax, &dct_type);

    /* ISO/IEC 13818-2 section 7.6 */
    motion_compensation(MBA, macroblock_type, motion_type, PMV, 
      motion_vertical_field_select, dmvector, stwtype, dct_type);


    /* advance to next macroblock */
    MBA++;
    MBAinc--;
 
    /* SCALABILITY: SNR */
    //if (Two_Streams && enhan.scalable_mode==SC_SNR)
    //{
    //  SNRMBA++;
    //  SNRMBAinc--;
    //}

    if (MBA>=MBAmax)
      return(-1); /* all macroblocks decoded */

    if( Show_Bits(24) == 0x01L )
      return 0;
  }
}

/* ISO/IEC 13818-2 section 7.6.6 */
static void skipped_macroblock (int dc_dct_pred[3], 
  int PMV[2][2][2], int *motion_type, int motion_vertical_field_select[2][2],
  int *stwtype, int *macroblock_type)
{
  int comp;
  
  /* SCALABILITY: Data Paritioning */
  /*
  if (base->scalable_mode==SC_DP)
  {
    spu_ld = base;
    spu_ldData = &baseData;
  }
  */

  for (comp=0; comp<jobData.block_count; comp++)
    Clear_Block(comp);

  /* reset intra_dc predictors */
  /* ISO/IEC 13818-2 section 7.2.1: DC coefficients in intra blocks */
  dc_dct_pred[0]=dc_dct_pred[1]=dc_dct_pred[2]=0;

  /* reset motion vector predictors */
  /* ISO/IEC 13818-2 section 7.6.3.4: Resetting motion vector predictors */
  if (jobData.picture_coding_type==P_TYPE)
    PMV[0][0][0]=PMV[0][0][1]=PMV[1][0][0]=PMV[1][0][1]=0;

  /* derive motion_type */
  if (jobData.picture_structure==FRAME_PICTURE)
    *motion_type = MC_FRAME;
  else
  {
    *motion_type = MC_FIELD;

    /* predict from field of same parity */
    /* ISO/IEC 13818-2 section 7.6.6.1 and 7.6.6.3: P field picture and B field
       picture */
    motion_vertical_field_select[0][0]=motion_vertical_field_select[0][1] = 
      (jobData.picture_structure==BOTTOM_FIELD);
  }

  /* skipped I are spatial-only predicted, */
  /* skipped P and B are temporal-only predicted */
  /* ISO/IEC 13818-2 section 7.7.6: Skipped macroblocks */
  *stwtype = (jobData.picture_coding_type==I_TYPE) ? 8 : 0;

 /* IMPLEMENTATION: clear MACROBLOCK_INTRA */
  *macroblock_type&= ~MACROBLOCK_INTRA;

}

/* IMPLEMENTATION: set scratch pad macroblock to zero */
static void Clear_Block(int comp)
{
  short *Block_Ptr;
  int i;

  Block_Ptr = spu_ld->block[comp];

  /*
  for (i=0; i<64; i++)
    *Block_Ptr++ = 0;
  */
  for( i = 0; i < 8; ++i )
  {
    Block_Ptr[0] = 0;
    Block_Ptr[1] = 0;
    Block_Ptr[2] = 0;
    Block_Ptr[3] = 0;
    Block_Ptr[4] = 0;
    Block_Ptr[5] = 0;
    Block_Ptr[6] = 0;
    Block_Ptr[7] = 0;

    Block_Ptr += 8;
  }
}

/* ISO/IEC 13818-2 sections 7.2 through 7.5 */
static int decode_macroblock (int *macroblock_type, 
  int *stwtype, int *stwclass, int *motion_type, int *dct_type,
  int PMV[2][2][2], int dc_dct_pred[3], 
  int motion_vertical_field_select[2][2], int dmvector[2])
{
  /* locals */
  int quantizer_scale_code; 
  int comp;

  int motion_vector_count; 
  int mv_format; 
  int dmv; 
  int mvscale;
  int coded_block_pattern;

  /* SCALABILITY: Data Patitioning */
  //if (base->scalable_mode==SC_DP)
  //{
  //  if (base->priority_breakpoint<=2)
  //    ld = &enhan;
  //  else
  //    ld = base;
  //}

  /* ISO/IEC 13818-2 section 6.3.17.1: Macroblock modes */
  macroblock_modes(macroblock_type, stwtype, stwclass,
    motion_type, &motion_vector_count, &mv_format, &dmv, &mvscale,
    dct_type);

  if (Fault_Flag) return(0);  /* trigger: go to next slice */

  if (*macroblock_type & MACROBLOCK_QUANT)
  {
    quantizer_scale_code = Get_Bits(5);

#ifdef TRACE
    if (jobData.Trace_Flag)
    {
      printf("quantiser_scale_code (");
      Print_Bits(quantizer_scale_code,5,5);
      printf("): %d\n",quantizer_scale_code);
    }
#endif /* TRACE */

    /* ISO/IEC 13818-2 section 7.4.2.2: Quantizer scale factor */
    if (spu_ld->MPEG2_Flag)
      spu_ld->quantizer_scale =
      spu_ld->q_scale_type ? Non_Linear_quantizer_scale[quantizer_scale_code] 
       : (quantizer_scale_code << 1);
    else
      spu_ld->quantizer_scale = quantizer_scale_code;

    /* SCALABILITY: Data Partitioning */
    //if (base->scalable_mode==SC_DP)
      /* make sure base->quantizer_scale is valid */
      //base->quantizer_scale = spu_ld->quantizer_scale;
  }

  /* motion vectors */


  /* ISO/IEC 13818-2 section 6.3.17.2: Motion vectors */

  /* decode forward motion vectors */
  if ((*macroblock_type & MACROBLOCK_MOTION_FORWARD) 
    || ((*macroblock_type & MACROBLOCK_INTRA) 
    && jobData.concealment_motion_vectors))
  {
    if (spu_ld->MPEG2_Flag)
      motion_vectors(PMV,dmvector,motion_vertical_field_select,
        0,motion_vector_count,mv_format,jobData.f_code[0][0]-1,jobData.f_code[0][1]-1,
        dmv,mvscale);
    else
      motion_vector(PMV[0][0],dmvector,
      jobData.forward_f_code-1,jobData.forward_f_code-1,0,0,jobData.full_pel_forward_vector);
  }

  if (Fault_Flag) return(0);  /* trigger: go to next slice */

  /* decode backward motion vectors */
  if (*macroblock_type & MACROBLOCK_MOTION_BACKWARD)
  {
    if (spu_ld->MPEG2_Flag)
      motion_vectors(PMV,dmvector,motion_vertical_field_select,
        1,motion_vector_count,mv_format,jobData.f_code[1][0]-1,jobData.f_code[1][1]-1,0,
        mvscale);
    else
      motion_vector(PMV[0][1],dmvector,
        jobData.backward_f_code-1,jobData.backward_f_code-1,0,0,jobData.full_pel_backward_vector);
  }

  if (Fault_Flag) return(0);  /* trigger: go to next slice */

  if ((*macroblock_type & MACROBLOCK_INTRA) && jobData.concealment_motion_vectors)
    Flush_Buffer(1); /* remove marker_bit */

  //if (base->scalable_mode==SC_DP && base->priority_breakpoint==3)
  //  ld = &enhan;

  /* macroblock_pattern */
  /* ISO/IEC 13818-2 section 6.3.17.4: Coded block pattern */
  if (*macroblock_type & MACROBLOCK_PATTERN)
  {
    coded_block_pattern = Get_coded_block_pattern();

    if (jobData.chroma_format==CHROMA422)
    {
      /* coded_block_pattern_1 */
      coded_block_pattern = (coded_block_pattern<<2) | Get_Bits(2); 

#ifdef TRACE
       if (jobData.Trace_Flag)
       {
         printf("coded_block_pattern_1: ");
         Print_Bits(coded_block_pattern,2,2);
         printf(" (%d)\n",coded_block_pattern&3);
       }
#endif /* TRACE */
     }
     else if (jobData.chroma_format==CHROMA444)
     {
      /* coded_block_pattern_2 */
      coded_block_pattern = (coded_block_pattern<<6) | Get_Bits(6); 

#ifdef TRACE
      if (jobData.Trace_Flag)
      {
        printf("coded_block_pattern_2: ");
        Print_Bits(coded_block_pattern,6,6);
        printf(" (%d)\n",coded_block_pattern&63);
      }
#endif /* TRACE */
    }
  }
  else
    coded_block_pattern = (*macroblock_type & MACROBLOCK_INTRA) ? 
      (1<<jobData.block_count)-1 : 0;

  if (Fault_Flag) return(0);  /* trigger: go to next slice */

  /* decode blocks */
  for (comp=0; comp<jobData.block_count; comp++)
  {
    /* SCALABILITY: Data Partitioning */
    //if (base->scalable_mode==SC_DP)
    //spu_ld = base;

    Clear_Block(comp);

    if (coded_block_pattern & (1<<(jobData.block_count-1-comp)))
    {
      if (*macroblock_type & MACROBLOCK_INTRA)
      {
        if (spu_ld->MPEG2_Flag)
          Decode_MPEG2_Intra_Block(comp,dc_dct_pred);
        else
          Decode_MPEG1_Intra_Block(comp,dc_dct_pred);
      }
      else
      {
        if (spu_ld->MPEG2_Flag)
          Decode_MPEG2_Non_Intra_Block(comp);
        else
          Decode_MPEG1_Non_Intra_Block(comp);
      }

      if (Fault_Flag) return(0);  /* trigger: go to next slice */
    }
  }

  if(jobData.picture_coding_type==D_TYPE)
  {
    /* remove end_of_macroblock (always 1, prevents startcode emulation) */
    /* ISO/IEC 11172-2 section 2.4.2.7 and 2.4.3.6 */
    marker_bit("D picture end_of_macroblock bit");
  }

  /* reset intra_dc predictors */
  /* ISO/IEC 13818-2 section 7.2.1: DC coefficients in intra blocks */
  if (!(*macroblock_type & MACROBLOCK_INTRA))
    dc_dct_pred[0]=dc_dct_pred[1]=dc_dct_pred[2]=0;

  /* reset motion vector predictors */
  if ((*macroblock_type & MACROBLOCK_INTRA) && !jobData.concealment_motion_vectors)
  {
    /* intra mb without concealment motion vectors */
    /* ISO/IEC 13818-2 section 7.6.3.4: Resetting motion vector predictors */
    PMV[0][0][0]=PMV[0][0][1]=PMV[1][0][0]=PMV[1][0][1]=0;
    PMV[0][1][0]=PMV[0][1][1]=PMV[1][1][0]=PMV[1][1][1]=0;
  }

  /* special "No_MC" macroblock_type case */
  /* ISO/IEC 13818-2 section 7.6.3.5: Prediction in P pictures */
  if ((jobData.picture_coding_type==P_TYPE) 
    && !(*macroblock_type & (MACROBLOCK_MOTION_FORWARD|MACROBLOCK_INTRA)))
  {
    /* non-intra mb without forward mv in a P picture */
    /* ISO/IEC 13818-2 section 7.6.3.4: Resetting motion vector predictors */
    PMV[0][0][0]=PMV[0][0][1]=PMV[1][0][0]=PMV[1][0][1]=0;

    /* derive motion_type */
    /* ISO/IEC 13818-2 section 6.3.17.1: Macroblock modes, frame_motion_type */
    if (jobData.picture_structure==FRAME_PICTURE)
      *motion_type = MC_FRAME;
    else
    {
      *motion_type = MC_FIELD;
      /* predict from field of same parity */
      motion_vertical_field_select[0][0] = (jobData.picture_structure==BOTTOM_FIELD);
    }
  }

  if (*stwclass==4)
  {
    /* purely spatially predicted macroblock */
    /* ISO/IEC 13818-2 section 7.7.5.1: Resetting motion vector predictions */
    PMV[0][0][0]=PMV[0][0][1]=PMV[1][0][0]=PMV[1][0][1]=0;
    PMV[0][1][0]=PMV[0][1][1]=PMV[1][1][0]=PMV[1][1][1]=0;
  }

  /* successfully decoded macroblock */
  return(1);

} /* decode_macroblock */

/* ISO/IEC 13818-2 section 7.6 */
static void motion_compensation(int MBA, int macroblock_type, int motion_type, int PMV[2][2][2], 
  int motion_vertical_field_select[2][2], int dmvector[2], int stwtype, int dct_type)
{
  int bx, by;
  int comp;

  /* derive current macroblock position within picture */
  /* ISO/IEC 13818-2 section 6.3.1.6 and 6.3.1.7 */
  bx = 16*(MBA%jobData.mb_width);
  by = 16*(MBA/jobData.mb_width);

  /* motion compensation */
  if (!(macroblock_type & MACROBLOCK_INTRA))
    form_predictions(bx,by,macroblock_type,motion_type,PMV,
      motion_vertical_field_select,dmvector,stwtype);
  
  /* SCALABILITY: Data Partitioning */
  //if (base->scalable_mode==SC_DP)
  //spu_ld = base;

  /* copy or add block data into picture */
  for (comp=0; comp<jobData.block_count; comp++)
  {
    /* SCALABILITY: SNR */
    /* ISO/IEC 13818-2 section 7.8.3.4: Addition of coefficients from 
       the two a layers */
    //if (Two_Streams && enhan.scalable_mode==SC_SNR)
    //  Sum_Block(comp); /* add SNR enhancement layer data to base layer */

    /* MPEG-2 saturation and mismatch control */
    /* base layer could be MPEG-1 stream, enhancement MPEG-2 SNR */
    /* ISO/IEC 13818-2 section 7.4.3 and 7.4.4: Saturation and Mismatch control */
    if (/*(Two_Streams && enhan.scalable_mode==SC_SNR) || */spu_ld->MPEG2_Flag)
      Saturate(spu_ld->block[comp]);

    /* ISO/IEC 13818-2 section Annex A: inverse DCT */
    Fast_IDCT(spu_ld->block[comp]);
    
    /* ISO/IEC 13818-2 section 7.6.8: Adding prediction and coefficient data */
    Add_Block(comp,bx,by,dct_type,(macroblock_type & MACROBLOCK_INTRA)==0);
  }

}

/* limit coefficients to -2048..2047 */
/* ISO/IEC 13818-2 section 7.4.3 and 7.4.4: Saturation and Mismatch control */
static void Saturate(short *Block_Ptr)
{
  int i, sum, val;

  sum = 0;

  /* ISO/IEC 13818-2 section 7.4.3: Saturation */
  for (i=0; i<64; i++)
  {
    val = Block_Ptr[i];

    if (val>2047)
      val = 2047;
    if (val<-2048)
      val = -2048;

    Block_Ptr[i] = val;
    sum+= val;
  }

  /* ISO/IEC 13818-2 section 7.4.4: Mismatch control */
  if ((sum&1)==0)
    Block_Ptr[63]^= 1;

}

/* move/add 8x8-Block from block[comp] to backward_reference_frame */
/* copy reconstructed 8x8 block from block[comp] to current_frame[]
 * ISO/IEC 13818-2 section 7.6.8: Adding prediction and coefficient data
 * This stage also embodies some of the operations implied by:
 *   - ISO/IEC 13818-2 section 7.6.7: Combining predictions
 *   - ISO/IEC 13818-2 section 6.1.3: Macroblock
*/
static void Add_Block (int comp, int bx, int by,
		       int dct_type, int addflag)
{
  int cc,i, j, iincr;
  unsigned char *rfp;
  short *bp;
  unsigned char *clp2;

  /* derive color component index */
  /* equivalent to ISO/IEC 13818-2 Table 7-1 */
  cc = (comp<4) ? 0 : (comp&1)+1; /* color component index */

  if (cc==0)
  {
    /* luminance */

    if (jobData.picture_structure==FRAME_PICTURE)
      if (dct_type)
      {
        /* field DCT coding */
        rfp = current_frame[0]
	  + jobData.Coded_Picture_Width*(((comp&2)>>1)) 
	  + bx + ((comp&1)<<3);
        iincr = (jobData.Coded_Picture_Width<<1);// - 8;
      }
      else
      {
        /* frame DCT coding */
        rfp = current_frame[0]
	  + jobData.Coded_Picture_Width*(((comp&2)<<2)) 
	  + bx + ((comp&1)<<3);
        iincr = jobData.Coded_Picture_Width;// - 8;
      }
    else
    {
      /* field picture */
      rfp = current_frame[0]
	+ (jobData.Coded_Picture_Width<<1)*(((comp&2)<<2)) 
	+ bx + ((comp&1)<<3);
      iincr = (jobData.Coded_Picture_Width<<1);// - 8;
    }
  }
  else
  {
    /* chrominance */

    /* scale coordinates */
    if (jobData.chroma_format!=CHROMA444)
      bx >>= 1;
    if (jobData.chroma_format==CHROMA420)
      by >>= 1;
    if (jobData.picture_structure==FRAME_PICTURE)
    {
      if (dct_type && (jobData.chroma_format!=CHROMA420))
      {
        /* field DCT coding */
        rfp = current_frame[cc]
	  + jobData.Chroma_Width*(((comp&2)>>1)) 
	  + bx + (comp&8);
        iincr = (jobData.Chroma_Width<<1);// - 8;
      }
      else
      {
        /* frame DCT coding */
        rfp = current_frame[cc]
	  + jobData.Chroma_Width*(((comp&2)<<2)) 
	  + bx + (comp&8);
        iincr = jobData.Chroma_Width;// - 8;
      }
    }
    else
    {
      /* field picture */
      rfp = current_frame[cc]
	+ (jobData.Chroma_Width<<1)*(((comp&2)<<2)) 
	+ bx + (comp&8);
      iincr = (jobData.Chroma_Width<<1);// - 8;
    }
  }

  bp = spu_ld->block[comp];
  clp2 = SPU_Clip;

  if (addflag)
  {
    for (i=0; i<8; i++)
    {
      /*
      for (j=0; j<8; j++)
      {
        *rfp = clp2[*bp++ + *rfp];
        rfp++;
      }
      */

      rfp[0] = clp2[ bp[0] + rfp[0] ];
      rfp[1] = clp2[ bp[1] + rfp[1] ];
      rfp[2] = clp2[ bp[2] + rfp[2] ];
      rfp[3] = clp2[ bp[3] + rfp[3] ];
      rfp[4] = clp2[ bp[4] + rfp[4] ];
      rfp[5] = clp2[ bp[5] + rfp[5] ];
      rfp[6] = clp2[ bp[6] + rfp[6] ];
      rfp[7] = clp2[ bp[7] + rfp[7] ];

      bp += 8;
      rfp+= iincr;
    }
  }
  else
  {
    clp2 += 128;

    for (i=0; i<8; i++)
    {
      /*
      for (j=0; j<8; j++)
        *rfp++ = clp2[*bp++ + 128];
      */

      rfp[0] = clp2[ bp[0] ];
      rfp[1] = clp2[ bp[1] ];
      rfp[2] = clp2[ bp[2] ];
      rfp[3] = clp2[ bp[3] ];
      rfp[4] = clp2[ bp[4] ];
      rfp[5] = clp2[ bp[5] ];
      rfp[6] = clp2[ bp[6] ];
      rfp[7] = clp2[ bp[7] ];

      bp += 8;
      rfp+= iincr;
    }
  }
}

/* ISO/IEC 13818-2 section 6.3.17.1: Macroblock modes */
static void macroblock_modes (int *pmacroblock_type, int *pstwtype,
  int *pstwclass, int *pmotion_type, int *pmotion_vector_count, int *pmv_format, int *pdmv,
  int *pmvscale, int *pdct_type)
{
  int macroblock_type;
  int stwtype, stwcode, stwclass;
  int motion_type = 0;
  int motion_vector_count, mv_format, dmv, mvscale;
  int dct_type;
  static unsigned char stwc_table[3][4]
    = { {6,3,7,4}, {2,1,5,4}, {2,5,7,4} };
  static unsigned char stwclass_table[9]
    = {0, 1, 2, 1, 1, 2, 3, 3, 4};

  /* get macroblock_type */
  macroblock_type = Get_macroblock_type();

  if (Fault_Flag) return;

  /* get spatial_temporal_weight_code */
  if (macroblock_type & MB_WEIGHT)
  {
    if (jobData.spatial_temporal_weight_code_table_index==0)
      stwtype = 4;
    else
    {
      stwcode = Get_Bits(2);
#ifdef TRACE
      if (jobData.Trace_Flag)
      {
        printf("spatial_temporal_weight_code (");
        Print_Bits(stwcode,2,2);
        printf("): %d\n",stwcode);
      }
#endif /* TRACE */
      stwtype = stwc_table[jobData.spatial_temporal_weight_code_table_index-1][stwcode];
    }
  }
  else
    stwtype = (macroblock_type & MB_CLASS4) ? 8 : 0;

  /* SCALABILITY: derive spatial_temporal_weight_class (Table 7-18) */
  stwclass = stwclass_table[stwtype];

  /* get frame/field motion type */
  if (macroblock_type & (MACROBLOCK_MOTION_FORWARD|MACROBLOCK_MOTION_BACKWARD))
  {
    if (jobData.picture_structure==FRAME_PICTURE) /* frame_motion_type */
    {
      motion_type = jobData.frame_pred_frame_dct ? MC_FRAME : Get_Bits(2);
#ifdef TRACE
      if (!jobData.frame_pred_frame_dct && jobData.Trace_Flag)
      {
        printf("frame_motion_type (");
        Print_Bits(motion_type,2,2);
        printf("): %s\n",motion_type==MC_FIELD?"Field":
                         motion_type==MC_FRAME?"Frame":
                         motion_type==MC_DMV?"Dual_Prime":"Invalid");
      }
#endif /* TRACE */
    }
    else /* field_motion_type */
    {
      motion_type = Get_Bits(2);
#ifdef TRACE
      if (jobData.Trace_Flag)
      {
        printf("field_motion_type (");
        Print_Bits(motion_type,2,2);
        printf("): %s\n",motion_type==MC_FIELD?"Field":
                         motion_type==MC_16X8?"16x8 MC":
                         motion_type==MC_DMV?"Dual_Prime":"Invalid");
      }
#endif /* TRACE */
    }
  }
  else if ((macroblock_type & MACROBLOCK_INTRA) && jobData.concealment_motion_vectors)
  {
    /* concealment motion vectors */
    motion_type = (jobData.picture_structure==FRAME_PICTURE) ? MC_FRAME : MC_FIELD;
  }
#if 0
  else
  {
    printf("maroblock_modes(): unknown macroblock type\n");
    motion_type = -1;
  }
#endif

  /* derive motion_vector_count, mv_format and dmv, (table 6-17, 6-18) */
  if (jobData.picture_structure==FRAME_PICTURE)
  {
    motion_vector_count = (motion_type==MC_FIELD && stwclass<2) ? 2 : 1;
    mv_format = (motion_type==MC_FRAME) ? MV_FRAME : MV_FIELD;
  }
  else
  {
    motion_vector_count = (motion_type==MC_16X8) ? 2 : 1;
    mv_format = MV_FIELD;
  }

  dmv = (motion_type==MC_DMV); /* dual prime */

  /* field mv predictions in frame pictures have to be scaled
   * ISO/IEC 13818-2 section 7.6.3.1 Decoding the motion vectors
   * IMPLEMENTATION: mvscale is derived for later use in motion_vectors()
   * it displaces the stage:
   *
   *    if((mv_format=="field")&&(t==1)&&(picture_structure=="Frame picture"))
   *      prediction = PMV[r][s][t] DIV 2;
   */

  mvscale = ((mv_format==MV_FIELD) && (jobData.picture_structure==FRAME_PICTURE));

  /* get dct_type (frame DCT / field DCT) */
  dct_type = (jobData.picture_structure==FRAME_PICTURE)
             && (!jobData.frame_pred_frame_dct)
             && (macroblock_type & (MACROBLOCK_PATTERN|MACROBLOCK_INTRA))
             ? Get_Bits(1)
             : 0;

#ifdef TRACE
  if (jobData.Trace_Flag  && (jobData.picture_structure==FRAME_PICTURE)
             && (!jobData.frame_pred_frame_dct)
             && (macroblock_type & (MACROBLOCK_PATTERN|MACROBLOCK_INTRA)))
    printf("dct_type (%d): %s\n",dct_type,dct_type?"Field":"Frame");
#endif /* TRACE */

  /* return values */
  *pmacroblock_type = macroblock_type;
  *pstwtype = stwtype;
  *pstwclass = stwclass;
  *pmotion_type = motion_type;
  *pmotion_vector_count = motion_vector_count;
  *pmv_format = mv_format;
  *pdmv = dmv;
  *pmvscale = mvscale;
  *pdct_type = dct_type;
}

///* SCALABILITY: add SNR enhancement layer block data to base layer */
///* ISO/IEC 13818-2 section 7.8.3.4: Addition of coefficients from the two layes */
//static void Sum_Block(comp)
//int comp;
//{
//  short *Block_Ptr1, *Block_Ptr2;
//  int i;
//
//  Block_Ptr1 = base->block[comp];
//  Block_Ptr2 = enhan.block[comp];
//
//  for (i=0; i<64; i++)
//    *Block_Ptr1++ += *Block_Ptr2++;
//}
