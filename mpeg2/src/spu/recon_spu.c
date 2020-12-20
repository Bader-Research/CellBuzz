#include <spu_mfcio.h>

#include "global_spu.h"

/* Predict.c, motion compensation routines                                    */

/* Copyright (C) 1996, MPEG Software Simulation Group. All Rights Reserved. */

/*
 * Disclaimer of Warranty
 *
 * These software programs are available to the user without any license fee or
 * royalty on an "as is" basis.  The MPEG Software Simulation Group disclaims
 * any and all warranties, whether express, implied, or statuary, including any
 * implied warranties or merchantability or of fitness for a particular
 * purpose.  In no event shall the copyright-holder be liable for any
 * incidental, punitive, or consequential damages of any kind whatsoever
 * arising from the use of these programs.
 *
 * This disclaimer of warranty extends to the user of these programs and user's
 * customers, employees, agents, transferees, successors, and assigns.
 *
 * The MPEG Software Simulation Group does not represent or warrant that the
 * programs furnished hereunder are free of infringement of any third-party
 * patents.
 *
 * Commercial implementations of MPEG-1 and MPEG-2 video, including shareware,
 * are subject to royalty fees to patent holders.  Many of these patents are
 * general enough such that they are unavoidable regardless of implementation
 * design.
 *
 */

#include <stdio.h>

/* private prototypes */
static void form_prediction _ANSI_ARGS_((unsigned char *src[], int sfield,
  unsigned char *dst[], int dfield,
  int lx, int lx2, int w, int h, int x, int y, int dx, int dy,
  int average_flag));

static void form_component_prediction _ANSI_ARGS_((unsigned char *src, unsigned char *dst, int lx, int lx2, int w, int h, int x, int y, int dx, int dy, int average_flag));

/* The following optimization functions were taken from mpeg2play */
static void rec _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			     int lx2, int h));
static void recc _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			      int lx2, int h));
static void reca _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			      int lx2, int h));
static void recac _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			       int lx2, int h));
static void rech _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			      int lx2, int h));
static void rechc _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			       int lx2, int h));
static void recha _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			       int lx2, int h));
static void rechac _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
				int lx2, int h));
static void recv _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			      int lx, int lx2, int h));
static void recvc _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			       int lx, int lx2, int h));
static void recva _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			       int lx, int lx2, int h));
static void recvac _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
				int lx, int lx2, int h));
static void rec4 _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			      int lx, int lx2, int h));
static void rec4c _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			       int lx, int lx2, int h));
static void rec4a _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
			       int lx, int lx2, int h));
static void rec4ac _ANSI_ARGS_((unsigned char *s, unsigned char *d, 
				int lx, int lx2, int h));

static int cacheOffsetsForward[2] = {-1, -1};
static int cacheOffsetsBack = -1;
static int cacheHeightsForward[2] = {-1, -1};
static int cacheHeightsBack = -1;
static int forwardEmpty = 0;


/* recon.c */
void form_predictions (int bx, int by, int macroblock_type, 
		       int motion_type, int PMV[2][2][2], int motion_vertical_field_select[2][2], 
		       int dmvector[2], int stwtype)
{
  int currentfield;
  unsigned char **predframe;
  int DMV[2][2];
  int stwtop, stwbot;

  stwtop = stwtype%3; /* 0:temporal, 1:(spat+temp)/2, 2:spatial */
  stwbot = stwtype/3;

  if ((macroblock_type & MACROBLOCK_MOTION_FORWARD) 
      || (jobData.picture_coding_type==P_TYPE))
  {
    if (jobData.picture_structure==FRAME_PICTURE)
    {
      if ((motion_type==MC_FRAME) 
	  || !(macroblock_type & MACROBLOCK_MOTION_FORWARD))
      {
        /* frame-based prediction (broken into top and bottom halves
	   for spatial scalability prediction purposes) */
        if (stwtop<2)
          form_prediction(forwardref_frame[0], 0,
			  current_frame, 0,
			  jobData.Coded_Picture_Width,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by,
			  PMV[0][0][0],PMV[0][0][1],
			  stwtop);

        if (stwbot<2)
          form_prediction(forwardref_frame[0],1,
			  current_frame,1,
			  jobData.Coded_Picture_Width,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by,
			  PMV[0][0][0],PMV[0][0][1],
			  stwbot);
      }
      else if (motion_type==MC_FIELD) /* field-based prediction */
      {
        /* top field prediction */
        if (stwtop<2)
          form_prediction(forwardref_frame[0],
			  motion_vertical_field_select[0][0],
			  current_frame,0,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  PMV[0][0][0],PMV[0][0][1]>>1,
			  stwtop);

        /* bottom field prediction */
        if (stwbot<2)
          form_prediction(forwardref_frame[0],
			  motion_vertical_field_select[1][0],
			  current_frame,1,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  PMV[1][0][0],PMV[1][0][1]>>1,
			  stwbot);
      }
      else if (motion_type==MC_DMV) /* dual prime prediction */
      {
        /* calculate derived motion vectors */
        Dual_Prime_Arithmetic(DMV,dmvector,PMV[0][0][0],PMV[0][0][1]>>1);

        if (stwtop<2)
        {
          /* predict top field from top field */
          form_prediction(forwardref_frame[0],0,
			  current_frame,0,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  PMV[0][0][0],PMV[0][0][1]>>1,
			  0);

          /* predict and add to top field from bottom field */
          form_prediction(forwardref_frame[0],1,
			  current_frame,0,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  DMV[0][0],DMV[0][1],
			  1);
        }

        if (stwbot<2)
        {
          /* predict bottom field from bottom field */
          form_prediction(forwardref_frame[0],1,
			  current_frame,1,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  PMV[0][0][0],PMV[0][0][1]>>1,
			  0);

          /* predict and add to bottom field from top field */
          form_prediction(forwardref_frame[0],0,
			  current_frame,1,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  DMV[1][0],DMV[1][1],
			  1);
        }
      }
      else
        /* invalid motion_type */
        printf("invalid motion_type\n");
    }
    else /* TOP_FIELD or BOTTOM_FIELD */
    {
      /* field picture */
      currentfield = (jobData.picture_structure==BOTTOM_FIELD);

      /* determine which frame to use for prediction */
      if ((jobData.picture_coding_type==P_TYPE) && jobData.Second_Field
	  && (currentfield!=motion_vertical_field_select[0][0]))
        predframe = backref_frame; /* same frame */
      else
        predframe = forwardref_frame[0]; /* previous frame */

      if ((motion_type==MC_FIELD)
	  || !(macroblock_type & MACROBLOCK_MOTION_FORWARD))
      {
        /* field-based prediction */
        if (stwtop<2)
          form_prediction(predframe,
			  motion_vertical_field_select[0][0],
			  current_frame,0,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,16,bx,by,
			  PMV[0][0][0],PMV[0][0][1],
			  stwtop);
      }
      else if (motion_type==MC_16X8)
      {
        if (stwtop<2)
        {
          form_prediction(predframe,
			  motion_vertical_field_select[0][0],
			  current_frame,0,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by,
			  PMV[0][0][0],PMV[0][0][1],
			  stwtop);

          /* determine which frame to use for lower half prediction */
          if ((jobData.picture_coding_type==P_TYPE) && jobData.Second_Field
	      && (currentfield!=motion_vertical_field_select[1][0]))
            predframe = backref_frame; /* same frame */
          else
            predframe = forwardref_frame[0]; /* previous frame */

          form_prediction(predframe,
			  motion_vertical_field_select[1][0],
			  current_frame,0,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by+8,
			  PMV[1][0][0],PMV[1][0][1],
			  stwtop);
        }
      }
      else if (motion_type==MC_DMV) /* dual prime prediction */
      {
        if (jobData.Second_Field)
          predframe = backref_frame; /* same frame */
        else
          predframe = forwardref_frame[0]; /* previous frame */

        /* calculate derived motion vectors */
        Dual_Prime_Arithmetic(DMV,dmvector,PMV[0][0][0],PMV[0][0][1]);

        /* predict from field of same parity */
        form_prediction(forwardref_frame[0],
			currentfield,
			current_frame,0,
			jobData.Coded_Picture_Width<<1,
			jobData.Coded_Picture_Width<<1,
			16,16,bx,by,
			PMV[0][0][0],PMV[0][0][1],
			0);

        /* predict from field of opposite parity */
        form_prediction(predframe,!currentfield,
			current_frame,0,
			jobData.Coded_Picture_Width<<1,
			jobData.Coded_Picture_Width<<1,
			16,16,bx,by,
			DMV[0][0],DMV[0][1],
			1);
      }
      else
        /* invalid motion_type */
        printf("invalid motion_type\n");
    }
    stwtop = stwbot = 1;
  }

  if (macroblock_type & MACROBLOCK_MOTION_BACKWARD)
  {
    if (jobData.picture_structure==FRAME_PICTURE)
    {
      if (motion_type==MC_FRAME)
      {
        /* frame-based prediction */
        if (stwtop<2)
          form_prediction(backref_frame,0,
			  current_frame,0,
			  jobData.Coded_Picture_Width,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by,
			  PMV[0][1][0],PMV[0][1][1],
			  stwtop);

        if (stwbot<2)
          form_prediction(backref_frame,1,
			  current_frame,1,
			  jobData.Coded_Picture_Width,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by,
			  PMV[0][1][0],PMV[0][1][1],
			  stwbot);
      }
      else /* field-based prediction */
      {
        /* top field prediction */
        if (stwtop<2)
          form_prediction(backref_frame,
			  motion_vertical_field_select[0][1],
			  current_frame,0,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  PMV[0][1][0],PMV[0][1][1]>>1,
			  stwtop);

        /* bottom field prediction */
        if (stwbot<2)
          form_prediction(backref_frame,
			  motion_vertical_field_select[1][1],
			  current_frame,1,
			  jobData.Coded_Picture_Width<<1,
			  jobData.Coded_Picture_Width<<1,
			  16,8,bx,by>>1,
			  PMV[1][1][0],PMV[1][1][1]>>1,
			  stwbot);
      }
    }
    else /* TOP_FIELD or BOTTOM_FIELD */
    {
      /* field picture */
      if (motion_type==MC_FIELD)
      {
        /* field-based prediction */
        form_prediction(backref_frame,
			motion_vertical_field_select[0][1],
			current_frame,0,
			jobData.Coded_Picture_Width<<1,
			jobData.Coded_Picture_Width<<1,
			16,16,bx,by,
			PMV[0][1][0],PMV[0][1][1],
			stwtop);
      }
      else if (motion_type==MC_16X8)
      {
        form_prediction(backref_frame,
			motion_vertical_field_select[0][1],
			current_frame,0,
			jobData.Coded_Picture_Width<<1,
			jobData.Coded_Picture_Width<<1,
			16,8,bx,by,
			PMV[0][1][0],PMV[0][1][1],
			stwtop);

        form_prediction(backref_frame,
			motion_vertical_field_select[1][1],
			current_frame,0,
			jobData.Coded_Picture_Width<<1,
			jobData.Coded_Picture_Width<<1,
			16,8,bx,by+8,
			PMV[1][1][0],PMV[1][1][1],
			stwtop);
      }
      else
        /* invalid motion_type */
        printf("invalid motion_type\n");
    }
  }
}

static void form_prediction (unsigned char *src[], int sfield,
			     unsigned char *dst[], int dfield,
			     int lx, int lx2, 
			     int w, int h, 
			     int x, int y, 
			     int dx, int dy,
			     int average_flag)
{
  int yint = dy>>1;
  int i = 0;
  int DMAflag = 0;
  int offsetLuma = (y+yint)*lx + (sfield?lx2>>1:0);
  int offsetChrom;
  int lxChrom, lx2Chrom, wChrom, xChrom, dxChrom, hChrom, yChrom, dyChrom;
  unsigned char **srcMain;

  lxChrom = lx;
  lx2Chrom = lx2;
  wChrom = w;
  xChrom = x;
  dxChrom = dx;
  hChrom = h;
  yChrom = y;
  dyChrom = dy;

  if (jobData.chroma_format!=CHROMA444)
  {
    lxChrom>>=1; lx2Chrom>>=1; wChrom>>=1; xChrom>>=1; dxChrom/=2;
  }

  if (jobData.chroma_format==CHROMA420)
  {
    hChrom>>=1; yChrom>>=1; dyChrom/=2;
  }

  offsetChrom = (yChrom+(dyChrom>>1))*lxChrom + (sfield?lx2Chrom>>1:0);

  if( src == forwardref_frame[0] )
  {
    if( cacheOffsetsForward[0] == offsetLuma &&
	cacheHeightsForward[0] == h )
    {
      src = forwardref_frame[0];
      forwardEmpty = 0;
    }
    else if( cacheOffsetsForward[1] == offsetLuma &&
	     cacheHeightsForward[1] == h )
    {
      src = forwardref_frame[1];
      forwardEmpty = 1;
    }
    else
    {
      DMAflag = 1;
      cacheOffsetsForward[forwardEmpty & 1] = offsetLuma;
      cacheHeightsForward[forwardEmpty & 1] = h;
      src = forwardref_frame[forwardEmpty & 1];
      forwardEmpty = (forwardEmpty + 1) & 1;
      srcMain = jobData.forward_reference_frame;
    }
  }
  else 
  {
    if( cacheOffsetsBack != offsetLuma ||
	cacheHeightsBack != h )
    {
      DMAflag = 1;
      cacheOffsetsBack = offsetLuma;
      cacheHeightsBack = h;
      srcMain = jobData.backward_reference_frame;
    }
  }

  if( DMAflag )
  {
    /* IBM CELL: Transfer in the reference slices necessary for motion
     * reconstruction and store the referenced range in case it is used
     * again
     * Because of the size of the slice, cache size is 1
     * TODO: Implement a better cache system where the length of the
     * motion vectors is taken into consideration
     */ 

    for( i = 0; i < h; ++i )
    {
      mfc_get(src[0] + lx2*i, 
	      srcMain[0] + offsetLuma + lx2*i,
	      lx2,
	      LUMAREF_GET_TAG, 0, 0);
    }
    
    for( i = 0; i < hChrom; ++i )
    {
      mfc_get(src[1] + lx2Chrom*i,
	      srcMain[1] + offsetChrom  + lx2Chrom*i,
	      lx2Chrom,
	      CHROMREF_GET_TAG, 0, 0);
      mfc_get(src[2] + lx2Chrom*i,
	      srcMain[2] + offsetChrom + lx2Chrom*i,
	      lx2Chrom,
	      CHROMREF_GET_TAG, 0, 0);
    }
  }

  mfc_write_tag_mask(1<<LUMAREF_GET_TAG);
  mfc_read_tag_status_all();

  /* Y */
  form_component_prediction(src[0],
			    dst[0]+(dfield?lx2>>1:0),
			    lx,lx2,w,h,x,y,dx,dy,average_flag);

  mfc_write_tag_mask(1<<CHROMREF_GET_TAG);
  mfc_read_tag_status_all();

  /* Cb */
  form_component_prediction(src[1],
			    dst[1]+(dfield?lx2Chrom>>1:0),
			    lxChrom,lx2Chrom,
			    wChrom,hChrom,
			    xChrom,yChrom,
			    dxChrom,dyChrom,
			    average_flag);

  /* Cr */
  form_component_prediction(src[2],
			    dst[2]+(dfield?lx2Chrom>>1:0),
			    lxChrom,lx2Chrom,
			    wChrom,hChrom,
			    xChrom,yChrom,
			    dxChrom,dyChrom,
			    average_flag);
}

/* ISO/IEC 13818-2 section 7.6.4: Forming predictions */
/* NOTE: the arithmetic below produces numerically equivalent results
 *  to 7.6.4, yet is more elegant. It differs in the following ways:
 *
 *   1. the vectors (dx, dy) are based on cartesian frame 
 *      coordiantes along a half-pel grid (always positive numbers)
 *      In contrast, vector[r][s][t] are differential (with positive and 
 *      negative values). As a result, deriving the integer vectors 
 *      (int_vec[t]) from dx, dy is accomplished by a simple right shift.
 *
 *   2. Half pel flags (xh, yh) are equivalent to the LSB (Least
 *      Significant Bit) of the half-pel coordinates (dx,dy).
 * 
 *
 *  NOTE: the work of combining predictions (ISO/IEC 13818-2 section 7.6.7)
 *  is distributed among several other stages.  This is accomplished by 
 *  folding line offsets into the source and destination (src,dst)
 *  addresses (note the call arguments to form_prediction() in Predict()),
 *  line stride variables lx and lx2, the block dimension variables (w,h), 
 *  average_flag, and by the very order in which Predict() is called.  
 *  This implementation design (implicitly different than the spec) 
 *  was chosen for its elegance.
*/

static void form_component_prediction (unsigned char *src, unsigned char *dst,
				       int lx, int lx2, int w, int h, int x, int y, int dx, int dy, int average_flag)
{
  int xint;      /* horizontal integer sample vector: analogous to int_vec[0] */
  int yint;      /* vertical integer sample vectors: analogous to int_vec[1] */
  int xh;        /* horizontal half sample flag: analogous to half_flag[0]  */
  int yh;        /* vertical half sample flag: analogous to half_flag[1]  */
  int i, j, v;
  unsigned char *s;    /* source pointer: analogous to pel_ref[][]   */
  unsigned char *d;    /* destination pointer:  analogous to pel_pred[][]  */

  /* half pel scaling for integer vectors */
  xint = dx>>1;
  yint = dy>>1;

  /* derive half pel flags */
  xh = dx & 1;
  yh = dy & 1;

  /* compute the linear address of pel_ref[][] and pel_pred[][] 
     based on cartesian/raster cordinates provided */
  //s = src + lx*(y+yint) + x + xint;
  //d = dst + lx*y + x;
  s = src + x + xint;
  d = dst + x;

  if (!xh && !yh)
    if (average_flag)
    {
      if (w!=8)
        reca(s,d,lx2,h);
      else
        recac(s,d,lx2,h);
    }
    else
    {
      if (w!=8)
        rec(s,d,lx2,h);
      else
        recc(s,d,lx2,h);
    }
  else if (!xh && yh)
    if (average_flag)
    {
      if (w!=8)
        recva(s,d,lx,lx2,h);
      else
        recvac(s,d,lx,lx2,h);
    }
    else
    {
      if (w!=8)
        recv(s,d,lx,lx2,h);
      else
        recvc(s,d,lx,lx2,h);
    }
  else if (xh && !yh)
    if (average_flag)
    {
      if (w!=8)
        recha(s,d,lx2,h);
      else
        rechac(s,d,lx2,h);
    }
    else
    {
      if (w!=8)
        rech(s,d,lx2,h);
      else
        rechc(s,d,lx2,h);
    }
  else /* if (xh && yh) */
    if (average_flag)
    {
      if (w!=8)
        rec4a(s,d,lx,lx2,h);
      else
        rec4ac(s,d,lx,lx2,h);
    }
    else
    {
      if (w!=8)
        rec4(s,d,lx,lx2,h);
      else
        rec4c(s,d,lx,lx2,h);
    }
}

static void rec(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  int j;

  for (j=0; j<h; j++)
  {
    d[0] = s[0];
    d[1] = s[1];
    d[2] = s[2];
    d[3] = s[3];
    d[4] = s[4];
    d[5] = s[5];
    d[6] = s[6];
    d[7] = s[7];
    d[8] = s[8];
    d[9] = s[9];
    d[10] = s[10];
    d[11] = s[11];
    d[12] = s[12];
    d[13] = s[13];
    d[14] = s[14];
    d[15] = s[15];
    s+= lx2;
    d+= lx2;
  }
}

static void recc(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  int j;

  for (j=0; j<h; j++)
  {
    d[0] = s[0];
    d[1] = s[1];
    d[2] = s[2];
    d[3] = s[3];
    d[4] = s[4];
    d[5] = s[5];
    d[6] = s[6];
    d[7] = s[7];
    s+= lx2;
    d+= lx2;
  }
}

static void reca(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  int j;

  for (j=0; j<h; j++)
  {
    d[0] = (unsigned int)(d[0] + s[0] + 1)>>1;
    d[1] = (unsigned int)(d[1] + s[1] + 1)>>1;
    d[2] = (unsigned int)(d[2] + s[2] + 1)>>1;
    d[3] = (unsigned int)(d[3] + s[3] + 1)>>1;
    d[4] = (unsigned int)(d[4] + s[4] + 1)>>1;
    d[5] = (unsigned int)(d[5] + s[5] + 1)>>1;
    d[6] = (unsigned int)(d[6] + s[6] + 1)>>1;
    d[7] = (unsigned int)(d[7] + s[7] + 1)>>1;
    d[8] = (unsigned int)(d[8] + s[8] + 1)>>1;
    d[9] = (unsigned int)(d[9] + s[9] + 1)>>1;
    d[10] = (unsigned int)(d[10] + s[10] + 1)>>1;
    d[11] = (unsigned int)(d[11] + s[11] + 1)>>1;
    d[12] = (unsigned int)(d[12] + s[12] + 1)>>1;
    d[13] = (unsigned int)(d[13] + s[13] + 1)>>1;
    d[14] = (unsigned int)(d[14] + s[14] + 1)>>1;
    d[15] = (unsigned int)(d[15] + s[15] + 1)>>1;
    s+= lx2;
    d+= lx2;
  }
}

static void recac(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  int j;

  for (j=0; j<h; j++)
  {
    d[0] = (unsigned int)(d[0] + s[0] + 1)>>1;
    d[1] = (unsigned int)(d[1] + s[1] + 1)>>1;
    d[2] = (unsigned int)(d[2] + s[2] + 1)>>1;
    d[3] = (unsigned int)(d[3] + s[3] + 1)>>1;
    d[4] = (unsigned int)(d[4] + s[4] + 1)>>1;
    d[5] = (unsigned int)(d[5] + s[5] + 1)>>1;
    d[6] = (unsigned int)(d[6] + s[6] + 1)>>1;
    d[7] = (unsigned int)(d[7] + s[7] + 1)>>1;
    s+= lx2;
    d+= lx2;
  }
}

static void rech(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  unsigned char *dp,*sp;
  int j;
  unsigned int s1,s2;

  sp = s;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0];
    dp[0] = (unsigned int)(s1+(s2=sp[1])+1)>>1;
    dp[1] = (unsigned int)(s2+(s1=sp[2])+1)>>1;
    dp[2] = (unsigned int)(s1+(s2=sp[3])+1)>>1;
    dp[3] = (unsigned int)(s2+(s1=sp[4])+1)>>1;
    dp[4] = (unsigned int)(s1+(s2=sp[5])+1)>>1;
    dp[5] = (unsigned int)(s2+(s1=sp[6])+1)>>1;
    dp[6] = (unsigned int)(s1+(s2=sp[7])+1)>>1;
    dp[7] = (unsigned int)(s2+(s1=sp[8])+1)>>1;
    dp[8] = (unsigned int)(s1+(s2=sp[9])+1)>>1;
    dp[9] = (unsigned int)(s2+(s1=sp[10])+1)>>1;
    dp[10] = (unsigned int)(s1+(s2=sp[11])+1)>>1;
    dp[11] = (unsigned int)(s2+(s1=sp[12])+1)>>1;
    dp[12] = (unsigned int)(s1+(s2=sp[13])+1)>>1;
    dp[13] = (unsigned int)(s2+(s1=sp[14])+1)>>1;
    dp[14] = (unsigned int)(s1+(s2=sp[15])+1)>>1;
    dp[15] = (unsigned int)(s2+sp[16]+1)>>1;
    sp+= lx2;
    dp+= lx2;
  }
}

static void rechc(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  unsigned char *dp,*sp;
  int j;
  unsigned int s1,s2;

  sp = s;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0];
    dp[0] = (unsigned int)(s1+(s2=sp[1])+1)>>1;
    dp[1] = (unsigned int)(s2+(s1=sp[2])+1)>>1;
    dp[2] = (unsigned int)(s1+(s2=sp[3])+1)>>1;
    dp[3] = (unsigned int)(s2+(s1=sp[4])+1)>>1;
    dp[4] = (unsigned int)(s1+(s2=sp[5])+1)>>1;
    dp[5] = (unsigned int)(s2+(s1=sp[6])+1)>>1;
    dp[6] = (unsigned int)(s1+(s2=sp[7])+1)>>1;
    dp[7] = (unsigned int)(s2+sp[8]+1)>>1;
    sp+= lx2;
    dp+= lx2;
  }
}

static void recha(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  unsigned char *dp,*sp;
  int j;
  unsigned int s1,s2;

  sp = s;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0];
    dp[0] = (dp[0] + ((unsigned int)(s1+(s2=sp[1])+1)>>1) + 1)>>1;
    dp[1] = (dp[1] + ((unsigned int)(s2+(s1=sp[2])+1)>>1) + 1)>>1;
    dp[2] = (dp[2] + ((unsigned int)(s1+(s2=sp[3])+1)>>1) + 1)>>1;
    dp[3] = (dp[3] + ((unsigned int)(s2+(s1=sp[4])+1)>>1) + 1)>>1;
    dp[4] = (dp[4] + ((unsigned int)(s1+(s2=sp[5])+1)>>1) + 1)>>1;
    dp[5] = (dp[5] + ((unsigned int)(s2+(s1=sp[6])+1)>>1) + 1)>>1;
    dp[6] = (dp[6] + ((unsigned int)(s1+(s2=sp[7])+1)>>1) + 1)>>1;
    dp[7] = (dp[7] + ((unsigned int)(s2+(s1=sp[8])+1)>>1) + 1)>>1;
    dp[8] = (dp[8] + ((unsigned int)(s1+(s2=sp[9])+1)>>1) + 1)>>1;
    dp[9] = (dp[9] + ((unsigned int)(s2+(s1=sp[10])+1)>>1) + 1)>>1;
    dp[10] = (dp[10] + ((unsigned int)(s1+(s2=sp[11])+1)>>1) + 1)>>1;
    dp[11] = (dp[11] + ((unsigned int)(s2+(s1=sp[12])+1)>>1) + 1)>>1;
    dp[12] = (dp[12] + ((unsigned int)(s1+(s2=sp[13])+1)>>1) + 1)>>1;
    dp[13] = (dp[13] + ((unsigned int)(s2+(s1=sp[14])+1)>>1) + 1)>>1;
    dp[14] = (dp[14] + ((unsigned int)(s1+(s2=sp[15])+1)>>1) + 1)>>1;
    dp[15] = (dp[15] + ((unsigned int)(s2+sp[16]+1)>>1) + 1)>>1;
    sp+= lx2;
    dp+= lx2;
  }
}

static void rechac(s,d,lx2,h)
unsigned char *s, *d;
int lx2,h;
{
  unsigned char *dp,*sp;
  int j;
  unsigned int s1,s2;

  sp = s;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0];
    dp[0] = (dp[0] + ((unsigned int)(s1+(s2=sp[1])+1)>>1) + 1)>>1;
    dp[1] = (dp[1] + ((unsigned int)(s2+(s1=sp[2])+1)>>1) + 1)>>1;
    dp[2] = (dp[2] + ((unsigned int)(s1+(s2=sp[3])+1)>>1) + 1)>>1;
    dp[3] = (dp[3] + ((unsigned int)(s2+(s1=sp[4])+1)>>1) + 1)>>1;
    dp[4] = (dp[4] + ((unsigned int)(s1+(s2=sp[5])+1)>>1) + 1)>>1;
    dp[5] = (dp[5] + ((unsigned int)(s2+(s1=sp[6])+1)>>1) + 1)>>1;
    dp[6] = (dp[6] + ((unsigned int)(s1+(s2=sp[7])+1)>>1) + 1)>>1;
    dp[7] = (dp[7] + ((unsigned int)(s2+sp[8]+1)>>1) + 1)>>1;
    sp+= lx2;
    dp+= lx2;
  }
}

static void recv(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    dp[0] = (unsigned int)(sp[0]+sp2[0]+1)>>1;
    dp[1] = (unsigned int)(sp[1]+sp2[1]+1)>>1;
    dp[2] = (unsigned int)(sp[2]+sp2[2]+1)>>1;
    dp[3] = (unsigned int)(sp[3]+sp2[3]+1)>>1;
    dp[4] = (unsigned int)(sp[4]+sp2[4]+1)>>1;
    dp[5] = (unsigned int)(sp[5]+sp2[5]+1)>>1;
    dp[6] = (unsigned int)(sp[6]+sp2[6]+1)>>1;
    dp[7] = (unsigned int)(sp[7]+sp2[7]+1)>>1;
    dp[8] = (unsigned int)(sp[8]+sp2[8]+1)>>1;
    dp[9] = (unsigned int)(sp[9]+sp2[9]+1)>>1;
    dp[10] = (unsigned int)(sp[10]+sp2[10]+1)>>1;
    dp[11] = (unsigned int)(sp[11]+sp2[11]+1)>>1;
    dp[12] = (unsigned int)(sp[12]+sp2[12]+1)>>1;
    dp[13] = (unsigned int)(sp[13]+sp2[13]+1)>>1;
    dp[14] = (unsigned int)(sp[14]+sp2[14]+1)>>1;
    dp[15] = (unsigned int)(sp[15]+sp2[15]+1)>>1;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}

static void recvc(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    dp[0] = (unsigned int)(sp[0]+sp2[0]+1)>>1;
    dp[1] = (unsigned int)(sp[1]+sp2[1]+1)>>1;
    dp[2] = (unsigned int)(sp[2]+sp2[2]+1)>>1;
    dp[3] = (unsigned int)(sp[3]+sp2[3]+1)>>1;
    dp[4] = (unsigned int)(sp[4]+sp2[4]+1)>>1;
    dp[5] = (unsigned int)(sp[5]+sp2[5]+1)>>1;
    dp[6] = (unsigned int)(sp[6]+sp2[6]+1)>>1;
    dp[7] = (unsigned int)(sp[7]+sp2[7]+1)>>1;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}

static void recva(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    dp[0] = (dp[0] + ((unsigned int)(sp[0]+sp2[0]+1)>>1) + 1)>>1;
    dp[1] = (dp[1] + ((unsigned int)(sp[1]+sp2[1]+1)>>1) + 1)>>1;
    dp[2] = (dp[2] + ((unsigned int)(sp[2]+sp2[2]+1)>>1) + 1)>>1;
    dp[3] = (dp[3] + ((unsigned int)(sp[3]+sp2[3]+1)>>1) + 1)>>1;
    dp[4] = (dp[4] + ((unsigned int)(sp[4]+sp2[4]+1)>>1) + 1)>>1;
    dp[5] = (dp[5] + ((unsigned int)(sp[5]+sp2[5]+1)>>1) + 1)>>1;
    dp[6] = (dp[6] + ((unsigned int)(sp[6]+sp2[6]+1)>>1) + 1)>>1;
    dp[7] = (dp[7] + ((unsigned int)(sp[7]+sp2[7]+1)>>1) + 1)>>1;
    dp[8] = (dp[8] + ((unsigned int)(sp[8]+sp2[8]+1)>>1) + 1)>>1;
    dp[9] = (dp[9] + ((unsigned int)(sp[9]+sp2[9]+1)>>1) + 1)>>1;
    dp[10] = (dp[10] + ((unsigned int)(sp[10]+sp2[10]+1)>>1) + 1)>>1;
    dp[11] = (dp[11] + ((unsigned int)(sp[11]+sp2[11]+1)>>1) + 1)>>1;
    dp[12] = (dp[12] + ((unsigned int)(sp[12]+sp2[12]+1)>>1) + 1)>>1;
    dp[13] = (dp[13] + ((unsigned int)(sp[13]+sp2[13]+1)>>1) + 1)>>1;
    dp[14] = (dp[14] + ((unsigned int)(sp[14]+sp2[14]+1)>>1) + 1)>>1;
    dp[15] = (dp[15] + ((unsigned int)(sp[15]+sp2[15]+1)>>1) + 1)>>1;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}

static void recvac(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    dp[0] = (dp[0] + ((unsigned int)(sp[0]+sp2[0]+1)>>1) + 1)>>1;
    dp[1] = (dp[1] + ((unsigned int)(sp[1]+sp2[1]+1)>>1) + 1)>>1;
    dp[2] = (dp[2] + ((unsigned int)(sp[2]+sp2[2]+1)>>1) + 1)>>1;
    dp[3] = (dp[3] + ((unsigned int)(sp[3]+sp2[3]+1)>>1) + 1)>>1;
    dp[4] = (dp[4] + ((unsigned int)(sp[4]+sp2[4]+1)>>1) + 1)>>1;
    dp[5] = (dp[5] + ((unsigned int)(sp[5]+sp2[5]+1)>>1) + 1)>>1;
    dp[6] = (dp[6] + ((unsigned int)(sp[6]+sp2[6]+1)>>1) + 1)>>1;
    dp[7] = (dp[7] + ((unsigned int)(sp[7]+sp2[7]+1)>>1) + 1)>>1;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}

static void rec4(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;
  unsigned int s1,s2,s3,s4;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0]; s3=sp2[0];
    dp[0] = (unsigned int)(s1+(s2=sp[1])+s3+(s4=sp2[1])+2)>>2;
    dp[1] = (unsigned int)(s2+(s1=sp[2])+s4+(s3=sp2[2])+2)>>2;
    dp[2] = (unsigned int)(s1+(s2=sp[3])+s3+(s4=sp2[3])+2)>>2;
    dp[3] = (unsigned int)(s2+(s1=sp[4])+s4+(s3=sp2[4])+2)>>2;
    dp[4] = (unsigned int)(s1+(s2=sp[5])+s3+(s4=sp2[5])+2)>>2;
    dp[5] = (unsigned int)(s2+(s1=sp[6])+s4+(s3=sp2[6])+2)>>2;
    dp[6] = (unsigned int)(s1+(s2=sp[7])+s3+(s4=sp2[7])+2)>>2;
    dp[7] = (unsigned int)(s2+(s1=sp[8])+s4+(s3=sp2[8])+2)>>2;
    dp[8] = (unsigned int)(s1+(s2=sp[9])+s3+(s4=sp2[9])+2)>>2;
    dp[9] = (unsigned int)(s2+(s1=sp[10])+s4+(s3=sp2[10])+2)>>2;
    dp[10] = (unsigned int)(s1+(s2=sp[11])+s3+(s4=sp2[11])+2)>>2;
    dp[11] = (unsigned int)(s2+(s1=sp[12])+s4+(s3=sp2[12])+2)>>2;
    dp[12] = (unsigned int)(s1+(s2=sp[13])+s3+(s4=sp2[13])+2)>>2;
    dp[13] = (unsigned int)(s2+(s1=sp[14])+s4+(s3=sp2[14])+2)>>2;
    dp[14] = (unsigned int)(s1+(s2=sp[15])+s3+(s4=sp2[15])+2)>>2;
    dp[15] = (unsigned int)(s2+sp[16]+s4+sp2[16]+2)>>2;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}

static void rec4c(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;
  unsigned int s1,s2,s3,s4;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0]; s3=sp2[0];
    dp[0] = (unsigned int)(s1+(s2=sp[1])+s3+(s4=sp2[1])+2)>>2;
    dp[1] = (unsigned int)(s2+(s1=sp[2])+s4+(s3=sp2[2])+2)>>2;
    dp[2] = (unsigned int)(s1+(s2=sp[3])+s3+(s4=sp2[3])+2)>>2;
    dp[3] = (unsigned int)(s2+(s1=sp[4])+s4+(s3=sp2[4])+2)>>2;
    dp[4] = (unsigned int)(s1+(s2=sp[5])+s3+(s4=sp2[5])+2)>>2;
    dp[5] = (unsigned int)(s2+(s1=sp[6])+s4+(s3=sp2[6])+2)>>2;
    dp[6] = (unsigned int)(s1+(s2=sp[7])+s3+(s4=sp2[7])+2)>>2;
    dp[7] = (unsigned int)(s2+sp[8]+s4+sp2[8]+2)>>2;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}

static void rec4a(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;
  unsigned int s1,s2,s3,s4;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0]; s3=sp2[0];
    dp[0] = (dp[0] + ((unsigned int)(s1+(s2=sp[1])+s3+(s4=sp2[1])+2)>>2) + 1)>>1;
    dp[1] = (dp[1] + ((unsigned int)(s2+(s1=sp[2])+s4+(s3=sp2[2])+2)>>2) + 1)>>1;
    dp[2] = (dp[2] + ((unsigned int)(s1+(s2=sp[3])+s3+(s4=sp2[3])+2)>>2) + 1)>>1;
    dp[3] = (dp[3] + ((unsigned int)(s2+(s1=sp[4])+s4+(s3=sp2[4])+2)>>2) + 1)>>1;
    dp[4] = (dp[4] + ((unsigned int)(s1+(s2=sp[5])+s3+(s4=sp2[5])+2)>>2) + 1)>>1;
    dp[5] = (dp[5] + ((unsigned int)(s2+(s1=sp[6])+s4+(s3=sp2[6])+2)>>2) + 1)>>1;
    dp[6] = (dp[6] + ((unsigned int)(s1+(s2=sp[7])+s3+(s4=sp2[7])+2)>>2) + 1)>>1;
    dp[7] = (dp[7] + ((unsigned int)(s2+(s1=sp[8])+s4+(s3=sp2[8])+2)>>2) + 1)>>1;
    dp[8] = (dp[8] + ((unsigned int)(s1+(s2=sp[9])+s3+(s4=sp2[9])+2)>>2) + 1)>>1;
    dp[9] = (dp[9] + ((unsigned int)(s2+(s1=sp[10])+s4+(s3=sp2[10])+2)>>2) + 1)>>1;
    dp[10] = (dp[10] + ((unsigned int)(s1+(s2=sp[11])+s3+(s4=sp2[11])+2)>>2) + 1)>>1;
    dp[11] = (dp[11] + ((unsigned int)(s2+(s1=sp[12])+s4+(s3=sp2[12])+2)>>2) + 1)>>1;
    dp[12] = (dp[12] + ((unsigned int)(s1+(s2=sp[13])+s3+(s4=sp2[13])+2)>>2) + 1)>>1;
    dp[13] = (dp[13] + ((unsigned int)(s2+(s1=sp[14])+s4+(s3=sp2[14])+2)>>2) + 1)>>1;
    dp[14] = (dp[14] + ((unsigned int)(s1+(s2=sp[15])+s3+(s4=sp2[15])+2)>>2) + 1)>>1;
    dp[15] = (dp[15] + ((unsigned int)(s2+sp[16]+s4+sp2[16]+2)>>2) + 1)>>1;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}

static void rec4ac(s,d,lx,lx2,h)
unsigned char *s, *d;
int lx,lx2,h;
{
  unsigned char *dp,*sp,*sp2;
  int j;
  unsigned int s1,s2,s3,s4;

  sp = s;
  sp2 = s+lx;
  dp = d;
  for (j=0; j<h; j++)
  {
    s1=sp[0]; s3=sp2[0];
    dp[0] = (dp[0] + ((unsigned int)(s1+(s2=sp[1])+s3+(s4=sp2[1])+2)>>2) + 1)>>1;
    dp[1] = (dp[1] + ((unsigned int)(s2+(s1=sp[2])+s4+(s3=sp2[2])+2)>>2) + 1)>>1;
    dp[2] = (dp[2] + ((unsigned int)(s1+(s2=sp[3])+s3+(s4=sp2[3])+2)>>2) + 1)>>1;
    dp[3] = (dp[3] + ((unsigned int)(s2+(s1=sp[4])+s4+(s3=sp2[4])+2)>>2) + 1)>>1;
    dp[4] = (dp[4] + ((unsigned int)(s1+(s2=sp[5])+s3+(s4=sp2[5])+2)>>2) + 1)>>1;
    dp[5] = (dp[5] + ((unsigned int)(s2+(s1=sp[6])+s4+(s3=sp2[6])+2)>>2) + 1)>>1;
    dp[6] = (dp[6] + ((unsigned int)(s1+(s2=sp[7])+s3+(s4=sp2[7])+2)>>2) + 1)>>1;
    dp[7] = (dp[7] + ((unsigned int)(s2+sp[8]+s4+sp2[8]+2)>>2) + 1)>>1;
    sp+= lx2;
    sp2+= lx2;
    dp+= lx2;
  }
}
