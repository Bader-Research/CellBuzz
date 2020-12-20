/* idct.c, inverse fast discrete cosine transform                           */

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

/**********************************************************/
/* inverse two dimensional DCT, Chen-Wang algorithm       */
/* (cf. IEEE ASSP-32, pp. 803-816, Aug. 1984)             */
/* 32-bit integer arithmetic (8 bit coefficients)         */
/* 11 mults, 29 adds per DCT                              */
/*                                      sE, 18.8.91       */
/**********************************************************/
/* coefficients extended to 12 bit for IEEE1180-1990      */
/* compliance                           sE,  2.1.94       */
/**********************************************************/

/* this code assumes >> to be a two's-complement arithmetic */
/* right shift: (-2)>>1 == -1 , (-3)>>1 == -2               */

#include "global_spu.h"

#define W1 2841 /* 2048*sqrt(2)*cos(1*pi/16) */
#define W2 2676 /* 2048*sqrt(2)*cos(2*pi/16) */
#define W3 2408 /* 2048*sqrt(2)*cos(3*pi/16) */
#define W5 1609 /* 2048*sqrt(2)*cos(5*pi/16) */
#define W6 1108 /* 2048*sqrt(2)*cos(6*pi/16) */
#define W7 565  /* 2048*sqrt(2)*cos(7*pi/16) */

/* global declarations */
void Initialize_Fast_IDCT _ANSI_ARGS_((void));
void Fast_IDCT _ANSI_ARGS_((short *block));

/* private data */
static short iclip[1024]; /* clipping table */
static short *iclp;

/* private prototypes */
static void idctrow _ANSI_ARGS_((short *blk));
static void idctcol _ANSI_ARGS_((short *blk));

/* As written, the SIMD code is actually slower than what xlc generates
 * My guess is that it is able to perform the multiplies more efficiently
 */
#define USE_SIMD 0

#if USE_SIMD
static int onea[4];
static int oneb[4];
static int onec[4];
static int rshifts[4];
static int fourteens[] = {-14, -14, -14, -14};

static vector signed int *oneav = (vector signed int*)onea;
static vector signed int *onebv = (vector signed int*)oneb;
static vector signed int *onecv = (vector signed int*)onec;
static vector signed int *rshiftsv = (vector signed int*)rshifts;
static vector signed int *fourteensv = (vector signed int*)fourteens;
#endif


/* row (horizontal) IDCT
 *
 *           7                       pi         1
 * dst[k] = sum c[l] * src[l] * cos( -- * ( k + - ) * l )
 *          l=0                      8          2
 *
 * where: c[0]    = 128
 *        c[1..7] = 128*sqrt(2)
 */

static void idctrow(short *blk)
{
  int x0, x1, x2, x3, x4, x5, x6, x7, x8;

  /* shortcut */
  if (!((x1 = blk[4]<<11) | (x2 = blk[6]) | (x3 = blk[2]) |
        (x4 = blk[1]) | (x5 = blk[7]) | (x6 = blk[5]) | (x7 = blk[3])))
  {
    blk[0]=blk[1]=blk[2]=blk[3]=blk[4]=blk[5]=blk[6]=blk[7]=blk[0]<<3;
    return;
  }

  x0 = (blk[0]<<11) + 128; /* for proper rounding in the fourth stage */

  /* first stage */
#if !USE_SIMD
  x8 = W7*(x4+x5);
  x4 = x8 + (W1-W7)*x4;
  x5 = x8 - (W1+W7)*x5;
  x8 = W3*(x6+x7);
  x6 = x8 - (W3-W5)*x6;
  x7 = x8 - (W3+W5)*x7;
#else

  onea[0] = W7*x5;
  onea[1] = W7*x4;
  onea[2] = W3*x7;
  onea[3] = W3*x6;
  
  oneb[0] = W1* x4;
  oneb[1] = -W1 * x5;
  oneb[2] = W5 * x6;
  oneb[3] = -W5 * x7;

  *oneav = spu_add(*oneav, *onebv);
#endif
  
  /* second stage */
#if !USE_SIMD
  x8 = x0 + x1;
  x0 -= x1;
  x1 = W6*(x3+x2);
  x2 = x1 - (W2+W6)*x2;
  x3 = x1 + (W2-W6)*x3;
  x1 = x4 + x6;
  x4 -= x6;
  x6 = x5 + x7;
  x5 -= x7;
#else

  oneb[0] = onea[0];
  oneb[1] = onea[0];
  oneb[2] = onea[1];
  oneb[3] = onea[1];

  onec[0] = onea[2];
  onec[1] = -onea[2];
  onec[2] = onea[3];
  onec[3] = -onea[3];

  *oneav = spu_add(*onebv, *onecv);

  x8 = x1;
  x1 = onea[0];
  x4 = onea[1];
  x6 = onea[2];
  x5 = onea[3];

  onea[0] = W6*x3;
  onea[1] = W6*x2;
  onea[2] = x0;
  onea[3] = x0;

  oneb[0] = -W2*x2;
  oneb[1] = W2*x3;
  oneb[2] = x8;
  oneb[3] = -x8;

  *onecv = spu_add(*oneav, *onebv);
#endif

  /* third stage */
#if !USE_SIMD
  x7 = x8 + x3;
  x8 -= x3;
  x3 = x0 + x2;
  x0 -= x2;
  x2 = (181*(x4+x5)+128)>>8;
  x4 = (181*(x4-x5)+128)>>8;
#else

  oneb[0] = onec[2];
  oneb[1] = onec[2];
  oneb[2] = onec[3];
  oneb[3] = onec[3];

  onea[0] = onec[1];
  onea[1] = -onec[1];
  onea[2] = onec[0];
  onea[3] = -onec[0];

  *onecv = spu_add(*oneav, *onebv);

  x7 = onec[0];
  x8 = onec[1];
  x3 = onec[2];
  x0 = onec[3];
  
  x2 = (181*(x4+x5)+128)>>8;
  x4 = (181*(x4-x5)+128)>>8;
#endif
  
  /* fourth stage */
#if !USE_SIMD
  blk[0] = (x7+x1)>>8;
  blk[1] = (x3+x2)>>8;
  blk[2] = (x0+x4)>>8;
  blk[3] = (x8+x6)>>8;
  blk[4] = (x8-x6)>>8;
  blk[5] = (x0-x4)>>8;
  blk[6] = (x3-x2)>>8;
  blk[7] = (x7-x1)>>8;
#else

  onea[0] = x7;
  onea[1] = x3;
  onea[2] = x0;
  onea[3] = x8;

  oneb[0] = x1;
  oneb[1] = x2;
  oneb[2] = x4;
  oneb[3] = x6;

  *onecv = spu_add(*oneav, *onebv);
  *onebv = spu_sub(*oneav, *onebv);

  *oneav = spu_rlmask(*onecv, *rshiftsv);
  *onecv = spu_rlmask(*onebv, *rshiftsv);

  blk[0] = onea[0];
  blk[1] = onea[1];
  blk[2] = onea[2];
  blk[3] = onea[3];
  blk[4] = onec[3];
  blk[5] = onec[2];
  blk[6] = onec[1];
  blk[7] = onec[0];
#endif
}

/* column (vertical) IDCT
 *
 *             7                         pi         1
 * dst[8*k] = sum c[l] * src[8*l] * cos( -- * ( k + - ) * l )
 *            l=0                        8          2
 *
 * where: c[0]    = 1/1024
 *        c[1..7] = (1/1024)*sqrt(2)
 */
static void idctcol(short *blk)
{
  int x0, x1, x2, x3, x4, x5, x6, x7, x8;

  /* shortcut */
  if (!((x1 = (blk[8*4]<<8)) | (x2 = blk[8*6]) | (x3 = blk[8*2]) |
        (x4 = blk[8*1]) | (x5 = blk[8*7]) | (x6 = blk[8*5]) | (x7 = blk[8*3])))
  {
    blk[8*0]=blk[8*1]=blk[8*2]=blk[8*3]=blk[8*4]=blk[8*5]=blk[8*6]=blk[8*7]=
      iclp[(blk[8*0]+32)>>6];
    return;
  }

  x0 = (blk[8*0]<<8) + 8192;

  /* first stage */
#if !USE_SIMD
    x8 = W7*(x4+x5) + 4;
    x4 = (x8+(W1-W7)*x4)>>3;
    x5 = (x8-(W1+W7)*x5)>>3;
    x8 = W3*(x6+x7) + 4;
    x6 = (x8-(W3-W5)*x6)>>3;
    x7 = (x8-(W3+W5)*x7)>>3;
#else
  
  onea[0] = W7*x5;
  onea[1] = W7*x4;
  onea[2] = W3*x7;
  onea[3] = W3*x6;

  oneb[0] = W1*x4;
  oneb[1] = -W1*x5;
  oneb[2] = W5*x6;
  oneb[3] = -W5*x7;

  *onecv = spu_add(*oneav, *onebv);

  onea[0] = onea[1] = onea[2] = onea[3] = 4;
  
  *onebv = spu_add(*onecv, *oneav);

  /* TODO: For some reason this does not work -
   * shift each individually
   */
  //*onecv = spu_rlmask(*onebv, -3);

  x4 = oneb[0]>>3;
  x5 = oneb[1]>>3;
  x6 = oneb[2]>>3;
  x7 = oneb[3]>>3;
#endif
  
  /* second stage */
#if !USE_SIMD
  x8 = x0 + x1;
  x0 -= x1;
  x1 = W6*(x3+x2) + 4;
  x2 = (x1-(W2+W6)*x2)>>3;
  x3 = (x1+(W2-W6)*x3)>>3;
  x1 = x4 + x6;
  x4 -= x6;
  x6 = x5 + x7;
  x5 -= x7;
#else

  onea[0] = x4;
  onea[1] = x4;
  onea[2] = x5;
  onea[3] = x5;

  oneb[0] = x6;
  oneb[1] = -x6;
  oneb[2] = x7;
  oneb[3] = -x7;

  *onecv = spu_add(*oneav, *onebv);

  x8 = x1;
  x1 = onec[0];
  x4 = onec[1];
  x6 = onec[2];
  x5 = onec[3];

  onea[0] = x0;
  onea[1] = x0;
  onea[2] = W6*x3+4;
  onea[3] = W6*x2+4;

  oneb[0] = x8;
  oneb[1] = -x8;
  oneb[2] = -W2*x2;
  oneb[3] = W2*x3;

  *onecv = spu_add(*oneav, *onebv);

  onec[2] = onec[2]>>3;
  onec[3] = onec[3]>>3;
#endif
  
  /* third stage */
#if !USE_SIMD
  x7 = x8 + x3;
  x8 -= x3;
  x3 = x0 + x2;
  x0 -= x2;
  x2 = (181*(x4+x5)+128)>>8;
  x4 = (181*(x4-x5)+128)>>8;
#else

  onea[0] = onec[0];
  onea[1] = onec[0];
  onea[2] = onec[1];
  onea[3] = onec[1];

  oneb[0] = onec[3];
  oneb[1] = -onec[3];
  oneb[2] = onec[2];
  oneb[3] = -onec[2];

  *onecv = spu_add(*oneav, *onebv);

  x7 = onec[0];
  x8 = onec[1];
  x3 = onec[2];
  x0 = onec[3];

  x2 = (181*(x4+x5)+128)>>8;
  x4 = (181*(x4-x5)+128)>>8;
#endif
  
  /* fourth stage */
#if !USE_SIMD
  blk[8*0] = iclp[(x7+x1)>>14];
  blk[8*1] = iclp[(x3+x2)>>14];
  blk[8*2] = iclp[(x0+x4)>>14];
  blk[8*3] = iclp[(x8+x6)>>14];
  blk[8*4] = iclp[(x8-x6)>>14];
  blk[8*5] = iclp[(x0-x4)>>14];
  blk[8*6] = iclp[(x3-x2)>>14];
  blk[8*7] = iclp[(x7-x1)>>14];
#else

  onea[0] = x7;
  onea[1] = x3;
  onea[2] = x0;
  onea[3] = x8;

  oneb[0] = x1;
  oneb[1] = x2;
  oneb[2] = x4;
  oneb[3] = x6;

  *onecv = spu_add(*oneav, *onebv);
  *onebv = spu_sub(*oneav, *onebv);

  *oneav = spu_rlmask(*onecv, *fourteensv);
  *onecv = spu_rlmask(*onebv, *fourteensv);

  blk[0] = iclp[ onea[0] ];
  blk[8] = iclp[ onea[1] ];
  blk[16] = iclp[ onea[2] ];
  blk[24] = iclp[ onea[3] ];
  blk[32] = iclp[ onec[3] ];
  blk[40] = iclp[ onec[2] ];
  blk[48] = iclp[ onec[1] ];
  blk[56] = iclp[ onec[0] ];
#endif
}

/* two dimensional inverse discrete cosine transform */
void Fast_IDCT(short *block)
{
  int i;

#if USE_SIMD
  rshifts[0] = rshifts[1] = rshifts[2] = rshifts[3] = -8;
#endif
  for (i=0; i<8; i++)
    idctrow(block+8*i);

#if USE_SIMD
  rshifts[0] = rshifts[1] = rshifts[2] = rshifts[3] = -3;
#endif
  for (i=0; i<8; i++)
    idctcol(block+i);
}

void Initialize_Fast_IDCT()
{
  int i;

  iclp = iclip+512;
  for (i= -512; i<512; i++)
    iclp[i] = (i<-256) ? -256 : ((i>255) ? 255 : i);
}

