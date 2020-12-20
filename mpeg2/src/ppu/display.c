/* display.c, X11 interface                                                 */

/*
 * All modifications (mpeg2decode -> mpeg2play) are
 * Copyright (C) 1994, Stefan Eckart. All Rights Reserved.
 */

/* Copyright (C) 1994, MPEG Software Simulation Group. All Rights Reserved. */

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

 /* the Xlib interface is closely modeled after
  * mpeg_play 2.0 by the Berkeley Plateau Research Group
  */

/* if this is defined, then the truecolor will use a large (2MB) lookup table
 *   to do colorspace conversion, using 7 bits of Y, 6 of V and 6 of U.
 */
#ifdef DISPLAY

#define FASTTRUECOLOR


#include <stdio.h>
#include <stdlib.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include "config.h"
#include "global.h"

/* private prototypes */
static void display_image _ANSI_ARGS_((XImage *ximage, unsigned char *dithered_image));
static void ditherframe _ANSI_ARGS_((unsigned char *src[]));
static void dithertop _ANSI_ARGS_((unsigned char *src[], unsigned char *dst));
static void ditherbot _ANSI_ARGS_((unsigned char *src[], unsigned char *dst));
static void ditherframe444 _ANSI_ARGS_((unsigned char *src[]));
static void dithertop444 _ANSI_ARGS_((unsigned char *src[], unsigned char *dst));
static void ditherbot444 _ANSI_ARGS_((unsigned char *src[], unsigned char *dst));
//static void conv422to444 _ANSI_ARGS_((unsigned char *src, unsigned char *dst));
//static void conv420to422 _ANSI_ARGS_((unsigned char *src, unsigned char *dst));

void disp_truecolor_frame(unsigned char *src[], unsigned char *dest);
void disp_truecolor_top(unsigned char *src[], unsigned char *dest);
void disp_truecolor_bot(unsigned char *src[], unsigned char *dest);
void disp_truecolor_frame444(unsigned char *src[], unsigned char *dest);
void disp_truecolor_top444(unsigned char *src[], unsigned char *dest);
void disp_truecolor_bot444(unsigned char *src[], unsigned char *dest);

/* local data */
static unsigned char *dithered_image, *dithered_image2;
static int UsingTrueColor = 0 ;
static unsigned char ytab[16*(256+16)];
static unsigned char uvtab[256*269+270];
/* truecolor lookup tables */
#ifdef FASTTRUECOLOR
/* this is a malloc'ed array giving a lookup from uvy (in that order, to
 *   save a few nanoseconds somewhere) to rgb
 */
static unsigned long *uvytab ;
#else
/* these are a group of lookup tables with some of the truecolor
 * work precomputed
 */
static long rvtab[256], guvtab[256*256], butab[256], yytab[256];
#endif

/* X11 related variables */
static Display *display;
static Window window;
static GC gc;
static XImage *ximage, *ximage2;
static unsigned char pixel[256];

#ifdef SH_MEM

#include <sys/ipc.h>
#include <sys/shm.h>
#include <X11/extensions/XShm.h>

static int HandleXError _ANSI_ARGS_((Display *dpy, XErrorEvent *event));
static void InstallXErrorHandler _ANSI_ARGS_((void));
static void DeInstallXErrorHandler _ANSI_ARGS_((void));

static int shmem_flag;
static XShmSegmentInfo shminfo1, shminfo2;
static int gXErrorFlag;
static int CompletionType = -1;

static int HandleXError(dpy, event)
Display *dpy;
XErrorEvent *event;
{
  gXErrorFlag = 1;

  return 0;
}

static void InstallXErrorHandler()
{
  XSetErrorHandler(HandleXError);
  XFlush(display);
}

static void DeInstallXErrorHandler()
{
  XSetErrorHandler(NULL);
  XFlush(display);
}

#endif

/* connect to server, create and map window,
 * allocate colors and (shared) memory
 */
void Initialize_Display_Process(name)
char *name;
{
  int crv, cbu, cgu, cgv;
  int y, u, v, r, g, b;
  int i;
  char dummy;
  int screen;
  Colormap cmap;
  int private;
  XColor xcolor;
  unsigned int fg, bg;
  char *hello = "MPEG-2 Display";
  XSizeHints hint;
  XVisualInfo vinfo;
  XEvent xev;
  unsigned long tmp_pixel;
  XWindowAttributes xwa;

  display = XOpenDisplay(name);

  if (display == NULL)
    Error("Can not open display\n");

  screen = DefaultScreen(display);

  hint.x = 200;
  hint.y = 200;
  hint.width = horizontal_size;
  hint.height = vertical_size;
  hint.flags = /* PPosition | */ PSize;

  /* Get some colors */

  bg = WhitePixel (display, screen);
  fg = BlackPixel (display, screen);

  /* Make the window */

  /* Try for 24 bit colour first (unless instructed otherwise) */
  if ( XMatchVisualInfo(display, screen, 24, TrueColor, &vinfo)) {
    UsingTrueColor = 1; 
  } else if (!XMatchVisualInfo(display, screen, 8, PseudoColor, &vinfo))
  {
    if (!XMatchVisualInfo(display, screen, 8, GrayScale, &vinfo))
      Error("requires 8 (or 24) bit display\n");
  }

  {
    XSetWindowAttributes attrib ;
    attrib.background_pixel = bg ;
    attrib.border_pixel = fg ;
    if ( UsingTrueColor ) {
      /* need to get the colormap for TrueColor */
      cmap = XCreateColormap(display,DefaultRootWindow (display),
			     vinfo.visual,AllocNone);
      attrib.colormap = cmap ;
    }
    window = XCreateWindow (display, DefaultRootWindow (display),
			    hint.x, hint.y, hint.width, hint.height, 4,
			    UsingTrueColor ? 24 : CopyFromParent,
			    InputOutput,
			    UsingTrueColor ? vinfo.visual : CopyFromParent,
			    CWBorderPixel | CWBackPixel | ( UsingTrueColor ? CWColormap : 0), &attrib);
  }

  XSelectInput(display, window, StructureNotifyMask);

  /* Tell other applications about this window */

  XSetStandardProperties (display, window, hello, hello, None, NULL, 0, &hint);

  /* Map window. */

  XMapWindow(display, window);

  /* Wait for map. */
  do
  {
    XNextEvent(display, &xev);
  }
  while (xev.type != MapNotify || xev.xmap.event != window);

  XSelectInput(display, window, NoEventMask);

  /* matrix coefficients */
  crv = Inverse_Table_6_9[matrix_coefficients][0];
  cbu = Inverse_Table_6_9[matrix_coefficients][1];
  cgu = Inverse_Table_6_9[matrix_coefficients][2];
  cgv = Inverse_Table_6_9[matrix_coefficients][3];

  /* not quite sure how much of this we don't need for truecolor ... */
  /* allocate colors */

  if ( UsingTrueColor ) {
    gc = XCreateGC(display,window,0,NULL);
  } else {
    gc = DefaultGC(display, screen);
    cmap = DefaultColormap(display, screen);
  }
  private = 0;

  /* color allocation:
   * i is the (internal) 8 bit color number, it consists of separate
   * bit fields for Y, U and V: i = (yyyyuuvv), we don't use yyyy=0000
   * yyyy=0001 and yyyy=1111, this leaves 48 colors for other applications
   *
   * the allocated colors correspond to the following Y, U and V values:
   * Y:   40, 56, 72, 88, 104, 120, 136, 152, 168, 184, 200, 216, 232
   * U,V: -48, -16, 16, 48
   *
   * U and V values span only about half the color space; this gives
   * usually much better quality, although highly saturated colors can
   * not be displayed properly
   *
   * translation to R,G,B is implicitly done by the color look-up table
   */
  if ( ! UsingTrueColor ) {
    for (i=32; i<240; i++)
    {
      /* color space conversion */
      y = 16*((i>>4)&15) + 8;
      u = 32*((i>>2)&3)  - 48;
      v = 32*(i&3)       - 48;

      y = 76309 * (y - 16);	/* (255/219)*65536 */

      r = Clip[(y + crv*v + 32768)>>16];
      g = Clip[(y - cgu*u -cgv*v + 32768)>>16];
      b = Clip[(y + cbu*u + 32786)>>16];

      /* X11 colors are 16 bit */
      xcolor.red   = r << 8;
      xcolor.green = g << 8;
      xcolor.blue  = b << 8;

      if (XAllocColor(display, cmap, &xcolor) != 0)
	pixel[i] = xcolor.pixel;
      else
      {
	/* allocation failed, have to use a private colormap */

	if (private)
	  Error("Couldn't allocate private colormap");

	private = 1;

	if (!Quiet_Flag)
	  fprintf(stderr, "Using private colormap (%d colors were available).\n",
		  i-32);

	/* Free colors. */
	while (--i >= 32)
	{
	  tmp_pixel = pixel[i]; /* because XFreeColors expects unsigned long */
	  XFreeColors(display, cmap, &tmp_pixel, 1, 0);
	}

	/* i is now 31, this restarts the outer loop */

	/* create private colormap */

	XGetWindowAttributes(display, window, &xwa);
	cmap = XCreateColormap(display, window, xwa.visual, AllocNone);
	XSetWindowColormap(display, window, cmap);
      }
    }
  } else {
    /* compute the large lookup tables used in the display function */
#ifdef FASTTRUECOLOR
    /* this table takes 6,6,7 bits of u,v,y and produces an rgb Pixel.
       Note, incidentally, that this assumes the red mask is in the low
       byte; this is server-dependent, and the assumption should be removed,
       here and everywhere else it's made!
    */
    int u, v, y, U, V, Y, r, g, b;
    unsigned long *tab;
#else
    int u, v, y, crv, cbu, cgu, cgv;
#endif
    /* matrix coefficients */
    crv = Inverse_Table_6_9[matrix_coefficients][0];
    cbu = Inverse_Table_6_9[matrix_coefficients][1];
    cgu = Inverse_Table_6_9[matrix_coefficients][2];
    cgv = Inverse_Table_6_9[matrix_coefficients][3];
 
#ifdef FASTTRUECOLOR
    uvytab = (unsigned long *) malloc(4 * (1 << 19));
    if ( ! uvytab ) {
      fprintf(stderr,"malloc failed\n");
      exit(1);
    }
    tab = uvytab;
    for ( u = 0 ; u < 256 ; u += 4 )
      for ( v = 0 ; v < 256 ; v += 4 )
	for ( y = 0 ; y < 256 ; y += 2 ) {
	  Y = 76309 * (y - 16);
	  U = u - 128;
	  V = v - 128;
	  b = Clip[(Y + crv*V + 32768)>>16];
	  g = Clip[(Y - cgu*U - cgv*V + 32768)>>16];
	  r = Clip[(Y + cbu*U + 32786)>>16];
	  *tab++ = r | ((g << 8) & 0xFF00) | ((b << 16) & 0xFF0000);
	}
#else
    for ( u = 0 ; u < 256 ; u++ )
      for ( v = 0 ; v < 256 ; v++ ) {
	rvtab[v] = crv*(v-128) + 32768;
	guvtab[256*u + v] = - cgu*(u-128) - cgv*(v-128) + 32768;
	butab[u] = cbu*(u-128) + 32786;
      }
    for ( y = 0 ; y < 256 ; y++ )
      yytab[y] = 76309 * (y - 16);
#endif
  }

#ifdef SH_MEM
  if (XShmQueryExtension(display))
    shmem_flag = 1;
  else
  {
    shmem_flag = 0;
    if (!Quiet_Flag)
      fprintf(stderr, "Shared memory not supported\nReverting to normal Xlib\n");
  }

  if (shmem_flag)
    CompletionType = XShmGetEventBase(display) + ShmCompletion;

  InstallXErrorHandler();

  if (shmem_flag)
  {

    ximage = XShmCreateImage(display, vinfo.visual, UsingTrueColor ? 24 : 8, ZPixmap, NULL,
                             &shminfo1,
                             Coded_Picture_Width, Coded_Picture_Height);

    if (!progressive_sequence)
      ximage2 = XShmCreateImage(display, vinfo.visual, UsingTrueColor ? 24 : 8, ZPixmap, NULL,
                                &shminfo2,
                                Coded_Picture_Width, Coded_Picture_Height);

    /* Make sure we get the error */
    XSync(display,False);

    /* If no go, then revert to normal Xlib calls. */

    if (ximage==NULL || (!progressive_sequence && ximage2==NULL))
    {
      if (ximage!=NULL)
        XDestroyImage(ximage);
      if (!progressive_sequence && ximage2!=NULL)
        XDestroyImage(ximage2);
      if (!Quiet_Flag)
        fprintf(stderr, "Shared memory error, disabling (Ximage error)\n");
      goto shmemerror;
    }

    /* Success here, continue. */

    shminfo1.shmid = shmget(IPC_PRIVATE, 
                            ximage->bytes_per_line * ximage->height,
                            IPC_CREAT | 0777);
    if (!progressive_sequence)
      shminfo2.shmid = shmget(IPC_PRIVATE, 
                              ximage2->bytes_per_line * ximage2->height,
                              IPC_CREAT | 0777);

    if (shminfo1.shmid<0 || (!progressive_sequence && shminfo2.shmid<0))
    {
      XDestroyImage(ximage);
      if (!progressive_sequence)
        XDestroyImage(ximage2);
      if (!Quiet_Flag)
        fprintf(stderr, "Shared memory error, disabling (seg id error)\n");
      goto shmemerror;
    }

    shminfo1.shmaddr = (char *) shmat(shminfo1.shmid, 0, 0);
    shminfo2.shmaddr = (char *) shmat(shminfo2.shmid, 0, 0);

    if (shminfo1.shmaddr==((char *) -1) ||
        (!progressive_sequence && shminfo2.shmaddr==((char *) -1)))
    {
      XDestroyImage(ximage);
      if (shminfo1.shmaddr!=((char *) -1))
        shmdt(shminfo1.shmaddr);
      if (!progressive_sequence)
      {
        XDestroyImage(ximage2);
        if (shminfo2.shmaddr!=((char *) -1))
          shmdt(shminfo2.shmaddr);
      }
      if (!Quiet_Flag)
      {
        fprintf(stderr, "Shared memory error, disabling (address error)\n");
      }
      goto shmemerror;
    }

    ximage->data = shminfo1.shmaddr;
    dithered_image = (unsigned char *)ximage->data;
    shminfo1.readOnly = False;
    XShmAttach(display, &shminfo1);
    if (!progressive_sequence)
    {
      ximage2->data = shminfo2.shmaddr;
      dithered_image2 = (unsigned char *)ximage2->data;
      shminfo2.readOnly = False;
      XShmAttach(display, &shminfo2);
    }

    XSync(display, False);

    if (gXErrorFlag)
    {
      /* Ultimate failure here. */
      XDestroyImage(ximage);
      shmdt(shminfo1.shmaddr);
      if (!progressive_sequence)
      {
        XDestroyImage(ximage2);
        shmdt(shminfo2.shmaddr);
      }
      if (!Quiet_Flag)
        fprintf(stderr, "Shared memory error, disabling.\n");
      gXErrorFlag = 0;
      goto shmemerror;
    }
    else
    {
      shmctl(shminfo1.shmid, IPC_RMID, 0);
      if (!progressive_sequence)
        shmctl(shminfo2.shmid, IPC_RMID, 0);
    }

    if (!Quiet_Flag)
    {
      fprintf(stderr, "Sharing memory.\n");
    }
  }
  else
  {
  shmemerror:
    shmem_flag = 0;
#endif

    ximage = XCreateImage(display, vinfo.visual, UsingTrueColor ? 24 : 8,ZPixmap,0,&dummy,
                          Coded_Picture_Width,Coded_Picture_Height,UsingTrueColor ? 32 : 8,0);

    if (!(dithered_image = (unsigned char *)malloc( (UsingTrueColor ? 4 : 1 ) * Coded_Picture_Width*
						    Coded_Picture_Height)))
      Error("malloc failed");

    if (!progressive_sequence)
    {
      ximage2 = XCreateImage(display,vinfo.visual,UsingTrueColor ? 24 : 8,ZPixmap,0,&dummy,
                             Coded_Picture_Width,Coded_Picture_Height,UsingTrueColor ? 32 : 8,0);

      if (!(dithered_image2 = (unsigned char *)malloc((UsingTrueColor ? 4 : 1 ) * Coded_Picture_Width*
                                                      Coded_Picture_Height)))
        Error("malloc failed");
    }

#ifdef SH_MEM
  }

  DeInstallXErrorHandler();
#endif
}

void Terminate_Display_Process()
{
#ifdef SH_MEM
  if (shmem_flag)
  {
    XShmDetach(display, &shminfo1);
    XDestroyImage(ximage);
    shmdt(shminfo1.shmaddr);
    if (!progressive_sequence)
    {
      XShmDetach(display, &shminfo2);
      XDestroyImage(ximage2);
      shmdt(shminfo2.shmaddr);
    }
  }
#endif
}

static void display_image(ximage,dithered_image)
XImage *ximage;
unsigned char *dithered_image;
{
  /* display dithered image */
#ifdef SH_MEM
  if (shmem_flag)
  {
    XShmPutImage(display, window, gc, ximage, 
       	         0, 0, 0, 0, ximage->width, ximage->height, True);
    XFlush(display);
      
    while (1)
    {
      XEvent xev;
	
      XNextEvent(display, &xev);
      if (xev.type == CompletionType)
        break;
    }
  }
  else 
#endif
  {
    ximage->data = (char *) dithered_image; 
    XPutImage(display, window, gc, ximage, 0, 0, 0, 0, ximage->width, ximage->height);
  }
}

void Display_Second_Field()
{
  display_image(ximage2,dithered_image2);
}

/* 4x4 ordered dither
 *
 * threshold pattern:
 *   0  8  2 10
 *  12  4 14  6
 *   3 11  1  9
 *  15  7 13  5
 */

void Initialize_Dither_Matrix()
{
  int i, j, v;
  unsigned char ctab[256+32];


  if ( ! UsingTrueColor ) {
    for (i=0; i<256+16; i++)
      {
	v = (i-8)>>4;
	if (v<2)
	  v = 2;
	else if (v>14)
	  v = 14;
	for (j=0; j<16; j++)
	  ytab[16*i+j] = pixel[(v<<4)+j];
      }
    
    for (i=0; i<256+32; i++)
      {
	v = (i+48-128)>>5;
	if (v<0)
	  v = 0;
	else if (v>3)
	  v = 3;
	ctab[i] = v;
      }
    
    for (i=0; i<255+15; i++)
      for (j=0; j<255+15; j++)
	uvtab[256*i+j]=(ctab[i+16]<<6)|(ctab[j+16]<<4)|(ctab[i]<<2)|ctab[j];
  } else {

  }
}

void dither(src)
unsigned char *src[];
{
  if (progressive_sequence)
  {
      if ( UsingTrueColor ) {
	  if (chroma_format!=CHROMA444)
	      disp_truecolor_frame(src,dithered_image);
	  else
	      disp_truecolor_frame444(src,dithered_image);
      } else {
	  if (chroma_format!=CHROMA444) {
	      ditherframe(src);
	  } else {
	      ditherframe444(src);
	  }
      }
  }
  else
  {
    if ((picture_structure==FRAME_PICTURE && top_field_first) || picture_structure==BOTTOM_FIELD)
    {
      /* top field first */
	if ( UsingTrueColor ) {
	    if (chroma_format!=CHROMA444) {
		disp_truecolor_top(src,dithered_image);
		disp_truecolor_bot(src,dithered_image2);
	    } else {
		disp_truecolor_top444(src,dithered_image);
		disp_truecolor_bot444(src,dithered_image2);
	    }
	    	} else {
	    if (chroma_format!=CHROMA444)
		{
		    dithertop(src,dithered_image);
		    ditherbot(src,dithered_image2);
		}
	    else
		{
		    dithertop444(src,dithered_image);
		    ditherbot444(src,dithered_image2);
		}
	}
    }
    else
    {
      /* bottom field first */
	if ( UsingTrueColor ) {
	    if (chroma_format!=CHROMA444) {
		disp_truecolor_bot(src,dithered_image);
		disp_truecolor_top(src,dithered_image2);
	    } else {
		disp_truecolor_bot444(src,dithered_image);
		disp_truecolor_top444(src,dithered_image2);
	    }
	} else {
	    if (chroma_format!=CHROMA444)
		{
		    ditherbot(src,dithered_image);
		    dithertop(src,dithered_image2);
		}
	    else
		{
		    ditherbot444(src,dithered_image);
		    dithertop444(src,dithered_image2);
		}
	}
    }
  }
  display_image(ximage,dithered_image);
}


/* display for TrueColor ; based on store_ppm_tga in mpeg2dec/store.c */
/* In the interests of code reuse, the function is in a separate file,
   included three times with different defines.
   */
#ifdef FASTTRUECOLOR
/* we'll be working with pixels, not single bytes */
static unsigned long *prevdst = 0;
#else
static unsigned char *prevdst = 0 ; /* used by all three */
#endif
#define C444
#define FRAME0
#include "truecolor.inl"
#undef FRAME0
#define FIELD1
#include "truecolor.inl"
#undef FIELD1
#define FIELD2
#include "truecolor.inl"
#undef FIELD2
#undef C444
#define FRAME0
#include "truecolor.inl"
#undef FRAME0
#define FIELD1
#include "truecolor.inl"
#undef FIELD1
#define FIELD2
#include "truecolor.inl"
#undef FIELD2

/* only for 4:2:0 and 4:2:2! */

static void ditherframe(src)
unsigned char *src[];
{
  int i,j;
  unsigned int uv;
  unsigned char *py,*pu,*pv,*dst;

  py = src[0];
  pu = src[1];
  pv = src[2];
  dst = dithered_image;
  for (j=0; j<Coded_Picture_Height; j+=4)
  {
    /* line j + 0 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      uv = uvtab[(*pu++<<8)|*pv++];
      *dst++ = ytab[((*py++)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +8)<<4)|(uv>>4)];
      uv = uvtab[((*pu++<<8)|*pv++)+1028];
      *dst++ = ytab[((*py++ +2)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +10)<<4)|(uv>>4)];
      uv = uvtab[(*pu++<<8)|*pv++];
      *dst++ = ytab[((*py++)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +8)<<4)|(uv>>4)];
      uv = uvtab[((*pu++<<8)|*pv++)+1028];
      *dst++ = ytab[((*py++ +2)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +10)<<4)|(uv>>4)];
    }
    if (chroma_format==CHROMA420)
    {
      pu -= Chroma_Width;
      pv -= Chroma_Width;
    }

    /* line j + 1 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      uv = uvtab[((*pu++<<8)|*pv++)+2056];
      *dst++ = ytab[((*py++ +12)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +4)<<4)|(uv&15)];
      uv = uvtab[((*pu++<<8)|*pv++)+3084];
      *dst++ = ytab[((*py++ +14)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +6)<<4)|(uv&15)];
      uv = uvtab[((*pu++<<8)|*pv++)+2056];
      *dst++ = ytab[((*py++ +12)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +4)<<4)|(uv&15)];
      uv = uvtab[((*pu++<<8)|*pv++)+3084];
      *dst++ = ytab[((*py++ +14)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +6)<<4)|(uv&15)];
    }

    /* line j + 2 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      uv = uvtab[((*pu++<<8)|*pv++)+1542];
      *dst++ = ytab[((*py++ +3)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +11)<<4)|(uv>>4)];
      uv = uvtab[((*pu++<<8)|*pv++)+514];
      *dst++ = ytab[((*py++ +1)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +9)<<4)|(uv>>4)];
      uv = uvtab[((*pu++<<8)|*pv++)+1542];
      *dst++ = ytab[((*py++ +3)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +11)<<4)|(uv>>4)];
      uv = uvtab[((*pu++<<8)|*pv++)+514];
      *dst++ = ytab[((*py++ +1)<<4)|(uv&15)];
      *dst++ = ytab[((*py++ +9)<<4)|(uv>>4)];
    }

    if (chroma_format==CHROMA420)
    {
      pu -= Chroma_Width;
      pv -= Chroma_Width;
    }

    /* line j + 3 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      uv = uvtab[((*pu++<<8)|*pv++)+3598];
      *dst++ = ytab[((*py++ +15)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +7)<<4)|(uv&15)];
      uv = uvtab[((*pu++<<8)|*pv++)+2570];
      *dst++ = ytab[((*py++ +13)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +5)<<4)|(uv&15)];
      uv = uvtab[((*pu++<<8)|*pv++)+3598];
      *dst++ = ytab[((*py++ +15)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +7)<<4)|(uv&15)];
      uv = uvtab[((*pu++<<8)|*pv++)+2570];
      *dst++ = ytab[((*py++ +13)<<4)|(uv>>4)];
      *dst++ = ytab[((*py++ +5)<<4)|(uv&15)];
    }
  }
}

static void dithertop(src,dst)
unsigned char *src[];
unsigned char *dst;
{
  int i,j;
  unsigned int y,uv1,uv2;
  unsigned char *py,*py2,*pu,*pv,*dst2;

  py = src[0];
  py2 = src[0] + (Coded_Picture_Width<<1);
  pu = src[1];
  pv = src[2];
  dst2 = dst + Coded_Picture_Width;

  for (j=0; j<Coded_Picture_Height; j+=4)
  {
    /* line j + 0, j + 1 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y = *py++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2];
      uv2 = uvtab[uv2+2056];
      *dst++  = ytab[((y)<<4)|(uv1&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+12)<<4)|(uv2>>4)];

      y = *py++;
      *dst++  = ytab[((y+8)<<4)|(uv1>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+4)<<4)|(uv2&15)];

      y = *py++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2+1028];
      uv2 = uvtab[uv2+3072];
      *dst++  = ytab[((y+2)<<4)|(uv1&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+14)<<4)|(uv2>>4)];

      y = *py++;
      *dst++  = ytab[((y+10)<<4)|(uv1>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+6)<<4)|(uv2&15)];
    }

    py += Coded_Picture_Width;

    if (j!=(Coded_Picture_Height-4))
      py2 += Coded_Picture_Width;
    else
      py2 -= Coded_Picture_Width;

    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;

    if (chroma_format==CHROMA420)
    {
      pu -= Chroma_Width;
      pv -= Chroma_Width;
    }
    else
    {
      pu += Chroma_Width;
      pv += Chroma_Width;
    }

    /* line j + 2, j + 3 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y = *py++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2+1542];
      uv2 = uvtab[uv2+3598];
      *dst++  = ytab[((y+3)<<4)|(uv1&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+15)<<4)|(uv2>>4)];

      y = *py++;
      *dst++  = ytab[((y+11)<<4)|(uv1>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+7)<<4)|(uv2&15)];

      y = *py++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2+514];
      uv2 = uvtab[uv2+2570];
      *dst++  = ytab[((y+1)<<4)|(uv1&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+13)<<4)|(uv2>>4)];

      y = *py++;
      *dst++  = ytab[((y+9)<<4)|(uv1>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+5)<<4)|(uv2&15)];
    }

    py += Coded_Picture_Width;
    py2 += Coded_Picture_Width;
    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;
    pu += Chroma_Width;
    pv += Chroma_Width;
  }
}

static void ditherbot(src,dst)
unsigned char *src[];
unsigned char *dst;
{
  int i,j;
  unsigned int y2,uv1,uv2;
  unsigned char *py,*py2,*pu,*pv,*dst2;

  py = src[0] + Coded_Picture_Width;
  py2 = py;
  pu = src[1] + Chroma_Width;
  pv = src[2] + Chroma_Width;
  dst2 = dst + Coded_Picture_Width;

  for (j=0; j<Coded_Picture_Height; j+=4)
  {
    /* line j + 0, j + 1 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y2 = *py2++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2];
      uv2 = uvtab[uv2+2056];
      *dst++  = ytab[((((*py++ + y2)>>1))<<4)|(uv1&15)];
      *dst2++ = ytab[((y2+12)<<4)|(uv2>>4)];

      y2 = *py2++;
      *dst++  = ytab[((((*py++ + y2)>>1)+8)<<4)|(uv1>>4)];
      *dst2++ = ytab[((y2+4)<<4)|(uv2&15)];

      y2 = *py2++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2+1028];
      uv2 = uvtab[uv2+3072];
      *dst++  = ytab[((((*py++ + y2)>>1)+2)<<4)|(uv1&15)];
      *dst2++ = ytab[((y2+14)<<4)|(uv2>>4)];

      y2 = *py2++;
      *dst++  = ytab[((((*py++ + y2)>>1)+10)<<4)|(uv1>>4)];
      *dst2++ = ytab[((y2+6)<<4)|(uv2&15)];
    }

    if (j==0)
      py -= Coded_Picture_Width;
    else
      py += Coded_Picture_Width;

    py2 += Coded_Picture_Width;
    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;

    if (chroma_format==CHROMA420)
    {
      pu -= Chroma_Width;
      pv -= Chroma_Width;
    }
    else
    {
      pu += Chroma_Width;
      pv += Chroma_Width;
    }

    /* line j + 2, j + 3 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y2 = *py2++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2+1542];
      uv2 = uvtab[uv2+3598];
      *dst++  = ytab[((((*py++ + y2)>>1)+3)<<4)|(uv1&15)];
      *dst2++ = ytab[((y2+15)<<4)|(uv2>>4)];

      y2 = *py2++;
      *dst++  = ytab[((((*py++ + y2)>>1)+11)<<4)|(uv1>>4)];
      *dst2++ = ytab[((y2+7)<<4)|(uv2&15)];

      y2 = *py2++;
      uv2 = (*pu++<<8)|*pv++;
      uv1 = uvtab[uv2+514];
      uv2 = uvtab[uv2+2570];
      *dst++  = ytab[((((*py++ + y2)>>1)+1)<<4)|(uv1&15)];
      *dst2++ = ytab[((y2+13)<<4)|(uv2>>4)];

      y2 = *py2++;
      *dst++  = ytab[((((*py++ + y2)>>1)+9)<<4)|(uv1>>4)];
      *dst2++ = ytab[((y2+5)<<4)|(uv2&15)];
    }

    py += Coded_Picture_Width;
    py2 += Coded_Picture_Width;
    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;
    pu += Chroma_Width;
    pv += Chroma_Width;
  }
}

/* only for 4:4:4 */

static void ditherframe444(src)
unsigned char *src[];
{
  int i,j;
  unsigned char *py,*pu,*pv,*dst;

  py = src[0];
  pu = src[1];
  pv = src[2];
  dst = dithered_image;

  for (j=0; j<Coded_Picture_Height; j+=4)
  {
    /* line j + 0 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      *dst++ = ytab[((*py++)<<4)|(uvtab[(*pu++<<8)|*pv++]&15)];
      *dst++ = ytab[((*py++ +8)<<4)|(uvtab[(*pu++<<8)|*pv++]>>4)];
      *dst++ = ytab[((*py++ +2)<<4)|(uvtab[((*pu++<<8)|*pv++)+1028]&15)];
      *dst++ = ytab[((*py++ +10)<<4)|(uvtab[((*pu++<<8)|*pv++)+1028]>>4)];
      *dst++ = ytab[((*py++)<<4)|(uvtab[(*pu++<<8)|*pv++]&15)];
      *dst++ = ytab[((*py++ +8)<<4)|(uvtab[(*pu++<<8)|*pv++]>>4)];
      *dst++ = ytab[((*py++ +2)<<4)|(uvtab[((*pu++<<8)|*pv++)+1028]&15)];
      *dst++ = ytab[((*py++ +10)<<4)|(uvtab[((*pu++<<8)|*pv++)+1028]>>4)];
    }

    /* line j + 1 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      *dst++ = ytab[((*py++ +12)<<4)|(uvtab[((*pu++<<8)|*pv++)+2056]>>4)];
      *dst++ = ytab[((*py++ +4)<<4)|(uvtab[((*pu++<<8)|*pv++)+2056]&15)];
      *dst++ = ytab[((*py++ +14)<<4)|(uvtab[((*pu++<<8)|*pv++)+3084]>>4)];
      *dst++ = ytab[((*py++ +6)<<4)|(uvtab[((*pu++<<8)|*pv++)+3084]&15)];
      *dst++ = ytab[((*py++ +12)<<4)|(uvtab[((*pu++<<8)|*pv++)+2056]>>4)];
      *dst++ = ytab[((*py++ +4)<<4)|(uvtab[((*pu++<<8)|*pv++)+2056]&15)];
      *dst++ = ytab[((*py++ +14)<<4)|(uvtab[((*pu++<<8)|*pv++)+3084]>>4)];
      *dst++ = ytab[((*py++ +6)<<4)|(uvtab[((*pu++<<8)|*pv++)+3084]&15)];
    }

    /* line j + 2 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      *dst++ = ytab[((*py++ +3)<<4)|(uvtab[((*pu++<<8)|*pv++)+1542]&15)];
      *dst++ = ytab[((*py++ +11)<<4)|(uvtab[((*pu++<<8)|*pv++)+1542]>>4)];
      *dst++ = ytab[((*py++ +1)<<4)|(uvtab[((*pu++<<8)|*pv++)+514]&15)];
      *dst++ = ytab[((*py++ +9)<<4)|(uvtab[((*pu++<<8)|*pv++)+514]>>4)];
      *dst++ = ytab[((*py++ +3)<<4)|(uvtab[((*pu++<<8)|*pv++)+1542]&15)];
      *dst++ = ytab[((*py++ +11)<<4)|(uvtab[((*pu++<<8)|*pv++)+1542]>>4)];
      *dst++ = ytab[((*py++ +1)<<4)|(uvtab[((*pu++<<8)|*pv++)+514]&15)];
      *dst++ = ytab[((*py++ +9)<<4)|(uvtab[((*pu++<<8)|*pv++)+514]>>4)];
    }

    /* line j + 3 */
    for (i=0; i<Coded_Picture_Width; i+=8)
    {
      *dst++ = ytab[((*py++ +15)<<4)|(uvtab[((*pu++<<8)|*pv++)+3598]>>4)];
      *dst++ = ytab[((*py++ +7)<<4)|(uvtab[((*pu++<<8)|*pv++)+3598]&15)];
      *dst++ = ytab[((*py++ +13)<<4)|(uvtab[((*pu++<<8)|*pv++)+2570]>>4)];
      *dst++ = ytab[((*py++ +5)<<4)|(uvtab[((*pu++<<8)|*pv++)+2570]&15)];
      *dst++ = ytab[((*py++ +15)<<4)|(uvtab[((*pu++<<8)|*pv++)+3598]>>4)];
      *dst++ = ytab[((*py++ +7)<<4)|(uvtab[((*pu++<<8)|*pv++)+3598]&15)];
      *dst++ = ytab[((*py++ +13)<<4)|(uvtab[((*pu++<<8)|*pv++)+2570]>>4)];
      *dst++ = ytab[((*py++ +5)<<4)|(uvtab[((*pu++<<8)|*pv++)+2570]&15)];
    }
  }
}

static void dithertop444(src,dst)
unsigned char *src[];
unsigned char *dst;
{
  int i,j;
  unsigned int y,uv;
  unsigned char *py,*py2,*pu,*pv,*dst2;

  py = src[0];
  py2 = src[0] + (Coded_Picture_Width<<1);
  pu = src[1];
  pv = src[2];
  dst2 = dst + Coded_Picture_Width;

  for (j=0; j<Coded_Picture_Height; j+=4)
  {
    /* line j + 0, j + 1 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y)<<4)|(uvtab[uv]&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+12)<<4)|(uvtab[uv+2056]>>4)];

      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y+8)<<4)|(uvtab[uv]>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+4)<<4)|(uvtab[uv+2056]&15)];

      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y+2)<<4)|(uvtab[uv+1028]&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+14)<<4)|(uvtab[uv+3072]>>4)];

      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y+10)<<4)|(uvtab[uv+1028]>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+6)<<4)|(uvtab[uv+3072]&15)];
    }

    py += Coded_Picture_Width;

    if (j!=(Coded_Picture_Height-4))
      py2 += Coded_Picture_Width;
    else
      py2 -= Coded_Picture_Width;

    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;

    pu += Chroma_Width;
    pv += Chroma_Width;

    /* line j + 2, j + 3 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y+3)<<4)|(uvtab[uv+1542]&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+15)<<4)|(uvtab[uv+3598]>>4)];

      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y+11)<<4)|(uvtab[uv+1542]>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+7)<<4)|(uvtab[uv+3598]&15)];

      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y+1)<<4)|(uvtab[uv+514]&15)];
      *dst2++ = ytab[((((y + *py2++)>>1)+13)<<4)|(uvtab[uv+2570]>>4)];

      y = *py++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((y+9)<<4)|(uvtab[uv+514]>>4)];
      *dst2++ = ytab[((((y + *py2++)>>1)+5)<<4)|(uvtab[uv+2570]&15)];
    }

    py += Coded_Picture_Width;
    py2 += Coded_Picture_Width;
    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;
    pu += Chroma_Width;
    pv += Chroma_Width;
  }
}

static void ditherbot444(src,dst)
unsigned char *src[];
unsigned char *dst;
{
  int i,j;
  unsigned int y2,uv;
  unsigned char *py,*py2,*pu,*pv,*dst2;

  py = src[0] + Coded_Picture_Width;
  py2 = py;
  pu = src[1] + Chroma_Width;
  pv = src[2] + Chroma_Width;
  dst2 = dst + Coded_Picture_Width;

  for (j=0; j<Coded_Picture_Height; j+=4)
  {
    /* line j + 0, j + 1 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1))<<4)|(uvtab[uv]&15)];
      *dst2++ = ytab[((y2+12)<<4)|(uvtab[uv+2056]>>4)];

      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1)+8)<<4)|(uvtab[uv]>>4)];
      *dst2++ = ytab[((y2+4)<<4)|(uvtab[uv+2056]&15)];

      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1)+2)<<4)|(uvtab[uv+1028]&15)];
      *dst2++ = ytab[((y2+14)<<4)|(uvtab[uv+3072]>>4)];

      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1)+10)<<4)|(uvtab[uv+1028]>>4)];
      *dst2++ = ytab[((y2+6)<<4)|(uvtab[uv+3072]&15)];
    }

    if (j==0)
      py -= Coded_Picture_Width;
    else
      py += Coded_Picture_Width;

    py2 += Coded_Picture_Width;
    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;

    pu += Chroma_Width;
    pv += Chroma_Width;

    /* line j + 2, j + 3 */
    for (i=0; i<Coded_Picture_Width; i+=4)
    {
      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1)+3)<<4)|(uvtab[uv+1542]&15)];
      *dst2++ = ytab[((y2+15)<<4)|(uvtab[uv+3598]>>4)];

      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1)+11)<<4)|(uvtab[uv+1542]>>4)];
      *dst2++ = ytab[((y2+7)<<4)|(uvtab[uv+3598]&15)];

      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1)+1)<<4)|(uvtab[uv+514]&15)];
      *dst2++ = ytab[((y2+13)<<4)|(uvtab[uv+2570]>>4)];

      y2 = *py2++;
      uv = (*pu++<<8)|*pv++;
      *dst++  = ytab[((((*py++ + y2)>>1)+9)<<4)|(uvtab[uv+514]>>4)];
      *dst2++ = ytab[((y2+5)<<4)|(uvtab[uv+2570]&15)];
    }

    py += Coded_Picture_Width;
    py2 += Coded_Picture_Width;
    dst += Coded_Picture_Width;
    dst2 += Coded_Picture_Width;
    pu += Chroma_Width;
    pv += Chroma_Width;
  }
}

#endif
