#ifndef C444
#ifdef FRAME0
void disp_truecolor_frame(src,dest)
#else
#ifdef FIELD1
void disp_truecolor_top(src,dest)
#else
void disp_truecolor_bot(src,dest)
#endif
#endif
#else
#ifdef FRAME0
void disp_truecolor_frame444(src,dest)
#else
#ifdef FIELD1
void disp_truecolor_top444(src,dest)
#else
void disp_truecolor_bot444(src,dest)
#endif
#endif
#endif
unsigned char *src[];
unsigned char *dest;
{
  int i, j;
#ifdef FASTTRUECOLOR
  unsigned long uvshift, uvy;
#else
  int y, u, v, r, g, b;
#endif
  int offset,incr,height ;
  unsigned char *py, *pu, *pv;
#ifdef FASTTRUECOLOR
  unsigned long *dst, *dst2, *pdst2;
#else
  unsigned char *dst,*dst2, *pdst2;
#endif

  /* the use of prevdst is a bit sick; if we're doing a field,
     then we copy the other field from the previous time round,
     which should get us an interlace effect---I hope! */

#ifdef FASTTRUECOLOR
  dst = (unsigned long *) dest;
#else
  dst = dest ;
#endif

#ifdef FIELD2
  offset = Coded_Picture_Width;
#else
  offset = 0;
#endif
#ifdef FRAME0
  incr = Coded_Picture_Width;
  height = Coded_Picture_Height;
#else
  incr = Coded_Picture_Width<<1 ;
  height = Coded_Picture_Height>>1;
#endif

/* DPP is the number of dst units (char or long) per pixel */
#ifdef FASTTRUECOLOR
#define DPP 1
#else
#define DPP 4
#endif

#ifdef FIELD1
  dst2 = dst + DPP*Coded_Picture_Width ;
  if ( prevdst ) {
      pdst2 = prevdst + DPP*Coded_Picture_Width ;
  } else {
      pdst2 = dst ;
  }
#else
#ifdef FIELD2
  dst2 = dst ;
  dst += DPP*Coded_Picture_Width ;
  if ( prevdst ) {
      pdst2 = prevdst ;
  } else {
      pdst2 = dst ;
  }
#endif
#endif


  for (i=0; i<height; i++) {
      py = src[0] + offset + incr*i;
      if ( chroma_format == CHROMA420 ) {
	  pu = src[1] + (offset>>1) + (incr>>1)*(i>>1);
	  pv = src[2] + (offset>>1) + (incr>>1)*(i>>1);
      } else if ( chroma_format == CHROMA422 ) {
	  pu = src[1] + (offset>>1) + (incr>>1)*i;
	  pv = src[2] + (offset>>1) + (incr>>1)*i;
      } else if ( chroma_format == CHROMA444 ) {
	  pu = src[1] + offset + incr*i;
	  pv = src[2] + offset + incr*i;
      }
      
      for (j=0; j<Coded_Picture_Width; j++)
	  {
	      /* we really really want to get this loop as fast as possible;
		 multiplication is expensive, especially on Sparcs, so let's
		 have a look-up table for the uv part of the following.
		 Controlled by ifdefs during experimentation.
		 */
#if 0
	      /* this was the original code, various bits of which are now
		 precomputed, depending on compile time options. */
	      y = 76309 * (*py++ - 16); /* (255/219)*65536 */
	      u = *pu++ - 128;
	      v = *pv++ - 128;
	      b = clp[(y + crv*v + 32768)>>16];
	      g = clp[(y - cgu*u - cgv*v + 32768)>>16];
	      r = clp[(y + cbu*u + 32786)>>16];
#endif
#ifdef FASTTRUECOLOR
	      uvshift = ((*pu++ & 0xFC) << 11) | ((*pv++ & 0xFC) << 5);
	      uvy = uvshift | ((*py++ >>1) &0x7F);
	      *dst++ = uvytab[uvy];
#else
	      y = yytab[*py++];
	      b = clp[(y + rvtab[*pv])>>16];
	      g = clp[(y + guvtab[256*(*pu) + *pv])>>16];
	      r = clp[(y + butab[*pu])>>16];

	      dst++ ;
	      *(dst++) = b ; *(dst++) = g ; *(dst++) = r;
#endif
#ifndef FRAME0
#ifdef FASTTRUECOLOR
	      *dst2++ = *pdst2++;
#else
	      *((unsigned long *) dst2) = *((unsigned long *) pdst2) ;
	      dst2 += 4 ; pdst2 += 4 ;
#endif
#endif
#ifdef C444
#ifdef FASTTRUECOLOR
	      uvshift = ((*pu++ & 0xFC) << 11) | ((*pv++ & 0xFC) << 5);

#else
	      pu++ ; pv++ ;
#endif
#endif
	      j++;

#ifdef FASTTRUECOLOR
	      uvy = uvshift | ((*py++ >>1) &0x7F);
	      *dst++ = uvytab[uvy];
#else
	      y = yytab[*py++];
	      b = clp[(y + rvtab[*pv])>>16];
	      g = clp[(y + guvtab[256*(*pu) + *pv++])>>16];
	      r = clp[(y + butab[*pu++])>>16];

	      dst++ ;
	      *(dst++) = b ; *(dst++) = g ; *(dst++) = r;
#endif
#ifndef FRAME0
#ifdef FASTTRUECOLOR
	      *dst2++ = *pdst2++;
#else
	      *((unsigned long *) dst2) = *((unsigned long *) pdst2) ;
	      dst2 += 4 ; pdst2 += 4 ;
#endif
#endif
	  }
#ifndef FRAME0
      dst += DPP*Coded_Picture_Width ;
      dst2 += DPP*Coded_Picture_Width ;
      pdst2 += DPP*Coded_Picture_Width ;
#endif
  }
#ifdef FASTTRUECOLOR
  prevdst = (unsigned long *) dest;
#else
  prevdst = dest ;
#endif
}
