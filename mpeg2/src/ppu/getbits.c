/* getbits.c, bit level routines                                            */

/*
 * All modifications (mpeg2decode -> mpeg2play) are
 * Copyright (C) 1996, Stefan Eckart. All Rights Reserved.
 */

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
#include <stdlib.h>
#include <unistd.h>

#include "config.h"
#include "global.h"

/* initialize buffer, call once before first getbits or showbits */

void Initialize_Buffer()
{
  ldData->Incnt = 0;
  ldData->Rdptr = ldData->Rdbfr + RDBFR_SIZE;
  ldData->Rdmax = ldData->Rdptr;

#ifdef VERIFY
  /*  only the verifier uses this particular bit counter 
   *  Bitcnt keeps track of the current parser position with respect
   *  to the video elementary stream being decoded, regardless 
   *  of whether or not it is wrapped within a systems layer stream 
   */
  ld->Bitcnt = 0;
#endif

  ldData->Bfr = 0;
  Flush_Buffer(0); /* fills valid data into bfr */
}

void Fill_Buffer()
{
  int Buffer_Level;

  Buffer_Level = read(ldData->Infile,ldData->Rdbfr,RDBFR_SIZE);
  ldData->Rdptr = ldData->Rdbfr;

  if (System_Stream_Flag)
    ldData->Rdmax -= RDBFR_SIZE;

  
  /* end of the bitstream file */
  if (Buffer_Level < RDBFR_SIZE)
  {
    /* just to be safe */
    if (Buffer_Level < 0)
      Buffer_Level = 0;

    /* pad until the next to the next 32-bit word boundary */
    while (Buffer_Level & 3)
      ldData->Rdbfr[Buffer_Level++] = 0;

	/* pad the buffer with sequence end codes */
    while (Buffer_Level < RDBFR_SIZE)
    {
      ldData->Rdbfr[Buffer_Level++] = SEQUENCE_END_CODE>>24;
      ldData->Rdbfr[Buffer_Level++] = SEQUENCE_END_CODE>>16;
      ldData->Rdbfr[Buffer_Level++] = SEQUENCE_END_CODE>>8;
      ldData->Rdbfr[Buffer_Level++] = SEQUENCE_END_CODE&0xff;
    }
  }
}


/* MPEG-1 system layer demultiplexer */

int Get_Byte()
{
  while(ldData->Rdptr >= ldData->Rdbfr+RDBFR_SIZE)
  {
    read(ldData->Infile,ldData->Rdbfr,RDBFR_SIZE);
    ldData->Rdptr -= RDBFR_SIZE;
    ldData->Rdmax -= RDBFR_SIZE;
  }
  return *ldData->Rdptr++;
}

/* extract a 16-bit word from the bitstream buffer */
int Get_Word()
{
  int Val;

  Val = Get_Byte();
  return (Val<<8) | Get_Byte();
}


/* return next n bits (right adjusted) without advancing */

unsigned int Show_Bits(N)
int N;
{
  return ldData->Bfr >> (32-N);
}


/* return next bit (could be made faster than Get_Bits(1)) */

unsigned int Get_Bits1()
{
  return Get_Bits(1);
}


/* advance by n bits */

void Flush_Buffer(N)
int N;
{
  int Incnt;

  ldData->Bfr <<= N;

  Incnt = ldData->Incnt -= N;

  if (Incnt <= 24)
  {
    if (System_Stream_Flag && (ldData->Rdptr >= ldData->Rdmax-4))
    {
      do
      {
        if (ldData->Rdptr >= ldData->Rdmax)
          Next_Packet();
        ldData->Bfr |= Get_Byte() << (24 - Incnt);
        Incnt += 8;
      }
      while (Incnt <= 24);
    }
    else if (ldData->Rdptr < ldData->Rdbfr+(RDBFR_SIZE-4))
    {
      do
      {
        ldData->Bfr |= *ldData->Rdptr++ << (24 - Incnt);
        Incnt += 8;
      }
      while (Incnt <= 24);
    }
    else
    {
      do
      {
        if (ldData->Rdptr >= ldData->Rdbfr+RDBFR_SIZE)
          Fill_Buffer();
        ldData->Bfr |= *ldData->Rdptr++ << (24 - Incnt);
        Incnt += 8;
      }
      while (Incnt <= 24);
    }
    ldData->Incnt = Incnt;
  }

#ifdef VERIFY 
  ld->Bitcnt += N;
#endif /* VERIFY */

}


/* return next n bits (right adjusted) */

unsigned int Get_Bits(N)
int N;
{
  unsigned int Val;

  Val = Show_Bits(N);
  Flush_Buffer(N);

  return Val;
}

