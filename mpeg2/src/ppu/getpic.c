/* getpic.c, picture decoding                                               */

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
#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "global.h"

#include <libmisc.h>

/* private prototypes*/
static void picture_data _ANSI_ARGS_((int framenum));

static void Update_Picture_Buffers _ANSI_ARGS_((void));
static void frame_reorder _ANSI_ARGS_((int bitstream_framenum, 
  int sequence_framenum));

static int fill_slice_buffer _ANSI_ARGS_ ((unsigned char **buffer));
static void ReadySPUJob _ANSI_ARGS_ ((int jobIndex, 
				      int framenum,
				      int MBAmax, int code));

/* decode one frame or field picture */
void Decode_Picture(bitstream_framenum, sequence_framenum)
int bitstream_framenum, sequence_framenum;
{

  if (picture_structure==FRAME_PICTURE && Second_Field)
  {
    /* recover from illegal number of field pictures */
    printf("odd number of field pictures\n");
    Second_Field = 0;
  }

  /* IMPLEMENTATION: update picture buffer pointers */
  Update_Picture_Buffers();

#ifdef VERIFY 
  Check_Headers(bitstream_framenum, sequence_framenum);
#endif /* VERIFY */

  /* ISO/IEC 13818-4 section 2.4.5.4 "frame buffer intercept method" */
  /* (section number based on November 1995 (Dallas) draft of the 
      conformance document) */
  if(Ersatz_Flag)
    Substitute_Frame_Buffer(bitstream_framenum, sequence_framenum);

  /* form spatial scalable picture */
 
  /* form spatial scalable picture */
  /* ISO/IEC 13818-2 section 7.7: Spatial scalability */
  if (base.pict_scal && !Second_Field) 
  {
    Spatial_Prediction();
  }

  /* decode picture data ISO/IEC 13818-2 section 6.2.3.7 */
  picture_data(bitstream_framenum);

  /* write or display current or previously decoded reference frame */
  /* ISO/IEC 13818-2 section 6.1.1.11: Frame reordering */
  frame_reorder(bitstream_framenum, sequence_framenum);

  if (picture_structure!=FRAME_PICTURE)
    Second_Field = !Second_Field;
}


/* decode all macroblocks of the current picture */
/* stages described in ISO/IEC 13818-2 section 7 */
static void picture_data(framenum)
     int framenum;
{
  int MBAmax;

  int spuJobIndex = 0;

  unsigned int code;

  /* number of macroblocks per picture */
  MBAmax = mb_width*mb_height;

  if (picture_structure!=FRAME_PICTURE)
    MBAmax>>=1; /* field picture has half as mnay macroblocks as frame */

  for(;;)
  {
    ld = &base;
    ldData = &baseData;
  
    Fault_Flag = 0;

    next_start_code();
    code = Show_Bits(32);

    if (code<SLICE_START_CODE_MIN || code>SLICE_START_CODE_MAX)
    {
      /* only slice headers are allowed in picture_data */
      if (!Quiet_Flag)
	printf("start_of_slice(): Premature end of picture\n");

      //return(-1);  /* trigger: go to next picture */
      return;
    }

    Flush_Buffer32();

    /* successfull: trigger decode macroblocks in slice */

    /* IBM CELL: The core of decoding using the SPEs */
    // Fill data buffer with data from stream between now and start of next
    // slice
    ReadySPUJob(spuJobIndex++, framenum,
		MBAmax, code);

    CheckAndSend();

    /* All slices have been decoded and are ready for SPUs */
    code = Show_Bits(32);
    if( code < SLICE_START_CODE_MIN || code > SLICE_START_CODE_MAX )
      break;
  }

  /* Sync SPUs to complete picture decoding for real */
  CellSync();
}


/* reuse old picture buffers as soon as they are no longer needed 
   based on life-time axioms of MPEG */
static void Update_Picture_Buffers()
{                           
  int cc;              /* color component index */
  unsigned char *tmp;  /* temporary swap pointer */

  for (cc=0; cc<3; cc++)
  {
    /* B pictures do not need to be save for future reference */
    if (picture_coding_type==B_TYPE)
    {
      current_frame[cc] = auxframe[cc];
    }
    else
    {
      /* only update at the beginning of the coded frame */
      if (!Second_Field)
      {
        tmp = forward_reference_frame[cc];

        /* the previously decoded reference frame is stored
           coincident with the location where the backward 
           reference frame is stored (backwards prediction is not
           needed in P pictures) */
        forward_reference_frame[cc] = backward_reference_frame[cc];
        
        /* update pointer for potential future B pictures */
        backward_reference_frame[cc] = tmp;
      }

      /* can erase over old backward reference frame since it is not used
         in a P picture, and since any subsequent B pictures will use the 
         previously decoded I or P frame as the backward_reference_frame */
      current_frame[cc] = backward_reference_frame[cc];
    }

    /* IMPLEMENTATION:
       one-time folding of a line offset into the pointer which stores the
       memory address of the current frame saves offsets and conditional 
       branches throughout the remainder of the picture processing loop */
    if (picture_structure==BOTTOM_FIELD)
      current_frame[cc]+= (cc==0) ? Coded_Picture_Width : Chroma_Width;
  }
}


/* store last frame */

void Output_Last_Frame_of_Sequence(Framenum)
int Framenum;
{
  if (Second_Field)
    printf("last frame incomplete, not stored\n");
  else
    Write_Frame(backward_reference_frame,Framenum-1);
}



static void frame_reorder(Bitstream_Framenum, Sequence_Framenum)
int Bitstream_Framenum, Sequence_Framenum;
{
  /* tracking variables to insure proper output in spatial scalability */
  static int Oldref_progressive_frame, Newref_progressive_frame;

  if (Sequence_Framenum!=0)
  {
    if (picture_structure==FRAME_PICTURE || Second_Field)
    {
      if (picture_coding_type==B_TYPE)
        Write_Frame(auxframe,Bitstream_Framenum-1);
      else
      {
        Newref_progressive_frame = progressive_frame;
        progressive_frame = Oldref_progressive_frame;

        Write_Frame(forward_reference_frame,Bitstream_Framenum-1);

        Oldref_progressive_frame = progressive_frame = Newref_progressive_frame;
      }
    }
#ifdef DISPLAY
    else if (Output_Type==T_X11)
    {
      if(!Display_Progressive_Flag)
        Display_Second_Field();
    }
#endif
  }
  else
    Oldref_progressive_frame = progressive_frame;

}

static int fill_slice_buffer(buffer)
unsigned char **buffer;
{
  int currentBufferSize = 2048;
  int i;
  int bufferSize = 0;
  
  (*buffer) = (unsigned char*)malloc_align(sizeof(unsigned char) * currentBufferSize, 4);

  while( Show_Bits(24) != 0x1L )
  {
    (*buffer)[bufferSize++] = (unsigned char)(Get_Bits(8) & 0xFF);
    if( bufferSize >= currentBufferSize )
    {
      unsigned char *tempBuffer;
      int origBuffSize = currentBufferSize;
      currentBufferSize *= 2;
      tempBuffer = (unsigned char*)malloc_align(sizeof(unsigned char) * currentBufferSize, 4);
      /* Warning: Do not use realloc_align as it might cause data at the
       * beginning of the buffer to be lost
       */
      //(*buffer) = (unsigned char*)realloc_align((*buffer), sizeof(unsigned char) * currentBufferSize, 4);
      memmove(tempBuffer, *buffer, origBuffSize);
      free_align(*buffer);
      *buffer = tempBuffer;
    }

    //Flush_Buffer(8);// - Not necessary beacuse of Get_Bits()
  }

  for( i = bufferSize; i < currentBufferSize - 3; ++i )
  {
    (*buffer)[i++] = SEQUENCE_END_CODE>>24;
    (*buffer)[i++] = SEQUENCE_END_CODE>>16;
    (*buffer)[i++] = SEQUENCE_END_CODE>>8;
    (*buffer)[i++] = SEQUENCE_END_CODE&0xff;
  }

  return bufferSize;
}

static void ReadySPUJob(jobIndex,
			framenum,
			MBAmax, code)
     int jobIndex, framenum, MBAmax, code;
{
  struct spuJobData *spuJob = &(spuJobs[jobIndex]);

  memcpy(&(spuJob->sliceData), ld, sizeof(struct layer_data));
  spuJob->ReadBufferSize = 0;
  spuJob->ReadBufferSize = fill_slice_buffer( &(spuJob->bitstreamBuffer) );

  spuJob->Quiet_Flag = Quiet_Flag;
  spuJob->code = code;
  spuJob->Trace_Flag = Trace_Flag;
  spuJob->block_count = block_count;
  spuJob->picture_coding_type = picture_coding_type;

  spuJob->picture_structure = picture_structure;
  spuJob->concealment_motion_vectors = concealment_motion_vectors;
  spuJob->f_code[0][0] = f_code[0][0];
  spuJob->f_code[0][1] = f_code[0][1];
  spuJob->f_code[1][0] = f_code[1][0];
  spuJob->f_code[1][1] = f_code[1][1];
  spuJob->full_pel_forward_vector = full_pel_forward_vector;
  spuJob->forward_f_code = forward_f_code;
  spuJob->full_pel_backward_vector = full_pel_backward_vector;
  spuJob->backward_f_code = backward_f_code;
  spuJob->chroma_format = chroma_format;
  spuJob->mb_width = mb_width;
  spuJob->mb_height = mb_height;

  spuJob->Coded_Picture_Width = Coded_Picture_Width;
  spuJob->Chroma_Width = Chroma_Width;
  spuJob->spatial_temporal_weight_code_table_index = spatial_temporal_weight_code_table_index;
  spuJob->frame_pred_frame_dct = frame_pred_frame_dct;
  spuJob->intra_dc_precision = intra_dc_precision;
  spuJob->intra_vlc_format = intra_vlc_format;

  spuJob->vertical_size = vertical_size;

  /* Can vary the amount of caching performed for these two DMAs - both too large, so must stream in */
  //for( ret = 0; ret < 3; ++ret )
  {
    spuJob->current_frame[0] = current_frame[0];
    spuJob->backward_reference_frame[0] = backward_reference_frame[0];
    spuJob->forward_reference_frame[0] = forward_reference_frame[0];

    spuJob->current_frame[1] = current_frame[1];
    spuJob->backward_reference_frame[1] = backward_reference_frame[1];
    spuJob->forward_reference_frame[1] = forward_reference_frame[1];

    spuJob->current_frame[2] = current_frame[2];
    spuJob->backward_reference_frame[2] = backward_reference_frame[2];
    spuJob->forward_reference_frame[2] = forward_reference_frame[2];
  }

  spuJob->Second_Field = Second_Field;
  spuJob->top_field_first = top_field_first;

  spuJob->framenum = framenum;
  spuJob->MBAmax = MBAmax;
  spuJob->MBA = 0;
  spuJob->MBAinc = 0;

  /* reset all DC coefficient and motion vector predictors */
  /* reset all DC coefficient and motion vector predictors */
  /* ISO/IEC 13818-2 section 7.2.1: DC coefficients in intra blocks */
  spuJob->dc_dct_pred[0] = spuJob->dc_dct_pred[1] = spuJob->dc_dct_pred[2] = 0;

  /* ISO/IEC 13818-2 section 7.6.3.4: Resetting motion vector predictors */
  spuJob->PMV[0][0][0] = spuJob->PMV[0][0][1] = spuJob->PMV[0][1][0] = 0;
  spuJob->PMV[0][1][1] = spuJob->PMV[1][0][0] = spuJob->PMV[1][0][1] = 0;
  spuJob->PMV[1][1][0] = spuJob->PMV[1][1][1] = 0;
}
