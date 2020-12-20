#ifndef __LAYER_DATA_H_
#define __LAYER_DATA_H_

/* This is the DMA size the SPE will use to bring in data read in
 * from the file to decode
 */
#if SPU_INCLUDE
#define RDBFR_SIZE 1024
#else
#define RDBFR_SIZE 4096
#endif

struct read_data
{
#if SPU_INCLUDE
#else
  int Infile;
#endif
  unsigned char Rdbfr[RDBFR_SIZE] __attribute__ ((aligned(16)));
  unsigned char *Rdptr;
  /* from mpeg2play */
  unsigned int Bfr;
  unsigned char *Rdmax;
  int Incnt;
};

struct layer_data {
  int Bitcnt;
  int MPEG2_Flag;
  /* sequence scalable extension */
  int scalable_mode;
  /* picture coding extension */
  int q_scale_type;
  int alternate_scan;
  /* picture spatial scalable extension */
  int pict_scal;
  /* slice/macroblock */
  int priority_breakpoint;
  int quantizer_scale;
  int intra_slice;
  short block[12][64];

  /* sequence header and quant_matrix_extension() */
  int intra_quantizer_matrix[64];
  int non_intra_quantizer_matrix[64];
  int chroma_intra_quantizer_matrix[64];
  int chroma_non_intra_quantizer_matrix[64];
};

struct spuJobData
{
  int code;
  int Quiet_Flag;
  int Trace_Flag;
  int block_count;
  int picture_coding_type;
  int picture_structure;
  int concealment_motion_vectors;
  int f_code[2][2];
  int full_pel_forward_vector;
  int forward_f_code;
  int full_pel_backward_vector;
  int backward_f_code;
  int chroma_format;
  int mb_width;
  int mb_height;

  unsigned char *current_frame[3];
  int Coded_Picture_Width;
  int Chroma_Width;
  int spatial_temporal_weight_code_table_index;
  int frame_pred_frame_dct;
  int intra_dc_precision;
  int intra_vlc_format;

  int vertical_size;

/* These three members have to be brought in through caching since they
 * can be very large
 */
  unsigned char *backward_reference_frame[3];
  unsigned char *forward_reference_frame[3];
  unsigned char *bitstreamBuffer;

  int ReadBufferSize;

  int Second_Field;
  int top_field_first;

  int framenum;
  int MBAmax;
  int MBA, MBAinc;
  int dc_dct_pred[3];
  int PMV[2][2][2];

  struct layer_data sliceData;
} __attribute__ ((aligned(128)));

#endif
