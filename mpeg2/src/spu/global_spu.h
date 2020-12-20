#ifndef __GLOBAL_SPU_H_
#define __GLOBAL_SPU_H_

#include "../common/mpeg2dec.h"
#define SPU_INCLUDE 1
#include "../common/LayerData.h"
#undef SPU_INCLUDE

#ifdef NON_ANSI_COMPILER
#define _ANSI_ARGS_(x) ()
#else
#define _ANSI_ARGS_(x) x
#endif

#ifndef GLOBAL
#define EXTERN extern
#else
#define EXTERN
#endif

/* getbits_spu.c */
void Flush_Buffer _ANSI_ARGS_((int n));
void Fill_Buffer _ANSI_ARGS_((void));
unsigned int Show_Bits _ANSI_ARGS_((int n));
unsigned int Get_Bits _ANSI_ARGS_((int n));
unsigned int Get_Bits1 _ANSI_ARGS_((void));
void Print_Bits _ANSI_ARGS_((int code, int bits, int len)); /* moved from mpeg2dec.c */

/* getblk_spu.c */
void Decode_MPEG1_Intra_Block _ANSI_ARGS_((int comp, int dc_dct_pred[]));
void Decode_MPEG1_Non_Intra_Block _ANSI_ARGS_((int comp));
void Decode_MPEG2_Intra_Block _ANSI_ARGS_((int comp, int dc_dct_pred[]));
void Decode_MPEG2_Non_Intra_Block _ANSI_ARGS_((int comp));

/* gethdr_spu.c */
void marker_bit _ANSI_ARGS_((char *text));
int slice_header _ANSI_ARGS_((void));

/* getpic_spu.c */
int slice _ANSI_ARGS_((int framenum, int MBAmax, int MBA, 
			      int MBAinc, int dc_dct_pred[3],
			      int PMV[2][2][2]));

/* getvlc_spu.c */
int Get_macroblock_type _ANSI_ARGS_((void));
int Get_motion_code _ANSI_ARGS_((void));
int Get_dmvector _ANSI_ARGS_((void));
int Get_coded_block_pattern _ANSI_ARGS_((void));
int Get_macroblock_address_increment _ANSI_ARGS_((void));
int Get_Luma_DC_dct_diff _ANSI_ARGS_((void));
int Get_Chroma_DC_dct_diff _ANSI_ARGS_((void));
int Get_macroblock_address_increment _ANSI_ARGS_((void));

/* idct.c */
void Fast_IDCT _ANSI_ARGS_((short *block));
void Initialize_Fast_IDCT _ANSI_ARGS_((void));

/* motion_spu.c */
void motion_vectors _ANSI_ARGS_((int PMV[2][2][2], int dmvector[2],
  int motion_vertical_field_select[2][2], int s, int motion_vector_count, 
  int mv_format, int h_r_size, int v_r_size, int dmv, int mvscale));
void motion_vector _ANSI_ARGS_((int *PMV, int *dmvector,
  int h_r_size, int v_r_size, int dmv, int mvscale, int full_pel_vector));
void Dual_Prime_Arithmetic _ANSI_ARGS_((int DMV[][2], int *dmvector, int mvx, int mvy));


/* recon.c */
void form_predictions _ANSI_ARGS_((int bx, int by, int macroblock_type, 
  int motion_type, int PMV[2][2][2], int motion_vertical_field_select[2][2], 
  int dmvector[2], int stwtype));



/* Variable Decls */
EXTERN int Fault_Flag;
/* non-linear quantization coefficient table */
EXTERN unsigned char Non_Linear_quantizer_scale[32]
#ifdef GLOBAL
=
{
   0, 1, 2, 3, 4, 5, 6, 7,
   8,10,12,14,16,18,20,22,
  24,28,32,36,40,44,48,52,
  56,64,72,80,88,96,104,112
}
#endif
;

/* zig-zag and alternate scan patterns */
EXTERN unsigned char scan[2][64]
#ifdef GLOBAL
=
{
  { /* Zig-Zag scan pattern  */
    0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,
    12,19,26,33,40,48,41,34,27,20,13,6,7,14,21,28,
    35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,
    58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63
  },
  { /* Alternate scan pattern */
    0,8,16,24,1,9,2,10,17,25,32,40,48,56,57,49,
    41,33,26,18,3,11,4,12,19,27,34,42,50,58,35,43,
    51,59,20,28,5,13,6,14,21,29,36,44,52,60,37,45,
    53,61,22,30,7,15,23,31,38,46,54,62,39,47,55,63
  }
}
#endif
;

/* Init this after launching DMAs */
EXTERN unsigned char *SPU_Clip;

//EXTERN int Two_Streams;

EXTERN struct spuJobData jobData;

EXTERN int lumaSliceSize;
EXTERN int chromSliceSize;

EXTERN unsigned char *current_frame[3];
EXTERN unsigned char *backref_frame[3];
EXTERN unsigned char *forwardref_frame[2][3];

/* Base is a pointer on the SPU side since the jobData will have a 
 * layer_data object within it already
 * Additional layer_data's should not be *, like enhan
 */
EXTERN struct layer_data *base, /*enhan,*/ *spu_ld;
EXTERN struct read_data baseData, *spu_ldData;

typedef union
{
  unsigned long long ull;
  unsigned int ui[2];
} addr64;

EXTERN int bitstreamPos;

#define JOBDATA_GET_TAG    31
#define RDBFR_GET_TAG      30
#define LUMAREF_GET_TAG    20
#define CHROMREF_GET_TAG   21
#define SLICE_PUT_TAG      15

#endif
