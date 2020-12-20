#ifndef __jasper_cell_h__
#define __jasper_cell_h__

#include "mutex.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SPU_THREADS 16/* maximum number of SPUs for static variable allocation */

#define MAX_SPU_THREADS_PER_CHIP 8

#define DMA_MAX_SIZE 16384

#define MAX_PPU_THREADS 2/* maximum number of PPUs for static variable allocation */

#define SPU_THREADS -1/* if -1 set, dynamically */

#define PPU_THREADS -1/* if -1 set dynamically, for tier-1 encoding and decoding */

#define CACHE_LINE_SIZE 128
#define CACHE_LINE_OFFSET_BITS 7/* log2( CACHE_LINE_SIZE ) */
#define CACHE_LINE_OFFSET_MASK 0xffffff80

#define DMA_MIN_SIZE 16
#define DMA_MIN_OFFSET_BITS 4/* log2( DMA_MIN_SIZE ) */
#define DMA_MIN_OFFSET_MASK 0xfffffff0

#define MULTI_BUF_DEPTH 4/* minimum 2, maximum 8, buffer depth for multi-level buffering, applied for vertical DWT, (de)quantization, pre/post-processing */
#define DWT_BUF_DEPTH 8/* minimum 5, maximum 8 */

#define DMA_AUX_BUF_SIZE 65536/* need to be larger than maximum tier-1 encoding output size for single code block */

#define JPC_RW_CMPT_BUF_ITEMS 128/* JPC_RW_CMPT_BUF_ITEMS need to be multiple of cache line size */
#define JPC_QUANT0_BUF_ITEMS 64/* first step, find the maximum magnitude value, JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t ) need to be multiple of cache line size */
#define JPC_QUANT1_BUF_ITEMS 64/* second step, do actual quantization, JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ) need to be multiple of cache line size */
#define JPC_PRE_PROC_BUF_ITEMS 64/* JPC_PRE_PROC_BUF_ITEMS * sizeof( jpc_fix_t ) need to be multiple of cache line size */
#define JPC_POST_PROC_BUF_ITEMS 64/* JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ) need to be multiple of cache line size */

#define PRC_IDLE 0
#define PRC_RUNNING 1
#define PRC_ERROR 2
#define PRC_DONE 3

#define CMD_RD_CMPT 0
#define CMD_WR_CMPT 1
#define CMD_ENC_PRE_PROC_WITH_MCT 2
#define CMD_DEC_POST_PROC_WITH_IMCT 3
#define CMD_ENC_PRE_PROC_NO_MCT 4
#define CMD_DEC_POST_PROC_NO_IMCT 5
#define CMD_FWD_DWT_ROW 6
#define CMD_INV_DWT_ROW 7
#define CMD_FWD_DWT_COLGRP 8
#define CMD_INV_DWT_COLGRP 9
#define CMD_QUANTIZATION0 10/* 1st step, find the max mag. value */
#define CMD_QUANTIZATION1 11/* 2nd step, do actual quantization */
#define CMD_DEQUANTIZATION 12
#define CMD_T1_ENCODE 13
#define CMD_T1_DECODE 14
#define CMD_SPU_TERM 15
#define CMD_PPU_TERM 16

typedef struct _enc_control_block_t {
	mutex_ea_t mutex_ea;
	int spu_id;
	unsigned int addr_prc_status;
	unsigned int addr_enc_pre_proc_with_mct_cmd_data;
	unsigned int addr_enc_pre_proc_no_mct_cmd_data;
	unsigned int addr_fwd_dwt_row_cmd_data;
	unsigned int addr_fwd_dwt_colgrp_cmd_data;
	unsigned int addr_quant_cmd_data;
	unsigned int addr_quant_ret_data;
	unsigned int addr_t1_enc_cmd_data;
	unsigned int addr_t1_enc_ret_data;
	unsigned int addr_t1_enc_pass_data;
	unsigned int addr_rd_cmpt_cmd_data;
	char a_pad[CACHE_LINE_SIZE - sizeof( int ) * 1 - sizeof( unsigned int ) * 11 - sizeof( mutex_ea_t ) * 1];
} enc_control_block_t;

typedef struct _dec_control_block_t {
	mutex_ea_t mutex_ea;
	int spu_id;
	unsigned int addr_prc_status;
	unsigned int addr_t1_dec_cmd_data;
	unsigned int addr_inv_dwt_row_cmd_data;
	unsigned int addr_inv_dwt_colgrp_cmd_data;
	unsigned int addr_dequant_cmd_data;
	unsigned int addr_dec_post_proc_with_imct_cmd_data;
	unsigned int addr_dec_post_proc_no_imct_cmd_data;
	unsigned int addr_wr_cmpt_cmd_data;
	char a_pad[CACHE_LINE_SIZE - sizeof( int ) * 1 - sizeof( unsigned int ) * 8 - sizeof( mutex_ea_t ) * 1];
} dec_control_block_t;

typedef struct _prc_status_t {
        volatile unsigned int status;
        char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int )];
} prc_status_t;

typedef struct _enc_pre_proc_with_mct_cmd_data_t {
	unsigned int a_addr_comp_data[3];
	int a_comp_stride[3];
	int numrows;
	int numcols;
	int mctid;
	int realmode;
	int a_adjust[3];
	int a_mn[3];
	int a_mx[3];

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 3 - sizeof( int ) * 16];
} enc_pre_proc_with_mct_cmd_data_t;

typedef struct _dec_post_proc_with_imct_cmd_data_t {
	unsigned int a_addr_comp_data[3];
	int a_comp_stride[3];
	int numrows;
	int numcols;
	int mctid;
	int realmode;
	int a_adjust[3];
	int a_mn[3];
	int a_mx[3];

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 3 - sizeof( int ) * 16];
} dec_post_proc_with_imct_cmd_data_t;

typedef struct _enc_pre_proc_no_mct_cmd_data_t {
	unsigned int addr_data;
	int stride;
	int numrows;
	int numcols;
	int realmode;
	int adjust;
	int mn;
	int mx;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 1 - sizeof( int ) * 7];	
} enc_pre_proc_no_mct_cmd_data_t;

typedef struct _dec_post_proc_no_imct_cmd_data_t {
	unsigned int addr_data;
	int stride;
	int numrows;
	int numcols;
	int realmode;
	int adjust;
	int mn;
	int mx;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 1 - sizeof( int ) * 7];	
} dec_post_proc_no_imct_cmd_data_t;

typedef struct _fwd_dwt_row_cmd_data_t {
	unsigned int startaddr;
	int numrows;
	int numcols;
	int stride;
	int colparity;
	int lossless;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 1 - sizeof( int ) * 5];
} fwd_dwt_row_cmd_data_t;

typedef struct _inv_dwt_row_cmd_data_t {
	unsigned int startaddr;
	int numrows;
	int numcols;
	int stride;
	int colparity;
	int lossless;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 1 - sizeof( int ) * 5];
} inv_dwt_row_cmd_data_t;

typedef struct _fwd_dwt_colgrp_cmd_data_t {
	unsigned int startaddr;
	unsigned int splitbufaddr;
	int numrows;
	int numcolgrps;
	int stride;
	int rowparity;
	int lossless;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 2 - sizeof( int ) * 5];	
} fwd_dwt_colgrp_cmd_data_t;

typedef struct _inv_dwt_colgrp_cmd_data_t {
	unsigned int startaddr;
	unsigned int joinbufaddr;
	int numrows;
	int numcolgrps;
	int stride;
	int rowparity;
	int lossless;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 2 - sizeof( int ) * 5];	
} inv_dwt_colgrp_cmd_data_t;

typedef struct _quant_cmd_data_t {
	unsigned int addr_data;
	int stride;
	int numrows;
	int numcols;
	int realmode;
	int absstepsize;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 1 - sizeof( int ) * 5];
} quant_cmd_data_t;

typedef struct _dequant_cmd_data_t {
	unsigned int addr_data;
	int stride;
	int numrows;
	int numcols;
	int roishift;
	int bgshift;
	int numbps;
	int realmode;
	int absstepsize;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 1 - sizeof( int ) * 8];
} dequant_cmd_data_t;

typedef struct _quant_ret_data_t {
	int mxmag;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( int ) * 1];
} quant_ret_data_t;

typedef struct _t1_enc_cmd_data_t {
	/* only for PPU side */
	void* p_cblk;

	/* per component coding parameter */
	unsigned int cblksty;
	int c_synweight;

	/* band parameter */
	int numbps;
	int orient;
	int b_synweight;
	int absstepsize;

	/* code block parameter */
	unsigned int addr_cblk_stream_obj_buf;
	int cblk_stream_obj_bufsize;

	/* code block matrix parameter */
	int numrows;
	int numcols;
	unsigned int addr_rows;
	unsigned int addr_data;

	/* code block stream parameter */
	unsigned int addr_stream;

	/* etc */
	unsigned int addr_aux_buf;
	int aux_bufsize;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( void* ) - sizeof( int ) * 7 - sizeof( unsigned int ) * 8];
} t1_enc_cmd_data_t;

typedef struct _t1_dec_cmd_data_t {
	/* only for PPU side */
	void* p_cblk;

	/* per component coding parameter */
	unsigned int cblkctx;
	unsigned int c_roishift;

	/* band parameter */
	int orient;
	int numbps;
	unsigned int b_roishift;

	/* code block parameter */
	int numimsbs;
	int firstpassno;
	unsigned int addr_seg_head;

	/* code block matrix parameter */
	int numrows;
	int numcols;
	unsigned int addr_rows;
	unsigned int addr_data;

	/* etc */
	int dopartial;
	int maxlyrs;

	/* padding */
	char a_pad[CACHE_LINE_SIZE - sizeof( void* ) - sizeof( int ) * 8 - sizeof( unsigned int ) * 6];
} t1_dec_cmd_data_t;

typedef struct _t1_enc_ret_data_t {
	int numbps;
	int numimsbs;
	int numpasses;
	int cblk_stream_size;
	unsigned int addr_passes;
	char a_pad[CACHE_LINE_SIZE - sizeof( int ) * 4 - sizeof( unsigned int )];
} t1_enc_ret_data_t;

typedef struct _rd_cmpt_cmd_data_t {
	unsigned int addr_cmpt_data;
	unsigned int addr_matrix_data;
	int cmpt_data_stride;
	int matrix_stride;
	int numrows;
	int numcols;
	int cps;
	int prec;
	int sgnd;
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 2 - sizeof( int ) * 7];
} rd_cmpt_cmd_data_t;

typedef struct _wr_cmpt_cmd_data_t {
	unsigned int addr_matrix_data;
	unsigned int addr_cmpt_data;
	int matrix_stride;
	int cmpt_data_stride;
	int numrows;
	int numcols;
	int cps;
	int prec;
	int sgnd;
	char a_pad[CACHE_LINE_SIZE - sizeof( unsigned int ) * 2 - sizeof( int ) * 7];
} wr_cmpt_cmd_data_t;

#ifdef __cplusplus
}
#endif

#endif/* __jasper_cell_h__ */

