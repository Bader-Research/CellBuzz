/* common_macros.h 
 * Copyright (C) 1995-2005 Jean-loup Gailly.
 */

/* s.kang start */
#define MEMCOPY_BYTE_BY_BYTE( p_out, p_from, len ) {\
	unsigned char* p_dest;\
	unsigned char* p_src;\
	int length;\
	vector unsigned char* p_v_dest;\
	vector unsigned char v_src;\
	vector unsigned char v_sel;\
	int diff;\
	int dest_offset;\
	int init_copy_bytes;\
	p_dest = ( p_out );\
	p_src = ( p_from );\
	length = ( int )( len );\
	if( __builtin_expect( ( p_dest > p_src ) && ( length > ( p_dest - p_src ) ) && ( p_dest - p_src < 16 ), 0 ) ) {/* overlap, overlap becomes problem only when p_dest > p_src and d_dest - p_src < 16*/\
		diff = p_dest - p_src;\
		dest_offset = ( unsigned int )p_dest & 0xf;\
		init_copy_bytes = 16 - dest_offset;\
		p_v_dest = ( vector unsigned char* )p_dest;\
		v_src = spu_shuffle( *( ( vector unsigned char* )p_src ), *( ( vector unsigned char* )( p_src + 16 ) ), va_uc_shuffle_pattern[( unsigned int )p_src & 0xf] );\
		v_src = spu_shuffle( v_src, v_src, va_uc_memcpy_shuffle_pattern[diff] );\
		v_src = spu_rlmaskqwbyte( v_src, -dest_offset );\
		v_sel = spu_rlmaskqwbyte( spu_splats( ( unsigned char )0xff ), -dest_offset );\
		if( init_copy_bytes > length ) {\
			v_sel = spu_and( v_sel, spu_slqwbyte( spu_splats( ( unsigned char )0xff ), init_copy_bytes - length ) );\
			*p_v_dest = spu_sel( *p_v_dest, v_src, v_sel );\
		}\
		else {\
			*p_v_dest = spu_sel( *p_v_dest, v_src, v_sel );\
			p_v_dest++;\
			p_src += init_copy_bytes;\
			length -= init_copy_bytes;\
			while( length > 15 ) {\
		 		v_src = spu_shuffle( *( ( vector unsigned char* )p_src ), *( ( vector unsigned char* )( p_src + 16 ) ), va_uc_shuffle_pattern[( unsigned int )p_src & 0xf] );\
				*p_v_dest = spu_shuffle( v_src, v_src, va_uc_memcpy_shuffle_pattern[diff] );\
				p_v_dest++;\
				p_src += 16;\
				length -= 16;\
			}\
			v_src = spu_shuffle( *( ( vector unsigned char* )p_src ), *( ( vector unsigned char* )( p_src + 16 ) ), va_uc_shuffle_pattern[( unsigned int )p_src & 0xf] );\
			v_src = spu_shuffle( v_src, v_src, va_uc_memcpy_shuffle_pattern[diff] );\
			v_sel = spu_slqwbyte( spu_splats( ( unsigned char )0xff ), 16 - length );\
			*p_v_dest = spu_sel( *p_v_dest, v_src, v_sel );\
		}\
	}\
	else {\
		dest_offset = ( unsigned int )p_dest & 0xf;\
		init_copy_bytes = 16 - dest_offset;\
		p_v_dest = ( vector unsigned char* )p_dest;\
		v_src = spu_shuffle( *( ( vector unsigned char* )p_src ), *( ( vector unsigned char* )( p_src + 16 ) ), va_uc_shuffle_pattern[( unsigned int )p_src & 0xf] );\
		v_src = spu_rlmaskqwbyte( v_src, -dest_offset );\
		v_sel = spu_rlmaskqwbyte( spu_splats( ( unsigned char )0xff ), -dest_offset );\
		if( init_copy_bytes > length ) {\
			v_sel = spu_and( v_sel, spu_slqwbyte( spu_splats( ( unsigned char )0xff ), init_copy_bytes - length ) );\
			*p_v_dest = spu_sel( *p_v_dest, v_src, v_sel );\
		}\
		else {\
			*p_v_dest = spu_sel( *p_v_dest, v_src, v_sel );\
			p_v_dest++;\
			p_src += init_copy_bytes;\
			length -= init_copy_bytes;\
			while( length > 15 ) {\
		 		*p_v_dest = spu_shuffle( *( ( vector unsigned char* )p_src ), *( ( vector unsigned char* )( p_src + 16 ) ), va_uc_shuffle_pattern[( unsigned int )p_src & 0xf] );\
				p_v_dest++;\
				p_src += 16;\
				length -= 16;\
			}\
			v_src = spu_shuffle( *( ( vector unsigned char* )p_src ), *( ( vector unsigned char* )( p_src + 16 ) ), va_uc_shuffle_pattern[( unsigned int )p_src & 0xf] );\
			v_sel = spu_slqwbyte( spu_splats( ( unsigned char )0xff ), 16 - length );\
			*p_v_dest = spu_sel( *p_v_dest, v_src, v_sel );\
		}\
	}\
}
/* s.kang end */

