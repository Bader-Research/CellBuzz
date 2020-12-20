/* crc32.c -- compute the CRC-32 of a data stream
 * Copyright (C) 1995-2005 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 *
 * Thanks to Rodney Brown <rbrown64@csc.com.au> for his contribution of faster
 * CRC methods: exclusive-oring 32 bits of data at a time, and pre-computing
 * tables for updating the shift register in one step with three exclusive-ors
 * instead of four steps with four exclusive-ors.  This results in about a
 * factor of two increase in speed on a Power PC G4 (PPC7455) using gcc -O3.
 */

/* @(#) $Id: crc32.c,v 1.3 2008/06/23 16:02:18 lrlemini Exp $ */

/*
  Note on the use of DYNAMIC_CRC_TABLE: there is no mutex or semaphore
  protection on the static variables used to control the first-use generation
  of the crc tables.  Therefore, if you #define DYNAMIC_CRC_TABLE, you should
  first call get_crc_table() to initialize the tables before allowing more than
  one thread to use crc32().
 */

/* s.kang start */
/* does not support dynamically making crc32.h file(and no need to do that, this library is only for IBM Cell processor) */
#ifdef MAKECRCH
ERROR: DO NOT USE MAKECRCH
#endif
/* s.kang end */

#ifdef MAKECRCH
#  include <stdio.h>
#  ifndef DYNAMIC_CRC_TABLE
#    define DYNAMIC_CRC_TABLE
#  endif /* !DYNAMIC_CRC_TABLE */
#endif /* MAKECRCH */

#include "zutil.h"      /* for STDC and FAR definitions */

/* s.kang start */
#include <spu_mfcio.h>
#include <spu_internals.h>

#include "vector_constant.h"
/* s.kang end */

#define local static

/* Find a four-byte integer type for crc32_little() and crc32_big(). */
#ifndef NOBYFOUR
#  ifdef STDC           /* need ANSI C limits.h to determine sizes */
#    include <limits.h>
#    define BYFOUR
#    if (UINT_MAX == 0xffffffffUL)
       typedef unsigned int u4;
#    else
#      if (ULONG_MAX == 0xffffffffUL)
         typedef unsigned long u4;
#      else
#        if (USHRT_MAX == 0xffffffffUL)
           typedef unsigned short u4;
#        else
#          undef BYFOUR     /* can't find a four-byte integer type! */
#        endif
#      endif
#    endif
#  endif /* STDC */
#endif /* !NOBYFOUR */

/* Definitions for doing the crc four data bytes at a time. */
#ifdef BYFOUR
#  define REV(w) (((w)>>24)+(((w)>>8)&0xff00)+ \
                (((w)&0xff00)<<8)+(((w)&0xff)<<24))
/* s.kang start */
#if 0
/* s.kang end */
   local unsigned long crc32_little OF((unsigned long,
                        const unsigned char FAR *, unsigned));
/* s.kang start */
#endif
/* s.kang end */
   local unsigned long crc32_big OF((unsigned long,
                        const unsigned char FAR *, unsigned));
/* s.kang start */
/* include only big endian portion of the crc_table */
#if 1
#define TBLS 4
#else
#  define TBLS 8
#endif
/* s.kang end */
#else
#  define TBLS 1
#endif /* BYFOUR */


/* Local functions for crc concatenation */
local unsigned long gf2_matrix_times OF((unsigned long *mat,
                                         unsigned long vec));
local void gf2_matrix_square OF((unsigned long *square, unsigned long *mat));

#ifdef DYNAMIC_CRC_TABLE

local volatile int crc_table_empty = 1;
local unsigned long FAR crc_table[TBLS][256];
local void make_crc_table OF((void));
#ifdef MAKECRCH
   local void write_table OF((FILE *, const unsigned long FAR *));
#endif /* MAKECRCH */
/*
  Generate tables for a byte-wise 32-bit CRC calculation on the polynomial:
  x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1.

  Polynomials over GF(2) are represented in binary, one bit per coefficient,
  with the lowest powers in the most significant bit.  Then adding polynomials
  is just exclusive-or, and multiplying a polynomial by x is a right shift by
  one.  If we call the above polynomial p, and represent a byte as the
  polynomial q, also with the lowest power in the most significant bit (so the
  byte 0xb1 is the polynomial x^7+x^3+x+1), then the CRC is (q*x^32) mod p,
  where a mod b means the remainder after dividing a by b.

  This calculation is done using the shift-register method of multiplying and
  taking the remainder.  The register is initialized to zero, and for each
  incoming bit, x^32 is added mod p to the register if the bit is a one (where
  x^32 mod p is p+x^32 = x^26+...+1), and the register is multiplied mod p by
  x (which is shifting right by one and adding x^32 mod p if the bit shifted
  out is a one).  We start with the highest power (least significant bit) of
  q and repeat for all eight bits of q.

  The first table is simply the CRC of all possible eight bit values.  This is
  all the information needed to generate CRCs on data a byte at a time for all
  combinations of CRC register values and incoming bytes.  The remaining tables
  allow for word-at-a-time CRC calculation for both big-endian and little-
  endian machines, where a word is four bytes.
*/
local void make_crc_table()
{
    unsigned long c;
    int n, k;
    unsigned long poly;                 /* polynomial exclusive-or pattern */
    /* terms of polynomial defining this crc (except x^32): */
    static volatile int first = 1;      /* flag to limit concurrent making */
    static const unsigned char p[] = {0,1,2,4,5,7,8,10,11,12,16,22,23,26};

    /* See if another task is already doing this (not thread-safe, but better
       than nothing -- significantly reduces duration of vulnerability in
       case the advice about DYNAMIC_CRC_TABLE is ignored) */
    if (first) {
        first = 0;

        /* make exclusive-or pattern from polynomial (0xedb88320UL) */
        poly = 0UL;
        for (n = 0; n < sizeof(p)/sizeof(unsigned char); n++)
            poly |= 1UL << (31 - p[n]);

        /* generate a crc for every 8-bit value */
        for (n = 0; n < 256; n++) {
            c = (unsigned long)n;
            for (k = 0; k < 8; k++)
                c = c & 1 ? poly ^ (c >> 1) : c >> 1;
            crc_table[0][n] = c;
        }

#ifdef BYFOUR
        /* generate crc for each value followed by one, two, and three zeros,
           and then the byte reversal of those as well as the first table */
        for (n = 0; n < 256; n++) {
            c = crc_table[0][n];
            crc_table[4][n] = REV(c);
            for (k = 1; k < 4; k++) {
                c = crc_table[0][c & 0xff] ^ (c >> 8);
                crc_table[k][n] = c;
                crc_table[k + 4][n] = REV(c);
            }
        }
#endif /* BYFOUR */

        crc_table_empty = 0;
    }
    else {      /* not first */
        /* wait for the other guy to finish (not efficient, but rare) */
        while (crc_table_empty)
            ;
    }

#ifdef MAKECRCH
    /* write out CRC tables to crc32.h */
    {
        FILE *out;

        out = fopen("crc32.h", "w");
        if (out == NULL) return;
        fprintf(out, "/* crc32.h -- tables for rapid CRC calculation\n");
        fprintf(out, " * Generated automatically by crc32.c\n */\n\n");
        fprintf(out, "local const unsigned long FAR ");
        fprintf(out, "crc_table[TBLS][256] =\n{\n  {\n");
        write_table(out, crc_table[0]);
#  ifdef BYFOUR
        fprintf(out, "#ifdef BYFOUR\n");
        for (k = 1; k < 8; k++) {
            fprintf(out, "  },\n  {\n");
            write_table(out, crc_table[k]);
        }
        fprintf(out, "#endif\n");
#  endif /* BYFOUR */
        fprintf(out, "  }\n};\n");
        fclose(out);
    }
#endif /* MAKECRCH */
}

#ifdef MAKECRCH
local void write_table(out, table)
    FILE *out;
    const unsigned long FAR *table;
{
    int n;

    for (n = 0; n < 256; n++)
        fprintf(out, "%s0x%08lxUL%s", n % 5 ? "" : "    ", table[n],
                n == 255 ? "\n" : (n % 5 == 4 ? ",\n" : ", "));
}
#endif /* MAKECRCH */

#else /* !DYNAMIC_CRC_TABLE */
/* ========================================================================
 * Tables of CRC-32s of all single-byte values, made by make_crc_table().
 */
#include "crc32.h"
#endif /* DYNAMIC_CRC_TABLE */

/* =========================================================================
 * This function can be used by asm versions of crc32()
 */
const unsigned long FAR * ZEXPORT get_crc_table()
{
#ifdef DYNAMIC_CRC_TABLE
    if (crc_table_empty)
        make_crc_table();
#endif /* DYNAMIC_CRC_TABLE */
    return (const unsigned long FAR *)crc_table;
}

/* s.kang start */
#ifndef BYFOUR
/* s.kang end */
/* ========================================================================= */
#define DO1 crc = crc_table[0][((int)crc ^ (*buf++)) & 0xff] ^ (crc >> 8)
#define DO8 DO1; DO1; DO1; DO1; DO1; DO1; DO1; DO1
/* s.kang start */
#endif
/* s.kang end */

/* ========================================================================= */
/* s.kang start */
#if 1
unsigned long ZEXPORT crc32( crc, buf, len )
	unsigned long crc;
	const unsigned char* buf;
	unsigned len;
{
	if( buf == Z_NULL ) {
		return 0UL;
	}

	return crc32_big( crc, buf, len );
}
#else
unsigned long ZEXPORT crc32(crc, buf, len)
    unsigned long crc;
    const unsigned char FAR *buf;
    unsigned len;
{
    if (buf == Z_NULL) return 0UL;

#ifdef DYNAMIC_CRC_TABLE
    if (crc_table_empty)
        make_crc_table();
#endif /* DYNAMIC_CRC_TABLE */

#ifdef BYFOUR
    if (sizeof(void *) == sizeof(ptrdiff_t)) {
        u4 endian;

        endian = 1;
        if (*((unsigned char *)(&endian)))
            return crc32_little(crc, buf, len);
        else
            return crc32_big(crc, buf, len);
    }
#endif /* BYFOUR */
    crc = crc ^ 0xffffffffUL;
    while (len >= 8) {
        DO8;
        len -= 8;
    }
    if (len) do {
        DO1;
    } while (--len);
    return crc ^ 0xffffffffUL;
}
#endif

#ifdef BYFOUR
/* s.kang start */
/* CELL uses big endian only, no need to increase code size and comsume more SPE LS memory */
#if 0
/* s.kang end */
/* ========================================================================= */
#define DOLIT4 c ^= *buf4++; \
        c = crc_table[3][c & 0xff] ^ crc_table[2][(c >> 8) & 0xff] ^ \
            crc_table[1][(c >> 16) & 0xff] ^ crc_table[0][c >> 24]
#define DOLIT32 DOLIT4; DOLIT4; DOLIT4; DOLIT4; DOLIT4; DOLIT4; DOLIT4; DOLIT4

/* ========================================================================= */
local unsigned long crc32_little(crc, buf, len)
    unsigned long crc;
    const unsigned char FAR *buf;
    unsigned len;
{
    register u4 c;
    register const u4 FAR *buf4;

    c = (u4)crc;
    c = ~c;
    while (len && ((ptrdiff_t)buf & 3)) {
        c = crc_table[0][(c ^ *buf++) & 0xff] ^ (c >> 8);
        len--;
    }

    buf4 = (const u4 FAR *)(const void FAR *)buf;
    while (len >= 32) {
        DOLIT32;
        len -= 32;
    }
    while (len >= 4) {
        DOLIT4;
        len -= 4;
    }
    buf = (const unsigned char FAR *)buf4;

    if (len) do {
        c = crc_table[0][(c ^ *buf++) & 0xff] ^ (c >> 8);
    } while (--len);
    c = ~c;
    return (unsigned long)c;
}
/* s.kang start */
#endif
/* s.kang end */

/* s.kang start */
#if 1
local vector unsigned char v_ltuh_0 = { 0x0, 0x5, 0x4b, 0x4e, 0xd7, 0xd2, 0x9c, 0x99, 0xae, 0xab, 0xe5, 0xe0, 0x79, 0x7c, 0x32, 0x37 };
local vector unsigned char v_ltuh_1 = { 0x0, 0x8d, 0x1c, 0x91, 0x3e, 0xb3, 0x22, 0xaf, 0x7d, 0xf0, 0x61, 0xec, 0x43, 0xce, 0x5f, 0xd2 };
local vector unsigned char v_ltuh_2 = { 0x0, 0xc8, 0xe0, 0x28, 0xb1, 0x79, 0x51, 0x99, 0x62, 0xaa, 0x82, 0x4a, 0xd3, 0x1b, 0x33, 0xfb };
local vector unsigned char v_ltuh_3 = { 0x0, 0x86, 0xd6, 0x50, 0x76, 0xf0, 0xa0, 0x26, 0xed, 0x6b, 0x3b, 0xbd, 0x9b, 0x1d, 0x4d, 0xcb };
local vector unsigned char v_ltul_0 = { 0x0, 0x24, 0x48, 0x6c, 0xd1, 0xf5, 0x99, 0xbd, 0xa2, 0x86, 0xea, 0xce, 0x73, 0x57, 0x3b, 0x1f };
local vector unsigned char v_ltul_1 = { 0x0, 0xd9, 0xb2, 0x6b, 0x62, 0xbb, 0xd0, 0x9, 0xc5, 0x1c, 0x77, 0xae, 0xa7, 0x7e, 0x15, 0xcc };
local vector unsigned char v_ltul_2 = { 0x0, 0x7, 0xf, 0x8, 0x6e, 0x69, 0x61, 0x66, 0xdc, 0xdb, 0xd3, 0xd4, 0xb2, 0xb5, 0xbd, 0xba };
local vector unsigned char v_ltul_3 = { 0x0, 0x63, 0xc6, 0xa5, 0x57, 0x34, 0x91, 0xf2, 0xae, 0xcd, 0x68, 0xb, 0xf9, 0x9a, 0x3f, 0x5c };

local vector unsigned char v_uc_low_nibble_mask = { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f }; 

/* ========================================================================= */
#define DOBIG4 c ^= *buf4++; \
        c = crc_table[0][c & 0xff] ^ crc_table[1][(c >> 8) & 0xff] ^ \
            crc_table[2][(c >> 16) & 0xff] ^ crc_table[3][c >> 24]
#define DOBIG32 DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4
#define DOBIG16_VEC \
{\
		v_2 = *( ( vector unsigned int* )( p_buf16 + 2 ) );\
		v_h_shuffle_pattern = spu_and( spu_rlmaskqw( v_0, -4 ), v_uc_low_nibble_mask );\
		v_l_shuffle_pattern = spu_and( v_0, v_uc_low_nibble_mask );\
		v_h_0 = spu_shuffle( v_ltuh_0, v_uc_zero, v_h_shuffle_pattern );\
		v_l_0 = spu_shuffle( v_ltul_0, v_uc_zero, v_l_shuffle_pattern );\
		v_h_1 = spu_shuffle( v_ltuh_1, v_uc_zero, v_h_shuffle_pattern );\
		v_l_1 = spu_shuffle( v_ltul_1, v_uc_zero, v_l_shuffle_pattern );\
		v_h_2 = spu_shuffle( v_ltuh_2, v_uc_zero, v_h_shuffle_pattern );\
		v_l_2 = spu_shuffle( v_ltul_2, v_uc_zero, v_l_shuffle_pattern );\
		v_h_3 = spu_shuffle( v_ltuh_3, v_uc_zero, v_h_shuffle_pattern );\
		v_l_3 = spu_shuffle( v_ltul_3, v_uc_zero, v_l_shuffle_pattern );\
		v_hxorl_0 = spu_xor( v_h_0, v_l_0 );\
		v_hxorl_1 = spu_xor( v_h_1, v_l_1 );\
		v_hxorl_2 = spu_xor( v_h_2, v_l_2 );\
		v_hxorl_3 = spu_xor( v_h_3, v_l_3 );\
		v_1 = spu_xor( v_1, ( vector unsigned int )spu_xor( spu_xor( spu_rlmaskqwbyte( v_hxorl_0, -13 ), spu_rlmaskqwbyte( v_hxorl_1, -14 ) ), spu_rlmaskqwbyte( v_hxorl_2, -15 ) ) );\
		v_2 = spu_xor( v_2, ( vector unsigned int )spu_xor( spu_xor( spu_slqwbyte( v_hxorl_0, 3 ), spu_slqwbyte( v_hxorl_1, 2 ) ), spu_xor( spu_slqwbyte( v_hxorl_2, 1 ), v_hxorl_3 ) ) );\
		v_0 = ( vector unsigned char )v_1;\
		v_1 = v_2;\
		len -= 16;\
		p_buf16++;\
}

/* ========================================================================= */
local unsigned long crc32_big( crc, buf, len )
	unsigned long crc;
	const unsigned char* buf;
	unsigned len;
{
	unsigned char a_temp_buf[48] __attribute__ ( ( aligned( 4 ) ) );
	u4 c;
	const u4* buf4;
	const vector unsigned char* p_buf16;
	vector unsigned char v_0;
	vector unsigned int v_1;
	vector unsigned int v_2;
	vector unsigned char v_h_0;
	vector unsigned char v_l_0;
	vector unsigned char v_h_1;
	vector unsigned char v_l_1;
	vector unsigned char v_h_2;
	vector unsigned char v_l_2;
	vector unsigned char v_h_3;
	vector unsigned char v_l_3;
	vector unsigned char v_hxorl_0;
	vector unsigned char v_hxorl_1;
	vector unsigned char v_hxorl_2;
	vector unsigned char v_hxorl_3;
	vector unsigned char v_h_shuffle_pattern;
	vector unsigned char v_l_shuffle_pattern;
	int vector_compressed;

	c = REV( ( u4 )crc );
	c = ~c;

	while( len && ( ( ptrdiff_t )buf & 0xf ) ) {
		c = crc_table[0][( c >> 24 ) ^ *buf++] ^ ( c << 8 );
		len--;
	}

	p_buf16 = ( const vector unsigned char* )buf;

	v_0 = *p_buf16;/* these three lines isn't necessary if len < 48 but doesn't harm the correctness in that case. considering that len is usually larger than or equal to 48, make this out of the if state and keep branch offset small is beneficial to performance */
	v_0 = spu_xor( v_0, ( vector unsigned char )spu_insert( ( unsigned int )c, v_ui_zero, 0 ) );
	v_1 = *( ( vector unsigned int* )( p_buf16 + 1 ) );

	vector_compressed = 0;
	if( len >= 48 ) {
		vector_compressed = 1;
		c = 0;
	}

	while( len >= 96 ) {/* loop unrolled by factor of 4 */
		DOBIG16_VEC;
		DOBIG16_VEC;
		DOBIG16_VEC;
		DOBIG16_VEC;
	}
	while( len >= 48 ) {
		DOBIG16_VEC;
	}

	buf = ( const unsigned char* )p_buf16;

	if( len ) {
		if( vector_compressed ) {
			*( ( vector unsigned char* )a_temp_buf ) = v_0;
			*( ( vector unsigned int* )( a_temp_buf + 16 ) ) = v_1;
			memcpy( ( a_temp_buf + 32 ), ( buf + 32 ), len - 32 );
		}
		else {
			memcpy( a_temp_buf, buf, len );
		}

		buf4 = ( const u4* )a_temp_buf;

		while( len >= 4 ) {
			DOBIG4;
			len -= 4;
		}
	
		if( len ) {
			buf = ( const unsigned char* )buf4;
			do {
				c = crc_table[0][( c >> 24 ) ^ *buf++] ^ ( c << 8 );
			} while( --len );
		}
	}

	c = ~c;
	return ( unsigned long )( REV( c ) );
}
#else
/* ========================================================================= */
#define DOBIG4 c ^= *++buf4; \
        c = crc_table[4][c & 0xff] ^ crc_table[5][(c >> 8) & 0xff] ^ \
            crc_table[6][(c >> 16) & 0xff] ^ crc_table[7][c >> 24]
#define DOBIG32 DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4; DOBIG4

/* ========================================================================= */
local unsigned long crc32_big(crc, buf, len)
    unsigned long crc;
    const unsigned char FAR *buf;
    unsigned len;
{
    register u4 c;
    register const u4 FAR *buf4;

    c = REV((u4)crc);
    c = ~c;
    while (len && ((ptrdiff_t)buf & 3)) {
        c = crc_table[4][(c >> 24) ^ *buf++] ^ (c << 8);
        len--;
    }

    buf4 = (const u4 FAR *)(const void FAR *)buf;
    buf4--;
    while (len >= 32) {
        DOBIG32;
        len -= 32;
    }
    while (len >= 4) {
        DOBIG4;
        len -= 4;
    }
    buf4++;
    buf = (const unsigned char FAR *)buf4;

    if (len) do {
        c = crc_table[4][(c >> 24) ^ *buf++] ^ (c << 8);
    } while (--len);
    c = ~c;
    return (unsigned long)(REV(c));
}
#endif
#endif /* BYFOUR */

#define GF2_DIM 32      /* dimension of GF(2) vectors (length of CRC) */

/* ========================================================================= */
local unsigned long gf2_matrix_times(mat, vec)
    unsigned long *mat;
    unsigned long vec;
{
    unsigned long sum;

    sum = 0;
    while (vec) {
        if (vec & 1)
            sum ^= *mat;
        vec >>= 1;
        mat++;
    }
    return sum;
}

/* ========================================================================= */
local void gf2_matrix_square(square, mat)
    unsigned long *square;
    unsigned long *mat;
{
    int n;

    for (n = 0; n < GF2_DIM; n++)
        square[n] = gf2_matrix_times(mat, mat[n]);
}

/* ========================================================================= */
uLong ZEXPORT crc32_combine(crc1, crc2, len2)
    uLong crc1;
    uLong crc2;
    z_off_t len2;
{
    int n;
    unsigned long row;
    unsigned long even[GF2_DIM];    /* even-power-of-two zeros operator */
    unsigned long odd[GF2_DIM];     /* odd-power-of-two zeros operator */

    /* degenerate case */
    if (len2 == 0)
        return crc1;

    /* put operator for one zero bit in odd */
    odd[0] = 0xedb88320L;           /* CRC-32 polynomial */
    row = 1;
    for (n = 1; n < GF2_DIM; n++) {
        odd[n] = row;
        row <<= 1;
    }

    /* put operator for two zero bits in even */
    gf2_matrix_square(even, odd);

    /* put operator for four zero bits in odd */
    gf2_matrix_square(odd, even);

    /* apply len2 zeros to crc1 (first square will put the operator for one
       zero byte, eight zero bits, in even) */
    do {
        /* apply zeros operator for this bit of len2 */
        gf2_matrix_square(even, odd);
        if (len2 & 1)
            crc1 = gf2_matrix_times(even, crc1);
        len2 >>= 1;

        /* if no more bits set, then done */
        if (len2 == 0)
            break;

        /* another iteration of the loop with odd and even swapped */
        gf2_matrix_square(odd, even);
        if (len2 & 1)
            crc1 = gf2_matrix_times(odd, crc1);
        len2 >>= 1;

        /* if no more bits set, then done */
    } while (len2 != 0);

    /* return combined crc */
    crc1 ^= crc2;
    return crc1;
}
