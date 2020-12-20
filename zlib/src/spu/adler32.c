/* adler32.c -- compute the Adler-32 checksum of a data stream
 * Copyright (C) 1995-2004 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/* @(#) $Id: adler32.c,v 1.1.1.2 2007/05/23 15:20:08 lrlemini Exp $ */

#define ZLIB_INTERNAL
#include "zlib.h"

#define BASE 65521UL    /* largest prime smaller than 65536 */
#define NMAX 5552
/* NMAX is the largest n such that 255n(n+1)/2 + (n+1)(BASE-1) <= 2^32-1 */
/* s.kang start */
#define NMAX_VEC 3854
/* NMAX_VEC is the largest n such that 255n(n+1)/2 + (n+1)(BASE-1) <= 2^31-1 */
/* s.kang end */

#define DO1(buf,i)  {adler += (buf)[i]; sum2 += adler;}
#define DO2(buf,i)  DO1(buf,i); DO1(buf,i+1);
#define DO4(buf,i)  DO2(buf,i); DO2(buf,i+2);
#define DO8(buf,i)  DO4(buf,i); DO4(buf,i+4);
#define DO16(buf)   DO8(buf,0); DO8(buf,8);

/* use NO_DIVIDE if your processor does not do division in hardware */
#ifdef NO_DIVIDE
#  define MOD(a) \
    do { \
        if (a >= (BASE << 16)) a -= (BASE << 16); \
        if (a >= (BASE << 15)) a -= (BASE << 15); \
        if (a >= (BASE << 14)) a -= (BASE << 14); \
        if (a >= (BASE << 13)) a -= (BASE << 13); \
        if (a >= (BASE << 12)) a -= (BASE << 12); \
        if (a >= (BASE << 11)) a -= (BASE << 11); \
        if (a >= (BASE << 10)) a -= (BASE << 10); \
        if (a >= (BASE << 9)) a -= (BASE << 9); \
        if (a >= (BASE << 8)) a -= (BASE << 8); \
        if (a >= (BASE << 7)) a -= (BASE << 7); \
        if (a >= (BASE << 6)) a -= (BASE << 6); \
        if (a >= (BASE << 5)) a -= (BASE << 5); \
        if (a >= (BASE << 4)) a -= (BASE << 4); \
        if (a >= (BASE << 3)) a -= (BASE << 3); \
        if (a >= (BASE << 2)) a -= (BASE << 2); \
        if (a >= (BASE << 1)) a -= (BASE << 1); \
        if (a >= BASE) a -= BASE; \
    } while (0)
#  define MOD4(a) \
    do { \
        if (a >= (BASE << 4)) a -= (BASE << 4); \
        if (a >= (BASE << 3)) a -= (BASE << 3); \
        if (a >= (BASE << 2)) a -= (BASE << 2); \
        if (a >= (BASE << 1)) a -= (BASE << 1); \
        if (a >= BASE) a -= BASE; \
    } while (0)
#else
#  define MOD(a) a %= BASE
#  define MOD4(a) a %= BASE
#endif

/* ========================================================================= */
/* s.kang start */
#if 0
uLong ZEXPORT adler32( adler, buf, len )
	uLong adler;
	const Bytef* buf;
	uInt len;
{
	unsigned long S1 = adler & 0xffff;
	unsigned long S2 = ( adler >> 16 ) & 0xffff;
	int k, offset;
    
	if( buf == Z_NULL ) {
		return 1L;
	}

	/* Handle small sizes */

	if (len < 16) {
		while( len-- ) {
			S1 += *buf++;
			S2 += S1;
		}
		MOD( S1 );
		MOD( S2 );

		return ( S2 << 16 ) | S1;
	}
    
	if( len >= 16 ) {
		vector unsigned long vs1;
		vector unsigned long vs2;
		vector unsigned long vs1_0;
		vector signed short v_suppress_even = { 0x00ff, 0x00ff, 0x00ff, 0x00ff, 0x00ff, 0x00ff, 0x00ff, 0x00ff };
		vector unsigned long v0 = spu_splats( ( unsigned long )0 );
		vector signed short v1 = spu_splats( ( signed short )1 );
		vector signed short v2_even = { 16, 14, 12, 10, 8, 6 , 4, 2 };
		vector signed short v2_odd = { 15, 13, 11, 9, 7, 5, 3, 1 };

		/* Align to 16-byte boundaries */
		offset = ( unsigned long )( buf ) % 16;
		if( offset ) {
			offset = 16 - offset;
			len -= offset;
			while( offset-- ) {
				S1 += *buf++;
				S2 += S1;
			}
		}

		while( len >= 16 ) {
			vs1 = spu_insert( S1, v0, 3 );
			vs2 = spu_insert( S2, v0, 3 );

			k = len < NMAX_VEC ? ( int )len : NMAX_VEC;
			k -= k % 16;
			len -= k;

			while( k >= 16 ) {
				vector signed short v_buf = *( ( vector signed short* )buf );
				vector signed short v_even;
				vector signed short v_odd;

				vs1_0 = vs1;
				v_even = spu_rlmask( v_buf, -8 );
				v_odd = spu_and( v_buf, v_suppress_even );
				vs1 = spu_add( vs1, ( vector unsigned long )spu_mule( v_even, v1 ) );
				vs1 = spu_add( vs1, ( vector unsigned long )spu_mulo( v_even, v1 ) );
				vs1 = spu_add( vs1, ( vector unsigned long )spu_mule( v_odd, v1 ) );
				vs1 = spu_add( vs1, ( vector unsigned long )spu_mulo( v_odd, v1 ) );
				vs2 = spu_add( vs2, ( vector unsigned long )spu_mule( v_even, v2_even ) );
				vs2 = spu_add( vs2, ( vector unsigned long )spu_mulo( v_even, v2_even ) );
				vs2 = spu_add( vs2, ( vector unsigned long )spu_mule( v_odd, v2_odd ) );
				vs2 = spu_add( vs2, ( vector unsigned long )spu_mulo( v_odd, v2_odd ) );
				vs2 = spu_add( vs2, spu_sl( vs1_0, 4 ) );

				buf += 16;
				k -= 16;
			}
			S1 = spu_extract( vs1, 0 ) + spu_extract( vs1, 1 ) + spu_extract( vs1, 2 ) + spu_extract( vs1, 3 );
			S2 = spu_extract( vs2, 0 ) + spu_extract( vs2, 1 ) + spu_extract( vs2, 2 ) + spu_extract( vs2, 3 );
			MOD( S1 );
			MOD( S2 );
		}
	}
    
	while( len-- ) {
		S1 += *buf++;
		S2 += S1;
	}

	MOD( S1 );
	MOD( S2 );

	return ( S2 << 16 ) | S1;
}
#else
uLong ZEXPORT adler32(adler, buf, len)
    uLong adler;
    const Bytef *buf;
    uInt len;
{
    unsigned long sum2;
    unsigned n;

    /* split Adler-32 into component sums */
    sum2 = (adler >> 16) & 0xffff;
    adler &= 0xffff;

    /* in case user likes doing a byte at a time, keep it fast */
    if (len == 1) {
        adler += buf[0];
        if (adler >= BASE)
            adler -= BASE;
        sum2 += adler;
        if (sum2 >= BASE)
            sum2 -= BASE;
        return adler | (sum2 << 16);
    }

    /* initial Adler-32 value (deferred check for len == 1 speed) */
    if (buf == Z_NULL)
        return 1L;

    /* in case short lengths are provided, keep it somewhat fast */
    if (len < 16) {
        while (len--) {
            adler += *buf++;
            sum2 += adler;
        }
        if (adler >= BASE)
            adler -= BASE;
        MOD4(sum2);             /* only added so many BASE's */
        return adler | (sum2 << 16);
    }

    /* do length NMAX blocks -- requires just one modulo operation */
    while (len >= NMAX) {
        len -= NMAX;
        n = NMAX / 16;          /* NMAX is divisible by 16 */
        do {
            DO16(buf);          /* 16 sums unrolled */
            buf += 16;
        } while (--n);
        MOD(adler);
        MOD(sum2);
    }

    /* do remaining bytes (less than NMAX, still just one modulo) */
    if (len) {                  /* avoid modulos if none remaining */
        while (len >= 16) {
            len -= 16;
            DO16(buf);
            buf += 16;
        }
        while (len--) {
            adler += *buf++;
            sum2 += adler;
        }
        MOD(adler);
        MOD(sum2);
    }

    /* return recombined sums */
    return adler | (sum2 << 16);
}
#endif

/* ========================================================================= */
uLong ZEXPORT adler32_combine(adler1, adler2, len2)
    uLong adler1;
    uLong adler2;
    z_off_t len2;
{
    unsigned long sum1;
    unsigned long sum2;
    unsigned rem;

    /* the derivation of this formula is left as an exercise for the reader */
    rem = (unsigned)(len2 % BASE);
    sum1 = adler1 & 0xffff;
    sum2 = rem * sum1;
    MOD(sum2);
    sum1 += (adler2 & 0xffff) + BASE - 1;
    sum2 += ((adler1 >> 16) & 0xffff) + ((adler2 >> 16) & 0xffff) + BASE - rem;
    if (sum1 > BASE) sum1 -= BASE;
    if (sum1 > BASE) sum1 -= BASE;
    if (sum2 > (BASE << 1)) sum2 -= (BASE << 1);
    if (sum2 > BASE) sum2 -= BASE;
    return sum1 | (sum2 << 16);
}
