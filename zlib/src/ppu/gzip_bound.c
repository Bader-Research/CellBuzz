/* gzip_bound.c
 */

/* @(#) $Id: gzip_bound.c,v 1.1.1.2 2007/05/23 15:20:08 lrlemini Exp $ */

#include <stdio.h>

unsigned int gzip_bound( unsigned int sourceLen ) {
	/* return conservative upper bound */

	return ( sourceLen + ( ( sourceLen + 7 ) >> 3 ) + ( ( sourceLen + 63 ) >> 6 ) + 11 );
}
              
