/* gzio.c -- IO on .gz files
 * Copyright (C) 1995-2005 Jean-loup Gailly.
 * For conditions of distribution and use, see copyright notice in zlib.h
 *
 * Compile this file with -DNO_GZCOMPRESS to avoid the compression code.
 */

/* @(#) $Id: gzio_w.c,v 1.1.1.2 2007/05/23 15:20:08 lrlemini Exp $ */

#include <stdio.h>

#include "zutil.h"

#include "gzio.h"

/* s.kang start */
static gz_w_data_func w_func;
/* s.kang end */

local gzFile gz_open      OF((const char *path, const char *mode, FILE* p_file));
local int do_flush        OF((gzFile file, int flush));
local int    destroy      OF((gz_stream *s));

/* ===========================================================================
     Opens a gzip (.gz) file for reading or writing. The mode parameter
   is as in fopen ("rb" or "wb"). The file is given either by file descriptor
   or path name (if fd == -1).
     gz_open returns NULL if the file could not be opened or if there was
   insufficient memory to allocate the (de)compression state; errno
   can be checked to distinguish the two cases (if errno is zero, the
   zlib error is Z_MEM_ERROR).
*/
local gzFile gz_open (path, mode, p_file)
    const char *path;
    const char *mode;
    FILE* p_file;
{
    int err;
    int level = Z_DEFAULT_COMPRESSION; /* compression level */
    int strategy = Z_DEFAULT_STRATEGY; /* compression strategy */
    char *p = (char*)mode;
    gz_stream *s;
    char fmode[80]; /* copy of mode, without the compression level */
    char *m = fmode;

    if (!path || !mode) return Z_NULL;

    s = (gz_stream *)ALLOC(sizeof(gz_stream));
    if (!s) return Z_NULL;

    s->stream.zalloc = (alloc_func)0;
    s->stream.zfree = (free_func)0;
    s->stream.opaque = (voidpf)0;
    s->stream.next_in = s->inbuf = Z_NULL;
    s->stream.next_out = s->outbuf = Z_NULL;
    s->stream.avail_in = s->stream.avail_out = 0;
    s->file = NULL;
    s->z_err = Z_OK;
    s->z_eof = 0;
    s->in = 0;
    s->out = 0;
    s->back = EOF;
    s->crc = crc32(0L, Z_NULL, 0);
    s->msg = NULL;
    s->transparent = 0;

    s->path = (char*)ALLOC(strlen(path)+1);
    if (s->path == NULL) {
        return destroy(s), (gzFile)Z_NULL;
    }
    strcpy(s->path, path); /* do this early for debugging */

    s->mode = '\0';
    do {
        if (*p == 'r') s->mode = 'r';
        if (*p == 'w' || *p == 'a') s->mode = 'w';
        if (*p >= '0' && *p <= '9') {
            level = *p - '0';
        } else if (*p == 'f') {
          strategy = Z_FILTERED;
        } else if (*p == 'h') {
          strategy = Z_HUFFMAN_ONLY;
        } else if (*p == 'R') {
          strategy = Z_RLE;
        } else {
            *m++ = *p; /* copy the mode */
        }
    } while (*p++ && m != fmode + sizeof(fmode));
    if (s->mode == '\0') return destroy(s), (gzFile)Z_NULL;

    if (s->mode == 'r') {
        err = Z_STREAM_ERROR;
        return destroy(s), (gzFile)Z_NULL;
    }

#ifdef NO_GZCOMPRESS
    err = Z_STREAM_ERROR;
#else
    err = deflateInit2(&(s->stream), level, Z_DEFLATED, -MAX_WBITS_COMPRESS, DEF_MEM_LEVEL, strategy);
                         
    /* windowBits is passed < 0 to suppress zlib header */

    s->stream.next_out = s->outbuf = (Byte*)ALLOC(Z_BUFSIZE);
#endif
    if (err != Z_OK || s->outbuf == Z_NULL) {
        return destroy(s), (gzFile)Z_NULL;
    }
    s->stream.avail_out = Z_BUFSIZE;

    errno = 0;
    s->file = p_file;
    s->start = 0L;

    return (gzFile)s;
}

/* ===========================================================================
     Opens a gzip (.gz) file for reading or writing.
*/
gzFile ZEXPORT gzopen_w (path, mode, p_file, func)
    const char *path;
    const char *mode;
    FILE* p_file;
    gz_w_data_func func; 
{
    w_func = func;
    return gz_open (path, mode, p_file);
}

/* ===========================================================================
 * Update the compression level and strategy
 */
int ZEXPORT gzsetparams (file, level, strategy)
    gzFile file;
    int level;
    int strategy;
{
    gz_stream *s = (gz_stream*)file;

    if (s == NULL || s->mode != 'w') return Z_STREAM_ERROR;

    /* Make room to allow flushing */
    if (s->stream.avail_out == 0) {
        s->stream.next_out = s->outbuf;
        if (w_func(s->outbuf, Z_BUFSIZE, ( void* )(s->file)) != Z_BUFSIZE) {
            s->z_err = Z_ERRNO;
        }
        s->stream.avail_out = Z_BUFSIZE;
    }

    return deflateParams (&(s->stream), level, strategy);
}

 /* ===========================================================================
 * Cleanup then free the given gz_stream. Return a zlib error code.
   Try freeing in the reverse order of allocations.
 */
local int destroy (s)
    gz_stream *s;
{
    int err = Z_OK;

    if (!s) return Z_STREAM_ERROR;

    TRYFREE(s->msg);

    if (s->stream.state != NULL) {
        if (s->mode == 'w') {
#ifdef NO_GZCOMPRESS
            err = Z_STREAM_ERROR;
#else
            err = deflateEnd(&(s->stream));
#endif
        }
    }
    if (s->z_err < 0) err = s->z_err;

    TRYFREE(s->inbuf);
    TRYFREE(s->outbuf);
    TRYFREE(s->path);
    TRYFREE(s);
    return err;
}

#ifndef NO_GZCOMPRESS
/* ===========================================================================
     Writes the given number of uncompressed bytes into the compressed file.
   gzwrite returns the number of bytes actually written (0 in case of error).
*/
/* s.kang start */
int ZEXPORT gzwrite (file, buf, len)
    gzFile file;
    voidpc buf;
    unsigned len;
{
    gz_stream *s = (gz_stream*)file;

    if (s == NULL || s->mode != 'w') return Z_STREAM_ERROR;

    s->stream.next_in = (Bytef*)buf;
    s->stream.avail_in = len;

    while (s->stream.avail_in != 0) {

        if (s->stream.avail_out == 0) {

            s->stream.next_out = s->outbuf;
            if( w_func( s->outbuf, Z_BUFSIZE, s->file ) != Z_BUFSIZE ) {
                s->z_err = Z_ERRNO;
                break;
            }
            s->stream.avail_out = Z_BUFSIZE;
        }
        s->in += s->stream.avail_in;
        s->out += s->stream.avail_out;
        s->z_err = deflate(&(s->stream), Z_NO_FLUSH);
        s->in -= s->stream.avail_in;
        s->out -= s->stream.avail_out;
        if (s->z_err != Z_OK) break;
    }
    s->crc = crc32(s->crc, (const Bytef *)buf, len);

    return (int)(len - s->stream.avail_in);
}
/* s.kang end */

/* ===========================================================================
     Flushes all pending output into the compressed file. The parameter
   flush is as in the deflate() function.
*/
local int do_flush (file, flush)
	gzFile file;
	int flush;
{
	uInt len;
/* s.kang start */
#if 0
	int done = 0;
#endif
/* s.kang end */
	gz_stream *s = (gz_stream*)file;

	if (s == NULL || s->mode != 'w') return Z_STREAM_ERROR;

	s->stream.avail_in = 0; /* should be zero already anyway */

/* s.kang start */
/* to avoid unaligned DMA */
/* size argument of w_func must be Z_BUFSIZE except for the last call */
#if 1
	while( 1 ) {
		len = Z_BUFSIZE - s->stream.avail_out;

		if( s->stream.avail_out == 0 ) {
			if ((uInt)w_func(s->outbuf, Z_BUFSIZE, ( void* )s->file) != Z_BUFSIZE) {
				s->z_err = Z_ERRNO;
				return Z_ERRNO;
			}
			s->stream.next_out = s->outbuf;
			s->stream.avail_out = Z_BUFSIZE;
		}
        	s->out += s->stream.avail_out;
        	s->z_err = deflate(&(s->stream), flush);
        	s->out -= s->stream.avail_out;
		
        	if( ( len == 0 ) && ( s->z_err == Z_BUF_ERROR ) ) {
			s->z_err = Z_OK;/* Ignore the second of two consecutive flushes: */
		}

		if( ( ( s->z_err == Z_OK ) && ( s->stream.avail_out != 0 ) ) || ( s->z_err == Z_STREAM_END ) ) {
			len = Z_BUFSIZE - s->stream.avail_out;
			if ((uInt)w_func(s->outbuf, len, ( void* )s->file) != len) {
				s->z_err = Z_ERRNO;
				return Z_ERRNO;
			}
			s->stream.next_out = s->outbuf;
			s->stream.avail_out = Z_BUFSIZE;
			
			break;
		}
		else if( ( s->z_err != Z_OK ) && ( s->z_err != Z_STREAM_END ) ) {
			break;
		}
	}
#else
    for (;;) {
        len = Z_BUFSIZE - s->stream.avail_out;

        if (len != 0) {
/* s.kang start */
#if 1
            if ((uInt)w_func(s->outbuf, len, ( void* )s->file) != len) {
                s->z_err = Z_ERRNO;
                return Z_ERRNO;
            }
#else
            if ((uInt)fwrite(s->outbuf, 1, len, s->file) != len) {
                s->z_err = Z_ERRNO;
                return Z_ERRNO;
            }
#endif
/* s.kang end */
            s->stream.next_out = s->outbuf;
            s->stream.avail_out = Z_BUFSIZE;
        }
        if (done) break;
        s->out += s->stream.avail_out;
        s->z_err = deflate(&(s->stream), flush);
        s->out -= s->stream.avail_out;

        /* Ignore the second of two consecutive flushes: */
        if (len == 0 && s->z_err == Z_BUF_ERROR) s->z_err = Z_OK;

        /* deflate has finished flushing only when it hasn't used up
         * all the available space in the output buffer:
         */
        done = (s->stream.avail_out != 0 || s->z_err == Z_STREAM_END);

        if (s->z_err != Z_OK && s->z_err != Z_STREAM_END) break;
    }
#endif
/* s.kang end */
	return  s->z_err == Z_STREAM_END ? Z_OK : s->z_err;
}
#endif /* NO_GZCOMPRESS */

/* ===========================================================================
     Flushes all pending output if necessary, closes the compressed file
   and deallocates all the (de)compression state.
*/
/* s.kang start */
#if 1
int ZEXPORT gzclose_w( gzFile file, unsigned int last, unsigned int* p_crc, unsigned int* p_in_size, unsigned int* p_out_size ) {
	gz_stream* s;
	int ret;

	s = ( gz_stream* )file;

	if( s == NULL ) {
		return Z_STREAM_ERROR;
	}

	if( s->mode != 'w' ) {
		return  Z_STREAM_ERROR;
	}

#ifdef NO_GZCOMPRESS
	return Z_STREAM_ERROR;
#else
	if( last == 1 ) {
		if( do_flush( file, Z_FINISH ) != Z_OK ) {
			return destroy( ( gz_stream* )file );
		}
	}
	else {
		if( do_flush( file, Z_FULL_FLUSH ) != Z_OK ) {
			return destroy( ( gz_stream* )file );
		}
	}
#endif

	*p_crc = s->crc;
	*p_in_size = ( unsigned int )( s->in & 0xffffffff );
	*p_out_size = ( unsigned int )( s->out & 0xffffffff );

	ret = destroy( ( gz_stream* )file );
	if( ( last != 1 ) && ( ret == Z_DATA_ERROR ) ) {
		return Z_OK;
	}
	else {
		return ret;
	}
}
#else
int ZEXPORT gzclose (file)
    gzFile file;
{
    gz_stream *s = (gz_stream*)file;

    if (s == NULL) return Z_STREAM_ERROR;

    if (s->mode == 'w') {
#ifdef NO_GZCOMPRESS
        return Z_STREAM_ERROR;
#else
        if (do_flush (file, Z_FINISH) != Z_OK)
            return destroy((gz_stream*)file);

        putLong (s->file, s->crc);
        putLong (s->file, (uLong)(s->in & 0xffffffff));
#endif
    }
    return destroy((gz_stream*)file);
}
#endif
/* s.kang end */

