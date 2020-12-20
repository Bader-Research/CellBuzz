/* gzio.c -- IO on .gz files
 * Copyright (C) 1995-2005 Jean-loup Gailly.
 * For conditions of distribution and use, see copyright notice in zlib.h
 *
 * Compile this file with -DNO_GZCOMPRESS to avoid the compression code.
 */

/* @(#) $Id: gzio_misc.c,v 1.1.1.2 2007/05/23 15:20:08 lrlemini Exp $ */

#include <stdio.h>

#include "zutil.h"

#include "gzio.h"

/* ===========================================================================
     Returns 1 when EOF has previously been detected reading the given
   input stream, otherwise zero.
*/
int ZEXPORT gzeof (file)
    gzFile file;
{
    gz_stream *s = (gz_stream*)file;

    /* With concatenated compressed files that can have embedded
     * crc trailers, z_eof is no longer the only/best indicator of EOF
     * on a gz_stream. Handle end-of-stream error explicitly here.
     */
    if (s == NULL || s->mode != 'r') return 0;
    if (s->z_eof) return 1;
    return s->z_err == Z_STREAM_END;
}

/* ===========================================================================
     Returns 1 if reading and doing so transparently, otherwise zero.
*/
int ZEXPORT gzdirect (file)
    gzFile file;
{
    gz_stream *s = (gz_stream*)file;

    if (s == NULL || s->mode != 'r') return 0;
    return s->transparent;
}

#ifdef STDC
#  define zstrerror(errnum) strerror(errnum)
#else
#  define zstrerror(errnum) ""
#endif

/* ===========================================================================
     Returns the error message for the last error which occurred on the
   given compressed file. errnum is set to zlib error number. If an
   error occurred in the file system and not in the compression library,
   errnum is set to Z_ERRNO and the application may consult errno
   to get the exact error code.
*/
const char * ZEXPORT gzerror (file, errnum)
    gzFile file;
    int *errnum;
{
    char *m;
    gz_stream *s = (gz_stream*)file;

    if (s == NULL) {
        *errnum = Z_STREAM_ERROR;
        return (const char*)ERR_MSG(Z_STREAM_ERROR);
    }
    *errnum = s->z_err;
    if (*errnum == Z_OK) return (const char*)"";

    m = (char*)(*errnum == Z_ERRNO ? zstrerror(errno) : s->stream.msg);

    if (m == NULL || *m == '\0') m = (char*)ERR_MSG(s->z_err);

    TRYFREE(s->msg);
    s->msg = (char*)ALLOC(strlen(s->path) + strlen(m) + 3);
    if (s->msg == Z_NULL) return (const char*)ERR_MSG(Z_MEM_ERROR);
    strcpy(s->msg, s->path);
    strcat(s->msg, ": ");
    strcat(s->msg, m);
    return (const char*)s->msg;
}

/* ===========================================================================
     Clear the error and end-of-file flags, and do the same for the real file.
*/
void ZEXPORT gzclearerr (file)
    gzFile file;
{
    gz_stream *s = (gz_stream*)file;

    if (s == NULL) return;
    if (s->z_err != Z_STREAM_END) s->z_err = Z_OK;
    s->z_eof = 0;
/* s.kang start */
#if 1
    if( s->file != NULL ) {
        clearerr(s->file);
    }
#else 
    clearerr(s->file);
#endif
/* s.kang end */
}

