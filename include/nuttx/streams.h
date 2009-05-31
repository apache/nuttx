/****************************************************************************
 * include/nuttx/streams.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _INCLUDE_NUTTX_STREAMS_H
#define _INCLUDE_NUTTX_STREAMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These are the generic representations of a streams used by the NuttX */

struct lib_instream_s;
struct lib_outstream_s;

typedef int (*lib_getc_t)(FAR struct lib_instream_s *this);
typedef void (*lib_putc_t)(FAR struct lib_outstream_s *this, int ch);

struct lib_instream_s
{
  lib_getc_t get;                 /* Pointer to function to get one character */
  int        nget;                /* Total number of characters gotten.  Written
                                   * by get method, readable by user */
};

struct lib_outstream_s
{
  lib_putc_t put;                 /* Pointer to function to put one character */
  int        nput;                /* Total number of characters put.  Written
                                   * by put method, readable by user */
};

/* These are streams that operate on a fixed-sized block of memory */

struct lib_meminstream_s
{
  struct lib_instream_s  public;
  FAR const char        *buffer;  /* Address of first byte in the buffer */
  int                    buflen;  /* Size of the buffer in bytes */
};

struct lib_memoutstream_s
{
  struct lib_outstream_s public;
  FAR char              *buffer;  /* Address of first byte in the buffer */
  int                    buflen;  /* Size of the buffer in bytes */
};

/* These are streams that operate on a FILE */

struct lib_stdinstream_s
{
  struct lib_instream_s  public;
  FAR FILE              *stream;
};

struct lib_stdoutstream_s
{
  struct lib_outstream_s public;
  FAR FILE              *stream;
};

/* These are streams that operate on a file descriptor */

struct lib_rawoutstream_s
{
  struct lib_outstream_s public;
  int                    fd;
};

struct lib_rawinstream_s
{
  struct lib_instream_s  public;
  int                    fd;
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/
 
#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined in lib/lib_meminstream.c */

EXTERN void lib_meminstream(FAR struct lib_meminstream_s *meminstream,
                            FAR const char *bufstart, int buflen);

/* Defined in lib/lib_memoutstream.c */

EXTERN void lib_memoutstream(FAR struct lib_memoutstream_s *memoutstream,
                             FAR char *bufstart, int buflen);

/* Defined in lib/lib_stdinstream.c */

EXTERN void lib_stdinstream(FAR struct lib_stdinstream_s *stdinstream,
                            FAR FILE *stream);

/* Defined in lib/lib_stdoutstream.c */

EXTERN void lib_stdoutstream(FAR struct lib_stdoutstream_s *stdoutstream,
                             FAR FILE *stream);

/* Defined in lib/lib_rawinstream.c */

EXTERN void lib_rawinstream(FAR struct lib_rawinstream_s *rawinstream,
                            int fd);

/* Defined in lib/lib_rawoutstream.c */

EXTERN void lib_rawoutstream(FAR struct lib_rawoutstream_s *rawoutstream,
                          int fd);

/* Defined in lib/lib_lowinstream.c */

#ifdef CONFIG_ARCH_LOWGETC
EXTERN void lib_lowinstream(FAR struct lib_instream_s *lowinstream);
#endif

/* Defined in lib/lib_lowoutstream.c */

#ifdef CONFIG_ARCH_LOWPUTC
EXTERN void lib_lowoutstream(FAR struct lib_outstream_s *lowoutstream);
#endif

/* Defined in lib/lib_nullinstream.c */

EXTERN void lib_nullinstream(FAR struct lib_instream_s *nullinstream);

/* Defined in lib/lib_nulloutstream.c */

EXTERN void lib_nulloutstream(FAR struct lib_outstream_s *nulloutstream);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* _INCLUDE_NUTTX_STREAMS_H */
