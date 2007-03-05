/************************************************************
 * lib_internal.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __LIB_INTERNAL_H
#define __LIB_INTERNAL_H

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <semaphore.h>

/************************************************************
 * Definitions
 ************************************************************/

#if CONFIG_STDIO_BUFFER_SIZE <= 0
# define lib_sem_initialize(s)
# define lib_take_semaphore(s)
# define lib_give_semaphore(s)
#endif

#define LIB_BUFLEN_UNKNOWN (0x7fffffff)

/************************************************************
 * Public Types
 ************************************************************/

/* This is the generic for of a stream used by the library
 * to manage variable sized output.
 */

struct lib_stream_s;

typedef void (*lib_putc_t)(struct lib_stream_s *this, int ch);

struct lib_stream_s
{
  lib_putc_t put;  /* Pointer to function to put one character */
  int        nput; /* Total number of characters put.  Written
                    * by put method, readable by user */
};

struct lib_memstream_s
{
  struct lib_stream_s  public;
  char                *buffer;  /* Address of first byte in the buffer */
  int                  buflen;    /* Size of the buffer in bytes */
};

struct lib_stdstream_s
{
  struct lib_stream_s  public;
  FILE                *stream;
};

struct lib_rawstream_s
{
  struct lib_stream_s  public;
  int                  fd;
};

/************************************************************
 * Public Variables
 ************************************************************/

/************************************************************
 * Pulblic Function Prototypes
 ************************************************************/

/* Defined in lib_streamsem.c */

extern void  stream_semtake(FAR struct streamlist *list);
extern void  stream_semgive(FAR struct streamlist *list);

/* Defined in lib_memstream.c */

extern void lib_memstream(struct lib_memstream_s *memstream,
                          char *bufstart, int buflen);

/* Defined in lib_stdstream.c */

extern void lib_stdstream(struct lib_stdstream_s *stdstream,
                          FILE *stream);

/* Defined in lib_rawstream.c */

extern void lib_rawstream(struct lib_rawstream_s *rawstream,
                          int fd);

/* Defined in lib_lowstream.c */

#ifdef CONFIG_ARCH_LOWPUTC
extern void lib_lowstream(struct lib_stream_s *rawstream);
#endif

/* Defined in lib_nullstream.c */

extern void lib_nullstream(struct lib_stream_s *nullstream);

/* Defined in lib_libsprintf.c */

extern int lib_sprintf (struct lib_stream_s *obj,
                        const char *fmt, ...);

/* Defined lib_libvsprintf.c */

extern int lib_vsprintf(struct lib_stream_s *obj,
                        const char *src, va_list ap);

/* Defined lib_rawprintf.c */

extern int lib_rawvprintf(const char *src, va_list ap);

/* Defined lib_lowprintf.c */

extern int lib_lowvprintf(const char *src, va_list ap);

/* Defined in lib_libwrite.c */

extern ssize_t lib_fwrite(const void *ptr, size_t count, FILE *stream);

/* Defined in lib_libfread.c */

extern ssize_t lib_fread(void *ptr, size_t count, FILE *stream);

/* Defined in lib_sem.c */

#if CONFIG_STDIO_BUFFER_SIZE > 0
extern void lib_sem_initialize(FAR struct file_struct *stream);
extern void lib_take_semaphore(FAR struct file_struct *stream);
extern void lib_give_semaphore(FAR struct file_struct *stream);
#endif

/* Defined in lib_libgetbase.c */

extern int lib_getbase(const char *nptr, const char **endptr);

#endif /* __LIB_INTERNAL_H */
