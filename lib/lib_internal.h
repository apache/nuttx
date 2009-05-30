/****************************************************************************
 * lib/lib_internal.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#ifndef __LIB_INTERNAL_H
#define __LIB_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <limits.h>
#include <semaphore.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_LIB_HOMEDIR
# define CONFIG_LIB_HOMEDIR "/"
#endif

#if CONFIG_STDIO_BUFFER_SIZE <= 0
# define lib_sem_initialize(s)
# define lib_take_semaphore(s)
# define lib_give_semaphore(s)
#endif

#define LIB_BUFLEN_UNKNOWN INT_MAX

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the generic for of a stream used by the library
 * to manage variable sized output.
 */

struct lib_instream_s;
struct lib_outstream_s;

typedef int (*lib_getc_t)(FAR struct lib_instream_s *this);
typedef void (*lib_putc_t)(FAR struct lib_outstream_s *this, int ch);

struct lib_instream_s
{
  lib_getc_t get;   /* Pointer to function to get one character */
  int        nget;  /* Total number of characters gotten.  Written
                     * by get method, readable by user */
};

struct lib_outstream_s
{
  lib_putc_t put;   /* Pointer to function to put one character */
  int        nput;  /* Total number of characters put.  Written
                     * by put method, readable by user */
};

struct lib_meminstream_s
{
  struct lib_instream_s  public;
  FAR char              *buffer;  /* Address of first byte in the buffer */
  int                    buflen;  /* Size of the buffer in bytes */
};

struct lib_memoutstream_s
{
  struct lib_outstream_s public;
  FAR char              *buffer;  /* Address of first byte in the buffer */
  int                    buflen;  /* Size of the buffer in bytes */
};

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
 
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined in lib_streamsem.c */

#if CONFIG_NFILE_STREAMS > 0
extern void  stream_semtake(FAR struct streamlist *list);
extern void  stream_semgive(FAR struct streamlist *list);
#endif

/* Defined in lib_memoutstream.c */

extern void lib_memoutstream(FAR struct lib_memoutstream_s *memoutstream,
                             FAR char *bufstart, int buflen);

/* Defined in lib_meminstream.c */

extern void lib_meminstream(FAR struct lib_meminstream_s *meminstream,
                            FAR char *bufstart, int buflen);

/* Defined in lib_stdinstream.c */

extern void lib_stdinstream(FAR struct lib_stdinstream_s *stdinstream,
                            FAR FILE *stream);

/* Defined in lib_stdoutstream.c */

extern void lib_stdoutstream(FAR struct lib_stdoutstream_s *stdoutstream,
                             FAR FILE *stream);

/* Defined in lib_rawinstream.c */

extern void lib_rawinstream(FAR struct lib_rawinstream_s *rawinstream,
                            int fd);

/* Defined in lib_rawoutstream.c */

extern void lib_rawoutstream(FAR struct lib_rawoutstream_s *rawoutstream,
                          int fd);

/* Defined in lib_lowinstream.c */

#ifdef CONFIG_ARCH_LOWGETC
extern void lib_lowinstream(FAR struct lib_instream_s *lowinstream);
#endif

/* Defined in lib_lowoutstream.c */

#ifdef CONFIG_ARCH_LOWPUTC
extern void lib_lowoutstream(FAR struct lib_outstream_s *lowoutstream);
#endif

/* Defined in lib_nullinstream.c */

extern void lib_nullinstream(FAR struct lib_instream_s *nullinstream);

/* Defined in lib_nulloutstream.c */

extern void lib_nulloutstream(FAR struct lib_outstream_s *nulloutstream);

/* Defined in lib_libsprintf.c */

extern int lib_sprintf (FAR struct lib_outstream_s *obj,
                        const char *fmt, ...);

/* Defined lib_libvsprintf.c */

extern int lib_vsprintf(FAR struct lib_outstream_s *obj,
                        const char *src, va_list ap);

/* Defined lib_rawprintf.c */

extern int lib_rawvprintf(const char *src, va_list ap);

/* Defined lib_lowprintf.c */

extern int lib_lowvprintf(const char *src, va_list ap);

/* Defined in lib_libwrite.c */

extern ssize_t lib_fwrite(FAR const void *ptr, size_t count, FAR FILE *stream);

/* Defined in lib_libfread.c */

extern ssize_t lib_fread(FAR void *ptr, size_t count, FAR FILE *stream);

/* Defined in lib_libfflush.c */

extern ssize_t lib_fflush(FAR FILE *stream, boolean bforce);

/* Defined in lib_rdflush.c */

extern int lib_rdflush(FAR FILE *stream);

/* Defined in lib_wrflush.c */

int lib_wrflush(FAR FILE *stream);

/* Defined in lib_sem.c */

#if CONFIG_STDIO_BUFFER_SIZE > 0
extern void lib_sem_initialize(FAR struct file_struct *stream);
extern void lib_take_semaphore(FAR struct file_struct *stream);
extern void lib_give_semaphore(FAR struct file_struct *stream);
#endif

/* Defined in lib_libgetbase.c */

extern int lib_getbase(const char *nptr, const char **endptr);

#endif /* __LIB_INTERNAL_H */
