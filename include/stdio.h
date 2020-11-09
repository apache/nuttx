/****************************************************************************
 * include/stdio.h
 *
 *   Copyright (C) 2007-2009, 2011, 2013-2015, 2018-2019 Gregory Nutt. All
 *     rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_STDIO_H
#define __INCLUDE_STDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdarg.h>
#include <sched.h>
#include <time.h>

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* File System Definitions **************************************************/

#define FILENAME_MAX _POSIX_NAME_MAX

/* The (default) size of the I/O buffers */

#if defined(CONFIG_STDIO_BUFFER_SIZE) && CONFIG_STDIO_BUFFER_SIZE > 0
#  define BUFSIZ   CONFIG_STDIO_BUFFER_SIZE
#else
#  define BUFSIZ   64
#endif

/* The following three definitions are for ANSI C, used by setvbuf */

#define _IOFBF     0               /* Fully buffered */
#define _IOLBF     1               /* Line buffered */
#define _IONBF     2               /* Unbuffered */

/* File system error values */

#define EOF        (-1)

/* The first three _iob entries are reserved for standard I/O */

#define stdin      (&nxsched_get_streams()->sl_std[0])
#define stdout     (&nxsched_get_streams()->sl_std[1])
#define stderr     (&nxsched_get_streams()->sl_std[2])

/* Path to the directory where temporary files can be created */

#ifndef CONFIG_LIBC_TMPDIR
#  define CONFIG_LIBC_TMPDIR "/tmp"
#endif

#define P_tmpdir   CONFIG_LIBC_TMPDIR

/* Maximum size of character array to hold tmpnam() output. */

#ifndef CONFIG_LIBC_MAX_TMPFILE
#  define CONFIG_LIBC_MAX_TMPFILE 32
#endif

#define L_tmpnam   CONFIG_LIBC_MAX_TMPFILE

/* The maximum number of unique temporary file names that can be generated */

#define TMP_MAX 56800235584ull

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Streams */

typedef struct file_struct FILE;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* ANSI-like File System Interfaces */

/* Operations on streams (FILE) */

void   clearerr(FAR FILE *stream);
int    fclose(FAR FILE *stream);
int    fflush(FAR FILE *stream);
int    feof(FAR FILE *stream);
int    ferror(FAR FILE *stream);
int    fileno(FAR FILE *stream);
int    fgetc(FAR FILE *stream);
int    fgetpos(FAR FILE *stream, FAR fpos_t *pos);
FAR char *fgets(FAR char *s, int n, FAR FILE *stream);
FAR FILE *fopen(FAR const char *path, FAR const char *type);
int    fprintf(FAR FILE *stream, FAR const IPTR char *format, ...);
int    fputc(int c, FAR FILE *stream);
int    fputs(FAR const IPTR char *s, FAR FILE *stream);
size_t fread(FAR void *ptr, size_t size, size_t n_items, FAR FILE *stream);
FAR FILE *freopen(FAR const char *path, FAR const char *mode,
         FAR FILE *stream);
int    fscanf(FAR FILE *stream, FAR const IPTR char *fmt, ...);
int    fseek(FAR FILE *stream, long int offset, int whence);
int    fseeko(FAR FILE *stream, off_t offset, int whence);
int    fsetpos(FAR FILE *stream, FAR fpos_t *pos);
long   ftell(FAR FILE *stream);
off_t  ftello(FAR FILE *stream);
size_t fwrite(FAR const void *ptr, size_t size, size_t n_items,
         FAR FILE *stream);
int     getc(FAR FILE *stream);
int     getchar(void);
ssize_t getdelim(FAR char **lineptr, size_t *n, int delimiter,
         FAR FILE *stream);
ssize_t getline(FAR char **lineptr, size_t *n, FAR FILE *stream);
FAR char *gets(FAR char *s);
FAR char *gets_s(FAR char *s, rsize_t n);
void   rewind(FAR FILE *stream);

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
void   setbuf(FAR FILE *stream, FAR char *buf);
int    setvbuf(FAR FILE *stream, FAR char *buffer, int mode, size_t size);
#endif

int    ungetc(int c, FAR FILE *stream);

/* Operations on the stdout stream, buffers, paths,
 * and the whole printf-family
 */

void   perror(FAR const char *s);
int    printf(FAR const IPTR char *fmt, ...);
int    putc(int c, FAR FILE *stream);
int    putchar(int c);
int    puts(FAR const IPTR char *s);
int    rename(FAR const char *oldpath, FAR const char *newpath);
int    sprintf(FAR char *buf, FAR const IPTR char *fmt, ...);
int    asprintf(FAR char **ptr, FAR const IPTR char *fmt, ...);
int    snprintf(FAR char *buf, size_t size,
         FAR const IPTR char *fmt, ...);
int    sscanf(FAR const char *buf, FAR const IPTR char *fmt, ...);

int    scanf(FAR const IPTR char *fmt, ...);
int    vasprintf(FAR char **ptr, FAR const IPTR char *fmt, va_list ap);
int    vfprintf(FAR FILE *stream, FAR const IPTR char *fmt,
         va_list ap);
int    vfscanf(FAR FILE *stream, FAR const IPTR char *fmt, va_list ap);
int    vprintf(FAR const IPTR char *fmt, va_list ap);
int    vscanf(FAR const IPTR char *fmt, va_list ap);
int    vsnprintf(FAR char *buf, size_t size, FAR const IPTR char *fmt,
         va_list ap);
int    vsprintf(FAR char *buf, FAR const IPTR char *fmt, va_list ap);
int    vsscanf(FAR const char *buf, FAR const IPTR char *fmt, va_list ap);

/* Operations on file descriptors including:
 *
 * POSIX-like File System Interfaces (fdopen), and
 * Extensions from the Open Group Technical Standard, 2006, Extended API Set
 *   Part 1 (dprintf and vdprintf)
 */

FAR FILE *fdopen(int fd, FAR const char *type);
int    dprintf(int fd, FAR const IPTR char *fmt, ...);
int    vdprintf(int fd, FAR const IPTR char *fmt, va_list ap);

/* Operations on paths */

FAR FILE *tmpfile(void);
FAR char *tmpnam(FAR char *s);
FAR char *tempnam(FAR const char *dir, FAR const char *pfx);
int       remove(FAR const char *path);

/* Shell operations.  These are not actually implemented in the OS.  See
 * apps/system/popen for implementation.
 */

#ifndef __KERNEL__
FILE *popen(FAR const char *command, FAR const char *mode);
int pclose(FILE *stream);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_STDIO_H */
