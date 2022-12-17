/****************************************************************************
 * include/stdio.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#include <time.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sched.h>

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

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define tmpfile64 tmpfile
#  define fopen64   fopen
#  define freopen64 freopen
#  define fseeko64  fseeko
#  define ftello64  ftello
#  define fgetpos64 fgetpos
#  define fsetpos64 fsetpos
#endif

#define setlinebuf(stream)   setvbuf(stream, NULL, _IOLBF, 0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Streams */

typedef struct file_struct FILE;

struct va_format
{
  FAR const char *fmt;
  FAR va_list *va;
};

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
FAR FILE *fopen(FAR const char *path, FAR const char *type) fopen_like;
int    fprintf(FAR FILE *stream, FAR const IPTR char *format, ...)
       printflike(2, 3);
int    fputc(int c, FAR FILE *stream);
int    fputs(FAR const IPTR char *s, FAR FILE *stream);
size_t fread(FAR void *ptr, size_t size, size_t n_items, FAR FILE *stream);
FAR FILE *freopen(FAR const char *path, FAR const char *mode,
         FAR FILE *stream);
int    fscanf(FAR FILE *stream, FAR const IPTR char *fmt, ...)
       scanflike(2, 3);
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

void   setbuf(FAR FILE *stream, FAR char *buf);
int    setvbuf(FAR FILE *stream, FAR char *buffer, int mode, size_t size);

int    ungetc(int c, FAR FILE *stream);

/* Operations on the stdout stream, buffers, paths,
 * and the whole printf-family
 */

void   perror(FAR const char *s);
int    printf(FAR const IPTR char *fmt, ...) printflike(1, 2);
int    putc(int c, FAR FILE *stream);
int    putchar(int c);
int    puts(FAR const IPTR char *s);
int    rename(FAR const char *oldpath, FAR const char *newpath);
int    sprintf(FAR char *buf, FAR const IPTR char *fmt, ...)
       printflike(2, 3);
int    asprintf(FAR char **ptr, FAR const IPTR char *fmt, ...)
       printflike(2, 3);
int    snprintf(FAR char *buf, size_t size,
         FAR const IPTR char *fmt, ...) printflike(3, 4);
int    sscanf(FAR const char *buf, FAR const IPTR char *fmt, ...)
       scanflike(2, 3);

int    scanf(FAR const IPTR char *fmt, ...) scanflike(1, 2);
int    vasprintf(FAR char **ptr, FAR const IPTR char *fmt, va_list ap)
       printflike(2, 0);
int    vfprintf(FAR FILE *stream, FAR const IPTR char *fmt,
         va_list ap) printflike(2, 0);
int    vfscanf(FAR FILE *stream, FAR const IPTR char *fmt, va_list ap)
       scanflike(2, 0);
int    vprintf(FAR const IPTR char *fmt, va_list ap) printflike(1, 0);
int    vscanf(FAR const IPTR char *fmt, va_list ap) scanflike(1, 0);
int    vsnprintf(FAR char *buf, size_t size, FAR const IPTR char *fmt,
         va_list ap) printflike(3, 0);
int    vsprintf(FAR char *buf, FAR const IPTR char *fmt, va_list ap)
       printflike(2, 0);
int    vsscanf(FAR const char *buf, FAR const IPTR char *fmt, va_list ap)
       scanflike(2, 0);

/* Operations on file descriptors including:
 *
 * POSIX-like File System Interfaces (fdopen), and
 * Extensions from the Open Group Technical Standard, 2006, Extended API Set
 *   Part 1 (dprintf and vdprintf)
 */

FAR FILE *fdopen(int fd, FAR const char *type) fopen_like;
int    dprintf(int fd, FAR const IPTR char *fmt, ...) printflike(2, 3);
int    vdprintf(int fd, FAR const IPTR char *fmt, va_list ap)
       printflike(2, 0);

/* Operations on paths */

FAR FILE *tmpfile(void) fopen_like;
FAR char *tmpnam(FAR char *s);
FAR char *tempnam(FAR const char *dir, FAR const char *pfx) malloc_like;
int       remove(FAR const char *path);

/* Shell operations.  These are not actually implemented in the OS.  See
 * apps/system/popen for implementation.
 */

#ifndef __KERNEL__
int pclose(FILE *stream);
FILE *popen(FAR const char *command, FAR const char *mode) popen_like;
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_STDIO_H */
