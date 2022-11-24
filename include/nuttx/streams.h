/****************************************************************************
 * include/nuttx/streams.h
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

#ifndef __INCLUDE_NUTTX_STREAMS_H
#define __INCLUDE_NUTTX_STREAMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <lzf.h>
#include <stdio.h>
#ifndef CONFIG_DISABLE_MOUNTPOINT
#include <nuttx/fs/fs.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_LZF
#define LZF_STREAM_BLOCKSIZE  ((1 << CONFIG_STREAM_LZF_BLOG) - 1)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These are the generic representations of a streams used by the NuttX */

struct lib_instream_s;
typedef CODE int  (*lib_getc_t)(FAR struct lib_instream_s *this);
typedef CODE int  (*lib_gets_t)(FAR struct lib_instream_s *this,
                                FAR void *buf, int len);

struct lib_outstream_s;
typedef CODE void (*lib_putc_t)(FAR struct lib_outstream_s *this, int ch);
typedef CODE int  (*lib_puts_t)(FAR struct lib_outstream_s *this,
                                FAR const void *buf, int len);
typedef CODE int  (*lib_flush_t)(FAR struct lib_outstream_s *this);

struct lib_instream_s
{
  int                    nget;    /* Total number of characters gotten.  Written
                                   * by get method, readable by user */
  lib_getc_t             get;     /* Get one character from the instream */
  lib_gets_t             gets;    /* Get the string from the instream */
};

struct lib_outstream_s
{
  int                    nput;    /* Total number of characters put.  Written
                                   * by put method, readable by user */
  lib_putc_t             put;     /* Put one character to the outstream */
  lib_puts_t             puts;    /* Writes the string to the outstream */
  lib_flush_t            flush;   /* Flush any buffered characters in the outstream */
};

/* Seek-able streams */

struct lib_sistream_s;
typedef CODE int   (*lib_sigetc_t)(FAR struct lib_sistream_s *this);
typedef CODE int   (*lib_sigets_t)(FAR struct lib_sistream_s *this,
                                   FAR void *buf, int len);
typedef CODE off_t (*lib_siseek_t)(FAR struct lib_sistream_s *this,
                                   off_t offset, int whence);

struct lib_sostream_s;
typedef CODE void  (*lib_soputc_t)(FAR struct lib_sostream_s *this, int ch);
typedef CODE int   (*lib_soputs_t)(FAR struct lib_sostream_s *this,
                                   FAR const void *buf, int len);
typedef CODE int   (*lib_soflush_t)(FAR struct lib_sostream_s *this);
typedef CODE off_t (*lib_soseek_t)(FAR struct lib_sostream_s *this,
                                   off_t offset, int whence);

struct lib_sistream_s
{
  int                    nget;    /* Total number of characters gotten.  Written
                                   * by get method, readable by user */
  lib_sigetc_t           get;     /* Get one character from the instream */
  lib_gets_t             gets;    /* Get the string from the instream */
  lib_siseek_t           seek;    /* Seek to a position in the instream */
};

struct lib_sostream_s
{
  int                    nput;    /* Total number of characters put.  Written
                                   * by put method, readable by user */
  lib_soputc_t           put;     /* Put one character to the outstream */
  lib_soputs_t           puts;    /* Writes the string to the outstream */
  lib_soflush_t          flush;   /* Flush any buffered characters in the outstream */
  lib_soseek_t           seek;    /* Seek a position in the output stream */
};

/* These are streams that operate on a fixed-sized block of memory */

struct lib_meminstream_s
{
  struct lib_instream_s  public;
  FAR const char        *buffer;  /* Address of first byte in the buffer */
  size_t                 buflen;  /* Size of the buffer in bytes */
};

struct lib_memoutstream_s
{
  struct lib_outstream_s public;
  FAR char              *buffer;  /* Address of first byte in the buffer */
  size_t                 buflen;  /* Size of the buffer in bytes */
};

struct lib_memsistream_s
{
  struct lib_sistream_s  public;
  FAR const char        *buffer;  /* Address of first byte in the buffer */
  size_t                 offset;  /* Current buffer offset in bytes */
  size_t                 buflen;  /* Size of the buffer in bytes */
};

struct lib_memsostream_s
{
  struct lib_sostream_s  public;
  FAR char              *buffer;  /* Address of first byte in the buffer */
  size_t                 offset;  /* Current buffer offset in bytes */
  size_t                 buflen;  /* Size of the buffer in bytes */
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

struct lib_stdsistream_s
{
  struct lib_sistream_s  public;
  FAR FILE              *stream;
};

struct lib_stdsostream_s
{
  struct lib_sostream_s  public;
  FAR FILE              *stream;
};

/* These are streams that operate on a file descriptor */

struct lib_rawinstream_s
{
  struct lib_instream_s  public;
  int                    fd;
};

struct lib_rawoutstream_s
{
  struct lib_outstream_s public;
  int                    fd;
};

struct lib_rawsistream_s
{
  struct lib_sistream_s  public;
  int                    fd;
};

struct lib_rawsostream_s
{
  struct lib_sostream_s  public;
  int                    fd;
};

/* LZF compressed stream pipeline */

#ifdef CONFIG_LIBC_LZF
struct lib_lzfoutstream_s
{
  struct lib_outstream_s      public;
  FAR struct lib_outstream_s *backend;
  lzf_state_t                 state;
  size_t                      offset;
  char                        in[LZF_STREAM_BLOCKSIZE];
  char                        out[LZF_MAX_HDR_SIZE + LZF_STREAM_BLOCKSIZE];
};
#endif

#ifndef CONFIG_DISABLE_MOUNTPOINT
struct lib_blkoutstream_s
{
  struct lib_outstream_s public;
  FAR struct inode      *inode;
  struct geometry        geo;
  FAR unsigned char     *cache;
};
#endif

/****************************************************************************
 * Public Data
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

/****************************************************************************
 * Name: lib_meminstream, lib_memoutstream, lib_memsistream, lib_memsostream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *   Defined in lib/stdio/lib_meminstream.c and lib/stdio/lib_memoutstream.c.
 *   Seekable versions are defined in lib/stdio/lib_memsistream.c and
 *   lib/stdio/lib_memsostream.c.
 *
 * Input Parameters:
 *   memstream    - User allocated, uninitialized instance of struct
 *                  lib_meminstream_s to be initialized.
 *   memstream    - User allocated, uninitialized instance of struct
 *                  lib_memoutstream_s to be initialized.
 *   bufstart     - Address of the beginning of the fixed-size memory buffer
 *   buflen       - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_meminstream(FAR struct lib_meminstream_s *instream,
                     FAR const char *bufstart, int buflen);
void lib_memoutstream(FAR struct lib_memoutstream_s *outstream,
                      FAR char *bufstart, int buflen);
void lib_memsistream(FAR struct lib_memsistream_s *instream,
                     FAR const char *bufstart, int buflen);
void lib_memsostream(FAR struct lib_memsostream_s *outstream,
                     FAR char *bufstart, int buflen);

/****************************************************************************
 * Name: lib_stdinstream, lib_stdoutstream
 *
 * Description:
 *   Initializes a stream for use with a FILE instance.
 *   Defined in lib/stdio/lib_stdinstream.c and lib/stdio/lib_stdoutstream.c
 *
 * Input Parameters:
 *   instream  - User allocated, uninitialized instance of struct
 *               lib_stdinstream_s to be initialized.
 *   outstream - User allocated, uninitialized instance of struct
 *               lib_stdoutstream_s to be initialized.
 *   stream    - User provided stream instance (must have been opened for
 *               the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdinstream(FAR struct lib_stdinstream_s *instream,
                     FAR FILE *stream);
void lib_stdoutstream(FAR struct lib_stdoutstream_s *outstream,
                      FAR FILE *stream);
void lib_stdsistream(FAR struct lib_stdsistream_s *instream,
                     FAR FILE *stream);
void lib_stdsostream(FAR struct lib_stdsostream_s *outstream,
                     FAR FILE *stream);

/****************************************************************************
 * Name: lib_rawinstream, lib_rawoutstream, lib_rawsistream, and
 *       lib_rawsostream,
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *   Defined in lib/stdio/lib_rawinstream.c and lib/stdio/lib_rawoutstream.c.
 *   Seekable versions are defined in lib/stdio/lib_rawsistream.c and
 *   lib/stdio/lib_rawsostream.c
 *
 * Input Parameters:
 *   instream  - User allocated, uninitialized instance of struct
 *               lib_rawinstream_s to be initialized.
 *   outstream - User allocated, uninitialized instance of struct
 *               lib_rawoutstream_s to be initialized.
 *   fd        - User provided file/socket descriptor (must have been opened
 *               for the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawinstream(FAR struct lib_rawinstream_s *instream, int fd);
void lib_rawoutstream(FAR struct lib_rawoutstream_s *outstream, int fd);
void lib_rawsistream(FAR struct lib_rawsistream_s *instream, int fd);
void lib_rawsostream(FAR struct lib_rawsostream_s *outstream, int fd);

/****************************************************************************
 * Name: lib_lowoutstream
 *
 * Description:
 *   Initializes a stream for use with low-level, architecture-specific
 *   output.
 *   Defined in ib/stdio/lib_lowoutstream.c
 *
 * Input Parameters:
 *   lowoutstream - User allocated, uninitialized instance of struct
 *                  lib_outstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LOWPUTC
void lib_lowoutstream(FAR struct lib_outstream_s *lowoutstream);
#endif

/****************************************************************************
 * Name: lib_zeroinstream, lib_nullinstream, lib_nulloutstream
 *
 * Description:
 *   Initializes NULL streams:
 *
 *   o The stream created by lib_zeroinstream will return an infinitely long
 *     stream of zeroes. Defined in lib/stdio/lib_zeroinstream.c
 *   o The stream created by lib_nullinstream will return only EOF.
 *     Defined in lib/stdio/lib_nullinstream.c
 *   o The stream created by lib_nulloutstream will write all data to the
 *     bit-bucket. Defined in lib/stdio/lib_nulloutstream.c
 *
 * Input Parameters:
 *   zeroinstream  - User allocated, uninitialized instance of struct
 *                   lib_instream_s to be initialized.
 *   nullinstream  - User allocated, uninitialized instance of struct
 *                   lib_instream_s to be initialized.
 *   nulloutstream - User allocated, uninitialized instance of struct
 *                   lib_outstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_zeroinstream(FAR struct lib_instream_s *zeroinstream);
void lib_nullinstream(FAR struct lib_instream_s *nullinstream);
void lib_nulloutstream(FAR struct lib_outstream_s *nulloutstream);

/****************************************************************************
 * Name: lib_lzfoutstream
 *
 * Description:
 *  LZF compressed pipeline stream
 *
 * Input Parameters:
 *   stream  - User allocated, uninitialized instance of struct
 *                lib_lzfoutstream_s to be initialized.
 *   backend - Stream backend port.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_LZF
void lib_lzfoutstream(FAR struct lib_lzfoutstream_s *stream,
                      FAR struct lib_outstream_s *backend);
#endif

/****************************************************************************
 * Name: lib_blkoutstream_open
 *
 * Description:
 *  open block driver stream backend
 *
 * Input Parameters:
 *   stream  - User allocated, uninitialized instance of struct
 *                lib_blkoutstream_s to be initialized.
 *   name    - The full path to the block driver to be opened.
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int lib_blkoutstream_open(FAR struct lib_blkoutstream_s *stream,
                          FAR const char *name);
#endif

/****************************************************************************
 * Name: lib_blkoutstream_close
 *
 * Description:
 *  close block driver stream backend
 *
 * Input Parameters:
 *   stream  - User allocated, uninitialized instance of struct
 *                lib_blkoutstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
void lib_blkoutstream_close(FAR struct lib_blkoutstream_s *stream);
#endif

/****************************************************************************
 * Name: lib_noflush
 *
 * Description:
 *  lib_noflush() provides a common, dummy flush method for output streams
 *  that are not flushable.
 *
 * Returned Value:
 *  Always returns OK
 *
 ****************************************************************************/

int lib_noflush(FAR struct lib_outstream_s *stream);

/****************************************************************************
 * Name: lib_snoflush
 *
 * Description:
 *  lib_snoflush() provides a common, dummy flush method for seekable output
 *  streams that are not flushable.
 *  is selected.
 *
 * Returned Value:
 *  Always returns OK
 *
 ****************************************************************************/

int lib_snoflush(FAR struct lib_sostream_s *this);

/****************************************************************************
 * Name: lib_sprintf
 *
 * Description:
 *  Stream-oriented implementation of sprintf.  Used only by the SYSLOG.
 *
 ****************************************************************************/

int lib_sprintf(FAR struct lib_outstream_s *obj,
                FAR const IPTR char *fmt, ...) printflike(2, 3);

/****************************************************************************
 * Name: lib_vsprintf
 *
 * Description:
 *  Stream-oriented implementation that underlies printf family:  printf,
 *  fprint, sprint, etc.
 *
 ****************************************************************************/

int lib_vsprintf(FAR struct lib_outstream_s *obj,
                 FAR const IPTR char *src, va_list ap) printflike(2, 0);

/****************************************************************************
 * Name: lib_vscanf
 *
 * Description:
 *  Stream-oriented implementation that underlies scanf family:  scanf,
 *  fscanf, vfscanf, sscanf, and vsscanf
 *
 ****************************************************************************/

int lib_vscanf(FAR struct lib_instream_s *obj, FAR int *lastc,
               FAR const IPTR char *src, va_list ap) scanflike(3, 0);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_STREAMS_H */
