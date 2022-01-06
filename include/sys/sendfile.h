/****************************************************************************
 * include/sys/sendfile.h
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

#ifndef __INCLUDE_SYS_SENDFILE_H
#define __INCLUDE_SYS_SENDFILE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SENDFILE_BUFSIZE
#  define CONFIG_SENDFILE_BUFSIZE 512
#endif

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define sendfile64              sendfile
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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
 * Name: sendfile
 *
 * Description:
 *   sendfile() copies data between one file descriptor and another.
 *   sendfile() basically just wraps a sequence of reads() and writes()
 *   to perform a copy.  It serves a purpose in systems where there is
 *   a penalty for copies to between user and kernel space, but really
 *   nothing in NuttX but provide some Linux compatible (and adding
 *   another 'almost standard' interface).
 *
 *   NOTE: This interface is *not* specified in POSIX.1-2001, or other
 *   standards.  The implementation here is very similar to the Linux
 *   sendfile interface.  Other UNIX systems implement sendfile() with
 *   different semantics and prototypes.  sendfile() should not be used
 *   in portable programs.
 *
 * Input Parameters:
 *   infd   - A file (or socket) descriptor opened for reading
 *   outfd  - A descriptor opened for writing.
 *   offset - If 'offset' is not NULL, then it points to a variable
 *            holding the file offset from which sendfile() will start
 *            reading data from 'infd'.  When sendfile() returns, this
 *            variable will be set to the offset of the byte following
 *            the last byte that was read.  If 'offset' is not NULL,
 *            then sendfile() does not modify the current file offset of
 *            'infd'; otherwise the current file offset is adjusted to
 *            reflect the number of bytes read from 'infd.'
 *
 *            If 'offset' is NULL, then data will be read from 'infd'
 *            starting at the current file offset, and the file offset
 *            will be updated by the call.
 *   count -  The number of bytes to copy between the file descriptors.
 *
 * Returned Value:
 *   If the transfer was successful, the number of bytes written to outfd is
 *   returned.  On error, -1 is returned, and errno is set appropriately.
 *   There error values are those returned by read() or write() plus:
 *
 *   EINVAL - Bad input parameters.
 *   ENOMEM - Could not allocated an I/O buffer
 *
 ****************************************************************************/

ssize_t sendfile(int outfd, int infd, FAR off_t *offset, size_t count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_SENDFILE_H */
