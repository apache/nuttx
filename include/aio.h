/****************************************************************************
 * include/aio.h
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

#ifndef __INCLUDE_AIO_H
#define __INCLUDE_AIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <time.h>

#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* These interfaces are not available to kernel code */

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
#  undef CONFIG_FS_AIO
#endif

/* Work queue support is required.  The low-priority work queue is required
 * so that the asynchronous I/O does not interfere with high priority driver
 * operations.  If this pre-requisite is met, then asynchronous I/O support
 * can be enabled with CONFIG_FS_AIO
 */

#ifdef CONFIG_FS_AIO

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Asynchronous I/O requires CONFIG_SCHED_WORKQUEUE
#endif

/* Standard Definitions *****************************************************/

/* aio_cancel return values
 *
 * AIO_ALLDONE     - Indicates that none of the requested operations could
 *                   be cancelled since they are already complete.
 * AIO_CANCELED    - Indicates that all requested operations have been
 *                   cancelled.
 * AIO_NOTCANCELED - Indicates that some of the requested operations could
 *                   not be cancelled since they are in progress.
 */

#define AIO_CANCELED    0
#define AIO_ALLDONE     1
#define AIO_NOTCANCELED 2

/* lio_listio element operations
 *
 * LIO_NOP         - Indicates that no transfer is requested.
 * LIO_READ        - Requests a read operation.
 * LIO_WRITE       - Requests a write operation.
 */

#define LIO_NOP         0
#define LIO_READ        1
#define LIO_WRITE       2

/* lio_listio modes
 *
 * LIO_NOWAIT      - Indicates that the calling thread is to continue
 *                   execution while the lio_listio() operation is being
 *                   performed, and no notification is given when the
 *                   operation is complete.
 * LIO_WAIT        - Indicates that the calling thread is to suspend until
 *                   the lio_listio() operation is complete.
 */

#define LIO_NOWAIT      0
#define LIO_WAIT        1

#if defined(CONFIG_FS_LARGEFILE)
#  define aiocb64       aiocb
#  define aio_read64    aio_read
#  define aio_write64   aio_write
#  define aio_error64   aio_error
#  define aio_return64  aio_return
#  define aio_cancel64  aio_cancel
#  define aio_suspend64 aio_suspend
#  define aio_fsync64   aio_fsync
#  define lio_listio64  lio_listio
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct aiocb
{
  /* Standard fields required by POSIX */

  struct sigevent aio_sigevent;  /* Signal number and value */
  FAR volatile void *aio_buf;    /* Location of buffer */
  off_t aio_offset;              /* File offset */
  size_t aio_nbytes;             /* Length of transfer */
  int16_t aio_fildes;            /* File descriptor (should be int) */
  int8_t aio_reqprio;            /* Request priority offset (not used, should be int) */
  uint8_t aio_lio_opcode;        /* Operation to be performed (should be int) */

  /* Non-standard, implementation-dependent data.  For portability reasons,
   * application code should never reference these elements.
   */

  struct sigwork_s aio_sigwork;  /* Signal work */
  volatile ssize_t aio_result;   /* Support for aio_error() and aio_return() */
  FAR void *aio_priv;            /* Used by signal handlers */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int aio_cancel(int fildes, FAR struct aiocb *aiocbp);
int aio_error(FAR const struct aiocb *aiocbp);
int aio_fsync(int op, FAR struct aiocb *aiocbp);
int aio_read(FAR struct aiocb *aiocbp);
ssize_t aio_return(FAR struct aiocb *aiocbp);
int aio_suspend(FAR const struct aiocb * const list[], int nent,
                FAR const struct timespec *timeout);
int aio_write(FAR struct aiocb *aiocbp);
int lio_listio(int mode, FAR struct aiocb * const list[], int nent,
               FAR struct sigevent *sig);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_FS_AIO */
#endif /* __INCLUDE_AIO_H */
