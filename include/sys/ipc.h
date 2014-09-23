/****************************************************************************
 * include/sys/ipc.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_IPC_H
#define __INCLUDE_SYS_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Mode bits:  The lower order 9-bit bits are the standard mode bits */

#define IPC_MODE    0x01ff     /* Mode bit mask */
#define IPC_CREAT   (1 << 10)  /* Create entry if key does not exist */
#define IPC_EXCL    (1 << 11)  /* Fail if key exists */
#define IPC_NOWAIT  (1 << 12)  /* Error if request must wait */

/* Keys: */

#define IPC_PRIVATE 0     /* Private key */

/* Control commands: */

#define IPC_RMID    0    /* Remove identifier */
#define IPC_SET     1    /* Set options */
#define IPC_STAT    2    /* Get options */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

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

/* The ipc_perm structure is used by three mechanisms for XSI interprocess
 * communication (IPC): messages, semaphores, and shared memory. All use a
 * common structure type, ipc_perm, to pass information used in determining
 * permission to perform an IPC operation.
 */

struct ipc_perm
{
#if 0 /* User and group IDs not yet supported by NuttX */
  uid_t  uid;    /* Owner's user ID */
  gid_t  gid;    /* Owner's group ID */
  uid_t  cuid;   /* Creator's user ID */
  gid_t  cgid;   /* Creator's group ID */
#endif
  mode_t mode;   /* Read/write permission */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

key_t ftok(FAR const char *path, int id);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_IPC_H */
