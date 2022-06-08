/****************************************************************************
 * include/sys/ipc.h
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

#ifndef __INCLUDE_SYS_IPC_H
#define __INCLUDE_SYS_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mode bits:  The lower order 9-bit bits are the standard mode bits */

#define IPC_MODE   0x01ff /* Mode bit mask */
#define IPC_CREAT  0x0200 /* Create key if key does not exist. */
#define IPC_EXCL   0x0400 /* Fail if key exists.  */
#define IPC_NOWAIT 0x0800 /* Return error on wait.  */

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
