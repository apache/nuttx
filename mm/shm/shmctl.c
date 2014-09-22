/****************************************************************************
 * mm/shm/shmctl.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/shm.h>
#include <sys/ipc.h>
#include <errno.h>

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmctl
 *
 * Description:
 *   The shmctl() function provides a variety of shared memory control
 *   operations as specified by cmd. The following values for cmd are
 *   available:
 *
 *     - IPC_STAT
 *       Place the current value of each member of the shmid_ds data
 *       structure associated with shmid into the structure pointed to by buf.
 *     - IPC_SET
 *       Set the value of the following members of the shmid_ds data
 *       structure associated with shmid to the corresponding value found
 *       in the structure pointed to by buf:
 *
 *         shm_perm.uid
 *         shm_perm.gid
 *         shm_perm.mode    Low-order nine bits.
 *
 *       IPC_SET can only be executed by a process that has an effective
 *       user ID equal to either that of a process with appropriate
 *       privileges or to the value of shm_perm.cuid or shm_perm.uid in the
 *       shmid_ds data structure associated with shmid.
 *     - IPC_RMID
 *       Remove the shared memory identifier specified by shmid from the
 *       system and destroy the shared memory segment and shmid_ds data
 *       structure associated with it. IPC_RMID can only be executed by a
 *       process that has an effective user ID equal to either that of a
 *       process with appropriate privileges or to the value of
 *       shm_perm.cuid or shm_perm.uid in the shmid_ds data structure
 *       associated with shmid.
 *
 * Input Parameters:
 *   shmid - Shared memory identifier
 *   cmd - shmctl() command
 *   buf - Data associated with the shmctl() command
 *
 * Returned Value:
 *   Upon successful completion, shmctl() will return 0; otherwise, it will
 *   return -1 and set errno to indicate the error.
 *
 *     - EACCES
 *       The argument cmd is equal to IPC_STAT and the calling process does
 *       not have read permission.
 *     - EINVAL
 *       The value of shmid is not a valid shared memory identifier, or the
 *       value of cmd is not a valid command.
 *     - EPERM
 *       The argument cmd is equal to IPC_RMID or IPC_SET and the effective
 *       user ID of the calling process is not equal to that of a process
 *       with appropriate privileges and it is not equal to the value of
 *       shm_perm.cuid or shm_perm.uid in the data structure associated with
 *       shmid.
 *     - EOVERFLOW
 *       The cmd argument is IPC_STAT and the gid or uid value is too large
 *       to be stored in the structure pointed to by the buf argument.
 *
 ****************************************************************************/

int shmctl(int shmid, int cmd, struct shmid_ds *buf)
{
#warning Not implemented
  set_errno(ENOSYS);
  return ERROR;
}

#endif /* CONFIG_MM_SHM */
