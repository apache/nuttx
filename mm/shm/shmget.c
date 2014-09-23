/****************************************************************************
 * mm/shm/shmget.c
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
 * Name: shmget
 *
 * Description:
 *   The shmget() function will return the shared memory identifier
 *   associated with key.
 *
 *   A shared memory identifier, associated data structure, and shared
 *   memory segment of at least size bytes is created for key if one of the
 *   following is true:
 *
 *     - The argument key is equal to IPC_PRIVATE.
 *     - The argument key does not already have a shared memory identifier
 *       associated with it and (shmflg & IPC_CREAT) is non-zero.
 *
 *   Upon creation, the data structure associated with the new shared memory
 *   identifier will be initialized as follows:
 *
 *     - The values of shm_perm.cuid, shm_perm.uid, shm_perm.cgid, and
 *       shm_perm.gid are set equal to the effective user ID and effective
 *       group ID, respectively, of the calling process.
 *     - The low-order nine bits of shm_perm.mode are set equal to the low-
 *       order nine bits of shmflg.
 *     - The value of shm_segsz is set equal to the value of size.
 *     - The values of shm_lpid, shm_nattch, shm_atime, and shm_dtime are
 *       set equal to 0.
 *     - The value of shm_ctime is set equal to the current time.
 *
 *   When the shared memory segment is created, it will be initialized with
 *   all zero values.
 *
 * Input Parameters:
 *   key     - The key that is used to access the unique shared memory
 *             identifier.
 *   size    - The shared memory region that is created will be at least
 *             this size in bytes.
 *   shmflgs - See IPC_* definitions in sys/ipc.h.  Only the values
 *             IPC_PRIVATE or IPC_CREAT are supported.
 *
 * Returned Value:
 *   Upon successful completion, shmget() will return a non-negative
 *   integer, namely a shared memory identifier; otherwise, it will return
 *   -1 and set errno to indicate the error.
 *
 *     - EACCES
 *       A shared memory identifier exists for key but operation permission
 *       as specified by the low-order nine bits of shmflg would not be
 *       granted.
 *     - EEXIST
 *       A shared memory identifier exists for the argument key but
 *      (shmflg & IPC_CREAT) && (shmflg & IPC_EXCL) are non-zero.
 *     - EINVAL
 *       A shared memory segment is to be created and the value of size is
 *       less than the system-imposed minimum or greater than the system-
 *       imposed maximum.
 *     - EINVAL
 *       No shared memory segment is to be created and a shared memory
 *       segment exists for key but the size of the segment associated with
 *       it is less than size and size is not 0.
 *     - ENOENT
 *       A shared memory identifier does not exist for the argument key and
 *       (shmflg & IPC_CREAT) is 0.
 *     - ENOMEM
 *       A shared memory identifier and associated shared memory segment
 *       will be created, but the amount of available physical memory is
 *       not sufficient to fill the request.
 *     - ENOSPC
 *       A shared memory identifier is to be created, but the system-imposed
 *       limit on the maximum number of allowed shared memory identifiers
 *       system-wide would be exceeded.
 *
 ****************************************************************************/

int shmget(key_t key, size_t size, int shmflg)
{
#warning Not implemented
  set_errno(ENOSYS);
  return ERROR;
}

#endif /* CONFIG_MM_SHM */

