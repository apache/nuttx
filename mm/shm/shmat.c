/****************************************************************************
 * mm/shm/shmat.c
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
 * Name: shmat
 *
 * Description:
 *   The shmat() function attaches the shared memory segment associated with
 *   the shared memory identifier specified by shmid to the address space of
 *   the calling process. The segment is attached at the address specified
 *   by one of the following criteria:
 *
 *     - If shmaddr is a null pointer, the segment is attached at the first
 *       available address as selected by the system.
 *     - If shmaddr is not a null pointer and (shmflg & SHM_RND) is non-
 *       zero, the segment is attached at the address given by
 *       (shmaddr - ((uintptr_t)shmaddr % SHMLBA)).
 *     - If shmaddr is not a null pointer and (shmflg & SHM_RND) is 0, the
 *       segment is attached at the address given by shmaddr.
 *     - The segment is attached for reading if (shmflg & SHM_RDONLY) is
 *       non-zero and the calling process has read permission; otherwise, if
 *       it is 0 and the calling process has read and write permission, the
 *       segment is attached for reading and writing.
 *
 * Input Parameters:
 *   shmid  - Shared memory identifier
 *   smaddr - Determines mapping of the shared memory region
 *   shmflg - See SHM_* definitions in include/sys/shm.h.  Only SHM_RDONLY
 *            and SHM_RND are supported.
 *
 * Returned Value:
 *   Upon successful completion, shmat() will increment the value of
 *   shm_nattch in the data structure associated with the shared memory ID
 *   of the attached shared memory segment and return the segment's start address.
 *
 *   Otherwise, the shared memory segment will not be attached, shmat() will
 *   return -1, and errno will be set to indicate the error.
 *
 *     - EACCES
 *       Operation permission is denied to the calling process
 *     - EINVAL
 *       The value of shmid is not a valid shared memory identifier, the
 *       shmaddr is not a null pointer, and the value of
 *       (shmaddr -((uintptr_t)shmaddr % SHMLBA)) is an illegal address for
 *       attaching shared memory; or the shmaddr is not a null pointer,
 *       (shmflg & SHM_RND) is 0, and the value of shmaddr is an illegal
 *       address for attaching shared memory.
 *     - EMFILE
 *       The number of shared memory segments attached to the calling
 *       process would exceed the system-imposed limit.
 *     - ENOMEM
 *       The available data space is not large enough to accommodate the
 *       shared memory segment.
 *
 ****************************************************************************/

FAR void *shmat(int shmid, FAR const void *shmaddr, int shmflg)
{
#warning Not implemented
  set_errno(ENOSYS);
  return (FAR void *)ERROR;
}

#endif /* CONFIG_MM_SHM */
