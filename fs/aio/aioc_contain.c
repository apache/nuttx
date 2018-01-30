/****************************************************************************
 * fs/aio/aioc_contain.c
 *
 *   Copyright (C) 2014, 2017-2018 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_contain
 *
 * Description:
 *   Create and initialize a container for the provided AIO control block
 *
 * Input Parameters:
 *   aiocbp - The AIO control block pointer
 *
 * Returned Value:
 *   A reference to the new AIO control block container.   This function
 *   will not fail but will wait if necessary for the resources to perform
 *   this operation.  NULL will be returned on certain errors with the
 *   errno value already set appropriately.
 *
 ****************************************************************************/

FAR struct aio_container_s *aio_contain(FAR struct aiocb *aiocbp)
{
  FAR struct aio_container_s *aioc;
  union
  {
#ifdef AIO_HAVE_FILEP
    FAR struct file *filep;
#endif
#ifdef AIO_HAVE_FILEP
    FAR struct socket *psock;
#endif
    FAR void *ptr;
  } u;
#ifdef CONFIG_PRIORITY_INHERITANCE
  struct sched_param param;
#endif
  int ret;

#if defined(AIO_HAVE_FILEP) && defined(AIO_HAVE_PSOCK)
  if (aiocbp->aio_fildes < CONFIG_NFILE_DESCRIPTORS)
#endif
#ifdef AIO_HAVE_FILEP
    {
      /* Get the file structure corresponding to the file descriptor. */

      ret = fs_getfilep(aiocbp->aio_fildes, &u.filep);
      if (ret < 0)
        {
          goto errout;
        }

      DEBUGASSERT(u.filep != NULL);
    }
#endif
#if defined(AIO_HAVE_FILEP) && defined(AIO_HAVE_PSOCK)
  else
#endif
#ifdef AIO_HAVE_PSOCK
    {
      /* Get the socket structure corresponding to the socket descriptor */

      u.psock = sockfd_socket(aiocbp->aio_fildes);
      if (u.psock == NULL)
        {
          /* Does not return error information.  EBADF is the most likely
           * explanation.
           */

          ret = -EBADF;
          goto errout;
        }
    }
#endif

  /* Allocate the AIO control block container, waiting for one to become
   * available if necessary.  This should never fail.
   */

  aioc = aioc_alloc();
  DEBUGASSERT(aioc);

  /* Initialize the container */

  memset(aioc, 0, sizeof(struct aio_container_s));
  aioc->aioc_aiocbp = aiocbp;
  aioc->u.ptr = u.ptr;
  aioc->aioc_pid = getpid();

#ifdef CONFIG_PRIORITY_INHERITANCE
  DEBUGVERIFY(nxsched_getparam (aioc->aioc_pid, &param));
  aioc->aioc_prio = param.sched_priority;
#endif

  /* Add the container to the pending transfer list. */

  aio_lock();
  dq_addlast(&aioc->aioc_link, &g_aio_pending);
  aio_unlock();
  return aioc;

errout:
  set_errno(-ret);
  return NULL;
}

/****************************************************************************
 * Name: aioc_decant
 *
 * Description:
 *   Remove the AIO control block from the container and free all resources
 *   used by the container.
 *
 * Input Parameters:
 *   aioc - Pointer to the AIO control block container
 *
 * Returned Value:
 *   A pointer to the no-longer contained AIO control block.
 *
 ****************************************************************************/

FAR struct aiocb *aioc_decant(FAR struct aio_container_s *aioc)
{
  FAR struct aiocb *aiocbp;

  DEBUGASSERT(aioc);

  /* Remove the container to the pending transfer list. */

  aio_lock();
  dq_rem(&aioc->aioc_link, &g_aio_pending);

  /* De-cant the AIO control block and return the container to the free list */

  aiocbp = aioc->aioc_aiocbp;
  aioc_free(aioc);

  aio_unlock();
  return aiocbp;
}

#endif /* CONFIG_FS_AIO */
