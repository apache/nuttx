/****************************************************************************
 * fs/aio/aioc_contain.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>

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
  FAR struct file *filep;

#ifdef CONFIG_PRIORITY_INHERITANCE
  struct sched_param param;
#endif
  int ret;

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(aiocbp->aio_fildes, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  DEBUGASSERT(filep != NULL);

  /* Allocate the AIO control block container, waiting for one to become
   * available if necessary.  This should not fail except for in the case
   * where the calling thread is canceled.
   */

  aioc = aioc_alloc();
  if (aioc != NULL)
    {
      /* Initialize the container */

      memset(aioc, 0, sizeof(struct aio_container_s));
      aioc->aioc_aiocbp = aiocbp;
      aioc->aioc_filep  = filep;
      aioc->aioc_pid    = getpid();

#ifdef CONFIG_PRIORITY_INHERITANCE
      DEBUGVERIFY(nxsched_get_param (aioc->aioc_pid, &param));
      aioc->aioc_prio   = param.sched_priority;
#endif

      /* Add the container to the pending transfer list. */

      ret = aio_lock();
      if (ret < 0)
        {
          aioc_free(aioc);
          goto errout;
        }

      dq_addlast(&aioc->aioc_link, &g_aio_pending);
      aio_unlock();
    }

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
  FAR struct aiocb *aiocbp = NULL;
  int ret;

  DEBUGASSERT(aioc);

  /* Remove the container to the pending transfer list. */

  ret = aio_lock();
  if (ret >= 0)
    {
      dq_rem(&aioc->aioc_link, &g_aio_pending);

      /* De-cant the AIO control block and return the container to the
       * free list.
       */

      aiocbp = aioc->aioc_aiocbp;
      aioc_free(aioc);

      aio_unlock();
    }

  return aiocbp;
}

#endif /* CONFIG_FS_AIO */
