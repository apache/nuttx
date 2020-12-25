/****************************************************************************
 * fs/inode/fs_inode.c
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NO_HOLDER ((pid_t)-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Implements a re-entrant mutex for inode access.  This must be re-entrant
 * because there can be cycles.  For example, it may be necessary to destroy
 * a block driver inode on umount() after a removable block device has been
 * removed.  In that case umount() holds the inode semaphore, but the block
 * driver may callback to unregister_blockdriver() after the un-mount,
 * requiring the semaphore again.
 */

struct inode_sem_s
{
  sem_t   sem;     /* The semaphore */
  pid_t   holder;  /* The current holder of the semaphore */
  int16_t count;   /* Number of counts held */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct inode_sem_s g_inode_sem;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_initialize
 *
 * Description:
 *   This is called from the OS initialization logic to configure the file
 *   system.
 *
 ****************************************************************************/

void inode_initialize(void)
{
  /* Initialize the semaphore to one (to support one-at-a-time access to the
   * inode tree).
   */

  nxsem_init(&g_inode_sem.sem, 0, 1);
  g_inode_sem.holder = NO_HOLDER;
  g_inode_sem.count  = 0;

  /* Reserve the root node */

  inode_root_reserve();

  /* Initialize files array (if it is used) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (files_initialize != NULL)
#endif
    {
      files_initialize();
    }
}

/****************************************************************************
 * Name: inode_semtake
 *
 * Description:
 *   Get exclusive access to the in-memory inode tree (g_inode_sem).
 *
 ****************************************************************************/

int inode_semtake(void)
{
  pid_t me;
  int ret = OK;

  /* Do we already hold the semaphore? */

  me = getpid();
  if (me == g_inode_sem.holder)
    {
      /* Yes... just increment the count */

      g_inode_sem.count++;
      DEBUGASSERT(g_inode_sem.count > 0);
    }

  /* Take the semaphore (perhaps waiting) */

  else
    {
      ret = nxsem_wait_uninterruptible(&g_inode_sem.sem);
      if (ret >= 0)
        {
          /* No we hold the semaphore */

          g_inode_sem.holder = me;
          g_inode_sem.count  = 1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: inode_semgive
 *
 * Description:
 *   Relinquish exclusive access to the in-memory inode tree (g_inode_sem).
 *
 ****************************************************************************/

void inode_semgive(void)
{
  DEBUGASSERT(g_inode_sem.holder == getpid());

  /* Is this our last count on the semaphore? */

  if (g_inode_sem.count > 1)
    {
      /* No.. just decrement the count */

      g_inode_sem.count--;
    }

  /* Yes.. then we can really release the semaphore */

  else
    {
      g_inode_sem.holder = NO_HOLDER;
      g_inode_sem.count  = 0;
      nxsem_post(&g_inode_sem.sem);
    }
}
