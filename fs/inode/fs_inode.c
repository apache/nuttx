/****************************************************************************
 * fs/inode/fs_inode.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016-2017 Gregory Nutt. All rights
 *     reserved.
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

#include <unistd.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

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
 * removed.  In that case umount() hold the inode semaphore, but the block
 * driver may callback to unregister_blockdriver() after the un-mount,
 * requiring the seamphore again.
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

  (void)nxsem_init(&g_inode_sem.sem, 0, 1);
  g_inode_sem.holder = NO_HOLDER;
  g_inode_sem.count  = 0;

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

void inode_semtake(void)
{
  pid_t me;

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
      int ret;

      do
        {
          ret = nxsem_wait(&g_inode_sem.sem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);

      /* No we hold the semaphore */

      g_inode_sem.holder = me;
      g_inode_sem.count  = 1;
    }
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
