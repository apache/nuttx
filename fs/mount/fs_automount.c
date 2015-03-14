/****************************************************************************
 * fs/mount/fs_automount.c
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

#if defined(CONFIG_FS_AUTOMOUNTER_DEBUG) && !defined(CONFIG_DEBUG_FS)
#  define CONFIG_DEBUG_FS 1
#endif

#include <sys/mount.h>

#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/automount.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 *
 * CONFIG_FS_AUTOMOUNTER - Enables AUTOMOUNT support
 */

/* Pre-requisites */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

/* Return Values */

#define OK_EXIST   0
#define OK_NOENT   1

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure describes the state of the automounter */

struct automounter_state_s
{
  FAR const struct automount_lower_s *lower; /* Board level interfaces */
  struct work_s work;                        /* Work queue support */
  WDOG_ID wdog;                              /* Delay to retry un-mounts */
  bool mounted;                              /* True: Volume has been mounted */
  bool inserted;                             /* True: Media has been inserted */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  automount_findinode(FAR const char *path);
static void automount_mount(FAR struct automounter_state_s *priv);
static int  automount_unmount(FAR struct automounter_state_s *priv);
static void automount_timeout(int argc, uint32_t arg1, ...);
static void automount_worker(FAR void *arg);
static int  automount_interrupt(FAR const struct automount_lower_s *lower,
              FAR void *arg, bool inserted);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: automount_findinode
 *
 * Description:
 *   Find the mountpoint inode in the inode tree.
 *
 * Input Parameters:
 *   mntpath - Mountpoint path
 *
 * Returned Value:
 *   OK_EXIST if the inode exists
 *   OK_NOENT if the indoe does not exist
 *   Negated errno if some failure occurs
 *
 ****************************************************************************/

static int automount_findinode(FAR const char *path)
{
  FAR struct inode *node;
  FAR const char *srchpath;
  FAR const char *relpath;
  int ret;

  /* Make sure that we were given an absolute path */

  DEBUGASSERT(path && path[0] == '/');

  /* Get exclusive access to the in-memory inode tree. */

  inode_semtake();

  /* Find the inode */

  srchpath = path;
  node = inode_search(&srchpath, (FAR struct inode**)NULL,
                      (FAR struct inode**)NULL, &relpath);
  /* Did we find it? */

  if (!node)
    {
      /* No.. Not found */

      ret = OK_NOENT;
    }

  /* Yes.. is it a mount point? */

  else if (INODE_IS_MOUNTPT(node))
    {
      /* Yes.. we found a mountpoint at this path */

      ret = OK_EXIST;
    }
  else
    {
      /* No.. then somethin is in the way */

      ret = -ENOTDIR;
    }

  /* Relinquish our exclusive access to the inode try and return the result */

  inode_semgive();
  return ret;
}

/****************************************************************************
 * Name: automount_mount
 *
 * Description:
 *   Media has been inserted, mount the volume.
 *
 * Input Parameters:
 *   priv - A reference to out private state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void automount_mount(FAR struct automounter_state_s *priv)
{
  FAR const struct automount_lower_s *lower = priv->lower;
  int ret;

  fvdbg("Mounting %s\n", lower->mountpoint);

  /* Check if the something is already mounted at the mountpoint. */

  ret = automount_findinode(lower->mountpoint);
  switch (ret)
    {
    case OK_EXIST:
      /* REVISIT: What should we do in this case?  I think that this would
       * happen only if a previous unmount failed?  I suppose that we should
       * try to unmount again because the mount might be stale.
       */

      fdbg("WARNING: Mountpoint %s already exists\n", lower->mountpoint);
      ret = automount_unmount(priv);
      if (ret < 0)
        {
          /* We failed to unmount (again?).  Complain and abort. */

          fdbg("ERROR: automount_unmount failed: %d\n", ret);
          return;
        }

      /* We successfully unmounted the file system.  Fall through to
       * mount it again.
       */

    case OK_NOENT:
      /* If we get here, then the volume must not be mounted */

      DEBUGASSERT(!priv->mounted);

       /* Mount the file system */

      ret = mount(lower->blockdev, lower->mountpoint, lower->fstype,
                  0, NULL);
      if (ret < 0)
        {
          int errcode = get_errno();
          DEBUGASSERT(errcode > 0);

          fdbg("ERROR: Mount failed: %d\n", errcode);
          UNUSED(errcode);
          return;
        }

      /* Indicate that the volume is mounted */

      priv->mounted = true;
      break;
      
    default:
      fdbg("ERROR: automount_findinode failed: %d\n", ret);
      break;
    }
}

/****************************************************************************
 * Name: automount_unmount
 *
 * Description:
 *   Media has been removed, unmount the volume.
 *
 * Input Parameters:
 *   priv - A reference to out private state structure
 *
 * Returned Value:
 *   OK if the volume was successfully mounted.  A negated errno value
 *   otherwise.
 *
 ****************************************************************************/

static int automount_unmount(FAR struct automounter_state_s *priv)
{
  FAR const struct automount_lower_s *lower = priv->lower;
  int ret;

  fvdbg("Unmounting %s\n", lower->mountpoint);

  /* Check if the something is already mounted at the mountpoint. */

  ret = automount_findinode(lower->mountpoint);
  switch (ret)
    {
    case OK_EXIST:
      /* If we get here, then the volume must be mounted */

      DEBUGASSERT(priv->mounted);

      /* Un-mount the volume */

      ret = umount2(lower->mountpoint, MNT_FORCE);
      if (ret < 0)
        {
          int errcode = get_errno();
          DEBUGASSERT(errcode > 0);

          /* We expect the error to be EBUSY meaning that the volume could
           * not be unmounted because there are currently reference via open
           * files or directories.
           */

          if (errcode == EBUSY)
            {
              fvdbg("WARNING: Volume is busy, try again later\n");

              /* Start a timer to retry the umount2 after a delay */

              ret = wd_start(priv->wdog, lower->udelay, automount_timeout, 1,
                             (uint32_t)((uintptr_t)priv));
              if (ret < 0)
                {
                  errcode = get_errno();
                  DEBUGASSERT(errcode > 0);

                  fdbg("ERROR: wd_start failed: %d\n", errcode);
                  return -ret;
                }
            }

          /* Other errors are fatal */

          else
            {
              fvdbg("ERROR: umount2 failed: %d\n", errcode);
              return -errcode;
            }
        }

      /* Fall through */

    case OK_NOENT:
      /* The mountpoint is not present.  This is normal behavior in the
       * case where the user manually un-mounted the volume before removing
       * media.  Nice job, Mr. user.
       */

      priv->mounted = false;
      return OK;

    default:
      fdbg("ERROR: automount_findinode failed: %d\n", ret);
      return ret;
    }
}

/****************************************************************************
 * Name: automount_timeout
 *
 * Description:
 *   A previous unmount failed because the volume was busy... busy meaning
 *   the volume could not be unmounted because there are open references
 *   the files or directories in the volume.  When this failure occurred,
 *   the unmount logic setup a delay and this function is called as a result
 *   of that delay timeout.
 *
 *   This function will attempt the unmount again.
 *
 * Input Parameters:
 *   Standard wdog timeout parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void automount_timeout(int argc, uint32_t arg1, ...)
{
  FAR struct automounter_state_s *priv =
    (FAR struct automounter_state_s *)((uintptr_t)arg1);
  int ret;

  fllvdbg("Timeout!\n");
  DEBUGASSERT(argc == 1 && priv);

  /* Check the state of things.  This timeout at the interrupt level and
   * will cancel the timeout if there is any change in the insertion
   * state.  So we should still have the saved state as NOT inserted and
   * there should be no pending work.
   */

  fllvdbg("inserted=%d\n", priv->inserted);
  DEBUGASSERT(!priv->inserted && work_available(&priv->work));

  /* Queue work to occur immediately. */

  ret = work_queue(LPWORK, &priv->work, automount_worker, priv, 0);
  if (ret < 0)
    {
      /* NOTE: Currently, work_queue only returns success */

      fdbg("ERROR: Failed to schedule work: %d\n", ret);
    }
}

/****************************************************************************
 * Name: automount_worker
 *
 * Description:
 *   Performs auto-mount actions on the worker thread.
 *
 * Input Parameters:
 *   arg - Work argument set by work_queue()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void automount_worker(FAR void *arg)
{
  FAR struct automounter_state_s *priv = (FAR struct automounter_state_s *)arg;
  FAR const struct automount_lower_s *lower;

  DEBUGASSERT(priv && priv->lower);
  lower = priv->lower;

  /* Disable interrupts.  We are commit now and everything must remain
   * stable.
   */

  AUTOMOUNT_DISABLE(lower);

  /* Are we mounting or unmounting? */

  if (priv->inserted)
    {
      /* We are mounting */

      automount_mount(priv);
    }
  else
    {
      /* We are unmounting */

      (void)automount_unmount(priv);
    }

  /* Re-enable interrupts */

  AUTOMOUNT_ENABLE(lower);
}

/****************************************************************************
 * Name: automount_interrupt
 *
 * Description:
 *   Called (probably from the interrupt level) when a media change event
 *   has been detected.
 *
 * Input Parameters:
 *   lower - Persistent board configuration data
 *   arg - Data associated with the auto-mounter
 *   inserted - True: Media has been inserted. False: media has been removed
 *
 * Returned Value:
 *   OK is returned on success; a negated errno value is returned on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that there is no race condition with the
 *   timer expiry.
 *
 ****************************************************************************/

static int automount_interrupt(FAR const struct automount_lower_s *lower,
                               FAR void *arg, bool inserted)
{
  FAR struct automounter_state_s *priv = (FAR struct automounter_state_s *)arg;
  int ret;

  DEBUGASSERT(lower && priv && priv->lower == lower);

  fllvdbg("inserted=%d\n", inserted);

  /* Cancel any pending work.  We could get called multiple times if, for
   * example there is bounce in the detection mechanism.  Work is performed
   * the low priority work queue if it is available.
   */

  ret = work_cancel(LPWORK, &priv->work);
  if (ret < 0)
    {
      /* NOTE: Currently, work_cancel only returns success */

      fdbg("ERROR: Failed to cancel work: %d\n", ret);
    }

  /* Set the media insertion/removal state */

  priv->inserted = inserted;

  /* Queue work to occur after a delay.  The delays performs debouncing:
   * If the insertion/removal detection logic has "chatter", then we may
   * receive this interrupt numerous times.  Each time, the previous work
   * will be cancelled (above) and the new work will scheduled with the
   * delay.  So the final mount operation will not be performed until the
   * insertion state is stable for that delay.
   */

  ret = work_queue(LPWORK, &priv->work, automount_worker, priv,
                   priv->lower->ddelay);
  if (ret < 0)
    {
      /* NOTE: Currently, work_queue only returns success */

      fdbg("ERROR: Failed to schedule work: %d\n", ret);
    }
  else
    {
      /* Cancel any retry delays */

      (void)wd_cancel(priv->wdog);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: automount_initialize
 *
 * Description:
 *   Configure the auto mounter.
 *
 * Input Parameters:
 *   lower - Persistent board configuration data
 *
 * Returned Value:
 *   A void* handle.  The only use for this handle is with automount_uninitialize().
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR void *automount_initialize(FAR const struct automount_lower_s *lower)
{
  FAR struct automounter_state_s *priv;
  int ret;

  fvdbg("lower=%p\n", lower);
  DEBUGASSERT(lower);

  /* Allocate an auto-mounter state structure */

  priv = (FAR struct automounter_state_s *)
    kmm_zalloc(sizeof(struct automounter_state_s));

  if (!priv)
    {
      fdbg("ERROR: Failed to allocate state structure\n");
      return NULL;
    }

  /* Initialize the automounter state structure */

  priv->lower = lower;

  /* Get a timer to handle unmount retries */

  priv->wdog  = wd_create();
  if (!priv->wdog)
    {
      fdbg("ERROR: Failed to create a timer\n");
      automount_uninitialize(priv);
      return NULL;
    }

  /* Handle the initial state of the mount on the caller's thread */

  priv->inserted = AUTOMOUNT_INSERTED(lower);

  /* Set up the first action at a delay from the initialization time (to
   * allow time for any extended block driver initialization to complete.
   */

  ret = work_queue(LPWORK, &priv->work, automount_worker, priv,
                   priv->lower->ddelay);
  if (ret < 0)
    {
      /* NOTE: Currently, work_queue only returns success */

      fdbg("ERROR: Failed to schedule work: %d\n", ret);
    }

  /* Attach and enable automounter interrupts */

  ret = AUTOMOUNT_ATTACH(lower, automount_interrupt, priv);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to attach automount interrupt: %d\n", ret);
      automount_uninitialize(priv);
      return NULL;
    }

  AUTOMOUNT_ENABLE(lower);
  return priv;
}

/****************************************************************************
 * Name: automount_uninitialize
 *
 * Description:
 *   Stop the automounter and free resources that it used.  NOTE that the
 *   mount is left in its last state mounted/unmounted state.
 *
 * Input Parameters:
 *   handle - The value previously returned by automount_initialize();
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void automount_uninitialize(FAR void *handle)
{
  FAR struct automounter_state_s *priv = (FAR struct automounter_state_s *)handle;
  FAR const struct automount_lower_s *lower;

  DEBUGASSERT(priv && priv->lower);
  lower = priv->lower;

  /* Disable and detach interrupts */

  AUTOMOUNT_DISABLE(lower);
  (void)AUTOMOUNT_DETACH(lower);

  /* Release resources */

  (void)wd_delete(priv->wdog);

  /* And free the state structure */

  kmm_free(priv);
}
