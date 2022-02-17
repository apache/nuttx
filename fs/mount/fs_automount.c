/****************************************************************************
 * fs/mount/fs_automount.c
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

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
#  include <stdio.h>

#  include <nuttx/signal.h>
#  include <nuttx/fs/fs.h>
#  include <nuttx/fs/ioctl.h>
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
  struct wdog_s wdog;                        /* Delay to retry un-mounts */
  bool mounted;                              /* True: Volume has been mounted */
  bool inserted;                             /* True: Media has been inserted */

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
  sem_t exclsem;                             /* Supports exclusive access to the device */
  bool registered;                           /* True: if driver has been registered */

  /* The following is a singly linked list of open references to the
   * automounter device.
   */

  FAR struct automounter_open_s *ao_open;
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */
};

/* This structure describes the state of one open automounter driver
 * instance
 */

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
struct automounter_open_s
{
  /* Supports a singly linked list */

  FAR struct automounter_open_s *ao_flink;

  /* Mount event notification information */

  pid_t ao_pid;
  struct automount_notify_s ao_notify;
  struct sigwork_s ao_work;
};
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
static void automount_notify(FAR struct automounter_state_s *priv);

static int  automount_open(FAR struct file *filep);
static int  automount_close(FAR struct file *filep);
static int  automount_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

static int  automount_findinode(FAR const char *path);
static void automount_mount(FAR struct automounter_state_s *priv);
static int  automount_unmount(FAR struct automounter_state_s *priv);
static void automount_timeout(wdparm_t arg);
static void automount_worker(FAR void *arg);
static int  automount_interrupt(FAR const struct automount_lower_s *lower,
                                FAR void *arg, bool inserted);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
static const struct file_operations automount_fops =
{
  automount_open,       /* open */
  automount_close,      /* close */
  NULL,                 /* read */
  NULL,                 /* write */
  NULL,                 /* seek */
  automount_ioctl,      /* ioctl */
  NULL                  /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                /* unlink */
#endif
};
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER

/****************************************************************************
 * Name: automount_notify
 ****************************************************************************/

static void automount_notify(FAR struct automounter_state_s *priv)
{
  FAR struct automounter_open_s *opriv;
  int ret;

  /* Get exclusive access to the driver structure */

  ret = nxsem_wait_uninterruptible(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait_uninterruptible failed: %d\n", ret);
      return;
    }

  /* Visit each opened reference to the device */

  for (opriv = priv->ao_open; opriv != NULL; opriv = opriv->ao_flink)
    {
      /* Have any signal events occurred? */

      if ((priv->mounted && opriv->ao_notify.an_mount) ||
          (!priv->mounted && opriv->ao_notify.an_umount))
        {
          /* Yes.. Signal the waiter */

          opriv->ao_notify.an_event.sigev_value.sival_int = priv->mounted;
          nxsig_notification(opriv->ao_pid, &opriv->ao_notify.an_event,
                             SI_QUEUE, &opriv->ao_work);
        }
    }

  nxsem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: automount_open
 ****************************************************************************/

static int automount_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct automounter_state_s *priv = inode->i_private;
  FAR struct automounter_open_s *opriv;
  int ret;

  /* Get exclusive access to the driver structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Allocate a new open structure */

  opriv = (FAR struct automounter_open_s *)kmm_zalloc(
      sizeof(struct automounter_open_s));
  if (opriv == NULL)
    {
      ierr("ERROR: Failed to allocate open structure\n");
      ret = -ENOMEM;
      goto errout_with_exclsem;
    }

  /* Attach the open structure to the device */

  opriv->ao_flink = priv->ao_open;
  priv->ao_open = opriv;

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;
  ret = OK;

errout_with_exclsem:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: automount_close
 ****************************************************************************/

static int automount_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct automounter_state_s *priv;
  FAR struct automounter_open_s *opriv;
  FAR struct automounter_open_s *curr;
  FAR struct automounter_open_s *prev;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct automounter_state_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = priv->ao_open;
       curr != NULL && curr != opriv;
       prev = curr, curr = curr->ao_flink);

  DEBUGASSERT(curr);
  if (curr == NULL)
    {
      ierr("ERROR: Failed to find open entry\n");
      ret = -ENOENT;
      goto errout_with_exclsem;
    }

  /* Remove the structure from the device */

  if (prev != NULL)
    {
      prev->ao_flink = opriv->ao_flink;
    }
  else
    {
      priv->ao_open = opriv->ao_flink;
    }

  /* Cancel any pending notification */

  nxsig_cancel_notification(&opriv->ao_work);

  /* And free the open structure */

  kmm_free(opriv);

  ret = OK;

errout_with_exclsem:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: automount_ioctl
 ****************************************************************************/

static int automount_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct automounter_state_s *priv;
  FAR struct automounter_open_s *opriv;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct automounter_state_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Handle the ioctl command */

  ret = -EINVAL;
  switch (cmd)
    {
      /* Command:     FIOC_NOTIFY
       * Description: Register to receive a signal whenever volume is mounted
       *              or unmounted by automounter.
       * Argument:    A read-only pointer to an instance of struct
       *              automount_notify_s
       * Return:      Zero (OK) on success.  Minus one will be returned on
       *              failure with the errno value set appropriately.
       */

      case FIOC_NOTIFY:
        {
          FAR struct automount_notify_s *notify =
            (FAR struct automount_notify_s *)((uintptr_t)arg);

          if (notify != NULL)
            {
              /* Save the notification events */

              opriv->ao_notify.an_mount  = notify->an_mount;
              opriv->ao_notify.an_umount = notify->an_umount;
              opriv->ao_notify.an_event  = notify->an_event;
              opriv->ao_pid              = getpid();
              ret = OK;
            }
        }
        break;

      default:
        ierr("ERROR: Unrecognized command: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

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
 *   OK_NOENT if the inode does not exist
 *   Negated errno if some failure occurs
 *
 ****************************************************************************/

static int automount_findinode(FAR const char *path)
{
  struct inode_search_s desc;
  int ret;

  /* Make sure that we were given a path */

  DEBUGASSERT(path != NULL);

  /* Get exclusive access to the in-memory inode tree. */

  ret = inode_semtake();
  if (ret < 0)
    {
      return ret;
    }

  /* Find the inode */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_search(&desc);

  /* Did we find it? */

  if (ret < 0)
    {
      /* No.. Not found */

      ret = OK_NOENT;
    }

  /* Yes.. is it a mount point? */

  else if (INODE_IS_MOUNTPT(desc.node))
    {
      /* Yes.. we found a mountpoint at this path */

      ret = OK_EXIST;
    }
  else
    {
      /* No.. then something is in the way */

      ret = -ENOTDIR;
    }

  /* Relinquish our exclusive access to the inode try and return the result */

  inode_semgive();
  RELEASE_SEARCH(&desc);
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

  finfo("Mounting %s\n", lower->mountpoint);

  /* Check if the something is already mounted at the mountpoint. */

  ret = automount_findinode(lower->mountpoint);
  switch (ret)
    {
    case OK_EXIST:

      /* REVISIT: What should we do in this case?  I think that this would
       * happen only if a previous unmount failed?  I suppose that we should
       * try to unmount again because the mount might be stale.
       */

      fwarn("WARNING: Mountpoint %s already exists\n", lower->mountpoint);
      ret = automount_unmount(priv);
      if (ret < 0)
        {
          /* We failed to unmount (again?).  Complain and abort. */

          ferr("ERROR: automount_unmount failed: %d\n", ret);
          return;
        }

      /* We successfully unmounted the file system.  Fall through to
       * mount it again.
       */

    case OK_NOENT:

      /* If we get here, then the volume must not be mounted */

      DEBUGASSERT(!priv->mounted);

       /* Mount the file system */

      ret = nx_mount(lower->blockdev, lower->mountpoint, lower->fstype,
                     0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Mount failed: %d\n", ret);
          return;
        }

      /* Indicate that the volume is mounted */

      priv->mounted = true;

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
      automount_notify(priv);
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

      break;

    default:
      ferr("ERROR: automount_findinode failed: %d\n", ret);
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

  finfo("Unmounting %s\n", lower->mountpoint);

  /* Check if the something is already mounted at the mountpoint. */

  ret = automount_findinode(lower->mountpoint);
  switch (ret)
    {
    case OK_EXIST:

      /* If we get here, then the volume must be mounted */

      DEBUGASSERT(priv->mounted);

      /* Un-mount the volume */

      ret = nx_umount2(lower->mountpoint, MNT_FORCE);
      if (ret < 0)
        {
          /* We expect the error to be EBUSY meaning that the volume could
           * not be unmounted because there are currently reference via open
           * files or directories.
           */

          if (ret == -EBUSY)
            {
              finfo("WARNING: Volume is busy, try again later\n");

              /* Start a timer to retry the umount2 after a delay */

              ret = wd_start(&priv->wdog, lower->udelay,
                             automount_timeout, (wdparm_t)priv);
              if (ret < 0)
                {
                  ferr("ERROR: wd_start failed: %d\n", ret);
                  return ret;
                }
            }

          /* Other errors are fatal */

          else
            {
              ferr("ERROR: umount2 failed: %d\n", ret);
              return ret;
            }
        }

      /* Fall through */

    case OK_NOENT:

      /* The mountpoint is not present.  This is normal behavior in the
       * case where the user manually un-mounted the volume before removing
       * media.  Nice job, Mr. user.
       */

      if (priv->mounted)
        {
          priv->mounted = false;

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
          automount_notify(priv);
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */
        }

      return OK;

    default:
      ferr("ERROR: automount_findinode failed: %d\n", ret);
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

static void automount_timeout(wdparm_t arg)
{
  FAR struct automounter_state_s *priv =
    (FAR struct automounter_state_s *)arg;
  int ret;

  finfo("Timeout!\n");
  DEBUGASSERT(priv);

  /* Check the state of things.  This timeout at the interrupt level and
   * will cancel the timeout if there is any change in the insertion
   * state.  So we should still have the saved state as NOT inserted and
   * there should be no pending work.
   */

  finfo("inserted=%d\n", priv->inserted);
  DEBUGASSERT(!priv->inserted && work_available(&priv->work));

  /* Queue work to occur immediately. */

  ret = work_queue(LPWORK, &priv->work, automount_worker, priv, 0);
  if (ret < 0)
    {
      /* NOTE: Currently, work_queue only returns success */

      ferr("ERROR: Failed to schedule work: %d\n", ret);
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
  FAR struct automounter_state_s *priv =
    (FAR struct automounter_state_s *)arg;
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

      automount_unmount(priv);
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
  FAR struct automounter_state_s *priv =
    (FAR struct automounter_state_s *)arg;
  int ret;

  DEBUGASSERT(lower && priv && priv->lower == lower);

  finfo("inserted=%d\n", inserted);

  /* Cancel any pending work.  We could get called multiple times if, for
   * example there is bounce in the detection mechanism.  Work is performed
   * the low priority work queue if it is available.
   *
   * NOTE:  The return values are ignored.  The error -ENOENT means that
   * there is no work to be canceled.  No other errors are expected.
   */

  work_cancel(LPWORK, &priv->work);

  /* Set the media insertion/removal state */

  priv->inserted = inserted;

  /* Queue work to occur after a delay.  The delays performs debouncing:
   * If the insertion/removal detection logic has "chatter", then we may
   * receive this interrupt numerous times.  Each time, the previous work
   * will be canceled (above) and the new work will scheduled with the
   * delay.  So the final mount operation will not be performed until the
   * insertion state is stable for that delay.
   */

  ret = work_queue(LPWORK, &priv->work, automount_worker, priv,
                   priv->lower->ddelay);
  if (ret < 0)
    {
      /* NOTE: Currently, work_queue only returns success */

      ferr("ERROR: Failed to schedule work: %d\n", ret);
    }
  else
    {
      /* Cancel any retry delays */

      wd_cancel(&priv->wdog);
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
 *   A void* handle.  The only use for this handle is with
 *   automount_uninitialize().  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR void *automount_initialize(FAR const struct automount_lower_s *lower)
{
  FAR struct automounter_state_s *priv;
  int ret;
#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
  char devpath[PATH_MAX];
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

  finfo("lower=%p\n", lower);
  DEBUGASSERT(lower);

  /* Allocate an auto-mounter state structure */

  priv = (FAR struct automounter_state_s *)
    kmm_zalloc(sizeof(struct automounter_state_s));

  if (priv == NULL)
    {
      ferr("ERROR: Failed to allocate state structure\n");
      return NULL;
    }

  /* Initialize the automounter state structure */

  priv->lower = lower;

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

      ferr("ERROR: Failed to schedule work: %d\n", ret);
    }

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER

  /* Initialize the new automount driver instance */

  nxsem_init(&priv->exclsem, 0, 1);

  /* Register driver */

  sprintf(devpath, CONFIG_FS_AUTOMOUNTER_VFS_PATH "%s", lower->mountpoint);

  ret = register_driver(devpath, &automount_fops, 0444, priv);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register automount driver: %d\n", ret);
      automount_uninitialize(priv);
      return NULL;
    }

  priv->registered = true;
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

  /* Attach and enable automounter interrupts */

  ret = AUTOMOUNT_ATTACH(lower, automount_interrupt, priv);
  if (ret < 0)
    {
      ferr("ERROR: Failed to attach automount interrupt: %d\n", ret);
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
  FAR struct automounter_state_s *priv =
    (FAR struct automounter_state_s *)handle;
  FAR const struct automount_lower_s *lower;

  DEBUGASSERT(priv && priv->lower);
  lower = priv->lower;

  /* Disable and detach interrupts */

  AUTOMOUNT_DISABLE(lower);
  AUTOMOUNT_DETACH(lower);

#ifdef CONFIG_FS_AUTOMOUNTER_DRIVER
  if (priv->registered)
    {
      char devpath[PATH_MAX];

      sprintf(devpath, CONFIG_FS_AUTOMOUNTER_VFS_PATH "%s",
              lower->mountpoint);

      unregister_driver(devpath);
    }

  nxsem_destroy(&priv->exclsem);
#endif /* CONFIG_FS_AUTOMOUNTER_DRIVER */

  /* Cancel the watchdog timer */

  wd_cancel(&priv->wdog);

  /* And free the state structure */

  kmm_free(priv);
}
