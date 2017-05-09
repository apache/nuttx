/****************************************************************************
 * wireless/ieee802154/mac802154_device.c
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/ieee802154_ioctl.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mac802154.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define DEVNAME_FMT    "/dev/ieee%d"
#define DEVNAME_FMTLEN (9 + 3 + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one open mac driver instance */

struct mac802154dev_open_s
{
  /* Supports a singly linked list */

  FAR struct mac802154dev_open_s *md_flink;

  /* The following will be true if we are closing */

  volatile bool md_closing;
};

struct mac802154dev_dwait_s
{
  uint8_t mw_handle;  /* The msdu handle identifying the frame */
  sem_t mw_sem;       /* The semaphore used to signal the completion */
  int mw_status;      /* The success/error of the transaction */

  /* Supports a singly linked list */

  FAR struct mac802154dev_dwait_s *mw_flink;
};

struct mac802154dev_callback_s
{
  /* This holds the information visible to the MAC layer */

  struct ieee802154_maccb_s mc_cb;     /* Interface understood by the MAC layer */
  FAR struct mac802154_chardevice_s *mc_priv;    /* Our priv data */
};

struct mac802154_chardevice_s
{
  MACHANDLE md_mac;                     /* Saved binding to the mac layer */
  struct mac802154dev_callback_s md_cb; /* Callback information */
  sem_t md_exclsem;                     /* Exclusive device access */

  bool readpending;                     /* Is there a read using the semaphore? */
  sem_t readsem;                        /* Signaling semaphore for waiting read */

  /* The following is a singly linked list of open references to the
   * MAC device.
   */

  FAR struct mac802154dev_open_s *md_open;
  FAR struct mac802154dev_dwait_s *md_dwait;

  /* Hold a list of transactions as a "readahead" buffer */

  FAR struct ieee802154_data_ind_s *dataind_head;
  FAR struct ieee802154_data_ind_s *dataind_tail;

#ifndef CONFIG_DISABLE_SIGNALS
  /* MCPS Service notification information */

  struct mac802154dev_notify_s md_mcps_notify;
  pid_t md_mcps_pid;

  /* MLME Service notification information */

  struct mac802154dev_notify_s md_mlme_notify;
  pid_t md_mlme_pid;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /* Semaphore helpers */

static inline int mac802154dev_takesem(sem_t *sem);
#define mac802154dev_givesem(s) sem_post(s);

static void mac802154dev_push_dataind(FAR struct mac802154_chardevice_s *dev,
                                      FAR struct ieee802154_data_ind_s *ind);
static FAR struct ieee802154_data_ind_s *
  mac802154dev_pop_dataind(FAR struct mac802154_chardevice_s *dev);


static void mac802154dev_mlme_notify(FAR const struct ieee802154_maccb_s *maccb,
                                     enum ieee802154_macnotify_e notif,
                                     FAR const union ieee802154_mlme_notify_u *arg);

static void mac802154dev_mcps_notify(FAR const struct ieee802154_maccb_s *maccb,
                                     enum ieee802154_macnotify_e notif,
                                     FAR const union ieee802154_mcps_notify_u *arg);

static int  mac802154dev_open(FAR struct file *filep);
static int  mac802154dev_close(FAR struct file *filep);
static ssize_t mac802154dev_read(FAR struct file *filep, FAR char *buffer,
              size_t len);
static ssize_t mac802154dev_write(FAR struct file *filep,
              FAR const char *buffer, size_t len);
static int  mac802154dev_ioctl(FAR struct file *filep, int cmd,
              unsigned long arg);

/* MAC callback helpers */

static void mac802154dev_conf_data(FAR struct mac802154_chardevice_s *dev,
                                   FAR const struct ieee802154_data_conf_s *conf);
static void mac802154dev_ind_data(FAR struct mac802154_chardevice_s *dev,
                                  FAR struct ieee802154_data_ind_s *ind);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations mac802154dev_fops =
{
  mac802154dev_open , /* open */
  mac802154dev_close, /* close */
  mac802154dev_read , /* read */
  mac802154dev_write, /* write */
  NULL,               /* seek */
  mac802154dev_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL              /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154dev_semtake
 *
 * Description:
 *   Acquire the semaphore used for access serialization.
 *
 ****************************************************************************/

static inline int mac802154dev_takesem(sem_t *sem)
{
  /* Take a count from the semaphore, possibly waiting */

  if (sem_wait(sem) < 0)
    {
      /* EINTR is the only error that we expect */

      int errcode = get_errno();
      DEBUGASSERT(errcode == EINTR);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: mac802154dev_push_dataind
 *
 * Description:
 *   Push a data indication onto the list to be processed
 *
 ****************************************************************************/

static void mac802154dev_push_dataind(FAR struct mac802154_chardevice_s *dev,
                                      FAR struct ieee802154_data_ind_s *ind)
{
  /* Ensure the forward link is NULL */

  ind->flink = NULL;

  /* If the tail is not empty, make the frame pointed to by the tail,
   * point to the new data indication */

  if (dev->dataind_tail != NULL)
    {
      dev->dataind_tail->flink = ind;
    }

  /* Point the tail at the new frame */

  dev->dataind_tail = ind;

  /* If the head is NULL, we need to point it at the data indication since there
   * is only one indication in the list at this point */

  if (dev->dataind_head == NULL)
    {
      dev->dataind_head = ind;
    }
}

/****************************************************************************
 * Name: mac802154dev_pop_dataind
 *
 * Description:
 *   Pop a data indication from the list
 *
 ****************************************************************************/

static FAR struct ieee802154_data_ind_s *
  mac802154dev_pop_dataind(FAR struct mac802154_chardevice_s *dev)
{
  FAR struct ieee802154_data_ind_s *ind;

  if (dev->dataind_head == NULL)
    {
      return NULL;
    }

  /* Get the data indication from the head of the list */

  ind = dev->dataind_head;
  ind->flink = NULL;

  /* Move the head pointer to the next data indication */

  dev->dataind_head = ind->flink;

  /* If the head is now NULL, the list is empty, so clear the tail too */

  if (dev->dataind_head == NULL)
    {
      dev->dataind_tail = NULL;
    }

  return ind;
}

/****************************************************************************
 * Name: mac802154dev_open
 *
 * Description:
 *   Open the 802.15.4 MAC character device.
 *
 ****************************************************************************/

static int mac802154dev_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mac802154_chardevice_s *dev;
  FAR struct mac802154dev_open_s *opriv;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the MAC driver data structure */

  ret = mac802154dev_takesem(&dev->md_exclsem);
  if (ret < 0)
    {
      wlerr("ERROR: mac802154dev_takesem failed: %d\n", ret);
      return ret;
    }

  /* Allocate a new open struct */

  opriv = (FAR struct mac802154dev_open_s *)
    kmm_zalloc(sizeof(struct mac802154dev_open_s));

  if (!opriv)
    {
      wlerr("ERROR: Failed to allocate new open struct\n");
      ret = -ENOMEM;
      goto errout_with_sem;
    }

  /* Attach the open struct to the device */

  opriv->md_flink = dev->md_open;
  dev->md_open = opriv;

  /* Attach the open struct to the file structure */

  filep->f_priv = (FAR void *)opriv;
  ret = OK;

errout_with_sem:
  mac802154dev_givesem(&dev->md_exclsem);
  return ret;
}

/****************************************************************************
 * Name: mac802154dev_close
 *
 * Description:
 *   Close the 802.15.4 MAC character device.
 *
 ****************************************************************************/

static int mac802154dev_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mac802154_chardevice_s *dev;
  FAR struct mac802154dev_open_s *opriv;
  FAR struct mac802154dev_open_s *curr;
  FAR struct mac802154dev_open_s *prev;
  irqstate_t flags;
  bool closing;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  dev = (FAR struct mac802154_chardevice_s *)inode->i_private;

  /* Handle an improbable race conditions with the following atomic test
   * and set.
   *
   * This is actually a pretty feeble attempt to handle this.  The
   * improbable race condition occurs if two different threads try to
   * close the joystick driver at the same time.  The rule:  don't do
   * that!  It is feeble because we do not really enforce stale pointer
   * detection anyway.
   */

  flags = enter_critical_section();
  closing = opriv->md_closing;
  opriv->md_closing = true;
  leave_critical_section(flags);

  if (closing)
    {
      /* Another thread is doing the close */

      return OK;
    }

  /* Get exclusive access to the driver structure */

  ret = mac802154dev_takesem(&dev->md_exclsem);
  if (ret < 0)
    {
      wlerr("ERROR: mac802154_takesem failed: %d\n", ret);
      return ret;
    }

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = dev->md_open;
       curr && curr != opriv;
       prev = curr, curr = curr->md_flink);

  DEBUGASSERT(curr);
  if (!curr)
    {
      wlerr("ERROR: Failed to find open entry\n");
      ret = -ENOENT;
      goto errout_with_exclsem;
    }

  /* Remove the structure from the device */

  if (prev)
    {
      prev->md_flink = opriv->md_flink;
    }
  else
    {
      dev->md_open = opriv->md_flink;
    }

  /* And free the open structure */

  kmm_free(opriv);
  ret = OK;

errout_with_exclsem:
  mac802154dev_givesem(&dev->md_exclsem);
  return ret;
}

/****************************************************************************
 * Name: mac802154dev_read
 *
 * Description:
 *   Return the last received packet.
 *
 ****************************************************************************/

static ssize_t mac802154dev_read(FAR struct file *filep, FAR char *buffer,
                                 size_t len)
{
  FAR struct inode *inode;
  FAR struct mac802154_chardevice_s *dev;
  FAR struct mac802154dev_rxframe_s *rx;
  FAR struct ieee802154_data_ind_s *ind;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  dev = (FAR struct mac802154_chardevice_s *)inode->i_private;

  /* Check to make sure the buffer is the right size for the struct */

  if (len != sizeof(struct mac802154dev_rxframe_s))
    {
      wlerr("ERROR: buffer isn't a mac802154dev_rxframe_s: %lu\n",
            (unsigned long)len);
      return -EINVAL;
    }

  DEBUGASSERT(buffer != NULL);
  rx = (FAR struct mac802154dev_rxframe_s *)buffer;

  while (1)
    {
      /* Get exclusive access to the driver structure */

      ret = mac802154dev_takesem(&dev->md_exclsem);
      if (ret < 0)
        {
          wlerr("ERROR: mac802154dev_takesem failed: %d\n", ret);
          return ret;
        }

      /* Try popping a data indication off the list */

      ind = mac802154dev_pop_dataind(dev);
      
      /* If the indication is not null, we can exit the loop and copy the data */

      if (ind != NULL)
        {
          mac802154dev_givesem(&dev->md_exclsem);
          break;
        }

      /* If this is a non-blocking call, or if there is another read operation 
       * already pending, don't block. This driver returns EAGAIN even when 
       * configured as non-blocking if another read operation is already pending
       * This situation should be rare. It will only occur when there are 2 calls
       * to read from separate threads and there was no data in the rx list.
       */

      if ((filep->f_oflags & O_NONBLOCK) || dev->readpending)
        {
          mac802154dev_givesem(&dev->md_exclsem);
          return -EAGAIN;
        }

      dev->readpending = true;
      mac802154dev_givesem(&dev->md_exclsem);

      /* Wait to be signaled when a frame is added to the list */
      
      if (sem_wait(&dev->readsem) < 0)
        {
          DEBUGASSERT(errno == EINTR);
          /* Need to get exclusive access again to change the pending bool */

          ret = mac802154dev_takesem(&dev->md_exclsem);
          dev->readpending = false;
          mac802154dev_givesem(&dev->md_exclsem);
          return -EINTR;
        }

      /* Let the loop wrap back around, we will then pop a indication and this
       * time, it should have a data indication
       */
    }
          
 rx->length = (ind->frame->io_len - ind->frame->io_offset);

 /* Copy the data from the IOB to the user supplied struct */

 memcpy(&rx->payload[0], &ind->frame->io_data[ind->frame->io_offset],
        rx->length); 

 memcpy(&rx->meta, ind, sizeof(struct ieee802154_data_ind_s));

 /* Zero out the forward link and IOB reference */
 
 rx->meta.flink = NULL;
 rx->meta.frame = NULL;

 /* Deallocate the data indication */

 ieee802154_ind_free(ind);

 return OK;
}

/****************************************************************************
 * Name: mac802154dev_write
 *
 * Description:
 *   Send a packet over the IEEE802.15.4 network.
 *
 ****************************************************************************/

static ssize_t mac802154dev_write(FAR struct file *filep,
                                  FAR const char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct mac802154_chardevice_s *dev;
  FAR struct mac802154dev_txframe_s *tx;
  FAR struct iob_s *iob;
  struct mac802154dev_dwait_s dwait;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  dev  = (FAR struct mac802154_chardevice_s *)inode->i_private;

  /* Check if the struct is write */

  if (len != sizeof(struct mac802154dev_txframe_s))
    {
      wlerr("ERROR: buffer isn't a mac802154dev_txframe_s: %lu\n",
            (unsigned long)len);

      return -EINVAL;
    }

  DEBUGASSERT(buffer != NULL);
  tx = (FAR struct mac802154dev_txframe_s *)buffer;

  /* Allocate an IOB to put the frame in */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  /* Get the MAC header length */

  ret = mac802154_get_mhrlen(dev->md_mac, &tx->meta);
  if (ret < 0)
    {
      wlerr("ERROR: TX meta-data is invalid");
      return ret;
    }

  iob->io_offset = ret;
  iob->io_len = iob->io_offset;

  memcpy(&iob->io_data[iob->io_offset], tx->payload, tx->length);

  iob->io_len += tx->length;

  /* If this is a blocking operation, we need to setup a wait struct so we
   * can unblock when the packet transmission has finished. If this is a
   * non-blocking write, we pass off the data and then move along. Technically
   * we stil have to wait for the transaction to get put into the buffer, but we
   * won't wait for the transaction to actually finish. */

  if (!(filep->f_oflags & O_NONBLOCK))
    {
      /* Get exclusive access to the driver structure */

      ret = mac802154dev_takesem(&dev->md_exclsem);
      if (ret < 0)
        {
          wlerr("ERROR: mac802154dev_takesem failed: %d\n", ret);
          return ret;
        }

      /* Setup the wait struct */

      dwait.mw_handle = tx->meta.msdu_handle;

      /* Link the wait struct */

      dwait.mw_flink = dev->md_dwait;
      dev->md_dwait = &dwait;

      sem_init(&dwait.mw_sem, 0, 0);
      sem_setprotocol(&dwait.mw_sem, SEM_PRIO_NONE);

      mac802154dev_givesem(&dev->md_exclsem);
  }

  /* Pass the request to the MAC layer */

  ret = mac802154_req_data(dev->md_mac, &tx->meta, iob);

  if (ret < 0)
    {
      wlerr("ERROR: req_data failed %d\n", ret);
      return ret;
    }

  if (!(filep->f_oflags & O_NONBLOCK))
    {
      /* Wait for the DATA.confirm callback to be called for our handle */

      if (sem_wait(&dwait.mw_sem) < 0)
        {
          /* This should only happen if the wait was canceled by an signal */

          sem_destroy(&dwait.mw_sem);
          DEBUGASSERT(errno == EINTR);
          return -EINTR;
        }

      /* The unlinking of the wait struct happens inside the callback. This
       * is more efficient since it will already have to find the struct in
       * the list in order to perform the sem_post.
       */

      sem_destroy(&dwait.mw_sem);
      return dwait.mw_status;
    }

  return OK;
}

/****************************************************************************
 * Name: mac802154dev_ioctl
 *
 * Description:
 *   Control the MAC layer via MLME IOCTL commands.
 *
 ****************************************************************************/

static int mac802154dev_ioctl(FAR struct file *filep, int cmd,
                                   unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct mac802154_chardevice_s *dev;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_priv != NULL &&
              filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  dev = (FAR struct mac802154_chardevice_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = mac802154dev_takesem(&dev->md_exclsem);
  if (ret < 0)
    {
      wlerr("ERROR: mac802154dev_takesem failed: %d\n", ret);
      return ret;
    }

  /* Handle the ioctl command */

  switch (cmd)
    {
#ifndef CONFIG_DISABLE_SIGNALS
      /* Command:     MAC802154IOC_MLME_REGISTER, MAC802154IOC_MCPS_REGISTER
       * Description: Register to receive a signal whenever there is a
       *              event primitive sent from the MAC layer.
       * Argument:    A read-only pointer to an instance of struct
       *              mac802154dev_notify_s
       * Return:      Zero (OK) on success.  Minus one will be returned on
       *              failure with the errno value set appropriately.
       */

      case MAC802154IOC_MLME_REGISTER:
        {
          FAR struct mac802154dev_notify_s *notify =
            (FAR struct mac802154dev_notify_s *)((uintptr_t)arg);

          if (notify)
            {
              /* Save the notification events */

              dev->md_mlme_notify.mn_signo      = notify->mn_signo;
              dev->md_mlme_pid                  = getpid();

              return OK;
            }
        }
        break;

      case MAC802154IOC_MCPS_REGISTER:
        {
          FAR struct mac802154dev_notify_s *notify =
            (FAR struct mac802154dev_notify_s *)((uintptr_t)arg);

          if (notify)
            {
              /* Save the notification events */

              dev->md_mcps_notify.mn_signo      = notify->mn_signo;
              dev->md_mcps_pid                  = getpid();

              return OK;
            }
        }
        break;
#endif
      default:
        {
          /* Forward any unrecognized commands to the MAC layer */
          
          mac802154_ioctl(dev->md_mac, cmd, arg);
        }
        
        break;
    }

  mac802154dev_givesem(&dev->md_exclsem);
  return ret;
}

static void mac802154dev_mlme_notify(FAR const struct ieee802154_maccb_s *maccb,
                                     enum ieee802154_macnotify_e notif,
                                     FAR const union ieee802154_mlme_notify_u *arg)
{
  FAR struct mac802154dev_callback_s *cb =
    (FAR struct mac802154dev_callback_s *)maccb;
  FAR struct mac802154_chardevice_s *dev;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  dev = cb->mc_priv;

  switch (notif)
    {
#warning Handle MLME notifications
      default:
        break;
    }
}

static void mac802154dev_mcps_notify(FAR const struct ieee802154_maccb_s *maccb,
                                     enum ieee802154_macnotify_e notif,
                                     FAR const union ieee802154_mcps_notify_u *arg)
{
  FAR struct mac802154dev_callback_s *cb =
    (FAR struct mac802154dev_callback_s *)maccb;
  FAR struct mac802154_chardevice_s *dev;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  dev = cb->mc_priv;

  switch (notif)
    {
      case IEEE802154_NOTIFY_CONF_DATA:
        {
          mac802154dev_conf_data(dev, &arg->dataconf);
        }
        break;
      case IEEE802154_NOTIFY_IND_DATA:
        {
          mac802154dev_ind_data(dev, arg->dataind);
        }
        break;
      default:
        break;
    }
}

static void mac802154dev_conf_data(FAR struct mac802154_chardevice_s *dev,
                                   FAR const struct ieee802154_data_conf_s *conf)
{
  FAR struct mac802154dev_dwait_s *curr;
  FAR struct mac802154dev_dwait_s *prev;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again */

  while (mac802154dev_takesem(&dev->md_exclsem) != 0);

  /* Search to see if there is a dwait pending for this transaction */

  for (prev = NULL, curr = dev->md_dwait;
       curr && curr->mw_handle != conf->handle;
       prev = curr, curr = curr->mw_flink);

  /* If a dwait is found */

  if (curr)
   {
      /* Unlink the structure from the list.  The struct should be allocated on
       * the calling write's stack, so we don't need to worry about deallocating
       * here */

      if (prev)
        {
          prev->mw_flink = curr->mw_flink;
        }
      else
        {
          dev->md_dwait = curr->mw_flink;
        }

      /* Copy the transmission status into the dwait struct */

      curr->mw_status = conf->status;

      /* Wake the thread waiting for the data transmission */

      sem_post(&curr->mw_sem);

      /* Release the driver */

      mac802154dev_givesem(&dev->md_exclsem);
    }

#ifndef CONFIG_DISABLE_SIGNALS
  /* Send a signal to the registered application */

#ifdef CONFIG_CAN_PASS_STRUCTS
  /* Copy the status as the signal data to be optionally used by app */

  union sigval value;
  value.sival_int = (int)conf->status;
  (void)sigqueue(dev->md_mcps_pid, dev->md_mcps_notify.mn_signo, value);
#else
  (void)sigqueue(dev->md_mcps_pid, dev->md_mcps_notify.mn_signo,
                 value.sival_ptr);
#endif
#endif
}

static void mac802154dev_ind_data(FAR struct mac802154_chardevice_s *dev,
                                  FAR struct ieee802154_data_ind_s *ind)
{
  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again */

  while (mac802154dev_takesem(&dev->md_exclsem) != 0);

  /* Push the indication onto the list */

  mac802154dev_push_dataind(dev, ind);

  /* Check if there is a read waiting for data */

  if (dev->readpending)
    {
      /* Wake the thread waiting for the data transmission */

      dev->readpending = false;
      sem_post(&dev->readsem);
    }
 
  /* Release the driver */

  mac802154dev_givesem(&dev->md_exclsem);

#ifndef CONFIG_DISABLE_SIGNALS
  /* Send a signal to the registered application */

#ifdef CONFIG_CAN_PASS_STRUCTS
  /* Copy the status as the signal data to be optionally used by app */

  union sigval value;
  value.sival_int = IEEE802154_STATUS_SUCCESS;
  (void)sigqueue(dev->md_mcps_pid, dev->md_mcps_notify.mn_signo, value);
#else
  (void)sigqueue(dev->md_mcps_pid, dev->md_mcps_notify.mn_signo,
                 value.sival_ptr);
#endif
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154dev_register
 *
 * Description:
 *   Register a character driver to access the IEEE 802.15.4 MAC layer from
 *   user-space
 *
 * Input Parameters:
 *   mac - Pointer to the mac layer struct to be registerd.
 *   minor - The device minor number.  The IEEE802.15.4 MAC character device
 *     will be registered as /dev/ieeeN where N is the minor number
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mac802154dev_register(MACHANDLE mac, int minor)
{
  FAR struct mac802154_chardevice_s *dev;
  FAR struct ieee802154_maccb_s *maccb;
  char devname[DEVNAME_FMTLEN];
  int ret;

  dev = kmm_zalloc(sizeof(struct mac802154_chardevice_s));
  if (!dev)
    {
      wlerr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Initialize the new mac driver instance */

  dev->md_mac = mac;
  sem_init(&dev->md_exclsem, 0, 1); /* Allow the device to be opened once
                                     * before blocking */
                                     
  sem_init(&dev->readsem, 0, 0);
  sem_setprotocol(&dev->readsem, SEM_PRIO_NONE);
  dev->readpending = false;

  /* Initialize the MAC callbacks */

  dev->md_cb.mc_priv  = dev;

  maccb               = &dev->md_cb.mc_cb;
  maccb->mlme_notify  = mac802154dev_mlme_notify;
  maccb->mcps_notify  = mac802154dev_mcps_notify;

  /* Bind the callback structure */

  ret = mac802154_bind(mac, maccb);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind the MAC callbacks: %d\n", ret);

      /* Free memory and return the error */
      kmm_free(dev);
      return ret;
    }

  /* Create the character device name */

  snprintf(devname, DEVNAME_FMTLEN, DEVNAME_FMT, minor);

  /* Register the mac character driver */

  ret = register_driver(devname, &mac802154dev_fops, 0666, dev);
  if (ret < 0)
    {
      wlerr("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_priv;
    }

  return OK;

errout_with_priv:
  sem_destroy(&dev->md_exclsem);
  kmm_free(dev);
  return ret;
}
