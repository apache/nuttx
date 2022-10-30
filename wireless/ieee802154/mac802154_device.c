/****************************************************************************
 * wireless/ieee802154/mac802154_device.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/ieee802154_device.h>
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

struct mac802154dev_callback_s
{
  /* This holds the information visible to the MAC layer */

  struct mac802154_maccb_s mc_cb;             /* Interface understood by
                                               * the MAC layer
                                               */
  FAR struct mac802154_chardevice_s *mc_priv; /* Our priv data */
};

struct mac802154_chardevice_s
{
  MACHANDLE md_mac;                     /* Saved binding to the mac layer */
  struct mac802154dev_callback_s md_cb; /* Callback information */
  mutex_t md_lock;                      /* Exclusive device access */

  /* Hold a list of events */

  bool enableevents : 1;          /* Are events enabled? */
  bool geteventpending : 1;       /* Is there a get event using the semaphore? */
  sem_t geteventsem;              /* Signaling semaphore for waiting get event */
  sq_queue_t primitive_queue;     /* For holding primitives to pass along */

  /* The following is a singly linked list of open references to the
   * MAC device.
   */

  FAR struct mac802154dev_open_s *md_open;

  /* Hold a list of transactions as a "readahead" buffer */

  bool readpending;                     /* Is there a read using the semaphore? */
  sem_t readsem;                        /* Signaling semaphore for waiting read */
  sq_queue_t dataind_queue;

  /* MAC Service notification information */

  bool    md_notify_registered;
  pid_t   md_notify_pid;
  struct sigevent md_notify_event;
  struct sigwork_s md_notify_work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mac802154dev_notify(FAR struct mac802154_maccb_s *maccb,
                               FAR struct ieee802154_primitive_s *primitive);
static int mac802154dev_rxframe(FAR struct mac802154_chardevice_s *dev,
                                FAR struct ieee802154_data_ind_s *ind);

static int  mac802154dev_open(FAR struct file *filep);
static int  mac802154dev_close(FAR struct file *filep);
static ssize_t mac802154dev_read(FAR struct file *filep, FAR char *buffer,
                                 size_t len);
static ssize_t mac802154dev_write(FAR struct file *filep,
                                  FAR const char *buffer, size_t len);
static int  mac802154dev_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations mac802154dev_fops =
{
  mac802154dev_open,  /* open */
  mac802154dev_close, /* close */
  mac802154dev_read,  /* read */
  mac802154dev_write, /* write */
  NULL,               /* seek */
  mac802154dev_ioctl, /* ioctl */
  NULL                /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  ret = nxmutex_lock(&dev->md_lock);
  if (ret < 0)
    {
      wlerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Allocate a new open struct */

  opriv = (FAR struct mac802154dev_open_s *)
    kmm_zalloc(sizeof(struct mac802154dev_open_s));

  if (!opriv)
    {
      wlerr("ERROR: Failed to allocate new open struct\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Attach the open struct to the device */

  opriv->md_flink = dev->md_open;
  dev->md_open = opriv;

  /* Attach the open struct to the file structure */

  filep->f_priv = (FAR void *)opriv;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&dev->md_lock);
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
   * close the driver at the same time.  The rule:  don't do
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

  ret = nxmutex_lock(&dev->md_lock);
  if (ret < 0)
    {
      wlerr("ERROR: nxsem_wait failed: %d\n", ret);
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

  /* If there are now no open instances of the driver and a signal handler is
   * not registered, purge the list of events.
   */

  if (dev->md_open)
    {
      FAR struct ieee802154_primitive_s *primitive;

      primitive =
        (FAR struct ieee802154_primitive_s *)
        sq_remfirst(&dev->primitive_queue);

      while (primitive)
        {
          ieee802154_primitive_free(primitive);
          primitive =
            (FAR struct ieee802154_primitive_s *)
            sq_remfirst(&dev->primitive_queue);
        }
    }

  ret = OK;

errout_with_exclsem:
  nxmutex_unlock(&dev->md_lock);
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
  struct ieee802154_get_req_s req;
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

      ret = nxmutex_lock(&dev->md_lock);
      if (ret < 0)
        {
          wlerr("ERROR: nxsem_wait failed: %d\n", ret);
          return ret;
        }

      /* Try popping a data indication off the list */

      ind = (FAR struct ieee802154_data_ind_s *)
            sq_remfirst(&dev->dataind_queue);

      /* If the indication is not null, we can exit the loop and copy the
       * data
       */

      if (ind != NULL)
        {
          nxmutex_unlock(&dev->md_lock);
          break;
        }

      /* If this is a non-blocking call, or if there is another read
       * operation already pending, don't block. This driver returns EAGAIN
       * even when configured as non-blocking if another read operation is
       * already pending. This situation should be rare.
       * It will only occur when there are 2 calls  to read from separate
       * threads and there was no data in the rx list.
       */

      if ((filep->f_oflags & O_NONBLOCK) || dev->readpending)
        {
          nxmutex_unlock(&dev->md_lock);
          return -EAGAIN;
        }

      dev->readpending = true;
      nxmutex_unlock(&dev->md_lock);

      /* Wait to be signaled when a frame is added to the list */

      ret = nxsem_wait(&dev->readsem);
      if (ret < 0)
        {
          dev->readpending = false;
          return ret;
        }

      /* Let the loop wrap back around, we will then pop a indication and
       * this time, it should have a data indication
       */
    }

  /* Check if the MAC layer is in promiscuous mode. */

  req.attr = IEEE802154_ATTR_MAC_PROMISCUOUS_MODE;

  ret = mac802154_ioctl(dev->md_mac, MAC802154IOC_MLME_GET_REQUEST,
                        (unsigned long)&req);

  if (ret == 0 && req.attrval.mac.promisc_mode)
    {
      /* If it is, add the FCS back into the frame by increasing the length
       * by the PHY layer's FCS length.
       */

      req.attr = IEEE802154_ATTR_PHY_FCS_LEN;
      ret = mac802154_ioctl(dev->md_mac, MAC802154IOC_MLME_GET_REQUEST,
                            (unsigned long)&req);
      if (ret == OK)
        {
          rx->length = ind->frame->io_len + req.attrval.phy.fcslen;
          rx->offset = ind->frame->io_offset;
        }

      /* Copy the entire frame from the IOB to the user supplied struct */

      memcpy(&rx->payload[0], &ind->frame->io_data[0], rx->length);
    }
  else
    {
      rx->length = (ind->frame->io_len - ind->frame->io_offset);
      rx->offset = 0;

      /* Copy just the payload from the IOB to the user supplied struct */

      memcpy(&rx->payload[0], &ind->frame->io_data[ind->frame->io_offset],
             rx->length);
    }

  memcpy(&rx->meta, ind, sizeof(struct ieee802154_data_ind_s));

  /* Zero out the forward link and IOB reference */

  rx->meta.flink = NULL;
  rx->meta.frame = NULL;

  /* Free the IOB */

  iob_free(ind->frame);

  /* Deallocate the data indication */

  ieee802154_primitive_free((FAR struct ieee802154_primitive_s *)ind);

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
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  dev  = (FAR struct mac802154_chardevice_s *)inode->i_private;

  /* Check if the struct is the correct size */

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

  /* Pass the request to the MAC layer */

  ret = mac802154_req_data(dev->md_mac, &tx->meta, iob);
  if (ret < 0)
    {
      iob_free(iob);
      wlerr("ERROR: req_data failed %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: mac802154dev_ioctl
 *
 * Description:
 *   Control the MAC layer via IOCTL commands.
 *
 ****************************************************************************/

static int mac802154dev_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct mac802154_chardevice_s *dev;
  FAR union ieee802154_macarg_u *macarg =
    (FAR union ieee802154_macarg_u *)((uintptr_t)arg);
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_priv != NULL &&
              filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  dev = (FAR struct mac802154_chardevice_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&dev->md_lock);
  if (ret < 0)
    {
      wlerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  switch (cmd)
    {
      /* Command:     MAC802154IOC_NOTIFY_REGISTER
       * Description: Register to receive a signal whenever there is a
       *              event primitive sent from the MAC layer.
       * Argument:    The signal number to use.
       * Return:      Zero (OK) on success.  Minus one will be returned on
       *              failure with the errno value set appropriately.
       */

      case MAC802154IOC_NOTIFY_REGISTER:
        {
          /* Save the notification events */

          dev->md_notify_event      = macarg->event;
          dev->md_notify_pid        = getpid();
          dev->md_notify_registered = true;

          ret = OK;
        }
        break;

      case MAC802154IOC_GET_EVENT:
        {
          FAR struct ieee802154_primitive_s *primitive;

          while (1)
            {
              /* Try popping an event off the queue */

              primitive = (FAR struct ieee802154_primitive_s *)
                              sq_remfirst(&dev->primitive_queue);

              /* If there was an event to pop off, copy it into the user data
               * and free it from the MAC layer's memory.
               */

              if (primitive != NULL)
                {
                  memcpy(&macarg->primitive,
                         primitive,
                         sizeof(struct ieee802154_primitive_s));

                  /* Free the notification */

                  ieee802154_primitive_free(primitive);
                  ret = OK;
                  break;
                }

              /* If this is a non-blocking call, or if there is another
               * getevent operation already pending, don't block. This driver
               * returns EAGAIN even when configured as non-blocking if
               * another getevent operation is already pending This situation
               * should be rare.
               * It will only occur when there are 2 calls from separate
               * threads and there was no events in the queue.
               */

              if ((filep->f_oflags & O_NONBLOCK) || dev->geteventpending)
                {
                  ret = -EAGAIN;
                  break;
                }

              dev->geteventpending = true;
              nxmutex_unlock(&dev->md_lock);

              /* Wait to be signaled when an event is queued */

              ret = nxsem_wait(&dev->geteventsem);
              if (ret < 0)
                {
                  dev->geteventpending = false;
                  return ret;
                }

              /* Get exclusive access again, then loop back around and try
               * and pop an event off the queue
               */

                ret = nxmutex_lock(&dev->md_lock);
                if (ret < 0)
                  {
                    wlerr("ERROR: nxsem_wait failed: %d\n", ret);
                    return ret;
                  }
            }
        }
        break;

      case MAC802154IOC_ENABLE_EVENTS:
        {
          dev->enableevents = macarg->enable;
          ret = OK;
        }
        break;

      default:
        {
          /* Forward any unrecognized commands to the MAC layer */

          ret = mac802154_ioctl(dev->md_mac, cmd, arg);
        }
        break;
    }

  nxmutex_unlock(&dev->md_lock);
  return ret;
}

static int mac802154dev_notify(FAR struct mac802154_maccb_s *maccb,
                                FAR struct ieee802154_primitive_s *primitive)
{
  FAR struct mac802154dev_callback_s *cb =
    (FAR struct mac802154dev_callback_s *)maccb;
  FAR struct mac802154_chardevice_s *dev;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  dev = cb->mc_priv;

  /* Handle the special case for data indications or "incoming frames" */

  if (primitive->type == IEEE802154_PRIMITIVE_IND_DATA)
    {
      return mac802154dev_rxframe(dev, &primitive->u.dataind);
    }

  /* If there is a registered notification receiver, queue the event and
   * signal the receiver. Events should be popped from the queue from the
   * application at a reasonable rate in order for the MAC layer to be able
   * to allocate new notifications.
   */

  if (dev->enableevents &&
      (dev->md_open != NULL || dev->md_notify_registered))
    {
      /* Get exclusive access to the driver structure.  We don't care about
       * any signals so if we see one, just go back to trying to get access
       * again
       */

      while (nxmutex_lock(&dev->md_lock) != 0);

      sq_addlast((FAR sq_entry_t *)primitive, &dev->primitive_queue);

      /* Check if there is a read waiting for data */

      if (dev->geteventpending)
        {
          /* Wake the thread waiting for the data transmission */

          dev->geteventpending = false;
          nxsem_post(&dev->geteventsem);
        }

      if (dev->md_notify_registered)
        {
          dev->md_notify_event.sigev_value.sival_int = primitive->type;
          nxsig_notification(dev->md_notify_pid, &dev->md_notify_event,
                             SI_QUEUE, &dev->md_notify_work);
        }

      nxmutex_unlock(&dev->md_lock);
      return OK;
    }

  /* By returning a negative value, we let the MAC know that we don't want
   * the primitive and it will free it for us
   */

  return -1;
}

/****************************************************************************
 * Name: mac802154dev_rxframe
 *
 * Description:
 *   Handle received frames forward by the IEEE 802.15.4 MAC.
 *
 * Returned Value:
 *   any failure.  On success, the ind and its contained iob will be freed.
 *   The ind will be intact if this function returns a failure.
 *
 ****************************************************************************/

static int mac802154dev_rxframe(FAR struct mac802154_chardevice_s *dev,
                                FAR struct ieee802154_data_ind_s *ind)
{
  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again
   */

  while (nxmutex_lock(&dev->md_lock) != 0);

  /* Push the indication onto the list */

  sq_addlast((FAR sq_entry_t *)ind, &dev->dataind_queue);

  /* Check if there is a read waiting for data */

  if (dev->readpending)
    {
      /* Wake the thread waiting for the data transmission */

      dev->readpending = false;
      nxsem_post(&dev->readsem);
    }

  /* Release the driver */

  nxmutex_unlock(&dev->md_lock);
  return OK;
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
 *   mac - Pointer to the mac layer struct to be registered.
 *   minor - The device minor number.  The IEEE802.15.4 MAC character device
 *     will be registered as /dev/ieeeN where N is the minor number
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mac802154dev_register(MACHANDLE mac, int minor)
{
  FAR struct mac802154_chardevice_s *dev;
  FAR struct mac802154_maccb_s *maccb;
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
  nxmutex_init(&dev->md_lock); /* Allow the device to be opened once
                                    * before blocking */

  nxsem_init(&dev->readsem, 0, 0);
  dev->readpending = false;

  sq_init(&dev->dataind_queue);

  dev->geteventpending = false;
  nxsem_init(&dev->geteventsem, 0, 0);

  sq_init(&dev->primitive_queue);

  dev->enableevents = true;
  dev->md_notify_registered = false;

  /* Initialize the MAC callbacks */

  dev->md_cb.mc_priv  = dev;

  maccb           = &dev->md_cb.mc_cb;
  maccb->flink    = NULL;
  maccb->prio     = CONFIG_IEEE802154_MACDEV_RECVRPRIO;
  maccb->notify   = mac802154dev_notify;

  /* Bind the callback structure */

  ret = mac802154_bind(mac, maccb);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind the MAC callbacks: %d\n", ret);
      goto errout_with_priv;
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
  nxmutex_destroy(&dev->md_lock);
  kmm_free(dev);
  return ret;
}
