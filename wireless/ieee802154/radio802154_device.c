/****************************************************************************
 * wireless/ieee802154/radio802154_device.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2014-2015 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "radio802154_ioctl.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct radio802154_devwrapper_s
{
  FAR struct ieee802154_radio_s *child;
  sem_t devsem;                         /* Device access serialization semaphore */
  bool opened;                          /* This device can only be opened once */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void radio802154dev_semtake(FAR struct radio802154_devwrapper_s *dev);
static int  radio802154dev_open(FAR struct file *filep);
static int  radio802154dev_close(FAR struct file *filep);
static ssize_t radio802154dev_read(FAR struct file *filep, FAR char *buffer,
              size_t len);
static ssize_t radio802154dev_write(FAR struct file *filep,
              FAR const char *buffer, size_t len);
static int  radio802154dev_ioctl(FAR struct file *filep, int cmd,
              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations radio802154dev_fops =
{
  radio802154dev_open , /* open */
  radio802154dev_close, /* close */
  radio802154dev_read , /* read */
  radio802154dev_write, /* write */
  NULL,                 /* seek */
  radio802154dev_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL               /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: radio802154dev_semtake
 *
 * Description:
 *   Acquire the semaphore used for access serialization.
 *
 ****************************************************************************/

static void radio802154dev_semtake(FAR struct radio802154_devwrapper_s *dev)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dev->devsem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: radio802154dev_semgive
 *
 * Description:
 *   Release the semaphore used for access serialization.
 *
 ****************************************************************************/

static inline void radio802154dev_semgive(FAR struct radio802154_devwrapper_s *dev)
{
  sem_post(&dev->devsem);
}

/****************************************************************************
 * Name: radio802154dev_open
 *
 * Description:
 *   Open the 802.15.4 radio character device.
 *
 ****************************************************************************/

static int radio802154dev_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct radio802154_devwrapper_s *dev;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the driver data structures */

  radio802154dev_semtake(dev);

  if (dev->opened)
    {
      /* Already opened */

      return -EMFILE;
    }
  else
    {
      /* Enable interrupts (only rx for now)*/

      //mrf24j40_setreg(dev->spi, MRF24J40_INTCON, ~(MRF24J40_INTCON_RXIE) );
      //dev->lower->enable(dev->lower, TRUE);

      dev->opened = true;
    }

  radio802154dev_semgive(dev);
  return OK;
}

/****************************************************************************
 * Name: radio802154dev_close
 *
 * Description:
 *   Close the 802.15.4 radio character device.
 *
 ****************************************************************************/

static int radio802154dev_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct radio802154_devwrapper_s *dev;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the driver data structures */

  radio802154dev_semtake(dev);

  if (!dev->opened)
    {
      /* Driver has not been opened */

      ret = -EIO;
    }
  else
    {
      /* Disable interrupts */

      //mrf24j40_setreg(dev->spi, MRF24J40_INTCON, 0xFF );
      //dev->lower->enable(dev->lower, FALSE);

      dev->opened = false;
    }

  radio802154dev_semgive(dev);
  return ret;
}

/****************************************************************************
 * Name: radio802154dev_read
 *
 * Description:
 *   Return the last received packet.
 *   TODO: Return a packet from the receive queue. The buffer must be a pointer to a
 *   struct ieee802154_packet_s structure, with a correct length.
 *
 ****************************************************************************/

static ssize_t radio802154dev_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct radio802154_devwrapper_s *dev;
  FAR struct ieee802154_packet_s *buf;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL && buffer != NULL);
  buf = (FAR struct ieee802154_packet_s*)buffer;

  /* Get exclusive access to the driver data structures */

  if (len < sizeof(struct ieee802154_packet_s))
    {
      ret = -EINVAL;
      goto done;
    }

  ret = dev->child->ops->rxenable(dev->child, 1, buf);
#if 0
  if (ret < 0)
    {
      goto done;
    }
#endif
  
  /* if no packet is received, this will produce -EAGAIN
   * The user is responsible for sleeping until sth arrives
   */

#if 0
  ret = sem_trywait(&dev->child->rxsem);
#else
  ret = sem_wait(&dev->child->rxsem);
#endif
  if (ret < 0)
    {
      ret = -errno;
      goto done;
    }

  /* Disable read until we have process the current read */

  dev->child->ops->rxenable(dev->child, 0, NULL);
  ret = buf->len;

done:
  return ret;
}

/****************************************************************************
 * Name: radio802154dev_write
 *
 * Description:
 *   Send a packet immediately.
 *   TODO: Put a packet in the send queue. The packet will be sent as soon
 *   as possible.  The buffer must point to a struct radio802154_packet_s
 *   with the correct length.
 *
 ****************************************************************************/

static ssize_t radio802154dev_write(FAR struct file *filep,
                                   FAR const char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct radio802154_devwrapper_s *dev;
  FAR struct ieee802154_packet_s *packet;
  FAR struct timespec timeout;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the driver data structures */

  /* TODO: Make this an option or have a new method of doing timeout from
   * ioctrl.
   */

  timeout.tv_sec = 1;
  timeout.tv_nsec = 0;

  /* Sanity checks */

  if (len < sizeof(struct ieee802154_packet_s))
    {
      ret = -EINVAL;
      //goto done;

      /* TODO Double check I like having assert here.  It is a bigger problem
       * if buffers are to small.
       */

      DEBUGASSERT(0);
    }

  packet = (FAR struct ieee802154_packet_s*) buffer;
  if (packet->len > 125) /* Max len 125, 2 FCS bytes auto added by mrf */
    {
      ret = -EPERM;
      //goto done;
      DEBUGASSERT(0);
    }

  /* Copy packet to normal device TX fifo.
   * Beacons and GTS transmission will be handled via IOCTLs
   */

  ret = dev->child->ops->transmit(dev->child, packet);
  if (ret != packet->len)
    {
      ret = -EPERM;
      goto done;
    }

  if (sem_timedwait(&dev->child->txsem, &timeout))
    {
      wlerr("Radio Device timedout on Tx\n");
    }

done:

  /* Okay, tx interrupt received. check transmission status to decide success. */

  return ret;
}

/****************************************************************************
 * Name: radio802154dev_ioctl
 *
 * Description:
 *   Control the MRF24J40 device. This is where the real operations happen.
 *
 ****************************************************************************/

static int radio802154dev_ioctl(FAR struct file *filep, int cmd,
                                   unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct radio802154_devwrapper_s *dev;
  FAR struct ieee802154_radio_s *child;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->child != NULL);
  child = dev->child;

  /* Get exclusive access to the driver data structures */

  switch (cmd)
    {
      case PHY802154IOC_SET_CHAN:
        ret = radio802154_setchannel(child, (uint8_t)arg);
        break;

      case PHY802154IOC_GET_CHAN:
        ret = radio802154_getchannel(child, (FAR uint8_t*)arg);
        break;

      case PHY802154IOC_SET_PANID:
        ret = radio802154_setpanid(child, (uint16_t)arg);
        break;

      case PHY802154IOC_GET_PANID:
        ret = radio802154_getpanid(child, (FAR uint16_t*)arg);
        break;

      case PHY802154IOC_SET_SADDR:
        ret = radio802154_setsaddr(child, (uint16_t)arg);
        break;

      case PHY802154IOC_GET_SADDR:
        ret = radio802154_getsaddr(child, (FAR uint16_t*)arg);
        break;

      case PHY802154IOC_SET_EADDR:
        ret = radio802154_seteaddr(child, (FAR uint8_t*)arg);
        break;

      case PHY802154IOC_GET_EADDR:
        ret = radio802154_geteaddr(child, (FAR uint8_t*)arg);
        break;

      case PHY802154IOC_SET_PROMISC:
        ret = radio802154_setpromisc(child, (bool)arg);
        break;

      case PHY802154IOC_GET_PROMISC:
        ret = radio802154_getpromisc(child, (FAR bool*)arg);
        break;

      case PHY802154IOC_SET_DEVMODE:
        ret = radio802154_setdevmode(child, (uint8_t)arg);
        break;

      case PHY802154IOC_GET_DEVMODE:
        ret = radio802154_getdevmode(child, (FAR uint8_t*)arg);
        break;

      case PHY802154IOC_SET_TXPWR:
        ret = radio802154_settxpower(child, (int32_t)arg);
        break;

      case PHY802154IOC_GET_TXPWR:
        ret = radio802154_gettxpower(child, (FAR int32_t*)arg);
        break;

      case PHY802154IOC_SET_CCA:
        ret = radio802154_setcca(child, (FAR struct ieee802154_cca_s*)arg);
        break;

      case PHY802154IOC_GET_CCA:
        ret = radio802154_getcca(child, (FAR struct ieee802154_cca_s*)arg);
        break;

      case PHY802154IOC_ENERGYDETECT:
        ret = radio802154_energydetect(child, (FAR uint8_t*)arg);
        break;

      default:
        ret = child->ops->ioctl(child, cmd,arg);
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: radio802154dev_register
 *
 * Description:
 *   Register a character driver to access the IEEE 802.15.4 radio from
 *   user-space
 *
 * Input Parameters:
 *   radio - Pointer to the radio struct to be registerd.
 *   devname - The name of the IEEE 802.15.4 radio to be registered.
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int radio802154dev_register(FAR struct ieee802154_radio_s *radio,
                            FAR char *devname)
{
  FAR struct radio802154_devwrapper_s *dev;

  dev = kmm_zalloc(sizeof(struct radio802154_devwrapper_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->child = radio;

  sem_init(&dev->devsem, 0, 1); /* Allow the device to be opened once before blocking */

  return register_driver(devname, &radio802154dev_fops, 0666, dev);
}
