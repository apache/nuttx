/****************************************************************************
 * drivers/ieee802154/ieee802154_device.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2014-2015 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/ieee802154/ieee802154_dev.h>

struct ieee802154_devwrapper_s
{
  FAR struct ieee802154_radio_s     *child;
  sem_t                             devsem;      /* Device access serialization semaphore */
  int                               opened;   /* this device can only be opened once */
};

/* when rx interrupt is complete, it calls sem_post(&dev->rxsem); */
/* when tx interrupt is complete, it calls sem_post(&dev->txsem); */

static void    ieee802154dev_semtake(FAR struct ieee802154_devwrapper_s *dev);
static int     ieee802154dev_open   (FAR struct file *filep);
static int     ieee802154dev_close  (FAR struct file *filep);
static ssize_t ieee802154dev_read   (FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t ieee802154dev_write  (FAR struct file *filep, FAR const char *buffer, size_t len);
static int     ieee802154dev_ioctl  (FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations ieee802154dev_fops =
{
  ieee802154dev_open , /* open */
  ieee802154dev_close, /* close */
  ieee802154dev_read , /* read */
  ieee802154dev_write, /* write */
  0                  , /* seek */
  ieee802154dev_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0                  /* poll */
#endif
};

/****************************************************************************
 * Name: ieee802154dev_semtake
 *
 * Description:
 *   Acquire the semaphore used for access serialization.
 *
 ****************************************************************************/

static void ieee802154dev_semtake(FAR struct ieee802154_devwrapper_s *dev)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dev->devsem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */
      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: ieee802154dev_semgive
 *
 * Description:
 *   Release the semaphore used for access serialization.
 *
 ****************************************************************************/

static inline void ieee802154dev_semgive(FAR struct ieee802154_devwrapper_s *dev)
{
  sem_post(&dev->devsem);
}

/****************************************************************************
 * Name: ieee802154dev_open
 *
 * Description:
 *   Open the MRF24J40 device.
 *
 ****************************************************************************/

static int ieee802154dev_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ieee802154_devwrapper_s *dev = inode->i_private;

  ieee802154dev_semtake(dev);

  if (dev->opened)
    {
      return -EMFILE;
    }
  else
    {

      /* Enable interrupts (only rx for now)*/

      //mrf24j40_setreg(dev->spi, MRF24J40_INTCON, ~(MRF24J40_INTCON_RXIE) );
      //dev->lower->enable(dev->lower, TRUE);

      dev->opened = TRUE;
    }

  ieee802154dev_semgive(dev);
  return OK;
}

/****************************************************************************
 * Name: ieee802154dev_close
 *
 * Description:
 *   Close the MRF24J40 device.
 *
 ****************************************************************************/

static int ieee802154dev_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ieee802154_devwrapper_s *dev = inode->i_private;
  int ret = OK;

  ieee802154dev_semtake(dev);

  if(!dev->opened)
    {
    ret = -EIO;
    }
  else
    {
      /* Disable interrupts */

      //mrf24j40_setreg(dev->spi, MRF24J40_INTCON, 0xFF );
      //dev->lower->enable(dev->lower, FALSE);

      dev->opened = FALSE;
    }

  ieee802154dev_semgive(dev);
  return ret;
}

/****************************************************************************
 * Name: ieee802154dev_read
 *
 * Description:
 *   Return the last received packet.
 *   TODO: Return a packet from the receive queue. The buffer must be a pointer to a
 *   struct ieee802154_packet_s structure, with a correct length.
 *
 ****************************************************************************/

static ssize_t ieee802154dev_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ieee802154_devwrapper_s *dev = inode->i_private;
  int ret = OK;

  FAR struct ieee802154_packet_s *buf = (FAR struct ieee802154_packet_s*)buffer;

  if (len<sizeof(struct ieee802154_packet_s))
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

  /* disable read until we have process the current read */
  dev->child->ops->rxenable(dev->child, 0, NULL);

  ret = buf->len;

done:

  return ret;
}

/****************************************************************************
 * Name: ieee802154dev_write
 *
 * Description:
 *   Send a packet immediately.
 *   TODO: Put a packet in the send queue. The packet will be sent as soon as possible.
 *   The buffer must point to a struct ieee802154_packet_s with the correct length.
 *
 ****************************************************************************/

static ssize_t ieee802154dev_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  FAR struct inode                     *inode = filep->f_inode;
  FAR struct ieee802154_devwrapper_s   *dev   = inode->i_private;
  FAR struct ieee802154_packet_s       *packet;
  int      ret = OK;

  /* TODO: Make this an option or have a new method of doing timeout from ioctrl */
  FAR struct timespec timeout;

  timeout.tv_sec = 1;
  timeout.tv_nsec = 0;

  /* sanity checks */

  if (len<sizeof(struct ieee802154_packet_s))
    {
      ret = -EINVAL;
  //    goto done;
/* TODO Double check I like having assert here.  It is a bigger problem if buffers are to small */
     DEBUGASSERT(0);
    }

  packet = (FAR struct ieee802154_packet_s*) buffer;
  if (packet->len > 125) /* Max len 125, 2 FCS bytes auto added by mrf*/
    {
      ret = -EPERM;
     // goto done;
      DEBUGASSERT(0);
    }

  /* Copy packet to normal device TX fifo.
   * Beacons and GTS transmission will be handled via IOCTLs
   */
  ret = dev->child->ops->transmit(dev->child, packet);

  if(ret != packet->len)
    {
      ret = -EPERM;
      goto done;
    }

  if(sem_timedwait(&dev->child->txsem, &timeout))
    {
      dbg("Radio Device timedout on Tx\n");
    }

done:

  /* okay, tx interrupt received. check transmission status to decide success. */

  return ret;
}

/****************************************************************************
 * Name: ieee802154dev_ioctl
 *
 * Description:
 *   Control the MRF24J40 device. This is where the real operations happen.
 *
 ****************************************************************************/

static int ieee802154dev_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode                   *inode = filep->f_inode;
  FAR struct ieee802154_devwrapper_s *dev   = inode->i_private;
  FAR struct ieee802154_radio_s        *child = dev->child;
  int ret = OK;

  switch(cmd)
    {
      case MAC854IOCSCHAN   : ret = child->ops->setchannel  (child, (uint8_t)                     arg); break;
      case MAC854IOCGCHAN   : ret = child->ops->getchannel  (child, (FAR uint8_t*)                arg); break;
      case MAC854IOCSPANID  : ret = child->ops->setpanid    (child, (uint16_t)                    arg); break;
      case MAC854IOCGPANID  : ret = child->ops->getpanid    (child, (FAR uint16_t*)               arg); break;
      case MAC854IOCSSADDR  : ret = child->ops->setsaddr    (child, (uint16_t)                    arg); break;
      case MAC854IOCGSADDR  : ret = child->ops->getsaddr    (child, (FAR uint16_t*)               arg); break;
      case MAC854IOCSEADDR  : ret = child->ops->seteaddr    (child, (FAR uint8_t*)                arg); break;
      case MAC854IOCGEADDR  : ret = child->ops->geteaddr    (child, (FAR uint8_t*)                arg); break;
      case MAC854IOCSPROMISC: ret = child->ops->setpromisc  (child, (bool)                        arg); break;
      case MAC854IOCGPROMISC: ret = child->ops->getpromisc  (child, (FAR bool*)                   arg); break;
      case MAC854IOCSDEVMODE: ret = child->ops->setdevmode  (child, (uint8_t)                     arg); break;
      case MAC854IOCGDEVMODE: ret = child->ops->getdevmode  (child, (FAR uint8_t*)                arg); break;
      case MAC854IOCSTXP    : ret = child->ops->settxpower  (child, (int32_t)                     arg); break;
      case MAC854IOCGTXP    : ret = child->ops->gettxpower  (child, (FAR int32_t*)                arg); break;
      case MAC854IOCSCCA    : ret = child->ops->setcca      (child, (FAR struct ieee802154_cca_s*)arg); break;
      case MAC854IOCGCCA    : ret = child->ops->getcca      (child, (FAR struct ieee802154_cca_s*)arg); break;
      case MAC854IOCGED     : ret = child->ops->energydetect(child, (FAR uint8_t*)                arg); break;
      default               : ret = child->ops->ioctl       (child, cmd, arg);
    }

  return ret;
}

/****************************************************************************
 * Name: ieee802154dev_register
 *
 * Description:
 *   Register /dev/ieee%d
 *
 ****************************************************************************/

int ieee802154_register(FAR struct ieee802154_radio_s *ieee, unsigned int minor)
{
  char                               devname[16];
  FAR struct ieee802154_devwrapper_s *dev;

  dev = kmm_zalloc(sizeof(struct ieee802154_devwrapper_s));

  if (!dev)
    {
      return -ENOMEM;
    }

  dev->child = ieee;

  sem_init(&dev->devsem  , 0, 1); /* Allow the device to be opened once before blocking */

  sprintf(devname, "/dev/ieee%d", minor);

  return register_driver(devname, &ieee802154dev_fops, 0666, dev);
}

