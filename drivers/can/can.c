/****************************************************************************
 * drivers/can/can.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <assert.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/can/can.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_CAN_TXREADY
#  include <nuttx/wqueue.h>
#endif

#include <nuttx/irq.h>

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_CAN_TXREADY
#  if !defined(CONFIG_SCHED_WORKQUEUE)
#    error Work queue support required in this configuration
#    undef CONFIG_CAN_TXREADY
#    undef CONFIG_CAN_TXREADY_LOPRI
#    undef CONFIG_CAN_TXREADY_HIPRI
#  elif defined(CONFIG_CAN_TXREADY_LOPRI)
#    undef CONFIG_CAN_TXREADY_HIPRI
#    ifdef CONFIG_SCHED_LPWORK
#      define CANWORK LPWORK
#    else
#      error Low priority work queue support required in this configuration
#      undef CONFIG_CAN_TXREADY
#      undef CONFIG_CAN_TXREADY_LOPRI
#    endif
#  elif defined(CONFIG_CAN_TXREADY_HIPRI)
#    ifdef CONFIG_SCHED_HPWORK
#      define CANWORK HPWORK
#    else
#      error High priority work queue support required in this configuration
#      undef CONFIG_CAN_TXREADY
#      undef CONFIG_CAN_TXREADY_HIPRI
#    endif
#  else
#    error No work queue selection
#    undef CONFIG_CAN_TXREADY
#  endif
#endif

/* Timing Definitions *******************************************************/

#define HALF_SECOND_MSEC 500
#define HALF_SECOND_USEC 500000L

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphore helpers */

static int            can_takesem(FAR sem_t *sem);

/* Poll helpers */

static void           can_pollnotify(FAR struct can_dev_s *dev,
                                     pollevent_t eventset);

/* CAN helpers */

static uint8_t        can_dlc2bytes(uint8_t dlc);
#if 0 /* Not used */
static uint8_t        can_bytes2dlc(uint8_t nbytes);
#endif
#ifdef CONFIG_CAN_TXREADY
static void           can_txready_work(FAR void *arg);
#endif

/* Character driver methods */

static int            can_open(FAR struct file *filep);
static int            can_close(FAR struct file *filep);
static ssize_t        can_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);
static int            can_xmit(FAR struct can_dev_s *dev);
static ssize_t        can_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static inline ssize_t can_rtrread(FAR struct file *filep,
                                  FAR struct canioc_rtr_s *rtr);
static int            can_ioctl(FAR struct file *filep, int cmd,
                                unsigned long arg);
static int            can_poll(FAR struct file *filep,
                               FAR struct pollfd *fds,
                               bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_canops =
{
  can_open,  /* open */
  can_close, /* close */
  can_read,  /* read */
  can_write, /* write */
  NULL,      /* seek */
  can_ioctl, /* ioctl */
  can_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL     /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_takesem
 ****************************************************************************/

static int can_takesem(FAR sem_t *sem)
{
  return nxsem_wait(sem);
}

/****************************************************************************
 * Name: can_givesem
 ****************************************************************************/

#define can_givesem(sem) nxsem_post(sem)

/****************************************************************************
 * Name: can_pollnotify
 ****************************************************************************/

static void can_pollnotify(FAR struct can_dev_s *dev, pollevent_t eventset)
{
  FAR struct pollfd *fds;
  int i;

  for (i = 0; i < CONFIG_CAN_NPOLLWAITERS; i++)
    {
      fds = dev->cd_fds[i];
      if (fds != NULL)
        {
          fds->revents |= fds->events & eventset;
          if (fds->revents != 0)
            {
              caninfo("Report events: %02x\n", fds->revents);
              nxsem_post(fds->sem);
            }
        }
    }
}

/****************************************************************************
 * Name: can_dlc2bytes
 *
 * Description:
 *   In the CAN FD format, the coding of the DLC differs from the standard
 *   CAN format. The DLC codes 0 to 8 have the same coding as in standard
 *   CAN.  But the codes 9 to 15 all imply a data field of 8 bytes with
 *   standard CAN.  In CAN FD mode, the values 9 to 15 are encoded to values
 *   in the range 12 to 64.
 *
 * Input Parameters:
 *   dlc    - the DLC value to convert to a byte count
 *
 * Returned Value:
 *   The number of bytes corresponding to the DLC value.
 *
 ****************************************************************************/

static uint8_t can_dlc2bytes(uint8_t dlc)
{
  if (dlc > 8)
    {
#ifdef CONFIG_CAN_FD
      switch (dlc)
        {
          case 9:
            return 12;

          case 10:
            return 16;

          case 11:
            return 20;

          case 12:
            return 24;

          case 13:
            return 32;

          case 14:
            return 48;

          default:
          case 15:
            return 64;
        }
#else
      return 8;
#endif
    }

  return dlc;
}

/****************************************************************************
 * Name: can_bytes2dlc
 *
 * Description:
 *   In the CAN FD format, the coding of the DLC differs from the standard
 *   CAN format. The DLC codes 0 to 8 have the same coding as in standard
 *   CAN.  But the codes 9 to 15 all imply a data field of 8 bytes with
 *   standard CAN.  In CAN FD mode, the values 9 to 15 are encoded to values
 *   in the range 12 to 64.
 *
 * Input Parameters:
 *   nbytes - the byte count to convert to a DLC value
 *
 * Returned Value:
 *   The encoded DLC value corresponding to at least that number of bytes.
 *
 ****************************************************************************/

#if 0 /* Not used */
static uint8_t can_bytes2dlc(FAR struct sam_can_s *priv, uint8_t nbytes)
{
  if (nbytes <= 8)
    {
      return nbytes;
    }
#ifdef CONFIG_CAN_FD
  else if (nbytes <= 12)
    {
      return 9;
    }
  else if (nbytes <= 16)
    {
      return 10;
    }
  else if (nbytes <= 20)
    {
      return 11;
    }
  else if (nbytes <= 24)
    {
      return 12;
    }
  else if (nbytes <= 32)
    {
      return 13;
    }
  else if (nbytes <= 48)
    {
      return 14;
    }
  else /* if (nbytes <= 64) */
    {
      return 15;
    }
#else
  else
    {
      return 8;
    }
#endif
}
#endif

/****************************************************************************
 * Name: can_txready_work
 *
 * Description:
 *   This function performs deferred processing from can_txready.  See the
 *   description of can_txready below for additional information.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_TXREADY
static void can_txready_work(FAR void *arg)
{
  FAR struct can_dev_s *dev = (FAR struct can_dev_s *)arg;
  irqstate_t flags;
  int ret;

  caninfo("xmit head: %d queue: %d tail: %d\n",
          dev->cd_xmit.tx_head, dev->cd_xmit.tx_queue,
          dev->cd_xmit.tx_tail);

  /* Verify that the xmit FIFO is not empty.  The following operations must
   * be performed with interrupt disabled.
   */

  flags = enter_critical_section();
  if (dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail)
    {
      /* Send the next message in the FIFO. */

      ret = can_xmit(dev);

      /* If the message was successfully queued in the H/W FIFO, then
       * can_txdone() should have been called.  If the S/W FIFO were
       * full before then there should now be free space in the S/W FIFO.
       */

      if (ret >= 0)
        {
          /* Are there any threads waiting for space in the TX FIFO? */

          if (dev->cd_ntxwaiters > 0)
            {
              /* Yes.. Inform them that new xmit space is available */

              can_givesem(&dev->cd_xmit.tx_sem);
            }
        }
    }

  leave_critical_section(flags);
}
#endif

static FAR struct can_reader_s *init_can_reader(FAR struct file *filep)
{
  FAR struct can_reader_s *reader = kmm_zalloc(sizeof(struct can_reader_s));
  DEBUGASSERT(reader != NULL);

  reader->fifo.rx_head  = 0;
  reader->fifo.rx_tail  = 0;

  nxsem_init(&reader->fifo.rx_sem, 0, 1);
  nxsem_set_protocol(&reader->fifo.rx_sem, SEM_PRIO_NONE);
  filep->f_priv = reader;

  return reader;
}

/****************************************************************************
 * Name: can_open
 *
 * Description:
 *   This function is called whenever the CAN device is opened.
 *
 ****************************************************************************/

static int can_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct can_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   tmp;
  int                   ret;

  /* If the port is the middle of closing, wait until the close is finished */

  ret = can_takesem(&dev->cd_closesem);
  if (ret < 0)
    {
      return ret;
    }

  /* If this is the first time that the driver has been opened
   * for this device, then perform hardware initialization.
   */

  if (list_is_empty(&dev->cd_readers))
    {
      caninfo("ocount: %d\n", 0);

      flags = enter_critical_section();
      ret = dev_setup(dev);
      if (ret >= 0)
        {
          /* Mark the FIFOs empty */

          dev->cd_xmit.tx_head  = 0;
          dev->cd_xmit.tx_queue = 0;
          dev->cd_xmit.tx_tail  = 0;

          /* Finally, Enable the CAN RX interrupt */

          dev_rxint(dev, true);
        }

      list_add_head(&dev->cd_readers,
                    (FAR struct list_node *)init_can_reader(filep));

      leave_critical_section(flags);
    }
  else
    {
      tmp = list_length(&dev->cd_readers);
      caninfo("ocount: %d\n", tmp);

      if (tmp >= 255)
        {
          /* Limit to no more than 255 opens */

          ret = -EMFILE;
          goto errout;
        }

      flags = enter_critical_section();
      list_add_head(&dev->cd_readers,
                    (FAR struct list_node *)init_can_reader(filep));

      leave_critical_section(flags);
    }

errout:
  can_givesem(&dev->cd_closesem);
  return ret;
}

/****************************************************************************
 * Name: can_close
 *
 * Description:
 *   This routine is called when the CAN device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int can_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct can_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  FAR struct list_node *node;
  FAR struct list_node *tmp;
  int                   ret;

#ifdef  CONFIG_DEBUG_CAN_INFO
  caninfo("ocount: %d\n", list_length(&dev->cd_readers));
#endif

  ret = can_takesem(&dev->cd_closesem);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_safe(&dev->cd_readers, node, tmp)
    {
      if (((FAR struct can_reader_s *)node) ==
          ((FAR struct can_reader_s *)filep->f_priv))
        {
          list_delete(node);
          kmm_free(node);
          break;
        }
    }

  filep->f_priv = NULL;

  /* Uninitialize the driver if there are no more readers */

  if (!list_is_empty(&dev->cd_readers))
    {
      goto errout;
    }

  /* Stop accepting input */

  dev_rxint(dev, false);

  /* Now we wait for the transmit FIFO to clear */

  while (dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail)
    {
       nxsig_usleep(HALF_SECOND_USEC);
    }

  /* And wait for the TX hardware FIFO to drain */

  while (!dev_txempty(dev))
    {
      nxsig_usleep(HALF_SECOND_USEC);
    }

  /* Free the IRQ and disable the CAN device */

  flags = enter_critical_section(); /* Disable interrupts */
  dev_shutdown(dev);                /* Disable the CAN */
  leave_critical_section(flags);

errout:
  can_givesem(&dev->cd_closesem);
  return ret;
}

/****************************************************************************
 * Name: can_read
 *
 * Description:
 *   Read standard CAN messages
 *
 ****************************************************************************/

static ssize_t can_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct can_reader_s  *reader = NULL;
  FAR struct can_rxfifo_s  *fifo;
  size_t                    nread;
  irqstate_t                flags;
  int                       ret = 0;
#ifdef CONFIG_CAN_ERRORS
  FAR struct inode         *inode = filep->f_inode;
  FAR struct can_dev_s     *dev = inode->i_private;
#endif

  caninfo("buflen: %d\n", buflen);

  /* The caller must provide enough memory to catch the smallest possible
   * message.  This is not a system error condition, but we won't permit
   * it,  Hence we return 0.
   */

  if (buflen >= CAN_MSGLEN(0))
    {
      /* Interrupts must be disabled while accessing the cd_recv FIFO */

      flags = enter_critical_section();

#ifdef CONFIG_CAN_ERRORS
      /* Check for internal errors */

      if (dev->cd_error != 0)
        {
          FAR struct can_msg_s *msg;

          /* Detected an internal driver error.  Generate a
           * CAN_ERROR_MESSAGE
           */

          if (buflen < CAN_MSGLEN(CAN_ERROR_DLC))
            {
              goto return_with_irqdisabled;
            }

          msg                   = (FAR struct can_msg_s *)buffer;
          msg->cm_hdr.ch_id     = CAN_ERROR_INTERNAL;
          msg->cm_hdr.ch_dlc    = CAN_ERROR_DLC;
          msg->cm_hdr.ch_rtr    = 0;
          msg->cm_hdr.ch_error  = 1;
#ifdef CONFIG_CAN_EXTID
          msg->cm_hdr.ch_extid  = 0;
#endif
          msg->cm_hdr.ch_unused = 0;
          memset(&(msg->cm_data), 0, CAN_ERROR_DLC);
          msg->cm_data[5]       = dev->cd_error;

          /* Reset the error flag */

          dev->cd_error         = 0;

          ret = CAN_MSGLEN(CAN_ERROR_DLC);
          goto return_with_irqdisabled;
        }
#endif /* CONFIG_CAN_ERRORS */

      DEBUGASSERT(filep->f_priv != NULL);
      reader = (FAR struct can_reader_s *)filep->f_priv;

      fifo = &reader->fifo;

      while (fifo->rx_head == fifo->rx_tail)
        {
          /* The receive FIFO is empty -- was non-blocking mode selected? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              ret = -EAGAIN;
              goto return_with_irqdisabled;
            }

          /* Wait for a message to be received */

          ret = can_takesem(&fifo->rx_sem);

          if (ret < 0)
            {
              goto return_with_irqdisabled;
            }
        }

      /* The cd_recv FIFO is not empty.  Copy all buffered data that will fit
       * in the user buffer.
       */

      nread = 0;
      do
        {
          /* Will the next message in the FIFO fit into the user buffer? */

          FAR struct can_msg_s *msg = &fifo->rx_buffer[fifo->rx_head];
          int nbytes = can_dlc2bytes(msg->cm_hdr.ch_dlc);
          int msglen = CAN_MSGLEN(nbytes);

          if (nread + msglen > buflen)
            {
              break;
            }

          /* Copy the message to the user buffer */

          memcpy(&buffer[nread], msg, msglen);
          nread += msglen;

          /* Increment the head of the circular message buffer */

          if (++fifo->rx_head >= CONFIG_CAN_FIFOSIZE)
            {
              fifo->rx_head = 0;
            }
        }
      while (fifo->rx_head != fifo->rx_tail);

      /* All on the messages have bee transferred.  Return the number of
       * bytes that were read.
       */

      ret = nread;

return_with_irqdisabled:
      leave_critical_section(flags);
    }

  return ret;
}

/****************************************************************************
 * Name: can_xmit
 *
 * Description:
 *   Send the message at the head of the cd_xmit FIFO
 *
 * Assumptions:
 *   Called with interrupts disabled
 *
 ****************************************************************************/

static int can_xmit(FAR struct can_dev_s *dev)
{
  int tmpndx;
  int ret = -EBUSY;

  caninfo("xmit head: %d queue: %d tail: %d\n",
          dev->cd_xmit.tx_head, dev->cd_xmit.tx_queue, dev->cd_xmit.tx_tail);

  /* If there is nothing to send, then just disable interrupts and return */

  if (dev->cd_xmit.tx_head == dev->cd_xmit.tx_tail)
    {
      DEBUGASSERT(dev->cd_xmit.tx_queue == dev->cd_xmit.tx_head);

#ifndef CONFIG_CAN_TXREADY
      /* We can disable CAN TX interrupts -- unless there is a H/W FIFO.  In
       * that case, TX interrupts must stay enabled until the H/W FIFO is
       * fully emptied.
       */

      dev_txint(dev, false);
#endif
      return -EIO;
    }

  /* Check if we have already queued all of the data in the TX fifo.
   *
   * tx_tail:  Incremented in can_write each time a message is queued in the
   *           FIFO
   * tx_head:  Incremented in can_txdone each time a message completes
   * tx_queue: Incremented each time that a message is sent to the hardware.
   *
   * Logically (ignoring buffer wrap-around): tx_head <= tx_queue <= tx_tail
   * tx_head == tx_queue == tx_tail means that the FIFO is empty
   * tx_head < tx_queue == tx_tail means that all data has been queued, but
   * we are still waiting for transmissions to complete.
   */

  while (dev->cd_xmit.tx_queue != dev->cd_xmit.tx_tail && dev_txready(dev))
    {
      /* No.. The FIFO should not be empty in this case */

      DEBUGASSERT(dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail);

      /* Increment the FIFO queue index before sending (because dev_send()
       * might call can_txdone()).
       */

      tmpndx = dev->cd_xmit.tx_queue;
      if (++dev->cd_xmit.tx_queue >= CONFIG_CAN_FIFOSIZE)
        {
          dev->cd_xmit.tx_queue = 0;
        }

      /* Send the next message at the FIFO queue index */

      ret = dev_send(dev, &dev->cd_xmit.tx_buffer[tmpndx]);
      if (ret < 0)
        {
          canerr("dev_send failed: %d\n", ret);
          break;
        }
    }

  /* Make sure that TX interrupts are enabled */

  dev_txint(dev, true);
  return ret;
}

/****************************************************************************
 * Name: can_write
 ****************************************************************************/

static ssize_t can_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct can_dev_s    *dev   = inode->i_private;
  FAR struct can_txfifo_s *fifo  = &dev->cd_xmit;
  FAR struct can_msg_s    *msg;
  bool                     inactive;
  ssize_t                  nsent = 0;
  irqstate_t               flags;
  int                      nexttail;
  int                      nbytes;
  int                      msglen;
  int                      ret   = 0;

  caninfo("buflen: %d\n", buflen);

  /* Interrupts must disabled throughout the following */

  flags = enter_critical_section();

  /* Check if the TX is inactive when we started. In certain race conditions,
   * there may be a pending interrupt to kick things back off, but we will
   * be sure here that there is not.  That the hardware is IDLE and will
   * need to be kick-started.
   */

  inactive = dev_txempty(dev);

  /* Add the messages to the FIFO.  Ignore any trailing messages that are
   * shorter than the minimum.
   */

  while (((ssize_t)buflen - nsent) >= CAN_MSGLEN(0))
    {
      /* Check if adding this new message would over-run the drivers ability
       * to enqueue xmit data.
       */

      nexttail = fifo->tx_tail + 1;
      if (nexttail >= CONFIG_CAN_FIFOSIZE)
        {
          nexttail = 0;
        }

      /* If the XMIT FIFO becomes full, then wait for space to become
       * available.
       */

      while (nexttail == fifo->tx_head)
        {
          /* The transmit FIFO is full  -- was non-blocking mode selected? */

          if ((filep->f_oflags & O_NONBLOCK) != 0)
            {
              if (nsent == 0)
                {
                  ret = -EAGAIN;
                }
              else
                {
                  ret = nsent;
                }

              goto return_with_irqdisabled;
            }

          /* If the TX hardware was inactive when we started, then we will
           * have start the XMIT sequence generate the TX done interrupts
           * needed to clear the FIFO.
           */

          if (inactive)
            {
              can_xmit(dev);
            }

          /* Wait for a message to be sent */

          DEBUGASSERT(dev->cd_ntxwaiters < 255);
          dev->cd_ntxwaiters++;
          ret = can_takesem(&fifo->tx_sem);
          dev->cd_ntxwaiters--;
          if (ret < 0)
            {
              goto return_with_irqdisabled;
            }

          /* Re-check the FIFO state */

          inactive = dev_txempty(dev);
        }

      /* We get here if there is space at the end of the FIFO.  Add the new
       * CAN message at the tail of the FIFO.
       */

      msg    = (FAR struct can_msg_s *)&buffer[nsent];
      nbytes = can_dlc2bytes(msg->cm_hdr.ch_dlc);
      msglen = CAN_MSGLEN(nbytes);
      memcpy(&fifo->tx_buffer[fifo->tx_tail], msg, msglen);

      /* Increment the tail of the circular buffer */

      fifo->tx_tail = nexttail;

      /* Increment the number of bytes that were sent */

      nsent += msglen;
    }

  /* We get here after all messages have been added to the FIFO.  Check if
   * we need to kick off the XMIT sequence.
   */

  if (inactive)
    {
      can_xmit(dev);
    }

  /* Return the number of bytes that were sent */

  ret = nsent;

return_with_irqdisabled:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: can_rtrread
 *
 * Description:
 *   Read RTR messages.  The RTR message is a special message -- it is an
 *   outgoing message that says "Please re-transmit the message with the
 *   same identifier as this message.  So the RTR read is really a
 *   send-wait-receive operation.
 *
 ****************************************************************************/

static inline ssize_t can_rtrread(FAR struct file *filep,
                                  FAR struct canioc_rtr_s *request)
{
  FAR struct can_dev_s *dev = filep->f_inode->i_private;
  FAR struct can_rtrwait_s *wait = NULL;
  struct timespec           abstimeout;
  irqstate_t                flags;
  int                       i;
  int                       sval;
  int                       ret = -ENOMEM;

  /* Disable interrupts through this operation */

  flags = enter_critical_section();

  /* Find an available slot in the pending RTR list */

  for (i = 0; i < CONFIG_CAN_NPENDINGRTR; i++)
    {
      FAR struct can_rtrwait_s *tmp = &dev->cd_rtr[i];

      ret = nxsem_get_value(&tmp->cr_sem, &sval);

      if (ret < 0)
        {
          continue;
        }

      if (sval == 0)
        {
          /* No one is waiting on RTR transaction; take it. */

          tmp->cr_msg     = request->ci_msg;
          dev->cd_npendrtr++;

          wait            = tmp;
          break;
        }
    }

  if (wait)
    {
      /* Send the remote transmission request with the "old method" unless
       * the lower-half driver indicates otherwise.
       */

      if (dev->cd_ops->co_remoterequest != NULL)
        {
          if (request->ci_msg->cm_hdr.ch_id < CAN_MAX_STDMSGID
#ifdef CONFIG_CAN_EXTID
              && !request->ci_msg->cm_hdr.ch_extid
#endif
            )
            {
              ret = dev_remoterequest(dev,
                                (uint16_t)(request->ci_msg->cm_hdr.ch_id));
            }
          else
            {
              ret = -EINVAL;
            }
        }
      else
        {
#ifdef CONFIG_CAN_USE_RTR
          /* Temporarily set the RTR bit, then send the remote transmission
           * request message with the lower-half driver's regular function.
           */

          request->ci_msg->cm_hdr.ch_rtr = 1;
          ret = can_write(filep,
                          (const char *) request->ci_msg,
                          CAN_MSGLEN(request->ci_msg->cm_hdr.ch_dlc));
          request->ci_msg->cm_hdr.ch_rtr = 0;
#else
          canerr("Error: Driver needs CONFIG_CAN_USE_RTR.\n");
          ret = -ENOSYS;
#endif
        }

      if (ret >= 0)
        {
          /* Then wait for the response */

          ret = clock_gettime(CLOCK_REALTIME, &abstimeout);

          if (ret >= 0)
            {
              clock_timespec_add(&abstimeout,
                                 &request->ci_timeout,
                                 &abstimeout);
              ret = nxsem_timedwait(&wait->cr_sem, &abstimeout);
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: can_ioctl
 ****************************************************************************/

static int can_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct can_dev_s *dev   = inode->i_private;
  int                   ret   = OK;

  caninfo("cmd: %d arg: %ld\n", cmd, arg);

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* CANIOC_RTR: Send the remote transmission request and wait for the
       * response.  Argument is a reference to struct canioc_rtr_s
       * (casting to uintptr_t first eliminates complaints on some
       * architectures where the sizeof long is different from the size of
       * a pointer).
       */

      case CANIOC_RTR:
        ret = can_rtrread(filep,
                          (FAR struct canioc_rtr_s *)((uintptr_t)arg));
        break;

      /* Not a "built-in" ioctl command.. perhaps it is unique to this
       * lower-half, device driver.
       */

      default:
        ret = dev_ioctl(dev, cmd, arg);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: can_poll
 ****************************************************************************/

static int can_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  FAR struct inode *inode = (FAR struct inode *)filep->f_inode;
  FAR struct can_dev_s *dev = (FAR struct can_dev_s *)inode->i_private;
  FAR struct can_reader_s *reader = NULL;
  pollevent_t eventset;
  int ndx;
  irqstate_t flags;
  int ret;
  int i;

  /* Some sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (dev == NULL || fds == NULL)
    {
      return -ENODEV;
    }
#endif

  flags = enter_critical_section();

  DEBUGASSERT(filep->f_priv != NULL);
  reader = (FAR struct can_reader_s *)filep->f_priv;

  /* Get exclusive access to the poll structures */

  ret = can_takesem(&dev->cd_pollsem);
  if (ret < 0)
    {
      /* A signal received while waiting for access to the poll data
       * will abort the operation
       */

      goto return_with_irqdisabled;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference.
       */

      for (i = 0; i < CONFIG_CAN_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (dev->cd_fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              dev->cd_fds[i] = fds;
              fds->priv       = &dev->cd_fds[i];
              break;
            }
        }

      if (i >= CONFIG_CAN_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events?
       * First, check if the xmit buffer is full.
       *
       * Get exclusive access to the cd_xmit buffer indices.  NOTE: that
       * we do not let this wait be interrupted by a signal (we probably
       * should, but that would be a little awkward).
       */

      eventset = 0;

      DEBUGASSERT(dev->cd_ntxwaiters < 255);
      dev->cd_ntxwaiters++;
      do
        {
          ret = can_takesem(&dev->cd_xmit.tx_sem);
        }
      while (ret < 0);
      dev->cd_ntxwaiters--;

      ndx = dev->cd_xmit.tx_tail + 1;
      if (ndx >= CONFIG_CAN_FIFOSIZE)
        {
          ndx = 0;
        }

      if (ndx != dev->cd_xmit.tx_head)
        {
          eventset |= fds->events & POLLOUT;
        }

      can_givesem(&dev->cd_xmit.tx_sem);

      /* Check if the receive buffer is empty.
       *
       * Get exclusive access to the cd_recv buffer indices.  NOTE: that
       * we do not let this wait be interrupted by a signal (we probably
       * should, but that would be a little awkward).
       */

      do
        {
          ret = can_takesem(&reader->fifo.rx_sem);
        }
      while (ret < 0);

      if (reader->fifo.rx_head != reader->fifo.rx_tail)
        {
          eventset |= fds->events & POLLIN;
        }

      can_givesem(&reader->fifo.rx_sem);

      if (eventset != 0)
        {
          can_pollnotify(dev, eventset);
        }
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG_FEATURES
      if (slot == NULL)
        {
          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  can_givesem(&dev->cd_pollsem);

return_with_irqdisabled:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_register
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

int can_register(FAR const char *path, FAR struct can_dev_s *dev)
{
  int i;

  /* Initialize the CAN device structure */

  list_initialize(&dev->cd_readers);
  dev->cd_ntxwaiters = 0;
  dev->cd_npendrtr   = 0;
#ifdef CONFIG_CAN_ERRORS
  dev->cd_error      = 0;
#endif

  /* Initialize semaphores */

  nxsem_init(&dev->cd_xmit.tx_sem, 0, 1);
  nxsem_set_protocol(&dev->cd_xmit.tx_sem, SEM_PRIO_NONE);
  nxsem_init(&dev->cd_closesem,    0, 1);
  nxsem_init(&dev->cd_pollsem,     0, 1);

  for (i = 0; i < CONFIG_CAN_NPENDINGRTR; i++)
    {
      /* Initialize wait semaphores.  These semaphores are used for signaling
       * and should not have priority inheritance enabled.
       */

      nxsem_init(&dev->cd_rtr[i].cr_sem, 0, 0);
      nxsem_set_protocol(&dev->cd_rtr[i].cr_sem, SEM_PRIO_NONE);
    }

  /* Initialize/reset the CAN hardware */

  dev_reset(dev);

  /* Register the CAN device */

  caninfo("Registering %s\n", path);
  return register_driver(path, &g_canops, 0666, dev);
}

/****************************************************************************
 * Name: can_receive
 *
 * Description:
 *   Called from the CAN interrupt handler when new read data is available
 *
 * Input Parameters:
 *   dev  - CAN driver state structure
 *   hdr  - CAN message header
 *   data - CAN message data (if DLC > 0)
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 * Assumptions:
 *   CAN interrupts are disabled.
 *
 ****************************************************************************/

int can_receive(FAR struct can_dev_s *dev, FAR struct can_hdr_s *hdr,
                FAR uint8_t *data)
{
  FAR struct can_rxfifo_s *fifo;
  FAR uint8_t             *dest;
  FAR struct list_node    *node;
  FAR struct list_node    *tmp;
  int                      nexttail;
  int                      errcode = -ENOMEM;
  int                      i;
  int                      sval;
  int                      ret;

  caninfo("ID: %" PRId32 " DLC: %d\n", (uint32_t)hdr->ch_id, hdr->ch_dlc);

  /* Check if adding this new message would over-run the drivers ability to
   * enqueue read data.
   */

  /* First, check if this response matches any RTR response that we may be
   * waiting for.
   */

  if (dev->cd_npendrtr > 0)
    {
      /* There are pending RTR requests -- search the lists of requests
       * and see any any matches this new message.
       */

      for (i = 0; i < CONFIG_CAN_NPENDINGRTR; i++)
        {
          FAR struct can_rtrwait_s *wait = &dev->cd_rtr[i];
          FAR struct can_msg_s     *waitmsg = wait->cr_msg;

          /* Check if the entry is in use and whether the ID matches */

          ret = nxsem_get_value(&wait->cr_sem, &sval);

          if (ret < 0)
            {
              continue;
            }

          else if (sval < 0
#ifdef CONFIG_CAN_ERRORS
                && hdr->ch_error == false
#endif
#ifdef CONFIG_CAN_EXTID
                && waitmsg->cm_hdr.ch_extid == hdr->ch_extid
#endif
                && waitmsg->cm_hdr.ch_id == hdr->ch_id)
            {
              int nbytes;

              /* We have the response... copy the data to the user's buffer */

              memcpy(&waitmsg->cm_hdr, hdr, sizeof(struct can_hdr_s));

              nbytes = can_dlc2bytes(hdr->ch_dlc);
              for (i = 0, dest = waitmsg->cm_data; i < nbytes; i++)
                {
                  *dest++ = *data++;
                }

              dev->cd_npendrtr--;

              /* Restart the waiting thread and mark the entry unused */

              can_givesem(&wait->cr_sem);
            }
        }
    }

  list_for_every_safe(&dev->cd_readers, node, tmp)
    {
      FAR struct can_reader_s *reader = (FAR struct can_reader_s *)node;
      fifo = &reader->fifo;

      nexttail = fifo->rx_tail + 1;
      if (nexttail >= CONFIG_CAN_FIFOSIZE)
        {
          nexttail = 0;
        }

      /* Refuse the new data if the FIFO is full */

      if (nexttail != fifo->rx_head)
        {
          int nbytes;

          /* Add the new, decoded CAN message at the tail of the FIFO.
           *
           * REVISIT:  In the CAN FD format, the coding of the DLC differs
           * from the standard CAN format. The DLC codes 0 to 8 have the
           * same coding as in standard CAN, the codes 9 to 15, which in
           * standard CAN all code a data field of 8 bytes, are encoded:
           *
           *   9->12, 10->16, 11->20, 12->24, 13->32, 14->48, 15->64
           */

          memcpy(&fifo->rx_buffer[fifo->rx_tail].cm_hdr, hdr,
                 sizeof(struct can_hdr_s));

          nbytes = can_dlc2bytes(hdr->ch_dlc);
          memcpy(fifo->rx_buffer[fifo->rx_tail].cm_data, data, nbytes);

          /* Increment the tail of the circular buffer */

          fifo->rx_tail = nexttail;

          /* Notify all poll/select waiters that they can read from the
           * cd_recv buffer
           */

          can_pollnotify(dev, POLLIN);

          sval = 0;
          if (nxsem_get_value(&fifo->rx_sem, &sval) < 0)
            {
              DEBUGASSERT(false);
#ifdef CONFIG_CAN_ERRORS
              /* Report unspecified error */

              dev->cd_error |= CAN_ERROR5_UNSPEC;
#endif
              return -EINVAL;
            }

          /* Increment the counting semaphore. The maximum value should
           * be CONFIG_CAN_FIFOSIZE -- one possible count for each allocated
           * message buffer.
           */

          if (sval <= 0)
            {
              can_givesem(&fifo->rx_sem);
            }

          errcode = OK;
        }
#ifdef CONFIG_CAN_ERRORS
      else
        {
          /* Report rx overflow error */

          dev->cd_error |= CAN_ERROR5_RXOVERFLOW;
        }
#endif
    }

  return errcode;
}

/****************************************************************************
 * Name: can_txdone
 *
 * Description:
 *   Called when the hardware has processed the outgoing TX message.  This
 *   normally means that the CAN messages was sent out on the wire.  But
 *   if the CAN hardware supports a H/W TX FIFO, then this call may mean
 *   only that the CAN message has been added to the H/W FIFO.  In either
 *   case, the upper-half CAN driver can remove the outgoing message from
 *   the S/W FIFO and discard it.
 *
 *   This function may be called in different contexts, depending upon the
 *   nature of the underlying CAN hardware.
 *
 *   1. No H/W TX FIFO (CONFIG_CAN_TXREADY not defined)
 *
 *      This function is only called from the CAN interrupt handler at the
 *      completion of a send operation.
 *
 *        can_write() -> can_xmit() -> dev_send()
 *        CAN interrupt -> can_txdone()
 *
 *      If the CAN hardware is busy, then the call to dev_send() will
 *      fail, the S/W TX FIFO will accumulate outgoing messages, and the
 *      thread calling can_write() may eventually block waiting for space in
 *      the S/W TX FIFO.
 *
 *      When the CAN hardware completes the transfer and processes the
 *      CAN interrupt, the call to can_txdone() will make space in the S/W
 *      TX FIFO and will awaken the waiting can_write() thread.
 *
 *   2a. H/W TX FIFO (CONFIG_CAN_TXREADY=y) and S/W TX FIFO not full
 *
 *      This function will be called back from dev_send() immediately when a
 *      new CAN message is added to H/W TX FIFO:
 *
 *        can_write() -> can_xmit() -> dev_send() -> can_txdone()
 *
 *      When the H/W TX FIFO becomes full, dev_send() will fail and
 *      can_txdone() will not be called.  In this case the S/W TX FIFO will
 *      accumulate outgoing messages, and the thread calling can_write() may
 *      eventually block waiting for space in the S/W TX FIFO.
 *
 *   2b. H/W TX FIFO (CONFIG_CAN_TXREADY=y) and S/W TX FIFO full
 *
 *      In this case, the thread calling can_write() is blocked waiting for
 *      space in the S/W TX FIFO.  can_txdone() will be called, indirectly,
 *      from can_txready_work() running on the thread of the work queue.
 *
 *        CAN interrupt -> can_txready() -> Schedule can_txready_work()
 *        can_txready_work() -> can_xmit() -> dev_send() -> can_txdone()
 *
 *      The call dev_send() should not fail in this case and the subsequent
 *      call to can_txdone() will make space in the S/W TX FIFO and will
 *      awaken the waiting thread.
 *
 * Input Parameters:
 *   dev  - The specific CAN device
 *   hdr  - The 16-bit CAN header
 *   data - An array contain the CAN data.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 * Assumptions:
 *   Interrupts are disabled.  This is required by can_xmit() which is called
 *   by this function.  Interrupts are explicitly disabled when called
 *   through can_write().  Interrupts are expected be disabled when called
 *   from the CAN interrupt handler.
 *
 ****************************************************************************/

int can_txdone(FAR struct can_dev_s *dev)
{
  int ret = -ENOENT;

  caninfo("xmit head: %d queue: %d tail: %d\n",
          dev->cd_xmit.tx_head, dev->cd_xmit.tx_queue, dev->cd_xmit.tx_tail);

  /* Verify that the xmit FIFO is not empty */

  if (dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail)
    {
      /* The tx_queue index is incremented each time can_xmit() queues
       * the transmission.  When can_txdone() is called, the tx_queue
       * index should always have been advanced beyond the current tx_head
       * index.
       */

      DEBUGASSERT(dev->cd_xmit.tx_head != dev->cd_xmit.tx_queue);

      /* Remove the message at the head of the xmit FIFO */

      if (++dev->cd_xmit.tx_head >= CONFIG_CAN_FIFOSIZE)
        {
          dev->cd_xmit.tx_head = 0;
        }

      /* Send the next message in the FIFO */

      can_xmit(dev);

      /* Notify all poll/select waiters that they can write to the cd_xmit
       * buffer
       */

      can_pollnotify(dev, POLLOUT);

      /* Are there any threads waiting for space in the TX FIFO? */

      if (dev->cd_ntxwaiters > 0)
        {
          /* Yes.. Inform them that new xmit space is available */

          ret = can_givesem(&dev->cd_xmit.tx_sem);
        }
      else
        {
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: can_txready
 *
 * Description:
 *   Called from the CAN interrupt handler at the completion of a send
 *   operation.  This interface is needed only for CAN hardware that
 *   supports queueing of outgoing messages in a H/W FIFO.
 *
 *   The CAN upper half driver also supports a queue of output messages in a
 *   S/W FIFO.  Messages are added to that queue when when can_write() is
 *   called and removed from the queue in can_txdone() when each TX message
 *   is complete.
 *
 *   After each message is added to the S/W FIFO, the CAN upper half driver
 *   will attempt to send the message by calling into the lower half driver.
 *   That send will not be performed if the lower half driver is busy, i.e.,
 *   if dev_txready() returns false.  In that case, the number of messages in
 *   the S/W FIFO can grow.  If the S/W FIFO becomes full, then can_write()
 *   will wait for space in the S/W FIFO.
 *
 *   If the CAN hardware does not support a H/W FIFO then busy means that
 *   the hardware is actively sending the message and is guaranteed to
 *   become non-busy (i.e, dev_txready()) when the send transfer completes
 *   and can_txdone() is called.  So the call to can_txdone() means that the
 *   transfer has completed and also that the hardware is ready to accept
 *   another transfer.
 *
 *   If the CAN hardware supports a H/W FIFO, can_txdone() is not called
 *   when the transfer is complete, but rather when the transfer is queued in
 *   the H/W FIFO.  When the H/W FIFO becomes full, then dev_txready() will
 *   report false and the number of queued messages in the S/W FIFO will
 *   grow.
 *
 *   There is no mechanism in this case to inform the upper half driver when
 *   the hardware is again available, when there is again space in the H/W
 *   FIFO.  can_txdone() will not be called again.  If the S/W FIFO becomes
 *   full, then the upper half driver will wait for space to become
 *   available, but there is no event to awaken it and the driver will hang.
 *
 *   Enabling this feature adds support for the can_txready() interface.
 *   This function is called from the lower half driver's CAN interrupt
 *   handler each time a TX transfer completes.  This is a sure indication
 *   that the H/W FIFO is no longer full.  can_txready() will then awaken
 *   the can_write() logic and the hang condition is avoided.
 *
 * Input Parameters:
 *   dev  - The specific CAN device
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 * Assumptions:
 *   Interrupts are disabled.  This function may execute in the context of
 *   and interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_TXREADY
int can_txready(FAR struct can_dev_s *dev)
{
  int ret = -ENOENT;

  caninfo("xmit head: %d queue: %d tail: %d waiters: %d\n",
          dev->cd_xmit.tx_head, dev->cd_xmit.tx_queue, dev->cd_xmit.tx_tail,
          dev->cd_ntxwaiters);

  /* Verify that the xmit FIFO is not empty.  This is safe because interrupts
   * are always disabled when calling into can_xmit(); this cannot collide
   * with ongoing activity from can_write().
   */

  if (dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail)
    {
      /* Is work already scheduled? */

      if (work_available(&dev->cd_work))
        {
          /* Yes... schedule to perform can_txready() work on the worker
           * thread.  Although data structures are protected by disabling
           * interrupts, the can_xmit() operations may involve semaphore
           * operations and, hence, should not be done at the interrupt
           * level.
           */

          ret = work_queue(CANWORK, &dev->cd_work, can_txready_work, dev, 0);
        }
      else
        {
          ret = -EBUSY;
        }
    }
  else
    {
      /* There should not be any threads waiting for space in the S/W TX
       * FIFO is it is empty.  However, an assertion would fire in certain
       * race conditions, i.e, when all waiters have been awakened but
       * have not yet had a chance to decrement cd_ntxwaiters.
       */

#if 0 /* REVISIT */
      /* When the H/W FIFO has been emptied, we can disable further TX
       * interrupts.
       *
       * REVISIT:  The fact that the S/W FIFO is empty does not mean that
       * the H/W FIFO is also empty.  If we really want this to work this
       * way, then we would probably need and additional parameter to tell
       * us if the H/W FIFO is empty.
       */

      dev_txint(dev, false);
#endif
    }

  return ret;
}
#endif /* CONFIG_CAN_TXREADY */
#endif /* CONFIG_CAN */
