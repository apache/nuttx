/****************************************************************************
 * drivers/wireless/cc3000.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *   		 David_s5 <david_s5@nscdg.com>
 *
 * References:
 *   CC30000 from Texas Instruments http://processors.wiki.ti.com/index.php/CC3000
 *
 * See also:
 * 		http://processors.wiki.ti.com/index.php/CC3000_Host_Driver_Porting_Guide
 * 		http://processors.wiki.ti.com/index.php/CC3000_Host_Programming_Guide
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

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>

#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/include/cc3000_upif.h> // For Lowevel SPI config
#include <nuttx/wireless/cc3000/cc3000_common.h>
#include <nuttx/wireless/cc3000/hci.h>
#include "cc3000.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low-level SPI helpers */


#ifdef CONFIG_SPI_OWNBUS
#  define cc3000_lock(spi)
#  define cc3000_unlock(spi)
#else
static void cc3000_lock(FAR struct spi_dev_s *spi);
static void cc3000_unlock(FAR struct spi_dev_s *spi);
#endif


/* Interrupts and data sampling */

static void cc3000_notify(FAR struct cc3000_dev_s *priv);
static void cc3000_worker(FAR void *arg);
static int cc3000_interrupt(int irq, FAR void *context);

/* Character driver methods */

static int cc3000_open(FAR struct file *filep);
static int cc3000_close(FAR struct file *filep);
static ssize_t cc3000_read(FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t cc3000_write(FAR struct file *filep, FAR const char *buffer, size_t len);
static int cc3000_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int cc3000_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations cc3000_fops =
{
  cc3000_open,    	/* open */
  cc3000_close,   	/* close */
  cc3000_read,    	/* read */
  cc3000_write,    	/* write */
  0,                /* seek */
  cc3000_ioctl,		/* ioctl */
#ifndef CONFIG_DISABLE_POLL
  cc3000_poll  		/* poll */
#endif
};

/* If only a single CC3000 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_CC3000_MULTIPLE
static struct cc3000_dev_s g_cc3000;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct cc3000_dev_s *g_cc3000list;
#endif

uint8_t spi_readCommand[] = READ_COMMAND;


/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Function: cc3000_configspi
 *
 * Description:
 *   Configure the SPI for use with the CC3000.  This function should be
 *   called to configure the SPI
 *   bus.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static inline void cc3000_configspi(FAR struct spi_dev_s *spi)
{
	  idbg("Mode: %d Bits: 8 Frequency: %d\n",CONFIG_CC3000_SPIMODE, CONFIG_CC3000_FREQUENCY);
	  SPI_SELECT(spi, SPIDEV_WIRELESS, true);
	  SPI_SETMODE(spi, CONFIG_CC3000_SPIMODE);
	  SPI_SETBITS(spi, 8);
	  SPI_SETFREQUENCY(spi, CONFIG_CC3000_SPI_FREQUENCY);
	  SPI_SELECT(spi, SPIDEV_WIRELESS, false);

}


/****************************************************************************
 * Function: cc3000_lock
 *
 * Description:
 *   Lock the SPI bus and re-configure as necessary.  This function must be
 *   to assure: (1) exclusive access to the SPI bus, and (2) to assure that
 *   the shared bus is properly configured for the cc3000 module.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static void cc3000_lock(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus because there are multiple devices competing for the
   * SPI bus
   */

  /* Lock the SPI bus so that we have exclusive access */
  (void)SPI_LOCK(spi, true);

  /* We have the lock.  Now make sure that the SPI bus is configured for the
   * CC3000 (it might have gotten configured for a different device while
   * unlocked)
   */
  cc3000_configspi(spi);
}
#endif

/****************************************************************************
 * Function: cc3000_unlock
 *
 * Description:
 *   If we are sharing the SPI bus with other devices (CONFIG_SPI_OWNBUS
 *   undefined) then we need to un-lock the SPI bus for each transfer,
 *   possibly losing the current configuration.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static void cc3000_unlock(FAR struct spi_dev_s *spi)
{
  /* Relinquish the SPI bus. */

  (void)SPI_LOCK(spi, false);
}
#endif

/****************************************************************************
 * Function: cc3000_wait_irq
 *
 * Description:
 *  Helper function to wait on the readysem signaled by the interrupt
 *
 * Parameters:
 *   priv - Reference to the CC3000 driver structure
 *
 * Returned Value:
 *   0 - Semaphore signaled and devsem retaken
 *	< 0 - Some Error occurred
 * Assumptions:
 *   Own the devsem on entry
 *
 ****************************************************************************/
static int cc3000_wait_irq(FAR struct cc3000_dev_s *priv)
{
  int ret;

  // Give up
  sched_lock();
  sem_post(&priv->devsem);

  // Wait on first IRQ t come after Power Up
  ret = sem_wait(&priv->readysem);
  sched_unlock();

  if (ret >= 0)
  {
	/* Yes... then retake the mutual exclusion semaphore */

	  ret = sem_wait(&priv->devsem);
  }

/* Was the semaphore wait successful? Did we successful re-take the
 * mutual exclusion semaphore?
 */

  if (ret < 0)
  {
	/* No.. One of the two sem_wait's failed. */
	  ret = -errno;
  }
  return ret;
}

/****************************************************************************
 * Name: cc3000_notify
 ****************************************************************************/

static void cc3000_notify(FAR struct cc3000_dev_s *priv)
{
#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the CC3000
       * is no longer available.
       */

      sem_post(&priv->waitsem);
    }

  /* If there are threads waiting on poll() for CC3000 data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

#ifndef CONFIG_DISABLE_POLL
  for (i = 0; i < CONFIG_CC3000_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          ivdbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
#endif
}


/****************************************************************************
 * Name: cc3000_schedule
 ****************************************************************************/

static int cc3000_schedule(FAR struct cc3000_dev_s *priv)
{
  FAR struct cc3000_config_s *config;
  int                           ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

    DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, cc3000_worker, priv, 0);
  if (ret != 0)
    {
      illdbg("Failed to queue work: %d\n", ret);
    }

  return OK;
}


/****************************************************************************
 * Name: cc3000_worker
 ****************************************************************************/

static void cc3000_worker(FAR void *arg)
{
  FAR struct cc3000_dev_s    *priv = (FAR struct cc3000_dev_s *)arg;
  FAR struct cc3000_config_s *config;
  int                         ret;

  ASSERT(priv != NULL);

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

    /* Get exclusive access to the driver data structure */

  do
    {
      ret = sem_wait(&priv->devsem);

      /* This should only fail if the wait was canceled by an signal
       * (and the worker thread will receive a lot of signals).
       */

      DEBUGASSERT(ret == OK || errno == EINTR);
    }
  while (ret < 0);

  switch (priv->state) {

  case eSPI_STATE_POWERUP:
	  // Signal the device has interrupted after power up
	  priv->state = eSPI_STATE_INITIALIZED;
	  sem_post(&priv->readysem);
  break;

  case eSPI_STATE_WRITE_WAIT_IRQ:
	  // Signal the device has interrupted after Chip Select During a write operation
	  priv->state = eSPI_STATE_WRITE_PROCEED;
	  sem_post(&priv->readysem);
  break;


  case eSPI_STATE_WRITE_DONE:	// IRQ post a write => Solicited
  case eSPI_STATE_IDLE: // IRQ when Idel => cc3000 has data for the hosts Unsolicited
	{
		  uint16_t data_to_recv;
		  priv->state = eSPI_STATE_READ_IRQ;
		  // Issue the read command
		  cc3000_lock(priv->spi); // Assert CS
		  priv->state = eSPI_STATE_READ_PROCEED;
		  SPI_EXCHANGE(priv->spi,spi_readCommand,priv->rx_buffer, ARRAY_SIZE(spi_readCommand));
		  // Extract Length bytes from Rx Buffer
          STREAM_TO_UINT16((char*)priv->rx_buffer, READ_OFFSET_TO_LENGTH, data_to_recv);
          // We will read ARRAY_SIZE(spi_readCommand) + data_to_recv. is it odd?
          if ((data_to_recv +  ARRAY_SIZE(spi_readCommand)) & 1) {
        	  // odd so make it even
        	  data_to_recv++;
          }
          // Read the whole payload in at the beginning of the buffer
          if (data_to_recv) {
              // Will it fit?
              DEBUGASSERT(data_to_recv < ARRAY_SIZE(priv->rx_buffer));
              SPI_RECVBLOCK(priv->spi, priv->rx_buffer, data_to_recv);
          }
		  cc3000_unlock(priv->spi); // De assert CS
		  // Disable IrQ as the wl code will resume via CC3000IOC_COMPLETE
		  priv->config->irq_enable(priv->config,false);
		  priv->state = eSPI_STATE_READ_READY;
		  priv->rx_buffer_len = data_to_recv;

		  ret = mq_send(priv->queue, priv->rx_buffer, data_to_recv, 1);
		  DEBUGASSERT(ret>=0);

	}
	break;
    default:
    	PANIC();
    	break;
	}

  /* Notify any waiters that new CC3000 data is available */

  cc3000_notify(priv);

  sem_post(&priv->devsem);
}

/****************************************************************************
 * Name: cc3000_interrupt
 ****************************************************************************/

static int cc3000_interrupt(int irq, FAR void *context)
{
  FAR struct cc3000_dev_s    *priv;
  int                           ret;

  /* Which CC3000 device caused the interrupt? */

#ifndef CONFIG_CC3000_MULTIPLE
  priv = &g_cc3000;
#else
  for (priv = g_cc3000list;
       priv && priv->configs->irq != irq;
       priv = priv->flink);

  ASSERT(priv != NULL);
#endif

  /* Schedule sampling to occur on the worker thread */
  ret = cc3000_schedule(priv);

  /* Clear any pending interrupts and return success */

  priv->config->irq_clear(priv->config);
  return ret;
}

/****************************************************************************
 * Name: cc3000_open
 ****************************************************************************/

static int cc3000_open(FAR struct file *filep)
{
  FAR struct inode         *inode;
  struct mq_attr attr;
  char queuename[QUEUE_NAMELEN];
  FAR struct cc3000_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  ivdbg("Opening\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */


  if (tmp==1) {

	  attr.mq_maxmsg  = 2;
	  attr.mq_msgsize = CC3000_RX_BUFFER_SIZE;
	  attr.mq_flags   = 0;

	  /* Set the flags for the open of the queue.
	   * Make it a blocking open on the queue, meaning it will block if
	   * this process tries to send to the queue and the queue is full.
	   *
	   *   O_CREAT - the queue will get created if it does not already exist.
	   *   O_WRONLY - we are only planning to write to the queue.
	   *
	   * Open the queue, and create it if the receiving process hasn't
	   * already created it.
	   */
	  snprintf(queuename, QUEUE_NAMELEN, QUEUE_FORMAT, priv->minor);
	  priv->queue = mq_open(queuename,O_WRONLY|O_CREAT, 0666, &attr);
	  if (priv->queue < 0)
	    {
		  priv->crefs--;
	      ret = -errno;
	      goto errout_with_sem;
	    }

	  priv->config->irq_clear(priv->config);
	  priv->config->irq_enable(priv->config, true);
	  priv->config->power_enable(priv->config, true);
	  // Bring the device Online A) on http://processors.wiki.ti.com/index.php/File:CC3000_Master_SPI_Write_Sequence_After_Power_Up.png
  }
  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_sem:
  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cc3000_close
 ****************************************************************************/

static int cc3000_close(FAR struct file *filep)
{
  FAR struct inode         *inode;
  FAR struct cc3000_dev_s *priv;
  int                       ret;

  ivdbg("Closing\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  int tmp = priv->crefs;
  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  if (tmp == 0)
    {
	  priv->config->irq_enable(priv->config, false);
	  priv->config->irq_clear(priv->config);
	  priv->config->power_enable(priv->config, false);
	  mq_close(priv->queue);
	  priv->queue = 0;
    }

  sem_post(&priv->devsem);
  return OK;

}

/****************************************************************************
 * Name: cc3000_read
 ****************************************************************************/
static ssize_t cc3000_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode          *inode;
  FAR struct cc3000_dev_s *priv;
  int                        ret;
  ssize_t                    nread;

  ivdbg("buffer:%p len:%d\n", buffer, len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the maximum data.
   */

  if (len < CC3000_RX_BUFFER_SIZE)
	{
	  idbg("Unsupported read size: %d\n", len);
	  nread = -ENOSYS;
	  goto errout_without_sem;

	}

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);

  if (ret < 0)
	{
	  /* This should only happen if the wait was canceled by an signal */

	  idbg("sem_wait: %d\n", errno);
	  DEBUGASSERT(errno == EINTR);
	  nread = -EINTR;
	  goto errout_without_sem;
	}

  for (nread = priv->rx_buffer_len; nread == 0; nread = priv->rx_buffer_len)
	{

	  if (nread > 0) {
        // Yes.. break out to return what we have.
             break;
      }

	  /* data is not available now.  We would have to wait to get
	   * receive sample data.  If the user has specified the O_NONBLOCK
	   * option, then just return an error.
	   */

	  ivdbg("CC3000 data is not available\n");
	  if (filep->f_oflags & O_NONBLOCK)
		{
		  nread = -EAGAIN;
		  break;
	   }

      /* Otherwise, wait for something to be written to the
       * buffer. Increment the number of waiters so that the notify
       * will know that it needs to post the semaphore to wake us up.
       */

	  sched_lock();
	  priv->nwaiters++;
	  sem_post(&priv->devsem);

	  /* We may now be pre-empted!  But that should be okay because we
       * have already incremented nwaiters.  Pre-emptions is disabled
       * but will be re-enabled while we are waiting.
       */

	  ivdbg("Waiting..\n");
	  ret = sem_wait(&priv->waitsem);
	  priv->nwaiters--;
      sched_unlock();


      /* Did we successfully get the waitsem? */

      if (ret >= 0)
        {
          /* Yes... then retake the mutual exclusion semaphore */

    	  ret = sem_wait(&priv->devsem);
        }

      /* Was the semaphore wait successful? Did we successful re-take the
       * mutual exclusion semaphore?
       */

      if (ret < 0)
        {
          /* No.. One of the two sem_wait's failed. */

          int errval = errno;

          /* Were we awakened by a signal?  Did we read anything before
           * we received the signal?
           */

          if (errval != EINTR || nread  >= 0)
            {
              /* Yes.. return the error. */

        	  nread = -errval;
            }

          /* Break out to return what we have.  Note, we can't exactly
           * "break" out because whichever error occurred, we do not hold
           * the exclusion semaphore.
           */

          goto errout_without_sem;
        }
	}

	sem_post(&priv->devsem);

errout_without_sem:
  ivdbg("Returning: %d\n", nread);
#ifndef CONFIG_DISABLE_POLL
  if (nread > 0)
    {
      cc3000_pollnotify(priv, POLLOUT);
    }
#endif

  return nread;
}

/****************************************************************************
 * Name:cc3000_write
 *
 * Bit of non standard buffer management ahead
 * The buffer is memory allocated in the user space with space for the spi header
 ****************************************************************************/
static ssize_t cc3000_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
	  FAR struct inode          *inode;
	  FAR struct cc3000_dev_s 	*priv;
	  int                       ret;
	  ssize_t					nwritten = 0;
	  // Set the padding if count(buffer) is even ( as it will be come odd with header)
	  size_t					tx_len = (len & 1) ? len : len +1;

	  ivdbg("buffer:%p len:%d tx_len:%d\n", buffer, len, tx_len );

	  DEBUGASSERT(filep);
	  inode = filep->f_inode;

	  DEBUGASSERT(inode && inode->i_private);
	  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

	  /* Get exclusive access to the driver data structure */

	  ret = sem_wait(&priv->devsem);
	  if (ret < 0)
	    {
	      /* This should only happen if the wait was canceled by an signal */

	      idbg("sem_wait: %d\n", errno);
	      DEBUGASSERT(errno == EINTR);
	      nwritten = -EINTR;
	      goto errout_without_sem;
	    }


	  //
	  // Figure out the total length of the packet in order to figure out if there is padding or not
	  //
	  memcpy(priv->tx_buffer,buffer,tx_len);
	  priv->tx_buffer[0] = WRITE;
	  priv->tx_buffer[1] = HI(tx_len);
	  priv->tx_buffer[2] = LO(tx_len);
	  priv->tx_buffer[3] = 0;
	  priv->tx_buffer[4] = 0;

	  tx_len += SPI_HEADER_SIZE;

      /*  The first write transaction to occur after release of the shutdown has slightly different timing than the others.
	   *  The normal Master SPI write sequence is nCS low, followed by IRQ low (CC3000 host), indicating that
	   *  the CC3000 core device is ready to accept data. However, after power up the sequence is slightly different,
	   *  as shown in the following Figure: http://processors.wiki.ti.com/index.php/File:CC3000_Master_SPI_Write_Sequence_After_Power_Up.png
	   *  The following is a sequence of operations:
	   *  The master detects the IRQ line low: in this case the detection of
	   *  IRQ low does not indicate the intention of the CC3000 device to communicate with the
	   *  master but rather CC3000 readiness after power up.
	   *  The master asserts nCS.
	   *  The master introduces a delay of at least 50 μs before starting actual transmission of data.
	   *  The master transmits the first 4 bytes of the SPI header.
	   *  The master introduces a delay of at least an additional 50 μs.
	   *  The master transmits the rest of the packet.
	   */
	  if (priv->state == eSPI_STATE_POWERUP)
	    {
		  ret = cc3000_wait_irq(priv);
		   if (ret < 0) {
        	  nwritten = ret;
	          goto errout_without_sem;
	        }
	    }

	  if (priv->state  == eSPI_STATE_INITIALIZED)
	    {

		  	  cc3000_lock(priv->spi); // Assert CS
		  	  usleep(50);
		  	  SPI_SNDBLOCK(priv->spi, buffer, 4);
		  	  usleep(50);
		  	  SPI_SNDBLOCK(priv->spi, buffer+4, tx_len-4);
	    }
	  else
	    {
	  	  priv->state  = eSPI_STATE_WRITE_WAIT_IRQ;
		  cc3000_lock(priv->spi); // Assert CS
		  // Wait on IRQ
		  ret = cc3000_wait_irq(priv);
		  if (ret < 0)
		    {
		      /* This should only happen if the wait was canceled by an signal */
		  	  cc3000_unlock(priv->spi);
		      idbg("sem_wait: %d\n", errno);
		      DEBUGASSERT(errno == EINTR);
        	  nwritten = ret;
	          goto errout_without_sem;
		    }
	  	  	SPI_SNDBLOCK(priv->spi, buffer, tx_len);
	    }
	  priv->state  = eSPI_STATE_WRITE_DONE;
	  cc3000_unlock(priv->spi);
	  nwritten = tx_len;
	  sem_post(&priv->devsem);

errout_without_sem:
	  ivdbg("Returning: %d\n", ret);
	  return nwritten;
}

/****************************************************************************
 * Name:cc3000_ioctl
 ****************************************************************************/

static int cc3000_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct cc3000_dev_s *priv;
  int                       ret;

  ivdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Process the IOCTL by command */
  switch (cmd)
    {
      case CC3000IOC_COMPLETE:  /* arg: Pointer to uint32_t frequency value */
        {
          DEBUGASSERT(priv->config);
		  priv->state = eSPI_STATE_IDLE;
          priv->config->irq_enable(priv->config,true);
        }
        break;

      case CC3000IOC_GETQUEID:
        {
           FAR int *pid = (FAR int *)(arg);
           DEBUGASSERT(pid != NULL);
           *pid = priv->minor;
           break;
        }


      default:
        ret = -ENOTTY;
        break;
    }
  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cc3000_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int cc3000_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode         *inode;
  FAR struct cc3000_dev_s *priv;
  int                       ret = OK;
  int                       i;

  ivdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_CC3000_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_CC3000_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          cc3000_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc3000_register
 *
 * Description:
 *   Configure the CC3000 to use the provided SPI device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An SPI driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int cc3000_register(FAR struct spi_dev_s *spi,
                      FAR struct cc3000_config_s *config, int minor)
{
  FAR struct cc3000_dev_s *priv;
  char devname[DEV_NAMELEN];

#ifdef CONFIG_CC3000_MULTIPLE
  irqstate_t flags;
#endif
  int ret;

  ivdbg("spi: %p minor: %d\n", spi, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(spi != NULL && config != NULL && minor >= 0 && minor < 100);

  /* Create and initialize a CC3000 device driver instance */

#ifndef CONFIG_CC3000_MULTIPLE
  priv = &g_cc3000;
#else
  priv = (FAR struct cc3000_dev_s *)kmalloc(sizeof(struct cc3000_dev_s));
  if (!priv)
    {
      idbg("kmalloc(%d) failed\n", sizeof(struct cc3000_dev_s));
      return -ENOMEM;
    }
#endif

  /* Initialize the CC3000 device driver instance */

  memset(priv, 0, sizeof(struct cc3000_dev_s));

  priv->minor = minor;				  /* Save the minor number */
  priv->spi     = spi;               /* Save the SPI device handle */
  priv->config  = config;            /* Save the board configuration */

  sem_init(&priv->devsem,  0, 1);    /* Initialize device structure semaphore */
  sem_init(&priv->waitsem, 0, 0);    /* Initialize  event wait semaphore */
  sem_init(&priv->readysem, 0, 0);    /* Initialize  IRQ Ready semaphore */

  /* Make sure that interrupts are disabled */

  config->irq_clear(config);
  config->irq_enable(config, false);

  /* Attach the interrupt handler */

  ret = config->irq_attach(config, cc3000_interrupt);
  if (ret < 0)
    {
      idbg("Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  /* Register the device as an input device */

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ivdbg("Registering %s\n", devname);

  ret = register_driver(devname, &cc3000_fops, 0666, priv);
  if (ret < 0)
    {
      idbg("register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }


  /* If multiple CC3000 devices are supported, then we will need to add
   * this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the recieved IRQ number.
   */

#ifdef CONFIG_CC3000_MULTIPLE
  priv->flink    = g_cc3000list;
  g_cc3000list = priv;
  irqrestore(flags);
#endif

  /* And return success (?) */

  return OK;

errout_with_priv:
  sem_destroy(&priv->devsem);
#ifdef CONFIG_CC3000_MULTIPLE
  kfree(priv);
#endif
  return ret;
}
