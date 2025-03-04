/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_i2c.c
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <sys/wait.h>
#include <time.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "bcm2711_gpio.h"
#include "bcm2711_i2c.h"
#include "chip.h"
#include "hardware/bcm2711_bsc.h"

#if defined(CONFIG_BCM2711_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeout duration in ms for transfers waiting on interrupt handler. */

#define I2C_TIMEOUT_MS 50

/* Default bus frequency at 400kbps. */

#define I2C_DEFAULT_FREQUENCY 400000

/* Core clock nominal frequency in Hz */

#define CORE_CLOCK_FREQUENCY 150000000

/* Get divisor for desired I2C bus frequency */

#define CLK_DIVISOR(freq) (CORE_CLOCK_FREQUENCY / (freq))

/* The FIFO buffer length of the I2C interfaces */

#define FIFO_DEPTH 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* BCM2711 I2C device. */

struct bcm2711_i2cdev_s
{
  struct i2c_master_s dev; /* Generic I2C device */
  uint32_t base;           /* Base address of I2C interface */
  uint8_t port;            /* Port number */

  mutex_t lock;       /* Exclusive access */
  sem_t wait;         /* Wait for transfer completion */
  uint32_t frequency; /* I2C bus frequency */

  struct i2c_msg_s *msgs; /* Messages to send */
  size_t reg_buff_offset; /* Offset into message buffer */
  uint8_t rw_size;        /* max(FIFO_DEPTH, remaining message size) */
  bool done;              /* If `wait` was posted due to done condition. */

  int err;  /* Error status of transfers */
  int refs; /* Reference count */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcm2711_i2c_semtimedwait(struct bcm2711_i2cdev_s *priv,
                                    uint32_t ms);
static void bcm2711_i2c_clearfifos(struct bcm2711_i2cdev_s *priv);
static void bcm2711_i2c_setfrequency(struct bcm2711_i2cdev_s *priv,
                                     uint32_t frequency);
static void bcm2711_i2c_setaddr(struct bcm2711_i2cdev_s *priv,
                                uint16_t addr);
static void bcm2711_i2c_starttransfer(struct bcm2711_i2cdev_s *priv);
static int bcm2711_i2c_interrupted(struct bcm2711_i2cdev_s *priv);
static void bcm2711_i2c_disable(struct bcm2711_i2cdev_s *priv);
static void bcm2711_i2c_enable(struct bcm2711_i2cdev_s *priv);
static void bcm2711_i2c_drainrxfifo(struct bcm2711_i2cdev_s *priv);
static int bcm2711_i2c_send(struct bcm2711_i2cdev_s *priv, bool stop);
static int bcm2711_i2c_receive(struct bcm2711_i2cdev_s *priv, bool stop);

static int bcm2711_i2c_secondary_handler(struct bcm2711_i2cdev_s *priv);

static int bcm2711_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int bcm2711_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* True if the IRQ for all I2C interrupts has been set up. */

static bool g_i2c_irqinit = false;

/* The number of I2C interfaces currently initialized. */

static uint8_t g_i2c_devsinit = 0;

/* I2C operations for BCM2711 I2C interfaces. */

struct i2c_ops_s bcm2711_i2c_ops =
{
  .transfer = bcm2711_i2c_transfer,
#if defined(CONFIG_I2C_RESET)
  .reset = bcm2711_i2c_reset,
#endif // defined(CONFIG_I2C_RESET)
};

#ifdef CONFIG_BCM2711_I2C1

/* I2C1 interface. */

static struct bcm2711_i2cdev_s g_i2c1dev =
{
  .base = BCM_BSC1,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .port = 1,
  .refs = 0,
  .err = 0,
  .done = false,
};

#endif // CONFIG_BCM2711_I2C1

/* I2C interfaces */

static struct bcm2711_i2cdev_s *g_i2c_devices[BCM_BSCS_NUM] =
{
#ifdef CONFIG_BCM2711_I2C0
#warning "I2C0 unsupported"
#else
    [0] = NULL,
#endif // CONFIG_BCM2711_I2C0

#ifdef CONFIG_BCM2711_I2C1
    [1] = &g_i2c1dev,
#else
    [1] = NULL,
#endif // CONFIG_BCM2711_I2C1

#ifdef CONFIG_BCM2711_I2C2
#warning "I2C2 unsupported"
#else
    [2] = NULL,
#endif // CONFIG_BCM2711_I2C2

#ifdef CONFIG_BCM2711_I2C3
#warning "I2C3 unsupported"
#else
    [3] = NULL,
#endif // CONFIG_BCM2711_I2C3

#ifdef CONFIG_BCM2711_I2C4
#warning "I2C4 unsupported"
#else
    [4] = NULL,
#endif // CONFIG_BCM2711_I2C4

#ifdef CONFIG_BCM2711_I2C5
#warning "I2C5 unsupported"
#else
    [5] = NULL,
#endif // CONFIG_BCM2711_I2C5

#ifdef CONFIG_BCM2711_I2C6
#warning "I2C6 unsupported"
#else
    [6] = NULL,
#endif // CONFIG_BCM2711_I2C6
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* TODO: remove */

static void bcm2711_i2c_print(struct bcm2711_i2cdev_s *priv)
{
  i2cerr("Port: %d\n", priv->port);
  i2cerr("Err: %d\n", priv->err);
  i2cerr("Refs: %d\n", priv->refs);
  i2cerr("Done: %s\n", priv->done ? "true" : "false");
  int val;
  nxsem_get_value(&priv->wait, &val);
  i2cerr("Semaphore: %d\n", val);
  i2cerr("Frequency: %d\n", priv->frequency);
  i2cerr("Cur msg: %p\n", priv->msgs);
  if (priv->msgs != NULL)
    {
      i2cerr("Cur msg len: %ld\n", priv->msgs->length);
    }
}

/****************************************************************************
 * Name: bcm2711_i2c_semtimedwait
 *
 * Description:
 *   Wait on the I2C interface semaphore for a given number of milliseconds.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to wait on.
 *     ms - The number of milliseconds before timeout
 *
 * Returns:
 *     0 for success, negated errno otherwise.
 *
 ****************************************************************************/

static int bcm2711_i2c_semtimedwait(struct bcm2711_i2cdev_s *priv,
                                    uint32_t ms)
{
  struct timespec future;
  int ret = 0;

  ret = clock_gettime(CLOCK_REALTIME, &future);
  if (ret < 0)
    {
      i2cerr("Timed wait failed on I2C%u.\n", priv->port);
      return ret;
    }

  future.tv_sec += (ms / 1000);
  future.tv_nsec += ((ms % 1000) * 1000);

  return nxsem_timedwait_uninterruptible(&priv->wait, &future);
}

/****************************************************************************
 * Name: bcm2711_i2c_clearfifos
 *
 * Description:
 *   Clear the FIFOs of the I2C interface.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to clear the FIFOs of.
 *
 ****************************************************************************/

static void bcm2711_i2c_clearfifos(struct bcm2711_i2cdev_s *priv)
{
  modreg32(0, BCM_BSC_C_CLRFIFO, BCM_BSC_C(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to set the frequency of.
 *     frequency - The new frequency.
 *
 ****************************************************************************/

static void bcm2711_i2c_setfrequency(struct bcm2711_i2cdev_s *priv,
                                     uint32_t frequency)
{
  putreg32(CLK_DIVISOR(frequency), BCM_BSC_DIV(priv->base));
  priv->frequency = frequency;
}

/****************************************************************************
 * Name: bcm2711_i2c_setaddr
 *
 * Description:
 *   Set the slave address for the next transfer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to set the slave address on.
 *     addr - The slave address.
 *
 ****************************************************************************/

static void bcm2711_i2c_setaddr(struct bcm2711_i2cdev_s *priv, uint16_t addr)
{
  /* TODO: handle 10-bit addresses */

  putreg32(addr & 0x7f, BCM_BSC_A(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_starttransfer
 *
 * Description:
 *   Start the next transfer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to start the transfer on.
 *
 ****************************************************************************/

static void bcm2711_i2c_starttransfer(struct bcm2711_i2cdev_s *priv)
{
  i2cinfo("Transfer started\n");
  modreg32(BCM_BSC_C_ST, BCM_BSC_C_ST, BCM_BSC_C(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_interrupted
 *
 * Description:
 *   Checks if the I2C device has a pending interrupt.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to check for a pending interrupt.
 *
 * Returns:
 *     0 if no interrupt pending, non-zero if interrupt pending.
 *
 ****************************************************************************/

static int bcm2711_i2c_interrupted(struct bcm2711_i2cdev_s *priv)
{
  return getreg32(BCM_BSC_S(priv->base)) &
         (BCM_BSC_S_DONE | BCM_BSC_S_RXR | BCM_BSC_S_TXW);
}

/****************************************************************************
 * Name: bcm2711_i2c_disable
 *
 * Description:
 *   Disable the I2C interface.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to disable.
 *
 ****************************************************************************/

static void bcm2711_i2c_disable(struct bcm2711_i2cdev_s *priv)
{
  i2cinfo("Disabled I2C%u\n", priv->port);

  /* Disable interrupts */

  modreg32(0, BCM_BSC_C_INTD, BCM_BSC_C(priv->base));
  modreg32(0, BCM_BSC_C_INTR, BCM_BSC_C(priv->base));
  modreg32(0, BCM_BSC_C_INTT, BCM_BSC_C(priv->base));

  /* Clear FIFO */

  bcm2711_i2c_clearfifos(priv);

  /* Disable interface */

  modreg32(0, BCM_BSC_C_I2CEN, BCM_BSC_C(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_enable
 *
 * Description:
 *   Enable the I2C interface.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to enable.
 *
 ****************************************************************************/

static void bcm2711_i2c_enable(struct bcm2711_i2cdev_s *priv)
{
  /* Enable interface */

  modreg32(BCM_BSC_C_I2CEN, BCM_BSC_C_I2CEN, BCM_BSC_C(priv->base));

  /* Clear FIFO */

  bcm2711_i2c_clearfifos(priv);

  /* Enable interrupts */

  modreg32(BCM_BSC_C_INTD, BCM_BSC_C_INTD, BCM_BSC_C(priv->base));
  modreg32(BCM_BSC_C_INTR, BCM_BSC_C_INTR, BCM_BSC_C(priv->base));
  modreg32(BCM_BSC_C_INTT, BCM_BSC_C_INTT, BCM_BSC_C(priv->base));

  i2cinfo("Enabled I2C%u\n", priv->port);
}

/****************************************************************************
 * Name: bcm2711_i2c_drainrxfifo
 *
 * Description:
 *   Drain the RX FIFO into the receive message buffer of the I2C device.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to receive on.
 *
 ****************************************************************************/

static void bcm2711_i2c_drainrxfifo(struct bcm2711_i2cdev_s *priv)
{
  struct i2c_msg_s *msg = priv->msgs;
  uint32_t status_addr = BCM_BSC_S(priv->base);
  uint32_t fifo_addr = BCM_BSC_FIFO(priv->base);
  size_t i;

  DEBUGASSERT(msg != NULL);

  /* While the RX FIFO contains data to be received, drain it into the
   * message buffer. Do not read more than the `rw_size` to avoid
   * overflowing the message buffer.
   */

  for (i = 0; (i < priv->rw_size) &&
       (getreg32(status_addr) & BCM_BSC_S_RXD);
      )
    {
      msg->buffer[priv->reg_buff_offset + i] = getreg32(fifo_addr) & 0xff;
      i++;
    }

  /* We have either reached the rw_size or the RX FIFO is out of data.
   * Update the buffer offset with the amount of data we have read.
   */

  priv->reg_buff_offset += i;
}

/****************************************************************************
 * Name: bcm2711_i2c_receive
 *
 * Description:
 *   Receive I2C data.
 *
 * Input Parameters:
 *     dev - The I2C interface to receive on.
 *     stop - Whether to send a stop condition at the end of the transfer.
 ****************************************************************************/

static int bcm2711_i2c_receive(struct bcm2711_i2cdev_s *priv, bool stop)
{
  struct i2c_msg_s *msg = priv->msgs;
  ssize_t msg_length;
  int ret = 0;

  DEBUGASSERT(msg != NULL);

  /* Set read bit */

  modreg32(BCM_BSC_C_READ, BCM_BSC_C_READ, BCM_BSC_C(priv->base));

  /* Set message length */

  putreg32(msg->length, BCM_BSC_DLEN(priv->base));

  /* Start buffer fresh for receiving full message. */

  priv->reg_buff_offset = 0;
  msg_length = msg->length;

  /* Start transfer. */

  bcm2711_i2c_starttransfer(priv);

  /* Handle special 0 byte read case by waiting for DONE signal. */

  if (msg->length == 0)
    {
      ret = bcm2711_i2c_semtimedwait(priv, I2C_TIMEOUT_MS);
    }

  /* Continuously read until message has been completely read. */

  while (msg_length > 0)
    {
      /* Read maximum FIFO depth or the remaining message length. */

      if (msg_length <= FIFO_DEPTH)
        {
          priv->rw_size = msg_length;
        }
      else
        {
          priv->rw_size = FIFO_DEPTH;
        }

      /* Wait here for interrupt handler to signal that RX FIFO has data.
       * We can then continue reading.
       */

      ret = bcm2711_i2c_semtimedwait(priv, I2C_TIMEOUT_MS);
      if (ret < 0)
        {
          return ret;
        }

      /* The semaphore was posted without a timeout, so we have to handle
       * some reading.
       */

      if (priv->err != 0)
        {
          return priv->err;
        }

      bcm2711_i2c_drainrxfifo(priv);

      /* The remaining message length is the total length minus how far into
       * the message we are.
       */

      msg_length = msg->length - priv->reg_buff_offset;
    }

  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_filltxfifo
 *
 * Description:
 *   Fill the TX FIFO with data to be sent.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to send on.
 *
 ****************************************************************************/

static void bcm2711_i2c_filltxfifo(struct bcm2711_i2cdev_s *priv)
{
  struct i2c_msg_s *msg = priv->msgs;
  uint32_t status_addr = BCM_BSC_S(priv->base);
  uint32_t fifo_addr = BCM_BSC_FIFO(priv->base);
  size_t i;

  DEBUGASSERT(msg != NULL);

  /* While there is data to be sent, and the TX FIFO is not full, write the
   * data to the TX FIFO. Stop when full or data stream is over.
   */

  for (i = 0; (i < priv->rw_size) &&
       (getreg32(status_addr) & BCM_BSC_S_TXD);
      )
    {
      putreg32(fifo_addr, msg->buffer[priv->reg_buff_offset + i] & 0xff);
      i++;
    }

  /* We have either reached the rw_size or the RX FIFO is out of data.
   * Update the buffer offset with the amount of data we have read.
   */

  priv->reg_buff_offset += i;
}

/****************************************************************************
 * Name: bcm2711_i2c_send
 *
 * Description:
 *   Send I2C data.
 *
 * Input Parameters:
 *     dev - The I2C interface to send on.
 *     stop - Whether to send a stop condition at the end of the transfer.
 ****************************************************************************/

static int bcm2711_i2c_send(struct bcm2711_i2cdev_s *priv, bool stop)
{
  struct i2c_msg_s *msg = priv->msgs;
  ssize_t msg_length;
  int ret = OK;

  DEBUGASSERT(msg != NULL);

  /* Set write bit */

  modreg32(0, BCM_BSC_C_READ, BCM_BSC_C(priv->base));

  /* Set message length */

  putreg32(msg->length, BCM_BSC_DLEN(priv->base));

  /* Start buffer fresh for sending message */

  priv->reg_buff_offset = 0;
  msg_length = msg->length;

  /* Start transfer. */

  bcm2711_i2c_starttransfer(priv);

  /* Send the entire message */

  do
    {
      /* Write maximum FIFO depth or the remaining message length. */

      if (msg_length <= FIFO_DEPTH)
        {
          priv->rw_size = msg_length;
        }
      else
        {
          priv->rw_size = FIFO_DEPTH;
        }

      /* Write data to FIFO. */

      i2cerr("Filling FIFO\n");
      bcm2711_i2c_filltxfifo(priv);

      /* The remaining message length is the total length minus how far into
       * the message we are.
       */

      msg_length = msg->length - priv->reg_buff_offset;

      /* Here we wait for an interrupt. There are two scenarios:
       *
       * 1) If there is still message data left to write AND we receive an
       * interrupt, then we can continue on another iteration of the outer
       * do-while loop to keep writing.
       *
       * 2) If there is no more message data left to write, then we need to
       * get a DONE interrupt to indicate the end of the transfer. If we
       * don't get a DONE interrupt, but instead get a TXW, we ignore it and
       * wait on the semaphore again. If we never get a done interrupt we
       * return the applicable error (timeout). If we do get a done
       * interrupt, we'll be able to exit the waiting loop.
       */

      while (msg_length == 0)
        {
          /* Wait for interrupt (timed). */

          ret = bcm2711_i2c_semtimedwait(priv, I2C_TIMEOUT_MS);

          /* First check for IO error because it's more important to report
           * that.
           */

          i2cerr("sem: %d\n", ret);
          i2cerr("dev: %d\n", priv->err);

          if (priv->err != 0)
            {
              return priv->err;
            }

          /* Now check if semaphore timed out. */

          if (ret < 0)
            {
              return ret;
            }

          /* Now check if the received interrupt was a done condition.
           * Otherwise we loop again.
           */

          if (priv->done)
            {
              priv->done = false;
              break;
            }
        }

      /* If we're here, the semaphore got posted with an interrupt and there
       * is still data left to write. Do another iteration.
       */
    }
  while (msg_length > 0);

  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers.
 *
 * Input Parameters:
 *     dev - The I2C master interface to transfer on.
 *     msgs - The messages to transfer.
 *     count - The number of messages to transfer.
 ****************************************************************************/

static int bcm2711_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count)
{
  struct bcm2711_i2cdev_s *priv = (struct bcm2711_i2cdev_s *)dev;
  int i;
  int ret = 0;
  bool stop = true;
  int semval = 0;

  DEBUGASSERT(dev != NULL);

  i2cerr("Device state:\n");
  bcm2711_i2c_print(priv);

  /* Get exclusive access before doing a transfer */

  nxmutex_lock(&priv->lock);

  /* If the semaphore value is not 0, we must be waiting on something, so a
   * transfer cannot be started. This state should never happen.
   */

  ret = nxsem_get_value(&priv->wait, &semval);
  DEBUGASSERT(ret == 0 && semval == 0);

  /* Perform send/receive operations for each message */

  for (i = 0; i < count; i++, msgs++)
    {
      /* Put message in device context */

      priv->msgs = msgs;
      priv->err = 0; /* No errors yet */
      priv->done = false;

      /* Configure I2C interface according to message. */

      bcm2711_i2c_disable(priv);
      bcm2711_i2c_setfrequency(priv, msgs->frequency);
      bcm2711_i2c_setaddr(priv, msgs->addr);
      bcm2711_i2c_clearfifos(priv);
      bcm2711_i2c_enable(priv);

      i2cinfo("I2C%u interface configured for message\n", priv->port);

      /* TODO: do I need to support I2C_M_NOSTART? */

      /* TODO: Support restart condition (no stop) */

      if (msgs->flags & I2C_M_NOSTOP)
        {
          stop = false;
        }
      else
        {
          stop = true;
        }

      /* Set read/write bit according to message configuration, and then
       * perform the corresponding operation.
       */

      if (msgs->flags & I2C_M_READ)
        {
          ret = bcm2711_i2c_receive(priv, stop);
        }
      else
        {
          ret = bcm2711_i2c_send(priv, stop);
        }

      /* Check if there was an error during the send/receive operation and
       * return early if so.
       */

      if (ret < 0)
        {
          break;
        }

      if (priv->err != 0)
        {
          ret = priv->err;
          break;
        }

      /* If no error occurred, we are here. TODO: Something about NULL `msgs`
       * for illegal access in interrupt.
       */
    }

  /* If our last message had a stop condition, we can safely disable this I2C
   * interface until it's used again.
   */

  if (stop)
    {
      bcm2711_i2c_disable(priv);
    }

  i2cerr("Device state:\n");
  bcm2711_i2c_print(priv);
  i2cerr("Return: %d\n", ret);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_primary_handler
 *
 * Description:
 *   Handle I2C interrupts.
 *
 * Input Parameters:
 *     irq - The IRQ number
 *     context - The interrupt context
 *     arg - The argument passed to the interrupt handler
 ****************************************************************************/

static int bcm2711_i2c_primary_handler(int irq, void *context, void *arg)
{
  int ret = 0;
  int tempret = 0;
  struct bcm2711_i2cdev_s *priv;

  /* Check all I2C interfaces for an interrupt */

  for (int i = 0; i < BCM_BSCS_NUM; i++)
    {
      priv = g_i2c_devices[i];
      if (priv == NULL)
        {
          continue;
        }

      /* If interface had interrupt triggered, call its handler. */

      if (bcm2711_i2c_interrupted(priv))
        {
          tempret = bcm2711_i2c_secondary_handler(priv);

          /* If there was an error, record it. Otherwise don't overwrite the
           * ret value, it may contain an error from a previous iteration
           * that we still need to report.
           */

          if (tempret < 0)
            {
              ret = tempret;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_secondary_handler
 *
 * Description:
 *   Handle I2C interrupts on a per-device basis.
 *
 * Input Parameters:
 *     priv - The I2C interface device which has an interrupt triggered.
 ****************************************************************************/

static int bcm2711_i2c_secondary_handler(struct bcm2711_i2cdev_s *priv)
{
  int ret = OK;
  uint32_t status;
  uint32_t status_addr;
  bool post_sem = false;

  /* Get interrupt status for this device. */

  status_addr = BCM_BSC_S(priv->base);
  status = getreg32(status_addr);

  /* Decide what to do with the status */

  /* There was an ACK error */

  if (status & BCM_BSC_S_ERR)
    {
      modreg32(BCM_BSC_S_ERR, BCM_BSC_S_ERR,
               status_addr); /* Acknowledge err */
      priv->err = -EIO;
      ret = -EIO;
      post_sem = true;
    }

  /* There was a clock stretch timeout */

  if (status & BCM_BSC_S_CLKT)
    {
      modreg32(BCM_BSC_S_CLKT, BCM_BSC_S_CLKT,
               status_addr); /* Acknowledge err */
      priv->err = -EIO;
      ret = -EIO;
      post_sem = true;
    }

  /* RX FIFO needs reading */

  if (status & BCM_BSC_S_RXR)
    {
      post_sem = true;
    }

  /* TX FIFO needs writing */

  if (status & BCM_BSC_S_TXW)
    {
      i2cerr("Need w\n");
      post_sem = true;
    }

  /* Transfer is done */

  if (status & BCM_BSC_S_DONE)
    {
      i2cerr("DONE");
      priv->done = true;
      modreg32(BCM_BSC_S_DONE, BCM_BSC_S_DONE, status_addr);
      post_sem = true;
    }

  if (post_sem)
    {
      nxsem_post(&priv->wait);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device for the BCM2711.
 *
 * Input parameters:
 *     port - The bus number for the I2C interface.
 *
 ****************************************************************************/

struct i2c_master_s *bcm2711_i2cbus_initialize(int port)
{
  int ret;
  struct bcm2711_i2cdev_s *priv;

  DEBUGASSERT(0 <= port && port < BCM_BSCS_NUM);

  /* Initialize selected port */

  switch (port)
    {
#if defined(CONFIG_BCM2711_I2C1)
    case 1:
      priv = &g_i2c1dev;
      priv->dev.ops = &bcm2711_i2c_ops;
      break;
#endif
    default:
      i2cerr("Port %d is unsupported.\n", port);
      return NULL;
    }

  i2cinfo("Initializing I2C%u\n", port);

  /* Exclusive access */

  nxmutex_lock(&priv->lock);

  /* Test for previous initialization. If already initialized, nothing more
   * needs to be done.
   */

  if (1 < ++priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      i2cinfo("I2C%u already initialized\n", port);
      return &priv->dev;
    }

  /* Not yet initialized, little more work to do. */

  /* TODO: allow pins to be configured for different I2C interfaces.
   * Currently hard-coded for default I2C1 interface.
   */

  bcm2711_gpio_set_pulls(2, false, false);
  bcm2711_gpio_set_pulls(3, false, false);
  bcm2711_gpio_set_func(2, BCM_GPIO_FUNC0);
  bcm2711_gpio_set_func(3, BCM_GPIO_FUNC0);

  bcm2711_i2c_disable(priv); /* Only enabled when used. */
  bcm2711_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  /* Attach interrupt handler if it hasn't been already */

  if (!g_i2c_irqinit)
    {
      ret = irq_attach(BCM_IRQ_VC_I2C, bcm2711_i2c_primary_handler, NULL);
      if (ret < 0)
        {
          i2cerr("Could not attach interrupt handler for port %d: %d\n",
                 port, ret);
          return NULL;
        }

      i2cinfo("I2C interrupt handler attached\n");

      /* Enable interrupt handler */

      arm64_gic_irq_set_priority(BCM_IRQ_VC_I2C, 0, IRQ_TYPE_EDGE);
      up_enable_irq(BCM_IRQ_VC_I2C);
      i2cinfo("I2C IRQ enabled\n");
      g_i2c_irqinit = true; /* Mark IRQ handler as initialized */
    }

  g_i2c_devsinit++; /* Another device initialized */
  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: bcm2711_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C device on the BCM2711.
 *
 * Input parameters;
 *     dev - The device to uninitialize.
 *
 ****************************************************************************/

int bcm2711_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  int ret = 0;
  struct bcm2711_i2cdev_s *priv = (struct bcm2711_i2cdev_s *)dev;

  if (priv->refs == 0)
    {
      return -1;
    }

  /* If there are still references to the device, just decrement references.
   */

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* This was the last reference to the I2C device. */

  bcm2711_i2c_disable(priv);
  g_i2c_devsinit--; /* One less device initialized */

  /* If there are no more I2C devices initialized, turn off interrupts. */

  if (g_i2c_devsinit == 0)
    {
      up_disable_irq(BCM_IRQ_VC_I2C);
      irq_detach(BCM_IRQ_VC_I2C);
      g_i2c_irqinit = false;
      i2cinfo("Detached I2C interrupt handler and disabled IRQ.\n");
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

#endif // defined(CONFIG_BCM2711_I2C)
