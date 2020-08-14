/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_i2c.c
 *
 *   Author: David Hewson
 *
 *   Copyright (C) 2010-2011, 2014, 2016-2017 Gregory Nutt. All rights
 *   reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc31_i2c.h"
#include "lpc31_evntrtr.h"
#include "lpc31_syscreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  ((20 * CLK_TCK) / 1000)      /* 20 mS */

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lpc31_i2cdev_s
{
    struct i2c_master_s dev;      /* Generic I2C device */
    unsigned int      base;       /* Base address of registers */
    uint16_t          clkid;      /* Clock for this device */
    uint16_t          rstid;      /* Reset for this device */
    uint16_t          irqid;      /* IRQ for this device */

    sem_t             mutex;      /* Only one thread can access at a time */
    sem_t             wait;       /* Place to wait for state machine completion */
    volatile uint8_t  state;      /* State of state machine */
    struct wdog_s     timeout;    /* Watchdog to timeout when bus hung */
    uint32_t          frequency;  /* Current I2C frequency */

    struct i2c_msg_s *msgs;       /* remaining transfers - first one is in progress */
    unsigned int      nmsg;       /* number of transfer remaining */

    uint16_t          header[3];  /* I2C address header */
    uint16_t          hdrcnt;     /* number of bytes of header */
    uint16_t          wrcnt;      /* number of bytes sent to tx fifo */
    uint16_t          rdcnt;      /* number of bytes read from rx fifo */
};

#define I2C_STATE_DONE      0
#define I2C_STATE_START     1
#define I2C_STATE_HEADER    2
#define I2C_STATE_TRANSFER  3

static struct lpc31_i2cdev_s i2cdevices[2];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  i2c_interrupt(int irq, FAR void *context, FAR void *arg);
static void i2c_progress(struct lpc31_i2cdev_s *priv);
static void i2c_timeout(wdparm_t arg);
static void i2c_hwreset(struct lpc31_i2cdev_s *priv);
static void i2c_setfrequency(struct lpc31_i2cdev_s *priv,
                             uint32_t frequency);
static int  i2c_transfer(FAR struct i2c_master_s *dev,
                         FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  i2c_reset(FAR struct i2c_master_s * dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct i2c_ops_s lpc31_i2c_ops =
{
  .transfer = i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = i2c_reset
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void i2c_setfrequency(struct lpc31_i2cdev_s *priv, uint32_t frequency)
{
  if (frequency != priv->frequency)
    {
      uint32_t freq = lpc31_clkfreq(priv->clkid, DOMAINID_AHB0APB1);

      if (freq > 100000)
        {
          /* asymmetric per 400Khz I2C spec */

          putreg32(((47 * freq) / (83 + 47)) / frequency,
                   priv->base + LPC31_I2C_CLKHI_OFFSET);
          putreg32(((83 * freq) / (83 + 47)) / frequency,
                   priv->base + LPC31_I2C_CLKLO_OFFSET);
        }
      else
        {
          /* 50/50 mark space ratio */

          putreg32(((50 * freq) / 100) / frequency,
                   priv->base + LPC31_I2C_CLKLO_OFFSET);
          putreg32(((50 * freq) / 100) / frequency,
                   priv->base + LPC31_I2C_CLKHI_OFFSET);
        }

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int i2c_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct lpc31_i2cdev_s *priv = (struct lpc31_i2cdev_s *)arg;

  DEBUGASSERT(priv != NULL);
  i2c_progress(priv);
  return OK;
}

/****************************************************************************
 * Name: i2c_progress
 *
 * Description:
 *   Progress any remaining I2C transfers
 *
 ****************************************************************************/

static void i2c_progress(struct lpc31_i2cdev_s *priv)
{
  struct i2c_msg_s *msg;
  uint32_t stat;
  uint32_t ctrl;

  stat = getreg32(priv->base + LPC31_I2C_STAT_OFFSET);

  /* Were there arbitration problems? */

  if ((stat & I2C_STAT_AFI) != 0)
    {
      /* Perform a soft reset */

      i2c_hwreset(priv);

      /* FIXME: automatic retry? */

      priv->state = I2C_STATE_DONE;
      nxsem_post(&priv->wait);
      return;
    }

  while (priv->nmsg > 0)
    {
      ctrl = I2C_CTRL_NAIE | I2C_CTRL_AFIE | I2C_CTRL_TDIE;
      msg  = priv->msgs;

      switch (priv->state)
        {
        case I2C_STATE_START:
          if ((msg->flags & I2C_M_TEN) != 0)
            {
              priv->header[0] = I2C_TX_START | 0xf0 |
                                ((msg->addr & 0x300) >> 7);
              priv->header[1] = msg->addr & 0xff;
              priv->hdrcnt = 2;
              if (msg->flags & I2C_M_READ)
                {
                  priv->header[2] = priv->header[0] | 1;
                  priv->hdrcnt++;
                }
            }
          else
            {
              priv->header[0] = I2C_TX_START | (msg->addr << 1) |
                                (msg->flags & I2C_M_READ);
              priv->hdrcnt = 1;
            }

          putreg32(ctrl, priv->base + LPC31_I2C_CTRL_OFFSET);

          priv->state = I2C_STATE_HEADER;
          priv->wrcnt = 0;

          /* DROP THROUGH */

        case I2C_STATE_HEADER:
          while ((priv->wrcnt != priv->hdrcnt) && (stat & I2C_STAT_TFF) == 0)
            {
              putreg32(priv->header[priv->wrcnt],
                       priv->base + LPC31_I2C_TX_OFFSET);
              priv->wrcnt++;

              stat = getreg32(priv->base + LPC31_I2C_STAT_OFFSET);
            }

          if (priv->wrcnt < priv->hdrcnt)
            {
              /* Enable Tx FIFO Not Full Interrupt */

              putreg32(ctrl | I2C_CTRL_TFFIE,
                       priv->base + LPC31_I2C_CTRL_OFFSET);
              goto out;
            }

          priv->state = I2C_STATE_TRANSFER;
          priv->wrcnt = 0;
          priv->rdcnt = 0;

          /* DROP THROUGH */

        case I2C_STATE_TRANSFER:
          if (msg->flags & I2C_M_READ)
            {
              while ((priv->rdcnt != msg->length) &&
                     (stat & I2C_STAT_RFE) == 0)
                {
                  msg->buffer[priv->rdcnt] =
                    getreg32(priv->base + LPC31_I2C_RX_OFFSET);
                  priv->rdcnt++;

                  stat = getreg32(priv->base + LPC31_I2C_STAT_OFFSET);
                }

              if (priv->rdcnt < msg->length)
                {
                  /* Not all data received,
                   * fill the Tx FIFO with more dummies
                   */

                  while ((priv->wrcnt != msg->length) &&
                         (stat & I2C_STAT_TFF) == 0)
                    {
                      if (priv->wrcnt + 1 == msg->length && priv->nmsg == 1)
                        {
                          putreg32(I2C_TX_STOP,
                                   priv->base + LPC31_I2C_TX_OFFSET);
                        }
                      else
                        {
                          putreg32(0, priv->base + LPC31_I2C_TX_OFFSET);
                        }

                      priv->wrcnt++;

                      stat = getreg32(priv->base + LPC31_I2C_STAT_OFFSET);
                    }

                  if (priv->wrcnt < msg->length)
                    {
                      /* Enable Tx FIFO not full and
                       * Rx Fifo Avail Interrupts
                       */

                      putreg32(ctrl | I2C_CTRL_TFFIE | I2C_CTRL_RFDAIE,
                               priv->base + LPC31_I2C_CTRL_OFFSET);
                    }
                  else
                    {
                      /* Enable Rx Fifo Avail Interrupts */

                      putreg32(ctrl | I2C_CTRL_RFDAIE,
                               priv->base + LPC31_I2C_CTRL_OFFSET);
                    }

                  goto out;
                }
            }
          else    /* WRITE */
            {
              while (!!(priv->wrcnt != msg->length) && (stat & I2C_STAT_TFF))
                {
                  if ((priv->wrcnt + 1) == msg->length && priv->nmsg == 1)
                    {
                      putreg32(I2C_TX_STOP | msg->buffer[priv->wrcnt],
                               priv->base + LPC31_I2C_TX_OFFSET);
                    }
                  else
                    {
                      putreg32(msg->buffer[priv->wrcnt],
                               priv->base + LPC31_I2C_TX_OFFSET);
                    }

                  priv->wrcnt++;

                  stat = getreg32(priv->base + LPC31_I2C_STAT_OFFSET);
                }

              if (priv->wrcnt < msg->length)
                {
                  /* Enable Tx Fifo not full Interrupt */

                  putreg32(ctrl | I2C_CTRL_TFFIE,
                           priv->base + LPC31_I2C_CTRL_OFFSET);
                  goto out;
                }
            }

          /* Transfer completed, move onto the next one */

          priv->state = I2C_STATE_START;

          if (--priv->nmsg == 0)
            {
              /* Final transfer, wait for Transmit Done Interrupt */

              putreg32(ctrl, priv->base + LPC31_I2C_CTRL_OFFSET);
              goto out;
            }

          priv->msgs++;
          break;
        }
    }

out:
  if (stat & I2C_STAT_TDI)
    {
      putreg32(I2C_STAT_TDI, priv->base + LPC31_I2C_STAT_OFFSET);

      /* You'd expect the NAI bit to be set when no acknowledge was
       * received - but it gets cleared whenever a write it done to
       * the TXFIFO - so we've gone and cleared it while priming the
       * rest of the transfer!
       */

      if ((stat = getreg32(priv->base + LPC31_I2C_TXFL_OFFSET)) != 0)
        {
          if (priv->nmsg == 0)
            {
              priv->nmsg++;
            }

          i2c_hwreset(priv);
        }

      priv->state = I2C_STATE_DONE;
      nxsem_post(&priv->wait);
    }
}

/****************************************************************************
 * Name: i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void i2c_timeout(wdparm_t arg)
{
  struct lpc31_i2cdev_s *priv = (struct lpc31_i2cdev_s *) arg;

  irqstate_t flags = enter_critical_section();

  if (priv->state != I2C_STATE_DONE)
    {
      /* If there's data remaining in the TXFIFO, then ensure at least
       * one transfer has failed to complete.
       */

      if (getreg32(priv->base + LPC31_I2C_TXFL_OFFSET) != 0)
        {
          if (priv->nmsg == 0)
            {
              priv->nmsg++;
            }
        }

      /* Soft reset the USB controller */

      i2c_hwreset(priv);

      /* Mark the transfer as finished */

      priv->state = I2C_STATE_DONE;
      nxsem_post(&priv->wait);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: i2c_hwreset
 *
 * Description:
 *   Perform a soft reset of the I2C controller
 *
 ****************************************************************************/

static void i2c_hwreset(struct lpc31_i2cdev_s *priv)
{
  putreg32(I2C_CTRL_RESET, priv->base + LPC31_I2C_CTRL_OFFSET);

  /* Wait for Reset to complete */

  while (!!(getreg32(priv->base + LPC31_I2C_CTRL_OFFSET) & I2C_CTRL_RESET))
      ;
}

/****************************************************************************
 * Name: i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int i2c_transfer(FAR struct i2c_master_s *dev,
                        FAR struct i2c_msg_s *msgs, int count)
{
  struct lpc31_i2cdev_s *priv = (struct lpc31_i2cdev_s *) dev;
  irqstate_t flags;
  int ret;

  /* Get exclusive access to the I2C bus */

  nxsem_wait(&priv->mutex);
  flags = enter_critical_section();

  /* Set up for the transfer */

  priv->state = I2C_STATE_START;
  priv->msgs  = msgs;
  priv->nmsg  = count;

  /* Configure the I2C frequency.
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  i2c_setfrequency(priv, msgs->frequency);

  /* Start the transfer */

  i2c_progress(priv);

  /* Start a watchdog to timeout the transfer if the bus is locked up... */

  wd_start(&priv->timeout, I2C_TIMEOUT, i2c_timeout, (wdparm_t)priv);

  /* Wait for the transfer to complete */

  while (priv->state != I2C_STATE_DONE)
    {
      nxsem_wait(&priv->wait);
    }

  wd_cancel(&priv->timeout);
  ret = count - priv->nmsg;

  leave_critical_section(flags);
  nxsem_post(&priv->mutex);
  return ret;
}

/****************************************************************************
 * Name: i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int i2c_reset(FAR struct i2c_master_s * dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *lpc31_i2cbus_initialize(int port)
{
  struct lpc31_i2cdev_s *priv = &i2cdevices[port];

  priv->base  = (port == 0) ? LPC31_I2C0_VBASE : LPC31_I2C1_VBASE;
  priv->clkid = (port == 0) ? CLKID_I2C0PCLK   : CLKID_I2C1PCLK;
  priv->rstid = (port == 0) ? RESETID_I2C0RST  : RESETID_I2C1RST;
  priv->irqid = (port == 0) ? LPC31_IRQ_I2C0   : LPC31_IRQ_I2C1;

  /* Initialize semaphores */

  nxsem_init(&priv->mutex, 0, 1);
  nxsem_init(&priv->wait, 0, 0);

  /* The wait semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->wait, SEM_PRIO_NONE);

  /* Enable I2C system clocks */

  lpc31_enableclock(priv->clkid);

  /* Reset I2C blocks */

  lpc31_softreset(priv->rstid);

  /* Soft reset the device */

  i2c_hwreset(priv);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Install our operations */

  priv->dev.ops = &lpc31_i2c_ops;
  return &priv->dev;
}

/****************************************************************************
 * Name: lpc31_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int lpc31_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  struct lpc31_i2cdev_s *priv = (struct lpc31_i2cdev_s *)dev;

  /* Disable All Interrupts, soft reset the device */

  i2c_hwreset(priv);

  /* Detach Interrupt Handler */

  irq_detach(priv->irqid);

  /* Reset I2C blocks */

  lpc31_softreset(priv->rstid);

  /* Disable I2C system clocks */

  lpc31_disableclock(priv->clkid);
  return OK;
}
