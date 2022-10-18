/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_i2c.c
 *
 *   Copyright (C) 2012, 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the LPC17 version:
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-20 initial version
 *
 * Derived from arch/arm/src/lpc31xx/lpc31_i2c.c
 *
 *   Author: David Hewson
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lpc43_i2c.h"
#include "lpc43_scu.h"
#include "lpc43_ccu.h"
#include "lpc43_pinconfig.h"

#if defined(CONFIG_LPC43_I2C0) || defined(CONFIG_LPC43_I2C1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  (20*1000/CONFIG_USEC_PER_TICK) /* 20 mS */

#ifdef CONFIG_LPC43_I2C0_SUPERFAST
#  define I2C0_DEFAULT_FREQUENCY 1000000
#else
#  define I2C0_DEFAULT_FREQUENCY 400000
#endif

#define I2C1_DEFAULT_FREQUENCY 400000

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lpc43_i2cdev_s
{
  struct i2c_master_s dev;     /* Generic I2C device */
  unsigned int     base;       /* Base address of registers */
  uint16_t         irqid;      /* IRQ for this device */
  uint32_t         base_freq;  /* branch frequency */

  mutex_t          lock;       /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for state machine completion */
  volatile uint8_t state;      /* State of state machine */
  struct wdog_s    timeout;    /* watchdog to timeout when bus hung */
  uint32_t         frequency;  /* Current I2C frequency */

  struct i2c_msg_s *msgs;      /* remaining transfers - first one is in progress */
  unsigned int     nmsg;       /* number of transfer remaining */

  uint16_t         wrcnt;      /* number of bytes sent to tx fifo */
  uint16_t         rdcnt;      /* number of bytes read from rx fifo */
};

#ifdef CONFIG_LPC43_I2C0
static struct lpc43_i2cdev_s g_i2c0dev;
#endif
#ifdef CONFIG_LPC43_I2C1
static struct lpc43_i2cdev_s g_i2c1dev;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int  lpc43_i2c_start(struct lpc43_i2cdev_s *priv);
static void lpc43_i2c_stop(struct lpc43_i2cdev_s *priv);
static int  lpc43_i2c_interrupt(int irq, void *context, void *arg);
static void lpc43_i2c_timeout(wdparm_t arg);
static void lpc43_i2c_setfrequency(struct lpc43_i2cdev_s *priv,
              uint32_t frequency);
static int  lpc43_i2c_transfer(struct i2c_master_s *dev,
              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int lpc43_i2c_reset(struct i2c_master_s * dev);
#endif

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

struct i2c_ops_s lpc43_i2c_ops =
{
  .transfer = lpc43_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = lpc43_i2c_reset
#endif
};

/****************************************************************************
 * Name: lpc43_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void lpc43_i2c_setfrequency(struct lpc43_i2cdev_s *priv,
                                   uint32_t frequency)
{
  if (frequency != priv->frequency)
    {
      if (frequency > 100000)
        {
          /* asymmetric per 400Khz I2C spec */

          putreg32(priv->base_freq / (83 + 47) * 47 / frequency,
                   priv->base + LPC43_I2C_SCLH_OFFSET);
          putreg32(priv->base_freq / (83 + 47) * 83 / frequency,
                   priv->base + LPC43_I2C_SCLL_OFFSET);
        }
      else
        {
          /* 50/50 mark space ratio */

          putreg32(priv->base_freq / 100 * 50 / frequency,
                   priv->base + LPC43_I2C_SCLH_OFFSET);
          putreg32(priv->base_freq / 100 * 50 / frequency,
                   priv->base + LPC43_I2C_SCLL_OFFSET);
        }

      priv->frequency =  frequency;
    }
}

/****************************************************************************
 * Name: lpc43_i2c_start
 *
 * Description:
 *   Perform a I2C transfer start
 *
 ****************************************************************************/

static int lpc43_i2c_start(struct lpc43_i2cdev_s *priv)
{
  putreg32(I2C_CONCLR_STAC | I2C_CONCLR_SIC,
           priv->base + LPC43_I2C_CONCLR_OFFSET);
  putreg32(I2C_CONSET_STA, priv->base + LPC43_I2C_CONSET_OFFSET);

  wd_start(&priv->timeout, I2C_TIMEOUT,
           lpc43_i2c_timeout, (wdparm_t)priv);
  nxsem_wait(&priv->wait);

  wd_cancel(&priv->timeout);
  return priv->nmsg;
}

/****************************************************************************
 * Name: lpc43_i2c_stop
 *
 * Description:
 *   Perform a I2C transfer stop
 *
 ****************************************************************************/

static void lpc43_i2c_stop(struct lpc43_i2cdev_s *priv)
{
  if (priv->state != 0x38)
    {
      putreg32(I2C_CONSET_STO | I2C_CONSET_AA,
               priv->base + LPC43_I2C_CONSET_OFFSET);
    }

  nxsem_post(&priv->wait);
}

/****************************************************************************
 * Name: lpc43_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void lpc43_i2c_timeout(wdparm_t arg)
{
  struct lpc43_i2cdev_s *priv = (struct lpc43_i2cdev_s *)arg;

  irqstate_t flags = enter_critical_section();
  priv->state = 0xff;
  nxsem_post(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc32_i2c_nextmsg
 *
 * Description:
 *   Setup for the next message.
 *
 ****************************************************************************/

void lpc32_i2c_nextmsg(struct lpc43_i2cdev_s *priv)
{
  priv->nmsg--;

  if (priv->nmsg > 0)
    {
      priv->msgs++;
      putreg32(I2C_CONSET_STA, priv->base + LPC43_I2C_CONSET_OFFSET);
    }
  else
    {
      lpc43_i2c_stop(priv);
    }
}

/****************************************************************************
 * Name: lpc43_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int lpc43_i2c_interrupt(int irq, void *context, void *arg)
{
  struct lpc43_i2cdev_s *priv = (struct lpc43_i2cdev_s *)arg;
  struct i2c_msg_s *msg;
  uint32_t state;

  DEBUGASSERT(priv != NULL);

  /* Reference UM10360 19.10.5 */

  state = getreg32(priv->base + LPC43_I2C_STAT_OFFSET);
  msg  = priv->msgs;

  priv->state = state;
  state &= 0xf8;  /* state mask, only 0xX8 is possible */
  switch (state)
    {
    case 0x08:    /* A START condition has been transmitted. */
    case 0x10:    /* A Repeated START condition has been transmitted. */

      /* Set address */

      putreg32(((I2C_M_READ & msg->flags) == I2C_M_READ) ?
        I2C_READADDR8(msg->addr) :
        I2C_WRITEADDR8(msg->addr), priv->base + LPC43_I2C_DAT_OFFSET);

      /* Clear start bit */

      putreg32(I2C_CONCLR_STAC, priv->base + LPC43_I2C_CONCLR_OFFSET);
      break;

    /* Write cases */

    case 0x18: /* SLA+W has been transmitted; ACK has been received  */
      priv->wrcnt = 0;
      putreg32(msg->buffer[0], priv->base + LPC43_I2C_DAT_OFFSET); /* put first byte */
      break;

    case 0x28: /* Data byte in DAT has been transmitted; ACK has been received. */
      priv->wrcnt++;

      if (priv->wrcnt < msg->length)
        {
          putreg32(msg->buffer[priv->wrcnt], priv->base + LPC43_I2C_DAT_OFFSET); /* Put next byte */
        }
      else
        {
          lpc32_i2c_nextmsg(priv);
        }
      break;

    /* Read cases */

    case 0x40:  /* SLA+R has been transmitted; ACK has been received */
      priv->rdcnt = 0;
      if (msg->length > 1)
        {
          putreg32(I2C_CONSET_AA, priv->base + LPC43_I2C_CONSET_OFFSET); /* Set ACK next read */
        }
      else
        {
          putreg32(I2C_CONCLR_AAC, priv->base + LPC43_I2C_CONCLR_OFFSET);  /* Do not ACK because only one byte */
        }
      break;

    case 0x50:  /* Data byte has been received; ACK has been returned. */
      priv->rdcnt++;
      msg->buffer[priv->rdcnt - 1] =
        getreg32(priv->base + LPC43_I2C_BUFR_OFFSET);

      if (priv->rdcnt >= (msg->length - 1))
        {
          putreg32(I2C_CONCLR_AAC, priv->base + LPC43_I2C_CONCLR_OFFSET);  /* Do not ACK any more */
        }
      break;

    case 0x58:  /* Data byte has been received; NACK has been returned. */
      msg->buffer[priv->rdcnt] =
        getreg32(priv->base + LPC43_I2C_BUFR_OFFSET);
      lpc32_i2c_nextmsg(priv);
      break;

    default:
      lpc43_i2c_stop(priv);
      break;
    }

  putreg32(I2C_CONCLR_SIC, priv->base + LPC43_I2C_CONCLR_OFFSET); /* clear interrupt */

  return OK;
}

/****************************************************************************
 * Name: lpc43_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int lpc43_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct lpc43_i2cdev_s *priv = (struct lpc43_i2cdev_s *)dev;
  int ret;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

  /* Set up for the transfer */

  priv->wrcnt = 0;
  priv->rdcnt = 0;
  priv->msgs  = msgs;
  priv->nmsg  = count;

  /* Configure the I2C frequency.
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  lpc43_i2c_setfrequency(priv, msgs->frequency);

  /* Perform the transfer */

  ret = lpc43_i2c_start(priv);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: lpc43_i2c_reset
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
static int lpc43_i2c_reset(struct i2c_master_s * dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *lpc43_i2cbus_initialize(int port)
{
  struct lpc43_i2cdev_s *priv;

  if (port > 1)
    {
      i2cerr("ERROR: lpc I2C only supports ports 0 and 1\n");
      return NULL;
    }

  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

#ifdef CONFIG_LPC43_I2C0
  if (port == 0)
    {
      priv           = &g_i2c0dev;
      priv->base     = LPC43_I2C0_BASE;
      priv->irqid    = LPC43M4_IRQ_I2C0;
      priv->base_freq = BOARD_ABP1_FREQUENCY;

      /* Enable, set mode */

      regval  = getreg32(LPC43_SCU_SFSI2C0);
      regval |= SCU_SFSI2C0_SCL_EZI | SCU_SFSI2C0_SDA_EZI;

#ifdef CONFIG_LPC43_I2C0_SUPERFAST
      /* Enable super fast mode */

      regval |= SCU_SFSI2C0_SCL_EHD | SCU_SFSI2C0_SDA_EHD;
#endif

      putreg32(regval, LPC43_SCU_SFSI2C0);

      /* Enable clock */

      regval  = getreg32(LPC43_CCU1_APB1_I2C0_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_APB1_I2C0_CFG);

      lpc43_i2c_setfrequency(priv, I2C0_DEFAULT_FREQUENCY);

      /* No pin configuration needed */
    }
  else
#endif
#ifdef CONFIG_LPC43_I2C1
  if (port == 1)
    {
      priv        = &g_i2c1dev;
      priv->base  = LPC43_I2C1_BASE;
      priv->irqid = LPC43M4_IRQ_I2C1;
      priv->base_freq = BOARD_ABP3_FREQUENCY;

      /* No need to enable */

      /* Enable clock */

      regval  = getreg32(LPC43_CCU1_APB3_I2C1_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_APB3_I2C1_CFG);

      /* Pin configuration */

      lpc43_pin_config(PINCONF_I2C1_SCL);
      lpc43_pin_config(PINCONF_I2C1_SDA);

      lpc43_i2c_setfrequency(priv, I2C1_DEFAULT_FREQUENCY);
    }
  else
#endif
    {
      leave_critical_section(flags);
      return NULL;
    }

  leave_critical_section(flags);

  putreg32(I2C_CONSET_I2EN, priv->base + LPC43_I2C_CONSET_OFFSET);

  /* Initialize mutex & semaphores */

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->wait, 0, 0);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, lpc43_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Install our operations */

  priv->dev.ops = &lpc43_i2c_ops;
  return &priv->dev;
}

/****************************************************************************
 * Name: lpc43_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int lpc43_i2cbus_uninitialize(struct i2c_master_s * dev)
{
  struct lpc43_i2cdev_s *priv = (struct lpc43_i2cdev_s *) dev;

  putreg32(I2C_CONCLRT_I2ENC, priv->base + LPC43_I2C_CONCLR_OFFSET);
  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);
  return OK;
}

#endif /* CONFIG_LPC43_I2C0 || CONFIG_LPC43_I2C1 */
