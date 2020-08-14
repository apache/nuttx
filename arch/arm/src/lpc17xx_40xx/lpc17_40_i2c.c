/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_i2c.c
 *
 *   Copyright (C) 2012, 2014-2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com> (Original author)
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

#include "chip.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_i2c.h"

#if defined(CONFIG_LPC17_40_I2C0) || defined(CONFIG_LPC17_40_I2C1) || defined(CONFIG_LPC17_40_I2C2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef GPIO_I2C1_SCL
#  define GPIO_I2C1_SCL GPIO_I2C1_SCL_1
#  define GPIO_I2C1_SDA GPIO_I2C1_SDA_1
#endif

#ifndef CONFIG_LPC17_40_I2C0_FREQUENCY
#  define CONFIG_LPC17_40_I2C0_FREQUENCY 100000
#endif

#ifndef CONFIG_LPC17_40_I2C1_FREQUENCY
#  define CONFIG_LPC17_40_I2C1_FREQUENCY 100000
#endif

#ifndef CONFIG_LPC17_40_I2C2_FREQUENCY
#  define CONFIG_LPC17_40_I2C2_FREQUENCY 100000
#endif

#define LPC17_40_I2C1_FREQUENCY 400000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lpc17_40_i2cdev_s
{
  struct i2c_master_s dev;     /* Generic I2C device */
  unsigned int     base;       /* Base address of registers */
  uint16_t         irqid;      /* IRQ for this device */

  sem_t            mutex;      /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for state machine completion */
  volatile uint8_t state;      /* State of state machine */
  struct wdog_s    timeout;    /* Watchdog to timeout when bus hung */
  uint32_t         frequency;  /* Current I2C frequency */

  struct i2c_msg_s *msgs;      /* remaining transfers - first one is in progress */
  unsigned int     nmsg;       /* number of transfer remaining */

  uint16_t         wrcnt;      /* number of bytes sent to tx fifo */
  uint16_t         rdcnt;      /* number of bytes read from rx fifo */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  lpc17_40_i2c_start(struct lpc17_40_i2cdev_s *priv);
static void lpc17_40_i2c_stop(struct lpc17_40_i2cdev_s *priv);
static int  lpc17_40_i2c_interrupt(int irq, FAR void *context, void *arg);
static void lpc17_40_i2c_timeout(wdparm_t arg);
static void lpc17_40_i2c_setfrequency(struct lpc17_40_i2cdev_s *priv,
              uint32_t frequency);
static void lpc17_40_stopnext(struct lpc17_40_i2cdev_s *priv);

/* I2C device operations */

static int  lpc17_40_i2c_transfer(FAR struct i2c_master_s *dev,
              FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  lpc17_40_i2c_reset(FAR struct i2c_master_s * dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_I2C0
static struct lpc17_40_i2cdev_s g_i2c0dev;
#endif
#ifdef CONFIG_LPC17_40_I2C1
static struct lpc17_40_i2cdev_s g_i2c1dev;
#endif
#ifdef CONFIG_LPC17_40_I2C2
static struct lpc17_40_i2cdev_s g_i2c2dev;
#endif

struct i2c_ops_s lpc17_40_i2c_ops =
{
  .transfer = lpc17_40_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = lpc17_40_i2c_reset
#endif
};

/****************************************************************************
 * Name: lpc17_40_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void lpc17_40_i2c_setfrequency(struct lpc17_40_i2cdev_s *priv,
                                   uint32_t frequency)
{
  if (frequency != priv->frequency)
    {
      if (frequency > 100000)
        {
          /* Asymmetric per 400Khz I2C spec */

          putreg32(LPC17_40_CCLK / (83 + 47) * 47 / frequency,
                   priv->base + LPC17_40_I2C_SCLH_OFFSET);
          putreg32(LPC17_40_CCLK / (83 + 47) * 83 / frequency,
                   priv->base + LPC17_40_I2C_SCLL_OFFSET);
        }
      else
        {
          /* 50/50 mark space ratio */

          putreg32(LPC17_40_CCLK / 100 * 50 / frequency,
                   priv->base + LPC17_40_I2C_SCLH_OFFSET);
          putreg32(LPC17_40_CCLK / 100 * 50 / frequency,
                   priv->base + LPC17_40_I2C_SCLL_OFFSET);
        }

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: lpc17_40_i2c_start
 *
 * Description:
 *   Perform a I2C transfer start
 *
 ****************************************************************************/

static int lpc17_40_i2c_start(struct lpc17_40_i2cdev_s *priv)
{
  uint32_t total_len = 0;
  uint32_t freq = 1000000;
  uint32_t timeout;
  int i;

  putreg32(I2C_CONCLR_STAC | I2C_CONCLR_SIC,
           priv->base + LPC17_40_I2C_CONCLR_OFFSET);
  putreg32(I2C_CONSET_STA, priv->base + LPC17_40_I2C_CONSET_OFFSET);

  /* Get the total transaction length and the minimum frequency */

  for (i = 0; i < priv->nmsg; i++)
    {
      total_len += priv->msgs[i].length;
      if (priv->msgs[i].frequency < freq)
        {
          freq = priv->msgs[i].frequency;
        }
    }

  /* Calculate the approximate timeout */

  timeout = ((total_len * (9000000 / CONFIG_USEC_PER_TICK)) / freq) + 1;

  /* Initializes the I2C state machine to a known value */

  priv->state = 0x00;

  wd_start(&priv->timeout, timeout,
           lpc17_40_i2c_timeout, (wdparm_t)priv);
  nxsem_wait(&priv->wait);

  return priv->nmsg;
}

/****************************************************************************
 * Name: lpc17_40_i2c_stop
 *
 * Description:
 *   Perform a I2C transfer stop
 *
 ****************************************************************************/

static void lpc17_40_i2c_stop(struct lpc17_40_i2cdev_s *priv)
{
  if (priv->state != 0x38)
    {
      putreg32(I2C_CONSET_STO | I2C_CONSET_AA,
               priv->base + LPC17_40_I2C_CONSET_OFFSET);
    }

  wd_cancel(&priv->timeout);
  nxsem_post(&priv->wait);
}

/****************************************************************************
 * Name: lpc17_40_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void lpc17_40_i2c_timeout(wdparm_t arg)
{
  struct lpc17_40_i2cdev_s *priv = (struct lpc17_40_i2cdev_s *)arg;

  irqstate_t flags = enter_critical_section();
  priv->state = 0xff;
  nxsem_post(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc17_40_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int lpc17_40_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  struct lpc17_40_i2cdev_s *priv = (struct lpc17_40_i2cdev_s *)dev;
  int ret;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Get exclusive access to the I2C bus */

  nxsem_wait(&priv->mutex);

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

  lpc17_40_i2c_setfrequency(priv, msgs->frequency);

  /* Perform the transfer */

  ret = lpc17_40_i2c_start(priv);

  nxsem_post(&priv->mutex);
  return ret;
}

/****************************************************************************
 * Name: lpc17_40_stopnext
 *
 * Description:
 *   Check if we need to issue STOP at the next message
 *
 ****************************************************************************/

static void lpc17_40_stopnext(struct lpc17_40_i2cdev_s *priv)
{
  priv->nmsg--;

  if (priv->nmsg > 0)
    {
      priv->msgs++;

      /* Check if a restart condition should be issued */

      if (priv->msgs->flags & I2C_M_NOSTART)
        {
          priv->wrcnt = 0;

          /* Starts transmitting the data buffer of the next message without
           * issuing a restart.
           */

          putreg32(priv->msgs->buffer[priv->wrcnt],
                   priv->base + LPC17_40_I2C_DAT_OFFSET);
        }
      else
        {
          putreg32(I2C_CONSET_STA, priv->base + LPC17_40_I2C_CONSET_OFFSET);
        }
    }
  else
    {
      lpc17_40_i2c_stop(priv);
    }
}

/****************************************************************************
 * Name: lpc17_40_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int lpc17_40_i2c_interrupt(int irq, FAR void *context, void *arg)
{
  struct lpc17_40_i2cdev_s *priv = (struct lpc17_40_i2cdev_s *)arg;
  struct i2c_msg_s *msg;
  uint32_t state;

  DEBUGASSERT(priv != NULL);

  /* Reference UM10360 19.10.5 */

  state = getreg32(priv->base + LPC17_40_I2C_STAT_OFFSET);
  msg  = priv->msgs;

  /* Checks if a timeout occurred */

  if (priv->state == 0xff)
    {
      state = 0xff;
    }
  else
    {
      priv->state = state;
      state &= 0xf8;  /* state mask, only 0xX8 is possible */
    }

  switch (state)
    {
    case 0x08:     /* A START condition has been transmitted. */
    case 0x10:     /* A Repeated START condition has been transmitted. */

      /* Set address */

      putreg32(((I2C_M_READ & msg->flags) == I2C_M_READ) ?
        I2C_READADDR8(msg->addr) :
        I2C_WRITEADDR8(msg->addr), priv->base + LPC17_40_I2C_DAT_OFFSET);

      /* Clear start bit */

      putreg32(I2C_CONCLR_STAC, priv->base + LPC17_40_I2C_CONCLR_OFFSET);
      break;

    /* Write cases */

    case 0x18: /* SLA+W has been transmitted; ACK has been received  */
      priv->wrcnt = 0;
      putreg32(msg->buffer[0], priv->base + LPC17_40_I2C_DAT_OFFSET); /* put first byte */
      break;

    case 0x28: /* Data byte in DAT has been transmitted; ACK has been received. */
      priv->wrcnt++;

      if (priv->wrcnt < msg->length)
        {
          putreg32(msg->buffer[priv->wrcnt], priv->base + LPC17_40_I2C_DAT_OFFSET); /* Put next byte */
        }
      else
        {
          lpc17_40_stopnext(priv);
        }
      break;

    /* Read cases */

    case 0x40:  /* SLA+R has been transmitted; ACK has been received */
      priv->rdcnt = 0;
      if (msg->length > 1)
        {
          putreg32(I2C_CONSET_AA, priv->base + LPC17_40_I2C_CONSET_OFFSET); /* Set ACK next read */
        }
      else
        {
          putreg32(I2C_CONCLR_AAC, priv->base + LPC17_40_I2C_CONCLR_OFFSET);  /* Do not ACK because only one byte */
        }
      break;

    case 0x50:  /* Data byte has been received; ACK has been returned. */
      priv->rdcnt++;
      msg->buffer[priv->rdcnt - 1] =
        getreg32(priv->base + LPC17_40_I2C_BUFR_OFFSET);

      if (priv->rdcnt >= (msg->length - 1))
        {
          putreg32(I2C_CONCLR_AAC, priv->base + LPC17_40_I2C_CONCLR_OFFSET);  /* Do not ACK any more */
        }
      break;

    case 0x58:  /* Data byte has been received; NACK has been returned. */
      msg->buffer[priv->rdcnt] =
        getreg32(priv->base + LPC17_40_I2C_BUFR_OFFSET);
      lpc17_40_stopnext(priv);
      break;

    default:
      lpc17_40_i2c_stop(priv);
      break;
    }

  putreg32(I2C_CONCLR_SIC, priv->base + LPC17_40_I2C_CONCLR_OFFSET); /* clear interrupt */

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_i2c_reset
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
static int lpc17_40_i2c_reset(FAR struct i2c_master_s * dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *lpc17_40_i2cbus_initialize(int port)
{
  struct lpc17_40_i2cdev_s *priv;

  if (port > 1)
    {
      i2cerr("ERROR: LPC I2C Only supports ports 0 and 1\n");
      return NULL;
    }

  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

#ifdef CONFIG_LPC17_40_I2C0
  if (port == 0)
    {
      priv        = &g_i2c0dev;
      priv->base  = LPC17_40_I2C0_BASE;
      priv->irqid = LPC17_40_IRQ_I2C0;

      /* Enable clocking */

      regval  = getreg32(LPC17_40_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCI2C0;
      putreg32(regval, LPC17_40_SYSCON_PCONP);

      regval  = getreg32(LPC17_40_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_I2C0_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL0_I2C0_SHIFT);
      putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);

      /* Pin configuration */

      lpc17_40_configgpio(GPIO_I2C0_SCL);
      lpc17_40_configgpio(GPIO_I2C0_SDA);

      /* Set default frequency */

      lpc17_40_i2c_setfrequency(priv, CONFIG_LPC17_40_I2C0_FREQUENCY);
    }
  else
#endif
#ifdef CONFIG_LPC17_40_I2C1
  if (port == 1)
    {
      priv        = &g_i2c1dev;
      priv->base  = LPC17_40_I2C1_BASE;
      priv->irqid = LPC17_40_IRQ_I2C1;

      /* Enable clocking */

      regval  = getreg32(LPC17_40_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCI2C1;
      putreg32(regval, LPC17_40_SYSCON_PCONP);

      regval  = getreg32(LPC17_40_SYSCON_PCLKSEL1);
      regval &= ~SYSCON_PCLKSEL1_I2C1_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL1_I2C1_SHIFT);
      putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);

      /* Pin configuration */

      lpc17_40_configgpio(GPIO_I2C1_SCL);
      lpc17_40_configgpio(GPIO_I2C1_SDA);

      /* Set default frequency */

      lpc17_40_i2c_setfrequency(priv, CONFIG_LPC17_40_I2C1_FREQUENCY);
    }
  else
#endif
#ifdef CONFIG_LPC17_40_I2C2
  if (port == 2)
    {
      priv        = &g_i2c2dev;
      priv->base  = LPC17_40_I2C2_BASE;
      priv->irqid = LPC17_40_IRQ_I2C2;

      /* Enable clocking */

      regval  = getreg32(LPC17_40_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCI2C2;
      putreg32(regval, LPC17_40_SYSCON_PCONP);

      regval  = getreg32(LPC17_40_SYSCON_PCLKSEL1);
      regval &= ~SYSCON_PCLKSEL1_I2C2_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL1_I2C2_SHIFT);
      putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);

      /* Pin configuration */

      lpc17_40_configgpio(GPIO_I2C2_SCL);
      lpc17_40_configgpio(GPIO_I2C2_SDA);

      /* Set default frequency */

      lpc17_40_i2c_setfrequency(priv, CONFIG_LPC17_40_I2C2_FREQUENCY);
    }
  else
#endif
    {
      return NULL;
    }

  leave_critical_section(flags);

  putreg32(I2C_CONSET_I2EN, priv->base + LPC17_40_I2C_CONSET_OFFSET);

  /* Initialize semaphores */

  nxsem_init(&priv->mutex, 0, 1);
  nxsem_init(&priv->wait, 0, 0);

  /* The wait semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->wait, SEM_PRIO_NONE);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, lpc17_40_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Install our operations */

  priv->dev.ops = &lpc17_40_i2c_ops;
  return &priv->dev;
}

/****************************************************************************
 * Name: lpc17_40_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int lpc17_40_i2cbus_uninitialize(FAR struct i2c_master_s * dev)
{
  struct lpc17_40_i2cdev_s *priv = (struct lpc17_40_i2cdev_s *) dev;

  /* Disable I2C */

  putreg32(I2C_CONCLRT_I2ENC, priv->base + LPC17_40_I2C_CONCLR_OFFSET);

  /* Reset data structures */

  nxsem_destroy(&priv->mutex);
  nxsem_destroy(&priv->wait);

  /* Cancel the watchdog timer */

  wd_cancel(&priv->timeout);

  /* Disable interrupts */

  up_disable_irq(priv->irqid);

  /* Detach Interrupt Handler */

  irq_detach(priv->irqid);
  return OK;
}

#endif /* CONFIG_LPC17_40_I2C0 || CONFIG_LPC17_40_I2C1 || CONFIG_LPC17_40_I2C2 */
