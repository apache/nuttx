/****************************************************************************
 * arch/arm/src/sam34/sam_twi.c
 *
 *   Copyright (C) 2013, 2015-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *    Copyright (c) 2011, Atmel Corporation
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
 * 3. Neither the name NuttX, Atmel, nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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

/* References:
 *   SAMA34 Series Data Sheet
 *   Atmel NoOS sample code.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_pinmap.h"

#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_twi.h"

/* REVISIT: Missing support for TWI2 master */

#if defined(CONFIG_SAM34_TWIM0) || defined(CONFIG_SAM34_TWIM1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAM34_TWIM0_FREQUENCY
#  define CONFIG_SAM34_TWIM0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM34_TWIM1_FREQUENCY
#  define CONFIG_SAM34_TWIM1_FREQUENCY 100000
#endif

#ifndef CONFIG_DEBUG_I2C_INFO
#  undef CONFIG_SAM34_TWI_REGDEBUG
#endif

/* Driver internal definitions **********************************************/

#define TWI_TIMEOUT ((100 * CLK_TCK) / 1000) /* 100 mS */

/* Clocking to the TWO module(s) is provided by the main clocked, divided
 * down as necessary.
 */

#define TWI_MAX_FREQUENCY 66000000   /* Maximum TWI frequency */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct twi_dev_s
{
  struct i2c_master_s dev;        /* Generic I2C device */
  struct i2c_msg_s    *msg;       /* Message list */
  uintptr_t           base;       /* Base address of registers */
  uint32_t            clkin;      /* TWI input clock frequency */
  uint32_t            i2cfreq;    /* Selected TWI frequency */
  uint16_t            irq;        /* IRQ number for this device */
  uint8_t             msgc;       /* Number of message in the message list */
  uint8_t             twi;        /* TWI peripheral number (for debug output) */
  uint8_t             pid;        /* TWI peripheral ID */

  mutex_t             lock;       /* Only one thread can access at a time */
  sem_t               waitsem;    /* Wait for TWI transfer completion */
  struct wdog_s       timeout;    /* Watchdog to recover from bus hangs */
  volatile int        result;     /* The result of the transfer */
  volatile int        xfrd;       /* Number of bytes transfers */

  /* Debug stuff */

#ifdef CONFIG_SAM34_TWI_REGDEBUG
  bool                wrlast;     /* Last was a write */
  uint32_t            addrlast;   /* Last address */
  uint32_t            vallast;    /* Last value */
  int                 ntimes;     /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SAM34_TWI_REGDEBUG
static bool twi_checkreg(struct twi_dev_s *priv, bool wr,
              uint32_t value, uintptr_t address);
static uint32_t twi_getabs(struct twi_dev_s *priv, uintptr_t address);
static void twi_putabs(struct twi_dev_s *priv, uintptr_t address,
              uint32_t value);
#else
# define    twi_checkreg(priv,wr,value,address) (false)
# define    twi_putabs(p,a,v) putreg32(v,a)
# define    twi_getabs(p,a) getreg32(a)
#endif

static inline uint32_t twi_getrel(struct twi_dev_s *priv,
          unsigned int offset);
static inline void twi_putrel(struct twi_dev_s *priv, unsigned int offset,
          uint32_t value);

/* I2C transfer helper functions */

static int  twi_wait(struct twi_dev_s *priv);
static void twi_wakeup(struct twi_dev_s *priv, int result);
static int  twi_interrupt(int irq, void *context, void *arg);
static void twi_timeout(wdparm_t arg);

static void twi_startread(struct twi_dev_s *priv, struct i2c_msg_s *msg);
static void twi_startwrite(struct twi_dev_s *priv, struct i2c_msg_s *msg);
static void twi_startmessage(struct twi_dev_s *priv, struct i2c_msg_s *msg);

/* I2C device operations */

static int twi_transfer(struct i2c_master_s *dev,
          struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  twi_reset(struct i2c_master_s * dev);
#endif

/* Initialization */

static void twi_setfrequency(struct twi_dev_s *priv, uint32_t frequency);
static void twi_hw_initialize(struct twi_dev_s *priv, unsigned int pid,
          uint32_t frequency);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_twiops =
{
  .transfer = twi_transfer,
#ifdef CONFIG_I2C_RESET
  .reset  = twi_reset
#endif
};

#ifdef CONFIG_SAM34_TWIM0
static struct twi_dev_s g_twi0 =
{
  .dev =
  {
    .ops = g_twiops,
  },
  .base = SAM_TWI0_BASE,
  .irq = SAM_IRQ_TWI0,
  .twi = 0,
  .lock = NXMUTEX_INITIALIZER,
  .waitsem = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_SAM34_TWIM1
static struct twi_dev_s g_twi1 =
{
  .dev =
  {
    .ops = g_twiops,
  },
  .base = SAM_TWI1_BASE,
  .irq = SAM_IRQ_TWI1,
  .twi = 1,
  .lock = NXMUTEX_INITIALIZER,
  .waitsem = SEM_INITIALIZER(0),
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: twi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   false: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_TWI_REGDEBUG
static bool twi_checkreg(struct twi_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      value   == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          i2cinfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = value;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: twi_getabs
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_TWI_REGDEBUG
static uint32_t twi_getabs(struct twi_dev_s *priv, uintptr_t address)
{
  uint32_t value = getreg32(address);

  if (twi_checkreg(priv, false, value, address))
    {
      i2cinfo("%08x->%08x\n", address, value);
    }

  return value;
}
#endif

/****************************************************************************
 * Name: twi_putabs
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_TWI_REGDEBUG
static void twi_putabs(struct twi_dev_s *priv, uintptr_t address,
                       uint32_t value)
{
  if (twi_checkreg(priv, true, value, address))
    {
      i2cinfo("%08x<-%08x\n", address, value);
    }

  putreg32(value, address);
}
#endif

/****************************************************************************
 * Name: twi_getrel
 *
 * Description:
 *  Read a TWI register using an offset relative to the TWI base address
 *
 ****************************************************************************/

static inline uint32_t twi_getrel(struct twi_dev_s *priv,
                                  unsigned int offset)
{
  return twi_getabs(priv, priv->base + offset);
}

/****************************************************************************
 * Name: twi_putrel
 *
 * Description:
 *  Write a value to a TWI register using an offset relative to the TWI base
 *  address.
 *
 ****************************************************************************/

static inline void twi_putrel(struct twi_dev_s *priv, unsigned int offset,
                              uint32_t value)
{
  twi_putabs(priv, priv->base + offset, value);
}

/****************************************************************************
 * I2C transfer helper functions
 ****************************************************************************/

/****************************************************************************
 * Name: twi_wait
 *
 * Description:
 *   Perform a I2C transfer start
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int twi_wait(struct twi_dev_s *priv)
{
  int ret;

  /* Start a timeout to avoid hangs */

  wd_start(&priv->timeout, TWI_TIMEOUT, twi_timeout, (wdparm_t)priv);

  /* Wait for either the TWI transfer or the timeout to complete */

  do
    {
      i2cinfo("TWI%d Waiting...\n", priv->twi);
      ret = nxsem_wait(&priv->waitsem);
      i2cinfo("TWI%d Awakened with result: %d\n", priv->twi, priv->result);

      if (ret < 0)
        {
          wd_cancel(&priv->timeout);
          return ret;
        }
    }
  while (priv->result == -EBUSY);

  /* We get here via twi_wakeup.  The watchdog timer has been disabled and
   * all further interrupts for the TWI have been disabled.
   */

  return priv->result;
}

/****************************************************************************
 * Name: twi_wakeup
 *
 * Description:
 *   A terminal event has occurred.  Wake-up the waiting thread
 *
 ****************************************************************************/

static void twi_wakeup(struct twi_dev_s *priv, int result)
{
  /* Cancel any pending timeout */

  wd_cancel(&priv->timeout);

  /* Disable any further TWI interrupts */

  twi_putrel(priv, SAM_TWI_IDR_OFFSET, TWI_INT_ALL);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: twi_interrupt
 *
 * Description:
 *   The TWI Interrupt Handler
 *
 ****************************************************************************/

static int twi_interrupt(int irq, void *context, void *arg)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)arg;
  struct i2c_msg_s *msg;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* Retrieve masked interrupt status */

  sr      = twi_getrel(priv, SAM_TWI_SR_OFFSET);
  imr     = twi_getrel(priv, SAM_TWI_IMR_OFFSET);
  pending = sr & imr;

  i2cinfo("TWI%d pending: %08x\n", priv->twi, pending);

  msg = priv->msg;

  /* Check for errors */

  if ((pending & TWI_INT_ERRORS) != 0)
    {
      /* Wake up the thread with an I/O error indication */

      i2cerr("ERROR: TWI%d pending: %08x\n", priv->twi, pending);
      twi_wakeup(priv, -EIO);
    }

  /* Byte received */

  else if ((pending & TWI_INT_RXRDY) != 0)
    {
      msg->buffer[priv->xfrd] = twi_getrel(priv, SAM_TWI_RHR_OFFSET);
      priv->xfrd++;

      /* Check for transfer complete */

      if (priv->xfrd >= msg->length)
        {
          /* The transfer is complete.  Disable the RXRDY interrupt and
           * enable the TXCOMP interrupt
           */

          twi_putrel(priv, SAM_TWI_IDR_OFFSET, TWI_INT_RXRDY);
          twi_putrel(priv, SAM_TWI_IER_OFFSET, TWI_INT_TXCOMP);
        }

      /* Not yet complete, but will the next be the last byte? */

      else if (priv->xfrd == (msg->length - 1))
        {
          /* Yes, set the stop signal */

          twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_STOP);
        }
    }

  /* Byte sent */

  else if ((pending & TWI_INT_TXRDY) != 0)
    {
      /* Transfer finished? */

      if (priv->xfrd >= msg->length)
        {
          /* The transfer is complete.  Disable the TXRDY interrupt and
           * enable the TXCOMP interrupt
           */

          twi_putrel(priv, SAM_TWI_IDR_OFFSET, TWI_INT_TXRDY);
          twi_putrel(priv, SAM_TWI_IER_OFFSET, TWI_INT_TXCOMP);

          /* Send the STOP condition */

          regval  = twi_getrel(priv, SAM_TWI_CR_OFFSET);
          regval |= TWI_CR_STOP;
          twi_putrel(priv, SAM_TWI_CR_OFFSET, regval);
        }

      /* No, there are more bytes remaining to be sent */

      else
        {
          twi_putrel(priv, SAM_TWI_THR_OFFSET, msg->buffer[priv->xfrd]);
          priv->xfrd++;
        }
    }

  /* Transfer complete */

  else if ((pending & TWI_INT_TXCOMP) != 0)
    {
      twi_putrel(priv, SAM_TWI_IDR_OFFSET, TWI_INT_TXCOMP);

      /* Is there another message to send? */

      if (priv->msgc > 1)
        {
          /* Yes... start the next message */

          priv->msg++;
          priv->msgc--;
          twi_startmessage(priv, priv->msg);
        }
      else
        {
          /* No.. we made it to the end of the message list with no errors.
           * Cancel any timeout and wake up the waiting thread with a
           * success indication.
           */

          twi_wakeup(priv, OK);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: twi_timeout
 *
 * Description:
 *   Watchdog timer for timeout of TWI operation
 *
 * Assumptions:
 *   Called from the timer interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static void twi_timeout(wdparm_t arg)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)arg;

  i2cerr("ERROR: TWI%d Timeout!\n", priv->twi);
  twi_wakeup(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: twi_startread
 *
 * Description:
 *   Start the next read message
 *
 ****************************************************************************/

static void twi_startread(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set STOP signal if only one byte is sent */

  if (msg->length == 1)
    {
      twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_STOP);
    }

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWI_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWI_MMR_OFFSET,
             TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD | TWI_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWI_IADR_OFFSET, 0);

  /* Enable read interrupt and send the START condition */

  twi_putrel(priv, SAM_TWI_IER_OFFSET, TWI_INT_RXRDY | TWI_INT_ERRORS);
  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_START);
}

/****************************************************************************
 * Name: twi_startwrite
 *
 * Description:
 *   Start the next write message
 *
 ****************************************************************************/

static void twi_startwrite(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWI_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWI_MMR_OFFSET,
             TWI_MMR_IADRSZ_NONE | TWI_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWI_IADR_OFFSET, 0);

  /* Write first byte to send. */

  twi_putrel(priv, SAM_TWI_THR_OFFSET, msg->buffer[priv->xfrd++]);

  /* Enable write interrupt */

  twi_putrel(priv, SAM_TWI_IER_OFFSET, TWI_INT_TXRDY | TWI_INT_ERRORS);
}

/****************************************************************************
 * Name: twi_startmessage
 *
 * Description:
 *   Start the next write message
 *
 ****************************************************************************/

static void twi_startmessage(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  if ((msg->flags & I2C_M_READ) != 0)
    {
      twi_startread(priv, msg);
    }
  else
    {
      twi_startwrite(priv, msg);
    }
}

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

/****************************************************************************
 * Name: twi_transfer
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 ****************************************************************************/

static int twi_transfer(struct i2c_master_s *dev,
                        struct i2c_msg_s *msgs, int count)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(dev != NULL);
  i2cinfo("TWI%d count: %d\n", priv->twi, count);

  /* Get exclusive access to the device */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Setup the message transfer */

  priv->msg  = msgs;
  priv->msgc = count;

  /* Configure the I2C frequency.
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  twi_setfrequency(priv, msgs->frequency);

  /* Initiate the transfer.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = enter_critical_section();
  twi_startmessage(priv, msgs);

  /* And wait for the transfers to complete.  Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = twi_wait(priv);
  if (ret < 0)
    {
      i2cerr("ERROR: Transfer failed: %d\n", ret);
    }

  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: twi_reset
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
static int twi_reset(struct i2c_master_s * dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: twi_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void twi_setfrequency(struct twi_dev_s *priv, uint32_t frequency)
{
  unsigned int ckdiv;
  unsigned int cldiv;
  uint32_t regval;

  if (frequency != priv->i2cfreq)
    {
      /* Configure TWI output clocking, trying each value of CKDIV {0..7} */

      for (ckdiv = 0; ckdiv < 8; ckdiv++)
        {
          /* Calculate the CLDIV value using the current CKDIV guess */

          cldiv = ((priv->clkin / (frequency << 1)) - 4) / (1 << ckdiv);

          /* Is CLDIV in range? */

          if (cldiv <= 255)
            {
              /* Yes, break out and use it */

              break;
            }
        }

      /* Then setup the TWI Clock Waveform Generator Register, using the same
       * value for CLDIV and CHDIV (for 1:1 duty).
       */

      twi_putrel(priv, SAM_TWI_CWGR_OFFSET, 0);

      regval = ((uint32_t)ckdiv << TWI_CWGR_CKDIV_SHIFT) |
               ((uint32_t)cldiv << TWI_CWGR_CHDIV_SHIFT) |
               ((uint32_t)cldiv << TWI_CWGR_CLDIV_SHIFT);
      twi_putrel(priv, SAM_TWI_CWGR_OFFSET, regval);

      /* Remember the selected frequency for error recovery */

      priv->i2cfreq = frequency;
    }
}

/****************************************************************************
 * Name: twi_hw_initialize
 *
 * Description:
 *   Initialize one TWI peripheral for I2C operation
 *
 ****************************************************************************/

static void twi_hw_initialize(struct twi_dev_s *priv, unsigned int pid,
                              uint32_t frequency)
{
#if 0
  uint32_t regval;
  uint32_t mck;
#endif

  i2cinfo("TWI%d Initializing\n", priv->twi);

  /* SVEN: TWI Slave Mode Enabled */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_SVEN);

  /* Reset the TWI */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_SWRST);
  twi_getrel(priv, SAM_TWI_RHR_OFFSET);

  /* TWI Slave Mode Disabled, TWI Master Mode Disabled. */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_SVDIS);
  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_MSDIS);

  /* Set master mode */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_MSEN);

  /* Set base frequency */

  priv->clkin = BOARD_MCK_FREQUENCY;

#if 0
  /* Determine the maximum valid frequency setting */

  mck = BOARD_MCK_FREQUENCY;
  DEBUGASSERT((mck >> 3) <= TWI_MAX_FREQUENCY);

  if (mck <= TWI_MAX_FREQUENCY)
    {
      priv->clkin = mck;
      regval      = PMC_PCR_DIV1;
    }
  else if ((mck >> 1) <= TWI_MAX_FREQUENCY)
    {
      priv->clkin = (mck >> 1);
      regval      = PMC_PCR_DIV2;
    }
  else if ((mck >> 2) <= TWI_MAX_FREQUENCY)
    {
      priv->clkin = (mck >> 2);
      regval      = PMC_PCR_DIV4;
    }
  else /* if ((mck >> 3) <= TWI_MAX_FREQUENCY) */
    {
      priv->clkin = (mck >> 3);
      regval      = PMC_PCR_DIV8;
    }

  /* Set the TWI peripheral input clock to the maximum, valid frequency */

  regval |= PMC_PCR_PID(pid) | PMC_PCR_CMD | PMC_PCR_EN;
  twi_putabs(priv, SAM_PMC_PCR, regval);
#endif

  /* Set the initial TWI data transfer frequency */

  priv->i2cfreq = 0;
  twi_setfrequency(priv, frequency);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_i2cbus_initialize
 *
 * Description:
 *   Initialize a TWI device for I2C operation
 *
 ****************************************************************************/

struct i2c_master_s *sam_i2cbus_initialize(int bus)
{
  struct twi_dev_s *priv;
  irqstate_t flags;
  uint32_t frequency;
  unsigned int pid;

  i2cinfo("Initializing TWI%d\n", bus);

  flags = enter_critical_section();

#ifdef CONFIG_SAM34_TWIM0
  if (bus == 0)
    {
      priv = &g_twi0;

      /* Enable peripheral clocking */

      sam_twi0_enableclk();

      /* Configure PIO pins */

      sam_configgpio(GPIO_TWI0_CK);
      sam_configgpio(GPIO_TWI0_D);

      /* Select the TWI frequency, and peripheral ID */

      frequency  = CONFIG_SAM34_TWIM0_FREQUENCY;
      pid        = SAM_PID_TWI0;
    }
  else
#endif
#ifdef CONFIG_SAM34_TWIM1
  if (bus == 1)
    {
      priv = &g_twi1;

      /* Enable peripheral clocking */

      sam_twi1_enableclk();

      /* Configure PIO pins */

      sam_configgpio(GPIO_TWI1_CK);
      sam_configgpio(GPIO_TWI1_D);

      /* Select the TWI frequency, and peripheral ID */

      frequency  = CONFIG_SAM34_TWIM1_FREQUENCY;
      pid        = SAM_PID_TWI1;
    }
  else
#endif
    {
      leave_critical_section(flags);
      i2cerr("ERROR: Unsupported bus: TWI%d\n", bus);
      return NULL;
    }

  /* Configure and enable the TWI hardware */

  priv->pid = pid;
  twi_hw_initialize(priv, pid, frequency);

  /* Attach Interrupt Handler */

  irq_attach(priv->irq, twi_interrupt, priv);

  /* Enable Interrupts */

  up_enable_irq(priv->irq);
  leave_critical_section(flags);
  return &priv->dev;
}

/****************************************************************************
 * Name: sam_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int sam_i2cbus_uninitialize(struct i2c_master_s * dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;

  i2cinfo("TWI%d Un-initializing\n", priv->twi);

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Cancel the watchdog timer */

  wd_cancel(&priv->timeout);

  /* Detach Interrupt Handler */

  irq_detach(priv->irq);
  return OK;
}

#endif /* CONFIG_SAM34_TWIM0 || CONFIG_SAM34_TWIM1 */
