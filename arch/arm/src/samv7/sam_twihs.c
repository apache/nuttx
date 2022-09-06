/****************************************************************************
 * arch/arm/src/samv7/sam_twihs.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
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
#include "sam_twihs.h"

#if defined(CONFIG_SAMV7_TWIHS0) || defined(CONFIG_SAMV7_TWIHS1) || \
    defined(CONFIG_SAMV7_TWIHS2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAMV7_TWIHS0_FREQUENCY
#  define CONFIG_SAMV7_TWIHS0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMV7_TWIHS1_FREQUENCY
#  define CONFIG_SAMV7_TWIHS1_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMV7_TWIHS2_FREQUENCY
#  define CONFIG_SAMV7_TWIHS2_FREQUENCY 100000
#endif

#ifndef CONFIG_DEBUG_I2C_INFO
#  undef CONFIG_SAMV7_TWIHSHS_REGDEBUG
#endif

/* Driver internal definitions **********************************************/

/* If verbose I2C debug output is enable, then allow more time before we
 * declare a timeout.  The debug output from twi_interrupt will really slow
 * things down!
 *
 * With a very slow clock (say 100,000 Hz), less than 100 usec would be
 * required to transfer on byte.  So these define a "long" timeout.
 */

#ifdef CONFIG_DEBUG_I2C_INFO
#  define TWIHS_TIMEOUT_MSPB (50)  /* 50 msec/byte */
#else
#  define TWIHS_TIMEOUT_MSPB (5)   /* 5 msec/byte */
#endif

/* Clocking to the TWIHS module(s) is provided by the main clock, divided
 * down as necessary.
 * REVISIT -- This number came from the SAMA5Dx driver.
 */

#define TWIHS_MAX_FREQUENCY 66000000   /* Maximum TWIHS frequency */

/* Macros to convert a I2C pin to a GPIO open-drain output */

#define I2C_INPUT       (GPIO_INPUT | GPIO_CFG_PULLUP)
#define I2C_OUTPUT      (GPIO_OUTPUT | GPIO_CFG_OPENDRAIN | GPIO_OUTPUT_SET)

#define MKI2C_INPUT(p)  (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_INPUT)
#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Invariant attributes of a TWIHS bus */

struct twi_attr_s
{
  uint8_t             twi;        /* TWIHS device number (for debug output) */
  uint8_t             pid;        /* TWIHS peripheral ID */
  uint8_t             glitchfltr; /* Pulse width of a glich to be suppressed by the filter */
  bool                s_master;   /* true: Single-Master Mode active */
  uint16_t            irq;        /* IRQ number for this TWIHS bus */
  gpio_pinset_t       sclcfg;     /* TWIHS CK pin configuration (SCL in I2C-ese) */
  gpio_pinset_t       sdacfg;     /* TWIHS D pin configuration (SDA in I2C-ese) */
  uintptr_t           base;       /* Base address of TWIHS registers */
};

/* State of a TWIHS bus */

struct twi_dev_s
{
  struct i2c_master_s dev;        /* Generic I2C device */
  const struct twi_attr_s *attr;  /* Invariant attributes of TWIHS device */
  struct i2c_msg_s    *msg;       /* Message list */
  uint32_t            twiclk;     /* TWIHS input clock frequency */
  uint32_t            frequency;  /* TWIHS transfer clock frequency */
  int                 refs;       /* Reference count */
  uint8_t             msgc;       /* Number of message in the message list */

  mutex_t             lock;       /* Only one thread can access at a time */
  sem_t               waitsem;    /* Wait for TWIHS transfer completion */
  struct wdog_s       timeout;    /* Watchdog to recover from bus hangs */
  volatile int        result;     /* The result of the transfer */
  volatile int        xfrd;       /* Number of bytes transfers */

  /* Debug stuff */

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
  bool                wrlast;     /* Last was a write */
  uint32_t            addrlast;   /* Last address */
  uint32_t            vallast;    /* Last value */
  int                 ntimes;     /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helper functions */

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
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

static int  twi_wait(struct twi_dev_s *priv, unsigned int size);
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
static int  twi_reset_internal(struct i2c_master_s *dev);
static int  twi_reset(struct i2c_master_s * dev);
#endif

/* Initialization */

static void twi_setfrequency(struct twi_dev_s *priv, uint32_t frequency);
static void twi_hw_initialize(struct twi_dev_s *priv, uint32_t frequency);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TWIHS0
static const struct twi_attr_s g_twi0attr =
{
  .twi        = 0,
  .pid        = SAM_PID_TWIHS0,
  .irq        = SAM_IRQ_TWIHS0,
  .glitchfltr = CONFIG_SAMV7_TWIHS0_GLITCH_FILTER,
#ifdef CONFIG_SAMV7_TWIHS0_SINGLE_MASTER
  .s_master   = true,
#else
  .s_master   = false,
#endif
  .sclcfg     = GPIO_TWIHS0_CK,
  .sdacfg     = GPIO_TWIHS0_D,
  .base       = SAM_TWIHS0_BASE,
};

static struct twi_dev_s g_twi0 =
{
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .waitsem    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
};
#endif

#ifdef CONFIG_SAMV7_TWIHS1
static const struct twi_attr_s g_twi1attr =
{
  .twi        = 1,
  .pid        = SAM_PID_TWIHS1,
  .irq        = SAM_IRQ_TWIHS1,
  .glitchfltr = CONFIG_SAMV7_TWIHS1_GLITCH_FILTER,
#ifdef CONFIG_SAMV7_TWIHS1_SINGLE_MASTER
  .s_master   = true,
#else
  .s_master   = false,
#endif
  .sclcfg     = GPIO_TWIHS1_CK,
  .sdacfg     = GPIO_TWIHS1_D,
  .base       = SAM_TWIHS1_BASE,
};

static struct twi_dev_s g_twi1 =
{
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .waitsem    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
};
#endif

#ifdef CONFIG_SAMV7_TWIHS2
static const struct twi_attr_s g_twi2attr =
{
  .twi        = 2,
  .pid        = SAM_PID_TWIHS2,
  .irq        = SAM_IRQ_TWIHS2,
  .glitchfltr = CONFIG_SAMV7_TWIHS2_GLITCH_FILTER,
#ifdef CONFIG_SAMV7_TWIHS2_SINGLE_MASTER
  .s_master   = true,
#else
  .s_master   = false,
#endif
  .sclcfg     = GPIO_TWIHS2_CK,
  .sdacfg     = GPIO_TWIHS2_D,
  .base       = SAM_TWIHS2_BASE,
};

static struct twi_dev_s g_twi2 =
{
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .waitsem    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
};
#endif

static const struct i2c_ops_s g_twiops =
{
  .transfer   = twi_transfer
#ifdef CONFIG_I2C_RESET
  , .reset    = twi_reset
#endif
};

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

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
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

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
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

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
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
 *  Read a TWIHS register using an offset relative to the TWIHS base address
 *
 ****************************************************************************/

static inline uint32_t twi_getrel(struct twi_dev_s *priv,
                                  unsigned int offset)
{
  return twi_getabs(priv, priv->attr->base + offset);
}

/****************************************************************************
 * Name: twi_putrel
 *
 * Description:
 *  Write a value to a TWIHS register using an offset relative to the TWIHS
 *  base address.
 *
 ****************************************************************************/

static inline void twi_putrel(struct twi_dev_s *priv, unsigned int offset,
                              uint32_t value)
{
  twi_putabs(priv, priv->attr->base + offset, value);
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

static int twi_wait(struct twi_dev_s *priv, unsigned int size)
{
  uint32_t timeout;
  int ret;

  /* Calculate a timeout value based on the size of the transfer
   *
   *   ticks = msec-per-byte * bytes / msec-per-tick
   *
   * There is no concern about arithmetic overflow for reasonable transfer
   * sizes.
   */

  timeout = MSEC2TICK(TWIHS_TIMEOUT_MSPB);
  if (timeout < 1)
    {
      timeout = 1;
    }

  /* Then start the timeout.  This timeout is needed to avoid hangs if/when
   * a TWIHS transfer stalls.
   */

  wd_start(&priv->timeout, (timeout * size),
           twi_timeout, (wdparm_t)priv);

  /* Wait for either the TWIHS transfer or the timeout to complete */

  do
    {
      i2cinfo("TWIHS%d Waiting...\n", priv->attr->twi);
      ret = nxsem_wait(&priv->waitsem);
      i2cinfo("TWIHS%d Awakened with result: %d\n",
              priv->attr->twi, priv->result);

      if (ret < 0)
        {
          wd_cancel(&priv->timeout);
          return ret;
        }
    }
  while (priv->result == -EBUSY);

  /* We get here via twi_wakeup.  The watchdog timer has been disabled and
   * all further interrupts for the TWIHS have been disabled.
   */

#ifdef CONFIG_I2C_RESET
  /* Check if an Arbitration Lost has occurred */

  if (priv->result == -EUSERS)
    {
      /* Something bad happened on the bus so force a reset */

      priv->result = twi_reset_internal(&priv->dev);

      /* Although the reset was successful tell the higher driver that it's
       * transfer has failed and should be repeated.
       */

      if (priv->result == OK)
        {
          priv->result = -EIO;
        }
    }
#endif

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

  /* Disable any further TWIHS interrupts */

  twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_ALL);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: twi_interrupt
 *
 * Description:
 *   The TWIHS Interrupt Handler
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

  sr      = twi_getrel(priv, SAM_TWIHS_SR_OFFSET);
  imr     = twi_getrel(priv, SAM_TWIHS_IMR_OFFSET);
  pending = sr & imr;

  i2cinfo("TWIHS%d pending: %08" PRIx32 "\n", priv->attr->twi, pending);

  /* Byte received */

  msg = priv->msg;
  if ((pending & TWIHS_INT_RXRDY) != 0)
    {
      msg->buffer[priv->xfrd] = twi_getrel(priv, SAM_TWIHS_RHR_OFFSET);
      priv->xfrd++;

      /* Check for transfer complete */

      if (priv->xfrd >= msg->length)
        {
          struct i2c_msg_s *next = (msg + 1);

          /* Is there another message to after this one?  Does it require a
           * restart?
           */

          if (priv->msgc <= 1 || (next->flags & I2C_M_NOSTART) == 0)
            {
              /* The transfer is complete.  Disable the RXRDY interrupt and
               * enable the TXCOMP interrupt
               */

              twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_RXRDY);
              twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXCOMP);
            }
          else
            {
              /* No.. just switch to the next message and continue
               * receiving.  On the next RXRDY, we will continue with the
               * first byte of the next message.
               */

              DEBUGASSERT((next->flags & I2C_M_READ) != 0);
              priv->msg = next;
              priv->msgc--;
              priv->xfrd = 0;
            }
        }

      /* Not yet complete, but will the next be the last byte? */

      else if (priv->xfrd == (msg->length - 1))
        {
          struct i2c_msg_s *next = (msg + 1);

          /* Is there another message to after this one?  Does it require a
           * restart?
           */

          if (priv->msgc <= 1 || (next->flags & I2C_M_NOSTART) == 0)
            {
              /* This is the last message OR a restart is required before
               * the next message.  Send the stop signal.
               */

              twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_STOP);
            }
        }
    }

#ifdef CONFIG_I2C_RESET
  /* If Single-Master Mode is enabled and we lost arbitration (someone else
   * or an EMC-Pulse did something on the bus) something went very wrong. So
   * we end the current transfer with an EUSERS. The wait function will then
   * reset the bus so further communication can take place.
   */

  else if ((priv->attr->s_master) && ((pending & TWIHS_INT_ARBLST) != 0))
    {
      /* Wake up the thread with an Arbitration Lost error indication */

      i2cerr("ERROR: TWIHS%d Arbitration Lost\n", priv->attr->twi);
      twi_wakeup(priv, -EUSERS);
    }
#endif

  /* Check for errors.  We must check for errors *before* checking TXRDY or
   * TXCMP because the error can be signaled in combination with TXRDY or
   * TXCOMP.
   */

  else if ((pending & TWIHS_INT_ERRORS) != 0)
    {
      /* Wake up the thread with an I/O error indication */

      i2cerr("ERROR: TWIHS%d pending: %08" PRIx32 "\n",
             priv->attr->twi, pending);
      twi_wakeup(priv, -EIO);
    }

  /* Byte sent */

  else if ((pending & TWIHS_INT_TXRDY) != 0)
    {
      /* Transfer finished? */

      if (priv->xfrd >= msg->length)
        {
          struct i2c_msg_s *next = (msg + 1);

          /* Is there another message to after this one?  Does it require a
           * restart?
           */

          if (priv->msgc <= 1 || (next->flags & I2C_M_NOSTART) == 0)
            {
              /* The transfer is complete.  Disable the TXRDY interrupt and
               * enable the TXCOMP interrupt
               */

              twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_TXRDY);
              twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXCOMP);

              /* Send the STOP condition */

              regval  = twi_getrel(priv, SAM_TWIHS_CR_OFFSET);
              regval |= TWIHS_CR_STOP;
              twi_putrel(priv, SAM_TWIHS_CR_OFFSET, regval);
            }
          else
            {
              /* No.. just switch to the next message and continue
               * sending.
               */

              DEBUGASSERT((next->flags & I2C_M_READ) == 0);
              priv->msg = next;
              priv->msgc--;

              twi_putrel(priv, SAM_TWIHS_THR_OFFSET, next->buffer[0]);
              priv->xfrd = 1;
            }
        }

      /* No, there are more bytes remaining to be sent */

      else
        {
          twi_putrel(priv, SAM_TWIHS_THR_OFFSET, msg->buffer[priv->xfrd]);
          priv->xfrd++;
        }
    }

  /* Transfer complete,  Occurs on message only if the STOP bit was set on
   * the previously sent byte.
   */

  else if ((pending & TWIHS_INT_TXCOMP) != 0)
    {
      twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_TXCOMP);

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
 *   Watchdog timer for timeout of TWIHS operation
 *
 * Assumptions:
 *   Called from the timer interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static void twi_timeout(wdparm_t arg)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)arg;

  i2cerr("ERROR: TWIHS%d Timeout!\n", priv->attr->twi);
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
      twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_STOP);
    }

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET,
             TWIHS_MMR_IADRSZ_NONE | TWIHS_MMR_MREAD |
             TWIHS_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWIHS_IADR_OFFSET, 0);

  /* Enable read interrupt and send the START condition */

  twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_RXRDY | TWIHS_INT_ERRORS);
  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_START);
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

  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET,
             TWIHS_MMR_IADRSZ_NONE | TWIHS_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWIHS_IADR_OFFSET, 0);

  /* Write first byte to send. */

  twi_putrel(priv, SAM_TWIHS_THR_OFFSET, msg->buffer[priv->xfrd++]);

  /* Enable write interrupt */

  twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXRDY | TWIHS_INT_ERRORS);
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
 * Returned Value:
 *   Returns zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int twi_transfer(struct i2c_master_s *dev,
                        struct i2c_msg_s *msgs, int count)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  irqstate_t flags;
  unsigned int size;
#ifdef CONFIG_I2C_RESET
  uint32_t sr;
#endif
  int i;
  int ret;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);
  i2cinfo("TWIHS%d count: %d\n", priv->attr->twi, count);

  /* Calculate the total transfer size so that we can calculate a reasonable
   * timeout value.
   */

  size = 0;
  for (i = 0; i < count; i++)
    {
      size += msgs[i].length;
    }

  DEBUGASSERT(size > 0);

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

#ifdef CONFIG_I2C_RESET
  /* When we are in Single Master Mode check if the bus is ready (no stuck
   * DATA or CLK line).
   * Otherwise initiate a bus reset.
   */

  if (priv->attr->s_master)
    {
      sr = twi_getrel(priv, SAM_TWIHS_SR_OFFSET);
      if (((sr & TWIHS_INT_SDA) == 0) || ((sr & TWIHS_INT_SCL) == 0))
        {
          ret = twi_reset_internal(&priv->dev);
          if (ret != OK)
            {
              goto errout;
            }
        }
    }
#endif

  /* Initiate the transfer.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = enter_critical_section();
  twi_startmessage(priv, msgs);

  /* And wait for the transfers to complete.  Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = twi_wait(priv, size);
  leave_critical_section(flags);

#ifdef CONFIG_I2C_RESET
errout:
#endif
  if (ret < 0)
    {
      i2cerr("ERROR: Transfer failed: %d\n", ret);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: twi_reset_internal
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *   This function can be called from inside the driver while the TWIHS
 *   device is already locked, so we must not handle any semaphores inside.
 *   To initiate a bus reset from outside the driver use twi_reset(dev).
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int twi_reset_internal(struct i2c_master_s *dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  unsigned int clockcnt;
  unsigned int stretchcnt;
  uint32_t sclpin;
  uint32_t sdapin;
  uint8_t wait_us;
  int ret;

  /* Disable TWIHS interrupts */

  up_disable_irq(priv->attr->irq);

  /* Use PIO configuration to un-wedge the bus.
   *
   * Reconfigure both pins as open drain outputs with initial output value
   * "high" (i.e., floating since these are open-drain outputs).
   */

  sclpin = MKI2C_OUTPUT(priv->attr->sclcfg);
  sdapin = MKI2C_OUTPUT(priv->attr->sdacfg);

  sam_configgpio(sclpin);
  sam_configgpio(sdapin);

  /* Clock the bus until any slaves currently driving it low let it float.
   * Reading from the output will return the actual sensed level on the
   * SDA pin (not the level that we wrote).
   */

  /* Set the wait-time according to the TWI-Bus-Frequency */

  if (priv->frequency >= 330000)
    {
      wait_us = 3;
    }
  else
    {
      wait_us = 10;
    }

  clockcnt = 0;
  while (sam_gpioread(sdapin) == false)
    {
      /* Give up if we have tried too hard */

      if (clockcnt++ > 10)
        {
          ret = -ETIMEDOUT;
          goto errout;
        }

      /* Sniff to make sure that clock stretching has finished.  SCL should
       * be floating high here unless something is driving it low.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretchcnt = 0;
      while (sam_gpioread(sclpin) == false)
        {
          /* Give up if we have tried too hard */

          if (stretchcnt++ > 10)
            {
              ret = -EAGAIN;
              goto errout;
            }

          up_udelay(wait_us);
        }

      /* Drive SCL low */

      sam_gpiowrite(sclpin, false);
      up_udelay(wait_us);

      /* Drive SCL high (floating) again */

      sam_gpiowrite(sclpin, true);
      up_udelay(wait_us);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  sam_gpiowrite(sdapin, false);
  up_udelay(wait_us);
  sam_gpiowrite(sclpin, false);
  up_udelay(wait_us);

  sam_gpiowrite(sclpin, true);
  up_udelay(wait_us);
  sam_gpiowrite(sdapin, true);
  up_udelay(wait_us);

  /* Re-initialize the port hardware */

  twi_hw_initialize(priv, priv->frequency);
  ret = OK;

errout:
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: twi_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C
 *   devices.  This function can be called from outside the driver, so lock
 *   the TWIHS Device and then let the internal reset function do the work.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int twi_reset(struct i2c_master_s *dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Get exclusive access to the TWIHS device */

  ret = nxmutex_lock(&priv->lock);
  if (ret >= 0)
    {
      /* Do the reset-procedure */

      ret = twi_reset_internal(dev);

      /* Release our lock on the bus */

      nxmutex_unlock(&priv->lock);
    }

  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: twi_enableclk
 *
 * Description:
 *   Enable clocking on the selected TWIHS
 *
 ****************************************************************************/

static void twi_enableclk(struct twi_dev_s *priv)
{
  int pid;

  /* Get the peripheral ID associated with the TWIHS device port and enable
   * clocking to the TWIHS block.
   */

  pid = priv->attr->pid;
  if (pid < 32)
    {
      sam_enableperiph0(pid);
    }
  else
    {
      sam_enableperiph1(pid);
    }
}

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

  if (frequency != priv->frequency)
    {
      /* Configure TWIHS output clocking, trying each value of CKDIV {0..7} */

      for (ckdiv = 0; ckdiv < 8; ckdiv++)
        {
          /* Calculate the CLDIV value using the current CKDIV guess */

          cldiv = ((priv->twiclk / (frequency << 1)) - 4) / (1 << ckdiv);

          /* Is CLDIV in range? */

          if (cldiv <= 255)
            {
              /* Yes, break out and use it */

              break;
            }
        }

      /* Then setup the TWIHS Clock Waveform Generator Register, using the
       * same value for CLDIV and CHDIV (for 1:1 duty).
       */

      twi_putrel(priv, SAM_TWIHS_CWGR_OFFSET, 0);

      regval = ((uint32_t)ckdiv << TWIHS_CWGR_CKDIV_SHIFT) |
               ((uint32_t)cldiv << TWIHS_CWGR_CHDIV_SHIFT) |
               ((uint32_t)cldiv << TWIHS_CWGR_CLDIV_SHIFT);
      twi_putrel(priv, SAM_TWIHS_CWGR_OFFSET, regval);

      /* Save the requested frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: twi_hw_initialize
 *
 * Description:
 *   Initialize/Re-initialize the TWIHS peripheral.  This logic performs
 *   only repeatable initialization after either (1) the one-time
 *   initialization, or (2) after each bus reset.
 *
 ****************************************************************************/

static void twi_hw_initialize(struct twi_dev_s *priv, uint32_t frequency)
{
  irqstate_t flags = enter_critical_section();
  uint32_t regval;
  uint32_t mck;

  i2cinfo("TWIHS%d Initializing\n", priv->attr->twi);

  /* Configure PIO pins */

  sam_configgpio(priv->attr->sclcfg);
  sam_configgpio(priv->attr->sdacfg);

  /* Enable peripheral clocking */

  twi_enableclk(priv);

  /* SVEN: TWIHS Slave Mode Enabled */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SVEN);

  /* Reset the TWIHS */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SWRST);
  twi_getrel(priv, SAM_TWIHS_RHR_OFFSET);

  /* TWIHS Slave Mode Disabled, TWIHS Master Mode Disabled. */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SVDIS);
  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_MSDIS);

  /* Set master mode */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_MSEN);

  /* Determine the maximum valid frequency setting */

  mck = BOARD_MCK_FREQUENCY;

#ifdef SAMV7_HAVE_PMC_PCR_DIV
  /* Select the optimal value for the PCR DIV field */

  DEBUGASSERT((mck >> 3) <= TWIHS_MAX_FREQUENCY);
  if (mck <= TWIHS_MAX_FREQUENCY)
    {
      priv->twiclk = mck;
      regval       = PMC_PCR_DIV1;
    }
  else if ((mck >> 1) <= TWIHS_MAX_FREQUENCY)
    {
      priv->twiclk = (mck >> 1);
      regval       = PMC_PCR_DIV2;
    }
  else if ((mck >> 2) <= TWIHS_MAX_FREQUENCY)
    {
      priv->twiclk = (mck >> 2);
      regval       = PMC_PCR_DIV4;
    }
  else /* if ((mck >> 3) <= TWIHS_MAX_FREQUENCY) */
    {
      priv->twiclk = (mck >> 3);
      regval       = PMC_PCR_DIV8;
    }

#else
  /* No DIV field in the PCR register */

  priv->twiclk     = mck;
  regval           = 0;

#endif /* SAMV7_HAVE_PMC_PCR_DIV */

  /* Set the TWIHS peripheral input clock to the maximum, valid frequency */

  regval |= PMC_PCR_PID(priv->attr->pid) | PMC_PCR_CMD | PMC_PCR_EN;
  twi_putabs(priv, SAM_PMC_PCR, regval);

  /* Set the TWIHS Input Filters */

  if (priv->attr->glitchfltr)
    {
      regval = TWIHS_FILTR_FILT | TWIHS_FILTR_THRES(priv->attr->glitchfltr);
    }
  else
    {
      regval = 0;
    }

  twi_putrel(priv, SAM_TWIHS_FILTR_OFFSET, regval);

  /* Set the initial TWIHS data transfer frequency */

  priv->frequency  = 0;
  twi_setfrequency(priv, frequency);

  /* Enable Interrupts */

  up_enable_irq(priv->attr->irq);
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_i2cbus_initialize
 *
 * Description:
 *   Initialize a TWIHS device for I2C operation
 *
 ****************************************************************************/

struct i2c_master_s *sam_i2cbus_initialize(int bus)
{
  struct twi_dev_s *priv;
  uint32_t frequency;
  const struct twi_attr_s *attr = 0;
  irqstate_t flags;
  int ret;

  i2cinfo("Initializing TWIHS%d\n", bus);

#ifdef CONFIG_SAMV7_TWIHS0
  if (bus == 0)
    {
      /* Select up TWIHS0 and setup invariant attributes */

      priv = &g_twi0;
      attr = &g_twi0attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS0_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TWIHS1
  if (bus == 1)
    {
      /* Select up TWIHS1 and setup invariant attributes */

      priv = &g_twi1;
      attr = &g_twi1attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS1_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TWIHS2
  if (bus == 2)
    {
      /* Select up TWIHS2 and setup invariant attributes */

      priv = &g_twi2;
      attr = &g_twi2attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS2_FREQUENCY;
    }
  else
#endif
    {
      i2cerr("ERROR: Unsupported bus: TWIHS%d\n", bus);
      return NULL;
    }

  flags = enter_critical_section();

  /* Has the device already been initialized? */

  if ((volatile int)priv->refs++ == 0)
    {
      /* Perform one-time TWIHS initialization */

      priv->attr = attr;

      /* Attach Interrupt Handler */

      ret = irq_attach(priv->attr->irq, twi_interrupt, priv);
      if (ret < 0)
        {
          ierr("ERROR: Failed to attach irq %d\n", priv->attr->irq);
          goto errout_with_lock;
        }

      /* Initialize the TWIHS driver structure */

      priv->dev.ops = &g_twiops;

      /* Perform repeatable TWIHS hardware initialization */

      twi_hw_initialize(priv, frequency);
    }

  leave_critical_section(flags);
  return &priv->dev;

errout_with_lock:
  priv->refs--;
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: sam_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C device
 *
 ****************************************************************************/

int sam_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  i2cinfo("TWIHS%d Un-initializing refs:%d\n", priv->attr->twi, priv->refs);

  /* Disable TWIHS interrupts */

  flags = enter_critical_section();

  if (--priv->refs == 0)
    {
      up_disable_irq(priv->attr->irq);

      /* Cancel the watchdog timer */

      wd_cancel(&priv->timeout);

      /* Detach Interrupt Handler */

      irq_detach(priv->attr->irq);
    }

  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_SAMV7_TWIHS0 || CONFIG_SAMV7_TWIHS1 || CONFIG_SAMV7_TWIHS2 */
