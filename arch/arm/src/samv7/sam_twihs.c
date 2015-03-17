/*******************************************************************************
 * arch/arm/src/samv7/sam_twihs.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This driver derives from the SAMA5Dx TWIHS driver.  References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/i2c.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/sam_pmc.h"
#include "chip/sam_pinmap.h"

#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_twihs.h"

#if defined(CONFIG_SAMV7_TWIHS0) || defined(CONFIG_SAMV7_TWIHS1) || \
    defined(CONFIG_SAMV7_TWIHS2)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/

#ifndef CONFIG_SAMV7_TWIHS0_FREQUENCY
#  define CONFIG_SAMV7_TWIHS0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMV7_TWIHS1_FREQUENCY
 #define CONFIG_SAMV7_TWIHS1_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMV7_TWIHS2_FREQUENCY
 #define CONFIG_SAMV7_TWIHS2_FREQUENCY 100000
#endif

/* Driver internal definitions *************************************************/
/* If verbose I2C debug output is enable, then allow more time before we declare
 * a timeout.  The debug output from twi_interrupt will really slow things down!
 *
 * With a very slow clock (say 100,000 Hz), less than 100 usec would be required
 * to transfer on byte.  So these define a "long" timeout.
 */

#if defined(CONFIG_DEBUG_I2C) && defined(CONFIG_DEBUG_VERBOSE)
#  define TWIHS_TIMEOUT_MSPB (50)  /* 50 msec/byte */
#else
#  define TWIHS_TIMEOUT_MSPB (5)   /* 5 msec/byte */
#endif

/* Clocking to the TWIHS module(s) is provided by the main clock, divided down
 * as necessary.
 * REVISIT -- This number came from the SAMA5Dx driver.
 */

#define TWIHS_MAX_FREQUENCY 66000000   /* Maximum TWIHS frequency */

/* Macros to convert a I2C pin to a PIO open-drain output */

#define I2C_INPUT       (PIO_INPUT | PIO_CFG_PULLUP)
#define I2C_OUTPUT      (PIO_OUTPUT | PIO_CFG_OPENDRAIN | PIO_OUTPUT_SET)

#define MKI2C_INPUT(p)  (((p) & (PIO_PORT_MASK | PIO_PIN_MASK)) | I2C_INPUT)
#define MKI2C_OUTPUT(p) (((p) & (PIO_PORT_MASK | PIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ***********************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cdbg    dbg
#  define i2cvdbg   vdbg
#  define i2clldbg  lldbg
#  define i2cllvdbg llvdbg
#else
#  define i2cdbg(x...)
#  define i2cvdbg(x...)
#  define i2clldbg(x...)
#  define i2cllvdbg(x...)
#endif

/*******************************************************************************
 * Private Types
 *******************************************************************************/
/* Invariant attributes of a TWIHS bus */

struct twi_attr_s
{
  uint8_t             twi;        /* TWIHS device number (for debug output) */
  uint8_t             pid;        /* TWIHS peripheral ID */
  uint16_t            irq;        /* IRQ number for this TWIHS bus */
  gpio_pinset_t       sclcfg;     /* TWIHS CK pin configuration (SCL in I2C-ese) */
  gpio_pinset_t       sdacfg;     /* TWIHS D pin configuration (SDA in I2C-ese) */
  uintptr_t           base;       /* Base address of TWIHS registers */
  xcpt_t              handler;    /* TWIHS interrupt handler */
};

/* State of a TWIHS bus */

struct twi_dev_s
{
  struct i2c_dev_s    dev;        /* Generic I2C device */
  const struct twi_attr_s *attr;  /* Invariant attributes of TWIHS device */
  struct i2c_msg_s    *msg;       /* Message list */
  uint32_t            twiclk;     /* TWIHS input clock frequency */
#ifdef CONFIG_I2C_RESET
  uint32_t            frequency;  /* TWIHS transfer clock frequency */
#endif
  uint16_t            address;    /* Slave address */
  uint16_t            flags;      /* Transfer flags */
  bool                initd;      /* True :device has been initialized */
  uint8_t             msgc;       /* Number of message in the message list */

  sem_t               exclsem;    /* Only one thread can access at a time */
  sem_t               waitsem;    /* Wait for TWIHS transfer completion */
  WDOG_ID             timeout;    /* Watchdog to recover from bus hangs */
  volatile int        result;     /* The result of the transfer */
  volatile int        xfrd;       /* Number of bytes transfers */

  /* Debug stuff */

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
   bool               wrlast;     /* Last was a write */
   uint32_t           addrlast;   /* Last address */
   uint32_t           vallast;    /* Last value */
   int                ntimes;     /* Number of times */
#endif
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Low-level helper functions */

static void twi_takesem(sem_t *sem);
#define     twi_givesem(sem) (sem_post(sem))

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

static int twi_wait(struct twi_dev_s *priv, unsigned int size);
static void twi_wakeup(struct twi_dev_s *priv, int result);
static int twi_interrupt(struct twi_dev_s *priv);
#ifdef CONFIG_SAMV7_TWIHS0
static int twi0_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAMV7_TWIHS1
static int twi1_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAMV7_TWIHS2
static int twi2_interrupt(int irq, FAR void *context);
#endif
static void twi_timeout(int argc, uint32_t arg, ...);

static void twi_startread(struct twi_dev_s *priv, struct i2c_msg_s *msg);
static void twi_startwrite(struct twi_dev_s *priv, struct i2c_msg_s *msg);
static void twi_startmessage(struct twi_dev_s *priv, struct i2c_msg_s *msg);

/* I2C device operations */

static uint32_t twi_setfrequency(FAR struct i2c_dev_s *dev,
          uint32_t frequency);
static int twi_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int twi_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer,
          int buflen);
static int twi_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);
#ifdef CONFIG_I2C_WRITEREAD
static int twi_writeread(FAR struct i2c_dev_s *inst, const uint8_t *wbuffer,
          int wbuflen, uint8_t *rbuffer, int rbuflen);
#endif
#ifdef CONFIG_I2C_TRANSFER
static int twi_transfer(FAR struct i2c_dev_s *dev,
          FAR struct i2c_msg_s *msgs, int count);
#endif
#ifdef CONFIG_I2C_SLAVE
static int twi_setownaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int twi_registercallback((FAR struct i2c_dev_s *dev,
          int (*callback)(FAR void *arg), FAR void *arg);
#endif

/* Initialization */

static uint32_t twi_hw_setfrequency(struct twi_dev_s *priv,
          uint32_t frequency);
static void twi_hw_initialize(struct twi_dev_s *priv, uint32_t frequency);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

#ifdef CONFIG_SAMV7_TWIHS0
static const struct twi_attr_s g_twi0attr =
{
  .twi     = 0,
  .pid     = SAM_PID_TWIHS0,
  .irq     = SAM_IRQ_TWIHS0,
  .sclcfg  = GPIO_TWIHS0_CK,
  .sdacfg  = GPIO_TWIHS0_D,
  .base    = SAM_TWIHS0_BASE,
  .handler = twi0_interrupt,
};

static struct twi_dev_s g_twi0;
#endif

#ifdef CONFIG_SAMV7_TWIHS1
static const struct twi_attr_s g_twi1attr =
{
  .twi     = 1,
  .pid     = SAM_PID_TWIHS1,
  .irq     = SAM_IRQ_TWIHS1,
  .sclcfg  = GPIO_TWIHS1_CK,
  .sdacfg  = GPIO_TWIHS1_D,
  .base    = SAM_TWIHS1_BASE,
  .handler = twi1_interrupt,
};

static struct twi_dev_s g_twi1;
#endif

#ifdef CONFIG_SAMV7_TWIHS2
static const struct twi_attr_s g_twi2attr =
{
  .twi     = 2,
  .pid     = SAM_PID_TWIHS2,
  .irq     = SAM_IRQ_TWIHS2,
  .sclcfg  = GPIO_TWIHS2_CK,
  .sdacfg  = GPIO_TWIHS2_D,
  .base    = SAM_TWIHS2_BASE,
  .handler = twi2_interrupt,
};

static struct twi_dev_s g_twi2;
#endif

struct i2c_ops_s g_twiops =
{
  .setfrequency     = twi_setfrequency,
  .setaddress       = twi_setaddress,
  .write            = twi_write,
  .read             = twi_read,
#ifdef CONFIG_I2C_WRITEREAD
  .writeread        = twi_writeread,
#endif
#ifdef CONFIG_I2C_TRANSFER
  .transfer         = twi_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
  .setownaddress    = twi_setownaddress
  .registercallback = twi_registercallback
#endif
};

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: twi_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wake-ups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void twi_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

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

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
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
      lldbg("%08x->%08x\n", address, value);
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
      lldbg("%08x<-%08x\n", address, value);
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

static inline uint32_t twi_getrel(struct twi_dev_s *priv, unsigned int offset)
{
  return twi_getabs(priv, priv->attr->base + offset);
}

/****************************************************************************
 * Name: twi_putrel
 *
 * Description:
 *  Write a value to a TWIHS register using an offset relative to the TWIHS base
 *  address.
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

/*******************************************************************************
 * Name: twi_wait
 *
 * Description:
 *   Perform a I2C transfer start
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 *******************************************************************************/

static int twi_wait(struct twi_dev_s *priv, unsigned int size)
{
  uint32_t timeout;

  /* Calculate a timeout value based on the size of the transfer
   *
   *   ticks = msec-per-byte * bytes / msec-per-tick
   *
   * There is no concern about arithmetic overflow for reasonable transfer sizes.
   */

  timeout = MSEC2TICK(TWIHS_TIMEOUT_MSPB);
  if (timeout < 1)
    {
      timeout = 1;
    }

  /* Then start the timeout.  This timeout is needed to avoid hangs if/when an
   * TWIHS transfer stalls.
   */

  wd_start(priv->timeout, timeout, twi_timeout, 1, (uint32_t)priv);

  /* Wait for either the TWIHS transfer or the timeout to complete */

  do
    {
      i2cvdbg("TWIHS%d Waiting...\n", priv->attr->twi);
      twi_takesem(&priv->waitsem);
      i2cvdbg("TWIHS%d Awakened with result: %d\n",
              priv->attr->twi, priv->result);
    }
  while (priv->result == -EBUSY);

  /* We get here via twi_wakeup.  The watchdog timer has been disabled and
   * all further interrupts for the TWIHS have been disabled.
   */

  return priv->result;
}

/*******************************************************************************
 * Name: twi_wakeup
 *
 * Description:
 *   A terminal event has occurred.  Wake-up the waiting thread
 *
 *******************************************************************************/

static void twi_wakeup(struct twi_dev_s *priv, int result)
{
  /* Cancel any pending timeout */

  wd_cancel(priv->timeout);

  /* Disable any further TWIHS interrupts */

  twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_ALL);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  twi_givesem(&priv->waitsem);
}

/*******************************************************************************
 * Name: twi_interrupt
 *
 * Description:
 *   The TWIHS Interrupt Handler
 *
 *******************************************************************************/

static int twi_interrupt(struct twi_dev_s *priv)
{
  struct i2c_msg_s *msg;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;
  uint32_t regval;

  /* Retrieve masked interrupt status */

  sr      = twi_getrel(priv, SAM_TWIHS_SR_OFFSET);
  imr     = twi_getrel(priv, SAM_TWIHS_IMR_OFFSET);
  pending = sr & imr;

  i2cllvdbg("TWIHS%d pending: %08x\n", priv->attr->twi, pending);

  /* Byte received */

  msg = priv->msg;
  if ((pending & TWIHS_INT_RXRDY) != 0)
    {
      msg->buffer[priv->xfrd] = twi_getrel(priv, SAM_TWIHS_RHR_OFFSET);
      priv->xfrd++;

      /* Check for transfer complete */

      if (priv->xfrd >= msg->length)
        {
          /* The transfer is complete.  Disable the RXRDY interrupt and
           * enable the TXCOMP interrupt
           */

          twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_RXRDY);
          twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXCOMP);
        }

      /* Not yet complete, but will the next be the last byte? */

      else if (priv->xfrd == (msg->length - 1))
        {
          /* Yes, set the stop signal */

          twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_STOP);
        }
    }

  /* Byte sent*/

  else if ((pending & TWIHS_INT_TXRDY) != 0)
    {
      /* Transfer finished? */

      if (priv->xfrd >= msg->length)
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

      /* No, there are more bytes remaining to be sent */

      else
        {
          twi_putrel(priv, SAM_TWIHS_THR_OFFSET, msg->buffer[priv->xfrd]);
          priv->xfrd++;
        }
    }

  /* Transfer complete */

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

  /* Check for errors */

  else if ((pending & TWIHS_INT_ERRORS) != 0)
    {
      /* Wake up the thread with an I/O error indication */

      i2clldbg("ERROR: TWIHS%d pending: %08x\n", priv->attr->twi, pending);
      twi_wakeup(priv, -EIO);
    }

  return OK;
}

#ifdef CONFIG_SAMV7_TWIHS0
static int twi0_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi0);
}
#endif

#ifdef CONFIG_SAMV7_TWIHS1
static int twi1_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi1);
}
#endif

#ifdef CONFIG_SAMV7_TWIHS2
static int twi2_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi2);
}
#endif

/*******************************************************************************
 * Name: twi_timeout
 *
 * Description:
 *   Watchdog timer for timeout of TWIHS operation
 *
 * Assumptions:
 *   Called from the timer interrupt handler with interrupts disabled.
 *
 *******************************************************************************/

static void twi_timeout(int argc, uint32_t arg, ...)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)arg;

  i2clldbg("ERROR: TWIHS%d Timeout!\n", priv->attr->twi);
  twi_wakeup(priv, -ETIMEDOUT);
}

/*******************************************************************************
 * Name: twi_startread
 *
 * Description:
 *   Start the next read message
 *
 *******************************************************************************/

static void twi_startread(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set STOP signal if only one byte is sent*/

  if (msg->length == 1)
    {
      twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_STOP);
    }

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, TWIHS_MMR_IADRSZ_NONE | TWIHS_MMR_MREAD |
                   TWIHS_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWIHS_IADR_OFFSET, 0);

  /* Enable read interrupt and send the START condition */

  twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_RXRDY | TWIHS_INT_ERRORS);
  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_START);
}

/*******************************************************************************
 * Name: twi_startwrite
 *
 * Description:
 *   Start the next write message
 *
 *******************************************************************************/

static void twi_startwrite(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, TWIHS_MMR_IADRSZ_NONE | TWIHS_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWIHS_IADR_OFFSET, 0);

  /* Write first byte to send.*/

  twi_putrel(priv, SAM_TWIHS_THR_OFFSET, msg->buffer[priv->xfrd++]);

  /* Enable write interrupt */

  twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXRDY | TWIHS_INT_ERRORS);
}

/*******************************************************************************
 * Name: twi_startmessage
 *
 * Description:
 *   Start the next write message
 *
 *******************************************************************************/

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

/*******************************************************************************
 * I2C device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: twi_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 *******************************************************************************/

static uint32_t twi_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  uint32_t actual;

  DEBUGASSERT(dev);

  /* Get exclusive access to the device */

  twi_takesem(&priv->exclsem);

  /* And setup the clock frequency */

  actual = twi_hw_setfrequency(priv, frequency);
  twi_givesem(&priv->exclsem);
  return actual;
}

/*******************************************************************************
 * Name: twi_setaddress
 *
 * Description:
 *   Set the I2C slave address for a subsequent read/write
 *
 *******************************************************************************/

static int twi_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;

  i2cvdbg("TWIHS%d address: %02x nbits: %d\n", priv->attr->twi, addr, nbits);
  DEBUGASSERT(dev != NULL && nbits == 7);

  /* Get exclusive access to the device */

  twi_takesem(&priv->exclsem);

  /* Remember 7- or 10-bit address */

  priv->address = addr;
  priv->flags   = (nbits == 10) ? I2C_M_TEN : 0;

  twi_givesem(&priv->exclsem);
  return OK;
}

/*******************************************************************************
 * Name: twi_write
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 *******************************************************************************/

static int twi_write(FAR struct i2c_dev_s *dev, const uint8_t *wbuffer, int wbuflen)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;
  irqstate_t flags;
  int ret;

  struct i2c_msg_s msg =
  {
    .addr   = priv->address,
    .flags  = priv->flags,
    .buffer = (uint8_t *)wbuffer,  /* Override const */
    .length = wbuflen
  };

  i2cvdbg("TWIHS%d buflen: %d\n", priv->attr->twi, wbuflen);
  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the device */

  twi_takesem(&priv->exclsem);

  /* Initiate the write */

  priv->msg  = &msg;
  priv->msgc = 1;

  /* Initiate the write operation.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  twi_startwrite(priv, &msg);

  /* And wait for the write to complete.  Interrupts will be re-enabled while
   * we are waiting.
   */

  ret = twi_wait(priv, wbuflen);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  twi_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: twi_read
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 *******************************************************************************/

static int twi_read(FAR struct i2c_dev_s *dev, uint8_t *rbuffer, int rbuflen)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  irqstate_t flags;
  int ret;

  struct i2c_msg_s msg =
  {
    .addr   = priv->address,
    .flags  = priv->flags | I2C_M_READ,
    .buffer = rbuffer,
    .length = rbuflen
  };

  DEBUGASSERT(dev != NULL);
  i2cvdbg("TWIHS%d rbuflen: %d\n", priv->attr->twi, rbuflen);

  /* Get exclusive access to the device */

  twi_takesem(&priv->exclsem);

  /* Initiate the read */

  priv->msg  = &msg;
  priv->msgc = 1;

  /* Initiate the read operation.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  twi_startread(priv, &msg);

  /* And wait for the read to complete.  Interrupts will be re-enabled while
   * we are waiting.
   */

  ret = twi_wait(priv, rbuflen);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  twi_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: twi_writeread
 *
 * Description:
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_WRITEREAD
static int twi_writeread(FAR struct i2c_dev_s *dev, const uint8_t *wbuffer,
                         int wbuflen, uint8_t *rbuffer, int rbuflen)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  struct i2c_msg_s msgv[2];
  irqstate_t flags;
  int ret;

  DEBUGASSERT(dev != NULL);
  i2cvdbg("TWIHS%d wbuflen: %d rbuflen: %d\n", priv->attr->twi, wbuflen, rbuflen);

  /* Format two messages: The first is a write */

  msgv[0].addr   = priv->address;
  msgv[0].flags  = priv->flags;
  msgv[0].buffer = (uint8_t *)wbuffer;  /* Override const */
  msgv[0].length = wbuflen;

  /* The second is either a read (rbuflen > 0) or a write (rbuflen < 0) with
   * no restart.
   */

  if (rbuflen > 0)
    {
      msgv[1].flags = (priv->flags | I2C_M_READ);
    }
  else
    {
      msgv[1].flags = (priv->flags | I2C_M_NORESTART),
      rbuflen = -rbuflen;
    }

  msgv[1].addr   = priv->address;
  msgv[1].buffer = rbuffer;
  msgv[1].length = rbuflen;

  /* Get exclusive access to the device */

  twi_takesem(&priv->exclsem);

  /* Initiate the read */

  priv->msg  = msgv;
  priv->msgc = 2;

  /* Initiate the write operation.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  twi_startwrite(priv, msgv);

  /* And wait for the write/read to complete.  Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = twi_wait(priv, wbuflen + rbuflen);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  twi_givesem(&priv->exclsem);
  return ret;
}
#endif

/*******************************************************************************
 * Name: twi_setownaddress
 *
 * Description:
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_SLAVE
static int twi_setownaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
#error Not implemented
  return -ENOSYS;
}
#endif

/*******************************************************************************
 * Name: twi_registercallback
 *
 * Description:
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_SLAVE
static int twi_registercallback((FAR struct i2c_dev_s *dev,
                                int (*callback)(FAR void *arg), FAR void *arg)
{
#error Not implemented
  return -ENOSYS;
}
#endif

/*******************************************************************************
 * Name: twi_transfer
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value on failure.
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
static int twi_transfer(FAR struct i2c_dev_s *dev,
                        FAR struct i2c_msg_s *msgs, int count)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  irqstate_t flags;
  unsigned int size;
  int i;
  int ret;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);
  i2cvdbg("TWIHS%d count: %d\n", priv->attr->twi, count);

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

  twi_takesem(&priv->exclsem);

  /* Initiate the message transfer */

  priv->msg  = msgs;
  priv->msgc = count;

  /* Initiate the transfer.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  twi_startmessage(priv, msgs);

  /* And wait for the transfers to complete.  Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = twi_wait(priv, size);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  twi_givesem(&priv->exclsem);
  return ret;
}
#endif

/*******************************************************************************
 * Initialization
 *******************************************************************************/

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

/*******************************************************************************
 * Name: twi_hw_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 *******************************************************************************/

static uint32_t twi_hw_setfrequency(struct twi_dev_s *priv, uint32_t frequency)
{
  unsigned int ckdiv;
  unsigned int cldiv;
  uint32_t actual;
  uint32_t regval;

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

  /* Then setup the TWIHS Clock Waveform Generator Register, using the same
   * value for CLDIV and CHDIV (for 1:1 duty).
   */

  twi_putrel(priv, SAM_TWIHS_CWGR_OFFSET, 0);

  regval = ((uint32_t)ckdiv << TWIHS_CWGR_CKDIV_SHIFT) |
           ((uint32_t)cldiv << TWIHS_CWGR_CHDIV_SHIFT) |
           ((uint32_t)cldiv << TWIHS_CWGR_CLDIV_SHIFT);
  twi_putrel(priv, SAM_TWIHS_CWGR_OFFSET, regval);

  /* Calculate the actual I2C frequency */

  actual = (priv->twiclk / 2) / (((1 << ckdiv) * cldiv) + 2);
  i2cvdbg("TWIHS%d frequency: %d ckdiv: %d cldiv: %d actual: %d\n",
          priv->attr->twi, frequency, ckdiv, cldiv, actual);

  /* Save the requested frequency (for I2C reset) and return the
   * actual frequency.
   */

#ifdef CONFIG_I2C_RESET
  priv->frequency = frequency;
#endif
  return actual;
}

/*******************************************************************************
 * Name: twi_hw_initialize
 *
 * Description:
 *   Initialize/Re-initialize the TWIHS peripheral.  This logic performs only
 *   repeatable initialization after either (1) the one-time initialization, or
 *   (2) after each bus reset.
 *
 *******************************************************************************/

static void twi_hw_initialize(struct twi_dev_s *priv, uint32_t frequency)
{
  irqstate_t flags = irqsave();
  uint32_t regval;
  uint32_t mck;

  i2cvdbg("TWIHS%d Initializing\n", priv->attr->twi);

  /* Configure PIO pins */

  sam_configgpio(priv->attr->sclcfg);
  sam_configgpio(priv->attr->sdacfg);

  /* Enable peripheral clocking */

  twi_enableclk(priv);

  /* SVEN: TWIHS Slave Mode Enabled */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SVEN);

  /* Reset the TWIHS */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SWRST);
  (void)twi_getrel(priv, SAM_TWIHS_RHR_OFFSET);

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

  /* Set the initial TWIHS data transfer frequency */

  twi_hw_setfrequency(priv, frequency);

  /* Enable Interrupts */

  up_enable_irq(priv->attr->irq);
  irqrestore(flags);
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize a TWIHS device for I2C operation
 *
 *******************************************************************************/

struct i2c_dev_s *up_i2cinitialize(int bus)
{
  struct twi_dev_s *priv;
  uint32_t frequency;
  irqstate_t flags;
  int ret;

  i2cvdbg("Initializing TWIHS%d\n", bus);

#ifdef CONFIG_SAMV7_TWIHS0
  if (bus == 0)
    {
      /* Select up TWIHS0 and setup invariant attributes */

      priv       = &g_twi0;
      priv->attr = &g_twi0attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS0_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TWIHS1
  if (bus == 1)
    {
      /* Select up TWIHS1 and setup invariant attributes */

      priv       = &g_twi1;
      priv->attr = &g_twi1attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS1_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TWIHS2
  if (bus == 2)
    {
      /* Select up TWIHS2 and setup invariant attributes */

      priv       = &g_twi2;
      priv->attr = &g_twi2attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS2_FREQUENCY;
    }
  else
#endif
    {
      i2cdbg("ERROR: Unsupported bus: TWIHS%d\n", bus);
      return NULL;
    }

  /* Perform one-time TWIHS initialization */

  flags = irqsave();

  /* Has the device already been initialized? */

  if (!priv->initd)
    {
      /* Allocate a watchdog timer */

      priv->timeout = wd_create();
      if (priv->timeout == NULL)
        {
          idbg("ERROR: Failed to allocate a timer\n");
          goto errout_with_irq;
        }

      /* Attach Interrupt Handler */

      ret = irq_attach(priv->attr->irq, priv->attr->handler);
      if (ret < 0)
        {
          idbg("ERROR: Failed to attach irq %d\n", priv->attr->irq);
          goto errout_with_wdog;
        }

      /* Initialize the TWIHS driver structure */

      priv->dev.ops = &g_twiops;
      priv->address = 0;
      priv->flags   = 0;

      (void)sem_init(&priv->exclsem, 0, 1);
      (void)sem_init(&priv->waitsem, 0, 0);

      /* Perform repeatable TWIHS hardware initialization */

      twi_hw_initialize(priv, frequency);

      /* Now it has been initialized */

      priv->initd = true;
    }

  irqrestore(flags);
  return &priv->dev;

errout_with_wdog:
  wd_delete(priv->timeout);
  priv->timeout = NULL;

errout_with_irq:
  irqrestore(flags);
  return NULL;
}

/*******************************************************************************
 * Name: up_i2cuninitalize
 *
 * Description:
 *   Uninitialize an I2C device
 *
 *******************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s *dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;
  irqstate_t flags;

  i2cvdbg("TWIHS%d Un-initializing\n", priv->attr->twi);

  /* Disable TWIHS interrupts */

  flags = irqsave();
  up_disable_irq(priv->attr->irq);

  /* Reset data structures */

  sem_destroy(&priv->exclsem);
  sem_destroy(&priv->waitsem);

  /* Free the watchdog timer */

  wd_delete(priv->timeout);
  priv->timeout = NULL;

  /* Detach Interrupt Handler */

  (void)irq_detach(priv->attr->irq);

  priv->initd = false;
  irqrestore(flags);
  return OK;
}

/************************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
int up_i2creset(FAR struct i2c_dev_s *dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  unsigned int clockcnt;
  unsigned int stretchcnt;
  uint32_t sclpin;
  uint32_t sdapin;
  int ret;

  ASSERT(priv);

  /* Get exclusive access to the TWIHS device */

  twi_takesem(&priv->exclsem);

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

  /* Peripheral clocking must be enabled in order to read valid data from
   * the output pin (clocking is enabled automatically for pins configured
   * as inputs).
   */

  sam_pio_forceclk(sclpin, true);
  sam_pio_forceclk(sdapin, true);

  /* Clock the bus until any slaves currently driving it low let it float.
   * Reading from the output will return the actual sensed level on the
   * SDA pin (not the level that we wrote).
   */

  clockcnt = 0;
  while (sam_pioread(sdapin) == false)
    {
      /* Give up if we have tried too hard */

      if (clockcnt++ > 10)
        {
          ret = -ETIMEDOUT;
          goto errout_with_lock;
        }

      /* Sniff to make sure that clock stretching has finished.  SCL should
       * be floating high here unless something is driving it low.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretchcnt = 0;
      while (sam_pioread(sclpin) == false)
        {
          /* Give up if we have tried too hard */

          if (stretchcnt++ > 10)
            {
              ret = -EAGAIN;
              goto errout_with_lock;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      sam_piowrite(sclpin, false);
      up_udelay(10);

      /* Drive SCL high (floating) again */

      sam_piowrite(sclpin, true);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  sam_piowrite(sdapin, false);
  up_udelay(10);
  sam_piowrite(sclpin, false);
  up_udelay(10);

  sam_piowrite(sclpin, true);
  up_udelay(10);
  sam_piowrite(sdapin, true);
  up_udelay(10);

  /* Clocking is no longer forced */

  sam_pio_forceclk(sclpin, false);
  sam_pio_forceclk(sdapin, false);

  /* Re-initialize the port hardware */

  twi_hw_initialize(priv, priv->frequency);
  ret = OK;

errout_with_lock:

  /* Release our lock on the bus */

  twi_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_I2C_RESET */
#endif /* CONFIG_SAMV7_TWIHS0 || CONFIG_SAMV7_TWIHS1 || CONFIG_SAMV7_TWIHS2 */
