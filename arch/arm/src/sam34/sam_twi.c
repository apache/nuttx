/*******************************************************************************
 * arch/arm/src/sam34/sam_twi.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA34 Series Data Sheet
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

#include <nuttx/wdog.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/sam_pmc.h"
#include "chip/sam_pinmap.h"

#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_twi.h"

#if defined(CONFIG_SAM34_TWI0) || defined(CONFIG_SAM34_TWI1)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/

#ifndef CONFIG_SAM34_TWI0_FREQUENCY
#  define CONFIG_SAM34_TWI0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM34_TWI1_FREQUENCY
 #define CONFIG_SAM34_TWI1_FREQUENCY 100000
#endif

/* Driver internal definitions *************************************************/

#define TWI_TIMEOUT ((100 * CLK_TCK) / 1000) /* 100 mS */

/* Clocking to the TWO module(s) is provided by the main clocked, divided down
 * as necessary.
 */

#define TWI_MAX_FREQUENCY 66000000   /* Maximum TWI frequency */

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

struct twi_dev_s
{
  struct i2c_dev_s    dev;        /* Generic I2C device */
  struct i2c_msg_s    *msg;       /* Message list */
  uintptr_t           base;       /* Base address of registers */
  uint32_t            frequency;  /* TWI input clock frequency */
  uint32_t            deffreq;    /* Selected TWI frequency */
  uint16_t            irq;        /* IRQ number for this device */
  uint16_t            address;    /* Slave address */
  uint16_t            flags;      /* Transfer flags */
  uint8_t             msgc;       /* Number of message in the message list */
  uint8_t             twi;        /* TWI peripheral number (for debug output) */
  uint8_t             pid;        /* TWI peripheral ID */

  sem_t               exclsem;    /* Only one thread can access at a time */
  sem_t               waitsem;    /* Wait for TWI transfer completion */
  WDOG_ID             timeout;    /* Watchdog to recover from bus hangs */
  volatile int        result;     /* The result of the transfer */
  volatile int        xfrd;       /* Number of bytes transfers */

  /* Debug stuff */

#ifdef CONFIG_SAM34_TWI_REGDEBUG
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

static int twi_wait(struct twi_dev_s *priv);
static void twi_wakeup(struct twi_dev_s *priv, int result);
static int twi_interrupt(struct twi_dev_s *priv);
#ifdef CONFIG_SAM34_TWI0
static int twi0_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAM34_TWI1
static int twi1_interrupt(int irq, FAR void *context);
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
static int twi_registercallback(FAR struct i2c_dev_s *dev,
          int (*callback)(void));
#endif

/* Initialization */

static uint32_t twi_hw_setfrequency(struct twi_dev_s *priv,
          uint32_t frequency);
static void twi_hw_initialize(struct twi_dev_s *priv, unsigned int pid,
          uint32_t frequency);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

#ifdef CONFIG_SAM34_TWI0
static struct twi_dev_s g_twi0;
#endif

#ifdef CONFIG_SAM34_TWI1
static struct twi_dev_s g_twi1;
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

#ifdef CONFIG_SAM34_TWI_REGDEBUG
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

#ifdef CONFIG_SAM34_TWI_REGDEBUG
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
 *  Read a TWI register using an offset relative to the TWI base address
 *
 ****************************************************************************/

static inline uint32_t twi_getrel(struct twi_dev_s *priv, unsigned int offset)
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

static int twi_wait(struct twi_dev_s *priv)
{
  /* Start a timeout to avoid hangs */

  wd_start(priv->timeout, TWI_TIMEOUT, twi_timeout, 1, (uint32_t)priv);

  /* Wait for either the TWI transfer or the timeout to complete */

  do
    {
      i2clldbg("TWI%d Waiting...\n", priv->twi);
      twi_takesem(&priv->waitsem);
      i2clldbg("TWI%d Awakened with result: %d\n", priv->twi, priv->result);
    }
  while (priv->result == -EBUSY);

  /* We get here via twi_wakeup.  The watchdog timer has been disabled and
   * all further interrupts for the TWI have been disabled.
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

  /* Disable any further TWI interrupts */

  twi_putrel(priv, SAM_TWI_IDR_OFFSET, TWI_INT_ALL);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  twi_givesem(&priv->waitsem);
}

/*******************************************************************************
 * Name: twi_interrupt
 *
 * Description:
 *   The TWI Interrupt Handler
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

  sr      = twi_getrel(priv, SAM_TWI_SR_OFFSET);
  imr     = twi_getrel(priv, SAM_TWI_IMR_OFFSET);
  pending = sr & imr;

  i2cllvdbg("TWI%d pending: %08x\n", priv->twi, pending);

  msg = priv->msg;

  /* Check for errors */

  if ((pending & TWI_INT_ERRORS) != 0)
    {
      /* Wake up the thread with an I/O error indication */

      i2clldbg("ERROR: TWI%d pending: %08x\n", priv->twi, pending);
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

  /* Byte sent*/

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

#ifdef CONFIG_SAM34_TWI0
static int twi0_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi0);
}
#endif

#ifdef CONFIG_SAM34_TWI1
static int twi1_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi1);
}
#endif

/*******************************************************************************
 * Name: twi_timeout
 *
 * Description:
 *   Watchdog timer for timeout of TWI operation
 *
 * Assumptions:
 *   Called from the timer interrupt handler with interrupts disabled.
 *
 *******************************************************************************/

static void twi_timeout(int argc, uint32_t arg, ...)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)arg;

  i2clldbg("TWI%d Timeout!\n", priv->twi);
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
      twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_STOP);
    }

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWI_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWI_MMR_OFFSET, TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD | TWI_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWI_IADR_OFFSET, 0);

  /* Enable read interrupt and send the START condition */

  twi_putrel(priv, SAM_TWI_IER_OFFSET, TWI_INT_RXRDY | TWI_INT_ERRORS);
  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_START);
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

  twi_putrel(priv, SAM_TWI_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWI_MMR_OFFSET, TWI_MMR_IADRSZ_NONE | TWI_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWI_IADR_OFFSET, 0);

  /* Write first byte to send.*/

  twi_putrel(priv, SAM_TWI_THR_OFFSET, msg->buffer[priv->xfrd++]);

  /* Enable write interrupt */

  twi_putrel(priv, SAM_TWI_IER_OFFSET, TWI_INT_TXRDY | TWI_INT_ERRORS);
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
  if ((msg->flags & I2C_M_READ) == 0)
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

  i2cvdbg("TWI%d address: %02x nbits: %d\n", priv->twi, addr, nbits);
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

  i2cvdbg("TWI%d buflen: %d\n", priv->twi, wbuflen);
  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the device */

  twi_takesem(&priv->exclsem);

  /* Initiate the wrte */

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

  ret = twi_wait(priv);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  if (ret == -ETIMEDOUT)
    {
      /* Reinitialize the TWI hardware */

      i2clldbg("Reinitialize after write\n");
      twi_hw_initialize(priv, priv->pid, priv->deffreq);
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
  i2cvdbg("TWI%d rbuflen: %d\n", priv->twi, rbuflen);

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

  ret = twi_wait(priv);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  if (ret == -ETIMEDOUT)
    {
      /* Reinitialize the TWI hardware */

      i2clldbg("Reinitialize after read\n");
      twi_hw_initialize(priv, priv->pid, priv->deffreq);
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
  irqstate_t flags;
  int ret;

  struct i2c_msg_s msgv[2] =
  {
    {
      .addr   = priv->address,
      .flags  = priv->flags,
      .buffer = (uint8_t *)wbuffer,  /* Override const */
      .length = wbuflen
    },
    {
      .addr   = priv->address,
      .flags  = priv->flags | ((rbuflen > 0) ? I2C_M_READ : I2C_M_NORESTART),
      .buffer = rbuffer,
      .length = (rbuflen < 0) ? -rbuflen : rbuflen
    }
  };

  DEBUGASSERT(dev != NULL);
  i2cvdbg("TWI%d wbuflen: %d rbuflen: %d\n", priv->twi, wbuflen, rbuflen);

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

  ret = twi_wait(priv);
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
static int twi_registercallback(FAR struct i2c_dev_s *dev,
                                int (*callback)(void))
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
 *******************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
static int twi_transfer(FAR struct i2c_dev_s *dev,
                        FAR struct i2c_msg_s *msgs, int count)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(dev != NULL);
  i2cvdbg("TWI%d count: %d\n", priv->twi, count);

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

  ret = twi_wait(priv);
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

  /* Configure TWI output clocking, trying each value of CKDIV {0..7} */

  for (ckdiv = 0; ckdiv < 8; ckdiv++)
    {
      /* Calulate the CLDIV value using the current CKDIV guess */

      cldiv = ((priv->frequency / (frequency << 1)) - 4) / (1 << ckdiv);

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

  /* Return the actual frequency */

  actual = (priv->frequency / 2) / (((1 << ckdiv) * cldiv) + 2);
  i2cvdbg("TWI%d frequency: %d ckdiv: %d cldiv: %d actual: %d\n",
          priv->twi, frequency, ckdiv, cldiv, actual);

  /* Remember the selected frequency for error recovery */

  priv->deffreq = frequency;
  return actual;
}

/*******************************************************************************
 * Name: twi_hw_initialize
 *
 * Description:
 *   Initialize one TWI peripheral for I2C operation
 *
 *******************************************************************************/

static void twi_hw_initialize(struct twi_dev_s *priv, unsigned int pid,
                              uint32_t frequency)
{
  //uint32_t regval;
  //uint32_t mck;

  i2cvdbg("TWI%d Initializing\n", priv->twi);

  /* SVEN: TWI Slave Mode Enabled */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_SVEN);

  /* Reset the TWI */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_SWRST);
  (void)twi_getrel(priv, SAM_TWI_RHR_OFFSET);

  /* TWI Slave Mode Disabled, TWI Master Mode Disabled. */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_SVDIS);
  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_MSDIS);

  /* Set master mode */

  twi_putrel(priv, SAM_TWI_CR_OFFSET, TWI_CR_MSEN);

  /* Set base frequency */

  priv->frequency = BOARD_MCK_FREQUENCY;

#if 0
  /* Determine the maximum valid frequency setting */

  mck = BOARD_MCK_FREQUENCY;
  DEBUGASSERT((mck >> 3) <= TWI_MAX_FREQUENCY);

  if (mck <= TWI_MAX_FREQUENCY)
    {
      priv->frequency = mck;
      regval          = PMC_PCR_DIV1;
    }
  else if ((mck >> 1) <= TWI_MAX_FREQUENCY)
    {
      priv->frequency = (mck >> 1);
      regval          = PMC_PCR_DIV2;
    }
  else if ((mck >> 2) <= TWI_MAX_FREQUENCY)
    {
      priv->frequency = (mck >> 2);
      regval          = PMC_PCR_DIV4;
    }
  else /* if ((mck >> 3) <= TWI_MAX_FREQUENCY) */
    {
      priv->frequency = (mck >> 3);
      regval          = PMC_PCR_DIV8;
    }

  /* Set the TWI peripheral input clock to the maximum, valid frequency */

  regval |= PMC_PCR_PID(pid) | PMC_PCR_CMD | PMC_PCR_EN;
  twi_putabs(priv, SAM_PMC_PCR, regval);
#endif

  /* Set the initial TWI data transfer frequency */

  (void)twi_hw_setfrequency(priv, frequency);
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize a TWI device for I2C operation
 *
 *******************************************************************************/

struct i2c_dev_s *up_i2cinitialize(int bus)
{
  struct twi_dev_s *priv;
  xcpt_t handler;
  irqstate_t flags;
  uint32_t frequency;
  unsigned int pid;

  i2cvdbg("Initializing TWI%d\n", bus);

  flags = irqsave();

#ifdef CONFIG_SAM34_TWI0
  if (bus == 0)
    {
      /* Set up TWI0 register base address and IRQ number */

      priv       = &g_twi0;
      priv->base = SAM_TWI0_BASE;
      priv->irq  = SAM_IRQ_TWI0;
      priv->twi  = 0;

      /* Enable peripheral clocking */

      sam_twi0_enableclk();

      /* Configure PIO pins */

      sam_configgpio(GPIO_TWI0_CK);
      sam_configgpio(GPIO_TWI0_D);

      /* Select the interrupt handler, TWI frequency, and peripheral ID */

      handler    = twi0_interrupt;
      frequency  = CONFIG_SAM34_TWI0_FREQUENCY;
      pid        = SAM_PID_TWI0;
    }
  else
#endif
#ifdef CONFIG_SAM34_TWI1
  if (bus == 1)
    {
      /* Set up TWI1 register base address and IRQ number */

      priv       = &g_twi1;
      priv->base = SAM_TWI0_BASE;
      priv->irq  = SAM_IRQ_TWI1;
      priv->twi  = 1;

      /* Enable peripheral clocking */

      sam_twi1_enableclk();

      /* Configure PIO pins */

      sam_configgpio(GPIO_TWI1_CK);
      sam_configgpio(GPIO_TWI1_D);

      /* Select the interrupt handler, TWI frequency, and peripheral ID */

      handler    = twi1_interrupt;
      frequency  = CONFIG_SAMA5_TWI1_FREQUENCY;
      pid        = SAM_PID_TWI1;
    }
  else
#endif
    {
      irqrestore(flags);
      i2cdbg("ERROR: Unsupported bus: TWI%d\n", bus);
      return NULL;
    }

  /* Initialize the device structure */

  priv->dev.ops = &g_twiops;
  priv->address = 0;
  priv->flags   = 0;

  sem_init(&priv->exclsem, 0, 1);
  sem_init(&priv->waitsem, 0, 0);

  /* Allocate a watchdog timer */

  priv->timeout = wd_create();
  DEBUGASSERT(priv->timeout != 0);

  /* Configure and enable the TWI hardware */

  priv->pid = pid;
  twi_hw_initialize(priv, pid, frequency);

  /* Attach Interrupt Handler */

  irq_attach(priv->irq, handler);

  /* Enable Interrupts */

  up_enable_irq(priv->irq);
  irqrestore(flags);
  return &priv->dev;
}

/*******************************************************************************
 * Name: up_i2cuninitalize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 *******************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s * dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;

  i2cvdbg("TWI%d Un-initializing\n", priv->twi);

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Reset data structures */

  sem_destroy(&priv->exclsem);
  sem_destroy(&priv->waitsem);

  /* Free the watchdog timer */

  wd_delete(priv->timeout);
  priv->timeout = NULL;

  /* Detach Interrupt Handler */

  irq_detach(priv->irq);
  return OK;
}

#endif /* CONFIG_SAM34_TWI0 || CONFIG_SAM34_TWI1 */
