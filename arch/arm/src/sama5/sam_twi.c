/*******************************************************************************
 * arch/arm/src/sama5/sam_twi.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <wdog.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "sam_periphclks.h"
#include "sam_twi.h"

#if defined(CONFIG_SAMA5_TWI0) || defined(CONFIG_SAMA5_TWI1) || defined(CONFIG_SAMA5_TWI2)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/

#ifndef CONFIG_SAMA5_TWI0_FREQUENCY
#  define CONFIG_SAMA5_TWI0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMA5_TWI1_FREQUENCY
 #define CONFIG_SAMA5_TWI1_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMA5_TWI2_FREQUENCY
 #define CONFIG_SAMA5_TWI2_FREQUENCY 100000
#endif

/* Driver internal definitions *************************************************/

#define TWI_TIMEOUT ((20 * CLK_TCK) / 1000) /* 20 mS */

/* Debug ***********************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cdbg  dbg
#  define i2cvdbg vdbg
#else
#  define i2cdbg(x...)
#  define i2cvdbg(x...)
#endif

/*******************************************************************************
 * Private Types
 *******************************************************************************/

struct twi_dev_s
{
  struct i2c_dev_s    dev;        /* Generic I2C device */
  struct i2c_msg_s    msg;        /* A single message for legacy read/write */
  unsigned int        base;       /* Base address of registers */
  uint16_t            irq;        /* IRQ number for this device */

  sem_t               exclsem;    /* Only one thread can access at a time */
  sem_t               waitsem;    /* Wait for TWI transfer completion */
  WDOG_ID             timeout;    /* Watchdog to recover from bus hangs */
  int                 result;     /* The result of the transfer */

  /* Debug stuff */

#ifdef CONFIG_SAMA5_TWI_REGDEBUG
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

static void sam_takesem(sem_t *sem);
#define     sam_givesem(sem) (sem_post(sem))

#ifdef CONFIG_SAMA5_TWI_REGDEBUG
static bool sam_checkreg(struct twi_dev_s *priv, bool wr,
              uint32_t value, uintptr_t address);
#else
# define    sam_checkreg(priv,wr,value,address) (false)
#endif

static inline uint32_t sam_getreg(struct twi_dev_s *priv,
          unsigned int offset);
static inline void sam_putreg(struct twi_dev_s *priv, uint32_t value,
          unsigned int offset);

/* I2C transfer helper functions */

static int twi_start(struct twi_dev_s *priv);
static int twi_interrupt(struct twi_dev_s *priv);
#ifdef CONFIG_SAMA5_TWI0
static int twi0_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAMA5_TWI1
static int twi1_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAMA5_TWI2
static int twi2_interrupt(int irq, FAR void *context);
#endif
static void twi_timeout(int argc, uint32_t arg, ...);

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

/*******************************************************************************
 * Private Data
 *******************************************************************************/

#ifdef CONFIG_SAMA5_TWI0
static struct twi_dev_s g_twi0;
#endif

#ifdef CONFIG_SAMA5_TWI1
static struct twi_dev_s g_twi1;
#endif

#ifdef CONFIG_SAMA5_TWI2
static struct twi_dev_s g_twi2;
#endif

struct i2c_ops_s g_twiops =
{
  .setfrequency = twi_setfrequency,
  .setaddress   = twi_setaddress,
  .write        = twi_write,
  .read         = twi_read,
#ifdef CONFIG_I2C_WRITEREAD
  .writeread    = twi_writeread,
#endif
#ifdef CONFIG_I2C_TRANSFER
  .transfer     = twi_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
  int    (*setownaddress)(FAR struct i2c_dev_s *dev, int addr, int nbits);
  int    (*registercallback)(FAR struct i2c_dev_s *dev, int (*callback)(void) );
#endif
};

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: sam_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_takesem(sem_t *sem)
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
 * Name: sam_checkreg
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
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_TWI_REGDEBUG
static bool sam_checkreg(struct twi_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      value   == priv->vallast &&  /* Same value? */
      address == priv->addrlast)  /* Same address? */
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
 * Name: sam_getreg
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint32_t sam_getreg(struct twi_dev_s *priv, unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMA5_TWI_REGDEBUG
  if (sam_checkreg(priv, false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void sam_putreg(struct twi_dev_s *priv, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = priv->base + offset;

#ifdef CONFIG_SAMA5_TWI_REGDEBUG
  if (sam_checkreg(priv, true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * I2C transfer helper functions
 ****************************************************************************/

/*******************************************************************************
 * Name: twi_start
 *
 * Description:
 *   Perform a I2C transfer start
 *
 *******************************************************************************/

static int twi_start(struct twi_dev_s *priv)
{
  sam_takesem(&priv->exclsem);
#warning Missing logic

  /* Start a timeout to avoid hangs */

  wd_start(priv->timeout, TWI_TIMEOUT, twi_timeout, 1, (uint32_t)priv);

  /* Wait for either the TWI transfer or the timeout to complete */

  sam_takesem(&priv->waitsem);
  wd_cancel(priv->timeout);
  sam_givesem(&priv->exclsem);

  /* Return the result of the transfer */

  return priv->result;
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
  /* Get the unmasked bits in the interrupt status register */
#warning Missing logic

  /* Process each unmasked bit in the interrupt status */
#warning Missing logic

  return OK;
}

#ifdef CONFIG_SAMA5_TWI0
static int twi0_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi0);
}
#endif

#ifdef CONFIG_SAMA5_TWI1
static int twi1_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi1);
}
#endif

#ifdef CONFIG_SAMA5_TWI2
static int twi2_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi2);
}
#endif

/*******************************************************************************
 * Name: twi_timeout
 *
 * Description:
 *   Watchdog timer for timeout of TWI operation
 *
 *******************************************************************************/

static void twi_timeout(int argc, uint32_t arg, ...)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) arg;

  irqstate_t flags = irqsave();
#warning Missing logic
  priv->result = -ENOSYS;
  sam_givesem(&priv->waitsem);
  irqrestore(flags);
}

/*******************************************************************************
 * I2C device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: twi_setfrequency
 *
 * Description:
 *   Set the frequence for the next transfer
 *
 *******************************************************************************/

static uint32_t twi_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;

  /* Setup clocking as close a possible to the selectd freqeuncy */
#warning Missing Logic

  /* Return the actual frequency */

  return frequency;
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

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(nbits == 7 );

  priv->msg.addr  = addr << 1;
  priv->msg.flags = 0 ;

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

static int twi_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;
  int ret;

  DEBUGASSERT(dev != NULL);

  priv->msg.addr  &= ~0x01;
  priv->msg.buffer = (uint8_t*)buffer;
  priv->msg.length = buflen;
  priv->result     = -EBUSY;

  ret = twi_start(priv);

  return ret > 0 ? OK : -ETIMEDOUT;
}

/*******************************************************************************
 * Name: twi_read
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 *******************************************************************************/

static int twi_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;
  int ret;

  DEBUGASSERT(dev != NULL);

  priv->msg.addr  |= 0x01;
  priv->msg.buffer = buffer;
  priv->msg.length = buflen;
  priv->result     = -EBUSY;

  ret = twi_start(priv);

  return ret > 0 ? OK : -ETIMEDOUT;
}

/*******************************************************************************
 * Name: twi_writeread
 *
 * Description:
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_WRITEREAD
static int twi_writeread(FAR struct i2c_dev_s *inst, const uint8_t *wbuffer,
          int wbuflen, uint8_t *rbuffer, int rbuflen)
{
#error Not implemented
  return -ENOSYS;
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
#error Not implemented
  return -ENOSYS;
}
#endif

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
  uint32_t regval;

  flags = irqsave();

#ifdef CONFIG_SAMA5_TWI0
  if (bus == 0)
    {
      priv       = &g_twi0;
      priv->base = SAM_TWI0_VBASE;
      priv->irq  = SAM_IRQ_TWI0;

      /* Enable peripheral clocking */

      sam_twi0_enableclk();

      /* Configure PIO pins */
#warning Missing logic

      /* Configure and enable the TWI block */
#warning Missing logic

      /* Select the interrupt handler */

      handler = twi0_interrupt;
    }
  else
#endif
#ifdef CONFIG_SAMA5_TWI1
  if (bus == 1)
    {
      priv       = &g_twi1;
      priv->base = SAM_TWI1_VBASE;
      priv->irq  = SAM_IRQ_TWI1;

      /* Enable peripheral clocking */

      sam_twi1_enableclk();

      /* Configure PIO pins */
#warning Missing logic

      /* Configure and enable the TWI block */
#warning Missing logic

      /* Select the interrupt handler */

      handler = twi1_interrupt;
    }
  else
#endif
#ifdef CONFIG_SAMA5_TWI2
  if (bus == 2)
    {
      priv       = &g_twi2;
      priv->base = SAM_TWI2_VBASE;
      priv->irq  = SAM_IRQ_TWI2;

      /* Enable peripheral clocking */

      sam_twi2_enableclk();

      /* Configure PIO pins */
#warning Missing logic

      /* Configure and enable the TWI block */
#warning Missing logic

      /* Select the interrupt handler */

      handler = twi2_interrupt;
    }
  else
#endif
    {
      irqrestore(flags);
      i2cdbg("ERROR: Unsupported bus: %d\n", bus);
      return NULL;
    }

  i2cvdbg("Initializing TWI%d\n", port);

  /* Initialize the device structure */

  priv->dev.ops = &g_twiops;
  sem_init(&priv->exclsem, 0, 1);
  sem_init(&priv->waitsem, 0, 0);

  /* Configure and enable the TWI hardware */
#warning Missing logic

  /* Allocate a watchdog timer */

  priv->timeout = wd_create();
  DEBUGASSERT(priv->timeout != 0);

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

  /* Disable TWI */
#warning Missing logic

  /* Reset data structures */

  sem_destroy(&priv->exclsem);
  sem_destroy(&priv->waitsem);

  /* Free the watchdog timer */

  wd_delete(priv->timeout);
  priv->timeout = NULL;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach Interrupt Handler */

  irq_detach(priv->irq);
  return OK;
}

#endif
