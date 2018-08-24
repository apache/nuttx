/*******************************************************************************
 * arch/arm/src/samd5e5/sam_i2c_master.c
 *
 *   Copyright (C) 2013-2014, 2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Filament - www.filament.com
 *   Author: Matt Thompson <mthompson@hexwave.com>
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/sam_i2c_master.h"
#include "chip/sam_pinmap.h"
#include "sam_gclk.h"
#include "sam_port.h"
#include "sam_sercom.h"

#if defined(SAMD5E5_HAVE_I2C_MASTER)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

#ifndef CONFIG_SAM_I2C0_FREQUENCY
#  define CONFIG_SAM_I2C0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C1_FREQUENCY
#  define CONFIG_SAM_I2C1_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C2_FREQUENCY
#  define CONFIG_SAM_I2C2_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C3_FREQUENCY
#  define CONFIG_SAM_I2C3_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C4_FREQUENCY
#  define CONFIG_SAM_I2C4_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C5_FREQUENCY
#  define CONFIG_SAM_I2C5_FREQUENCY 100000
#endif

/* Driver internal definitions *************************************************/

/* If verbose I2C debug output is enable, then allow more time before we declare
 * a timeout.  The debug output from i2c_interrupt will really slow things down!
 *
 * With a very slow clock (say 100,000 Hz), less than 100 usec would be required
 * to transfer on byte.  So these define a "long" timeout.
 */

#ifdef CONFIG_DEBUG_I2C_INFO
#  define I2C_TIMEOUT_MSPB (65000)    /* 50 msec/byte */
#else
#  define I2C_TIMEOUT_MSPB (5000)     /* 5 msec/byte */
#endif

/* Clocking to the I2C module(s) is provided by the main clock, divided down
 * as necessary.
 */

#define I2C_MAX_FREQUENCY 66000000    /* Maximum I2C frequency */

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* Invariant attributes of a I2C bus */

struct i2c_attr_s
{
  uint8_t i2c;                /* I2C device number (for debug output) */
  uint8_t sercom;             /* Identifies the SERCOM peripheral */
  uint8_t irq;                /* SERCOM IRQ number */
  uint8_t coregen;            /* Source GCLK generator */
  uint8_t slowgen;            /* Slow GCLK generator */
  port_pinset_t pad0;         /* Pin configuration for PAD0 */
  port_pinset_t pad1;         /* Pin configuration for PAD1 */
  uint32_t muxconfig;         /* Pad multiplexing configuration */
  uint32_t srcfreq;           /* Source clock frequency */
  uintptr_t base;             /* Base address of I2C registers */
  bool runinstdby;            /* Run in Stand-by ? */
  uint32_t sdaholdtime;       /* Hold time after start bit */
  uint32_t speed;             /* I2C Speed: Standard; Fast; High */
  bool scllowtout;            /* SCL low timeout */
  uint32_t inactout;          /* Inactive Bus Timeout */
  bool sclstretch;            /* SCL stretch only after ACK */
  bool sclslvextout;          /* SCL Slave extend timeout */
  bool sclmstextout;          /* SCL Master extend timeout */
};

/* State of a I2C bus */

struct sam_i2c_dev_s
{
  struct i2c_master_s dev;    /* I2C master device */
  const struct i2c_attr_s *attr;  /* Invariant attributes of I2C device */
  struct i2c_msg_s *msg;      /* Current message being processed */
  uint32_t frequency;         /* I2C transfer clock frequency */
  uint16_t flags;             /* Transfer flags */

  sem_t exclsem;              /* Only one thread can access at a time */
  sem_t waitsem;              /* Wait for I2C transfer completion */
  volatile int result;        /* The result of the transfer */
  volatile int xfrd;          /* Number of bytes transfers */

  /* Debug stuff */

#ifdef CONFIG_SAM_I2C_REGDEBUG
  bool wrlast;                /* Last was a write */
  uint32_t addrlast;          /* Last address */
  uint32_t vallast;           /* Last value */
  int ntimes;                 /* Number of times */
#endif
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Low-level helper functions */

static uint8_t i2c_getreg8(struct sam_i2c_dev_s *priv, unsigned int offset);
static void i2c_putreg8(struct sam_i2c_dev_s *priv, uint8_t regval,
                        unsigned int offset);
static uint16_t i2c_getreg16(struct sam_i2c_dev_s *priv, unsigned int offset);
static void i2c_putreg16(struct sam_i2c_dev_s *priv, uint16_t regval,
                         unsigned int offset);
static uint32_t i2c_getreg32(struct sam_i2c_dev_s *priv, unsigned int offset);
static void i2c_putreg32(struct sam_i2c_dev_s *priv, uint32_t regval,
                         unsigned int offset);

static void i2c_takesem(sem_t * sem);
#define i2c_givesem(sem) (nxsem_post(sem))

#ifdef CONFIG_SAM_I2C_REGDEBUG
static bool i2c_checkreg(struct sam_i2c_dev_s *priv, bool wr,
                         uint32_t value, uintptr_t address);
static uint32_t i2c_getabs(struct sam_i2c_dev_s *priv, uintptr_t address);
static void i2c_putabs(struct sam_i2c_dev_s *priv, uintptr_t address,
                       uint32_t value);
#else
#  define i2c_checkreg(priv,wr,value,address) (false)
#  define i2c_putabs(p,a,v) putreg32(v,a)
#  define i2c_getabs(p,a) getreg32(a)
#endif

static inline uint32_t i2c_getrel(struct sam_i2c_dev_s *priv,
                                  unsigned int offset);
static inline void i2c_putrel(struct sam_i2c_dev_s *priv, unsigned int offset,
                              uint32_t value);

/* I2C transfer helper functions */

static int i2c_wait_for_bus(struct sam_i2c_dev_s *priv, unsigned int size);

static void i2c_wakeup(struct sam_i2c_dev_s *priv, int result);
static int i2c_interrupt(int irq, FAR void *context, void *arg);

static void i2c_startread(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg);
static void i2c_startwrite(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg);
static void i2c_startmessage(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg);

static int sam_i2c_transfer(FAR struct i2c_master_s *dev,
                            FAR struct i2c_msg_s *msgs, int count);

/* Initialization */

static uint32_t sam_i2c_setfrequency(struct sam_i2c_dev_s *priv,
                                     uint32_t frequency);
static void i2c_hw_initialize(struct sam_i2c_dev_s *priv, uint32_t frequency);
static void i2c_wait_synchronization(struct sam_i2c_dev_s *priv);
static void i2c_pad_configure(struct sam_i2c_dev_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

#ifdef SAMD5E5_HAVE_I2C0_MASTER
static const struct i2c_attr_s g_i2c0attr =
{
  .i2c       = 0,
  .sercom    = 0,
  .irq       = SAM_IRQ_SERCOM0,
  .coregen   = BOARD_SERCOM0_GCLKGEN,
  .slowgen   = BOARD_SERCOM0_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM0_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM0_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM0_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM0_FREQUENCY,
  .base      = SAM_SERCOM0_BASE,
};

static struct sam_i2c_dev_s g_i2c0;
#endif

#ifdef SAMD5E5_HAVE_I2C1_MASTER
static const struct i2c_attr_s g_i2c1attr =
{
  .i2c       = 1,
  .sercom    = 1,
  .irq       = SAM_IRQ_SERCOM1,
  .coregen   = BOARD_SERCOM1_GCLKGEN,
  .slowgen   = BOARD_SERCOM1_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM1_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM1_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM1_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM1_FREQUENCY,
  .base      = SAM_SERCOM1_BASE,
};

static struct sam_i2c_dev_s g_i2c1;
#endif

#ifdef SAMD5E5_HAVE_I2C2_MASTER
static const struct i2c_attr_s g_i2c2attr =
{
  .i2c       = 2,
  .sercom    = 2,
  .irq       = SAM_IRQ_SERCOM2,
  .coregen   = BOARD_SERCOM2_GCLKGEN,
  .slowgen   = BOARD_SERCOM2_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM2_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM2_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM2_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM2_FREQUENCY,
  .base      = SAM_SERCOM2_BASE,
};

static struct sam_i2c_dev_s g_i2c2;
#endif

#ifdef SAMD5E5_HAVE_I2C3_MASTER
static const struct i2c_attr_s g_i2c3attr =
{
  .i2c       = 3,
  .sercom    = 3,
  .irq       = SAM_IRQ_SERCOM3,
  .coregen   = BOARD_SERCOM3_GCLKGEN,
  .slowgen   = BOARD_SERCOM3_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM3_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM3_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM3_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM3_FREQUENCY,
  .base      = SAM_SERCOM3_BASE,
};

static struct sam_i2c_dev_s g_i2c3;
#endif

#ifdef SAMD5E5_HAVE_I2C4_MASTER
static const struct i2c_attr_s g_i2c4attr =
{
  .i2c       = 4,
  .sercom    = 4,
  .irq       = SAM_IRQ_SERCOM4,
  .coregen   = BOARD_SERCOM4_GCLKGEN,
  .slowgen   = BOARD_SERCOM4_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM4_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM4_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM4_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM4_FREQUENCY,
  .base      = SAM_SERCOM4_BASE,
};

static struct sam_i2c_dev_s g_i2c4;
#endif

#ifdef SAMD5E5_HAVE_I2C5_MASTER
static const struct i2c_attr_s g_i2c5attr =
{
  .i2c       = 5,
  .sercom    = 5,
  .irq       = SAM_IRQ_SERCOM5,
  .coregen   = BOARD_SERCOM5_GCLKGEN,
  .slowgen   = BOARD_SERCOM5_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM5_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM5_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM5_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM5_FREQUENCY,
  .base      = SAM_SERCOM5_BASE,
};

static struct sam_i2c_dev_s g_i2c5;
#endif

#ifdef SAMD5E5_HAVE_I2C6_MASTER
static const struct i2c_attr_s g_i2c6attr =
{
  .i2c       = 6,
  .sercom    = 6,
  .irq       = SAM_IRQ_SERCOM6,
  .coregen   = BOARD_SERCOM6_GCLKGEN,
  .slowgen   = BOARD_SERCOM6_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM6_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM6_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM6_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM6_FREQUENCY,
  .base      = SAM_SERCOM6_BASE,
};

static struct sam_i2c_dev_s g_i2c6;
#endif

#ifdef SAMD5E5_HAVE_I2C7_MASTER
static const struct i2c_attr_s g_i2c7attr =
{
  .i2c       = 7,
  .sercom    = 7,
  .irq       = SAM_IRQ_SERCOM7,
  .coregen   = BOARD_SERCOM7_GCLKGEN,
  .slowgen   = BOARD_SERCOM7_SLOW_GCLKGEN,
  .pad0      = BOARD_SERCOM7_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM7_PINMAP_PAD1,
  .muxconfig = BOARD_SERCOM7_MUXCONFIG,
  .srcfreq   = BOARD_SERCOM7_FREQUENCY,
  .base      = SAM_SERCOM7_BASE,
};

static struct sam_i2c_dev_s g_i2c7;
#endif

struct i2c_ops_s g_i2cops =
{
  .transfer = sam_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = sam_i2c_reset,
#endif
};

/*******************************************************************************
 * Low-level Helpers
 *******************************************************************************/

/*******************************************************************************
 * Name: i2c_getreg8
 *
 * Description:
 *   Get a 8-bit register value by offset
 *
 *******************************************************************************/

static uint8_t i2c_getreg8(struct sam_i2c_dev_s *priv, unsigned int offset)
{
  return getreg8(priv->attr->base + offset);
}

/*******************************************************************************
 * Name: i2c_putreg8
 *
 * Description:
 *  Put a 8-bit register value by offset
 *
 *******************************************************************************/

static void i2c_putreg8(struct sam_i2c_dev_s *priv, uint8_t regval,
                        unsigned int offset)
{
  putreg8(regval, priv->attr->base + offset);
}

/*******************************************************************************
 * Name: i2c_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 *******************************************************************************/

static uint16_t i2c_getreg16(struct sam_i2c_dev_s *priv, unsigned int offset)
{
  return getreg16(priv->attr->base + offset);
}

/*******************************************************************************
 * Name: i2c_putreg16
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 *******************************************************************************/

static void i2c_putreg16(struct sam_i2c_dev_s *priv, uint16_t regval,
                         unsigned int offset)
{
  putreg16(regval, priv->attr->base + offset);
}

/*******************************************************************************
 * Name: i2c_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 *******************************************************************************/

static uint32_t i2c_getreg32(struct sam_i2c_dev_s *priv, unsigned int offset)
{
  return getreg32(priv->attr->base + offset);
}

/*******************************************************************************
 * Name: i2c_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 *******************************************************************************/

static void i2c_putreg32(struct sam_i2c_dev_s *priv, uint32_t regval,
                         unsigned int offset)
{
  putreg32(regval, priv->attr->base + offset);
}

/*******************************************************************************
 * Name: i2c_takesem
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
 *******************************************************************************/

static void i2c_takesem(sem_t *sem)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/*******************************************************************************
 * Name: i2c_checkreg
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
 *******************************************************************************/

#ifdef CONFIG_SAM_I2C_REGDEBUG
static bool i2c_checkreg(struct sam_i2c_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr == priv->wrlast &&       /* Same kind of access? */
      value == priv->vallast &&   /* Same value? */
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

/*******************************************************************************
 * Name: i2c_getabs
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 *******************************************************************************/

#ifdef CONFIG_SAM_I2C_REGDEBUG
static uint32_t i2c_getabs(struct sam_i2c_dev_s *priv, uintptr_t address)
{
  uint32_t value = getreg32(address);

  if (i2c_checkreg(priv, false, value, address))
    {
      i2cinfo("%08x->%08x\n", address, value);
    }

  return value;
}
#endif

/*******************************************************************************
 * Name: i2c_putabs
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 *******************************************************************************/

#ifdef CONFIG_SAM_I2C_REGDEBUG
static void i2c_putabs(struct sam_i2c_dev_s *priv, uintptr_t address,
                       uint32_t value)
{
  if (i2c_checkreg(priv, true, value, address))
    {
      i2cinfo("%08x<-%08x\n", address, value);
    }

  putreg32(value, address);
}
#endif

/*******************************************************************************
 * Name: i2c_getrel
 *
 * Description:
 *  Read a I2C register using an offset relative to the I2C base address
 *
 *******************************************************************************/

static inline uint32_t i2c_getrel(struct sam_i2c_dev_s *priv,
                                  unsigned int offset)
{
  return i2c_getabs(priv, priv->attr->base + offset);
}

/*******************************************************************************
 * Name: i2c_putrel
 *
 * Description:
 *  Write a value to a I2C register using an offset relative to the I2C base
 *  address.
 *
 *******************************************************************************/

static inline void i2c_putrel(struct sam_i2c_dev_s *priv, unsigned int offset,
                              uint32_t value)
{
  i2c_putabs(priv, priv->attr->base + offset, value);
}

/*******************************************************************************
 * I2C transfer helper functions
 *******************************************************************************/

/*******************************************************************************
 * Name: i2c_wait_for_bus
 *
 * Description:
 *   Wait for the ISR to post the semaphore, indicating the transaction is
 *   complete
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 *******************************************************************************/

static int i2c_wait_for_bus(struct sam_i2c_dev_s *priv, unsigned int size)
{
  struct timespec ts;
  int ret;

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_nsec += 200e3;

  ret = nxsem_timedwait(&priv->waitsem, (const struct timespec *)&ts);
  if (ret < 0)
    {
      i2cinfo("timedwait error %d\n", ret);
      return ret;
    }

  return priv->result;
}

/*******************************************************************************
 * Name: i2c_wakeup
 *
 * Description:
 *   A terminal event has occurred.  Wake-up the waiting thread
 *
 *******************************************************************************/

static void i2c_wakeup(struct sam_i2c_dev_s *priv, int result)
{
  /* Disable any further I2C interrupts */

  i2c_putreg8(priv, I2C_INT_MB | I2C_INT_SB, SAM_I2C_INTENCLR_OFFSET);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  nxsem_post(&priv->waitsem);
}

/*******************************************************************************
 * Name: i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 *******************************************************************************/

static int i2c_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct sam_i2c_dev_s *priv = (struct sam_i2c_dev_s *)arg;
  struct i2c_msg_s *msg;
  uint32_t regval;

  msg = priv->msg;

  /* Check arbitration */

  if (i2c_getreg16(priv, SAM_I2C_STATUS_OFFSET) & I2C_STATUS_ARBLOST)
    {
      i2cinfo("I2C Bus Collision!\n");

      /* Clear error INTFLAG */

      i2c_putreg16(priv, I2C_INT_ERR, SAM_I2C_INTFLAG_OFFSET);

      /* Cancel timeout */

      i2c_wakeup(priv, -EBUSY);

      return -EBUSY;
    }

  if ((i2c_getreg8(priv, SAM_I2C_INTFLAG_OFFSET) & I2C_INT_SB) == I2C_INT_SB)
    {
      /* Send NACK and STOP after transmission of last byte */

      if (priv->xfrd == (msg->length - 1))
        {
          /* NACK */

          regval = i2c_getreg32(priv, SAM_I2C_CTRLB_OFFSET);
          regval |= I2C_CTRLB_ACKACT;
          i2c_putreg32(priv, regval, SAM_I2C_CTRLB_OFFSET);
          i2c_wait_synchronization(priv);

          /* STOP */

          regval = i2c_getreg32(priv, SAM_I2C_CTRLB_OFFSET);
          regval |= I2C_CTRLB_CMD_ACKSTOP;
          i2c_putreg32(priv, regval, SAM_I2C_CTRLB_OFFSET);
          i2c_wait_synchronization(priv);

          msg->buffer[priv->xfrd++] = i2c_getreg8(priv, SAM_I2C_DATA_OFFSET);
          i2c_wait_synchronization(priv);
        }
      else
        {
          regval = i2c_getreg32(priv, SAM_I2C_CTRLB_OFFSET);
          regval &= ~I2C_CTRLB_ACKACT;
          i2c_putreg32(priv, regval, SAM_I2C_CTRLB_OFFSET);
          i2c_wait_synchronization(priv);

          msg->buffer[priv->xfrd++] = i2c_getreg8(priv, SAM_I2C_DATA_OFFSET);
          i2c_wait_synchronization(priv);

          regval = i2c_getreg32(priv, SAM_I2C_CTRLB_OFFSET);
          regval |= I2C_CTRLB_CMD_ACKREAD;
          i2c_putreg32(priv, regval, SAM_I2C_CTRLB_OFFSET);
          i2c_wait_synchronization(priv);
        }

      /* Disable Interrupt after last byte */

      if (priv->xfrd == msg->length)
        {
          /* Cancel timeout */

          i2c_wakeup(priv, OK);
          // i2cinfo("Got data = 0x%02X\n", msg->buffer[0]);
        }

      i2c_putreg8(priv, I2C_INT_SB, SAM_I2C_INTFLAG_OFFSET);
    }

  if ((i2c_getreg8(priv, SAM_I2C_INTFLAG_OFFSET) & I2C_INT_MB) == I2C_INT_MB)
    {
      /* If no device responded to the address packet, STATUS.RXNACK will be
       * set
       */

      if ((i2c_getreg16(priv, SAM_I2C_STATUS_OFFSET) & I2C_STATUS_RXNACK) ==
          I2C_STATUS_RXNACK)
        {
          i2c_wakeup(priv, -ENODEV);
          return OK;
        }

      if (priv->xfrd == msg->length)
        {
          /* Send STOP condition */

          regval = i2c_getreg32(priv, SAM_I2C_CTRLB_OFFSET);
          regval |= I2C_CTRLB_CMD_ACKSTOP;
          i2c_putreg32(priv, regval, SAM_I2C_CTRLB_OFFSET);
          i2c_wait_synchronization(priv);

          i2c_wakeup(priv, OK);
        }
      else
        {
          i2c_putreg8(priv, msg->buffer[priv->xfrd++], SAM_I2C_DATA_OFFSET);
        }

      i2c_putreg8(priv, I2C_INT_MB, SAM_I2C_INTFLAG_OFFSET);
    }

  return OK;
}

/*******************************************************************************
 * Name: i2c_startread
 *
 * Description:
 *   Start the next read message
 *
 *******************************************************************************/

static void i2c_startread(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg)
{
  uint32_t regval;

  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd = 0;

  /* Set action to ACK */

  regval = i2c_getreg32(priv, SAM_I2C_CTRLB_OFFSET);
  regval &= ~I2C_CTRLB_ACKACT;
  i2c_putreg32(priv, regval, SAM_I2C_CTRLB_OFFSET);

  /* Enable Interrupts */

  regval = I2C_INT_MB | I2C_INT_SB;
  i2c_putreg8(priv, regval, SAM_I2C_INTENSET_OFFSET);

  /* Create the ADDR register */

  regval = (msg->addr) << 1;

  /* 7 or 10 bits ? */

  if (msg->flags & I2C_M_TEN)
    {
      regval |= I2C_ADDR_TENBITEN;
    }

  /* Is it a read or write? */

  regval |= (msg->flags & I2C_M_READ);

  /* Set the ADDR register */

  i2c_putreg32(priv, regval, SAM_I2C_ADDR_OFFSET);
  i2c_wait_synchronization(priv);
}

/*******************************************************************************
 * Name: i2c_startwrite
 *
 * Description:
 *   Start the next write message
 *
 *******************************************************************************/

static void i2c_startwrite(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg)
{
  uint32_t regval;

  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd = 0;

  /* Wait bus sync */

  i2c_wait_synchronization(priv);

  /* Set action to ACK */

  regval = i2c_getreg32(priv, SAM_I2C_CTRLB_OFFSET);
  regval &= ~I2C_CTRLB_ACKACT;
  i2c_putreg32(priv, regval, SAM_I2C_CTRLB_OFFSET);

  /* Enable Interrupts */

  regval = I2C_INT_MB | I2C_INT_SB;
  i2c_putreg8(priv, regval, SAM_I2C_INTENSET_OFFSET);

  /* Create the ADDR register */

  regval = (msg->addr) << 1;

  /* 7 or 10 bits ? */

  if (priv->flags & I2C_M_TEN)
    {
      regval |= I2C_ADDR_TENBITEN;
    }

  /* Is it a read or write? */

  regval |= (priv->flags & I2C_M_READ);

  /* Set the ADDR register */

  i2c_putreg32(priv, regval, SAM_I2C_ADDR_OFFSET);
  i2c_wait_synchronization(priv);
}

/*******************************************************************************
 * Name: i2c_startmessage
 *
 * Description:
 *   Start the next write message
 *
 *******************************************************************************/

static void i2c_startmessage(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg)
{
  if ((msg->flags & I2C_M_READ) != 0)
    {
      i2c_startread(priv, msg);
    }
  else
    {
      i2c_startwrite(priv, msg);
    }
}

/*******************************************************************************
 * I2C device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_i2c_transfer
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value on failure.
 *
 *******************************************************************************/

static int sam_i2c_transfer(FAR struct i2c_master_s *dev,
                            FAR struct i2c_msg_s *msgs, int count)
{
  struct sam_i2c_dev_s *priv = (struct sam_i2c_dev_s *)dev;
  irqstate_t flags;
  unsigned int size;
  int i;
  int ret = -EBUSY;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Set the frequency from the first message in the msgs vector */

  if (count)
    {
      if (priv->frequency != msgs->frequency)
        {
          sam_i2c_setfrequency(priv, msgs->frequency);
          priv->frequency = msgs->frequency;
        }
    }

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

  i2c_takesem(&priv->exclsem);

  /* Initiate the message transfer */

  /* Initiate the transfer.  The rest will be handled from interrupt logic.
   * Interrupts must be disabled to prevent re-entrance from the interrupt
   * level.
   */

  while (count--)
    {
      priv->msg = msgs;
      flags = enter_critical_section();
      i2c_startmessage(priv, msgs);

      /* And wait for the transfers to complete.  Interrupts will be re-enabled
       * while we are waiting.
       */

      ret = i2c_wait_for_bus(priv, msgs->length);
      if (ret < 0)
        {
#if 0
          i2cerr("ERROR: Transfer failed: %d\n", ret);
          i2cinfo("STATUS: 0x%08x\n",
                  i2c_getreg16(priv, SAM_I2C_STATUS_OFFSET));
          i2cinfo("INTFLAG: 0x%02x\n",
                  i2c_getreg8(priv, SAM_I2C_INTFLAG_OFFSET));
#endif
          leave_critical_section(flags);
          i2c_givesem(&priv->exclsem);
          return ret;
        }

      leave_critical_section(flags);

      /* Move to the next message */

      msgs++;
    }

  i2c_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Initialization
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 *******************************************************************************/

static uint32_t sam_i2c_setfrequency(struct sam_i2c_dev_s *priv,
                                     uint32_t frequency)
{
  uint32_t maxfreq;
  uint32_t baud = 0;
  uint32_t baud_hs = 0;
  uint32_t ctrla;

  // i2cinfo("sercom=%d frequency=%d\n", priv->attr->sercom, frequency);

  /* Check if the configured BAUD is within the valid range */

  maxfreq = (priv->attr->srcfreq >> 1);
  if (frequency > maxfreq)
    {
      /* Set the frequency to the maximum */

      i2cerr("ERROR: Cannot realize frequency: %ld\n", (long)frequency);
      frequency = maxfreq;
    }

  /* Check if the requested frequency is the same as the frequency selection */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->frequency;
    }

  /* Calculate and setup baud rate */

  baud = ((priv->attr->srcfreq * 10) / (24 * frequency)) - 4;

  /* Verify that the resulting if BAUD divisor is within range */

  if (baud > 255)
    {
      i2cerr("ERROR: BAUD is out of range: %d\n", baud);
      baud = 255;
    }
  else
    {
      /* Find baudrate for high speed */

      baud_hs = ((priv->attr->srcfreq * 10) / (478 * frequency)) - 1;

      if (baud_hs > 255)
        {
          i2cerr("ERROR: BAUD is out of range: %d\n", baud);
          baud_hs = 255;
        }
    }

  /* Momentarily disable I2C while we apply the new BAUD setting (if it was
   * previously enabled).
   */

  ctrla = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  if ((ctrla & I2C_CTRLA_ENABLE) != 0)
    {
      /* Disable I2C.. waiting for synchronization */

      i2c_putreg32(priv, ctrla & ~I2C_CTRLA_ENABLE, SAM_I2C_CTRLA_OFFSET);
      i2c_wait_synchronization(priv);

      /* Set the new BAUD value */

      i2c_putreg32(priv, (uint32_t) ((baud_hs << 16) | baud),
                   SAM_I2C_BAUD_OFFSET);

      /* Re-enable I2C.. waiting for synchronization */

      i2c_putreg32(priv, ctrla, SAM_I2C_CTRLA_OFFSET);
      i2c_wait_synchronization(priv);

      i2c_putreg16(priv, I2C_STATUS_BUSSTATE_IDLE, SAM_I2C_STATUS_OFFSET);
      i2c_wait_synchronization(priv);
    }
  else
    {
      /* Set the new BAUD when the I2C is already disabled */

      i2c_putreg32(priv, (uint32_t) ((baud_hs << 16) | baud),
                   SAM_I2C_BAUD_OFFSET);
    }

  priv->frequency = frequency;
  return priv->frequency;
}

/*******************************************************************************
 * Name: i2c_hw_initialize
 *
 * Description:
 *   Initialize/Re-initialize the I2C peripheral.  This logic performs only
 *   repeatable initialization after either (1) the one-time initialization, or
 *   (2) after each bus reset.
 *
 *******************************************************************************/

static void i2c_hw_initialize(struct sam_i2c_dev_s *priv, uint32_t frequency)
{
  irqstate_t flags;
  uint32_t regval;
  uint32_t ctrla = 0;

  i2cinfo("I2C%d Initializing\n", priv->attr->i2c);

  /* Enable clocking to the SERCOM module in PM */

  flags = enter_critical_section();
  sercom_enable(priv->attr->sercom);

  /* Configure the GCLKs for the SERCOM module */

  sercom_coreclk_configure(priv->attr->sercom, priv->attr->coregen, false);
  sercom_slowclk_configure(priv->attr->sercom, priv->attr->slowgen);

  /* Check if module is enabled */

  regval = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  if (regval & I2C_CTRLA_ENABLE)
    {
      i2cerr("ERROR: Cannot initialize I2C because it is already initialized!\n");
      return;
    }

  /* Check if reset is in progress */

  regval = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  if (regval & I2C_CTRLA_SWRST)
    {
      i2cerr("ERROR: Module is in RESET process!\n");
      return;
    }

  /* Configure pads */

  i2c_pad_configure(priv);

  ctrla = I2C_CTRLA_MODE_MASTER | I2C_CTRLA_RUNSTDBY | I2C_CTRLA_SPEED_FAST |
          I2C_CTRLA_SDAHOLD_450NS | priv->attr->muxconfig;
  i2c_putreg32(priv, ctrla, SAM_I2C_CTRLA_OFFSET);
  i2c_wait_synchronization(priv);

  /* Enable Smart Mode */

  i2c_putreg32(priv, I2C_CTRLB_SMEN, SAM_I2C_CTRLB_OFFSET);

  /* Set an initial baud value. */

  sam_i2c_setfrequency(priv, 100000);

  /* Enable I2C */

  regval = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  regval |= I2C_CTRLA_ENABLE;
  i2c_putreg32(priv, regval, SAM_I2C_CTRLA_OFFSET);
  i2c_wait_synchronization(priv);

  /* Force IDLE bus state */

  i2c_putreg16(priv, I2C_STATUS_BUSSTATE_IDLE, SAM_I2C_STATUS_OFFSET);
  i2c_wait_synchronization(priv);

  i2c_putreg8(priv, I2C_INT_ALL, SAM_I2C_INTENCLR_OFFSET);
  i2c_putreg8(priv, I2C_INT_ALL, SAM_I2C_INTFLAG_OFFSET);

  /* Enable SERCOM interrupts at the NVIC */

  up_enable_irq(priv->attr->irq);
  leave_critical_section(flags);
}

/*******************************************************************************
 * Name: i2c_wait_synchronization
 *
 * Description:
 *   Wait until the SERCOM I2C reports that it is synchronized.
 *
 *******************************************************************************/

static void i2c_wait_synchronization(struct sam_i2c_dev_s *priv)
{
  while ((i2c_getreg16(priv, SAM_I2C_SYNCBUSY_OFFSET) & 0x7) != 0);
}

/*******************************************************************************
 * Name: i2c_pad_configure
 *
 * Description:
 *   Configure the SERCOM I2C pads.
 *
 *******************************************************************************/

static void i2c_pad_configure(struct sam_i2c_dev_s *priv)
{
  /* Configure SERCOM pads */

  if (priv->attr->pad0 != 0)
    {
      sam_portconfig(priv->attr->pad0);
    }

  if (priv->attr->pad1 != 0)
    {
      sam_portconfig(priv->attr->pad1);
    }
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_i2c_master_initialize
 *
 * Description:
 *   Initialize a I2C device for I2C operation
 *
 *******************************************************************************/

struct i2c_master_s *sam_i2c_master_initialize(int bus)
{
  struct sam_i2c_dev_s *priv;
  uint32_t frequency;
  irqstate_t flags;
  int ret = 0;

#ifdef SAMD5E5_HAVE_I2C0_MASTER
  if (bus == 0)
    {
      /* Select up I2C0 and setup invariant attributes */

      priv = &g_i2c0;
      priv->attr = &g_i2c0attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C0_FREQUENCY;
    }
  else
#endif
#ifdef SAMD5E5_HAVE_I2C1_MASTER
  if (bus == 1)
    {
      /* Select up I2C1 and setup invariant attributes */

      priv = &g_i2c1;
      priv->attr = &g_i2c1attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C1_FREQUENCY;
    }
  else
#endif
#ifdef SAMD5E5_HAVE_I2C2_MASTER
  if (bus == 2)
    {
      /* Select up I2C2 and setup invariant attributes */

      priv = &g_i2c2;
      priv->attr = &g_i2c2attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C2_FREQUENCY;
    }
  else
#endif
#ifdef SAMD5E5_HAVE_I2C3_MASTER
  if (bus == 3)
    {
      /* Select up I2C3 and setup invariant attributes */

      priv = &g_i2c3;
      priv->attr = &g_i2c3attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C3_FREQUENCY;
    }
  else
#endif
#ifdef SAMD5E5_HAVE_I2C4_MASTER
  if (bus == 4)
    {
      /* Select up I2C4 and setup invariant attributes */

      priv = &g_i2c4;
      priv->attr = &g_i2c4attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C4_FREQUENCY;
    }
  else
#endif
#ifdef SAMD5E5_HAVE_I2C5_MASTER
  if (bus == 5)
    {
      /* Select up I2C5 and setup invariant attributes */

      priv = &g_i2c5;
      priv->attr = &g_i2c5attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C5_FREQUENCY;
    }
  else
#endif
#ifdef SAMD5E5_HAVE_I2C6_MASTER
  if (bus == 6)
    {
      /* Select up I2C6 and setup invariant attributes */

      priv = &g_i2c6;
      priv->attr = &g_i2c6attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C6_FREQUENCY;
    }
  else
#endif
#ifdef SAMD5E5_HAVE_I2C7_MASTER
  if (bus == 7)
    {
      /* Select up I2C7 and setup invariant attributes */

      priv = &g_i2c7;
      priv->attr = &g_i2c7attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C7_FREQUENCY;
    }
  else
#endif
    {
      i2cerr("ERROR: Unsupported bus: I2C%d\n", bus);
      return NULL;
    }

  /* Perform one-time I2C initialization */

  flags = enter_critical_section();

  /* Attach Interrupt Handler */

  ret = irq_attach(priv->attr->irq, i2c_interrupt, priv);
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to attach irq %d\n", priv->attr->irq);
      leave_critical_section(flags);
      return NULL;
    }

  /* Initialize the I2C driver structure */

  priv->dev.ops = &g_i2cops;
  priv->flags = 0;

  (void)nxsem_init(&priv->exclsem, 0, 1);
  (void)nxsem_init(&priv->waitsem, 0, 0);

  /* Perform repeatable I2C hardware initialization */

  i2c_hw_initialize(priv, frequency);
  leave_critical_section(flags);
  return &priv->dev;
}

/*******************************************************************************
 * Name: sam_i2c_uninitalize
 *
 * Description:
 *   Uninitialize an I2C device
 *
 *******************************************************************************/

int sam_i2c_uninitialize(FAR struct i2c_master_s *dev)
{
  struct sam_i2c_dev_s *priv = (struct sam_i2c_dev_s *)dev;

  i2cinfo("I2C%d Un-initializing\n", priv->attr->i2c);

  /* Disable I2C interrupts */

  up_disable_irq(priv->attr->irq);

  /* Reset data structures */

  nxsem_destroy(&priv->exclsem);
  nxsem_destroy(&priv->waitsem);

  /* Detach Interrupt Handler */

  (void)irq_detach(priv->attr->irq);
  return OK;
}

/*******************************************************************************
 * Name: sam_i2c_reset
 *
 * Description:
 *   Reset an I2C bus
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_RESET
int sam_i2c_reset(FAR struct i2c_master_s *dev)
{
  struct sam_i2c_dev_s *priv = (struct sam_i2c_dev_s *)dev;
  int ret;

  DEBUGASSERT(priv);

  /* Get exclusive access to the I2C device */

  i2c_takesem(&priv->exclsem);

  /* Disable I2C interrupts */

  up_disable_irq(priv->attr->irq);

  /* Disable I2C */

  i2c_putreg32(priv, ctrla & ~I2C_CTRLA_ENABLE, SAM_I2C_CTRLA_OFFSET);

  /* Wait it get sync */

  i2c_wait_synchronization(priv);

  /* Reset I2C */

  i2c_putreg32(priv, I2C_CTRLA_SWRST, SAM_I2C_CTRLA_OFFSET);

  /* Wait sync again before re-initialize */

  i2c_wait_synchronization(priv);

  /* Re-initialize the port hardware */

  i2c_hw_initialize(priv, priv->frequency);
  ret = OK;

  /* Release our lock on the bus */

  i2c_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_I2C_RESET */
#endif /* CONFIG_SAM_I2C_MASTER */
