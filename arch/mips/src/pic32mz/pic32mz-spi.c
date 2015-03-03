/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz-spi.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/pic32mz-spi.h"
#include "chip/pic32mz-pps.h"
#include "pic32mz-spi.h"

#ifdef CONFIG_PIC32MZ_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* All SPI peripherals are clocked by PBCLK2 */

#define BOARD_PBCLOCK BOARD_PBCLK2

/* Enables non-standard debug output from this file.
 *
 * CONFIG_SPI_DEBUG && CONFIG_DEBUG - Define to enable basic SPI debug
 * CONFIG_DEBUG_VERBOSE - Define to enable verbose SPI debug
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_SPI
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_SPI_REGDEBUG
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SSP driver */

struct pic32mz_dev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         base;       /* SPI register base address */
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  uint8_t          firq;       /* SPI fault interrupt number */
  uint8_t          rxirq;      /* SPI receive done interrupt number */
  uint8_t          txirq;      /* SPI transfer done interrupt number */
#endif
  uint8_t          sdipps;     /* SDI peripheral pin selection */
  uint8_t          sdopps;     /* SDO peripheral pin selection */
  uintptr_t        sdoreg;     /* SDO peripheral pin configuration register */
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low-level register access */

static uint32_t spi_getreg(FAR struct pic32mz_dev_s *priv,
                  unsigned int offset);
static void     spi_putreg(FAR struct pic32mz_dev_s *priv,
                  unsigned int offset, uint32_t value);

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
static void     spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI1

static const struct spi_ops_s g_spi1ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = pic32mz_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi1cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi1register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mz_dev_s g_spi1dev =
{
  .spidev            = { &g_spi1ops },
  .base              = PIC32MZ_SPI1_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI1F,
  .rxirq             = PIC32MZ_IRQ_SPI1RX,
  .txirq             = PIC32MZ_IRQ_SPI1TX,
#endif
  .sdipps            = BOARD_SDI1_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO1_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO1_PPS),
};
#endif

#ifdef CONFIG_PIC32MZ_SPI2
static const struct spi_ops_s g_spi2ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = pic32mz_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi2cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi2register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mz_dev_s g_spi2dev =
{
  .spidev            = { &g_spi2ops },
  .base              = PIC32MZ_SPI2_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI2F,
  .rxirq             = PIC32MZ_IRQ_SPI2RX,
  .txirq             = PIC32MZ_IRQ_SPI2TX,
#endif
  .sdipps            = BOARD_SDI2_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO2_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO2_PPS),
};
#endif

#ifdef CONFIG_PIC32MZ_SPI3
static const struct spi_ops_s g_spi3ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = pic32mz_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi3cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi3register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mz_dev_s g_spi3dev =
{
  .spidev            = { &g_spi3ops },
  .base              = PIC32MZ_SPI3_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI3F,
  .rxirq             = PIC32MZ_IRQ_SPI3RX,
  .txirq             = PIC32MZ_IRQ_SPI3TX,
#endif
  .sdipps            = BOARD_SDI3_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO3_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO3_PPS),
};
#endif

#ifdef CONFIG_PIC32MZ_SPI4
static const struct spi_ops_s g_spi4ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = pic32mz_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi4cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi4register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mz_dev_s g_spi4dev =
{
  .spidev            = { &g_spi4ops },
  .base              = PIC32MZ_SPI4_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI4F,
  .rxirq             = PIC32MZ_IRQ_SPI4RX,
  .txirq             = PIC32MZ_IRQ_SPI4TX,
#endif
  .sdipps            = BOARD_SDI4_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO4_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO4_PPS),
};
#endif

#ifdef CONFIG_PIC32MZ_SPI5
static const struct spi_ops_s g_spi5ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = pic32mz_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi5cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi5register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mz_dev_s g_spi5dev =
{
  .spidev            = { &g_spi5ops },
  .base              = PIC32MZ_SPI5_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI5F,
  .rxirq             = PIC32MZ_IRQ_SPI5RX,
  .txirq             = PIC32MZ_IRQ_SPI5TX,
#endif
  .sdipps            = BOARD_SDI5_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO5_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO5_PPS),
};
#endif

#ifdef CONFIG_PIC32MZ_SPI6
static const struct spi_ops_s g_spi6ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = pic32mz_spi6select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi6status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi6cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi6register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mz_dev_s g_spi6dev =
{
  .spidev            = { &g_spi6ops },
  .base              = PIC32MZ_SPI6_K1BASE,
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  .firq              = PIC32MZ_IRQ_SPI6F,
  .rxirq             = PIC32MZ_IRQ_SPI6RX,
  .txirq             = PIC32MZ_IRQ_SPI6TX,
#endif
  .sdipps            = BOARD_SDI6_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO6_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO6_PPS),
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Return the contents of one, 32-bit SPI register
 *
 * Input Parameters:
 *   priv   - A pointer to a PIC32MZ SPI state structure
 *   offset - Offset from the SPI base address to the register of interest
 *
 * Returned Value:
 *   The current contents of the register
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_REGDEBUG
static uint32_t spi_getreg(FAR struct pic32mz_dev_s *priv, unsigned int offset)
{
  /* Last address, value, and count */

  static uint32_t prevaddr = 0;
  static uint32_t prevalue = 0;
  static uint32_t count = 0;

  /* New address and value */

  uint32_t addr;
  uint32_t value;

  /* Read the value from the register */

  addr  = priv->base + offset;
  value = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && value == prevalue)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return value;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       prevalue = value;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", addr, value);
  return value;
}
#else
static uint32_t spi_getreg(FAR struct pic32mz_dev_s *priv, unsigned int offset)
{
  return getreg32(priv->base + offset);
}
#endif

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a value to one, 32-bit SPI register
 *
 * Input Parameters:
 *   priv   - A pointer to a PIC32MZ SPI state structure
 *   offset - Offset from the SPI base address to the register of interest
 *   value  - The value to write to the SPI register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_REGDEBUG
static void spi_putreg(FAR struct pic32mz_dev_s *priv, unsigned int offset,
                       uint32_t value)
{
  uint32_t addr;

  /* Get the address to write to */

  addr = priv->base + offset;

  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, value);

  /* Then do the write */

  putreg32(value, addr);
}
#else
static void spi_putreg(FAR struct pic32mz_dev_s *priv, unsigned int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}
#endif

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->exclsem);
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  uint32_t divisor;
  uint32_t actual;
  uint32_t regval;

#ifndef CONFIG_SPI_OWNBUS
  spivdbg("Old frequency: %d actual: %d New frequency: %d\n",
          priv->frequency, priv->actual, frequency);
#else
  spivdbg("New frequency: %d\n", regval);
#endif

  /* Check if the requested frequency is the same as the frequency selection */

#ifndef CONFIG_SPI_OWNBUS
  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }
#endif

  /* Calculate the divisor
   *
   * frequency = BOARD_PBCLOCK / (2 * divisor), or
   * divisor  = (BOARD_PBCLOCK / 2) / frequency
   */

  divisor = (BOARD_PBCLOCK / 2) / frequency;

  /* The a BRG register value is that divisor minus one
   *
   * frequency = BOARD_PBCLOCK /(2 * (BRG + 1)), or
   * BRG       = (BOARD_PBCLOCK / 2) / frequency - 1
   */

  regval = divisor;
  if (regval > 0)
    {
      regval--;
    }

  /* Save the new BRG value */

  spi_putreg(priv, PIC32MZ_SPI_BRG_OFFSET, regval);
  spivdbg("PBCLOCK: %d frequency: %d divisor: %d BRG: %d\n",
          BOARD_PBCLOCK, frequency, divisor, regval);

  /* Calculate the new actual frequency.
   *
   * frequency = BOARD_PBCLOCK / (2 * divisor)
   */

  actual = (BOARD_PBCLOCK / 2) / divisor;

  /* Save the frequency setting */

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = frequency;
  priv->actual    = actual;
#endif

  spidbg("New frequency: %d Actual: %d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  uint32_t regval;

#ifndef CONFIG_SPI_OWNBUS
  spivdbg("Old mode: %d New mode: %d\n", priv->mode, mode);
#else
  spivdbg("New mode: %d\n", mode);
#endif

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->mode)
    {
#endif
      /* Yes... Set CON register appropriately.
       *
       * Standard terminology is as follows:
       *
       *   Mode  CPOL  CPHA
       *     0     0     0
       *     1     0     1
       *     2     1     0
       *     3     1     1
       *
       *   CPOL=0: The inactive value of the clock is zero
       *    CPOL=1: The inactive value of the clock is one
       *   CPHA=0: Data is captured on the clock's inactive-to-active edge and
       *           data is propagated on a active-to-inactive edge.
       *   CPHA=1: Data is captured on the clock's active-to-inactive edge and
       *           data is propagated on a active-to-inactive edge.
       *
       * CON Register mapping:
       *   CPOL=0 corresponds to CON:CKP=0; CPOL=1 corresponds to CON:CKP=1
       *   CPHA=0 corresponds to CON:CKE=1; CPHA=1 corresponds to CON:CKE=1
       *
       * In addition, the CON register supports SMP: SPI Data Input Sample
       * Phase bit:
       *
       *    1 = Input data sampled at end of data output time
       *    0 = Input data sampled at middle of data output time
       *
       * Which is hardcoded to 1.
       */

      regval = spi_getreg(priv, PIC32MZ_SPI_CON_OFFSET);
      regval &= ~(SPI_CON_CKP|SPI_CON_CKE);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SPI_CON_CKE;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SPI_CON_CKP;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SPI_CON_CKP|SPI_CON_CKE);
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, regval);
      spivdbg("CON: %08x\n", regval);

      /* Save the mode so that subsequent re-configuratins will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->mode = mode;
    }
#endif
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  uint32_t setting;
  uint32_t regval;

#ifndef CONFIG_SPI_OWNBUS
  spivdbg("Old nbits: %d New nbits: %d\n", priv->nbits, nbits);
#else
  spivdbg("New nbits: %d\n", nbits);
#endif

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 7 && nbits < 17);
#ifndef CONFIG_SPI_OWNBUS
  if (nbits != priv->nbits)
    {
#endif
      /* Yes... Set the CON register appropriately */

      if (nbits == 8)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else if (nbits == 16)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else if (nbits == 32)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else
        {
          spidbg("Unsupported nbits: %d\n", nbits);
          return;
        }

      regval = spi_getreg(priv, PIC32MZ_SPI_CON_OFFSET);
      regval &= ~SPI_CON_MODE_MASK;
      regval |= setting;
      regval = spi_getreg(priv, PIC32MZ_SPI_CON_OFFSET);
      spivdbg("CON: %08x\n", regval);

      /* Save the selection so the subsequence re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->nbits = nbits;
    }
#endif
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;

  spivdbg("wd: %04x\n", wd);

  /* Write the data to transmitted to the SPI Data Register */

  spi_putreg(priv, PIC32MZ_SPI_BUF_OFFSET, (uint32_t)wd);

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
  /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0. In
   * enhanced buffer mode, the SPIRBE bit will be cleared in  when the
   * receive buffer is not empty.
   */

 while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBE) != 0);
#else
  /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1. In
   * normal mode, the SPIRBF bit will be set when receive data is available.
   */

 while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBF) == 0);
#endif

 /* Return the SPI data */

 return (uint16_t)spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;
  uint32_t regval;
  uint8_t data;

  spivdbg("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write the data to transmitted to the SPI Data Register */

      data = *ptr++;
      spi_putreg(priv, PIC32MZ_SPI_BUF_OFFSET, (uint32_t)data);

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0. In
       * enhanced buffer mode, the SPIRBE bit will be cleared in  when the
       * receive buffer is not empty.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1. In
       * normal mode, the SPIRBF bit will be set when receive data is available.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBF) == 0);
#endif

     /* Read from the buffer register to clear the status bit */

     regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
     UNUSED(regval);
     nwords--;
    }
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  FAR struct pic32mz_dev_s *priv = (FAR struct pic32mz_dev_s *)dev;
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;

  spivdbg("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write some dummy data to the SPI Data Register in order to clock the
       * read data.
       */

      spi_putreg(priv, PIC32MZ_SPI_BUF_OFFSET, 0xff);

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0. In
       * enhanced buffer mode, the SPIRBE bit will be cleared in  when the
       * receive buffer is not empty.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1. In
       * normal mode, the SPIRBF bit will be set when receive data is available.
       */

     while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBF) == 0);
#endif

     /* Read the received data from the SPI Data Register */

     *ptr++ = (uint8_t)spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
     nwords--;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct pic32mz_dev_s *priv;
  uintptr_t regaddr;
  irqstate_t flags;
  uint32_t regval;

  spivdbg("port: %d\n", port);

  /* Select the SPI state structure and SDI PPS register for this port */

#ifdef CONFIG_PIC32MZ_SPI1
  if (port == 1)
    {
      priv    = &g_spi1dev;
      regaddr = PIC32MZ_SDI1R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI2
  if (port == 2)
    {
      priv = &g_spi2dev;
      regaddr = PIC32MZ_SDI2R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI3
  if (port == 3)
    {
      priv = &g_spi3dev;
      regaddr = PIC32MZ_SDI3R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI4
  if (port == 4)
    {
      priv = &g_spi4dev;
      regaddr = PIC32MZ_SDI4R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI5
  if (port == 5)
    {
      priv = &g_spi5dev;
      regaddr = PIC32MZ_SDI5R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI6
  if (port == 6)
    {
      priv = &g_spi6dev;
      regaddr = PIC32MZ_SDI6R;
    }
  else
#endif
   {
     spidbg("Unsuppport port: %d\n", port);
     return NULL;
   }

  /* Disable SPI interrupts */

  flags = irqsave();
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  up_disable_irq(priv->firq);
  up_disable_irq(priv->txirq);
  up_disable_irq(priv->rxirq);
#endif

  /* Stop and reset the SPI module by clearing the ON bit in the CON register. */

  spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, 0);

  /* Clear the receive buffer by reading from the BUF register */

  regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);

  /* Configure SPI SDI (input) and SDO (output) pins. SS (output) pins are
   * managed as GPIOs; CLK (output) pins are not selectable.
   */

  putreg32((uint32_t)priv->sdipps, regaddr);
  putreg32((uint32_t)priv->sdopps, priv->sdoreg);

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  /* Attach the interrupt handlers.  We do this early to make sure that the
   * resources are available.
   */

  ret = irq_attach(priv->rxirq, spi_interrupt);
  if (ret < 0)
    {
      spidbg("Failed to attach RX interrupt: %d port: %d\n", priv->rxirq, port);
      goto errout;
    }

  ret = irq_attach(priv->txirq, spi_interrupt);
  if (ret < 0)
    {
      spidbg("Failed to attach TX interrupt: %d port: %d\n", priv->txirq, port);
      goto errout_with_rxirq;
    }

  ret = irq_attach(priv->firq, spi_interrupt);
  if (ret < 0)
    {
      spidbg("Failed to attach fault interrupt: %d port: %d\n", priv->firq, port);
      goto errout_with_txirq;
    }
#endif

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Clear the SPIROV overflow bit (SPIxSTAT:6). */

  spi_putreg(priv, PIC32MZ_SPI_STATCLR_OFFSET, SPI_STAT_SPIROV);

  /* Initial settings 8 bit +  master mode + mode 0.  NOTE that MSSEN
   * not set:  The slave select pin must be driven manually via the
   * board-specific pic32mz_spiNselect() interface.
   */

  regval = (SPI_CON_MSTEN | SPI_CON_SMP | SPI_CON_MODE_8BIT | SPI_CON_ON);

  /* Set the ENHBUF bit if using Enhanced Buffer mode. */

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
  regval |= (SPI_CON_ENHBUF | SPI_CON_SRXISEL_HALF | SPI_CON_STXISEL_HALF);
#endif
  spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, regval);
  spivdbg("CON: %08x\n", regval);

  /* Set the initial SPI configuration */

#ifndef CONFIG_SPI_OWNBUS
  priv->nbits = 8;
  priv->mode  = SPIDEV_MODE0;
#endif

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  /* Enable interrupts at the SPI controller */

  up_enable_irq(priv->firq);
  up_enable_irq(priv->txirq);
  up_enable_irq(priv->rxirq);
#endif

  /* Enable interrupts at the interrupt controller */

  irqrestore(flags);
  return &priv->spidev;

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
errout_with_txirq:
  irq_detatch(priv->txirq);
errout_with_rxirq:
  irq_detatch(priv->rxirq);
errout:
  irqrestore(flags);
  return NULL;
#endif
}

#endif /* CONFIG_PIC32MZ_SPI */
