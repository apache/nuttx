/****************************************************************************
 * arch/arm/src/samd/sam_spi.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *      Datasheet", 42129J–SAM–12/2013
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
#include <semaphore.h>
#include <wdog.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spi/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "chip/sam_port.h"
#include "chip/sam_pinmap.h"
#include "chip/sam_spi.h"

#include "sam_sercom.h"
#include "sam_spi.h"

#ifdef SAMD_HAVE_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Debug *******************************************************************/
/* Check if SPI debug is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#  undef CONFIG_SAMD_SPI_REGDEBUG
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
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

/* The state of the one SPI chip select */

struct sam_spidev_s
{
  const struct spi_ops_s *ops; /* Externally visible part of the SPI interface */

  /* Fixed configuration */

  uint8_t sercom;              /* Identifies the SERCOM peripheral */
  uint8_t irq;                 /* SERCOM IRQ number */
  uint8_t gclkgen;             /* Source GCLK generator */
  port_pinset_t pad0;          /* Pin configuration for PAD0 */
  port_pinset_t pad1;          /* Pin configuration for PAD1 */
  port_pinset_t pad2;          /* Pin configuration for PAD2 */
  port_pinset_t pad3;          /* Pin configuration for PAD3 */
  uint32_t muxconfig;          /* Pad multiplexing configuration */
  uint32_t frequency;          /* Source clock frequency */
  uintptr_t base;              /* SERCOM base address */

  /* Dynamic configuration */

#ifndef CONFIG_SPI_OWNBUS
  sem_t spilock;               /* Used to managed exclusive access to the bus */
  uint32_t frequency;          /* Requested clock frequency */
  uint32_t actual;             /* Actual clock frequency */
  uint8_t nbits;               /* Width of word in bits (8 to 16) */
  uint8_t mode;                /* Mode 0,1,2,3 */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMD_SPI_REGDEBUG
   bool     wrlast;            /* Last was a write */
   uint32_t addresslast;       /* Last address */
   uint32_t valuelast;         /* Last value */
   int      ntimes;            /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_SAMD_SPI_REGDEBUG
static bool     spi_checkreg(struct sam_spidev_s *spi, bool wr,
                  uint32_t regval, uint32_t regaddr);
#else
# define        spi_checkreg(spi,wr,regval,regaddr) (false)
#endif

static inline uint8_t spi_getreg8(struct sam_spidev_s *spi,
                  unsigned int offset);
static inline void spi_putreg8(struct sam_spidev_s *spi, uint8_t regval,
                  unsigned int offset);
static inline uint16_t spi_getreg16(struct sam_spidev_s *spi,
                  unsigned int offset);
static inline void spi_putreg16(struct sam_spidev_s *spi, uint16_t regval,
                  unsigned int offset);
static inline uint32_t spi_getreg32(struct sam_spidev_s *spi,
                  unsigned int offset);
static inline void spi_putreg32(struct sam_spidev_s *spi, uint32_t regval,
                  unsigned int offset);

#if defined(CONFIG_DEBUG_SPI) && defined(CONFIG_DEBUG_VERBOSE)
static void     spi_dumpregs(struct sam_spidev_s *spi, const char *msg);
#else
# define        spi_dumpregs(spi,msg)
#endif

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      spi_lock(struct spi_dev_s *dev, bool lock);
#endif
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(struct spi_dev_s *dev, uint16_t ch);
static void     spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                   void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(struct spi_dev_s *dev,
                   const void *buffer, size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                   size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef SAMD_HAVE_SPI0
/* SPI0 driver operations */

static const struct spi_ops_s g_spi0ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = sam_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI0 controller */

static struct sam_spidev_s g_spi0dev =
{
  .ops       = &g_spi0ops,
  .sercom    = 0,
  .irq       = SAM_IRQ_SERCOM0,
  .gclkgen   = (BOARD_SERCOM0_GCLKGEN >> GCLK_CLKCTRL_GEN_SHIFT),
  .pad0      = BOARD_SERCOM0_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM0_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM0_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM0_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM0_MUXCONFIG,
  .frequency = BOARD_SERCOM0_FREQUENCY,
  .base      = SAM_SERCOM0_BASE,
#ifndef CONFIG_SPI_OWNBUS
  .spilock   = SEM_INITIALIZER(1),
#endif
};
#endif

#ifdef SAMD_HAVE_SPI1
/* SPI1 driver operations */

static const struct spi_ops_s g_spi1ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = sam_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI1 controller */

static struct sam_spidev_s g_spi1dev =
{
  .ops       = &g_spi1ops,
  .sercom    = 1,
  .irq       = SAM_IRQ_SERCOM1,
  .gclkgen   = (BOARD_SERCOM1_GCLKGEN >> GCLK_CLKCTRL_GEN_SHIFT),
  .pad0      = BOARD_SERCOM1_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM1_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM1_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM1_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM1_MUXCONFIG,
  .frequency = BOARD_SERCOM1_FREQUENCY,
  .base      = SAM_SERCOM1_BASE,
#ifndef CONFIG_SPI_OWNBUS
  .spilock   = SEM_INITIALIZER(1),
#endif
};
#endif

#ifdef SAMD_HAVE_SPI2
/* SPI2 driver operations */

static const struct spi_ops_s g_spi2ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = sam_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI2 controller */

static struct sam_spidev_s g_spi2dev =
{
  .ops       = &g_spi1ops,
  .sercom    = 2,
  .irq       = SAM_IRQ_SERCOM2,
  .gclkgen   = (BOARD_SERCOM2_GCLKGEN >> GCLK_CLKCTRL_GEN_SHIFT),
  .pad0      = BOARD_SERCOM2_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM2_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM2_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM2_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM2_MUXCONFIG,
  .frequency = BOARD_SERCOM2_FREQUENCY,
  .base      = SAM_SERCOM2_BASE,
#ifndef CONFIG_SPI_OWNBUS
  .spilock   = SEM_INITIALIZER(1),
#endif
};
#endif

#ifdef SAMD_HAVE_SPI3
/* SPI3 driver operations */

static const struct spi_ops_s g_spi3ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = sam_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI3 controller */

static struct sam_spidev_s g_spi3dev =
{
  .ops       = &g_spi3ops,
  .sercom    = 3,
  .irq       = SAM_IRQ_SERCOM3,
  .gclkgen   = (BOARD_SERCOM3_GCLKGEN >> GCLK_CLKCTRL_GEN_SHIFT),
  .pad0      = BOARD_SERCOM3_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM3_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM3_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM3_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM3_MUXCONFIG,
  .frequency = BOARD_SERCOM3_FREQUENCY,
  .base      = SAM_SERCOM3_BASE,
#ifndef CONFIG_SPI_OWNBUS
  .spilock   = SEM_INITIALIZER(1),
#endif
};
#endif

#ifdef SAMD_HAVE_SPI4
/* SPI4 driver operations */

static const struct spi_ops_s g_spi4ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = sam_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi4cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI4 controller */

static struct sam_spidev_s g_spi4dev =
{
  .ops       = &g_spi4ops,
  .sercom    = 4,
  .irq       = SAM_IRQ_SERCOM4,
  .gclkgen   = (BOARD_SERCOM4_GCLKGEN >> GCLK_CLKCTRL_GEN_SHIFT),
  .pad0      = BOARD_SERCOM4_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM4_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM4_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM4_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM4_MUXCONFIG,
  .frequency = BOARD_SERCOM4_FREQUENCY,
  .base      = SAM_SERCOM4_BASE,
#ifndef CONFIG_SPI_OWNBUS
  .spilock   = SEM_INITIALIZER(1),
#endif
};
#endif

#ifdef SAMD_HAVE_SPI5
/* SPI5 driver operations */

static const struct spi_ops_s g_spi5ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = sam_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi5cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI5 controller */

static struct sam_spidev_s g_spi5dev =
{
  .ops       = &g_spi5ops,
  .sercom    = 5,
  .irq       = SAM_IRQ_SERCOM5,
  .gclkgen   = (BOARD_SERCOM5_GCLKGEN >> GCLK_CLKCTRL_GEN_SHIFT),
  .pad0      = BOARD_SERCOM5_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM5_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM5_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM5_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM5_MUXCONFIG,
  .frequency = BOARD_SERCOM5_FREQUENCY,
  .base      = SAM_SERCOM5_BASE,
#ifndef CONFIG_SPI_OWNBUS
  .spilock   = SEM_INITIALIZER(1),
#endif
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval   - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD_SPI_REGDEBUG
static bool spi_checkreg(struct sam_spidev_s *priv, bool wr, uint32_t regval,
                         uint32_t regaddr)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      regval  == priv->valuelast &&  /* Same value? */
      regaddr == priv->addresslast)  /* Same address? */
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

      priv->wrlast      = wr;
      priv->valuelast   = regval;
      priv->addresslast = regaddr;
      priv->ntimes      = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: spi_getreg8
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint8_t spi_getreg8(struct sam_spidev_s *priv,
                                   unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;
  uint8_t regval = getreg8(regaddr);

#ifdef CONFIG_SAMD_SPI_REGDEBUG
  if (spi_checkreg(priv, false, (uint32_t)regval, regaddr))
    {
      lldbg("%08x->%02x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: spi_putreg8
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void spi_putreg8(struct sam_spidev_s *priv, uint8_t regval,
                               unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;

#ifdef CONFIG_SAMD_SPI_REGDEBUG
  if (spi_checkreg(priv, true, (uint32_t)regval, regaddr))
    {
      lldbg("%08x<-%02x\n", regaddr, regval);
    }
#endif

  putreg8(regval, regaddr);
}

/****************************************************************************
 * Name: spi_getreg16
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint16_t spi_getreg16(struct sam_spidev_s *priv,
                                    unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;
  uint16_t regval = getreg16(regaddr);

#ifdef CONFIG_SAMD_SPI_REGDEBUG
  if (spi_checkreg(priv, false, (uint32_t)regval, regaddr))
    {
      lldbg("%08x->%04x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: spi_putreg16
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void spi_putreg16(struct sam_spidev_s *priv, uint16_t regval,
                                unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;

#ifdef CONFIG_SAMD_SPI_REGDEBUG
  if (spi_checkreg(priv, true, (uint32_t)regval, regaddr))
    {
      lldbg("%08x<-%04x\n", regaddr, regval);
    }
#endif

  putreg16(regval, regaddr);
}

/****************************************************************************
 * Name: spi_getreg32
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg32(struct sam_spidev_s *priv,
                                    unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;
  uint32_t regval = getreg32(regaddr);

#ifdef CONFIG_SAMD_SPI_REGDEBUG
  if (spi_checkreg(priv, false, regval, regaddr))
    {
      lldbg("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: spi_putreg32
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void spi_putreg32(struct sam_spidev_s *priv, uint32_t regval,
                                unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;

#ifdef CONFIG_SAMD_SPI_REGDEBUG
  if (spi_checkreg(priv, true, regval, regaddr))
    {
      lldbg("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: spi_dumpregs
 *
 * Description:
 *   Dump the contents of all SPI registers
 *
 * Input Parameters:
 *   priv - The SPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_SPI) && defined(CONFIG_DEBUG_VERBOSE)
static void spi_dumpregs(struct sam_spidev_s *priv, const char *msg)
{
  spivdbg("%s:\n", msg);
  spivdbg("   CTRLA:%08x CTRLB:%08x DBGCTRL:%08x\n",
          getreg32(priv->base + SAM_SPI_CTRLA_OFFSET),
          getreg32(priv->base + SAM_SPI_CTRLB_OFFSET),
          getreg32(priv->base + SAM_SPI_DBGCTRL_OFFSET));
  spivdbg("    BAUD:%08x INTEN:%08x INTFLAG:%08x\n",
          getreg32(priv->base + SAM_SPI_BAUD_OFFSET),
          getreg32(priv->base + SAM_SPI_INTENCLR_OFFSET),
          getreg32(priv->base + SAM_SPI_INTFLAG_OFFSET));
  spivdbg("  STATUS:%08x  ADDR:%08x\n",
          getreg32(priv->base + SAM_SPI_STATUS_OFFSET),
          getreg32(priv->base + SAM_SPI_ADDR_OFFSET));
}
#endif

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock priv bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)dev;

  spivdbg("lock=%d\n", lock);
  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->spilock) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->spilock);
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

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct sam_spidev_s *priv =(struct sam_spidev_s *)dev;
  uint32_t actual;

  spivdbg("sercom=%d frequency=%d\n", priv->sercom, frequency);

  /* Check if the requested frequency is the same as the frequency selection */

#ifndef CONFIG_SPI_OWNBUS
  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }
#endif

  /* Configure SPI to a frequency as close as possible to the requested
   * frequency.
   */
#warning Missing logic

  /* Calculate the new actual frequency */
#warning Missing logic

  spivdbg("actual=%d\n", offset, regval, actual);

  /* Save the frequency setting */

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = frequency;
  priv->actual    = actual;
#endif

  spidbg("Frequency %d->%d\n", frequency, actual);
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

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)dev;
  uint32_t regval;

  spivdbg("sercom=%d mode=%d\n", priv->sercom, mode);

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->mode)
#endif
    {
      /* Yes... Set the mode appropriately */

      regval = spi_regetreg(SAM_SPI_CTRLA_OFFSET);
      regval &= ~(SPI_CTRLA_CPOL | SPI_CTRLA_CPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SPI_CTRLA_CPHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SPI_CTRLA_CPOL;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SPI_CTRLA_CPOL | SPI_CTRLA_CPHA);
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg32(priv, regval, offset);

      /* Save the mode so that subsequent re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->mode = mode;
#endif
    }
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

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)dev;
  uint32_t regval;
  unsigned int offset;

  spivdbg("sercom=%d nbits=%d\n", priv->sercom, nbits);
  DEBUGASSERT(priv && nbits > 7 && nbits < 10);

  /* NOTE:  The logic in spi_send and in spi_exchange only handles 8-bit
   * data at the present time.  So the following extra assertion is a
   * reminder that we have to fix that someday.
   */

  DEBUGASSERT(nbits == 8); /* Temporary -- FIX ME */

  /* Has the number of bits changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (nbits != priv->nbits)
#endif
    {
      /* Yes... Set number of bits appropriately */

      regval  = spi_getreg32(priv, SAM_SPI_CTRLB_OFFSET);
      regval &= ~SPI_CTRLB_CHSIZE_MASK;

      if (nbits == 9)
        {
          regval |= SPI_CTRLB_CHSIZE_9BITS;
        }

      spi_putreg32(priv, regval, SAM_SPI_CTRLB_OFFSET);

      /* Save the selection so the subsequence re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->nbits = nbits;
#endif
    }
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

static uint16_t spi_send(struct spi_dev_s *dev, uint16_t wd)
{
  uint8_t txbyte;
  uint8_t rxbyte;

  /* spi_exchange can do this. Note: right now, this only deals with 8-bit
   * words.  If the SPI interface were configured for words of other sizes,
   * this would fail.
   */

  txbyte = (uint8_t)wd;
  rxbyte = (uint8_t)0;
  spi_exchange(dev, &txbyte, &rxbyte, 1);

  spivdbg("Sent %02x received %02x\n", txbyte, rxbyte);
  return (uint16_t)rxbyte;
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
              void *rxbuffer, size_t nwords)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)dev;
  cont uint8_t *txptr = (const uint8_t *)txbuffer;
  uint8_t *rxptr = (uint8_t *)rxbuffer;
  uint16_t data;

  spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Loop, sending each word in the user-provided data buffer.
   *
   * Note 1: Right now, this only deals with 8-bit words.  If the SPI
   *         interface were configured for words of other sizes, this
   *         would fail.
   * Note 2: This loop might be made more efficient.  Would logic
   *         like the following improve the throughput?  Or would it
   *         just add the risk of overruns?
   *
   *   Get word 1;
   *   Send word 1;  Now word 1 is "in flight"
   *   nwords--;
   *   for ( ; nwords > 0; nwords--)
   *     {
   *       Get word N.
   *       Wait for DRE:: meaning that word N-1 has moved to the shift
   *          register.
   *       Disable interrupts to keep the following atomic
   *       Send word N.  Now both work N-1 and N are "in flight"
   *       Wait for RXC: meaning that word N-1 is available
   *       Read word N-1.
   *       Re-enable interrupts.
   *       Save word N-1.
   *     }
   *   Wait for RXC: meaning that the final word is available
   *   Read the final word.
   *   Save the final word.
   */

  for ( ; nwords > 0; nwords--)
    {
      /* Get the data to send (0xff if there is no data source) */

      if (txptr)
        {
          data = (uint32_t)*txptr++;
        }
      else
        {
          data = 0xffff;
        }

      /* Wait for any previous data written to the DATA register to be
       * transferred to the serializer.
       */

      while ((spi_getreg32(priv, SAM_SPI_INTFLAG_OFFSET) & SPI_INT_DRE) == 0);

      /* Write the data to transmitted to the DATA Register (TDR) */

      spi_putreg32(priv, data, SAM_SPI_DATA_OFFSET);

      /* Wait for the read data to be available in the DATA register. */

      while ((spi_getreg32(priv, SAM_SPI_INTFLAG_OFFSET) & SPI_INT_RXC) == 0);

      /* Read the received data from the SPI DATA Register..
       * TODO: The following only works if nbits <= 8.
       */

      data = spi_getreg16(priv, SAM_SPI_DATA_OFFSET);
      if (rxptr)
        {
          *rxptr++ = (uint8_t)data;
        }
    }
}

/***************************************************************************
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

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer, size_t nwords)
{
  /* spi_exchange can do this. */

  spi_exchange(dev, buffer, NULL, nwords);
}
#endif

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

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev, void *buffer, size_t nwords)
{
  /* spi_exchange can do this. */

  spi_exchange(dev, NULL, buffer, nwords);
}
#endif

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
 *   port - SPI "port" number (i.e., SERCOM number)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *up_spiinitialize(int port)
{
  struct sam_spidev_s *priv;
  irqstate_t flags;
#ifndef CONFIG_SPI_OWNBUS
  uint32_t regval;
  unsigned int offset;
#endif

  /* Get the port state structure */

  spivdbg("port: %d \n", port);

#ifdef SAMD_HAVE_SPI0
  if (port == 0)
    {
      priv = &g_spi0dev;
    }
  else
#endif

#ifdef SAMD_HAVE_SPI1
  if (port == 1)
    {
      priv = &g_spi1dev;
    }
  else
#endif

#ifdef SAMD_HAVE_SPI2
  if (port == 2)
    {
      priv = &g_spi2dev;
    }
  else
#endif

#ifdef SAMD_HAVE_SPI3
  if (port == 3)
    {
      priv = &g_spi3dev;
    }
  else
#endif

#ifdef SAMD_HAVE_SPI4
  if (port == 4)
    {
      priv = &g_spi4dev;
    }
  else
#endif

#ifdef SAMD_HAVE_SPI5
  if (port == 5)
    {
      priv = &g_spi5dev;
    }
  else
#endif
    {
      spidbg("ERROR: Unsupported port: %d\n", port);
      return NULL;
    }

  /* Enable clocking to the SPI SERCOM */

      flags = irqsave();
#warning Missing logic

  /* Configure multiplexed pins as connected on the board. */
#warning Missing logic

  /* Execute a software reset of the SPI SERCOM */
#warning Missing logic

  /* Configure the SPI SERCOM */
#warning Missing logic

 /* And enable the SPI */
#warning Missing logic

  spi_dumpregs(priv, "After initialization");

#ifndef CONFIG_SPI_OWNBUS
  /* Set to mode=0 and nbits=8 and some initial frequency.  It is only
   * critical to do this if CONFIG_SPI_OWNBUS is not defined because in
   * that case, the SPI will only be reconfigured if there is a change.
   */

  spi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);
  spi_setfrequency((struct spi_dev_s *)priv, 400000);
  spi_setbits((struct spi_dev_s *)priv, 8);
#endif

  return (struct spi_dev_s *)priv;
}

#endif /* SAMD_HAVE_SPI */
