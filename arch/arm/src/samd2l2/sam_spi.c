/****************************************************************************
 * arch/arm/src/samd2l2/sam_spi.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* References:
 *   1. "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *      Datasheet", 42129J–SAM–12/2013
 */

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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "sam_pinmap.h"
#include "sam_gclk.h"
#include "sam_port.h"
#include "sam_sercom.h"
#include "sam_spi.h"

#ifdef CONFIG_SAMD2L2_SPI_DMA
#  include "sam_dmac.h"
#endif

#include <arch/board/board.h>

#ifdef SAMD2L2_HAVE_SPI

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG_SPI_INFO
#  undef CONFIG_SAMD2L2_SPI_REGDEBUG
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
#if 0 /* Not used */
  uint8_t irq;                 /* SERCOM IRQ number */
#endif
  uint8_t gclkgen;             /* Source GCLK generator */
  uint8_t slowgen;             /* Slow GCLK generator */
  port_pinset_t pad0;          /* Pin configuration for PAD0 */
  port_pinset_t pad1;          /* Pin configuration for PAD1 */
  port_pinset_t pad2;          /* Pin configuration for PAD2 */
  port_pinset_t pad3;          /* Pin configuration for PAD3 */
  uint32_t muxconfig;          /* Pad multiplexing configuration */
  uint32_t srcfreq;            /* Source clock frequency */
  uintptr_t base;              /* SERCOM base address */
#if 0 /* Not used */
  xcpt_t handler;              /* SERCOM interrupt handler */
#endif

  /* Dynamic configuration */

  mutex_t spilock;             /* Used to managed exclusive access to the bus */
  uint32_t frequency;          /* Requested clock frequency */
  uint32_t actual;             /* Actual clock frequency */
  uint8_t mode;                /* Mode 0,1,2,3 */
  uint8_t nbits;               /* Width of word in bits (8 to 16) */

#ifdef CONFIG_SAMD2L2_SPI_DMA
  /* DMA */

  uint8_t dma_tx_trig;         /* DMA TX trigger source to use */
  uint8_t dma_rx_trig;         /* DMA RX trigger source to use */
  DMA_HANDLE dma_tx;           /* DMA TX channel handle */
  DMA_HANDLE dma_rx;           /* DMA RX channel handle */
  sem_t dmasem;                /* Transfer wait semaphore */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
  bool     wr;                /* Last was a write */
  uint32_t regaddr;           /* Last address */
  uint32_t regval;            /* Last value */
  int      ntimes;            /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
static bool     spi_checkreg(struct sam_spidev_s *priv, bool wr,
                  uint32_t regval, uint32_t regaddr);
#else
# define        spi_checkreg(priv,wr,regval,regaddr) (false)
#endif

static uint8_t  spi_getreg8(struct sam_spidev_s *priv,
                  unsigned int offset);
static void     spi_putreg8(struct sam_spidev_s *priv, uint8_t regval,
                  unsigned int offset);
static uint16_t spi_getreg16(struct sam_spidev_s *priv,
                  unsigned int offset);
static void     spi_putreg16(struct sam_spidev_s *priv, uint16_t regval,
                  unsigned int offset);
static uint32_t spi_getreg32(struct sam_spidev_s *priv,
                  unsigned int offset);
static void     spi_putreg32(struct sam_spidev_s *priv, uint32_t regval,
                  unsigned int offset);

#ifdef CONFIG_SAMD2L2_SPI_DMA
static void spi_dma_setup(struct sam_spidev_s *priv);
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
static void     spi_dumpregs(struct sam_spidev_s *priv, const char *msg);
#else
# define        spi_dumpregs(priv,msg)
#endif

/* Interrupt handling */

#if 0 /* Not used */
static int      spi_interrupt(int irq, void *context, void *arg);
#endif

/* SPI methods */

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void     spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                   void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(struct spi_dev_s *dev,
                   const void *buffer, size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                   size_t nwords);
#endif

/* Initialization */

static void     spi_wait_synchronization(struct sam_spidev_s *priv);
static void     spi_pad_configure(struct sam_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef SAMD2L2_HAVE_SPI0
/* SPI0 driver operations */

static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = sam_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
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
  .ops         = &g_spi0ops,
  .sercom      = 0,
#if 0 /* Not used */
  .irq         = SAM_IRQ_SERCOM0,
#endif
  .gclkgen     = BOARD_SERCOM0_GCLKGEN,
  .slowgen     = BOARD_SERCOM0_SLOW_GCLKGEN,
  .pad0        = BOARD_SERCOM0_PINMAP_PAD0,
  .pad1        = BOARD_SERCOM0_PINMAP_PAD1,
  .pad2        = BOARD_SERCOM0_PINMAP_PAD2,
  .pad3        = BOARD_SERCOM0_PINMAP_PAD3,
  .muxconfig   = BOARD_SERCOM0_MUXCONFIG,
  .srcfreq     = BOARD_SERCOM0_FREQUENCY,
  .base        = SAM_SERCOM0_BASE,
  .spilock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_SAMD2L2_SPI_DMA
  .dma_tx_trig = DMAC_TRIGSRC_SERCOM0_TX,
  .dma_rx_trig = DMAC_TRIGSRC_SERCOM0_RX,
#endif
};
#endif

#ifdef SAMD2L2_HAVE_SPI1
/* SPI1 driver operations */

static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
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
  .ops         = &g_spi1ops,
  .sercom      = 1,
#if 0 /* Not used */
  .irq         = SAM_IRQ_SERCOM1,
#endif
  .gclkgen     = BOARD_SERCOM1_GCLKGEN,
  .slowgen     = BOARD_SERCOM1_SLOW_GCLKGEN,
  .pad0        = BOARD_SERCOM1_PINMAP_PAD0,
  .pad1        = BOARD_SERCOM1_PINMAP_PAD1,
  .pad2        = BOARD_SERCOM1_PINMAP_PAD2,
  .pad3        = BOARD_SERCOM1_PINMAP_PAD3,
  .muxconfig   = BOARD_SERCOM1_MUXCONFIG,
  .srcfreq     = BOARD_SERCOM1_FREQUENCY,
  .base        = SAM_SERCOM1_BASE,
  .spilock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_SAMD2L2_SPI_DMA
  .dma_tx_trig = DMAC_TRIGSRC_SERCOM1_TX,
  .dma_rx_trig = DMAC_TRIGSRC_SERCOM1_RX,
#endif
};
#endif

#ifdef SAMD2L2_HAVE_SPI2
/* SPI2 driver operations */

static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = sam_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi2cmddata,
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
  .ops         = &g_spi2ops,
  .sercom      = 2,
#if 0 /* Not used */
  .irq         = SAM_IRQ_SERCOM2,
#endif
  .gclkgen     = BOARD_SERCOM2_GCLKGEN,
  .slowgen     = BOARD_SERCOM2_SLOW_GCLKGEN,
  .pad0        = BOARD_SERCOM2_PINMAP_PAD0,
  .pad1        = BOARD_SERCOM2_PINMAP_PAD1,
  .pad2        = BOARD_SERCOM2_PINMAP_PAD2,
  .pad3        = BOARD_SERCOM2_PINMAP_PAD3,
  .muxconfig   = BOARD_SERCOM2_MUXCONFIG,
  .srcfreq     = BOARD_SERCOM2_FREQUENCY,
  .base        = SAM_SERCOM2_BASE,
  .spilock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_SAMD2L2_SPI_DMA
  .dma_tx_trig = DMAC_TRIGSRC_SERCOM2_TX,
  .dma_rx_trig = DMAC_TRIGSRC_SERCOM2_RX,
#endif
};
#endif

#ifdef SAMD2L2_HAVE_SPI3
/* SPI3 driver operations */

static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
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
  .ops         = &g_spi3ops,
  .sercom      = 3,
#if 0 /* Not used */
  .irq         = SAM_IRQ_SERCOM3,
#endif
  .gclkgen     = BOARD_SERCOM3_GCLKGEN,
  .slowgen     = BOARD_SERCOM3_SLOW_GCLKGEN,
  .pad0        = BOARD_SERCOM3_PINMAP_PAD0,
  .pad1        = BOARD_SERCOM3_PINMAP_PAD1,
  .pad2        = BOARD_SERCOM3_PINMAP_PAD2,
  .pad3        = BOARD_SERCOM3_PINMAP_PAD3,
  .muxconfig   = BOARD_SERCOM3_MUXCONFIG,
  .srcfreq     = BOARD_SERCOM3_FREQUENCY,
  .base        = SAM_SERCOM3_BASE,
  .spilock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_SAMD2L2_SPI_DMA
  .dma_tx_trig = DMAC_TRIGSRC_SERCOM3_TX,
  .dma_rx_trig = DMAC_TRIGSRC_SERCOM3_RX,
#endif
};
#endif

#ifdef SAMD2L2_HAVE_SPI4
/* SPI4 driver operations */

static const struct spi_ops_s g_spi4ops =
{
  .lock              = spi_lock,
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
  .ops         = &g_spi4ops,
  .sercom      = 4,
#if 0 /* Not used */
  .irq         = SAM_IRQ_SERCOM4,
#endif
  .gclkgen     = BOARD_SERCOM4_GCLKGEN,
  .slowgen     = BOARD_SERCOM4_SLOW_GCLKGEN,
  .pad0        = BOARD_SERCOM4_PINMAP_PAD0,
  .pad1        = BOARD_SERCOM4_PINMAP_PAD1,
  .pad2        = BOARD_SERCOM4_PINMAP_PAD2,
  .pad3        = BOARD_SERCOM4_PINMAP_PAD3,
  .muxconfig   = BOARD_SERCOM4_MUXCONFIG,
  .srcfreq     = BOARD_SERCOM4_FREQUENCY,
  .base        = SAM_SERCOM4_BASE,
  .spilock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_SAMD2L2_SPI_DMA
  .dma_tx_trig = DMAC_TRIGSRC_SERCOM4_TX,
  .dma_rx_trig = DMAC_TRIGSRC_SERCOM4_RX,
#endif
};
#endif

#ifdef SAMD2L2_HAVE_SPI5
/* SPI5 driver operations */

static const struct spi_ops_s g_spi5ops =
{
  .lock              = spi_lock,
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
  .ops         = &g_spi5ops,
  .sercom      = 5,
#if 0 /* Not used */
  .irq         = SAM_IRQ_SERCOM5,
#endif
  .gclkgen     = BOARD_SERCOM5_GCLKGEN,
  .slowgen     = BOARD_SERCOM5_SLOW_GCLKGEN,
  .pad0        = BOARD_SERCOM5_PINMAP_PAD0,
  .pad1        = BOARD_SERCOM5_PINMAP_PAD1,
  .pad2        = BOARD_SERCOM5_PINMAP_PAD2,
  .pad3        = BOARD_SERCOM5_PINMAP_PAD3,
  .muxconfig   = BOARD_SERCOM5_MUXCONFIG,
  .srcfreq     = BOARD_SERCOM5_FREQUENCY,
  .base        = SAM_SERCOM5_BASE,
  .spilock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_SAMD2L2_SPI_DMA
  .dma_tx_trig = DMAC_TRIGSRC_SERCOM5_TX,
  .dma_rx_trig = DMAC_TRIGSRC_SERCOM5_RX,
#endif
};
#endif

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

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
static bool spi_checkreg(struct sam_spidev_s *priv, bool wr, uint32_t regval,
                         uint32_t regaddr)
{
  if (wr      == priv->wr &&      /* Same kind of access? */
      regval  == priv->regval &&  /* Same value? */
      regaddr == priv->regaddr)   /* Same address? */
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

          spiinfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wr      = wr;
      priv->regval  = regval;
      priv->regaddr = regaddr;
      priv->ntimes  = 0;
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

static uint8_t spi_getreg8(struct sam_spidev_s *priv, unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;
  uint8_t regval    = getreg8(regaddr);

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
  if (spi_checkreg(priv, false, (uint32_t)regval, regaddr))
    {
      spiinfo("%08x->%02x\n", regaddr, regval);
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

static void spi_putreg8(struct sam_spidev_s *priv, uint8_t regval,
                        unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
  if (spi_checkreg(priv, true, (uint32_t)regval, regaddr))
    {
      spiinfo("%08x<-%02x\n", regaddr, regval);
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

static uint16_t spi_getreg16(struct sam_spidev_s *priv, unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;
  uint16_t regval   = getreg16(regaddr);

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
  if (spi_checkreg(priv, false, (uint32_t)regval, regaddr))
    {
      spiinfo("%08x->%04x\n", regaddr, regval);
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

static void spi_putreg16(struct sam_spidev_s *priv, uint16_t regval,
                         unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
  if (spi_checkreg(priv, true, (uint32_t)regval, regaddr))
    {
      spiinfo("%08x<-%04x\n", regaddr, regval);
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

static uint32_t spi_getreg32(struct sam_spidev_s *priv, unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;
  uint32_t regval = getreg32(regaddr);

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
  if (spi_checkreg(priv, false, regval, regaddr))
    {
      spiinfo("%08x->%08x\n", regaddr, regval);
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

static void spi_putreg32(struct sam_spidev_s *priv, uint32_t regval,
                         unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;

#ifdef CONFIG_SAMD2L2_SPI_REGDEBUG
  if (spi_checkreg(priv, true, regval, regaddr))
    {
      spiinfo("%08x<-%08x\n", regaddr, regval);
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

#ifdef CONFIG_DEBUG_SPI_INFO
static void spi_dumpregs(struct sam_spidev_s *priv, const char *msg)
{
  spiinfo("%s:\n", msg);
  spiinfo("   CTRLA:%08x CTRLB:%08x DBGCTRL:%02x\n",
          getreg32(priv->base + SAM_SPI_CTRLA_OFFSET),
          getreg32(priv->base + SAM_SPI_CTRLB_OFFSET),
          getreg8(priv->base + SAM_SPI_DBGCTRL_OFFSET));
  spiinfo("    BAUD:%02x       INTEN:%02x       INTFLAG:%02x\n",
          getreg8(priv->base + SAM_SPI_BAUD_OFFSET),
          getreg8(priv->base + SAM_SPI_INTENCLR_OFFSET),
          getreg8(priv->base + SAM_SPI_INTFLAG_OFFSET));
  spiinfo("  STATUS:%04x      ADDR:%08x\n",
          getreg16(priv->base + SAM_SPI_STATUS_OFFSET),
          getreg32(priv->base + SAM_SPI_ADDR_OFFSET));
}
#endif

/****************************************************************************
 * Name: spi_interrupt
 *
 * Description:
 *   This is the SPI interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq' indicating either that the DATA
 *   register is available for the next transmission (DRE) or that the
 *   DATA register holds a new incoming work.
 *
 ****************************************************************************/

#if 0 /* Not used */
static int spi_interrupt(int irq, void *context, void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg
  uint8_t pending;
  uint8_t intflag;
  uint8_t inten;

  DEBUGASSERT(priv != NULL);

  /* Get the set of pending SPI interrupts (we are only interested in the
   * unmasked interrupts).
   */

  intflag = sam_getreg8(priv, SAM_SPI_INTFLAG_OFFSET);
  inten   = sam_getreg8(priv, SAM_SPI_INTENCLR_OFFSET);
  pending  = intflag & inten;

  /* Handle an incoming, receive byte.  The RXC flag is set when there is
   * unread data in DATA register.  This flag is cleared by reading the DATA
   * register (or by disabling the receiver).
   */

  if ((pending & SPI_INT_RXC) != 0)
    {
      /* Received data ready... process incoming SPI ata */
#warning Missing logic
    }

  /* Handle outgoing, transmit bytes. The DRE flag is set when the DATA
   * register is empty and ready to be written.  This flag is cleared by
   * writing new data to the DATA register.  If there is no further data to
   * be transmitted,  the serial driver will disable TX interrupts, prohibit
   * further interrupts until TX interrupts are re-enabled.
   */

  if ((pending & SPI_INT_DRE) != 0)
    {
      /* Transmit data register empty ... process outgoing bytes */
#warning Missing logic
    }

  return OK;
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
 *   configured for the device.  If the SPI bus is being shared, then it
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

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)dev;
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxmutex_lock(&priv->spilock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->spilock);
    }

  return ret;
}

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
  struct sam_spidev_s *priv = (struct sam_spidev_s *)dev;
  uint32_t maxfreq;
  uint32_t actual;
  uint32_t baud;
  uint32_t ctrla;

  spiinfo("sercom=%d frequency=%d\n", priv->sercom, frequency);

  /* Check if the configured BAUD is within the valid range */

  maxfreq = (priv->srcfreq >> 1);
  if (frequency > maxfreq)
    {
      /* Set the frequency to the maximum */

      spierr("ERROR: Cannot realize frequency: %ld\n", (long)frequency);
      frequency = maxfreq;
    }

  /* Check if the requested frequency is the same as the frequency
   * selection.
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* For synchronous mode, the BAUD rate (Fbaud) is generated from the
   * source clock frequency (Fref) as follows:
   *
   *   Fbaud = Fref / (2 * (BAUD + 1))
   *
   * Or
   *
   *   BAUD = (Fref / (2 * Fbaud)) - 1
   *
   * Where BAUD <= 255
   */

  baud = ((priv->srcfreq + frequency) / (frequency << 1)) - 1;

  /* Verify that the resulting if BAUD divisor is within range */

  if (baud > 255)
    {
      spierr("ERROR: BAUD is out of range: %ld\n", (long)baud);
      baud = 255;
    }

  /* Momentarily disable SPI while we apply the new BAUD setting (if it was
   * previously enabled)
   */

  ctrla = spi_getreg32(priv, SAM_SPI_CTRLA_OFFSET);
  if ((ctrla & SPI_CTRLA_ENABLE) != 0)
    {
      /* Disable SPI.. waiting for synchronization */

      spi_putreg32(priv, ctrla & ~SPI_CTRLA_ENABLE, SAM_SPI_CTRLA_OFFSET);
      spi_wait_synchronization(priv);

      /* Set the new BAUD value */

      spi_putreg8(priv, (uint8_t)baud, SAM_SPI_BAUD_OFFSET);

      /* Re-enable SPI.. waiting for synchronization */

      spi_putreg32(priv, ctrla, SAM_SPI_CTRLA_OFFSET);
      spi_wait_synchronization(priv);
    }
  else
    {
      /* Set the new BAUD when the SPI is already disabled */

      spi_putreg8(priv, (uint8_t)baud, SAM_SPI_BAUD_OFFSET);
    }

  /* Calculate the new actual frequency */

  actual = priv->srcfreq / ((baud + 1) << 1);

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %d->%d\n", frequency, actual);
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

  spiinfo("sercom=%d mode=%d\n", priv->sercom, mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set the mode appropriately */

      /* First we need to disable SPI while we change the mode */

      regval = spi_getreg32(priv, SAM_SPI_CTRLA_OFFSET);
      spi_putreg32(priv, regval & ~SPI_CTRLA_ENABLE, SAM_SPI_CTRLA_OFFSET);
      spi_wait_synchronization(priv);

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

      spi_putreg32(priv, regval | SPI_CTRLA_ENABLE, SAM_SPI_CTRLA_OFFSET);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
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

  DEBUGASSERT(priv != NULL);
  spiinfo("sercom=%d nbits=%d\n", priv->sercom, nbits);
  DEBUGASSERT(nbits > 7 && nbits < 10);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set number of bits appropriately */

      regval  = spi_getreg32(priv, SAM_SPI_CTRLB_OFFSET);
      regval &= ~SPI_CTRLB_CHSIZE_MASK;

      if (nbits == 9)
        {
          regval |= SPI_CTRLB_CHSIZE_9BITS;
        }

      spi_putreg32(priv, regval, SAM_SPI_CTRLB_OFFSET);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
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

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
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

  spiinfo("Sent %02x received %02x\n", txbyte, rxbyte);
  return (uint32_t)rxbyte;
}

/****************************************************************************
 * Name: spi_dma_callback
 *
 * Description:
 *   DMA completion callback
 *
 * Input Parameters:
 *   dma    - Allocate DMA handle
 *   arg    - User argument provided with callback
 *   result - The result of the DMA operation
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_SPI_DMA
static void spi_dma_callback(DMA_HANDLE dma, void *arg, int result)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)arg;

  if (dma == priv->dma_rx)
    {
      /* Notify the blocked spi_exchange() call that the transaction
       * has completed by posting to the semaphore
       */

      nxsem_post(&priv->dmasem);
    }
  else if (dma == priv->dma_tx)
    {
      if (result != OK)
        {
          spierr("ERROR: DMA transmission failed: %d\n", result);
        }
    }
}
#endif

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   Data must be 16-bit aligned in 9-bit data transfer mode.
 *
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)dev;

#ifdef CONFIG_SAMD2L2_SPI_DMA
  uint32_t regval;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Disable SPI while we configure new DMA descriptors */

  regval  = spi_getreg32(priv, SAM_SPI_CTRLA_OFFSET);
  regval &= ~SPI_CTRLA_ENABLE;
  spi_putreg32(priv, regval, SAM_SPI_CTRLA_OFFSET);
  spi_wait_synchronization(priv);

  /* Setup RX and TX DMA channels */

  sam_dmatxsetup(priv->dma_tx, priv->base + SAM_SPI_DATA_OFFSET,
                 (uint32_t)txbuffer, nwords);
  sam_dmarxsetup(priv->dma_rx, priv->base + SAM_SPI_DATA_OFFSET,
                 (uint32_t)rxbuffer, nwords);

  /* Start RX and TX DMA channels */

  sam_dmastart(priv->dma_tx, spi_dma_callback, (void *)priv);
  sam_dmastart(priv->dma_rx, spi_dma_callback, (void *)priv);

  /* Enable SPI to trigger the TX DMA channel */

  regval  = spi_getreg32(priv, SAM_SPI_CTRLA_OFFSET);
  regval |= SPI_CTRLA_ENABLE;
  spi_putreg32(priv, regval, SAM_SPI_CTRLA_OFFSET);
  spi_wait_synchronization(priv);

  /* Wait for the DMA callback to notify us that the transfer is complete */

  nxsem_wait_uninterruptible(&priv->dmasem);
#else
  const uint16_t *ptx16;
  const uint8_t *ptx8;
  uint16_t *prx16;
  uint8_t *prx8;
  uint16_t data;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Set up data receive and transmit pointers */

  if (priv->nbits > 8)
    {
      ptx8  = NULL;
      prx8  = NULL;
      ptx16 = (const uint16_t *)txbuffer;
      prx16 = (uint16_t *)rxbuffer;
    }
  else
    {
      ptx8  = (const uint8_t *)txbuffer;
      prx8  = (uint8_t *)rxbuffer;
      ptx16 = NULL;
      prx16 = NULL;
    }

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
   *   for (; nwords > 0; nwords--)
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

  for (; nwords > 0; nwords--)
    {
      /* Get the data to send (0xff if there is no data source) */

      if (ptx8)
        {
          data = (uint16_t)*ptx8++;
        }
      else if (ptx16)
        {
          data = *ptx16++;
        }
      else
        {
          data = 0x01ff;
        }

      /* Wait for any previous data written to the DATA register to be
       * transferred to the serializer.
       */

      while ((spi_getreg8(priv, SAM_SPI_INTFLAG_OFFSET) & SPI_INT_DRE) == 0);

      /* Write the data to transmitted to the DATA Register (TDR) */

      spi_putreg16(priv, data, SAM_SPI_DATA_OFFSET);

      /* Wait for the read data to be available in the DATA register. */

      while ((spi_getreg8(priv, SAM_SPI_INTFLAG_OFFSET) & SPI_INT_RXC) == 0);

      /* Check for data overflow.  The BUFOVF bit provides the status of the
       * next DATA to be read.  On buffer overflow, the corresponding DATA
       * will be 0.
       */

      data = spi_getreg16(priv, SAM_SPI_STATUS_OFFSET);
      if ((data & SPI_STATUS_BUFOVF) != 0)
        {
          spierr("ERROR: Buffer overflow!\n");

          /* Clear the buffer overflow flag */

          spi_putreg16(priv, data, SAM_SPI_STATUS_OFFSET);
        }

      /* Read the received data from the SPI DATA Register..
       * TODO: The following only works if nbits <= 8.
       */

      data = spi_getreg16(priv, SAM_SPI_DATA_OFFSET);
      if (prx8)
        {
          *prx8++ = (uint8_t)data;
        }
      else if (prx16)
        {
          *prx16++ = (uint16_t)data;
        }
    }
#endif
}

/****************************************************************************
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
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
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
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
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
 * Name: spi_wait_synchronization
 *
 * Description:
 *   Wait until the SERCOM SPI reports that it is synchronized.
 *
 ****************************************************************************/

static void spi_wait_synchronization(struct sam_spidev_s *priv)
{
#if defined(CONFIG_ARCH_FAMILY_SAMD20)
  while ((spi_getreg16(priv, SAM_SPI_STATUS_OFFSET) &
         SPI_STATUS_SYNCBUSY) != 0);
#elif defined(CONFIG_ARCH_FAMILY_SAMD21) || defined(CONFIG_ARCH_FAMILY_SAML21)
  while ((spi_getreg16(priv, SAM_SPI_SYNCBUSY_OFFSET) &
         SPI_SYNCBUSY_ALL) != 0);
#endif
}

/****************************************************************************
 * Name: spi_pad_configure
 *
 * Description:
 *   Configure the SERCOM SPI pads.
 *
 ****************************************************************************/

static void spi_pad_configure(struct sam_spidev_s *priv)
{
  /* Configure SERCOM pads */

  if (priv->pad0 != 0)
    {
      sam_configport(priv->pad0);
    }

  if (priv->pad1 != 0)
    {
      sam_configport(priv->pad1);
    }

  if (priv->pad2 != 0)
    {
      sam_configport(priv->pad2);
    }

  if (priv->pad3 != 0)
    {
      sam_configport(priv->pad3);
    }
}

/****************************************************************************
 * Name: spi_dma_setup
 *
 * Description:
 *   Configure the SPI DMA operation.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_SPI_DMA
static void spi_dma_setup(struct sam_spidev_s *priv)
{
  /* Allocate a pair of DMA channels */

  priv->dma_rx = sam_dmachannel(DMACH_FLAG_BEATSIZE_BYTE |
                                DMACH_FLAG_MEM_INCREMENT |
                                DMACH_FLAG_PERIPH_RXTRIG(priv->dma_rx_trig));

  priv->dma_tx = sam_dmachannel(DMACH_FLAG_BEATSIZE_BYTE |
                                DMACH_FLAG_MEM_INCREMENT |
                                DMACH_FLAG_PERIPH_TXTRIG(priv->dma_tx_trig));

  /* Initialize the semaphore used to notify when DMA is complete */

  nxsem_init(&priv->dmasem, 0, 0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   port - SPI "port" number (i.e., SERCOM number)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *sam_spibus_initialize(int port)
{
  struct sam_spidev_s *priv;
  irqstate_t flags;
  uint32_t regval;
#ifdef CONFIG_ARCH_FAMILY_SAML21
  int channel;
#endif
#if 0 /* Not used */
  int ret;
#endif

  /* Get the port state structure */

  spiinfo("port: %d\n", port);

#ifdef SAMD2L2_HAVE_SPI0
  if (port == 0)
    {
      priv = &g_spi0dev;
    }
  else
#endif

#ifdef SAMD2L2_HAVE_SPI1
  if (port == 1)
    {
      priv = &g_spi1dev;
    }
  else
#endif

#ifdef SAMD2L2_HAVE_SPI2
  if (port == 2)
    {
      priv = &g_spi2dev;
    }
  else
#endif

#ifdef SAMD2L2_HAVE_SPI3
  if (port == 3)
    {
      priv = &g_spi3dev;
    }
  else
#endif

#ifdef SAMD2L2_HAVE_SPI4
  if (port == 4)
    {
      priv = &g_spi4dev;
    }
  else
#endif

#ifdef SAMD2L2_HAVE_SPI5
  if (port == 5)
    {
      priv = &g_spi5dev;
    }
  else
#endif
    {
      spierr("ERROR: Unsupported port: %d\n", port);
      return NULL;
    }

#ifdef CONFIG_SAMD2L2_SPI_DMA
  spi_dma_setup(priv);
#endif

  /* Enable clocking to the SERCOM module in PM */

  flags = enter_critical_section();
  sercom_enable(priv->sercom);

  /* Configure the GCLKs for the SERCOM module */

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)
  sercom_coreclk_configure(priv->sercom, priv->gclkgen, false);

#elif defined(CONFIG_ARCH_FAMILY_SAML21)
  if (priv->sercom == 5)
    {
      channel = GCLK_CHAN_SERCOM5_CORE;
    }
  else
    {
      channel = priv->sercom + GCLK_CHAN_SERCOM0_CORE;
    }

  sam_gclk_chan_enable(channel, config->gclkgen);
#endif

  sercom_slowclk_configure(priv->sercom, priv->slowgen);

  /* Set the SERCOM in SPI master mode (no address) */

  regval  = spi_getreg32(priv, SAM_SPI_CTRLA_OFFSET);
  regval &= ~(SPI_CTRLA_MODE_MASK | SPI_CTRLA_FORM_MASK);
  regval |= (SPI_CTRLA_MODE_MASTER | SPI_CTRLA_FORM_SPI);
  spi_putreg32(priv, regval, SAM_SPI_CTRLA_OFFSET);

  /* Configure pads */

  spi_pad_configure(priv);

  /* Set an initial baud value.  This will be changed by the upper-half
   * driver as soon as it starts.
   */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Set MSB first data order and the configured pad mux setting.
   * SPI mode 0 is assumed initially (CPOL=0 and CPHA=0).
   */

  regval &= ~(SPI_CTRLA_DOPO_MASK | SPI_CTRLA_DIPO_MASK);
  regval &= ~(SPI_CTRLA_CPHA | SPI_CTRLA_CPOL);
  regval |= (SPI_CTRLA_MSBFIRST | priv->muxconfig);
  spi_putreg32(priv, regval, SAM_SPI_CTRLA_OFFSET);

  /* Enable the receiver.  Note that 8-bit data width is assumed initially */

  regval = (SPI_CTRLB_RXEN | SPI_CTRLB_CHSIZE_8BITS);
  spi_putreg32(priv, regval, SAM_SPI_CTRLB_OFFSET);
  spi_wait_synchronization(priv);

  priv->nbits = 8;

  /* Enable SPI */

  regval  = spi_getreg32(priv, SAM_SPI_CTRLA_OFFSET);
  regval |= SPI_CTRLA_ENABLE;
  spi_putreg32(priv, regval, SAM_SPI_CTRLA_OFFSET);
  spi_wait_synchronization(priv);

  /* Disable all interrupts at the SPI source and clear all pending
   * status that we can.
   */

  spi_putreg8(priv, SPI_INT_ALL, SAM_SPI_INTENCLR_OFFSET);
  spi_putreg8(priv, SPI_INT_ALL, SAM_SPI_INTFLAG_OFFSET);
  spi_putreg16(priv, SPI_STATUS_CLRALL, SAM_SPI_STATUS_OFFSET);

#if 0 /* Not used */
  /* Attach and enable the SERCOM interrupt handler */

  ret = irq_attach(priv->irq, spi_interrupt, priv);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach interrupt: %d\n", irq);
      return NULL;
    }

  /* Enable SERCOM interrupts at the NVIC */

  up_enable_irq(priv->irq);
#endif

  spi_dumpregs(priv, "After initialization");
  leave_critical_section(flags);
  return (struct spi_dev_s *)priv;
}

#endif /* SAMD2L2_HAVE_SPI */
