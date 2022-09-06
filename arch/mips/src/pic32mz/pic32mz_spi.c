/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_spi.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "mips_internal.h"
#include "hardware/pic32mz_spi.h"
#include "hardware/pic32mz_pps.h"
#include "pic32mz_spi.h"
#include "pic32mz_dma.h"

#ifdef CONFIG_PIC32MZ_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All SPI peripherals are clocked by PBCLK2 */

#define BOARD_PBCLOCK BOARD_PBCLK2

#ifdef CONFIG_PIC32MZ_SPI_DMA

/* When SPI DMA is enabled, small DMA transfers will still be performed by
 * polling logic. But we need a threshold value to determine what is small.
 * That value is provided by CONFIG_PIC32MZ_SPI_DMATHRESHOLD.
 */

#  ifndef CONFIG_PIC32MZ_SPI_DMATHRESHOLD
#    define CONFIG_PIC32MZ_SPI_DMATHRESHOLD 4
#  endif

/* Default values for channels' priority */

#  ifndef CONFIG_PIC32MZ_SPI_DMA_RXPRIO
#    define CONFIG_PIC32MZ_SPI_DMA_RXPRIO 0
#  endif

#  ifndef CONFIG_PIC32MZ_SPI_DMA_TXPRIO
#    define CONFIG_PIC32MZ_SPI_DMA_TXPRIO 0
#  endif

/* DMA timeout. The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.
 */

#  define DMA_TIMEOUT_MS    (800)
#  define DMA_TIMEOUT_TICKS MSEC2TICK(DMA_TIMEOUT_MS)

#  ifdef CONFIG_PIC32MZ_SPI_DMADEBUG
#    define DMA_INITIAL      0
#    define DMA_AFTER_SETUP  1
#    define DMA_AFTER_START  2
#    define DMA_CALLBACK     3
#    define DMA_TIMEOUT      3
#    define DMA_END_TRANSFER 4
#    define DMA_NSAMPLES     5
#  endif

/* Default value for the DMA buffer size */

#  ifndef CONFIG_PIC32MZ_SPI_DMABUFFSIZE
#    define CONFIG_PIC32MZ_SPI_DMABUFFSIZE 256
#  endif

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the fixed (ROM-able) configuration of the SPI
 * peripheral.
 */

struct pic32mz_config_s
{
  uint32_t         base;       /* SPI register base address */
#if defined(CONFIG_PIC32MZ_SPI_INTERRUPTS) || defined (CONFIG_PIC32MZ_SPI_DMA)
  uint8_t          firq;       /* SPI fault interrupt number */
  uint8_t          rxirq;      /* SPI receive done interrupt number */
  uint8_t          txirq;      /* SPI transfer done interrupt number */
#endif
  uint8_t          sdipps;     /* SDI peripheral pin selection */
  uint8_t          sdopps;     /* SDO peripheral pin selection */
  uintptr_t        sdoreg;     /* SDO peripheral pin configuration register */
};

/* This structure describes the state of the SPI driver */

struct pic32mz_dev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  const struct pic32mz_config_s *config;
  mutex_t          lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */

#ifdef CONFIG_PIC32MZ_SPI_DMA
  DMA_HANDLE rxdma;            /* SPI RX DMA handle */
  DMA_HANDLE txdma;            /* SPI TX DMA handle */
  int result;                  /* DMA result */
  sem_t dmawait;               /* Used to wait for DMA completion */
  struct wdog_s dmadog;        /* Watchdog that handles DMA timeouts */
#endif

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
  bool             wrlast;     /* Last was a write */
  uint32_t         addrlast;   /* Last address */
  uint32_t         vallast;    /* Last value */
  int              ntimes;     /* Number of times */
#endif

#ifdef CONFIG_PIC32MZ_SPI_DMADEBUG
  struct pic32mz_dmaregs_s rxdmaregs[DMA_NSAMPLES];
  struct pic32mz_dmaregs_s txdmaregs[DMA_NSAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level register access */

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static bool     spi_checkreg(struct pic32mz_dev_s *priv,
                  uintptr_t regaddr, uint32_t regvaql, bool wr);
static uint32_t spi_getreg(struct pic32mz_dev_s *priv,
                  unsigned int offset);
static void     spi_putaddr(struct pic32mz_dev_s *priv,
                  uintptr_t regaddr, uint32_t value);
#else
static inline uint32_t spi_getreg(struct pic32mz_dev_s *priv,
                  unsigned int offset);
static inline void     spi_putaddr(struct pic32mz_dev_s *priv,
                  uintptr_t regaddr, uint32_t value);
#endif
static inline void spi_putreg(struct pic32mz_dev_s *priv,
                  unsigned int offset, uint32_t value);
static inline void spi_flush(struct pic32mz_dev_s *priv);

static void     spi_exchange16(struct pic32mz_dev_s *priv,
                   const uint16_t *txbuffer, uint16_t *rxbuffer,
                   size_t nwords);
static void     spi_exchange8(struct pic32mz_dev_s *priv,
                   const uint8_t *txbuffer, uint8_t *rxbuffer,
                   size_t nbytes);

/* DMA Support */

#ifdef CONFIG_PIC32MZ_SPI_DMA
#  ifdef CONFIG_PIC32MZ_SPI_DMADEBUG
#    define spi_rxdma_sample(s,i) pic32mz_dma_sample((s)->rxdma,\
                                                    &(s)->rxdmaregs[i])
#    define spi_txdma_sample(s,i) pic32mz_dma_sample((s)->txdma,\
                                                    &(s)->txdmaregs[i])
static void spi_dma_sampleinit(struct pic32mz_dev_s *priv);
static void spi_dma_sampledone(struct pic32mz_dev_s *priv);
#  else
#    define spi_rxdma_sample(s,i)
#    define spi_txdma_sample(s,i)
#    define spi_dma_sampleinit(s)
#    define spi_dma_sampledone(s)
#  endif
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t status, void *arg);
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t status, void *arg);
static void spi_dmatimeout(wdparm_t arg);
#endif

/* SPI methods */

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                   uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void     spi_exchange(struct spi_dev_s *dev,
                             const void *txbuffer, void *rxbuffer,
                             size_t nwords);
#ifdef CONFIG_PIC32MZ_SPI_DMA
static void     spi_exchange_nodma(struct spi_dev_s *dev,
                                   const void *txbuffer,
                                   void *rxbuffer,
                                   size_t nwords);
#endif

#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(struct spi_dev_s *dev,
                             const void *buffer, size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI1

static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                    /* Not supported */
#endif
  .status            = pic32mz_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi1register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi1config =
{
  .base              = PIC32MZ_SPI1_K1BASE,
#if defined(CONFIG_PIC32MZ_SPI_INTERRUPTS) || defined (CONFIG_PIC32MZ_SPI_DMA)
  .firq              = PIC32MZ_IRQ_SPI1F,
  .rxirq             = PIC32MZ_IRQ_SPI1RX,
  .txirq             = PIC32MZ_IRQ_SPI1TX,
#endif
  .sdipps            = BOARD_SDI1_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO1_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO1_PPS)
};

static struct pic32mz_dev_s g_spi1dev =
{
  .spidev            =
  {
    &g_spi1ops
  },
  .config            = &g_spi1config
};
#endif

#ifdef CONFIG_PIC32MZ_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi2cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi2register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi2config =
{
  .base              = PIC32MZ_SPI2_K1BASE,
#if defined(CONFIG_PIC32MZ_SPI_INTERRUPTS) || defined (CONFIG_PIC32MZ_SPI_DMA)
  .firq              = PIC32MZ_IRQ_SPI2F,
  .rxirq             = PIC32MZ_IRQ_SPI2RX,
  .txirq             = PIC32MZ_IRQ_SPI2TX,
#endif
  .sdipps            = BOARD_SDI2_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO2_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO2_PPS),
};

static struct pic32mz_dev_s g_spi2dev =
{
  .spidev            =
  {
    &g_spi2ops
  },
  .config            = &g_spi2config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi3register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi3config =
{
  .base              = PIC32MZ_SPI3_K1BASE,
#if defined(CONFIG_PIC32MZ_SPI_INTERRUPTS) || defined (CONFIG_PIC32MZ_SPI_DMA)
  .firq              = PIC32MZ_IRQ_SPI3F,
  .rxirq             = PIC32MZ_IRQ_SPI3RX,
  .txirq             = PIC32MZ_IRQ_SPI3TX,
#endif
  .sdipps            = BOARD_SDI3_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO3_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO3_PPS),
};

static struct pic32mz_dev_s g_spi3dev =
{
  .spidev            =
  {
    &g_spi3ops
  },
  .config            = &g_spi3config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi4cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi4register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi4config =
{
  .base              = PIC32MZ_SPI4_K1BASE,
#if defined(CONFIG_PIC32MZ_SPI_INTERRUPTS) || defined (CONFIG_PIC32MZ_SPI_DMA)
  .firq              = PIC32MZ_IRQ_SPI4F,
  .rxirq             = PIC32MZ_IRQ_SPI4RX,
  .txirq             = PIC32MZ_IRQ_SPI4TX,
#endif
  .sdipps            = BOARD_SDI4_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO4_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO4_PPS),
};

static struct pic32mz_dev_s g_spi4dev =
{
  .spidev            =
  {
    &g_spi4ops
  },
  .config            = &g_spi4config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI5
static const struct spi_ops_s g_spi5ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi5cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi5register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi5config =
{
  .base              = PIC32MZ_SPI5_K1BASE,
#if defined(CONFIG_PIC32MZ_SPI_INTERRUPTS) || defined (CONFIG_PIC32MZ_SPI_DMA)
  .firq              = PIC32MZ_IRQ_SPI5F,
  .rxirq             = PIC32MZ_IRQ_SPI5RX,
  .txirq             = PIC32MZ_IRQ_SPI5TX,
#endif
  .sdipps            = BOARD_SDI5_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO5_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO5_PPS),
};

static struct pic32mz_dev_s g_spi5dev =
{
  .spidev            =
  {
    &g_spi5ops
  },
  .config            = &g_spi5config,
};
#endif

#ifdef CONFIG_PIC32MZ_SPI6
static const struct spi_ops_s g_spi6ops =
{
  .lock              = spi_lock,
  .select            = pic32mz_spi6select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mz_spi6status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mz_spi6cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mz_spi6register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static const struct pic32mz_config_s g_spi6config =
{
  .base              = PIC32MZ_SPI6_K1BASE,
#if defined(CONFIG_PIC32MZ_SPI_INTERRUPTS) || defined (CONFIG_PIC32MZ_SPI_DMA)
  .firq              = PIC32MZ_IRQ_SPI6F,
  .rxirq             = PIC32MZ_IRQ_SPI6RX,
  .txirq             = PIC32MZ_IRQ_SPI6TX,
#endif
  .sdipps            = BOARD_SDI6_PPS,
  .sdopps            = PPS_OUTPUT_REGVAL(BOARD_SDO6_PPS),
  .sdoreg            = PPS_OUTPUT_REGADDR(BOARD_SDO6_PPS),
};

static struct pic32mz_dev_s g_spi6dev =
{
  .spidev            =
  {
    &g_spi6ops
  },
  .config            = &g_spi6config,
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
 *   priv    - SPI driver instance
 *   regaddr - The address of the register to write to
 *   regval  - The value to be written
 *   wr      - True: write access
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   false: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static bool spi_checkreg(struct pic32mz_dev_s *priv, uintptr_t regaddr,
                         uint32_t regval, bool wr)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      regaddr == priv->addrlast)   /* Same address? */
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

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = regaddr;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

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

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static uint32_t spi_getreg(struct pic32mz_dev_s *priv,
                           unsigned int offset)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* Read the register value */

  regaddr = priv->config->base + offset;
  regval  = getreg32(regaddr);

  /* Should we print something? */

  if (spi_checkreg(priv, regaddr, regval, false))
    {
      /* Yes.. */

      spiinfo("%08lx->%08lx\n",
             (unsigned long)regaddr, (unsigned long)regval);
    }

  /* Return the value read */

  return regval;
}
#else
static inline uint32_t spi_getreg(struct pic32mz_dev_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->config->base + offset);
}
#endif

/****************************************************************************
 * Name: spi_putaddr
 *
 * Description:
 *   Write a 32-bit value value an absolute address
 *
 * Input Parameters:
 *   priv    - A pointer to a PIC32MZ SPI state structure
 *   regaddr - Address to write to
 *   regval  - The value to write to the SPI register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_REGDEBUG
static void spi_putaddr(struct pic32mz_dev_s *priv, uintptr_t regaddr,
                        uint32_t regval)
{
  /* Should we print something? */

  if (spi_checkreg(priv, regaddr, regval, true))
    {
      /* Yes.. */

      spiinfo("%08lx<-%08lx\n",
              (unsigned long)regaddr, (unsigned long)regval);
    }

  /* Write the value to the register */

  putreg32(regval, regaddr);
}
#else
static inline void spi_putaddr(struct pic32mz_dev_s *priv,
                               uintptr_t regaddr, uint32_t regval)
{
  /* Write the value to the register */

  putreg32(regval, regaddr);
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

static inline void spi_putreg(struct pic32mz_dev_s *priv,
                              unsigned int offset, uint32_t regval)
{
  spi_putaddr(priv, priv->config->base + offset, regval);
}

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Make sure that there are now dangling SPI transfer in progress
 *
 * Input Parameters:
 *   spi - SPI controller state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flush(struct pic32mz_dev_s *priv)
{
  /* Make sure that no TX activity is in progress... waiting if necessary */

  while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPITBF) != 0);

  /* Then make sure that there is no pending RX data... reading and
   * discarding as necessary.
   */

  while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) & SPI_STAT_SPIRBF) != 0)
    {
      spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
    }
}

/****************************************************************************
 * Name: spi_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_PIC32MZ_SPI_DMADEBUG)
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_DMADEBUG
static void spi_dma_sampleinit(struct pic32mz_dev_s *priv)
{
  /* Put contents of register samples into a known state */

  memset(priv->rxdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct pic32mz_dmaregs_s));
  memset(priv->txdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct pic32mz_dmaregs_s));

  /* Then get the initial samples */

  spi_rxdma_sample(priv, DMA_INITIAL);
  spi_txdma_sample(priv, DMA_INITIAL);
}
#endif

/****************************************************************************
 * Name: spi_dma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_DMADEBUG
static void spi_dma_sampledone(struct pic32mz_dev_s *priv)
{
  /* Sample the final registers */

  spi_rxdma_sample(priv, DMA_END_TRANSFER);
  spi_txdma_sample(priv, DMA_END_TRANSFER);

  /* Then dump the sampled DMA registers */

  /* Initial register values */

  pic32mz_dma_dump(priv->txdma, &priv->txdmaregs[DMA_INITIAL],
              "TX: Initial Registers");
  pic32mz_dma_dump(priv->rxdma, &priv->rxdmaregs[DMA_INITIAL],
              "RX: Initial Registers");

  /* Register values after DMA setup */

  pic32mz_dma_dump(priv->txdma, &priv->txdmaregs[DMA_AFTER_SETUP],
              "TX: After DMA Setup");
  pic32mz_dma_dump(priv->rxdma, &priv->rxdmaregs[DMA_AFTER_SETUP],
              "RX: After DMA Setup");

  /* Register values after DMA start */

  pic32mz_dma_dump(priv->txdma, &priv->txdmaregs[DMA_AFTER_START],
              "TX: After DMA Start");
  pic32mz_dma_dump(priv->rxdma, &priv->rxdmaregs[DMA_AFTER_START],
              "RX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timed out, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  pic32mz_dma_dump(priv->txdma, &priv->txdmaregs[DMA_CALLBACK],
                   "TX: At DMA callback");

  /* Register values at the end of the DMA */

  if (priv->result == -ETIMEDOUT)
    {
      pic32mz_dma_dump(priv->rxdma, &priv->rxdmaregs[DMA_TIMEOUT],
                       "RX: At DMA timeout");
    }
  else
    {
      pic32mz_dma_dump(priv->rxdma, &priv->rxdmaregs[DMA_CALLBACK],
                       "RX: At DMA callback");
    }

  pic32mz_dma_dump(priv->txdma, &priv->txdmaregs[DMA_END_TRANSFER],
                   "TX: At End-of-Transfer");
  pic32mz_dma_dump(priv->rxdma, &priv->rxdmaregs[DMA_END_TRANSFER],
                   "RX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SPI RX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   status - The status of the DMA transfer
 *   arg - A pointer to the dev structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->dmadog);

  /* Sample DMA registers at the time of the callback */

  spi_rxdma_sample(priv, DMA_CALLBACK);

  /* Report the result of the transfer */

  if (status & PIC32MZ_DMA_INT_ADDRERR)
    {
      priv->result = -EFAULT;
    }
  else if (status & PIC32MZ_DMA_INT_ABORT)
    {
      priv->result = -ECONNABORTED;
    }
  else if (status & PIC32MZ_DMA_INT_BLOCKDONE)
    {
      priv->result = OK;
    }
  else
    {
      priv->result = -EIO;
    }

  /* Then wake up the waiting thread */

  nxsem_post(&priv->dmawait);
}
#endif /* CONFIG_PIC32MZ_SPI_DMA */

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SPI TX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   status - The status of the DMA transfer
 *   arg - A pointer to the dev structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->dmadog);

  /* Sample DMA registers at the time of the callback */

  spi_txdma_sample(priv, DMA_CALLBACK);

  /* Report the result of the transfer */

  if (status & PIC32MZ_DMA_INT_ADDRERR)
    {
      priv->result = -EFAULT;
    }
  else if (status & PIC32MZ_DMA_INT_ABORT)
    {
      priv->result = -ECONNABORTED;
    }
  else if (status & PIC32MZ_DMA_INT_BLOCKDONE)
    {
      priv->result = OK;
    }
  else
    {
      priv->result = -EIO;
    }

  /* Then wake up the waiting thread */

  nxsem_post(&priv->dmawait);
}
#endif /* CONFIG_PIC32MZ_SPI_DMA */

/****************************************************************************
 * Name: spi_dmatimeout
 *
 * Description:
 *   The watchdog timeout has expired without the completion of the DMA
 *   transfer.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_DMA
static void spi_dmatimeout(wdparm_t arg)
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  spi_rxdma_sample(priv, DMA_CALLBACK);

  /* Report timeout result, perhaps overwriting any failure reports from
   * the TX callback.
   */

  priv->result = -ETIMEDOUT;

  /* Then wake up the waiting thread */

  nxsem_post(&priv->dmawait);
}
#endif /* CONFIG_PIC32MZ_SPI_DMA */

/****************************************************************************
 * Name: spi_exchange8
 *
 * Description:
 *   Exchange a block of 8-bit data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nbytes   - the length of bytes that to be exchanged
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange8(struct pic32mz_dev_s *priv,
                          const uint8_t *txbuffer, uint8_t *rxbuffer,
                          size_t nbytes)
{
  uint32_t regval;
  uint8_t data;

  spiinfo("nbytes: %d\n", nbytes);

  while (nbytes)
    {
      /* Write the data to be transmitted to the SPI Data Register */

      if (txbuffer)
        {
          data = *txbuffer++;
        }
      else
        {
          data = 0xaa;
        }

      spi_putreg(priv, PIC32MZ_SPI_BUF_OFFSET, (uint32_t)data);

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0.
       * In enhanced buffer mode, the SPIRBE bit will be cleared in  when the
       * receive buffer is not empty.
       */

      while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1.
       * In normal mode, the SPIRBF bit will be set when receive data is
       * available.
       */

      while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBF) == 0);
#endif

      /* Read from the buffer register to clear the status bit */

      regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
      if (rxbuffer)
        {
          *rxbuffer++ = (uint8_t)regval;
        }
      else
        {
          UNUSED(regval);
        }

      nbytes--;
    }
}

/****************************************************************************
 * Name: spi_exchange16
 *
 * Description:
 *   Exchange a block of 16-bit data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange16(struct pic32mz_dev_s *priv,
                           const uint16_t *txbuffer,
                           uint16_t *rxbuffer, size_t nwords)
{
  uint32_t regval;
  uint16_t data;

  spiinfo("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write the data to transmitted to the SPI Data Register */

      if (txbuffer)
        {
          data = *txbuffer++;
        }
      else
        {
          data = 0xaaaa;
        }

      spi_putreg(priv, PIC32MZ_SPI_BUF_OFFSET, (uint32_t)data);

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0.
       * In enhanced buffer mode, the SPIRBE bit will be cleared in  when the
       * receive buffer is not empty.
       */

      while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1.
       * In normal mode, the SPIRBF bit will be set when receive data is
       * available.
       */

      while ((spi_getreg(priv, PIC32MZ_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBF) == 0);
#endif

      /* Read from the buffer register to clear the status bit */

      regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);
      if (rxbuffer)
        {
          *rxbuffer++ = (uint16_t)regval;
        }
      else
        {
          UNUSED(regval);
        }

      nwords--;
    }
}

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
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
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

static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)dev;
  uint32_t divisor;
  uint32_t actual;
  uint32_t regval;

  /* Check if the requested frequency is the same as the frequency
   * selection
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

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
  /* Calculate the new actual frequency.
   *
   * frequency = BOARD_PBCLOCK / (2 * divisor)
   */

  actual = (BOARD_PBCLOCK / 2) / divisor;

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

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
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)dev;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
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
       *   CPOL=1: The inactive value of the clock is one
       *   CPHA=0: Data is captured on the clock's inactive-to-active edge
       *           and data is propagated on a active-to-inactive edge.
       *   CPHA=1: Data is captured on the clock's active-to-inactive edge
       *           and data is propagated on a active-to-inactive edge.
       *
       * CON Register mapping:
       *   CPOL=0 corresponds to CON:CKP=0; CPOL=1 corresponds to CON:CKP=1
       *   CPHA=0 corresponds to CON:CKE=1; CPHA=1 corresponds to CON:CKE=0
       *
       * In addition, the CON register supports SMP: SPI Data Input Sample
       * Phase bit:
       *
       *    1 = Input data sampled at end of data output time
       *    0 = Input data sampled at middle of data output time
       *
       * Which is hardcoded to 1.
       */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 => CKP=0; CKE=1 */
          spi_putreg(priv, PIC32MZ_SPI_CONCLR_OFFSET, SPI_CON_CKP);
          spi_putreg(priv, PIC32MZ_SPI_CONSET_OFFSET, SPI_CON_CKE);
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 => CKP=0; CKE=0 */
          spi_putreg(priv, PIC32MZ_SPI_CONCLR_OFFSET, SPI_CON_CKP);
          spi_putreg(priv, PIC32MZ_SPI_CONCLR_OFFSET, SPI_CON_CKE);
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 => CKP=1; CKE=1 */
          spi_putreg(priv, PIC32MZ_SPI_CONSET_OFFSET, SPI_CON_CKP);
          spi_putreg(priv, PIC32MZ_SPI_CONSET_OFFSET, SPI_CON_CKE);
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 => CKP=1; CKE=0 */
          spi_putreg(priv, PIC32MZ_SPI_CONSET_OFFSET, SPI_CON_CKP);
          spi_putreg(priv, PIC32MZ_SPI_CONCLR_OFFSET, SPI_CON_CKE);
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

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
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)dev;
  uint32_t setting;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 7 && nbits < 33);
  if (nbits != priv->nbits)
    {
      /* Yes... Set the CON register appropriately */

      if (nbits == 8)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else if (nbits == 16)
        {
          setting = SPI_CON_MODE_16BIT;
        }
      else if (nbits == 32)
        {
          setting = SPI_CON_MODE_32BIT;
        }
      else
        {
          spierr("ERROR: Unsupported nbits: %d\n", nbits);
          return;
        }

      regval  = spi_getreg(priv, PIC32MZ_SPI_CON_OFFSET);
      regval &= ~SPI_CON_MODE_MASK;
      regval |= setting;
      spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, regval);

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
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)dev;

  DEBUGASSERT(priv);

  /* Make sure that any previous transfer is flushed from the hardware */

  spi_flush(priv);

  if (priv->nbits > 8)
    {
      uint16_t txword;
      uint16_t rxword;

      /* spi_exchange16() can do this. */

      txword = wd;
      rxword = 0;
      spi_exchange16(priv, &txword, &rxword, 1);

      spiinfo("Sent %04x received %04x\n", txword, rxword);
      return (uint32_t)rxword;
    }
  else
    {
      uint8_t txbyte;
      uint8_t rxbyte;

      /* spi_exchange8() can do this. */

      txbyte = (uint8_t)wd;
      rxbyte = (uint8_t)0;
      spi_exchange8(priv, &txbyte, &rxbyte, 1);

      spiinfo("Sent %02x received %02x\n", txbyte, rxbyte);
      return (uint32_t)rxbyte;
    }
}

/****************************************************************************
 * Name: spi_exchange (and spi_exchange_nodma)
 *
 * Description:
 *   Exchange a block of data from SPI. There are two versions of this
 *   function:
 *      1- The first is enabled when CONFIG_PIC32MZ_SPI_DMA=y and performs
 *      DMA SPI transfers, but only when a large block of data is bieng
 *      transferred.
 *      2- The second does polled SPI transfers, however it is also used for
 *      short SPI transfers when CONFIG_PIC32MZ_SPI_DMA=y
 *      If CONFIG_PIC32MZ_SPI_DMA=n, only the second version is available.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - The length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits > 8 and <= 16, the data is
 *              packed into uint16_t's; if nbits > 16 the data is packed
 *              into uint32_t's.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI_DMA
static void spi_exchange_nodma(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#else
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#endif
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)dev;

  DEBUGASSERT(priv);

  /* Make sure that any previous transfer is flushed from the hardware */

  spi_flush(priv);

  if (priv->nbits > 8)
    {
      /* spi_exchange16() can do this. */

      spi_exchange16(priv, (const uint16_t *)txbuffer,
                     (uint16_t *)rxbuffer, nwords);
    }
  else
    {
      /* spi_exchange8() can do this. */

      spi_exchange8(priv, (const uint8_t *)txbuffer,
                    (uint8_t *)rxbuffer, nwords);
    }
}

#ifdef CONFIG_PIC32MZ_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct pic32mz_dev_s *priv = (struct pic32mz_dev_s *)dev;
  struct pic32mz_dma_chcfg_s rxcfg;
  struct pic32mz_dma_chcfg_s txcfg;
  struct pic32mz_dma_xfrcfg_s rxxfr;
  struct pic32mz_dma_xfrcfg_s txxfr;
  uint8_t dummy[CONFIG_PIC32MZ_SPI_DMABUFFSIZE];
  uint32_t width;
  int ret;

  /* If we failed to allocate a DMA channel or this is a small transfer
   * then let spi_exchange_nodma() do the work.
   */

  if ((priv->rxdma == NULL) || (priv->txdma == NULL) ||
       nwords <= CONFIG_PIC32MZ_SPI_DMATHRESHOLD)
    {
      spiinfo("Non DMA transfer will be performed\n");
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Make sure that any previous transfer is flushed from the hardware */

  spi_flush(priv);

  /* Sample initial DMA registers */

  spi_dma_sampleinit(priv);

  /* Select the number of bytes and the width of a cell transfer */

  if (priv->nbits > 8)
    {
      /* 16-bit transfer (2 bytes) */

      width  = 2;
    }
  else
    {
      /* 8-bit transfer (1 byte) */

      width  = 1;
    }

  /* Configure the DMA channels. There are four different cases:
   *
   * 1) A true exchange with the memory address incrementing on both
   *    RX and TX channels,
   * 2) A read operation with the memory address incrementing only on
   *    the receive channel,
   * 3) A write operation where the memory address increments only on
   *    the receive channel, and
   * 4) A corner case where the memory address does not increment
   *    on either channel. This case might be used in certain cases
   *    where you want to assure that certain number of clocks are
   *    provided on the SPI bus.
   */

  /* Do we have an RX buffer? */

  if (rxbuffer)
    {
      /* Yes, setup the DMA transfer with the RX buffer.
       * We need a DMA ISR for:
       *  1 - Block done: The DMA transfer has completed succuessfully.
       *  2 - Address error: An address error occurred during the transfer.
       *  3 - Transfer abort: Abort event occurred. (i.e. SPI Error)
       */

      rxcfg.priority = CONFIG_PIC32MZ_SPI_DMA_RXPRIO;
      rxcfg.startirq = priv->config->rxirq;
      rxcfg.abortirq = priv->config->firq;
      rxcfg.event    = PIC32MZ_DMA_INT_BLOCKDONE | PIC32MZ_DMA_INT_ADDRERR |
                       PIC32MZ_DMA_INT_ABORT;
      rxcfg.mode     = PIC32MZ_DMA_MODE_BASIC;

      rxxfr.srcaddr  = priv->config->base + PIC32MZ_SPI_BUF_OFFSET;
      rxxfr.destaddr = (uint32_t)rxbuffer;
      rxxfr.srcsize  = width;
      rxxfr.destsize = nwords;
      rxxfr.cellsize = width;
    }
  else
    {
      /* No RX buffer. Setup the DMA transfer with a dummy RX buffer.
       *
       *  In this case we don't need ISR events, TX side will handle the
       *  transfer.
       */

      rxcfg.priority = CONFIG_PIC32MZ_SPI_DMA_RXPRIO;
      rxcfg.startirq = priv->config->rxirq;
      rxcfg.abortirq = PIC32MZ_DMA_NOIRQ;
      rxcfg.event    = PIC32MZ_DMA_INT_DISABLE;
      rxcfg.mode     = PIC32MZ_DMA_MODE_BASIC;

      rxxfr.srcaddr  = priv->config->base + PIC32MZ_SPI_BUF_OFFSET;
      rxxfr.destaddr = (uint32_t)dummy;
      rxxfr.srcsize  = width;
      rxxfr.destsize = nwords;
      rxxfr.cellsize = width;
    }

  /* Do we have a TX buffer? */

  if (txbuffer)
    {
      /* Yes, setup the DMA transfer with the TX buffer.
       * We need a DMA ISR for:
       *  1 - Block done: The DMA transfer has completed succuessfully.
       *  2 - Address error: An address error occurred during the transfer.
       *  3 - Transfer abort: Abort event occurred. (i.e. SPI Error)
       */

      txcfg.priority = CONFIG_PIC32MZ_SPI_DMA_TXPRIO;
      txcfg.startirq = priv->config->txirq;
      txcfg.abortirq = priv->config->firq;
      txcfg.event    = PIC32MZ_DMA_INT_BLOCKDONE | PIC32MZ_DMA_INT_ADDRERR |
                       PIC32MZ_DMA_INT_ABORT;
      txcfg.mode     = PIC32MZ_DMA_MODE_BASIC;

      txxfr.srcaddr  = (uint32_t)txbuffer;
      txxfr.destaddr = priv->config->base + PIC32MZ_SPI_BUF_OFFSET;
      txxfr.srcsize  = nwords;
      txxfr.destsize = width;
      txxfr.cellsize = width;
    }
  else
    {
      /* No TX buffer. Setup the DMA transfer with a dummy TX buffer.
       *
       *  In this case we don't need ISR events, RX side will handle the
       *  transfer.
       */

      txcfg.priority = CONFIG_PIC32MZ_SPI_DMA_TXPRIO;
      txcfg.startirq = priv->config->txirq;
      rxcfg.abortirq = PIC32MZ_DMA_NOIRQ;
      txcfg.event    = PIC32MZ_DMA_INT_DISABLE;
      txcfg.mode     = PIC32MZ_DMA_MODE_BASIC;

      txxfr.srcaddr  = (uint32_t)dummy;
      txxfr.destaddr = priv->config->base + PIC32MZ_SPI_BUF_OFFSET;
      txxfr.srcsize  = nwords;
      txxfr.destsize = width;
      txxfr.cellsize = width;
    }

  /* Configure the channels */

  ret = pic32mz_dma_chcfg(priv->rxdma, &rxcfg);
  if (ret < 0)
    {
      spierr("ERROR: RX transfer config failed: %d\n", ret);
      return;
    }

  ret = pic32mz_dma_chcfg(priv->txdma, &txcfg);
  if (ret < 0)
    {
      spierr("ERROR: RX transfer config failed: %d\n", ret);
      return;
    }

  /* Then setup the transfers */

  ret = pic32mz_dma_xfrsetup(priv->rxdma, &rxxfr);
  if (ret < 0)
    {
      spierr("ERROR: RX transfer setup failed: %d\n", ret);
      return;
    }

  spi_rxdma_sample(priv, DMA_AFTER_SETUP);

  ret = pic32mz_dma_xfrsetup(priv->txdma, &txxfr);
  if (ret < 0)
    {
      spierr("ERROR: TX transfer setup failed: %d\n", ret);
      return;
    }

  spi_txdma_sample(priv, DMA_AFTER_SETUP);

  /* Flush cache to physical memory */

  if (txbuffer)
    {
      up_flush_dcache((uintptr_t)txbuffer,
                      (uintptr_t)txbuffer + nwords);
    }
  else
    {
      up_flush_dcache((uintptr_t)dummy,
                      (uintptr_t)dummy + CONFIG_PIC32MZ_SPI_DMABUFFSIZE);
    }

  /* Start the DMA transfer */

  priv->result = -EBUSY;

  ret = pic32mz_dma_start(priv->rxdma, spi_dmarxcallback, (void *)priv);
  if (ret < 0)
    {
      spierr("ERROR: RX DMA start failed: %d\n", ret);
      return;
    }

  spi_rxdma_sample(priv, DMA_AFTER_START);

  ret = pic32mz_dma_start(priv->txdma, spi_dmatxcallback, (void *)priv);
  if (ret < 0)
    {
      spierr("ERROR: TX DMA start failed: %d\n", ret);
      return;
    }

  spi_txdma_sample(priv, DMA_AFTER_START);

  /* Wait for DMA completion. This is done in a loop because there may be
   * false alarm semaphore counts that cause nxsem_wait() not fail to wait
   * or to wake-up prematurely (for example due to the receipt of a signal).
   * We know that the DMA has completed when the result is anything other
   * that -EBUSY.
   */

  do
    {
      /* Start (or re-start) the watchdog timeout */

      ret = wd_start(&priv->dmadog, DMA_TIMEOUT_TICKS,
                     spi_dmatimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          spierr("ERROR: wd_start failed: %d\n", ret);
        }

      /* Wait for the DMA to complete */

      ret = nxsem_wait(&priv->dmawait);

      /* Cancel the watchdog timeout */

      wd_cancel(&priv->dmadog);

      /* Check if we were awakened by an error of some kind. EINTR is not a
       * failure. It simply means that the wait was awakened by a signal.
       */

      if (ret < 0 && ret != -EINTR)
        {
          DEBUGPANIC();
          return;
        }

      /* We might be awakened before the wait is over due to
       * residual counts on the semaphore.  So, to handle, that case,
       * we loop until something changes the DMA result to any value other
       * than -EBUSY.
       */
    }
  while (priv->result == -EBUSY);

  /* Dump the sampled DMA registers */

  spi_dma_sampledone(priv);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition).
   */

  pic32mz_dma_stop(priv->rxdma);
  pic32mz_dma_stop(priv->txdma);

  /* All we can do is complain if the DMA fails */

  if (priv->result < 0)
    {
      spierr("ERROR: DMA failed with result: %d\n", priv->result);
    }
  else
    {
      spiinfo("DMA Transfer success\n");
    }

  /* Force RAM re-read */

  if (rxbuffer)
    {
      up_invalidate_dcache((uintptr_t)rxbuffer,
                           (uintptr_t)rxbuffer + nwords);
    }
  else
    {
      up_invalidate_dcache((uintptr_t)dummy, (uintptr_t)dummy +
                           CONFIG_PIC32MZ_SPI_DMABUFFSIZE);
    }
}
#endif /* CONFIG_PIC32MZ_SPI_DMA */

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of
 *            words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface. If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits > 8 and <= 16, the
 *            data is packed into uint16_t's; if nbits > 16 the data is
 *            packed into uint32_t's.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  /* spi_exchange() can do this. */

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
 *   dev   -  Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in
 *            number of words. The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface. If nbits <= 8,
 *            the data is  packed into uint8_t's; if nbits > 8 and <= 16, the
 *            data is packed into uint16_t's; if nbits > 16 the data is
 *            packed into uint32_t's.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  /* spi_exchange() can do this. */

  spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *pic32mz_spibus_initialize(int port)
{
  struct pic32mz_dev_s *priv;
  uintptr_t regaddr;
  irqstate_t flags;
  uint32_t regval;
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  int ret;
#endif

  spiinfo("port: %d\n", port);

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
      priv    = &g_spi2dev;
      regaddr = PIC32MZ_SDI2R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI3
  if (port == 3)
    {
      priv    = &g_spi3dev;
      regaddr = PIC32MZ_SDI3R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI4
  if (port == 4)
    {
      priv    = &g_spi4dev;
      regaddr = PIC32MZ_SDI4R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI5
  if (port == 5)
    {
      priv    = &g_spi5dev;
      regaddr = PIC32MZ_SDI5R;
    }
  else
#endif
#ifdef CONFIG_PIC32MZ_SPI6
  if (port == 6)
    {
      priv    = &g_spi6dev;
      regaddr = PIC32MZ_SDI6R;
    }
  else
#endif
    {
      spierr("ERROR: Unsupported port: %d\n", port);
      return NULL;
    }

  /* Disable SPI interrupts */

  flags = enter_critical_section();
#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  up_disable_irq(priv->config->firq);
  up_disable_irq(priv->config->rxirq);
  up_disable_irq(priv->config->txirq);
#endif

  /* Stop and reset the SPI module by clearing the ON bit in the CON
   * register.
   */

  spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, 0);

  /* Clear the receive buffer by reading from the BUF register */

  regval = spi_getreg(priv, PIC32MZ_SPI_BUF_OFFSET);

  /* Configure SPI SDI (input) and SDO (output) pins. SS (output) pins are
   * managed as GPIOs; CLK (output) pins are not selectable.
   */

  spi_putaddr(priv, regaddr, (uint32_t)priv->config->sdipps);
  spi_putaddr(priv, priv->config->sdoreg, (uint32_t)priv->config->sdopps);

#ifdef CONFIG_PIC32MZ_SPI_DMA
  /* Allocate the RX and TX DMA channels, configuration will be done later. */

  priv->rxdma = pic32mz_dma_alloc(NULL);
  if (priv->rxdma == NULL)
    {
      spierr("ERROR: Failed to allocate the RX DMA channel\n");
    }

  priv->txdma = pic32mz_dma_alloc(NULL);
  if (priv->txdma == NULL)
    {
      spierr("ERROR: Failed to allocate the TX DMA channel\n");
    }

  /* Initialize the SPI semaphore. This semaphore is used for signaling and,
   * hence, should not have priority inheritance enabled.
   */

  nxsem_init(&priv->dmawait, 0, 0);
  nxsem_set_protocol(&priv->dmawait, SEM_PRIO_NONE);
#endif

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  /* Attach the interrupt handlers.  We do this early to make sure that the
   * resources are available.
   */

  ret = irq_attach(priv->config->rxirq, spi_interrupt, NULL);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach RX interrupt: %d port: %d\n",
             priv->config->rxirq, port);
      goto errout;
    }

  ret = irq_attach(priv->config->txirq, spi_interrupt, NULL);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach TX interrupt: %d port: %d\n",
             priv->tconfig->xirq, port);
      goto errout_with_rxirq;
    }

  ret = irq_attach(priv->config->firq, spi_interrupt, NULL);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach fault interrupt: %d port: %d\n",
             priv->config->firq, port);
      goto errout_with_txirq;
    }
#endif

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Clear the SPIROV overflow bit (SPIxSTAT:6). */

  spi_putreg(priv, PIC32MZ_SPI_STATCLR_OFFSET, SPI_STAT_SPIROV);

  /* Initial settings 8 bit +  master mode + mode 0.  NOTE that MSSEN
   * not set:  The slave select pin must be driven manually via the
   * board-specific pic32mz_spiNselect() interface.
   */

  regval = (SPI_CON_MSTEN | SPI_CON_SMP | SPI_CON_MODE_8BIT |
            SPI_CON_CKE | SPI_CON_ON);

  /* Set the ENHBUF bit if using Enhanced Buffer mode. */

#ifdef CONFIG_PIC32MZ_SPI_ENHBUF
  regval |= (SPI_CON_ENHBUF | SPI_CON_SRXISEL_HALF | SPI_CON_STXISEL_HALF);
#endif
  spi_putreg(priv, PIC32MZ_SPI_CON_OFFSET, regval);
  spiinfo("CON: %08" PRIx32 "\n", regval);

  /* Set the initial SPI configuration */

  priv->nbits = 8;
  priv->mode  = SPIDEV_MODE0;

  /* Initialize the SPI mutex that enforces mutually exclusive access */

  nxmutex_init(&priv->lock);

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
  /* Enable interrupts at the SPI controller */

  up_enable_irq(priv->config->firq);
  up_enable_irq(priv->config->rxirq);
  up_enable_irq(priv->config->txirq);
#endif

  /* Enable interrupts at the interrupt controller */

  leave_critical_section(flags);
  return &priv->spidev;

#ifdef CONFIG_PIC32MZ_SPI_INTERRUPTS
errout_with_txirq:
  irq_detatch(priv->config->txirq);
errout_with_rxirq:
  irq_detatch(priv->config->rxirq);
errout:
  leave_critical_section(flags);
  return NULL;
#endif
}

#endif /* CONFIG_PIC32MZ_SPI */
