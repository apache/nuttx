/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_spi.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <arch/board/board.h>
#include <arch/chip/pm.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"

#include "cxd56_spi.h"
#include "hardware/cxd56_spi.h"
#include "cxd56_clock.h"
#include "cxd56_pinconfig.h"
#include "cxd56_powermgr.h"

#ifdef CONFIG_CXD56_DMAC
#include "cxd56_dmac.h"
#endif

#ifdef CONFIG_CXD56_SPI

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SPI driver */

struct cxd56_spidev_s
{
  struct spi_dev_s spidev;      /* Externally visible part of the SPI interface */
  uint32_t         spibase;     /* SPIn base address */
  uint32_t         spibasefreq;
#ifdef CONFIG_CXD56_SPI_INTERRUPTS
  uint8_t          spiirq;      /* SPI IRQ number */
#endif
  mutex_t          lock;        /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;   /* Requested clock frequency */
  uint32_t         actual;      /* Actual clock frequency */
  uint8_t          nbits;       /* Width of word in bits (4 to 16) */
  uint8_t          mode;        /* Mode 0,1,2,3 */
  uint8_t          port;        /* Port number */
  int              initialized; /* Initialized flag */
#ifdef CONFIG_CXD56_DMAC
  bool             dmaenable;   /* Use DMA or not */
  DMA_HANDLE       rxdmach;     /* RX DMA channel handle */
  DMA_HANDLE       txdmach;     /* TX DMA channel handle */
  sem_t            dmasem;      /* Wait for DMA to complete */
  dma_config_t     rxconfig;    /* RX DMA configuration */
  dma_config_t     txconfig;    /* TX DMA configuration */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(struct cxd56_spidev_s *priv,
                                  uint8_t offset);
static inline void spi_putreg(struct cxd56_spidev_s *priv,
                              uint8_t offset, uint32_t value);

/* DMA support */

#ifdef CONFIG_CXD56_DMAC
static void unused_code spi_dmaexchange(struct spi_dev_s *dev,
                                        const void *txbuffer,
                                        void *rxbuffer, size_t nwords);
static void spi_dmatrxwait(struct cxd56_spidev_s *priv);
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t status, void *data);
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t status, void *data);
static void spi_dmatxsetup(struct cxd56_spidev_s *priv,
                           const void *txbuffer, size_t nwords);
static void spi_dmarxsetup(struct cxd56_spidev_s *priv,
                           const void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void spi_dmasndblock(struct spi_dev_s *dev,
                            const void *buffer, size_t nwords);
#endif
#endif

/* SPI methods */

static int spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency);
static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void unused_code spi_exchange(struct spi_dev_s *dev,
                                     const void *txbuffer,
                                     void *rxbuffer,
                                     size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords);
static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * SPI Channels mapped as follows:
 * 4 - IMG SPI
 * 5 - IMG WSPI
 * 0 - SPIM (SYSIOP)
 * 3 - SCU SPI
 ****************************************************************************/

#ifdef CONFIG_CXD56_SPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock              = spi_lock,
  .select            = cxd56_spi4select,   /* Provided externally */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                  /* Not supported */
#endif
  .status            = cxd56_spi4status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = cxd56_spi4cmddata,  /* Provided externally */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = cxd56_spi4register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct cxd56_spidev_s g_spi4dev =
{
  .spidev            =
  {
    .ops             = &g_spi4ops,
  },
  .spibase           = CXD56_IMG_SPI_BASE,
  .spibasefreq       = 0,
  .port              = 4,
  .initialized       = 0,
#ifdef CONFIG_CXD56_SPI_INTERRUPTS
  .spiirq            = CXD56_IRQ_IMG_SPI,
#endif
  .lock              = NXMUTEX_INITIALIZER,
#ifdef CONFIG_CXD56_DMAC
  .dmasem            = SEM_INITIALIZER(0),
#endif
};

#endif

#ifdef CONFIG_CXD56_SPI5
static const struct spi_ops_s g_spi5ops =
{
  .lock              = spi_lock,
  .select            = cxd56_spi5select,   /* Provided externally */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = cxd56_spi5status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = cxd56_spi5cmddata,  /* Provided externally */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = cxd56_spi5register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct cxd56_spidev_s g_spi5dev =
{
  .spidev            =
  {
    .ops             = &g_spi5ops,
  },
  .spibase           = CXD56_IMG_WSPI_BASE,
  .spibasefreq       = 0,
  .port              = 5,
  .initialized       = 0,
#ifdef CONFIG_CXD56_SPI_INTERRUPTS
  .spiirq            = CXD56_IRQ_IMG_WSPI,
#endif
  .lock              = NXMUTEX_INITIALIZER,
#ifdef CONFIG_CXD56_DMAC
  .dmasem            = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_CXD56_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = cxd56_spi0select,   /* Provided externally */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = cxd56_spi0status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = cxd56_spi0cmddata,  /* Provided externally */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = cxd56_spi0register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct cxd56_spidev_s g_spi0dev =
{
  .spidev            =
  {
    .ops             = &g_spi0ops,
  },
  .spibase           = CXD56_SPIM_BASE,
  .spibasefreq       = 0,
  .port              = 0,
  .initialized       = 0,
#ifdef CONFIG_CXD56_SPI_INTERRUPTS
  .spiirq            = CXD56_IRQ_SPIM,
#endif
  .lock              = NXMUTEX_INITIALIZER,
#ifdef CONFIG_CXD56_DMAC
  .dmasem            = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_CXD56_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
  .select            = cxd56_spi3select,   /* Provided externally */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = cxd56_spi3status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = cxd56_spi3cmddata,  /* Provided externally */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = cxd56_spi3register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct cxd56_spidev_s g_spi3dev =
{
  .spidev            =
  {
    .ops             = &g_spi3ops,
  },
  .spibase           = CXD56_SCU_SPI_BASE,
  .spibasefreq       = 0,
  .port              = 3,
  .initialized       = 0,
#ifdef CONFIG_CXD56_SPI_INTERRUPTS
  .spiirq            = CXD56_IRQ_SCU_SPI,
#endif
  .lock              = NXMUTEX_INITIALIZER,
#ifdef CONFIG_CXD56_DMAC
  .dmasem            = SEM_INITIALIZER(0),
#endif
};
#endif

/* Inhibit clock change */

static struct pm_cpu_freqlock_s g_hold_lock =
  PM_CPUFREQLOCK_INIT(0, PM_CPUFREQLOCK_FLAG_HOLD);

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
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct cxd56_spidev_s *priv,
                                  uint8_t offset)
{
  return getreg32(priv->spibase + (uint32_t)offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_putreg(struct cxd56_spidev_s *priv,
                              uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + (uint32_t)offset);
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
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;

  if (lock)
    {
      /* Take the mutex (perhaps waiting) */

      return nxmutex_lock(&priv->lock);
    }
  else
    {
      return nxmutex_unlock(&priv->lock);
    }
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
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;
  uint32_t divisor;
  uint32_t actual;

  /* Set SPI_CLOCK */

  cxd56_spi_clock_gear_adjust(priv->port, frequency);

  /* frequency = SPI_CLOCK / divisor, or divisor = SPI_CLOCK / frequency */

  priv->spibasefreq = cxd56_get_spi_baseclock(priv->port);
  divisor = priv->spibasefreq / frequency;

  /* "In master mode, CPSDVSRmin = 2 or larger (even numbers only)" */

  if (divisor < 2)
    {
      divisor = 2;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  divisor = (divisor + 1) & ~1;

  /* Disable clock gating (clock enable) */

  cxd56_spi_clock_gate_disable(priv->port);

  /* Save the new divisor value */

  spi_putreg(priv, CXD56_SPI_CPSR_OFFSET, divisor);

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(priv->port);

  /* Calculate the new actual */

  actual = priv->spibasefreq / divisor;

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, actual);
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
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;
  uint32_t regval;
  uint32_t cr1val;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR0 appropriately */

      /* Disable clock gating (clock enable) */

      cxd56_spi_clock_gate_disable(priv->port);

      regval = spi_getreg(priv, CXD56_SPI_CR0_OFFSET);
      regval &= ~(SPI_CR0_CPOL | SPI_CR0_CPHA);

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            break;

          case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            regval |= SPI_CR0_CPHA;
            break;

          case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            regval |= SPI_CR0_CPOL;
            break;

          case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            regval |= (SPI_CR0_CPOL | SPI_CR0_CPHA);
            break;

          default:
            spierr("Bad mode: %d\n", mode);
            DEBUGASSERT(FALSE);

            /* Enable clock gating (clock disable) */

            cxd56_spi_clock_gate_enable(priv->port);

            return;
        }

      /* Disable SSE */

      cr1val = spi_getreg(priv, CXD56_SPI_CR1_OFFSET);
      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, cr1val & ~SPI_CR1_SSE);

      spi_putreg(priv, CXD56_SPI_CR0_OFFSET, regval);

      /* Enable SSE after a few microseconds delay */

      up_udelay(3);

      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, cr1val);

      /* Enable clock gating (clock disable) */

      cxd56_spi_clock_gate_enable(priv->port);

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
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 3 && nbits < 17);

  if (nbits != priv->nbits)
    {
      /* Yes... Set CR0 appropriately */

      /* Disable clock gating (clock enable) */

      cxd56_spi_clock_gate_disable(priv->port);

      regval = spi_getreg(priv, CXD56_SPI_CR0_OFFSET);
      regval &= ~SPI_CR0_DSS_MASK;
      regval |= ((nbits - 1) << SPI_CR0_DSS_SHIFT);
      spi_putreg(priv, CXD56_SPI_CR0_OFFSET, regval);

      /* Enable clock gating (clock disable) */

      cxd56_spi_clock_gate_enable(priv->port);

      /* Save the selection so that re-configurations will be faster
       */

      priv->nbits = nbits;
#ifdef CONFIG_CXD56_DMAC
      if (priv->nbits > 8)
        {
          priv->txconfig.dest_width = CXD56_DMAC_WIDTH16;
          priv->txconfig.src_width = CXD56_DMAC_WIDTH16;
          priv->rxconfig.dest_width = CXD56_DMAC_WIDTH16;
          priv->rxconfig.src_width = CXD56_DMAC_WIDTH16;
        }
      else
        {
          priv->txconfig.dest_width = CXD56_DMAC_WIDTH8;
          priv->txconfig.src_width = CXD56_DMAC_WIDTH8;
          priv->rxconfig.dest_width = CXD56_DMAC_WIDTH8;
          priv->rxconfig.src_width = CXD56_DMAC_WIDTH8;
        }
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

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;
  register uint32_t regval;
  register uint32_t cr1val = 0;

  /* Prohibit the clock change during SPI transfer */

  up_pm_acquire_freqlock(&g_hold_lock);

  /* Disable clock gating (clock enable) */

  cxd56_spi_clock_gate_disable(priv->port);

  if (priv->port == 3)
    {
      /* Enable SPI HW */

      cr1val = spi_getreg(priv, CXD56_SPI_CR1_OFFSET);
      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, cr1val | SPI_CR1_SSE);
    }

  /* Wait while the TX FIFO is full */

  while (!(spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_TNF));

  /* Write the byte to the TX FIFO */

  spi_putreg(priv, CXD56_SPI_DR_OFFSET, wd);

  /* Wait for the RX FIFO not empty */

  while (!(spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_RNE));

  /* Get the value from the RX FIFO and return it */

  regval = spi_getreg(priv, CXD56_SPI_DR_OFFSET);
  spiinfo("%04" PRIx32 "->%04" PRIx32 "\n", wd, regval);

  if (priv->port == 3)
    {
      /* Restore SPI HW state */

      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, cr1val);
    }

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(priv->port);

  /* Allow the clock change after SPI transfer */

  up_pm_release_freqlock(&g_hold_lock);

  return regval;
}

/****************************************************************************
 * Name: spi_do_exchange
 *
 * Description:
 *   Exchange a block of data from SPI. Required.
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
 ****************************************************************************/

static void spi_do_exchange(struct spi_dev_s *dev,
                            const void *txbuffer, void *rxbuffer,
                            size_t nwords)
{
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;
  uint32_t regval                 = 0;

  union
  {
    const uint8_t *p8;
    const uint16_t *p16;
    const void *pv;
  } tx;

  union
  {
    uint8_t *p8;
    uint16_t *p16;
    void *pv;
  } rx;

  uint32_t data;
  uint32_t datadummy = (priv->nbits > 8) ? 0xffff : 0xff;
  uint32_t rxpending = 0;

  /* Remaining data to be sent (and no synchronization error has occurred) */

  tx.pv = txbuffer;
  rx.pv = rxbuffer;

  /* Prohibit the clock change during SPI transfer */

  up_pm_acquire_freqlock(&g_hold_lock);

  /* Disable clock gating (clock enable) */

  cxd56_spi_clock_gate_disable(priv->port);

  if (priv->port == 3)
    {
      /* Enable SPI HW */

      regval = spi_getreg(priv, CXD56_SPI_CR1_OFFSET);
      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, regval | SPI_CR1_SSE);
    }

  while (nwords || rxpending)
    {
      /* Write data to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      spiinfo("TX: rxpending: %" PRId32 " nwords: %d\n", rxpending, nwords);
      while ((spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_TNF) &&
             (rxpending < CXD56_SPI_FIFOSZ) && nwords)
        {
          if (txbuffer)
            {
              if (priv->nbits > 8)
                {
                  data = (uint32_t)*tx.p16++;
                }
              else
                {
                  data = (uint32_t)*tx.p8++;
                }
            }

          spi_putreg(priv, CXD56_SPI_DR_OFFSET, txbuffer ? data : datadummy);
          nwords--;
          rxpending++;
        }

      /* Now, read the RX data from the RX FIFO
       * while the RX FIFO is not empty
       */

      spiinfo("RX: rxpending: %" PRId32 "\n", rxpending);
      while (spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_RNE)
        {
          data = spi_getreg(priv, CXD56_SPI_DR_OFFSET);
          if (rxbuffer)
            {
              if (priv->nbits > 8)
                {
                  *rx.p16++ = (uint16_t)data;
                }
              else
                {
                  *rx.p8++ = (uint8_t)data;
                }
            }

          rxpending--;
        }
    }

  if (priv->port == 3)
    {
      /* Restore SPI HW state */

      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, regval);
    }

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(priv->port);

  /* Allow the clock change after SPI transfer */

  up_pm_release_freqlock(&g_hold_lock);
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Wrapper function of exchange a block of data from SPI.
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
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
#ifdef CONFIG_CXD56_DMAC
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;

#ifdef CONFIG_CXD56_SPI_DMATHRESHOLD
  size_t dmath = CONFIG_CXD56_SPI_DMATHRESHOLD;
#else
  size_t dmath = 0;
#endif

  if (priv->dmaenable && dmath < nwords)
    {
      spi_dmaexchange(dev, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      spi_do_exchange(dev, txbuffer, rxbuffer, nwords);
    }
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
  return spi_exchange(dev, buffer, NULL, nwords);
}

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
 *bits-per-word selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  return spi_exchange(dev, NULL, buffer, nwords);
}
#endif /* !CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: cxd56_spi_pincontrol
 *
 * Description:
 *   Configure the SPI pin
 *
 * Input Parameter:
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void cxd56_spi_pincontrol(int ch, bool on)
{
  switch (ch)
    {
#ifdef CONFIG_CXD56_SPI0
      case 0:
#ifdef CONFIG_CXD56_SPI0_PINMAP_DEBUG_UART
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0_GPIO);
          }
#endif

#ifdef CONFIG_CXD56_SPI0_PINMAP_SPIFLASH
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI1A_SPI0);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI1A_GPIO);
          }
#endif
        break;
#endif

#ifdef CONFIG_CXD56_SPI3
      case 3:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3_GPIO);
          }

#ifdef CONFIG_CXD56_SPI3_CS0
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3_CS0_X);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3_CS0_X_GPIO);
          }
#endif

#ifdef CONFIG_CXD56_SPI3_CS1
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3_CS1_X);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3_CS1_X_GPIO);
          }
#endif

#ifdef CONFIG_CXD56_SPI3_CS2
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3_CS2_X);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI3_CS2_X_GPIO);
          }
#endif
        break;
#endif

#ifdef CONFIG_CXD56_SPI4
      case 4:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4_GPIO);
          }
        break;
#endif

#ifdef CONFIG_CXD56_SPI5
      case 5:
#ifdef CONFIG_CXD56_SPI5_PINMAP_EMMC
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_GPIO);
          }
#endif

#ifdef CONFIG_CXD56_SPI5_PINMAP_SDIO
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
          }
#endif
        break;
#endif
      default:
        break;
    }
}

#ifdef CONFIG_CXD56_SPI4

/****************************************************************************
 * Name: spi4_colockchange
 *
 * Description:
 *   pm event callback for SPI4
 *
 * Input Parameter:
 *   id - PM callback ID
 *
 ****************************************************************************/

static int spi4_colockchange(uint8_t id)
{
  struct cxd56_spidev_s *priv = &g_spi4dev;

  switch (id)
    {
      case CXD56_PM_CALLBACK_ID_CLK_CHG_END:
        spi_setfrequency(&priv->spidev, priv->frequency);
        break;
      default:
        break;
    }

  return 0;
}

#endif

#ifdef CONFIG_CXD56_SPI5

/****************************************************************************
 * Name: spi5_colockchange
 *
 * Description:
 *   pm event callback for SPI5
 *
 * Input Parameter:
 *   id - PM callback ID
 *
 ****************************************************************************/

static int spi5_colockchange(uint8_t id)
{
  struct cxd56_spidev_s *priv = &g_spi5dev;

  switch (id)
    {
      case CXD56_PM_CALLBACK_ID_CLK_CHG_END:
        spi_setfrequency(&priv->spidev, priv->frequency);
        break;
      default:
        break;
    }

  return 0;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   port - Port number
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *cxd56_spibus_initialize(int port)
{
  struct cxd56_spidev_s *priv;
  uint32_t regval;
  int i;

  switch (port)
    {
#ifdef CONFIG_CXD56_SPI4
      case 4:
        priv = &g_spi4dev;
        if (!priv->initialized)
          {
            cxd56_pm_register_callback(PM_CLOCK_APP_SPI, spi4_colockchange);
          }
        break;
#endif

#ifdef CONFIG_CXD56_SPI5
      case 5:
        priv = &g_spi5dev;
        if (!priv->initialized)
          {
            cxd56_pm_register_callback(PM_CLOCK_APP_WSPI, spi5_colockchange);
          }
        break;
#endif

#ifdef CONFIG_CXD56_SPI0
      case 0:
        priv = &g_spi0dev;
        break;
#endif

#ifdef CONFIG_CXD56_SPI3
      case 3:
        priv = &g_spi3dev;
        break;
#endif
      default:
        return NULL;
    }

  /* If already initialized */

  if (priv->initialized)
    {
      return &priv->spidev;
    }

  /* Configure clocking */

  cxd56_spi_clock_enable(port);
  priv->spibasefreq = cxd56_get_spi_baseclock(port);

  /* DMA settings */

#ifdef CONFIG_CXD56_DMAC
  priv->dmaenable = false;
  priv->txdmach   = NULL;
  priv->rxdmach   = NULL;
#endif

  /* CS control */

  if ((port == 0) || (port == 3))
    {
      spi_putreg(priv, CXD56_SPI_CSMODE_OFFSET, 1);
      spi_putreg(priv, CXD56_SPI_CS_OFFSET, 1);
    }

  /* Configure pin */

  cxd56_spi_pincontrol(port, true);

  /* Configure 8-bit SPI mode */

  spi_putreg(priv, CXD56_SPI_CR0_OFFSET, SPI_CR0_DSS_8BIT | SPI_CR0_FRF_SPI);

  /* Disable SPI and all interrupts (we'll poll for all data) */

  spi_putreg(priv, CXD56_SPI_CR1_OFFSET, 0);
  spi_putreg(priv, CXD56_SPI_IMSC_OFFSET, 0);

  /* Clear interrupts */

  spi_putreg(priv, CXD56_SPI_ICR_OFFSET, 0x3);

  /* Set the initial SPI configuration */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

#ifdef CONFIG_CXD56_SPI3_SCUSEQ
  /* Enable the SPI, but not enable port 3 when SCU support enabled.
   * Because this enabler will be controlled by SCU.
   */

  if (port != 3)
    {
#endif
      regval = spi_getreg(priv, CXD56_SPI_CR1_OFFSET);
      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, regval | SPI_CR1_SSE);
#ifdef CONFIG_CXD56_SPI3_SCUSEQ
    }
#endif

  for (i = 0; i < CXD56_SPI_FIFOSZ; i++)
    {
      spi_getreg(priv, CXD56_SPI_DR_OFFSET);
    }

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(port);

  /* Set a initialized flag */

  priv->initialized = 1;

  return &priv->spidev;
}

#ifdef CONFIG_CXD56_DMAC

/****************************************************************************
 * Name: cxd56_spi_dmaconfig
 *
 * Description:
 *   Enable DMA configuration.
 *
 * Input Parameter:
 *   port   - Port number
 *   chtype - Channel type(TX or RX)
 *   handle - DMA channel handle
 *   conf   - DMA configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_spi_dmaconfig(int port, int chtype, DMA_HANDLE handle,
                         dma_config_t *conf)
{
  struct cxd56_spidev_s *priv = NULL;

  switch (port)
    {
#if defined(CONFIG_CXD56_SPI4) && defined(CONFIG_CXD56_DMAC_SPI4_TX)
      case 4:
        priv = &g_spi4dev;
        break;
#endif

#if defined(CONFIG_CXD56_SPI5) && defined(CONFIG_CXD56_DMAC_SPI5_TX)
      case 5:
        priv = &g_spi5dev;
        break;
#endif

      default:
        break;
    }

  if (priv && priv->initialized)
    {
      if ((chtype == CXD56_SPI_DMAC_CHTYPE_TX) && (!priv->txdmach))
        {
          /* TX DMA setting */

          priv->dmaenable = true;
          priv->txdmach = handle;
          memcpy(&priv->txconfig, conf, sizeof(dma_config_t));
        }
      else if ((chtype == CXD56_SPI_DMAC_CHTYPE_RX) && (!priv->rxdmach))
        {
          /* RX DMA setting */

          priv->dmaenable = true;
          priv->rxdmach = handle;
          memcpy(&priv->rxconfig, conf, sizeof(dma_config_t));
        }
    }
}

#endif

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be done
 *   after a device is deselected if you worry about such things.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spi_flush(struct spi_dev_s *dev)
{
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;
  uint32_t regval                 = 0;

  /* Prohibit the clock change during SPI transfer */

  up_pm_acquire_freqlock(&g_hold_lock);

  /* Disable clock gating (clock enable) */

  cxd56_spi_clock_gate_disable(priv->port);

  if (priv->port == 3)
    {
      /* Enable SPI HW */

      regval = spi_getreg(priv, CXD56_SPI_CR1_OFFSET);
      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, regval | SPI_CR1_SSE);
    }

  /* Wait for the TX FIFO not full indication */

  while (!(spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_TNF));
  spi_putreg(priv, CXD56_SPI_DR_OFFSET, 0xff);

  /* Wait until TX FIFO and TX shift buffer are empty */

  while (spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_BSY);

  /* Wait until RX FIFO is not empty */

  while (!(spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_RNE));

  /* Then read and discard bytes until the RX FIFO is empty */

  do
    {
      spi_getreg(priv, CXD56_SPI_DR_OFFSET);
    }
  while (spi_getreg(priv, CXD56_SPI_SR_OFFSET) & SPI_SR_RNE);

  if (priv->port == 3)
    {
      /* Restore SPI HW state */

      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, regval);
    }

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(priv->port);

  /* Allow the clock change after SPI transfer */

  up_pm_release_freqlock(&g_hold_lock);
}

#ifdef CONFIG_CXD56_DMAC

/****************************************************************************
 * Name: spi_dmaexchange
 *
 * Description:
 *   Exchange a block of data from SPI using DMA
 *
 ****************************************************************************/

static void spi_dmaexchange(struct spi_dev_s *dev,
                            const void *txbuffer,
                            void *rxbuffer, size_t nwords)
{
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)dev;
  uint32_t regval                 = 0;

  DEBUGASSERT(priv && priv->spibase);

  /* Prohibit the clock change during SPI transfer */

  up_pm_acquire_freqlock(&g_hold_lock);

  /* Disable clock gating (clock enable) */

  cxd56_spi_clock_gate_disable(priv->port);

  if (priv->port == 3)
    {
      /* Enable SPI HW */

      regval = spi_getreg(priv, CXD56_SPI_CR1_OFFSET);
      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, regval | SPI_CR1_SSE);
    }

  /* Setup DMAs */

  spi_dmatxsetup(priv, txbuffer, nwords);
  spi_dmarxsetup(priv, rxbuffer, nwords);

  /* Start the DMAs */

  cxd56_dmastart(priv->rxdmach, spi_dmarxcallback, priv);
  cxd56_dmastart(priv->txdmach, spi_dmatxcallback, priv);

  /* Then wait for each to complete */

  spi_dmatrxwait(priv);

  if (priv->port == 3)
    {
      /* Restore SPI HW state */

      spi_putreg(priv, CXD56_SPI_CR1_OFFSET, regval);
    }

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(priv->port);

  /* Allow the clock change after SPI transfer */

  up_pm_release_freqlock(&g_hold_lock);
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: spi_dmasndblock
 *
 * Description:
 *   Send a block of data on SPI using DMA
 *
 ****************************************************************************/

static void spi_dmasndblock(struct spi_dev_s *dev,
                            const void *buffer, size_t nwords)
{
  spi_dmaexchange(dev, buffer, NULL, nwords);
}

/****************************************************************************
 * Name: spi_dmarecvblock
 *
 * Description:
 *   Receive a block of data on SPI using DMA
 *
 ****************************************************************************/

static void spi_dmarecvblock(struct spi_dev_s *dev,
                             const void *buffer, size_t nwords)
{
  spi_dmaexchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the TX DMA completes
 *
 ****************************************************************************/

static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t status, void *data)
{
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)data;

  /* Wake-up the SPI driver */

  if ((status & CXD56_DMA_INTR_ERR) != 0)
    {
      spierr("dma error\n");
    }

  nxsem_post(&priv->dmasem);
}

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t status, void *data)
{
  struct cxd56_spidev_s *priv = (struct cxd56_spidev_s *)data;

  /* Wake-up the SPI driver */

  if ((status & CXD56_DMA_INTR_ERR) != 0)
    {
      spierr("dma error\n");
    }

  nxsem_post(&priv->dmasem);
}

/****************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ****************************************************************************/

static void spi_dmatxsetup(struct cxd56_spidev_s *priv,
                           const void *txbuffer, size_t nwords)
{
  uint32_t dst;
  uint32_t val;

  val = spi_getreg(priv, CXD56_SPI_DMACR_OFFSET);
  val |= SPI_DMACR_TXDMAE;
  spi_putreg(priv, CXD56_SPI_DMACR_OFFSET, val);

  dst = (priv->spibase + CXD56_SPI_DR_OFFSET) & 0x03ffffffu;
  cxd56_txdmasetup(priv->txdmach, (uintptr_t)dst, (uintptr_t)txbuffer,
                   nwords, priv->txconfig);
}

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ****************************************************************************/

static void spi_dmarxsetup(struct cxd56_spidev_s *priv,
                           const void *rxbuffer, size_t nwords)
{
  uint32_t src;
  uint32_t val;

  val = spi_getreg(priv, CXD56_SPI_DMACR_OFFSET);
  val |= SPI_DMACR_RXDMAE;
  spi_putreg(priv, CXD56_SPI_DMACR_OFFSET, val);

  src = (priv->spibase + CXD56_SPI_DR_OFFSET) & 0x03ffffffu;
  cxd56_rxdmasetup(priv->rxdmach, (uintptr_t)src, (uintptr_t)rxbuffer,
                   nwords, priv->rxconfig);
}

/****************************************************************************
 * Name: spi_dmatrxwait
 *
 * Description:
 *   Wait for TX RX DMA to complete.
 *
 ****************************************************************************/

static void spi_dmatrxwait(struct cxd56_spidev_s *priv)
{
  uint32_t val;

  if (nxsem_wait_uninterruptible(&priv->dmasem) != OK)
    {
      spierr("dma error\n");
    }

  if (nxsem_wait_uninterruptible(&priv->dmasem) != OK)
    {
      spierr("dma error\n");
    }

  cxd56_dmastop(priv->txdmach);
  cxd56_dmastop(priv->rxdmach);

  val = spi_getreg(priv, CXD56_SPI_DMACR_OFFSET);
  val &= ~(SPI_DMACR_RXDMAE | SPI_DMACR_TXDMAE);
  spi_putreg(priv, CXD56_SPI_DMACR_OFFSET, val);
}

#endif

#endif
