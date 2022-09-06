/****************************************************************************
 * arch/arm/src/rp2040/rp2040_spi.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"

#include "rp2040_spi.h"
#include "hardware/rp2040_spi.h"

#ifdef CONFIG_RP2040_SPI_DMA
#include "rp2040_dmac.h"
#endif

#ifdef CONFIG_RP2040_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 8 frame FIFOs for both transmit and receive */

#define RP2040_SPI_FIFOSZ        8

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SPI driver */

struct rp2040_spidev_s
{
  struct spi_dev_s spidev;      /* Externally visible part of the SPI interface */
  uint32_t         spibase;     /* SPIn base address */
  uint32_t         spibasefreq;
#ifdef CONFIG_RP2040_SPI_INTERRUPTS
  uint8_t          spiirq;      /* SPI IRQ number */
#endif
  mutex_t          lock;        /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;   /* Requested clock frequency */
  uint32_t         actual;      /* Actual clock frequency */
  uint8_t          nbits;       /* Width of word in bits (4 to 16) */
  uint8_t          mode;        /* Mode 0,1,2,3 */
  uint8_t          port;        /* Port number */
  int              initialized; /* Initialized flag */
#ifdef CONFIG_RP2040_SPI_DMA
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

static inline uint32_t spi_getreg(struct rp2040_spidev_s *priv,
                                  uint8_t offset);
static inline void spi_putreg(struct rp2040_spidev_s *priv,
                              uint8_t offset, uint32_t value);

/* DMA support */

#ifdef CONFIG_RP2040_SPI_DMA
static void unused_code spi_dmaexchange(struct spi_dev_s *dev,
                                        const void *txbuffer,
                                        void *rxbuffer, size_t nwords);
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t status, void *data);
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t status, void *data);
static void spi_dmatxsetup(struct rp2040_spidev_s *priv,
                           const void *txbuffer, size_t nwords);
static void spi_dmarxsetup(struct rp2040_spidev_s *priv,
                           const void *rxbuffer, size_t nwords);
static void spi_dmatrxwait(struct rp2040_spidev_s *priv);
#ifndef CONFIG_SPI_EXCHANGE
static void spi_dmasndblock(struct spi_dev_s *dev,
                            const void *buffer, size_t nwords);
static void spi_dmarecvblock(struct spi_dev_s *dev,
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

#ifdef CONFIG_RP2040_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = rp2040_spi0select,   /* Provided externally */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                   /* Not supported */
#endif
  .status            = rp2040_spi0status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = rp2040_spi0cmddata,  /* Provided externally */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = rp2040_spi0register, /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct rp2040_spidev_s g_spi0dev =
{
  .spidev            =
  {
    .ops             = &g_spi0ops,
  },
  .spibase           = RP2040_SPI0_BASE,
  .spibasefreq       = 0,
  .port              = 0,
  .initialized       = 0,
#ifdef CONFIG_RP2040_SPI_INTERRUPTS
  .spiirq            = RP2040_SPI0_IRQ,
#endif
  .lock              = NXMUTEX_INITIALIZER,
#ifdef CONFIG_RP2040_SPI_DMA
  .dmasem            = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_RP2040_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = rp2040_spi1select,   /* Provided externally */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                   /* Not supported */
#endif
  .status            = rp2040_spi1status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = rp2040_spi1cmddata,  /* Provided externally */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = rp2040_spi1register, /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct rp2040_spidev_s g_spi1dev =
{
  .spidev            =
  {
    .ops             = &g_spi1ops,
  },
  .spibase           = RP2040_SPI1_BASE,
  .spibasefreq       = 0,
  .port              = 1,
  .initialized       = 0,
#ifdef CONFIG_RP2040_SPI_INTERRUPTS
  .spiirq            = RP2040_SPI1_IRQ,
#endif
  .lock              = NXMUTEX_INITIALIZER,
#ifdef CONFIG_RP2040_SPI_DMA
  .dmasem            = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_RP2040_SPI_DMA
/* Dummy data if no data transfer needed */

uint32_t g_spitxdmadummy = 0xffffffff;
uint32_t g_spirxdmadummy = 0;
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

static inline uint32_t spi_getreg(struct rp2040_spidev_s *priv,
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

static inline void spi_putreg(struct rp2040_spidev_s *priv,
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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;

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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;
  uint32_t divisor;
  uint32_t actual;

  /* Set SPI_CLOCK */

  /* frequency = SPI_CLOCK / divisor, or divisor = SPI_CLOCK / frequency */

  priv->spibasefreq = BOARD_PERI_FREQ;
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

  /* Save the new divisor value */

  spi_putreg(priv, RP2040_SPI_SSPCPSR_OFFSET, divisor);

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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR0 appropriately */

      regval = spi_getreg(priv, RP2040_SPI_SSPCR0_OFFSET);
      regval &= ~(RP2040_SPI_SSPCR0_SPO | RP2040_SPI_SSPCR0_SPH);

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            break;

          case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            regval |= RP2040_SPI_SSPCR0_SPH;
            break;

          case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            regval |= RP2040_SPI_SSPCR0_SPO;
            break;

          case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            regval |= (RP2040_SPI_SSPCR0_SPO | RP2040_SPI_SSPCR0_SPH);
            break;

          default:
            spierr("Bad mode: %d\n", mode);
            DEBUGASSERT(FALSE);

            return;
        }

      spi_putreg(priv, RP2040_SPI_SSPCR0_OFFSET, regval);

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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 3 && nbits < 17);

  if (nbits != priv->nbits)
    {
      /* Yes... Set CR0 appropriately */

      regval = spi_getreg(priv, RP2040_SPI_SSPCR0_OFFSET);
      regval &= ~RP2040_SPI_SSPCR0_DSS_MASK;
      regval |= ((nbits - 1) << RP2040_SPI_SSPCR0_DSS_SHIFT);
      spi_putreg(priv, RP2040_SPI_SSPCR0_OFFSET, regval);

      /* Save the selection so that re-configurations will be faster
       */

      priv->nbits = nbits;
#ifdef CONFIG_RP2040_SPI_DMA
      if (priv->nbits > 8)
        {
          priv->txconfig.size = RP2040_DMA_SIZE_HALFWORD;
          priv->rxconfig.size = RP2040_DMA_SIZE_HALFWORD;
        }
      else
        {
          priv->txconfig.size = RP2040_DMA_SIZE_BYTE;
          priv->rxconfig.size = RP2040_DMA_SIZE_BYTE;
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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;
  register uint32_t regval;

  /* Wait while the TX FIFO is full */

  while (!(spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) & RP2040_SPI_SSPSR_TNF))
    ;

  /* Write the byte to the TX FIFO */

  spi_putreg(priv, RP2040_SPI_SSPDR_OFFSET, wd);

  /* Wait for the RX FIFO not empty */

  while (!(spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) & RP2040_SPI_SSPSR_RNE))
    ;

  /* Get the value from the RX FIFO and return it */

  regval = spi_getreg(priv, RP2040_SPI_SSPDR_OFFSET);
  spiinfo("%04" PRIx32 "->%04" PRIx32 "\n", wd, regval);

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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;

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

  while (nwords || rxpending)
    {
      /* Write data to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      spiinfo("TX: rxpending: %" PRId32 " nwords: %d\n", rxpending, nwords);
      while ((spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) &
              RP2040_SPI_SSPSR_TNF) &&
             (rxpending < RP2040_SPI_FIFOSZ) && nwords)
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

          spi_putreg(priv, RP2040_SPI_SSPDR_OFFSET,
                     txbuffer ? data : datadummy);
          nwords--;
          rxpending++;
        }

      /* Now, read the RX data from the RX FIFO
       * while the RX FIFO is not empty
       */

      spiinfo("RX: rxpending: %" PRId32 "\n", rxpending);
      while (spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) &
             RP2040_SPI_SSPSR_RNE)
        {
          data = spi_getreg(priv, RP2040_SPI_SSPDR_OFFSET);
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
#ifdef CONFIG_RP2040_SPI_DMA
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;

#ifdef CONFIG_RP2040_SPI_DMATHRESHOLD
  size_t dmath = CONFIG_RP2040_SPI_DMATHRESHOLD;
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_spibus_initialize
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

struct spi_dev_s *rp2040_spibus_initialize(int port)
{
  struct rp2040_spidev_s *priv;
  uint32_t regval;
  int i;
#ifdef CONFIG_RP2040_SPI_DMA
  dma_config_t txconf;
  dma_config_t rxconf;
#endif

  switch (port)
    {
#ifdef CONFIG_RP2040_SPI0
      case 0:
        priv = &g_spi0dev;
#ifdef CONFIG_RP2040_SPI_DMA
        txconf.dreq = RP2040_DMA_DREQ_SPI0_TX;
        rxconf.dreq = RP2040_DMA_DREQ_SPI0_RX;
#endif
        break;
#endif

#ifdef CONFIG_RP2040_SPI1
      case 1:
        priv = &g_spi1dev;
#ifdef CONFIG_RP2040_SPI_DMA
        txconf.dreq = RP2040_DMA_DREQ_SPI1_TX;
        rxconf.dreq = RP2040_DMA_DREQ_SPI1_RX;
#endif
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

  priv->spibasefreq = BOARD_PERI_FREQ;

  /* DMA settings */

#ifdef CONFIG_RP2040_SPI_DMA
  priv->txdmach = rp2040_dmachannel();
  txconf.size = RP2040_DMA_SIZE_BYTE;
  txconf.noincr = false;
  priv->txconfig = txconf;

  priv->rxdmach = rp2040_dmachannel();
  rxconf.size = RP2040_DMA_SIZE_BYTE;
  rxconf.noincr = false;
  priv->rxconfig = rxconf;

  priv->dmaenable = true;
#endif

  /* Configure 8-bit SPI mode */

  spi_putreg(priv, RP2040_SPI_SSPCR0_OFFSET,
             ((8 - 1) << RP2040_SPI_SSPCR0_DSS_SHIFT) |
             (0 << RP2040_SPI_SSPCR0_FRF_SHIFT));

  /* Disable SPI and all interrupts (we'll poll for all data) */

  spi_putreg(priv, RP2040_SPI_SSPCR1_OFFSET, 0);
  spi_putreg(priv, RP2040_SPI_SSPIMSC_OFFSET, 0);

  /* Clear interrupts */

  spi_putreg(priv, RP2040_SPI_SSPICR_OFFSET, 0x3);

  /* Set the initial SPI configuration */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  regval = spi_getreg(priv, RP2040_SPI_SSPCR1_OFFSET);
  spi_putreg(priv, RP2040_SPI_SSPCR1_OFFSET, regval | RP2040_SPI_SSPCR1_SSE);

  for (i = 0; i < RP2040_SPI_FIFOSZ; i++)
    {
      spi_getreg(priv, RP2040_SPI_SSPDR_OFFSET);
    }

  /* Set a initialized flag */

  priv->initialized = 1;
  return &priv->spidev;
}

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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;

  /* Wait for the TX FIFO not full indication */

  while (!(spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) & RP2040_SPI_SSPSR_TNF))
    ;
  spi_putreg(priv, RP2040_SPI_SSPDR_OFFSET, 0xff);

  /* Wait until TX FIFO and TX shift buffer are empty */

  while (spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) & RP2040_SPI_SSPSR_BSY);

  /* Wait until RX FIFO is not empty */

  while (!(spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) & RP2040_SPI_SSPSR_RNE))
    ;

  /* Then read and discard bytes until the RX FIFO is empty */

  do
    {
      spi_getreg(priv, RP2040_SPI_SSPDR_OFFSET);
    }
  while (spi_getreg(priv, RP2040_SPI_SSPSR_OFFSET) & RP2040_SPI_SSPSR_RNE);
}

#ifdef CONFIG_RP2040_SPI_DMA

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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)dev;

  DEBUGASSERT(priv && priv->spibase);

  /* Setup DMAs */

  spi_dmatxsetup(priv, txbuffer, nwords);
  spi_dmarxsetup(priv, rxbuffer, nwords);

  /* Start the DMAs */

  rp2040_dmastart(priv->rxdmach, spi_dmarxcallback, priv);
  rp2040_dmastart(priv->txdmach, spi_dmatxcallback, priv);

  /* Then wait for each to complete */

  spi_dmatrxwait(priv);
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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)data;

  /* Wake-up the SPI driver */

  if (status != 0)
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
  struct rp2040_spidev_s *priv = (struct rp2040_spidev_s *)data;

  /* Wake-up the SPI driver */

  if (status != 0)
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

static void spi_dmatxsetup(struct rp2040_spidev_s *priv,
                           const void *txbuffer, size_t nwords)
{
  uint32_t dst;
  uint32_t val;

  val = spi_getreg(priv, RP2040_SPI_SSPDMACR_OFFSET);
  val |= RP2040_SPI_SSPDMACR_TXDMAE;
  spi_putreg(priv, RP2040_SPI_SSPDMACR_OFFSET, val);

  dst = priv->spibase + RP2040_SPI_SSPDR_OFFSET;

  if (txbuffer == NULL)
    {
      /* No source data buffer.  Point to our dummy buffer and leave
       * the txconfig so that no address increment is performed.
       */

      txbuffer = (const void *)&g_spitxdmadummy;
      priv->txconfig.noincr = true;
    }
  else
    {
      /* Source data is available.  Use normal TX memory incrementing. */

      priv->txconfig.noincr = false;
    }

  rp2040_txdmasetup(priv->txdmach, (uintptr_t)dst, (uintptr_t)txbuffer,
                   nwords << priv->txconfig.size, priv->txconfig);
}

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ****************************************************************************/

static void spi_dmarxsetup(struct rp2040_spidev_s *priv,
                           const void *rxbuffer, size_t nwords)
{
  uint32_t src;
  uint32_t val;

  val = spi_getreg(priv, RP2040_SPI_SSPDMACR_OFFSET);
  val |= RP2040_SPI_SSPDMACR_RXDMAE;
  spi_putreg(priv, RP2040_SPI_SSPDMACR_OFFSET, val);

  src = priv->spibase + RP2040_SPI_SSPDR_OFFSET;

  if (rxbuffer == NULL)
    {
      /* No sink data buffer.  Point to our dummy buffer and leave
       * the rxconfig so that no address increment is performed.
       */

      rxbuffer = (const void *)&g_spirxdmadummy;
      priv->rxconfig.noincr = true;
    }
  else
    {
      /* Receive buffer is available.  Use normal RX memory incrementing. */

      priv->rxconfig.noincr = false;
    }

  rp2040_rxdmasetup(priv->rxdmach, (uintptr_t)src, (uintptr_t)rxbuffer,
                   nwords << priv->rxconfig.size, priv->rxconfig);
}

/****************************************************************************
 * Name: spi_dmatrxwait
 *
 * Description:
 *   Wait for TX RX DMA to complete.
 *
 ****************************************************************************/

static void spi_dmatrxwait(struct rp2040_spidev_s *priv)
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

  rp2040_dmastop(priv->txdmach);
  rp2040_dmastop(priv->rxdmach);

  val = spi_getreg(priv, RP2040_SPI_SSPDMACR_OFFSET);
  val &= ~(RP2040_SPI_SSPDMACR_RXDMAE | RP2040_SPI_SSPDMACR_TXDMAE);
  spi_putreg(priv, RP2040_SPI_SSPDMACR_OFFSET, val);
}

#endif

#endif
