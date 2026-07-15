/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * The external functions, gd32_spi0select, gd32_spi0status and
 * gd32_spi0cmddata must be provided by board-specific logic.  They are
 * implementations of the select, status and cmddata methods of the SPI
 * interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 * All other methods (including gd32_spibus_initialize()) are provided by
 * common GD32VW55x logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in gd32vw55x_boardinitialize() to configure the SPI
 *      chip select pins.
 *   2. Provide gd32_spi0select() and gd32_spi0status() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is
 *      configured.  Weak default implementations are provided here so that
 *      a board that does not need them still links.
 *   3. Add a call to gd32_spibus_initialize() in your low level application
 *      initialization logic.
 *   4. The handle returned by gd32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic.
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "riscv_internal.h"

#include "chip.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_gpio.h"
#include "gd32vw55x_spi.h"
#include "hardware/gd32vw55x_rcu.h"

#ifdef CONFIG_GD32VW55X_SPI_DMA
#  include "gd32vw55x_dma.h"
#endif

#ifdef CONFIG_GD32VW55X_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SPI is clocked from CK_APB2.  The maximum SCK frequency of the
 * GD32VW553 is 40 MHz (datasheet Table 4-31).
 */

#define GD32_SPI_CLK_MAX        (40000000ul)
#define GD32_SPI_CLK_INIT       (400000ul)

/* Bits of CTL0 that are preserved when the bus is (re-)initialized: the
 * CRC enable and the data frame format.
 */

#define SPI_INIT_MASK           (SPI_CTL0_CRCEN | SPI_CTL0_FF16)

#ifdef CONFIG_GD32VW55X_SPI_DMA

/* The DMA interrupts that are enabled while an SPI DMA is in progress */

#  define SPI_DMA_INTEN         (DMA_INT_MASK)

/* SPI DMA priority */

#  ifndef SPI_DMA_PRIO
#    define SPI_DMA_PRIO        DMA_PRIO_MEDIUM_SELECT
#  endif

#endif /* CONFIG_GD32VW55X_SPI_DMA */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_spidev_s
{
  struct spi_dev_s  spidev;      /* Externally visible part of the SPI
                                  * interface */
  uint32_t          spibase;     /* SPI base address */
  uint32_t          spiclock;    /* Clocking for the SPI module */
  uint32_t          frequency;   /* Requested clock frequency */
  uint32_t          actual;      /* Actual clock frequency */
  mutex_t           lock;        /* Held while the bus is locked */
  uint8_t           nbits;       /* Width of word in bits (8 or 16) */
  uint8_t           mode;        /* Mode 0,1,2,3 */
#ifdef CONFIG_GD32VW55X_SPI_DMA
  volatile uint16_t rxresult;    /* Result of the RX DMA */
  volatile uint16_t txresult;    /* Result of the TX DMA */
#ifdef CONFIG_SPI_TRIGGER
  bool              defertrig;   /* Trigger should be deferred */
  bool              trigarmed;   /* The trigger is armed */
#endif
  uint8_t           rxch;        /* The RX DMA channel request */
  uint8_t           txch;        /* The TX DMA channel request */
  DMA_HANDLE        rxdma;       /* DMA channel handle for RX transfers */
  DMA_HANDLE        txdma;       /* DMA channel handle for TX transfers */
  sem_t             rxsem;       /* Wait for RX DMA to complete */
  sem_t             txsem;       /* Wait for TX DMA to complete */
#endif
  bool              initialized; /* Has the SPI been initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Clock */

static void     gd32_spi_reset(void);
static void     gd32_spi_clock_enable(void);
static void     gd32_spi_clock_disable(void);

/* Helpers */

static inline uint32_t spi_getreg(struct gd32_spidev_s *priv,
                                  uint32_t offset);
static inline void     spi_putreg(struct gd32_spidev_s *priv,
                                  uint32_t offset, uint32_t value);
static void            spi_modifyreg(struct gd32_spidev_s *priv,
                                     uint32_t offset, uint32_t setbits,
                                     uint32_t clrbits);
static inline uint16_t spi_readword(struct gd32_spidev_s *priv);
static inline void     spi_writeword(struct gd32_spidev_s *priv,
                                     uint16_t word);

/* SPI methods */

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int      spi_hwfeatures(struct spi_dev_s *dev,
                               spi_hwfeatures_t features);
#endif
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void     spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                             void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int      spi_trigger(struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                             size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                              size_t nwords);
#endif

/* DMA support */

#ifdef CONFIG_GD32VW55X_SPI_DMA
static int      spi_dmarxwait(struct gd32_spidev_s *priv);
static int      spi_dmatxwait(struct gd32_spidev_s *priv);
static void     spi_dmarxcallback(DMA_HANDLE handle, uint16_t isr,
                                  void *arg);
static void     spi_dmatxcallback(DMA_HANDLE handle, uint16_t isr,
                                  void *arg);
static void     spi_dmarxsetup(struct gd32_spidev_s *priv, void *rxbuffer,
                               void *rxdummy, size_t nwords);
static void     spi_dmatxsetup(struct gd32_spidev_s *priv,
                               const void *txbuffer, const void *txdummy,
                               size_t nwords);
static inline void spi_dmarxstart(struct gd32_spidev_s *priv);
static inline void spi_dmatxstart(struct gd32_spidev_s *priv);
static void     spi_exchange_nodma(struct spi_dev_s *dev,
                                   const void *txbuffer, void *rxbuffer,
                                   size_t nwords);
#endif

/* Initialization */

static void     spi_bus_initialize(struct gd32_spidev_s *priv);
static void     gd32_spi_gpio_config(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi0register,  /* Provided externally */
#else
  .registercallback  = NULL,               /* Not implemented */
#endif
};

static struct gd32_spidev_s g_spi0dev =
{
  .spidev   =
  {
    .ops    = &g_spi0ops
  },
  .spibase  = GD32VW55X_SPI_BASE,
  .spiclock = GD32VW55X_PCLK2_FREQ,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32VW55X_SPI_DMA
  .rxch     = DMA_REQ_SPI_RX,
  .txch     = DMA_REQ_SPI_TX,
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spi_reset
 *
 * Description:
 *   Reset the SPI peripheral
 *
 ****************************************************************************/

static void gd32_spi_reset(void)
{
  modifyreg32(GD32VW55X_RCU_APB2RST, 0, RCU_APB2RST_SPIRST);
  modifyreg32(GD32VW55X_RCU_APB2RST, RCU_APB2RST_SPIRST, 0);
}

/****************************************************************************
 * Name: gd32_spi_clock_enable
 *
 * Description:
 *   Enable the SPI clock
 *
 ****************************************************************************/

static void gd32_spi_clock_enable(void)
{
  modifyreg32(GD32VW55X_RCU_APB2EN, 0, RCU_APB2EN_SPIEN);
}

/****************************************************************************
 * Name: gd32_spi_clock_disable
 *
 * Description:
 *   Disable the SPI clock
 *
 ****************************************************************************/

static void gd32_spi_clock_disable(void)
{
  modifyreg32(GD32VW55X_RCU_APB2EN, RCU_APB2EN_SPIEN, 0);
}

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct gd32_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   value  - The value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_putreg(struct gd32_spidev_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_modifyreg
 *
 * Description:
 *   Clear and set bits in an SPI register
 *
 * Input Parameters:
 *   priv    - Private SPI device structure
 *   offset  - Offset to the register of interest
 *   setbits - The bits to set
 *   clrbits - The bits to clear
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_modifyreg(struct gd32_spidev_s *priv, uint32_t offset,
                          uint32_t setbits, uint32_t clrbits)
{
  uint32_t regval;

  regval  = spi_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  spi_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: spi_wait_status
 *
 * Description:
 *   Wait until all of the bits in 'status' are set in the STAT register
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   status - The bits to wait for
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_wait_status(struct gd32_spidev_s *priv,
                                   uint32_t status)
{
  while (status != (spi_getreg(priv, GD32VW55X_SPI_STAT_OFFSET) & status))
    {
    }
}

/****************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one word from the SPI data register
 *
 * Input Parameters:
 *   priv - Private SPI device structure
 *
 * Returned Value:
 *   The word as read
 *
 ****************************************************************************/

static inline uint16_t spi_readword(struct gd32_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  spi_wait_status(priv, SPI_STAT_RBNE);

  /* Return the data */

  return (uint16_t)spi_getreg(priv, GD32VW55X_SPI_DATA_OFFSET);
}

/****************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one word to the SPI data register
 *
 * Input Parameters:
 *   priv - Private SPI device structure
 *   word - The word to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writeword(struct gd32_spidev_s *priv, uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  spi_wait_status(priv, SPI_STAT_TBE);

  /* Then send the word */

  spi_putreg(priv, GD32VW55X_SPI_DATA_OFFSET, (uint32_t)word);
}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the bus for a sequence of
 *   transfers.  The bus should be locked before the chip is selected.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock the SPI bus, false: unlock the SPI bus
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
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
 *   Set the SPI frequency
 *
 * Input Parameters:
 *   dev       - Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t setbits;
  uint32_t actual;
  uint32_t plk_div;

  /* Check if the requested frequency is the current selection */

  if (priv->frequency == frequency)
    {
      return priv->actual;
    }

  /* Check if the requested frequency is reasonable */

  if (frequency > GD32_SPI_CLK_MAX)
    {
      frequency = GD32_SPI_CLK_MAX;
    }
  else if (frequency == 0)
    {
      frequency = GD32_SPI_CLK_INIT;
    }

  /* Configure the SCK frequency:
   *
   *   SCK frequency = PCLK / plk_div, or plk_div = PCLK / frequency
   */

  plk_div = priv->spiclock / frequency;

  if (plk_div < 2)
    {
      plk_div = 2;
    }
  else if (plk_div > 256)
    {
      plk_div = 256;
    }

  if (plk_div > 128)
    {
      setbits = SPI_CTL0_PSC_256;
    }
  else if (plk_div > 64)
    {
      setbits = SPI_CTL0_PSC_128;
    }
  else if (plk_div > 32)
    {
      setbits = SPI_CTL0_PSC_64;
    }
  else if (plk_div > 16)
    {
      setbits = SPI_CTL0_PSC_32;
    }
  else if (plk_div > 8)
    {
      setbits = SPI_CTL0_PSC_16;
    }
  else if (plk_div > 4)
    {
      setbits = SPI_CTL0_PSC_8;
    }
  else if (plk_div > 2)
    {
      setbits = SPI_CTL0_PSC_4;
    }
  else
    {
      setbits = SPI_CTL0_PSC_2;
    }

  /* The prescaler can only be changed while the SPI is disabled */

  spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
  spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, setbits,
                SPI_CTL0_PSC_MASK);
  spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

  actual = priv->spiclock >> ((setbits >> SPI_CTL0_PSC_SHIFT) + 1);

  spiinfo("Frequency %" PRIu32 "->%" PRIu32 "\n", frequency, actual);

  /* Save the frequency selection so that subsequent reconfigurations will
   * be faster.
   */

  priv->frequency = frequency;
  priv->actual    = actual;

  return priv->actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  See enum spi_mode_e for the mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t setbits;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
        case SPIDEV_MODE0: /* CKPL=0, CKPH=0 */
          setbits = 0;
          break;

        case SPIDEV_MODE1: /* CKPL=0, CKPH=1 */
          setbits = SPI_CTL0_CKPH;
          break;

        case SPIDEV_MODE2: /* CKPL=1, CKPH=0 */
          setbits = SPI_CTL0_CKPL;
          break;

        case SPIDEV_MODE3: /* CKPL=1, CKPH=1 */
          setbits = SPI_CTL0_CKPH | SPI_CTL0_CKPL;
          break;

        default:
          return;
        }

      /* The clock configuration can only be changed while the SPI is
       * disabled.
       */

      spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
      spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, setbits,
                    SPI_CTL0_CKPH | SPI_CTL0_CKPL);
      spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

      /* Save the mode so that subsequent re-configurations will be
       * faster.
       */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested (8 or 16)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t setbits;

  spiinfo("nbits=%d\n", nbits);

  /* The GD32VW55x SPI only supports 8- and 16-bit frames */

  if (nbits != 8 && nbits != 16)
    {
      spierr("ERROR: Unsupported nbits: %d\n", nbits);
      return;
    }

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      setbits = (nbits == 16) ? SPI_CTL0_FF16 : 0;

      spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
      spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, setbits,
                    SPI_CTL0_FF16);
      spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t setbits;

  spiinfo("features=%08" PRIx32 "\n", (uint32_t)features);

  /* Transfer data LSB first? */

  setbits = ((features & HWFEAT_LSBFIRST) != 0) ? SPI_CTL0_LF : 0;

  spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
  spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, setbits, SPI_CTL0_LF);
  spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

  features &= ~HWFEAT_LSBFIRST;
#endif

  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  The size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   The word received from the slave
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv != NULL && priv->spibase != 0);

  spi_writeword(priv, (uint16_t)(wd & 0xffff));
  ret = (uint32_t)spi_readword(priv);

  /* Check and clear any error flags */

  regval = spi_getreg(priv, GD32VW55X_SPI_STAT_OFFSET);
  if ((regval & SPI_STAT_CRCERR) != 0)
    {
      spi_modifyreg(priv, GD32VW55X_SPI_STAT_OFFSET, 0, SPI_STAT_CRCERR);
    }

  spiinfo("Sent: %04" PRIx32 " Return: %04" PRIx32
          " Status: %04" PRIx32 "\n", wd, ret, regval);

  return ret;
}

/****************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - The length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits > 8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_GD32VW55X_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#endif
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint8_t        *brxptr = (uint8_t *)rxbuffer;
  const uint8_t  *btxptr = (const uint8_t *)txbuffer;
  uint16_t       *wrxptr = (uint16_t *)rxbuffer;
  const uint16_t *wtxptr = (const uint16_t *)txbuffer;
  uint8_t         byte;
  uint16_t        word;

  DEBUGASSERT(priv != NULL && priv->spibase != 0);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%zu\n",
          txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode */

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (wtxptr != NULL)
            {
              word = *wtxptr++;
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = (uint16_t)spi_send(dev, (uint32_t)word);

          /* Is there a buffer to receive the return value? */

          if (wrxptr != NULL)
            {
              *wrxptr++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      while (nwords-- > 0)
        {
          /* Get the next byte to write.  Is there a source buffer? */

          if (btxptr != NULL)
            {
              byte = *btxptr++;
            }
          else
            {
              byte = 0xff;
            }

          /* Exchange one byte */

          byte = (uint8_t)spi_send(dev, (uint32_t)byte);

          /* Is there a buffer to receive the return value? */

          if (brxptr != NULL)
            {
              *brxptr++ = byte;
            }
        }
    }
}

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for the RX DMA to complete
 *
 ****************************************************************************/

#ifdef CONFIG_GD32VW55X_SPI_DMA
static int spi_dmarxwait(struct gd32_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->rxresult == 0 && ret == OK);

  return ret;
}

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for the TX DMA to complete
 *
 ****************************************************************************/

static int spi_dmatxwait(struct gd32_spidev_s *priv)
{
  int ret;

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->txresult == 0 && ret == OK);

  return ret;
}

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

static void spi_dmarxcallback(DMA_HANDLE handle, uint16_t isr, void *arg)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)arg;

  /* Wake up the SPI driver.  The status is OR'ed with 0x0080 to assure that
   * it is non-zero.
   */

  priv->rxresult = isr | 0x0080;
  nxsem_post(&priv->rxsem);
}

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the TX DMA completes
 *
 ****************************************************************************/

static void spi_dmatxcallback(DMA_HANDLE handle, uint16_t isr, void *arg)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)arg;

  priv->txresult = isr | 0x0080;
  nxsem_post(&priv->txsem);
}

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform the RX DMA
 *
 ****************************************************************************/

static void spi_dmarxsetup(struct gd32_spidev_s *priv, void *rxbuffer,
                           void *rxdummy, size_t nwords)
{
  struct gd32_dma_config_s dma_init_struct;

  /* Enable the SPI RX DMA */

  spi_modifyreg(priv, GD32VW55X_SPI_CTL1_OFFSET, SPI_CTL1_DMAREN, 0);

  /* Is there a buffer to receive the data in? */

  if (rxbuffer != NULL)
    {
      dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    }
  else
    {
      rxbuffer = rxdummy;
      dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
    }

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      dma_init_struct.periph_memory_width = DMA_WIDTH_16BITS_SELECT;
    }
  else
    {
      dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
    }

  dma_init_struct.direction    = DMA_PERIPH_TO_MEMORY;
  dma_init_struct.memory0_addr = (uint32_t)rxbuffer;
  dma_init_struct.number       = nwords;
  dma_init_struct.periph_addr  = GD32VW55X_SPI_DATA(priv->spibase);
  dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
  dma_init_struct.priority     = SPI_DMA_PRIO;

  /* Configure the RX DMA */

  gd32_dma_setup(priv->rxdma, &dma_init_struct,
                 GD32_DMA_SINGLE_DATA_MODE);
}

/****************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform the TX DMA
 *
 ****************************************************************************/

static void spi_dmatxsetup(struct gd32_spidev_s *priv, const void *txbuffer,
                           const void *txdummy, size_t nwords)
{
  struct gd32_dma_config_s dma_init_struct;

  /* Enable the SPI TX DMA */

  spi_modifyreg(priv, GD32VW55X_SPI_CTL1_OFFSET, SPI_CTL1_DMATEN, 0);

  /* Is there a buffer to transfer the data from? */

  if (txbuffer != NULL)
    {
      dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    }
  else
    {
      txbuffer = txdummy;
      dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
    }

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      dma_init_struct.periph_memory_width = DMA_WIDTH_16BITS_SELECT;
    }
  else
    {
      dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
    }

  dma_init_struct.direction    = DMA_MEMORY_TO_PERIPH;
  dma_init_struct.memory0_addr = (uint32_t)txbuffer;
  dma_init_struct.number       = nwords;
  dma_init_struct.periph_addr  = GD32VW55X_SPI_DATA(priv->spibase);
  dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
  dma_init_struct.priority     = SPI_DMA_PRIO;

  /* Configure the TX DMA */

  gd32_dma_setup(priv->txdma, &dma_init_struct,
                 GD32_DMA_SINGLE_DATA_MODE);
}

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start the RX DMA
 *
 ****************************************************************************/

static inline void spi_dmarxstart(struct gd32_spidev_s *priv)
{
  priv->rxresult = 0;
  gd32_dma_start(priv->rxdma, spi_dmarxcallback, priv, SPI_DMA_INTEN);
}

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start the TX DMA
 *
 ****************************************************************************/

static inline void spi_dmatxstart(struct gd32_spidev_s *priv)
{
  priv->txresult = 0;
  gd32_dma_start(priv->txdma, spi_dmatxcallback, priv, SPI_DMA_INTEN);
}

/****************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - The length of data to be exchanged in units of words
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  static uint16_t rxdummy = 0xffff;
  static const uint16_t txdummy = 0xffff;
  size_t nbytes;
  int ret;

  DEBUGASSERT(priv != NULL && priv->spibase != 0);

  /* Convert the number of words to a number of bytes */

  nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;
  UNUSED(nbytes);

  if (priv->rxdma == NULL || priv->txdma == NULL ||
      up_interrupt_context()
#ifdef CONFIG_GD32VW55X_SPI_DMATHRESHOLD
      || nbytes <= CONFIG_GD32VW55X_SPI_DMATHRESHOLD
#endif
     )
    {
      /* Invalid DMA channels, or interrupt context: fall back to the
       * non-DMA method.
       */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%zu\n",
          txbuffer, rxbuffer, nwords);

  /* Setup the DMAs */

  spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);
  spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

#ifdef CONFIG_SPI_TRIGGER
  /* Is deferred triggering in effect? */

  if (!priv->defertrig)
    {
      /* No: start the DMAs now */

      spi_dmarxstart(priv);
      spi_dmatxstart(priv);
    }
  else
    {
      /* Yes: indicate that we are ready to be started */

      priv->trigarmed = true;
      return;
    }
#else
  /* Start the DMAs */

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);
#endif

  /* Then wait for each to complete */

  ret = spi_dmarxwait(priv);
  if (ret < 0)
    {
      spi_dmatxwait(priv);
    }
  else
    {
      ret = spi_dmatxwait(priv);
    }

  /* Disable the SPI DMA requests */

  spi_modifyreg(priv, GD32VW55X_SPI_CTL1_OFFSET, 0,
                SPI_CTL1_DMAREN | SPI_CTL1_DMATEN);

#ifdef CONFIG_SPI_TRIGGER
  priv->trigarmed = false;
#endif
}
#endif /* CONFIG_GD32VW55X_SPI_DMA */

/****************************************************************************
 * Name: spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev)
{
#ifdef CONFIG_GD32VW55X_SPI_DMA
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  if (!priv->trigarmed)
    {
      return -EIO;
    }

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);

  return OK;
#else
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - The length of data to send from the buffer in number of
 *              words
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", txbuffer, nwords);
  spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - The length of data that can be received in the buffer in
 *              number of words
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                          size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%zu\n", rxbuffer, nwords);
  spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the SPI bus in its default state (Master, 8-bit, mode 0)
 *
 * Input Parameters:
 *   priv - Private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_bus_initialize(struct gd32_spidev_s *priv)
{
  uint32_t regval;

  /* Configure CTL0.  Default configuration:
   *
   *   Mode 0:                   CKPH=0 and CKPL=0
   *   Master:                   MSTMOD=1
   *   8-bit:                    FF16=0
   *   MSB transmitted first:    LF=0
   *   NSS software mode:        SWNSSEN=1
   *   Full duplex:              RO=0, BDEN=0
   */

  regval  = spi_getreg(priv, GD32VW55X_SPI_CTL0_OFFSET);
  regval &= SPI_INIT_MASK;
  regval |= (SPI_MASTER | SPI_CTL0_SWNSSEN);
  spi_putreg(priv, GD32VW55X_SPI_CTL0_OFFSET, regval);

  /* CTL1: no DMA, no interrupts, no TI mode */

  spi_putreg(priv, GD32VW55X_SPI_CTL1_OFFSET, 0);

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approximately 400 KHz */

  spi_setfrequency((struct spi_dev_s *)priv, GD32_SPI_CLK_INIT);

#ifdef CONFIG_GD32VW55X_SPI_DMA
  if (priv->txdma == NULL && priv->rxdma == NULL)
    {
      /* Get the DMA channels */

      priv->rxdma = gd32_dma_channel_alloc(priv->rxch);
      priv->txdma = gd32_dma_channel_alloc(priv->txch);
      DEBUGASSERT(priv->rxdma != NULL && priv->txdma != NULL);
    }
#endif

  /* Enable the SPI */

  spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);
}

/****************************************************************************
 * Name: gd32_spi_gpio_config
 *
 * Description:
 *   Configure the GPIO pins of the SPI: SCK, MISO and MOSI.  The chip
 *   select pins are configured by the board-specific logic.
 *
 ****************************************************************************/

static void gd32_spi_gpio_config(void)
{
  gd32_gpio_config(GPIO_SPI_SCK);
  gd32_gpio_config(GPIO_SPI_MISO);
  gd32_gpio_config(GPIO_SPI_MOSI);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spi0select, gd32_spi0status and gd32_spi0cmddata
 *
 * Description:
 *   Weak default implementations of the board-specific SPI methods.  A
 *   board that has SPI slaves must override these.
 *
 ****************************************************************************/

void weak_function gd32_spi0select(struct spi_dev_s *dev, uint32_t devid,
                                   bool selected)
{
  spiwarn("WARNING: gd32_spi0select not provided by the board\n");
}

uint8_t weak_function gd32_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int weak_function gd32_spi0cmddata(struct spi_dev_s *dev, uint32_t devid,
                                   bool cmd)
{
  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: gd32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   bus - The SPI bus number.  The GD32VW55x has a single SPI: bus 0.
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *gd32_spibus_initialize(int bus)
{
  struct gd32_spidev_s *priv = NULL;
  irqstate_t flags;

  if (bus != GD32VW55X_SPI_BUS)
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
      return NULL;
    }

  priv  = &g_spi0dev;
  flags = enter_critical_section();

  /* Has the SPI already been initialized? */

  if (!priv->initialized)
    {
      /* Configure the GPIO pins of the SPI */

      gd32_spi_gpio_config();

      /* Enable the SPI clock */

      gd32_spi_clock_enable();

      /* Set up the default configuration */

      spi_bus_initialize(priv);

      priv->initialized = true;
    }

  leave_critical_section(flags);

  return (struct spi_dev_s *)priv;
}

/****************************************************************************
 * Name: gd32_spibus_deinitialize
 *
 * Description:
 *   Deinitialize the selected SPI bus
 *
 * Input Parameters:
 *   bus - The SPI bus number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_spibus_deinitialize(int bus)
{
  struct gd32_spidev_s *priv = NULL;
  irqstate_t flags;

  if (bus != GD32VW55X_SPI_BUS)
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
      return;
    }

  priv  = &g_spi0dev;
  flags = enter_critical_section();

  if (priv->initialized)
    {
#ifdef CONFIG_GD32VW55X_SPI_DMA
      if (priv->rxdma != NULL)
        {
          gd32_dma_channel_free(priv->rxdma);
          priv->rxdma = NULL;
        }

      if (priv->txdma != NULL)
        {
          gd32_dma_channel_free(priv->txdma);
          priv->txdma = NULL;
        }
#endif

      /* Disable the SPI and its clock, then reset the peripheral */

      spi_modifyreg(priv, GD32VW55X_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);

      gd32_spi_clock_disable();
      gd32_spi_reset();

      priv->initialized = false;
    }

  leave_critical_section(flags);
}

#endif /* CONFIG_GD32VW55X_SPI */
