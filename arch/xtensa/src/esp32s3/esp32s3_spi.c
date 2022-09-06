/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spi.c
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

#ifdef CONFIG_ESP32S3_SPI

#include <assert.h>
#include <debug.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "esp32s3_spi.h"
#include "esp32s3_irq.h"
#include "esp32s3_gpio.h"

#ifdef CONFIG_ESP32S3_SPI2_DMA
#include "esp32s3_dma.h"
#endif

#include "xtensa.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_pinmap.h"
#include "hardware/esp32s3_spi.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if Chip-Select pin will be controlled via software */

#ifdef CONFIG_ESP32S3_SPI_SWCS
#  define SPI_HAVE_SWCS 1
#else
#  define SPI_HAVE_SWCS 0
#endif

#ifdef CONFIG_ESP32S3_SPI2_DMA

/* SPI DMA RX/TX number of descriptors */

#define SPI_DMA_DESC_NUM    (CONFIG_ESP32S3_SPI2_DMADESC_NUM)

/* SPI DMA reset before exchange */

#define SPI_DMA_RESET_MASK  (SPI_DMA_AFIFO_RST_M | SPI_RX_AFIFO_RST_M)

#endif

/* SPI default frequency (limited by clock divider) */

#define SPI_DEFAULT_FREQ  (400000)

/* SPI default width */

#define SPI_DEFAULT_WIDTH (8)

/* SPI default mode */

#define SPI_DEFAULT_MODE  (SPIDEV_MODE0)

/* Helper for applying the mask for a given register field.
 * Mask is determined by the macros suffixed with _V and _S from the
 * peripheral register description.
 */

#define VALUE_MASK(_val, _field) (((_val) & (_field##_V)) << (_field##_S))

/* SPI Maximum buffer size in bytes */

#define SPI_MAX_BUF_SIZE (64)

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device hardware configuration */

struct esp32s3_spi_config_s
{
  uint32_t clk_freq;          /* SPI default clock frequency */
  uint32_t width;             /* SPI default width */
  enum spi_mode_e mode;       /* SPI default mode */

  uint8_t id;                 /* ESP32-S3 SPI device ID: SPIx {2,3} */
  uint8_t cs_pin;             /* GPIO configuration for CS */
  uint8_t mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t miso_pin;           /* GPIO configuration for MISO */
  uint8_t clk_pin;            /* GPIO configuration for CLK */
#ifdef CONFIG_ESP32S3_SPI2_DMA
  uint8_t periph;             /* Peripheral ID */
  uint8_t irq;                /* Interrupt ID */
#endif
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* SPI reset bit */
#ifdef CONFIG_ESP32S3_SPI2_DMA
  uint32_t dma_clk_bit;       /* DMA clock enable bit */
  uint32_t dma_rst_bit;       /* DMA reset bit */
#endif
  uint32_t cs_insig;          /* SPI CS input signal index */
  uint32_t cs_outsig;         /* SPI CS output signal index */
  uint32_t mosi_insig;        /* SPI MOSI input signal index */
  uint32_t mosi_outsig;       /* SPI MOSI output signal index */
  uint32_t miso_insig;        /* SPI MISO input signal index */
  uint32_t miso_outsig;       /* SPI MISO output signal index */
  uint32_t clk_insig;         /* SPI CLK input signal index */
  uint32_t clk_outsig;        /* SPI CLK output signal index */
};

struct esp32s3_spi_priv_s
{
  /* Externally visible part of the SPI interface */

  struct spi_dev_s spi_dev;

  /* Port configuration */

  const struct esp32s3_spi_config_s *config;
  int refs;             /* Reference count */
  mutex_t lock;         /* Held while chip is selected for mutual exclusion */
#ifdef CONFIG_ESP32S3_SPI2_DMA
  sem_t sem_isr;        /* Interrupt wait semaphore */
  int cpu;              /* CPU ID */
  int cpuint;           /* SPI interrupt ID */
  int32_t dma_channel;  /* Channel assigned by the GDMA driver */
#endif
  uint32_t frequency;   /* Requested clock frequency */
  uint32_t actual;      /* Actual clock frequency */
  enum spi_mode_e mode; /* Actual SPI hardware mode */
  uint8_t nbits;        /* Actual SPI send/receive bits once transmission */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_spi_lock(struct spi_dev_s *dev, bool lock);
#ifndef CONFIG_ESP32S3_SPI_UDCS
static void esp32s3_spi_select(struct spi_dev_s *dev,
                               uint32_t devid, bool selected);
#endif
static uint32_t esp32s3_spi_setfrequency(struct spi_dev_s *dev,
                                         uint32_t frequency);
static void esp32s3_spi_setmode(struct spi_dev_s *dev,
                                enum spi_mode_e mode);
static void esp32s3_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int esp32s3_spi_hwfeatures(struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint32_t esp32s3_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void esp32s3_spi_exchange(struct spi_dev_s *dev,
                                 const void *txbuffer,
                                 void *rxbuffer, size_t nwords);
#ifdef CONFIG_ESP32S3_SPI2_DMA
static int esp32s3_spi_interrupt(int irq, void *context, void *arg);
static int esp32s3_spi_sem_waitdone(struct esp32s3_spi_priv_s *priv);
static void esp32s3_spi_dma_exchange(struct esp32s3_spi_priv_s *priv,
                                     const void *txbuffer,
                                     void *rxbuffer,
                                     uint32_t nwords);
#else
static void esp32s3_spi_poll_exchange(struct esp32s3_spi_priv_s *priv,
                                      const void *txbuffer,
                                      void *rxbuffer,
                                      size_t nwords);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void esp32s3_spi_sndblock(struct spi_dev_s *dev,
                                 const void *txbuffer,
                                 size_t nwords);
static void esp32s3_spi_recvblock(struct spi_dev_s *dev,
                                  void *rxbuffer,
                                  size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int esp32s3_spi_trigger(struct spi_dev_s *dev);
#endif
#ifdef CONFIG_ESP32S3_SPI2_DMA
static void esp32s3_spi_dma_init(struct spi_dev_s *dev);
#endif
static void esp32s3_spi_init(struct spi_dev_s *dev);
static void esp32s3_spi_deinit(struct spi_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2
static const struct esp32s3_spi_config_s esp32s3_spi2_config =
{
  .clk_freq     = SPI_DEFAULT_FREQ,
  .width        = SPI_DEFAULT_WIDTH,
  .id           = 2,
  .mode         = SPI_DEFAULT_MODE,
  .cs_pin       = CONFIG_ESP32S3_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESP32S3_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESP32S3_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESP32S3_SPI2_CLKPIN,
#ifdef CONFIG_ESP32S3_SPI2_DMA
  .periph       = ESP32S3_PERIPH_SPI2,
  .irq          = ESP32S3_IRQ_SPI2,
#endif
  .clk_bit      = SYSTEM_SPI2_CLK_EN,
  .rst_bit      = SYSTEM_SPI2_RST,
#ifdef CONFIG_ESP32S3_SPI2_DMA
  .dma_clk_bit  = SYSTEM_SPI2_DMA_CLK_EN,
  .dma_rst_bit  = SYSTEM_SPI2_DMA_RST,
#endif
  .cs_insig     = FSPICS0_IN_IDX,
  .cs_outsig    = FSPICS0_OUT_IDX,
  .mosi_insig   = FSPID_IN_IDX,
  .mosi_outsig  = FSPID_OUT_IDX,
  .miso_insig   = FSPIQ_IN_IDX,
  .miso_outsig  = FSPIQ_OUT_IDX,
  .clk_insig    = FSPICLK_IN_IDX,
  .clk_outsig   = FSPICLK_OUT_IDX
};

static const struct spi_ops_s esp32s3_spi2_ops =
{
  .lock              = esp32s3_spi_lock,
#ifdef CONFIG_ESP32S3_SPI_UDCS
  .select            = esp32s3_spi2_select,
#else
  .select            = esp32s3_spi_select,
#endif
  .setfrequency      = esp32s3_spi_setfrequency,
  .setmode           = esp32s3_spi_setmode,
  .setbits           = esp32s3_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = esp32s3_spi_hwfeatures,
#endif
  .status            = esp32s3_spi2_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = esp32s3_spi2_cmddata,
#endif
  .send              = esp32s3_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = esp32s3_spi_exchange,
#else
  .sndblock          = esp32s3_spi_sndblock,
  .recvblock         = esp32s3_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = esp32s3_spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct esp32s3_spi_priv_s esp32s3_spi2_priv =
{
  .spi_dev     =
  {
    .ops       = &esp32s3_spi2_ops
  },
  .config      = &esp32s3_spi2_config,
  .refs        = 0,
  .lock        = NXMUTEX_INITIALIZER,
#ifdef CONFIG_ESP32S3_SPI2_DMA
  .sem_isr     = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .cpuint      = -ENOMEM,
  .dma_channel = -1,
#endif
  .frequency   = 0,
  .actual      = 0,
  .mode        = 0,
  .nbits       = 0
};
#endif /* CONFIG_ESP32S3_SPI2 */

#ifdef CONFIG_ESP32S3_SPI3
static const struct esp32s3_spi_config_s esp32s3_spi3_config =
{
  .clk_freq     = SPI_DEFAULT_FREQ,
  .width        = SPI_DEFAULT_WIDTH,
  .id           = 3,
  .mode         = SPI_DEFAULT_MODE,
  .cs_pin       = CONFIG_ESP32S3_SPI3_CSPIN,
  .mosi_pin     = CONFIG_ESP32S3_SPI3_MOSIPIN,
  .miso_pin     = CONFIG_ESP32S3_SPI3_MISOPIN,
  .clk_pin      = CONFIG_ESP32S3_SPI3_CLKPIN,
  .clk_bit      = SYSTEM_SPI3_CLK_EN,
  .rst_bit      = SYSTEM_SPI3_RST,
  .cs_insig     = FSPICS0_IN_IDX,
  .cs_outsig    = FSPICS0_OUT_IDX,
  .mosi_insig   = FSPID_IN_IDX,
  .mosi_outsig  = FSPID_OUT_IDX,
  .miso_insig   = FSPIQ_IN_IDX,
  .miso_outsig  = FSPIQ_OUT_IDX,
  .clk_insig    = FSPICLK_IN_IDX,
  .clk_outsig   = FSPICLK_OUT_IDX
};

static const struct spi_ops_s esp32s3_spi3_ops =
{
  .lock              = esp32s3_spi_lock,
#ifdef CONFIG_ESP32S3_SPI_UDCS
  .select            = esp32s3_spi3_select,
#else
  .select            = esp32s3_spi_select,
#endif
  .setfrequency      = esp32s3_spi_setfrequency,
  .setmode           = esp32s3_spi_setmode,
  .setbits           = esp32s3_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = esp32s3_spi_hwfeatures,
#endif
  .status            = esp32s3_spi3_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = esp32s3_spi3_cmddata,
#endif
  .send              = esp32s3_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = esp32s3_spi_exchange,
#else
  .sndblock          = esp32s3_spi_sndblock,
  .recvblock         = esp32s3_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = esp32s3_spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct esp32s3_spi_priv_s esp32s3_spi3_priv =
{
  .spi_dev     =
  {
    .ops       = &esp32s3_spi3_ops
  },
  .config      = &esp32s3_spi3_config,
  .refs        = 0,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = 0,
  .actual      = 0,
  .mode        = 0,
  .nbits       = 0
};
#endif /* CONFIG_ESP32S3_SPI3 */

#ifdef CONFIG_ESP32S3_SPI2_DMA

/* SPI DMA RX/TX description */

static struct esp32s3_dmadesc_s dma_rxdesc[SPI_DMA_DESC_NUM];
static struct esp32s3_dmadesc_s dma_txdesc[SPI_DMA_DESC_NUM];

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_spi_set_regbits
 *
 * Description:
 *   Set the bits of the SPI register.
 *
 * Input Parameters:
 *   addr   - Address of the register of interest
 *   bits   - Bits to be set
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void esp32s3_spi_set_regbits(uint32_t addr, uint32_t bits)
{
  uint32_t tmp = getreg32(addr);

  putreg32(tmp | bits, addr);
}

/****************************************************************************
 * Name: esp32s3_spi_clr_regbits
 *
 * Description:
 *   Clear the bits of the SPI register.
 *
 * Input Parameters:
 *   addr   - Address of the register of interest
 *   bits   - Bits to be cleared
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void esp32s3_spi_clr_regbits(uint32_t addr, uint32_t bits)
{
  uint32_t tmp = getreg32(addr);

  putreg32(tmp & ~bits, addr);
}

/****************************************************************************
 * Name: esp32s3_spi_iomux
 *
 * Description:
 *   Check if the option SPI GPIO pins can use IOMUX directly
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *
 * Returned Value:
 *   True if can use IOMUX or false if can't.
 *
 ****************************************************************************/

static inline bool esp32s3_spi_iomux(struct esp32s3_spi_priv_s *priv)
{
  bool mapped = false;
  const struct esp32s3_spi_config_s *cfg = priv->config;

  /* We only need to check SPI2, SPI3 doesn't support IOMUX */

  if (cfg->id == 2)
    {
      if (cfg->mosi_pin == SPI2_IOMUX_MOSIPIN &&
#ifndef CONFIG_ESP32S3_SPI_SWCS
          cfg->cs_pin == SPI2_IOMUX_CSPIN &&
#endif
          cfg->miso_pin == SPI2_IOMUX_MISOPIN &&
          cfg->clk_pin == SPI2_IOMUX_CLKPIN)
        {
          mapped = true;
        }
    }

  return mapped;
}

/****************************************************************************
 * Name: esp32s3_spi_lock
 *
 * Description:
 *   Lock or unlock the SPI device.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   lock   - true: Lock SPI bus, false: unlock SPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the SPI device.
 *
 ****************************************************************************/

static int esp32s3_spi_lock(struct spi_dev_s *dev, bool lock)
{
  int ret;
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

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
 * Name: esp32s3_spi_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete.
 *
 * Input Parameters:
 *   priv - SPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2_DMA
static int esp32s3_spi_sem_waitdone(struct esp32s3_spi_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr, SEC2TICK(10));
}
#endif

/****************************************************************************
 * Name: esp32s3_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select. The implementation of this method
 *   must include handshaking: If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 *   If ESP32S3_SPI_SWCS is disabled, the driver will use hardware CS so that
 *   once transmission is started the hardware selects the device and when
 *   this transmission is done hardware deselects the device automatically.
 *   So, this function will do nothing.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   devid    - Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifndef CONFIG_ESP32S3_SPI_UDCS
static void esp32s3_spi_select(struct spi_dev_s *dev,
                               uint32_t devid, bool selected)
{
#if SPI_HAVE_SWCS
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

  esp32s3_gpiowrite(priv->config->cs_pin, !selected);
#endif

  spiinfo("devid: %08" PRIx32 " CS: %s\n",
          devid, selected ? "select" : "free");
}
#endif

/****************************************************************************
 * Name: esp32s3_spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev       - Device-specific state data
 *   frequency - The requested SPI frequency
 *
 * Returned Value:
 *   Returns the current selected frequency.
 *
 ****************************************************************************/

static uint32_t esp32s3_spi_setfrequency(struct spi_dev_s *dev,
                                         uint32_t frequency)
{
  uint32_t reg_val;
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;
  const uint32_t duty_cycle = 128;

  if (priv->frequency == frequency)
    {
      /* Requested frequency is the same as the current frequency. */

      return priv->actual;
    }

  /* In HW, n, h and l fields range from 1 to 64, pre ranges from 1 to 8K.
   * The value written to register is one lower than the used value.
   */

  if (frequency > ((APB_CLK_FREQ / 4) * 3))
    {
      /* Using APB frequency directly will give us the best result here. */

      reg_val = SPI_CLK_EQU_SYSCLK_M;
      priv->actual = APB_CLK_FREQ;
    }
  else
    {
      /* For best duty cycle resolution, we want n to be as close to 32 as
       * possible, but we also need a pre/n combo that gets us as close as
       * possible to the intended frequency. To do this, we bruteforce n and
       * calculate the best pre to go along with that. If there's a choice
       * between pre/n combos that give the same result, use the one with the
       * higher n.
       */

      int32_t pre;
      int32_t n;
      int32_t h;
      int32_t l;
      int32_t bestn = -1;
      int32_t bestpre = -1;
      int32_t besterr = 0;
      int32_t errval;

      /* Start at n = 2. We need to be able to set h/l so we have at least
       * one high and one low pulse.
       */

      for (n = 2; n <= 64; n++)
        {
          /* Effectively, this does:
           *   pre = round((APB_CLK_FREQ / n) / frequency)
           */

          pre = ((APB_CLK_FREQ / n) + (frequency / 2)) / frequency;

          if (pre <= 0)
            {
              pre = 1;
            }

          if (pre > 16)
            {
              pre = 16;
            }

          errval = abs(APB_CLK_FREQ / (pre * n) - frequency);
          if (bestn == -1 || errval <= besterr)
            {
              besterr = errval;
              bestn = n;
              bestpre = pre;
            }
        }

      n = bestn;
      pre = bestpre;
      l = n;

      /* Effectively, this does:
       *   h = round((duty_cycle * n) / 256)
       */

      h = (duty_cycle * n + 127) / 256;
      if (h <= 0)
        {
          h = 1;
        }

      reg_val = ((l - 1) << SPI_CLKCNT_L_S) |
                ((h - 1) << SPI_CLKCNT_H_S) |
                ((n - 1) << SPI_CLKCNT_N_S) |
                ((pre - 1) << SPI_CLKDIV_PRE_S);

      priv->actual = APB_CLK_FREQ / (n * pre);
    }

  priv->frequency = frequency;

  putreg32(reg_val, SPI_CLOCK_REG(priv->config->id));

  spiinfo("frequency=%" PRIu32 ", actual=%" PRIu32 "\n",
          priv->frequency, priv->actual);

  return priv->actual;
}

/****************************************************************************
 * Name: esp32s3_spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The requested SPI mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_setmode(struct spi_dev_s *dev,
                                enum spi_mode_e mode)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      uint32_t ck_idle_edge;
      uint32_t ck_out_edge;

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            ck_idle_edge = 0;
            ck_out_edge = 0;
            break;

          case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            ck_idle_edge = 0;
            ck_out_edge = 1;
            break;

          case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            ck_idle_edge = 1;
            ck_out_edge = 1;
            break;

          case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            ck_idle_edge = 1;
            ck_out_edge = 0;
            break;

          default:
            spierr("Invalid mode: %d\n", mode);
            DEBUGPANIC();
            return;
        }

      esp32s3_spi_clr_regbits(SPI_MISC_REG(priv->config->id),
                              SPI_CK_IDLE_EDGE_M);
      esp32s3_spi_set_regbits(SPI_MISC_REG(priv->config->id),
                              VALUE_MASK(ck_idle_edge, SPI_CK_IDLE_EDGE));

      esp32s3_spi_clr_regbits(SPI_USER_REG(priv->config->id),
                              SPI_CK_OUT_EDGE_M);
      esp32s3_spi_set_regbits(SPI_USER_REG(priv->config->id),
                              VALUE_MASK(ck_out_edge, SPI_CK_OUT_EDGE));

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: esp32s3_spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  priv->nbits = nbits;
}

/****************************************************************************
 * Name: esp32s3_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
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
static int esp32s3_spi_hwfeatures(struct spi_dev_s *dev,
                                  spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp32s3_spi_dma_exchange
 *
 * Description:
 *   Exchange a block of data from SPI by DMA.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2_DMA
static void esp32s3_spi_dma_exchange(struct esp32s3_spi_priv_s *priv,
                                     const void *txbuffer,
                                     void *rxbuffer,
                                     uint32_t nwords)
{
  const uint32_t total = nwords * (priv->nbits / 8);
  const int32_t channel = priv->dma_channel;
  uint32_t bytes = total;
  uint32_t n;
  uint8_t *tp;
  uint8_t *rp;

  DEBUGASSERT((txbuffer != NULL) || (rxbuffer != NULL));

  spiinfo("nwords=%" PRIu32 "\n", nwords);

  tp = (uint8_t *)txbuffer;
  rp = (uint8_t *)rxbuffer;

  if (tp == NULL)
    {
      tp = rp;
    }

  esp32s3_spi_set_regbits(SPI_DMA_INT_CLR_REG(priv->config->id),
                          SPI_TRANS_DONE_INT_CLR_M);

  esp32s3_spi_set_regbits(SPI_DMA_INT_ENA_REG(priv->config->id),
                          SPI_TRANS_DONE_INT_ENA_M);

  while (bytes != 0)
    {
      /* Reset SPI DMA TX FIFO */

      esp32s3_spi_set_regbits(SPI_DMA_CONF_REG(priv->config->id),
                              SPI_DMA_RESET_MASK);
      esp32s3_spi_clr_regbits(SPI_DMA_CONF_REG(priv->config->id),
                              SPI_DMA_RESET_MASK);

      /* Enable SPI DMA TX */

      esp32s3_spi_set_regbits(SPI_DMA_CONF_REG(priv->config->id),
                              SPI_DMA_TX_ENA_M);

      n = esp32s3_dma_setup(channel, true, dma_txdesc, SPI_DMA_DESC_NUM,
                            tp, bytes);
      esp32s3_dma_enable(channel, true);

      putreg32((n * 8 - 1), SPI_MS_DLEN_REG(priv->config->id));
      esp32s3_spi_set_regbits(SPI_USER_REG(priv->config->id),
                              SPI_USR_MOSI_M);

      tp += n;

      if (rp != NULL)
        {
          /* Enable SPI DMA RX */

          esp32s3_spi_set_regbits(SPI_DMA_CONF_REG(priv->config->id),
                                  SPI_DMA_RX_ENA_M);

          esp32s3_dma_setup(channel, false, dma_rxdesc, SPI_DMA_DESC_NUM,
                            rp, bytes);
          esp32s3_dma_enable(channel, false);

          esp32s3_spi_set_regbits(SPI_USER_REG(priv->config->id),
                                  SPI_USR_MISO_M);

          rp += n;
        }
      else
        {
          esp32s3_spi_clr_regbits(SPI_USER_REG(priv->config->id),
                                  SPI_USR_MISO_M);
        }

      /* Trigger start of user-defined transaction for master. */

      esp32s3_spi_set_regbits(SPI_CMD_REG(priv->config->id),
                              SPI_UPDATE_M);

      while ((getreg32(SPI_CMD_REG(priv->config->id)) & SPI_UPDATE_M) != 0)
        {
          ;
        }

      esp32s3_spi_set_regbits(SPI_CMD_REG(priv->config->id), SPI_USR_M);

      esp32s3_spi_sem_waitdone(priv);

      bytes -= n;
    }

  esp32s3_spi_clr_regbits(SPI_DMA_INT_ENA_REG(priv->config->id),
                          SPI_TRANS_DONE_INT_ENA_M);
}
#endif

/****************************************************************************
 * Name: esp32s3_spi_poll_send
 *
 * Description:
 *   Send one word on SPI by polling mode.
 *
 * Input Parameters:
 *   priv - SPI private state data
 *   wd   - The word to send. The size of the data is determined by the
 *          number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value.
 *
 ****************************************************************************/

static uint32_t esp32s3_spi_poll_send(struct esp32s3_spi_priv_s *priv,
                                      uint32_t wd)
{
  uint32_t val;

  putreg32((priv->nbits - 1), SPI_MS_DLEN_REG(priv->config->id));

  putreg32(wd, SPI_W0_REG(priv->config->id));

  /* Trigger start of user-defined transaction for master. */

  esp32s3_spi_set_regbits(SPI_CMD_REG(priv->config->id), SPI_UPDATE_M);

  while ((getreg32(SPI_CMD_REG(priv->config->id)) & SPI_UPDATE_M) != 0)
    {
      ;
    }

  esp32s3_spi_set_regbits(SPI_CMD_REG(priv->config->id), SPI_USR_M);

  /* Wait for the user-defined transaction to finish. */

  while ((getreg32(SPI_CMD_REG(priv->config->id)) & SPI_USR_M) != 0)
    {
      ;
    }

  val = getreg32(SPI_W0_REG(priv->config->id));

  spiinfo("send=0x%" PRIx32 " and recv=0x%" PRIx32 "\n", wd, val);

  return val;
}

/****************************************************************************
 * Name: esp32s3_spi_send
 *
 * Description:
 *   Send one word on SPI.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send. The size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value.
 *
 ****************************************************************************/

static uint32_t esp32s3_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

  return esp32s3_spi_poll_send(priv, wd);
}

/****************************************************************************
 * Name: esp32s3_spi_poll_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - The length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface. If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_poll_exchange(struct esp32s3_spi_priv_s *priv,
                                      const void *txbuffer,
                                      void *rxbuffer,
                                      size_t nwords)
{
  const uint32_t total_bytes = nwords * (priv->nbits / 8);
  uintptr_t bytes_remaining = total_bytes;
  const uint8_t *tp = (const uint8_t *)txbuffer;
  uint8_t *rp = (uint8_t *)rxbuffer;

  while (bytes_remaining != 0)
    {
      /* Initialize data_buf_reg with the address of the first data buffer
       * register (W0).
       */

      uintptr_t data_buf_reg = SPI_W0_REG(priv->config->id);
      uint32_t transfer_size = MIN(SPI_MAX_BUF_SIZE, bytes_remaining);

      /* Write data words to data buffer registers.
       * SPI peripheral contains 16 registers (W0 - W15).
       */

      for (int i = 0 ; i < transfer_size; i += sizeof(uintptr_t))
        {
          uint32_t w_wd = UINT32_MAX;

          if (tp != NULL)
            {
              memcpy(&w_wd, tp, sizeof(uintptr_t));

              tp += sizeof(uintptr_t);
            }

          putreg32(w_wd, data_buf_reg);

          spiinfo("send=0x%" PRIx32 " data_reg=0x%" PRIxPTR "\n",
                  w_wd, data_buf_reg);

          /* Update data_buf_reg to point to the next data buffer register. */

          data_buf_reg += sizeof(uintptr_t);
        }

      esp32s3_spi_set_regbits(SPI_USER_REG(priv->config->id),
                              SPI_USR_MOSI_M);

      if (rp == NULL)
        {
          esp32s3_spi_clr_regbits(SPI_USER_REG(priv->config->id),
                                  SPI_USR_MISO_M);
        }
      else
        {
          esp32s3_spi_set_regbits(SPI_USER_REG(priv->config->id),
                                  SPI_USR_MISO_M);
        }

      putreg32((transfer_size * 8) - 1,
               SPI_MS_DLEN_REG(priv->config->id));

      /* Trigger start of user-defined transaction for master. */

      esp32s3_spi_set_regbits(SPI_CMD_REG(priv->config->id),
                              SPI_UPDATE_M);

      while ((getreg32(SPI_CMD_REG(priv->config->id)) & SPI_UPDATE_M) != 0)
        {
          ;
        }

      esp32s3_spi_set_regbits(SPI_CMD_REG(priv->config->id), SPI_USR_M);

      /* Wait for the user-defined transaction to finish. */

      while ((getreg32(SPI_CMD_REG(priv->config->id)) & SPI_USR_M) != 0)
        {
          ;
        }

      if (rp != NULL)
        {
          /* Set data_buf_reg with the address of the first data buffer
           * register (W0).
           */

          data_buf_reg = SPI_W0_REG(priv->config->id);

          /* Read received data words from SPI data buffer registers. */

          for (int i = 0 ; i < transfer_size; i += sizeof(uintptr_t))
            {
              uint32_t r_wd = getreg32(data_buf_reg);

              spiinfo("recv=0x%" PRIx32 " data_reg=0x%" PRIxPTR "\n",
                      r_wd, data_buf_reg);

              memcpy(rp, &r_wd, sizeof(uintptr_t));

              rp += sizeof(uintptr_t);

              /* Update data_buf_reg to point to the next data buffer
               * register.
               */

              data_buf_reg += sizeof(uintptr_t);
            }
        }

      bytes_remaining -= transfer_size;
    }
}

/****************************************************************************
 * Name: esp32s3_spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - The length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface. If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_exchange(struct spi_dev_s *dev,
                                 const void *txbuffer,
                                 void *rxbuffer,
                                 size_t nwords)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

#ifdef CONFIG_ESP32S3_SPI2_DMA
  size_t thld = CONFIG_ESP32S3_SPI2_DMATHRESHOLD;

  if (nwords > thld)
    {
      esp32s3_spi_dma_exchange(priv, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      esp32s3_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: esp32s3_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - The length of data to send from the buffer in number of
 *              words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface. If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_sndblock(struct spi_dev_s *dev,
                                 const void *txbuffer,
                                 size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);

  esp32s3_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: esp32s3_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - The length of data that can be received in the buffer in
 *              number of words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface. If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_recvblock(struct spi_dev_s *dev,
                                  void *rxbuffer,
                                  size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);

  esp32s3_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: esp32s3_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int esp32s3_spi_trigger(struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp32s3_spi_dma_init
 *
 * Description:
 *   Initialize ESP32-S3 SPI connection to GDMA engine.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2_DMA
void esp32s3_spi_dma_init(struct spi_dev_s *dev)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

  /* Enable GDMA clock for the SPI peripheral */

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, priv->config->dma_clk_bit);

  /* Reset GDMA for the SPI peripheral */

  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, priv->config->dma_rst_bit, 0);

  /* Initialize GDMA controller */

  esp32s3_dma_init();

  /* Request a GDMA channel for SPI peripheral */

  priv->dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_SPI2, 1, 1,
                                          true);
  if (priv->dma_channel < 0)
    {
      spierr("Failed to allocate GDMA channel\n");

      DEBUGPANIC();
    }

  /* Disable segment transaction mode for SPI Master */

  putreg32((SPI_SLV_RX_SEG_TRANS_CLR_EN_M | SPI_SLV_TX_SEG_TRANS_CLR_EN_M),
           SPI_DMA_CONF_REG(priv->config->id));
}
#endif

/****************************************************************************
 * Name: esp32s3_spi_init
 *
 * Description:
 *   Initialize ESP32-S3 SPI hardware interface.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_init(struct spi_dev_s *dev)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;
  const struct esp32s3_spi_config_s *config = priv->config;
  uint32_t regval;

  esp32s3_gpiowrite(config->cs_pin, true);
  esp32s3_gpiowrite(config->mosi_pin, true);
  esp32s3_gpiowrite(config->miso_pin, true);
  esp32s3_gpiowrite(config->clk_pin, true);

#if SPI_HAVE_SWCS
  esp32s3_configgpio(config->cs_pin, OUTPUT_FUNCTION_1);
  esp32s3_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif

  /* SPI3 doesn't have IOMUX, if SPI3 is enabled use GPIO Matrix for both */

  if (esp32s3_spi_iomux(priv))
    {
#if !SPI_HAVE_SWCS
      esp32s3_configgpio(config->cs_pin, OUTPUT_FUNCTION_5);
      esp32s3_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif
      esp32s3_configgpio(config->mosi_pin, OUTPUT_FUNCTION_5);
      esp32s3_gpio_matrix_out(config->mosi_pin, SIG_GPIO_OUT_IDX, 0, 0);

      esp32s3_configgpio(config->miso_pin, INPUT_FUNCTION_5 | PULLUP);
      esp32s3_gpio_matrix_out(config->miso_pin, SIG_GPIO_OUT_IDX, 0, 0);

      esp32s3_configgpio(config->clk_pin, OUTPUT_FUNCTION_5);
      esp32s3_gpio_matrix_out(config->clk_pin, SIG_GPIO_OUT_IDX, 0, 0);
    }
  else
    {
#if !SPI_HAVE_SWCS
      esp32s3_configgpio(config->cs_pin, OUTPUT);
      esp32s3_gpio_matrix_out(config->cs_pin, config->cs_outsig, 0, 0);
#endif
      esp32s3_configgpio(config->mosi_pin, OUTPUT);
      esp32s3_gpio_matrix_out(config->mosi_pin, config->mosi_outsig, 0, 0);

      esp32s3_configgpio(config->miso_pin, INPUT | PULLUP);
      esp32s3_gpio_matrix_in(config->miso_pin, config->miso_insig, 0);

      esp32s3_configgpio(config->clk_pin, OUTPUT);
      esp32s3_gpio_matrix_out(config->clk_pin, config->clk_outsig, 0, 0);
    }

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, config->clk_bit);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, config->rst_bit, 0);

  regval = SPI_DOUTDIN_M | SPI_USR_MISO_M | SPI_USR_MOSI_M | SPI_CS_HOLD_M;
  putreg32(regval, SPI_USER_REG(priv->config->id));
  putreg32(0, SPI_USER1_REG(priv->config->id));
  putreg32(0, SPI_SLAVE_REG(priv->config->id));
  putreg32(SPI_CS1_DIS_M | SPI_CS2_DIS_M,
           SPI_MISC_REG(priv->config->id));

  regval = SPI_CLK_EN_M | SPI_MST_CLK_ACTIVE_M | SPI_MST_CLK_SEL_M;
  putreg32(regval, SPI_CLK_GATE_REG(priv->config->id));

#if SPI_HAVE_SWCS
  esp32s3_spi_set_regbits(SPI_MISC_REG(priv->config->id), SPI_CS0_DIS_M);
#endif

  putreg32(0, SPI_CTRL_REG(priv->config->id));
  putreg32(VALUE_MASK(0, SPI_CS_HOLD_TIME),
           SPI_USER1_REG(priv->config->id));

#ifdef CONFIG_ESP32S3_SPI2_DMA
  esp32s3_spi_dma_init(dev);
#endif

  esp32s3_spi_setfrequency(dev, config->clk_freq);
  esp32s3_spi_setbits(dev, config->width);
  esp32s3_spi_setmode(dev, config->mode);
}

/****************************************************************************
 * Name: esp32s3_spi_deinit
 *
 * Description:
 *   Deinitialize ESP32-S3 SPI hardware interface.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_spi_deinit(struct spi_dev_s *dev)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

#ifdef CONFIG_ESP32S3_SPI2_DMA
  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, priv->config->dma_clk_bit, 0);
#endif

  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, priv->config->clk_bit);
  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, priv->config->clk_bit, 0);

  priv->frequency = 0;
  priv->actual    = 0;
  priv->mode      = SPIDEV_MODE0;
  priv->nbits     = 0;
}

/****************************************************************************
 * Name: esp32s3_spi_interrupt
 *
 * Description:
 *   Common SPI DMA interrupt handler.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - SPI controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2_DMA
static int esp32s3_spi_interrupt(int irq, void *context, void *arg)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)arg;

  /* Write 1 to clear interrupt bit */

  esp32s3_spi_set_regbits(SPI_DMA_INT_CLR_REG(priv->config->id),
                          SPI_TRANS_DONE_INT_CLR_M);
  nxsem_post(&priv->sem_isr);

  return 0;
}
#endif

/****************************************************************************
 * Name: esp32s3_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus.
 *
 * Input Parameters:
 *   port     - Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s *esp32s3_spibus_initialize(int port)
{
  struct spi_dev_s *spi_dev;
  struct esp32s3_spi_priv_s *priv;

  switch (port)
    {
#ifdef CONFIG_ESP32S3_SPI2
      case ESP32S3_SPI2:
        priv = &esp32s3_spi2_priv;
        break;
#endif
#ifdef CONFIG_ESP32S3_SPI3
      case ESP32S3_SPI3:
        priv = &esp32s3_spi3_priv;
        break;
#endif
      default:
        return NULL;
    }

  spi_dev = (struct spi_dev_s *)priv;

  nxmutex_lock(&priv->lock);
  if (priv->refs != 0)
    {
      priv->refs++;
      nxmutex_unlock(&priv->lock);
      return spi_dev;
    }

#ifdef CONFIG_ESP32S3_SPI2_DMA
  /* If a CPU Interrupt was previously allocated, then deallocate it */

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(priv->config->irq);
      esp32s3_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
      irq_detach(priv->config->irq);

      priv->cpuint = -ENOMEM;
      priv->cpu = -ENODEV;
    }

  /* Set up to receive peripheral interrupts on the current CPU */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, priv->config->periph,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  /* Attach and enable the IRQ */

  if (irq_attach(priv->config->irq, esp32s3_spi_interrupt, priv) != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32s3_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the SPI device. */

  up_enable_irq(priv->config->irq);
#endif

  esp32s3_spi_init(spi_dev);
  priv->refs++;

  nxmutex_unlock(&priv->lock);
  return spi_dev;
}

/****************************************************************************
 * Name: esp32s3_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_spibus_uninitialize(struct spi_dev_s *dev)
{
  struct esp32s3_spi_priv_s *priv = (struct esp32s3_spi_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs != 0)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

#ifdef CONFIG_ESP32S3_SPI2_DMA
  up_disable_irq(priv->config->irq);
  esp32s3_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
  irq_detach(priv->config->irq);

  priv->cpuint = -ENOMEM;

#endif

  esp32s3_spi_deinit(dev);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESP32S3_SPI */
