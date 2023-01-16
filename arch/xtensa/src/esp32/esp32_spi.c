/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spi.c
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

#ifdef CONFIG_ESP32_SPI

#include <debug.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "esp32_spi.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"
#include "esp32_dma.h"

#include "xtensa.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPI2
#  if defined(CONFIG_ESP32_SPI2_MASTER_IO_RW)
#    define ESP32_SPI2_IO   ESP32_SPI_IO_RW
#  elif defined(CONFIG_ESP32_SPI2_MASTER_IO_RO)
#    define ESP32_SPI2_IO   ESP32_SPI_IO_R
#  elif defined(CONFIG_ESP32_SPI2_MASTER_IO_WO)
#    define ESP32_SPI2_IO   ESP32_SPI_IO_W
#  endif
#endif

#ifdef CONFIG_ESP32_SPI3
#  if defined(CONFIG_ESP32_SPI3_MASTER_IO_RW)
#    define ESP32_SPI3_IO   ESP32_SPI_IO_RW
#  elif defined(CONFIG_ESP32_SPI3_MASTER_IO_RO)
#    define ESP32_SPI3_IO   ESP32_SPI_IO_R
#  elif defined(CONFIG_ESP32_SPI3_MASTER_IO_WO)
#    define ESP32_SPI3_IO   ESP32_SPI_IO_W
#  endif
#endif

/* SPI DMA RX/TX description number */

#define SPI_DMADESC_NUM     (CONFIG_SPI_DMADESC_NUM)

/* SPI DMA channel number */

#define SPI_DMA_CHANNEL_MAX (2)

/* SPI DMA reset before exchange */

#define SPI_DMA_RESET_MASK  (SPI_AHBM_RST_M | SPI_AHBM_FIFO_RST_M | \
                             SPI_OUT_RST_M | SPI_IN_RST_M)

/* SPI Default speed (limited by clock divider) */

#define SPI_FREQ_DEFAULT  (400000)

/* Helper for applying the mask for a given register field.
 * Mask is determined by the macros suffixed with _V and _S from the
 * peripheral register description.
 */

#define VALUE_MASK(_val, _field) ((_val & (_field##_V)) << (_field##_S))

/* SPI Maximum buffer size in bytes */

#define SPI_MAX_BUF_SIZE (64)

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device hardware configuration */

struct esp32_spi_config_s
{
  uint32_t id;                /* SPI instance */

  uint32_t clk_freq;          /* SPI clock frequency */
  enum spi_mode_e mode;       /* SPI default mode */

  uint8_t cs_pin;             /* GPIO configuration for CS */
  uint8_t mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t miso_pin;           /* GPIO configuration for MISO */
  uint8_t clk_pin;            /* GPIO configuration for CLK */

  uint8_t periph;             /* peripher ID */
  uint8_t irq;                /* Interrupt ID */

  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* SPI reset bit */

  bool use_dma;               /* Use DMA */
  uint8_t dma_chan_s;         /* DMA channel register shift */
  uint8_t dma_chan;           /* DMA channel */
  uint32_t dma_clk_bit;       /* DMA clock enable bit */
  uint32_t dma_rst_bit;       /* DMA reset bit */

  uint32_t cs_insig;          /* SPI CS input signal index */
  uint32_t cs_outsig;         /* SPI CS output signal index */
  uint32_t mosi_insig;        /* SPI MOSI input signal index */
  uint32_t mosi_outsig;       /* SPI MOSI output signal index */
  uint32_t miso_insig;        /* SPI MISO input signal index */
  uint32_t miso_outsig;       /* SPI MISO output signal index */
  uint32_t clk_insig;         /* SPI CLK input signal index */
  uint32_t clk_outsig;        /* SPI CLK output signal index */

  uint32_t flags;             /* SPI supports features */
};

struct esp32_spi_priv_s
{
  /* Externally visible part of the SPI interface */

  struct spi_dev_s spi_dev;

  /* Port configuration */

  const struct esp32_spi_config_s *config;

  int refs;                    /* Reference count */

  /* Held while chip is selected for mutual exclusion */

  mutex_t          lock;

  /* Interrupt wait semaphore */

  sem_t            sem_isr;

  int              cpuint;      /* SPI interrupt ID */
  uint8_t          cpu;         /* CPU ID */

  uint32_t         frequency;   /* Requested clock frequency */
  uint32_t         actual;      /* Actual clock frequency */

  enum spi_mode_e  mode;        /* Actual SPI hardware mode */

  /* Actual SPI send/receive bits once transmission */

  uint8_t          nbits;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32_spi_lock(struct spi_dev_s *dev, bool lock);
#ifndef CONFIG_ESP32_SPI_UDCS
static void esp32_spi_select(struct spi_dev_s *dev,
                             uint32_t devid, bool selected);
#endif
static uint32_t esp32_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void esp32_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode);
static void esp32_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int esp32_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint32_t esp32_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void esp32_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void esp32_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords);
static void esp32_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int esp32_spi_trigger(struct spi_dev_s *dev);
#endif
static void esp32_spi_init(struct spi_dev_s *dev);
static void esp32_spi_deinit(struct spi_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPI2
static const struct esp32_spi_config_s esp32_spi2_config =
{
  .id           = 2,
  .clk_freq     = SPI_FREQ_DEFAULT,
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI2_CLKPIN,
  .periph       = ESP32_PERIPH_SPI2,
  .irq          = ESP32_IRQ_SPI2,
  .clk_bit      = DPORT_SPI_CLK_EN_2,
  .rst_bit      = DPORT_SPI_RST_2,
#ifdef CONFIG_ESP32_SPI2_DMA
  .use_dma      = true,
#else
  .use_dma      = false,
#endif
  .dma_chan_s   = 2,
  .dma_chan     = 1,
  .dma_clk_bit  = DPORT_SPI_DMA_CLK_EN,
  .dma_rst_bit  = DPORT_SPI_DMA_RST,
  .cs_insig     = HSPICS0_IN_IDX,
  .cs_outsig    = HSPICS0_OUT_IDX,
  .mosi_insig   = HSPID_IN_IDX,
  .mosi_outsig  = HSPID_OUT_IDX,
  .miso_insig   = HSPIQ_IN_IDX,
  .miso_outsig  = HSPIQ_OUT_IDX,
  .clk_insig    = HSPICLK_IN_IDX,
  .clk_outsig   = HSPICLK_OUT_IDX,
  .flags        = ESP32_SPI2_IO
};

static const struct spi_ops_s esp32_spi2_ops =
{
  .lock              = esp32_spi_lock,
#ifdef CONFIG_ESP32_SPI_UDCS
  .select            = esp32_spi2_select,
#else
  .select            = esp32_spi_select,
#endif
  .setfrequency      = esp32_spi_setfrequency,
  .setmode           = esp32_spi_setmode,
  .setbits           = esp32_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = esp32_spi_hwfeatures,
#endif
  .status            = esp32_spi2_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = esp32_spi2_cmddata,
#endif
  .send              = esp32_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = esp32_spi_exchange,
#else
  .sndblock          = esp32_spi_sndblock,
  .recvblock         = esp32_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = esp32_spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct esp32_spi_priv_s esp32_spi2_priv =
{
  .spi_dev =
  {
    .ops   = &esp32_spi2_ops
  },
  .config  = &esp32_spi2_config,
  .lock    = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32_SPI2 */

#ifdef CONFIG_ESP32_SPI3
static const struct esp32_spi_config_s esp32_spi3_config =
{
  .id           = 3,
  .clk_freq     = SPI_FREQ_DEFAULT,
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI3_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI3_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI3_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI3_CLKPIN,
  .periph       = ESP32_PERIPH_SPI3,
  .irq          = ESP32_IRQ_SPI3,
  .clk_bit      = DPORT_SPI_CLK_EN,
  .rst_bit      = DPORT_SPI_RST,
#ifdef CONFIG_ESP32_SPI3_DMA
  .use_dma      = true,
#else
  .use_dma      = false,
#endif
  .dma_chan_s   = 4,
  .dma_chan     = 2,
  .dma_clk_bit  = DPORT_SPI_DMA_CLK_EN,
  .dma_rst_bit  = DPORT_SPI_DMA_RST,
  .cs_insig     = VSPICS0_IN_IDX,
  .cs_outsig    = VSPICS0_OUT_IDX,
  .mosi_insig   = VSPID_IN_IDX,
  .mosi_outsig  = VSPID_OUT_IDX,
  .miso_insig   = VSPIQ_IN_IDX,
  .miso_outsig  = VSPIQ_OUT_IDX,
  .clk_insig    = VSPICLK_IN_IDX,
  .clk_outsig   = VSPICLK_OUT_MUX_IDX,
  .flags        = ESP32_SPI3_IO
};

static const struct spi_ops_s esp32_spi3_ops =
{
  .lock              = esp32_spi_lock,
#ifdef CONFIG_ESP32_SPI_UDCS
  .select            = esp32_spi3_select,
#else
  .select            = esp32_spi_select,
#endif
  .setfrequency      = esp32_spi_setfrequency,
  .setmode           = esp32_spi_setmode,
  .setbits           = esp32_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = esp32_spi_hwfeatures,
#endif
  .status            = esp32_spi3_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = esp32_spi3_cmddata,
#endif
  .send              = esp32_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = esp32_spi_exchange,
#else
  .sndblock          = esp32_spi_sndblock,
  .recvblock         = esp32_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = esp32_spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct esp32_spi_priv_s esp32_spi3_priv =
{
  .spi_dev =
  {
    .ops = &esp32_spi3_ops
  },
  .config  = &esp32_spi3_config,
  .lock    = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32_SPI3 */

/* SPI DMA RX/TX description */

struct esp32_dmadesc_s s_dma_rxdesc[SPI_DMA_CHANNEL_MAX][SPI_DMADESC_NUM];
struct esp32_dmadesc_s s_dma_txdesc[SPI_DMA_CHANNEL_MAX][SPI_DMADESC_NUM];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spi_set_regbits
 *
 * Description:
 *   Set the bits of the SPI register
 *
 * Input Parameters:
 *   addr   - Address of the register of interest
 *   bits   - Bits to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_set_regbits(uint32_t addr, uint32_t bits)
{
  uint32_t tmp = getreg32(addr);

  putreg32(tmp | bits, addr);
}

/****************************************************************************
 * Name: esp32_spi_reset_regbits
 *
 * Description:
 *   Clear the bits of the SPI register
 *
 * Input Parameters:
 *   addr   - Address of the register of interest
 *   bits   - Bits to be cleared
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_reset_regbits(uint32_t addr, uint32_t bits)
{
  uint32_t tmp = getreg32(addr);

  putreg32(tmp & (~bits), addr);
}

/****************************************************************************
 * Name: esp32_spi_iomux
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

static inline bool esp32_spi_iomux(struct esp32_spi_priv_s *priv)
{
  bool mapped = false;
  const struct esp32_spi_config_s *cfg = priv->config;

  if (cfg->id == 2)
    {
      if ((!(cfg->flags & ESP32_SPI_IO_W) ||
           cfg->mosi_pin == SPI2_IOMUX_MOSIPIN) &&

#ifndef CONFIG_ESP32_SPI_SWCS
          cfg->cs_pin == SPI2_IOMUX_CSPIN &&
#endif
          (!(cfg->flags & ESP32_SPI_IO_R) ||
           cfg->miso_pin == SPI2_IOMUX_MISOPIN) &&

          cfg->clk_pin == SPI2_IOMUX_CLKPIN)
        {
          mapped = true;
        }
    }
  else if (cfg->id == 3)
    {
      if ((!(cfg->flags & ESP32_SPI_IO_W) ||
           cfg->mosi_pin == SPI3_IOMUX_MOSIPIN) &&

#ifndef CONFIG_ESP32_SPI_SWCS
          cfg->cs_pin == SPI3_IOMUX_CSPIN &&
#endif
          (!(cfg->flags & ESP32_SPI_IO_R) ||
           cfg->miso_pin == SPI3_IOMUX_MISOPIN) &&

          cfg->clk_pin == SPI3_IOMUX_CLKPIN)
        {
          mapped = true;
        }
    }

  return mapped;
}

/****************************************************************************
 * Name: esp32_spi_lock
 *
 * Description:
 *   Lock or unlock the SPI device
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   lock   - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the SPI device
 *
 ****************************************************************************/

static int esp32_spi_lock(struct spi_dev_s *dev, bool lock)
{
  int ret;
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;

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
 * Name: esp32_spi_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

static int esp32_spi_sem_waitdone(struct esp32_spi_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr, SEC2TICK(10));
}

/****************************************************************************
 * Name: esp32_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 *   If disable ESP32_SPI_SWCS, driver will use hardware CS so that when
 *   once transmission is started, hardware select the device and when this
 *   transmission is done, hardware deselect the device automatically. And
 *   the function will do nothing.
 *
 * Input Parameters:
 *   priv     - Private SPI device structure
 *   devid    - Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_ESP32_SPI_UDCS
static void esp32_spi_select(struct spi_dev_s *dev,
                             uint32_t devid, bool selected)
{
#ifdef CONFIG_ESP32_SPI_SWCS
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;
  bool value = selected ? false : true;

  esp32_gpiowrite(priv->config->cs_pin, value);
#endif

  spiinfo("devid: %08" PRIx32 " CS: %s\n",
          devid, selected ? "select" : "free");
}
#endif

/****************************************************************************
 * Name: esp32_spi_setfrequency
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

static uint32_t esp32_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  uint32_t reg_val;
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;
  const uint32_t duty_cycle = 128;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency. Return the actual. */

      return priv->actual;
    }

  if (frequency > ((APB_CLK_FREQ / 4) * 3))
    {
      reg_val = SPI_CLK_EQU_SYSCLK_M;
      priv->actual = APB_CLK_FREQ;
    }
  else
    {
      int pre;
      int n;
      int h;
      int l;
      int bestn = -1;
      int bestpre = -1;
      int besterr = 0;
      int errval;

      for (n = 2; n <= 64; n++)
        {
          pre = ((APB_CLK_FREQ / n) + (frequency / 2)) / frequency;

          if (pre <= 0)
            {
              pre = 1;
            }

          if (pre > 8192)
            {
              pre = 8192;
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

  spiinfo("frequency=%d, actual=%d\n", priv->frequency, priv->actual);

  return priv->actual;
}

/****************************************************************************
 * Name: esp32_spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  uint32_t ck_idle_edge;
  uint32_t ck_out_edge;
  uint32_t delay_mode;
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          ck_idle_edge = 0;
          ck_out_edge = 0;
          delay_mode = 0;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          ck_idle_edge = 0;
          ck_out_edge = 1;
          delay_mode = 2;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          ck_idle_edge = 1;
          ck_out_edge = 1;
          delay_mode = 2;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          ck_idle_edge = 1;
          ck_out_edge = 0;
          delay_mode = 0;
          break;

        default:
          return;
        }

      const uint32_t id = priv->config->id;

      esp32_spi_reset_regbits(SPI_PIN_REG(id), SPI_CK_IDLE_EDGE_M);
      esp32_spi_set_regbits(SPI_PIN_REG(id),
                            VALUE_MASK(ck_idle_edge, SPI_CK_IDLE_EDGE));

      esp32_spi_reset_regbits(SPI_USER_REG(id), SPI_CK_OUT_EDGE_M);
      esp32_spi_set_regbits(SPI_USER_REG(id),
                            VALUE_MASK(ck_out_edge, SPI_CK_OUT_EDGE));

      esp32_spi_reset_regbits(SPI_CTRL2_REG(id),
                              SPI_MISO_DELAY_MODE_M |
                              SPI_MISO_DELAY_NUM_M |
                              SPI_MOSI_DELAY_NUM_M |
                              SPI_MOSI_DELAY_MODE_M);
      esp32_spi_set_regbits(SPI_CTRL2_REG(id),
                            VALUE_MASK(delay_mode, SPI_MISO_DELAY_MODE) |
                            VALUE_MASK(delay_mode, SPI_MOSI_DELAY_MODE));

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: esp32_spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  priv->nbits = nbits;
}

/****************************************************************************
 * Name: esp32_spi_hwfeatures
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
static int esp32_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp32_spi_dma_exchange
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
 *   None
 *
 ****************************************************************************/

static void esp32_spi_dma_exchange(struct esp32_spi_priv_s *priv,
                                   const void *txbuffer,
                                   void *rxbuffer,
                                   uint32_t nwords)
{
  const uint32_t total = nwords * (priv->nbits / 8);
  uint32_t bytes = total;
  uint8_t *tp;
  uint8_t *rp;
  uint32_t n;
  uint32_t regval;
  struct esp32_dmadesc_s *dma_tx_desc;
  struct esp32_dmadesc_s *dma_rx_desc;
#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  uint8_t *alloctp = NULL;
  uint8_t *allocrp;
#endif

  /* Define these constants outside transfer loop to avoid wasting CPU time
   * with register offset calculation.
   */

  const uint32_t id = priv->config->id;
  const uint8_t dma_desc_idx = priv->config->dma_chan - 1;
  const uintptr_t spi_dma_in_link_reg = SPI_DMA_IN_LINK_REG(id);
  const uintptr_t spi_dma_out_link_reg = SPI_DMA_OUT_LINK_REG(id);
  const uintptr_t spi_slave_reg = SPI_SLAVE_REG(id);
  const uintptr_t spi_dma_conf_reg = SPI_DMA_CONF_REG(id);
  const uintptr_t spi_mosi_dlen_reg = SPI_MOSI_DLEN_REG(id);
  const uintptr_t spi_miso_dlen_reg = SPI_MISO_DLEN_REG(id);
  const uintptr_t spi_user_reg = SPI_USER_REG(id);
  const uintptr_t spi_cmd_reg = SPI_CMD_REG(id);

  DEBUGASSERT((txbuffer != NULL) || (rxbuffer != NULL));

  /* If the buffer comes from PSRAM, allocate a new one from DRAM */

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  if (esp32_ptr_extram(txbuffer))
    {
      alloctp = xtensa_imm_malloc(total);
      DEBUGASSERT(alloctp != NULL);
      memcpy(alloctp, txbuffer, total);
      tp = alloctp;
    }
  else
#endif
    {
      tp = (uint8_t *)txbuffer;
    }

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  if (esp32_ptr_extram(rxbuffer))
    {
      allocrp = xtensa_imm_malloc(total);
      DEBUGASSERT(allocrp != NULL);
      rp = allocrp;
    }
  else
#endif
    {
      rp = (uint8_t *)rxbuffer;
    }

  if (tp == NULL)
    {
      tp = rp;
    }

  dma_tx_desc = s_dma_txdesc[dma_desc_idx];
  dma_rx_desc = s_dma_rxdesc[dma_desc_idx];

  esp32_spi_reset_regbits(spi_slave_reg, SPI_TRANS_DONE_M);
  esp32_spi_set_regbits(spi_slave_reg, SPI_INT_EN_M);

  while (bytes != 0)
    {
      putreg32(0, spi_dma_in_link_reg);
      putreg32(0, spi_dma_out_link_reg);

      esp32_spi_set_regbits(spi_slave_reg, SPI_SYNC_RESET_M);
      esp32_spi_reset_regbits(spi_slave_reg, SPI_SYNC_RESET_M);

      esp32_spi_set_regbits(spi_dma_conf_reg, SPI_DMA_RESET_MASK);
      esp32_spi_reset_regbits(spi_dma_conf_reg, SPI_DMA_RESET_MASK);

      n = esp32_dma_init(dma_tx_desc, SPI_DMADESC_NUM, tp, bytes);

      regval  = VALUE_MASK((uintptr_t)dma_tx_desc, SPI_OUTLINK_ADDR);
      regval |= SPI_OUTLINK_START_M;
      putreg32(regval, spi_dma_out_link_reg);
      putreg32((n * 8 - 1), spi_mosi_dlen_reg);
      esp32_spi_set_regbits(spi_user_reg, SPI_USR_MOSI_M);

      tp += n;

      if (rp != NULL)
        {
          esp32_dma_init(dma_rx_desc, SPI_DMADESC_NUM, rp, bytes);

          regval  = VALUE_MASK((uintptr_t)dma_rx_desc, SPI_INLINK_ADDR);
          regval |= SPI_INLINK_START_M;
          putreg32(regval, spi_dma_in_link_reg);
          putreg32((n * 8 - 1), spi_miso_dlen_reg);
          esp32_spi_set_regbits(spi_user_reg, SPI_USR_MISO_M);

          rp += n;
        }
      else
        {
          esp32_spi_reset_regbits(spi_user_reg, SPI_USR_MISO_M);
        }

      esp32_spi_set_regbits(spi_cmd_reg, SPI_USR_M);

      esp32_spi_sem_waitdone(priv);

      bytes -= n;
    }

  esp32_spi_reset_regbits(spi_slave_reg, SPI_INT_EN_M);

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  if (esp32_ptr_extram(rxbuffer))
    {
      memcpy(rxbuffer, allocrp, total);
      xtensa_imm_free(allocrp);
    }
#endif

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  if (esp32_ptr_extram(txbuffer))
    {
      xtensa_imm_free(alloctp);
    }
#endif
}

/****************************************************************************
 * Name: esp32_spi_poll_send
 *
 * Description:
 *   Send one word on SPI by polling mode.
 *
 * Input Parameters:
 *   priv - SPI private state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t esp32_spi_poll_send(struct esp32_spi_priv_s *priv,
                                    uint32_t wd)
{
  uint32_t val;
  const uintptr_t spi_miso_dlen_reg = SPI_MISO_DLEN_REG(priv->config->id);
  const uintptr_t spi_mosi_dlen_reg = SPI_MOSI_DLEN_REG(priv->config->id);
  const uintptr_t spi_w0_reg = SPI_W0_REG(priv->config->id);
  const uintptr_t spi_cmd_reg = SPI_CMD_REG(priv->config->id);

  putreg32((priv->nbits - 1), spi_miso_dlen_reg);
  putreg32((priv->nbits - 1), spi_mosi_dlen_reg);

  putreg32(wd, spi_w0_reg);

  esp32_spi_set_regbits(spi_cmd_reg, SPI_USR_M);

  while ((getreg32(spi_cmd_reg) & SPI_USR_M) != 0)
    {
      ;
    }

  val = getreg32(spi_w0_reg);

  spiinfo("send=%x and recv=%x\n", wd, val);

  return val;
}

/****************************************************************************
 * Name: esp32_spi_send
 *
 * Description:
 *   Send one word on SPI.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t esp32_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;

  return esp32_spi_poll_send(priv, wd);
}

/****************************************************************************
 * Name: esp32_spi_poll_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
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
 *   None
 *
 ****************************************************************************/

static void esp32_spi_poll_exchange(struct esp32_spi_priv_s *priv,
                                    const void *txbuffer,
                                    void *rxbuffer,
                                    size_t nwords)
{
  const uintptr_t spi_user_reg = SPI_USER_REG(priv->config->id);
  const uintptr_t spi_w0_reg = SPI_W0_REG(priv->config->id);
  const uintptr_t spi_cmd_reg = SPI_CMD_REG(priv->config->id);
  const uintptr_t spi_miso_dlen_reg = SPI_MISO_DLEN_REG(priv->config->id);
  const uintptr_t spi_mosi_dlen_reg = SPI_MOSI_DLEN_REG(priv->config->id);
  const uint32_t total_bytes = nwords * (priv->nbits / 8);
  uintptr_t bytes_remaining = total_bytes;
  uint8_t *tp = (uint8_t *)txbuffer;
  uint8_t *rp = (uint8_t *)rxbuffer;

  while (bytes_remaining != 0)
    {
      /* Initialize data_buf_reg with the address of the first data buffer
       * register (W0).
       */

      uintptr_t data_buf_reg = spi_w0_reg;
      uint32_t transfer_size = MIN(SPI_MAX_BUF_SIZE, bytes_remaining);

      /* Write data words to data buffer registers.
       * SPI peripheral contains 16 registers (W0 - W15).
       */

      for (int i = 0 ; i < transfer_size; i += sizeof(uint32_t))
        {
          uint32_t w_wd = UINT32_MAX;

          if (tp != NULL)
            {
              memcpy(&w_wd, tp, sizeof(uint32_t));

              tp += sizeof(uintptr_t);
            }

          putreg32(w_wd, data_buf_reg);

          spiinfo("send=0x%" PRIx32 " data_reg=0x%" PRIx32 "\n",
                  w_wd, data_buf_reg);

          /* Update data_buf_reg to point to the next data buffer register. */

          data_buf_reg += sizeof(uintptr_t);
        }

      esp32_spi_set_regbits(spi_user_reg, SPI_USR_MOSI_M);

      if (rp == NULL)
        {
          esp32_spi_reset_regbits(spi_user_reg, SPI_USR_MISO_M);
        }
      else
        {
          esp32_spi_set_regbits(spi_user_reg, SPI_USR_MISO_M);
        }

      putreg32((transfer_size * 8) - 1, spi_mosi_dlen_reg);
      putreg32((transfer_size * 8) - 1, spi_miso_dlen_reg);

      /* Trigger start of user-defined transaction for master. */

      esp32_spi_set_regbits(spi_cmd_reg, SPI_USR_M);

      /* Wait for the user-defined transaction to finish. */

      while ((getreg32(spi_cmd_reg) & SPI_USR_M) != 0)
        {
          ;
        }

      if (rp != NULL)
        {
          /* Set data_buf_reg with the address of the first data buffer
           * register (W0).
           */

          data_buf_reg = spi_w0_reg;

          /* Read received data words from SPI data buffer registers. */

          for (int i = 0 ; i < transfer_size; i += sizeof(uint32_t))
            {
              uint32_t r_wd = getreg32(data_buf_reg);

              spiinfo("recv=0x%" PRIx32 " data_reg=0x%" PRIx32 "\n",
                      r_wd, data_buf_reg);

              memcpy(rp, &r_wd, sizeof(uint32_t));

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
 * Name: esp32_spi_exchange
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
 ****************************************************************************/

static void esp32_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer,
                               size_t nwords)
{
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;

#ifdef CONFIG_ESP32_SPI_DMATHRESHOLD
  size_t thld = CONFIG_ESP32_SPI_DMATHRESHOLD;
#else
  size_t thld = 0;
#endif

  if (priv->config->use_dma && nwords > thld)
    {
      esp32_spi_dma_exchange(priv, txbuffer, rxbuffer, nwords);
    }
  else
    {
      esp32_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: esp32_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
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

static void esp32_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);

  esp32_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: esp32_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);

  esp32_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: esp32_spi_trigger
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
static int esp32_spi_trigger(struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp32_spi_init
 *
 * Description:
 *   Initialize ESP32 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_init(struct spi_dev_s *dev)
{
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;
  const struct esp32_spi_config_s *config = priv->config;
  uint32_t regval;

  esp32_gpiowrite(config->cs_pin, 1);
  esp32_gpiowrite(config->clk_pin, 1);

  if (config->flags & ESP32_SPI_IO_W)
    {
      esp32_gpiowrite(config->mosi_pin, 1);
    }

  if (config->flags & ESP32_SPI_IO_R)
    {
      esp32_gpiowrite(config->miso_pin, 1);
    }

#ifdef CONFIG_ESP32_SPI_SWCS
  esp32_configgpio(config->cs_pin, OUTPUT);
  esp32_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif

  if (esp32_spi_iomux(priv))
    {
#ifndef CONFIG_ESP32_SPI_SWCS
      esp32_configgpio(config->cs_pin, OUTPUT_FUNCTION_2);
      esp32_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif

      esp32_configgpio(config->clk_pin, OUTPUT_FUNCTION_2);
      esp32_gpio_matrix_out(config->clk_pin, SIG_GPIO_OUT_IDX, 0, 0);

      if (config->flags & ESP32_SPI_IO_W)
        {
          esp32_configgpio(config->mosi_pin, OUTPUT_FUNCTION_2);
          esp32_gpio_matrix_out(config->mosi_pin, SIG_GPIO_OUT_IDX, 0, 0);
        }

      if (config->flags & ESP32_SPI_IO_R)
        {
          esp32_configgpio(config->miso_pin, INPUT_FUNCTION_2 | PULLUP);
          esp32_gpio_matrix_out(config->miso_pin, SIG_GPIO_OUT_IDX, 0, 0);
        }
    }
  else
    {
#ifndef CONFIG_ESP32_SPI_SWCS
      esp32_configgpio(config->cs_pin, OUTPUT_FUNCTION_3);
      esp32_gpio_matrix_out(config->cs_pin, config->cs_outsig, 0, 0);
#endif

      esp32_configgpio(config->clk_pin, OUTPUT_FUNCTION_3);
      esp32_gpio_matrix_out(config->clk_pin, config->clk_outsig, 0, 0);

      if (config->flags & ESP32_SPI_IO_W)
        {
          esp32_configgpio(config->mosi_pin, OUTPUT_FUNCTION_3);
          esp32_gpio_matrix_out(config->mosi_pin, config->mosi_outsig, 0, 0);
        }

      if (config->flags & ESP32_SPI_IO_R)
        {
          esp32_configgpio(config->miso_pin, INPUT_FUNCTION_3 | PULLUP);
          esp32_gpio_matrix_in(config->miso_pin, config->miso_insig, 0);
        }
    }

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->clk_bit);
  modifyreg32(DPORT_PERIP_RST_EN_REG, config->rst_bit, 0);

  regval = SPI_DOUTDIN_M | SPI_USR_MISO_M | SPI_USR_MOSI_M | SPI_CS_HOLD_M;
  putreg32(regval, SPI_USER_REG(config->id));
  putreg32(0, SPI_USER1_REG(config->id));
  putreg32(0, SPI_SLAVE_REG(config->id));
  putreg32(SPI_CS1_DIS_M | SPI_CS2_DIS_M, SPI_PIN_REG(config->id));

#ifdef CONFIG_ESP32_SPI_SWCS
  esp32_spi_set_regbits(SPI_PIN_REG(config->id), SPI_CS0_DIS_M);
#endif

  putreg32(0, SPI_CTRL_REG(config->id));
  putreg32(VALUE_MASK(0, SPI_HOLD_TIME), SPI_CTRL2_REG(config->id));

  if (config->use_dma)
    {
      modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->dma_clk_bit);
      modifyreg32(DPORT_PERIP_RST_EN_REG, config->dma_rst_bit, 0);

      modifyreg32(DPORT_SPI_DMA_CHAN_SEL_REG, 0,
                  (config->dma_chan << config->dma_chan_s));

      regval = SPI_OUT_DATA_BURST_EN_M |
               SPI_INDSCR_BURST_EN_M |
               SPI_OUTDSCR_BURST_EN_M;
      putreg32(regval, SPI_DMA_CONF_REG(config->id));
    }

  esp32_spi_setfrequency(dev, config->clk_freq);
  esp32_spi_setbits(dev, 8);
  esp32_spi_setmode(dev, config->mode);
}

/****************************************************************************
 * Name: esp32_spi_deinit
 *
 * Description:
 *   Deinitialize ESP32 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_deinit(struct spi_dev_s *dev)
{
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;

  if (priv->config->use_dma)
    {
      modifyreg32(DPORT_PERIP_RST_EN_REG, 0, priv->config->dma_rst_bit);
      modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->dma_clk_bit, 0);
    }

  modifyreg32(DPORT_PERIP_RST_EN_REG, 0, priv->config->rst_bit);
  modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->clk_bit, 0);

  priv->frequency = 0;
  priv->actual    = 0;
  priv->mode      = SPIDEV_MODE0;
  priv->nbits     = 0;
}

/****************************************************************************
 * Name: esp32_spi_interrupt
 *
 * Description:
 *   Common SPI DMA interrupt handler
 *
 * Input Parameters:
 *   arg - SPI controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int esp32_spi_interrupt(int irq, void *context, void *arg)
{
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)arg;

  esp32_spi_reset_regbits(SPI_SLAVE_REG(priv->config->id), SPI_TRANS_DONE_M);
  nxsem_post(&priv->sem_isr);

  return 0;
}

/****************************************************************************
 * Name: esp32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *esp32_spibus_initialize(int port)
{
  int ret;
  struct spi_dev_s *spi_dev;
  struct esp32_spi_priv_s *priv;

  switch (port)
    {
#ifdef CONFIG_ESP32_SPI2
      case ESP32_SPI2:
        priv = &esp32_spi2_priv;
        break;
#endif
#ifdef CONFIG_ESP32_SPI3
      case ESP32_SPI3:
        priv = &esp32_spi3_priv;
        break;
#endif
      default:
        return NULL;
    }

  nxmutex_lock(&priv->lock);

  spi_dev = (struct spi_dev_s *)priv;
  if (priv->refs != 0)
    {
      priv->refs++;
      nxmutex_unlock(&priv->lock);

      return spi_dev;
    }

  if (priv->config->use_dma)
    {
      /* Set up to receive peripheral interrupts on the current CPU */

      priv->cpu = up_cpu_index();
      priv->cpuint = esp32_setup_irq(priv->cpu, priv->config->periph,
                                     1, ESP32_CPUINT_LEVEL);
      if (priv->cpuint < 0)
        {
          nxmutex_unlock(&priv->lock);
          return NULL;
        }

      ret = irq_attach(priv->config->irq, esp32_spi_interrupt, priv);
      if (ret != OK)
        {
          esp32_teardown_irq(priv->cpu,
                             priv->config->periph,
                             priv->cpuint);
          nxmutex_unlock(&priv->lock);
          return NULL;
        }

      up_enable_irq(priv->config->irq);
    }

  esp32_spi_init(spi_dev);
  priv->refs++;

  nxmutex_unlock(&priv->lock);
  return spi_dev;
}

/****************************************************************************
 * Name: esp32_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus
 *
 ****************************************************************************/

int esp32_spibus_uninitialize(struct spi_dev_s *dev)
{
  struct esp32_spi_priv_s *priv = (struct esp32_spi_priv_s *)dev;

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

  if (priv->config->use_dma)
    {
      up_disable_irq(priv->config->irq);
      esp32_teardown_irq(priv->cpu,
                         priv->config->periph,
                         priv->cpuint);
    }

  esp32_spi_deinit(dev);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESP32_SPI */
