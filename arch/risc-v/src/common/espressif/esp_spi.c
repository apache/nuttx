/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spi.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESPRESSIF_SPI_PERIPH

#include <assert.h>
#include <debug.h>
#include <sys/param.h>
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
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "esp_spi.h"
#include "esp_irq.h"
#include "esp_gpio.h"

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
#include "esp_dma.h"
#endif

#include "riscv_internal.h"

#include "hal/spi_hal.h"
#include "hal/spi_types.h"
#include "hal/spi_hal.h"
#include "esp_clk_tree.h"
#include "hal/clk_tree_hal.h"
#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ESPRESSIF_SPI_TEST_MODE
#  define MOSI_PIN_ATTR (OUTPUT_FUNCTION_2 | INPUT_FUNCTION_2 | PULLUP)
#  define MISO_PIN_ATTR (OUTPUT_FUNCTION_2 | INPUT_FUNCTION_2 | PULLUP)
#else
#  define MOSI_PIN_ATTR (OUTPUT_FUNCTION_2)
#  define MISO_PIN_ATTR (INPUT_FUNCTION_2 | PULLUP)
#endif

#if defined(CONFIG_ARCH_CHIP_ESP32C6) || defined(CONFIG_ARCH_CHIP_ESP32H2)
#  define SPI2_INTR_SOURCE GSPI2_INTR_SOURCE
#  define ESP_IRQ_SPI2     ESP_IRQ_GSPI2
#endif

#if defined(CONFIG_ARCH_CHIP_ESP32C3_GENERIC)
#define SPI2_IOMUX_MISOPIN  2
#define SPI2_IOMUX_MOSIPIN  7
#define SPI2_IOMUX_CLKPIN   6
#define SPI2_IOMUX_CSPIN    10
#elif defined(CONFIG_ARCH_CHIP_ESP32C6)
#define SPI2_IOMUX_MISOPIN  2
#define SPI2_IOMUX_MOSIPIN  7
#define SPI2_IOMUX_CLKPIN   6
#define SPI2_IOMUX_CSPIN    16
#elif defined(CONFIG_ARCH_CHIP_ESP32H2)
#define SPI2_IOMUX_MISOPIN  0
#define SPI2_IOMUX_MOSIPIN  5
#define SPI2_IOMUX_CLKPIN   4
#define SPI2_IOMUX_CSPIN    1
#endif

/* Check if Chip-Select pin will be controlled via software */

#ifdef CONFIG_ESPRESSIF_SPI_SWCS
#  define SPI_HAVE_SWCS TRUE
#else
#  define SPI_HAVE_SWCS FALSE
#endif

#ifdef CONFIG_ESPRESSIF_SPI2_DMA

/* SPI DMA RX/TX number of descriptors */

#define SPI_DMA_DESC_NUM    (CONFIG_ESPRESSIF_SPI2_DMADESC_NUM)

#endif

/* Verify whether SPI has been assigned IOMUX pins.
 * Otherwise, SPI signals will be routed via GPIO Matrix.
 */

#define SPI_IS_CS_IOMUX   (CONFIG_ESPRESSIF_SPI2_CSPIN == SPI2_IOMUX_CSPIN)
#define SPI_IS_CLK_IOMUX  (CONFIG_ESPRESSIF_SPI2_CLKPIN == SPI2_IOMUX_CLKPIN)
#define SPI_IS_MOSI_IOMUX (CONFIG_ESPRESSIF_SPI2_MOSIPIN == SPI2_IOMUX_MOSIPIN)
#define SPI_IS_MISO_IOMUX (CONFIG_ESPRESSIF_SPI2_MISOPIN == SPI2_IOMUX_MISOPIN)

#define SPI_VIA_IOMUX     (SPI_IS_CS_IOMUX || SPI_HAVE_SWCS) && \
                          (SPI_IS_CLK_IOMUX) &&                 \
                          (SPI_IS_MOSI_IOMUX) &&                \
                          (SPI_IS_MISO_IOMUX)

/* SPI default frequency (limited by clock divider) */

#define SPI_DEFAULT_FREQ  (4000000)

/* SPI default width */

#define SPI_DEFAULT_WIDTH (8)

/* SPI default mode */

#define SPI_DEFAULT_MODE  (SPIDEV_MODE0)

/* SPI Maximum buffer size in bytes */

#define SPI_MAX_BUF_SIZE (64)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device hardware configuration */

struct esp_spi_config_s
{
  uint32_t width;              /* SPI default width */
  uint8_t  cs_pin;             /* GPIO configuration for CS */
  uint8_t  mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t  miso_pin;           /* GPIO configuration for MISO */
  uint8_t  clk_pin;            /* GPIO configuration for CLK */
  uint32_t cs_insig;           /* SPI CS input signal index */
  uint32_t cs_outsig;          /* SPI CS output signal index */
  uint32_t mosi_insig;         /* SPI MOSI input signal index */
  uint32_t mosi_outsig;        /* SPI MOSI output signal index */
  uint32_t miso_insig;         /* SPI MISO input signal index */
  uint32_t miso_outsig;        /* SPI MISO output signal index */
  uint32_t clk_insig;          /* SPI CLK input signal index */
  uint32_t clk_outsig;         /* SPI CLK output signal index */
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  uint8_t  periph;             /* Peripheral ID */
  uint8_t  irq;                /* Interrupt ID */
#endif
};

struct esp_spi_priv_s
{
  /* Externally visible part of the SPI interface */

  struct spi_dev_s spi_dev;

  /* Port configuration */

  const struct esp_spi_config_s *config;
  int refs;                             /* Reference count */
  mutex_t lock;                         /* Held while chip is selected for mutual exclusion */
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  sem_t sem_isr;                        /* Interrupt wait semaphore */
  int cpuint;                           /* SPI interrupt ID */
  int32_t dma_channel;                  /* Channel assigned by the GDMA driver */
#endif
  uint8_t nbits;                        /* Actual SPI send/receive bits once transmission */
  uint8_t id;                           /* ID number of SPI interface */
  uint8_t module;                       /* Module ID of SPI interface */
  spi_hal_context_t *ctx;               /* Context struct of common layer */
  spi_hal_config_t *cfg;
  spi_hal_dev_config_t *dev_cfg;        /* Device configuration struct of common layer */
  spi_hal_timing_param_t *timing_param; /* Timing struct of common layer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_spi_lock(struct spi_dev_s *dev, bool lock);
#ifndef CONFIG_ESPRESSIF_SPI_UDCS
static void esp_spi_select(struct spi_dev_s *dev,
                           uint32_t devid, bool selected);
#endif
static uint32_t esp_spi_setfrequency(struct spi_dev_s *dev,
                                     uint32_t frequency);
static void esp_spi_setmode(struct spi_dev_s *dev,
                            enum spi_mode_e mode);
static void esp_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int esp_spi_hwfeatures(struct spi_dev_s *dev,
                              spi_hwfeatures_t features);
#endif
static uint32_t esp_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void esp_spi_exchange(struct spi_dev_s *dev,
                             const void *txbuffer,
                             void *rxbuffer, size_t nwords);
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static int esp_spi_interrupt(int irq, void *context, void *arg);
static int esp_spi_sem_waitdone(struct esp_spi_priv_s *priv);
static void esp_spi_dma_exchange(struct esp_spi_priv_s *priv,
                                     const void *txbuffer,
                                     void *rxbuffer,
                                     uint32_t nwords);
#else
static void esp_spi_poll_exchange(struct esp_spi_priv_s *priv,
                                  const void *txbuffer,
                                  void *rxbuffer,
                                  size_t nwords);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void esp_spi_sndblock(struct spi_dev_s *dev,
                             const void *txbuffer,
                             size_t nwords);
static void esp_spi_recvblock(struct spi_dev_s *dev,
                              void *rxbuffer,
                              size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int esp_spi_trigger(struct spi_dev_s *dev);
#endif
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static void esp_spi_dma_init(struct spi_dev_s *dev);
#endif
static void esp_spi_init(struct spi_dev_s *dev);
static void esp_spi_deinit(struct spi_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2

static spi_hal_config_t cfg =
{
    0
};

static spi_hal_context_t ctx =
{
    0
};

static spi_hal_dev_config_t dev_cfg  =
{
    .mode = SPI_DEFAULT_MODE,
    .cs_setup = 0,
    .cs_hold = 0,
    .cs_pin_id = 0,
    .timing_conf =
    {
      0
    },
    {
      0
    }
};

static spi_hal_timing_param_t timing_param =
{
    .no_compensate = 0,
    .half_duplex = 0,
    .input_delay_ns = 0,
#ifdef SPI_VIA_IOMUX
    .use_gpio = 0,
#else
    .use_gpio = 1,
#endif
    .duty_cycle = 128,
    .clk_src_hz = 0,
    .expected_freq = SPI_DEFAULT_FREQ,
};

static const struct esp_spi_config_s esp_spi2_config =
{
  .width        = SPI_DEFAULT_WIDTH,
  .cs_pin       = CONFIG_ESPRESSIF_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESPRESSIF_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESPRESSIF_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESPRESSIF_SPI2_CLKPIN,
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  .periph       = SPI2_INTR_SOURCE,
  .irq          = ESP_IRQ_SPI2,
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

static const struct spi_ops_s esp_spi2_ops =
{
  .lock              = esp_spi_lock,
#ifdef CONFIG_ESPRESSIF_SPI_UDCS
  .select            = esp_spi2_select,
#else
  .select            = esp_spi_select,
#endif
  .setfrequency      = esp_spi_setfrequency,
  .setmode           = esp_spi_setmode,
  .setbits           = esp_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = esp_spi_hwfeatures,
#endif
  .status            = esp_spi2_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = esp_spi2_cmddata,
#endif
  .send              = esp_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = esp_spi_exchange,
#else
  .sndblock          = esp_spi_sndblock,
  .recvblock         = esp_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = esp_spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct esp_spi_priv_s esp_spi2_priv =
{
  .spi_dev      =
  {
    .ops        = &esp_spi2_ops
  },
  .config       = &esp_spi2_config,
  .refs         = 0,
  .lock         = NXMUTEX_INITIALIZER,
  .id           = SPI2_HOST,
  .module       = PERIPH_SPI2_MODULE,
  .nbits        = 0,
  .cfg          = &cfg,
  .dev_cfg      = &dev_cfg,
  .ctx          = &ctx,
  .timing_param = &timing_param,
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  .sem_isr      = SEM_INITIALIZER(0),
  .cpuint       = -ENOMEM,
  .dma_channel  = -1,
#endif
};
#endif /* CONFIG_ESPRESSIF_SPI2 */

#ifdef CONFIG_ESPRESSIF_SPI2_DMA

/* SPI DMA RX/TX description */

static struct esp_dmadesc_s dma_rxdesc[SPI_DMA_DESC_NUM];
static struct esp_dmadesc_s dma_txdesc[SPI_DMA_DESC_NUM];

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_spi_lock
 *
 * Description:
 *   Lock or unlock the SPI device.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock SPI bus, false: unlock SPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the SPI device.
 *
 ****************************************************************************/

static int esp_spi_lock(struct spi_dev_s *dev, bool lock)
{
  int ret;
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

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
 * Name: esp_spi_sem_waitdone
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

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static int esp_spi_sem_waitdone(struct esp_spi_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr, SEC2TICK(10));
}
#endif

/****************************************************************************
 * Name: esp_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select. The implementation of this method
 *   must include handshaking: If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 *   If ESPRESSIF_SPI_SWCS is disabled, the driver will use hardware CS so
 *   that once transmission is started the hardware selects the device and
 *   when this transmission is done hardware deselects the device
 *   automatically.
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

#ifndef CONFIG_ESPRESSIF_SPI_UDCS
static void esp_spi_select(struct spi_dev_s *dev,
                           uint32_t devid, bool selected)
{
#if SPI_HAVE_SWCS
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;
  bool value = selected ? false : true;

  esp_gpiowrite(priv->config->cs_pin, value);
#endif

  spiinfo("devid: %08" PRIx32 " CS: %s\n",
          devid, selected ? "select" : "free");
}
#endif

/****************************************************************************
 * Name: esp_spi_setfrequency
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

static uint32_t esp_spi_setfrequency(struct spi_dev_s *dev,
                                     uint32_t frequency)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

  if (priv->timing_param->clk_src_hz == frequency)
    {
      /* Requested frequency is the same as the current frequency. */

      return priv->timing_param->clk_src_hz;
    }

  priv->timing_param->expected_freq = frequency;

  esp_clk_tree_src_get_freq_hz(SPI_CLK_SRC_DEFAULT,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX,
                               &priv->timing_param->clk_src_hz);

  spi_hal_cal_clock_conf(priv->timing_param,
                         (int *)&(priv->timing_param->clk_src_hz),
                         &(priv->dev_cfg->timing_conf));
  spi_hal_setup_device(priv->ctx, priv->dev_cfg);

  spiinfo("frequency=%" PRIu32 ", actual=%" PRIu32 "\n",
          priv->timing_param->expected_freq, priv->timing_param->clk_src_hz);

  return priv->timing_param->clk_src_hz;
}

/****************************************************************************
 * Name: esp_spi_setmode
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

static void esp_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->dev_cfg->mode)
    {
      if (mode > SPIDEV_MODE3 || mode < SPIDEV_MODE0)
        {
          spierr("Invalid mode: %d\n", mode);
          DEBUGPANIC();
          return;
        }
      else
        {
          spi_ll_master_set_mode(priv->ctx->hw, mode);
        }

      priv->dev_cfg->mode = mode;
    }
}

/****************************************************************************
 * Name: esp_spi_setbits
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

static void esp_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  priv->nbits = nbits;
}

/****************************************************************************
 * Name: esp_spi_hwfeatures
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
static int esp_spi_hwfeatures(struct spi_dev_s *dev,
                              spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp_spi_dma_exchange
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

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static void esp_spi_dma_exchange(struct esp_spi_priv_s *priv,
                                     const void *txbuffer,
                                     void *rxbuffer,
                                     uint32_t nwords)
{
  const uint32_t total = nwords * (priv->nbits / 8);
  const int32_t channel = priv->dma_channel;
  uint32_t bytes = total;
  uint32_t n;
  uint8_t *tp = (uint8_t *)txbuffer;
  uint8_t *rp = (uint8_t *)rxbuffer;
  spi_hal_trans_config_t trans =
    {
      0
    };

  if (tp == NULL)
    {
      tp = rp;
    }

  spi_hal_setup_device(priv->ctx, priv->dev_cfg);
  spi_ll_clear_int_stat(priv->ctx->hw);

  trans.cs_keep_active = priv->dev_cfg->cs_hold;
  trans.line_mode.data_lines = 2;
  trans.line_mode.addr_lines = 1;
  trans.line_mode.cmd_lines = 1;

  while (bytes != 0)
    {
      /* Reset SPI DMA TX FIFO */

      spi_ll_cpu_rx_fifo_reset(priv->ctx->hw);

      /* Enable SPI DMA TX */

      spi_ll_dma_tx_fifo_reset(priv->ctx->hw);
      spi_ll_dma_tx_enable(priv->ctx->hw, 1);

      n = esp_dma_setup(channel, true, dma_txdesc,
                        SPI_DMA_DESC_NUM, tp, bytes);
      esp_dma_enable(channel, true);

      /* Write data words to data buffer registers.
       * SPI peripheral contains 16 registers (W0 - W15).
       */

      trans.tx_bitlen = n * 8;
      trans.rx_bitlen = n * 8;
      trans.rcv_buffer = (uint8_t *)rp;
      trans.send_buffer = (uint8_t *)tp;
      priv->ctx->trans_config = trans;

      spi_ll_set_mosi_bitlen(priv->ctx->hw, (n * 8));
      spi_ll_enable_mosi(priv->ctx->hw, true);

      tp += n;

      if (rp != NULL)
        {
          /* Enable SPI DMA RX */

          spi_ll_dma_rx_enable(priv->ctx->hw, 1);
          esp_dma_setup(channel, false, dma_rxdesc, SPI_DMA_DESC_NUM,
                        rp, bytes);
          esp_dma_enable(channel, false);
          spi_ll_enable_miso(priv->ctx->hw, true);

          rp += n;
        }
      else
        {
          spi_ll_enable_miso(priv->ctx->hw, false);
        }

      spi_ll_apply_config(priv->ctx->hw);
      spi_ll_user_start(priv->ctx->hw);

      esp_spi_sem_waitdone(priv);
      bytes -= n;
    }

  spi_ll_clear_int_stat(priv->ctx->hw);
}
#endif

/****************************************************************************
 * Name: esp_spi_poll_send
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

static uint32_t esp_spi_poll_send(struct esp_spi_priv_s *priv, uint32_t wd)
{
  uint32_t val;
  spi_hal_trans_config_t trans =
    {
      0
    };

  trans.tx_bitlen = priv->nbits;
  trans.rx_bitlen = priv->nbits;
  trans.rcv_buffer = (uint8_t *)&val;
  trans.send_buffer = (uint8_t *)&wd;
  trans.cs_keep_active = priv->dev_cfg->cs_hold;
  trans.line_mode.data_lines = 1;
  trans.line_mode.addr_lines = 0;
  trans.line_mode.cmd_lines = 0;
  priv->ctx->trans_config = trans;

  spi_ll_set_mosi_bitlen(priv->ctx->hw, priv->nbits);
  spi_hal_prepare_data(priv->ctx, priv->dev_cfg, &trans);
  spi_hal_setup_trans(priv->ctx, priv->dev_cfg, &trans);
  spi_hal_user_start(priv->ctx);

  while (!spi_hal_usr_is_done(priv->ctx));
  spi_hal_fetch_result(priv->ctx);

  spiinfo("send=0x%" PRIx32 " and recv=0x%" PRIx32 "\n", wd, val);

  return val;
}

/****************************************************************************
 * Name: esp_spi_send
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

static uint32_t esp_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

  return esp_spi_poll_send(priv, wd);
}

/****************************************************************************
 * Name: esp_spi_poll_exchange
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

static void esp_spi_poll_exchange(struct esp_spi_priv_s *priv,
                                  const void *txbuffer,
                                  void *rxbuffer,
                                  size_t nwords)
{
  const uint32_t total_bytes = nwords * (priv->nbits / 8);
  uintptr_t bytes_remaining = total_bytes;
  uint8_t *tp = (uint8_t *)txbuffer;
  uint8_t *rp = (uint8_t *)rxbuffer;
  spi_hal_trans_config_t trans =
    {
      0
    };

  trans.cs_keep_active = priv->dev_cfg->cs_hold;
  trans.line_mode.data_lines = 1;
  trans.line_mode.addr_lines = 0;
  trans.line_mode.cmd_lines = 0;

  while (bytes_remaining != 0)
    {
      uint32_t transfer_size = MIN(SPI_MAX_BUF_SIZE, bytes_remaining);

      /* Write data words to data buffer registers.
       * SPI peripheral contains 16 registers (W0 - W15).
       */

      trans.tx_bitlen = transfer_size * 8;
      trans.rx_bitlen = transfer_size * 8;
      trans.rcv_buffer = (uint8_t *)rp;
      trans.send_buffer = (uint8_t *)tp;
      priv->ctx->trans_config = trans;

      spi_hal_prepare_data(priv->ctx, priv->dev_cfg, &trans);
      spi_hal_setup_trans(priv->ctx, priv->dev_cfg, &trans);
      spi_hal_user_start(priv->ctx);

      while (!spi_hal_usr_is_done(priv->ctx));

      if (rp != NULL)
        {
          rp += transfer_size;
          spi_hal_fetch_result(priv->ctx);
        }

      bytes_remaining -= transfer_size;
      tp += transfer_size;
    }
}

/****************************************************************************
 * Name: esp_spi_exchange
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

static void esp_spi_exchange(struct spi_dev_s *dev,
                             const void *txbuffer,
                             void *rxbuffer,
                             size_t nwords)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  size_t thld = CONFIG_ESPRESSIF_SPI2_DMATHRESHOLD;

  if (nwords > thld)
    {
      esp_spi_dma_exchange(priv, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      esp_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: esp_spi_sndblock
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

static void esp_spi_sndblock(struct spi_dev_s *dev,
                             const void *txbuffer,
                             size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);

  esp_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: esp_spi_recvblock
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

static void esp_spi_recvblock(struct spi_dev_s *dev,
                              void *rxbuffer,
                              size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);

  esp_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: esp_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   -OK     - Trigger was fired
 *   -ENOSYS - Trigger not fired due to lack of DMA or low level support
 *   -EIO    - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int esp_spi_trigger(struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp_spi_dma_init
 *
 * Description:
 *   Initialize SPI connection to GDMA engine.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
void esp_spi_dma_init(struct spi_dev_s *dev)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

  /* Initialize GDMA controller */

  esp_dma_init();

  /* Request a GDMA channel for SPI peripheral */

  priv->dma_channel = esp_dma_request(ESPRESSIF_DMA_PERIPH_M2M, 1, 1, true);
  if (priv->dma_channel < 0)
    {
      spierr("Failed to allocate GDMA channel\n");

      DEBUGPANIC();
    }
}
#endif

/****************************************************************************
 * Name: esp_spi_init
 *
 * Description:
 *   Initialize SPI hardware interface.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_spi_init(struct spi_dev_s *dev)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;
  const struct esp_spi_config_s *config = priv->config;
  uint32_t regval;

  esp_gpiowrite(config->cs_pin, true);
  esp_gpiowrite(config->mosi_pin, true);
  esp_gpiowrite(config->miso_pin, true);
  esp_gpiowrite(config->clk_pin, true);

#if SPI_HAVE_SWCS
  esp_configgpio(config->cs_pin, OUTPUT_FUNCTION_2);
  esp_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif

#if SPI_VIA_IOMUX
#if !SPI_HAVE_SWCS
  esp_configgpio(config->cs_pin, OUTPUT_FUNCTION_3);
  esp_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif
  esp_configgpio(config->mosi_pin, OUTPUT_FUNCTION_3);
  esp_gpio_matrix_out(config->mosi_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp_configgpio(config->miso_pin, INPUT_FUNCTION_3 | PULLUP);
  esp_gpio_matrix_out(config->miso_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp_configgpio(config->clk_pin, OUTPUT_FUNCTION_3);
  esp_gpio_matrix_out(config->clk_pin, SIG_GPIO_OUT_IDX, 0, 0);
#else
#if !SPI_HAVE_SWCS
  esp_configgpio(config->cs_pin, OUTPUT_FUNCTION_2);
  esp_gpio_matrix_out(config->cs_pin, config->cs_outsig, 0, 0);
#endif
  esp_configgpio(config->mosi_pin, MOSI_PIN_ATTR);
  esp_gpio_matrix_out(config->mosi_pin, config->mosi_outsig, 0, 0);

  esp_configgpio(config->miso_pin, MISO_PIN_ATTR);
  esp_gpio_matrix_in(config->miso_pin, config->miso_insig, 0);

  esp_configgpio(config->clk_pin, OUTPUT_FUNCTION_2);
  esp_gpio_matrix_out(config->clk_pin, config->clk_outsig, 0, 0);
#endif

  periph_module_enable(priv->module);

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  esp_spi_dma_init(dev);

  priv->ctx->hw = SPI_LL_GET_HW(priv->id);
  priv->cfg->dma_enabled = true;
  priv->cfg->dmadesc_rx = (lldesc_t *)dma_rxdesc;
  priv->cfg->dmadesc_tx = (lldesc_t *)dma_txdesc;
  priv->cfg->rx_dma_chan = priv->dma_channel;
  priv->cfg->tx_dma_chan = priv->dma_channel;

  spi_ll_master_init(priv->ctx->hw);
  spi_ll_enable_int(priv->ctx->hw);
  spi_ll_set_mosi_delay(priv->ctx->hw, 0, 0);
#else
  spi_hal_init(priv->ctx, priv->id, priv->cfg);
#endif

  priv->dev_cfg->timing_conf.clock_source = SPI_CLK_SRC_DEFAULT;
  esp_clk_tree_src_get_freq_hz(priv->dev_cfg->timing_conf.clock_source,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX,
                               &priv->timing_param->clk_src_hz);

  esp_spi_setbits(dev, config->width);
  esp_spi_setmode(dev, priv->dev_cfg->mode);
  esp_spi_setfrequency(dev, priv->timing_param->expected_freq);
}

/****************************************************************************
 * Name: esp_spi_deinit
 *
 * Description:
 *   Deinitialize SPI hardware interface.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_spi_deinit(struct spi_dev_s *dev)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

  periph_module_disable(priv->module);
  spi_hal_deinit(priv->ctx);

  priv->nbits     = 0;
}

/****************************************************************************
 * Name: esp_spi_interrupt
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

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static int esp_spi_interrupt(int irq, void *context, void *arg)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)arg;

  spi_ll_clear_intr(priv->ctx->hw, SPI_LL_INTR_TRANS_DONE);
  nxsem_post(&priv->sem_isr);

  return 0;
}
#endif

/****************************************************************************
 * Name: esp_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s *esp_spibus_initialize(int port)
{
  struct spi_dev_s *spi_dev;
  struct esp_spi_priv_s *priv;

  switch (port)
    {
#ifdef CONFIG_ESPRESSIF_SPI2
      case ESPRESSIF_SPI2:
        priv = &esp_spi2_priv;
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

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(priv->config->irq);
    }

  priv->cpuint = esp_setup_irq(priv->config->periph,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  if (irq_attach(priv->config->irq, esp_spi_interrupt, priv) != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp_teardown_irq(priv->config->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the SPI device. */

  up_enable_irq(priv->config->irq);
#endif

  esp_spi_init(spi_dev);
  priv->refs++;

  nxmutex_unlock(&priv->lock);
  return spi_dev;
}

/****************************************************************************
 * Name: esp_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp_spibus_uninitialize(struct spi_dev_s *dev)
{
  struct esp_spi_priv_s *priv = (struct esp_spi_priv_s *)dev;

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

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  up_disable_irq(priv->config->irq);
  esp_teardown_irq(priv->config->periph, priv->cpuint);
  priv->cpuint = -ENOMEM;
#endif

  esp_spi_deinit(dev);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESPRESSIF_SPI_PERIPH */
