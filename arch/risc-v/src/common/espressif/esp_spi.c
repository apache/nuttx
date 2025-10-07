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
#include <nuttx/nuttx.h>

#include <arch/board/board.h>

#include "esp_spi.h"
#include "esp_irq.h"
#include "esp_gpio.h"

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
#include "esp_dma.h"
#endif

#include "riscv_internal.h"

#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "hal/spi_hal.h"
#include "hal/spi_types.h"
#include "esp_clk_tree.h"
#include "esp_clk_tree_common.h"
#include "hal/clk_tree_hal.h"
#include "hal/hal_utils.h"
#include "periph_ctrl.h"
#include "esp_private/spi_share_hw_ctrl.h"
#include "soc/gdma_periph.h"
#include "hal/gdma_ll.h"
#include "esp_memory_utils.h"

#if SOC_GDMA_SUPPORTED
#  include "esp_private/gdma.h"
#endif

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
#  define SPI_HAVE_SWCS 1
#else
#  define SPI_HAVE_SWCS 0
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

#define SPI_VIA_IOMUX     ((SPI_IS_CS_IOMUX || SPI_HAVE_SWCS) && \
                           (SPI_IS_CLK_IOMUX) &&                 \
                           (SPI_IS_MOSI_IOMUX) &&                \
                           (SPI_IS_MISO_IOMUX)) ? 1 : 0

/* SPI default frequency (limited by clock divider) */

#define SPI_DEFAULT_FREQ  (4000000)

/* SPI default width */

#define SPI_DEFAULT_WIDTH (8)

/* SPI default mode */

#define SPI_DEFAULT_MODE  (SPIDEV_MODE0)

/* SPI Maximum buffer size in bytes */

#define SPI_MAX_BUF_SIZE (64)

/* SPI blank array size */

#define SPI_BLANK_ARRAY_SIZE (16)

#if SOC_PERIPH_CLK_CTRL_SHARED
#define SPI_MASTER_PERI_CLOCK_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define SPI_MASTER_PERI_CLOCK_ATOMIC()
#endif

#if defined(SOC_GDMA_BUS_AXI) && \
    (SOC_GDMA_TRIG_PERIPH_SPI2_BUS == SOC_GDMA_BUS_AXI)
#  define SPI_GDMA_NEW_CHANNEL     gdma_new_axi_channel
#elif defined(SOC_GDMA_BUS_AHB) && \
      (SOC_GDMA_TRIG_PERIPH_SPI2_BUS == SOC_GDMA_BUS_AHB)
#  define SPI_GDMA_NEW_CHANNEL    gdma_new_ahb_channel
#endif

#if SOC_GPSPI_SUPPORTED && defined(SOC_GDMA_BUS_AXI) && (SOC_GDMA_TRIG_PERIPH_SPI2_BUS == SOC_GDMA_BUS_AXI)
#  define DMA_DESC_MEM_ALIGN_SIZE 8
typedef dma_descriptor_align8_t spi_dma_desc_t;
#else
#  define DMA_DESC_MEM_ALIGN_SIZE 4
typedef dma_descriptor_align4_t spi_dma_desc_t;
#endif

#if SOC_GDMA_SUPPORTED  /* AHB_DMA_V1 and AXI_DMA */

/* DMA is provided by gdma driver on these targets */

#define spi_dma_reset               gdma_reset
#define spi_dma_start(chan, addr)   gdma_start(chan, (intptr_t)(addr))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device hardware configuration */

struct esp_spi_config_s
{
  uint32_t width;              /* SPI default width */
  uint8_t  cs_pin;             /* GPIO pin for CS */
  uint8_t  mosi_pin;           /* GPIO pin for MOSI */
  uint8_t  miso_pin;           /* GPIO pin for MISO */
  uint8_t  clk_pin;            /* GPIO pin for CLK */
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
  gdma_channel_handle_t dma_channel_tx; /* I2S DMA TX channel being used */
  gdma_channel_handle_t dma_channel_rx; /* I2S DMA RX channel being used */
  int32_t dma_channel;                  /* Channel assigned by the GDMA driver */
#endif
  uint8_t nbits;                        /* Actual SPI send/receive bits once transmission */
  uint8_t id;                           /* ID number of SPI interface */
  spi_hal_context_t *ctx;               /* Context struct of common layer */
  spi_hal_dev_config_t *dev_cfg;        /* Device configuration struct of common layer */
  spi_hal_timing_param_t *timing_param; /* Timing struct of common layer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static uint32_t spi_common_dma_setup(int chan, bool tx,
                                     spi_dma_desc_t *dmadesc, uint32_t num,
                                     uint8_t *pbuf, uint32_t len);
#endif
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
  .nbits        = 0,
  .dev_cfg      = &dev_cfg,
  .ctx          = &ctx,
  .timing_param = &timing_param,
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  .sem_isr      = SEM_INITIALIZER(0),
  .cpuint       = -ENOMEM,
#endif
};
#endif /* CONFIG_ESPRESSIF_SPI2 */

#ifdef CONFIG_ESPRESSIF_SPI2_DMA

/* SPI DMA RX/TX description */

static spi_dma_desc_t dma_rxdesc[SPI_DMA_DESC_NUM];
static spi_dma_desc_t dma_txdesc[SPI_DMA_DESC_NUM];

#endif

/* Blank array to fill the send buffer */

static uint32_t blank_arr[SPI_BLANK_ARRAY_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_common_dma_setup
 *
 * Description:
 *   Set up DMA descriptor with given parameters.
 *
 * Input Parameters:
 *   chan    - DMA channel
 *   tx      - true: TX mode; false: RX mode
 *   dmadesc - DMA descriptor pointer
 *   num     - DMA descriptor number
 *   pbuf    - Buffer pointer
 *   len     - Buffer length by byte
 *
 * Returned Value:
 *   Bind pbuf data bytes.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static uint32_t spi_common_dma_setup(int chan, bool tx,
                                     spi_dma_desc_t *dmadesc, uint32_t num,
                                     uint8_t *pbuf, uint32_t len)
{
  int i;
  uint32_t regval;
  uint32_t bytes = len;
  uint8_t *pdata = pbuf;
  uint32_t data_len;
  uint32_t buf_len;
  gdma_channel_direction_t dir = tx == true ? GDMA_CHANNEL_DIRECTION_TX : \
                                              GDMA_CHANNEL_DIRECTION_RX;
  spi_dma_desc_t *dma_desc = (spi_dma_desc_t *)dmadesc;

  DEBUGASSERT(chan >= 0);
  DEBUGASSERT(dmadesc != NULL);
  DEBUGASSERT(num > 0);
  DEBUGASSERT(pbuf != NULL);
  DEBUGASSERT(len > 0);

  for (i = 0; i < num; i++)
    {
      data_len = MIN(bytes, ESPRESSIF_DMA_BUFLEN_MAX);

      /* Buffer length must be rounded to next 32-bit boundary. */

      buf_len = ALIGN_UP(data_len, sizeof(uintptr_t));

      dma_desc[i].dw0.size = buf_len;
      dma_desc[i].dw0.length = tx ? data_len : 0;
      dma_desc[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
      dma_desc[i].buffer = pdata;

      dma_desc[i].next = &dma_desc[i + 1];

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
      esp_cache_msync(&dma_desc[i], sizeof(dma_descriptor_t),
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M | \
                      ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif

      bytes -= data_len;

      if (bytes == 0)
        {
          break;
        }

      pdata += data_len;
    }

  dma_desc[i].dw0.suc_eof = tx ? 1 : 0;
  dma_desc[i].next = NULL;

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync(&dma_desc[i], sizeof(dma_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M | \
                  ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif

  return len - bytes;
}
#endif

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
  spi_hal_timing_conf_t temp_timing_conf;
  uint32_t clock_source_hz = 0;
  uint32_t clock_source_div = 1;

  esp_clk_tree_enable_src(SPI_CLK_SRC_DEFAULT, true);
  esp_clk_tree_src_get_freq_hz(SPI_CLK_SRC_DEFAULT,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                               &clock_source_hz);

  if (priv->dev_cfg->timing_conf.real_freq == frequency)
    {
      /* Requested frequency is the same as the current frequency. */

      return priv->dev_cfg->timing_conf.real_freq;
    }

#if SPI_LL_SUPPORT_CLK_SRC_PRE_DIV
  if (clock_source_hz / 2 > (80 * 1000000))
    {
      /* clock_source_hz beyond peripheral HW limitation, calc pre-divider */

      hal_utils_clk_info_t clk_cfg =
        {
          .src_freq_hz = clock_source_hz,
          .exp_freq_hz = frequency * 2,  /* we have (hs_clk = 2*mst_clk), calc hs_clk first */
          .round_opt = HAL_DIV_ROUND,
          .min_integ = 1,
          .max_integ = SPI_LL_CLK_SRC_PRE_DIV_MAX / 2,
        };

      hal_utils_calc_clk_div_integer(&clk_cfg, &clock_source_div);
    }

  clock_source_div *= 2;                /* convert to mst_clk function divider */
  clock_source_hz /= clock_source_div;  /* actual freq enter to SPI peripheral */
#endif

  priv->timing_param->clk_src_hz = clock_source_hz;
  priv->timing_param->expected_freq = frequency;

  spi_hal_cal_clock_conf(priv->timing_param, &temp_timing_conf);

  temp_timing_conf.clock_source = SPI_CLK_SRC_DEFAULT;
  temp_timing_conf.source_pre_div = clock_source_div;
  temp_timing_conf.source_real_freq = clock_source_hz;

  priv->dev_cfg->timing_conf = temp_timing_conf;

  spi_hal_setup_device(priv->ctx, priv->dev_cfg);

  SPI_MASTER_PERI_CLOCK_ATOMIC()
    {
#if SPI_LL_SUPPORT_CLK_SRC_PRE_DIV

      /* We set mst_div as const 2, then (hs_clk = 2*mst_clk) to ensure
       * timing turning work as past and ensure
       * (hs_div * mst_div = source_pre_div)
       */

      spi_ll_clk_source_pre_div(priv->ctx->hw,
                                clock_source_div / 2, 2);
#endif
      spi_ll_set_clk_source(priv->ctx->hw,
                            priv->dev_cfg->timing_conf.clock_source);
    }

  spiinfo("frequency=%" PRIu32 ", actual=%d\n",
          priv->timing_param->expected_freq,
          priv->dev_cfg->timing_conf.real_freq);

  return priv->dev_cfg->timing_conf.real_freq;
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
  const uint32_t total = nwords * ((priv->nbits + 7) / 8);
  uint32_t tx_byte_len;
  uint32_t rx_byte_len;
  int tx_dma_channel_id;
  int rx_dma_channel_id;
  spi_dma_dev_t *spi_dma = SPI_LL_GET_HW(SPI2_HOST);
  uint32_t n;
  uint16_t alignment;
  uint32_t bytes = total;
  uint8_t *txbuffer_temp;
  uint8_t *rxbuffer_temp;
  uint8_t *tp;
  uint8_t *rp;
  spi_hal_trans_config_t trans =
    {
      0
    };

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  bool tx_unaligned;
  bool rx_unaligned;
#else
  bool tx_unaligned;
  bool rx_unaligned;
#endif
  bool tx_realoc = false;
  bool rx_realoc = false;

  if (txbuffer == NULL)
    {
      txbuffer = rxbuffer;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_get_alignment(MALLOC_CAP_DMA, (size_t *)&alignment);
#else
  alignment = 4;
#endif

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  tx_unaligned = ((((uint32_t)txbuffer) | total) & (alignment - 1));
  rx_unaligned = ((((uint32_t)rxbuffer) | total) & (alignment - 1));
#else
  tx_unaligned = false;   /* tx don't need align on addr or length, for other chips */
  rx_unaligned = (((uint32_t)rxbuffer) & (alignment - 1));
#endif

  gdma_get_group_channel_id(priv->dma_channel_tx, NULL, &tx_dma_channel_id);
  gdma_get_group_channel_id(priv->dma_channel_rx, NULL, &rx_dma_channel_id);

  spiinfo("tx_dma_channel_id: %d, rx_dma_channel_id: %d\n",
          tx_dma_channel_id, rx_dma_channel_id);

  if ((!esp_ptr_dma_capable(txbuffer) || tx_unaligned))
    {
      /* If txbuf in the desc not DMA-capable, or not bytes aligned to
       * alignment, malloc a new one.
       */

      spiinfo("Allocate TX buffer for DMA");

      /* Up align alignment */

      tx_byte_len = (total + alignment - 1) & (~(alignment - 1));
      uint32_t *temp = kmm_memalign(alignment, tx_byte_len);

      memcpy(temp, txbuffer, tx_byte_len);
      txbuffer_temp = (uint8_t *)temp;
      tp = txbuffer_temp;
      tx_realoc = true;
    }
  else
    {
      tx_byte_len = total;
      txbuffer_temp = (uint8_t *)txbuffer;
      tp = txbuffer_temp;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_err_t ret = esp_cache_msync((void *)txbuffer_temp,
                                  tx_byte_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  assert(ret == ESP_OK);
#endif

  if ((!esp_ptr_dma_capable(rxbuffer) || rx_unaligned))
    {
      uint32_t *temp;

      /* If rxbuffer in the desc is not DMA-capable, or not aligned to the
       * required alignment, allocate a new one
       */

      spiinfo("Allocate RX buffer for DMA");
      rx_byte_len = (total + alignment - 1) & (~(alignment - 1));
      temp = kmm_memalign(alignment, rx_byte_len);

      memcpy(temp, rxbuffer, rx_byte_len);
      rxbuffer_temp = (uint8_t *)temp;
      rp = rxbuffer_temp;
      rx_realoc = true;
    }
  else
    {
      rx_byte_len = total;
      rxbuffer_temp = (uint8_t *)rxbuffer;
      rp = rxbuffer_temp;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  /* Invalidate here to hold on cache status to avoid hardware auto-write
   * back during dma transaction
   */

  ret = esp_cache_msync((void *)rxbuffer_temp,
                        rx_byte_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  assert(ret == ESP_OK);
#endif

  trans.cs_keep_active = priv->dev_cfg->cs_hold;
  trans.line_mode.data_lines = 1;
  trans.line_mode.addr_lines = 1;
  trans.line_mode.cmd_lines = 1;
  trans.rcv_buffer = (uint8_t *)rp;
  trans.send_buffer = (uint8_t *)tp;
  priv->ctx->trans_config = trans;

  spi_hal_setup_trans(priv->ctx, priv->dev_cfg, &trans);

  spi_hal_setup_device(priv->ctx, priv->dev_cfg);

  while (bytes != 0)
    {
      n = spi_common_dma_setup(tx_dma_channel_id, true, dma_txdesc,
                               SPI_DMA_DESC_NUM, tp, bytes);

      trans.tx_bitlen = n * 8;
      trans.rx_bitlen = n * 8;
      trans.rcv_buffer = (uint8_t *)rp;
      trans.send_buffer = (uint8_t *)tp;

      spi_hal_setup_trans(priv->ctx, priv->dev_cfg, &trans);

      spi_hal_setup_device(priv->ctx, priv->dev_cfg);

      /* Reset SPI DMA TX FIFO */

      spi_dma_reset(priv->dma_channel_tx);

      spi_hal_hw_prepare_tx(priv->ctx->hw);

      spi_dma_start(priv->dma_channel_tx, dma_txdesc);

      spi_ll_enable_mosi(priv->ctx->hw, true);

      tp += n;

      if (rp != NULL)
        {
          /* Enable SPI DMA RX */

          spi_common_dma_setup(rx_dma_channel_id, false, dma_rxdesc,
                               SPI_DMA_DESC_NUM, rp, bytes);

          spi_dma_reset(priv->dma_channel_rx);

          spi_hal_hw_prepare_rx(priv->ctx->hw);

          spi_dma_start(priv->dma_channel_rx, dma_rxdesc);

          spi_ll_enable_miso(priv->ctx->hw, true);

          rp += n;
        }
      else
        {
          spi_ll_enable_miso(priv->ctx->hw, false);
        }

      spi_hal_user_start(priv->ctx);

      esp_spi_sem_waitdone(priv);

      bytes -= n;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  ret = esp_cache_msync((void *)rxbuffer_temp,
                        rx_byte_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
#endif

  if (tx_realoc)
    {
      kmm_free(txbuffer_temp);
    }

  if (rx_realoc)
    {
      memcpy(rxbuffer, rxbuffer_temp, rx_byte_len);
      kmm_free(rxbuffer_temp);
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
  spi_hal_dev_config_t *hal_dev = priv->dev_cfg;
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

  spi_hal_setup_trans(priv->ctx, priv->dev_cfg, &trans);
  spi_hal_push_tx_buffer(priv->ctx, &trans);
  spi_hal_enable_data_line(priv->ctx->hw,
                           (!hal_dev->half_duplex && trans.rcv_buffer) \
                            || trans.send_buffer, !!trans.rcv_buffer);

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
       * SPI peripheral contains 16 registers (W0 - W15)
       * allowing up to 64 bytes to be sent or received at one time.
       */

      trans.tx_bitlen = transfer_size * 8;
      trans.rx_bitlen = transfer_size * 8;
      trans.rcv_buffer = (uint8_t *)rp;
      priv->ctx->trans_config = trans;

      if (tp == NULL)
        {
          /* Write 0xffffffff to fill the send buffer */

          trans.send_buffer = (uint8_t *)blank_arr;
          spiinfo("send %" PRIu32 " bytes value=0x%" PRIx32 "\n",
                  transfer_size, blank_arr[0]);
        }
      else
        {
          trans.send_buffer = (uint8_t *)tp;
          spiinfo("send %" PRIu32 " bytes addr=0x%p\n",
                  transfer_size, tp);
        }

      spi_hal_setup_trans(priv->ctx, priv->dev_cfg, &trans);
      spi_hal_push_tx_buffer(priv->ctx, &trans);
      spi_hal_user_start(priv->ctx);
      spi_hal_enable_data_line(priv->ctx->hw,
                               (!priv->timing_param->half_duplex && \
                                trans.rcv_buffer) || trans.send_buffer,
                               !!trans.rcv_buffer);

      while (!spi_hal_usr_is_done(priv->ctx));

      if (rp != NULL)
        {
          spi_hal_fetch_result(priv->ctx);
          spiinfo("recv %" PRIu32 " bytes addr=0x%p\n",
                  transfer_size, rp);
          rp += transfer_size;
        }

      if (tp != NULL)
        {
          tp += transfer_size;
        }

      bytes_remaining -= transfer_size;
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
  gdma_channel_alloc_config_t tx_handle =
    {
      .direction = GDMA_CHANNEL_DIRECTION_TX,
      .flags.reserve_sibling = 1,
    };

  gdma_channel_alloc_config_t rx_handle =
    {
      .direction = GDMA_CHANNEL_DIRECTION_RX,
    };

  /* Request a GDMA channel for SPI peripheral */

  SPI_GDMA_NEW_CHANNEL(&tx_handle, &priv->dma_channel_tx);
  gdma_connect(priv->dma_channel_tx,
               GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 2));

  rx_handle.sibling_chan = priv->dma_channel_tx;

  SPI_GDMA_NEW_CHANNEL(&rx_handle, &priv->dma_channel_rx);
  gdma_connect(priv->dma_channel_rx,
               GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 2));
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
#if !SPI_VIA_IOMUX
#  if !SPI_HAVE_SWCS
  int cs_id = 0;
#  endif
#endif

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
  esp_gpio_matrix_out(config->cs_pin,
                      spi_periph_signal[priv->id].spics_out[cs_id], 0, 0);
#endif
  esp_configgpio(config->mosi_pin, MOSI_PIN_ATTR);
  esp_gpio_matrix_out(config->mosi_pin,
                      spi_periph_signal[priv->id].spid_out, 0, 0);

  esp_configgpio(config->miso_pin, MISO_PIN_ATTR);
  esp_gpio_matrix_in(config->miso_pin,
                     spi_periph_signal[priv->id].spiq_in, 0);

  esp_configgpio(config->clk_pin, OUTPUT_FUNCTION_2);
  esp_gpio_matrix_out(config->clk_pin,
                      spi_periph_signal[priv->id].spiclk_out, 0, 0);
#endif

  SPI_COMMON_RCC_CLOCK_ATOMIC()
    {
      spi_ll_enable_bus_clock(priv->id, true);
      spi_ll_reset_register(priv->id);
    }

  SPI_MASTER_PERI_CLOCK_ATOMIC()
    {
      spi_ll_enable_clock(priv->id, true);
    }

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  esp_spi_dma_init(dev);

  priv->ctx->hw = SPI_LL_GET_HW(priv->id);

  spi_ll_master_init(priv->ctx->hw);
  spi_ll_enable_int(priv->ctx->hw);
  spi_ll_set_mosi_delay(priv->ctx->hw, 0, 0);
#else
  spi_hal_init(priv->ctx, priv->id);
#endif

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

  esp_clk_tree_enable_src(priv->dev_cfg->timing_conf.clock_source, false);

  spi_hal_deinit(priv->ctx);

#if SOC_GDMA_SUPPORTED
#  ifdef CONFIG_ESPRESSIF_SPI2_DMA
  if (priv->dma_channel_rx)
    {
      gdma_disconnect(priv->dma_channel_rx);
      gdma_del_channel(priv->dma_channel_rx);
    }

  if (priv->dma_channel_tx)
    {
      gdma_disconnect(priv->dma_channel_tx);
      gdma_del_channel(priv->dma_channel_tx);
    }
#  endif
#endif

  SPI_COMMON_RCC_CLOCK_ATOMIC()
    {
      spi_ll_enable_bus_clock(priv->id, false);
    }

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

  /* Initialize the blank array */

  for (int i = 0; i < SPI_BLANK_ARRAY_SIZE; i++)
    {
      blank_arr[i] = UINT32_MAX;
    }

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(ESP_SOURCE2IRQ(spi_periph_signal[priv->id].irq));
    }

  priv->cpuint = esp_setup_irq(spi_periph_signal[priv->id].irq,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  if (irq_attach(ESP_SOURCE2IRQ(spi_periph_signal[priv->id].irq),
                 esp_spi_interrupt, priv) != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp_teardown_irq(spi_periph_signal[priv->id].irq, priv->cpuint);
      priv->cpuint = -ENOMEM;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the SPI device. */

  up_enable_irq(ESP_SOURCE2IRQ(spi_periph_signal[priv->id].irq));
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
  up_disable_irq(ESP_SOURCE2IRQ(spi_periph_signal[priv->id].irq));
  esp_teardown_irq(spi_periph_signal[priv->id].irq, priv->cpuint);
  priv->cpuint = -ENOMEM;
#endif

  esp_spi_deinit(dev);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESPRESSIF_SPI_PERIPH */
