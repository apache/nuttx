/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_i2s.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "riscv_internal.h"

#include "esp_gpio.h"
#include "esp_irq.h"

#include "esp_i2s.h"

#include "hal/i2s_hal.h"
#include "hal/i2s_ll.h"
#include "soc/i2s_periph.h"
#include "soc/i2s_reg.h"
#include "hal/i2s_types.h"
#include "soc/gpio_sig_map.h"
#include "periph_ctrl.h"

#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_bit_defs.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "riscv/interrupt.h"
#include "soc/lldesc.h"
#include "hal/dma_types.h"
#if SOC_I2S_SUPPORTS_APLL
#include "hal/clk_tree_ll.h"
#include "clk_ctrl_os.h"
#endif
#include "soc/gdma_periph.h"
#include "hal/gdma_ll.h"

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#endif

#if SOC_GDMA_SUPPORTED
#include "esp_private/gdma.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_I2S0_DATA_BIT_WIDTH_8BIT
#  define ESPRESSIF_I2S0_DATA_BIT_WIDTH 8
#elif CONFIG_ESPRESSIF_I2S0_DATA_BIT_WIDTH_16BIT
#  define ESPRESSIF_I2S0_DATA_BIT_WIDTH 16
#elif CONFIG_ESPRESSIF_I2S0_DATA_BIT_WIDTH_24BIT
#  define ESPRESSIF_I2S0_DATA_BIT_WIDTH 24
#elif CONFIG_ESPRESSIF_I2S0_DATA_BIT_WIDTH_32BIT
#  define ESPRESSIF_I2S0_DATA_BIT_WIDTH 32
#endif

/* I2S DMA RX/TX description number */

#define I2S_DMADESC_NUM                 (CONFIG_I2S_DMADESC_NUM)

/* I2S DMA channel number */

#define I2S_DMA_CHANNEL_MAX (2)

#ifdef CONFIG_ESPRESSIF_I2S0_TX
#  define I2S0_TX_ENABLED 1
#else
#  define I2S0_TX_ENABLED 0
#endif

#ifdef CONFIG_ESPRESSIF_I2S0_RX
#  define I2S0_RX_ENABLED 1
#else
#  define I2S0_RX_ENABLED 0
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
#  define CONFIG_ESPRESSIF_I2S_DUMPBUFFERS
#else
#  undef CONFIG_ESPRESSIF_I2S_DUMPBUFFERS
#endif

#define I2S_GPIO_UNUSED -1      /* For signals which are not used */

#define I2S_TDM_AUTO_SLOT_NUM    (0)
#define I2S_TDM_AUTO_WS_WIDTH    (0)
#define I2S_TDM_AUTO_SLOT        (I2S_TDM_SLOT0 | I2S_TDM_SLOT1)

#define I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(cfg, bits_per_sample, mask) \
    cfg.slot_mask = (mask),                                             \
    cfg.ws_width = I2S_TDM_AUTO_WS_WIDTH,                               \
    cfg.ws_pol = false,                                                 \
    cfg.bit_shift = true,                                               \
    cfg.left_align = false,                                             \
    cfg.big_endian = false,                                             \
    cfg.bit_order_lsb = false,                                          \
    cfg.skip_mask = false                                               \

#define I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(cfg, bits_per_sample, mask) \
    cfg.slot_mask = (mask),                                         \
    cfg.ws_width = I2S_TDM_AUTO_WS_WIDTH,                           \
    cfg.ws_pol = false,                                             \
    cfg.bit_shift = false,                                          \
    cfg.left_align = false,                                         \
    cfg.big_endian = false,                                         \
    cfg.bit_order_lsb = false,                                      \
    cfg.skip_mask = false                                           \

#define I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(cfg, bits_per_sample, mask) \
    cfg.slot_mask = (mask),                                               \
    cfg.ws_width = 1,                                                     \
    cfg.ws_pol = true,                                                    \
    cfg.bit_shift = true,                                                 \
    cfg.left_align = false,                                               \
    cfg.big_endian = false,                                               \
    cfg.bit_order_lsb = false,                                            \
    cfg.skip_mask = false                                                 \

#define I2S_PDM_TX_SLOT_DEFAULT_CONFIG(cfg)                   \
    cfg.sd_prescale = 0,                                      \
    cfg.sd_scale = I2S_PDM_SIG_SCALING_MUL_1,                 \
    cfg.hp_scale = I2S_PDM_SIG_SCALING_DIV_2,                 \
    cfg.lp_scale = I2S_PDM_SIG_SCALING_MUL_1,                 \
    cfg.sinc_scale = I2S_PDM_SIG_SCALING_MUL_1,               \
    cfg.line_mode = I2S_PDM_TX_ONE_LINE_CODEC,                \
    cfg.hp_en = true,                                         \
    cfg.hp_cut_off_freq_hzx10 = 35.5,                         \
    cfg.sd_dither = 0,                                        \
    cfg.sd_dither2 = 1                                        \

#if SOC_PERIPH_CLK_CTRL_SHARED
#  define I2S_CLOCK_SRC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#  define I2S_CLOCK_SRC_ATOMIC()
#endif

#if !SOC_RCC_IS_INDEPENDENT
#  define I2S_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#  define I2S_RCC_ATOMIC()
#endif

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#  define I2S_DMA_BUFFER_MAX_SIZE   DMA_DESCRIPTOR_BUFFER_MAX_SIZE_64B_ALIGNED
#else
#  define I2S_DMA_BUFFER_MAX_SIZE   DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Multiplier of MCLK to sample rate */

typedef enum
{
  I2S_MCLK_MULTIPLE_128 = 128,  /* mclk = sample_rate * 128 */
  I2S_MCLK_MULTIPLE_256 = 256,  /* mclk = sample_rate * 256 */
  I2S_MCLK_MULTIPLE_384 = 384,  /* mclk = sample_rate * 384 */
  I2S_MCLK_MULTIPLE_512 = 512,  /* mclk = sample_rate * 512 */
} i2s_mclk_multiple_t;

/* I2S Audio Standard Mode */

typedef enum
{
  I2S_TDM_PHILIPS = 0,
  I2S_TDM_MSB,
  I2S_TDM_PCM,
  I2S_PDM,
} i2s_audio_mode_t;

/* I2S Device hardware configuration */

struct esp_i2s_config_s
{
  uint32_t port;                    /* I2S port */
  uint32_t role;                    /* I2S port role (master or slave) */
  uint8_t data_width;               /* I2S sample data width */
  uint32_t rate;                    /* I2S sample-rate */
  uint32_t total_slot;              /* Total slot number */

  bool tx_en;                       /* Is TX enabled? */
  bool rx_en;                       /* Is RX enabled? */
  int8_t mclk_pin;                  /* MCLK pin, output */

  int tx_clk_src;                   /* Select the I2S TX source clock */
  int rx_clk_src;                   /* Select the I2S TX source clock */

  /* BCLK pin, input in slave role, output in master role */

  int8_t bclk_pin;

  /* WS pin, input in slave role, output in master role */

  int8_t ws_pin;

  int8_t dout_pin;                  /* DATA pin, output */
  int8_t din_pin;                   /* DATA pin, input */

  uint8_t  audio_std_mode;          /* Select audio standard (i2s_audio_mode_t) */

  /* WS signal polarity, set true to enable high level first */

  bool ws_pol;

  i2s_hal_context_t *ctx;           /* Common layer struct */
  i2s_hal_clock_info_t *clk_info;   /* Common layer clock info struct */
};

struct esp_buffer_s
{
  struct esp_buffer_s *flink; /* Supports a singly linked list */

  /* The associated DMA in/outlink */

  lldesc_t *dma_link[I2S_DMADESC_NUM];

  i2s_callback_t callback;      /* DMA completion callback */
  uint32_t timeout;             /* Timeout value of the DMA transfers */
  void *arg;                    /* Callback's argument */
  struct ap_buffer_s *apb;      /* The audio buffer */
  uint8_t *buf;                 /* The DMA's descriptor buffer */
  uint32_t nbytes;              /* The DMA's descriptor buffer size */
  int result;                   /* The result of the transfer */
};

/* Internal buffer must be aligned to the bytes_per_frame. Sometimes,
 * however, the audio buffer is not aligned and additional bytes must
 * be copied to be inserted on the next buffer. This structure keeps
 * track of the bytes that were not written to the internal buffer yet.
 */

struct esp_buffer_carry_s
{
  uint32_t value;
  size_t bytes;
};

/* This structure describes the state of one receiver or transmitter
 * transport.
 */

struct esp_transport_s
{
  sq_queue_t pend;              /* A queue of pending transfers */
  sq_queue_t act;               /* A queue of active transfers */
  sq_queue_t done;              /* A queue of completed transfers */
  struct work_s work;           /* Supports worker thread operations */

  /* Bytes to be written at the beginning of the next DMA buffer */

  struct esp_buffer_carry_s carry;
};

/* The state of the one I2S peripheral */

struct esp_i2s_s
{
  struct i2s_dev_s  dev;        /* Externally visible I2S interface */
  mutex_t           lock;       /* Ensures mutually exclusive access */
  uint8_t           cpu;        /* CPU ID */
  spinlock_t        slock;      /* Device specific lock. */

  /* Port configuration */

  const struct esp_i2s_config_s *config;

  uint32_t               mclk_freq;      /* I2S actual master clock */
  uint32_t               mclk_multiple;  /* The multiple of mclk to the sample rate */
  uint32_t               channels;       /* Audio channels (1:mono or 2:stereo) */
  uint32_t               rate;           /* I2S actual configured sample-rate */
  uint32_t               data_width;     /* I2S actual configured data_width */
  gdma_channel_handle_t  dma_channel_tx; /* I2S DMA TX channel being used */
  gdma_channel_handle_t  dma_channel_rx; /* I2S DMA RX channel being used */

  struct esp_transport_s tx;  /* TX transport state */

  bool tx_started;                /* TX channel started */

  struct esp_transport_s rx;  /* RX transport state */

  bool rx_started;                /* RX channel started */

  bool streaming;                 /* Is I2S peripheral active? */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                         /* Buffer wait semaphore */
  struct esp_buffer_s *bf_freelist;     /* A list a free buffer containers */
  struct esp_buffer_s containers[CONFIG_ESPRESSIF_I2S_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_ESPRESSIF_I2S_DUMPBUFFERS
#  define       i2s_dump_buffer(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define       i2s_dump_buffer(m,b,s)
#endif

/* I2S configuration */

static int i2s_configure(struct esp_i2s_s *priv);

/* Buffer container helpers */

static struct esp_buffer_s *
                i2s_buf_allocate(struct esp_i2s_s *priv);
static int      i2s_buf_free(struct esp_i2s_s *priv,
                             struct esp_buffer_s *bfcontainer);
static int      i2s_buf_initialize(struct esp_i2s_s *priv);

/* I2S DMA setup function */

static uint32_t i2s_common_dma_setup(struct esp_buffer_s *bfcontainer,
                                     bool tx, uint32_t len);

/* DMA support */

static IRAM_ATTR int  i2s_txdma_setup(struct esp_i2s_s *priv,
                                      struct esp_buffer_s *bfcontainer);
static void           i2s_tx_worker(void *arg);
static void           i2s_tx_schedule(struct esp_i2s_s *priv,
                                      lldesc_t *outlink);

static IRAM_ATTR int  i2s_rxdma_setup(struct esp_i2s_s *priv,
                                      struct esp_buffer_s *bfcontainer);
static void           i2s_rx_worker(void *arg);
static void           i2s_rx_schedule(struct esp_i2s_s *priv,
                                      lldesc_t *inlink);

/* I2S methods (and close friends) */
#if SOC_I2S_SUPPORTS_APLL
static uint32_t i2s_set_get_apll_freq(uint32_t mclk_freq_hz);
#endif
static uint32_t i2s_get_source_clk_freq(i2s_clock_src_t clk_src,
                                        uint32_t mclk_freq_hz);
static int32_t  i2s_check_mclkfrequency(struct esp_i2s_s *priv);
static uint32_t i2s_set_datawidth(struct esp_i2s_s *priv);
static int      i2s_set_clock(struct esp_i2s_s *priv);
static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev);
static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency);
static int      i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg);

static void     i2s_tx_channel_start(struct esp_i2s_s *priv);
static int      i2s_tx_channel_stop(struct esp_i2s_s *priv);
static int      i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                         i2s_callback_t callback, void *arg,
                         uint32_t timeout);

static void     i2s_rx_channel_start(struct esp_i2s_s *priv);
static int      i2s_rx_channel_stop(struct esp_i2s_s *priv);
static bool     i2s_tx_error(gdma_channel_handle_t dma_chan,
                             gdma_event_data_t *event_data,
                             void *arg);
static bool     i2s_rx_error(gdma_channel_handle_t dma_chan,
                             gdma_event_data_t *event_data,
                             void *arg);

static int      i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                            i2s_callback_t callback, void *arg,
                            uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_i2sops =
{
  .i2s_txchannels     = i2s_txchannels,
  .i2s_txsamplerate   = i2s_txsamplerate,
  .i2s_txdatawidth    = i2s_txdatawidth,
  .i2s_send           = i2s_send,

  .i2s_rxchannels     = i2s_rxchannels,
  .i2s_rxsamplerate   = i2s_rxsamplerate,
  .i2s_rxdatawidth    = i2s_rxdatawidth,
  .i2s_receive        = i2s_receive,

  .i2s_ioctl             = i2s_ioctl,
  .i2s_getmclkfrequency  = i2s_getmclkfrequency,
  .i2s_setmclkfrequency  = i2s_setmclkfrequency,
};

#ifdef CONFIG_ESPRESSIF_I2S0

i2s_hal_context_t ctx_i2s0 =
{
  0
};

i2s_hal_clock_info_t clk_info_i2s0 =
{
  0
};

static struct esp_i2s_config_s esp_i2s0_config =
{
  .port             = 0,
#ifdef CONFIG_ESPRESSIF_I2S0_ROLE_MASTER
  .role             = I2S_ROLE_MASTER,
#else
  .role             = I2S_ROLE_SLAVE,
#endif /* CONFIG_ESPRESSIF_I2S0_ROLE_MASTER */
  .data_width       = ESPRESSIF_I2S0_DATA_BIT_WIDTH,
  .rate             = CONFIG_ESPRESSIF_I2S0_SAMPLE_RATE,
  .total_slot       = 2,
  .tx_en            = I2S0_TX_ENABLED,
  .rx_en            = I2S0_RX_ENABLED,
  .tx_clk_src       = I2S_CLK_SRC_DEFAULT,
  .rx_clk_src       = I2S_CLK_SRC_DEFAULT,
#ifdef CONFIG_ESPRESSIF_I2S0_MCLK
  .mclk_pin         = CONFIG_ESPRESSIF_I2S0_MCLKPIN,
#else
  .mclk_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESPRESSIF_I2S0_MCLK */
  .bclk_pin         = CONFIG_ESPRESSIF_I2S0_BCLKPIN,
  .ws_pin           = CONFIG_ESPRESSIF_I2S0_WSPIN,
#ifdef CONFIG_ESPRESSIF_I2S0_DOUTPIN
  .dout_pin         = CONFIG_ESPRESSIF_I2S0_DOUTPIN,
#else
  .dout_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESPRESSIF_I2S0_DOUTPIN */
#ifdef CONFIG_ESPRESSIF_I2S0_DINPIN
  .din_pin          = CONFIG_ESPRESSIF_I2S0_DINPIN,
#else
  .din_pin          = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESPRESSIF_I2S0_DINPIN */
  .audio_std_mode   = I2S_TDM_PHILIPS,
  .ctx              = &ctx_i2s0,
  .clk_info         = &clk_info_i2s0,
};

static struct esp_i2s_s esp_i2s0_priv =
{
  .dev =
  {
    .ops = &g_i2sops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .slock = SP_UNLOCKED,
  .config = &esp_i2s0_config,
  .bufsem = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESPRESSIF_I2S0 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the I2S state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct esp_buffer_s *i2s_buf_allocate(struct esp_i2s_s *priv)
{
  struct esp_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;
  int alignment;
  int i;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  ret = nxsem_wait_uninterruptible(&priv->bufsem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get the buffer from the head of the free list */

  flags = spin_lock_irqsave(&priv->slock);
  bfcontainer = priv->bf_freelist;
  DEBUGASSERT(bfcontainer);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  alignment = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM,
                                            CACHE_TYPE_DATA);
#else
  alignment = sizeof(uint32_t);
#endif

  for (i = 0; i < I2S_DMADESC_NUM; i++)
    {
      size_t size = ALIGN_UP(sizeof(lldesc_t), alignment);
      bfcontainer->dma_link[i] = (lldesc_t *)kmm_memalign(alignment, size);
    }

  /* Unlink the buffer from the freelist */

  priv->bf_freelist = bfcontainer->flink;
  spin_unlock_irqrestore(&priv->slock, flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: i2s_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv        - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 * Assumptions:
 *   The caller has exclusive access to the I2S state structure
 *
 ****************************************************************************/

static int i2s_buf_free(struct esp_i2s_s *priv,
                        struct esp_buffer_s *bfcontainer)
{
  irqstate_t flags;
  int i;

  /* Put the buffer container back on the free list (circbuf) */

  flags = spin_lock_irqsave(&priv->slock);

  for (i = 0; i < I2S_DMADESC_NUM; i++)
    {
      kmm_free(bfcontainer->dma_link[i]);
      bfcontainer->dma_link[i] = NULL;
    }

  bfcontainer->apb = NULL;
  bfcontainer->buf = NULL;
  bfcontainer->nbytes = 0;
  bfcontainer->flink  = priv->bf_freelist;
  priv->bf_freelist = bfcontainer;

  spin_unlock_irqrestore(&priv->slock, flags);

  /* Wake up any threads waiting for a buffer container */

  return nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: i2s_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 * Assumptions:
 *   Called early in I2S initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static int i2s_buf_initialize(struct esp_i2s_s *priv)
{
  int ret;

  priv->tx.carry.bytes = 0;
  priv->tx.carry.value = 0;

  priv->bf_freelist = NULL;
  for (int i = 0; i < CONFIG_ESPRESSIF_I2S_MAXINFLIGHT; i++)
    {
      ret = i2s_buf_free(priv, &priv->containers[i]);
      if (ret < 0)
        {
          i2serr("Failed to free buffer container: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_common_dma_setup
 *
 * Description:
 *   Set up I2S DMA descriptors with given parameters using lldesc_t.
 *   This function is based on esp_dma_setup but adapted for I2S using
 *   lldesc_t descriptors and struct esp_buffer_s.
 *
 * Input Parameters:
 *   bfcontainer - Buffer container with DMA descriptors
 *   tx          - true: TX mode; false: RX mode
 *   len         - Buffer length by byte
 *
 * Returned Value:
 *   Number of bytes bound to descriptors
 *
 ****************************************************************************/

static uint32_t i2s_common_dma_setup(struct esp_buffer_s *bfcontainer,
                                     bool tx, uint32_t len)
{
  int i;
  uint32_t bytes = len;
  uint8_t *pdata = bfcontainer->buf;
  uint32_t data_len;
  uint32_t buf_len;
  lldesc_t *dma_desc;

  DEBUGASSERT(bfcontainer != NULL);
  DEBUGASSERT(bfcontainer->dma_link != NULL);
  DEBUGASSERT(bfcontainer->buf != NULL);
  DEBUGASSERT(len > 0);

  for (i = 0; i < I2S_DMADESC_NUM; i++)
    {
      data_len = MIN(bytes, I2S_DMA_BUFFER_MAX_SIZE);

      /* Buffer length must be rounded to next 32-bit boundary. */

      buf_len = ALIGN_UP(data_len, sizeof(uintptr_t));

      dma_desc = bfcontainer->dma_link[i];
      dma_desc->size = buf_len;
      dma_desc->length = tx ? data_len : 0;
      dma_desc->owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
      dma_desc->buf = pdata;
      dma_desc->eof = 0;
      dma_desc->sosf = 0;
      dma_desc->offset = 0;

      /* Link to the next descriptor */

      if (i < (I2S_DMADESC_NUM - 1))
        {
          STAILQ_NEXT(dma_desc, qe) = bfcontainer->dma_link[i + 1];
        }
      else
        {
          STAILQ_NEXT(dma_desc, qe) = NULL;
        }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
      esp_cache_msync(dma_desc, sizeof(lldesc_t),
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

  /* Set EOF flag on the last descriptor */

  dma_desc->eof = tx ? 1 : 0;

  /* Set the next pointer to NULL on the last descriptor */

  STAILQ_NEXT(dma_desc, qe) = NULL;

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync(dma_desc, sizeof(lldesc_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M | \
                  ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif

  return len - bytes;
}

/****************************************************************************
 * Name: i2s_txdma_start
 *
 * Description:
 *   Initiate the next TX DMA transfer. The DMA outlink was previously bound
 *   so it is safe to start the next DMA transfer at interrupt level.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int IRAM_ATTR i2s_txdma_start(struct esp_i2s_s *priv)
{
  struct esp_buffer_s *bfcontainer;
  esp_err_t err;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->tx.act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->tx.pend))
    {
      return OK;
    }

  i2s_hal_tx_reset(priv->config->ctx);

  /* Reset the DMA operation */

  err = gdma_reset(priv->dma_channel_tx);
  if (err != ESP_OK)
    {
      i2serr("Failed to reset DMA channel: %d\n", err);
      return -EINVAL;
    }

  /* Reset TX FIFO */

  i2s_hal_tx_reset_fifo(priv->config->ctx);

  /* Start transmission if no data is already being transmitted */

  bfcontainer = (struct esp_buffer_s *)sq_remfirst(&priv->tx.pend);

  err = gdma_start(priv->dma_channel_tx, (intptr_t)bfcontainer->dma_link[0]);
  if (err != ESP_OK)
    {
      i2serr("Failed to start DMA channel: %d\n", err);
      return -EINVAL;
    }

  i2s_hal_tx_start(priv->config->ctx);

  priv->tx_started = true;

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.act);

  return OK;
}

/****************************************************************************
 * Name: i2s_rxdma_start
 *
 * Description:
 *   Initiate the next RX DMA transfer. Assuming the DMA inlink is already
 *   bound, it's safe to start the next DMA transfer in an interrupt context.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int i2s_rxdma_start(struct esp_i2s_s *priv)
{
  struct esp_buffer_s *bfcontainer;
  size_t eof_nbytes;
  esp_err_t err;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->rx.act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->rx.pend))
    {
      return OK;
    }

  i2s_hal_rx_reset(priv->config->ctx);

  /* Reset the DMA operation */

  err = gdma_reset(priv->dma_channel_rx);
  if (err != ESP_OK)
    {
      i2serr("Failed to reset DMA channel: %d\n", err);
      return -EINVAL;
    }

  /* Reset RX FIFO */

  i2s_hal_rx_reset_fifo(priv->config->ctx);

  bfcontainer = (struct esp_buffer_s *)sq_remfirst(&priv->rx.pend);

  eof_nbytes = MIN(bfcontainer->nbytes, I2S_DMA_BUFFER_MAX_SIZE);

  i2s_ll_rx_set_eof_num(priv->config->ctx->dev, eof_nbytes);

  /* If there isn't already an active transmission in progress,
   * then start it.
   */

  err = gdma_start(priv->dma_channel_rx, (intptr_t)bfcontainer->dma_link[0]);
  if (err != ESP_OK)
    {
      i2serr("Failed to start DMA channel: %d\n", err);
      return -EINVAL;
    }

  i2s_hal_rx_start(priv->config->ctx);

  priv->rx_started = true;

  sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.act);

  return OK;
}

/****************************************************************************
 * Name: i2s_txdma_setup
 *
 * Description:
 *   Setup the next TX DMA transfer
 *
 * Input Parameters:
 *   priv        - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be set up
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static IRAM_ATTR int i2s_txdma_setup(struct esp_i2s_s *priv,
                                     struct esp_buffer_s *bfcontainer)
{
  int ret = OK;
  size_t carry_size;
  uint32_t bytes_queued;
  uint32_t data_copied;
  struct ap_buffer_s *apb;
  apb_samp_t samp_size;
  apb_samp_t bytes_per_sample;
  uint16_t bytes_per_frame;
  irqstate_t flags;
  uint8_t *buf;
  uint8_t padding;
  uint8_t *samp;
  uint32_t alignment;
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  uint32_t bufsize;
#endif

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  apb = bfcontainer->apb;

  /* Get the transfer information, accounting for any data offset */

  bytes_per_sample = (priv->data_width + 7) / 8;
  bytes_per_frame = bytes_per_sample * priv->channels;

  samp = &apb->samp[apb->curbyte];
  samp_size = (apb->nbytes - apb->curbyte) + priv->tx.carry.bytes;

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  /* bufsize need to align with cache line size */

  bufsize = samp_size;
  alignment = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM,
                                            CACHE_TYPE_DATA);

  /* First, calculate if the buffer contains a complete sample */

  carry_size = samp_size % bytes_per_frame;

  bufsize -= carry_size;

  /* Now, the buffer contains complete samples */

  uint32_t aligned_frame_num = bufsize / bytes_per_frame;

  /* To make the buffer aligned with the cache line size, search for the ceil
   * aligned size first. If the buffer size exceed the max DMA buffer size,
   * toggle the sign to search for the floor aligned size.
   */

  for (; bufsize % alignment != 0; aligned_frame_num--)
    {
      bufsize = aligned_frame_num * bytes_per_frame;
      carry_size += bytes_per_frame;
    }

  DEBUGASSERT((samp_size - carry_size) % alignment == 0);

#else
  alignment = sizeof(uint32_t);
  carry_size = samp_size % bytes_per_frame;
#endif

  DEBUGASSERT((samp_size - carry_size) % bytes_per_frame == 0);

  /* Allocate the current audio buffer considering the remaining bytes
   * carried from the last upper half audio buffer.
   */

  bfcontainer->buf = kmm_memalign(alignment, bfcontainer->nbytes);
  if (bfcontainer->buf == NULL)
    {
      i2serr("Failed to allocate the DMA internal buffer "
             "[%" PRIu32 " bytes]", bfcontainer->nbytes);
      return -ENOMEM;
    }

  data_copied = 0;
  buf = bfcontainer->buf;

  /* Copy the remaining bytes from the last audio buffer to the current
   * audio buffer. The remaining bytes are part of a sample that was split
   * between the last and the current audio buffer. Also, copy the bytes
   * from that split sample that are on the current buffer to the internal
   * buffer.
   */

  if (priv->tx.carry.bytes)
    {
      memcpy(buf, &priv->tx.carry.value, priv->tx.carry.bytes);
      buf += priv->tx.carry.bytes;
      data_copied += priv->tx.carry.bytes;
      memcpy(buf, samp, (bytes_per_frame - priv->tx.carry.bytes));
      buf += (bytes_per_frame - priv->tx.carry.bytes);
      samp += (bytes_per_frame - priv->tx.carry.bytes);
      data_copied += (bytes_per_frame - priv->tx.carry.bytes);
    }

  /* Copy the upper half buffer to the internal buffer considering that
   * the current upper half buffer may not contain a complete sample at
   * the end of the buffer (and those bytes needs to be carried to the
   * next audio buffer).
   */

  memcpy(buf, samp, samp_size - (data_copied + carry_size));
  buf += samp_size - (data_copied + carry_size);
  samp += samp_size - (data_copied + carry_size);
  data_copied += samp_size - (data_copied + carry_size);

  /* If the audio buffer's size is not a multiple of the sample size,
   * it's necessary to carry the remaining bytes that are part of what
   * would be the last sample on this buffer. These bytes will then be
   * saved and inserted at the beginning of the next DMA buffer to
   * rebuild the sample correctly.
   */

  priv->tx.carry.bytes = carry_size;
  if (priv->tx.carry.bytes)
    {
      memcpy((uint8_t *)&priv->tx.carry.value, samp, priv->tx.carry.bytes);
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync((void *)bfcontainer->buf,
                  bfcontainer->nbytes,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif

  /* Configure DMA stream */

  bytes_queued = i2s_common_dma_setup(bfcontainer,
                                      true,
                                      bfcontainer->nbytes);

  if (bytes_queued != bfcontainer->nbytes)
    {
      i2serr("Failed to enqueue I2S buffer "
             "(%" PRIu32 " bytes of %" PRIu32 ")\n",
             bytes_queued, bfcontainer->nbytes);
      return -bytes_queued;
    }

  flags = spin_lock_irqsave(&priv->slock);

  /* Add the buffer container to the end of the TX pending queue */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.pend);

  /* Trigger DMA transfer if no transmission is in progress */

  ret = i2s_txdma_start(priv);

  spin_unlock_irqrestore(&priv->slock, flags);

  return ret;
}

/****************************************************************************
 * Name: i2s_rxdma_setup
 *
 * Description:
 *   Setup the next RX DMA transfer
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be set up
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static IRAM_ATTR int i2s_rxdma_setup(struct esp_i2s_s *priv,
                                     struct esp_buffer_s *bfcontainer)
{
  int ret = OK;
  uint32_t bytes_queued;
  irqstate_t flags;
  uint32_t alignment;

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  /* bufsize need to align with cache line size */

  alignment = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM,
                                            CACHE_TYPE_DATA);
#else
  alignment = sizeof(uint32_t);
#endif

  bfcontainer->buf = kmm_memalign(alignment, bfcontainer->nbytes);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync((void *)bfcontainer,
                  sizeof(struct esp_buffer_s),
                  (ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                   ESP_CACHE_MSYNC_FLAG_UNALIGNED));
#endif

  /* Configure DMA stream */

  bytes_queued = i2s_common_dma_setup(bfcontainer,
                                      false,
                                      bfcontainer->nbytes);

  if (bytes_queued != bfcontainer->nbytes)
    {
      i2serr("Failed to enqueue I2S buffer "
             "(%" PRIu32 " bytes of %" PRIu32 ")\n",
             bytes_queued, bfcontainer->nbytes);
      return -bytes_queued;
    }

  flags = spin_lock_irqsave(&priv->slock);

  /* Add the buffer container to the end of the RX pending queue */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.pend);

  /* Trigger DMA transfer if no transmission is in progress */

  ret = i2s_rxdma_start(priv);

  spin_unlock_irqrestore(&priv->slock, flags);

  return ret;
}

/****************************************************************************
 * Name: i2s_tx_schedule
 *
 * Description:
 *   An TX DMA completion has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   outlink - DMA outlink descriptor that triggered the interrupt.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 ****************************************************************************/

static void IRAM_ATTR i2s_tx_schedule(struct esp_i2s_s *priv,
                                      lldesc_t *outlink)
{
  struct esp_buffer_s *bfcontainer;
  lldesc_t *bfdesc;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.
   */

  /* Move all entries from the tx.act queue to the tx.done queue */

  if (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct esp_buffer_s *)sq_peek(&priv->tx.act);

      /* Check if the DMA descriptor that generated an EOF interrupt is the
       * last descriptor of the current buffer container's DMA outlink.
       * REVISIT: what to do if we miss synchronization and the descriptor
       * that generated the interrupt is different from the expected (the
       * oldest of the list containing active transmissions)?
       */

      bfdesc = bfcontainer->dma_link[0];

      while (bfdesc->eof == 0 && bfdesc != NULL)
        {
          bfdesc = STAILQ_NEXT(bfdesc, qe);
        }

      if (bfdesc == outlink)
        {
          sq_remfirst(&priv->tx.act);

          /* Report the result of the transfer */

          bfcontainer->result = OK;

          /* Add the completed buffer container to the tail of the tx.done
           * queue
           */

          sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.done);

          /* Check if the DMA is IDLE */

          if (sq_empty(&priv->tx.act))
            {
              /* Then start the next DMA. */

              i2s_txdma_start(priv);
            }
        }

      /* If the worker has completed running, then reschedule the working
       * thread.
       */

      if (work_available(&priv->tx.work))
        {
          /* Schedule the TX DMA done processing to occur on the worker
           * thread.
           */

          ret = work_queue(HPWORK, &priv->tx.work, i2s_tx_worker, priv, 0);
          if (ret != 0)
            {
              i2serr("ERROR: Failed to queue TX work: %d\n", ret);
            }
        }
    }
}

/****************************************************************************
 * Name: i2s_rx_schedule
 *
 * Description:
 *   An RX DMA completion has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   inlink - DMA inlink descriptor that triggered the interrupt.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 ****************************************************************************/

static void i2s_rx_schedule(struct esp_i2s_s *priv,
                            lldesc_t *inlink)
{
  struct esp_buffer_s *bfcontainer;
  lldesc_t *bfdesc;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->rx.act queue.
   */

  /* Move all entries from the rx.act queue to the rx.done queue */

  if (!sq_empty(&priv->rx.act))
    {
      /* Remove the next buffer container from the rx.act list */

      bfcontainer = (struct esp_buffer_s *)sq_peek(&priv->rx.act);

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link[0];

      while (bfdesc->eof == 1 && STAILQ_NEXT(bfdesc, qe) != NULL)
        {
          bfdesc = STAILQ_NEXT(bfdesc, qe);
        }

      if (bfdesc == inlink)
        {
          sq_remfirst(&priv->rx.act);

          /* Report the result of the transfer */

          bfcontainer->result = OK;

          /* Add the completed buffer container to the tail of the rx.done
           * queue
           */

          sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.done);

          /* Check if the DMA is IDLE */

          if (sq_empty(&priv->rx.act))
            {
              /* Then start the next DMA. */

              i2s_rxdma_start(priv);
            }
        }

      /* If the worker has completed running, then reschedule the working
       * thread.
       */

      if (work_available(&priv->rx.work))
        {
          /* Schedule the RX DMA done processing to occur on the worker
           * thread.
           */

          ret = work_queue(HPWORK, &priv->rx.work, i2s_rx_worker, priv, 0);
          if (ret != 0)
            {
              i2serr("ERROR: Failed to queue RX work: %d\n", ret);
            }
        }
    }
}

/****************************************************************************
 * Name: i2s_tx_worker
 *
 * Description:
 *   TX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_tx_worker(void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;
  struct esp_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the tx.pend queue and saved in the tx.act queue.  We get here when
   * the DMA is finished.
   *
   * In any case, the buffer containers in tx.act will be moved to the end
   * of the tx.done queue and tx.act will be emptied before this worker is
   * started.
   *
   */

  i2sinfo("tx.act.head=%p tx.done.head=%p\n",
          priv->tx.act.head, priv->tx.done.head);

  /* Process each buffer in the tx.done queue */

  while (sq_peek(&priv->tx.done) != NULL)
    {
      /* Remove the buffer container from the tx.done queue.  NOTE that
       * interrupts must be disabled to do this because the tx.done queue is
       * also modified from the interrupt level.
       */

      flags = spin_lock_irqsave(&priv->slock);
      bfcontainer = (struct esp_buffer_s *)sq_remfirst(&priv->tx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      /* Perform the TX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release the internal buffer used by the DMA outlink */

      kmm_free(bfcontainer->buf);

      /* And release the buffer container */

      VERIFY(i2s_buf_free(priv, bfcontainer));
    }

  /* TX channel can only be stopped here if either 1) the RX channel is
   * disabled or 2) the I2S is in slave role. If the I2S is in master role,
   * the TX channel can only be stopped after the RX channel has finished
   * because the WS and BCLK are shared with the RX.
   */

  if ((!priv->config->rx_en || priv->config->role == I2S_ROLE_SLAVE) &&
      (sq_empty(&priv->tx.act) && sq_empty(&priv->tx.pend)))
    {
      i2s_tx_channel_stop(priv);

      /* If the I2S is in slave role and RX is enabled, the TX's WS and
       * BCLK are shared with the RX. Then, the RX channel can only be
       * stopped after the TX channel has finished.
       */

      if ((priv->config->rx_en) &&
          (sq_empty(&priv->rx.act) && sq_empty(&priv->rx.pend)))
        {
          i2s_rx_channel_stop(priv);
        }
    }
}

/****************************************************************************
 * Name: i2s_rx_worker
 *
 * Description:
 *   RX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_rx_worker(void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;
  struct esp_buffer_s *bfcontainer;
  lldesc_t *dmadesc;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the rx.pend queue and saved in the rx.act queue. We get here when
   * the DMA is finished.
   *
   * In any case, the buffer containers in rx.act will be moved to the end
   * of the rx.done queue and rx.act will be emptied before this worker is
   * started.
   *
   */

  i2sinfo("rx.act.head=%p rx.done.head=%p\n",
          priv->rx.act.head, priv->rx.done.head);

  /* Process each buffer in the rx.done queue */

  while (sq_peek(&priv->rx.done) != NULL)
    {
      /* Remove the buffer container from the rx.done queue.  NOTE that
       * interrupts must be disabled to do this because the rx.done queue is
       * also modified from the interrupt level.
       */

      flags = spin_lock_irqsave(&priv->slock);
      bfcontainer = (struct esp_buffer_s *)sq_remfirst(&priv->rx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      bfcontainer->apb->nbytes = 0;

      dmadesc = bfcontainer->dma_link[0];

      do
        {
          memcpy(bfcontainer->apb->samp + bfcontainer->apb->nbytes,
                 (const void *)dmadesc->buf,
                 dmadesc->length);
          bfcontainer->apb->nbytes += dmadesc->length;
          dmadesc = STAILQ_NEXT(dmadesc, qe);
        }
      while (dmadesc != NULL && dmadesc->eof == 1);

      /* Perform the RX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);

      if (priv->streaming == false)
        {
          bfcontainer->apb->flags |= AUDIO_APB_FINAL;
        }

      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release the internal buffer used by the DMA inlink */

      kmm_free(bfcontainer->buf);

      /* And release the buffer container */

      VERIFY(i2s_buf_free(priv, bfcontainer));
    }

  /* RX channel can only be stopped here if either 1) the TX channel is
   * disabled or 2) the I2S is in master role. If the I2S is in slave role,
   * the RX channel can only be stopped after the TX channel has finished
   * because the WS and BCLK are shared with the TX.
   */

  if ((!priv->config->tx_en || priv->config->role == I2S_ROLE_MASTER) &&
      (sq_empty(&priv->rx.act) && sq_empty(&priv->rx.pend)))
    {
      i2s_rx_channel_stop(priv);

      /* If the I2S is in master role and TX is enabled, the RX's WS and
       * BCLK are shared with the TX. Then, the TX channel can only be
       * stopped after the RX channel has finished.
       */

      if ((priv->config->tx_en) &&
          (sq_empty(&priv->tx.act) && sq_empty(&priv->tx.pend)))
        {
          i2s_tx_channel_stop(priv);
        }
    }
}

/****************************************************************************
 * Name: i2s_configure
 *
 * Description:
 *   Configure I2S
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure.  This function
 *          will complete the I2S specific portions of the initialization
 *
 * Returned Value:
 *   Returns OK on success, a negative error code on failure
 *
 ****************************************************************************/

static int i2s_configure(struct esp_i2s_s *priv)
{
  uint32_t tx_conf  = 0;
  uint32_t rx_conf  = 0;
  uint32_t port;
  int ret;
  i2s_hal_slot_config_t tx_slot_cfg =
    {
      0
    };

  i2s_hal_slot_config_t rx_slot_cfg =
    {
      0
    };

  port = priv->config->port;

  i2s_hal_init(priv->config->ctx, port);
  I2S_RCC_ATOMIC()
    {
      i2s_ll_enable_bus_clock(port, true);
      i2s_ll_reset_register(port);
      i2s_ll_enable_core_clock(I2S_LL_GET_HW(port), true);
    }

  /* Configure multiplexed pins as connected on the board */

  /* Enable TX channel */

  if (priv->config->dout_pin != I2S_GPIO_UNUSED)
    {
      /* If TX channel is used, enable the clock source */

      esp_gpiowrite(priv->config->dout_pin, 1);
      esp_gpiowrite(priv->config->dout_pin, 0);
      esp_gpiowrite(priv->config->dout_pin, 1);
      esp_configgpio(priv->config->dout_pin, OUTPUT_FUNCTION_2);
      esp_gpio_matrix_out(priv->config->dout_pin,
                          i2s_periph_signal[port].data_out_sigs[0], 0, 0);
    }

  /* Enable RX channel */

  if (priv->config->din_pin != I2S_GPIO_UNUSED)
    {
      /* If RX channel is used, enable the clock source */

      /* Check for loopback mode */

      if (priv->config->dout_pin != I2S_GPIO_UNUSED &&
          priv->config->din_pin == priv->config->dout_pin)
        {
          esp_configgpio(priv->config->din_pin,
                         INPUT_FUNCTION_2 | OUTPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->din_pin,
                             i2s_periph_signal[port].data_in_sig, 0);
          esp_gpio_matrix_out(priv->config->din_pin,
                              i2s_periph_signal[port].data_out_sigs[0],
                              0, 0);
        }
      else
        {
          esp_configgpio(priv->config->din_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->din_pin,
                             i2s_periph_signal[port].data_in_sig, 0);
        }
    }

  if (priv->config->role == I2S_ROLE_SLAVE)
    {
      /* For "tx + slave" mode, select TX signal index for ws and bck */

      if (priv->config->tx_en && !priv->config->rx_en)
        {
#if SOC_I2S_HW_VERSION_2
          I2S_CLOCK_SRC_ATOMIC()
            {
              i2s_ll_mclk_bind_to_tx_clk(priv->config->ctx->dev);
            }
#endif

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->ws_pin,
                             i2s_periph_signal[port].s_tx_ws_sig, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->bclk_pin,
                             i2s_periph_signal[port].s_tx_bck_sig, 0);
        }
      else
        {
          /* For "tx + rx + slave" or "rx + slave" mode, select RX signal
           * index for ws and bck.
           */

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->ws_pin,
                             i2s_periph_signal[port].s_rx_ws_sig, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->bclk_pin,
                             i2s_periph_signal[port].s_rx_bck_sig, 0);
        }
    }
  else
    {
      /* Considering master role for the I2S port */

      /* Set MCLK pin */

      if (priv->config->mclk_pin != I2S_GPIO_UNUSED)
        {
          i2sinfo("Configuring GPIO%" PRIu8 " to output master clock\n",
                  priv->config->mclk_pin);

          esp_gpiowrite(priv->config->mclk_pin, 1);
          esp_configgpio(priv->config->mclk_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->mclk_pin,
                              i2s_periph_signal[port].mck_out_sig, 0, 0);
        }

      if (priv->config->rx_en && !priv->config->tx_en)
        {
          /* For "rx + master" mode, select RX signal index for ws and bck */

#if SOC_I2S_HW_VERSION_2
          I2S_CLOCK_SRC_ATOMIC()
            {
              i2s_ll_mclk_bind_to_rx_clk(priv->config->ctx->dev);
            }
#endif

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->ws_pin,
                              i2s_periph_signal[port].m_rx_ws_sig, 0, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->bclk_pin,
                              i2s_periph_signal[port].m_rx_bck_sig, 0, 0);
        }
      else
        {
          /* For "tx + rx + master" or "tx + master" mode, select TX signal
           * index for ws and bck.
           */

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->ws_pin,
                              i2s_periph_signal[port].m_tx_ws_sig, 0, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->bclk_pin,
                              i2s_periph_signal[port].m_tx_bck_sig, 0, 0);
        }
    }

  /* Share BCLK and WS if in full-duplex mode */

  i2s_ll_share_bck_ws(priv->config->ctx->dev,
                      priv->config->tx_en && priv->config->rx_en);

  priv->data_width = priv->config->data_width;
  priv->channels = priv->config->total_slot;

  /* Configure the TX module */

  if (priv->config->tx_en)
    {
      if (priv->channels == 1)
        {
          tx_slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
        }
      else
        {
          tx_slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
        }

      tx_slot_cfg.data_bit_width = priv->config->data_width;
      tx_slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;

      if (priv->config->audio_std_mode <= I2S_TDM_PCM)
        {
          if (priv->config->audio_std_mode == I2S_TDM_PHILIPS)
            {
              I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(tx_slot_cfg.tdm,
                                                  priv->data_width,
                                                  I2S_TDM_AUTO_SLOT);
            }
          else if (priv->config->audio_std_mode == I2S_TDM_MSB)
            {
              I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(tx_slot_cfg.tdm,
                                              priv->data_width,
                                              I2S_TDM_AUTO_SLOT);
            }
          else
            {
              I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(tx_slot_cfg.tdm,
                                                    priv->data_width,
                                                    I2S_TDM_AUTO_SLOT);
            }

          i2s_hal_tdm_set_tx_slot(priv->config->ctx,
                                  priv->config->role == I2S_ROLE_SLAVE,
                                  &tx_slot_cfg);

          i2s_ll_tx_enable_tdm(priv->config->ctx->dev);
        }
      else
        {
          i2s_ll_tx_enable_pdm(priv->config->ctx->dev, true);
          I2S_PDM_TX_SLOT_DEFAULT_CONFIG(tx_slot_cfg.pdm_tx);
          i2s_hal_pdm_set_tx_slot(priv->config->ctx,
                                  priv->config->role == I2S_ROLE_SLAVE,
                                  &tx_slot_cfg);
        }

      /* The default value for the master clock frequency (MCLK frequency)
       * can be set from the sample rate multiplied by a fixed value, known
       * as MCLK multiplier. This multiplier, however, should be divisible
       * by the number of bytes from a sample, i.e, for 24 bits, the
       * multiplier should be divisible by 3. NOTE: the MCLK frequency can
       * be adjusted on runtime, so this value remains valid only if the
       * upper half does not implement the `i2s_setmclkfrequency` method.
       */

      if (priv->config->data_width == I2S_DATA_BIT_WIDTH_24BIT)
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_384;
        }
      else
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_256;
        }

      ret = i2s_setmclkfrequency((struct i2s_dev_s *)priv,
                                 (priv->config->rate * priv->mclk_multiple));
      if (ret <= 0)
        {
          i2serr("Failed to set MCLK frequency: %d\n", ret);
          return ret;
        }

      priv->rate = priv->config->rate;
      ret = i2s_set_clock(priv);
      if (ret != OK)
        {
          i2serr("Failed to set clock: %d\n", ret);
          return ret;
        }
    }

  /* Configure the RX module */

  if (priv->config->rx_en)
    {
      if (priv->channels == 1)
        {
          rx_slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
        }
      else
        {
          rx_slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
        }

      rx_slot_cfg.data_bit_width = priv->config->data_width;
      rx_slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;

      if (priv->config->audio_std_mode <= I2S_TDM_PCM)
        {
          /* If the role is slave or master and tx is enabled, then the role
           * is slave
           */

          bool is_slave = priv->config->role == I2S_ROLE_SLAVE || \
                          (priv->config->role == I2S_ROLE_MASTER && \
                           priv->config->tx_en);

          if (priv->config->audio_std_mode == I2S_TDM_PHILIPS)
            {
              I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(rx_slot_cfg.tdm,
                                                  priv->data_width,
                                                  I2S_TDM_AUTO_SLOT);
            }
          else if (priv->config->audio_std_mode == I2S_TDM_MSB)
            {
              I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(rx_slot_cfg.tdm,
                                              priv->data_width,
                                              I2S_TDM_AUTO_SLOT);
            }
          else
            {
              I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(tx_slot_cfg.tdm,
                                                    priv->data_width,
                                                    I2S_TDM_AUTO_SLOT);
            }

          i2s_hal_tdm_set_rx_slot(priv->config->ctx,
                                  is_slave,
                                  &rx_slot_cfg);

          i2s_ll_rx_enable_tdm(priv->config->ctx->dev);
        }
      else
        {
          i2serr("Due to the lack of `PDM to PCM` module, \
                  PDM RX is not available\n");
        }

      /* The default value for the master clock frequency (MCLK frequency)
       * can be set from the sample rate multiplied by a fixed value, known
       * as MCLK multiplier. This multiplier, however, should be divisible
       * by the number of bytes from a sample, i.e, for 24 bits, the
       * multiplier should be divisible by 3. NOTE: the MCLK frequency can
       * be adjusted on runtime, so this value remains valid only if the
       * upper half does not implement the `i2s_setmclkfrequency` method.
       */

      if (priv->config->data_width == I2S_DATA_BIT_WIDTH_24BIT)
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_384;
        }
      else
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_256;
        }

      ret = i2s_setmclkfrequency((struct i2s_dev_s *)priv,
                                 (priv->config->rate * priv->mclk_multiple));
      if (ret <= 0)
        {
          i2serr("Failed to set MCLK frequency: %d\n", ret);
          return ret;
        }

      priv->rate = priv->config->rate;
      ret = i2s_set_clock(priv);
      if (ret != OK)
        {
          i2serr("Failed to set clock: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_set_get_apll_freq
 *
 * Description:
 *   Calculates and sets the Audio Phase-Locked Loop (APLL) frequency
 *   required for a given master clock (MCLK) frequency. This function
 *   determines the appropriate divider to generate the expected APLL
 *   frequency, sets the hardware APLL to this frequency, and returns
 *   the actual frequency set.
 *   It also handles error conditions such as exceeding the maximum APLL
 *   frequency or invalid arguments, and logs relevant information and
 *   warnings.
 *
 * Input Parameters:
 *   mclk_freq_hz - The desired master clock frequency in Hz.
 *
 * Returned Value:
 *   The actual APLL frequency set in Hz, or 0 on failure.
 *
 ****************************************************************************/

#if SOC_I2S_SUPPORTS_APLL
static uint32_t i2s_set_get_apll_freq(uint32_t mclk_freq_hz)
{
  int mclk_div = (int)((CLK_LL_APLL_MIN_HZ / mclk_freq_hz) + 1);
  esp_err_t ret = ESP_OK;
  uint32_t expt_freq;
  uint32_t real_freq;

  /* Calculate the expected APLL  */

  /* apll_freq = mclk * div
   * when div = 1, hardware will still divide 2
   * when div = 0, the final mclk will be unpredictable
   * So the div here should be at least 2
   */

  mclk_div = mclk_div < 2 ? 2 : mclk_div;
  expt_freq = mclk_freq_hz * mclk_div;
  if (expt_freq > CLK_LL_APLL_MAX_HZ)
    {
      i2serr("The required APLL frequency exceed its maximum value");
      goto errout;
    }

  real_freq = 0;
  ret = periph_rtc_apll_freq_set(expt_freq, &real_freq);

  if (ret == ESP_ERR_INVALID_ARG)
    {
      i2serr("set APLL freq failed due to invalid argument");
      goto errout;
    }

  if (ret == ESP_ERR_INVALID_STATE)
    {
      i2swarn("APLL is occupied already, it is working at %"PRIu32" Hz while"
              " the expected frequency is %"PRIu32" Hz",
              real_freq, expt_freq);
      i2swarn("Trying to work at %"PRIu32" Hz...", real_freq);
    }

  i2sinfo("APLL expected frequency is %"PRIu32" Hz, real frequency is "
          "%"PRIu32" Hz", expt_freq, real_freq);
  return real_freq;

errout:
  UNUSED(real_freq);
  UNUSED(ret);
  return 0;
}
#endif

/****************************************************************************
 * Name: i2s_get_source_clk_freq
 *
 * Description:
 *   Retrieve the frequency of the specified I2S clock source.
 *   If the clock source is APLL and supported, it returns the frequency
 *   calculated for the given master clock frequency. Otherwise, it queries
 *   the clock tree for the frequency of the specified source.
 *
 * Input Parameters:
 *   clk_src      - The I2S clock source to query.
 *   mclk_freq_hz - The desired master clock frequency in Hz (used for APLL).
 *
 * Returned Value:
 *   The frequency of the specified clock source in Hz.
 *
 ****************************************************************************/

static uint32_t i2s_get_source_clk_freq(i2s_clock_src_t clk_src,
                                        uint32_t mclk_freq_hz)
{
  uint32_t clk_freq = 0;

#if SOC_I2S_SUPPORTS_APLL
  if (clk_src == I2S_CLK_SRC_APLL)
    {
      return i2s_set_get_apll_freq(mclk_freq_hz);
    }
#endif

  esp_clk_tree_src_get_freq_hz(clk_src,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                               &clk_freq);
  return clk_freq;
}

/****************************************************************************
 * Name: i2s_check_mclkfrequency
 *
 * Description:
 *   Check if MCLK frequency is compatible with the current data width and
 *   bits/sample set. Master clock should be multiple of the sample rate and
 *   bclk at the same time.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   Returns the current master clock or a negated errno value on failure.
 *
 ****************************************************************************/

static int32_t i2s_check_mclkfrequency(struct esp_i2s_s *priv)
{
  uint32_t mclk_freq;
  uint32_t mclk_multiple = priv->mclk_multiple;
  uint32_t bclk = priv->rate * priv->config->total_slot * priv->data_width;
  int i;

  /* If the master clock is divisible by both the sample rate and the bit
   * clock, everything is as expected and we can return the current master
   * clock frequency.
   */

  if (priv->mclk_freq % priv->rate == 0 && priv->mclk_freq % bclk == 0)
    {
      priv->mclk_multiple = priv->mclk_freq / priv->rate;
      return priv->mclk_freq;
    }

  /* Select the lowest multiplier for setting the master clock */

  for (mclk_multiple = I2S_MCLK_MULTIPLE_128;
       mclk_multiple <= I2S_MCLK_MULTIPLE_512;
       mclk_multiple += I2S_MCLK_MULTIPLE_128)
    {
      mclk_freq = priv->rate * mclk_multiple;
      if (mclk_freq % priv->rate == 0 && mclk_freq % bclk == 0)
        {
          priv->mclk_multiple = mclk_multiple;
          mclk_freq = i2s_setmclkfrequency((struct i2s_dev_s *)priv,
                                           mclk_freq);
          if (mclk_freq <= 0)
            {
              i2serr("Failed to set MCLK frequency: %"PRIu32"\n", mclk_freq);
              return -EINVAL;
            }

          return mclk_freq;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: i2s_set_datawidth
 *
 * Description:
 *   Set the I2S TX/RX data width.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

static uint32_t i2s_set_datawidth(struct esp_i2s_s *priv)
{
  int width;
  if (priv->config->tx_en)
    {
      i2s_ll_tx_set_sample_bit(priv->config->ctx->dev,
                               priv->data_width, priv->data_width);
      i2s_ll_tx_set_half_sample_bit(priv->config->ctx->dev,
                                    priv->data_width);

      if (priv->config->audio_std_mode != I2S_TDM_PCM)
        {
          width = priv->data_width;
        }
      else
        {
          width = 1;
        }

      i2s_ll_tx_set_ws_width(priv->config->ctx->dev, width);
    }

  if (priv->config->rx_en)
    {
      i2s_ll_rx_set_sample_bit(priv->config->ctx->dev,
                               priv->data_width, priv->data_width);
      i2s_ll_rx_set_half_sample_bit(priv->config->ctx->dev,
                                    priv->data_width);

      if (priv->config->audio_std_mode != I2S_TDM_PCM)
        {
          width = priv->data_width;
        }
      else
        {
          width = 1;
        }

      i2s_ll_rx_set_ws_width(priv->config->ctx->dev, width);
    }

  return priv->data_width;
}

/****************************************************************************
 * Name: i2s_set_clock
 *
 * Description:
 *   Set the I2S TX sample rate by adjusting I2S clock.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   Returns OK on success, a negative error code on failure
 *
 ****************************************************************************/

static int i2s_set_clock(struct esp_i2s_s *priv)
{
  uint32_t bclk;
  uint32_t mclk;
  uint32_t sclk;
  uint32_t mclk_div;
  uint16_t bclk_div;

  sclk = i2s_get_source_clk_freq(priv->config->tx_clk_src, priv->mclk_freq);

  if (sclk <= 0)
    {
      i2serr("Invalid source clock frequency: %"PRIu32"\n", sclk);
      return -EINVAL;
    }

  /* fmclk = bck_div * fbclk = fsclk / (mclk_div + b / a)
   * mclk_div is the I2S clock divider's integral value
   * b is the fraction clock divider's numerator value
   * a is the fraction clock divider's denominator value
   */

  if (priv->config->role == I2S_ROLE_MASTER)
    {
      bclk = priv->rate * priv->config->total_slot * priv->data_width;
      mclk = priv->mclk_freq;
      bclk_div = mclk / bclk;
    }
  else
    {
      /* For slave mode, mclk >= bclk * 8, so fix bclk_div to 2 first */

      bclk_div = 8;
      bclk = priv->rate * priv->config->total_slot * priv->data_width;
      mclk = bclk * bclk_div;
    }

  /* Calculate the nearest integer value of the I2S clock divider */

  mclk_div = sclk / mclk;

  i2sinfo("Clock division info: [sclk]%" PRIu32 " Hz [mdiv] %ld "
          "[mclk] %" PRIu32 " Hz [bdiv] %d [bclk] %" PRIu32 " Hz\n",
          sclk, mclk_div, mclk, bclk_div, bclk);

  priv->config->clk_info->bclk = bclk;
  priv->config->clk_info->bclk_div = bclk_div;
  priv->config->clk_info->mclk = mclk;
  priv->config->clk_info->mclk_div = mclk_div;
  priv->config->clk_info->sclk = sclk;

  I2S_CLOCK_SRC_ATOMIC()
    {
      i2s_hal_set_tx_clock(priv->config->ctx,
                           priv->config->clk_info,
                           priv->config->tx_clk_src,
                           NULL);

      i2s_hal_set_rx_clock(priv->config->ctx,
                           priv->config->clk_info,
                           priv->config->rx_clk_src,
                           NULL);
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_tx_channel_start
 *
 * Description:
 *   Start TX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void i2s_tx_channel_start(struct esp_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      /* Reset the TX channel */

      i2s_hal_tx_reset(priv->config->ctx);

      /* Reset TX FIFO */

      i2s_hal_tx_reset_fifo(priv->config->ctx);

      /* Set I2S_RX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      i2s_hal_tx_start(priv->config->ctx);

      priv->tx_started = true;

      i2sinfo("Started TX channel of port %ld\n", priv->config->port);
    }
}

/****************************************************************************
 * Name: i2s_rx_channel_start
 *
 * Description:
 *   Start RX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void i2s_rx_channel_start(struct esp_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      /* Reset the RX channel */

      i2s_hal_rx_reset(priv->config->ctx);

      /* Reset RX FIFO */

      i2s_hal_rx_reset_fifo(priv->config->ctx);

      priv->rx_started = true;

      i2sinfo("Started RX channel of port %ld\n", priv->config->port);
    }
}

/****************************************************************************
 * Name: i2s_tx_channel_stop
 *
 * Description:
 *   Stop TX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_tx_channel_stop(struct esp_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      esp_err_t err;

      if (!priv->tx_started)
        {
          i2swarn("TX channel of port %ld was previously stopped\n",
                  priv->config->port);
          return OK;
        }

      /* Stop TX channel */

      i2s_hal_tx_stop(priv->config->ctx);

      /* Stop outlink */

      err = gdma_stop(priv->dma_channel_tx);
      if (err != ESP_OK)
        {
          i2serr("Failed to stop DMA channel: %d\n", err);
          return -EINVAL;
        }

      priv->tx_started = false;

      i2sinfo("Stopped TX channel of port %ld\n", priv->config->port);
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_rx_channel_stop
 *
 * Description:
 *   Stop RX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_rx_channel_stop(struct esp_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      esp_err_t err;

      if (!priv->rx_started)
        {
          i2swarn("RX channel of port %ld was previously stopped\n",
                  priv->config->port);
          return OK;
        }

      /* Stop RX channel */

      i2s_hal_rx_stop(priv->config->ctx);

      err = gdma_stop(priv->dma_channel_rx);
      if (err != ESP_OK)
        {
          i2serr("Failed to stop DMA channel: %d\n", err);
          return -EINVAL;
        }

      priv->rx_started = false;

      i2sinfo("Stopped RX channel of port %ld\n", priv->config->port);
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_tx_interrupt
 *
 * Description:
 *   I2S TX DMA interrupt handler. This function is called when a DMA
 *   transmit event occurs. It checks if the current DMA descriptor is
 *   the last in the chain and, if so, schedules the next transfer.
 *
 * Input Parameters:
 *   dma_chan    - GDMA channel handle
 *   event_data  - Pointer to GDMA event data structure
 *   arg         - I2S controller private data
 *
 * Returned Value:
 *   Returns false. (No context switch required.)
 *
 ****************************************************************************/

static bool IRAM_ATTR i2s_tx_interrupt(gdma_channel_handle_t dma_chan,
                                       gdma_event_data_t *event_data,
                                       void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;
  lldesc_t *cur = NULL;

  if (event_data->flags.normal_eof)
    {
      cur = (lldesc_t *)(event_data->tx_eof_desc_addr);

      /* If the current descriptor is the last one, schedule the transfer */

      if (STAILQ_NEXT(cur, qe) == NULL)
        {
          i2s_tx_schedule(priv, cur);
        }
    }

  return false;
}

/****************************************************************************
 * Name: i2s_tx_error
 *
 * Description:
 *   I2S TX DMA error interrupt handler. This function is called when a
 *   transmit DMA error occurs. Currently, it triggers a system panic.
 *
 * Input Parameters:
 *   dma_chan    - GDMA channel handle
 *   event_data  - Pointer to GDMA event data structure
 *   arg         - I2S controller private data
 *
 * Returned Value:
 *   Returns false. (No context switch required.)
 *
 ****************************************************************************/

static bool IRAM_ATTR i2s_tx_error(gdma_channel_handle_t dma_chan,
                                   gdma_event_data_t *event_data,
                                   void *arg)
{
  /* Just panic for now */

  PANIC();
  return false;
}

/****************************************************************************
 * Name: i2s_rx_error
 *
 * Description:
 *   I2S RX DMA error interrupt handler. This function is called when a
 *   receive DMA error occurs. Currently, it triggers a system panic.
 *
 * Input Parameters:
 *   dma_chan    - GDMA channel handle
 *   event_data  - Pointer to GDMA event data structure
 *   arg         - I2S controller private data
 *
 * Returned Value:
 *   Returns false. (No context switch required.)
 *
 ****************************************************************************/

static bool IRAM_ATTR i2s_rx_error(gdma_channel_handle_t dma_chan,
                                   gdma_event_data_t *event_data,
                                   void *arg)
{
  /* Just panic for now */

  PANIC();
  return false;
}

/****************************************************************************
 * Name: i2s_rx_interrupt
 *
 * Description:
 *   I2S RX DMA interrupt handler. This function is called when a DMA
 *   transfer completes or an RX event occurs. It processes the DMA
 *   descriptor, synchronizes the buffer if needed, and schedules the
 *   next RX operation or updates the EOF number for the next descriptor.
 *
 * Input Parameters:
 *   dma_chan    - GDMA channel handle
 *   event_data  - Pointer to GDMA event data structure
 *   arg         - I2S controller private data
 *
 * Returned Value:
 *   Returns false. (No context switch required.)
 *
 ****************************************************************************/

static bool IRAM_ATTR i2s_rx_interrupt(gdma_channel_handle_t dma_chan,
                                       gdma_event_data_t *event_data,
                                       void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;
  lldesc_t *cur = NULL;
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  int alignment = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM,
                                                CACHE_TYPE_DATA);
#endif
  if (event_data->flags.normal_eof)
    {
      cur = (lldesc_t *)(event_data->rx_eof_desc_addr);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
      esp_cache_msync(cur,
                      ALIGN_UP(sizeof(lldesc_t), alignment),
                      ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      esp_cache_msync((void *)cur->buf, cur->length,
                      ESP_CACHE_MSYNC_FLAG_DIR_M2C);
#endif

      /* If the current descriptor is the last one, schedule the transfer */

      if (STAILQ_NEXT(cur, qe) == NULL)
        {
          if (cur->eof == 1 && cur->owner == 0)
            {
              i2s_rx_schedule(priv, cur);
            }
        }
      else
        {
          i2s_ll_rx_set_eof_num(priv->config->ctx->dev,
                                STAILQ_NEXT(cur, qe)->size);
        }
    }

  return false;
}

/****************************************************************************
 * Name: i2s_getmclkfrequency
 *
 * Description:
 *   Get the current master clock frequency.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Returns the current master clock.
 *
 ****************************************************************************/

static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  return priv->mclk_freq;
}

/****************************************************************************
 * Name: i2s_setmclkfrequency
 *
 * Description:
 *   Set the master clock frequency. Usually, the MCLK is a multiple of the
 *   sample rate. Most of the audio codecs require setting specific MCLK
 *   frequency according to the sample rate.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   frequency  - The I2S master clock's frequency
 *
 * Returned Value:
 *   Returns the resulting master clock or a negated errno value on failure.
 *
 ****************************************************************************/

static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;
  uint32_t source_clk_freq = 0;

  source_clk_freq = i2s_get_source_clk_freq(priv->config->tx_clk_src,
                                            frequency);

  /* Check if the master clock frequency is beyond the highest possible
   * value and return an error.
   */

  if (frequency >= (source_clk_freq / 2))
    {
      return -EINVAL;
    }

  priv->mclk_freq = frequency;

  return frequency;
}

/****************************************************************************
 * Name: i2s_txchannels
 *
 * Description:
 *   Set the I2S TX number of channels.
 *   If channels is 0, the current number of channels is returned.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   Returns the number of TX channels
 *
 ****************************************************************************/

static int i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;
  uint32_t channels_mask;
  bool is_mono = true;

  if (priv->config->tx_en)
    {
      bool is_started = priv->tx_started;

      if (channels == 0)
        {
          return priv->channels;
        }

      if (channels != 1 && channels != 2)
        {
          return 0;
        }

      if (is_started)
        {
          i2s_tx_channel_stop(priv);
        }

      priv->channels = channels;

      /* Always consider two channels. For mono (1-channel), we set the
       * I2S_TX_TDM_CHAN1_EN to 0 and I2S_TX_CHAN_EQUAL to 1 to send out
       * the data of the previous channel.
       */

      /* I2S_TX_TDM_TOT_CHAN_NUM = channels - 1 */

      i2s_ll_tx_set_chan_num(priv->config->ctx->dev, 2);

      channels_mask = I2S_TX_TDM_CHAN0_EN;
      if (priv->channels > 1)
        {
          channels_mask |= I2S_TX_TDM_CHAN0_EN | I2S_TX_TDM_CHAN1_EN;
          is_mono = false;
        }

      i2s_ll_tx_enable_mono_mode(priv->config->ctx->dev,
                                 is_mono);

      i2s_ll_tx_set_active_chan_mask(priv->config->ctx->dev, channels_mask);

      /* Set I2S_TX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      if (is_started)
        {
          i2s_tx_channel_start(priv);
        }

      return priv->channels;
    }

  return 0;
}

/****************************************************************************
 * Name: i2s_rxchannels
 *
 * Description:
 *   Set the I2S RX number of channels.
 *   If channels is 0, the current number of channels is returned.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   Returns the number of RX channels
 *
 ****************************************************************************/

static int i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      if (channels == 0)
        {
          return priv->channels;
        }

      if (channels != 1 && channels != 2)
        {
          return 0;
        }

      priv->channels = channels;
      return priv->channels;
    }

  return 0;
}

/****************************************************************************
 * Name: i2s_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2S transmitter or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *   If rate is 0, the current sample rate is returned.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   OK on success, ERROR on fail
 *
 ****************************************************************************/

static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      bool is_started = priv->tx_started;

      if (rate == 0)
        {
          return priv->rate;
        }

      if (is_started)
        {
          i2s_tx_channel_stop(priv);
        }

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < 0)
        {
          return 0;
        }

      if (i2s_set_clock(priv) != OK)
        {
          i2serr("Failed to set clock\n");
          return ERROR;
        }

      if (is_started)
        {
          i2s_tx_channel_start(priv);
        }

      return priv->rate;
    }

  return 0;
}

/****************************************************************************
 * Name: i2s_rxsamplerate
 *
 * Description:
 *   Set the I2S RX sample rate.
 *   If rate is 0, the current sample rate is returned.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      bool is_started = priv->rx_started;

      if (rate == 0)
        {
          return priv->rate;
        }

      if (is_started)
        {
          i2s_rx_channel_stop(priv);
        }

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < 0)
        {
          return 0;
        }

      if (i2s_set_clock(priv) != OK)
        {
          i2serr("Failed to set clock\n");
          return ERROR;
        }

      if (is_started)
        {
          i2s_rx_channel_start(priv);
        }

      return priv->rate;
    }

  return 0;
}

/****************************************************************************
 * Name: i2s_txdatawidth
 *
 * Description:
 *   Set the I2S TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *   If width is 0, the current data width is returned.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      bool is_started = priv->tx_started;

      if (bits == 0)
        {
          return priv->data_width;
        }

      if (is_started)
        {
          i2s_tx_channel_stop(priv);
        }

      priv->data_width = bits;

      i2s_set_datawidth(priv);

      if (is_started)
        {
          i2s_tx_channel_start(priv);
        }

      return bits;
    }

  return 0;
}

/****************************************************************************
 * Name: i2s_rxdatawidth
 *
 * Description:
 *   Set the I2S RX data width.  The RX bitrate is determined by
 *   sample_rate * data_width.
 *   If width is 0, the current data width is returned.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      bool is_started = priv->rx_started;

      if (bits == 0)
        {
          return priv->data_width;
        }

      if (is_started)
        {
          i2s_rx_channel_stop(priv);
        }

      priv->data_width = bits;

      i2s_set_datawidth(priv);

      if (is_started)
        {
          i2s_rx_channel_start(priv);
        }

      return bits;
    }

  return 0;
}

/****************************************************************************
 * Name: i2s_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      struct esp_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check audio buffer data size from the upper half. If the buffer
       * size is not a multiple of the data width, the remaining bytes
       * must be sent along with the next audio buffer.
       */

      nbytes = (apb->nbytes - apb->curbyte) + priv->tx.carry.bytes;

      nbytes -= (nbytes % (priv->data_width / 8));

      if (nbytes > (I2S_DMA_BUFFER_MAX_SIZE * I2S_DMADESC_NUM))
        {
          i2serr("Required buffer size can't fit into DMA outlink "
                 "(exceeds in %" PRIu32 " bytes). Try to increase the "
                 "number of the DMA descriptors (CONFIG_I2S_DMADESC_NUM).",
                 nbytes - (I2S_DMA_BUFFER_MAX_SIZE * I2S_DMADESC_NUM));
          return -EFBIG;
        }

      /* Allocate a buffer container in advance */

      bfcontainer = i2s_buf_allocate(priv);
      if (bfcontainer == NULL)
        {
          i2serr("Failed to allocate the buffer container");
          return -ENOMEM;
        }

      /* Get exclusive access to the I2S driver data */

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          goto errout_with_buf;
        }

      /* Add a reference to the audio buffer */

      apb_reference(apb);

      /* Initialize the buffer container structure */

      bfcontainer->callback = callback;
      bfcontainer->timeout  = timeout;
      bfcontainer->arg      = arg;
      bfcontainer->apb      = apb;
      bfcontainer->nbytes   = nbytes;
      bfcontainer->result   = -EBUSY;

      ret = i2s_txdma_setup(priv, bfcontainer);

      if (ret != OK)
        {
          goto errout_with_buf;
        }

      i2sinfo("Queued %d bytes into DMA buffers\n", apb->nbytes);
      i2s_dump_buffer("Audio pipeline buffer:", &apb->samp[apb->curbyte],
                      apb->nbytes - apb->curbyte);

      nxmutex_unlock(&priv->lock);

      return OK;

errout_with_buf:
      nxmutex_unlock(&priv->lock);
      if (i2s_buf_free(priv, bfcontainer) != OK)
        {
          i2serr("Failed to free buffer container\n");
        }

      return ret;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Name: i2s_receive
 *
 * Description:
 *   Receive a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to receive data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      struct esp_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check max audio buffer data size from the upper half and align the
       * receiving buffer according to the data width.
       */

      nbytes = apb->nmaxbytes;

      nbytes -= (nbytes % (priv->data_width / 8));

      /* Allocate a buffer container in advance */

      bfcontainer = i2s_buf_allocate(priv);
      if (bfcontainer == NULL)
        {
          i2serr("Failed to allocate the buffer container");
          return -ENOMEM;
        }

      /* Get exclusive access to the I2S driver data */

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          goto errout_with_buf;
        }

      /* Add a reference to the audio buffer */

      apb_reference(apb);

      /* Initialize the buffer container structure */

      bfcontainer->callback = callback;
      bfcontainer->timeout  = timeout;
      bfcontainer->arg      = arg;
      bfcontainer->apb      = apb;
      bfcontainer->nbytes   = nbytes;
      bfcontainer->result   = -EBUSY;

      ret = i2s_rxdma_setup(priv, bfcontainer);

      if (ret != OK)
        {
          goto errout_with_buf;
        }

      i2sinfo("Prepared %d bytes to receive DMA buffers\n", apb->nmaxbytes);
      i2s_dump_buffer("Received Audio pipeline buffer:",
                      &apb->samp[apb->curbyte],
                      apb->nbytes - apb->curbyte);

      nxmutex_unlock(&priv->lock);

      return OK;

errout_with_buf:
      nxmutex_unlock(&priv->lock);
      if (i2s_buf_free(priv, bfcontainer) != OK)
        {
          i2serr("Failed to free buffer container\n");
        }

      return ret;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Name: i2s_ioctl
 *
 * Description:
 *   Implement the lower-half logic ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower-half I2S driver device
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;
  struct audio_buf_desc_s  *bufdesc;
  int ret = -ENOTTY;

  switch (cmd)
    {
      /* AUDIOIOC_START - Start the audio stream.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_START:
        {
          i2sinfo("AUDIOIOC_START\n");

          priv->streaming = true;

          ret = OK;
        }
        break;

      /* AUDIOIOC_STOP - Stop the audio stream.
       *
       *   ioctl argument:  Audio session
       */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      case AUDIOIOC_STOP:
        {
          i2sinfo("AUDIOIOC_STOP\n");

          priv->streaming = false;

          ret = OK;
        }
        break;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

      /* AUDIOIOC_ALLOCBUFFER - Allocate an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_ALLOCBUFFER:
        {
          i2sinfo("AUDIOIOC_ALLOCBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *) arg;
          ret = apb_alloc(bufdesc);
        }
        break;

      /* AUDIOIOC_FREEBUFFER - Free an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_FREEBUFFER:
        {
          i2sinfo("AUDIOIOC_FREEBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *) arg;
          DEBUGASSERT(bufdesc->u.buffer != NULL);
          apb_free(bufdesc->u.buffer);
          ret = sizeof(struct audio_buf_desc_s);
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: i2s_dma_setup
 *
 * Description:
 *   Configure the DMA for the I2S peripheral
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure. This function
 *          will complete the I2S specific portions of the initialization
 *          regarding the DMA operation.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_dma_setup(struct esp_i2s_s *priv)
{
  int ret = OK;
  esp_err_t err;
  gdma_trigger_t trig =
    {
      .periph = GDMA_TRIG_PERIPH_I2S
    };

  switch (priv->config->port)
    {
#if SOC_I2S_NUM > 2
      case I2S_NUM_2:
        trig.instance_id = SOC_GDMA_TRIG_PERIPH_I2S2;
        break;
#endif

#if SOC_I2S_NUM > 1
      case I2S_NUM_1:
        trig.instance_id = SOC_GDMA_TRIG_PERIPH_I2S1;
        break;
#endif

      case I2S_NUM_0:
        trig.instance_id = SOC_GDMA_TRIG_PERIPH_I2S0;
        break;

      default:
        i2serr("Unsupported I2S port number");
        return ESP_ERR_NOT_SUPPORTED;
    }

  /* Set up to receive GDMA interrupts on the current CPU. Each TX/RX channel
   * will be assigned to a different CPU interrupt.
   */

  priv->cpu = this_cpu();

  if (priv->config->tx_en)
    {
      gdma_channel_alloc_config_t tx_handle =
        {
          .direction = GDMA_CHANNEL_DIRECTION_TX,
          .flags.reserve_sibling = 1,
        };

      err = gdma_new_ahb_channel(&tx_handle, &priv->dma_channel_tx);
      if (err != ESP_OK)
        {
          i2serr("Failed to register tx dma channel: %d\n", err);
          return -EINVAL;
        }

      err = gdma_connect(priv->dma_channel_tx, trig);
      if (err != ESP_OK)
        {
          i2serr("Failed to connect tx dma channel: %d\n", err);
          ret = -EINVAL;
          goto err1;
        }

      gdma_tx_event_callbacks_t cb_tx =
        {
          .on_trans_eof = i2s_tx_interrupt,
          .on_descr_err = i2s_tx_error,
        };

      /* Set callback function for GDMA, the interrupt is triggered by GDMA,
       * then the GDMA ISR will call the callback function.
       */

      err = gdma_register_tx_event_callbacks(priv->dma_channel_tx,
                                             &cb_tx, priv);
      if (err != ESP_OK)
        {
          i2serr("Failed to register tx callback: %d\n", err);
          ret = -EINVAL;
          goto err2;
        }
    }

  if (priv->config->rx_en)
    {
      gdma_channel_alloc_config_t rx_handle =
        {
          .direction = GDMA_CHANNEL_DIRECTION_RX,
          .flags.reserve_sibling = 1,
        };

      err = gdma_new_ahb_channel(&rx_handle, &priv->dma_channel_rx);
      if (err != ESP_OK)
        {
          i2serr("Failed to register rx dma channel: %d\n", err);
          return -EINVAL;
        }

      err = gdma_connect(priv->dma_channel_rx, trig);
      if (err != ESP_OK)
        {
          i2serr("Failed to connect rx dma channel: %d\n", err);
          ret = -EINVAL;
          goto err1;
        }

      gdma_rx_event_callbacks_t cb_rx =
        {
          .on_recv_eof = i2s_rx_interrupt,
          .on_descr_err = i2s_rx_error,
        };

      /* Set callback function for GDMA, the interrupt is triggered by GDMA,
       * then the GDMA ISR will call the callback function.
       */

      err = gdma_register_rx_event_callbacks(priv->dma_channel_rx,
                                             &cb_rx, priv);
      if (err != ESP_OK)
        {
          i2serr("Failed to register rx callback: %d\n", err);
          ret = -EINVAL;
          goto err2;
        }
    }

  return OK;

err2:
  if (priv->config->tx_en)
    {
      gdma_disconnect(priv->dma_channel_tx);
    }

  if (priv->config->rx_en)
    {
      gdma_disconnect(priv->dma_channel_rx);
    }

err1:
  if (priv->config->tx_en)
    {
      gdma_del_channel(priv->dma_channel_tx);
    }

  if (priv->config->rx_en)
    {
      gdma_del_channel(priv->dma_channel_rx);
    }

  return ret;
}

/****************************************************************************
 * Name: esp_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected I2S port
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple I2S interfaces)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *esp_i2sbus_initialize(int port)
{
  int ret;
  struct esp_i2s_s *priv = NULL;
  irqstate_t flags;

  i2sinfo("port: %d\n", port);

  /* Statically allocated I2S' device structure */

  switch (port)
    {
#ifdef CONFIG_ESPRESSIF_I2S0
      case ESPRESSIF_I2S0:
        priv = &esp_i2s0_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Allocate buffer containers */

  ret = i2s_buf_initialize(priv);
  if (ret < 0)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&priv->slock);

  ret = i2s_configure(priv);
  if (ret < 0)
    {
      goto err;
    }

  ret = i2s_dma_setup(priv);
  if (ret < 0)
    {
      goto err;
    }

  /* Start TX channel */

  if (priv->config->tx_en)
    {
      priv->tx_started = false;
    }

  /* Start RX channel */

  if (priv->config->rx_en)
    {
      priv->rx_started = false;
    }

  spin_unlock_irqrestore(&priv->slock, flags);

  /* Success exit */

  i2sinfo("I2S%ld was successfully initialized\n", priv->config->port);

  return &priv->dev;

  /* Failure exit */

err:
  spin_unlock_irqrestore(&priv->slock, flags);
  return NULL;
}
