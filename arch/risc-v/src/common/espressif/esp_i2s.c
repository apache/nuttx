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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "riscv_internal.h"

#include "esp_gpio.h"
#include "esp_irq.h"
#include "esp_dma.h"
#include "esp_i2s.h"

#include "hal/i2s_hal.h"
#include "hal/i2s_ll.h"
#include "soc/i2s_periph.h"
#include "soc/i2s_reg.h"
#include "hal/i2s_types.h"
#include "soc/gpio_sig_map.h"
#include "soc/gdma_reg.h"
#include "periph_ctrl.h"

#include "esp_attr.h"
#include "esp_bit_defs.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "riscv/interrupt.h"
#include "soc/soc.h"
#include "hal/dma_types.h"
#include "soc/gdma_periph.h"
#include "hal/gdma_ll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_ESP32C6
#  define ETS_I2S0_INTR_SOURCE ETS_I2S1_INTR_SOURCE
#  define CLK_SRC              I2S_CLK_SRC_PLL_160M
#endif

#ifdef CONFIG_ESPRESSIF_ESP32C3
#  define GDMA_OUT_TOTAL_EOF_CH0_INT_ENA  DMA_OUT_TOTAL_EOF_CH0_INT_ENA
#  define GDMA_OUT_DSCR_ERR_CH0_INT_ENA   DMA_OUT_DSCR_ERR_CH0_INT_ENA
#  define GDMA_IN_SUC_EOF_CH0_INT_ENA     DMA_IN_SUC_EOF_CH0_INT_ENA
#  define GDMA_OUT_EOF_CH0_INT_ENA        DMA_OUT_EOF_CH0_INT_ENA
#  define GDMA_OUT_TOTAL_EOF_CH0_INT_ST   DMA_OUT_TOTAL_EOF_CH0_INT_ST
#  define GDMA_IN_SUC_EOF_CH0_INT_ST      DMA_IN_SUC_EOF_CH0_INT_ST
#  define CLK_SRC                         I2S_CLK_SRC_PLL_160M
#endif

#ifdef CONFIG_ESPRESSIF_ESP32H2
#  define CLK_SRC I2S_CLK_SRC_PLL_64M
#endif

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
#  define I2S_HAVE_TX 1
#else
#  define I2S0_TX_ENABLED 0
#endif

#ifdef CONFIG_ESPRESSIF_I2S0_RX
#  define I2S0_RX_ENABLED 1
#  define I2S_HAVE_RX 1
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
    cfg.skip_mask = false,                                              \
    cfg.total_slot = I2S_TDM_AUTO_SLOT_NUM                              \

#define I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(cfg, bits_per_sample, mask) \
    cfg.slot_mask = (mask),                                         \
    cfg.ws_width = I2S_TDM_AUTO_WS_WIDTH,                           \
    cfg.ws_pol = false,                                             \
    cfg.bit_shift = false,                                          \
    cfg.left_align = false,                                         \
    cfg.big_endian = false,                                         \
    cfg.bit_order_lsb = false,                                      \
    cfg.skip_mask = false ,                                         \
    cfg.total_slot = I2S_TDM_AUTO_SLOT_NUM                          \

#define I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(cfg, bits_per_sample, mask) \
    cfg.slot_mask = (mask),                                               \
    cfg.ws_width = 1,                                                     \
    cfg.ws_pol = true,                                                    \
    cfg.bit_shift = true,                                                 \
    cfg.left_align = false,                                               \
    cfg.big_endian = false,                                               \
    cfg.bit_order_lsb = false,                                            \
    cfg.skip_mask = false,                                                \
    cfg.total_slot = I2S_TDM_AUTO_SLOT_NUM                                \

#define I2S_PDM_TX_SLOT_DEFAULT_CONFIG(cfg)                   \
    cfg.sd_prescale = 0,                                      \
    cfg.sd_scale = I2S_PDM_SIG_SCALING_MUL_1,                 \
    cfg.hp_scale = I2S_PDM_SIG_SCALING_DIV_2,                 \
    cfg.lp_scale = I2S_PDM_SIG_SCALING_MUL_1,                 \
    cfg.sinc_scale = I2S_PDM_SIG_SCALING_MUL_1,               \
    cfg.line_mode = I2S_PDM_TX_ONE_LINE_CODEC,                \
    cfg.hp_en = true,                                         \
    cfg.hp_cut_off_freq_hz = 35.5,                            \
    cfg.sd_dither = 0,                                        \
    cfg.sd_dither2 = 1                                        \

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

  uint32_t bclk_in_insig;           /* RX channel BCK signal (slave mode) index */
  uint32_t bclk_in_outsig;          /* RX channel BCK signal (master mode) index */
  uint32_t bclk_out_insig;          /* TX channel BCK signal (slave mode) index */
  uint32_t bclk_out_outsig;         /* TX channel BCK signal (master mode) index */
  uint32_t ws_in_insig;             /* RX channel WS signal (slave mode) index */
  uint32_t ws_in_outsig;            /* RX channel WS signal (master mode) index */
  uint32_t ws_out_insig;            /* TX channel WS signal (slave mode) index */
  uint32_t ws_out_outsig;           /* TX channel WS signal (master mode) index */
  uint32_t din_insig;               /* RX channel Data Input signal index */
  uint32_t dout_outsig;             /* TX channel Data Output signal index */
  uint32_t mclk_out_sig;            /* Master clock output index */

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

  struct esp_dmadesc_s dma_link[I2S_DMADESC_NUM];

  i2s_callback_t callback;      /* DMA completion callback */
  uint32_t timeout;             /* Timeout value of the DMA transfers */
  void *arg;                    /* Callback's argument */
  struct ap_buffer_s *apb;      /* The audio buffer */
  uint8_t *buf;                 /* The DMA's descriptor buffer */
  uint32_t nbytes;              /* The DMA's descriptor buffer size */
  int result;                   /* The result of the transfer */
};

/* Internal buffer must be aligned to the bytes_per_sample. Sometimes,
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

  uint32_t    mclk_freq;      /* I2S actual master clock */
  uint32_t    mclk_multiple;  /* The multiple of mclk to the sample rate */
  uint32_t    channels;       /* Audio channels (1:mono or 2:stereo) */
  uint32_t    rate;           /* I2S actual configured sample-rate */
  uint32_t    data_width;     /* I2S actual configured data_width */
  uint32_t    dma_channel;    /* I2S DMA channel being used */

#ifdef I2S_HAVE_TX
  struct esp_transport_s tx;  /* TX transport state */

  int  tx_irq;                    /* TX IRQ */
  bool tx_started;                /* TX channel started */
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  struct esp_transport_s rx;  /* RX transport state */

  int  rx_irq;                    /* RX IRQ */
  bool rx_started;                /* RX channel started */
#endif /* I2S_HAVE_RX */

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

/* Buffer container helpers */

static struct esp_buffer_s *
                i2s_buf_allocate(struct esp_i2s_s *priv);
static void     i2s_buf_free(struct esp_i2s_s *priv,
                             struct esp_buffer_s *bfcontainer);
static int      i2s_buf_initialize(struct esp_i2s_s *priv);

/* DMA support */

#ifdef I2S_HAVE_TX
static IRAM_ATTR int  i2s_txdma_setup(struct esp_i2s_s *priv,
                                      struct esp_buffer_s *bfcontainer);
static void           i2s_tx_worker(void *arg);
static void           i2s_tx_schedule(struct esp_i2s_s *priv,
                                      struct esp_dmadesc_s *outlink);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static IRAM_ATTR int  i2s_rxdma_setup(struct esp_i2s_s *priv,
                                      struct esp_buffer_s *bfcontainer);
static void           i2s_rx_worker(void *arg);
static void           i2s_rx_schedule(struct esp_i2s_s *priv,
                                      struct esp_dmadesc_s *outlink);
#endif /* I2S_HAVE_RX */

/* I2S methods (and close friends) */

static int32_t  i2s_check_mclkfrequency(struct esp_i2s_s *priv);
static uint32_t i2s_set_datawidth(struct esp_i2s_s *priv);
static void i2s_set_clock(struct esp_i2s_s *priv);
static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev);
static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency);
static int      i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg);

#ifdef I2S_HAVE_TX
static void     i2s_tx_channel_start(struct esp_i2s_s *priv);
static void     i2s_tx_channel_stop(struct esp_i2s_s *priv);
static int      i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                         i2s_callback_t callback, void *arg,
                         uint32_t timeout);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static void     i2s_rx_channel_start(struct esp_i2s_s *priv);
static void     i2s_rx_channel_stop(struct esp_i2s_s *priv);
static int      i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                            i2s_callback_t callback, void *arg,
                            uint32_t timeout);
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_i2sops =
{
#ifdef I2S_HAVE_TX
  .i2s_txchannels     = i2s_txchannels,
  .i2s_txsamplerate   = i2s_txsamplerate,
  .i2s_txdatawidth    = i2s_txdatawidth,
  .i2s_send           = i2s_send,
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  .i2s_rxchannels     = i2s_rxchannels,
  .i2s_rxsamplerate   = i2s_rxsamplerate,
  .i2s_rxdatawidth    = i2s_rxdatawidth,
  .i2s_receive        = i2s_receive,
#endif /* I2S_HAVE_RX */

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

static const struct esp_i2s_config_s esp_i2s0_config =
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
  .tx_clk_src       = CLK_SRC,
  .rx_clk_src       = CLK_SRC,
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
  .bclk_in_insig    = I2SO_BCK_IN_IDX,
  .bclk_in_outsig   = I2SO_BCK_OUT_IDX,
  .bclk_out_insig   = I2SO_BCK_IN_IDX,
  .bclk_out_outsig  = I2SO_BCK_OUT_IDX,
  .ws_in_insig      = I2SO_WS_IN_IDX,
  .ws_in_outsig     = I2SO_WS_OUT_IDX,
  .ws_out_insig     = I2SO_WS_IN_IDX,
  .ws_out_outsig    = I2SO_WS_OUT_IDX,
  .din_insig        = I2SI_SD_IN_IDX,
  .dout_outsig      = I2SO_SD_OUT_IDX,
  .mclk_out_sig     = I2S_MCLK_OUT_IDX,
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
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the I2S state structure
 *
 ****************************************************************************/

static void i2s_buf_free(struct esp_i2s_s *priv,
                         struct esp_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list (circbuf) */

  flags = spin_lock_irqsave(&priv->slock);

  bfcontainer->apb = NULL;
  bfcontainer->buf = NULL;
  bfcontainer->nbytes = 0;
  bfcontainer->flink  = priv->bf_freelist;
  priv->bf_freelist = bfcontainer;

  spin_unlock_irqrestore(&priv->slock, flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
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
#ifdef I2S_HAVE_TX
  priv->tx.carry.bytes = 0;
  priv->tx.carry.value = 0;
#endif /* I2S_HAVE_TX */

  priv->bf_freelist = NULL;
  for (int i = 0; i < CONFIG_ESPRESSIF_I2S_MAXINFLIGHT; i++)
    {
      i2s_buf_free(priv, &priv->containers[i]);
    }

  return OK;
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

#ifdef I2S_HAVE_TX
static int IRAM_ATTR i2s_txdma_start(struct esp_i2s_s *priv)
{
  struct esp_buffer_s *bfcontainer;

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
  i2s_hal_tx_reset_fifo(priv->config->ctx);

  /* Start transmission if no data is already being transmitted */

  bfcontainer = (struct esp_buffer_s *)sq_remfirst(&priv->tx.pend);

  esp_dma_load(bfcontainer->dma_link, priv->dma_channel, I2S_DIR_TX);
  esp_dma_enable(priv->dma_channel, I2S_DIR_TX);

  i2s_hal_tx_start(priv->config->ctx);

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.act);

  return OK;
}
#endif /* I2S_HAVE_TX */

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

#ifdef I2S_HAVE_RX
static int i2s_rxdma_start(struct esp_i2s_s *priv)
{
  struct esp_buffer_s *bfcontainer;
  size_t eof_nbytes;

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
  i2s_hal_rx_reset_fifo(priv->config->ctx);

  bfcontainer = (struct esp_buffer_s *)sq_remfirst(&priv->rx.pend);

  /* If there isn't already an active transmission in progress,
   * then start it.
   */

  eof_nbytes = MIN(bfcontainer->nbytes, ESPRESSIF_DMA_BUFLEN_MAX);

  i2s_ll_rx_set_eof_num(priv->config->ctx->dev, eof_nbytes);

  esp_dma_load(bfcontainer->dma_link, priv->dma_channel, I2S_DIR_RX);
  esp_dma_enable(priv->dma_channel, I2S_DIR_RX);

  i2s_hal_rx_start(priv->config->ctx);

  sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.act);

  return OK;
}
#endif /* I2S_HAVE_RX */

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

#ifdef I2S_HAVE_TX
static IRAM_ATTR int i2s_txdma_setup(struct esp_i2s_s *priv,
                                     struct esp_buffer_s *bfcontainer)
{
  int ret = OK;
  size_t carry_size;
  uint32_t bytes_queued;
  uint32_t data_copied;
  struct ap_buffer_s *apb;
  struct esp_dmadesc_s *outlink;
  apb_samp_t samp_size;
  irqstate_t flags;
  uint8_t *buf;
  uint8_t padding;
  uint8_t *samp;

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  apb = bfcontainer->apb;
  outlink = bfcontainer->dma_link;

  /* Get the transfer information, accounting for any data offset */

  const apb_samp_t bytes_per_sample = priv->data_width / 8;
  samp = &apb->samp[apb->curbyte];
  samp_size = (apb->nbytes - apb->curbyte) + priv->tx.carry.bytes;
  carry_size = samp_size % bytes_per_sample;

  /* Allocate the current audio buffer considering the remaining bytes
   * carried from the last upper half audio buffer.
   */

  bfcontainer->buf = calloc(bfcontainer->nbytes, 1);
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
      memcpy(buf, samp, (bytes_per_sample - priv->tx.carry.bytes));
      buf += (bytes_per_sample - priv->tx.carry.bytes);
      samp += (bytes_per_sample - priv->tx.carry.bytes);
      data_copied += (bytes_per_sample - priv->tx.carry.bytes);
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

  /* Release our reference on the audio buffer. This may very likely
   * cause the audio buffer to be freed.
   */

  apb_free(bfcontainer->apb);

  /* Configure DMA stream */

  bytes_queued = esp_dma_setup(priv->dma_channel,
                               true,
                               (struct esp_dmadesc_s *)outlink,
                               I2S_DMADESC_NUM,
                               (uint8_t *) bfcontainer->buf,
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
#endif /* I2S_HAVE_TX */

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

#ifdef I2S_HAVE_RX
static int i2s_rxdma_setup(struct esp_i2s_s *priv,
                           struct esp_buffer_s *bfcontainer)
{
  int ret = OK;
  struct esp_dmadesc_s *inlink;
  uint32_t bytes_queued;
  irqstate_t flags;

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  inlink = bfcontainer->dma_link;

  /* Configure DMA stream */

  bytes_queued = esp_dma_setup(priv->dma_channel,
                               false,
                               inlink,
                               I2S_DMADESC_NUM,
                               bfcontainer->apb->samp,
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
#endif /* I2S_HAVE_RX */

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

#ifdef I2S_HAVE_TX
static void IRAM_ATTR i2s_tx_schedule(struct esp_i2s_s *priv,
                                      struct esp_dmadesc_s *outlink)
{
  struct esp_buffer_s *bfcontainer;
  struct esp_dmadesc_s *bfdesc;
  dma_descriptor_t *bfdesc_ctrl;
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
       * REVISIT: what to do if we miss syncronization and the descriptor
       * that generated the interrupt is different from the expected (the
       * oldest of the list containing active transmissions)?
       */

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link;
      bfdesc_ctrl = (dma_descriptor_t *)bfdesc;
      while (!(bfdesc_ctrl->dw0.suc_eof))
        {
          DEBUGASSERT(bfdesc->next);
          bfdesc = bfdesc->next;
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
#endif /* I2S_HAVE_TX */

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

#ifdef I2S_HAVE_RX
static void i2s_rx_schedule(struct esp_i2s_s *priv,
                            struct esp_dmadesc_s *inlink)
{
  struct esp_buffer_s *bfcontainer;
  struct esp_dmadesc_s *bfdesc;
  dma_descriptor_t *bfdesc_ctrl;
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

      bfdesc = bfcontainer->dma_link;
      bfdesc_ctrl = (dma_descriptor_t *)bfdesc;

      while (bfdesc->next != NULL &&
             (bfdesc_ctrl->dw0.suc_eof))
        {
          bfdesc = bfdesc->next;
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
#endif /* I2S_HAVE_RX */

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

#ifdef I2S_HAVE_TX
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

      free(bfcontainer->buf);

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}
#endif /* I2S_HAVE_TX */

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

#ifdef I2S_HAVE_RX
static void i2s_rx_worker(void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;
  struct esp_buffer_s *bfcontainer;
  struct esp_dmadesc_s *dmadesc;
  dma_descriptor_t *dmadesc_ctrl;
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

      dmadesc = bfcontainer->dma_link;
      dmadesc_ctrl = (dma_descriptor_t *)dmadesc;

      bfcontainer->apb->nbytes = 0;

      while (dmadesc != NULL && (dmadesc_ctrl->dw0.suc_eof))
        {
          bfcontainer->apb->nbytes += dmadesc_ctrl->dw0.length;
          dmadesc = dmadesc->next;
        }

      /* Perform the RX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);

      if (priv->streaming == false)
        {
          bfcontainer->apb->flags |= AUDIO_APB_FINAL;
        }

      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer. This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}
#endif /* I2S_HAVE_RX */

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
 *   None
 *
 ****************************************************************************/

static void i2s_configure(struct esp_i2s_s *priv)
{
  uint32_t tx_conf  = 0;
  uint32_t rx_conf  = 0;
  bool loopback = false;
  i2s_hal_slot_config_t tx_slot_cfg =
    {
      0
    };

  i2s_hal_slot_config_t rx_slot_cfg =
    {
      0
    };

  /* Set peripheral clock and clear reset */

  periph_module_enable(i2s_periph_signal[priv->config->port].module);

  i2s_hal_init(priv->config->ctx, priv->config->port);
  i2s_ll_enable_clock(priv->config->ctx->dev);

  /* Configure multiplexed pins as connected on the board */

  /* Enable TX channel */

  if (priv->config->dout_pin != I2S_GPIO_UNUSED)
    {
      /* If TX channel is used, enable the clock source */

      esp_gpiowrite(priv->config->dout_pin, 1);
      esp_configgpio(priv->config->dout_pin, OUTPUT_FUNCTION_2);
      esp_gpio_matrix_out(priv->config->dout_pin,
                              priv->config->dout_outsig, 0, 0);
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
                            priv->config->din_insig, 0);
          esp_gpio_matrix_out(priv->config->din_pin,
                                  priv->config->dout_outsig, 0, 0);
        }
      else
        {
          esp_configgpio(priv->config->din_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->din_pin,
                            priv->config->din_insig, 0);
        }
    }

  if (priv->config->role == I2S_ROLE_SLAVE)
    {
      if (priv->config->tx_en && !priv->config->rx_en)
        {
          /* For "tx + slave" mode, select TX signal index for ws and bck */

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->ws_pin,
                                 priv->config->ws_out_insig, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->bclk_pin,
                                 priv->config->bclk_out_insig, 0);
        }
      else
        {
          /* For "tx + rx + slave" or "rx + slave" mode, select RX signal
           * index for ws and bck.
           */

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->ws_pin,
                                 priv->config->ws_in_insig, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp_gpio_matrix_in(priv->config->bclk_pin,
                                 priv->config->bclk_in_insig, 0);
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
                                  priv->config->mclk_out_sig, 0, 0);
        }

      if (priv->config->rx_en && !priv->config->tx_en)
        {
          /* For "rx + master" mode, select RX signal index for ws and bck */

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->ws_pin,
                                  priv->config->ws_in_outsig, 0, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->bclk_pin,
                                  priv->config->bclk_in_outsig, 0, 0);
        }
      else
        {
          /* For "tx + rx + master" or "tx + master" mode, select TX signal
           * index for ws and bck.
           */

          esp_gpiowrite(priv->config->ws_pin, 1);
          esp_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->ws_pin,
                                  priv->config->ws_out_outsig, 0, 0);

          esp_gpiowrite(priv->config->bclk_pin, 1);
          esp_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp_gpio_matrix_out(priv->config->bclk_pin,
                                  priv->config->bclk_out_outsig, 0, 0);
        }
    }

  /* Share BCLK and WS if in full-duplex mode */

  if (priv->config->tx_en && priv->config->rx_en)
    {
      loopback = true;
    }

  i2s_ll_share_bck_ws(priv->config->ctx->dev, loopback);

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
      priv->data_width = priv->config->data_width;

      if (priv->config->audio_std_mode <= I2S_TDM_PCM)
        {
          i2s_ll_tx_enable_std(priv->config->ctx->dev);

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
        }
      else
        {
          i2s_ll_tx_enable_pdm(priv->config->ctx->dev);
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

      i2s_setmclkfrequency((struct i2s_dev_s *)priv, (priv->config->rate *
                           priv->mclk_multiple));

      priv->rate = priv->config->rate;
      i2s_set_clock(priv);
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
          i2s_ll_rx_enable_std(priv->config->ctx->dev);

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
                                  priv->config->role == I2S_ROLE_SLAVE,
                                  &rx_slot_cfg);
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

      i2s_setmclkfrequency((struct i2s_dev_s *)priv, (priv->config->rate *
                           priv->mclk_multiple));

      priv->rate = priv->config->rate;
      i2s_set_clock(priv);
    }
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
          i2s_setmclkfrequency((struct i2s_dev_s *)priv, mclk_freq);
          return priv->mclk_freq;
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
#ifdef I2S_HAVE_TX
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
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
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
#endif /* I2S_HAVE_RX */

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
 *   None
 *
 ****************************************************************************/

static void i2s_set_clock(struct esp_i2s_s *priv)
{
  uint32_t bclk;
  uint32_t mclk;
  uint32_t sclk;
  uint32_t mclk_div;
  uint16_t bclk_div;

  sclk = priv->config->tx_clk_src;

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

#ifdef I2S_HAVE_TX
  i2s_hal_set_tx_clock(priv->config->ctx,
                       priv->config->clk_info,
                       priv->config->tx_clk_src);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  i2s_hal_set_rx_clock(priv->config->ctx,
                       priv->config->clk_info,
                       priv->config->rx_clk_src);
#endif /* I2S_HAVE_RX */
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
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static void i2s_tx_channel_start(struct esp_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      if (priv->tx_started)
        {
          i2swarn("TX channel of port %ld was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the DMA operation */

      esp_dma_reset_channel(priv->dma_channel, true);

      /* Reset the TX channel */

      /* Reset TX FIFO */

      i2s_hal_tx_reset(priv->config->ctx);
      i2s_hal_tx_reset_fifo(priv->config->ctx);

      /* Set I2S_RX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      i2s_hal_tx_start(priv->config->ctx);

      /* Enable DMA interrupt */

      up_enable_irq(priv->tx_irq);

      esp_dma_enable_interrupt(priv->dma_channel, true,
                               GDMA_LL_EVENT_TX_TOTAL_EOF |
                               GDMA_LL_EVENT_TX_DESC_ERROR,
                               true);

      priv->tx_started = true;

      i2sinfo("Started TX channel of port %ld\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_TX */

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
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static void i2s_rx_channel_start(struct esp_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      if (priv->rx_started)
        {
          i2swarn("RX channel of port %ld was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the DMA operation */

      esp_dma_reset_channel(priv->dma_channel, false);

      /* Reset the RX channel */

      /* Reset RX FIFO */

      i2s_hal_rx_reset(priv->config->ctx);
      i2s_hal_rx_reset_fifo(priv->config->ctx);

      /* Set I2S_RX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      i2s_hal_rx_start(priv->config->ctx);

      /* Enable DMA interrupt */

      up_enable_irq(priv->rx_irq);

      esp_dma_enable_interrupt(priv->dma_channel, false,
                               GDMA_LL_EVENT_RX_SUC_EOF,
                               true);

      priv->rx_started = true;

      i2sinfo("Started RX channel of port %ld\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_RX */

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
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static void i2s_tx_channel_stop(struct esp_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      if (!priv->tx_started)
        {
          i2swarn("TX channel of port %ld was previously stopped\n",
                  priv->config->port);
          return;
        }

      /* Stop TX channel */

      i2s_hal_tx_stop(priv->config->ctx);

      /* Stop outlink */

      esp_dma_disable(priv->dma_channel, true);

      /* Disable DMA interrupt */

      esp_dma_enable_interrupt(priv->dma_channel, true,
                               GDMA_LL_EVENT_TX_EOF, false);

      /* Disable DMA operation mode */

      up_disable_irq(priv->tx_irq);

      priv->tx_started = false;

      i2sinfo("Stopped TX channel of port %ld\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_TX */

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
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static void i2s_rx_channel_stop(struct esp_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      if (!priv->rx_started)
        {
          i2swarn("RX channel of port %ld was previously stopped\n",
                  priv->config->port);
          return;
        }

      /* Stop RX channel */

      i2s_hal_rx_stop(priv->config->ctx);

      /* Stop outlink */

      esp_dma_disable(priv->dma_channel, false);

      /* Disable DMA interrupt */

      esp_dma_enable_interrupt(priv->dma_channel, false,
                               GDMA_LL_EVENT_RX_SUC_EOF, false);

      /* Disable DMA operation mode */

      up_disable_irq(priv->rx_irq);

      priv->rx_started = false;

      i2sinfo("Stopped RX channel of port %ld\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_tx_interrupt
 *
 * Description:
 *   Common I2S DMA interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - I2S controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int IRAM_ATTR i2s_tx_interrupt(int irq, void *context, void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;
  struct esp_dmadesc_s *cur = NULL;

  uint32_t status =  esp_dma_get_interrupt(priv->dma_channel, true);

  esp_dma_clear_interrupt(priv->dma_channel, true, status);

  if (priv->config->tx_en)
    {
      if (status & GDMA_LL_EVENT_TX_TOTAL_EOF)
        {
          cur = (struct esp_dmadesc_s *)
                esp_dma_get_desc_addr(priv->dma_channel, true);

          /* Schedule completion of the transfer on the worker thread */

          i2s_tx_schedule(priv, cur);
        }
    }

  return 0;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rx_interrupt
 *
 * Description:
 *   Common I2S DMA interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - I2S controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_rx_interrupt(int irq, void *context, void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;
  struct esp_dmadesc_s *cur = NULL;

  uint32_t status =  esp_dma_get_interrupt(priv->dma_channel, false);

  esp_dma_clear_interrupt(priv->dma_channel, false, status);

  if (priv->config->rx_en)
    {
      if (status & GDMA_LL_EVENT_RX_SUC_EOF)
        {
          cur = (struct esp_dmadesc_s *)
                 esp_dma_get_desc_addr(priv->dma_channel, true);

          /* Schedule completion of the transfer on the worker thread */

          i2s_rx_schedule(priv, cur);
        }
    }

  return 0;
}
#endif /* I2S_HAVE_RX */

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

  /* Check if the master clock frequency is beyond the highest possible
   * value and return an error.
   */

  if (frequency >= (priv->config->tx_clk_src / 2))
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
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;
  uint32_t channels_mask;
  bool is_mono = true;

  if (priv->config->tx_en)
    {
      if (channels != 1 && channels != 2)
        {
          return -EINVAL;
        }

      i2s_tx_channel_stop(priv);

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

      i2s_tx_channel_start(priv);

      return OK;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxchannels
 *
 * Description:
 *   Set the I2S RX number of channels.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      if (channels != 1 && channels != 2)
        {
          return -EINVAL;
        }

      priv->channels = channels;
      return OK;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2S transmitter or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   OK on success, ERROR on fail
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      i2s_tx_channel_stop(priv);

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < OK)
        {
          return ERROR;
        }

      i2s_set_clock(priv);

      i2s_tx_channel_start(priv);

      return OK;
    }

  return ERROR;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxsamplerate
 *
 * Description:
 *   Set the I2S RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   OK on success, ERROR on fail
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      i2s_rx_channel_stop(priv);

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < OK)
        {
          return ERROR;
        }

      i2s_set_clock(priv);

      i2s_rx_channel_start(priv);

      return OK;
    }

  return ERROR;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_txdatawidth
 *
 * Description:
 *   Set the I2S TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      i2s_tx_channel_stop(priv);

      priv->data_width = bits;

      i2s_set_datawidth(priv);

      i2s_tx_channel_start(priv);

      return bits;
    }

  return 0;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxdatawidth
 *
 * Description:
 *   Set the I2S RX data width.  The RX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      i2s_rx_channel_stop(priv);

      priv->data_width = bits;

      i2s_set_datawidth(priv);

      i2s_rx_channel_start(priv);

      return bits;
    }

  return 0;
}
#endif /* I2S_HAVE_RX */

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

#ifdef I2S_HAVE_TX
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

      if (nbytes > (ESPRESSIF_DMA_BUFLEN_MAX * I2S_DMADESC_NUM))
        {
          i2serr("Required buffer size can't fit into DMA outlink "
                 "(exceeds in %" PRIu32 " bytes). Try to increase the "
                 "number of the DMA descriptors (CONFIG_I2S_DMADESC_NUM).",
                 nbytes - (ESPRESSIF_DMA_BUFLEN_MAX * I2S_DMADESC_NUM));
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
      i2s_buf_free(priv, bfcontainer);
      return ret;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_TX */

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

#ifdef I2S_HAVE_RX
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

      nbytes = MIN(nbytes, ESPRESSIF_DMA_BUFLEN_MAX);

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
      i2s_dump_buffer("Recieved Audio pipeline buffer:",
                      &apb->samp[apb->curbyte],
                      apb->nbytes - apb->curbyte);

      nxmutex_unlock(&priv->lock);

      return OK;

errout_with_buf:
      nxmutex_unlock(&priv->lock);
      i2s_buf_free(priv, bfcontainer);
      return ret;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_RX */

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
  int ret;
  int i2s_dma_dev;
  int periph;

  i2s_dma_dev = ESPRESSIF_DMA_PERIPH_I2S;

  /* Request a GDMA channel for the I2S peripheral */

  esp_dma_init();
  priv->dma_channel = esp_dma_request(i2s_dma_dev, 1, 1, false);
  if (priv->dma_channel < 0)
    {
      i2serr("Failed to allocate GDMA channel\n");
      return ERROR;
    }

  /* Set up to receive GDMA interrupts on the current CPU. Each TX/RX channel
   * will be assigned to a different CPU interrupt.
   */

  priv->cpu = this_cpu();

#ifdef I2S_HAVE_TX
  if (priv->config->tx_en)
    {
      periph =
        gdma_periph_signals.groups[priv->dma_channel].pairs[0].tx_irq_id;
      int cpuint = esp_setup_irq(periph, 1,
                                 ESP_IRQ_TRIGGER_LEVEL);
      if (cpuint < 0)
        {
          i2serr("Failed to allocate a CPU interrupt.\n");
          return ERROR;
        }

      priv->tx_irq = ESP_SOURCE2IRQ(periph);
      ret = irq_attach(priv->tx_irq, i2s_tx_interrupt, priv);
      if (ret != OK)
        {
          i2serr("Couldn't attach IRQ to handler.\n");
          esp_teardown_irq(periph, cpuint);
          return ret;
        }
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  if (priv->config->rx_en)
    {
      periph =
        gdma_periph_signals.groups[priv->dma_channel].pairs[0].rx_irq_id;
      int cpuint = esp_setup_irq(periph, 1,
                                 ESP_IRQ_TRIGGER_LEVEL);
      if (cpuint < 0)
        {
          i2serr("Failed to allocate a CPU interrupt.\n");
          return ERROR;
        }

      priv->rx_irq = ESP_SOURCE2IRQ(periph);
      ret = irq_attach(priv->rx_irq, i2s_rx_interrupt, priv);
      if (ret != OK)
        {
          i2serr("Couldn't attach IRQ to handler.\n");
          esp_teardown_irq(periph, cpuint);
          return ret;
        }
    }
#endif /* I2S_HAVE_RX */

  return OK;
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

  /* Statically allocated I2S' device strucuture */

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
      goto err;
    }

  flags = spin_lock_irqsave(&priv->slock);

  i2s_configure(priv);

  ret = i2s_dma_setup(priv);
  if (ret < 0)
    {
      goto err;
    }

#ifdef I2S_HAVE_TX
  /* Start TX channel */

  if (priv->config->tx_en)
    {
      priv->tx_started = false;
      i2s_tx_channel_start(priv);
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  /* Start RX channel */

  if (priv->config->rx_en)
    {
      priv->rx_started = false;
      i2s_rx_channel_start(priv);
    }
#endif /* I2S_HAVE_RX */

  spin_unlock_irqrestore(&priv->slock, flags);

  /* Success exit */

  i2sinfo("I2S%ld was successfully initialized\n", priv->config->port);

  return &priv->dev;

  /* Failure exit */

err:
  spin_unlock_irqrestore(&priv->slock, flags);
  return NULL;
}
