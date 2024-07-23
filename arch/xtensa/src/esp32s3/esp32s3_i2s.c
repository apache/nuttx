/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_i2s.c
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

#ifdef CONFIG_ESP32S3_I2S

#include <debug.h>
#include <sys/param.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/mqueue.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include <arch/board/board.h>

#include "esp32s3_i2s.h"
#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#include "esp32s3_dma.h"

#include "xtensa.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_i2s.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_iomux.h"
#include "hardware/esp32s3_pinmap.h"
#include "hardware/esp32s3_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S DMA RX/TX description number */

#define I2S_DMADESC_NUM                 (CONFIG_I2S_DMADESC_NUM)

/* I2S Clock */

#define I2S_LL_BASE_CLK                 (2 * APB_CLK_FREQ)
#define I2S_LL_MCLK_DIVIDER_BIT_WIDTH   (6)
#define I2S_LL_MCLK_DIVIDER_MAX         ((1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1)

/* I2S DMA channel number */

#define I2S_DMA_CHANNEL_MAX (2)

#ifdef CONFIG_ESP32S3_I2S0_TX
#  define I2S0_TX_ENABLED 1
#  define I2S_HAVE_TX 1
#else
#  define I2S0_TX_ENABLED 0
#endif

#ifdef CONFIG_ESP32S3_I2S0_RX
#  define I2S0_RX_ENABLED 1
#  define I2S_HAVE_RX 1
#else
#  define I2S0_RX_ENABLED 0
#endif

#ifdef CONFIG_ESP32S3_I2S1_TX
#  define I2S1_TX_ENABLED 1
#  define I2S_HAVE_TX 1
#else
#  define I2S1_TX_ENABLED 0
#endif

#ifdef CONFIG_ESP32S3_I2S1_RX
#  define I2S1_RX_ENABLED 1
#  define I2S_HAVE_RX 1
#else
#  define I2S1_RX_ENABLED 0
#endif

#ifndef ALIGN_UP
#  define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
#  define CONFIG_ESP32S3_I2S_DUMPBUFFERS
#else
#  undef CONFIG_ESP32S3_I2S_DUMPBUFFERS
#endif

#ifndef CONFIG_ESP32S3_I2S_MAXINFLIGHT
#  define CONFIG_ESP32S3_I2S_MAXINFLIGHT 4
#endif

#define I2S_GPIO_UNUSED -1      /* For signals which are not used */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Role of the I2S port */

typedef enum
{
  I2S_ROLE_MASTER,  /* I2S controller master role, bclk and ws signal will be set to output */
  I2S_ROLE_SLAVE    /* I2S controller slave role, bclk and ws signal will be set to input */
} i2s_role_t;

/* Data width of the I2S channel */

typedef enum
{
  I2S_DATA_BIT_WIDTH_8BIT   = 8,    /* I2S channel data bit-width: 8 */
  I2S_DATA_BIT_WIDTH_16BIT  = 16,   /* I2S channel data bit-width: 16 */
  I2S_DATA_BIT_WIDTH_24BIT  = 24,   /* I2S channel data bit-width: 24 */
  I2S_DATA_BIT_WIDTH_32BIT  = 32,   /* I2S channel data bit-width: 32 */
} i2s_data_bit_width_t;

/* Multiplier of MCLK to sample rate */

typedef enum
{
  I2S_MCLK_MULTIPLE_128 = 128,  /* mclk = sample_rate * 128 */
  I2S_MCLK_MULTIPLE_256 = 256,  /* mclk = sample_rate * 256 */
  I2S_MCLK_MULTIPLE_384 = 384,  /* mclk = sample_rate * 384 */
  I2S_MCLK_MULTIPLE_512 = 512,  /* mclk = sample_rate * 512 */
} i2s_mclk_multiple_t;

/* I2S Clock Source */

typedef enum
{
  I2S_XTAL_CLK = 0,
  I2S_PLL_D2_CLK,
  I2S_PLL_F160M_CLK,
  I2S_MCLK_IN,
} i2s_clk_src_t;

/* I2S Audio Standard Mode */

typedef enum
{
  I2S_TDM_PHILIPS = 0,
  I2S_TDM_MSB,
  I2S_TDM_PCM,
  I2S_PDM,
} i2s_audio_mode_t;

/* Role of the I2S port */

typedef enum
{
  I2S_RX,   /* I2S controller receiver (RX) */
  I2S_TX    /* I2S controller transmitter (TX) */
} i2s_dir_t;

/* I2S Device hardware configuration */

struct esp32s3_i2s_config_s
{
  uint32_t port;              /* I2S port */
  uint32_t role;              /* I2S port role (master or slave) */
  uint8_t data_width;         /* I2S sample data width */
  uint32_t rate;              /* I2S sample-rate */
  uint32_t total_slot;        /* Total slot number */

  bool tx_en;                 /* Is TX enabled? */
  bool rx_en;                 /* Is RX enabled? */
  int8_t mclk_pin;            /* MCLK pin, output */

  int tx_clk_src;             /* Select the I2S TX source clock */
  int rx_clk_src;             /* Select the I2S TX source clock */

  /* BCLK pin, input in slave role, output in master role */

  int8_t bclk_pin;

  /* WS pin, input in slave role, output in master role */

  int8_t ws_pin;

  int8_t dout_pin;            /* DATA pin, output */
  int8_t din_pin;             /* DATA pin, input */

  uint32_t bclk_in_insig;     /* RX channel BCK signal (slave mode) index */
  uint32_t bclk_in_outsig;    /* RX channel BCK signal (master mode) index */
  uint32_t bclk_out_insig;    /* TX channel BCK signal (slave mode) index */
  uint32_t bclk_out_outsig;   /* TX channel BCK signal (master mode) index */
  uint32_t ws_in_insig;       /* RX channel WS signal (slave mode) index */
  uint32_t ws_in_outsig;      /* RX channel WS signal (master mode) index */
  uint32_t ws_out_insig;      /* TX channel WS signal (slave mode) index */
  uint32_t ws_out_outsig;     /* TX channel WS signal (master mode) index */
  uint32_t din_insig;         /* RX channel Data Input signal index */
  uint32_t dout_outsig;       /* TX channel Data Output signal index */
  uint32_t mclk_out_sig;      /* Master clock output index */

  uint8_t  audio_std_mode;    /* Select audio standard (i2s_audio_mode_t) */

  /* WS signal polarity, set true to enable high level first */

  bool ws_pol;
};

struct esp32s3_buffer_s
{
  struct esp32s3_buffer_s *flink; /* Supports a singly linked list */

  /* The associated DMA in/outlink */

  struct esp32s3_dmadesc_s dma_link[I2S_DMADESC_NUM];

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

struct esp32s3_buffer_carry_s
{
  uint32_t value;
  size_t bytes;
};

/* This structure describes the state of one receiver or transmitter
 * transport.
 */

struct esp32s3_transport_s
{
  sq_queue_t pend;              /* A queue of pending transfers */
  sq_queue_t act;               /* A queue of active transfers */
  sq_queue_t done;              /* A queue of completed transfers */
  struct work_s work;           /* Supports worker thread operations */

  /* Bytes to be written at the beginning of the next DMA buffer */

  struct esp32s3_buffer_carry_s carry;
};

/* The state of the one I2S peripheral */

struct esp32s3_i2s_s
{
  struct i2s_dev_s  dev;        /* Externally visible I2S interface */
  mutex_t           lock;       /* Ensures mutually exclusive access */
  uint8_t           cpu;        /* CPU ID */
  spinlock_t        slock;      /* Device specific lock. */

  /* Port configuration */

  const struct esp32s3_i2s_config_s *config;

  uint32_t    mclk_freq;      /* I2S actual master clock */
  uint32_t    mclk_multiple;  /* The multiple of mclk to the sample rate */
  uint32_t    channels;       /* Audio channels (1:mono or 2:stereo) */
  uint32_t    rate;           /* I2S actual configured sample-rate */
  uint32_t    data_width;     /* I2S actual configured data_width */
  uint32_t    dma_channel;    /* I2S DMA channel being used */

#ifdef I2S_HAVE_TX
  struct esp32s3_transport_s tx;  /* TX transport state */

  int  tx_irq;                    /* TX IRQ */
  bool tx_started;                /* TX channel started */
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  struct esp32s3_transport_s rx;  /* RX transport state */

  int  rx_irq;                    /* RX IRQ */
  bool rx_started;                /* RX channel started */
#endif /* I2S_HAVE_RX */

  bool streaming;                 /* Is I2S peripheral active? */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                         /* Buffer wait semaphore */
  struct esp32s3_buffer_s *bf_freelist; /* A list a free buffer containers */
  struct esp32s3_buffer_s containers[CONFIG_ESP32S3_I2S_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_ESP32S3_I2S_DUMPBUFFERS
#  define       i2s_dump_buffer(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define       i2s_dump_buffer(m,b,s)
#endif

/* Buffer container helpers */

static struct esp32s3_buffer_s *
                i2s_buf_allocate(struct esp32s3_i2s_s *priv);
static void     i2s_buf_free(struct esp32s3_i2s_s *priv,
                             struct esp32s3_buffer_s *bfcontainer);
static int      i2s_buf_initialize(struct esp32s3_i2s_s *priv);

/* DMA support */

#ifdef I2S_HAVE_TX
static IRAM_ATTR int  i2s_txdma_setup(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_buffer_s *bfcontainer);
static void           i2s_tx_worker(void *arg);
static void           i2s_tx_schedule(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_dmadesc_s *outlink);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static IRAM_ATTR int  i2s_rxdma_setup(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_buffer_s *bfcontainer);
static void           i2s_rx_worker(void *arg);
static void           i2s_rx_schedule(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_dmadesc_s *outlink);
#endif /* I2S_HAVE_RX */

/* I2S methods (and close friends) */

static int32_t  i2s_check_mclkfrequency(struct esp32s3_i2s_s *priv);
static uint32_t i2s_set_datawidth(struct esp32s3_i2s_s *priv);
static uint32_t i2s_set_clock(struct esp32s3_i2s_s *priv);
static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev);
static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency);
static int      i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg);

#ifdef I2S_HAVE_TX
static void     i2s_tx_channel_start(struct esp32s3_i2s_s *priv);
static void     i2s_tx_channel_stop(struct esp32s3_i2s_s *priv);
static int      i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                         i2s_callback_t callback, void *arg,
                         uint32_t timeout);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static void     i2s_rx_channel_start(struct esp32s3_i2s_s *priv);
static void     i2s_rx_channel_stop(struct esp32s3_i2s_s *priv);
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

#ifdef CONFIG_ESP32S3_I2S0
static const struct esp32s3_i2s_config_s esp32s3_i2s0_config =
{
  .port             = 0,
#ifdef CONFIG_ESP32S3_I2S0_ROLE_MASTER
  .role             = I2S_ROLE_MASTER,
#else
  .role             = I2S_ROLE_SLAVE,
#endif /* CONFIG_ESP32S3_I2S0_ROLE_MASTER */
  .data_width       = CONFIG_ESP32S3_I2S0_DATA_BIT_WIDTH,
  .rate             = CONFIG_ESP32S3_I2S0_SAMPLE_RATE,
  .total_slot       = 2,
  .tx_en            = I2S0_TX_ENABLED,
  .rx_en            = I2S0_RX_ENABLED,
  .tx_clk_src       = I2S_PLL_D2_CLK,
  .rx_clk_src       = I2S_PLL_D2_CLK,
#ifdef CONFIG_ESP32S3_I2S0_MCLK
  .mclk_pin         = CONFIG_ESP32S3_I2S0_MCLKPIN,
#else
  .mclk_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S0_MCLK */
  .bclk_pin         = CONFIG_ESP32S3_I2S0_BCLKPIN,
  .ws_pin           = CONFIG_ESP32S3_I2S0_WSPIN,
#ifdef CONFIG_ESP32S3_I2S0_DOUTPIN
  .dout_pin         = CONFIG_ESP32S3_I2S0_DOUTPIN,
#else
  .dout_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S0_DOUTPIN */
#ifdef CONFIG_ESP32S3_I2S0_DINPIN
  .din_pin          = CONFIG_ESP32S3_I2S0_DINPIN,
#else
  .din_pin          = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S0_DINPIN */
  .bclk_in_insig    = I2S0I_BCK_IN_IDX,
  .bclk_in_outsig   = I2S0I_BCK_OUT_IDX,
  .bclk_out_insig   = I2S0O_BCK_IN_IDX,
  .bclk_out_outsig  = I2S0O_BCK_OUT_IDX,
  .ws_in_insig      = I2S0I_WS_IN_IDX,
  .ws_in_outsig     = I2S0I_WS_OUT_IDX,
  .ws_out_insig     = I2S0O_WS_IN_IDX,
  .ws_out_outsig    = I2S0O_WS_OUT_IDX,
  .din_insig        = I2S0I_SD_IN_IDX,
  .dout_outsig      = I2S0O_SD_OUT_IDX,
  .mclk_out_sig     = I2S0_MCLK_OUT_IDX,
  .audio_std_mode   = I2S_TDM_PHILIPS,
};

static struct esp32s3_i2s_s esp32s3_i2s0_priv =
{
  .dev =
  {
    .ops = &g_i2sops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .config = &esp32s3_i2s0_config,
  .bufsem = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32S3_I2S0 */

#ifdef CONFIG_ESP32S3_I2S1
static const struct esp32s3_i2s_config_s esp32s3_i2s1_config =
{
  .port             = 1,
#ifdef CONFIG_ESP32S3_I2S1_ROLE_MASTER
  .role             = I2S_ROLE_MASTER,
#else
  .role             = I2S_ROLE_SLAVE,
#endif /* CONFIG_ESP32S3_I2S1_ROLE_MASTER */
  .data_width       = CONFIG_ESP32S3_I2S1_DATA_BIT_WIDTH,
  .rate             = CONFIG_ESP32S3_I2S1_SAMPLE_RATE,
  .total_slot       = 2,
  .tx_en            = I2S1_TX_ENABLED,
  .rx_en            = I2S1_RX_ENABLED,
  .tx_clk_src       = I2S_PLL_D2_CLK,
  .tx_clk_src       = I2S_PLL_D2_CLK,
#ifdef CONFIG_ESP32S3_I2S1_MCLK
  .mclk_pin         = CONFIG_ESP32S3_I2S1_MCLKPIN,
#else
  .mclk_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S1_MCLK */
  .bclk_pin         = CONFIG_ESP32S3_I2S1_BCLKPIN,
  .ws_pin           = CONFIG_ESP32S3_I2S1_WSPIN,
#ifdef CONFIG_ESP32S3_I2S1_DOUTPIN
  .dout_pin         = CONFIG_ESP32S3_I2S1_DOUTPIN,
#else
  .dout_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S1_DOUTPIN */
#ifdef CONFIG_ESP32S3_I2S1_DINPIN
  .din_pin          = CONFIG_ESP32S3_I2S1_DINPIN,
#else
  .din_pin          = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S1_DINPIN */
  .bclk_in_insig    = I2S1I_BCK_IN_IDX,
  .bclk_in_outsig   = I2S1I_BCK_OUT_IDX,
  .bclk_out_insig   = I2S1O_BCK_IN_IDX,
  .bclk_out_outsig  = I2S1O_BCK_OUT_IDX,
  .ws_in_insig      = I2S1I_WS_IN_IDX,
  .ws_in_outsig     = I2S1I_WS_OUT_IDX,
  .ws_out_insig     = I2S1O_WS_IN_IDX,
  .ws_out_outsig    = I2S1O_WS_OUT_IDX,
  .din_insig        = I2S1I_SD_IN_IDX,
  .dout_outsig      = I2S1O_SD_OUT_IDX,
  .mclk_out_sig     = I2S1_MCLK_OUT_IDX,
  .audio_std_mode   = I2S_TDM_PHILIPS,
};

static struct esp32s3_i2s_s esp32s3_i2s1_priv =
{
  .dev =
  {
    .ops = &g_i2sops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .config = &esp32s3_i2s1_config,
  .bufsem = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32S3_I2S1 */

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

static struct esp32s3_buffer_s *i2s_buf_allocate(struct esp32s3_i2s_s *priv)
{
  struct esp32s3_buffer_s *bfcontainer;
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
 *   priv - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the I2S state structure
 *
 ****************************************************************************/

static void i2s_buf_free(struct esp32s3_i2s_s *priv,
                         struct esp32s3_buffer_s *bfcontainer)
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

static int i2s_buf_initialize(struct esp32s3_i2s_s *priv)
{
#ifdef I2S_HAVE_TX
  priv->tx.carry.bytes = 0;
  priv->tx.carry.value = 0;
#endif /* I2S_HAVE_TX */

  priv->bf_freelist = NULL;
  for (int i = 0; i < CONFIG_ESP32S3_I2S_MAXINFLIGHT; i++)
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
static int IRAM_ATTR i2s_txdma_start(struct esp32s3_i2s_s *priv)
{
  struct esp32s3_buffer_s *bfcontainer;

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

  /* Start transmission if no data is already being transmitted */

  bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->tx.pend);

  esp32s3_dma_load(bfcontainer->dma_link, priv->dma_channel, I2S_TX);
  esp32s3_dma_enable(priv->dma_channel, I2S_TX);

  modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_START);

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
static int i2s_rxdma_start(struct esp32s3_i2s_s *priv)
{
  struct esp32s3_buffer_s *bfcontainer;
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

  bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->rx.pend);

  /* If there isn't already an active transmission in progress,
   * then start it.
   */

  eof_nbytes = MIN(bfcontainer->nbytes, ESP32S3_DMA_BUFLEN_MAX);

  modifyreg32(I2S_RXEOF_NUM_REG(priv->config->port), I2S_RX_EOF_NUM_M,
              FIELD_TO_VALUE(I2S_RX_EOF_NUM, eof_nbytes));

  esp32s3_dma_load(bfcontainer->dma_link, priv->dma_channel, I2S_RX);
  esp32s3_dma_enable(priv->dma_channel, I2S_RX);

  modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_START);

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

#ifdef I2S_HAVE_TX
static IRAM_ATTR int i2s_txdma_setup(struct esp32s3_i2s_s *priv,
                                     struct esp32s3_buffer_s *bfcontainer)
{
  int ret = OK;
  size_t carry_size;
  uint32_t bytes_queued;
  uint32_t data_copied;
  struct ap_buffer_s *apb;
  struct esp32s3_dmadesc_s *outlink;
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

  bytes_queued = esp32s3_dma_setup(outlink, I2S_DMADESC_NUM,
                                   bfcontainer->buf,
                                   bfcontainer->nbytes, I2S_TX,
                                   priv->dma_channel);

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
static int i2s_rxdma_setup(struct esp32s3_i2s_s *priv,
                           struct esp32s3_buffer_s *bfcontainer)
{
  int ret = OK;
  struct esp32s3_dmadesc_s *inlink;
  uint32_t bytes_queued;
  irqstate_t flags;

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  inlink = bfcontainer->dma_link;

  /* Configure DMA stream */

  bytes_queued = esp32s3_dma_setup(inlink, I2S_DMADESC_NUM,
                                   bfcontainer->apb->samp,
                                   bfcontainer->nbytes, I2S_RX,
                                   priv->dma_channel);

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
static void IRAM_ATTR i2s_tx_schedule(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_dmadesc_s *outlink)
{
  struct esp32s3_buffer_s *bfcontainer;
  struct esp32s3_dmadesc_s *bfdesc;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.
   */

  /* Move all entries from the tx.act queue to the tx.done queue */

  if (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct esp32s3_buffer_s *)sq_peek(&priv->tx.act);

      /* Check if the DMA descriptor that generated an EOF interrupt is the
       * last descriptor of the current buffer container's DMA outlink.
       * REVISIT: what to do if we miss syncronization and the descriptor
       * that generated the interrupt is different from the expected (the
       * oldest of the list containing active transmissions)?
       */

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link;
      while (!(bfdesc->ctrl & ESP32S3_DMA_CTRL_EOF))
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
static void i2s_rx_schedule(struct esp32s3_i2s_s *priv,
                            struct esp32s3_dmadesc_s *inlink)
{
  struct esp32s3_buffer_s *bfcontainer;
  struct esp32s3_dmadesc_s *bfdesc;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->rx.act queue.
   */

  /* Move all entries from the rx.act queue to the rx.done queue */

  if (!sq_empty(&priv->rx.act))
    {
      /* Remove the next buffer container from the rx.act list */

      bfcontainer = (struct esp32s3_buffer_s *)sq_peek(&priv->rx.act);

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link;

      while (bfdesc->next != NULL &&
             (bfdesc->next->ctrl & ESP32S3_DMA_CTRL_EOF))
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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_buffer_s *bfcontainer;
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
      bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->tx.done);
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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_buffer_s *bfcontainer;
  struct esp32s3_dmadesc_s *dmadesc;
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
      bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->rx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      dmadesc = bfcontainer->dma_link;

      bfcontainer->apb->nbytes = 0;

      while (dmadesc != NULL && (dmadesc->ctrl & ESP32S3_DMA_CTRL_EOF))
        {
          bfcontainer->apb->nbytes +=
              (dmadesc->ctrl >> ESP32S3_DMA_CTRL_DATALEN_S) &
              ESP32S3_DMA_CTRL_DATALEN_V;
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

static void i2s_configure(struct esp32s3_i2s_s *priv)
{
  uint32_t tx_conf  = 0;
  uint32_t tx_conf1 = 0;
  uint32_t rx_conf  = 0;
  uint32_t rx_conf1 = 0;

  /* Set peripheral clock and clear reset */

  if (priv->config->port == ESP32S3_I2S0)
    {
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_I2S0_CLK_EN);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, SYSTEM_I2S0_RST);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2S0_RST, 0);
    }
  else
    {
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_I2S1_CLK_EN);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, SYSTEM_I2S1_RST);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2S1_RST, 0);
    }

  if (!(getreg32(I2S_TX_CLKM_CONF_REG(priv->config->port)) & I2S_CLK_EN))
    {
      i2sinfo("Enabling I2S clock...\n");
      modifyreg32(I2S_TX_CLKM_CONF_REG(priv->config->port), 0, I2S_CLK_EN);
    }

  /* Configure multiplexed pins as connected on the board */

  /* TODO: check for loopback mode */

  /* Enable TX channel */

  if (priv->config->dout_pin != I2S_GPIO_UNUSED)
    {
      /* If TX channel is used, enable the clock source */

      if (!(getreg32(I2S_TX_CLKM_CONF_REG(priv->config->port)) &
            I2S_TX_CLK_ACTIVE))
        {
          i2sinfo("Activating I2S TX clock...\n");
          modifyreg32(I2S_TX_CLKM_CONF_REG(priv->config->port),
                      I2S_TX_CLK_SEL_M,
                      FIELD_TO_VALUE(I2S_TX_CLK_SEL, 2));
          modifyreg32(I2S_TX_CLKM_CONF_REG(priv->config->port), 0,
                      I2S_TX_CLK_ACTIVE);
        }

      esp32s3_gpiowrite(priv->config->dout_pin, 1);
      esp32s3_configgpio(priv->config->dout_pin, OUTPUT_FUNCTION_2);
      esp32s3_gpio_matrix_out(priv->config->dout_pin,
                              priv->config->dout_outsig, 0, 0);
    }

  /* Enable RX channel */

  if (priv->config->din_pin != I2S_GPIO_UNUSED)
    {
      /* If RX channel is used, enable the clock source */

      if (!(getreg32(I2S_RX_CLKM_CONF_REG(priv->config->port)) &
            I2S_RX_CLK_ACTIVE))
        {
          i2sinfo("Activating I2S TX clock...\n");
          modifyreg32(I2S_RX_CLKM_CONF_REG(priv->config->port),
                      I2S_RX_CLK_SEL_M,
                      FIELD_TO_VALUE(I2S_RX_CLK_SEL, 2));
          modifyreg32(I2S_RX_CLKM_CONF_REG(priv->config->port), 0,
                      I2S_RX_CLK_ACTIVE);
        }

      esp32s3_configgpio(priv->config->din_pin, INPUT_FUNCTION_2);
      esp32s3_gpio_matrix_in(priv->config->din_pin,
                             priv->config->din_insig, 0);
    }

  if (priv->config->role == I2S_ROLE_SLAVE)
    {
      if (priv->config->tx_en && !priv->config->rx_en)
        {
          /* For "tx + slave" mode, select TX signal index for ws and bck */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->ws_pin,
                                 priv->config->ws_out_insig, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->bclk_pin,
                                 priv->config->bclk_out_insig, 0);
        }
      else
        {
          /* For "tx + rx + slave" or "rx + slave" mode, select RX signal
           * index for ws and bck.
           */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->ws_pin,
                                 priv->config->ws_in_insig, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->bclk_pin,
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

          esp32s3_gpiowrite(priv->config->mclk_pin, 1);
          esp32s3_configgpio(priv->config->mclk_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->mclk_pin,
                                  priv->config->mclk_out_sig, 0, 0);
        }

      if (priv->config->rx_en && !priv->config->tx_en)
        {
          /* For "rx + master" mode, select RX signal index for ws and bck */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->ws_pin,
                                  priv->config->ws_in_outsig, 0, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->bclk_pin,
                                  priv->config->bclk_in_outsig, 0, 0);
        }
      else
        {
          /* For "tx + rx + master" or "tx + master" mode, select TX signal
           * index for ws and bck.
           */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->ws_pin,
                                  priv->config->ws_out_outsig, 0, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->bclk_pin,
                                  priv->config->bclk_out_outsig, 0, 0);
        }
    }

  /* Share BCLK and WS if in full-duplex mode */

  if (priv->config->tx_en && priv->config->rx_en)
    {
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_SIG_LOOPBACK);
    }
  else
    {
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_SIG_LOOPBACK, 0);
    }

  /* Configure the TX module */

  if (priv->config->tx_en)
    {
      /* Reset I2S TX module */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_RESET);
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_RESET, 0);

      /* Enable/disable I2S TX slave mode */

      if (priv->config->role == I2S_ROLE_SLAVE)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      0, I2S_TX_SLAVE_MOD);
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_SLAVE_MOD, 0);
        }

      /* Configure TX chan bit, audio data bit and mono mode.
       * On ESP32-S3, sample_bit should equals to data_bit.
       */

      /* Set TX data width */

      priv->data_width = priv->config->data_width;
      i2s_set_datawidth(priv);

      /* Set channel mode control */

      if (priv->config->audio_std_mode <= I2S_TDM_PCM)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_TDM_EN_M,
                      I2S_TX_TDM_EN);

          if (priv->config->audio_std_mode == I2S_TDM_PHILIPS)
            {
              modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                          I2S_TX_MSB_SHIFT_M, I2S_TX_MSB_SHIFT);
            }
          else
            {
              modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                          I2S_TX_MSB_SHIFT, 0);
            }
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_PDM_EN_M,
                      I2S_TX_PDM_EN);
        }

      /* Set the default level of WS */

      if (priv->config->ws_pol == 1)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0,
                      I2S_TX_WS_IDLE_POL);
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_WS_IDLE_POL, 0);
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
      /* Reset I2S RX module */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_RESET);
      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_RESET, 0);

      /* Enable/disable I2S RX slave mode */

      if (priv->config->role == I2S_ROLE_SLAVE)
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      0, I2S_RX_SLAVE_MOD);
        }
      else
        {
          /* Since BCLK and WS are shared, only TX or RX can be master. In
           * this case, force RX as slave to avoid conflict of clock signal.
           */

          if (priv->config->tx_en)
            {
              modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0,
                          I2S_RX_SLAVE_MOD);
            }
          else
            {
              modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                          I2S_RX_SLAVE_MOD, 0);
            }
        }

      /* Congfigure RX chan bit, audio data bit and mono mode.
       * On ESP32, sample_bit should equals to data_bit.
       */

      /* Set RX data width */

      priv->data_width = priv->config->data_width;
      i2s_set_datawidth(priv);

      /* Set channel mode control */

      if (priv->config->audio_std_mode <= I2S_TDM_PCM)
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_TDM_EN_M,
                      I2S_RX_TDM_EN);

          if (priv->config->audio_std_mode == I2S_TDM_PHILIPS)
            {
              modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                          I2S_RX_MSB_SHIFT_M, I2S_RX_MSB_SHIFT);
            }
          else
            {
              modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                          I2S_RX_MSB_SHIFT, 0);
            }
        }
      else
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_PDM_EN_M,
                      I2S_RX_PDM_EN);
        }

      /* Set I2S RX right channel first */

      if (priv->config->ws_pol == 1)
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0,
                      I2S_RX_BIG_ENDIAN);
        }
      else
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      I2S_RX_BIG_ENDIAN, 0);
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

static int32_t i2s_check_mclkfrequency(struct esp32s3_i2s_s *priv)
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

static uint32_t i2s_set_datawidth(struct esp32s3_i2s_s *priv)
{
#ifdef I2S_HAVE_TX
  if (priv->config->tx_en)
    {
      modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                  I2S_TX_BITS_MOD_M, FIELD_TO_VALUE(I2S_TX_BITS_MOD,
                  priv->data_width - 1));

      modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                  I2S_TX_TDM_CHAN_BITS_M,
                  FIELD_TO_VALUE(I2S_TX_TDM_CHAN_BITS,
                  priv->data_width - 1));

      modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                  I2S_TX_HALF_SAMPLE_BITS_M,
                  FIELD_TO_VALUE(I2S_TX_HALF_SAMPLE_BITS,
                  priv->data_width - 1));

      if (priv->config->audio_std_mode == I2S_TDM_PCM)
        {
          modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                      I2S_TX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_TX_TDM_WS_WIDTH, 1));
        }
      else
        {
          modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                      I2S_TX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_TX_TDM_WS_WIDTH,
                      priv->data_width - 1));
        }

      /* Set I2S_TX_UPDATE bit to update configs. Cleared automatically */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                  0, I2S_TX_UPDATE);
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  if (priv->config->rx_en)
    {
      modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                  I2S_RX_BITS_MOD_M, FIELD_TO_VALUE(I2S_RX_BITS_MOD,
                  priv->data_width - 1));

      modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                  I2S_RX_TDM_CHAN_BITS_M,
                  FIELD_TO_VALUE(I2S_RX_TDM_CHAN_BITS,
                  priv->data_width - 1));

      modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                  I2S_RX_HALF_SAMPLE_BITS_M,
                  FIELD_TO_VALUE(I2S_RX_HALF_SAMPLE_BITS,
                  priv->data_width - 1));

      if (priv->config->audio_std_mode == I2S_TDM_PCM)
        {
          modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                      I2S_RX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_RX_TDM_WS_WIDTH,
                      priv->data_width - 1));
        }
      else
        {
          modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                      I2S_RX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_RX_TDM_WS_WIDTH, 1));
        }
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
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t i2s_set_clock(struct esp32s3_i2s_s *priv)
{
  uint32_t rate;
  uint32_t bclk;
  uint32_t mclk;
  uint32_t sclk;
  uint32_t mclk_div;
  int denominator;
  int numerator;
  uint32_t regval;
  uint32_t freq_diff;
  uint16_t bclk_div;

  /* TODO: provide other clock support modes */

  /* I2S module will using PLL_D2_CLK(160M) as source clock. */

  sclk = I2S_LL_BASE_CLK;

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

  i2sinfo("Clock division info: [sclk]%" PRIu32 " Hz [mdiv] %d "
          "[mclk] %" PRIu32 " Hz [bdiv] %d [bclk] %" PRIu32 " Hz\n",
          sclk, mclk_div, mclk, bclk_div, bclk);

  freq_diff = abs((int)sclk - (int)(mclk * mclk_div));

  denominator = 1;
  numerator = 0;

  if (freq_diff)
    {
      float decimal = freq_diff / (float)mclk;

      /* Carry bit if the decimal is greater than
       * 1.0 - 1.0 / (63.0 * 2) = 125.0 / 126.0
       */

      if (decimal > 125.0f / 126.0f)
        {
          mclk_div++;
        }
      else
        {
          uint32_t min = UINT32_MAX;

          for (int a = 2; a <= I2S_LL_MCLK_DIVIDER_MAX; a++)
            {
              int b = (int)(a * (freq_diff / (double)mclk) + 0.5);
              int ma = freq_diff * a;
              int mb = mclk * b;
              if (ma == mb)
                {
                  denominator = a;
                  numerator = b;
                  break;
                }

              if (abs((mb - ma)) < min)
                {
                  denominator = a;
                  numerator = b;
                  min = abs(mb - ma);
                }
            }
        }
    }

  i2sinfo("Clock register: [mclk] %" PRIu32 " Hz [numerator] %d "
          "[denominator] %d\n", mclk, numerator, denominator);

  /* Setup TX Clock */

  regval = getreg32(I2S_TX_CLKM_CONF_REG(priv->config->port));
  regval &= ~I2S_TX_CLKM_DIV_NUM_M;
  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_NUM, mclk_div);
  putreg32(regval, I2S_TX_CLKM_CONF_REG(priv->config->port));

  regval = getreg32(I2S_TX_CLKM_DIV_CONF_REG(priv->config->port));
  regval &= ~(I2S_TX_CLKM_DIV_Z_M | I2S_TX_CLKM_DIV_Y_M |
              I2S_TX_CLKM_DIV_X_M | I2S_TX_CLKM_DIV_YN1_M);
  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_Z,
                           numerator <= (denominator / 2) ?
                           numerator : (denominator - numerator));

  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_Y,
                           numerator <= (denominator / 2) ?
                           (denominator % numerator) :
                           (denominator % (denominator - numerator)));

  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_X,
                           numerator <= (denominator / 2) ?
                           (int)(floor(denominator / numerator) - 1) :
                           (int)(floor(denominator /
                                       (denominator - numerator))-1));

  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_YN1,
                           numerator <= (denominator / 2) ?
                           0 : 1);
  putreg32(regval, I2S_TX_CLKM_DIV_CONF_REG(priv->config->port));

  /* Setup RX Clock */

  regval = getreg32(I2S_RX_CLKM_CONF_REG(priv->config->port));
  regval &= ~I2S_RX_CLKM_DIV_NUM_M;
  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_NUM, mclk_div);
  putreg32(regval, I2S_RX_CLKM_CONF_REG(priv->config->port));

  regval = getreg32(I2S_RX_CLKM_DIV_CONF_REG(priv->config->port));
  regval &= ~(I2S_RX_CLKM_DIV_Z_M | I2S_RX_CLKM_DIV_Y_M |
              I2S_RX_CLKM_DIV_X_M | I2S_RX_CLKM_DIV_YN1_M);
  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_Z,
                           numerator <= (denominator / 2) ?
                           numerator : (denominator - numerator));

  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_Y,
                           numerator <= (denominator / 2) ?
                           (denominator % numerator) :
                           (denominator % (denominator - numerator)));

  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_X,
                           numerator <= (denominator / 2) ?
                           (int)(floor(denominator / numerator) - 1) :
                           (int)(floor(denominator /
                                       (denominator - numerator))-1));

  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_YN1,
                           numerator <= (denominator / 2) ?
                           0 : 1);
  putreg32(regval, I2S_RX_CLKM_DIV_CONF_REG(priv->config->port));

  /* Set I2S TX bck div num
   * Note: I2S_TX/RX_BCK_DIV_NUM cannot be 1
   */

#ifdef I2S_HAVE_TX
  modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
              I2S_TX_BCK_DIV_NUM_M,
              FIELD_TO_VALUE(I2S_TX_BCK_DIV_NUM, bclk_div - 1));

  /* Set I2S_TX_UPDATE bit to update the configs.
   * This bit is automatically cleared.
   */

  modifyreg32(I2S_TX_CONF_REG(priv->config->port),
              0, I2S_TX_UPDATE);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
              I2S_RX_BCK_DIV_NUM_M,
              FIELD_TO_VALUE(I2S_RX_BCK_DIV_NUM, bclk_div - 1));

  /* Set I2S_RX_UPDATE bit to update the configs.
   * This bit is automatically cleared.
   */

  modifyreg32(I2S_RX_CONF_REG(priv->config->port),
              0, I2S_RX_UPDATE);
#endif /* I2S_HAVE_RX */

  /* Returns the actual sample rate */

  bclk = sclk / (float)((mclk_div + numerator / (float)denominator) *
                        bclk_div);
  rate = bclk / (float)(priv->config->total_slot * priv->data_width);

  return rate;
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
static void i2s_tx_channel_start(struct esp32s3_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      if (priv->tx_started)
        {
          i2swarn("TX channel of port %d was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the DMA operation */

      SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, priv->dma_channel,
                       DMA_OUT_CHECK_OWNER_CH0);

      /* Reset the TX channel */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_RESET);
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_RESET, 0);

      /* Reset TX FIFO */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_FIFO_RESET);
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_FIFO_RESET, 0);

      /* Set I2S_RX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_UPDATE);

      /* Enable DMA interrupt */

      up_enable_irq(priv->tx_irq);

      SET_GDMA_CH_BITS(DMA_OUT_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_OUT_TOTAL_EOF_CH0_INT_ENA |
                       DMA_OUT_DSCR_ERR_CH0_INT_ENA);

      /* REVISIT: enable I2S interrupt and set the FIFO hung counter togit
       * enable the `I2S_TX_HUNG_INT` and check for data timeout.
       */

      priv->tx_started = true;

      i2sinfo("Started TX channel of port %d\n", priv->config->port);
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
static void i2s_rx_channel_start(struct esp32s3_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      if (priv->rx_started)
        {
          i2swarn("RX channel of port %d was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the DMA operation */

      SET_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, priv->dma_channel,
                       DMA_IN_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, priv->dma_channel,
                       DMA_IN_RST_CH0_M);

      /* Reset the RX channel */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_RESET);
      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_RESET, 0);

      /* Reset RX FIFO */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_FIFO_RESET);
      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_FIFO_RESET, 0);

      /* Set I2S_RX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_UPDATE);

      /* Enable DMA interrupt */

      up_enable_irq(priv->rx_irq);

      SET_GDMA_CH_BITS(DMA_IN_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_IN_SUC_EOF_CH0_INT_ENA);

      /* REVISIT: enable I2S interrupt and set the FIFO hung counter to
       * enable the `I2S_RX_HUNG_INT` and check for data timeout.
       */

      priv->rx_started = true;

      i2sinfo("Started RX channel of port %d\n", priv->config->port);
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
static void i2s_tx_channel_stop(struct esp32s3_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      if (!priv->tx_started)
        {
          i2swarn("TX channel of port %d was previously stopped\n",
                  priv->config->port);
          return;
        }

      /* Stop TX channel */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_START, 0);

      /* Stop outlink */

      SET_GDMA_CH_BITS(DMA_OUT_LINK_CH0_REG, priv->dma_channel,
                       DMA_OUTLINK_STOP_CH0_M);

      /* Disable DMA interrupt */

      CLR_GDMA_CH_BITS(DMA_OUT_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_OUT_EOF_CH0_INT_ENA);

      /* Disable DMA operation mode */

      up_disable_irq(priv->tx_irq);

      priv->tx_started = false;

      i2sinfo("Stopped TX channel of port %d\n", priv->config->port);
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
static void i2s_rx_channel_stop(struct esp32s3_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      if (!priv->rx_started)
        {
          i2swarn("RX channel of port %d was previously stopped\n",
                  priv->config->port);
          return;
        }

      /* Stop RX channel */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_START, 0);

      /* Stop outlink */

      SET_GDMA_CH_BITS(DMA_IN_LINK_CH0_REG, priv->dma_channel,
                       DMA_INLINK_STOP_CH0_M);

      /* Disable DMA interrupt */

      CLR_GDMA_CH_BITS(DMA_IN_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_IN_SUC_EOF_CH0_INT_ENA);

      /* Disable DMA operation mode */

      up_disable_irq(priv->rx_irq);

      priv->rx_started = false;

      i2sinfo("Stopped RX channel of port %d\n", priv->config->port);
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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_dmadesc_s *cur = NULL;

  uint32_t status = GET_GDMA_CH_REG(DMA_OUT_INT_ST_CH0_REG,
                                    priv->dma_channel);

  SET_GDMA_CH_REG(DMA_OUT_INT_CLR_CH0_REG, priv->dma_channel,
                  UINT32_MAX);

  if (priv->config->tx_en)
    {
      if (status & DMA_OUT_TOTAL_EOF_CH0_INT_ST)
        {
          cur = (struct esp32s3_dmadesc_s *)
                GET_GDMA_CH_REG(DMA_OUT_EOF_DES_ADDR_CH0_REG,
                                priv->dma_channel);

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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_dmadesc_s *cur = NULL;

  uint32_t status = GET_GDMA_CH_REG(DMA_IN_INT_ST_CH0_REG,
                                    priv->dma_channel);

  SET_GDMA_CH_REG(DMA_IN_INT_CLR_CH0_REG, priv->dma_channel,
                  UINT32_MAX);

  if (priv->config->rx_en)
    {
      if (status & DMA_IN_SUC_EOF_CH0_INT_ST)
        {
          cur = (struct esp32s3_dmadesc_s *)
                GET_GDMA_CH_REG(DMA_IN_SUC_EOF_DES_ADDR_CH0_REG,
                                priv->dma_channel);

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
 *   dev        - Device-specific state data
 *
 * Returned Value:
 *   Returns the current master clock.
 *
 ****************************************************************************/

static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  /* Check if the master clock frequency is beyond the highest possible
   * value and return an error.
   */

  if (frequency >= (I2S_LL_BASE_CLK / 2))
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
 *   dev  - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;
  uint32_t channels_mask;

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

      modifyreg32(I2S_TX_TDM_CTRL_REG(priv->config->port),
                  I2S_TX_TDM_TOT_CHAN_NUM_M,
                  FIELD_TO_VALUE(I2S_TX_TDM_TOT_CHAN_NUM, 1));

      channels_mask = getreg32(I2S_TX_TDM_CTRL_REG(priv->config->port));
      channels_mask &= 0xffff0000;

      if (priv->channels == 1)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_MONO_M, I2S_TX_MONO);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_CHAN_EQUAL_M, I2S_TX_CHAN_EQUAL);
          channels_mask |= I2S_TX_TDM_CHAN0_EN;
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_MONO_M, 0);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_CHAN_EQUAL_M, 0);
          channels_mask |= I2S_TX_TDM_CHAN0_EN | I2S_TX_TDM_CHAN1_EN;
        }

      putreg32(channels_mask, I2S_TX_TDM_CTRL_REG(priv->config->port));

      /* Set I2S_TX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                  0, I2S_TX_UPDATE);

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
 *   dev  - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

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
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      i2s_tx_channel_stop(priv);

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < OK)
        {
          return 0;
        }

      rate = i2s_set_clock(priv);

      i2s_tx_channel_start(priv);

      return rate;
    }

  return 0;
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
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      i2s_rx_channel_stop(priv);

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < OK)
        {
          return 0;
        }

      rate = i2s_set_clock(priv);

      i2s_rx_channel_start(priv);

      return rate;
    }

  return 0;
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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      struct esp32s3_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check audio buffer data size from the upper half. If the buffer
       * size is not a multiple of the data width, the remaining bytes
       * must be sent along with the next audio buffer.
       */

      nbytes = (apb->nbytes - apb->curbyte) + priv->tx.carry.bytes;

      nbytes -= (nbytes % (priv->data_width / 8));

      if (nbytes > (ESP32S3_DMA_BUFLEN_MAX * I2S_DMADESC_NUM))
        {
          i2serr("Required buffer size can't fit into DMA outlink "
                 "(exceeds in %" PRIu32 " bytes). Try to increase the "
                 "number of the DMA descriptors (CONFIG_I2S_DMADESC_NUM).",
                 nbytes - (ESP32S3_DMA_BUFLEN_MAX * I2S_DMADESC_NUM));
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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      struct esp32s3_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check max audio buffer data size from the upper half and align the
       * receiving buffer according to the data width.
       */

      nbytes = apb->nmaxbytes;

      nbytes -= (nbytes % (priv->data_width / 8));

      nbytes = MIN(nbytes, ESP32S3_DMA_BUFLEN_MAX);

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
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;
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

static int i2s_dma_setup(struct esp32s3_i2s_s *priv)
{
  int ret;
  int i2s_dma_dev;

  /* Initialize GDMA controller */

  esp32s3_dma_init();

  if (priv->config->port == 0)
    {
      i2s_dma_dev = ESP32S3_DMA_PERIPH_I2S0;
    }
  else
    {
      i2s_dma_dev = ESP32S3_DMA_PERIPH_I2S1;
    }

  /* Request a GDMA channel for the I2S peripheral */

  priv->dma_channel = esp32s3_dma_request(i2s_dma_dev, 1, 1, false);
  if (priv->dma_channel < 0)
    {
      i2serr("Failed to allocate GDMA channel\n");

      return ERROR;
    }

  /* Set up to receive GDMA interrupts on the current CPU. Each TX/RX channel
   * will be assigned to a different CPU interrupt.
   */

  priv->cpu = up_cpu_index();

#ifdef I2S_HAVE_TX
  if (priv->config->tx_en)
    {
      int periph = ESP32S3_PERIPH_DMA_OUT_CH0 + priv->dma_channel;
      int cpuint = esp32s3_setup_irq(priv->cpu, periph, 1,
                                     ESP32S3_CPUINT_LEVEL);
      if (cpuint < 0)
        {
          i2serr("Failed to allocate a CPU interrupt.\n");
          return ERROR;
        }

      priv->tx_irq = ESP32S3_PERIPH2IRQ(periph);
      ret = irq_attach(priv->tx_irq, i2s_tx_interrupt, priv);
      if (ret != OK)
        {
          i2serr("Couldn't attach IRQ to handler.\n");
          esp32s3_teardown_irq(priv->cpu, periph, cpuint);
          return ret;
        }
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  if (priv->config->rx_en)
    {
      int periph = ESP32S3_PERIPH_DMA_IN_CH0 + priv->dma_channel;
      int cpuint = esp32s3_setup_irq(priv->cpu, periph, 1,
                                     ESP32S3_CPUINT_LEVEL);
      if (cpuint < 0)
        {
          i2serr("Failed to allocate a CPU interrupt.\n");
          return ERROR;
        }

      priv->rx_irq = ESP32S3_PERIPH2IRQ(periph);
      ret = irq_attach(priv->rx_irq, i2s_rx_interrupt, priv);
      if (ret != OK)
        {
          i2serr("Couldn't attach IRQ to handler.\n");
          esp32s3_teardown_irq(priv->cpu, periph, cpuint);
          return ret;
        }
    }
#endif /* I2S_HAVE_RX */

  return OK;
}

/****************************************************************************
 * Name: esp32s3_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected I2S port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2S interfaces)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *esp32s3_i2sbus_initialize(int port)
{
  int ret;
  struct esp32s3_i2s_s *priv = NULL;
  irqstate_t flags;

  i2sinfo("port: %d\n", port);

  /* Statically allocated I2S' device strucuture */

  switch (port)
    {
#ifdef CONFIG_ESP32S3_I2S0
      case ESP32S3_I2S0:
        priv = &esp32s3_i2s0_priv;
        break;
#endif
#ifdef CONFIG_ESP32S3_I2S1
      case ESP32S3_I2S1:
        priv = &esp32s3_i2s1_priv;
        break;
#endif
      default:
        return NULL;
    }

  flags = spin_lock_irqsave(&priv->slock);

  i2s_configure(priv);

  /* Allocate buffer containers */

  ret = i2s_buf_initialize(priv);
  if (ret < 0)
    {
      goto err;
    }

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

  i2sinfo("I2S%d was successfully initialized\n", priv->config->port);

  return &priv->dev;

  /* Failure exit */

err:
  spin_unlock_irqrestore(&priv->slock, flags);
  return NULL;
}

#endif /* CONFIG_ESP32S3_I2S */
