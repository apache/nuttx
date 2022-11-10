/****************************************************************************
 * arch/xtensa/src/esp32/esp32_i2s.c
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

#ifdef CONFIG_ESP32_I2S

#include <debug.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

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

#include "esp32_i2s.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"
#include "esp32_dma.h"

#include "xtensa.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_i2s.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_iomux.h"
#include "hardware/esp32_pinmap.h"
#include "hardware/esp32_dma.h"

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

#ifdef CONFIG_ESP32_I2S0_TX
#  define I2S0_TX_ENABLED 1
#  define I2S_HAVE_TX 1
#else
#  define I2S0_TX_ENABLED 0
#endif

#ifdef CONFIG_ESP32_I2S0_RX
#  define I2S0_RX_ENABLED 1
#  define I2S_HAVE_RX 1
#else
#  define I2S0_RX_ENABLED 0
#endif

#ifdef CONFIG_ESP32_I2S1_TX
#  define I2S1_TX_ENABLED 1
#  define I2S_HAVE_TX 1
#else
#  define I2S1_TX_ENABLED 0
#endif

#ifdef CONFIG_ESP32_I2S1_RX
#  define I2S1_RX_ENABLED 1
#  define I2S_HAVE_RX 1
#else
#  define I2S1_RX_ENABLED 0
#endif

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef ALIGN_UP
#  define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
#  define CONFIG_ESP32_I2S_DUMPBUFFERS
#else
#  undef CONFIG_ESP32_I2S_DUMPBUFFERS
#endif

#ifndef CONFIG_ESP32_I2S_MAXINFLIGHT
#  define CONFIG_ESP32_I2S_MAXINFLIGHT 4
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

/* I2S Device hardware configuration */

struct esp32_i2s_config_s
{
  uint32_t port;              /* I2S port */
  uint32_t role;              /* I2S port role (master or slave) */
  uint8_t data_width;         /* I2S sample data width */
  uint32_t rate;              /* I2S sample-rate */
  uint32_t total_slot;        /* Total slot number */

  bool is_apll;               /* Select APLL as the source clock */

  bool tx_en;                 /* Is TX enabled? */
  bool rx_en;                 /* Is RX enabled? */
  int8_t mclk_pin;            /* MCLK pin, output */

  /* BCLK pin, input in slave role, output in master role */

  int8_t bclk_pin;

  /* WS pin, input in slave role, output in master role */

  int8_t ws_pin;

  int8_t dout_pin;            /* DATA pin, output */
  int8_t din_pin;             /* DATA pin, input */

  uint8_t periph;             /* peripher ID */
  uint8_t irq;                /* Interrupt ID */

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

  bool bit_shift;             /* Set to enable bit shift in Philips mode */
  bool mono_en;               /* Set to enable mono mode on slot */

  /* WS signal width (the number of bclk ticks that ws signal is high) */

  uint32_t ws_width;

  /* WS signal polarity, set true to enable high lever first */

  bool ws_pol;
};

struct esp32_buffer_s
{
  struct esp32_buffer_s *flink; /* Supports a singly linked list */

  /* The associated DMA in/outlink */

  struct esp32_dmadesc_s dma_link[I2S_DMADESC_NUM];

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

struct esp32_buffer_carry_s
{
  uint32_t value;
  size_t bytes;
};

/* This structure describes the state of one receiver or transmitter
 * transport.
 */

struct esp32_transport_s
{
  sq_queue_t pend;              /* A queue of pending transfers */
  sq_queue_t act;               /* A queue of active transfers */
  sq_queue_t done;              /* A queue of completed transfers */
  struct work_s work;           /* Supports worker thread operations */

  /* Bytes to be written at the beginning of the next DMA buffer */

  struct esp32_buffer_carry_s carry;
};

/* The state of the one I2S peripheral */

struct esp32_i2s_s
{
  struct i2s_dev_s  dev;        /* Externally visible I2S interface */
  mutex_t           lock;       /* Ensures mutually exclusive access */
  int               cpuint;     /* I2S interrupt ID */
  uint8_t           cpu;        /* CPU ID */
  spinlock_t        slock;      /* Device specific lock. */

  /* Port configuration */

  const struct esp32_i2s_config_s *config;

  uint32_t    mclk_freq;      /* I2S actual master clock */
  uint32_t    mclk_multiple;  /* The multiple of mclk to the sample rate */
  uint32_t    channels;       /* Audio channels (1:mono or 2:stereo) */
  uint32_t    rate;           /* I2S actual configured sample-rate */
  uint32_t    data_width;     /* I2S actual configured data_width */

#ifdef I2S_HAVE_TX
  struct esp32_transport_s tx;  /* TX transport state */

  bool tx_started;              /* TX channel started */
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  struct esp32_transport_s rx;  /* RX transport state */

  bool rx_started;              /* RX channel started */
#endif /* I2S_HAVE_RX */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                       /* Buffer wait semaphore */
  struct esp32_buffer_s *bf_freelist; /* A list a free buffer containers */
  struct esp32_buffer_s containers[CONFIG_ESP32_I2S_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_ESP32_I2S_DUMPBUFFERS
#  define       i2s_dump_buffer(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define       i2s_dump_buffer(m,b,s)
#endif

/* Buffer container helpers */

static struct esp32_buffer_s *
                i2s_buf_allocate(struct esp32_i2s_s *priv);
static void     i2s_buf_free(struct esp32_i2s_s *priv,
                             struct esp32_buffer_s *bfcontainer);
static int      i2s_buf_initialize(struct esp32_i2s_s *priv);

/* DMA support */

#ifdef I2S_HAVE_TX
static IRAM_ATTR int  i2s_txdma_setup(struct esp32_i2s_s *priv,
                                      struct esp32_buffer_s *bfcontainer);
static void           i2s_tx_worker(void *arg);
static void           i2s_tx_schedule(struct esp32_i2s_s *priv,
                                      struct esp32_dmadesc_s *outlink);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static IRAM_ATTR int  i2s_rxdma_setup(struct esp32_i2s_s *priv,
                                      struct esp32_buffer_s *bfcontainer);
static void           i2s_rx_worker(void *arg);
static void           i2s_rx_schedule(struct esp32_i2s_s *priv,
                                      struct esp32_dmadesc_s *outlink);
#endif /* I2S_HAVE_RX */

/* I2S methods (and close friends) */

static uint32_t i2s_set_datawidth(struct esp32_i2s_s *priv);
static uint32_t i2s_set_clock(struct esp32_i2s_s *priv);
static uint32_t esp32_i2s_mclkfrequency(struct i2s_dev_s *dev,
                                        uint32_t frequency);
static int      esp32_i2s_ioctl(struct i2s_dev_s *dev, int cmd,
                                unsigned long arg);

#ifdef I2S_HAVE_TX
static void     i2s_tx_channel_start(struct esp32_i2s_s *priv);
static void     i2s_tx_channel_stop(struct esp32_i2s_s *priv);
static int      esp32_i2s_txchannels(struct i2s_dev_s *dev,
                                     uint8_t channels);
static uint32_t esp32_i2s_txsamplerate(struct i2s_dev_s *dev,
                                       uint32_t rate);
static uint32_t esp32_i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      esp32_i2s_send(struct i2s_dev_s *dev,
                               struct ap_buffer_s *apb,
                               i2s_callback_t callback, void *arg,
                               uint32_t timeout);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static void     i2s_rx_channel_start(struct esp32_i2s_s *priv);
static void     i2s_rx_channel_stop(struct esp32_i2s_s *priv);
static int      esp32_i2s_rxchannels(struct i2s_dev_s *dev,
                                     uint8_t channels);
static uint32_t esp32_i2s_rxsamplerate(struct i2s_dev_s *dev,
                                       uint32_t rate);
static uint32_t esp32_i2s_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      esp32_i2s_receive(struct i2s_dev_s *dev,
                                  struct ap_buffer_s *apb,
                                  i2s_callback_t callback, void *arg,
                                  uint32_t timeout);
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_i2sops =
{
#ifdef I2S_HAVE_TX
  .i2s_txchannels     = esp32_i2s_txchannels,
  .i2s_txsamplerate   = esp32_i2s_txsamplerate,
  .i2s_txdatawidth    = esp32_i2s_txdatawidth,
  .i2s_send           = esp32_i2s_send,
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  .i2s_rxchannels     = esp32_i2s_rxchannels,
  .i2s_rxsamplerate   = esp32_i2s_rxsamplerate,
  .i2s_rxdatawidth    = esp32_i2s_rxdatawidth,
  .i2s_receive        = esp32_i2s_receive,
#endif /* I2S_HAVE_RX */

  .i2s_ioctl          = esp32_i2s_ioctl,
  .i2s_mclkfrequency  = esp32_i2s_mclkfrequency,
};

#ifdef CONFIG_ESP32_I2S0
static const struct esp32_i2s_config_s esp32_i2s0_config =
{
  .port             = 0,
#ifdef CONFIG_ESP32_I2S0_ROLE_MASTER
  .role             = I2S_ROLE_MASTER,
#else
  .role             = I2S_ROLE_SLAVE,
#endif /* CONFIG_ESP32_I2S0_ROLE_MASTER */
  .data_width       = CONFIG_ESP32_I2S0_DATA_BIT_WIDTH,
  .rate             = CONFIG_ESP32_I2S0_SAMPLE_RATE,
  .total_slot       = 2,
  .tx_en            = I2S0_TX_ENABLED,
  .rx_en            = I2S0_RX_ENABLED,
#ifdef CONFIG_ESP32_I2S0_MCLK
  .mclk_pin         = CONFIG_ESP32_I2S0_MCLKPIN,
#else
  .mclk_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32_I2S0_MCLK */
  .bclk_pin         = CONFIG_ESP32_I2S0_BCLKPIN,
  .ws_pin           = CONFIG_ESP32_I2S0_WSPIN,
#ifdef CONFIG_ESP32_I2S0_DOUTPIN
  .dout_pin         = CONFIG_ESP32_I2S0_DOUTPIN,
#else
  .dout_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32_I2S0_DOUTPIN */
#ifdef CONFIG_ESP32_I2S0_DINPIN
  .din_pin          = CONFIG_ESP32_I2S0_DINPIN,
#else
  .din_pin          = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32_I2S0_DINPIN */
  .periph           = ESP32_PERIPH_I2S0,
  .irq              = ESP32_IRQ_I2S0,
  .bclk_in_insig    = I2S0I_BCK_IN_IDX,
  .bclk_in_outsig   = I2S0I_BCK_OUT_IDX,
  .bclk_out_insig   = I2S0O_BCK_IN_IDX,
  .bclk_out_outsig  = I2S0O_BCK_OUT_IDX,
  .ws_in_insig      = I2S0I_WS_IN_IDX,
  .ws_in_outsig     = I2S0I_WS_OUT_IDX,
  .ws_out_insig     = I2S0O_WS_IN_IDX,
  .ws_out_outsig    = I2S0O_WS_OUT_IDX,
  .din_insig        = I2S0I_DATA_IN15_IDX,
  .dout_outsig      = I2S0O_DATA_OUT23_IDX,
  .bit_shift        = true,
  .mono_en          = false,
  .ws_width         = CONFIG_ESP32_I2S0_DATA_BIT_WIDTH,
};

static struct esp32_i2s_s esp32_i2s0_priv =
{
  .dev =
  {
    .ops = &g_i2sops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .config = &esp32_i2s0_config,
  .bufsem = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32_I2S0 */

#ifdef CONFIG_ESP32_I2S1
static const struct esp32_i2s_config_s esp32_i2s1_config =
{
  .port             = 1,
#ifdef CONFIG_ESP32_I2S1_ROLE_MASTER
  .role             = I2S_ROLE_MASTER,
#else
  .role             = I2S_ROLE_SLAVE,
#endif /* CONFIG_ESP32_I2S1_ROLE_MASTER */
  .data_width       = CONFIG_ESP32_I2S1_DATA_BIT_WIDTH,
  .rate             = CONFIG_ESP32_I2S1_SAMPLE_RATE,
  .total_slot       = 2,
  .tx_en            = I2S1_TX_ENABLED,
  .rx_en            = I2S1_RX_ENABLED,
#ifdef CONFIG_ESP32_I2S1_MCLK
  .mclk_pin         = CONFIG_ESP32_I2S1_MCLKPIN,
#else
  .mclk_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32_I2S1_MCLK */
  .bclk_pin         = CONFIG_ESP32_I2S1_BCLKPIN,
  .ws_pin           = CONFIG_ESP32_I2S1_WSPIN,
#ifdef CONFIG_ESP32_I2S1_DOUTPIN
  .dout_pin         = CONFIG_ESP32_I2S1_DOUTPIN,
#else
  .dout_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32_I2S1_DOUTPIN */
#ifdef CONFIG_ESP32_I2S1_DINPIN
  .din_pin          = CONFIG_ESP32_I2S1_DINPIN,
#else
  .din_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32_I2S1_DINPIN */
  .periph           = ESP32_PERIPH_I2S1,
  .irq              = ESP32_IRQ_I2S1,
  .bclk_in_insig    = I2S1I_BCK_IN_IDX,
  .bclk_in_outsig   = I2S1I_BCK_OUT_IDX,
  .bclk_out_insig   = I2S1O_BCK_IN_IDX,
  .bclk_out_outsig  = I2S1O_BCK_OUT_IDX,
  .ws_in_insig      = I2S1I_WS_IN_IDX,
  .ws_in_outsig     = I2S1I_WS_OUT_IDX,
  .ws_out_insig     = I2S1O_WS_IN_IDX,
  .ws_out_outsig    = I2S1O_WS_OUT_IDX,
  .din_insig        = I2S1I_DATA_IN15_IDX,
  .dout_outsig      = I2S1O_DATA_OUT23_IDX,
  .bit_shift        = true,
  .mono_en          = false,
  .ws_width         = CONFIG_ESP32_I2S1_DATA_BIT_WIDTH,
};

static struct esp32_i2s_s esp32_i2s1_priv =
{
  .dev =
  {
    .ops = &g_i2sops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .config = &esp32_i2s1_config,
  .bufsem = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32_I2S1 */

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

static struct esp32_buffer_s *i2s_buf_allocate(struct esp32_i2s_s *priv)
{
  struct esp32_buffer_s *bfcontainer;
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

static void i2s_buf_free(struct esp32_i2s_s *priv,
                         struct esp32_buffer_s *bfcontainer)
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

static int i2s_buf_initialize(struct esp32_i2s_s *priv)
{
#ifdef I2S_HAVE_TX
  priv->tx.carry.bytes = 0;
  priv->tx.carry.value = 0;
#endif /* I2S_HAVE_TX */

  priv->bf_freelist = NULL;
  for (int i = 0; i < CONFIG_ESP32_I2S_MAXINFLIGHT; i++)
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
static int i2s_txdma_start(struct esp32_i2s_s *priv)
{
  struct esp32_buffer_s *bfcontainer;

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

  bfcontainer = (struct esp32_buffer_s *)sq_remfirst(&priv->tx.pend);

  /* If there isn't already an active transmission in progress,
   * then start it.
   */

  modifyreg32(I2S_OUT_LINK_REG(priv->config->port), I2S_OUTLINK_ADDR_M,
              FIELD_TO_VALUE(I2S_OUTLINK_ADDR,
              (uintptr_t) bfcontainer->dma_link));

  modifyreg32(I2S_OUT_LINK_REG(priv->config->port), I2S_OUTLINK_STOP,
              I2S_OUTLINK_START);

  modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_TX_START);

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
static int i2s_rxdma_start(struct esp32_i2s_s *priv)
{
  struct esp32_buffer_s *bfcontainer;

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

  bfcontainer = (struct esp32_buffer_s *)sq_remfirst(&priv->rx.pend);

  /* If there isn't already an active transmission in progress,
   * then start it.
   */

  modifyreg32(I2S_RXEOF_NUM_REG(priv->config->port), I2S_RX_EOF_NUM_M,
              FIELD_TO_VALUE(I2S_RX_EOF_NUM,
              (bfcontainer->nbytes / 4)));

  modifyreg32(I2S_IN_LINK_REG(priv->config->port), I2S_INLINK_ADDR_M,
              FIELD_TO_VALUE(I2S_INLINK_ADDR,
              (uintptr_t) bfcontainer->dma_link));

  modifyreg32(I2S_IN_LINK_REG(priv->config->port), I2S_INLINK_STOP,
              I2S_INLINK_START);

  modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_RX_START);

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
static IRAM_ATTR int i2s_txdma_setup(struct esp32_i2s_s *priv,
                                     struct esp32_buffer_s *bfcontainer)
{
  int ret = OK;
  size_t carry_size;
  uint32_t bytes_queued;
  uint32_t data_copied;
  struct ap_buffer_s *apb;
  struct esp32_dmadesc_s *outlink;
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

  /* Padding contains the size (in bytes) to be skipped on the
   * internal buffer do provide 1) the ability to handle mono
   * audio files and 2) to correctly fill the buffer to 16 or 32-bits
   * aligned positions.
   */

  padding = priv->channels == 1 ?
            ((priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT ?
              I2S_DATA_BIT_WIDTH_16BIT : I2S_DATA_BIT_WIDTH_32BIT) / 8) : 0;

  padding += priv->data_width != I2S_DATA_BIT_WIDTH_16BIT ?
             (priv->data_width != I2S_DATA_BIT_WIDTH_32BIT ? 1 : 0) : 0;

  /* Copy audio data into internal buffer */

  bfcontainer->buf = calloc(bfcontainer->nbytes, 1);
  if (bfcontainer->buf == NULL)
    {
      i2serr("Failed to allocate the DMA internal buffer "
             "[%" PRIu32 " bytes]", bfcontainer->nbytes);
      return -ENOMEM;
    }

  data_copied = 0;
  buf = bfcontainer->buf + padding;

  if (priv->tx.carry.bytes)
    {
      memcpy(buf, &priv->tx.carry.value, priv->tx.carry.bytes);
      buf += priv->tx.carry.bytes;
      data_copied += priv->tx.carry.bytes;
      memcpy(buf, samp, (bytes_per_sample - priv->tx.carry.bytes));
      buf += (bytes_per_sample - priv->tx.carry.bytes + padding);
      samp += (bytes_per_sample - priv->tx.carry.bytes);
      data_copied += (bytes_per_sample - priv->tx.carry.bytes);
    }

  /* If there is no need to add padding bytes, the memcpy may be done at
   * once. Otherwise, the operation must add the padding bytes to each
   * sample in the internal buffer.
   */

  if (padding)
    {
      while (data_copied < (samp_size - carry_size))
        {
          memcpy(buf, samp, bytes_per_sample);
          buf += (bytes_per_sample + padding);
          samp += bytes_per_sample;
          data_copied += bytes_per_sample;
        }
    }
  else
    {
      memcpy(buf, samp, samp_size - (data_copied + carry_size));
      buf += samp_size - (data_copied + carry_size);
      samp += samp_size - (data_copied + carry_size);
      data_copied += samp_size - (data_copied + carry_size);
    }

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

  bytes_queued = esp32_dma_init(outlink, I2S_DMADESC_NUM,
                                bfcontainer->buf, bfcontainer->nbytes);

  if (bytes_queued != bfcontainer->nbytes)
    {
      i2serr("Failed to enqueue I2S buffer "
             "(%" PRIu32 " bytes of %" PRIu32 ")\n",
             bytes_queued, bfcontainer->nbytes);
      return bytes_queued;
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
static int i2s_rxdma_setup(struct esp32_i2s_s *priv,
                           struct esp32_buffer_s *bfcontainer)
{
  int ret = OK;
  struct esp32_dmadesc_s *inlink;
  uint32_t bytes_queued;
  irqstate_t flags;

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  inlink = bfcontainer->dma_link;

  /* Allocate the internal buffer for RX */

  bfcontainer->buf = calloc(bfcontainer->nbytes, 1);
  if (bfcontainer->buf == NULL)
    {
      i2serr("Failed to allocate the DMA internal buffer "
             "[%" PRIu32 " bytes]", bfcontainer->nbytes);
      return -ENOMEM;
    }

  /* Configure DMA stream */

  bytes_queued = esp32_dma_init(inlink, I2S_DMADESC_NUM,
                                bfcontainer->buf,
                                bfcontainer->nbytes);

  if (bytes_queued != bfcontainer->nbytes)
    {
      i2serr("Failed to enqueue I2S buffer "
             "(%" PRIu32 " bytes of %" PRIu32 ")\n",
             bytes_queued, bfcontainer->nbytes);
      return bytes_queued;
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
static void i2s_tx_schedule(struct esp32_i2s_s *priv,
                            struct esp32_dmadesc_s *outlink)
{
  struct esp32_buffer_s *bfcontainer;
  struct esp32_dmadesc_s *bfdesc;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.
   */

  /* Move all entries from the tx.act queue to the tx.done queue */

  if (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct esp32_buffer_s *)sq_peek(&priv->tx.act);

      /* Check if the DMA descriptor that generated an EOF interrupt is the
       * last descriptor of the current buffer container's DMA outlink.
       * REVISIT: what to do if we miss syncronization and the descriptor
       * that generated the interrupt is different from the expected (the
       * oldest of the list containing active transmissions)?
       */

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link;
      while (!(bfdesc->ctrl & DMA_CTRL_EOF))
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
static void i2s_rx_schedule(struct esp32_i2s_s *priv,
                            struct esp32_dmadesc_s *inlink)
{
  struct esp32_buffer_s *bfcontainer;
  struct esp32_dmadesc_s *bfdesc;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->rx.act queue.
   */

  /* Move all entries from the rx.act queue to the rx.done queue */

  if (!sq_empty(&priv->rx.act))
    {
      /* Remove the next buffer container from the rx.act list */

      bfcontainer = (struct esp32_buffer_s *)sq_peek(&priv->rx.act);

      /* Check if the DMA descriptor that generated an EOF interrupt is the
       * last descriptor of the current buffer container's DMA inlink.
       * REVISIT: what to do if we miss syncronization and the descriptor
       * that generated the interrupt is different from the expected (the
       * oldest of the list containing active transmissions)?
       */

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link;
      while (!(bfdesc->ctrl & DMA_CTRL_EOF))
        {
          DEBUGASSERT(bfdesc->next);
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
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)arg;
  struct esp32_buffer_s *bfcontainer;
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
      bfcontainer = (struct esp32_buffer_s *)sq_remfirst(&priv->tx.done);
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
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)arg;
  struct esp32_buffer_s *bfcontainer;
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
      bfcontainer = (struct esp32_buffer_s *)sq_remfirst(&priv->rx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      apb_samp_t samp_size;
      uint32_t data_copied;
      uint8_t padding;
      uint8_t *buf;
      uint8_t *samp;

      /* Padding contains the size (in bytes) to be skipped on the internal
       * buffer - which store data aligned with 2 or 4 bytes - to correctly
       * copy and fill the audio buffer according to the actual data width.
       */

      padding = priv->data_width != I2S_DATA_BIT_WIDTH_16BIT ?
                (priv->data_width != I2S_DATA_BIT_WIDTH_32BIT ? 1 : 0) : 0;

      /* Get the transfer information, accounting for any data offset */

      const apb_samp_t bytes_per_sample = priv->data_width / 8;
      samp = &bfcontainer->apb->samp[bfcontainer->apb->curbyte];
      samp_size = (bfcontainer->apb->nbytes - bfcontainer->apb->curbyte);

      /* If there is no need to add padding bytes, the memcpy may be done at
       * once. Otherwise, the operation must add the padding bytes to each
       * sample in the internal buffer.
       */

      data_copied = 0;
      buf = bfcontainer->buf + padding;

      if (padding)
        {
          while (data_copied < samp_size)
            {
              memcpy(samp, buf, bytes_per_sample);
              buf += (bytes_per_sample + padding);
              samp += bytes_per_sample;
              data_copied += bytes_per_sample;
            }
        }
      else
        {
          memcpy(samp, buf, samp_size - data_copied);
          buf += samp_size - data_copied;
          samp += samp_size - data_copied;
          data_copied += samp_size - data_copied;
        }

      /* Calculate the audio buffer size from the internal buffer size.
       * The internal I2S buffer size is calculated from I2S_RXEOF_NUM_REG,
       * which returns the size in number of words.
       */

      bfcontainer->apb->nbytes =
                  (getreg32(I2S_RXEOF_NUM_REG(priv->config->port)) * 4);

      /* The internal buffer size must be divided by either 16 or 32 bits
       * as the I2S peripheral aligns each sample to one of these boundaries.
       * The result represents the number of samples on the internal buffer.
       */

      bfcontainer->apb->nbytes /=
                  ((priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT ?
                   I2S_DATA_BIT_WIDTH_16BIT : I2S_DATA_BIT_WIDTH_32BIT) / 8);

      /* Finally, calculate the number of bytes on the audio buffer */

      bfcontainer->apb->nbytes *= bytes_per_sample;

      DEBUGASSERT(bfcontainer->apb->nbytes == data_copied);

      /* Perform the RX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release the internal buffer used by the DMA inlink */

      free(bfcontainer->buf);

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

static void i2s_configure(struct esp32_i2s_s *priv)
{
  /* Set peripheral clock and clear reset */

  if (priv->config->port == ESP32_I2S0)
    {
      modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, DPORT_I2S0_CLK_EN);
      modifyreg32(DPORT_PERIP_RST_EN_REG, 0, DPORT_I2S0_RST);
      modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST, 0);
    }
  else
    {
      modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, DPORT_I2S1_CLK_EN);
      modifyreg32(DPORT_PERIP_RST_EN_REG, 0, DPORT_I2S1_RST);
      modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_I2S1_RST, 0);
    }

  /* I2S module general init, enable I2S clock */

  if (!(getreg32(I2S_CLKM_CONF_REG(priv->config->port)) & I2S_CLK_ENA))
    {
      i2sinfo("Enabling I2S port clock...\n");
      modifyreg32(I2S_CLKM_CONF_REG(priv->config->port), 0, I2S_CLK_ENA);
      putreg32(0, I2S_CONF2_REG(priv->config->port));
    }

  /* Configure multiplexed pins as connected on the board */

  /* TODO: check for loopback mode */

  /* Enable TX channel */

  if (priv->config->dout_pin != I2S_GPIO_UNUSED)
    {
      esp32_gpiowrite(priv->config->dout_pin, 1);
      esp32_configgpio(priv->config->dout_pin, OUTPUT_FUNCTION_3);
      esp32_gpio_matrix_out(priv->config->dout_pin,
                            priv->config->dout_outsig, 0, 0);
    }

  /* Enable RX channel */

  if (priv->config->din_pin != I2S_GPIO_UNUSED)
    {
      esp32_configgpio(priv->config->din_pin, INPUT_FUNCTION_3);
      esp32_gpio_matrix_in(priv->config->din_pin,
                           priv->config->din_insig, 0);
    }

  if (priv->config->role == I2S_ROLE_SLAVE)
    {
      if (priv->config->tx_en && !priv->config->rx_en)
        {
          /* For "tx + slave" mode, select TX signal index for ws and bck */

          esp32_gpiowrite(priv->config->ws_pin, 1);
          esp32_configgpio(priv->config->ws_pin, INPUT_FUNCTION_3);
          esp32_gpio_matrix_in(priv->config->ws_pin,
                               priv->config->ws_out_insig, 0);

          esp32_gpiowrite(priv->config->bclk_pin, 1);
          esp32_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_3);
          esp32_gpio_matrix_in(priv->config->bclk_pin,
                               priv->config->bclk_out_insig, 0);
        }
      else
        {
          /* For "tx + rx + slave" or "rx + slave" mode, select RX signal
           * index for ws and bck.
           */

          esp32_gpiowrite(priv->config->ws_pin, 1);
          esp32_configgpio(priv->config->ws_pin, INPUT_FUNCTION_3);
          esp32_gpio_matrix_in(priv->config->ws_pin,
                               priv->config->ws_in_insig, 0);

          esp32_gpiowrite(priv->config->bclk_pin, 1);
          esp32_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_3);
          esp32_gpio_matrix_in(priv->config->bclk_pin,
                               priv->config->bclk_in_insig, 0);
        }
    }
  else
    {
      /* Considering master role for the I2S port */

      /* Set MCLK pin */

      if (priv->config->mclk_pin != I2S_GPIO_UNUSED)
        {
          bool is_i2s0 = priv->config->port == ESP32_I2S0 ? true : false;

          i2sinfo("Configuring GPIO%" PRIu8 " to output master clock\n",
                  priv->config->mclk_pin);

          esp32_configgpio(priv->config->mclk_pin, OUTPUT_FUNCTION_2);
          esp32_gpio_matrix_out(priv->config->mclk_pin,
                                SIG_GPIO_OUT_IDX, 0, 0);

          if (priv->config->mclk_pin == 0)
            {
              putreg32(priv->config->is_apll ?
                       0xfff6 : (is_i2s0 ? 0xfff0 : 0xffff), PIN_CTRL);
            }
          else if (priv->config->mclk_pin == 1)
            {
              putreg32(priv->config->is_apll ?
                       0xf6f6 : (is_i2s0 ? 0xf0f0 : 0xf0ff), PIN_CTRL);
            }
          else
            {
              putreg32(priv->config->is_apll ?
                       0xff66 : (is_i2s0 ? 0xff00 : 0xff0f), PIN_CTRL);
            }
        }

      if (priv->config->rx_en && !priv->config->tx_en)
        {
          /* For "rx + master" mode, select RX signal index for ws and bck */

          esp32_gpiowrite(priv->config->ws_pin, 1);
          esp32_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_3);
          esp32_gpio_matrix_out(priv->config->ws_pin,
                                priv->config->ws_in_outsig, 0, 0);

          esp32_gpiowrite(priv->config->bclk_pin, 1);
          esp32_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_3);
          esp32_gpio_matrix_out(priv->config->bclk_pin,
                                priv->config->bclk_in_outsig, 0, 0);
        }
      else
        {
          /* For "tx + rx + master" or "tx + master" mode, select TX signal
           * index for ws and bck.
           */

          esp32_gpiowrite(priv->config->ws_pin, 1);
          esp32_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_3);
          esp32_gpio_matrix_out(priv->config->ws_pin,
                                priv->config->ws_out_outsig, 0, 0);

          esp32_gpiowrite(priv->config->bclk_pin, 1);
          esp32_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_3);
          esp32_gpio_matrix_out(priv->config->bclk_pin,
                                priv->config->bclk_out_outsig, 0, 0);
        }
    }

  /* Share BCLK and WS if in full-duplex mode */

  if (priv->config->tx_en && priv->config->rx_en)
    {
      modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_SIG_LOOPBACK);
    }
  else
    {
      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_SIG_LOOPBACK, 0);
    }

  /* Configure the TX module */

  if (priv->config->tx_en)
    {
      /* Reset I2S TX module */

      modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_TX_RESET);
      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_TX_RESET, 0);
      modifyreg32(I2S_LC_CONF_REG(priv->config->port), 0, I2S_OUT_RST);
      modifyreg32(I2S_LC_CONF_REG(priv->config->port), I2S_OUT_RST, 0);

      /* Enable/disable I2S TX slave mode */

      if (priv->config->role == I2S_ROLE_SLAVE)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_TX_SLAVE_MOD);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), I2S_TX_SLAVE_MOD, 0);
        }

      /* Configure TX chan bit, audio data bit and mono mode.
       * On ESP32, sample_bit should equals to data_bit.
       */

      /* Set TX data width */

      priv->data_width = priv->config->data_width;
      i2s_set_datawidth(priv);

      /* Set I2S tx chan mode */

      modifyreg32(I2S_CONF_CHAN_REG(priv->config->port), I2S_TX_CHAN_MOD_M,
                  FIELD_TO_VALUE(I2S_TX_CHAN_MOD, priv->config->mono_en ?
                  4 : 0));

      /* Enable/disable TX MSB shift, the data will be launch at the first
       * BCK clock.
       */

      if (priv->config->bit_shift)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_TX_MSB_SHIFT);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), I2S_TX_MSB_SHIFT, 0);
        }

      /* Configure TX WS signal width. Set to to enable transmitter in PCM
       * standard mode.
       */

      if (priv->config->ws_width == 1)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0,
                      I2S_TX_SHORT_SYNC);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port),
                      I2S_TX_SHORT_SYNC, 0);
        }

      /* Set I2S tx right channel first */

      if (priv->config->ws_pol == 1)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0,
                      I2S_TX_RIGHT_FIRST);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port),
                      I2S_TX_RIGHT_FIRST, 0);
        }

      /* I2S tx fifo module force enable */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), 0,
                  I2S_TX_FIFO_MOD_FORCE_EN);

      /* The default value for the master clock frequency (MCLK frequency)
       * can be set from the sample rate multiplied by a fixed value, known
       * as MCLK multiplier. This multiplier, however, should be divisible
       * by the number of bytes from a sample, i.e, for 24 bits, the
       * multiplier should be divisible by 3. NOTE: the MCLK frequency can
       * be adjusted on runtime, so this value remains valid only if the
       * upper half does not implement the `i2s_mclkfrequency` method.
       */

      if (priv->config->data_width == I2S_DATA_BIT_WIDTH_24BIT)
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_384;
        }
      else
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_256;
        }

      esp32_i2s_mclkfrequency((struct i2s_dev_s *)priv, (priv->config->rate *
                              priv->mclk_multiple));

      priv->rate = priv->config->rate;
      i2s_set_clock(priv);
    }

  /* Configure the RX module */

  if (priv->config->rx_en)
    {
      /* Reset I2S RX module */

      modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_RX_RESET);
      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_RX_RESET, 0);
      modifyreg32(I2S_LC_CONF_REG(priv->config->port), 0, I2S_IN_RST);
      modifyreg32(I2S_LC_CONF_REG(priv->config->port), I2S_IN_RST, 0);

      /* Enable/disable I2S RX slave mode */

      if (priv->config->role == I2S_ROLE_SLAVE)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_RX_SLAVE_MOD);
        }
      else
        {
          /* Since BCLK and WS are shared, only TX or RX can be master. In
           * this case, force RX as slave to avoid conflict of clock signal.
           */

          if (priv->config->tx_en)
            {
              modifyreg32(I2S_CONF_REG(priv->config->port), 0,
                          I2S_RX_SLAVE_MOD);
            }
          else
            {
              modifyreg32(I2S_CONF_REG(priv->config->port),
                          I2S_RX_SLAVE_MOD, 0);
            }
        }

      /* Congfigure RX chan bit, audio data bit and mono mode.
       * On ESP32, sample_bit should equals to data_bit.
       */

      /* Set RX data width */

      priv->data_width = priv->config->data_width;
      i2s_set_datawidth(priv);

      /* Set I2S RX chan mode */

      modifyreg32(I2S_CONF_CHAN_REG(priv->config->port),
                  I2S_RX_CHAN_MOD_M, 0);

      /* Enable/disable RX MSB shift, the data will be read at the first
       * BCK clock.
       */

      if (priv->config->bit_shift)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_RX_MSB_SHIFT);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), I2S_RX_MSB_SHIFT, 0);
        }

      /* Configure RX WS signal width. Set to to enable receiver in PCM
       * standard mode.
       */

      if (priv->config->ws_width == 1)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0,
                      I2S_RX_SHORT_SYNC);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port),
                      I2S_RX_SHORT_SYNC, 0);
        }

      /* Set I2S RX right channel first */

      if (priv->config->ws_pol == 1)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0,
                      I2S_RX_RIGHT_FIRST);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port),
                      I2S_RX_RIGHT_FIRST, 0);
        }

      /* I2S RX fifo module force enable */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), 0,
                  I2S_RX_FIFO_MOD_FORCE_EN);

      /* The default value for the master clock frequency (MCLK frequency)
       * can be set from the sample rate multiplied by a fixed value, known
       * as MCLK multiplier. This multiplier, however, should be divisible
       * by the number of bytes from a sample, i.e, for 24 bits, the
       * multiplier should be divisible by 3. NOTE: the MCLK frequency can
       * be adjusted on runtime, so this value remains valid only if the
       * upper half does not implement the `i2s_mclkfrequency` method.
       */

      if (priv->config->data_width == I2S_DATA_BIT_WIDTH_24BIT)
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_384;
        }
      else
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_256;
        }

      esp32_i2s_mclkfrequency((struct i2s_dev_s *)priv, (priv->config->rate *
                              priv->mclk_multiple));

      priv->rate = priv->config->rate;
      i2s_set_clock(priv);
    }
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

static uint32_t i2s_set_datawidth(struct esp32_i2s_s *priv)
{
#ifdef I2S_HAVE_TX
  if (priv->config->tx_en)
    {
      modifyreg32(I2S_SAMPLE_RATE_CONF_REG(priv->config->port),
                  I2S_TX_BITS_MOD_M, FIELD_TO_VALUE(I2S_TX_BITS_MOD,
                  priv->data_width));

      /* Set TX FIFO operation mode */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), I2S_TX_FIFO_MOD_M,
                  priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT ?
                  FIELD_TO_VALUE(I2S_TX_FIFO_MOD,
                                 0 + priv->config->mono_en) :
                  FIELD_TO_VALUE(I2S_TX_FIFO_MOD,
                                 2 + priv->config->mono_en));

      /* I2S TX MSB right enable */

      if (priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_TX_MSB_RIGHT);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), I2S_TX_MSB_RIGHT, 0);
        }
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  if (priv->config->rx_en)
    {
      modifyreg32(I2S_SAMPLE_RATE_CONF_REG(priv->config->port),
                  I2S_RX_BITS_MOD_M, FIELD_TO_VALUE(I2S_RX_BITS_MOD,
                  priv->data_width));

      /* Set RX FIFO operation mode */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), I2S_RX_FIFO_MOD_M,
                  priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT ?
                  FIELD_TO_VALUE(I2S_RX_FIFO_MOD,
                                 0 + priv->config->mono_en) :
                  FIELD_TO_VALUE(I2S_RX_FIFO_MOD,
                                 2 + priv->config->mono_en));

      /* I2S RX MSB right enable */

      if (priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT)
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_RX_MSB_RIGHT);
        }
      else
        {
          modifyreg32(I2S_CONF_REG(priv->config->port), I2S_RX_MSB_RIGHT, 0);
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

static uint32_t i2s_set_clock(struct esp32_i2s_s *priv)
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

  /* TODO: provide APLL clock support */

  /* Disable APLL clock, I2S module will using PLL_D2_CLK(160M) as source
   * clock.
   */

  modifyreg32(I2S_CLKM_CONF_REG(priv->config->port), I2S_CLKA_ENA, 0);
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

  regval = getreg32(I2S_CLKM_CONF_REG(priv->config->port));
  regval &= ~I2S_CLKM_DIV_NUM_M;
  regval |= FIELD_TO_VALUE(I2S_CLKM_DIV_NUM, mclk_div);
  regval &= ~I2S_CLKM_DIV_B_M;
  regval |= FIELD_TO_VALUE(I2S_CLKM_DIV_B, numerator);
  regval &= ~I2S_CLKM_DIV_A_M;
  regval |= FIELD_TO_VALUE(I2S_CLKM_DIV_A, denominator);
  putreg32(regval, I2S_CLKM_CONF_REG(priv->config->port));

  /* Set I2S TX bck div num */

#ifdef I2S_HAVE_TX
  modifyreg32(I2S_SAMPLE_RATE_CONF_REG(priv->config->port),
              I2S_TX_BCK_DIV_NUM_M,
              FIELD_TO_VALUE(I2S_TX_BCK_DIV_NUM, bclk_div));
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  modifyreg32(I2S_SAMPLE_RATE_CONF_REG(priv->config->port),
              I2S_RX_BCK_DIV_NUM_M,
              FIELD_TO_VALUE(I2S_RX_BCK_DIV_NUM, bclk_div));
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
static void i2s_tx_channel_start(struct esp32_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      if (priv->tx_started)
        {
          i2swarn("TX channel of port %d was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the TX channel */

      modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_TX_RESET);
      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_TX_RESET, 0);

      /* Reset the DMA operation */

      modifyreg32(I2S_LC_CONF_REG(priv->config->port), 0, I2S_OUT_RST);
      modifyreg32(I2S_LC_CONF_REG(priv->config->port), I2S_OUT_RST, 0);

      /* Reset TX FIFO */

      modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_TX_FIFO_RESET);
      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_TX_FIFO_RESET, 0);

      /* Enable DMA interrupt */

      up_enable_irq(priv->config->irq);

      modifyreg32(I2S_INT_ENA_REG(priv->config->port), 0,
                  I2S_OUT_EOF_INT_ENA);

      /* Enable DMA operation mode */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), 0, I2S_DSCR_EN);

      /* Unset the DMA outlink */

      putreg32(0, I2S_OUT_LINK_REG(priv->config->port));

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
static void i2s_rx_channel_start(struct esp32_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      if (priv->rx_started)
        {
          i2swarn("RX channel of port %d was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the RX channel */

      modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_RX_RESET);
      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_RX_RESET, 0);

      /* Reset the DMA operation */

      modifyreg32(I2S_LC_CONF_REG(priv->config->port), 0, I2S_IN_RST);
      modifyreg32(I2S_LC_CONF_REG(priv->config->port), I2S_IN_RST, 0);

      /* Reset RX FIFO */

      modifyreg32(I2S_CONF_REG(priv->config->port), 0, I2S_RX_FIFO_RESET);
      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_RX_FIFO_RESET, 0);

      /* Enable DMA interrupt */

      up_enable_irq(priv->config->irq);

      modifyreg32(I2S_INT_ENA_REG(priv->config->port), 0,
                  I2S_IN_SUC_EOF_INT_ENA);

      /* Enable DMA operation mode */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), 0, I2S_DSCR_EN);

      /* Unset the DMA outlink */

      putreg32(0, I2S_IN_LINK_REG(priv->config->port));

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
static void i2s_tx_channel_stop(struct esp32_i2s_s *priv)
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

      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_TX_START, 0);

      /* Stop outlink */

      modifyreg32(I2S_OUT_LINK_REG(priv->config->port), I2S_OUTLINK_START,
                  I2S_OUTLINK_STOP);

      /* Disable DMA interrupt */

      modifyreg32(I2S_INT_ENA_REG(priv->config->port),
                  I2S_OUT_EOF_INT_ENA, 0);

      /* Disable DMA operation mode */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), I2S_DSCR_EN, 0);

      up_disable_irq(priv->config->irq);

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
static void i2s_rx_channel_stop(struct esp32_i2s_s *priv)
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

      modifyreg32(I2S_CONF_REG(priv->config->port), I2S_RX_START, 0);

      /* Stop outlink */

      modifyreg32(I2S_IN_LINK_REG(priv->config->port), I2S_INLINK_START,
                  I2S_INLINK_STOP);

      /* Disable DMA interrupt */

      modifyreg32(I2S_INT_ENA_REG(priv->config->port),
                  I2S_IN_SUC_EOF_INT_ENA, 0);

      /* Disable DMA operation mode */

      modifyreg32(I2S_FIFO_CONF_REG(priv->config->port), I2S_DSCR_EN, 0);

      up_disable_irq(priv->config->irq);

      priv->rx_started = false;

      i2sinfo("Stopped RX channel of port %d\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: esp32_i2s_interrupt
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

static int esp32_i2s_interrupt(int irq, void *context, void *arg)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)arg;
  struct esp32_dmadesc_s *cur = NULL;

  uint32_t status = getreg32(I2S_INT_ST_REG(priv->config->port));

  putreg32(UINT32_MAX, I2S_INT_CLR_REG(priv->config->port));

#ifdef I2S_HAVE_TX
  if (priv->config->tx_en)
    {
      if (status & I2S_OUT_EOF_INT_ST)
        {
          cur = (struct esp32_dmadesc_s *)
                getreg32(I2S_OUT_EOF_DES_ADDR_REG(priv->config->port));

          /* Schedule completion of the transfer on the worker thread */

          i2s_tx_schedule(priv, cur);
        }
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  if (priv->config->rx_en)
    {
      if (status & I2S_IN_SUC_EOF_INT_ST)
        {
          cur = (struct esp32_dmadesc_s *)
                getreg32(I2S_IN_EOF_DES_ADDR_REG(priv->config->port));

          /* Schedule completion of the transfer on the worker thread */

          i2s_rx_schedule(priv, cur);
        }
    }
#endif /* I2S_HAVE_RX */

  return 0;
}

/****************************************************************************
 * Name: esp32_i2s_mclkfrequency
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

static uint32_t esp32_i2s_mclkfrequency(struct i2s_dev_s *dev,
                                        uint32_t frequency)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

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
 * Name: esp32_i2s_txchannels
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
static int esp32_i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

  if (priv->config->tx_en)
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
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: esp32_i2s_rxchannels
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
static int esp32_i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

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
 * Name: esp32_i2s_txsamplerate
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
static uint32_t esp32_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      i2s_tx_channel_stop(priv);

      priv->rate = rate;

      rate = i2s_set_clock(priv);

      i2s_tx_channel_start(priv);

      return rate;
    }

  return 0;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: esp32_i2s_rxsamplerate
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
static uint32_t esp32_i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      i2s_rx_channel_stop(priv);

      priv->rate = rate;

      rate = i2s_set_clock(priv);

      i2s_rx_channel_start(priv);

      return rate;
    }

  return 0;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: esp32_i2s_txdatawidth
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
static uint32_t esp32_i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

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
 * Name: esp32_i2s_rxdatawidth
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
static uint32_t esp32_i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

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
 * Name: esp32_i2s_send
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
static int esp32_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                          i2s_callback_t callback, void *arg,
                          uint32_t timeout)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      struct esp32_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check audio buffer data size */

      nbytes = (apb->nbytes - apb->curbyte) + priv->tx.carry.bytes;

      /* If data width is 8, it is necessary to use a word of 16 bits;
       * If data width is 24, it is necessary to use a word of 32 bits;
       */

      nsamp = nbytes / (priv->data_width / 8);
      nbytes = nsamp *
              ((priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT ?
                I2S_DATA_BIT_WIDTH_16BIT : I2S_DATA_BIT_WIDTH_32BIT) / 8);

      /* ESP32's I2S peripheral always consider two channels. If the source
       * is mono, it is necessary to copy zero-ed data between each sample.
       */

      nbytes *= priv->channels == 1 ? 2 : 1;

      /* Ensure nbytes is 4-byte aligned */

      nbytes = ALIGN_UP(nbytes, sizeof(uintptr_t));

      if (nbytes > (ESP32_DMA_DATALEN_MAX * I2S_DMADESC_NUM))
        {
          i2serr("Required buffer size can't fit into DMA outlink "
                "(exceeds in %" PRIu32 " bytes). Try to increase the "
                "number of the DMA descriptors (CONFIG_I2S_DMADESC_NUM).",
                nbytes - (ESP32_DMA_DATALEN_MAX * I2S_DMADESC_NUM));
          return -EFBIG;
        }

      /* Allocate a buffer container in advance */

      bfcontainer = i2s_buf_allocate(priv);
      DEBUGASSERT(bfcontainer);

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
 * Name: esp32_i2s_receive
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
static int esp32_i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                             i2s_callback_t callback, void *arg,
                             uint32_t timeout)
{
  struct esp32_i2s_s *priv = (struct esp32_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      struct esp32_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check max audio buffer data size */

      nbytes = apb->nmaxbytes;

      /* If data width is 8, it is necessary to use a word of 16 bits;
       * If data width is 24, it is necessary to use a word of 32 bits;
       */

      nsamp = nbytes / (priv->data_width / 8);
      nbytes = nsamp *
              ((priv->data_width <= I2S_DATA_BIT_WIDTH_16BIT ?
                I2S_DATA_BIT_WIDTH_16BIT : I2S_DATA_BIT_WIDTH_32BIT) / 8);

      if (nbytes > (ESP32_DMA_DATALEN_MAX * I2S_DMADESC_NUM))
        {
          i2serr("Required buffer size can't fit into DMA inlink "
                "(exceeds in %" PRIu32 " bytes). Try to increase the "
                "number of the DMA descriptors (CONFIG_I2S_DMADESC_NUM).",
                nbytes - (ESP32_DMA_DATALEN_MAX * I2S_DMADESC_NUM));
          return -EFBIG;
        }

      /* Allocate a buffer container in advance */

      bfcontainer = i2s_buf_allocate(priv);
      DEBUGASSERT(bfcontainer);

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
 * Name: esp32_i2s_ioctl
 *
 * Description:
 *   Perform a device ioctl
 *
 ****************************************************************************/

static int esp32_i2s_ioctl(struct i2s_dev_s *dev, int cmd,
                            unsigned long arg)
{
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

          ret = OK;
        }
        break;

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

static int i2s_dma_setup(struct esp32_i2s_s *priv)
{
  int ret;

  /* Clear the interrupts */

  putreg32(UINT32_MAX, I2S_INT_CLR_REG(priv->config->port));

  /* Set up to receive peripheral interrupts on the current CPU */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32_setup_irq(priv->cpu, priv->config->periph,
                                 1, ESP32_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      i2serr("Failed to allocate a CPU interrupt.\n");
      return priv->cpuint;
    }

  ret = irq_attach(priv->config->irq, esp32_i2s_interrupt, priv);
  if (ret != OK)
    {
      i2serr("Couldn't attach IRQ to handler.\n");
      esp32_teardown_irq(priv->cpu,
                         priv->config->periph,
                         priv->cpuint);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_i2sbus_initialize
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

struct i2s_dev_s *esp32_i2sbus_initialize(int port)
{
  int ret;
  struct esp32_i2s_s *priv = NULL;
  irqstate_t flags;

  i2sinfo("port: %d\n", port);

  /* Statically allocated I2S' device strucuture */

  switch (port)
    {
#ifdef CONFIG_ESP32_I2S0
      case ESP32_I2S0:
        priv = &esp32_i2s0_priv;
        break;
#endif
#ifdef CONFIG_ESP32_I2S1
      case ESP32_I2S1:
        priv = &esp32_i2s1_priv;
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

#endif /* CONFIG_ESP32_I2S */
