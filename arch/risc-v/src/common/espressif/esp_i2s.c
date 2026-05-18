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
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>

#include "esp_i2s.h"

#include "esp_attr.h"
#include "esp_cache.h"
#include "hal/dma_types.h"

#include "i2s_private.h"
#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#if SOC_I2S_SUPPORTS_TDM
#  include "driver/i2s_tdm.h"
#endif
#if SOC_I2S_SUPPORTS_PDM_TX || SOC_I2S_SUPPORTS_PDM_RX
#  include "driver/i2s_pdm.h"
#endif

#ifndef CONFIG_ESPRESSIF_I2S_DMA_DESC_NUM
#  define CONFIG_ESPRESSIF_I2S_DMA_DESC_NUM  6
#endif

#ifndef CONFIG_ESPRESSIF_I2S_DMA_FRAME_NUM
#  define CONFIG_ESPRESSIF_I2S_DMA_FRAME_NUM  240
#endif

/* Defer channel disable so the next i2s_send/i2s_receive can cancel it. */

#define ESP_I2S_IDLE_SHUTDOWN_DELAY  0

/* Stack size for the I/O thread that runs blocking HAL read/write. */

#ifdef CONFIG_ESPRESSIF_I2S_ASYNC_THREAD_STACKSIZE
#  define ESP_I2S_ASYNC_STACKSIZE CONFIG_ESPRESSIF_I2S_ASYNC_THREAD_STACKSIZE
#else
#  define ESP_I2S_ASYNC_STACKSIZE 3072
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

#if SOC_I2S_SUPPORTS_TDM
#  define I2S_TDM_AUTO_SLOT_NUM    (0)
#  define I2S_TDM_AUTO_WS_WIDTH    (0)
#  define I2S_TDM_AUTO_SLOT        (I2S_TDM_SLOT0 | I2S_TDM_SLOT1)
#endif

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#  define I2S_DMA_BUFFER_MAX_SIZE   DMA_DESCRIPTOR_BUFFER_MAX_SIZE_64B_ALIGNED
#else
#  define I2S_DMA_BUFFER_MAX_SIZE   DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED
#endif

/* Partial frame bytes held between successive i2s_send calls */

#define ESP_I2S_TX_CARRY_MAX  24

/****************************************************************************
 * Private Types
 ****************************************************************************/

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
  uint32_t port;              /* I2S port */
  uint32_t role;              /* I2S port role (master or slave) */
  uint8_t data_width;         /* I2S sample data width */
  uint32_t rate;              /* I2S sample-rate */
  uint32_t total_slot;        /* Total slot number */

  bool tx_en;                 /* Is TX enabled? */
  bool rx_en;                 /* Is RX enabled? */
  int8_t mclk_pin;            /* MCLK pin, output */

  int tx_clk_src;             /* Select the I2S TX source clock */
  int rx_clk_src;             /* Select the I2S RX source clock */

  /* BCLK pin, input in slave role, output in master role */

  int8_t bclk_pin;

  /* WS pin, input in slave role, output in master role */

  int8_t ws_pin;

  int8_t dout_pin;            /* DATA pin, output */
  int8_t din_pin;             /* DATA pin, input */

  uint8_t audio_std_mode;     /* Audio standard (i2s_audio_mode_t) */

  /* WS signal polarity, set true to enable high level first */

  bool ws_pol;

#ifdef CONFIG_PM
  esp_pm_lock_handle_t pm_lock; /* Power management lock */
#endif
};

struct esp_buffer_carry_s
{
  uint8_t data[ESP_I2S_TX_CARRY_MAX];
  size_t  bytes;
};

struct esp_i2s_tx_send_prep_s
{
  uint8_t *xfer;
  size_t   send_len;
  size_t   preloaded;
  size_t   new_carry_len;
  uint8_t  new_carry[ESP_I2S_TX_CARRY_MAX];
};

struct esp_i2s_async_job_s
{
  sq_entry_t                    qe;
  struct ap_buffer_s           *apb;
  i2s_callback_t                cb;
  void                         *cbarg;
  uint32_t                      timeout;
  struct esp_i2s_tx_send_prep_s prep;
  bool                          prep_valid;
  size_t                        send_quota;
};

/* Async RX job (`i2s_receive`, drained by the I/O thread). */

struct esp_i2s_rx_async_job_s
{
  sq_entry_t          qe;
  struct ap_buffer_s *apb;
  i2s_callback_t      cb;
  void               *cbarg;
  uint32_t            timeout;
};

struct esp_i2s_burst_rx_helper_s
{
  struct esp_i2s_s   *priv;
  struct ap_buffer_s *apb;
  uint32_t            timeout;
  int                 result;
};

/* The state of the one I2S peripheral */

struct esp_i2s_s
{
  struct i2s_dev_s dev;         /* Externally visible I2S interface */
  mutex_t          lock;        /* Ensures mutually exclusive access */
  spinlock_t       slock;       /* Device specific lock. */

  /* Port configuration */

  const struct esp_i2s_config_s *config;

  uint32_t mclk_freq;           /* I2S actual master clock */
  uint32_t mclk_multiple;       /* Multiple of MCLK to sample rate */
  uint32_t channels;            /* Audio channels (1:mono or 2:stereo) */
  uint32_t rate;                /* I2S actual configured sample-rate */
  uint32_t data_width;          /* I2S actual configured data_width */

  i2s_chan_handle_t tx_handle;  /* TX handle */

  /* Partial frame tail between successive `i2s_send` calls */

  struct esp_buffer_carry_s tx_carry;

  volatile bool tx_started;     /* TX channel started (read from ISR) */

  i2s_chan_handle_t rx_handle;  /* RX handle */

  bool rx_started;              /* RX channel started */

  volatile bool rx_busy;        /* RX HAL read in progress on I/O thread */
  volatile bool tx_busy;        /* TX HAL write in progress on I/O thread */
           bool session_active; /* AUDIOIOC_START stream session */

  /* Async I/O: enqueue jobs guarded by `priv->lock`; the I/O thread
   * dequeues and runs blocking HAL read/write outside the mutex.
   */

  sq_queue_t tx_jobs;
  sq_queue_t rx_jobs;
  sem_t      io_sem;
  pthread_t  io_thread;
  bool       io_thread_created;

  /* TX `on_sent` tracking: count down DMA block completions for the current
   * send operation.  When the count reaches zero the ISR clears `tx_start`,
   * stops GDMA, and posts `tx_on_sent_done_sem`.  Streaming sessions queue
   * an immediate idle-shutdown for `i2s_channel_disable()`.  Per-APB sends
   * (no AUDIOIOC_START) also disable from the I/O thread after the semaphore
   * is posted.
   */

  size_t            tx_dma_buf_size;
  volatile uint32_t tx_on_sent_blocks_left;
  sem_t             tx_on_sent_done_sem;

  struct work_s tx_stop_work;
  struct work_s rx_stop_work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_ESPRESSIF_I2S_DUMPBUFFERS
#  define i2s_dump_buffer(m, b, s) lib_dumpbuffer(m, b, s)
#else
#  define i2s_dump_buffer(m, b, s)
#endif

/* I2S configuration and channel control */

static int i2s_configure(struct esp_i2s_s *priv);

static void esp_i2s_tx_try_idle_shutdown_locked(struct esp_i2s_s *priv);
static void esp_i2s_rx_try_idle_shutdown_locked(struct esp_i2s_s *priv);
static void esp_i2s_tx_try_idle_shutdown(struct esp_i2s_s *priv);
static void esp_i2s_rx_try_idle_shutdown(struct esp_i2s_s *priv);
static void esp_i2s_tx_idle_shutdown_arm(struct esp_i2s_s *priv);
static void esp_i2s_rx_idle_shutdown_arm(struct esp_i2s_s *priv);
static void esp_i2s_tx_idle_shutdown_disarm(struct esp_i2s_s *priv);
static void esp_i2s_rx_idle_shutdown_disarm(struct esp_i2s_s *priv);
static int  esp_i2s_tx_channel_enable_locked(struct esp_i2s_s *priv,
                                            bool with_rx);
static int  esp_i2s_rx_channel_start_locked(struct esp_i2s_s *priv);
static int  esp_i2s_tx_channel_start(struct esp_i2s_s *priv);
static int  esp_i2s_rx_channel_start(struct esp_i2s_s *priv);
static int  esp_i2s_channels_start(struct esp_i2s_s *priv);
static void esp_i2s_tx_disable_locked(struct esp_i2s_s *priv);
static void esp_i2s_rx_disable_locked(struct esp_i2s_s *priv);
static void esp_i2s_burst_channels_down_locked(struct esp_i2s_s *priv,
                                               bool with_rx);
static void esp_i2s_burst_reset_channels_locked(struct esp_i2s_s *priv,
                                                bool with_rx);
static bool esp_i2s_burst_full_duplex(struct esp_i2s_s *priv);
static bool esp_i2s_tx_in_flight_locked(struct esp_i2s_s *priv);

/* Async I/O */

static bool esp_i2s_io_run_tx_burst_job(struct esp_i2s_s *priv);
static bool esp_i2s_io_run_burst_full_duplex_paired(struct esp_i2s_s *priv);
static bool esp_i2s_io_run_rx_burst_job(struct esp_i2s_s *priv);
static bool esp_i2s_io_run_rx_job(struct esp_i2s_s *priv);
static void esp_i2s_io_dispatch(struct esp_i2s_s *priv);
static void esp_i2s_io_wakeup(struct esp_i2s_s *priv);
static void *esp_i2s_io_thread_entry(void *arg);
static int  esp_i2s_io_thread_create(struct esp_i2s_s *priv);
static int  esp_i2s_io_submit_job(struct esp_i2s_s *priv,
                                  sq_queue_t *queue, sq_entry_t *job,
                                  struct ap_buffer_s *apb, bool wake_io);
static void esp_i2s_io_invoke_cb(struct i2s_dev_s *dev,
                                 struct ap_buffer_s *apb,
                                 i2s_callback_t cb, void *arg,
                                 int result);
static void esp_i2s_wake_queued_streams(struct esp_i2s_s *priv);
static void esp_i2s_rx_finish_job(struct esp_i2s_s *priv);
static void esp_i2s_rx_jobs_drain_cancel(struct esp_i2s_s *priv,
                                          int result);
static void esp_i2s_jobs_drain_cancel(struct esp_i2s_s *priv,
                                        sq_queue_t *jobs,
                                        int result);

/* TX/RX buffer transfer */

static int  esp_i2s_check_io_apb(struct esp_i2s_s *priv,
                                 struct ap_buffer_s *apb);
static int  esp_i2s_apb_span_end_excl(struct ap_buffer_s *apb,
                                      apb_samp_t *span_end_excl_out);
static int  esp_i2s_tx_prep_apb(struct esp_i2s_s *priv,
                                struct ap_buffer_s *apb,
                                struct esp_i2s_tx_send_prep_s *prep);
static void esp_i2s_tx_prep_release(struct esp_i2s_tx_send_prep_s *prep);
static void esp_i2s_tx_apply_carry_locked(struct esp_i2s_s *priv,
       struct esp_i2s_tx_send_prep_s *prep);
static int  esp_i2s_tx_preload(struct esp_i2s_s *priv,
                               struct ap_buffer_s *apb,
                               struct esp_i2s_tx_send_prep_s *prep);
static int  esp_i2s_tx_run_job_locked(struct esp_i2s_s *priv,
                                      struct ap_buffer_s *apb,
                                      uint32_t timeout,
                                      struct esp_i2s_tx_send_prep_s *prep);
static int  esp_i2s_rx_run_job_locked(struct esp_i2s_s *priv,
                                      struct ap_buffer_s *apb,
                                      uint32_t timeout);
static int  esp_i2s_tx_wait_on_sent_done(struct esp_i2s_s *priv,
                                         uint32_t timeout);
static int  esp_i2s_tx_job_send_quota(struct esp_i2s_s *priv,
                                      struct ap_buffer_s *apb,
                                      struct esp_i2s_tx_send_prep_s *prep,
                                      bool prep_valid,
                                      size_t *quota_out);

/* TX event callbacks */

static bool IRAM_ATTR esp_i2s_tx_on_sent(i2s_chan_handle_t handle,
                                          i2s_event_data_t *event,
                                          void *user_ctx);
static int  esp_i2s_tx_register_event_callbacks(struct esp_i2s_s *priv);
static void esp_i2s_tx_on_sent_done_sem_drain(struct esp_i2s_s *priv);
static void esp_i2s_tx_on_sent_disable_arm(struct esp_i2s_s *priv,
                                            size_t send_len);

/* I2S lower-half methods */

static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev);
static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency);
static int      i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg);
static int      i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                         i2s_callback_t callback, void *arg,
                         uint32_t timeout);
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
  .i2s_txchannels   = i2s_txchannels,
  .i2s_txsamplerate = i2s_txsamplerate,
  .i2s_txdatawidth  = i2s_txdatawidth,
  .i2s_send         = i2s_send,

  .i2s_rxchannels   = i2s_rxchannels,
  .i2s_rxsamplerate = i2s_rxsamplerate,
  .i2s_rxdatawidth  = i2s_rxdatawidth,
  .i2s_receive      = i2s_receive,

  .i2s_ioctl            = i2s_ioctl,
  .i2s_getmclkfrequency = i2s_getmclkfrequency,
  .i2s_setmclkfrequency = i2s_setmclkfrequency,
};

#ifdef CONFIG_ESPRESSIF_I2S0

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
#ifdef CONFIG_PM
  .pm_lock          = NULL,
#endif
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
};
#endif /* CONFIG_ESPRESSIF_I2S0 */

/****************************************************************************
 * Private helpers - I2S channel setup
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_map_esp_err
 *
 * Description:
 *   Map an Espressif `esp_err_t` value to a NuttX errno-style return code.
 *
 * Input Parameters:
 *   err - Espressif HAL error code
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_map_esp_err(esp_err_t err)
{
  switch (err)
    {
    case ESP_OK:
      return OK;

    case ESP_ERR_NO_MEM:
      return -ENOMEM;

    case ESP_ERR_INVALID_ARG:
      return -EINVAL;

    default:
      return -EIO;
    }
}

/****************************************************************************
 * Name: i2s_configure_del_channels
 *
 * Description:
 *   Delete any I2S TX/RX channel handles held in `priv`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void i2s_configure_del_channels(struct esp_i2s_s *priv)
{
  if (priv->rx_handle)
    {
      i2s_del_channel(priv->rx_handle);
      priv->rx_handle = NULL;
    }

  if (priv->tx_handle)
    {
      i2s_del_channel(priv->tx_handle);
      priv->tx_handle = NULL;
    }
}

/****************************************************************************
 * Name: esp_i2s_tx_try_idle_shutdown_locked
 *
 * Description:
 *   Disable the TX channel when no AUDIOIOC_START session is active and no
 *   buffer is waiting to be transmitted.
 *
 *   Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_tx_try_idle_shutdown_locked(struct esp_i2s_s *priv)
{
  if (priv->session_active)
    {
      return;
    }

  if (esp_i2s_tx_in_flight_locked(priv))
    {
      return;
    }

  esp_i2s_tx_disable_locked(priv);
}

/****************************************************************************
 * Name: esp_i2s_rx_try_idle_shutdown_locked
 *
 * Description:
 *   Disable the RX channel when no AUDIOIOC_START session is active and no
 *   buffer is waiting to be received.
 *
 *   Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_rx_try_idle_shutdown_locked(struct esp_i2s_s *priv)
{
  if (priv->session_active)
    {
      return;
    }

  if (priv->rx_busy || !sq_empty(&priv->rx_jobs))
    {
      return;
    }

  esp_i2s_rx_disable_locked(priv);
}

/****************************************************************************
 * Name: esp_i2s_tx_try_idle_shutdown
 *
 * Description:
 *   Thread/work-context wrapper for `esp_i2s_tx_try_idle_shutdown_locked()`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_tx_try_idle_shutdown(struct esp_i2s_s *priv)
{
  nxmutex_lock(&priv->lock);
  esp_i2s_tx_try_idle_shutdown_locked(priv);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: esp_i2s_rx_try_idle_shutdown
 *
 * Description:
 *   Thread/work-context wrapper for `esp_i2s_rx_try_idle_shutdown_locked()`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_rx_try_idle_shutdown(struct esp_i2s_s *priv)
{
  nxmutex_lock(&priv->lock);
  esp_i2s_rx_try_idle_shutdown_locked(priv);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: esp_i2s_tx_idle_shutdown_worker
 *
 * Description:
 *   HPWORK callback: stop the TX channel when it is idle.
 *
 * Input Parameters:
 *   arg - I2S device structure cast from the work-queue argument
 *
 ****************************************************************************/

static void esp_i2s_tx_idle_shutdown_worker(FAR void *arg)
{
  esp_i2s_tx_try_idle_shutdown((struct esp_i2s_s *)arg);
}

/****************************************************************************
 * Name: esp_i2s_rx_idle_shutdown_worker
 *
 * Description:
 *   HPWORK callback: stop the RX channel when it is idle.
 *
 * Input Parameters:
 *   arg - I2S device structure cast from the work-queue argument
 *
 ****************************************************************************/

static void esp_i2s_rx_idle_shutdown_worker(FAR void *arg)
{
  esp_i2s_rx_try_idle_shutdown((struct esp_i2s_s *)arg);
}

/****************************************************************************
 * Name: esp_i2s_tx_idle_shutdown_disarm
 *
 * Description:
 *   Cancel a pending TX idle-shutdown.  Called when a new send starts.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_tx_idle_shutdown_disarm(struct esp_i2s_s *priv)
{
  work_cancel(HPWORK, &priv->tx_stop_work);
}

/****************************************************************************
 * Name: esp_i2s_rx_idle_shutdown_disarm
 *
 * Description:
 *   Cancel a pending RX idle-shutdown.  Called when a new receive starts.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_rx_idle_shutdown_disarm(struct esp_i2s_s *priv)
{
  work_cancel(HPWORK, &priv->rx_stop_work);
}

/****************************************************************************
 * Name: esp_i2s_tx_idle_shutdown_arm
 *
 * Description:
 *   Schedule a deferred TX idle-shutdown check.  A subsequent `i2s_send()`
 *   cancels this so multi-buffer TX streams stay clocked.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_tx_idle_shutdown_arm(struct esp_i2s_s *priv)
{
  work_cancel(HPWORK, &priv->tx_stop_work);
  work_queue(HPWORK, &priv->tx_stop_work, esp_i2s_tx_idle_shutdown_worker,
             priv, ESP_I2S_IDLE_SHUTDOWN_DELAY);
}

/****************************************************************************
 * Name: esp_i2s_rx_idle_shutdown_arm
 *
 * Description:
 *   Schedule a deferred RX idle-shutdown check.  A subsequent
 *   `i2s_receive()` cancels this so multi-buffer RX streams stay clocked.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_rx_idle_shutdown_arm(struct esp_i2s_s *priv)
{
  work_cancel(HPWORK, &priv->rx_stop_work);
  work_queue(HPWORK, &priv->rx_stop_work, esp_i2s_rx_idle_shutdown_worker,
             priv, ESP_I2S_IDLE_SHUTDOWN_DELAY);
}

/****************************************************************************
 * Name: esp_i2s_tx_on_sent_done_sem_drain
 *
 * Description:
 *   Consume any posts left on `tx_on_sent_done_sem` from a prior send.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_tx_on_sent_done_sem_drain(struct esp_i2s_s *priv)
{
  while (nxsem_trywait(&priv->tx_on_sent_done_sem) == OK)
    {
      /* Discard stale completion posts. */
    }
}

/****************************************************************************
 * Name: esp_i2s_tx_on_sent_disable_arm
 *
 * Description:
 *   Arm TX stop after `ceil(send_len / tx_dma_buf_size)` `on_sent` events.
 *   Caller must hold `priv->lock`.  Must run before `i2s_channel_enable()`.
 *
 * Input Parameters:
 *   priv     - I2S device structure
 *   send_len - Number of bytes to be sent in the current operation
 *
 *
 ****************************************************************************/

static void esp_i2s_tx_on_sent_disable_arm(struct esp_i2s_s *priv,
                                           size_t send_len)
{
  size_t   blk;
  uint32_t blocks;

  esp_i2s_tx_on_sent_done_sem_drain(priv);

  if (send_len == 0 || priv->tx_dma_buf_size == 0)
    {
      priv->tx_on_sent_blocks_left = 0;
      return;
    }

  blk = priv->tx_dma_buf_size;
  blocks = (uint32_t)((send_len + blk - 1) / blk);
  priv->tx_on_sent_blocks_left = blocks;
}

/****************************************************************************
 * Name: esp_i2s_tx_job_send_quota
 *
 * Description:
 *   Compute the number of bytes that will be sent for one TX job, using an
 *   existing prep when valid or building a temporary one from `apb`.
 *
 * Input Parameters:
 *   priv       - I2S device structure
 *   apb        - Audio buffer for the transfer
 *   prep       - Optional prior TX prep result
 *   prep_valid - True when `prep` is populated
 *
 * Output Parameters:
 *   quota_out - Receives the send length in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_job_send_quota(struct esp_i2s_s *priv,
                                     struct ap_buffer_s *apb,
                                     struct esp_i2s_tx_send_prep_s *prep,
                                     bool prep_valid,
                                     size_t *quota_out)
{
  struct esp_i2s_tx_send_prep_s local;
  int                           ret;

  if (quota_out == NULL)
    {
      return -EINVAL;
    }

  if (prep_valid && prep != NULL)
    {
      *quota_out = prep->send_len;
      return OK;
    }

  ret = esp_i2s_tx_prep_apb(priv, apb, &local);
  if (ret < OK)
    {
      return ret;
    }

  *quota_out = local.send_len;
  esp_i2s_tx_prep_release(&local);
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_tx_on_sent
 *
 * Description:
 *   TX DMA EOF callback registered via
 *   `i2s_channel_register_event_callback`.
 *
 * Input Parameters:
 *   handle   - TX channel handle
 *   event    - I2S event data from the HAL
 *   user_ctx - I2S device structure (`priv`)
 *
 * Returned Value:
 *   Always false (do not yield from the ISR callback).
 *
 ****************************************************************************/

static bool IRAM_ATTR esp_i2s_tx_on_sent(i2s_chan_handle_t handle,
                                               i2s_event_data_t *event,
                                               void *user_ctx)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)user_ctx;
  uint32_t          left;

  if (priv == NULL || event == NULL || !priv->tx_started ||
      priv->tx_on_sent_blocks_left == 0)
    {
      return false;
    }

  left = --priv->tx_on_sent_blocks_left;

  if (left == 0)
    {
      nxsem_post(&priv->tx_on_sent_done_sem);

      if (priv->session_active)
        {
          esp_i2s_tx_idle_shutdown_arm(priv);
        }
    }

  return false;
}

/****************************************************************************
 * Name: esp_i2s_tx_register_event_callbacks
 *
 * Description:
 *   Register TX event callbacks before the channel is enabled.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_register_event_callbacks(struct esp_i2s_s *priv)
{
  i2s_event_callbacks_t cbs;
  esp_err_t             err;

  if (!priv->config->tx_en || priv->tx_handle == NULL)
    {
      return OK;
    }

  memset(&cbs, 0, sizeof(cbs));
  cbs.on_sent = esp_i2s_tx_on_sent;

  err = i2s_channel_register_event_callback(priv->tx_handle, &cbs, priv);
  if (err != ESP_OK)
    {
      i2serr("I2S: register TX event callback failed: %d\n",
             i2s_map_esp_err(err));
      return i2s_map_esp_err(err);
    }

  priv->tx_on_sent_blocks_left = 0;
  priv->tx_dma_buf_size = priv->tx_handle->dma.buf_size;
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_burst_full_duplex
 *
 * Description:
 *   True during per-APB burst I/O when TX and RX are both configured and no
 *   AUDIOIOC_START session is active.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   True when burst full-duplex I/O is active.
 *
 ****************************************************************************/

static bool esp_i2s_burst_full_duplex(struct esp_i2s_s *priv)
{
  return !priv->session_active &&
         priv->config->tx_en && priv->config->rx_en;
}

/****************************************************************************
 * Name: esp_i2s_check_io_apb
 *
 * Description:
 *   Common buffer validation for `i2s_send` and `i2s_receive`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *   apb  - Audio buffer to validate
 *
 * Returned Value:
 *   OK when the buffer is valid; a negated errno value otherwise.
 *
 ****************************************************************************/

static int esp_i2s_check_io_apb(struct esp_i2s_s *priv,
                                struct ap_buffer_s *apb)
{
  apb_samp_t span_end_excl_chk;

  DEBUGASSERT(apb != NULL && apb->samp != NULL);

  if (apb->nbytes > apb->nmaxbytes ||
      priv->channels == 0 || priv->data_width == 0)
    {
      return -EINVAL;
    }

  if (esp_i2s_apb_span_end_excl(apb, &span_end_excl_chk) < OK)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_i2s_io_invoke_cb
 *
 * Description:
 *   Invoke a user-provided I2S completion callback when non-NULL.
 *
 * Input Parameters:
 *   dev    - I2S device structure
 *   apb    - Audio buffer associated with the transfer
 *   cb     - User callback function
 *   arg    - Opaque argument passed to the callback
 *   result - Transfer result passed to the callback
 *
 ****************************************************************************/

static void esp_i2s_io_invoke_cb(struct i2s_dev_s *dev,
                                 struct ap_buffer_s *apb,
                                 i2s_callback_t cb, void *arg, int result)
{
  if (cb != NULL)
    {
      cb(dev, apb, arg, result);
    }
}

/****************************************************************************
 * Name: esp_i2s_io_submit_job
 *
 * Description:
 *   Reference `apb`, enqueue `job` on `queue`, and optionally wake the I/O
 *   thread.  On wakeup failure the job is removed and the APB reference is
 *   dropped; the caller must free `job`.
 *
 * Input Parameters:
 *   priv    - I2S device structure
 *   queue   - Job queue on which to enqueue `job`
 *   job     - Job entry to enqueue
 *   apb     - Audio buffer referenced by the job
 *   wake_io - True to post `io_sem` and wake the I/O thread
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_io_submit_job(struct esp_i2s_s *priv, sq_queue_t *queue,
                                 sq_entry_t *job, struct ap_buffer_s *apb,
                                 bool wake_io)
{
  int sret;

  apb_reference(apb);

  nxmutex_lock(&priv->lock);
  sq_addlast(job, queue);
  nxmutex_unlock(&priv->lock);

  if (!wake_io || !priv->io_thread_created)
    {
      return OK;
    }

  sret = nxsem_post(&priv->io_sem);
  if (sret != OK)
    {
      nxmutex_lock(&priv->lock);
      sq_remfirst(queue);
      nxmutex_unlock(&priv->lock);
      apb_free(apb);
    }

  return sret;
}

/****************************************************************************
 * Name: esp_i2s_tx_apply_carry_locked
 *
 * Description:
 *   Store TX carry bytes from a prep with zero send length.  Caller must
 *   hold `priv->lock`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *   prep - TX prep containing carry bytes to store
 *
 ****************************************************************************/

static void esp_i2s_tx_apply_carry_locked(struct esp_i2s_s *priv,
       struct esp_i2s_tx_send_prep_s *prep)
{
  priv->tx_carry.bytes = prep->new_carry_len;
  if (prep->new_carry_len != 0)
    {
      memcpy(priv->tx_carry.data, prep->new_carry, prep->new_carry_len);
    }
}

/****************************************************************************
 * Name: esp_i2s_tx_preload
 *
 * Description:
 *   Prepare an APB for TX, cache-sync, and preload into the TX DMA buffer.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *   apb  - Audio buffer to preload
 *   prep - TX prep structure to populate
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_preload(struct esp_i2s_s *priv,
                              struct ap_buffer_s *apb,
                              struct esp_i2s_tx_send_prep_s *prep)
{
  esp_err_t esp_ret;
  size_t    bytes_loaded;
  int       ret;

  ret = esp_i2s_tx_prep_apb(priv, apb, prep);
  if (ret < OK || prep->send_len == 0)
    {
      return ret;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync(prep->xfer, prep->send_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif

  esp_ret = i2s_channel_preload_data(priv->tx_handle, prep->xfer,
                                     prep->send_len, &bytes_loaded);
  if (esp_ret != ESP_OK)
    {
      esp_i2s_tx_prep_release(prep);
      return i2s_map_esp_err(esp_ret);
    }

  prep->preloaded = bytes_loaded;
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_burst_channels_down_locked
 *
 * Description:
 *   Disable both TX and RX channels during burst full duplex.  Caller must
 *   hold `priv->lock`.
 *
 * Input Parameters:
 *   priv    - I2S device structure
 *   with_rx - True to disable RX as well as TX
 *
 ****************************************************************************/

static void esp_i2s_burst_channels_down_locked(struct esp_i2s_s *priv,
                                               bool with_rx)
{
  esp_i2s_tx_disable_locked(priv);
  if (with_rx)
    {
      esp_i2s_rx_disable_locked(priv);
    }
}

/****************************************************************************
 * Name: esp_i2s_burst_reset_channels_locked
 *
 * Description:
 *   Disable stale TX (and optionally RX) channels before a burst job.
 *   Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv    - I2S device structure
 *   with_rx - True to reset RX as well as TX when active
 *
 ****************************************************************************/

static void esp_i2s_burst_reset_channels_locked(struct esp_i2s_s *priv,
                                                bool with_rx)
{
  if (priv->tx_started)
    {
      esp_i2s_burst_channels_down_locked(priv, with_rx);
    }
}

/****************************************************************************
 * Name: esp_i2s_tx_channel_enable_locked
 *
 * Description:
 *   Enable the TX I2S channel when configured and not already started.
 *   When `with_rx` is true during burst full duplex, RX is enabled as well
 *   so shared BCLK/WS stay up for a subsequent burst `i2s_receive()`.
 *   Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv    - I2S device structure
 *   with_rx - True to enable RX during burst full duplex
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_channel_enable_locked(struct esp_i2s_s *priv,
                                            bool with_rx)
{
  esp_err_t err;
  int       ret;

  if (!priv->config->tx_en || priv->tx_handle == NULL || priv->tx_started)
    {
      return OK;
    }

  err = i2s_channel_enable(priv->tx_handle);
  if (err != ESP_OK)
    {
      return i2s_map_esp_err(err);
    }

  priv->tx_started = true;

  if (with_rx && esp_i2s_burst_full_duplex(priv))
    {
      ret = esp_i2s_rx_channel_start_locked(priv);
      if (ret < OK)
        {
          i2s_channel_disable(priv->tx_handle);
          priv->tx_started = false;
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_i2s_rx_channel_start_locked
 *
 * Description:
 *   Enable the RX I2S channel via `i2s_channel_enable()` when configured and
 *   not already started. Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_rx_channel_start_locked(struct esp_i2s_s *priv)
{
  esp_err_t err;

  if (!priv->config->rx_en || priv->rx_handle == NULL || priv->rx_started)
    {
      return OK;
    }

  err = i2s_channel_enable(priv->rx_handle);
  if (err != ESP_OK)
    {
      return i2s_map_esp_err(err);
    }

  priv->rx_started = true;
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_tx_in_flight_locked
 *
 * Description:
 *   True when a buffer is being transmitted (queued, in the daemon, or
 *   draining in DMA).  Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   True when TX data is queued or still in DMA.
 *
 ****************************************************************************/

static bool esp_i2s_tx_in_flight_locked(struct esp_i2s_s *priv)
{
  return priv->tx_busy || !sq_empty(&priv->tx_jobs) ||
         priv->tx_on_sent_blocks_left > 0;
}

/****************************************************************************
 * Name: esp_i2s_tx_disable_locked
 *
 * Description:
 *   Disable the TX channel.  Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_tx_disable_locked(struct esp_i2s_s *priv)
{
  if (priv->tx_started && priv->tx_handle != NULL)
    {
      i2s_channel_disable(priv->tx_handle);
      priv->tx_started = false;
    }

  priv->tx_on_sent_blocks_left = 0;
}

/****************************************************************************
 * Name: esp_i2s_rx_disable_locked
 *
 * Description:
 *   Disable the RX channel.  Caller must hold `priv->lock`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_rx_disable_locked(struct esp_i2s_s *priv)
{
  if (priv->rx_started && priv->rx_handle != NULL)
    {
      i2s_channel_disable(priv->rx_handle);
      priv->rx_started = false;
    }
}

/****************************************************************************
 * Name: esp_i2s_rx_finish_job
 *
 * Description:
 *   Mark the current receive transfer complete and stop the RX channel when
 *   no session is active and no buffer is waiting to be received.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_rx_finish_job(struct esp_i2s_s *priv)
{
  nxmutex_lock(&priv->lock);
  priv->rx_busy = false;

  if (!priv->session_active)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  nxmutex_unlock(&priv->lock);

  esp_i2s_rx_idle_shutdown_arm(priv);
}

/****************************************************************************
 * Name: esp_i2s_tx_channel_start
 *
 * Description:
 *   Like `esp_i2s_tx_channel_enable_locked` but acquires `priv->lock`.
 *   Called from `i2s_send`.  Enables TX, and RX too during burst full
 *   duplex.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_channel_start(struct esp_i2s_s *priv)
{
  int ret;

  nxmutex_lock(&priv->lock);
  ret = esp_i2s_tx_channel_enable_locked(priv, true);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: esp_i2s_rx_channel_start
 *
 * Description:
 *   Like `esp_i2s_rx_channel_start_locked` but acquires `priv->lock`.
 *   Called from `i2s_receive`.  Burst mode defers enable to the I/O thread.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_rx_channel_start(struct esp_i2s_s *priv)
{
  int ret;

  if (!priv->session_active)
    {
      /* Burst mode: enable/disable per job on the I/O thread. */

      return OK;
    }

  nxmutex_lock(&priv->lock);
  ret = esp_i2s_rx_channel_start_locked(priv);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: esp_i2s_channels_start
 *
 * Description:
 *   Enable TX then RX under one `priv->lock` critical section
 *   (`AUDIOIOC_START`).  If RX enable fails after TX succeeded, both
 *   channels are disabled again.  Call `esp_i2s_wake_queued_streams()` after
 *   OK so queued jobs run.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_channels_start(struct esp_i2s_s *priv)
{
  int ret;

  nxmutex_lock(&priv->lock);

  ret = esp_i2s_tx_channel_enable_locked(priv, true);
  if (ret == OK)
    {
      ret = esp_i2s_rx_channel_start_locked(priv);
    }

  if (ret != OK)
    {
      if (priv->tx_started && priv->tx_handle != NULL)
        {
          i2s_channel_disable(priv->tx_handle);
          priv->tx_started = false;
        }

      if (priv->rx_started && priv->rx_handle != NULL)
        {
          i2s_channel_disable(priv->rx_handle);
          priv->rx_started = false;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp_i2s_tx_wait_on_sent_done
 *
 * Description:
 *   Block until `on_sent` has accounted for every DMA block in the current
 *   send operation.  The last `on_sent` posts `tx_on_sent_done_sem`.
 *   Used for per-APB TX when no AUDIOIOC_START session is active.
 *
 * Input Parameters:
 *   priv    - I2S device structure
 *   timeout - Wait timeout in system ticks (0 means wait forever)
 *
 * Returned Value:
 *   OK when all `on_sent` blocks completed; a negated errno on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_wait_on_sent_done(struct esp_i2s_s *priv,
                                        uint32_t timeout)
{
  int ret;

  if (priv->tx_on_sent_blocks_left == 0)
    {
      return OK;
    }

  if (timeout == 0)
    {
      ret = nxsem_wait_uninterruptible(&priv->tx_on_sent_done_sem);
    }
  else
    {
      ret = nxsem_tickwait_uninterruptible(&priv->tx_on_sent_done_sem,
                                           timeout);
    }

  return ret;
}

/****************************************************************************
 * Name: esp_i2s_burst_rx_helper_entry
 *
 * Description:
 *   Helper thread that blocks in `i2s_channel_read()` until burst TX data
 *   is on the wire.
 *
 * Input Parameters:
 *   arg - Pointer to `struct esp_i2s_burst_rx_helper_s`
 *
 * Returned Value:
 *   NULL
 *
 ****************************************************************************/

static void *esp_i2s_burst_rx_helper_entry(void *arg)
{
  struct esp_i2s_burst_rx_helper_s *ctx = arg;

  ctx->result = esp_i2s_rx_run_job_locked(ctx->priv, ctx->apb,
                                          ctx->timeout);
  return NULL;
}

/****************************************************************************
 * Name: esp_i2s_io_run_burst_full_duplex_paired
 *
 * Description:
 *   Run one queued TX and one queued RX APB together: preload TX, enable RX,
 *   start a blocking RX read on a helper thread, enable TX (starting the
 *   preloaded stream), complete any remaining TX write, wait for `on_sent`,
 *   then disable both channels.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   True when a paired TX/RX burst job was executed.
 *
 ****************************************************************************/

static bool esp_i2s_io_run_burst_full_duplex_paired(struct esp_i2s_s *priv)
{
  struct esp_i2s_async_job_s     *tx_job;
  struct esp_i2s_rx_async_job_s  *rx_job;
  struct ap_buffer_s             *tx_apb;
  struct ap_buffer_s             *rx_apb;
  i2s_callback_t                  tx_cb;
  i2s_callback_t                  rx_cb;
  void                           *tx_cba;
  void                           *rx_cba;
  uint32_t                        tx_tmo;
  uint32_t                        rx_tmo;
  struct esp_i2s_tx_send_prep_s   prep;
  int                             result = OK;
  int                             rx_result = OK;

  nxmutex_lock(&priv->lock);

  if (!priv->config->tx_en || sq_empty(&priv->tx_jobs) ||
      sq_empty(&priv->rx_jobs))
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  esp_i2s_burst_reset_channels_locked(priv, true);

  tx_job = (struct esp_i2s_async_job_s *)sq_remfirst(&priv->tx_jobs);
  rx_job = (struct esp_i2s_rx_async_job_s *)sq_remfirst(&priv->rx_jobs);

  tx_apb = tx_job->apb;
  tx_tmo = tx_job->timeout;
  tx_cb = tx_job->cb;
  tx_cba = tx_job->cbarg;

  rx_apb = rx_job->apb;
  rx_tmo = rx_job->timeout;
  rx_cb = rx_job->cb;
  rx_cba = rx_job->cbarg;

  kmm_free(tx_job);
  kmm_free(rx_job);

  priv->tx_busy = true;
  nxmutex_unlock(&priv->lock);

  result = esp_i2s_tx_preload(priv, tx_apb, &prep);
  if (result < OK)
    {
      goto out_tx_busy;
    }

  if (prep.send_len == 0)
    {
      nxmutex_lock(&priv->lock);
      esp_i2s_tx_apply_carry_locked(priv, &prep);
      priv->tx_busy = false;
      nxmutex_unlock(&priv->lock);

      esp_i2s_io_invoke_cb(&priv->dev, tx_apb, tx_cb, tx_cba, OK);
      esp_i2s_io_invoke_cb(&priv->dev, rx_apb, rx_cb, rx_cba, -ECANCELED);
      return true;
    }

  nxmutex_lock(&priv->lock);
  esp_i2s_tx_on_sent_disable_arm(priv, prep.send_len);
  result = esp_i2s_rx_channel_start_locked(priv);
  if (result == OK)
    {
      priv->rx_busy = true;
    }

  nxmutex_unlock(&priv->lock);

  if (result < OK)
    {
      esp_i2s_tx_prep_release(&prep);
      goto out_tx_busy;
    }

    {
      struct esp_i2s_burst_rx_helper_s rx_helper;
      pthread_t                       rx_thread;
      pthread_attr_t                  attr;
      int                             ret;

      rx_helper.priv = priv;
      rx_helper.apb = rx_apb;
      rx_helper.timeout = rx_tmo;
      rx_helper.result = -EIO;

      pthread_attr_init(&attr);
      pthread_attr_setstacksize(&attr, ESP_I2S_ASYNC_STACKSIZE);

      ret = pthread_create(&rx_thread, &attr, esp_i2s_burst_rx_helper_entry,
                           &rx_helper);
      pthread_attr_destroy(&attr);

      if (ret != OK)
        {
          esp_i2s_tx_prep_release(&prep);
          rx_result = -ret;
          goto out_disable;
        }

      nxmutex_lock(&priv->lock);
      result = esp_i2s_tx_channel_enable_locked(priv, false);
      nxmutex_unlock(&priv->lock);

      if (result < OK)
        {
          pthread_join(rx_thread, NULL);
          esp_i2s_tx_prep_release(&prep);
          rx_result = rx_helper.result;
          goto out_disable;
        }

      result = esp_i2s_tx_run_job_locked(priv, tx_apb, tx_tmo, &prep);

      pthread_join(rx_thread, NULL);
      rx_result = rx_helper.result;
    }

  if (result < OK)
    {
      goto out_disable;
    }

  result = esp_i2s_tx_wait_on_sent_done(priv, tx_tmo);

out_disable:
  nxmutex_lock(&priv->lock);
  esp_i2s_burst_channels_down_locked(priv, true);
  priv->tx_busy = false;
  nxmutex_unlock(&priv->lock);

  esp_i2s_rx_finish_job(priv);

  esp_i2s_io_invoke_cb(&priv->dev, tx_apb, tx_cb, tx_cba, result);
  esp_i2s_io_invoke_cb(&priv->dev, rx_apb, rx_cb, rx_cba, rx_result);

  return true;

out_tx_busy:
  nxmutex_lock(&priv->lock);
  priv->tx_busy = false;
  nxmutex_unlock(&priv->lock);

  esp_i2s_io_invoke_cb(&priv->dev, tx_apb, tx_cb, tx_cba, result);
  esp_i2s_io_invoke_cb(&priv->dev, rx_apb, rx_cb, rx_cba, -ECANCELED);

  return true;
}

/****************************************************************************
 * Name: esp_i2s_io_run_tx_burst_job
 *
 * Description:
 *   When no AUDIOIOC_START session is active, run one queued APB through a
 *   full TX cycle: preload, enable, write, wait for `on_sent`, disable.
 *   TX-only: enable/disable `tx_handle` per APB.  Full duplex with a queued
 *   RX job: paired preload/enable/read/write cycle.  Full duplex TX-only
 *   waits until an RX job is queued.  Otherwise preload, enable both, write,
 *   wait for `on_sent`, disable both.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   True when a burst TX job was executed.
 *
 ****************************************************************************/

static bool esp_i2s_io_run_tx_burst_job(struct esp_i2s_s *priv)
{
  struct esp_i2s_async_job_s     *job;
  struct ap_buffer_s             *apb;
  i2s_callback_t                  cb;
  void                           *cba;
  uint32_t                        tmo;
  struct esp_i2s_tx_send_prep_s   prep;
  int                             result = OK;
  bool                            full_duplex;

  nxmutex_lock(&priv->lock);

  if (!priv->config->tx_en || sq_empty(&priv->tx_jobs))
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  full_duplex = esp_i2s_burst_full_duplex(priv);

  if (full_duplex && sq_empty(&priv->rx_jobs))
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  if (full_duplex && !sq_empty(&priv->rx_jobs))
    {
      nxmutex_unlock(&priv->lock);
      return esp_i2s_io_run_burst_full_duplex_paired(priv);
    }

  esp_i2s_burst_reset_channels_locked(priv, false);

  job = (struct esp_i2s_async_job_s *)sq_remfirst(&priv->tx_jobs);
  apb = job->apb;
  tmo = job->timeout;
  cb = job->cb;
  cba = job->cbarg;
  kmm_free(job);
  priv->tx_busy = true;
  nxmutex_unlock(&priv->lock);

  result = esp_i2s_tx_preload(priv, apb, &prep);
  if (result < OK)
    {
      goto out_busy;
    }

  if (prep.send_len == 0)
    {
      nxmutex_lock(&priv->lock);
      esp_i2s_tx_apply_carry_locked(priv, &prep);
      priv->tx_busy = false;
      nxmutex_unlock(&priv->lock);
      goto out_cb;
    }

  nxmutex_lock(&priv->lock);
  esp_i2s_tx_on_sent_disable_arm(priv, prep.send_len);
  result = esp_i2s_tx_channel_enable_locked(priv, false);
  nxmutex_unlock(&priv->lock);

  if (result < OK)
    {
      esp_i2s_tx_prep_release(&prep);
      goto out_disable;
    }

  result = esp_i2s_tx_run_job_locked(priv, apb, tmo, &prep);
  if (result < OK)
    {
      goto out_disable;
    }

  result = esp_i2s_tx_wait_on_sent_done(priv, tmo);

  nxmutex_lock(&priv->lock);
  esp_i2s_burst_channels_down_locked(priv, false);
  priv->tx_busy = false;
  nxmutex_unlock(&priv->lock);
  goto out_cb;

out_disable:
  nxmutex_lock(&priv->lock);
  esp_i2s_burst_channels_down_locked(priv, false);
  priv->tx_busy = false;
  nxmutex_unlock(&priv->lock);

out_cb:
  esp_i2s_io_invoke_cb(&priv->dev, apb, cb, cba, result);
  return true;

out_busy:
  nxmutex_lock(&priv->lock);
  priv->tx_busy = false;
  nxmutex_unlock(&priv->lock);
  goto out_cb;
}

/****************************************************************************
 * Name: esp_i2s_io_run_tx_job
 *
 * Description:
 *   Dequeue and run one TX job.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   True when a TX job was executed.
 *
 ****************************************************************************/

static bool esp_i2s_io_run_tx_job(struct esp_i2s_s *priv)
{
  int                  result;
  struct ap_buffer_s  *apb;
  i2s_callback_t       cb;
  void                *cba;
  uint32_t             tmo;
  size_t               send_quota;
  struct esp_i2s_tx_send_prep_s prep_local;
  struct esp_i2s_tx_send_prep_s *prep = NULL;

  nxmutex_lock(&priv->lock);

  if (!priv->config->tx_en || sq_empty(&priv->tx_jobs))
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  if (!priv->session_active)
    {
      nxmutex_unlock(&priv->lock);
      return esp_i2s_io_run_tx_burst_job(priv);
    }

  if (!priv->tx_started)
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  struct esp_i2s_async_job_s *job =
    (struct esp_i2s_async_job_s *)sq_remfirst(&priv->tx_jobs);

  apb = job->apb;
  tmo = job->timeout;
  cb = job->cb;
  cba = job->cbarg;
  send_quota = job->send_quota;

  if (job->prep_valid)
    {
      prep_local = job->prep;
      prep = &prep_local;
    }

  kmm_free(job);

  priv->tx_busy = true;
  esp_i2s_tx_on_sent_disable_arm(priv, send_quota);

  nxmutex_unlock(&priv->lock);

  result = esp_i2s_tx_run_job_locked(priv, apb, tmo, prep);

  nxmutex_lock(&priv->lock);
  priv->tx_busy = false;
  nxmutex_unlock(&priv->lock);

  esp_i2s_io_invoke_cb(&priv->dev, apb, cb, cba, result);

  return true;
}

/****************************************************************************
 * Name: esp_i2s_io_run_rx_burst_job
 *
 * Description:
 *   When no AUDIOIOC_START session is active, run one queued RX APB.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   True when a burst RX job was executed.
 *
 ****************************************************************************/

static bool esp_i2s_io_run_rx_burst_job(struct esp_i2s_s *priv)
{
  int                  result;
  struct ap_buffer_s  *apb;
  i2s_callback_t       cb;
  void                *cba;
  uint32_t             tmo;
  bool                 full_duplex;
  bool                 rx_only;
  int                  enret;

  nxmutex_lock(&priv->lock);

  if (!priv->config->rx_en || sq_empty(&priv->rx_jobs))
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  full_duplex = esp_i2s_burst_full_duplex(priv);
  rx_only = priv->config->rx_en && !priv->config->tx_en;

  if (full_duplex && !priv->rx_started)
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  struct esp_i2s_rx_async_job_s *job =
    (struct esp_i2s_rx_async_job_s *)sq_remfirst(&priv->rx_jobs);

  apb = job->apb;
  tmo = job->timeout;
  cb = job->cb;
  cba = job->cbarg;

  kmm_free(job);

  if (rx_only)
    {
      enret = esp_i2s_rx_channel_start_locked(priv);
      if (enret < OK)
        {
          nxmutex_unlock(&priv->lock);
          esp_i2s_io_invoke_cb(&priv->dev, apb, cb, cba, enret);
          return true;
        }
    }

  priv->rx_busy = true;
  nxmutex_unlock(&priv->lock);

  result = esp_i2s_rx_run_job_locked(priv, apb, tmo);

  if (rx_only)
    {
      nxmutex_lock(&priv->lock);
      esp_i2s_rx_disable_locked(priv);
      nxmutex_unlock(&priv->lock);
    }

  esp_i2s_rx_finish_job(priv);
  esp_i2s_io_invoke_cb(&priv->dev, apb, cb, cba, result);

  return true;
}

/****************************************************************************
 * Name: esp_i2s_io_run_rx_job
 *
 * Description:
 *   Dequeue and run one RX job when the channel is started.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   True when an RX job was executed.
 *
 ****************************************************************************/

static bool esp_i2s_io_run_rx_job(struct esp_i2s_s *priv)
{
  int                  result;
  struct ap_buffer_s  *apb;
  i2s_callback_t       cb;
  void                *cba;
  uint32_t             tmo;

  if (!priv->session_active)
    {
      return esp_i2s_io_run_rx_burst_job(priv);
    }

  nxmutex_lock(&priv->lock);

  if (!priv->config->rx_en || !priv->rx_started || sq_empty(&priv->rx_jobs))
    {
      nxmutex_unlock(&priv->lock);
      return false;
    }

  struct esp_i2s_rx_async_job_s *job =
    (struct esp_i2s_rx_async_job_s *)sq_remfirst(&priv->rx_jobs);

  apb = job->apb;
  tmo = job->timeout;
  cb = job->cb;
  cba = job->cbarg;

  kmm_free(job);

  priv->rx_busy = true;
  nxmutex_unlock(&priv->lock);

  result = esp_i2s_rx_run_job_locked(priv, apb, tmo);

  esp_i2s_rx_finish_job(priv);
  esp_i2s_io_invoke_cb(&priv->dev, apb, cb, cba, result);

  return true;
}

/****************************************************************************
 * Name: esp_i2s_io_dispatch
 *
 * Description:
 *   Drain queued TX/RX jobs on the I/O thread.  When both queues have work,
 *   TX runs before RX so the master clock is up before a blocking read.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_io_dispatch(struct esp_i2s_s *priv)
{
  DEBUGASSERT(priv != NULL);

  for (; ; )
    {
      bool tx_done;
      bool rx_done;

      tx_done = esp_i2s_io_run_tx_job(priv);
      rx_done = esp_i2s_io_run_rx_job(priv);

      if (!tx_done && !rx_done)
        {
          return;
        }
    }
}

/****************************************************************************
 * Name: esp_i2s_io_wakeup
 *
 * Description:
 *   Post `io_sem` to wake the async I/O thread when it is running.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_io_wakeup(struct esp_i2s_s *priv)
{
  if (priv->io_thread_created && nxsem_post(&priv->io_sem) != OK)
    {
      i2serr("I2S: I/O wakeup nxsem_post failed\n");
    }
}

/****************************************************************************
 * Name: esp_i2s_io_thread_entry
 *
 * Description:
 *   Async I/O thread entry point.  Waits on `io_sem` and dispatches queued
 *   TX/RX jobs.
 *
 * Input Parameters:
 *   arg - I2S device structure cast from the pthread argument
 *
 * Returned Value:
 *   NULL (unreachable)
 *
 ****************************************************************************/

static void *esp_i2s_io_thread_entry(void *arg)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)arg;

  DEBUGASSERT(priv != NULL);

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->io_sem);
      esp_i2s_io_dispatch(priv);
      if (priv->session_active)
        {
          esp_i2s_tx_idle_shutdown_arm(priv);
          esp_i2s_rx_idle_shutdown_arm(priv);
        }
    }

  return NULL; /* unreachable */
}

/****************************************************************************
 * Name: esp_i2s_io_thread_create
 *
 * Description:
 *   Create the async I/O pthread and its semaphores when TX or RX is
 *   enabled.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_io_thread_create(struct esp_i2s_s *priv)
{
  pthread_attr_t tattr;
  struct sched_param sparam;
  int                ret;
  int                pthr;

  priv->io_thread_created = false;

  if (!priv->config->tx_en && !priv->config->rx_en)
    {
      return OK;
    }

  ret = nxsem_init(&priv->io_sem, 0, 0);
  if (ret != OK)
    {
      i2serr("I2S: I/O semaphore init failed: %d\n", ret);
      return ret;
    }

  ret = nxsem_init(&priv->tx_on_sent_done_sem, 0, 0);
  if (ret != OK)
    {
      i2serr("I2S: TX on_sent done semaphore init failed: %d\n", ret);
      nxsem_destroy(&priv->io_sem);
      return ret;
    }

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setstacksize(&tattr, ESP_I2S_ASYNC_STACKSIZE);

  pthr = pthread_create(&priv->io_thread, &tattr, esp_i2s_io_thread_entry,
                        (void *)priv);
  pthread_attr_destroy(&tattr);

  if (pthr != 0)
    {
      i2serr("I2S: I/O pthread_create failed: %d\n", pthr);
      nxsem_destroy(&priv->tx_on_sent_done_sem);
      nxsem_destroy(&priv->io_sem);
      return -pthr;
    }

  pthread_setname_np(priv->io_thread, "esp_i2s_io");
  priv->io_thread_created = true;
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_wake_queued_streams
 *
 * Description:
 *   After TX/RX channels are started, poke the I/O thread so any jobs
 *   already queued by `i2s_send` / `i2s_receive` execute.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *
 ****************************************************************************/

static void esp_i2s_wake_queued_streams(struct esp_i2s_s *priv)
{
  bool do_wakeup;

  nxmutex_lock(&priv->lock);
  do_wakeup = priv->io_thread_created &&
              ((priv->config->tx_en && priv->tx_started &&
                !sq_empty(&priv->tx_jobs)) ||
               (priv->config->rx_en && priv->rx_started &&
                !sq_empty(&priv->rx_jobs)));
  nxmutex_unlock(&priv->lock);

  if (do_wakeup)
    {
      esp_i2s_io_wakeup(priv);
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
  esp_err_t error;
  int       mfreq;
  int       ret;

  i2s_chan_config_t chan_cfg =
  {
    .id = priv->config->port,
    .role = priv->config->role,
    .dma_desc_num = CONFIG_ESPRESSIF_I2S_DMA_DESC_NUM,
    .dma_frame_num = CONFIG_ESPRESSIF_I2S_DMA_FRAME_NUM,
    .auto_clear_after_cb = true,
    .auto_clear_before_cb = false,
    .allow_pd = false,
    .intr_priority = 0,
  };

  /* PDM TX only (matches legacy limitation). */

  if (priv->config->audio_std_mode == I2S_PDM)
    {
#if SOC_I2S_SUPPORTS_PDM_TX
      const i2s_slot_mode_t pdm_slots =
        (priv->config->total_slot <= 1) ?
        I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;

      i2s_pdm_tx_config_t pdm_cfg;

      if (priv->config->rx_en)
        {
          i2serr("esp_i2s: PDM RX is not available\n");
          return -ENOTSUP;
        }

      if (!priv->config->tx_en)
        {
          return -ENODEV;
        }

      error = i2s_new_channel(&chan_cfg, &priv->tx_handle, NULL);
      if (error != ESP_OK)
        {
          return i2s_map_esp_err(error);
        }

      priv->channels = priv->config->total_slot;
      priv->data_width = (uint32_t)I2S_DATA_BIT_WIDTH_16BIT;
      priv->rate = priv->config->rate;
      priv->mclk_multiple = (uint32_t)I2S_MCLK_MULTIPLE_256;

      mfreq = (int)i2s_setmclkfrequency((struct i2s_dev_s *)priv,
                                        priv->config->rate *
                                        priv->mclk_multiple);

      if (mfreq <= 0)
        {
          i2s_configure_del_channels(priv);
          i2serr("Failed to set PDM target MCLK: %d\n", mfreq);
          return mfreq;
        }

      memset(&pdm_cfg, 0, sizeof(pdm_cfg));
      pdm_cfg.clk_cfg =
        (i2s_pdm_tx_clk_config_t)
        I2S_PDM_TX_CLK_DEFAULT_CONFIG(priv->config->rate);
      pdm_cfg.clk_cfg.clk_src =
        (i2s_clock_src_t)priv->config->tx_clk_src;
      pdm_cfg.slot_cfg =
        (i2s_pdm_tx_slot_config_t)
        I2S_PDM_TX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                        pdm_slots);
      pdm_cfg.gpio_cfg.clk = priv->config->bclk_pin;
      pdm_cfg.gpio_cfg.dout = priv->config->dout_pin;
#if SOC_I2S_PDM_MAX_TX_LINES > 1
      pdm_cfg.gpio_cfg.dout2 = I2S_GPIO_UNUSED;
#endif

      error = i2s_channel_init_pdm_tx_mode(priv->tx_handle, &pdm_cfg);
      if (error != ESP_OK)
        {
          i2s_configure_del_channels(priv);
          return i2s_map_esp_err(error);
        }

      goto sync_mclk;
#else
      return -ENOTSUP;
#endif
    }

  if (!priv->config->tx_en && !priv->config->rx_en)
    {
      return -ENODEV;
    }

  if (priv->config->tx_en && priv->config->rx_en)
    {
      error = i2s_new_channel(&chan_cfg,
                              &priv->tx_handle,
                              &priv->rx_handle);
    }
  else if (priv->config->tx_en)
    {
      error = i2s_new_channel(&chan_cfg,
                              &priv->tx_handle,
                              NULL);
    }
  else
    {
      error = i2s_new_channel(&chan_cfg,
                              NULL,
                              &priv->rx_handle);
    }

  if (error != ESP_OK)
    {
      i2s_configure_del_channels(priv);
      return i2s_map_esp_err(error);
    }

  priv->channels = priv->config->total_slot;
  priv->data_width = priv->config->data_width;
  priv->rate = priv->config->rate;

  priv->mclk_multiple =
    (uint32_t)(((i2s_data_bit_width_t)priv->config->data_width ==
                I2S_DATA_BIT_WIDTH_24BIT) ?
                 I2S_MCLK_MULTIPLE_384 :
                 I2S_MCLK_MULTIPLE_256);

  mfreq = (int)i2s_setmclkfrequency((struct i2s_dev_s *)priv,
                                    priv->config->rate *
                                    priv->mclk_multiple);

  if (mfreq <= 0)
    {
      i2s_configure_del_channels(priv);
      i2serr("Failed to set MCLK frequency: %d\n", mfreq);
      return mfreq;
    }

    {
      const i2s_clock_src_t pcm_clk_src =
        priv->config->tx_en ?
        (i2s_clock_src_t)priv->config->tx_clk_src :
        (i2s_clock_src_t)priv->config->rx_clk_src;

      const i2s_data_bit_width_t bits_pcm =
        (i2s_data_bit_width_t)priv->config->data_width;

      const i2s_slot_mode_t slot_mode_pcm =
        (priv->config->total_slot <= 1) ?
        I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;

#if SOC_I2S_SUPPORTS_TDM
      i2s_tdm_config_t tdm_cfg;

      memset(&tdm_cfg, 0, sizeof(tdm_cfg));
      tdm_cfg.gpio_cfg.mclk = priv->config->mclk_pin;
      tdm_cfg.gpio_cfg.bclk = priv->config->bclk_pin;
      tdm_cfg.gpio_cfg.ws = priv->config->ws_pin;
      tdm_cfg.gpio_cfg.dout = priv->config->dout_pin;
      tdm_cfg.gpio_cfg.din = priv->config->din_pin;

      tdm_cfg.clk_cfg =
        (i2s_tdm_clk_config_t)
        I2S_TDM_CLK_DEFAULT_CONFIG(priv->config->rate);
      tdm_cfg.clk_cfg.clk_src = pcm_clk_src;
      tdm_cfg.clk_cfg.mclk_multiple =
        (i2s_mclk_multiple_t)priv->mclk_multiple;

      switch (priv->config->audio_std_mode)
        {
        case I2S_TDM_MSB:
          tdm_cfg.slot_cfg =
            (i2s_tdm_slot_config_t)
            I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(bits_pcm,
                                            slot_mode_pcm,
                                            I2S_TDM_AUTO_SLOT);
          break;

        case I2S_TDM_PCM:
          tdm_cfg.slot_cfg =
            (i2s_tdm_slot_config_t)
            I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(bits_pcm,
                                                  slot_mode_pcm,
                                                  I2S_TDM_AUTO_SLOT);
          break;

        default:
          tdm_cfg.slot_cfg =
            (i2s_tdm_slot_config_t)
            I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(bits_pcm,
                                                slot_mode_pcm,
                                                I2S_TDM_AUTO_SLOT);
          break;
        }

      tdm_cfg.slot_cfg.ws_pol = priv->config->ws_pol;

      if (priv->config->tx_en)
        {
          error = i2s_channel_init_tdm_mode(priv->tx_handle,
                                            &tdm_cfg);
          if (error != ESP_OK)
            {
              goto err;
            }
        }

      /* TX init first aligns with ESP-IDF full-duplex pairing. */

      if (priv->config->rx_en)
        {
          error = i2s_channel_init_tdm_mode(priv->rx_handle,
                                            &tdm_cfg);
          if (error != ESP_OK)
            {
              goto err;
            }
        }
#else
      i2s_std_config_t std_cfg;

      memset(&std_cfg, 0, sizeof(std_cfg));

      std_cfg.clk_cfg =
        (i2s_std_clk_config_t)
        I2S_STD_CLK_DEFAULT_CONFIG(priv->config->rate);
      std_cfg.clk_cfg.clk_src = pcm_clk_src;
      std_cfg.clk_cfg.mclk_multiple =
        (i2s_mclk_multiple_t)priv->mclk_multiple;

      switch (priv->config->audio_std_mode)
        {
        case I2S_TDM_MSB:
          std_cfg.slot_cfg =
            (i2s_std_slot_config_t)
            I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bits_pcm,
                                            slot_mode_pcm);
          break;

        case I2S_TDM_PCM:
          std_cfg.slot_cfg =
            (i2s_std_slot_config_t)
            I2S_STD_PCM_SLOT_DEFAULT_CONFIG(bits_pcm,
                                            slot_mode_pcm);
          break;

        default:
          std_cfg.slot_cfg =
            (i2s_std_slot_config_t)
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_pcm,
                                                slot_mode_pcm);
          break;
        }

      std_cfg.slot_cfg.ws_pol = priv->config->ws_pol;

      std_cfg.gpio_cfg.mclk = priv->config->mclk_pin;
      std_cfg.gpio_cfg.bclk = priv->config->bclk_pin;
      std_cfg.gpio_cfg.ws = priv->config->ws_pin;
      std_cfg.gpio_cfg.dout = priv->config->dout_pin;
      std_cfg.gpio_cfg.din = priv->config->din_pin;

      if (priv->config->tx_en)
        {
          error = i2s_channel_init_std_mode(priv->tx_handle,
                                               &std_cfg);
          if (error != ESP_OK)
            {
              goto err;
            }
        }

      if (priv->config->rx_en)
        {
          error = i2s_channel_init_std_mode(priv->rx_handle,
                                               &std_cfg);
          if (error != ESP_OK)
            {
              goto err;
            }
        }
#endif
    }

sync_mclk:
    {
      i2s_chan_info_t chan_info;

      if (priv->tx_handle != NULL &&
          i2s_channel_get_info(priv->tx_handle, &chan_info) == ESP_OK)
        {
          priv->mclk_freq = chan_info.mclk_hz;
        }
      else if (priv->rx_handle != NULL &&
               i2s_channel_get_info(priv->rx_handle, &chan_info) == ESP_OK)
        {
          priv->mclk_freq = chan_info.mclk_hz;
        }
      else
        {
          priv->mclk_freq = priv->config->rate *
            (uint32_t)priv->mclk_multiple;
        }
    }

  ret = esp_i2s_tx_register_event_callbacks(priv);
  if (ret < OK)
    {
      i2s_configure_del_channels(priv);
      return ret;
    }

  return OK;

err:
  i2s_configure_del_channels(priv);
  return i2s_map_esp_err(error);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  esp_err_t err = ESP_OK;
  i2s_chan_info_t info;

  memset(&info, 0, sizeof(info));

  const i2s_slot_mode_t slot_mode = (channels == 1) ?
                                    I2S_SLOT_MODE_MONO :
                                    I2S_SLOT_MODE_STEREO;

  if (!priv->config->tx_en)
    {
      return 0;
    }

  if (channels == 0)
    {
      return priv->channels;
    }

  if (channels != 1 && channels != 2)
    {
      return 0;
    }

  if (priv->tx_handle == NULL)
    {
      return 0;
    }

  const bool was_tx_started = priv->tx_started;
  const bool was_rx_started = (priv->rx_handle != NULL && priv->rx_started);

  if (was_tx_started)
    {
      i2s_channel_disable(priv->tx_handle);
    }

  if (was_rx_started)
    {
      i2s_channel_disable(priv->rx_handle);
    }

  err = i2s_channel_get_info(priv->tx_handle, &info);
  if (err != ESP_OK || info.mode_cfg == NULL)
    {
      err = ESP_ERR_INVALID_STATE;
      goto out_enable;
    }

  switch (info.mode)
    {
    case I2S_COMM_MODE_STD:
      {
        const i2s_std_config_t *std_cfg =
          (const i2s_std_config_t *)info.mode_cfg;
        i2s_std_slot_config_t slot_cfg = std_cfg->slot_cfg;

        slot_cfg.slot_mode = slot_mode;

        err = i2s_channel_reconfig_std_slot(priv->tx_handle, &slot_cfg);
        if (err != ESP_OK)
          {
            goto out_enable;
          }

        if (priv->rx_handle != NULL)
          {
            err = i2s_channel_reconfig_std_slot(priv->rx_handle, &slot_cfg);
          }

        break;
      }

#if SOC_I2S_SUPPORTS_TDM
    case I2S_COMM_MODE_TDM:
      {
        const i2s_tdm_config_t *tdm_cfg =
          (const i2s_tdm_config_t *)info.mode_cfg;
        i2s_tdm_slot_config_t slot_cfg = tdm_cfg->slot_cfg;

        slot_cfg.slot_mode = slot_mode;
        slot_cfg.slot_mask = (channels == 1) ?
                             I2S_TDM_SLOT0 :
                             (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 |
                                                   I2S_TDM_SLOT1);

        err = i2s_channel_reconfig_tdm_slot(priv->tx_handle, &slot_cfg);
        if (err != ESP_OK)
          {
            goto out_enable;
          }

        if (priv->rx_handle != NULL)
          {
            err = i2s_channel_reconfig_tdm_slot(priv->rx_handle, &slot_cfg);
          }

        break;
      }
#endif

#if SOC_I2S_SUPPORTS_PDM_TX
    case I2S_COMM_MODE_PDM:
      {
        const i2s_pdm_tx_config_t *pdm_cfg =
          (const i2s_pdm_tx_config_t *)info.mode_cfg;
        i2s_pdm_tx_slot_config_t slot_cfg = pdm_cfg->slot_cfg;

        slot_cfg.slot_mode = slot_mode;
#if SOC_I2S_HW_VERSION_1
        slot_cfg.slot_mask = (channels == 1) ?
                             I2S_PDM_SLOT_LEFT : I2S_PDM_SLOT_BOTH;
#endif

        err = i2s_channel_reconfig_pdm_tx_slot(priv->tx_handle, &slot_cfg);
        break;
      }
#endif

    default:
      err = ESP_ERR_NOT_SUPPORTED;
      break;
    }

  if (err == ESP_OK)
    {
      priv->channels = channels;
    }

out_enable:
  if (was_tx_started)
    {
      i2s_channel_enable(priv->tx_handle);
    }

  if (was_rx_started)
    {
      i2s_channel_enable(priv->rx_handle);
    }

  return (err == ESP_OK) ? (int)priv->channels : 0;
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
 * Name: i2s_samplerate_refresh_mclk
 *
 * Description:
 *   Update stored MCLK frequency from the hardware after a successful clock
 *   reconfiguration. Prefer the primary channel handle, then the secondary.
 *
 * Input Parameters:
 *   priv      - I2S device structure
 *   primary   - Preferred channel handle for MCLK query
 *   secondary - Fallback channel handle for MCLK query
 *
 ****************************************************************************/

static void i2s_samplerate_refresh_mclk(struct esp_i2s_s *priv,
                                        i2s_chan_handle_t primary,
                                        i2s_chan_handle_t secondary)
{
  i2s_chan_info_t info;

  memset(&info, 0, sizeof(info));

  if (primary != NULL && i2s_channel_get_info(primary, &info) == ESP_OK)
    {
      priv->mclk_freq = info.mclk_hz;
    }
  else if (secondary != NULL &&
           i2s_channel_get_info(secondary, &info) == ESP_OK)
    {
      priv->mclk_freq = info.mclk_hz;
    }
  else
    {
      priv->mclk_freq = priv->rate * (uint32_t)priv->mclk_multiple;
    }
}

/****************************************************************************
 * Name: i2s_reconfig_samplerate_clk
 *
 * Description:
 *   Apply sample rate / MCLK multiple to I2S clock using the comm mode
 *   reported for info_handle.
 *
 * Input Parameters:
 *   priv        - I2S device structure
 *   info_handle - Channel handle used to probe comm mode
 *   is_tx_path  - True when reconfiguring the TX PDM path
 *
 * Returned Value:
 *   ESP_OK on success; an Espressif error code on failure.
 *
 ****************************************************************************/

static esp_err_t i2s_reconfig_samplerate_clk(struct esp_i2s_s *priv,
                                             i2s_chan_handle_t info_handle,
                                             bool is_tx_path)
{
  esp_err_t err;
  i2s_chan_info_t info;

  memset(&info, 0, sizeof(info));

#if !(SOC_I2S_SUPPORTS_PDM_TX || SOC_I2S_SUPPORTS_PDM_RX)
  UNUSED(is_tx_path);
#endif

  err = i2s_channel_get_info(info_handle, &info);
  if (err != ESP_OK || info.mode_cfg == NULL)
    {
      return ESP_ERR_INVALID_STATE;
    }

  switch (info.mode)
    {
    case I2S_COMM_MODE_STD:
      {
        const i2s_std_config_t *std_cfg =
          (const i2s_std_config_t *)info.mode_cfg;
        i2s_std_clk_config_t clk_cfg = std_cfg->clk_cfg;

        clk_cfg.sample_rate_hz = priv->rate;
        clk_cfg.mclk_multiple = (i2s_mclk_multiple_t)priv->mclk_multiple;

        if (priv->tx_handle != NULL)
          {
            err = i2s_channel_reconfig_std_clock(priv->tx_handle, &clk_cfg);
            if (err == ESP_OK && priv->rx_handle != NULL)
              {
                err =
                  i2s_channel_reconfig_std_clock(priv->rx_handle, &clk_cfg);
              }
          }
        else
          {
            err = i2s_channel_reconfig_std_clock(priv->rx_handle, &clk_cfg);
          }

        break;
      }

#if SOC_I2S_SUPPORTS_TDM
    case I2S_COMM_MODE_TDM:
      {
        const i2s_tdm_config_t *tdm_cfg =
          (const i2s_tdm_config_t *)info.mode_cfg;
        i2s_tdm_clk_config_t clk_cfg = tdm_cfg->clk_cfg;

        clk_cfg.sample_rate_hz = priv->rate;
        clk_cfg.mclk_multiple = (i2s_mclk_multiple_t)priv->mclk_multiple;

        if (priv->tx_handle != NULL)
          {
            err = i2s_channel_reconfig_tdm_clock(priv->tx_handle, &clk_cfg);
            if (err == ESP_OK && priv->rx_handle != NULL)
              {
                err =
                  i2s_channel_reconfig_tdm_clock(priv->rx_handle, &clk_cfg);
              }
          }
        else
          {
            err = i2s_channel_reconfig_tdm_clock(priv->rx_handle, &clk_cfg);
          }

        break;
      }
#endif

#if SOC_I2S_SUPPORTS_PDM_TX || SOC_I2S_SUPPORTS_PDM_RX
    case I2S_COMM_MODE_PDM:
      {
        if (is_tx_path)
          {
#if SOC_I2S_SUPPORTS_PDM_TX
            const i2s_pdm_tx_config_t *pdm_cfg =
              (const i2s_pdm_tx_config_t *)info.mode_cfg;
            i2s_pdm_tx_clk_config_t clk_cfg = pdm_cfg->clk_cfg;

            clk_cfg.sample_rate_hz = priv->rate;
            clk_cfg.mclk_multiple =
              (i2s_mclk_multiple_t)priv->mclk_multiple;
            clk_cfg.clk_src =
              (i2s_clock_src_t)priv->config->tx_clk_src;

            err =
              i2s_channel_reconfig_pdm_tx_clock(priv->tx_handle, &clk_cfg);
#else
            err = ESP_ERR_NOT_SUPPORTED;
#endif
          }
        else
          {
#if SOC_I2S_SUPPORTS_PDM_RX
            const i2s_pdm_rx_config_t *pdm_cfg =
              (const i2s_pdm_rx_config_t *)info.mode_cfg;
            i2s_pdm_rx_clk_config_t clk_cfg = pdm_cfg->clk_cfg;

            clk_cfg.sample_rate_hz = priv->rate;
            clk_cfg.mclk_multiple =
              (i2s_mclk_multiple_t)priv->mclk_multiple;
            clk_cfg.clk_src =
              (i2s_clock_src_t)priv->config->rx_clk_src;

            err =
              i2s_channel_reconfig_pdm_rx_clock(priv->rx_handle, &clk_cfg);
#else
            err = ESP_ERR_NOT_SUPPORTED;
#endif
          }

        break;
      }
#endif

    default:
      err = ESP_ERR_NOT_SUPPORTED;
      break;
    }

  return err;
}

/****************************************************************************
 * Name: i2s_apply_samplerate
 *
 * Description:
 *   Shared sample-rate change: pause active channels, update rate and MCLK
 *   multiple, reconfigure clock from info_handle, refresh mclk_freq, resume.
 *
 * Input Parameters:
 *   priv        - I2S device structure
 *   rate        - New sample rate in Hz (0 returns current rate)
 *   info_handle - Channel handle used to probe comm mode
 *   is_tx_path  - True when applying through the TX PDM path
 *
 * Returned Value:
 *   Resulting sample rate on success; 0 on failure.
 *
 ****************************************************************************/

static uint32_t i2s_apply_samplerate(struct esp_i2s_s *priv,
                                     uint32_t rate,
                                     i2s_chan_handle_t info_handle,
                                     bool is_tx_path)
{
  esp_err_t err;
  const bool was_tx_started = (priv->tx_handle != NULL && priv->tx_started);
  const bool was_rx_started = (priv->rx_handle != NULL && priv->rx_started);

  if (was_tx_started)
    {
      i2s_channel_disable(priv->tx_handle);
    }

  if (was_rx_started)
    {
      i2s_channel_disable(priv->rx_handle);
    }

  priv->rate = rate;

  priv->mclk_multiple =
    (uint32_t)(((i2s_data_bit_width_t)priv->data_width ==
                I2S_DATA_BIT_WIDTH_24BIT) ?
                 I2S_MCLK_MULTIPLE_384 :
                 I2S_MCLK_MULTIPLE_256);

  err = i2s_reconfig_samplerate_clk(priv, info_handle, is_tx_path);

  if (err == ESP_OK)
    {
      if (is_tx_path)
        {
          i2s_samplerate_refresh_mclk(priv,
                                      priv->tx_handle,
                                      priv->rx_handle);
        }
      else
        {
          i2s_samplerate_refresh_mclk(priv,
                                      priv->rx_handle,
                                      priv->tx_handle);
        }
    }
  else
    {
      i2serr("Failed to reconfigure I2S %s clock: %d\n",
             is_tx_path ? "TX" : "RX", err);
    }

  if (was_tx_started)
    {
      i2s_channel_enable(priv->tx_handle);
    }

  if (was_rx_started)
    {
      i2s_channel_enable(priv->rx_handle);
    }

  if (err != ESP_OK)
    {
      return (uint32_t)ERROR;
    }

  return priv->rate;
}

/****************************************************************************
 * Name: i2s_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.
 *   If sample rate is 0, the current sample rate is returned.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S TX sample rate in Hz
 *
 * Returned Value:
 *   Returns the resulting sample rate or 0 on failure.
 *
 ****************************************************************************/

static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;

  if (!priv->config->tx_en)
    {
      return 0;
    }

  if (rate == 0)
    {
      return priv->rate;
    }

  if (priv->tx_handle == NULL)
    {
      return 0;
    }

  /* Match i2s_txchannels: stop before reconfiguration; duplex shares a port
   * clock.
   */

  return i2s_apply_samplerate(priv, rate, priv->tx_handle, true);
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

  if (!priv->config->rx_en)
    {
      return 0;
    }

  if (rate == 0)
    {
      return priv->rate;
    }

  if (priv->rx_handle == NULL)
    {
      return 0;
    }

  /* Duplex shares one port clock; match i2s_txsamplerate / i2s_txchannels. */

  return i2s_apply_samplerate(priv, rate, priv->rx_handle, false);
}

/****************************************************************************
 * Name: i2s_pcm_bits_to_hal
 *
 * Description:
 *   Convert a PCM bit width in bits to the corresponding HAL enum value.
 *
 * Input Parameters:
 *   bits - PCM data width in bits (8, 16, 24, or 32)
 *
 * Output Parameters:
 *   bits_hal_out - Receives the HAL data bit width on success
 *
 * Returned Value:
 *   True when `bits` is supported; false otherwise.
 *
 ****************************************************************************/

static bool i2s_pcm_bits_to_hal(int bits, i2s_data_bit_width_t *bits_hal_out)
{
  switch (bits)
    {
    case 8:
      *bits_hal_out = I2S_DATA_BIT_WIDTH_8BIT;
      return true;

    case 16:
      *bits_hal_out = I2S_DATA_BIT_WIDTH_16BIT;
      return true;

    case 24:
      *bits_hal_out = I2S_DATA_BIT_WIDTH_24BIT;
      return true;

    case 32:
      *bits_hal_out = I2S_DATA_BIT_WIDTH_32BIT;
      return true;

    default:
      return false;
    }
}

/****************************************************************************
 * Name: i2s_apply_datawidth
 *
 * Description:
 *   Shared I2S data-width change path for TX or RX (`is_rx_path`).  Uses the
 *   path channel handle to probe comm mode and applies duplex-safe STD/TDM
 *   slot/clock updates; PDM is split TX vs RX.
 *
 * Input Parameters:
 *   priv       - I2S device structure
 *   bits       - New PCM data width in bits (0 returns current width)
 *   bits_hal   - HAL data bit width corresponding to `bits`
 *   is_rx_path - True when applying through the RX channel
 *
 * Returned Value:
 *   Resulting data width in bits on success; 0 on failure.
 *
 ****************************************************************************/

static uint32_t i2s_apply_datawidth(struct esp_i2s_s *priv,
                                    int bits,
                                    i2s_data_bit_width_t bits_hal,
                                    bool is_rx_path)
{
  esp_err_t err = ESP_OK;
  i2s_chan_info_t info;

  memset(&info, 0, sizeof(info));

  const i2s_chan_handle_t path_handle =
    is_rx_path ? priv->rx_handle : priv->tx_handle;
  const i2s_chan_handle_t pair_handle =
    is_rx_path ? priv->tx_handle : priv->rx_handle;
  DEBUGASSERT(path_handle != NULL);

  /* Duplex shares the port clock; symmetric start/stop like samplerate
   * paths.
   */

  const bool was_tx_started = (priv->tx_handle != NULL && priv->tx_started);
  const bool was_rx_started = (priv->rx_handle != NULL && priv->rx_started);

  if (was_tx_started)
    {
      i2s_channel_disable(priv->tx_handle);
    }

  if (was_rx_started)
    {
      i2s_channel_disable(priv->rx_handle);
    }

  priv->data_width = (uint32_t)bits;
  priv->mclk_multiple =
    (uint32_t)((bits_hal == I2S_DATA_BIT_WIDTH_24BIT) ?
               I2S_MCLK_MULTIPLE_384 :
               I2S_MCLK_MULTIPLE_256);

  const i2s_slot_mode_t slot_mode_pcm =
    (priv->channels <= 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;

  err = i2s_channel_get_info(path_handle, &info);
  if (err != ESP_OK || info.mode_cfg == NULL)
    {
      err = ESP_ERR_INVALID_STATE;
      goto out_enable;
    }

  switch (info.mode)
    {
    case I2S_COMM_MODE_STD:
      {
        i2s_std_slot_config_t slot_cfg;

        switch (priv->config->audio_std_mode)
          {
          case I2S_TDM_MSB:
            slot_cfg =
              (i2s_std_slot_config_t)
              I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bits_hal, slot_mode_pcm);
            break;

          case I2S_TDM_PCM:
            slot_cfg =
              (i2s_std_slot_config_t)
              I2S_STD_PCM_SLOT_DEFAULT_CONFIG(bits_hal, slot_mode_pcm);
            break;

          default:
            slot_cfg =
              (i2s_std_slot_config_t)
              I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_hal, slot_mode_pcm);
            break;
          }

        slot_cfg.ws_pol = priv->config->ws_pol;

        if (priv->tx_handle != NULL)
          {
            err =
              i2s_channel_reconfig_std_slot(priv->tx_handle,
                                            &slot_cfg);
            if (err == ESP_OK && priv->rx_handle != NULL)
              {
                err =
                  i2s_channel_reconfig_std_slot(priv->rx_handle,
                                                &slot_cfg);
              }
          }
        else
          {
            err =
              i2s_channel_reconfig_std_slot(priv->rx_handle,
                                            &slot_cfg);
          }

        if (err == ESP_OK)
          {
            err = i2s_channel_get_info(path_handle, &info);
            if (err == ESP_OK && info.mode_cfg == NULL)
              {
                err = ESP_ERR_INVALID_STATE;
              }
          }

        if (err == ESP_OK && info.mode_cfg != NULL)
          {
            const i2s_std_config_t *std_cfg =
              (const i2s_std_config_t *)info.mode_cfg;
            i2s_std_clk_config_t clk_cfg = std_cfg->clk_cfg;

            clk_cfg.sample_rate_hz = priv->rate;
            clk_cfg.mclk_multiple =
              (i2s_mclk_multiple_t)priv->mclk_multiple;

            if (priv->tx_handle != NULL)
              {
                err =
                  i2s_channel_reconfig_std_clock(priv->tx_handle,
                                                 &clk_cfg);
                if (err == ESP_OK && priv->rx_handle != NULL)
                  {
                    err =
                      i2s_channel_reconfig_std_clock(priv->rx_handle,
                                                     &clk_cfg);
                  }
              }
            else
              {
                err =
                  i2s_channel_reconfig_std_clock(priv->rx_handle,
                                                 &clk_cfg);
              }
          }

        break;
      }

#if SOC_I2S_SUPPORTS_TDM
    case I2S_COMM_MODE_TDM:
      {
        i2s_tdm_slot_config_t slot_cfg;

        switch (priv->config->audio_std_mode)
          {
          case I2S_TDM_MSB:
            slot_cfg =
              (i2s_tdm_slot_config_t)
              I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(bits_hal,
                                              slot_mode_pcm,
                                              I2S_TDM_AUTO_SLOT);
            break;

          case I2S_TDM_PCM:
            slot_cfg =
              (i2s_tdm_slot_config_t)
              I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(bits_hal,
                                                     slot_mode_pcm,
                                                     I2S_TDM_AUTO_SLOT);
            break;

          default:
            slot_cfg =
              (i2s_tdm_slot_config_t)
              I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(bits_hal,
                                                   slot_mode_pcm,
                                                   I2S_TDM_AUTO_SLOT);
            break;
          }

        slot_cfg.ws_pol = priv->config->ws_pol;

        if (priv->tx_handle != NULL)
          {
            err =
              i2s_channel_reconfig_tdm_slot(priv->tx_handle,
                                            &slot_cfg);
            if (err == ESP_OK && priv->rx_handle != NULL)
              {
                err =
                  i2s_channel_reconfig_tdm_slot(priv->rx_handle,
                                                 &slot_cfg);
              }
          }
        else
          {
            err =
              i2s_channel_reconfig_tdm_slot(priv->rx_handle,
                                              &slot_cfg);
          }

        if (err == ESP_OK)
          {
            err = i2s_channel_get_info(path_handle, &info);
            if (err == ESP_OK && info.mode_cfg == NULL)
              {
                err = ESP_ERR_INVALID_STATE;
              }
          }

        if (err == ESP_OK && info.mode_cfg != NULL)
          {
            const i2s_tdm_config_t *tdm_cfg =
              (const i2s_tdm_config_t *)info.mode_cfg;
            i2s_tdm_clk_config_t clk_cfg = tdm_cfg->clk_cfg;

            clk_cfg.sample_rate_hz = priv->rate;
            clk_cfg.mclk_multiple =
              (i2s_mclk_multiple_t)priv->mclk_multiple;

            if (priv->tx_handle != NULL)
              {
                err =
                  i2s_channel_reconfig_tdm_clock(priv->tx_handle,
                                                 &clk_cfg);
                if (err == ESP_OK && priv->rx_handle != NULL)
                  {
                    err =
                      i2s_channel_reconfig_tdm_clock(priv->rx_handle,
                                                      &clk_cfg);
                  }
              }
            else
              {
                err =
                  i2s_channel_reconfig_tdm_clock(priv->rx_handle,
                                                   &clk_cfg);
              }
          }

        break;
      }
#endif /* SOC_I2S_SUPPORTS_TDM */

#if SOC_I2S_SUPPORTS_PDM_TX || SOC_I2S_SUPPORTS_PDM_RX
    case I2S_COMM_MODE_PDM:
      if (bits_hal != I2S_DATA_BIT_WIDTH_16BIT)
        {
          err = ESP_ERR_NOT_SUPPORTED;
          break;
        }

      if (is_rx_path)
        {
#if SOC_I2S_SUPPORTS_PDM_RX
          i2s_pdm_rx_slot_config_t slot_cfg =
            (i2s_pdm_rx_slot_config_t)
            I2S_PDM_RX_SLOT_DEFAULT_CONFIG(bits_hal,
                                           slot_mode_pcm);

#  if SOC_I2S_HW_VERSION_1
          slot_cfg.slot_mask = (priv->channels <= 1) ?
                               I2S_PDM_SLOT_LEFT :
                               I2S_PDM_SLOT_BOTH;
#  endif

          err =
            i2s_channel_reconfig_pdm_rx_slot(priv->rx_handle,
                                               &slot_cfg);
#else
          err = ESP_ERR_NOT_SUPPORTED;
#endif
        }
      else
        {
#if SOC_I2S_SUPPORTS_PDM_TX
          i2s_pdm_tx_slot_config_t slot_cfg =
            (i2s_pdm_tx_slot_config_t)
            I2S_PDM_TX_SLOT_DEFAULT_CONFIG(bits_hal,
                                           slot_mode_pcm);

#  if SOC_I2S_HW_VERSION_1
          slot_cfg.slot_mask = (priv->channels <= 1) ?
                               I2S_PDM_SLOT_LEFT :
                               I2S_PDM_SLOT_BOTH;
#  endif

          err =
            i2s_channel_reconfig_pdm_tx_slot(priv->tx_handle,
                                               &slot_cfg);
#else
          err = ESP_ERR_NOT_SUPPORTED;
#endif
        }

      break;
#endif /* PDM TX || RX */

    default:
      err = ESP_ERR_NOT_SUPPORTED;
      break;
    }

  if (err == ESP_OK)
    {
      if (i2s_channel_get_info(path_handle, &info) == ESP_OK)
        {
          priv->mclk_freq = info.mclk_hz;
        }
      else if (pair_handle != NULL &&
               i2s_channel_get_info(pair_handle, &info) == ESP_OK)
        {
          priv->mclk_freq = info.mclk_hz;
        }
      else
        {
          priv->mclk_freq =
            priv->rate * (uint32_t)priv->mclk_multiple;
        }
    }
  else
    {
      i2serr("Failed to set I2S %s data width: %d\n",
             is_rx_path ? "RX" : "TX", err);
    }

out_enable:
  if (was_tx_started)
    {
      i2s_channel_enable(priv->tx_handle);
    }

  if (was_rx_started)
    {
      i2s_channel_enable(priv->rx_handle);
    }

  if (err != ESP_OK)
    {
      return 0;
    }

  return (uint32_t)bits;
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
 *   dev  - Device-specific state data
 *   bits - The I2S data width in bits.
 *
 * Returned Value:
 *   Returns the resulting data width or 0 on failure.
 *
 ****************************************************************************/

static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;
  i2s_data_bit_width_t bits_hal;

  if (!priv->config->tx_en)
    {
      return 0;
    }

  if (bits == 0)
    {
      return priv->data_width;
    }

  if (!i2s_pcm_bits_to_hal(bits, &bits_hal))
    {
      return 0;
    }

  if (priv->tx_handle == NULL)
    {
      return 0;
    }

  return i2s_apply_datawidth(priv, bits, bits_hal, false);
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
 *   dev  - Device-specific state data
 *   bits - The I2S data width in bits.
 *
 * Returned Value:
 *   Returns the resulting data width or 0 on failure.
 *
 ****************************************************************************/

static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp_i2s_s *priv = (struct esp_i2s_s *)dev;
  i2s_data_bit_width_t bits_hal;

  if (!priv->config->rx_en)
    {
      return 0;
    }

  if (bits == 0)
    {
      return priv->data_width;
    }

  if (!i2s_pcm_bits_to_hal(bits, &bits_hal))
    {
      return 0;
    }

  if (priv->rx_handle == NULL)
    {
      return 0;
    }

  return i2s_apply_datawidth(priv, bits, bits_hal, true);
}

/****************************************************************************
 * Name: esp_i2s_tx_prep_release
 *
 * Description:
 *   Free the aligned TX transfer buffer allocated by
 *   `esp_i2s_tx_prep_apb()`.
 *
 * Input Parameters:
 *   prep - TX prep structure whose `xfer` buffer should be released
 *
 ****************************************************************************/

static void esp_i2s_tx_prep_release(struct esp_i2s_tx_send_prep_s *prep)
{
  if (prep != NULL && prep->xfer != NULL)
    {
      kmm_free(prep->xfer);
      prep->xfer = NULL;
    }
}

/****************************************************************************
 * Name: esp_i2s_tx_prep_apb
 *
 * Description:
 *   Build an aligned TX block from an APB (merging `tx_carry`).  On success,
 *   `prep->xfer` is allocated when `prep->send_len > 0`.
 *
 * Input Parameters:
 *   priv - I2S device structure
 *   apb  - Source audio buffer
 *   prep - TX prep structure to populate
 *
 * Output Parameters:
 *   prep - Receives aligned transfer data and carry state on success
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_prep_apb(struct esp_i2s_s *priv,
                               struct ap_buffer_s *apb,
                               struct esp_i2s_tx_send_prep_s *prep)
{
  uint16_t           bytes_per_sample;
  uint16_t           bytes_per_frame;
  uint8_t           *buf;
  uint8_t           *samp_run;
  apb_samp_t         samp_size;
  apb_samp_t         carry_size;
  apb_samp_t         send_len;
  size_t             data_copied;
  size_t             carry_in_bytes;
  uint8_t            carry_in_buf[ESP_I2S_TX_CARRY_MAX];
  apb_samp_t         span_end_excl;
  int                span_rc;

  DEBUGASSERT(priv != NULL && apb != NULL && apb->samp != NULL &&
              prep != NULL);

  memset(prep, 0, sizeof(*prep));

  if (priv->channels == 0 || priv->data_width == 0)
    {
      return -EINVAL;
    }

  span_rc = esp_i2s_apb_span_end_excl(apb, &span_end_excl);
  if (span_rc < OK)
    {
      return span_rc;
    }

  bytes_per_sample = (uint16_t)((priv->data_width + 7u) / 8u);
  bytes_per_frame = (uint16_t)((uint32_t)bytes_per_sample * priv->channels);

  if (bytes_per_frame == 0 || bytes_per_frame > ESP_I2S_TX_CARRY_MAX)
    {
      return -EINVAL;
    }

  carry_in_bytes = priv->tx_carry.bytes;

  if (carry_in_bytes != 0)
    {
      if (carry_in_bytes > ESP_I2S_TX_CARRY_MAX)
        {
          return -EINVAL;
        }

      memcpy(carry_in_buf, priv->tx_carry.data, carry_in_bytes);
    }

  samp_run = apb->samp + apb->curbyte;
  samp_size = (apb_samp_t)carry_in_bytes + (span_end_excl - apb->curbyte);
  carry_size = samp_size % bytes_per_frame;
  send_len = samp_size - carry_size;

  if ((uint32_t)send_len >
      (uint32_t)I2S_DMA_BUFFER_MAX_SIZE * (uint32_t)I2S_DMADESC_NUM)
    {
      i2serr("I2S TX block (%" PRIu32 " bytes) exceeds DMA link limit\n",
             (uint32_t)send_len);
      return -EFBIG;
    }

  if (send_len == 0)
    {
      uint8_t tmp[ESP_I2S_TX_CARRY_MAX];

      if ((size_t)samp_size > ESP_I2S_TX_CARRY_MAX)
        {
          return -EINVAL;
        }

      if (carry_in_bytes != 0)
        {
          memcpy(tmp, carry_in_buf, carry_in_bytes);
        }

      memcpy(tmp + carry_in_bytes, samp_run,
             (size_t)(span_end_excl - apb->curbyte));

      prep->new_carry_len = (size_t)samp_size;
      memcpy(prep->new_carry, tmp, prep->new_carry_len);
      return OK;
    }

  prep->xfer = kmm_malloc((size_t)send_len);
  if (prep->xfer == NULL)
    {
      return -ENOMEM;
    }

  buf = prep->xfer;
  data_copied = 0;

  if (carry_in_bytes != 0)
    {
      memcpy(buf, carry_in_buf, carry_in_bytes);
      buf += carry_in_bytes;
      data_copied += carry_in_bytes;
      memcpy(buf, samp_run, (size_t)(bytes_per_frame - carry_in_bytes));
      buf += (size_t)(bytes_per_frame - carry_in_bytes);
      samp_run += (size_t)(bytes_per_frame - carry_in_bytes);
      data_copied += (size_t)(bytes_per_frame - carry_in_bytes);
    }

  memcpy(buf, samp_run,
         (size_t)(samp_size - (apb_samp_t)(data_copied + carry_size)));
  samp_run += (size_t)(samp_size - (apb_samp_t)(data_copied + carry_size));

  prep->new_carry_len = (size_t)carry_size;
  if (prep->new_carry_len != 0)
    {
      memcpy(prep->new_carry, samp_run, prep->new_carry_len);
    }

  prep->send_len = (size_t)send_len;
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_tx_run_job_locked
 *
 * Description:
 *   Transmit one APB worth of PCM (plus TX carry merge).
 *   `priv->lock` must NOT be held during this call: TX may block inside
 *   `i2s_channel_write()`.
 *
 * Input Parameters:
 *   priv    - I2S device structure
 *   apb     - Audio buffer to transmit
 *   timeout - Write timeout in system ticks (0 means wait forever)
 *   prep    - Optional prior TX prep result
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_tx_run_job_locked(struct esp_i2s_s *priv,
                                     struct ap_buffer_s *apb,
                                     uint32_t timeout,
                                     struct esp_i2s_tx_send_prep_s *prep)
{
  esp_err_t                       esp_ret;
  struct esp_i2s_tx_send_prep_s   local;
  struct esp_i2s_tx_send_prep_s  *active;
  size_t                          bytes_written;
  uint32_t                        timeout_ms;
  int                             ret;

  DEBUGASSERT(priv != NULL && apb != NULL && apb->samp != NULL);

  if (priv->tx_handle == NULL || !priv->tx_started)
    {
      return -EAGAIN;
    }

  if (prep != NULL)
    {
      active = prep;
    }
  else
    {
      ret = esp_i2s_tx_prep_apb(priv, apb, &local);
      if (ret < OK)
        {
          return ret;
        }

      active = &local;
    }

  if (active->send_len == 0)
    {
      priv->tx_carry.bytes = active->new_carry_len;
      if (active->new_carry_len != 0)
        {
          memcpy(priv->tx_carry.data, active->new_carry,
                 active->new_carry_len);
        }

      esp_i2s_tx_prep_release(active);
      return OK;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync(active->xfer, active->send_len,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif

  if (active->preloaded >= active->send_len)
    {
      priv->tx_carry.bytes = active->new_carry_len;
      if (active->new_carry_len != 0)
        {
          memcpy(priv->tx_carry.data, active->new_carry,
                 active->new_carry_len);
        }

      esp_i2s_tx_prep_release(active);
      return OK;
    }

  if (timeout == 0)
    {
      timeout_ms = UINT32_MAX;
    }
  else
    {
      timeout_ms = TICK2MSEC(timeout);
      if (timeout_ms == 0)
        {
          timeout_ms = 1;
        }
    }

  esp_ret = i2s_channel_write(priv->tx_handle,
                              active->xfer + active->preloaded,
                              active->send_len - active->preloaded,
                              &bytes_written, timeout_ms);

  if (esp_ret != ESP_OK ||
      bytes_written != active->send_len - active->preloaded)
    {
      esp_i2s_tx_prep_release(active);

      if (esp_ret == ESP_ERR_TIMEOUT)
        {
          return -ETIMEDOUT;
        }

      return (esp_ret != ESP_OK) ? i2s_map_esp_err(esp_ret) : -EIO;
    }

  priv->tx_carry.bytes = active->new_carry_len;
  if (active->new_carry_len != 0)
    {
      memcpy(priv->tx_carry.data, active->new_carry,
             active->new_carry_len);
    }

  esp_i2s_tx_prep_release(active);
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_apb_span_end_excl
 *
 * Description:
 *   Computes `span_end_excl`, the exclusive end byte offset for the APB
 *   payload slice `[apb->curbyte, span_end_excl)`.
 *
 * Input Parameters:
 *   apb - Audio buffer whose payload span is computed
 *
 * Output Parameters:
 *   span_end_excl_out - Receives the exclusive end byte offset
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_apb_span_end_excl(struct ap_buffer_s *apb,
                                     apb_samp_t *span_end_excl_out)
{
  apb_samp_t span_end_excl;

  if (apb->nbytes > apb->curbyte)
    {
      span_end_excl = apb->nbytes;
    }
  else if (apb->curbyte == 0 && apb->nbytes == 0 && apb->nmaxbytes > 0)
    {
      span_end_excl = apb->nmaxbytes;
    }
  else
    {
      return -EINVAL;
    }

  if (span_end_excl > apb->nmaxbytes)
    {
      return -EINVAL;
    }

  *span_end_excl_out = span_end_excl;
  return OK;
}

/****************************************************************************
 * Name: esp_i2s_rx_run_job_locked
 *
 * Description:
 *   Receive one aligned PCM chunk into APB via `i2s_channel_read()`.
 *   `priv->lock` must NOT be held during this call: RX may block for an
 *   unbounded time when `timeout` is zero (converted to limitless wait).
 *
 * Input Parameters:
 *   priv    - I2S device structure
 *   apb     - Audio buffer to receive into
 *   timeout - Read timeout in system ticks (0 means wait forever)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_i2s_rx_run_job_locked(struct esp_i2s_s *priv,
                                     struct ap_buffer_s *apb,
                                     uint32_t timeout)
{
  esp_err_t          esp_ret;
  uint16_t           bytes_per_sample;
  uint16_t           bytes_per_frame;
  apb_samp_t         recv_len;
  size_t             bytes_read;
  uint32_t           timeout_ms;
  uint8_t           *dest;
  apb_samp_t         span_end_excl;
  int                span_rc;

  DEBUGASSERT(priv != NULL && apb != NULL && apb->samp != NULL);

  if (priv->channels == 0 || priv->data_width == 0)
    {
      return -EINVAL;
    }

  span_rc = esp_i2s_apb_span_end_excl(apb, &span_end_excl);
  if (span_rc < OK)
    {
      return span_rc;
    }

  if (priv->rx_handle == NULL || !priv->rx_started)
    {
      return -EAGAIN;
    }

  bytes_per_sample = (uint16_t)((priv->data_width + 7u) / 8u);
  bytes_per_frame = (uint16_t)((uint32_t)bytes_per_sample * priv->channels);

  if (bytes_per_frame == 0 || bytes_per_frame > ESP_I2S_TX_CARRY_MAX)
    {
      return -EINVAL;
    }

  recv_len = span_end_excl - apb->curbyte;
  recv_len = (apb_samp_t)((uint32_t)recv_len -
                          ((uint32_t)recv_len % (uint32_t)bytes_per_frame));

  if ((uint32_t)recv_len == 0)
    {
      return -EINVAL;
    }

  if ((uint32_t)recv_len >
      (uint32_t)priv->rx_handle->dma.buf_size *
      (uint32_t)priv->rx_handle->dma.desc_num)
    {
      i2serr("I2S RX block (%" PRIu32 " bytes) exceeds DMA link limit\n",
             (uint32_t)recv_len);
      return -EFBIG;
    }

  dest = apb->samp + apb->curbyte;

  if (timeout == 0)
    {
      timeout_ms = UINT32_MAX;
    }
  else
    {
      timeout_ms = TICK2MSEC(timeout);
      if (timeout_ms == 0)
        {
          timeout_ms = 1;
        }
    }

  bytes_read = 0;
  while (bytes_read < (size_t)recv_len)
    {
      size_t chunk;
      size_t chunk_read = 0;

      chunk = (size_t)recv_len - bytes_read;
      if (chunk > priv->rx_handle->dma.buf_size)
        {
          chunk = priv->rx_handle->dma.buf_size;
        }

      esp_ret = i2s_channel_read(priv->rx_handle, dest + bytes_read, chunk,
                                 &chunk_read, timeout_ms);

      if (esp_ret != ESP_OK || chunk_read != chunk)
        {
          if (esp_ret == ESP_ERR_TIMEOUT)
            {
              return -ETIMEDOUT;
            }

          i2serr("I2S RX read failed: esp=%d chunk=%zu/%zu "
                 "total=%zu/%zu\n",
                 (int)esp_ret, chunk_read, chunk, bytes_read,
                 (size_t)recv_len);
          return (esp_ret != ESP_OK) ? i2s_map_esp_err(esp_ret) : -EIO;
        }

      bytes_read += chunk_read;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync(dest, (size_t)recv_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
#endif

  apb->nbytes = apb->curbyte + recv_len;

  return OK;
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
  int               ret = OK;
  struct esp_i2s_async_job_s *job;
  bool              preload_done = false;
  struct esp_i2s_tx_send_prep_s preload_prep;

  if (!priv->config->tx_en)
    {
      return -ENOTTY;
    }

  if (priv->tx_handle == NULL)
    {
      return -EAGAIN;
    }

  ret = esp_i2s_check_io_apb(priv, apb);
  if (ret < OK)
    {
      return ret;
    }

  job = kmm_malloc(sizeof(*job));
  if (job == NULL)
    {
      return -ENOMEM;
    }

  memset(job, 0, sizeof(*job));
  job->apb = apb;
  job->cb = callback;
  job->cbarg = arg;
  job->timeout = timeout;

  if (!priv->session_active)
    {
      ret = esp_i2s_io_submit_job(priv, &priv->tx_jobs, (sq_entry_t *)job,
                                  apb, true);
      if (ret < OK)
        {
          kmm_free(job);
        }

      return ret;
    }

  memset(&preload_prep, 0, sizeof(preload_prep));
  esp_i2s_tx_idle_shutdown_disarm(priv);

  nxmutex_lock(&priv->lock);
  if (!priv->tx_started)
    {
      size_t bytes_loaded;
      esp_err_t esp_ret;

      ret = esp_i2s_tx_prep_apb(priv, apb, &preload_prep);
      if (ret == OK && preload_prep.send_len > 0)
        {
          esp_ret = i2s_channel_preload_data(priv->tx_handle,
                                             preload_prep.xfer,
                                             preload_prep.send_len,
                                             &bytes_loaded);
          if (esp_ret != ESP_OK)
            {
              esp_i2s_tx_prep_release(&preload_prep);
              ret = i2s_map_esp_err(esp_ret);
            }
          else
            {
              preload_prep.preloaded = bytes_loaded;
              preload_done = true;
            }
        }
      else if (ret == OK)
        {
          preload_done = true;
        }
    }

  nxmutex_unlock(&priv->lock);

  if (ret < OK)
    {
      kmm_free(job);
      return ret;
    }

  if (preload_done)
    {
      job->prep = preload_prep;
      job->prep_valid = true;
      memset(&preload_prep, 0, sizeof(preload_prep));
    }

  ret = esp_i2s_tx_job_send_quota(priv, apb,
                                  job->prep_valid ? &job->prep : NULL,
                                  job->prep_valid, &job->send_quota);
  if (ret < OK)
    {
      if (job->prep_valid)
        {
          esp_i2s_tx_prep_release(&job->prep);
        }

      kmm_free(job);
      return ret;
    }

  ret = esp_i2s_tx_channel_start(priv);
  if (ret < OK)
    {
      if (job->prep_valid)
        {
          esp_i2s_tx_prep_release(&job->prep);
        }

      kmm_free(job);
      return ret;
    }

    {
      bool wake_io;

      nxmutex_lock(&priv->lock);
      wake_io = priv->tx_started;
      nxmutex_unlock(&priv->lock);

      ret = esp_i2s_io_submit_job(priv, &priv->tx_jobs, (sq_entry_t *)job,
                                  apb, wake_io);
    }

  if (ret < OK)
    {
      if (job->prep_valid)
        {
          esp_i2s_tx_prep_release(&job->prep);
        }

      kmm_free(job);
    }

  return ret;
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
  struct esp_i2s_rx_async_job_s *job;
  int               ret;

  if (!priv->config->rx_en)
    {
      return -ENOTTY;
    }

  if (priv->rx_handle == NULL)
    {
      return -EAGAIN;
    }

  ret = esp_i2s_check_io_apb(priv, apb);
  if (ret < OK)
    {
      return ret;
    }

  job = kmm_malloc(sizeof(*job));
  if (job == NULL)
    {
      return -ENOMEM;
    }

  memset(job, 0, sizeof(*job));
  job->apb = apb;
  job->cb = callback;
  job->cbarg = arg;
  job->timeout = timeout;

  if (priv->session_active)
    {
      esp_i2s_rx_idle_shutdown_disarm(priv);

      ret = esp_i2s_rx_channel_start(priv);
      if (ret < OK)
        {
          kmm_free(job);
          return ret;
        }
    }

  ret = esp_i2s_io_submit_job(priv, &priv->rx_jobs, (sq_entry_t *)job, apb,
                              true);
  if (ret < OK)
    {
      kmm_free(job);
    }

  return ret;
}

/****************************************************************************
 * Name: esp_i2s_rx_jobs_drain_cancel
 *
 * Description:
 *   Drain the RX job queue and invoke each callback with `result`.
 *
 * Input Parameters:
 *   priv   - I2S device structure
 *   result - Result code passed to each drained callback
 *
 ****************************************************************************/

static void esp_i2s_rx_jobs_drain_cancel(struct esp_i2s_s *priv, int result)
{
  for (; ; )
    {
      struct esp_i2s_rx_async_job_s *job;

      nxmutex_lock(&priv->lock);
      job = (struct esp_i2s_rx_async_job_s *)sq_remfirst(&priv->rx_jobs);
      nxmutex_unlock(&priv->lock);

      if (job == NULL)
        {
          return;
        }

      struct ap_buffer_s *apb = job->apb;
      i2s_callback_t      cb = job->cb;
      void               *cba = job->cbarg;

      kmm_free(job);

      if (cb != NULL)
        {
          cb(&priv->dev, apb, cba, result);
        }
    }
}

/****************************************************************************
 * Name: esp_i2s_jobs_drain_cancel
 *
 * Description:
 *   Drain a TX or RX job queue and invoke each callback with `result`.
 *
 * Input Parameters:
 *   priv   - I2S device structure
 *   jobs   - Job queue to drain
 *   result - Result code passed to each drained callback
 *
 ****************************************************************************/

static void esp_i2s_jobs_drain_cancel(struct esp_i2s_s *priv,
                                      sq_queue_t *jobs,
                                      int result)
{
  for (; ; )
    {
      struct esp_i2s_async_job_s *job;

      nxmutex_lock(&priv->lock);
      job = (struct esp_i2s_async_job_s *)sq_remfirst(jobs);
      nxmutex_unlock(&priv->lock);

      if (job == NULL)
        {
          return;
        }

      struct ap_buffer_s *apb = job->apb;
      i2s_callback_t      cb = job->cb;
      void               *cba = job->cbarg;

      kmm_free(job);

      if (cb != NULL)
        {
          cb(&priv->dev, apb, cba, result);
        }
    }
}

/****************************************************************************
 * Name: i2s_ioctl
 *
 * Description:
 *   Implement the lower-half logic ioctl commands
 *
 * Input Parameters:
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

          ret = esp_i2s_channels_start(priv);
          if (ret == OK)
            {
              nxmutex_lock(&priv->lock);
              priv->session_active = true;
              nxmutex_unlock(&priv->lock);
              esp_i2s_wake_queued_streams(priv);
            }
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

          esp_i2s_jobs_drain_cancel(priv, &priv->tx_jobs, -ECANCELED);
          esp_i2s_rx_jobs_drain_cancel(priv, -ECANCELED);

          esp_i2s_tx_idle_shutdown_disarm(priv);
          esp_i2s_rx_idle_shutdown_disarm(priv);

          nxmutex_lock(&priv->lock);

          if (priv->tx_started && priv->tx_handle != NULL)
            {
              i2s_channel_disable(priv->tx_handle);
              priv->tx_started = false;
            }

          priv->tx_on_sent_blocks_left = 0;
          priv->rx_busy = false;
          priv->tx_busy = false;
          priv->session_active = false;

          if (priv->rx_started && priv->rx_handle != NULL)
            {
              i2s_channel_disable(priv->rx_handle);
              priv->rx_started = false;
            }

          nxmutex_unlock(&priv->lock);

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
#ifdef CONFIG_PM
  esp_pm_lock_type_t pm_type = ESP_PM_APB_FREQ_MAX;
#endif

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

  sq_init(&priv->tx_jobs);
  sq_init(&priv->rx_jobs);

  flags = spin_lock_irqsave(&priv->slock);

#ifdef CONFIG_PM
#  if SOC_I2S_SUPPORTS_APLL && SOC_I2S_HW_VERSION_2
  if (priv->config->tx_clk_src == (int)I2S_CLK_SRC_APLL ||
      priv->config->rx_clk_src == (int)I2S_CLK_SRC_APLL)
    {
      pm_type = ESP_PM_NO_LIGHT_SLEEP;
    }
#  endif

  if (priv->config->pm_lock == NULL)
    {
      esp_pm_lock_handle_t pm_lock = priv->config->pm_lock;
      ret =  esp_pm_lock_create(pm_type,
                                0,
                                "i2s_driver",
                                &pm_lock);
      if (ret != OK)
        {
          i2serr("Failed to create I2S PM lock\n");
          goto err;
        }
    }
#endif

  ret = i2s_configure(priv);
  if (ret < 0)
    {
      goto err;
    }

  priv->tx_started = false;
  priv->rx_started = false;

  spin_unlock_irqrestore(&priv->slock, flags);

  ret = esp_i2s_io_thread_create(priv);
  if (ret != OK)
    {
      irqstate_t f2;

      i2serr("I2S: I/O thread create: %d\n", ret);

      f2 = spin_lock_irqsave(&priv->slock);
      i2s_configure_del_channels(priv);
      spin_unlock_irqrestore(&priv->slock, f2);

      return NULL;
    }

  /* Success exit */

  i2sinfo("I2S%ld was successfully initialized\n", priv->config->port);

  return &priv->dev;

  /* Failure exit */

err:
  spin_unlock_irqrestore(&priv->slock, flags);
  return NULL;
}
