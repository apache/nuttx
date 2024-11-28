/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_rmt.c
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

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/rmt/rmt.h>
#include <nuttx/spinlock.h>
#include <nuttx/circbuf.h>

#include "xtensa.h"
#ifdef CONFIG_ARCH_CHIP_ESP32
#include "hardware/esp32_soc.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"
#elif CONFIG_ARCH_CHIP_ESP32S2
#include "hardware/esp32s2_soc.h"
#include "esp32s2_gpio.h"
#include "esp32s2_irq.h"
#elif CONFIG_ARCH_CHIP_ESP32S3
#include "hardware/esp32s3_soc.h"
#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#endif

#include "hal/gpio_types.h"
#include "hal/rmt_hal.h"
#include "hal/rmt_ll.h"
#include "periph_ctrl.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_periph.h"
#include "soc/soc_caps.h"
#include "esp_clk_tree.h"

#include "esp_rmt.h"

#ifdef CONFIG_ESP_RMT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RMT_RX_CHANNEL_ENCODING_START \
  (SOC_RMT_CHANNELS_PER_GROUP-SOC_RMT_TX_CANDIDATES_PER_GROUP)
#define RMT_TX_CHANNEL_ENCODING_END   (SOC_RMT_TX_CANDIDATES_PER_GROUP-1)

#define RMT_IS_RX_CHANNEL(channel)  \
  ((channel) >= RMT_RX_CHANNEL_ENCODING_START)
#define RMT_IS_TX_CHANNEL(channel)  \
  ((channel) <= RMT_TX_CHANNEL_ENCODING_END)
#define RMT_DECODE_RX_CHANNEL(encode_chan)  \
  ((encode_chan - RMT_RX_CHANNEL_ENCODING_START))
#define RMT_ENCODE_RX_CHANNEL(decode_chan)  \
  ((decode_chan + RMT_RX_CHANNEL_ENCODING_START))

/* Default configuration for TX channel */

#define RMT_DEFAULT_CONFIG_TX(gpio, channel_id)     \
  {                                                 \
    .rmt_mode = RMT_MODE_TX,                        \
    .channel = channel_id,                          \
    .gpio_num = gpio,                               \
    .clk_div = RMT_DEFAULT_CLK_DIV,                 \
    .mem_block_num = 1,                             \
    .flags = 0,                                     \
    .tx_config = {                                  \
        .carrier_freq_hz = 38000,                   \
        .carrier_level = RMT_CARRIER_LEVEL_HIGH,    \
        .idle_level = RMT_IDLE_LEVEL_LOW,           \
        .carrier_duty_percent = 33,                 \
        .loop_count = 0,                            \
        .carrier_en = false,                        \
        .loop_en = false,                           \
        .idle_output_en = true,                     \
    }                                               \
  }

/* Default configuration for RX channel */

#define RMT_DEFAULT_CONFIG_RX(gpio, channel_id)   \
  {                                               \
      .rmt_mode = RMT_MODE_RX,                    \
      .channel = channel_id,                      \
      .gpio_num = gpio,                           \
      .clk_div = RMT_DEFAULT_CLK_DIV,             \
      .mem_block_num = 1,                         \
      .flags = 0,                                 \
      .rx_config = {                              \
          .idle_threshold = 12000,                \
          .filter_ticks_thresh = 100,             \
          .filter_en = true,                      \
      }                                           \
  }

#define rmt_item32_t rmt_symbol_word_t

#ifdef CONFIG_ARCH_CHIP_ESP32
#  define esp_configgpio      esp32_configgpio
#  define esp_gpio_matrix_out esp32_gpio_matrix_out
#  define esp_gpio_matrix_in  esp32_gpio_matrix_in
#  define esp_setup_irq       esp32_setup_irq
#  define esp_teardown_irq    esp32_teardown_irq

#  define GPIO_OUT_FUNC       OUTPUT_FUNCTION_3
#  define GPIO_IN_FUNC        INPUT_FUNCTION_3
#  define ESP_CPUINT_LEVEL    ESP32_CPUINT_LEVEL

#elif CONFIG_ARCH_CHIP_ESP32S2
#  define esp_configgpio      esp32s2_configgpio
#  define esp_gpio_matrix_out esp32s2_gpio_matrix_out
#  define esp_gpio_matrix_in  esp32s2_gpio_matrix_in
#  define esp_setup_irq       esp32s2_setup_irq
#  define esp_teardown_irq    esp32s2_teardown_irq

#  define GPIO_OUT_FUNC       OUTPUT_FUNCTION_2
#  define GPIO_IN_FUNC        INPUT_FUNCTION_2
#  define ESP_CPUINT_LEVEL    ESP32S2_CPUINT_LEVEL

#elif CONFIG_ARCH_CHIP_ESP32S3
#  define esp_configgpio      esp32s3_configgpio
#  define esp_gpio_matrix_out esp32s3_gpio_matrix_out
#  define esp_gpio_matrix_in  esp32s3_gpio_matrix_in
#  define esp_setup_irq       esp32s3_setup_irq
#  define esp_teardown_irq    esp32s3_teardown_irq

#  define GPIO_OUT_FUNC       OUTPUT_FUNCTION_2
#  define GPIO_IN_FUNC        INPUT_FUNCTION_2
#  define ESP_CPUINT_LEVEL    ESP32S3_CPUINT_LEVEL

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* RMT channel ID */

enum rmt_channel_e
{
  RMT_CHANNEL_0,  /* RMT channel number 0 */
  RMT_CHANNEL_1,  /* RMT channel number 1 */
  RMT_CHANNEL_2,  /* RMT channel number 2 */
  RMT_CHANNEL_3,  /* RMT channel number 3 */
#if SOC_RMT_CHANNELS_PER_GROUP > 4
  RMT_CHANNEL_4,  /* RMT channel number 4 */
  RMT_CHANNEL_5,  /* RMT channel number 5 */
  RMT_CHANNEL_6,  /* RMT channel number 6 */
  RMT_CHANNEL_7,  /* RMT channel number 7 */
#endif
  RMT_CHANNEL_MAX /* Number of RMT channels */
};

typedef enum rmt_channel_e rmt_channel_t;

/* RMT Channel Working Mode (TX or RX) */

enum rmt_mode_e
{
  RMT_MODE_TX, /* RMT TX mode */
  RMT_MODE_RX, /* RMT RX mode */
  RMT_MODE_MAX
};

typedef enum rmt_mode_e rmt_mode_t;

/* RMT Idle Level */

enum rmt_idle_level_e
{
  RMT_IDLE_LEVEL_LOW,  /* RMT TX idle level: low Level */
  RMT_IDLE_LEVEL_HIGH, /* RMT TX idle level: high Level */
  RMT_IDLE_LEVEL_MAX,
};

typedef enum rmt_idle_level_e rmt_idle_level_t;

/* RMT Carrier Level */

enum rmt_carrier_level_e
{
  RMT_CARRIER_LEVEL_LOW,  /* RMT carrier wave is modulated for low Level output */
  RMT_CARRIER_LEVEL_HIGH, /* RMT carrier wave is modulated for high Level output */
  RMT_CARRIER_LEVEL_MAX
};

typedef enum rmt_carrier_level_e rmt_carrier_level_t;

/* RMT Channel Status */

enum rmt_channel_status_e
{
  RMT_CHANNEL_UNINIT, /* RMT channel uninitialized */
  RMT_CHANNEL_IDLE,   /* RMT channel status idle */
  RMT_CHANNEL_BUSY,   /* RMT channel status busy */
};

typedef enum rmt_channel_status_e rmt_channel_status_t;

/* RMT hardware memory layout */

struct rmt_channel_data_s
{
  volatile rmt_item32_t data32[SOC_RMT_MEM_WORDS_PER_CHANNEL];
};

struct rmt_mem_s
{
  struct rmt_channel_data_s chan[SOC_RMT_CHANNELS_PER_GROUP];
};

typedef struct rmt_mem_s rmt_mem_t;

struct rmt_dev_common_s
{
  rmt_hal_context_t hal;          /* HAL context */
  rmutex_t rmt_driver_isr_lock;

  /* Mutex lock for protecting concurrent register/unregister of the RMT
   * channels' ISR.
   */

  spinlock_t rmt_spinlock;

  /* Bitmask of installed drivers' channels, used to protect concurrent
   * register/unregister of the RMT channels' ISR.
   */

  uint8_t rmt_driver_channels;
  bool rmt_module_enabled;

  /* Bitmap of channels already added in the synchronous group */

  uint32_t synchro_channel_mask;
};

struct rmt_dev_lowerhalf_s
{
  /* The following block is part of the upper-half device struct */

  const struct rmt_ops_s *ops;
  struct circbuf_s       *circbuf;
  sem_t                  *recvsem;
  int                     minor;

  /* The following is private to the ESP32 RMT driver */

  rmt_mode_t               mode;
  struct rmt_dev_common_s *common; /* RMT peripheral common parameters */
};

struct rmt_obj_s
{
  size_t tx_offset;
  size_t tx_len_rem;
  size_t tx_sub_len;
  bool wait_done;     /* Mark whether wait tx done */
  bool loop_autostop; /* Mark whether loop auto-stop is enabled */
  rmt_channel_t channel;
  const rmt_item32_t *tx_data;
  sem_t tx_sem;
#ifdef CONFIG_SPIRAM_USE_MALLOC
  int intr_alloc_flags;
  sem_t tx_sem_buffer;
#endif
  rmt_item32_t *tx_buf;
  struct circbuf_s rx_buf;
  sem_t rx_sem;
#ifdef SOC_RMT_SUPPORT_RX_PINGPONG
  rmt_item32_t *rx_item_buf;
  uint32_t rx_item_buf_size;
  uint32_t rx_item_len;
  int rx_item_start_idx;
#endif
  void *tx_context;
  size_t sample_size_remain;
  const uint8_t *sample_cur;
};

typedef struct rmt_obj_s rmt_obj_t;

/* Data struct of RMT TX configure parameters */

struct rmt_tx_config_s
{
  uint32_t carrier_freq_hz;           /* RMT carrier frequency */
  rmt_carrier_level_t carrier_level;  /* Level of the RMT output, when the carrier is applied */
  rmt_idle_level_t idle_level;        /* RMT idle level */
  uint8_t carrier_duty_percent;       /* RMT carrier duty (%) */
  uint32_t loop_count;                /* Maximum loop count, only take effect for chips that is capable of `SOC_RMT_SUPPORT_TX_LOOP_COUNT` */
  bool carrier_en;                    /* RMT carrier enable */
  bool loop_en;                       /* Enable sending RMT items in a loop */
  bool idle_output_en;                /* RMT idle level output enable */
};

/* Data struct of RMT RX configure parameters */

struct rmt_rx_config_s
{
  uint16_t idle_threshold;            /* RMT RX idle threshold */
  uint8_t filter_ticks_thresh;        /* RMT filter tick number */
  bool filter_en;                     /* RMT receiver filter enable */
#if SOC_RMT_SUPPORT_RX_DEMODULATION
  bool rm_carrier;                    /* RMT receiver remove carrier enable */
  uint32_t carrier_freq_hz;           /* RMT carrier frequency */
  uint8_t carrier_duty_percent;       /* RMT carrier duty (%) */
  rmt_carrier_level_t carrier_level;  /* The level to remove the carrier */
#endif
};

struct rmt_channel_config_s
{
  rmt_mode_t rmt_mode;   /* RMT mode: transmitter or receiver */
  rmt_channel_t channel; /* RMT channel */
  int gpio_num;          /* RMT GPIO number */
  uint8_t clk_div;       /* RMT channel counter divider */
  uint8_t mem_block_num; /* RMT memory block number */
  uint32_t flags;        /* RMT channel extra configurations, OR'd with RMT_CHANNEL_FLAGS_[*] */
  union
    {
      struct rmt_tx_config_s tx_config; /* RMT TX parameter */
      struct rmt_rx_config_s rx_config; /* RMT RX parameter */
    };
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rmt_module_enable(void);
static int rmt_rx_start(rmt_channel_t channel, bool rx_idx_rst);
static int rmt_tx_start(rmt_channel_t channel, bool tx_idx_rst);
static int rmt_set_tx_loop_mode(rmt_channel_t channel, bool loop_en);
static int rmt_set_tx_thr_intr_en(rmt_channel_t channel, bool en,
                                  uint16_t evt_thresh);
static int rmt_set_gpio(rmt_channel_t channel, rmt_mode_t mode,
                        gpio_num_t gpio_num, bool invert_signal);
static bool rmt_is_channel_number_valid(rmt_channel_t channel, uint8_t mode);
static int rmt_internal_config(rmt_dev_t *dev,
                               const struct rmt_channel_config_s *rmt_param);
static int rmt_config(const struct rmt_channel_config_s *rmt_param);
static void rmt_fill_memory(rmt_channel_t channel, const rmt_item32_t *item,
                            uint16_t item_num, uint16_t mem_offset);
static int rmt_isr_register(int (*fn)(int, void *, void *), void *arg,
                            int intr_alloc_flags);
static int rmt_driver_isr_default(int irq, void *context, void *arg);
static int rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size,
                              int intr_alloc_flags);
static int rmt_write_items(rmt_channel_t channel,
                           const rmt_item32_t *rmt_item,
                           int item_num,
                           bool wait_tx_done);
static ssize_t esp_rmt_read(struct rmt_dev_s *dev, char *buffer,
                              size_t buflen);
static ssize_t esp_rmt_write(struct rmt_dev_s *dev,
                             const char *buffer,
                             size_t buflen);
static struct rmt_dev_s
    *esp_rmtinitialize(struct rmt_channel_config_s config);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rmt_ops_s g_rmtops =
{
  .read = esp_rmt_read,
  .write = esp_rmt_write,
};

static struct rmt_dev_common_s g_rmtdev_common =
{
  .hal.regs = &RMT,
  .rmt_driver_isr_lock = NXRMUTEX_INITIALIZER,
  .rmt_driver_channels = 0,
  .rmt_module_enabled = false,
  .synchro_channel_mask = 0
};

static struct rmt_obj_s *p_rmt_obj[RMT_CHANNEL_MAX];

#ifdef CONFIG_RMT_LOOP_TEST_MODE
static rmt_channel_t g_tx_channel = RMT_CHANNEL_MAX;
static rmt_channel_t g_rx_channel = RMT_CHANNEL_MAX;
#endif

#if SOC_RMT_CHANNEL_CLK_INDEPENDENT
uint32_t g_rmt_source_clock_hz[RMT_CHANNEL_MAX];
#else
uint32_t g_rmt_source_clock_hz;
#endif

/* RMTMEM address is declared in <target>.peripherals.ld */

extern rmt_mem_t RMTMEM;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmt_module_enable
 *
 * Description:
 *   This function enables the RMT (Remote Control) module if it's not
 *   already enabled.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void rmt_module_enable(void)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);

  if (g_rmtdev_common.rmt_module_enabled == false)
    {
      periph_module_reset(rmt_periph_signals.groups[0].module);
      periph_module_enable(rmt_periph_signals.groups[0].module);
      g_rmtdev_common.rmt_module_enabled = true;
    }

  spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);
}

/****************************************************************************
 * Name: rmt_set_rx_thr_intr_en
 *
 * Description:
 *   This function enables or disables the RMT RX threshold interrupt. When
 *   the number of received items reaches the threshold, an interrupt is
 *   triggered if this feature is enabled.
 *
 * Input Parameters:
 *   channel    - The RMT channel.
 *   en         - Enable (true) or disable (false) the RX threshold int.
 *   evt_thresh - The number of received items that triggers the interrupt.
 *
 * Returned Value:
 *   Returns 0 on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef SOC_RMT_SUPPORT_RX_PINGPONG
static int rmt_set_rx_thr_intr_en(rmt_channel_t channel, bool en,
                                  uint16_t evt_thresh)
{
  irqstate_t flags;
  uint32_t mask;

  DEBUGASSERT(RMT_IS_RX_CHANNEL(channel) && channel < RMT_CHANNEL_MAX);

  if (en)
    {
      uint32_t item_block_len =
          rmt_ll_rx_get_mem_blocks(g_rmtdev_common.hal.regs,
                                   RMT_DECODE_RX_CHANNEL(channel)) *
          RMT_MEM_ITEM_NUM;

      if (evt_thresh >= item_block_len)
        {
          rmterr("Invalid threshold value %d\n", evt_thresh);
          return -EINVAL;
        }

      flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
      rmt_ll_rx_set_limit(g_rmtdev_common.hal.regs,
                          RMT_DECODE_RX_CHANNEL(channel), evt_thresh);
      mask = RMT_LL_EVENT_RX_THRES(RMT_DECODE_RX_CHANNEL(channel));
      rmt_ll_enable_interrupt(g_rmtdev_common.hal.regs, mask, true);
      spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);
    }
  else
    {
      flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
      mask = RMT_LL_EVENT_RX_THRES(RMT_DECODE_RX_CHANNEL(channel));
      rmt_ll_enable_interrupt(g_rmtdev_common.hal.regs, mask, false);
      spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: rmt_rx_start
 *
 * Description:
 *   This function starts the RMT module in receiving mode for a specific
 *   channel.
 *
 * Input Parameters:
 *   channel    - The RMT peripheral channel number.
 *   rx_idx_rst - If true, the RX index for the channel is reset, which means
 *                the receiving process will start from the beginning of the
 *                RMT memory block.
 *
 * Returned Value:
 *   Returns OK on successful start of the RMT module in receiving mode; a
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rmt_rx_start(rmt_channel_t channel, bool rx_idx_rst)
{
  irqstate_t flags;
  rmt_channel_t ch = RMT_DECODE_RX_CHANNEL(channel);
#ifdef SOC_RMT_SUPPORT_RX_PINGPONG
  const uint32_t item_block_len =
    rmt_ll_rx_get_mem_blocks(g_rmtdev_common.hal.regs, ch) *
    RMT_MEM_ITEM_NUM;
#endif

  DEBUGASSERT(RMT_IS_RX_CHANNEL(channel));

  flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);

  rmt_ll_rx_enable(g_rmtdev_common.hal.regs, ch, false);
  if (rx_idx_rst)
    {
      rmt_ll_rx_reset_pointer(g_rmtdev_common.hal.regs, ch);
    }

  rmt_ll_clear_interrupt_status(g_rmtdev_common.hal.regs,
                                RMT_LL_EVENT_RX_DONE(ch));
  rmt_ll_enable_interrupt(g_rmtdev_common.hal.regs,
                          RMT_LL_EVENT_RX_DONE(ch), true);

#ifdef SOC_RMT_SUPPORT_RX_PINGPONG
  p_rmt_obj[channel]->rx_item_start_idx = 0;
  p_rmt_obj[channel]->rx_item_len = 0;
  rmt_set_rx_thr_intr_en(channel, true, item_block_len / 2);
#endif

  rmt_ll_rx_enable(g_rmtdev_common.hal.regs, ch, true);

  spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);

  return OK;
}

/****************************************************************************
 * Name: rmt_tx_start
 *
 * Description:
 *   This function starts sending RMT items from the specific channel.
 *
 * Input Parameters:
 *   channel    - The RMT peripheral channel number.
 *   tx_idx_rst - Set it true to reset memory index for TX.
 *
 * Returned Value:
 *   Returns OK on successful start of transmission.
 *
 ****************************************************************************/

static int rmt_tx_start(rmt_channel_t channel, bool tx_idx_rst)
{
  irqstate_t flags;

  DEBUGASSERT(RMT_IS_TX_CHANNEL(channel));

  flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
  if (tx_idx_rst)
    {
      rmt_ll_tx_reset_pointer(g_rmtdev_common.hal.regs, channel);
    }

  rmt_ll_clear_interrupt_status(g_rmtdev_common.hal.regs,
                                RMT_LL_EVENT_TX_DONE(channel));

  /* Enable tx end interrupt in non-loop mode */

  if (!rmt_ll_tx_is_loop_enabled(g_rmtdev_common.hal.regs, channel))
    {
      rmt_ll_enable_interrupt(g_rmtdev_common.hal.regs,
                              RMT_LL_EVENT_TX_DONE(channel), true);
    }
  else
    {
#if SOC_RMT_SUPPORT_TX_LOOP_COUNT
      rmt_ll_tx_reset_loop_count(g_rmtdev_common.hal.regs, channel);
      rmt_ll_tx_enable_loop_count(g_rmtdev_common.hal.regs, channel, true);
      rmt_ll_clear_interrupt_status(g_rmtdev_common.hal.regs,
                                    RMT_LL_EVENT_TX_LOOP_END(channel));
      rmt_ll_enable_interrupt(g_rmtdev_common.hal.regs,
                              RMT_LL_EVENT_TX_LOOP_END(channel), true);
#endif
    }

  rmt_ll_tx_start(g_rmtdev_common.hal.regs, channel);
  spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);

  return OK;
}

/****************************************************************************
 * Name: rmt_set_tx_loop_mode
 *
 * Description:
 *   This function enables or disables the loop mode for RMT transmission on
 *   the specified channel. The loop mode, when enabled, allows the RMT
 *   transmitter to continuously send items.
 *
 * Input Parameters:
 *   channel - The RMT peripheral channel number.
 *   loop_en - A boolean indicating whether to enable (true) or disable
 *             (false) the loop mode.
 *
 * Returned Value:
 *   Returns OK on successful setting of the loop mode.
 *
 ****************************************************************************/

static int rmt_set_tx_loop_mode(rmt_channel_t channel, bool loop_en)
{
  irqstate_t flags;

  DEBUGASSERT(RMT_IS_TX_CHANNEL(channel));

  flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
  rmt_ll_tx_enable_loop(g_rmtdev_common.hal.regs, channel, loop_en);
  spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);

  return OK;
}

/****************************************************************************
 * Name: rmt_set_tx_thr_intr_en
 *
 * Description:
 *   This function enables or disables the RMT TX threshold interrupt for the
 *   specified channel. The threshold is set to trigger an interrupt when the
 *   number of transmitted items reaches the specified value.
 *
 * Input Parameters:
 *   channel    - The RMT peripheral channel number.
 *   en         - A boolean indicating whether to enable (true) or disable
 *                (false) the TX threshold interrupt.
 *   evt_thresh - The number of transmitted items at which to trigger the
 *                interrupt.
 *
 * Returned Value:
 *   Returns OK on successful setting of the interrupt.
 *
 ****************************************************************************/

static int rmt_set_tx_thr_intr_en(rmt_channel_t channel, bool en,
                                  uint16_t evt_thresh)
{
  irqstate_t flags;

  DEBUGASSERT(RMT_IS_TX_CHANNEL(channel));

  if (en)
    {
      uint32_t item_block_len =
          rmt_ll_tx_get_mem_blocks(g_rmtdev_common.hal.regs, channel) * \
          RMT_MEM_ITEM_NUM;

      DEBUGASSERT(evt_thresh <= item_block_len);

      flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
      rmt_ll_tx_set_limit(g_rmtdev_common.hal.regs, channel, evt_thresh);
      rmt_ll_enable_interrupt(g_rmtdev_common.hal.regs,
                              RMT_LL_EVENT_TX_THRES(channel), true);
      spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);
    }
  else
    {
      flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
      rmt_ll_enable_interrupt(g_rmtdev_common.hal.regs,
                              RMT_LL_EVENT_TX_THRES(channel), false);
      spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);
    }

  return OK;
}

/****************************************************************************
 * Name: rmt_set_gpio
 *
 * Description:
 *   This function configures the GPIO for the specified RMT (Remote Control)
 *   channel and mode. It sets the GPIO to the appropriate input or output
 *   function based on the mode, and configures the signal inversion if
 *   necessary.
 *
 * Input Parameters:
 *   channel       - The RMT peripheral channel number.
 *   mode          - The mode of operation for the RMT channel (RMT_MODE_TX
 *                   for transmission, RMT_MODE_RX for reception).
 *   gpio_num      - The GPIO number to configure for the RMT channel.
 *   invert_signal - A boolean indicating whether to invert the signal.
 *
 * Returned Value:
 *   Returns OK on successful configuration of the GPIO.
 *
 ****************************************************************************/

static int rmt_set_gpio(rmt_channel_t channel, rmt_mode_t mode,
                        gpio_num_t gpio_num, bool invert_signal)
{
  int ret;

  DEBUGASSERT(channel < RMT_CHANNEL_MAX);
  DEBUGASSERT(mode < RMT_MODE_MAX);
  DEBUGASSERT((GPIO_IS_VALID_GPIO(gpio_num) && (mode == RMT_MODE_RX)) ||
              (GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) &&
               (mode == RMT_MODE_TX)));

  if (mode == RMT_MODE_TX)
    {
      DEBUGASSERT(RMT_IS_TX_CHANNEL(channel));
      esp_configgpio(gpio_num, GPIO_OUT_FUNC);
      esp_gpio_matrix_out(
        gpio_num,
        rmt_periph_signals.groups[0].channels[channel].tx_sig,
        invert_signal, 0);
    }
  else
    {
      DEBUGASSERT(RMT_IS_RX_CHANNEL(channel));
      esp_configgpio(gpio_num, GPIO_IN_FUNC);
      esp_gpio_matrix_in(
        gpio_num,
        rmt_periph_signals.groups[0].channels[channel].rx_sig,
        invert_signal);
    }

  return OK;
}

/****************************************************************************
 * Name: rmt_is_channel_number_valid
 *
 * Description:
 *   This function checks if the provided RMT channel number is valid for the
 *   specified mode (TX or RX). For RX mode, it checks if the channel number
 *   is within the range of valid RX channels and less than the maximum
 *   channel number. For TX mode, it checks if the channel number is a valid
 *   TX channel.
 *
 * Input Parameters:
 *   channel - The RMT peripheral channel number.
 *   mode    - The mode of operation for the RMT channel (RMT_MODE_TX for
 *             transmission, RMT_MODE_RX for reception).
 *
 * Returned Value:
 *   Returns true if the channel number is valid, false otherwise.
 *
 ****************************************************************************/

static bool rmt_is_channel_number_valid(rmt_channel_t channel, uint8_t mode)
{
  if (mode == RMT_MODE_RX)
    {
      return RMT_IS_RX_CHANNEL(channel) && (channel < RMT_CHANNEL_MAX);
    }

  return (channel >= 0) && RMT_IS_TX_CHANNEL(channel);
}

/****************************************************************************
 * Name: rmt_internal_config
 *
 * Description:
 *   This function configures the RMT peripheral with provided parameters.
 *   It sets the mode (TX or RX), channel, GPIO number, memory block number,
 *   clock divider, carrier frequency, and carrier enable flag. It also
 *   configures the clock source, memory access, idle level, carrier
 *   modulation, and other settings based on the mode and parameters.
 *
 * Input Parameters:
 *   dev       - Pointer to the RMT peripheral device structure.
 *   rmt_param - Pointer to the structure containing the RMT channel
 *               configuration parameters.
 *
 * Returned Value:
 *   Returns OK on successful configuration of the RMT peripheral.
 *
 ****************************************************************************/

static int rmt_internal_config(rmt_dev_t *dev,
                               const struct rmt_channel_config_s *rmt_param)
{
  uint8_t mode = rmt_param->rmt_mode;
  uint8_t channel = rmt_param->channel;
  uint8_t gpio_num = rmt_param->gpio_num;
  uint8_t mem_cnt = rmt_param->mem_block_num;
  uint8_t clk_div = rmt_param->clk_div;
  uint32_t carrier_freq_hz = rmt_param->tx_config.carrier_freq_hz;
  bool carrier_en = rmt_param->tx_config.carrier_en;
  uint32_t rmt_source_clk_hz;
  irqstate_t flags;

  if (!rmt_is_channel_number_valid(channel, mode))
    {
      rmterr("Invalid channel number %u for %s mode!",
             channel, mode == RMT_MODE_TX ? "transmitter" : "receiver");
      return -EINVAL;
    }

  DEBUGASSERT(mem_cnt + channel <= SOC_RMT_CHANNELS_PER_GROUP &&
              mem_cnt > 0);
  DEBUGASSERT(clk_div > 0);

  if (mode == RMT_MODE_TX && carrier_en && carrier_freq_hz <= 0)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);

  rmt_ll_enable_mem_access_nonfifo(dev, true);

  if (rmt_param->flags & RMT_CHANNEL_FLAGS_AWARE_DFS)
    {
#if SOC_RMT_SUPPORT_XTAL

      /* clock src: XTAL_CLK */

      esp_clk_tree_src_get_freq_hz((soc_module_clk_t)RMT_BASECLK_XTAL,
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                   &rmt_source_clk_hz);
      rmt_ll_set_group_clock_src(dev, channel,
                                 (rmt_clock_source_t)RMT_BASECLK_XTAL,
                                 1, 0, 0);
#elif SOC_RMT_SUPPORT_REF_TICK

      /* clock src: REF_CLK */

      esp_clk_tree_src_get_freq_hz((soc_module_clk_t)RMT_BASECLK_REF,
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                   &rmt_source_clk_hz);
      rmt_ll_set_group_clock_src(dev, channel,
                                 (rmt_clock_source_t)RMT_BASECLK_REF,
                                 1, 0, 0);
#else
#error "No clock source is aware of DFS"
#endif
    }
  else
    {
      /* fallback to use default clock source */

      esp_clk_tree_src_get_freq_hz((soc_module_clk_t)RMT_BASECLK_DEFAULT,
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                   &rmt_source_clk_hz);
      rmt_ll_set_group_clock_src(dev, channel,
                                 (rmt_clock_source_t)RMT_BASECLK_DEFAULT,
                                 1, 0, 0);
    }

  spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);

#if SOC_RMT_CHANNEL_CLK_INDEPENDENT
  g_rmt_source_clock_hz[channel] = rmt_source_clk_hz;
#else
  if (g_rmt_source_clock_hz && rmt_source_clk_hz != g_rmt_source_clock_hz)
    {
      rmterr("RMT clock source has been configured to %"PRIu32" by other "
             "channel, now reconfigure it to %"PRIu32"",
             g_rmt_source_clock_hz, rmt_source_clk_hz);
    }

  g_rmt_source_clock_hz = rmt_source_clk_hz;
#endif
  rmtinfo("rmt_source_clk_hz: %"PRIu32, rmt_source_clk_hz);

  if (mode == RMT_MODE_TX)
    {
      uint16_t carrier_duty_percent =
                  rmt_param->tx_config.carrier_duty_percent;
      uint8_t carrier_level = rmt_param->tx_config.carrier_level;
      uint8_t idle_level = rmt_param->tx_config.idle_level;

      flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
      rmt_ll_tx_set_channel_clock_div(dev, channel, clk_div);
      rmt_ll_tx_set_mem_blocks(dev, channel, mem_cnt);
      rmt_ll_tx_reset_pointer(dev, channel);
      rmt_ll_tx_enable_loop(dev, channel, rmt_param->tx_config.loop_en);
#if SOC_RMT_SUPPORT_TX_LOOP_COUNT
      if (rmt_param->tx_config.loop_en)
        {
          rmt_ll_tx_set_loop_count(dev, channel,
                                   rmt_param->tx_config.loop_count);
        }
#endif

      /* always enable tx ping-pong */

      rmt_ll_tx_enable_wrap(dev, channel, true);

      /* Set idle level */

      rmt_ll_tx_fix_idle_level(dev, channel, idle_level,
                               rmt_param->tx_config.idle_output_en);

      /* Set carrier */

      rmt_ll_tx_enable_carrier_modulation(dev, channel, carrier_en);
      if (carrier_en)
        {
          uint32_t duty_div;
          uint32_t duty_h;
          uint32_t duty_l;
          duty_div = rmt_source_clk_hz / carrier_freq_hz;
          duty_h = duty_div * carrier_duty_percent / 100;
          duty_l = duty_div - duty_h;
          rmt_ll_tx_set_carrier_level(dev, channel, carrier_level);
          rmt_ll_tx_set_carrier_high_low_ticks(dev, channel, duty_h, duty_l);
        }
      else
        {
          rmt_ll_tx_set_carrier_level(dev, channel, 0);
        }

      spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);

      rmtinfo("Rmt Tx Channel %u|Gpio %u|Sclk_Hz %"PRIu32"|Div %u|Carrier_Hz"
              " %"PRIu32"|Duty %u", channel, gpio_num, rmt_source_clk_hz,
              clk_div, carrier_freq_hz, carrier_duty_percent);
    }
  else if (RMT_MODE_RX == mode)
    {
      uint8_t filter_cnt = rmt_param->rx_config.filter_ticks_thresh;
      uint16_t threshold = rmt_param->rx_config.idle_threshold;

      flags = spin_lock_irqsave(&g_rmtdev_common.rmt_spinlock);
      rmt_ll_rx_set_channel_clock_div(dev, RMT_DECODE_RX_CHANNEL(channel),
                                      clk_div);
      rmt_ll_rx_set_mem_blocks(dev, RMT_DECODE_RX_CHANNEL(channel), mem_cnt);
      rmt_ll_rx_reset_pointer(dev, RMT_DECODE_RX_CHANNEL(channel));
      rmt_ll_rx_set_mem_owner(dev, RMT_DECODE_RX_CHANNEL(channel),
                              RMT_LL_MEM_OWNER_HW);

      /* Set idle threshold */

      rmt_ll_rx_set_idle_thres(dev, RMT_DECODE_RX_CHANNEL(channel),
                               threshold);

      /* Set RX filter */

      rmt_ll_rx_set_filter_thres(dev, RMT_DECODE_RX_CHANNEL(channel),
                                 filter_cnt);
      rmt_ll_rx_enable_filter(dev, RMT_DECODE_RX_CHANNEL(channel),
                              rmt_param->rx_config.filter_en);

#ifdef SOC_RMT_SUPPORT_RX_PINGPONG

      /* always enable rx ping-pong */

      rmt_ll_rx_enable_wrap(dev, RMT_DECODE_RX_CHANNEL(channel), true);
#endif

#if SOC_RMT_SUPPORT_RX_DEMODULATION
      rmt_ll_rx_enable_carrier_demodulation(dev,
                                            RMT_DECODE_RX_CHANNEL(channel),
                                            rmt_param->rx_config.rm_carrier);
      if (rmt_param->rx_config.rm_carrier)
        {
          uint32_t duty_total;
          uint32_t duty_high;
          uint32_t ch_clk_div =
            rmt_ll_rx_get_channel_clock_div(dev,
                                            RMT_DECODE_RX_CHANNEL(channel));
          duty_total = rmt_source_clk_hz / \
                       ch_clk_div / \
                       rmt_param->rx_config.carrier_freq_hz;
          duty_high = duty_total *
                      rmt_param->rx_config.carrier_duty_percent / 100;

          /* there could be residual in timing the carrier pulse, so double
           * enlarge the theoretical value.
           */

          rmt_ll_rx_set_carrier_high_low_ticks(
              dev, RMT_DECODE_RX_CHANNEL(channel), duty_high * 2,
              (duty_total - duty_high) * 2);
          rmt_ll_rx_set_carrier_level(dev, RMT_DECODE_RX_CHANNEL(channel),
                                      rmt_param->rx_config.carrier_level);
        }
#endif

      spin_unlock_irqrestore(&g_rmtdev_common.rmt_spinlock, flags);

      rmtinfo("Rmt Rx Channel %u|Gpio %u|Sclk_Hz %"PRIu32"|Div %u|Thresold "
              "%u|Filter %u", channel, gpio_num, rmt_source_clk_hz, clk_div,
              threshold, filter_cnt);
    }

  return OK;
}

/****************************************************************************
 * Name: rmt_config
 *
 * Description:
 *   This function configures the RMT channel with the provided parameters.
 *   It enables the RMT module, sets the GPIO for the RMT channel, and
 *   configures the RMT peripheral using the internal configuration function.
 *
 * Input Parameters:
 *   rmt_param - Pointer to the structure containing the RMT channel
 *               configuration parameters.
 *
 * Returned Value:
 *   Returns OK on successful configuration of the RMT channel; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int rmt_config(const struct rmt_channel_config_s *rmt_param)
{
  int ret = ERROR;

  rmt_module_enable();

  rmt_set_gpio(rmt_param->channel, rmt_param->rmt_mode, rmt_param->gpio_num,
               rmt_param->flags & RMT_CHANNEL_FLAGS_INVERT_SIG);

  ret = rmt_internal_config(&RMT, rmt_param);

  return ret;
}

/****************************************************************************
 * Name: rmt_fill_memory
 *
 * Description:
 *   This function fills the RMT memory with the provided items. It copies
 *   the items from the source to the RMT memory for the specified channel,
 *   starting at the specified memory offset.
 *
 * Input Parameters:
 *   channel    - The RMT peripheral channel number.
 *   item       - Pointer to the items to be copied to the RMT memory.
 *   item_num   - The number of items to be copied.
 *   mem_offset - The memory offset at which to start copying.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR rmt_fill_memory(rmt_channel_t channel,
                                      const rmt_item32_t *item,
                                      uint16_t item_num,
                                      uint16_t mem_offset)
{
  uint32_t *from = (uint32_t *)item;
  volatile uint32_t *to =
      (volatile uint32_t *)&RMTMEM.chan[channel].data32[0].val;

  to += mem_offset;

  while (item_num--)
    {
      *to++ = *from++;
    }
}

/****************************************************************************
 * Name: rmt_isr_register
 *
 * Description:
 *   This function registers an interrupt service routine (ISR) for the RMT
 *   peripheral. It allocates a CPU interrupt, attaches the ISR to the
 *   interrupt, and returns the status of the operation.
 *
 * Input Parameters:
 *   fn               - Pointer to the ISR function.
 *   arg              - Pointer to the argument to be passed to the ISR.
 *   intr_alloc_flags - Flags for the interrupt allocation.
 *
 * Returned Value:
 *   Returns OK on successful registration of the ISR; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

static int rmt_isr_register(int (*fn)(int, void *, void *), void *arg,
                            int intr_alloc_flags)
{
  int cpuint;
  int ret;
  int cpu = this_cpu();

  DEBUGASSERT(fn);
  DEBUGASSERT(g_rmtdev_common.rmt_driver_channels == 0);

  cpuint = esp_setup_irq(
#ifndef CONFIG_ARCH_CHIP_ESP32S2
    cpu,
#endif
    rmt_periph_signals.groups[0].irq, 1, ESP_CPUINT_LEVEL);
  if (cpuint < 0)
    {
      rmterr("Failed to allocate a CPU interrupt.\n");
      return -ENOMEM;
    }

  ret = irq_attach(rmt_periph_signals.groups[0].irq + XTENSA_IRQ_FIRSTPERIPH,
                   fn, &g_rmtdev_common.hal);
  if (ret < 0)
    {
      rmterr("Couldn't attach IRQ to handler.\n");
      esp_teardown_irq(
#ifndef CONFIG_ARCH_CHIP_ESP32S2
        cpu,
#endif
        rmt_periph_signals.groups[0].irq, cpuint);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: rmt_driver_isr_default
 *
 * Description:
 *   This function is the default interrupt service routine (ISR) for the RMT
 *   peripheral. It handles TX end, TX threshold, RX end, RX threshold, loop
 *   count, RX error, and TX error interrupts. For each interrupt type, it
 *   checks the status, clears the interrupt, and performs the appropriate
 *   actions based on the RMT object associated with the channel.
 *
 * Input Parameters:
 *   irq     - The interrupt request number.
 *   context - Pointer to the interrupt context.
 *   arg     - Pointer to the argument to be passed to the ISR.
 *
 * Returned Value:
 *   Returns OK after handling all active interrupts.
 *
 ****************************************************************************/

static int IRAM_ATTR rmt_driver_isr_default(int irq, void *context,
                                            void *arg)
{
  uint32_t status = 0;
  rmt_item32_t *addr = NULL;
  uint8_t channel = 0;
  rmt_hal_context_t *hal = (rmt_hal_context_t *)arg;

  /* Tx end interrupt */

  status = rmt_ll_get_tx_end_interrupt_status(hal->regs);
  while (status)
    {
      channel = __builtin_ffs(status) - 1;
      status &= ~(1 << channel);
      rmt_obj_t *p_rmt = p_rmt_obj[channel];
      if (p_rmt)
        {
          nxsem_post(&p_rmt->tx_sem);
          rmt_ll_tx_reset_pointer(g_rmtdev_common.hal.regs, channel);
          p_rmt->tx_data = NULL;
          p_rmt->tx_len_rem = 0;
          p_rmt->tx_offset = 0;
          p_rmt->tx_sub_len = 0;
          p_rmt->sample_cur = NULL;
        }

      rmt_ll_clear_interrupt_status(hal->regs,
                                    RMT_LL_EVENT_TX_DONE(channel));
    }

  /* Tx thres interrupt */

  status = rmt_ll_get_tx_thres_interrupt_status(hal->regs);
  while (status)
    {
      channel = __builtin_ffs(status) - 1;
      status &= ~(1 << channel);
      rmt_obj_t *p_rmt = p_rmt_obj[channel];
      if (p_rmt)
        {
          const rmt_item32_t *pdata = p_rmt->tx_data;
          size_t len_rem = p_rmt->tx_len_rem;
          rmt_idle_level_t idle_level =
              rmt_ll_tx_get_idle_level(hal->regs, channel);
          rmt_item32_t stop_data = (rmt_item32_t)
            {
              .level0 = idle_level,
              .duration0 = 0,
            };

          if (len_rem >= p_rmt->tx_sub_len)
            {
              rmt_fill_memory(channel, pdata, p_rmt->tx_sub_len,
                              p_rmt->tx_offset);
              p_rmt->tx_data += p_rmt->tx_sub_len;
              p_rmt->tx_len_rem -= p_rmt->tx_sub_len;
            }
          else if (len_rem == 0)
            {
              rmt_fill_memory(channel, &stop_data, 1, p_rmt->tx_offset);
            }
          else
            {
              rmt_fill_memory(channel, pdata, len_rem, p_rmt->tx_offset);
              rmt_fill_memory(channel, &stop_data, 1,
                              p_rmt->tx_offset + len_rem);
              p_rmt->tx_data += len_rem;
              p_rmt->tx_len_rem -= len_rem;
            }

          if (p_rmt->tx_offset == 0)
            {
              p_rmt->tx_offset = p_rmt->tx_sub_len;
            }
          else
            {
              p_rmt->tx_offset = 0;
            }
        }

      rmt_ll_clear_interrupt_status(hal->regs,
                                    RMT_LL_EVENT_TX_THRES(channel));
    }

  /* Rx end interrupt */

  status = rmt_ll_get_rx_end_interrupt_status(hal->regs);
  while (status)
    {
      channel = __builtin_ffs(status) - 1;
      status &= ~(1 << channel);
      rmt_obj_t *p_rmt = p_rmt_obj[RMT_ENCODE_RX_CHANNEL(channel)];
      if (p_rmt)
        {
          int item_len;
          rmt_ll_rx_enable(g_rmtdev_common.hal.regs, channel, false);
          item_len =
            rmt_ll_rx_get_memory_writer_offset(g_rmtdev_common.hal.regs,
                                               channel);
          rmt_ll_rx_set_mem_owner(g_rmtdev_common.hal.regs, channel,
                                  RMT_LL_MEM_OWNER_SW);
          if (circbuf_is_init(&p_rmt->rx_buf))
            {
              int bytes;

              addr = (rmt_item32_t *)
                RMTMEM.chan[RMT_ENCODE_RX_CHANNEL(channel)].data32;
#ifdef SOC_RMT_SUPPORT_RX_PINGPONG
              if (item_len > p_rmt->rx_item_start_idx)
                {
                  item_len = item_len - p_rmt->rx_item_start_idx;
                }

              /* Check for RX buffer max length */

              if ((p_rmt->rx_item_len + item_len) > \
                  (p_rmt->rx_item_buf_size / 4))
                {
                  int remaining_len = (p_rmt->rx_item_buf_size / 4) - \
                                      p_rmt->rx_item_len;
                  rmterr("ERROR: RX buffer too small: %d items dropped\n",
                         item_len - remaining_len);
                  item_len = remaining_len;
                }

              memcpy((void *)(p_rmt->rx_item_buf + p_rmt->rx_item_len),
                     (void *)(addr + p_rmt->rx_item_start_idx),
                     item_len * 4);
              p_rmt->rx_item_len += item_len;
              bytes = circbuf_write(&p_rmt->rx_buf,
                                    (void *)(p_rmt->rx_item_buf),
                                    p_rmt->rx_item_len * 4);
#else
              bytes = circbuf_write(&p_rmt->rx_buf, (void *)addr,
                                    item_len * 4);
#endif
              nxsem_post(&p_rmt->rx_sem);
              if (bytes < (item_len * 4))
                {
                  rmterr("RMT RX BUFFER FULL");
                }
            }
          else
            {
              rmterr("RMT RX BUFFER ERROR");
            }

#ifdef SOC_RMT_SUPPORT_RX_PINGPONG
          p_rmt->rx_item_start_idx = 0;
          p_rmt->rx_item_len = 0;
          memset((void *)p_rmt->rx_item_buf, 0, p_rmt->rx_item_buf_size);
#endif
          rmt_ll_rx_reset_pointer(g_rmtdev_common.hal.regs, channel);
          rmt_ll_rx_set_mem_owner(g_rmtdev_common.hal.regs, channel,
                                  RMT_LL_MEM_OWNER_HW);
          rmt_ll_rx_enable(g_rmtdev_common.hal.regs, channel, true);
        }

      rmt_ll_clear_interrupt_status(hal->regs,
                                    RMT_LL_EVENT_RX_DONE(channel));
    }

#ifdef SOC_RMT_SUPPORT_RX_PINGPONG

  /* Rx thres interrupt */

  status = rmt_ll_get_rx_thres_interrupt_status(hal->regs);
  while (status)
    {
      int mem_item_size;
      int rx_thres_lim;
      int item_len;

      channel = __builtin_ffs(status) - 1;
      status &= ~(1 << channel);
      rmt_obj_t *p_rmt = p_rmt_obj[RMT_ENCODE_RX_CHANNEL(channel)];
      mem_item_size = rmt_ll_rx_get_mem_blocks(g_rmtdev_common.hal.regs,
                                               channel) * RMT_MEM_ITEM_NUM;
      rx_thres_lim = rmt_ll_rx_get_limit(g_rmtdev_common.hal.regs, channel);
      item_len = (p_rmt->rx_item_start_idx == 0) ? rx_thres_lim : \
                 (mem_item_size - rx_thres_lim);
      if ((p_rmt->rx_item_len + item_len) > (p_rmt->rx_item_buf_size / 4))
        {
          int remaining_len = (p_rmt->rx_item_buf_size / 4) - \
                              p_rmt->rx_item_len;
          rmterr("ERROR: RX buffer too small!\n");
          item_len = remaining_len;
        }

      rmt_ll_rx_set_mem_owner(g_rmtdev_common.hal.regs, channel,
                                  RMT_LL_MEM_OWNER_SW);
      memcpy(
        (void *)(p_rmt->rx_item_buf + p_rmt->rx_item_len),
        (void *)(RMTMEM.chan[RMT_ENCODE_RX_CHANNEL(channel)].data32 \
        + p_rmt->rx_item_start_idx), item_len * 4);
      rmt_ll_rx_set_mem_owner(g_rmtdev_common.hal.regs, channel,
                              RMT_LL_MEM_OWNER_HW);
      p_rmt->rx_item_len += item_len;
      p_rmt->rx_item_start_idx += item_len;
      if (p_rmt->rx_item_start_idx >= mem_item_size)
        {
          p_rmt->rx_item_start_idx = 0;
        }

      rmt_ll_clear_interrupt_status(hal->regs,
                                    RMT_LL_EVENT_RX_THRES(channel));
    }
#endif

#if SOC_RMT_SUPPORT_TX_LOOP_COUNT

  /* loop count interrupt */

  status = rmt_ll_get_tx_loop_interrupt_status(hal->regs);
  while (status)
    {
      channel = __builtin_ffs(status) - 1;
      status &= ~(1 << channel);
      rmt_obj_t *p_rmt = p_rmt_obj[channel];
      if (p_rmt)
        {
          if (p_rmt->loop_autostop)
            {
#ifndef SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP

              /* hardware doesn't support automatically stop output so driver
               * should stop output here (possibility already overshotted
               * several us).
               */

              rmt_ll_tx_stop(g_rmtdev_common.hal.regs, channel);
              rmt_ll_tx_reset_pointer(g_rmtdev_common.hal.regs, channel);
#endif
            }

          nxsem_post(&p_rmt->tx_sem);
        }

      rmt_ll_clear_interrupt_status(hal->regs,
                                    RMT_LL_EVENT_TX_LOOP_END(channel));
    }
#endif

  /* RX Err interrupt */

  status = rmt_ll_get_rx_err_interrupt_status(hal->regs);
  while (status)
    {
      channel = __builtin_ffs(status) - 1;
      status &= ~(1 << channel);
      rmt_obj_t *p_rmt = p_rmt_obj[RMT_ENCODE_RX_CHANNEL(channel)];
      if (p_rmt)
        {
          /* Reset the receiver's write/read addresses to prevent endless
           * err interrupts.
           */

          rmt_ll_rx_reset_pointer(g_rmtdev_common.hal.regs, channel);
          rmtinfo("RMT RX channel %d error", channel);
          rmtinfo("status: 0x%08x",
                  rmt_ll_rx_get_status_word(g_rmtdev_common.hal.regs,
                                            channel));
        }

      rmt_ll_clear_interrupt_status(hal->regs,
                                    RMT_LL_EVENT_RX_ERROR(channel));
    }

  /* TX Err interrupt */

  status = rmt_ll_get_tx_err_interrupt_status(hal->regs);
  while (status)
    {
      channel = __builtin_ffs(status) - 1;
      status &= ~(1 << channel);
      rmt_obj_t *p_rmt = p_rmt_obj[channel];
      if (p_rmt)
        {
          /* Reset the transmitter's write/read addresses to prevent
           * endless err interrupts.
           */

          rmt_ll_tx_reset_pointer(g_rmtdev_common.hal.regs, channel);
          rmtinfo("RMT TX channel %d error", channel);
          rmtinfo("status: 0x%08x",
                  rmt_ll_tx_get_status_word(g_rmtdev_common.hal.regs,
                                            channel));
        }

      rmt_ll_clear_interrupt_status(hal->regs,
                                    RMT_LL_EVENT_TX_ERROR(channel));
    }

  return OK;
}

/****************************************************************************
 * Name: rmt_driver_install
 *
 * Description:
 *   This function installs the RMT driver for a specific channel. It
 *   allocates memory for the RMT object, initializes the object properties,
 *   and sets up the RX buffer if specified. It also registers the default
 *   ISR if this is the first RMT channel using the driver, and resets the
 *   RMT channel.
 *
 * Input Parameters:
 *   channel          - The RMT peripheral channel number.
 *   rx_buf_size      - The size of the RX buffer.
 *   intr_alloc_flags - Flags for the interrupt allocation.
 *
 * Returned Value:
 *   Returns OK on successful installation of the RMT driver; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size,
                              int intr_alloc_flags)
{
  DEBUGASSERT(channel < RMT_CHANNEL_MAX);

  int ret = OK;

  if (p_rmt_obj[channel])
    {
      rmtwarn("RMT driver already installed");
      return ERROR;
    }

#if CONFIG_RINGBUF_PLACE_ISR_FUNCTIONS_INTO_FLASH
  if (intr_alloc_flags & ESP_INTR_FLAG_IRAM)
    {
      rmterr("ringbuf ISR functions in flash, but used in IRAM interrupt");
      return -EINVAL;
    }
#endif

#ifndef CONFIG_SPIRAM_USE_MALLOC
  p_rmt_obj[channel] = kmm_calloc(1, sizeof(rmt_obj_t));
#else
  if (!(intr_alloc_flags & ESP_INTR_FLAG_IRAM))
    {
      p_rmt_obj[channel] = calloc(1, sizeof(rmt_obj_t));
    }
  else
    {
      p_rmt_obj[channel] = kmm_calloc(1, sizeof(rmt_obj_t));
    }
#endif

  if (p_rmt_obj[channel] == NULL)
    {
      rmterr("RMT driver malloc error");
      return -ENOMEM;
    }

  p_rmt_obj[channel]->tx_len_rem = 0;
  p_rmt_obj[channel]->tx_data = NULL;
  p_rmt_obj[channel]->channel = channel;
  p_rmt_obj[channel]->tx_offset = 0;
  p_rmt_obj[channel]->tx_sub_len = 0;
  p_rmt_obj[channel]->wait_done = false;
  p_rmt_obj[channel]->loop_autostop = false;

#ifndef CONFIG_SPIRAM_USE_MALLOC
  nxsem_init(&p_rmt_obj[channel]->tx_sem, 0, 0);
  nxsem_init(&p_rmt_obj[channel]->rx_sem, 0, 0);
#endif

  nxsem_post(&p_rmt_obj[channel]->tx_sem);

  if (!circbuf_is_init(&p_rmt_obj[channel]->rx_buf) && rx_buf_size > 0)
    {
      circbuf_init(&p_rmt_obj[channel]->rx_buf, NULL, rx_buf_size);
    }

#ifdef SOC_RMT_SUPPORT_RX_PINGPONG
  if (p_rmt_obj[channel]->rx_item_buf == NULL && rx_buf_size > 0)
    {
#  ifndef CONFIG_SPIRAM_USE_MALLOC
      p_rmt_obj[channel]->rx_item_buf = kmm_calloc(1, rx_buf_size);
#  else
      if (!(p_rmt_obj[channel]->intr_alloc_flags & ESP_INTR_FLAG_IRAM))
        {
          p_rmt_obj[channel]->rx_item_buf = calloc(1, rx_buf_size);
        }
      else
        {
          p_rmt_obj[channel]->rx_item_buf = kmm_calloc(1, rx_buf_size);
        }

#  endif
      if (p_rmt_obj[channel]->rx_item_buf == NULL)
        {
          rmterr("RMT malloc fail");
          nxsem_destroy(&p_rmt_obj[channel]->rx_sem);
          return -ENOMEM;
        }

      p_rmt_obj[channel]->rx_item_buf_size = rx_buf_size;
    }
#endif

  nxrmutex_lock(&(g_rmtdev_common.rmt_driver_isr_lock));

  if (g_rmtdev_common.rmt_driver_channels == 0)
    {
      /* first RMT channel using driver */

      ret = rmt_isr_register(rmt_driver_isr_default, &g_rmtdev_common.hal,
                             intr_alloc_flags);
    }

  if (ret == OK)
    {
      g_rmtdev_common.rmt_driver_channels |= BIT(channel);
    }

  nxrmutex_unlock(&(g_rmtdev_common.rmt_driver_isr_lock));

  rmt_module_enable();

  if (RMT_IS_RX_CHANNEL(channel))
    {
      rmt_hal_rx_channel_reset(&g_rmtdev_common.hal,
                               RMT_DECODE_RX_CHANNEL(channel));
    }
  else
    {
      rmt_hal_tx_channel_reset(&g_rmtdev_common.hal, channel);
    }

  return OK;
}

/****************************************************************************
 * Name: rmt_write_items
 *
 * Description:
 *   This function writes items to the RMT memory for a specific channel. It
 *   checks the validity of the parameters, calculates the memory blocks and
 *   item lengths, and fills the memory with the items. If the number of
 *   items is greater than the memory block length, it enables the TX
 *   threshold interrupt and sets up the remaining items to be sent. If the
 *   number of items is less than the memory block length, it fills the
 *   remaining memory with idle level items. It then starts the TX process
 *   and waits for it to finish if specified.
 *
 * Input Parameters:
 *   channel       - The RMT peripheral channel number.
 *   rmt_item      - Pointer to the items to be written to the RMT memory.
 *   item_num      - The number of items to be written.
 *   wait_tx_done  - Flag to indicate whether to wait for the TX process to
 *                   finish.
 *
 * Returned Value:
 *   Returns OK on successful writing of the items to the RMT memory; a
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rmt_write_items(rmt_channel_t channel,
                           const rmt_item32_t *rmt_item,
                           int item_num,
                           bool wait_tx_done)
{
  DEBUGASSERT(RMT_IS_TX_CHANNEL(channel));
  DEBUGASSERT(p_rmt_obj[channel]);
  DEBUGASSERT(rmt_item);
  DEBUGASSERT(item_num > 0);

  uint32_t mem_blocks = rmt_ll_tx_get_mem_blocks(g_rmtdev_common.hal.regs,
                                                 channel);

  DEBUGASSERT(mem_blocks + channel <= SOC_RMT_CHANNELS_PER_GROUP);
#ifdef CONFIG_SPIRAM_USE_MALLOC
  if (p_rmt_obj[channel]->intr_alloc_flags & ESP_INTR_FLAG_IRAM)
    {
      if (!esp_ptr_internal(rmt_item))
        {
          rmterr(RMT_PSRAM_BUFFER_WARN_STR);
          return ESP_ERR_INVALID_ARG;
        }
    }
#endif

  rmt_obj_t *p_rmt = p_rmt_obj[channel];
  int item_block_len = mem_blocks * RMT_MEM_ITEM_NUM;
  int item_sub_len = mem_blocks * RMT_MEM_ITEM_NUM / 2;
  int len_rem = item_num;
  nxsem_wait(&p_rmt->tx_sem);

  /* fill the memory block first */

  if (item_num >= item_block_len)
    {
      rmt_fill_memory(channel, rmt_item, item_block_len, 0);
      len_rem -= item_block_len;
      rmt_set_tx_loop_mode(channel, false);
      rmt_set_tx_thr_intr_en(channel, 1, item_sub_len);
      p_rmt->tx_data = rmt_item + item_block_len;
      p_rmt->tx_len_rem = len_rem;
      p_rmt->tx_offset = 0;
      p_rmt->tx_sub_len = item_sub_len;
    }
  else
    {
      rmt_idle_level_t idle_level;
      rmt_fill_memory(channel, rmt_item, len_rem, 0);
      idle_level = rmt_ll_tx_get_idle_level(g_rmtdev_common.hal.regs,
                                            channel);
      rmt_item32_t stop_data = (rmt_item32_t)
        {
          .level0 = idle_level,
          .duration0 = 0,
        };

      rmt_fill_memory(channel, &stop_data, 1, len_rem);
      p_rmt->tx_len_rem = 0;
    }

  rmt_tx_start(channel, true);
  p_rmt->wait_done = wait_tx_done;
  if (wait_tx_done)
    {
      /* wait loop done */

      if (rmt_ll_tx_is_loop_enabled(g_rmtdev_common.hal.regs, channel))
        {
#if SOC_RMT_SUPPORT_TX_LOOP_COUNT
          nxsem_wait(&p_rmt->tx_sem);
          nxsem_post(&p_rmt->tx_sem);
#endif
        }
      else
        {
          /* wait tx end */

          nxsem_wait(&p_rmt->tx_sem);
          nxsem_post(&p_rmt->tx_sem);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_rmt_read
 *
 * Description:
 *   This function reads data from the RMT device.
 *   It starts the RMT module in receiving mode for a specific channel and
 *   checks for any errors. If an error occurs during the start of the RMT
 *   module, it returns the error code. Please note that this function
 *   starts the receiver, but the actual data is read from the ring buffer
 *   by the upper half driver.
 *
 * Input Parameters:
 *   dev     - Pointer to the RMT device structure.
 *   buffer  - Pointer to the buffer where the read data should be stored.
 *   buflen  - The maximum amount of data to be read.
 *
 * Returned Value:
 *   Returns the number of bytes read from the RMT device; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t esp_rmt_read(struct rmt_dev_s *dev, char *buffer,
                              size_t buflen)
{
  struct rmt_dev_lowerhalf_s *priv = (struct rmt_dev_lowerhalf_s *)dev;
  rmt_mode_t mode = priv->mode;
  int channel = priv->minor;
  int ret;
  ssize_t nread;

  if (mode != RMT_MODE_RX)
    {
      rmterr("ERROR: RMT channel %d is not in RX mode\n", channel);
      return -EINVAL;
    }

  DEBUGASSERT((buflen % 4) == 0);

  if ((buflen / 4) > (CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE / 4))
    {
      rmtwarn("WARN: RMT RX buffer (%d bytes) is smaller than requested "
              "read bytes (%d bytes). A partial read will take place!\n",
              CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE,
              buflen);
    }

#ifndef SOC_RMT_SUPPORT_RX_PINGPONG
  if ((buflen / 4) > RMT_MEM_ITEM_NUM)
    {
      rmtwarn("WARN: RMT RX channel is able to receive up to "
              "%d RMT items (%d bytes)!",
              RMT_MEM_ITEM_NUM, RMT_MEM_ITEM_NUM * 4);
    }
#endif

  ret = rmt_rx_start(channel, true);
  if (ret < 0)
    {
      rmterr("ERROR: rmt_rx_start failed: %d\n", ret);
      return (ssize_t)ret;
    }

  return (ssize_t)ret;
}

/****************************************************************************
 * Name: esp_rmt_write
 *
 * Description:
 *   This function writes data to the RMT memory for a specific channel. It
 *   asserts that the length of the data is a multiple of 4, then calls the
 *   rmt_write_items function to write the items to the RMT memory.
 *
 * Input Parameters:
 *   dev     - Pointer to the RMT device structure.
 *   buffer  - Pointer to the data to be written to the RMT memory.
 *   buflen  - The length of the data to be written.
 *
 * Returned Value:
 *   Returns the number of items written to the RMT memory.
 *
 ****************************************************************************/

static ssize_t esp_rmt_write(struct rmt_dev_s *dev, const char *buffer,
                               size_t buflen)
{
  struct rmt_dev_lowerhalf_s *priv = (struct rmt_dev_lowerhalf_s *)dev;
  rmt_mode_t mode = priv->mode;
  int channel = priv->minor;
  int ret;
  struct timespec timeout;

  if (mode != RMT_MODE_TX)
    {
      rmterr("ERROR: RMT channel %d is not in TX mode\n", channel);
      return -EINVAL;
    }

  DEBUGASSERT((buflen % 4) == 0);

  ret = rmt_write_items(channel, (const rmt_item32_t *)buffer,
                        (buflen / 4), true);

  if (ret < 0)
    {
      rmterr("ERROR: rmt_write_items failed: %d\n", ret);
      return (ssize_t)0;
    }

  return (ssize_t)buflen;
}

/****************************************************************************
 * Name: esp_rmtinitialize
 *
 * Description:
 *   This function initializes the specified RMT (Remote Control) device
 *   with the provided configuration.
 *
 * Input Parameters:
 *   config - A structure containing the configuration settings for the
 *            RMT channel to be initialized.
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the RMT device
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

static struct rmt_dev_s
    *esp_rmtinitialize(struct rmt_channel_config_s config)
{
  struct rmt_dev_lowerhalf_s *priv;
  int ret;
#ifdef CONFIG_RMT_LOOP_TEST_MODE
  uint8_t channel;
#endif

#if (CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE % 4) != 0
#  error "CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE must be a multiple of 4"
#endif

  priv = kmm_zalloc(sizeof(struct rmt_dev_lowerhalf_s));
  if (priv)
    {
      ret = rmt_config(&config);
      if (ret < 0)
        {
          rmterr("ERROR: rmt_config failed: %d\n", ret);
          return NULL;
        }

#ifdef CONFIG_RMT_LOOP_TEST_MODE
      if (config.rmt_mode == RMT_MODE_TX)
        {
          if (g_tx_channel != RMT_CHANNEL_MAX)
            {
              rmterr("ERROR: only one TX channel can be used in loop test "
                     "mode\n");
              PANIC();
            }

          g_tx_channel = config.channel;
        }
      else
        {
          if (g_rx_channel != RMT_CHANNEL_MAX)
            {
              rmterr("ERROR: only one RX channel can be used in loop test "
                     "mode\n");
              PANIC();
            }

          g_rx_channel = config.channel;
        }

      if (g_rx_channel != RMT_CHANNEL_MAX && g_tx_channel != RMT_CHANNEL_MAX)
        {
          esp_configgpio(config.gpio_num, GPIO_OUT_FUNC | GPIO_IN_FUNC);
          esp_gpio_matrix_out(config.gpio_num,
                              RMT_SIG_OUT0_IDX + g_tx_channel,
                              0, 0);
          esp_gpio_matrix_in(config.gpio_num,
                             RMT_SIG_IN0_IDX + g_rx_channel,
                             0);
          rmtwarn("RX channel %d and TX channel %d are used in loop test "
                  "mode\n", g_rx_channel, g_tx_channel);
        }
#endif

      ret = rmt_driver_install(config.channel,
                               config.rmt_mode == RMT_MODE_RX ? \
                               CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE : 0, 0);
      if (ret < 0)
        {
          rmterr("ERROR: rmt_driver_install failed: %d\n", ret);
          return NULL;
        }

      priv->ops = &g_rmtops;
      priv->recvsem = &p_rmt_obj[config.channel]->rx_sem;
      priv->circbuf = &p_rmt_obj[config.channel]->rx_buf;
      priv->minor = config.channel;

      priv->common = &g_rmtdev_common;
      priv->mode = config.rmt_mode;
    }
  else
    {
      rmterr("ERROR: memory allocation failed\n");
      return NULL;
    }

  return (struct rmt_dev_s *)priv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_rmt_tx_init
 *
 * Description:
 *   Initialize the selected RMT device in TX mode
 *
 * Input Parameters:
 *   ch   - The RMT's channel that will be used
 *   pin  - The pin used for the TX channel
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; NULL, otherwise.
 *
 ****************************************************************************/

struct rmt_dev_s *esp_rmt_tx_init(int ch, int pin)
{
  struct rmt_channel_config_s config = RMT_DEFAULT_CONFIG_TX(pin, ch);

  return esp_rmtinitialize(config);
}

/****************************************************************************
 * Name: esp_rmt_rx_init
 *
 * Description:
 *   Initialize the selected RMT device in RC mode
 *
 * Input Parameters:
 *   ch   - The RMT's channel that will be used
 *   pin  - The pin used for the RX channel
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; NULL, otherwise.
 *
 ****************************************************************************/

struct rmt_dev_s *esp_rmt_rx_init(int ch, int pin)
{
  struct rmt_channel_config_s config = RMT_DEFAULT_CONFIG_RX(pin, ch);

  return esp_rmtinitialize(config);
}

#endif
