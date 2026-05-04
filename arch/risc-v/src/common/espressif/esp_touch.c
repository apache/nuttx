/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_touch.c
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
#include <nuttx/debug.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "esp_touch.h"

#include "esp_bit_defs.h"
#include "soc/soc_caps.h"
#include "driver/touch_version_types.h"
#include "driver/touch_sens_types.h"
#include "driver/touch_sens.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOUCH_CFG_DEFAULT()                             \
{                                                       \
  TOUCH_SENSOR_V3_DEFAULT_SAMPLE_CONFIG2(3, 29, 8, 3),  \
  TOUCH_SENSOR_V3_DEFAULT_SAMPLE_CONFIG2(2, 88, 31, 7), \
  TOUCH_SENSOR_V3_DEFAULT_SAMPLE_CONFIG2(3, 10, 31, 7)  \
}

#define TOUCH_CHAN_CFG_DEFAULT()                        \
{                                                       \
  .active_thresh =                                      \
  {                                                     \
    1000,                                               \
    2500,                                               \
    5000                                                \
  },                                                    \
}

#define SCAN_TIMEOUT_MS 2000
#define TOUCH_THRESH_RATIO_PERMILLE 15

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_touch_priv_s
{
  touch_sensor_handle_t sens_handle;                             /* Touch sensor handler */
  touch_channel_handle_t chan_handle[SOC_TOUCH_MAX_CHAN_ID];     /* Touch sensor channels handler */
  touch_sensor_config_t sens_cfg;                                /* Touch sensor configuration struct */
  touch_channel_config_t chan_cfg;                               /* Touch sensor channel configuration struct */
  touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM]; /* Sample configs for sens_cfg */
  int touch_threshold[SOC_TOUCH_MAX_CHAN_ID];                    /* Threshold values for touch activison */
#ifdef CONFIG_ESP_TOUCH_IRQ
  xcpt_t irq_handler[SOC_TOUCH_MAX_CHAN_ID];                     /* Interrupt callback function */
  void *irq_arg[SOC_TOUCH_MAX_CHAN_ID];                          /* Interrupt callback function argument */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint64_t esp_touch_get_channel_mask(void);
static int esp_touch_update_channel_thresholds(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_touch_priv_s esp_touch_priv =
{
  .sens_handle = NULL,
  .chan_handle =
  {
    0
  },
  .sample_cfg = TOUCH_CFG_DEFAULT(),
  .chan_cfg = TOUCH_CHAN_CFG_DEFAULT(),
  .touch_threshold =
  {
    [0 ... SOC_TOUCH_MAX_CHAN_ID - 1] = 0,
  },
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_touch_calibrate
 *
 * Description:
 *   Do the initial scanning to initialize the touch channel data.
 *   Without this step, the channel data in the first read will be invalid
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_touch_calibrate(void)
{
  int i;

  touch_sensor_enable(esp_touch_priv.sens_handle);
  for (i = 0; i < 3; i++)
    {
      touch_sensor_trigger_oneshot_scanning(esp_touch_priv.sens_handle,
                                            SCAN_TIMEOUT_MS);
    }

  touch_sensor_disable(esp_touch_priv.sens_handle);
  esp_touch_update_channel_thresholds();
}

/****************************************************************************
 * Name: esp_touch_update_channel_thresholds
 *
 * Description:
 *   Reconfigure per-sample active thresholds from benchmark values.
 *   This matches the IDF example flow and avoids channels booting active.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_touch_update_channel_thresholds(void)
{
  int ret = OK;
  int mask;
  uint64_t chan_mask;
  touch_channel_config_t chan_cfg = TOUCH_CHAN_CFG_DEFAULT();
  uint32_t benchmark[TOUCH_SAMPLE_CFG_NUM];

  chan_mask = esp_touch_get_channel_mask();
  for (int i = SOC_TOUCH_MIN_CHAN_ID; i < SOC_TOUCH_MAX_CHAN_ID; i++)
    {
      mask = BIT(i);
      if ((chan_mask & mask) == 0)
        {
          continue;
        }

      ret = touch_channel_read_data(esp_touch_priv.chan_handle[i],
                                    TOUCH_CHAN_DATA_TYPE_BENCHMARK,
                                    benchmark);
      if (ret != OK)
        {
          ierr("Failed to read benchmark for touch channel: %d\n", i);
          return ret;
        }

      for (int j = 0; j < TOUCH_SAMPLE_CFG_NUM; j++)
        {
          uint32_t thresh = (benchmark[j] *
            TOUCH_THRESH_RATIO_PERMILLE) / 1000;
          chan_cfg.active_thresh[j] = thresh ? thresh : 1;
        }

      esp_touch_priv.touch_threshold[i] =
        benchmark[0] + chan_cfg.active_thresh[0];
      ret = touch_sensor_reconfig_channel(esp_touch_priv.chan_handle[i],
                                          &chan_cfg);
      if (ret != OK)
        {
          ierr("Failed to reconfigure touch channel: %d\n", i);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esp_touch_get_channel_mask
 *
 * Description:
 *   Get touch channel mask value to initialize.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A 64-bit unsigned integer where each bit corresponds to an channel
 *   that has been configured as a touch gpio channel.
 *
 ****************************************************************************/

static uint64_t esp_touch_get_channel_mask(void)
{
  uint64_t chan_mask = 0;

#ifdef CONFIG_ESP_TOUCH_CHANNEL1
  chan_mask |= BIT(1);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL2
  chan_mask |= BIT(2);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL3
  chan_mask |= BIT(3);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL4
  chan_mask |= BIT(4);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL5
  chan_mask |= BIT(5);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL6
  chan_mask |= BIT(6);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL7
  chan_mask |= BIT(7);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL8
  chan_mask |= BIT(8);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL9
  chan_mask |= BIT(9);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL10
  chan_mask |= BIT(10);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL11
  chan_mask |= BIT(11);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL12
  chan_mask |= BIT(12);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL13
  chan_mask |= BIT(13);
#endif
#ifdef CONFIG_ESP_TOUCH_CHANNEL14
  chan_mask |= BIT(14);
#endif

  return chan_mask;
}

/****************************************************************************
 * Name: esp_touch_channel_prepare
 *
 * Description:
 *   Configure touch pad channels.
 *
 * Input Parameters:
 *   button_num - Address to return number of buttons initialized
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_touch_channel_prepare(int *button_num)
{
  int ret = OK;
  int buttons = 0;
  int mask;
  uint64_t chan_mask;
  touch_chan_info_t chan_info =
    {
      0
    };

  chan_mask = esp_touch_get_channel_mask();
  for (int i = SOC_TOUCH_MIN_CHAN_ID; i < SOC_TOUCH_MAX_CHAN_ID; i++)
    {
      mask = BIT(i);
      if ((chan_mask & mask) != 0)
        {
          buttons++;
          ret = touch_sensor_new_channel(esp_touch_priv.sens_handle, i,
                                         &esp_touch_priv.chan_cfg,
                                         &esp_touch_priv.chan_handle[i]);
          if (ret != OK)
            {
              ierr("Failed to create touch pad channel: %d\n", i);
              return ret;
            }

          touch_sensor_get_channel_info(esp_touch_priv.chan_handle[i],
                                        &chan_info);
          iinfo("Touch pad [CH %d] enabled on GPIO%d\n",
                i, chan_info.chan_gpio);
        }
    }

  *button_num = buttons;
  return ret;
}

#ifdef CONFIG_ESP_TOUCH_IRQ
/****************************************************************************
 * Name: esp_touch_callback
 *
 * Description:
 *   Interrupt callback handler.
 *
 * Input Parameters:
 *   sens_handle - Touch pad handler
 *   event       - Event information of interrupt
 *   user_ctx    - Argument to pass to the handler
 *
 * Returned Value:
 *   False.
 *
 ****************************************************************************/

static bool esp_touch_callback(touch_sensor_handle_t sens_handle,
                               const touch_active_event_data_t *event,
                               void *user_ctx)
{
  int channel = event->chan_id;
  if (esp_touch_priv.irq_handler[channel] != NULL)
    {
      esp_touch_priv.irq_handler[channel](channel,
                                          (void *)event,
                                          esp_touch_priv.irq_arg[channel]);
    }

  return false;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_touchread
 *
 * Description:
 *   Read a touch pad channel.
 *
 * Input Parameters:
 *   channel - The touch pad channel.
 *
 * Returned Value:
 *   0 if touch pad pressed, 1 if released.
 *
 ****************************************************************************/

bool esp_touchread(int channel)
{
  uint32_t data[TOUCH_SAMPLE_CFG_NUM];
#ifndef CONFIG_ESP_TOUCH_MODE_CONTINUOUS
  touch_sensor_trigger_oneshot_scanning(esp_touch_priv.sens_handle,
                                        SCAN_TIMEOUT_MS);
#endif
  touch_channel_read_data(esp_touch_priv.chan_handle[channel],
                          TOUCH_CHAN_DATA_TYPE_SMOOTH,
                          data);
#ifndef CONFIG_ESP_TOUCH_INVERTED
  if (data[0] >= esp_touch_priv.touch_threshold[channel])
#else
  if (data[0] <= esp_touch_priv.touch_threshold[channel])
#endif
    {
      return true;
    }

  return false;
}

#ifdef CONFIG_ESP_TOUCH_IRQ
/****************************************************************************
 * Name: esp_touchirqattach
 *
 * Description:
 *   Attach an interrupt handler to specified channel.
 *
 * Input Parameters:
 *   channel - Touch pad channel number
 *   handler - Interrupt handler function
 *   arg     - Argument to pass to the handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int esp_touchirqattach(int channel, xcpt_t handler, void *arg)
{
  int ret = OK;
  touch_sensor_disable(esp_touch_priv.sens_handle);
  touch_event_callbacks_t callbacks =
    {
#ifndef CONFIG_ESP_TOUCH_INVERTED
      .on_active = esp_touch_callback,
#else
      .on_inactive = esp_touch_callback,
#endif
    };

  esp_touch_priv.irq_handler[channel] = handler;
  esp_touch_priv.irq_arg[channel] = arg;
  ret = touch_sensor_register_callbacks(esp_touch_priv.sens_handle,
                                        &callbacks,
                                        NULL);
  touch_sensor_enable(esp_touch_priv.sens_handle);
  return ret;
}

/****************************************************************************
 * Name: esp_touchirqdetach
 *
 * Description:
 *   Detach interrupt handler from specified channel.
 *
 * Input Parameters:
 *   channel - Touch pad channel number
 *
 * Returned Value:
 *   Zero (OK) is returned on success; -1 (ERROR) in failure
 *
 ****************************************************************************/

int esp_touchirqdetach(int channel)
{
  if (channel > SOC_TOUCH_MAX_CHAN_ID)
    {
      ierr("Invalid channel number\n");
      return ERROR;
    }

  if (esp_touch_priv.irq_handler[channel] != NULL)
    {
      esp_touch_priv.irq_handler[channel] = NULL;
      esp_touch_priv.irq_arg[channel] = NULL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_configtouch
 *
 * Description:
 *   Configures touch pad channels.
 *
 * Input Parameters:
 *   button_num - Address to return number of buttons initialized
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_configtouch(int *button_num)
{
  int ret = OK;
#ifdef CONFIG_ESP_TOUCH_FILTER
  touch_sensor_filter_config_t filter_cfg =
    TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
  filter_cfg.benchmark.denoise_lvl = CONFIG_ESP_TOUCH_DENOISE_LVL;
#endif
#ifdef CONFIG_PM_TOUCH_WAKEUP
  touch_sleep_config_t slp_cfg = TOUCH_SENSOR_DEFAULT_DSLP_CONFIG();
#endif

  if (esp_touch_priv.sens_handle != NULL)
    {
      iinfo("Touch pad driver already initialized.\n");
      return ERROR;
    }

  esp_touch_priv.sens_cfg = (touch_sensor_config_t)
    TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(TOUCH_SAMPLE_CFG_NUM,
                                      esp_touch_priv.sample_cfg);

  ret = touch_sensor_new_controller(&esp_touch_priv.sens_cfg,
                                    &esp_touch_priv.sens_handle);
  if (ret != OK)
    {
      ierr("Failed to initialize touch pad controller\n");
      return ret;
    }

  ret = esp_touch_channel_prepare(button_num);
  if (ret != OK)
    {
      ierr("Failed to initialize touch pad channels\n");
      return ret;
    }

#ifdef CONFIG_ESP_TOUCH_FILTER
  touch_sensor_config_filter(esp_touch_priv.sens_handle, &filter_cfg);
#else
  touch_sensor_config_filter(esp_touch_priv.sens_handle, NULL);
#endif
  esp_touch_calibrate();

#ifdef CONFIG_PM_TOUCH_WAKEUP
  touch_sensor_config_sleep_wakeup(esp_touch_priv.sens_handle, &slp_cfg);
#endif
  touch_sensor_enable(esp_touch_priv.sens_handle);

#ifdef CONFIG_ESP_TOUCH_MODE_CONTINUOUS
  touch_sensor_start_continuous_scanning(esp_touch_priv.sens_handle);
#else
  touch_sensor_trigger_oneshot_scanning(esp_touch_priv.sens_handle,
                                        SCAN_TIMEOUT_MS);
#endif
  return ret;
}
