/****************************************************************************
 * boards/xtensa/esp32s2/common/src/esp32s2_board_pcnt.c
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <nuttx/timers/capture.h>
#include <nuttx/sensors/sensor.h>
#include "espressif/esp_pcnt.h"
#ifdef CONFIG_ESP_PCNT_AS_QE
#include "espressif/esp_qencoder.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  -1000

#define PCNT_GLITCH_FILTER(pcnt, thres) pcnt->ops->ioctl(pcnt,          \
                                                         CAPIOC_FILTER, \
                                                         thres)         \

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_pcnt_init
 *
 * Description:
 *   Initialize and register the pulse counter driver
 *
 * Input Parameters:
 *   devpath          - The full path to the driver to register.
 *   unit_cfg         - PCNT unit configuration
 *   chan0_cfg        - PCNT unit channel 0 configuration
 *   chan1_cfg        - PCNT unit channel 1 configuration
 *   glitch_threshold - Threshold value for glitch filter in ns
 *
 * Returned Value:
 *   Valid PCNT device structure reference on success; NULL, otherwise.
 *
 ****************************************************************************/

static struct cap_lowerhalf_s *board_pcnt_init(
  const char *devpath,
  struct esp_pcnt_unit_config_s *unit_cfg,
  struct esp_pcnt_chan_config_s *chan0_cfg,
  struct esp_pcnt_chan_config_s *chan1_cfg,
  uint32_t glitch_threshold)
{
  struct cap_lowerhalf_s *pcnt;
  int chan0;
  int chan1;
  int ret;

  pcnt = esp_pcnt_new_unit(unit_cfg);
  if (!pcnt)
    {
      syslog(LOG_ERR, "Failed to create unit!\n");
      return NULL;
    }

  chan0 = esp_pcnt_new_channel(pcnt, chan0_cfg);
  if (chan0 == ERROR)
    {
      syslog(LOG_ERR, "Failed to create channel!\n");
      esp_pcnt_del_unit(pcnt);
      return NULL;
    }

#ifdef CONFIG_ESP_PCNT_TEST_MODE
  esp_pcnt_channel_set_edge_action(chan0, ESP_PCNT_CHAN_EDGE_ACTION_HOLD,
                                   ESP_PCNT_CHAN_EDGE_ACTION_INCREASE);
  esp_pcnt_channel_set_level_action(chan0, ESP_PCNT_CHAN_LEVEL_ACTION_KEEP,
                                    ESP_PCNT_CHAN_LEVEL_ACTION_KEEP);
#else
  esp_pcnt_channel_set_edge_action(chan0, ESP_PCNT_CHAN_EDGE_ACTION_DECREASE,
                                   ESP_PCNT_CHAN_EDGE_ACTION_INCREASE);
  esp_pcnt_channel_set_level_action(chan0, ESP_PCNT_CHAN_LEVEL_ACTION_KEEP,
                                    ESP_PCNT_CHAN_LEVEL_ACTION_INVERSE);
#endif

  if (chan1_cfg)
    {
      chan1 = esp_pcnt_new_channel(pcnt, chan1_cfg);
      if (chan1 == ERROR)
        {
          syslog(LOG_ERR, "Failed to create channel!\n");
          esp_pcnt_del_channel(chan0);
          esp_pcnt_del_unit(pcnt);
          return NULL;
        }

      esp_pcnt_channel_set_edge_action(chan1,
                                       ESP_PCNT_CHAN_EDGE_ACTION_INCREASE,
                                       ESP_PCNT_CHAN_EDGE_ACTION_DECREASE);
      esp_pcnt_channel_set_level_action(chan1,
                                        ESP_PCNT_CHAN_LEVEL_ACTION_KEEP,
                                        ESP_PCNT_CHAN_LEVEL_ACTION_INVERSE);
    }

  PCNT_GLITCH_FILTER(pcnt, glitch_threshold);

  ret = cap_register(devpath, pcnt);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Error registering PCNT!\n");
      esp_pcnt_del_channel(chan0);
      if (chan1_cfg)
        {
          esp_pcnt_del_channel(chan1);
        }

      esp_pcnt_del_unit(pcnt);
      return NULL;
    }

  return pcnt;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_pcnt_initialize
 *
 * Description:
 *   Initialize the pulse counter/quadrature encoder driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; errno on failure.
 *
 ****************************************************************************/

int board_pcnt_initialize(void)
{
  struct cap_lowerhalf_s *pcnt;
  int ret = OK;
  int glitch_threshold = 0;

#ifdef CONFIG_ESP_PCNT_AS_QE
  char devpath[12];
  int devno = 0;
#endif

  struct esp_pcnt_unit_config_s unit_cfg =
  {
    .high_limit = PCNT_HIGH_LIMIT,
    .low_limit =  PCNT_LOW_LIMIT,
    .accum_count = false,
  };

  struct esp_pcnt_chan_config_s chan0_cfg =
  {
    0
  };

  struct esp_pcnt_chan_config_s chan1_cfg =
  {
    0
  };

#ifdef CONFIG_ESP_PCNT_U0
  chan0_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U0_CH0_EDGE_PIN;
  chan0_cfg.level_gpio_num = CONFIG_ESP_PCNT_U0_CH0_LEVEL_PIN;
#ifdef CONFIG_ESP_PCNT_TEST_MODE
  chan0_cfg.flags = ESP_PCNT_CHAN_IO_LOOPBACK;
#endif

  chan1_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U0_CH1_LEVEL_PIN;
  chan1_cfg.level_gpio_num = CONFIG_ESP_PCNT_U0_CH1_EDGE_PIN;
#ifndef CONFIG_ESP_PCNT_U0_FILTER_EN
  glitch_threshold = 0;
#else
  glitch_threshold = CONFIG_ESP_PCNT_U0_FILTER_THRES;
#endif

  pcnt = board_pcnt_init("/dev/pcnt0", &unit_cfg, &chan0_cfg,
                         &chan1_cfg, glitch_threshold);
  if (!pcnt)
    {
      syslog(LOG_ERR, "ERROR: pcnt initialize failed: %d\n", ret);
      return ERROR;
    }

#ifdef CONFIG_ESP_PCNT_U0_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, pcnt, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }

  pcnt = NULL;
#endif
#endif

#ifdef CONFIG_ESP_PCNT_U1
  chan0_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U1_CH0_EDGE_PIN;
  chan0_cfg.level_gpio_num = CONFIG_ESP_PCNT_U1_CH0_LEVEL_PIN;

  chan1_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U1_CH1_EDGE_PIN;
  chan1_cfg.level_gpio_num = CONFIG_ESP_PCNT_U1_CH1_LEVEL_PIN;
#ifndef CONFIG_ESP_PCNT_U1_FILTER_EN
  glitch_threshold = 0;
#else
  glitch_threshold = CONFIG_ESP_PCNT_U1_FILTER_THRES;
#endif

  pcnt = board_pcnt_init("/dev/pcnt1", &unit_cfg, &chan0_cfg,
                         &chan1_cfg, glitch_threshold);
  if (!pcnt)
    {
      syslog(LOG_ERR, "ERROR: pcnt initialize failed: %d\n", ret);
      return ERROR;
    }

#ifdef CONFIG_ESP_PCNT_U1_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, pcnt, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }

  pcnt = NULL;
#endif
#endif

#ifdef CONFIG_ESP_PCNT_U2
  chan0_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U2_CH0_EDGE_PIN;
  chan0_cfg.level_gpio_num = CONFIG_ESP_PCNT_U2_CH0_LEVEL_PIN;

  chan1_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U2_CH1_EDGE_PIN;
  chan1_cfg.level_gpio_num = CONFIG_ESP_PCNT_U2_CH1_LEVEL_PIN;
#ifndef CONFIG_ESP_PCNT_U2_FILTER_EN
  glitch_threshold = 0;
#else
  glitch_threshold = CONFIG_ESP_PCNT_U2_FILTER_THRES;
#endif

  pcnt = board_pcnt_init("/dev/pcnt2", &unit_cfg, &chan0_cfg,
                         &chan1_cfg, glitch_threshold);

  if (!pcnt)
    {
      syslog(LOG_ERR, "ERROR: pcnt initialize failed: %d\n", ret);
      return ERROR;
    }

#ifdef CONFIG_ESP_PCNT_U2_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, pcnt, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }

  pcnt = NULL;
#endif
#endif

#ifdef CONFIG_ESP_PCNT_U3
  chan0_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U3_CH0_EDGE_PIN;
  chan0_cfg.level_gpio_num = CONFIG_ESP_PCNT_U3_CH0_LEVEL_PIN;

  chan1_cfg.edge_gpio_num = CONFIG_ESP_PCNT_U3_CH1_EDGE_PIN;
  chan1_cfg.level_gpio_num = CONFIG_ESP_PCNT_U3_CH1_LEVEL_PIN;
#ifndef CONFIG_ESP_PCNT_U3_FILTER_EN
  glitch_threshold = 0;
#else
  glitch_threshold = CONFIG_ESP_PCNT_U3_FILTER_THRES;
#endif

  pcnt = board_pcnt_init("/dev/pcnt3", &unit_cfg, &chan0_cfg,
                         &chan1_cfg, glitch_threshold);
  if (!pcnt)
    {
      syslog(LOG_ERR, "ERROR: pcnt initialize failed: %d\n", ret);
      return ERROR;
    }

#ifdef CONFIG_ESP_PCNT_U3_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, pcnt, 3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }

  pcnt = NULL;
#endif
#endif

  return ret;
}
