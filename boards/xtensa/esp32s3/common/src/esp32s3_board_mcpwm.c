/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_mcpwm.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/timers/capture.h>

#include <arch/board/board.h>

#include "espressif/esp_mcpwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_capture_initialize
 *
 * Description:
 *   Initialize MCPWM Capture submodule and register the capture device.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_capture_initialize(void)
{
  int ret;
  struct cap_lowerhalf_s *cap;

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH0
  cap = esp_mcpwm_capture_initialize(0,
                                     CONFIG_ESP_MCPWM_CAPTURE_CH0_GPIO);

  if (!cap)
    {
      syslog(LOG_ERR, "ERROR: Failed to start MCPWM Capture: CH0\n");
      return -ENODEV;
    }

  ret = cap_register("/dev/capture0", cap);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cap_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH1
  cap = esp_mcpwm_capture_initialize(1,
                                     CONFIG_ESP_MCPWM_CAPTURE_CH1_GPIO);
  if (!cap)
    {
      syslog(LOG_ERR, "ERROR: Failed to start MCPWM Capture: CH1\n");
      return -ENODEV;
    }

  ret = cap_register("/dev/capture1", cap);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cap_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH2
  cap = esp_mcpwm_capture_initialize(2,
                                     CONFIG_ESP_MCPWM_CAPTURE_CH2_GPIO);
  if (!cap)
    {
      syslog(LOG_ERR, "ERROR: Failed to start MCPWM Capture: CH2\n");
      return -ENODEV;
    }

  ret = cap_register("/dev/capture2", cap);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cap_register failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
