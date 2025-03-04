/****************************************************************************
 * boards/risc-v/esp32h2/common/src/esp_board_mcpwm.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#ifdef CONFIG_MOTOR
#include <nuttx/motor/motor.h>
#endif
#ifdef CONFIG_CAPTURE
#include <nuttx/timers/capture.h>
#endif

#include <arch/board/board.h>

#include "espressif/esp_mcpwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPMW_MOTOR_CH0_FAULT
#  define MCPWM_FAULT_GPIO CONFIG_ESP_MCPMW_MOTOR_CH0_FAULT_GPIO
#else
#  define MCPWM_FAULT_GPIO 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_motor_initialize
 *
 * Description:
 *   Initialize MCPWM peripheral for motor control and register the motor
 *   driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
int board_motor_initialize(void)
{
  int ret;
  struct motor_lowerhalf_s *motor;

#ifdef CONFIG_ESP_MCPWM_MOTOR_CH0
  motor = esp_motor_bdc_initialize(0,
                                   CONFIG_ESP_MCPWM_MOTOR_CH0_PWM_FREQ,
                                   CONFIG_ESP_MCPWM_MOTOR_CH0_PWMA_GPIO,
                                   CONFIG_ESP_MCPWM_MOTOR_CH0_PWMB_GPIO,
                                   MCPWM_FAULT_GPIO);
  if (motor == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to start MCPWM BDC Motor: CH0\n");
      return -ENODEV;
    }

  ret = motor_register("/dev/motor0", motor);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: motor_register failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
#endif

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

#ifdef CONFIG_ESP_MCPWM_CAPTURE
int board_capture_initialize(void)
{
  int ret;
  struct cap_lowerhalf_s *cap;

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH0
  cap = esp_mcpwm_capture_initialize(0, CONFIG_ESP_MCPWM_CAPTURE_CH0_GPIO);
  if (cap == NULL)
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
  cap = esp_mcpwm_capture_initialize(1, CONFIG_ESP_MCPWM_CAPTURE_CH1_GPIO);
  if (cap == NULL)
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
  cap = esp_mcpwm_capture_initialize(2, CONFIG_ESP_MCPWM_CAPTURE_CH2_GPIO);
  if (cap == NULL)
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
#endif
