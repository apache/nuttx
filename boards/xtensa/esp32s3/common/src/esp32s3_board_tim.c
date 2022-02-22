/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_tim.c
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
#include <debug.h>

#include "esp32s3_board_tim.h"
#include "esp32s3_tim.h"
#include "esp32s3_tim_lowerhalf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tim_init
 *
 * Description:
 *   Configure the timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_tim_init(void)
{
  int ret = OK;

#if defined(CONFIG_ESP32S3_TIMER0)
  ret = esp32s3_timer_initialize("/dev/timer0", ESP32S3_TIMER0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize TIMER0: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32S3_TIMER1)
  ret = esp32s3_timer_initialize("/dev/timer1", ESP32S3_TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize TIMER1: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32S3_TIMER2)
  ret = esp32s3_timer_initialize("/dev/timer2", ESP32S3_TIMER2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize TIMER2: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32S3_TIMER3)
  ret = esp32s3_timer_initialize("/dev/timer3", ESP32S3_TIMER3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize TIMER3: %d\n", ret);
    }
#endif

  return ret;
}
