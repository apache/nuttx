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

#ifdef CONFIG_ONESHOT
#include <nuttx/timers/oneshot.h>
#ifdef CONFIG_CPULOAD_ONESHOT
#include <nuttx/clock.h>
#endif
#endif

#include "esp32s3_board_tim.h"
#include "esp32s3_tim.h"
#include "esp32s3_tim_lowerhalf.h"

#ifdef CONFIG_ONESHOT
#include "esp32s3_oneshot.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: config_oneshot_timer
 *
 * Description:
 *   Configure the oneshot timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ONESHOT
static int config_oneshot_timer(int timer, uint16_t resolution)
{
  int ret = OK;
  struct oneshot_lowerhalf_s *os_lower = NULL;

  os_lower = oneshot_initialize(timer, resolution);
  if (os_lower == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize oneshot timer.\n");
      return -EBUSY;
    }

#if defined(CONFIG_CPULOAD_ONESHOT)
  /* Configure the oneshot timer to support CPU load measurement */

  nxsched_oneshot_extclk(os_lower);
#else
  ret = oneshot_register("/dev/oneshot", os_lower);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register oneshot device: %d\n", ret);
    }
#endif /* CONFIG_CPULOAD_ONESHOT */

  return ret;
}
#endif

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

#if defined(CONFIG_ESP32S3_TIMER0) && !defined(CONFIG_ONESHOT)
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

#if defined(CONFIG_ONESHOT) && defined(CONFIG_ESP32S3_TIMER0)
  /* Now register one oneshot driver */

  ret = config_oneshot_timer(ESP32S3_TIMER0, ONESHOT_RESOLUTION_US);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize oneshot timer: %d\n", ret);
    }

#endif /* CONFIG_ONESHOT */

  return ret;
}
