/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_wdt.c
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

#include "esp32_wdt_lowerhalf.h"
#include "esp32_board_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32_MWDT0
#  define ESP32_MWDT0 (0)
#endif

#ifdef CONFIG_ESP32_MWDT1
#  define ESP32_MWDT1 (1)
#endif

#ifdef CONFIG_ESP32_RWDT
#  define ESP32_RWDT  (2)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_wdt_init
 *
 * Description:
 *   Configure the timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_wdt_init(void)
{
  int ret = OK;

#ifdef CONFIG_ESP32_MWDT0
  ret = esp32_wdt_initialize("/dev/watchdog0", ESP32_MWDT0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog driver: %d\n",
             ret);
      goto errout;
    }
#endif

#ifdef CONFIG_ESP32_MWDT1
  ret = esp32_wdt_initialize("/dev/watchdog1", ESP32_MWDT1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog driver: %d\n",
             ret);
      goto errout;
    }
#endif

#ifdef CONFIG_ESP32_RWDT
  ret = esp32_wdt_initialize("/dev/watchdog2", ESP32_RWDT);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog driver: %d\n",
             ret);
      goto errout;
    }
#endif

errout:
  return ret;
}

