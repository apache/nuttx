/****************************************************************************
 * boards/arm/tlsr82/tlsr8278adk80d/src/tlsr8278_bringup.c
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
#include <syslog.h>
#include <debug.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>

#include "tlsr8278adk80d.h"

#include "tlsr82_timer.h"
#include "tlsr82_timer_lowerhalf.h"
#include "tlsr82_watchdog.h"
#include "tlsr82_pwm.h"
#include "tlsr82_adc.h"
#include "tlsr82_gpio.h"
#include "tlsr82_flash_mtd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tlsr8278_bringup(void)
{
  int ret = 0;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
      return ret;
    }

#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_TLSR82_TIMER
#ifdef CONFIG_TLSR82_TIMER1
  ret = tlsr82_timer_initialize("/dev/timer1", TLSR82_TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer1 driver: %d\n",
             ret);
      return ret;
    }
#endif /* CONFIG_TLSR82_TIMER1 */

#ifdef CONFIG_TLSR82_TIMER2
  ret = tlsr82_timer_initialize("/dev/timer2", TLSR82_TIMER2);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer2 driver: %d\n",
             ret);
      return ret;
    }
#endif /* CONFIG_TLSR82_TIMER2 */

#ifdef CONFIG_TLSR82_WATCHDOG
  ret = tlsr82_wdginitialize("dev/watchdog0");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog driver: %d\n",
             ret);
      return ret;
    }
#endif /* CONFIG_TLSR82_WATCHDOG */

#endif /* CONFIG_TIMER */

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
  ret = tlsr82_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize gpio driver: %d\n",
             ret);
      return ret;
    }

#endif /* defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF) */

#ifdef CONFIG_TLSR82_PWM
#ifdef CONFIG_TLSR82_PWM0
  ret = tlsr82_pwminitialize("/dev/pwm0", 0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize pwm1 driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_TLSR82_PWM1
  ret = tlsr82_pwminitialize("/dev/pwm1", 1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize pwm1 driver: %d\n",
             ret);
      return ret;
    }
#endif
#endif /* CONFIG_TLSR82_PWM */

#ifdef CONFIG_TLSR82_ADC

#ifdef CONFIG_TLSR82_ADC_CHAN0
  ret = tlsr82_adc_init("/dev/adc0", ADC_CHAN_0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize adc0 driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN1
  ret = tlsr82_adc_init("/dev/adc1", ADC_CHAN_1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize adc1 driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN2
  ret = tlsr82_adc_init("/dev/adc2", ADC_CHAN_2);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize adc2 driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_TLSR82_ADC_VBAT
  ret = tlsr82_adc_init("/dev/adcvbat", ADC_CHAN_VBAT);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize adcbat driver: %d\n",
             ret);
      return ret;
    }
#endif

#endif /* CONFIG_TLSR82_ADC */

#ifdef CONFIG_TLSR82_FLASH
  struct mtd_dev_s *mtd = tlsr82_flash_initialize(
                            CONFIG_TLSR82_FLASH_FS_OFFSET,
                            CONFIG_TLSR82_FLASH_FS_SIZE);
  if (mtd == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize mtd driver: %d\n",
             ret);
      return ret;
    }

  ret = register_mtddriver("/dev/mtd", mtd, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_MTD_PARTITION
  ret = tlsr82_partition_init("/dev/mtd");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init MTD partition: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_FS_LITTLEFS
  /* Mount the LittleFS file system */

  ret = nx_mount("/dev/mtd", "/data", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount LittleFS at /data: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_FS_LITTLEFS */

#endif /* CONFIG_TLSR82_FLASH */

  return ret;
}
