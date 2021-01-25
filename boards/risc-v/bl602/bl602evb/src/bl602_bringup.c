/****************************************************************************
 * boards/risc-v/bl602/evb/src/bl602_bringup.c
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
#include <nuttx/timers/oneshot.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/input/buttons.h>
#include <bl602_tim_lowerhalf.h>
#include <bl602_oneshot_lowerhalf.h>
#include <bl602_pwm_lowerhalf.h>
#include <bl602_wdt_lowerhalf.h>
#include <bl602_gpio.h>
#include <bl602_i2c.h>

#if defined(CONFIG_BL602_SPIFLASH)
#include <bl602_spiflash.h>
#endif

#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_bringup
 ****************************************************************************/

int bl602_bringup(void)
{
#if defined(CONFIG_TIMER) && defined(CONFIG_ONESHOT) && \
  defined(CONFIG_BL602_TIMER1)
  struct oneshot_lowerhalf_s *os = NULL;
#endif
#if defined(CONFIG_BL602_SPIFLASH)
  FAR struct mtd_dev_s *mtd_part = NULL;
  const char *path = "/dev/mtdflash";
#endif
#ifdef CONFIG_I2C
  struct i2c_master_s *i2c_bus;
#endif
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
      return ret;
    }
#endif

#if defined(CONFIG_TIMER)
#if defined(CONFIG_BL602_TIMER0)
  ret = bl602_timer_initialize("/dev/timer0", 0);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer0 Driver: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_BL602_TIMER1) && !defined(CONFIG_ONESHOT)
  ret = bl602_timer_initialize("/dev/timer1", 1);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer1 Driver: %d\n", ret);
      return ret;
    }
#elif defined(CONFIG_BL602_TIMER1) && defined(CONFIG_ONESHOT)
  os = oneshot_initialize(1, 1);
  if (os == NULL)
    {
      syslog(LOG_DEBUG, "ERROR: oneshot_initialize failed\n");
    }
  else
    {
#ifdef CONFIG_CPULOAD_ONESHOT
      /* Configure the oneshot timer to support CPU load measurement */

      nxsched_oneshot_extclk(os);

#else
      ret = oneshot_register("/dev/oneshot", os);
      if (ret < 0)
        {
          syslog(LOG_DEBUG,
            "ERROR: Failed to register oneshot at /dev/oneshot: %d\n", ret);
        }
#endif
    }
#endif
#endif

#ifdef CONFIG_PWM
  struct pwm_lowerhalf_s *pwm;

  /* Initialize PWM and register the PWM driver */

  pwm = bl602_pwminitialize(0);
  if (pwm == NULL)
    {
      syslog(LOG_DEBUG, "ERROR: bl602_pwminitialize failed\n");
    }
  else
    {
      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          syslog(LOG_DEBUG, "ERROR: pwm_register failed: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_WATCHDOG
  ret = bl602_wdt_initialize(CONFIG_WATCHDOG_DEVPATH);
  if (ret < 0)
    {
      syslog(LOG_DEBUG, "ERROR: bl602_wdt_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = bl602_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_I2C
  i2c_bus = bl602_i2cbus_initialize(0);
  i2c_register(i2c_bus, 0);
#endif

#ifdef CONFIG_BL602_SPIFLASH
  mtd_part = bl602_spiflash_alloc_mtdpart();

  if (!mtd_part)
    {
      syslog(LOG_DEBUG,
        "ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return -1;
    }

  /* Register the MTD driver so that it can be accessed from the  VFS */

  ret = register_mtddriver(path, mtd_part, 0777, NULL);
  if (ret < 0)
    {
      syslog(LOG_DEBUG, "ERROR: Failed to regitser MTD: %d\n", ret);
      return -1;
    }

  /* Mount the SPIFFS file system */

#ifdef CONFIG_FS_LITTLEFS
  ret = mount(path, "/mnt/lfs", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "ERROR: Failed to mount littlefs at /mnt/llfs: %d\n", ret);
      return -1;
    }

#endif /* CONFIG_FS_LITTLEFS */
#endif /* CONFIG_BL602_SPIFLASH */

  return ret;
}
