/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-saola-1/src/esp32s2_bringup.c
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
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>
#include <debug.h>
#include <stdio.h>

#include <errno.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_TIMER
#  include "esp32s2_tim_lowerhalf.h"
#endif

#ifdef CONFIG_ESP32S2_I2C
#  include "esp32s2_i2c.h"
#endif

#ifdef CONFIG_ESP32S2_RT_TIMER
#  include "esp32s2_rt_timer.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "esp32s2_board_wdt.h"
#endif

#ifdef CONFIG_SENSORS_MAX6675
#  include "esp32s2_max6675.h"
#endif

#include "esp32s2-saola-1.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int esp32s2_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = board_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize watchdog timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = esp32s2_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

  /* Register the timer drivers */

#ifdef CONFIG_TIMER

#if defined(CONFIG_ESP32S2_TIMER0) && !defined(CONFIG_ONESHOT)
  ret = esp32s2_timer_initialize("/dev/timer0", TIMER0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S2_TIMER1
  ret = esp32s2_timer_initialize("/dev/timer1", TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S2_TIMER2
  ret = esp32s2_timer_initialize("/dev/timer2", TIMER2);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S2_TIMER3
  ret = esp32s2_timer_initialize("/dev/timer3", TIMER3);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#endif /* CONFIG_TIMER */

#ifdef CONFIG_ESP32S2_RT_TIMER
  ret = esp32s2_rt_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize RT timer: %d\n", ret);
    }

#endif
  /* Now register one oneshot driver */

#if defined(CONFIG_ONESHOT) && defined(CONFIG_ESP32S2_TIMER0)

  ret = board_oneshot_init(ONESHOT_TIMER, ONESHOT_RESOLUTION_US);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_oneshot_init() failed: %d\n", ret);
    }

#endif /* CONFIG_ONESHOT */

#ifdef CONFIG_I2C_DRIVER
  /* Configure I2C peripheral interfaces */

  ret = board_i2c_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Try to register BMP180 device in I2C0 */

  ret = board_bmp180_initialize(0, ESP32S2_I2C0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "Failed to initialize BMP180 driver for I2C0: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX6675
  ret = board_max6675_initialize(0, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MAX6675 initialization failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S2_I2S

#ifdef CONFIG_AUDIO_CS4344

  /* Configure CS4344 audio on I2S0 */

  ret = esp32s2_cs4344_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize CS4344 audio: %d\n", ret);
    }
#else

  /* Configure I2S generic audio on I2S0 */

  ret = board_i2sdev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S0 driver: %d\n", ret);
    }
#endif /* CONFIG_AUDIO_CS4344 */

#endif /* CONFIG_ESP32S2_I2S */

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
