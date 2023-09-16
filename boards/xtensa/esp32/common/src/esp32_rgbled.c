/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_rgbled.c
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
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/leds/rgbled.h>

#include "esp32-sparrow-kit.h"
#include "esp32_gpio.h"
#include "esp32_ledc.h"
#include "esp32_rgbled.h"

#if defined(CONFIG_ESP32_TIMER0) && \
    defined(CONFIG_ESP32_LEDC) && \
    defined(CONFIG_ESP32_LEDC_TIM0) && \
    (CONFIG_ESP32_LEDC_TIM0_CHANNELS >= 3) && \
    defined(CONFIG_PWM_MULTICHAN) && (CONFIG_PWM_NCHANNELS >= 3)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_rgbled_initialize
 *
 * Description:
 *   Initialize support for RGB LED using PWM.
 *
 ****************************************************************************/

int esp32_rgbled_initialize(const char *devname)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *ledr;
  struct pwm_lowerhalf_s *ledg;
  struct pwm_lowerhalf_s *ledb;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      ledr = esp32_ledc_init(RGB_LED_TIMER);
      if (!ledr)
        {
          lederr("ERROR: Failed to get the ESP32 PWM lower half to LEDR\n");
          return -ENODEV;
        }

      /* Initialize LED R */

      ledr->ops->setup(ledr);

      ledg = esp32_ledc_init(RGB_LED_TIMER);
      if (!ledg)
        {
          lederr("ERROR: Failed to get the ESP32 PWM lower half to LEDG\n");
          return -ENODEV;
        }

      /* Initialize LED G */

      ledg->ops->setup(ledg);

      ledb = esp32_ledc_init(RGB_LED_TIMER);
      if (!ledb)
        {
          lederr("ERROR: Failed to get the ESP32 PWM lower half to LEDB\n");
          return -ENODEV;
        }

      /* Initialize LED B */

      ledb->ops->setup(ledb);

      /* Register the RGB LED diver at <devname> */

      ret = rgbled_register(devname, ledr, ledg, ledb, RGB_R_CHANN,
                                                       RGB_G_CHANN,
                                                       RGB_B_CHANN);
      if (ret < 0)
        {
          lederr("ERROR: rgbled_register failed: %d\n", ret);
          return ret;
        }

      int fd = nx_open(devname, O_WRONLY);

      if (fd < 0)
        {
          lederr("ERROR: rgbled_open failed\n");
          return fd;
        }

      /* Turn OFF the LED */

      ret = nx_write(fd, "#000000", 8);
      nx_close(fd);

      if (ret < 0)
        {
          lederr("ERROR: rgbled_write failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#else
#  error "RGB LED bad configuration"
#endif

