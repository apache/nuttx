/****************************************************************************
 * boards/arm/rp2040/pimoroni-tiny2040/src/rp2040_bringup.c
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

#include <debug.h>
#include <stddef.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp2040_tiny2040.h"

#ifdef CONFIG_RP2040_PWM
#include "rp2040_pwm.h"
#include "rp2040_pwmdev.h"
#endif

#if defined(CONFIG_RP2040_BOARD_HAS_WS2812) && defined(CONFIG_WS2812)
#include "rp2040_ws2812.h"
#endif

#ifdef CONFIG_WS2812_HAS_WHITE
#define HAS_WHITE true
#else /* CONFIG_WS2812_HAS_WHITE */
#define HAS_WHITE false
#endif /* CONFIG_WS2812_HAS_WHITE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_bringup
 ****************************************************************************/

int rp2040_bringup(void)
{
  int ret = 0;

#ifdef CONFIG_RP2040_I2C_DRIVER
  #ifdef CONFIG_RP2040_I2C0
  ret = board_i2cdev_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize I2C0.\n");
    }
  #endif

  #ifdef CONFIG_RP2040_I2C1
  ret = board_i2cdev_initialize(1);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize I2C1.\n");
    }
  #endif
#endif

#ifdef CONFIG_RP2040_SPI_DRIVER
  #ifdef CONFIG_RP2040_SPI0
  ret = board_spidev_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI0.\n");
    }
  #endif

  #ifdef CONFIG_RP2040_SPI1
  ret = board_spidev_initialize(1);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI1.\n");
    }
  #endif
#endif

#ifdef CONFIG_RP2040_SPISD
  /* Mount the SPI-based MMC/SD block driver */

  ret = board_spisd_initialize(0, CONFIG_RP2040_SPISD_SPI_CH);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI device to MMC/SD: %d\n",
           ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_RP2040_I2S
  ret = board_i2sdev_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize I2S.\n");
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = rp2040_dev_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RP2040_PWM
#  ifdef CONFIG_RP2040_PWM0
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(0,
                                 CONFIG_RP2040_PWM0A_GPIO,
                                 CONFIG_RP2040_PWM0B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM0A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM0B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM0_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(0,
                                 CONFIG_RP2040_PWM0A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM0A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM0_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM0.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM1
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(1,
                                 CONFIG_RP2040_PWM1A_GPIO,
                                 CONFIG_RP2040_PWM1B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM1A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM1B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM1_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(1,
                                 CONFIG_RP2040_PWM1A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM1A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM1_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM1.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM2
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(2,
                                 CONFIG_RP2040_PWM2A_GPIO,
                                 CONFIG_RP2040_PWM2B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM2A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM2B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM2_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(2,
                                 CONFIG_RP2040_PWM2A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM2A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM2_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM2.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM3
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(3,
                                 CONFIG_RP2040_PWM3A_GPIO,
                                 CONFIG_RP2040_PWM3B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM3A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM3B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM3_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(3,
                                 CONFIG_RP2040_PWM3A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM3A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM3_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM3.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM4
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(4,
                                 CONFIG_RP2040_PWM4A_GPIO,
                                 CONFIG_RP2040_PWM4B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM4A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM4B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM4_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(4,
                                 CONFIG_RP2040_PWM4A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM4A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM4_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM4.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM5
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(5,
                                 CONFIG_RP2040_PWM5A_GPIO,
                                 CONFIG_RP2040_PWM5B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM5A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM5B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM5_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#  else
  ret = rp2040_pwmdev_initialize(5,
                                 CONFIG_RP2040_PWM5A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM5A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM5_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#  endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM5.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM6
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(6,
                                 CONFIG_RP2040_PWM6A_GPIO,
                                 CONFIG_RP2040_PWM6B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM6A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM6B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM6_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(6,
                                 CONFIG_RP2040_PWM6A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM6A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM6_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM6.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM7
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(7,
                                 CONFIG_RP2040_PWM7A_GPIO,
                                 CONFIG_RP2040_PWM7B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM7A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM7B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM7_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(7,
                                 CONFIG_RP2040_PWM7A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM7A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM7_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize PWM7.\n");
    }
#  endif
#endif

  /* Initialize board neo-pixel */

#if defined(CONFIG_RP2040_BOARD_HAS_WS2812) && defined(CONFIG_WS2812)
  rp2040_ws2812_setup("/dev/leds0",
                      CONFIG_RP2040_WS2812_GPIO_PIN,
                      CONFIG_WS2812_LED_COUNT,
                      HAS_WHITE);
#endif

  return ret;
}
