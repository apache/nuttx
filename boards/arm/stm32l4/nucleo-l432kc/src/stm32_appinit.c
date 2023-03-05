/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_appinit.c
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
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/leds/userled.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>
#include <stm32l4_i2c.h>

#include <arch/board/board.h>

#include "nucleo-l432kc.h"

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_I2C_DRIVER
#if (defined(CONFIG_STM32L4_I2C1) || defined(CONFIG_STM32L4_I2C3)) && defined(CONFIG_I2C_DRIVER)
#  define HAVE_I2C_DRIVER 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Dummy function expected to start-up logic.
 *
 ****************************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
}
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *rtclower;
#endif
#ifdef CONFIG_STM32L4_I2C1
  struct i2c_master_s *i2c1;
#endif
#ifdef CONFIG_STM32L4_I2C3
  struct i2c_master_s *i2c3;
#endif
#ifdef CONFIG_SENSORS_QENCODER
  int index;
  char buf[9];
#endif
  int ret = OK;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d\n",
             ret);
      return ret;
    }
#endif

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = stm32l4_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32L4 lower-half RTC driver */

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef CONFIG_STM32L4_I2C1
  /* Get the I2C lower half instance */

  i2c1 = stm32l4_i2cbus_initialize(1);
  if (i2c1 == NULL)
    {
      i2cerr("ERROR: Initialize I2C1: %d\n", ret);
    }
  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c1, 1);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register I2C1 device: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_STM32L4_I2C3
  /* Get the I2C lower half instance */

  i2c3 = stm32l4_i2cbus_initialize(3);
  if (i2c3 == NULL)
    {
      i2cerr("ERROR: Initialize I2C3: %d\n", ret);
    }
  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c3, 3);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register I2C3 device: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_SPI_DRIVER
  stm32l4_spiregister();
  /* driver registering must be processed in appinit.
   * If called it during board_init,
   * registering failed due to heap doesn't be initialized yet.
   */
#endif

#ifdef HAVE_AT45DB
  /* Initialize and register the ATDB FLASH file system. */

  ret = stm32_at45dbinitialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize AT45DB minor %d: %d\n",
             0, ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32l4_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32L4_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32l4_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DAC7571
  /* Initialize and register DAC7571 */

  ret = stm32_dac7571initialize("/dev/dac0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_dac7571initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_INA226
  /* Initialize and register INA226 */

  ret = stm32_ina226initialize("/dev/ina226");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_ina226initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_INA219
  /* Initialize and register INA219 */

  ret = stm32_ina219initialize("/dev/ina219");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_ina219initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ZEROCROSS
  /* Configure the zero-crossing driver */

  ret = stm32_zerocross_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Zero-Cross, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

  ret = board_timer_driver_initialize("/dev/timer0", 2);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  index = 0;

#ifdef CONFIG_STM32L4_TIM1_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 1);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM2_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 2);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM3_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 3);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM4_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 4);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM5_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 5);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM8_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 8);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#endif

  return ret;
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  return -ENOTTY;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == NULL)
    {
      return -EINVAL;
    }

  stm32l4_get_uniqueid(uniqueid);
  return OK;
}
#endif
