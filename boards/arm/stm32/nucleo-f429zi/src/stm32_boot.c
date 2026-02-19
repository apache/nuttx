/****************************************************************************
 * boards/arm/stm32/nucleo-f429zi/src/stm32_boot.c
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
#include <debug.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nucleo-144.h"

#ifdef CONFIG_STM32_ROMFS
#include "stm32_romfs.h"
#endif

#ifdef CONFIG_SENSORS_AMG88XX
#include "stm32_amg88xx.h"
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
#  include "stm32_i2c.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization
 *   after all memory has been configured and mapped but
 *   before any devices have been initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#if defined(CONFIG_STM32_OTGFS) || defined(CONFIG_STM32_HOST)
  stm32_usbinitialize();
#endif

#if defined(CONFIG_SPI)
  /* Configure SPI chip selects */

  stm32_spidev_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize()
 *   will be called immediately after up_initialize() is called and
 *   just before the initial application is started. This additional
 *   initialization phase may be used, for example, to initialize
 *   board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_STM32_ROMFS
  /* Mount the romfs partition */

  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount romfs at %s: %d\n",
             CONFIG_STM32_ROMFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  /* Register the GPIO driver */

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return;
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

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_BBSRAM
  /* Initialize battery-backed RAM */

  stm32_bbsram_int();
#endif

#if defined(CONFIG_FAT_DMAMEMORY)
  if (stm32_dma_alloc_init() < 0)
    {
      syslog(LOG_ERR, "DMA alloc FAILED");
    }
#endif

#if defined(CONFIG_NUCLEO_SPI_TEST)
  /* Create SPI interfaces */

  ret = stm32_spidev_bus_test();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI interfaces: %d\n",
             ret);
      return;
    }
#endif

#if defined(CONFIG_MMCSD)
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret != OK)
    {
      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
      return;
    }
#endif

#if defined(CONFIG_PWM)
  /* Initialize PWM and register the PWM device */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2c_register(1);
#endif

#ifdef CONFIG_SENSORS_AMG88XX
  board_amg88xx_initialize(1);
#endif

  UNUSED(ret);
}
#endif
