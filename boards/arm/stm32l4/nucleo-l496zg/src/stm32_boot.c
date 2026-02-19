/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/stm32_boot.c
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

#include <debug.h>
#include <sys/types.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

#include "arm_internal.h"
#include "nucleo-144.h"
#include "stm32l4_i2c.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_STM32L4_I2C1)
struct i2c_master_s *i2c1;
#endif
#if defined(CONFIG_STM32L4_I2C2)
struct i2c_master_s *i2c2;
#endif
#if defined(CONFIG_STM32L4_I2C3)
struct i2c_master_s *i2c3;
#endif
#if defined(CONFIG_STM32L4_I2C4)
struct i2c_master_s *i2c4;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32l4_board_initialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#if defined(CONFIG_STM32L4_OTGFS) || defined(CONFIG_STM32L4_HOST)
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
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started. This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

  stm32_bringup();

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
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

#ifdef CONFIG_STM32L4_DFSDM
  /* Initialize DFSDM and register its filters as additional ADC devices. */

  ret = stm32_dfsdm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_dfsdm_setup failed: %d\n", ret);
    }

#endif
#endif /* CONFIG_ADC */

#ifdef CONFIG_DAC
  /* Initialize DAC and register the DAC driver. */

  ret = stm32_dac_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_dac_setup failed: %d\n", ret);
    }
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
  /* Configure SDIO */

  /* Initialize the SDIO block driver */

  ret = stm32l4_sdio_initialize();
  if (ret != OK)
    {
      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
      return;
    }
#endif

#if defined(CONFIG_I2C)
  /* Configure I2C */

  /* REVISIT: this is ugly! */

#if defined(CONFIG_STM32L4_I2C1)
  i2c1 = stm32l4_i2cbus_initialize(1);
#endif
#if defined(CONFIG_STM32L4_I2C2)
  i2c2 = stm32l4_i2cbus_initialize(2);
#endif
#if defined(CONFIG_STM32L4_I2C3)
  i2c3 = stm32l4_i2cbus_initialize(3);
#endif
#if defined(CONFIG_STM32L4_I2C4)
  i2c4 = stm32l4_i2cbus_initialize(4);
#endif
#ifdef CONFIG_I2C_DRIVER
#if defined(CONFIG_STM32L4_I2C1)
  i2c_register(i2c1, 1);
#endif
#if defined(CONFIG_STM32L4_I2C2)
  i2c_register(i2c2, 2);
#endif
#if defined(CONFIG_STM32L4_I2C3)
  i2c_register(i2c3, 3);
#endif
#if defined(CONFIG_STM32L4_I2C4)
  i2c_register(i2c4, 4);
#endif
#endif
#endif /* CONFIG_I2C */

  UNUSED(ret);
}
#endif
