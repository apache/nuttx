/****************************************************************************
 * boards/arm/stm32h7/h747ai/src/stm32_bringup.c
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
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_RPTUN
#  include "stm32_rptun.h"
#endif
#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#if defined(CONFIG_I2C)
#  include "stm32_i2c.h"
#endif

#if defined(CONFIG_SPI)
#  include "stm32_spi.h"
#  include "stm32_gpio.h"
#  if defined(CONFIG_SPI_DRIVER)
#    include <nuttx/spi/spi.h>
#    include <nuttx/spi/spi_transfer.h>
#  endif
#endif

#include "h747ai.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNO_ZERO   0
#define DEVNO_ONE    1
#define DEVNO_TWO    2
#define DEVNO_THREE  3

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_serialinit
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  uart_rpmsg_init("cm7", "proxy", 4096, false);
#endif

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4
#  ifdef CONFIG_RPMSG_UART_CONSOLE
  uart_rpmsg_init("cm4", "proxy", 4096, true);
#  else
  uart_rpmsg_init("cm4", "proxy", 4096, false);
#  endif
#endif
}
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;
#ifdef CONFIG_STM32H7_I2C1
  struct i2c_master_s *i2c1;
#endif
#ifdef CONFIG_STM32H7_I2C2
  struct i2c_master_s *i2c2;
#endif
#ifdef CONFIG_STM32H7_I2C3
  struct i2c_master_s *i2c3;
#endif
#ifdef CONFIG_STM32H7_I2C4
  struct i2c_master_s *i2c4;
#endif

#ifdef CONFIG_STM32H7_SPI1
  struct spi_dev_s *spi1;
#endif
#ifdef CONFIG_STM32H7_SPI2
  struct spi_dev_s *spi2;
#endif
#ifdef CONFIG_STM32H7_SPI3
  struct spi_dev_s *spi3;
#endif
#ifdef CONFIG_STM32H7_SPI4
  struct spi_dev_s *spi4;
#endif
#ifdef CONFIG_STM32H7_SPI5
  struct spi_dev_s *spi5;
#endif
#ifdef CONFIG_STM32H7_SPI6
  struct spi_dev_s *spi6;
#endif

  UNUSED(ret);

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_RPTUN
#  ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  stm32_rptun_init("cm7-shmem", "cm7");
#  else
  stm32_rptun_init("cm4-shmem", "cm4");
#  endif
#endif

#ifdef CONFIG_DEV_GPIO
  /* Register the GPIO driver */

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
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

#ifdef CONFIG_STM32H7_I2C1
  i2c1 = stm32_i2cbus_initialize(1);
  if (i2c1 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init i2c controller 1\n");
    }
  else
    {
      ret = i2c_register(i2c1, DEVNO_ZERO);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                DEVNO_ZERO, ret);
          stm32_i2cbus_uninitialize(i2c1);
        }
    }

  i2c2_m = stm32_i2cbus_initialize(2);
  if (i2c2_m == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init i2c controller 2\n");
    }
  else
    {
      ret = i2c_register(i2c2, DEVNO_ONE);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                DEVNO_ONE, ret);
          stm32_i2cbus_uninitialize(i2c2);
        }
    }
#endif

#ifdef CONFIG_STM32H7_I2C3
  i2c3 = stm32_i2cbus_initialize(3);
  if (i2c3 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init i2c controller 3\n");
    }
  else
    {
      ret = i2c_register(i2c3, DEVNO_TWO);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                DEVNO_TWO, ret);
          stm32_i2cbus_uninitialize(i2c3);
        }
    }
#endif

#ifdef CONFIG_STM32H7_I2C4
  i2c4 = stm32_i2cbus_initialize(4);
  if (i2c4 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init i2c controller 4\n");
    }
  else
    {
      ret = i2c_register(i2c4, DEVNO_THREE);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                DEVNO_THREE, ret);
          stm32_i2cbus_uninitialize(i2c4);
        }
    }
#endif

#ifdef CONFIG_STM32H7_SPI1
  /* Set up SPI1 NSS */

  stm32_configgpio(GPIO_SPI1_NSS);

  /* Initialize the SPI1 bus */

  spi1 = stm32_spibus_initialize(1);
  if (spi1 == NULL)
    {
      spierr("ERROR: Initialize SPI1: \n");
    }
  else
    {
#ifdef CONFIG_SPI_DRIVER
      /* Register the SPI1 character driver */

      ret = spi_register(spi1, 1);
      if (ret < 0)
        {
          spierr("ERROR: Failed to register SPI1 device: %d\n", ret);
        }
#endif
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: board_early_initialize
 *
 * Description:
 *   If CONFIG_BOARD_EARLY_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_early_initialize().  board_early_initialize() will
 *   be called immediately after up_initialize() and well before
 *   board_early_initialize() is called and the initial application is
 *   started.  The context in which board_early_initialize() executes is
 *   suitable for early initialization of most, simple device drivers and is
 *   a logical, board-specific extension of up_initialize().
 *
 *   board_early_initialize() runs on the startup, initialization thread.
 *   Some initialization operations cannot be performed on the start-up,
 *   initialization thread.  That is because the initialization thread cannot
 *   wait for event.  Waiting may be required, for example, to mount a file
 *   system or or initialize a device such as an SD card. For this reason,
 *   such driver initialize must be deferred to board_late_initialize().

 ****************************************************************************/

#ifdef CONFIG_BOARD_EARLY_INITIALIZE
void board_early_initialize(void)
{
  int ret = OK;

  UNUSED(ret);
}
#endif
