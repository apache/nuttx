/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/stm32_appinitialize.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Mark Olsson <post@markolsson.se>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>
#include <syslog.h>

#include "nucleo-144.h"
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

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
      return ret;
    }
#endif

#if defined(CONFIG_MMCSD)
  /* Configure SDIO */

  /* Initialize the SDIO block driver */

  ret = stm32l4_sdio_initialize();
  if (ret != OK)
    {
      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
      return ret;
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
  return OK;
}
