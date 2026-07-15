/****************************************************************************
 * boards/risc-v/gd32vw55x/gd32vw553k-start/src/gd32_bringup.c
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

#include <nuttx/fs/fs.h>

#ifdef CONFIG_I2C_DRIVER
#  include <nuttx/i2c/i2c_master.h>
#endif
#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#endif
#ifdef CONFIG_GD32VW55X_ADC
#  include <nuttx/analog/adc.h>
#endif
#ifdef CONFIG_GD32VW55X_PWM
#  include <nuttx/timers/pwm.h>
#endif
#ifdef CONFIG_GD32VW55X_CAPTURE
#  include <nuttx/timers/capture.h>
#endif
#ifdef CONFIG_USERLED_LOWER
#  include <nuttx/leds/userled.h>
#endif
#ifdef CONFIG_MTD_PROGMEM
#  include <nuttx/mtd/mtd.h>
#endif

#include "gd32vw55x_i2c.h"
#include "gd32vw55x_spi.h"
#include "gd32vw55x_adc.h"
#include "gd32vw55x_pwm.h"
#include "gd32vw55x_capture.h"
#include "gd32vw55x_fwdgt.h"
#include "gd32vw55x_wwdgt.h"

#include <arch/board/board.h>

#include "gd32vw553k-start.h"

#ifdef CONFIG_GD32VW55X_WIFI
/* Brings the radio up at boot and registers wlan0 -- same pattern as the
 * board_wlan_init() of the Espressif boards.  It must happen before the NSH
 * netinit, which looks for the interface.
 */

int gdwifi_start(void);
#endif

#ifdef CONFIG_GD32VW55X_BLE
int gd32_ble_initialize(void);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_bringup
 ****************************************************************************/

int gd32_bringup(void)
{
#ifdef CONFIG_GD32VW55X_I2C0
  struct i2c_master_s *i2c;
#endif
#ifdef CONFIG_GD32VW55X_SPI
  struct spi_dev_s *spi;
#endif
#ifdef CONFIG_GD32VW55X_ADC
  static const uint8_t chanlist[1] =
  {
    0
  };

  struct adc_dev_s *adc;
#endif
#ifdef CONFIG_GD32VW55X_PWM
  struct pwm_lowerhalf_s *pwm;
#endif
#ifdef CONFIG_GD32VW55X_CAPTURE
  struct cap_lowerhalf_s *cap;
#endif
#ifdef CONFIG_MTD_PROGMEM
  struct mtd_dev_s *mtd;
#endif
  int ret = OK;

#ifdef CONFIG_GD32VW55X_WIFI
  /* Initialize the platform + the SDK Wi-Fi stack.  The SDK calls
   * net_if_add() when it creates the VIF, and that is where wlan0 is
   * registered.
   */

  if (gdwifi_start() != 0)
    {
      ferr("ERROR: failed to start Wi-Fi\n");
    }
#endif

#ifdef CONFIG_GD32VW55X_BLE
  /* After Wi-Fi: the platform (clocks, RF, NVDS) is brought up there, and
   * the BLE controller reuses it.
   */

  if (gd32_ble_initialize() < 0)
    {
      ferr("ERROR: failed to start BLE\n");
    }
#endif

#ifdef CONFIG_GD32VW55X_I2C0
  i2c = gd32_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      ferr("ERROR: failed to initialize I2C0\n");
    }
#ifdef CONFIG_I2C_DRIVER
  else if (i2c_register(i2c, 0) < 0)
    {
      ferr("ERROR: failed to register /dev/i2c0\n");
      gd32_i2cbus_uninitialize(i2c);
    }
#endif
#endif

#ifdef CONFIG_GD32VW55X_SPI
  /* The family has a single SPI instance: bus 0 */

  spi = gd32_spibus_initialize(0);
  if (spi == NULL)
    {
      ferr("ERROR: failed to initialize SPI0\n");
    }
#ifdef CONFIG_SPI_DRIVER
  else if (spi_register(spi, 0) < 0)
    {
      ferr("ERROR: failed to register /dev/spi0\n");
    }
#endif
#endif

#ifdef CONFIG_GD32VW55X_ADC
  adc = gd32_adc_initialize(0, chanlist, 1);
  if (adc == NULL)
    {
      ferr("ERROR: failed to initialize the ADC\n");
    }
  else if (adc_register("/dev/adc0", adc) < 0)
    {
      ferr("ERROR: failed to register /dev/adc0\n");
    }
#endif

#ifdef CONFIG_GD32VW55X_PWM
  pwm = gd32_pwminitialize(1);
  if (pwm == NULL)
    {
      ferr("ERROR: failed to initialize PWM on TIMER1\n");
    }
  else if (pwm_register("/dev/pwm0", pwm) < 0)
    {
      ferr("ERROR: failed to register /dev/pwm0\n");
    }
#endif

#ifdef CONFIG_GD32VW55X_CAPTURE
  cap = gd32_cap_initialize(2);
  if (cap == NULL)
    {
      ferr("ERROR: failed to initialize capture on TIMER2\n");
    }
  else if (cap_register("/dev/capture0", cap) < 0)
    {
      ferr("ERROR: failed to register /dev/capture0\n");
    }
#endif

#ifdef CONFIG_MTD_PROGMEM
  /* The progmem region of the internal flash as an MTD device.  It sits
   * below the Wi-Fi NVDS -- see CONFIG_GD32VW55X_PROGMEM_START_ADDR.
   */

  mtd = progmem_initialize();
  if (mtd == NULL)
    {
      ferr("ERROR: progmem_initialize failed\n");
    }
  else
    {
      ret = register_mtddriver("/dev/gd32flash", mtd, 0750, NULL);
      if (ret < 0)
        {
          ferr("ERROR: failed to register /dev/gd32flash: %d\n", ret);
        }

#ifdef CONFIG_FS_LITTLEFS
      /* Mount it as LittleFS, formatting it on the first boot */

      else
        {
          ret = nx_mount("/dev/gd32flash", "/data", "littlefs", 0,
                         "autoformat");
          if (ret < 0)
            {
              ferr("ERROR: failed to mount /data: %d\n", ret);
            }
        }
#endif
    }
#endif

#ifdef CONFIG_USERLED_LOWER
  /* The three board LEDs -> /dev/userleds */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      ferr("ERROR: failed to register /dev/userleds: %d\n", ret);
    }
#endif

#ifdef CONFIG_GD32VW55X_FWDGT
  gd32_fwdgt_initialize("/dev/watchdog0", BOARD_IRC32K_FREQUENCY);
#endif

#ifdef CONFIG_GD32VW55X_WWDGT
  gd32_wwdgt_initialize("/dev/watchdog1");
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

  return ret;
}
