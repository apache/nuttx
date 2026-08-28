/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_lsm6ds3trc.c
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

#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/lsm6ds3trc.h>

#include "espressif/esp_gpio.h"
#include "esp32s3_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* INT1/INT2 routing on Seeed's "IMU Breakout Board for XIAO" (XIAOML Kit),
 * confirmed from the board's schematic (IMU_Breakout_Board_for_XIAO_SCH,
 * rev V1.0): LSM6DS3TR-C INT1 -> XIAO GPIO3 (D2), INT2 -> XIAO GPIO4 (D3),
 * each through a 33ohm series resistor, both populated. The chip can OR
 * both DRDY_XL and DRDY_G onto either pin, so only one is used -- INT1 is
 * an arbitrary but fixed choice. GPIO4/INT2 stays unused by the driver.
 */

#define LSM6DS3TRC_IRQ_PIN 3  /* INT1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lsm6ds3trc_attach
 *
 * Description:
 *   Attach (or detach, if handler is NULL) the shared data-ready
 *   interrupt handler to INT1 (XIAO GPIO3).
 *
 ****************************************************************************/

static int board_lsm6ds3trc_attach(xcpt_t handler, FAR void *arg)
{
  int ret;

  esp_gpioirqdisable(LSM6DS3TRC_IRQ_PIN);

  ret = esp_gpio_irq(LSM6DS3TRC_IRQ_PIN, handler, arg);
  if (ret < 0)
    {
      return ret;
    }

  esp_gpioirqenable(LSM6DS3TRC_IRQ_PIN);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lsm6ds3trc_initialize
 *
 * Description:
 *   Initialize and register the LSM6DS3TR-C 6-axis IMU driver, exposing
 *   it through uORB as /dev/uorb/sensor_accelN and /dev/uorb/sensor_gyroN.
 *   Data is delivered from the shared INT1 data-ready interrupt rather
 *   than kthread polling.
 *
 * Input Parameters:
 *   devno - The device number, used to build the uORB device paths
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lsm6ds3trc_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;

  static const struct lsm6ds3trc_config_s config =
  {
    .int_pin = LSM6DS3TRC_INT1,
    .attach = board_lsm6ds3trc_attach,
  };

  /* The IMU drives INT1 push-pull, active high by default (CTRL3_C
   * H_LACTIVE reset value) -- rising edge signals data ready. No pull
   * needed once the sensor drives the line, but PULLDOWN gives a defined
   * idle state before CTRL registers are written during registration.
   */

  esp_configgpio(LSM6DS3TRC_IRQ_PIN, INPUT_FUNCTION_2 | PULLDOWN | RISING);

  i2c = esp32s3_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* SA0 tied low on this board -> 0x6a. Use 0x6b if SA0 is pulled high. */

  return lsm6ds3trc_register(i2c, 0x6a, devno,
                             (FAR struct lsm6ds3trc_config_s *)&config);
}
