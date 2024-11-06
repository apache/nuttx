/****************************************************************************
 * boards/arm/nrf53/thingy53/src/nrf53_sensors.c
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
#include <sys/types.h>
#include <syslog.h>

#ifdef CONFIG_SENSORS_BMI270
#  include "nrf53_bmi270.h"
#endif

#ifdef CONFIG_SENSORS_ADXL362
#  include "nrf53_adxl362.h"
#endif

#ifdef CONFIG_SENSORS_BH1749NUC
#  include "nrf53_bh1749nuc.h"
#endif

#ifdef CONFIG_SENSORS_BMM150
#  include "nrf53_bmm150.h"
#endif

#include "arm_internal.h"

#include "thingy53.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMM150_I2C_BUS     (2)
#define BMM150_I2C_ADDR    (0x10)

#define BH1749NUC_I2C_BUS  (2)
#define BH1749NUC_I2C_ADDR (0x38)

#define BME688_I2C_BUS     (2)
#define BME688_I2C_ADDR    (0x76)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_sensors_init
 *
 * Description:
 *   Initialize on-board sensors
 *
 ****************************************************************************/

int nrf53_sensors_init(void)
{
  int ret = OK;

  UNUSED(ret);

  /* Enable sensor power.
   * TODO: this should be controlled by power management logic
   */

  nrf53_gpio_config(GPIO_SENS_PWRCTRL);
  nrf53_gpio_write(GPIO_SENS_PWRCTRL, false);
  up_mdelay(10);
  nrf53_gpio_write(GPIO_SENS_PWRCTRL, true);
  up_mdelay(10);

#ifdef CONFIG_SENSORS_BMI270
  /* Initialize BMI270 */

  ret = nrf53_bmi270spi_initialize(0, BMI270_SPIDEV);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_bmi270_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BH1749NUC
  /* Initialize BH1749NUC */

  ret = nrf53_bh1749nuc_init(0, BH1749NUC_I2C_BUS, BH1749NUC_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_bh1749nuc_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMM150
  /* Initialize BMM150 */

  ret = nrf53_bmm150_init(0, BMM150_I2C_BUS, BMM150_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_bmm150_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ADXL362
  /* Initialize ADXL362 */

  ret = nrf53_adxl362_init(1, ADXL362_SPIDEV);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_adxl362_init failed: %d\n", ret);
    }
#endif

  return ret;
}
