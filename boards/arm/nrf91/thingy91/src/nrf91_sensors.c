/****************************************************************************
 * boards/arm/nrf91/thingy91/src/nrf91_sensors.c
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

#ifdef CONFIG_SENSORS_BH1749NUC
#  include "nrf91_bh1749nuc.h"
#endif

#ifdef CONFIG_SENSORS_BME680
#  include "nrf91_bme680.h"
#endif

#ifdef CONFIG_SENSORS_ADXL362
#  include "nrf91_adxl362.h"
#endif

#ifdef CONFIG_SENSORS_ADXL372
#  include "nrf91_adxl372.h"
#endif

#include "thingy91.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BH1749NUC_I2C_ADDR 0x38
#define BH1749NUC_I2C_BUS  2

#define BME680_I2C_BUS     2

#define ADXL372_SPI_BUS    1
#define ADXL362_SPI_BUS    1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_sensors_init
 *
 * Description:
 *   Initialzie on-board sensors
 *
 ****************************************************************************/

int nrf91_sensors_init(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_SENSORS_BH1749NUC
  nrf91_gpio_config(GPIO_BH1749_INT);

  /* Initialize BH1749NUC */

  ret = nrf91_bh1749nuc_init(0, BH1749NUC_I2C_BUS, BH1749NUC_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf91_bh1749nuc_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BME680
  /* Initialize BME680 */

  ret = nrf91_bme680_init(0, BME680_I2C_BUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf91_bme680_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ADXL372
  nrf91_gpio_config(GPIO_ADXL372_INT1);

  /* Initialize ADXL372 */

  ret = nrf91_adxl372_init(ADXL372_SPI_DEVNO, ADXL372_SPI_BUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf91_adxl372_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ADXL362
  nrf91_gpio_config(GPIO_ADXL362_INT1);

  /* Initialize ADXL362 */

  ret = nrf91_adxl362_init(ADXL362_SPI_DEVNO, ADXL362_SPI_BUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf91_adxl362_init failed: %d\n", ret);
    }
#endif

  return ret;
}
