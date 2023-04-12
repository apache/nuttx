/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_bringup.c
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

#include <nuttx/fs/fs.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_NRF52_SOFTDEVICE_CONTROLLER
#  include "nrf52_sdc.h"
#endif

#include "nrf52840-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF52_TIMER (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void nrf52_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = nrf52_i2cbus_initialize(bus);
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
          nrf52_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: nrf52_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void nrf52_i2ctool(void)
{
#ifdef CONFIG_NRF52_I2C0
  nrf52_i2c_register(0);
#endif
#ifdef CONFIG_NRF52_I2C1
  nrf52_i2c_register(1);
#endif
}
#endif

/****************************************************************************
 * Name: nrf52_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf52_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, NRF52_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize(CONFIG_EXAMPLES_LEDS_DEVPATH);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: userled_lower_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_LSM6DSL
  ret = nrf52_lsm6dsl_initialize("/dev/lsm6dsl0");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize LSM6DSL driver: %d\n",
             ret);
    }
#endif /* CONFIG_SENSORS_LSM6DSL */

#ifdef CONFIG_SENSORS_LSM303AGR
  ret = nrf52_lsm303agr_initialize("/dev/lsm303agr0");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize LSM303AGR driver: %d\n",
             ret);
    }
#endif /* CONFIG_SENSORS_LSM303AGR */

#ifdef CONFIG_SENSORS_HTS221
  ret = nrf52_hts221_initialize("/dev/hts2210");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize HTS221 driver: %d\n",
             ret);
    }
#endif /* CONFIG_SENSORS_HTS221 */

#ifdef CONFIG_LPWAN_SX127X
  ret = nrf52_lpwaninitialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize wireless driver: %d\n",
             ret);
    }
#endif /* CONFIG_LPWAN_SX127X */

#if defined(CONFIG_TIMER) && defined(CONFIG_NRF52_TIMER)
  /* Configure TIMER driver */

  ret = nrf52_timer_driver_setup("/dev/timer0", NRF52_TIMER);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Configure PWM driver */

  ret = nrf52_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize PWM driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Configure ADC driver */

  ret = nrf52_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize ADC driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#if defined(CONFIG_RNDIS) && !defined(CONFIG_RNDIS_COMPOSITE)
  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_NRF52_QSPI
  /* Initialize the MX25 QSPU memory */

  ret = nrf52_mx25_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf52_mx25_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NRF52_SOFTDEVICE_CONTROLLER
  ret = nrf52_sdc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf52_sdc_initialize() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
