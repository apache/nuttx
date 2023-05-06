/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c_bringup.c
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

#include <stdio.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi160.h>
#include <nuttx/sensors/mpu60x0.h>
#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>
#include <nuttx/fs/fs.h>

#include <nuttx/timers/pwm.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include "tiva_i2c.h"
#include "tiva_pwm.h"
#include "tiva_qencoder.h"
#include "tm4c1294-launchpad.h"

#ifdef HAVE_USERLED_DRIVER
#  include <nuttx/leds/userled.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DK_TM4C129X_TIMER
#  define HAVE_TIMER
#endif

#ifdef CONFIG_TM4C1294_LAUNCHPAD_PWM
#  define HAVE_PWM
#endif

#if defined(CONFIG_TIVA_QEI0) || defined(CONFIG_TIVA_QEI1)
#  define HAVE_QEI
#endif

#define PWM_PATH_FMT        "/dev/pwm%d"
#define PWM_PATH_FMTLEN     (10)

#define QEI_PATH_FMT        "/dev/qei%d"
#define QEI_PATH_FMTLEN     (10)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void tm4c_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = tiva_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          tiva_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: tm4c_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void tm4c_i2ctool(void)
{
#ifdef CONFIG_TIVA_I2C0
  tm4c_i2c_register(0);
#endif
#ifdef CONFIG_TIVA_I2C1
  tm4c_i2c_register(1);
#endif
#ifdef CONFIG_TIVA_I2C2
  tm4c_i2c_register(2);
#endif
#ifdef CONFIG_TIVA_I2C3
  tm4c_i2c_register(3);
#endif
#ifdef CONFIG_TIVA_I2C4
  tm4c_i2c_register(4);
#endif
#ifdef CONFIG_TIVA_I2C5
  tm4c_i2c_register(5);
#endif
#ifdef CONFIG_TIVA_I2C6
  tm4c_i2c_register(6);
#endif
#ifdef CONFIG_TIVA_I2C7
  tm4c_i2c_register(7);
#endif
#ifdef CONFIG_TIVA_I2C8
  tm4c_i2c_register(8);
#endif
#ifdef CONFIG_TIVA_I2C9
  tm4c_i2c_register(9);
#endif
}
#else
#  define tm4c_i2ctool()
#endif

/****************************************************************************
 * Name: tm4c_bmi160_setup
 *
 * Description:
 *   Initialize and register the bmi160 sensor.
 *
 * Input Parameters:
 *   bus - A number identifying the I2C bus.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, a negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160
static int tm4c_bmi160_setup(int bus)
{
  int ret;
  struct i2c_master_s *i2c;

  /* Initialize i2c device */

  i2c = tiva_i2cbus_initialize(bus);
  if (!i2c)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = bmi160_register("/dev/accel0", i2c);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Error registering BMI160\n");
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: tm4c_mpu60x0_setup
 *
 * Description:
 *   Initialize and register the Invensense MPU60x0 sensor.
 *
 * Input Parameters:
 *   bus - A number identifying the I2C bus.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On failure, a negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MPU60X0
static int tm4c_mpu60x0_setup(int bus)
{
  int ret;
  struct i2c_master_s *i2c;
  struct mpu_config_s *mpu_config;

  /* Initialize i2c device */

  i2c = tiva_i2cbus_initialize(bus);
  if (!i2c)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
  if (mpu_config == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to allocate mpu60x0 driver\n");
      return -ENOMEM;
    }
    else
    {
      mpu_config->i2c = i2c;
      mpu_config->addr = 0x68;
      ret = mpu60x0_register("/dev/imu0", mpu_config);
      if (ret < 0)
        {
          syslog(LOG_ERR, "Error registering MPU60X0\n");
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: tm4c_pwm_register
 *
 * Description:
 *   Register a PWM dev file with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the PWM channel use.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef HAVE_PWM
static void tm4c_pwm_register(int channel)
{
  struct pwm_lowerhalf_s *dev;
  char pwm_path[PWM_PATH_FMTLEN];
  int ret;

  dev = tiva_pwm_initialize(channel);
  if (dev == NULL)
    {
      _err("ERROR: Failed to get PWM%d interface\n", channel);
    }
  else
    {
      snprintf(pwm_path, PWM_PATH_FMTLEN, PWM_PATH_FMT, channel);
      ret = pwm_register(pwm_path, dev);
      if (ret < 0)
        {
          _err("ERROR: Failed to register PWM%d driver: %d\n",
                 channel, ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: tm4c_pwm
 *
 * Description:
 *   Register PWM drivers for the PWM tool.
 *
 ****************************************************************************/

#ifdef HAVE_PWM
static void tm4c_pwm(void)
{
#ifdef CONFIG_TIVA_PWM0_CHAN0
  tm4c_pwm_register(0);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN1
  tm4c_pwm_register(1);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN2
  tm4c_pwm_register(2);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN3
  tm4c_pwm_register(3);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN4
  tm4c_pwm_register(4);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN5
  tm4c_pwm_register(5);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN6
  tm4c_pwm_register(6);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN7
  tm4c_pwm_register(7);
#endif
}
#endif

/****************************************************************************
 * Name: tm4c_qei_register
 *
 * Description:
 *   Register a QEI dev file with the upper_level QEI driver.
 *
 * Input Parameters:
 *   id - A number identifying the QEI.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef HAVE_QEI
static void tm4c_qei_register(int id)
{
  struct qe_lowerhalf_s *dev;
  int ret;
  char qe_path[QEI_PATH_FMTLEN];

  dev = tiva_qei_initialize(id);
  if (dev == NULL)
    {
      _err("ERROR: Failed to get QEI %d\n", id);
    }
  else
    {
      snprintf(qe_path, QEI_PATH_FMTLEN, QEI_PATH_FMT, id);
      ret = qe_register(qe_path, dev);
      if (ret < 0)
        {
          _err("ERROR: Failed to register QEI %d driver: %d\n", id, ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: tm4c_qei
 *
 * Description:
 *   Register QEI drivers for the QEI tool.
 *
 ****************************************************************************/

#ifdef HAVE_QEI
static void tm4c_qei(void)
{
#ifdef CONFIG_TIVA_QEI0
  tm4c_qei_register(0);
#endif
#ifdef CONFIG_TIVA_QEI1
  tm4c_qei_register(1);
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int tm4c_bringup(void)
{
  int ret;

  /* Register I2C drivers on behalf of the I2C tool */

  tm4c_i2ctool();

#ifdef CONFIG_SENSORS_BMI160
#if defined(CONFIG_BOOSTXL_SENSORS_1)

  ret = tm4c_bmi160_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: BMI160 on 1 failed %d\n", ret);
    }
#endif
#if defined(CONFIG_BOOSTXL_SENSORS_2)

  ret = tm4c_bmi160_setup(2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: BMI160 on 2 failed %d\n", ret);
    }
#endif
#endif

#ifdef CONFIG_SENSORS_MPU60X0
  ret = tm4c_mpu60x0_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MPU60X0 on I2C0 failed %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, TIVA_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             TIVA_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef HAVE_PWM
  /* Register PWM drivers */

  tm4c_pwm();
#endif

#ifdef HAVE_QEI
  /* Register QEI drivers */

  tm4c_qei();
#endif

#ifdef CONFIG_TIVA_CAN
  /* Initialize CAN module and register the CAN driver(s) */

  ret = tm4c_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_can_setup failed %d\n", ret);
    }
#endif

#ifdef HAVE_TIMER
  /* Initialize the timer driver */

  ret = tiva_timer_configure();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize timer driver: %d\n", ret);
    }
#endif

#ifdef HAVE_HCIUART
  /* Register the Bluetooth HCI UART device */

  ret = hciuart_dev_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize HCI UART driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_USERLED_DRIVER
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n",
             ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
