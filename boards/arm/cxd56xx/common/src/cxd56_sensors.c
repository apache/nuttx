/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_sensors.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef SENSOR_I2C
#  error "SENSOR_I2C must be defined in board.h !!"
#endif
#ifndef SENSOR_SPI
#  error "SENSOR_SPI must be defined in board.h !!"
#endif

/* Configuration Sanity check */

#if defined(CONFIG_SENSORS_BMI160) || defined(CONFIG_SENSORS_BMI160_SCU)
#  define _BMI160  1
#else
#  define _BMI160  0
#endif

#if defined(CONFIG_SENSORS_KX022) || defined(CONFIG_SENSORS_KX022_SCU)
#  define _KX022  1
#else
#  define _KX022  0
#endif

#if defined(CONFIG_SENSORS_BMP280) || defined(CONFIG_SENSORS_BMP280_SCU)
#  define _BMP280  1
#else
#  define _BMP280  0
#endif

#if defined(CONFIG_SENSORS_BM1383GLV)&& defined(CONFIG_SENSORS_BM1383GLV_SCU)
#  define _BM1383GLV  1
#else
#  define _BM1383GLV  0
#endif

#if defined(CONFIG_SENSORS_AK09912) ||  defined(CONFIG_SENSORS_AK09912_SCU)
#  define _AK09912  1
#else
#  define _AK09912  0
#endif

#if defined(CONFIG_SENSORS_BM1422GMV) || defined(CONFIG_SENSORS_BM1422GMV_SCU)
#  define _BM1422GMV  1
#else
#  define _BM1422GMV  0
#endif

#if defined(CONFIG_SENSORS_APDS9930) || defined (CONFIG_SENSORS_APDS9930_SCU)
#  define _APDS9930  1
#else
#  define _APDS9930  0
#endif

#if defined(CONFIG_SENSORS_LT1PA01) || defined(CONFIG_SENSORS_LT1PA01_SCU)
#  define _LT1PA01  1
#else
#  define _LT1PA01  0
#endif

#if defined(CONFIG_SENSORS_BH1721FVC) || defined(CONFIG_SENSORS_BH1721FVC_SCU)
#  define _BH1721FVC  1
#else
#  define _BH1721FVC  0
#endif

#if defined(CONFIG_SENSORS_RPR0521RS) || defined(CONFIG_SENSORS_RPR0521RS_SCU)
#  define _RPR0521RS  1
#else
#  define _RPR0521RS  0
#endif

#if (_BMI160 + _KX022) > 1
#  error "Duplicate accelerometer sensor device."
#endif

#if (_AK09912 + _BM1422GMV) > 1
#  error "Duplicate magnetic sensor device."
#endif

#if (_BMP280 + _BM1383GLV) > 1
#  error "Duplicate pressure sensor device."
#endif

#if (_APDS9930 + _LT1PA01 + _BH1721FVC + _RPR0521RS) > 1
# error "Duplicate proximity and ambient light sensor device."
#endif

/* Sensor Device Registration Macro */

#define _DEVICE_WOPATH(_name, _bus) \
  { \
    .name = #_name, \
    .devpath = NULL, \
    .bus = _bus, \
    { \
      .init = board_ ## _name ##_initialize, \
    }, \
  }

#define _DEVICE(_name, _path, _bus) \
  { \
    .name = #_name, \
    .devpath = _path, \
    .bus = _bus, \
    { \
      .initdev = board_ ## _name ##_initialize, \
    }, \
  }

#define _I2C_DEVICE(_name, _path) _DEVICE(_name, _path, SENSOR_I2C)
#define _SPI_DEVICE(_name, _path) _DEVICE(_name, _path, SENSOR_SPI)

#define _I2C_DEVICE_WOPATH(_name) _DEVICE_WOPATH(_name, SENSOR_I2C)
#define _SPI_DEVICE_WOPATH(_name) _DEVICE_WOPATH(_name, SENSOR_SPI)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*_init_t)(int bus);
typedef int (*_initdev_t)(const char *devpath, int bus);

struct sensor_device_s
{
  const char    *name;    /* Sensor device name */
  const char    *devpath; /* Sensor device path */
  int           bus;      /* I2C or SPI bus number */
  union
  {
    _init_t     init;     /* Sensor initializer w/o devpath */
    _initdev_t  initdev;  /* Sensor initializer with devpath */
  } init_u;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sensor_device_s sensor_device[] =
{
#if defined(CONFIG_SENSORS_BMI160) || defined(CONFIG_SENSORS_BMI160_SCU)
#  if defined(CONFIG_SENSORS_BMI160_I2C) || defined(CONFIG_SENSORS_BMI160_SCU_I2C)
  _I2C_DEVICE_WOPATH(bmi160), /* Accel + Gyro */
#  else /* CONFIG_SENSORS_BMI160_SPI */
  _SPI_DEVICE_WOPATH(bmi160),
#  endif
#endif
#if defined(CONFIG_SENSORS_KX022) || defined(CONFIG_SENSORS_KX022_SCU)
  _I2C_DEVICE(kx022, "/dev/accel"), /* Accel */
#endif
#if defined(CONFIG_SENSORS_BMP280) || defined(CONFIG_SENSORS_BMP280_SCU)
  _I2C_DEVICE_WOPATH(bmp280), /* Pressure */
#endif
#if defined(CONFIG_SENSORS_BM1383GLV) || defined(CONFIG_SENSORS_BM1383GLV_SCU)
  _I2C_DEVICE(bm1383glv, "/dev/press"),
#endif
#if defined(CONFIG_SENSORS_AK09912) || defined(CONFIG_SENSORS_AK09912_SCU)
  _I2C_DEVICE(ak09912, "/dev/mag"), /* Magnetic */
#endif
#if defined(CONFIG_SENSORS_BM1422GMV) || defined(CONFIG_SENSORS_BM1422GMV_SCU)
  _I2C_DEVICE(bm1422gmv, "/dev/mag"),
#endif
#if defined(CONFIG_SENSORS_APDS9930) || defined(CONFIG_SENSORS_APDS9930_SCU)
  _I2C_DEVICE_WOPATH(apds9930), /* Proximity + Light */
#endif
#if defined(CONFIG_SENSORS_LT1PA01) || defined(CONFIG_SENSORS_LT1PA01_SCU)
  _I2C_DEVICE_WOPATH(lt1pa01),
#endif
#if defined(CONFIG_SENSORS_BH1721FVC) || defined(CONFIG_SENSORS_BH1721FVC_SCU)
  _I2C_DEVICE(bh1721fvc, "/dev/light"),
#endif
#if defined(CONFIG_SENSORS_RPR0521RS) || defined(CONFIG_SENSORS_RPR0521RS_SCU)
  _I2C_DEVICE_WOPATH(rpr0521rs),
#endif
#if defined(CONFIG_SENSORS_APDS9960) || defined(CONFIG_SENSORS_APDS9960_SCU)
  _I2C_DEVICE(apds9960, "/dev/gesture"), /* Gesture */
#endif
#if defined(CONFIG_SENSORS_BH1745NUC) || defined(CONFIG_SENSORS_BH1745NUC_SCU)
  _I2C_DEVICE(bh1745nuc, "/dev/color"), /* Color */
#endif
#if defined(CONFIG_SENSORS_SCD41)
  _I2C_DEVICE(scd41, "/dev/co2"), /* CO2 */
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sensors_initialize
 *
 * Description:
 *   Perform sensor devices initialization
 *
 ****************************************************************************/

int board_sensors_initialize(void)
{
  int ret = 0;
  int i;
  struct sensor_device_s *dev;

  ret = board_power_control(POWER_SENSOR, true);
  if (ret)
    {
      _err("Failed to power on sensor: %d\n", ret);
      return -EPERM;
    }

  /* Wait for power-up max time */

  up_mdelay(10);

  /* Initialize each sensor device */

  for (i = 0; i < sizeof(sensor_device) / sizeof(sensor_device[0]); i++)
    {
      dev = &sensor_device[i];
      if (dev->devpath)
        {
          ret = dev->init_u.initdev(dev->devpath, dev->bus);
        }
      else
        {
          ret = dev->init_u.init(dev->bus);
        }

      if (ret < 0)
        {
          _err("Failed to init %s at bus %d: %d\n",
                dev->name, dev->bus, ret);
        }
    }

  return ret;
}
