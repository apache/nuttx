/****************************************************************************
 * include/nuttx/sensors/sensors.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_SENSOR_H
#define __INCLUDE_NUTTX_SENSORS_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* sensor type definition */

/* Accelerometer
 * All values are in SI units (m/s^2), and measure the acceleration of the
 * device minus the acceleration dut to gravity.
 */

#define SENSOR_TYPE_ACCELEROMETER                   0

/* Magneric Field
 * All values are in micro-Tesla (uT) and measure the geomagnetic field
 * in X, Y and Z axis.
 */

#define SENSOR_TYPE_MAGNETIC_FIELD                  1

/* Gyroscope
 * All values are in radians/second and measure the rate of rotation around
 * the X, Y and Z axis.
 */

#define SENSOR_TYPE_GYROSCOPE                       2

/* Ambient Light
 * The ambient light sensor value is returned in SI units lux.
 */

#define SENSOR_TYPE_LIGHT                           3

/* Barometer
 * All values are in hectopascal (hPa) and measure the athmospheric pressure
 * we can calculate altitude by perssure.
 */

#define SENSOR_TYPE_BAROMETER                       4

/* Proximity
 * The values correspond to the distance to the nearest
 * object in centimeters.
 */

#define SENSOR_TYPE_PROXIMITY                       5

/* Relative Humidity
 * A relative humidity sensor measure relative ambient air humidity and
 * return a value in percent.
 */

#define SENSOR_TYPE_RELATIVE_HUMIDITY               6

/* Ambient Temperature
 * The ambient (room) temperature in degree Celsius
 */

#define SENSOR_TYPE_AMBIENT_TEMPERATURE             7

/* RGB
 * We use these values of RGB to weighted to obtain the color of LED.
 * These values is in unit percent.
 */

#define SENSOR_TYPE_RGB                             8

/* Hall
 * All values are in bool type (0 or 1) and it often is used to as switch.
 * A values of 1 indicates that switch on.
 */

#define SENSOR_TYPE_HALL                            9

/* IR (Infrared Ray)
 * This sensor can detect a human approach and outputs a signal from
 * interrupt pins. This sensor value is in lux.
 */

#define SENSOR_TYPE_IR                              10

/* GPS
 * A sensor of this type returns gps data. Include year, month, day,
 * hour, minutes, seconds, altitude, longitude, latitude.
 */

#define SENSOR_TYPE_GPS                             11

/* Ultraviolet light sensor
 * This sensor can identify the UV index in ambient light help people
 * to effectively protect themselves from sunburns, cancer or eye damage.
 * This value range is 0 - 15.
 */
#define SENSOR_TYPE_ULTRAVIOLET                     12

/* Noise Loudness
 * A sensor of this type returns the loudness of noise in SI units (db)
 */

#define SENSOR_TYPE_NOISE                           13

/* PM25
 * A sensor of this type returns the content of pm2.5 in the air
 * This value is in SI units (ug/m^3)
 */

#define SENSOR_TYPE_PM25                            14

/* PM1P0
 * A sensor of this type returns the content of pm1.0 in the air
 * This value is in SI units (ug/m^3)
 */

#define SENSOR_TYPE_PM1P0                           15

/* PM10
 * A sensor of this type returns the content of pm10 in the air
 * This value is in SI units (ug/m^3)
 */

#define SENSOR_TYPE_PM10                            16

/* CO2
 * A sensor of this type returns the content of CO2 in the air
 * This vaule is in units (ppm-part per million).
 */

#define SENSOR_TYPE_CO2                             17

/* HCHO
 * The HCHO pollution is an important indicator of household air
 * pollution. This value is in units (ppm-part per million).
 */

#define SENSOR_TYPE_HCHO                            18

/* TVOC (total volatile organic compounds)
 * The indoor TVOC is cause indoor air pollution is one of the
 * main reasons why. This value is in units (ppb-part per billion).
 */

#define SENSOR_TYPE_TVOC                            19

/* PH
 * The acid-base degree describes the strength of the aqueous
 * solution, expressed by pH. In the thermodynamic standard
 * condition, the aqueous solution with pH=7 is neutral,
 * pH<7 is acidic, and pH>7 is alkaline.
 */

#define SENSOR_TYPE_PH                              20

/* Dust
 * A sensor of this type returns the content of dust in the air
 * values is in ug/m^3.
 */

#define SENSOR_TYPE_DUST                            21

/* Heart Rate
 * A sensor of this type returns the current heart rate.
 * Current heart rate is in beats per minute (BPM).
 */

#define SENSOR_TYPE_HEART_RATE                      22

/* Heart Beat
 * A sensor of this type returns an event evetytime
 * a hear beat peek is detected. Peak here ideally corresponds
 * to the positive peak in the QRS complex of and ECG signal.
 */

#define SENSOR_TYPE_HEART_BEAT                      23

/* The total number of sensor */

#define SENSOR_TYPE_COUNT                           24

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t sensor_get_timestamp(void)
{
  struct timespec ts;

#ifdef CONFIG_CLOCK_MONOTONIC
  clock_gettime(CLOCK_MONOTONIC, &ts);
#else
  clock_gettime(CLOCK_REALTIME, &ts);
#endif

  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures prefixed with sensor_event are sensor data, and member
 * that are not used must be written as NAN or INT_MIN/INT_MAX, than
 * reported.
 */

struct sensor_event_accel   /* Type: Accerometer */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in m/s^2 */
  float y;                  /* Axis Y in m/s^2 */
  float z;                  /* Axis Z in m/s^2 */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_event_gyro    /* Type: Gyroscope */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in rad/s */
  float y;                  /* Axis Y in rad/s */
  float z;                  /* Axis Z in rad/s */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_event_mag     /* Type: Magnetic Field */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in Gauss or micro Tesla (uT) */
  float y;                  /* Axis Y in Gauss or micro Tesla (uT) */
  float z;                  /* Axis Z in Gauss or micro Tesla (uT) */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_event_baro    /* Type: Barometer */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pressure;           /* pressure measurement in millibar or hpa */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_event_prox    /* Type: proximity */
{
  uint64_t timestamp;       /* Units is microseconds */
  float proximity;          /* distance to the nearest object in centimeters */
};

struct sensor_event_light   /* Type: Light */
{
  uint64_t timestamp;       /* Units is microseconds */
  float light;              /* in SI lux units */
};

struct sensor_event_humi    /* Type: Relative Humidity */
{
  uint64_t timestamp;       /* Units is microseconds */
  float humidity;           /* in percent  */
};

struct sensor_event_temp    /* Type: Ambient Temperature */
{
  uint64_t timestamp;       /* Units is microseconds */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_event_rgb     /* Type: RGB */
{
  uint64_t timestamp;       /* Units is microseconds */
  float r;                  /* Units is percent */
  float g;                  /* Units is percent */
  float b;                  /* Units is percent */
};

struct sensor_event_hall    /* Type: HALL */
{
  uint64_t timestamp;       /* Units is microseconds */
  bool hall;                /* Boolean type */
};

struct sensor_event_ir      /* Type: Infrared Ray */
{
  uint64_t timestamp;       /* Units is microseconds */
  float ir;                 /* in SI units lux */
};

struct sensor_event_gps     /* Type: Gps */
{
  int year;                 /* Time */
  int month;
  int day;
  int hour;
  int min;
  int sec;
  int msec;

  float yaw;                /* Unit is Si degrees */
  float height;             /* Unit is SI m */
  float speed;              /* Unit is m/s */
  float latitude;           /* Unit is degrees */
  float longitude;          /* Unit is degrees */
};

struct sensor_event_uv      /* Type: Ultraviolet Light */
{
  uint64_t timestamp;       /* Units is microseconds */
  float uvi;                /* the vaule range is 0 - 15 */
};

struct sensor_event_noise   /* Type: Noise Loudness */
{
  uint64_t timestamp;       /* Units is microseconds */
  float db;                 /* in SI units db */
};

struct sensor_event_pm25    /* Type: PM25 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm25;               /* in SI units ug/m^3 */
};

struct sensor_event_pm10    /* Type: PM10 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm10;               /* in SI units ug/m^3 */
};

struct sensor_event_pm1p0   /* Type: PM1P0 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm1p0;              /* in SI units ug/m^3 */
};

struct sensor_event_co2     /* Type: CO2 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float co2;                /* in SI units ppm */
};

struct sensor_event_hcho    /* Type: HCHO */
{
  uint64_t timestamp;       /* Units is microseconds */
  float hcho;               /* in SI units ppm */
};

struct sensor_event_tvoc    /* Type: TVOC */
{
  uint64_t timestamp;       /* Units is microseconds */
  float tvoc;               /* in SI units ppm */
};

struct sensor_event_ph      /* Type: PH */
{
  uint64_t timestamp;       /* Units is microseconds */
  float ph;                 /* PH = 7.0 neutral, PH < 7.0 acidic, PH > 7.0 alkaline */
};

struct sensor_event_dust    /* Type: DUST */
{
  uint64_t timestamp;       /* Units is microseconds */
  float dust;               /* is SI units ug/m^3 */
};

struct sensor_event_hrate   /* Type: Heart Rate */
{
  uint64_t timestamp;       /* Units is microseconds */
  float bpm;                /* is SI units BPM */
};

struct sensor_event_hbeat   /* Type: Heart Beat */
{
  uint64_t timestamp;       /* Units is microseconds */
  float beat;               /* Units is times/minutes */
};

/* The sensor lower half driver interface */

struct sensor_lowerhalf_s;
struct sensor_ops_s
{
  /**************************************************************************
   * Name: activate
   *
   * Description:
   *   Enable or disable sensor device. when enable sensor, sensor will
   *   work in  current mode(if not set, use default mode). when disable
   *   sensor, it will disable sense path and stop convert.
   *
   * Input Parameters:
   *   lower  - The instance of lower half sensor driver
   *   enable - true(enable) and false(disable)
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*activate)(FAR struct sensor_lowerhalf_s *lower, bool enable);

  /**************************************************************************
   * Name: set_interval
   *
   * Description:
   *   Set the sensor output data period in microseconds for a given sensor.
   *   If *period_us > max_delay it will be truncated to max_dealy and if
   *   *period_us < min_delay it will be replaced by min_delay.
   *
   *   Before changing the interval, we need to push the prepared data to
   *   ensure that they are not lost.
   *
   * Input Parameters:
   *   lower     - The instance of lower half sensor driver.
   *   period_us - the time between samples, in us, it may be overwrite by
   *               lower half driver.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*set_interval)(FAR struct sensor_lowerhalf_s *lower,
                           FAR unsigned int *period_us);

  /**************************************************************************
   * Name: batch
   *
   * Description:
   *   Set sensor's maximum report latency in microseconds.
   *
   *   This function can be called while the sensor is activated,
   *   in which case it must not cause any sensor measurements to be lost.
   *   So, it is necessary to flush fifo or read ready data from data
   *   register to prevent data lost before we using batch mode.
   *
   *   This sensor default mode isn't batch mode, so we need call this
   *   function and *latency_us != 0.
   *   If *latency_us > max_report_latency it will be truncated to
   *   max_report_latency and return *latency_us to user
   *   And we must flush fifo data to prevent data lost, then adjust latency.
   *
   *   We can exit batch mode by call this function with *latency_us = 0.
   *   And we must flush fifo data to prevent data lost, then stop batch.
   *
   *   If sensor doesn't support batching (FIFO size zero), set batch to
   *   NULL.
   *
   *   We must set interval by calling set_interval before calling batch(),
   *   othrewise, -EINVAL is returned.
   *
   *   The reason why we don't have flush operation is that we need to push
   *   the prepared data out before adjusting the latency to ensure that the
   *   data will not be lost.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   latency_us - the time between batch data, in us. It may by overwrite
   *                by lower half driver.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*batch)(FAR struct sensor_lowerhalf_s *lower,
                    FAR unsigned int *latency_us);
};

/* This structure is the generic form of state structure used by lower half
 * Sensor driver.
 */

struct sensor_lowerhalf_s
{
  /* The type of sensor device */

  int type;

  /* The bytes length of the circular buffer used.
   * This sensor circular buffer is used to slove issue that application
   * can't read sensor event in time. If this length of buffer is too large,
   * the latency of sensor event will be too larage. If the length of buffer
   * is too small, the event will be overwrite before application read them.
   * So, it's recommended to set according to sensor odr. If odr is low, you
   * can set to a length of sensor event. If odr is high, you can set to two
   * or three length of sensor event.
   */

  uint32_t buffer_bytes;

  /* The uncalibrated use to describe whether the sensor event is
   * uncalibrated. True is uncalibrated data, false is calibrated data,
   * default false.
   */

  bool uncalibrated;

  /* The lower half sensor driver operations */

  FAR const struct sensor_ops_s *ops;

  /**************************************************************************
   * Name: push_event
   *
   * Description:
   *   Lower half driver push sensor event by calling this function.
   *   It is provided by upper half driver to lower half driver.
   *
   * Input Parameters:
   *   priv   - Upper half driver handle
   *   data   - The buffer of event, it can be all type of sensor events.
   *   bytes  - The number of bytes of sensor event
   **************************************************************************/

  CODE void (*push_event)(FAR void *priv, FAR const void *data,
                          uint32_t bytes);

  /* The private opaque pointer to be passed to upper-layer during callback */

  FAR void *priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * "Upper Half" Sensor Driver Interfaces
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the chararter device by node name format based on the
 *   type of sensor. Multiple types of the same type are distinguished by
 *   numbers. eg: accel0, accel1
 *
 * Input Parameters:
 *   dev  - A pointer to an instance of lower half sensor driver. This
 *          instance is bound to the sensor driver and must persists as long
 *          as the driver persists.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_register(FAR struct sensor_lowerhalf_s *dev);

/****************************************************************************
 * Name: sensor_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver.
 *
 * Input Parameters:
 *   dev  - A pointer to an instance of lower half sensor driver. This
 *          instance is bound to the sensor driver and must persists as long
 *          as the driver persists.
 ****************************************************************************/

void sensor_unregister(FAR struct sensor_lowerhalf_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_SENSORS_SENSOR_H */
