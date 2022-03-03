/****************************************************************************
 * include/nuttx/sensors/sensor.h
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

/* Custom Sensor
 * Some special sensor whose event size is not fixed or dynamically change,
 * are called sensor of custom type. You should treat its events as byte
 * streams and use sensor_custom_register to register character device
 * with specific path, ex: "/dev/sensor/custom_dummy".
 */

#define SENSOR_TYPE_CUSTOM                          0

/* Accelerometer
 * All values are in SI units (m/s^2), and measure the acceleration of the
 * device minus the acceleration dut to gravity.
 */

#define SENSOR_TYPE_ACCELEROMETER                   1

/* Magneric Field
 * All values are in micro-Tesla (uT) and measure the geomagnetic field
 * in X, Y and Z axis.
 */

#define SENSOR_TYPE_MAGNETIC_FIELD                  2

/* Gyroscope
 * All values are in radians/second and measure the rate of rotation around
 * the X, Y and Z axis.
 */

#define SENSOR_TYPE_GYROSCOPE                       3

/* Ambient Light
 * The ambient light sensor value is returned in SI units lux.
 */

#define SENSOR_TYPE_LIGHT                           4

/* Barometer
 * All values are in hectopascal (hPa) and measure the athmospheric pressure.
 * You can calculate altitude by perssure.
 */

#define SENSOR_TYPE_BAROMETER                       5

/* Proximity
 * The values correspond to the distance to the nearest
 * object in centimeters.
 */

#define SENSOR_TYPE_PROXIMITY                       6

/* Relative Humidity
 * A relative humidity sensor measure relative ambient air humidity and
 * return a value in percent.
 */

#define SENSOR_TYPE_RELATIVE_HUMIDITY               7

/* Ambient Temperature
 * The ambient (room) temperature in degree Celsius
 */

#define SENSOR_TYPE_AMBIENT_TEMPERATURE             8

/* RGB
 * We use these values of RGB to weighted to obtain the color of LED.
 * These values is in unit percent.
 */

#define SENSOR_TYPE_RGB                             9

/* Hall
 * All values are in bool type (0 or 1) and it often is used to as switch.
 * A values of 1 indicates that switch on.
 */

#define SENSOR_TYPE_HALL                            10

/* IR (Infrared Ray)
 * This sensor can detect a human approach and outputs a signal from
 * interrupt pins. This sensor value is in lux.
 */

#define SENSOR_TYPE_IR                              11

/* GPS
 * A sensor of this type returns gps data. Include latitude, longitude,
 * altitude, horizontal position accuracy, vertical position accuracy,
 * horizontal dilution of precision, vertical dilution of precision...
 */

#define SENSOR_TYPE_GPS                             12

/* Ultraviolet light sensor
 * This sensor can identify the UV index in ambient light help people
 * to effectively protect themselves from sunburns, cancer or eye damage.
 * This value range is 0 - 15.
 */
#define SENSOR_TYPE_ULTRAVIOLET                     13

/* Noise Loudness
 * A sensor of this type returns the loudness of noise in SI units (db)
 */

#define SENSOR_TYPE_NOISE                           14

/* PM25
 * A sensor of this type returns the content of pm2.5 in the air
 * This value is in SI units (ug/m^3)
 */

#define SENSOR_TYPE_PM25                            15

/* PM1P0
 * A sensor of this type returns the content of pm1.0 in the air
 * This value is in SI units (ug/m^3)
 */

#define SENSOR_TYPE_PM1P0                           16

/* PM10
 * A sensor of this type returns the content of pm10 in the air
 * This value is in SI units (ug/m^3)
 */

#define SENSOR_TYPE_PM10                            17

/* CO2
 * A sensor of this type returns the content of CO2 in the air
 * This value is in units (ppm-part per million).
 */

#define SENSOR_TYPE_CO2                             18

/* HCHO
 * The HCHO pollution is an important indicator of household air
 * pollution. This value is in units (ppm-part per million).
 */

#define SENSOR_TYPE_HCHO                            19

/* TVOC (total volatile organic compounds)
 * The indoor TVOC is cause indoor air pollution is one of the
 * main reasons why. This value is in units (ppb-part per billion).
 */

#define SENSOR_TYPE_TVOC                            20

/* PH
 * The acid-base degree describes the strength of the aqueous
 * solution, expressed by pH. In the thermodynamic standard
 * condition, the aqueous solution with pH=7 is neutral,
 * pH<7 is acidic, and pH>7 is alkaline.
 */

#define SENSOR_TYPE_PH                              21

/* Dust
 * A sensor of this type returns the content of dust in the air
 * values is in ug/m^3.
 */

#define SENSOR_TYPE_DUST                            22

/* Heart Rate
 * A sensor of this type returns the current heart rate.
 * Current heart rate is in beats per minute (BPM).
 */

#define SENSOR_TYPE_HEART_RATE                      23

/* Heart Beat
 * A sensor of this type returns an event evetytime
 * a hear beat peek is detected. Peak here ideally corresponds
 * to the positive peak in the QRS complex of and ECG signal.
 */

#define SENSOR_TYPE_HEART_BEAT                      24

/* ECG (Electrocardiogram)
 * A sensor of this type returns the ECG voltage in μV. Sensors may amplify
 * the input ECG signal. Here the ECG voltage is the un-amplified ECG
 * voltage.
 */

#define SENSOR_TYPE_ECG                             25

/* PPG Dual (2-channel photoplethysmography)
 * A sensor of this type returns the 2 channels PPG measurements in ADC
 * counts and their corresponding LED current and ADC gains. The PPG
 * measurements come from photodiodes and following current amplifiers and
 * ADCs, where a photodiode switches reflected light intensity to current.
 * The LED current decides the lightness of LED, which is the input of PPG
 * measurements. The ADC gains are multipled on the output and affect SNR.
 */

#define SENSOR_TYPE_PPGD                            26

/* PPG Quad (4-channel photoplethysmography)
 * A sensor of this type returns the 4 channels PPG measurements in ADC
 * counts and their corresponding LED current and ADC gains. The PPG
 * measurements come from photodiodes and following current amplifiers and
 * ADCs, where a photodiode switches reflected light intensity to current.
 * The LED current decides the lightness of LED, which is the input of PPG
 * measurements. The ADC gains are multipled on the output and affect SNR.
 */

#define SENSOR_TYPE_PPGQ                            27

/* Imdepance
 * A sensor of this type returns the impedance measurements. An impedance
 * is a complex number, which consists of a real part(resistance) and an
 * imaginary part(reactance). Both of them are in uint Ohm(Ω).
 */

#define SENSOR_TYPE_IMPEDANCE                       28

/* OTS (Optical tracking sensor)
 * A sensor of this type returns the OTS measurements in counts. It
 * integrates an optical chip and a LASER light source in a single miniature
 * package. It provies wide depth of field range on glossy surface, and
 * design flexibility into a compact device.
 */

#define SENSOR_TYPE_OTS                             29

/* Sensor of gps satellite
 * A sensor of this type returns the gps satellite information.
 */

#define SENSOR_TYPE_GPS_SATELLITE                   30

/* Wake gesture
 * A sensor enabling waking up the device based on a device specific
 * motion. 0: the device should sleep, 1: the device should wake up.
 * Other values ​​are uncalibrated values ​​reported by the driver to
 * uncalibrated topics.
 */

#define SENSOR_TYPE_WAKE_GESTURE                    31

/* CAP (Capacitive proximity sensor)
 * The purpose of the proximity sensing interface is to detect when a
 * conductive object (usually a body part i.e. finger, palm, face, etc.)
 * is in the proximity of the system.
 */

#define SENSOR_TYPE_CAP                             32

/* The total number of sensor */

#define SENSOR_TYPE_COUNT                           33

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t sensor_get_timestamp(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
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
  uint64_t timestamp;       /* Time since system start, Units is microseconds */

  /* This is the timestamp which comes from the gps module. It might be
   * unavailable right after cold start, indicated by a value of 0,
   * Units is microseconds
   */

  uint64_t time_utc;

  float latitude;           /* Unit is degrees */
  float longitude;          /* Unit is degrees */
  float altitude;           /* Altitude above MSL(mean seal level), Unit is SI m */
  float altitude_ellipsoid; /* Altitude bove Ellipsoid, Unit is SI m */

  float eph;                /* GPS horizontal position accuracy (metres) */
  float epv;                /* GPS vertical position accuracy (metres) */

  float hdop;               /* Horizontal dilution of precision */
  float vdop;               /* Vertical dilution of precision */

  float ground_speed;       /* GPS ground speed, Unit is m/s */

  /* Course over ground (NOT heading, but direction of movement),
   * Unit is Si degrees
   */

  float course;

  uint32_t satellites_used; /* Number of satellites used */
};

struct sensor_event_uv      /* Type: Ultraviolet Light */
{
  uint64_t timestamp;       /* Units is microseconds */
  float uvi;                /* the value range is 0 - 15 */
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

struct sensor_event_ecg     /* Type: ECG */
{
  uint64_t timestamp;       /* Unit is microseconds */
  float ecg;                /* Unit is μV */
};

struct sensor_event_ppgd    /* Type: PPGD */
{
  uint64_t timestamp;       /* Unit is microseconds */
  uint32_t ppg[2];          /* PPG from 2 channels. Units are ADC counts. */
  uint32_t current;         /* LED current. Unit is uA. */
  uint16_t gain[2];         /* ADC gains of channels. Units are V/V or V/A. */
};

struct sensor_event_ppgq    /* Type: PPDQ */
{
  uint64_t timestamp;       /* Unit is microseconds */
  uint32_t ppg[4];          /* PPG from 4 channels. Units are ADC counts. */
  uint32_t current;         /* LED current. Unit is uA. */
  uint16_t gain[4];         /* ADC gains of channels. Units are V/V or V/A. */
};

struct sensor_event_impd    /* Type: Impedance */
{
  uint64_t timestamp;       /* Unit is microseconds */
  float real;               /* Real part, unit is Ohm(Ω) */
  float imag;               /* Imaginary part, unit is Ohm(Ω) */
};

struct sensor_event_ots     /* Type: OTS */
{
  uint64_t timestamp;       /* Unit is microseconds */
  int32_t x;                /* Axis X in counts */
  int32_t y;                /* Axis Y in counts */
};

struct sensor_event_gps_satellite
{
  uint64_t timestamp;       /* Time since system start, Units is microseconds */
  uint32_t count;           /* Total number of messages of satellites visible */
  uint32_t satellites;      /* Total number of satellites in view */

  struct satellite
  {
    uint32_t svid;          /* Space vehicle ID */

  /* Elevation (0: right on top of receiver,
   * 90: on the horizon) of satellite
   */

    uint32_t elevation;

    /* Direction of satellite, 0: 0 deg, 255: 360 deg. */

    uint32_t azimuth;

  /* dBHz, Signal to noise ratio of satellite C/N0, range 0..99,
   * zero when not tracking this satellite
   */

    uint32_t snr;
  }
  info[4];
};

struct sensor_event_wake_gesture     /* Type: Wake gesture */
{
  uint64_t timestamp;                /* Units is microseconds */

  /* wake gesture event, 0: sleep, 1: wake,
   * others: Uncalibrated status value.
   */

  uint32_t event;
};

struct sensor_event_cap     /* Type: Capacitance */
{
  uint64_t timestamp;       /* Unit is microseconds */
  int32_t status;           /* Detection status */
  int32_t rawdata[4];       /* in SI units pF */
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
   *   If *period_us > max_delay it will be truncated to max_delay and if
   *   *period_us < min_delay it will be replaced by min_delay.
   *
   *   The lower-half can update update *period_us to reflect the actual
   *   period in case the value is rounded up to nearest supported value.
   *
   *   Before changing the interval, you need to push the prepared data to
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
                           FAR unsigned long *period_us);

  /**************************************************************************
   * Name: batch
   *
   * Description:
   *   Set sensor's maximum report latency in microseconds.
   *
   *   This function can be called while the sensor is activated,
   *   in which case it must not cause any sensor measurements to be lost.
   *   So, it is necessary to flush fifo or read ready data from data
   *   register to prevent data lost before you using batch mode.
   *
   *   This sensor default mode isn't batch mode, so you need call this
   *   function and *latency_us != 0.
   *   If *latency_us > max_report_latency it will be truncated to
   *   max_report_latency and return *latency_us to user
   *   And you must flush fifo data to prevent data lost, then adjust
   *   latency.
   *
   *   You can exit batch mode by call this function with *latency_us = 0.
   *   And you must flush fifo data to prevent data lost, then stop batch.
   *
   *   If sensor doesn't support batching (FIFO size zero), set batch to
   *   NULL.
   *
   *   You must set interval by calling set_interval before calling batch(),
   *   othrewise, -EINVAL is returned.
   *
   *   The reason why you don't have flush operation is that you need to push
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
                    FAR unsigned long *latency_us);

  /**************************************************************************
   * Name: fetch
   *
   * Fetch sensor register data by this function. It will use buffer of
   * userspace provided and disables intermediate buffer of upper half. It's
   * recommend that the lowerhalf driver writer to use this function for
   * slower sensor ODR (output data rate) of sensor because this way saves
   * space and it's simple.
   *
   * If fetch isn't NULL, upper half driver will disable intermediate
   * buffer and userspace can't set buffer size by ioctl.
   *
   * You can call this function to read sensor register data by I2C/SPI bus
   * when open mode is non-block, and poll are always successful.
   * When you call this function and open mode is block, you will wait
   * until sensor data ready, then read sensor data.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   buffer     - The buffer of receive sensor event, it's provided by
   *                file_operation::sensor_read.
   *   buflen     - The size of buffer.
   *
   * Returned Value:
   *   The size of read buffer returned on success; a negated errno value
   *   on failure.
   *
   **************************************************************************/

  CODE int (*fetch)(FAR struct sensor_lowerhalf_s *lower,
                    FAR char *buffer, size_t buflen);

  /**************************************************************************
   * Name: selftest
   *
   * Selftest allows for the testing of the mechanical and electrical
   * portions of the sensors. When the selftest is activated, the
   * electronics cause the sensors to be actuated and produce an output
   * signal. The output signal is used to observe the selftest response.
   * When the selftest response exceeds the min/max values,
   * the part is deemed to have failed selftest.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   arg        - The parameters associated with selftest.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*selftest)(FAR struct sensor_lowerhalf_s *lower,
                       unsigned long arg);

  /**************************************************************************
   * Name: set_calibvalue
   *
   * The calibration value to be written in or the non-volatile memory of the
   * sensor or dedicated registers. At each power-on, so that the values read
   * from the sensor are already corrected. When the device is calibrated,
   * the absolute accuracy will be better than before.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   arg        - The parameters associated with calibration value.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*set_calibvalue)(FAR struct sensor_lowerhalf_s *lower,
                             unsigned long arg);

/****************************************************************************
   * Name: calibrate
   *
   * This operation can trigger the calibration operation, and if the
   * calibration operation is short-lived, the calibration result value can
   * be obtained at the same time, the calibration value to be written in or
   * the non-volatile memory of the sensor or dedicated registers. When the
   * upper-level application calibration is completed, the current
   * calibration value of the sensor needs to be obtained and backed up,
   * so that the last calibration value can be directly obtained after
   * power-on.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   arg        - The parameters associated with calibration value.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*calibrate)(FAR struct sensor_lowerhalf_s *lower,
                        unsigned long arg);

  /**************************************************************************
   * Name: control
   *
   * With this method, the user can set some special config for the sensor,
   * such as changing the custom mode, setting the custom resolution, reset,
   * etc, which are all parsed and implemented by lower half driver.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   cmd        - The special cmd for sensor.
   *   arg        - The parameters associated with cmd.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *   -ENOTTY    - The cmd don't support.
   *
   **************************************************************************/

  CODE int (*control)(FAR struct sensor_lowerhalf_s *lower,
                      int cmd, unsigned long arg);
};

/* This structure is the generic form of state structure used by lower half
 * Sensor driver.
 */

struct sensor_lowerhalf_s
{
  /* The type of sensor device */

  int type;

  /* The number of events that the circular buffer can hold.
   * This sensor circular buffer is used to slove issue that application
   * can't read sensor event in time. If this number of events is too large,
   * the latency of sensor event will be too larage. If the number of events
   * is too small, the event will be overwrite before application read them.
   * So, it's recommended to set according to sensor odr. If odr is low, you
   * can set to one. If odr is high, you can set to two or three.
   *
   * If device support batch mode, the number of events that hardware fifo
   * hold maximum number of samples, must be aligned with size of
   * struct sensor_event_xxx.
   */

  uint32_t buffer_number;

  /* The uncalibrated use to describe whether the sensor event is
   * uncalibrated. True is uncalibrated data, false is calibrated data,
   * default false.
   */

  bool uncalibrated;

  /* The lower half sensor driver operations */

  FAR const struct sensor_ops_s *ops;

  union
    {
      /**********************************************************************
       * Name: push_event
       *
       * Description:
       *   Lower half driver pushes a sensor event by calling this function.
       *   It is provided by upper half driver to lower half driver.
       *
       * Input Parameters:
       *   priv   - Upper half driver handle.
       *   data   - The buffer of event, it can be all type of sensor events.
       *   bytes  - The number of bytes of sensor event.
       *
       * Returned Value:
       *   The bytes of push is returned when success;
       *   A negated errno value is returned on any failure.
       **********************************************************************/

      CODE ssize_t (*push_event)(FAR void *priv, FAR const void *data,
                                 size_t bytes);

      /**********************************************************************
       * Name: notify_event
       *
       * Description:
       *   Lower half driver notifies that sensor data is ready and can be
       *   read by fetch. It is provided by upper half driver to lower half
       *   driver.
       *
       *   This api is used when sensor_ops_s::fetch isn't NULL.
       *
       * Input Parameters:
       *   priv   - Upper half driver handle
       **********************************************************************/

      CODE void (*notify_event)(FAR void *priv);
    };

  /* The private opaque pointer to be passed to upper-layer during callback */

  FAR void *priv;
};

/* This structure describes the register info for the user sensor */

#ifdef CONFIG_USENSOR
struct sensor_reginfo_s
{
  FAR const char *path; /* The path of user sensor */
  uint16_t esize;       /* The element size of user sensor */
  uint32_t nqueue;      /* The number of queue buffered elements */
};
#endif

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
 *   You can register the character device by node name format based on the
 *   type of sensor. Multiple types of the same type are distinguished by
 *   numbers. eg: accel0, accel1. This type of sensor must be less than
 *   SENSOR_TYPE_COUNT. This API corresponds to the sensor_unregister.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persist as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0. If the
 *           devno alerady exists, -EEXIST will be returned.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_register(FAR struct sensor_lowerhalf_s *dev, int devno);

/****************************************************************************
 * Name: sensor_custom_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   You can register the character device type by specific path and esize.
 *   This API corresponds to the sensor_custom_unregister.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persist as long
 *           as the driver persists.
 *   path  - The user specifies path of device. ex: /dev/sensor/xxx.
 *   esize - The element size of intermediate circular buffer.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_custom_register(FAR struct sensor_lowerhalf_s *dev,
                           FAR const char *path, uint8_t esize);

/****************************************************************************
 * Name: sensor_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the sensor_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 ****************************************************************************/

void sensor_unregister(FAR struct sensor_lowerhalf_s *dev, int devno);

/****************************************************************************
 * Name: sensor_custom_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the sensor_custom_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   path  - The user specifies path of device, ex: /dev/sensor/xxx
 ****************************************************************************/

void sensor_custom_unregister(FAR struct sensor_lowerhalf_s *dev,
                              FAR const char *path);

/****************************************************************************
 * Name: usensor_initialize
 *
 * Description:
 *   This function registers usensor character node "/dev/usensor", so that
 *   application can register user sensor by this node. The node will
 *   manager all user sensor in this character dirver.
 ****************************************************************************/

#ifdef CONFIG_USENSOR
int usensor_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_SENSORS_SENSOR_H */
