/****************************************************************************
 * include/nuttx/uorb.h
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

#ifndef __INCLUDE_NUTTX_UORB_H
#define __INCLUDE_NUTTX_UORB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* sensor type definition */

/* Custom Sensor
 * Some special sensor whose event size is not fixed or dynamically change,
 * are called sensor of custom type. You should treat its events as byte
 * streams and use sensor_custom_register to register character device
 * with specific path, ex: "/dev/uorb/custom_dummy".
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

/* Gas sensor
 * This sensor measures the gas resistance, indicating the presence
 * of volatile organic compounds in the air.
 */

#define SENSOR_TYPE_GAS                             33

/* Force
 * A sensor of this type measures the force on it, and additionally
 * compares the force with one or more specified thresholds. The sensor
 * can output the force value directly. Moreover, it's usually applied
 * as a press key. In that case, when it detects a force greater than
 * some given threshold, a corresponding event is reported.
 */

#define SENSOR_TYPE_FORCE                           34

/* The total number of sensor */

#define SENSOR_TYPE_COUNT                           35

/* The additional sensor open flags */

#define SENSOR_REMOTE                               (1u << 31)
#define SENSOR_PERSIST                              (1u << 30)

/* GPS satellite info slots */

#define SENSOR_GPS_SAT_INFO_MAX                     4

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures prefixed with sensor_event are sensor data, and member
 * that are not used must be written as NAN or INT_MIN/INT_MAX, than
 * reported.
 */

struct sensor_accel         /* Type: Accerometer */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in m/s^2 */
  float y;                  /* Axis Y in m/s^2 */
  float z;                  /* Axis Z in m/s^2 */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_gyro          /* Type: Gyroscope */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in rad/s */
  float y;                  /* Axis Y in rad/s */
  float z;                  /* Axis Z in rad/s */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_mag           /* Type: Magnetic Field */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in Gauss or micro Tesla (uT) */
  float y;                  /* Axis Y in Gauss or micro Tesla (uT) */
  float z;                  /* Axis Z in Gauss or micro Tesla (uT) */
  float temperature;        /* Temperature in degrees celsius */
  int32_t status;           /* Status of calibration */
};

struct sensor_baro          /* Type: Barometer */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pressure;           /* pressure measurement in millibar or hpa */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_prox          /* Type: proximity */
{
  uint64_t timestamp;       /* Units is microseconds */
  float proximity;          /* distance to the nearest object in centimeters */
};

struct sensor_light         /* Type: Light */
{
  uint64_t timestamp;       /* Units is microseconds */
  float light;              /* in SI lux units */
  float ir;                 /* in SI lux units */
};

struct sensor_humi          /* Type: Relative Humidity */
{
  uint64_t timestamp;       /* Units is microseconds */
  float humidity;           /* in percent  */
};

struct sensor_temp          /* Type: Ambient Temperature */
{
  uint64_t timestamp;       /* Units is microseconds */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_rgb           /* Type: RGB */
{
  uint64_t timestamp;       /* Units is microseconds */
  float r;                  /* Units is percent */
  float g;                  /* Units is percent */
  float b;                  /* Units is percent */
};

struct sensor_hall          /* Type: HALL */
{
  uint64_t timestamp;       /* Units is microseconds */
  int32_t hall;             /* Hall state */
};

struct sensor_ir            /* Type: Infrared Ray */
{
  uint64_t timestamp;       /* Units is microseconds */
  float ir;                 /* in SI units lux */
};

struct sensor_gps           /* Type: Gps */
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
  float pdop;               /* Position dilution of precision */
  float vdop;               /* Vertical dilution of precision */

  float ground_speed;       /* GPS ground speed, Unit is m/s */

  /* Course over ground (NOT heading, but direction of movement),
   * Unit is Si degrees
   */

  float course;

  uint32_t satellites_used; /* Number of satellites used */
};

struct sensor_uv            /* Type: Ultraviolet Light */
{
  uint64_t timestamp;       /* Units is microseconds */
  float uvi;                /* the value range is 0 - 15 */
};

struct sensor_noise         /* Type: Noise Loudness */
{
  uint64_t timestamp;       /* Units is microseconds */
  float db;                 /* in SI units db */
};

struct sensor_pm25          /* Type: PM25 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm25;               /* in SI units ug/m^3 */
};

struct sensor_pm10          /* Type: PM10 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm10;               /* in SI units ug/m^3 */
};

struct sensor_pm1p0         /* Type: PM1P0 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm1p0;              /* in SI units ug/m^3 */
};

struct sensor_co2           /* Type: CO2 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float co2;                /* in SI units ppm */
};

struct sensor_hcho          /* Type: HCHO */
{
  uint64_t timestamp;       /* Units is microseconds */
  float hcho;               /* in SI units ppm */
};

struct sensor_tvoc          /* Type: TVOC */
{
  uint64_t timestamp;       /* Units is microseconds */
  float tvoc;               /* in SI units ppm */
};

struct sensor_ph            /* Type: PH */
{
  uint64_t timestamp;       /* Units is microseconds */
  float ph;                 /* PH = 7.0 neutral, PH < 7.0 acidic, PH > 7.0 alkaline */
};

struct sensor_dust          /* Type: DUST */
{
  uint64_t timestamp;       /* Units is microseconds */
  float dust;               /* is SI units ug/m^3 */
};

struct sensor_hrate         /* Type: Heart Rate */
{
  uint64_t timestamp;       /* Units is microseconds */
  float bpm;                /* is SI units BPM */
};

struct sensor_hbeat         /* Type: Heart Beat */
{
  uint64_t timestamp;       /* Units is microseconds */
  float beat;               /* Units is times/minutes */
};

struct sensor_ecg           /* Type: ECG */
{
  uint64_t timestamp;       /* Unit is microseconds */
  float ecg;                /* Unit is μV */
  uint32_t status;          /* Status info */
};

struct sensor_ppgd          /* Type: PPGD */
{
  uint64_t timestamp;       /* Unit is microseconds */
  uint32_t ppg[2];          /* PPG from 2 channels. Units are ADC counts. */
  uint32_t current;         /* LED current. Unit is uA. */
  uint16_t gain[2];         /* ADC gains of channels. Units are V/V or V/A. */
};

struct sensor_ppgq          /* Type: PPDQ */
{
  uint64_t timestamp;       /* Unit is microseconds */
  uint32_t ppg[4];          /* PPG from 4 channels. Units are ADC counts. */
  uint32_t current;         /* LED current. Unit is uA. */
  uint16_t gain[4];         /* ADC gains of channels. Units are V/V or V/A. */
};

struct sensor_impd          /* Type: Impedance */
{
  uint64_t timestamp;       /* Unit is microseconds */
  float real;               /* Real part, unit is Ohm(Ω) */
  float imag;               /* Imaginary part, unit is Ohm(Ω) */
};

struct sensor_ots           /* Type: OTS */
{
  uint64_t timestamp;       /* Unit is microseconds */
  int32_t x;                /* Axis X in counts */
  int32_t y;                /* Axis Y in counts */
};

struct sensor_gps_satellite
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
  info[SENSOR_GPS_SAT_INFO_MAX];
};

struct sensor_wake_gesture  /* Type: Wake gesture */
{
  uint64_t timestamp;                /* Units is microseconds */

  /* wake gesture event, 0: sleep, 1: wake,
   * others: Uncalibrated status value.
   */

  uint32_t event;
};

struct sensor_cap           /* Type: Capacitance */
{
  uint64_t timestamp;       /* Unit is microseconds */
  int32_t status;           /* Detection status */
  int32_t rawdata[4];       /* in SI units pF */
};

struct sensor_gas           /* Type: Gas */
{
  uint64_t timestamp;       /* Units is microseconds */
  float gas_resistance;     /* Gas resistance in kOhm */
};

struct sensor_force         /* Type: Force */
{
  uint64_t timestamp;       /* Unit is microseconds */
  float force;              /* Force value, units is N */
  int32_t event;            /* Force event */
};

/* This structure describes the state for the sensor device */

struct sensor_state_s
{
  unsigned long esize;         /* The element size of circular buffer */
  unsigned long nbuffer;       /* The number of events that the circular buffer can hold */
  unsigned long min_latency;   /* The minimum batch latency for sensor, in us */
  unsigned long min_interval;  /* The minimum subscription interval for sensor, in us */
  unsigned long nsubscribers;  /* The number of subcribers */
  unsigned long nadvertisers;  /* The number of advertisers */
  unsigned long generation;    /* The recent generation of circular buffer */
  FAR void     *priv;          /* The pointer to private data of userspace user */
};

/* This structure describes the state for the sensor user */

struct sensor_ustate_s
{
  unsigned long esize;         /* The element size of circular buffer */
  unsigned long latency;       /* The batch latency for user, in us */
  unsigned long interval;      /* The subscription interval for user, in us */
  unsigned long generation;    /* The recent generation of circular buffer */
};

/* This structure describes the register info for the user sensor */

#ifdef CONFIG_USENSOR
struct sensor_reginfo_s
{
  FAR const char *path;        /* The path of user sensor */
  unsigned long   esize;       /* The element size of user sensor */
  unsigned long   nbuffer;     /* The number of queue buffered elements */

  /* The flag is used to indicate that the validity of sensor data
   * is persistent.
   */

  bool            persist;
};
#endif

/* This structure describes the context custom ioctl for device */

struct sensor_ioctl_s
{
  size_t len;                  /* The length of argument of ioctl */
  char data[1];                /* The argument buf of ioctl */
};

#endif /* __INCLUDE_NUTTX_SENSORS_SENSOR_H */
