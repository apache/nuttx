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
#include <limits.h>

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

/* Orientation
 * An orientation sensor reports the attitude of the device. The measurements
 * are reported in degrees in the x, y, and z.
 *   x:azimuth, the angle between the magnetic north direction and the
 *     Y axis, around the Z axis (0<=azimuth<360). 0=North, 90=East,
 *     180=South, 270=West.
 *   y:pitch, rotation around X axis (-180<=pitch<=180), with positive values
 *     when the Z axis moves toward the Y axis.
 *   z:roll, rotation around Y axis (-90<=roll<=90), with positive values
 *     when the X axis moves towards the Z axis.
 */

#define SENSOR_TYPE_ORIENTATION                     3

/* Gyroscope
 * All values are in radians/second and measure the rate of rotation around
 * the X, Y and Z axis.
 */

#define SENSOR_TYPE_GYROSCOPE                       4

/* Ambient Light
 * The ambient light sensor value is returned in SI units lux.
 */

#define SENSOR_TYPE_LIGHT                           5

/* Barometer
 * All values are in hectopascal (hPa) and measure the athmospheric pressure.
 * You can calculate altitude by perssure.
 */

#define SENSOR_TYPE_BAROMETER                       6

/* Noise Loudness
 * A sensor of this type returns the loudness of noise in SI units (db)
 */

#define SENSOR_TYPE_NOISE                           7

/* Proximity
 * The values correspond to the distance to the nearest
 * object in centimeters.
 */

#define SENSOR_TYPE_PROXIMITY                       8

/* RGB
 * We use these values of RGB to weighted to obtain the color of LED.
 * These values is in unit percent.
 */

#define SENSOR_TYPE_RGB                             9

/* Linear acceleration
 * A linear acceleration sensor reports the linear acceleration of the device
 * in the sensor frame, not including gravity(output of the accelerometer
 * minus the output of the gravity sensor).
 */

#define SENSOR_TYPE_LINEAR_ACCELERATION             10

/* Rotation
 * A rotation vector sensor reports the orientation of the device relative
 * to the East-North-Up coordinates frame. It's usually obtained by
 * integration of accelerometer, gyroscope, and magnetometer readings.
 * The East-North-Up coordinate system is defined as a direct orthonormal
 * basis where:
 *   X points east and is tangential to the ground.
 *   Y points north and is tangential to the ground.
 *   Z points towards the sky and is perpendicular to the ground.
 */

#define SENSOR_TYPE_ROTATION_VECTOR                 11

/* Relative Humidity
 * A relative humidity sensor measure relative ambient air humidity and
 * return a value in percent.
 */

#define SENSOR_TYPE_RELATIVE_HUMIDITY               12

/* Ambient Temperature
 * The ambient (room) temperature in degree Celsius
 */

#define SENSOR_TYPE_AMBIENT_TEMPERATURE             13

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

/* Significant motion
 * A significant motion detector triggers when detecting a significant
 * motion: a motion that might lead to a change in the user location.
 */

#define SENSOR_TYPE_SIGNIFICANT_MOTION              17

/* Step detector
 * A step detector generates an event each time a step is taken by the user.
 * The timestamp of the event corresponds to when the foot hit the ground,
 * generating a high variation in acceleration. Compared to the step counter,
 * the step detector should have a lower latency (less than two seconds).
 * Both the step detector and the step counter detect when the user is
 * walking, running, and walking up the stairs. They shouldn't trigger when
 * the user is biking, driving, or in other vehicles.
 */

#define SENSOR_TYPE_STEP_DETECTOR                   18

/* Step counter
 * A step counter reports the number of steps taken by the user since the
 * last reboot while activated. A step counter reports the number of steps
 * taken by the user since the last reboot while activated.
 */

#define SENSOR_TYPE_STEP_COUNTER                    19

/* PH
 * The acid-base degree describes the strength of the aqueous
 * solution, expressed by pH. In the thermodynamic standard
 * condition, the aqueous solution with pH=7 is neutral,
 * pH<7 is acidic, and pH>7 is alkaline.
 */

#define SENSOR_TYPE_PH                              20

/* Heart Rate
 * A sensor of this type returns the current heart rate.
 * Current heart rate is in beats per minute (BPM).
 */

#define SENSOR_TYPE_HEART_RATE                      21

/* Tilt detector
 * A tilt detector generates an event each time a tilt event is detected.
 */

#define SENSOR_TYPE_TILT_DETECTOR                   22

/* Wake gesture
 * A sensor enabling waking up the device based on a device specific
 * motion. 0: the device should sleep, 1: the device should wake up.
 * Other values ​​are uncalibrated values ​​reported by the driver to
 * uncalibrated topics.
 */

#define SENSOR_TYPE_WAKE_GESTURE                    23

/* Glance gesture
 * A glance gesture sensor enables briefly turning the screen on to enable
 * the user to glance content on screen based on a specific motion.
 * When this sensor triggers, the device will turn the screen on momentarily
 * to allow the user to glance notifications or other content while the
 * device remains locked in a non-interactive state (dozing), then the screen
 * will turn off again.
 */

#define SENSOR_TYPE_GLANCE_GESTURE                  24

/* Pick up gesture
 * A pick-up gesture sensor triggers when the device is picked up regardless
 * of wherever it was before (desk, pocket, bag).
 */

#define SENSOR_TYPE_PICK_UP_GESTURE                 25

/* Wrist tilt
 * The wrist-off detection sensor is only triggered when the device is off
 * the wrist.
 */

#define SENSOR_TYPE_WRIST_TILT_GESTURE              26

/* Device_orientation
 * A device orientation sensor reports the current orientation of the device.
 */

#define SENSOR_TYPE_DEVICE_ORIENTATION              27

/* Pose 6DOF
 * A Pose 6DOF sensor reports the orientation of the device relative to the
 * East-North-Up coordinates frame. It is obtained by integration of
 * accelerometer and gyroscope readings.
 * The East-North-Up coordinate system is defined as a direct orthonormal
 * basis where:
 *   X points east and is tangential to the ground.
 *   Y points north and is tangential to the ground.
 *   Z points towards the sky and is perpendicular to the ground.
 * The orientation is represented by the rotation necessary to align the
 * East-North-Up coordinates with the device's coordinates. That is, applying
 * the rotation to the world frame (X,Y,Z) would align them with the device
 * coordinates (x,y,z).
 * The rotation can be seen as rotating the device by an angle theta around
 * an axis rot_axis to go from the reference (East-North-Up aligned) device
 * orientation to the current device orientation. The rotation is encoded as
 * the four unitless x, y, z, w components of a unit quaternion.
 */

#define SENSOR_TYPE_POSE_6DOF                       28

/* Gas sensor
 * This sensor measures the gas resistance, indicating the presence
 * of volatile organic compounds in the air.
 */

#define SENSOR_TYPE_GAS                             29

/* Motion detect
 * Motion detection sensor is used to detect the motion status of the device.
 * motion detect event is produced if the device has been in motion
 * for at least 5 seconds with a maximal latency of 5 additional seconds.
 * ie: it may take up anywhere from 5 to 10 seconds afte the device has been
 * at rest to trigger this event. The only allowed value is 1.0.
 */

#define SENSOR_TYPE_MOTION_DETECT                   30

/* Heart Beat
 * A sensor of this type returns an event evetytime
 * a hear beat peek is detected. Peak here ideally corresponds
 * to the positive peak in the QRS complex of and ECG signal.
 */

#define SENSOR_TYPE_HEART_BEAT                      31

/* Force
 * A sensor of this type measures the force on it, and additionally
 * compares the force with one or more specified thresholds. The sensor
 * can output the force value directly. Moreover, it's usually applied
 * as a press key. In that case, when it detects a force greater than
 * some given threshold, a corresponding event is reported.
 */

#define SENSOR_TYPE_FORCE                           32

/* Hall
 * All values are in bool type (0 or 1) and it often is used to as switch.
 * A values of 1 indicates that switch on.
 */

#define SENSOR_TYPE_HALL                            33

/* Offbody detect
 * An offbody detect sensor reports every time the device transitions from
 * off-body to on-body and from on-body to off-body (e.g. a wearable device
 * being removed from the wrist would trigger an event indicating an off-body
 * transition).
 */

#define SENSOR_TYPE_LOW_LATENCY_OFFBODY_DETECT      34

/* Ultraviolet light sensor
 * This sensor can identify the UV index in ambient light help people
 * to effectively protect themselves from sunburns, cancer or eye damage.
 * This value range is 0 - 15.
 */
#define SENSOR_TYPE_ULTRAVIOLET                     35

/* Hinge angle
 * A hinge angle sensor measures the angle, in degrees, between two integral
 * parts of the device. Movement of a hinge measured by this sensor type is
 * expected to alter the ways in which the user can interact with the device,
 * for example, by unfolding or revealing a display.
 */

#define SENSOR_TYPE_HINGE_ANGLE                     36

/* IR (Infrared Ray)
 * This sensor can detect a human approach and outputs a signal from
 * interrupt pins. This sensor value is in lux.
 */

#define SENSOR_TYPE_IR                              37

/* HCHO
 * The HCHO pollution is an important indicator of household air
 * pollution. This value is in units (ppm-part per million).
 */

#define SENSOR_TYPE_HCHO                            38

/* TVOC (total volatile organic compounds)
 * The indoor TVOC is cause indoor air pollution is one of the
 * main reasons why. This value is in units (ppb-part per billion).
 */

#define SENSOR_TYPE_TVOC                            39

/* Dust
 * A sensor of this type returns the content of dust in the air
 * values is in ug/m^3.
 */

#define SENSOR_TYPE_DUST                            40

/* ECG (Electrocardiogram)
 * A sensor of this type returns the ECG voltage in μV. Sensors may amplify
 * the input ECG signal. Here the ECG voltage is the un-amplified ECG
 * voltage.
 */

#define SENSOR_TYPE_ECG                             41

/* PPG Dual (2-channel photoplethysmography)
 * A sensor of this type returns the 2 channels PPG measurements in ADC
 * counts and their corresponding LED current and ADC gains. The PPG
 * measurements come from photodiodes and following current amplifiers and
 * ADCs, where a photodiode switches reflected light intensity to current.
 * The LED current decides the lightness of LED, which is the input of PPG
 * measurements. The ADC gains are multipled on the output and affect SNR.
 */

#define SENSOR_TYPE_PPGD                            42

/* PPG Quad (4-channel photoplethysmography)
 * A sensor of this type returns the 4 channels PPG measurements in ADC
 * counts and their corresponding LED current and ADC gains. The PPG
 * measurements come from photodiodes and following current amplifiers and
 * ADCs, where a photodiode switches reflected light intensity to current.
 * The LED current decides the lightness of LED, which is the input of PPG
 * measurements. The ADC gains are multipled on the output and affect SNR.
 */

#define SENSOR_TYPE_PPGQ                            43

/* Imdepance
 * A sensor of this type returns the impedance measurements. An impedance
 * is a complex number, which consists of a real part(resistance) and an
 * imaginary part(reactance). Both of them are in uint Ohm(Ω).
 */

#define SENSOR_TYPE_IMPEDANCE                       44

/* OTS (Optical tracking sensor)
 * A sensor of this type returns the OTS measurements in counts. It
 * integrates an optical chip and a LASER light source in a single miniature
 * package. It provies wide depth of field range on glossy surface, and
 * design flexibility into a compact device.
 */

#define SENSOR_TYPE_OTS                             45

/* CO2
 * A sensor of this type returns the content of CO2 in the air
 * This value is in units (ppm-part per million).
 */

#define SENSOR_TYPE_CO2                             46

/* CAP (Capacitive proximity sensor)
 * The purpose of the proximity sensing interface is to detect when a
 * conductive object (usually a body part i.e. finger, palm, face, etc.)
 * is in the proximity of the system.
 */

#define SENSOR_TYPE_CAP                             47

/* GNSS
 * A sensor of this type returns GNSS data. Include latitude, longitude,
 * altitude, horizontal position accuracy, vertical position accuracy,
 * horizontal dilution of precision, vertical dilution of precision...
 */

#define SENSOR_TYPE_GNSS                            48

/* Sensor of GNSS satellite
 * A sensor of this type returns the GNSS satellite information.
 */

#define SENSOR_TYPE_GNSS_SATELLITE                  49

/* GNSS Measurement */

#define SENSOR_TYPE_GNSS_MEASUREMENT                50

/* GNSS Clock */

#define SENSOR_TYPE_GNSS_CLOCK                      51

/* GNSS Geofence */

#define SENSOR_TYPE_GNSS_GEOFENCE                   52

/* The total number of sensor */

#define SENSOR_TYPE_COUNT                           53

/* The additional sensor open flags */

#define SENSOR_REMOTE                               (1u << 31)
#define SENSOR_PERSIST                              (1u << 30)

/* GNSS satellite info slots */

#define SENSOR_GNSS_SAT_INFO_MAX                    4

/* Maximum length of sensor device information name and path name. */

#define SENSOR_INFO_NAME_SIZE                       32

/* Sensor event flags */

#define SENSOR_EVENT_FLUSH_COMPLETE                 0x01

/* GNSS Clock Flags, see `flags` of `struct sensor_gnss_clock`
 * Refs: https://android.googlesource.com/platform/hardware/libhardware/+/
 *       refs/heads/android14-release/include/hardware/gps.h#140
 */

#define SENSOR_GNSS_CLOCK_HAS_LEAP_SECOND                     (1 << 0)
#define SENSOR_GNSS_CLOCK_HAS_TIME_UNCERTAINTY                (1 << 1)
#define SENSOR_GNSS_CLOCK_HAS_FULL_BIAS                       (1 << 2)
#define SENSOR_GNSS_CLOCK_HAS_BIAS                            (1 << 3)
#define SENSOR_GNSS_CLOCK_HAS_BIAS_UNCERTAINTY                (1 << 4)
#define SENSOR_GNSS_CLOCK_HAS_DRIFT                           (1 << 5)
#define SENSOR_GNSS_CLOCK_HAS_DRIFT_UNCERTAINTY               (1 << 6)

/* GNSS Measurement Flags */

#define SENSOR_GNSS_MEASUREMENT_HAS_SNR                       (1 << 0)
#define SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY         (1 << 9)
#define SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_CYCLES            (1 << 10)
#define SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_PHASE             (1 << 11)
#define SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_PHASE_UNCERTAINTY (1 << 12)
#define SENSOR_GNSS_MEASUREMENT_HAS_AUTOMATIC_GAIN_CONTROL    (1 << 13)

/* GNSS Measurement States */

#define SENSOR_GNSS_MEASUREMENT_STATE_UNKNOWN                 (0)
#define SENSOR_GNSS_MEASUREMENT_STATE_CODE_LOCK               (1 << 0)
#define SENSOR_GNSS_MEASUREMENT_STATE_BIT_SYNC                (1 << 1)
#define SENSOR_GNSS_MEASUREMENT_STATE_SUBFRAME_SYNC           (1 << 2)
#define SENSOR_GNSS_MEASUREMENT_STATE_TOW_DECODED             (1 << 3)
#define SENSOR_GNSS_MEASUREMENT_STATE_MSEC_AMBIGUOUS          (1 << 4)
#define SENSOR_GNSS_MEASUREMENT_STATE_SYMBOL_SYNC             (1 << 5)
#define SENSOR_GNSS_MEASUREMENT_STATE_GLO_STRING_SYNC         (1 << 6)
#define SENSOR_GNSS_MEASUREMENT_STATE_GLO_TOD_DECODED         (1 << 7)
#define SENSOR_GNSS_MEASUREMENT_STATE_BDS_D2_BIT_SYNC         (1 << 8)
#define SENSOR_GNSS_MEASUREMENT_STATE_BDS_D2_SUBFRAME_SYNC    (1 << 9)
#define SENSOR_GNSS_MEASUREMENT_STATE_GAL_E1BC_CODE_LOCK      (1 << 10)
#define SENSOR_GNSS_MEASUREMENT_STATE_GAL_E1C_2ND_CODE_LOCK   (1 << 11)
#define SENSOR_GNSS_MEASUREMENT_STATE_GAL_E1B_PAGE_SYNC       (1 << 12)
#define SENSOR_GNSS_MEASUREMENT_STATE_SBAS_SYNC               (1 << 13)
#define SENSOR_GNSS_MEASUREMENT_STATE_TOW_KNOWN               (1 << 14)
#define SENSOR_GNSS_MEASUREMENT_STATE_GLO_TOD_KNOWN           (1 << 15)

/* SENSOR_GNSS_GEOFENCE_TRANS_*:
 * struct sensor_gnss_geofence_event -> transition
 * Ref: android-14-release/hardware/libhardware/include/hardware/gnss-base.h
 */

#define SENSOR_GNSS_GEOFENCE_TRANS_ENTERED                    (1 << 0)
#define SENSOR_GNSS_GEOFENCE_TRANS_EXITED                     (1 << 1)
#define SENSOR_GNSS_GEOFENCE_TRANS_UNCERTAIN                  (1 << 2)

/* SENSOR_GNSS_GEOFENCE_STATUS_*:
 * struct sensor_gnss_geofence_event -> status
 * Ref: android-14-release/hardware/libhardware/include/hardware/gnss-base.h
 */

#define SENSOR_GNSS_GEOFENCE_STATUS_UNAVAILABLE               (1 << 0)
#define SENSOR_GNSS_GEOFENCE_STATUS_AVAILABLE                 (1 << 1)
#define SENSOR_GNSS_GEOFENCE_STATUS_OPERATION_SUCCESS         (0)
#define SENSOR_GNSS_GEOFENCE_STATUS_ERROR_TOO_MANY_GEOFENCES  (-100)
#define SENSOR_GNSS_GEOFENCE_STATUS_ERROR_ID_EXISTS           (-101)
#define SENSOR_GNSS_GEOFENCE_STATUS_ERROR_ID_UNKNOWN          (-102)
#define SENSOR_GNSS_GEOFENCE_STATUS_ERROR_INVALID_TRANSITION  (-103)
#define SENSOR_GNSS_GEOFENCE_STATUS_ERROR_GENERIC             (-149)

/* SENSOR_GNSS_GEOFENCE_TYPE_*:
 * `type` of `struct sensor_gnss_geofence_param` and
 *           `struct sensor_gnss_geofence_event`
 */
#define SENSOR_GNSS_GEOFENCE_TYPE_TRANSITION                  (1 << 0)
#define SENSOR_GNSS_GEOFENCE_TYPE_STATUS                      (1 << 1)
#define SENSOR_GNSS_GEOFENCE_TYPE_ADD                         (1 << 2)
#define SENSOR_GNSS_GEOFENCE_TYPE_REMOVE                      (1 << 3)
#define SENSOR_GNSS_GEOFENCE_TYPE_PAUSE                       (1 << 4)
#define SENSOR_GNSS_GEOFENCE_TYPE_RESUME                      (1 << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures prefixed with sensor_event are sensor data, and member
 * that are not used must be written as NAN or INT_MIN/INT_MAX, than
 * reported.
 */

struct sensor_event  /* Type: Sensor Common Event */
{
  uint64_t timestamp;       /* Units is microseconds */
  uint32_t event;           /* Common events */
};

struct sensor_accel         /* Type: Accerometer */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in m/s^2 */
  float y;                  /* Axis Y in m/s^2 */
  float z;                  /* Axis Z in m/s^2 */
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

struct sensor_orientation   /* Type: Orientation */
{
  uint64_t timestamp;       /* Units is microseconds */
  float    x;               /* azimuth */
  float    y;               /* pitch */
  float    z;               /* roll */
};

struct sensor_gyro          /* Type: Gyroscope */
{
  uint64_t timestamp;       /* Units is microseconds */
  float x;                  /* Axis X in rad/s */
  float y;                  /* Axis Y in rad/s */
  float z;                  /* Axis Z in rad/s */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_light         /* Type: Light */
{
  uint64_t timestamp;       /* Units is microseconds */
  float light;              /* in SI lux units */
  float ir;                 /* in SI lux units */
};

struct sensor_baro          /* Type: Barometer */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pressure;           /* pressure measurement in millibar or hpa */
  float temperature;        /* Temperature in degrees celsius */
};

struct sensor_noise         /* Type: Noise Loudness */
{
  uint64_t timestamp;       /* Units is microseconds */
  float db;                 /* in SI units db */
};

struct sensor_prox          /* Type: proximity */
{
  uint64_t timestamp;       /* Units is microseconds */
  float proximity;          /* distance to the nearest object in centimeters */
};

struct sensor_rgb           /* Type: RGB */
{
  uint64_t timestamp;       /* Units is microseconds */
  float r;                  /* Units is percent */
  float g;                  /* Units is percent */
  float b;                  /* Units is percent */
};

struct sensor_rotation      /* Type: Rotation */
{
  uint64_t timestamp;       /* Units is microseconds */
  float    x;               /* x*sin(θ/2) */
  float    y;               /* y*sin(θ/2) */
  float    z;               /* z*sin(θ/2) */
  float    w;               /* cos(θ/2) */
  float    status;          /* estimated heading Accuracy (in radians) (-1 if unavailable) */
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

struct sensor_pm25          /* Type: PM25 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm25;               /* in SI units ug/m^3 */
};

struct sensor_pm1p0         /* Type: PM1P0 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm1p0;              /* in SI units ug/m^3 */
};

struct sensor_pm10          /* Type: PM10 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float pm10;               /* in SI units ug/m^3 */
};

struct sensor_step_counter  /* Type: Step Coun */
{
  uint64_t timestamp;       /* Units is microseconds */
  uint32_t steps;           /* Step counting */
  uint32_t cadence;         /* Stride frequency */
};

struct sensor_ph            /* Type: PH */
{
  uint64_t timestamp;       /* Units is microseconds */
  float ph;                 /* PH = 7.0 neutral, PH < 7.0 acidic, PH > 7.0 alkaline */
};

struct sensor_hrate         /* Type: Heart Rate */
{
  uint64_t timestamp;       /* Units is microseconds */
  float bpm;                /* is SI units BPM */
};

struct sensor_pose_6dof     /* Type: Pose 6dof */
{
  uint64_t timestamp;       /* Units is microseconds */
  float    x;               /* x*sin(theta/2) */
  float    y;               /* y*sin(theta/2) */
  float    z;               /* z*sin(theta/2) */
  float    w;               /* cos(theta/2) */
  float    tx;              /* Translation along x axis from an arbitrary origin. */
  float    ty;              /* Translation along y axis from an arbitrary origin. */
  float    tz;              /* Translation along z axis from an arbitrary origin. */
  float    dx;              /* Delta quaternion rotation x*sin(theta/2) */
  float    dy;              /* Delta quaternion rotation y*sin(theta/2) */
  float    dz;              /* Delta quaternion rotation z*sin(theta/2) */
  float    dw;              /* Delta quaternion rotation cos(theta/2) */
  float    dtx;             /* Delta translation along x axis. */
  float    dty;             /* Delta translation along y axis. */
  float    dtz;             /* Delta translation along z axis. */
  uint64_t number;          /* Sequence number; ascending sequentially from 0 */
};

struct sensor_gas           /* Type: Gas */
{
  uint64_t timestamp;       /* Units is microseconds */
  float gas_resistance;     /* Gas resistance in kOhm */
};

struct sensor_hbeat         /* Type: Heart Beat */
{
  uint64_t timestamp;       /* Units is microseconds */
  float beat;               /* Units is times/minutes */
};

struct sensor_force         /* Type: Force */
{
  uint64_t timestamp;       /* Unit is microseconds */
  float force;              /* Force value, units is N */
  int32_t event;            /* Force event */
};

struct sensor_hall          /* Type: HALL */
{
  uint64_t timestamp;       /* Units is microseconds */
  int32_t hall;             /* Hall state */
};

struct sensor_uv            /* Type: Ultraviolet Light */
{
  uint64_t timestamp;       /* Units is microseconds */
  float uvi;                /* the value range is 0 - 15 */
};

struct sensor_angle         /* Type: Angle */
{
  uint64_t timestamp;       /* Units is microseconds */
  float    angle;           /* Angle. Unit is degree */
};

struct sensor_ir            /* Type: Infrared Ray */
{
  uint64_t timestamp;       /* Units is microseconds */
  float ir;                 /* in SI units lux */
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

struct sensor_dust          /* Type: DUST */
{
  uint64_t timestamp;       /* Units is microseconds */
  float dust;               /* is SI units ug/m^3 */
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

struct sensor_co2           /* Type: CO2 */
{
  uint64_t timestamp;       /* Units is microseconds */
  float co2;                /* in SI units ppm */
};

struct sensor_cap           /* Type: Capacitance */
{
  uint64_t timestamp;       /* Unit is microseconds */
  int32_t status;           /* Detection status */
  int32_t rawdata[4];       /* in SI units pF */
};

struct sensor_gnss          /* Type: GNSS */
{
  uint64_t timestamp;       /* Time since system start, Units is microseconds */

  /* This is the timestamp which comes from the GNSS module. It might be
   * unavailable right after cold start, indicated by a value of 0,
   * Units is microseconds
   */

  uint64_t time_utc;

  float latitude;           /* Unit is degrees */
  float longitude;          /* Unit is degrees */
  float altitude;           /* Altitude above MSL(mean seal level), Unit is SI m */
  float altitude_ellipsoid; /* Altitude bove Ellipsoid, Unit is SI m */

  float eph;                /* GNSS horizontal position accuracy (metres) */
  float epv;                /* GNSS vertical position accuracy (metres) */

  float hdop;               /* Horizontal dilution of precision */
  float pdop;               /* Position dilution of precision */
  float vdop;               /* Vertical dilution of precision */

  float ground_speed;       /* GNSS ground speed, Unit is m/s */

  /* Course over ground (NOT heading, but direction of movement),
   * Unit is Si degrees
   */

  float course;

  uint32_t satellites_used; /* Number of satellites used */
};

/* Ref: android14-release/hardware/libhardware/include_all/hardware/\
 *      gnss-base.h
 */

enum sensor_gnss_constellation
{
  SENSOR_GNSS_CONSTELLATION_UNKNOWN = 0,
  SENSOR_GNSS_CONSTELLATION_GPS     = 1,
  SENSOR_GNSS_CONSTELLATION_SBAS    = 2,
  SENSOR_GNSS_CONSTELLATION_GLONASS = 3,
  SENSOR_GNSS_CONSTELLATION_QZSS    = 4,
  SENSOR_GNSS_CONSTELLATION_BEIDOU  = 5,
  SENSOR_GNSS_CONSTELLATION_GALILEO = 6,
};

struct sensor_gnss_satellite
{
  uint64_t timestamp;       /* Time since system start, Units is microseconds */
  uint32_t count;           /* Total number of messages of satellites visible */
  uint32_t satellites;      /* Total number of satellites in view */

  /* Constellation of the given svid, see SENSOR_GNSS_CONSTELLATION_*. */

  uint32_t constellation;

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
  info[SENSOR_GNSS_SAT_INFO_MAX];
};

struct sensor_gnss_measurement
{
  /* Indicating what fields are valid.
   * See SENSOR_GNSS_MEASUREMENT_HAS_*.
   */

  uint32_t flags;

  /* Space vehicle ID. */

  int32_t  svid;

  /* Constellation of the given SV, see GNSS_CONSTELLATION_*. */

  uint32_t constellation;

  /* Offset between clock and time at which the measurement was taken in
   * nanoseconds.
   */

  float    time_offset_ns;

  /* The received GNSS Time-of-Week at the measurement time, in
   * nanoseconds.
   */

  int64_t  received_sv_time_in_ns;
  int64_t  received_sv_time_uncertainty_in_ns;

  /* GNSS measurement state, see SENSOR_GNSS_MEASUREMENT_STATE_*. */

  uint32_t state;

  /* dBHz, Carrier-to-noise density. */

  float    c_n0_dbhz;

  /* Pseudorange rate(m/s) at the timestamp. */

  float    pseudorange_rate_mps;
  float    pseudorange_rate_uncertainty_mps;

  /* Accumulated delta range. */

  uint32_t accumulated_delta_range_state;
  float    accumulated_delta_range_m;
  float    accumulated_delta_range_uncertainty_m;

  /* Carrier related between the satellite and the receiver.
   * flags:
   *   SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_CYCLES
   *   SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY
   *   SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_PHASE
   *   SENSOR_GNSS_MEASUREMENT_HAS_CARRIER_PHASE_UNCERTAINTY
   */

  float    carrier_frequency_hz;
  int64_t  carrier_cycles;
  float    carrier_phase;
  float    carrier_phase_uncertainty;

  uint32_t multipath_indicator;

  /* dBHz, Signal to noise ratio of satellite C/N0.
   * flags: SENSOR_GNSS_MEASUREMENT_HAS_SNR
   */

  uint32_t snr;
};

struct sensor_gnss_clock
{
  /* Indicating what fields are valid.
   * See SENSOR_GNSS_CLOCK_HAS_*.
   */

  uint32_t flags;

  /* Leap second data.
   * flags: SENSOR_GNSS_CLOCK_HAS_LEAP_SECOND
   */

  int32_t  leap_second;

  /* The GNSS receiver internal local hardware clock value.
   * flags:
   *   SENSOR_GNSS_CLOCK_HAS_TIME_UNCERTAINTY
   */

  int64_t  time_ns;
  float    time_uncertainty_ns;

  /* Discontinuities in the HW clock. */

  uint32_t hw_clock_discontinuity_count;

  /* The difference between hardware clock ('time' field) inside
   * GPS receiver and the true GPS time since 0000Z, January 6, 1980, in
   * nanoseconds.
   * flags:
   *   SENSOR_GNSS_CLOCK_HAS_FULL_BIAS
   *   SENSOR_GNSS_CLOCK_HAS_BIAS
   *   SENSOR_GNSS_CLOCK_HAS_BIAS_UNCERTAINTY
   */

  int64_t  full_bias_ns;
  float    bias_ns;             /* Sub-nanosecond bias */
  float    bias_uncertainty_ns;

  /* The clock's drift in nanoseconds (per second).
   * A positive value means that the frequency is higher than
   * the nominal frequency.
   * flags:
   *   SENSOR_GNSS_CLOCK_HAS_DRIFT
   *   SENSOR_GNSS_CLOCK_HAS_DRIFT_UNCERTAINTY
   */

  float    drift_nsps;
  float    drift_uncertainty_nsps;
};

/* GNSS Geofence events */

struct sensor_gnss_geofence_event
{
  /* Type of events
   * Fields below are optional according to this `type`,
   * Available: see SENSOR_GNSS_GEOFENCE_TYPE_VALID_EVENT.
   *
   * Mandatory:
   *   |Fields \ Type |TRANSITION |STATUS |ADD |REMOVE |PAUSE |RESUME |
   *   |--------------|:---------:|:-----:|:--:|:-----:|:----:|:-----:|
   *   |geofence_id   |     v     |       | v  |    v  |   v  |   v   |
   *   |transition    |     v     |       |    |       |      |       |
   *   |location      |     v     |   v   |    |       |      |       |
   *   |timestamp     |     v     |       |    |       |      |       |
   *   |status        |           |   v   | v  |    v  |   v  |   v   |
   */

  int32_t            type;

  int32_t            geofence_id; /* Id of the geofence. */
  struct sensor_gnss location;    /* Location. */

  /* Milliseconds when the transition was detected since January 1, 1970 */

  int64_t            timestamp;
  int32_t            status;      /* Status of Geofence operation/event. */
  int32_t            transition;  /* See SENSOR_GNSS_GEOFENCE_TRANS_*. */
};

/* GNSS Geofence parameters */

struct sensor_gnss_geofence_param
{
  /* Type of events
   * Available: see SENSOR_GNSS_GEOFENCE_TYPE_VALID_PARAM.
   *
   * Mandatory:
   *   |Fields \ Type |ADD |REMOVE |PAUSE |RESUME |
   *   |--------------|:--:|:-----:|:----:|:-----:|
   *   |geofence_id   | v  |   v   |  v   |   v   |
   *   |transition    | v  |       |      |   v   |
   *   |latitude      | v  |       |      |       |
   *   |longitude     | v  |       |      |       |
   *   |radius_meters | v  |       |      |       |
   */

  int32_t            type;

  int32_t            geofence_id;
  float              latitude;
  float              longitude;
  float              radius_meters;

  /* Which transitions to monitor.
   * Available: see SENSOR_GNSS_GEOFENCE_TRANS_*.
   */

  int32_t            transition;
};

/* This structure describes the state for the sensor device */

struct sensor_state_s
{
  uint32_t esize;              /* The element size of circular buffer */
  uint32_t nbuffer;            /* The number of events that the circular buffer can hold */
  uint32_t min_latency;        /* The minimum batch latency for sensor, in us */
  uint32_t min_interval;       /* The minimum subscription interval for sensor, in us */
  uint32_t nsubscribers;       /* The number of subcribers */
  uint32_t nadvertisers;       /* The number of advertisers */
  uint32_t generation;         /* The recent generation of circular buffer */
  uint64_t priv;               /* The pointer to private data of userspace user */
};

/* This structure describes the state for the sensor user */

struct sensor_ustate_s
{
  uint32_t esize;              /* The element size of circular buffer */
  uint32_t latency;            /* The batch latency for user, in us */
  uint32_t interval;           /* The subscription interval for user, in us */
  uint64_t generation;         /* The recent generation of circular buffer */
};

/* This structure describes the register info for the user sensor */

#ifdef CONFIG_USENSOR
struct sensor_reginfo_s
{
  char     path[NAME_MAX];     /* The path of user sensor */
  uint32_t esize;              /* The element size of user sensor */
  uint32_t nbuffer;            /* The number of queue buffered elements */

  /* The flag is used to indicate that the validity of sensor data
   * is persistent.
   */

  int persist;
};
#endif

/* This structure describes the context custom ioctl for device */

struct sensor_ioctl_s
{
  uint32_t len;                /* The length of argument of ioctl */
  char data[1];                /* The argument buf of ioctl */
};

/* This structure describes the information of the sensor device and
 * requires the manufacturer to implement the device info function.
 */

struct sensor_device_info_s
{
  /* Version of the hardware part + driver. */

  uint32_t      version;

  /* Rough estimate of this sensor's power consumption in mA. */

  float         power;

  /* Maximum range of this sensor's value in SI units. */

  float         max_range;

  /* Smallest difference between two values reported by this sensor. */

  float         resolution;

  /* This value depends on the reporting mode:
   *
   *   continuous: minimum sample period allowed in microseconds
   *   on-change : 0
   *   one-shot  :-1
   *   special   : 0, unless otherwise noted
   */

  int32_t       min_delay;

  /* This value is defined only for continuous mode and on-change sensors.
   * it is the delay between two sensor events corresponding to the lowest
   * frequency that this sensor supports. when lower frequencies are
   * requested through batch()/set_interval() the events will be generated
   * at this frequency instead. it can be used by the framework or
   * applications to estimate when the batch FIFO may be full.
   */

  int32_t       max_delay;

  /* Number of events reserved for this sensor in the batch mode FIFO.
   * if there is a dedicated FIFO for this sensor, then this is the
   * size of this FIFO. If the FIFO is shared with other sensors,
   * this is the size reserved for that sensor and it can be zero.
   */

  uint32_t      fifo_reserved_event_count;

  /* Maximum number of events of this sensor that could be batched.
   * this is especially relevant when the FIFO is shared between
   * several sensors; this value is then set to the size of that FIFO.
   */

  uint32_t      fifo_max_event_count;

  /* Name of this sensor. */

  char          name[SENSOR_INFO_NAME_SIZE];

  /* Vendor of the hardware part. */

  char          vendor[SENSOR_INFO_NAME_SIZE];
};

#endif /* __INCLUDE_NUTTX_SENSORS_SENSOR_H */
