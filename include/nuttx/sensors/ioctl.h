/****************************************************************************
 * include/nuttx/sensors/ioctl.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_IOCTL_H
#define __INCLUDE_NUTTX_SENSORS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL commands unique to the BH1750FVI */

#define SNIOC_CHRM                 _SNIOC(0x0001) /* Contin. H-Res Mode Arg: None   */
#define SNIOC_CHRM2                _SNIOC(0x0002) /* Contin. H-Res Mode2 Arg: None  */
#define SNIOC_CLRM                 _SNIOC(0x0003) /* Contin. L-Res Mode Arg: None   */
#define SNIOC_OTHRM                _SNIOC(0x0004) /* One Time H-Res Mode Arg: None  */
#define SNIOC_OTHRM2               _SNIOC(0x0005) /* One Time H-Res Mode2 Arg: None */
#define SNIOC_OTLRM                _SNIOC(0x0006) /* One Time L-Res Mode Arg: None  */
#define SNIOC_CHMEATIME            _SNIOC(0x0007) /* Change Meas. Time Arg: uint8_t */

/* IOCTL commands unique to the KXTJ9 */

#define SNIOC_ENABLE               _SNIOC(0x0008) /* Arg: None */
#define SNIOC_DISABLE              _SNIOC(0x0009) /* Arg: None */
#define SNIOC_CONFIGURE            _SNIOC(0x000a) /* Arg: enum kxtj9_odr_e value */

/* IOCTL commands common to the LM75, LM92 (and compatible parts) */

#define SNIOC_READCONF             _SNIOC(0x000b) /* Arg: uint8_t* pointer */
#define SNIOC_WRITECONF            _SNIOC(0x000c) /* Arg: uint8_t value */
#define SNIOC_SHUTDOWN             _SNIOC(0x000d) /* Arg: None */
#define SNIOC_POWERUP              _SNIOC(0x000e) /* Arg: None */
#define SNIOC_FAHRENHEIT           _SNIOC(0x000f) /* Arg: None */
#define SNIOC_CENTIGRADE           _SNIOC(0x0010) /* Arg: None */
#define SNIOC_READTHYS             _SNIOC(0x0011) /* Arg: b16_t* pointer */
#define SNIOC_WRITETHYS            _SNIOC(0x0012) /* Arg: b16_t value */

/* IOCTL commands unique to the LM75 */

#define SNIOC_READTOS              _SNIOC(0x0013) /* Arg: b16_t* pointer */
#define SNIOC_WRITETOS             _SNIOC(0x0014) /* Arg: b16_t value */

/* IOCTL commands unique to the LM92 */

#define SNIOC_READTCRIT            _SNIOC(0x0015) /* Arg: b16_t* pointer */
#define SNIOC_WRITETCRIT           _SNIOC(0x0016) /* Arg: b16_t value */
#define SNIOC_READTLOW             _SNIOC(0x0017) /* Arg: b16_t* pointer */
#define SNIOC_WRITETLOW            _SNIOC(0x0018) /* Arg: b16_t value */
#define SNIOC_READTHIGH            _SNIOC(0x0019) /* Arg: b16_t* pointer */
#define SNIOC_WRITETHIGH           _SNIOC(0x001a) /* Arg: b16_t value */
#define SNIOC_READID               _SNIOC(0x001b) /* Arg: uint16_t* pointer */

/* IOCTL commands unique to the LSM9DS1 */

#define SNIOC_START                _SNIOC(0x001c) /* Arg: None */
#define SNIOC_STOP                 _SNIOC(0x001d) /* Arg: None */
#define SNIOC_SETSAMPLERATE        _SNIOC(0x001e) /* Arg: uint32_t value */
#define SNIOC_SETFULLSCALE         _SNIOC(0x001f) /* Arg: uint32_t value */

/* IOCTL commands unique to the MB7040 */

#define SNIOC_MEASURE              _SNIOC(0x0020) /* Arg: None */
#define SNIOC_RANGE                _SNIOC(0x0021) /* Arg: int32_t* pointer */
#define SNIOC_CHANGEADDR           _SNIOC(0x0022) /* Arg: uint8_t value */

/* IOCTL commands unique to the MCP9844 */

#define SNIOC_READTEMP             _SNIOC(0x0023)  /* Arg: mcp9844_temp_arg_s* pointer */
#define SNIOC_SETRESOLUTION        _SNIOC(0x0024)  /* Arg: uint16_t value */

/* IOCTL commands unique to the MS58XX */

#define SNIOC_TEMPERATURE          _SNIOC(0x0026) /* Arg: int32_t* pointer */
#define SNIOC_PRESSURE             _SNIOC(0x0027) /* Arg: int32_t* pointer */
#define SNIOC_RESET                _SNIOC(0x0028) /* Arg: None */
#define SNIOC_OVERSAMPLING         _SNIOC(0x0029) /* Arg: uint16_t value */

/* IOCTL commands to the HTS221 & LPS25H */

#define SNIOC_CFGR                 _SNIOC(0x002a)
#define SNIOC_GET_DEV_ID           _SNIOC(0x002b)

/* IOCTL commands unique to the HTS221 */

#define SNIOC_START_CONVERSION     _SNIOC(0x002c)
#define SNIOC_CHECK_STATUS_REG     _SNIOC(0x002d)
#define SNIOC_READ_RAW_DATA        _SNIOC(0x002e)
#define SNIOC_READ_CONVERT_DATA    _SNIOC(0x002f)
#define SNIOC_DUMP_REGS            _SNIOC(0x0030)

/* IOCTL commands unique to the LPS25H */

#define SNIOC_TEMPERATURE_OUT      _SNIOC(0x0031)
#define SNIOC_PRESSURE_OUT         _SNIOC(0x0032)
#define SNIOC_SENSOR_OFF           _SNIOC(0x0033)

/* IOCTL commands unique to the LIS2DH */

#define SNIOC_WRITESETUP           _SNIOC(0x0034) /* Arg: uint8_t value */
#define SNIOC_WRITE_INT1THRESHOLD  _SNIOC(0x0035) /* Arg: uint8_t value */
#define SNIOC_WRITE_INT2THRESHOLD  _SNIOC(0x0036) /* Arg: uint8_t value */
#define SNIOC_RESET_HPFILTER       _SNIOC(0x0037) /* Arg: uint8_t value */
#define SNIOC_START_SELFTEST       _SNIOC(0x0038) /* Arg: uint8_t value */
#define SNIOC_WHO_AM_I             _SNIOC(0x0039)
#define SNIOC_READ_TEMP            _SNIOC(0x003a) /* Arg: int16_t value */

/* IOCTL commands unique to the MAX44009 */

#define SNIOC_INIT                 _SNIOC(0x003b)
#define SNIOC_THRESHOLD            _SNIOC(0x003c)

/* IOCTL commands unique to LIS3DH */

#define SNIOC_SET_POWER_MODE       _SNIOC(0x003d) /* Arg: LIS3DH_POWER_xxx */
#define SNIOC_SET_DATA_RATE        _SNIOC(0x003e) /* Arg: LIS3DH_ODR_xxx */
#define SNIOC_SET_DATA_FORMAT      _SNIOC(0x003f) /* Arg: LIS3DH_FORMAT_xxx */

/* IOCTL commands unique to T67XX */

#define SNIOC_SPCALIB              _SNIOC(0x0040) /* Arg: uint8_t value */
#define SNIOC_ABCLOGIC             _SNIOC(0x0041) /* Arg: uint8_t value */

/* IOCTL commands unique to the LSM6DSL */

#define SNIOC_LSM6DSLSENSORREAD    _SNIOC(0x0046) /* Arg: file *filep, FAR char *buffer,size_t buflen */

/* SNIOC_START_SELFTEST */                        /* Arg: file *filep, FAR char *buffer,size_t mode */

/* IOCTL commands unique to the LSM303AGR */

#define SNIOC_LSM303AGRSENSORREAD  _SNIOC(0x0051) /* Arg: file *filep, FAR char *buffer,size_t buflen */

/* SNIOC_START_SELFTEST */                        /* Arg: file *filep, FAR char *buffer,size_t mode */

/* IOCTL commands unique to the MLX90614 */

#define SNIOC_CHANGE_SMBUSADDR     _SNIOC(0x0053) /* Arg: uint8_t value */

/* IOCTL commands unique to the SCD30 */

/* SNIOC_RESET */                                 /* Arg: None */

/* SNIOC_START */                                 /* Arg: None */

/* SNIOC_STOP */                                  /* Arg: None */

/* SNIOC_READ_CONVERT_DATA */                     /* Arg: struct scd30_conv_data_s* */

/* SNIOC_SET_INTERVAL */                          /* Arg: uint16_t value (seconds) */
#define SNIOC_SET_TEMP_OFFSET      _SNIOC(0x0055) /* Arg: uint16_t value (0.01 Kelvin) */
#define SNIOC_SET_PRESSURE_COMP    _SNIOC(0x0056) /* Arg: uint16_t value (mbar) */
#define SNIOC_SET_ALTITUDE_COMP    _SNIOC(0x0057) /* Arg: uint16_t value (meters) */
#define SNIOC_SET_FRC              _SNIOC(0x0058) /* Arg: uint16_t value (CO₂ ppm) */
#define SNIOC_ENABLE_ASC           _SNIOC(0x0059) /* Arg: bool value */

/* IOCTL commands unique to the SGP30 */

/* SNIOC_RESET */                                 /* Arg: None */

/* SNIOC_START_SELFTEST */                        /* Arg: None */

/* SNIOC_READ_CONVERT_DATA */                     /* Arg: struct sgp30_conv_data_s* */

/* SNIOC_READ_RAW_DATA */                         /* Arg: struct sgp30_raw_data_s* */

#define SNIOC_GET_BASELINE         _SNIOC(0x005a) /* Arg: struct sgp30_baseline_s* */
#define SNIOC_SET_BASELINE         _SNIOC(0x005b) /* Arg: const struct sgp30_baseline_s* */
#define SNIOC_SET_HUMIDITY         _SNIOC(0x005c) /* Arg: uint32_t value (mg/m³) */

/* IOCTL commands unique to the SPS30 */

/* SNIOC_RESET */                                 /* Arg: None */

/* SNIOC_START */                                 /* Arg: None */

/* SNIOC_STOP */                                  /* Arg: None */

/* SNIOC_READ_CONVERT_DATA */                     /* Arg: struct sps30_conv_data_s* */

#define SNIOC_SET_CLEAN_INTERVAL   _SNIOC(0x005d) /* Arg: uint32_t value (seconds) */
#define SNIOC_START_FAN_CLEANING   _SNIOC(0x005e) /* Arg: None */

/* IOCTL commands unique to the ADT7320 */

#define SNIOC_READSTAT             _SNIOC(0x005f) /* Arg: uint8_t* pointer */

/* IOCTL commands unique to the VL53L1X */

#define SNIOC_DISTANCESHORT        _SNIOC(0x0060) /* Arg: None */
#define SNIOC_DISTANCELONG         _SNIOC(0x0061) /* Arg: None */
#define SNIOC_TEMPUPDATE           _SNIOC(0x0063) /* Arg: b16_t value */

/* IOCTL commands unique to the ISL29023 */

#define SNIOC_SET_OPERATIONAL_MODE _SNIOC(0x0064) /* Arg: uint8_t value */
#define SNIOC_SET_RESOLUTION       _SNIOC(0x0065) /* Arg: uint8_t value */
#define SNIOC_SET_RANGE            _SNIOC(0x0066) /* Arg: uint8_t value */

/* IOCTL commands unique to the HYS271 */

/* SNIOC_CHANGEADDR */                            /* Arg: uint8_t value */

#define SNIOC_READADDR             SNIOC_READID   /* Arg: uint8_t* pointer */

/* IOCTL commands unique to the DS18B20 */

/* SNIOC_SETRESOLUTION */                          /* Arg: uint8_t value */

#define SNIOC_READROMCODE          _SNIOC(0x0067)  /* Arg: uint64_t* pointer */
#define SNIOC_SETALARM             _SNIOC(0x0068)  /* Arg: struct ds18b20_alarm_s* */

/* IOCTL commands for accelerators */

#define SNIOC_SIMPLE_CHECK         _SNIOC(0x0069)  /* Simple check */
#define SNIOC_FULL_CHECK           _SNIOC(0x006a)  /* Full check */
#define SNIOC_FEAT_MANAGE          _SNIOC(0x006b)  /* Feature manage command */
#define SNIOC_SET_SCALE_XL         _SNIOC(0x006c)  /* Set accelerator scale command */

/* Command:      SNIOC_GET_STATE
 * Description:  Get state for all subscribers, include min_interval,
 *               min_latency and the number of subscribers.
 * Argument:     This is the state pointer
 */

#define SNIOC_GET_STATE            _SNIOC(0x0080)

/* Command:      SNIOC_SET_INTERVAL
 * Description:  Set interval between samples
 * Argument:     This is the interval pointer, in microseconds
 */

#define SNIOC_SET_INTERVAL         _SNIOC(0x0081)

/* Command:      SNIOC_BATCH
 * Description:  Set batch latency between batch data.
 * Argument:     This is the latency pointer, in microseconds
 */

#define SNIOC_BATCH                _SNIOC(0x0082)

/* Command:      SNIOC_SET_USERPRIV
 * Description:  Set the private data of userspace user.
 * Argument:     This is the pointer of private data.
 */

#define SNIOC_SET_USERPRIV         _SNIOC(0x0083)

/* Command:      SNIOC_SET_BUFFER_NUMBER
 * Description:  Set the number of events intermediate circualr buffer can
 *               hold in upper half driver.
 * Argument:     This is the number of events that buffer can hold.
 * Note:         The application layer can set number of events intermediate
 *               circualr buffer can hold by this ioctl command.
 */

#define SNIOC_SET_BUFFER_NUMBER    _SNIOC(0x0084)

/* IOCTL commands unique to the Hall effect sensor */

#define SNIOC_GET_POSITION         _SNIOC(0x0085)

/* Command:      SNIOC_SELFTEST
 * Description:  Take a selftest for sensor.
 * Argument:     A argument of selftest for sensor.
 * Note:         If selftest is failed, return errno, otherwise, return OK.
 *               This cmd is handled by sensor_ops_s::selftest.
 */

#define SNIOC_SELFTEST             _SNIOC(0x0086)

/* Command:      SNIOC_SET_CALIBVALUE
 * Description:  Set calibration value for sensor.
 * Argument:     Calibration value for the sensor.
 * Note:         If setting calibvalue failed, a negated errno value is
 *               returned, otherwise, OK is returned.
 *               This cmd is handled by sensor_ops_s::set_calibvalue.
 */

#define SNIOC_SET_CALIBVALUE       _SNIOC(0x0087)

/* Command:      SNIOC_CALIBRATE
 * Description:  Trigger calibration and obtain calibration value.
 * Argument:     A argument of calibration value for sensor.
 * Note:         If getting calibvalue is failed, return errno, otherwise,
 *               return OK.
 *               This cmd is handled by sensor_ops_s::get_calibvalue.
 */

#define SNIOC_CALIBRATE            _SNIOC(0x0088)

#ifdef CONFIG_USENSOR
/* Command:      SNIOC_REGISTER
 * Description:  Register user sensor.
 * Argument:     A pointer of structure sensor_reginfo_s for user sensor.
 * Note:         If register is failed, return errno, otherwise,
 *               return OK.
 */

#define SNIOC_REGISTER             _SNIOC(0x0089)

/* Command:      SNIOC_UNREGISTER
 * Description:  Unregister user sensor.
 * Argument:     The path name of user sensor character node.
 * Note:         If register is failed, return errno, otherwise,
 *               return OK.
 */

#define SNIOC_UNREGISTER           _SNIOC(0x0090)
#endif

/* Command:      SNIOC_UPDATED
 * Description:  Check whether the topic has been updated since
 *               it was last read.
 * Argument:     Sets *(bool *)arg
 */

#define SNIOC_UPDATED              _SNIOC(0x0091)

/* Command:      SNIOC_GET_USTATE
 * Description:  Get state for user.
 * Argument:     This is the state pointer(struct sensor_state_s)
 */

#define SNIOC_GET_USTATE           _SNIOC(0x0092)

/* IOCTL commands unique to the APDS9922 sensor.
 * The sensor detects both ambient light and proximity.
 */

                                                  /* Args:                 */

/* SNIOC_RESET - as defined already, above           None                  */

/* SNIOC_GET_DEV_ID - as defined already, above      uint8_t* pointer      */

#define SNIOC_ALS_CONFIG           _SNIOC(0x0093) /* struct
                                                   * apds9922_als_setup_s* */
#define SNIOC_PS_CONFIG            _SNIOC(0x0094) /* struct
                                                   * apds9922_ps_setup_s*  */

/* Set proximity sensor cancellation level */

#define SNIOC_PS_CANC_LVL          _SNIOC(0x0095) /* uint16_t level        */                                                  

/* IOCTL commands for MPU60x0 IMU */

/* Command:      SNIOC_SET_AFS_SEL
 * Description:  Sets the accelerometer full scale range.
 * Argument:     Accepts 0, 1, 2 or 3 (uint8_t).
 */

#define SNIOC_SET_AFS_SEL             _SNIOC(0x0096)

/* Command:      SNIOC_SMPLRT_DIV
 * Description:  Set the divider from the gyroscope output rate
 *               used to generate the sample rate.
 * Argument:     Sets the 8bit sample rate divider (uint8_t).
 */

#define SNIOC_SMPLRT_DIV              _SNIOC(0x0097)

/* Command:      SNIOC_READ_SAMPLE_RATE
 * Description:  Reads the current sample rate for the IMU.
 * Argument:     Requires pointer *uint32_t as argument to
 *               store integer approximation of current sample rate.
 */

#define SNIOC_READ_SAMPLE_RATE        _SNIOC(0x0098)

/* Command:      SNIOC_READ_FIFO_COUNT
 * Description:  Reads the current number of bytes in FIFO buffer.
 * Argument:     Requires pointer *uint16_t as argument to
 *               store current number of bytes available.
 */

#define SNIOC_READ_FIFO_COUNT         _SNIOC(0x0099)

/* Command:      SNIOC_ENABLE_FIFO
 * Description:  Enables or disabled FIFO buffer.
 * Argument:     Bool value: true enables and false disables.
 */

#define SNIOC_ENABLE_FIFO             _SNIOC(0x009A)

#endif /* __INCLUDE_NUTTX_SENSORS_IOCTL_H */
