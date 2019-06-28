/*****************************************************************************
 * drivers/sensors/vl53l1x.h
 *
 *   Copyright (C) 2019 Acutronics Robotics. All rights reserved.
 *   Author: Acutronics Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_VL53L1X_H
#define __INCLUDE_NUTTX_SENSORS_VL53L1X_H

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_VL53L1X)

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define VL53L1X_I2C_PORTNO 1
#define VL53L1X_IMPLEMENTATION_VER_MAJOR                        1
#define VL53L1X_IMPLEMENTATION_VER_MINOR                        0
#define VL53L1X_IMPLEMENTATION_VER_SUB                          1
#define VL53L1X_IMPLEMENTATION_VER_REVISION                     0000

#define SOFT_RESET                                              0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                        0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND            0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS          0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS      0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS      0x001a
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM                      0x001e
#define MM_CONFIG__INNER_OFFSET_MM                              0x0020
#define MM_CONFIG__OUTER_OFFSET_MM                              0x0022
#define GPIO_HV_MUX__CTRL                                       0x0030
#define GPIO__TIO_HV_STATUS                                     0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO                           0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP                         0x004b
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI                       0x005e
#define RANGE_CONFIG__VCSEL_PERIOD_A                            0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                            0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI                       0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO                       0x0062
#define RANGE_CONFIG__SIGMA_THRESH                              0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS             0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH                          0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD                  0x006c
#define SYSTEM__THRESH_HIGH                                     0x0072
#define SYSTEM__THRESH_LOW                                      0x0074
#define SD_CONFIG__WOI_SD0                                      0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0                            0x007a
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD                        0x007f
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE           0x0080
#define SYSTEM__SEQUENCE_CONFIG                                 0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD                   0x0082
#define SYSTEM__INTERRUPT_CLEAR                                 0x0086
#define SYSTEM__MODE_START                                      0x0087
#define VL53L1_RESULT__RANGE_STATUS                             0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0           0x008c
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD                      0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0   0x0096
#define VL53L1_RESULT__OSC_CALIBRATE_VAL                        0x00de
#define VL53L1_FIRMWARE__SYSTEM_STATUS                          0x00e5
#define VL53L1_IDENTIFICATION__MODEL_ID                         0x010f
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD                 0x013e

#define VL53L1X_DEFAULT_DEVICE_ADDRESS                          0x52

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: bmp180_register
 *
 * Description:
 *   Register the BMP180 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP180
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vl53l1x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif  /* CONFIG_I2C && CONFIG_SENSORS_VL53L1X */
#endif  /* __INCLUDE_NUTTX_SENSORS_VL53L1X_H */
