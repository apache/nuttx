/****************************************************************************
 * include/nuttx/sensors/as5048b.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_AS5048B
#define __INCLUDE_NUTTX_SENSORS_AS5048B

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/qencoder.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_QENCODER) && defined(CONFIG_SENSORS_AS5048B)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_AS5048B
 *   Enables support for the AS5048B driver
 */

/* IOCTL Commands ***********************************************************/

#define QEIOC_ZEROPOSITION    _QEIOC(QE_AS5048B_FIRST+0) /* Arg: int32_t* pointer */
#define QEIOC_AUTOGAINCTL     _QEIOC(QE_AS5048B_FIRST+1) /* Arg: uint8_t* pointer */
#define QEIOC_DIAGNOSTICS     _QEIOC(QE_AS5048B_FIRST+2) /* Arg: uint8_t* pointer */
#define QEIOC_MAGNITUDE       _QEIOC(QE_AS5048B_FIRST+3) /* Arg: int32_t* pointer */

/* Resolution ***************************************************************/

#define AS5048B_MAX           0x3fff   /* Maximum value (14 bits) */

/* Register Definitions *****************************************************/

/* Register Addresses */

#define AS5048B_PROG_REG      0x03     /* Programming Control Register */
#define AS5048B_ADDR_REG      0x15     /* I2C Slave Address Register */
#define AS5048B_ZEROLO_REG    0x16     /* Zero Position Register Bits 0 to 5 */
#define AS5048B_ZEROHI_REG    0x17     /* Zero Position Register Bits 6 to 13 */
#define AS5048B_AGC_REG       0xfa     /* Automatic Gain Control Register */
#define AS5048B_DIAG_REG      0xfb     /* Diagnostics Register */
#define AS5048B_MAGLO_REG     0xfc     /* Magnitude Register Bits 0 to 5 */
#define AS5048B_MAGHI_REG     0xfd     /* Magnitude Register Bits 6 to 13 */
#define AS5048B_ANGLO_REG     0xfe     /* Angle Register Bits 0 to 5 */
#define AS5048B_ANGHI_REG     0xff     /* Angle Register Bits 6 to 13 */

/* Programming Control Register Bit Definitions */

#define AS5048B_PROG_ENABLE   (1 << 0)
#define AS5048B_PROG_BURN     (1 << 3)
#define AS5048B_PROG_VERIFY   (1 << 6)

/* Diagnostics Register Bit Definitions */

#define AS5048B_DIAG_OCF      (1 << 0) /* Offset Compensation Finished */
#define AS5048B_DIAG_COF      (1 << 1) /* Cordic Overflow */
#define AS5048B_DIAG_COMPLOW  (1 << 2) /* High Magnetic Field */
#define AS5048B_DIAG_COMPHIGH (1 << 3) /* Low Magnetic Field */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: as5048b_initialize
 *
 * Description:
 *   Initialize the AS5048B device.
 *
 * Input Parameters:
 *   i2c  - An I2C driver instance.
 *   addr - The I2C address of the AS5048B.
 *
 * Returned Value:
 *   A new lower half quadrature encoder interface for the AS5048B
 *        on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *as5048b_initialize(FAR struct i2c_master_s *i2c,
                                              uint8_t addr,
                                              uint32_t frequency);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_QENCODER && CONFIG_SENSORS_AS5048B */
#endif /* __INCLUDE_NUTTX_SENSORS_AS5048B */
