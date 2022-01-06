/****************************************************************************
 * include/nuttx/sensors/lsm303agr.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM303AGR
#define __INCLUDE_NUTTX_SENSORS_LSM303AGR

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM303AGR)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Addresses ************************************************************/

/* https://github.com/RIOT-OS/RIOT/issues/7133 */

/* Accelerometer addresses */

#define LSM303AGRACCELERO_ADDR  (0x32 >> 1)

/* Magnetometer addresses */

#define LSM303AGRMAGNETO_ADDR  (0x3C >> 1) /* 7-bit */

/* Register Addresses *******************************************************/

/* Accelerometer and magnetometer registers */

#define LSM303AGR_STATUS_REG_AUX_A                        0x07
#define LSM303AGR_OUT_TEMP_L_A                            0x0C
#define LSM303AGR_OUT_TEMP_H_A                            0x0D
#define LSM303AGR_INT_COUNTER_REG_A                       0x0E
#define LSM303AGR_WHO_AM_I                                0x0F
#define LSM303AGR_WHO_AM_I_VALUE                          0x00
#define LSM303AGR_TEMP_CFG_REG_A                          0x1F
#define LSM303AGR_CTRL_REG1_A                             0x20
#define LSM303AGR_CTRL_REG2_A                             0x21
#define LSM303AGR_CTRL_REG3_A                             0x22
#define LSM303AGR_CTRL_REG4_A                             0x23
#define LSM303AGR_CTRL_REG5_A                             0x24
#define LSM303AGR_CTRL_REG6_A                             0x25
#define LSM303AGR_REF_DATA_CAP_A                          0x26
#define LSM303AGR_STATUS_REG_A                            0x27

#define LSM303AGR_STATUS_REG_A_SHIFT                      0
#define LSM303AGR_STATUS_REG_A_MASK                       (0 << LSM303AGR_STATUS_REG_A_SHIFT)
#define LSM303AGR_STATUS_REG_A_ZYXDA                      (1 << 3)
#define LSM303AGR_STATUS_REG_A_ZYXOR                      (1 << 7)

#define LSM303AGR_OUT_X_L_A                               0x28
#define LSM303AGR_OUT_X_H_A                               0x29
#define LSM303AGR_OUT_Y_L_A                               0x2A
#define LSM303AGR_OUT_Y_H_A                               0x2B
#define LSM303AGR_OUT_Z_L_A                               0x2C
#define LSM303AGR_OUT_Z_H_A                               0x2D

#define LSM303AGR_OUTX_L_A_SHIFT                          0
#define LSM303AGR_OUTX_L_A_MASK                           (255 << LSM303AGR_OUTX_L_A_SHIFT)

#define LSM303AGR_FIFO_CTRL_REG_A                         0x2E
#define LSM303AGR_FIFO_SRC_REG_A                          0x2F
#define LSM303AGR_INT1_CFG_A                              0x30
#define LSM303AGR_INT1_SRC_A                              0x3A
#define LSM303AGR_INT1_THS_A                              0x32
#define LSM303AGR_INT1_DURATION_A                         0x33
#define LSM303AGR_INT2_CFG_A                              0x34
#define LSM303AGR_INT2_SRC_A                              0x35
#define LSM303AGR_INT2_THS_A                              0x36
#define LSM303AGR_INT2_DURATION_A                         0x37
#define LSM303AGR_CLICK_CFG_A                             0x38
#define LSM303AGR_CLICK_SRC_A                             0x39
#define LSM303AGR_CLICK_THS_A                             0x3A
#define LSM303AGR_TIME_LIMIT_A                            0x3B
#define LSM303AGR_TIME_LATENCY_A                          0x3C
#define LSM303AGR_TIME_WINDOW_A                           0x3D
#define LSM303AGR_Act_THS_A                               0x3E
#define LSM303AGR_Act_DUR_A                               0x3F
#define LSM303AGR_OFFSET_X_REG_L_M                        0x45
#define LSM303AGR_OFFSET_X_REG_H_M                        0x46
#define LSM303AGR_OFFSET_Y_REG_L_M                        0x47
#define LSM303AGR_OFFSET_Y_REG_H_M                        0x48
#define LSM303AGR_OFFSET_Z_REG_L_M                        0x49
#define LSM303AGR_OFFSET_Z_REG_H_M                        0x4A
#define LSM303AGR_WHO_AM_I_M                              0x4F
#define LSM303AGR_CFG_REG_A_M                             0x60
#define LSM303AGR_CFG_REG_B_M                             0x61
#define LSM303AGR_CFG_REG_C_M                             0x62
#define LSM303AGR_INT_CTRL_REG_M                          0x63
#define LSM303AGR_INT_SOURCE_REG_M                        0x64
#define LSM303AGR_INT_THS_L_REG_M                         0x65
#define LSM303AGR_INT_THS_H_REG_M                         0x66
#define LSM303AGR_STATUS_REG_M                            0x67

#define LSM303AGR_STATUS_REG_M_SHIFT                      0
#define LSM303AGR_STATUS_REG_M_MASK                       (0 << LSM303AGR_STATUS_REG_M_SHIFT)
#define LSM303AGR_STATUS_REG_M_ZYXDA                      (1 << 3)
#define LSM303AGR_STATUS_REG_M_ZYXOR                      (1 << 7)

#define LSM303AGR_OUTX_L_REG_M                            0x68
#define LSM303AGR_OUTX_H_REG_M                            0x69
#define LSM303AGR_OUTY_L_REG_M                            0x6A
#define LSM303AGR_OUTY_H_REG_M                            0x6B
#define LSM303AGR_OUTZ_L_REG_M                            0x6C
#define LSM303AGR_OUTZ_H_REG_M                            0x6D

#define LSM303AGR_OUTX_L_M_SHIFT                          0
#define LSM303AGR_OUTX_L_M_MASK                           (255 << LSM303AGR_OUTX_L_M_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Container for sensor data */

struct lsm303agr_sensor_data_s
{
  int16_t  x_data;
  int16_t  y_data;
  int16_t  z_data;
  uint16_t temperature;
  int16_t  m_x_data;
  int16_t  m_y_data;
  int16_t  m_z_data;
  uint16_t timestamp;
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lsm303agr_dev_s;
struct lsm303agr_ops_s
{
  CODE int (*config)(FAR struct lsm303agr_dev_s *priv);
  CODE int (*start)(FAR struct lsm303agr_dev_s *priv);
  CODE int (*stop)(FAR struct lsm303agr_dev_s *priv);
  CODE int (*sensor_read)(FAR struct lsm303agr_dev_s *priv,
                          FAR struct lsm303agr_sensor_data_s *sensor_data);
  CODE int (*selftest)(FAR struct lsm303agr_dev_s *priv,
                       uint32_t mode);
};

struct lsm303agr_dev_s
{
  FAR struct i2c_master_s          *i2c;  /* I2C interface */
  uint8_t                           addr; /* I2C address */

  FAR const struct lsm303agr_ops_s *ops;

  uint8_t                           datareg;     /* Output data register of X low byte */
  struct lsm303agr_sensor_data_s    sensor_data; /* Sensor data container */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: lsm303agr_sensor_register
 *
 * Description:
 *   Register the lsm303agr accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/sensor0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm303agr_sensor_register(FAR const char *devpath,
                              FAR struct i2c_master_s *i2c, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM303AGR */
#endif /* __INCLUDE_NUTTX_SENSORS_LSM303AGR */
