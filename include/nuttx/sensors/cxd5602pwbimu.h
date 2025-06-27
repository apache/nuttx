/****************************************************************************
 * include/nuttx/sensors/cxd5602pwbimu.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_SENSORS_CXD5602PWBIMU_H
#define __INCLUDE_NUTTX_SENSORS_CXD5602PWBIMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <sys/types.h>
#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_SENSORS_CXD5602PWBIMU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

#define SNIOC_ENABLE        _SNIOC(0x0001)
#define SNIOC_SSAMPRATE     _SNIOC(0x0002)
#define SNIOC_SDRANGE       _SNIOC(0x0003)
#define SNIOC_SCALIB        _SNIOC(0x0004)
#define SNIOC_SFIFOTHRESH   _SNIOC(0x0005)
#define SNIOC_UPDATEFW      _SNIOC(0x0010)

#define SNIOC_SETDATASIZE   _SNIOC(0x0080)
#define SNIOC_WREGSPI       _SNIOC(0x0081)
#define SNIOC_RREGSPI       _SNIOC(0x0082)
#define SNIOC_WREGS         _SNIOC(0x0083)
#define SNIOC_RREGS         _SNIOC(0x0084)
#define SNIOC_RREGS_WOADR   _SNIOC(0x0085)
#define SNIOC_GETBNUM       _SNIOC(0x0086)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Interrupt configuration data structure */

typedef struct cxd5602pwbimu_config_s
{
  CODE int (*irq_attach)(FAR const struct cxd5602pwbimu_config_s *state,
                         xcpt_t isr, FAR void *arg);
  CODE void (*irq_enable)(FAR const struct cxd5602pwbimu_config_s *state,
                          bool enable);
  CODE void (*csx)(FAR const struct cxd5602pwbimu_config_s *state,
                   bool pol);
  CODE void (*power)(FAR const struct cxd5602pwbimu_config_s *state,
                     bool pol);
  CODE void (*reset)(FAR const struct cxd5602pwbimu_config_s *state,
                     bool assert);
} cxd5602pwbimu_config_t;

/****************************************************************************
 * struct 6-axis data
 ****************************************************************************/

struct cxd5602pwbimu_data_s
{
  uint32_t timestamp;       /* timestamp */
  float temp;               /* temperature */
  float gx;                 /* gyro x */
  float gy;                 /* gyro y */
  float gz;                 /* gyro z */
  float ax;                 /* accel x */
  float ay;                 /* accel y */
  float az;                 /* accel z */
};
typedef struct cxd5602pwbimu_data_s cxd5602pwbimu_data_t;

struct cxd5602pwbimu_range_s
{
  int accel; /* 2, 4, 8, 16 */
  int gyro;  /* 125, 250, 500, 1000, 2000, 4000 */
};
typedef struct cxd5602pwbimu_range_s cxd5602pwbimu_range_t;

begin_packed_struct struct cxd5602pwbimu_calib_s
{
  uint8_t  offset;
  uint32_t coef;
} end_packed_struct;
typedef struct cxd5602pwbimu_calib_s cxd5602pwbimu_calib_t;

struct cxd5602pwbimu_regs_s
{
  uint8_t      addr;    /* Register address */
  FAR uint8_t *value;   /* Write value or read value */
  uint8_t      len;     /* Length of value */
  int          slaveid; /* Target 0=master, 1,2,3=slave{1,2,3} */
};
typedef struct cxd5602pwbimu_regs_s cxd5602pwbimu_regs_t;

struct cxd5602pwbimu_updatefw_s
{
  FAR const char *path;
  void (*progress)(off_t current, off_t total);
};
typedef struct cxd5602pwbimu_updatefw_s cxd5602pwbimu_updatefw_t;

struct spi_dev_s;
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
 * Name: cxd5602pwbimu_register
 *
 * Description:
 *   Register the CXD5602PWBIMU character device as 'devpath'
 *
 * Input Parameters:
 *   devpath   - The full path to the driver to register. E.g., "/dev/imu0"
 *   dev_spi   - An instance of the SPI interface to use to communicate
 *               with CXD5602PWBIMU
 *   dev_i2c   - An instance of the I2C interface to use to communicate
 *               with CXD5602PWBIMU
 *   config    - An instance of the interrupt configuration data structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd5602pwbimu_register(FAR const char *devpath,
                           FAR struct spi_dev_s *dev_spi,
                           FAR struct i2c_master_s *dev_i2c,
                           FAR cxd5602pwbimu_config_t *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_CXD5602PWBIMU */
#endif /* __INCLUDE_NUTTX_SENSORS_CXD5602PWBIMU_H */
