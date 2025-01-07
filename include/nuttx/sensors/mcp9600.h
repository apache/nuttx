/****************************************************************************
 * include/nuttx/sensors/mcp9600.h
 *
 * Contributed by Matteo Golin
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

#ifndef __INCLUDE_NUTTX_SENSORS_MCP9600_H
#define __INCLUDE_NUTTX_SENSORS_MCP9600_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define MCP9600_REV_MAJOR(rev) (((rev) & 0xf0) >> 4) /* Major revision */
#define MCP9600_REV_MINOR(rev) ((rev) & 0x0f)        /* Minor revision */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

/* Alerts */

enum mcp9600_alert_e
{
  MCP9600_ALERT1 = 0, /* Alert 1 */
  MCP9600_ALERT2 = 1, /* Alert 2 */
  MCP9600_ALERT3 = 2, /* Alert 3 */
  MCP9600_ALERT4 = 3, /* Alert 4 */
};

/* Thermocouple types */

enum mcp9600_thermocouple_e
{
  MCP9600_THERMO_TYPE_K = 0x0,
  MCP9600_THERMO_TYPE_J = 0x1,
  MCP9600_THERMO_TYPE_T = 0x2,
  MCP9600_THERMO_TYPE_N = 0x3,
  MCP9600_THERMO_TYPE_S = 0x4,
  MCP9600_THERMO_TYPE_E = 0x5,
  MCP9600_THERMO_TYPE_B = 0x6,
  MCP9600_THERMO_TYPE_R = 0x7,
};

/* ADC resolution */

enum mcp9600_resolution_e
{
  MCP9600_ADC_RES_12 = 0x3, /* 12 bits */
  MCP9600_ADC_RES_14 = 0x2, /* 14 bits */
  MCP9600_ADC_RES_16 = 0x1, /* 16 bits */
  MCP9600_ADC_RES_18 = 0x0, /* 18 bits */
};

/* Number of samples */

enum mcp9600_samples_e
{
  MCP9600_SAMPLE_1 = 0x0,
  MCP9600_SAMPLE_2 = 0x1,
  MCP9600_SAMPLE_4 = 0x2,
  MCP9600_SAMPLE_8 = 0x3,
  MCP9600_SAMPLE_16 = 0x4,
  MCP9600_SAMPLE_32 = 0x5,
  MCP9600_SAMPLE_64 = 0x6,
  MCP9600_SAMPLE_128 = 0x7,
};

/* Shutdown mode options */

enum mcp9600_modes_e
{
  MCP9600_MODE_NORMAL = 0x0,   /* Normal mode */
  MCP9600_MODE_SHUTDOWN = 0x1, /* Shutdown mode */
  MCP9600_MODE_BURST = 0x2,    /* Burst mode */
};

/* Cold junction resolutions */

enum mcp9600_cold_res_e
{
  MCP9600_COLDRES_25 = 1,   /* 0.25 degrees Celsius */
  MCP9600_COLDRES_0625 = 0, /* 0.0625 degrees Celsius */
};

/* Alert configuration */

struct mcp9600_alert_conf_s
{
  enum mcp9600_alert_e
      alert;         /* The alert associated with this configuration */
  int16_t limit;     /* The temperature limit for the alert, in 0.25 degrees
                      * Celsius/LSB */
  uint8_t temp;      /* The temperature for the hysteresis threshold, in degrees
                      * Celsius. */
  bool cold_junc;    /* True to monitor cold junction, false to monitor
                      * thermocouple */
  bool falling_temp; /* True to monitor for falling temperature, false to
                      * monitor rising */
  bool active_high;  /* False for active low */
  bool int_mode;     /* Interrupt mode, or false for comparator mode */
  bool enable;       /* Enable alert output */
};

/* Device configuration of the MCP9600 */

struct mcp9600_devconf_s
{
  enum mcp9600_thermocouple_e thermo_type; /* Thermocouple type */
  uint8_t filter_coeff;                    /* Filter coefficient */
  enum mcp9600_resolution_e resolution;    /* ADC resolution */
  enum mcp9600_samples_e num_samples;      /* Number of samples */
  enum mcp9600_modes_e mode;               /* Mode of operation */
  enum mcp9600_cold_res_e cold_res;        /* Resolution of the cold
                                            * junction */
};

/* Device information of the MCP9600 */

struct mcp9600_devinfo_s
{
  uint8_t devid;    /* Device ID */
  uint8_t revision; /* Revision number; major 4 MSBs, minor 3 LSBs */
};

/* Device status */

struct mcp9600_status_s
{
  bool burst_complete; /* Burst mode conversions complete */
  bool temp_update;    /* Temperature updated */
  bool temp_exceeded;  /* Temperature range exceeded */
  bool alerts[4];      /* Alert statuses for alerts 1-4 (0-3) */
};

/* Temperature readings */

struct mcp9600_temp_s
{
  int16_t temp_delta; /* Temperature delta in degrees Celsius */
  int16_t hot_junc;   /* Hot junction temperature in degrees Celsius */
  int16_t cold_junc;  /* Cold junction temperature in degrees Celsius */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mcp9600_register
 *
 * Description:
 *   Register the MCP9600 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the MCP9600
 *   addr    - The I2C address of the MCP9600, between 0x60 and 0x67
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mcp9600_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr);

#endif /* __INCLUDE_NUTTX_SENSORS_MCP9600_H */
