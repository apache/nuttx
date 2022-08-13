/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/include/smbus_sbd.h
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

/* Copyright 2022 NXP */

#ifndef __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_INCLUDE_SMBUS_SBD_H
#define __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_INCLUDE_SMBUS_SBD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_SMBUS_SBD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Smart Battery Data Specification v1.1 registers **************************/

/* 0x00-0x07 NOT YET IMPLEMENTED! */

#if 0
#define SBD_MANUFACTURER_ACCESS       0x00
#define SBD_REMAINING_CAPACITY_ALARM  0x01
#define SBD_REMAINING_TIME_ALARM      0x02
#define SBD_BATTERY_MODE              0x03
#define SBD_AT_RATE                   0x04
#define SBD_AT_RATE_TIME_TO_FULL      0x05
#define SBD_AT_RATE_TIME_TO_EMPTY     0x06
#define SBD_AT_RATE_OK                0x07
#endif

#define SBD_TEMPERATURE               0x08
#define SBD_VOLTAGE                   0x09
#define SBD_CURRENT                   0x0a
#define SBD_AVERAGE_CURRENT           0x0b
#define SBD_MAX_ERROR                 0x0c
#define SBD_RELATIVE_STATE_OF_CHARGE  0x0d
#define SBD_ABSOLUTE_STATE_OF_CHARGE  0x0e
#define SBD_REMAINING_CAPACITY        0x0f
#define SBD_FULL_CHARGE_CAPACITY      0x10
#define SBD_RUN_TIME_TO_EMPTY         0x11
#define SBD_AVERAGE_TIME_TO_EMPTY     0x12

/* 0x13-0x16 NOT YET IMPLEMENTED! */

#if 0
#define SBD_AVERAGE_TIME_TO_FULL      0x13
#define SBD_CHARGING_CURRENT          0x14
#define SBD_CHARGING_VOLTAGE          0x15
#define SBD_BATTERY_STATUS            0x16
#endif

#define SBD_CYCLE_COUNT               0x17
#define SBD_DESIGN_CAPACITY           0x18
#define SBD_DESIGN_VOLTAGE            0x19

/* 0x1a NOT YET IMPLEMENTED! */

#if 0
#define SBD_SPECIFICATION_INFO        0x1a
#endif

#define SBD_MANUFACTURE_DATE          0x1b
#define SBD_SERIAL_NUMBER             0x1c
#define SBD_MANUFACTURER_NAME         0x20
#define SBD_DEVICE_NAME               0x21
#define SBD_DEVICE_CHEMISTRY          0x22
#define SBD_MANUFACTURER_DATA         0x23

/* Non-standard registers ***************************************************/

#define SBD_CELL6_VOLTAGE             0x3a
#define SBD_CELL5_VOLTAGE             0x3b
#define SBD_CELL4_VOLTAGE             0x3c
#define SBD_CELL3_VOLTAGE             0x3d
#define SBD_CELL2_VOLTAGE             0x3e
#define SBD_CELL1_VOLTAGE             0x3f

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Battery data */

struct smbus_sbd_data_s
{
  uint16_t temperature;              /* 0.1  K,       0 <->  6,553.5  K */
  uint16_t voltage;                  /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t current;                  /* 1.0 mA, -32,767 <-> 32,767.0 mA */
  uint16_t average_current;          /* 1.0 mA, -32,767 <-> 32,767.0 mA */
  uint16_t max_error;                /* 1.0  %,       0 <->    100.0  % */
  uint16_t relative_state_of_charge; /* 1.0  %,       0 <->    100.0  % */
  uint16_t absolute_state_of_charge; /* 1.0  %,       0 <->    100.0  % */
  uint16_t remaining_capacity;       /* 1.0 mAh,      0 <-> 65,535.0 mAh */
  uint16_t full_charge_capacity;     /* 1.0 mAh,      0 <-> 65,535.0 mAh */
  uint16_t run_time_to_empty;        /* 1.0 min,      0 <-> 65,535.0 min */
  uint16_t average_time_to_empty;    /* 1.0 min,      0 <-> 65,535.0 min */

  uint16_t cycle_count;              /* 1.0 cycle,    0 <-> 65,535.0 cycles */
  uint16_t design_capacity;          /* 1.0 mAh,      0 <-> 65,535.0 mAh */
  uint16_t design_voltage;           /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t manufacture_date;         /* (year-1980)*512 + month*32 + day */
  uint16_t serial_number;
  const char *manufacturer_name;
  const char *device_name;
  const char *device_chemistry;
  const uint8_t *manufacturer_data;
  uint8_t manufacturer_data_length;

  uint16_t cell1_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell2_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell3_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell4_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell5_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell6_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
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
 * Name: smbus_sbd_initialize
 *
 * Description:
 *   Create and register a SMBus Smart Battery Data character driver.
 *
 *   This SMBus Smart Battery Data slave character driver supports (a subset
 *   of) the Smart Battery Data Specification, Revision 1.1.  This driver
 *   provides a buffer to the I2C slave driver.  This buffer can be updated
 *   at regular intervals by a user-space application.
 *
 * Input Parameters:
 *   minor         - The SMBus Smart Battery Data slave character device will
 *                   be registered as /dev/smbus-sbdN where N is the
 *                   minor number
 *   i2c_slave_dev - An instance of the lower half I2C slave driver
 *
 * Returned Value:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int smbus_sbd_initialize(int minor, struct i2c_slave_s *i2c_slave_dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SMBUS_SBD */
#endif /* __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_INCLUDE_SMBUS_SBD_H */
