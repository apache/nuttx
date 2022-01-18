/****************************************************************************
 * include/nuttx/input/cypress_mbr3108.h
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

#ifndef __INCLUDE_NUTTX_INPUT_CYPRESS_MBR3108_H
#define __INCLUDE_NUTTX_INPUT_CYPRESS_MBR3108_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Sensor configuration for Cypress MBR3108 device */

begin_packed_struct struct mbr3108_sensor_conf_s
{
  uint8_t conf_data[128];  /* Sensor configuration, generated with EZ-Click. */
} end_packed_struct;

/* Debug configuration */

begin_packed_struct struct mbr3108_debug_conf_s
{
  bool debug_mode;         /* Configure to debug mode if 'true'. */
  uint8_t debug_sensor_id; /* Sensor to read in debug mode. */
} end_packed_struct;

/* Write commands to MBR3108 driver. */

begin_packed_struct enum mbr3108_cmd_e
{
  CYPRESS_MBR3108_CMD_SENSOR_CONF = -3,
  CYPRESS_MBR3108_CMD_DEBUG_CONF,
  CYPRESS_MBR3108_CMD_CLEAR_LATCHED,
} end_packed_struct;

/* CYPRESS_MBR3108_CMD_SENSOR_CONF command structure.
 * Used to reconfigure chip with new configuration generated using
 * EZ-Click tool.
 */

begin_packed_struct struct mbr3108_cmd_sensor_conf_s
{
  enum mbr3108_cmd_e id;
  struct mbr3108_sensor_conf_s conf;
} end_packed_struct;

/* CYPRESS_MBR3108_CMD_DEBUG_CONF command structure.
 * Use to enable debug output from chip/sensor,
 * see 'struct mbr3108_sensor_data_s'.
 */

begin_packed_struct struct mbr3108_cmd_debug_conf_s
{
  enum mbr3108_cmd_e id;
  struct mbr3108_debug_conf_s conf;
} end_packed_struct;

/* Sensor status output */

begin_packed_struct struct mbr3108_sensor_status_s
{
  unsigned int button:8;            /* MBR3108 has maximum of 8 button sensors
                                     * configurable.
                                     * Each bit in this field indicate if
                                     * corresponding button is pressed. */
  unsigned int latched_button:8;
  unsigned int proximity:2;         /* MBR3108 has maximum of 2 proximity
                                     * sensors configurable. */
  unsigned int latched_proximity:2;
} end_packed_struct;

/* Sensor debug data output */

begin_packed_struct struct mbr3108_sensor_debug_s
{
  uint8_t sensor_total_capacitance;
  uint16_t sensor_diff_counts;
  uint16_t sensor_baseline;
  uint16_t sensor_raw_counts;
  uint16_t sensor_average_counts;
} end_packed_struct;

/* Board configuration */

struct mbr3108_board_s
{
  int (*irq_attach) (FAR struct mbr3108_board_s *state,
                     xcpt_t isr,
                     FAR void *arg);
  void (*irq_enable) (FAR struct mbr3108_board_s *state, bool enable);
  int (*set_power) (FAR struct mbr3108_board_s *state, bool on);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Device registration */

int cypress_mbr3108_register(FAR const char *devpath,
                        FAR struct i2c_master_s *dev,
                        uint8_t i2c_devaddr,
                        struct mbr3108_board_s *board_config,
                        const struct mbr3108_sensor_conf_s *sensor_conf);

#endif /* __INCLUDE_NUTTX_INPUT_CYPRESS_MBR3108_H */
