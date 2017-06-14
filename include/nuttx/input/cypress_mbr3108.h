/****************************************************************************
 * include/nuttx/input/cypress_mbr3108.c
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_CYPRESS_MBR3108_H_
#define __INCLUDE_NUTTX_INPUT_CYPRESS_MBR3108_H_

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

/* CYPRESS_MBR3108_CMD_SENSOR_CONF command structure. Used to reconfigure
 * chip with new configuration generated using EZ-Click tool. */

begin_packed_struct struct mbr3108_cmd_sensor_conf_s
{
  enum mbr3108_cmd_e id;
  struct mbr3108_sensor_conf_s conf;
} end_packed_struct;

/* CYPRESS_MBR3108_CMD_DEBUG_CONF command structure. Use to enable debug
 * output from chip/sensor, see 'struct mbr3108_sensor_data_s'. */

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
  int (*irq_attach) (FAR struct mbr3108_board_s *state, xcpt_t isr, FAR void *arg);
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

#endif /* __INCLUDE_NUTTX_INPUT_CYPRESS_MBR3108_H_ */
