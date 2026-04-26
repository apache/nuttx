/****************************************************************************
 * include/nuttx/timers/dshot.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_DSHOT_H
#define __INCLUDE_NUTTX_TIMERS_DSHOT_H

/* Common definitions for DShot drivers */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>

#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DSHOT_NCHANNELS
#  if CONFIG_DSHOT_NCHANNELS > 16
#    error "max 16 channels supported"
#  endif
#  define _DSHOT_NCHANNELS CONFIG_DSHOT_NCHANNELS
#else
#  define _DSHOT_NCHANNELS 1
#endif

/* IOCTL Commands ***********************************************************/

/* Configure the DShot instance; speed, bidirectional mode, active channels */

#define DSHOTIOC_CONFIGURE       _DSHOTIOC(0x01)

/* Write throttle data or special command for one or more channels */

#define DSHOTIOC_SET_THROTTLE    _DSHOTIOC(0x02)

/* Retrieve telemetry from the last successful bidirectional RX */

#define DSHOTIOC_GET_TELEMETRY   _DSHOTIOC(0x03)

/* DSHOT Speeds *************************************************************/

#define DSHOT_SPEED_150             150000u
#define DSHOT_SPEED_300             300000u
#define DSHOT_SPEED_600             600000u
#define DSHOT_SPEED_1200            1200000u
#define DSHOT_SPEED_2400            2400000u
#define DSHOT_SPEED_3600            3600000u

/* DSHOT special commands (throttle values 0-47)*****************************/

#define DSHOT_CMD_MOTOR_STOP                                   0
#define DSHOT_CMD_BEEP1                                        1
#define DSHOT_CMD_BEEP2                                        2
#define DSHOT_CMD_BEEP3                                        3
#define DSHOT_CMD_BEEP4                                        4
#define DSHOT_CMD_BEEP5                                        5
#define DSHOT_CMD_ESC_INFO                                     6
#define DSHOT_CMD_SPIN_DIRECTION_1                             7
#define DSHOT_CMD_SPIN_DIRECTION_2                             8
#define DSHOT_CMD_3D_MODE_OFF                                  9
#define DSHOT_CMD_3D_MODE_ON                                   10
#define DSHOT_CMD_SETTINGS_REQUEST                             11
#define DSHOT_CMD_SAVE_SETTINGS                                12
#define DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE                    13
#define DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE                   14

/* 15 - 19 Unassigned */

#define DSHOT_CMD_SPIN_DIRECTION_NORMAL                        20
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED                      21
#define DSHOT_CMD_LED0_ON                                      22
#define DSHOT_CMD_LED1_ON                                      23
#define DSHOT_CMD_LED2_ON                                      24
#define DSHOT_CMD_LED3_ON                                      25
#define DSHOT_CMD_LED0_OFF                                     26
#define DSHOT_CMD_LED1_OFF                                     27
#define DSHOT_CMD_LED2_OFF                                     28
#define DSHOT_CMD_LED3_OFF                                     29

#define DSHOT_CMD_AUDIO_STREAM_MODE                            30
#define DSHOT_CMD_SILENT_MODE                                  31

#define DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE                32
#define DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE                 33
#define DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY        34
#define DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY 35

/* 36 - 41 Unassigned */

#define DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY            42
#define DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY                43
#define DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY                44
#define DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY            45
#define DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY                   46
#define DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY            47

/* Extended telemetry types *************************************************/

#define DSHOT_EDT_TYPE_TEMP  0x02 /* Temperature in C */
#define DSHOT_EDT_TYPE_V     0x04 /* Voltage: 0.25V per step */
#define DSHOT_EDT_TYPE_A     0x06 /* Current in Amp */
#define DSHOT_EDT_TYPE_DBG1  0x08 /* Debug value 1 */
#define DSHOT_EDT_TYPE_DBG2  0x0A /* Debug value 2 */
#define DSHOT_EDT_TYPE_DBG3  0x0C /* Debug value 3 */
#define DSHOT_EDT_TYPE_STATE 0x0E /* State/Event */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Configuration argument for DSHOTIOC_CONFIGURE
 *
 * freq       - Bit rate in Hz: DSHOT_SPEED_150/300/600/1200/2400/3600
 * telem_freq - Telemetry frequency. Use 1.25 * freq for standard speed
 *              (e.g., 750kHz for DShot600). Some ESCs may require
 *              different values (e.g., 1.15 * freq for T-motor F55A).
 * active_mask - Bitmask of active channels (bit0=ch0, bit1=ch1, etc.)
 * bidir       - Enable bidirectional DShot (inverted command data,
 *               response on same line).
 */

struct dshot_config_s
{
  uint32_t freq;
  uint32_t telem_freq;
  uint16_t active_mask;
  bool bidir;
};

/* Telemetry data for a single DShot channel. Contains eRPM, extended
 * telemetry type/value, and timestamp of the last valid response.
 *
 * erpm      - eRPM value from telemetry response.
 * edt_type  - Extended telemetry type (if supported).
 * edt_value - Extended telemetry value (if supported).
 * timestamp - Timestamp of the last valid response.
 */

struct dshot_ch_telemetry_s
{
  uint16_t erpm;
  uint8_t  edt_type;               /* Extended telemetry type */
  uint8_t  edt_value;              /* Extended telemetry value */
  struct timespec timestamp;
};

/* Telemetry result for DSHOTIOC_GET_TELEMETRY ioctl command.
 * Contains bitmask of channels to read and telemetry data for each.
 *
 * ch_mask      - Bitmask of logical channels to read telemetry from.
 * ch_telemetry - Telemetry data array for each channel.
 */

struct dshot_telemetry_s
{
  uint16_t ch_mask;
  struct dshot_ch_telemetry_s ch_telemetry[_DSHOT_NCHANNELS];
};

/* Per-channel throttle value for DSHOTIOC_SET_THROTTLE ioctl command.
 * Specifies throttle values, channel mask, and optional telemetry
 * request for efficient batching of operations.
 *
 * throttle      - Throttle value array (0 = disarm, 48-2047 = armed range,
 *                 or special command values).
 * ch_mask       - Bitmask of logical channels to set.
 * telemetry_req - Bitmask: bit N requests telemetry from channel N.
 * ch_telemetry  - Telemetry data from previous request. Allows reading
 *                 previous telemetry response without separate ioctl.
 */

struct dshot_throttle_s
{
  uint16_t throttle[_DSHOT_NCHANNELS]; /* Throttle value or special command */
  uint16_t ch_mask;                    /* Bitmask of logical channels to set */
  uint16_t telemetry_req;              /* Bitmask: bit N = request telemetry ch N */
  struct dshot_ch_telemetry_s ch_telemetry[_DSHOT_NCHANNELS];
};

/* Raw telemetry frame captured from the DShot signal line.
 * Contains the raw GCR-encoded packet and capture timestamp.
 *
 * raw       - Raw 20-bit GCR encoded packet data.
 * timestamp - Capture timestamp of the raw packet.
 */

struct dshot_raw_telemetry_s
{
  uint32_t raw;                /* Raw 20-bit GCR packet data */
  struct timespec timestamp;   /* Capture timestamp */
};

/* Lower-half driver operations table. Provides callback functions for
 * the upper-half driver to control DShot hardware. All methods except
 * ioctl are mandatory.
 *
 * setup            - Called when driver is opened. Configure and initialize
 *                    device for use.
 * shutdown         - Called when driver is closed. Stop output and free
 *                    hardware resources.
 * configure        - Configure speed, active channels, and bidirectional
 *                    mode according to dshot_config_s.
 * send_command     - Send DShot command packets to specified channels.
 * get_raw_telemetry - Fetch raw telemetry packets from specified channels.
 * ioctl            - Optional platform-specific ioctl handler.
 */

struct dshot_lowerhalf_s;
struct dshot_ops_s
{
  CODE int (*setup)(FAR struct dshot_lowerhalf_s *dev);

  CODE int (*shutdown)(FAR struct dshot_lowerhalf_s *dev);

  CODE int (*configure)(FAR struct dshot_lowerhalf_s *dev,
                        FAR const struct dshot_config_s *cfg);

  CODE int (*send_command)(FAR struct dshot_lowerhalf_s *dev,
                           FAR const uint16_t *packets,
                           uint16_t ch_mask);

  CODE int (*get_raw_telemetry)(FAR struct dshot_lowerhalf_s *dev,
                                FAR struct dshot_raw_telemetry_s *raw,
                                uint16_t ch_mask);

  CODE int (*ioctl)(FAR struct dshot_lowerhalf_s *dev,
                    int cmd, unsigned long arg);
};

/* Public representation of lower-half state. */

struct dshot_lowerhalf_s
{
  FAR const struct dshot_ops_s *ops;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * "Upper-Half" DShot Driver Interfaces
 ****************************************************************************/

/****************************************************************************
 * Name: dshot_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" DSHOT device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input Parameters:
 *   path - The full path to the driver to be registered in the NuttX pseudo-
 *     filesystem.
 *   dev - A pointer to an instance of lower half DShot driver. This instance
 *     is bound to the DShot driver and must persist as long as the driver
 *     persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int dshot_register(FAR const char *path, FAR struct dshot_lowerhalf_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_DSHOT_H */
