/****************************************************************************
 * include/nuttx/input/aw86225.h
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

#ifndef __INCLUDE_NUTTX_INPUT_AW86225_H_
#define __INCLUDE_NUTTX_INPUT_AW86225_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdbool.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AW86225_HAP_BRAKE_PATTERN_MAX 4   /* waveform brake pattern length */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* struct aw86225_pattern_s - effect pattern configuration parameters
 */

struct aw86225_pattern_s
{
  uint8_t patternid[8];
  uint8_t waveloop[8];
  uint8_t mainloop;
  float strength;
  uint32_t duration; /* in millisecond */
};

/* struct aw86225_hap_effect - effect configuration parameters
 */

struct aw86225_hap_effect
{
  int id;
  FAR uint8_t *pattern;
  int pattern_length;
  uint16_t play_rate_us;
  uint16_t vmax_mv;
  uint8_t wf_repeat_n;
  uint8_t wf_s_repeat_n;
  uint8_t brake[AW86225_HAP_BRAKE_PATTERN_MAX];
  int brake_pattern_length;
  bool brake_en;
  bool lra_auto_res_disable;
};

/* struct aw86225_config - defines parameters of aw86225
 * configuration.
 */

struct aw86225_config
{
  /* AW86225 work mode init */

  unsigned int mode;

  /* F0 work param */

  unsigned int f0_ref;
  unsigned int f0_cali_percent;

  /* Cont work param */

  unsigned int cont_drv1_lvl_dt;
  unsigned int cont_drv2_lvl_dt;
  unsigned int cont_drv1_time_dt;
  unsigned int cont_drv2_time_dt;
  unsigned int cont_wait_num_dt;
  unsigned int cont_brk_time_dt;
  unsigned int cont_track_margin;
  unsigned int cont_tset;
  unsigned int cont_bemf_set;
  unsigned int cont_brk_gain;

  /* Motor d2s_gain strength param */

  unsigned int d2s_gain;

  /*  AW86225 reg sysctrl value */

  unsigned int sine_array[4];

  /* The boundry between ram mode and rtp mode */

  unsigned int effect_id_boundary;

  /* The number of rtp file max */

  unsigned int effect_max;

  /* Duration of vibration for each rtp file */

  unsigned int rtp_time[190];

  /* Whether to activate automatic braking after RTP/RAM/CONT */

  bool is_enabled_auto_bst;
};

/* struct aw86225_board_config - aw86225 driver param
 * configuration.
 */

struct aw86225_board_config
{
  bool is_used_irq;
  int rstpin;
  int intpin;
  int powerpin;
  int irq;
  int effects_count;
  uint8_t addr;            /* I2C address */
  int freq;                /* I2C frequency */
  FAR struct aw86225_hap_effect *predefined;
  FAR struct aw86225_pattern_s *pattern;

  /* Motor driver registration path */

  FAR const char *path;
  FAR struct aw86225_config *config;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aw86225_initialize
 *
 * Description:
 *   aw86225 motor driver initialize
 *
 * Input Parameters:
 *   master - i2c master param
 *   ioedev - io dev pin set
 *   config - the board config param of aw86225
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int aw86225_initialize(FAR struct i2c_master_s *master,
                       FAR struct ioexpander_dev_s *ioedev,
                       FAR const struct aw86225_board_config *config);

#endif /* __INCLUDE_NUTTX_INPUT_AW86225_H_ */
