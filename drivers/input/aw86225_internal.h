/****************************************************************************
 * drivers/input/aw86225_internal.h
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

#ifndef __DRIVERS_INPUT_AW86225_INTERNAL_H_
#define __DRIVERS_INPUT_AW86225_INTERNAL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <time.h>

#include <nuttx/atomic.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/ff.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AW86225_I2C_RETRIES           (5)
#define AW86225_RTP_NAME_MAX          (64)
#define AW86225_SEQUENCER_SIZE        (8)
#define AW86225_SEQUENCER_LOOP_SIZE   (4)
#define AW86225_OSC_CALI_MAX_LENGTH   (11000000)
#define AW86225_PM_QOS_VALUE_VB       (0)
#define AW86225_VBAT_REFER            (4200)
#define AW86225_VBAT_MIN              (3000)
#define AW86225_VBAT_MAX              (5500)
#define AW86225_TRIG_NUM              (3)
#define AW86225_I2C_RETRY_DELAY       (2)
#define FF_EFFECT_COUNT_MAX           5
#define HAP_BRAKE_PATTERN_MAX         4

#define AW_CHECK_RAM_DATA
#define AW_READ_BIN_FLEXBALLY

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum aw86225_haptics_custom_effect_param
{
  AW86225_CUSTOM_DATA_EFFECT_IDX,
  AW86225_CUSTOM_DATA_TIMEOUT_SEC_IDX,
  AW86225_CUSTOM_DATA_TIMEOUT_MSEC_IDX,
  AW86225_CUSTOM_DATA_LEN,
};

enum haptic_nv_read_chip_type
{
  AW86225_FIRST_TRY = 0,
  AW86225_LAST_TRY = 1,
};

enum aw86225_flags
{
  AW86225_FLAG_NONR = 0,
  AW86225_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw86225_haptic_work_mode
{
  AW86225_HAPTIC_STANDBY_MODE = 0,
  AW86225_HAPTIC_RAM_MODE = 1,
  AW86225_HAPTIC_RTP_MODE = 2,
  AW86225_HAPTIC_TRIG_MODE = 3,
  AW86225_HAPTIC_CONT_MODE = 4,
  AW86225_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw86225_haptic_activate_mode
{
  AW86225_HAPTIC_ACTIVATE_RAM_MODE = 0,
  AW86225_HAPTIC_ACTIVATE_CONT_MODE = 1,
  AW86225_HAPTIC_ACTIVATE_RTP_MODE = 2,
  AW86225_HAPTIC_ACTIVATE_RAM_LOOP_MODE = 3,
};

enum aw86225_haptic_cont_vbat_comp_mode
{
  AW86225_HAPTIC_CONT_VBAT_SW_ADJUST_MODE = 0,
  AW86225_HAPTIC_CONT_VBAT_HW_ADJUST_MODE = 1,
};

enum aw86225_haptic_ram_vbat_compensate_mode
{
  AW86225_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
  AW86225_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw86225_haptic_f0_flag
{
  AW86225_HAPTIC_LRA_F0 = 0,
  AW86225_HAPTIC_CALI_F0 = 1,
};

enum aw86225_sram_size_flag
{
  AW86225_HAPTIC_SRAM_1K = 0,
  AW86225_HAPTIC_SRAM_2K = 1,
  AW86225_HAPTIC_SRAM_3K = 2,
};

enum aw86225_haptic_pwm_mode
{
  AW86225_PWM_48K = 0,
  AW86225_PWM_24K = 1,
  AW86225_PWM_12K = 2,
};

enum aw86225_haptic_play
{
  AW86225_HAPTIC_PLAY_NULL = 0,
  AW86225_HAPTIC_PLAY_ENABLE = 1,
  AW86225_HAPTIC_PLAY_STOP = 2,
  AW86225_HAPTIC_PLAY_GAIN = 8,
};

enum aw86225_haptic_cmd
{
  AW86225_HAPTIC_CMD_NULL = 0,
  AW86225_HAPTIC_CMD_ENABLE = 1,
  AW86225_HAPTIC_CMD_HAPTIC = 0x0f,
  AW86225_HAPTIC_CMD_TP = 0x10,
  AW86225_HAPTIC_CMD_SYS = 0xf0,
  AW86225_HAPTIC_CMD_STOP = 255,
};

enum aw86225_haptic_cali_lra
{
  AW86225_WRITE_ZERO = 0,
  AW86225_F0_CALI = 1,
  AW86225_OSC_CALI = 2,
};

enum aw86225_haptic_rtp_mode
{
  AW86225_RTP_SHORT = 4,
  AW86225_RTP_LONG = 5,
  AW86225_RTP_SEGMENT = 6,
};

enum aw86225_ef_id
{
  AW86225_EF_ID = 0x00,
};

struct aw86225_firmware
{
  FAR const uint8_t *data;
  size_t size;
};

struct aw86225_hap_play_info
{
  FAR struct aw86225_hap_effect *effect;
  uint16_t vmax_mv;
  int length_us;
  int playing_pos;
  bool playing_pattern;
};

struct aw86225_hap_config
{
  uint16_t vmax_mv;
  uint16_t play_rate_us;
  bool lra_allow_variable_play_rate;
  bool use_ext_wf_src;
};

struct aw86225_ram
{
  unsigned int len;
  unsigned int check_sum;
  unsigned int base_addr;
  unsigned char version;
  unsigned char ram_shift;
  unsigned char baseaddr_shift;
};

struct aw86225_container
{
  int len;
  unsigned char data[];
};

struct aw86225
{
  struct ff_lowerhalf_s lower;
  FAR struct i2c_master_s *i2c;
  FAR struct ioexpander_dev_s *ioedev;

  /* Struct snd_soc_codec *codec; */

  mutex_t lock;
  mutex_t rtp_lock;
  struct wdog_s timer;
  struct wdog_s ram_timer;
  struct work_s long_vibrate_work;
  struct work_s rtp_work;
  struct work_s set_gain_work;
  struct work_s ram_work;
  struct aw86225_hap_config hap_config;
  struct aw86225_hap_play_info play;
  FAR struct aw86225_pattern_s *pattern;
  FAR struct aw86225_hap_effect *predefined;
  struct aw86225_hap_effect constant;
  FAR struct aw86225_config *config;
  struct aw86225_ram ram;
  FAR struct aw86225_container *rtp_container;

  unsigned char seq[AW86225_SEQUENCER_SIZE];
  unsigned char loop[AW86225_SEQUENCER_SIZE];
  unsigned char rtp_init;
  unsigned char ram_init;
  unsigned char ram_vbat_compensate;
  unsigned char play_mode;
  unsigned char activate_mode;
  unsigned char ram_state;
  unsigned char wk_lock_flag;

  bool is_used_irq;

  int name;
  int freq;
  int reset_gpio;
  int irq_gpio;
  int irq;
  int state;
  int duration;
  int effect_type;
  int amplitude;
  int index;
  int gain;
  int effect_id;
  int effects_count;

  unsigned int rtp_cnt;
  unsigned int rtp_file_num;
  unsigned int f0;
  unsigned int cont_f0;
  unsigned int cont_drv1_lvl;
  unsigned int cont_drv2_lvl;
  unsigned int cont_brk_time;
  unsigned int cont_wait_num;
  unsigned int cont_drv1_time;
  unsigned int cont_drv2_time;
  unsigned int vbat;
  unsigned int lra;
  unsigned int ram_update_flag;
  unsigned int rtp_update_flag;
  unsigned int osc_cali_data;
  unsigned int f0_cali_data;
  unsigned int timeval_flags;
  unsigned int osc_cali_flag;
  unsigned int sys_frequency;
  unsigned int rtp_len;
  unsigned long int microsecond;

  uint8_t addr;
  uint16_t new_gain;
  unsigned char level;
  unsigned int osc_cali_run;
  unsigned char ram_vbat_comp;
  atomic_t is_in_rtp_loop;
  atomic_t exit_in_rtp_loop;
  sem_t wait_q;
  sem_t stop_wait_q;
};

#endif /* __DRIVERS_INPUT_AW86225_INTERNAL_H */
