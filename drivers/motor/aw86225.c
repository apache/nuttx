/****************************************************************************
 * drivers/motor/aw86225.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>

#include <nuttx/motor/motor.h>
#include <nuttx/motor/aw86225.h>
#include "aw86225_reg.h"
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AW86225_BASERAM_ADDR                (0x0000)
#define AW86225_RAMDATA_SHIFT               (0)
#define AW86225_SOFT_RESET                  (0xaa)
#define AW86225_VBAT_MIN                    (3000)
#define AW86225_VBAT_MAX                    (4500)
#define AW86225_VBAT_REFER                  (4200)
#define AW86225_DEFAULT_GAIN                (0x80)
#define AW86225_RAMDATA_MAX_SIZE            (1024 * 3)
#define AW86225_RL_DELAY                    (3000)
#define AW86225_F0_DELAY                    (100000)
#define AW86225_GO_DELAY                    (2000)
#define AW86225_SET_RAMADDR_H(addr)         ((addr) >> 8)
#define AW86225_SET_RAMADDR_L(addr)         ((addr) & 0x00ff)
#define AW86225_SET_BASEADDR_H(addr)        ((addr) >> 8)
#define AW86225_SET_BASEADDR_L(addr)        ((addr) & 0x00ff)
#define AW86225_F0_CALI_ACCURACY            (24)
#define AW86225_VBAT_FORMULA(code)          (6100 * (code) / 1024)
#define AW86225_F0_FORMULA(code)            (384000 * 10 / (code))
#define AW86225_RL_FORMULA(code)            (((code) * 678* 100) / (1024))
#define AW86225_RL_MAX                      (29000 * 1.3)
#define AW86225_SINEWAVE_ID                 (6) /* sinewave id */
#define MAX_RETRIES                          5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aw86225_dts_info_s
{
  bool enable_auto_brk;
  uint32_t f0_cali_percent;
  uint32_t f0_pre;
  uint8_t cont_tset;
  uint8_t cont_drv1_lvl;
  uint8_t cont_drv2_lvl;
  uint8_t cont_drv1_time;
  uint8_t cont_drv2_time;
  uint8_t cont_wait_num;
  uint8_t cont_brk_time;
  uint8_t cont_track_margin;
  uint8_t cont_drv_width;
  uint8_t cont_bemf_set;
  uint8_t cont_brk_gain;
  uint8_t d2s_gain;
  uint8_t sine_array[4];
};

struct aw86225_data_s
{
  uint8_t calib_finish;
  uint8_t f0_cali_data;
  uint32_t f0;
  uint32_t lra;
};

struct aw86225_dev_s
{
  struct motor_lowerhalf_s lower;            /* The struct of lower half */
  FAR const struct aw86225_config_s *config; /* The board config function */
  struct aw86225_data_s data;                /* The struct of data */
  int pattern_index;
  uint8_t state;
  uint8_t pattern_play_cnt;
  FAR struct aw86225_patterns_s *patterns;
  bool patterns_reload;                      /* Patterns changed */
  struct wdog_s wdog;
  struct work_s worker;

  clock_t timestamp;                         /* timestamp of last vibration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions */

static int aw86225_i2c_read(FAR struct aw86225_dev_s *priv, uint8_t addr,
                            FAR uint8_t *val, uint32_t cnt);
static int aw86225_i2c_write(FAR struct aw86225_dev_s *priv, uint8_t addr,
                             FAR const uint8_t *val, uint32_t cnt);
static int aw86225_i2c_writereg(FAR struct aw86225_dev_s *priv, uint8_t addr,
                                uint8_t val);
static int aw86225_i2c_readreg(FAR struct aw86225_dev_s *priv, uint8_t addr,
                               FAR uint8_t *val);
static int aw86225_i2c_write_bits(FAR struct aw86225_dev_s *priv,
                                  uint8_t addr, uint8_t mask, uint8_t val);

/* control functions */

static int aw86225_play_go(FAR struct aw86225_dev_s *priv, bool flag);
static int aw86225_haptic_stop(FAR struct aw86225_dev_s *priv);
static int aw86225_misc_para_init(FAR struct aw86225_dev_s *priv);
static int aw86225_raminit(FAR struct aw86225_dev_s *priv, bool flag);
static int aw86225_offset_cali(FAR struct aw86225_dev_s *priv);
static int aw86225_set_trim_lra(FAR struct aw86225_dev_s *priv, uint8_t val);
static int aw86225_set_wav_seq(FAR struct aw86225_dev_s *priv, uint8_t wav,
                               uint8_t seq);
static int aw86225_set_wav_loop(FAR struct aw86225_dev_s *priv, uint8_t wav,
                                uint8_t loop);
static int aw86225_read_lra_f0(FAR struct aw86225_dev_s *priv);
static int aw86225_cont_get_f0(FAR struct aw86225_dev_s *priv);
static int aw86225_calculate_cali_data(FAR struct aw86225_dev_s *priv);
static int aw86225_set_base_addr(FAR struct aw86225_dev_s *priv);
static int aw86225_set_ram_addr(FAR struct aw86225_dev_s *priv);
static int aw86225_set_ram_data(FAR struct aw86225_dev_s *priv);
static uint32_t aw86225_get_lra_resistance(FAR struct aw86225_dev_s *priv);
static int aw86225_set_calib_param(FAR struct aw86225_dev_s *priv,
                                   unsigned long arg);
static int aw86225_cali_judge(FAR struct aw86225_dev_s *priv);
static int aw86225_calibration(FAR struct aw86225_dev_s *priv,
                               unsigned long arg);
static int aw86225_read_chipid(FAR struct aw86225_dev_s *priv);
static int aw86225_haptic_init(FAR struct aw86225_dev_s *priv);
static int aw86225_ram_update(FAR struct aw86225_dev_s *priv);
static int aw86225_reg_dump(FAR struct aw86225_dev_s *priv,
                            uint8_t startaddr, uint8_t endaddr);
static void worker_cb(FAR void *arg);
static void wdog_timer_cb(wdparm_t arg);
static int aw86225_excute_pattern(FAR struct aw86225_dev_s *priv,
                                  FAR struct aw86225_pattern_s *pattern);
static int aw86225_excute_patterns(FAR struct aw86225_dev_s *priv,
                                   FAR struct aw86225_patterns_s
                                   *patterns);
static int aw86225_continus_work(FAR struct aw86225_dev_s *priv, float gain);

/* vibrator ops functions */

static int aw86225_setup(FAR struct motor_lowerhalf_s *dev);
static int aw86225_shutdown(FAR struct motor_lowerhalf_s *dev);
static int aw86225_stop(FAR struct motor_lowerhalf_s *dev);
static int aw86225_start(FAR struct motor_lowerhalf_s *dev);
static int aw86225_setparam(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_params_s *param);
static int aw86225_setmode(FAR struct motor_lowerhalf_s *dev, uint8_t mode);
static int aw86225_setlimit(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_limits_s *limits);
static int aw86225_setfault(FAR struct motor_lowerhalf_s *dev,
                            uint8_t fault);
static int aw86225_getstate(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_state_s *state);
static int aw86225_getfault(FAR struct motor_lowerhalf_s *dev,
                            FAR uint8_t *fault);
static int aw86225_clearfault(FAR struct motor_lowerhalf_s *dev,
                              uint8_t fault);
static int aw86225_ioctl(FAR struct motor_lowerhalf_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_aw86225_ram_data[] =
{
  0x55, 0x00, 0x21, 0x00, 0xcf, 0x00, 0xd0, 0x02, 0x01, 0x02, 0x02, 0x03,
  0x95, 0x03, 0x96, 0x05, 0x8e, 0x05, 0x8f, 0x07, 0xeb, 0x07, 0xec, 0x08,
  0x81, 0x08, 0x82, 0x09, 0x7b, 0x09, 0x7c, 0x0a, 0x75, 0x4c, 0x5d, 0x69,
  0x70, 0x75, 0x78, 0x7a, 0x7c, 0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7d, 0x7d,
  0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4,
  0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x83, 0x83,
  0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6, 0x1a,
  0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d, 0x7d,
  0x7d, 0x7d, 0x7c, 0x7c, 0x7a, 0x78, 0x74, 0x6f, 0x67, 0x5b, 0x48, 0x2b,
  0x00, 0xf4, 0xe8, 0xde, 0xd6, 0xd1, 0xcf, 0xcf, 0xd3, 0xda, 0xe3, 0xee,
  0xfa, 0x06, 0x12, 0x1d, 0x26, 0x2d, 0x31, 0x31, 0x2f, 0x2a, 0x22, 0x18,
  0x0c, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95, 0x8e, 0x88,
  0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6, 0xc3, 0xd2,
  0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72,
  0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d,
  0x2e, 0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95,
  0x8e, 0x88, 0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6,
  0xc3, 0xd2, 0xe1, 0xf1, 0x4c, 0x5d, 0x69, 0x70, 0x75, 0x78, 0x7a, 0x7c,
  0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72,
  0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a,
  0x87, 0x85, 0x84, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 0x85, 0x87,
  0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6, 0x1a, 0x3c, 0x53, 0x62, 0x6c,
  0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x7c, 0x7b,
  0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e,
  0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x82, 0x82, 0x83, 0x83,
  0x84, 0x84, 0x86, 0x88, 0x8c, 0x91, 0x99, 0xa5, 0xb8, 0xd5, 0x00, 0x0f,
  0x1e, 0x2d, 0x3c, 0x49, 0x55, 0x60, 0x69, 0x71, 0x77, 0x7c, 0x7e, 0x7e,
  0x7d, 0x7a, 0x75, 0x6e, 0x65, 0x5b, 0x4f, 0x43, 0x35, 0x26, 0x17, 0x08,
  0xf9, 0xea, 0xdb, 0xcc, 0xbe, 0xb1, 0xa6, 0x9c, 0x93, 0x8c, 0x87, 0x83,
  0x82, 0x82, 0x84, 0x88, 0x8e, 0x96, 0x9f, 0xaa, 0xb6, 0xc4, 0xd2, 0xe1,
  0xf0, 0x00, 0xf1, 0xe2, 0xd3, 0xc4, 0xb7, 0xab, 0xa0, 0x97, 0x8f, 0x89,
  0x84, 0x82, 0x82, 0x83, 0x86, 0x8b, 0x92, 0x9b, 0xa5, 0xb1, 0xbd, 0xcb,
  0xda, 0xe9, 0xf8, 0x07, 0x16, 0x25, 0x34, 0x42, 0x4f, 0x5a, 0x64, 0x6d,
  0x74, 0x79, 0x7d, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6a, 0x61, 0x56, 0x4a,
  0x3c, 0x2e, 0x1f, 0x10, 0x00, 0xf1, 0xe2, 0xd3, 0xc4, 0xb7, 0xab, 0xa0,
  0x97, 0x8f, 0x89, 0x84, 0x82, 0x82, 0x83, 0x86, 0x8b, 0x92, 0x9b, 0xa5,
  0xb1, 0xbd, 0xcb, 0xda, 0xe9, 0xf8, 0x07, 0x16, 0x25, 0x34, 0x42, 0x4f,
  0x5a, 0x64, 0x6d, 0x74, 0x79, 0x7d, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6a,
  0x61, 0x56, 0x4a, 0x3c, 0x2e, 0x1f, 0x10, 0x00, 0xf1, 0xe2, 0xd3, 0xc4,
  0xb7, 0xab, 0xa0, 0x97, 0x8f, 0x89, 0x84, 0x82, 0x82, 0x83, 0x86, 0x8b,
  0x92, 0x9b, 0xa5, 0xb1, 0xbd, 0xcb, 0xda, 0xe9, 0xf8, 0x07, 0x16, 0x25,
  0x34, 0x42, 0x4f, 0x5a, 0x64, 0x6d, 0x74, 0x79, 0x7d, 0x7e, 0x7e, 0x7c,
  0x78, 0x72, 0x6a, 0x61, 0x56, 0x4a, 0x3c, 0x2e, 0x1f, 0x10, 0x4c, 0x5d,
  0x69, 0x70, 0x75, 0x78, 0x7a, 0x7c, 0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7d,
  0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6,
  0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x83,
  0x83, 0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6,
  0x1a, 0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d,
  0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c,
  0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83,
  0x83, 0x83, 0x83, 0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad,
  0xc4, 0xe6, 0x1a, 0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c,
  0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62,
  0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85,
  0x84, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94,
  0x9e, 0xad, 0xc4, 0xe6, 0x1a, 0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79,
  0x7b, 0x7c, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72,
  0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a,
  0x87, 0x85, 0x84, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 0x84, 0x86, 0x88,
  0x8c, 0x91, 0x99, 0xa5, 0xb8, 0xd5, 0x00, 0x1a, 0x34, 0x4c, 0x5f, 0x6f,
  0x79, 0x7e, 0x7d, 0x77, 0x6a, 0x59, 0x44, 0x2c, 0x12, 0xf8, 0xdd, 0xc4,
  0xad, 0x9b, 0x8d, 0x84, 0x82, 0x84, 0x8d, 0x9b, 0xad, 0xc3, 0xdd, 0x00,
  0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e,
  0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f,
  0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95, 0x8e, 0x88, 0x84,
  0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6, 0xc3, 0xd2, 0xe1,
  0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72, 0x78,
  0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d, 0x2e,
  0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95, 0x8e,
  0x88, 0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6, 0xc3,
  0xd2, 0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b,
  0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a,
  0x3d, 0x2e, 0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f,
  0x95, 0x8e, 0x88, 0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa,
  0xb6, 0xc3, 0xd2, 0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56,
  0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61,
  0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f, 0x4c, 0x5d, 0x69, 0x70, 0x75, 0x78,
  0x7a, 0x7c, 0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7d, 0x7d, 0x7c, 0x7b, 0x79,
  0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94,
  0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 0x85,
  0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6, 0x1a, 0x3c, 0x53, 0x62,
  0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x7c,
  0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad,
  0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x83, 0x83, 0x83,
  0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6, 0x1a, 0x3c,
  0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d, 0x7d, 0x7d,
  0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6,
  0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x83,
  0x83, 0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6,
  0x1a, 0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d,
  0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c,
  0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83,
  0x83, 0x83, 0x83, 0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad,
  0xc4, 0xe6, 0x1a, 0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c,
  0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62,
  0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85,
  0x84, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 0x84, 0x86, 0x88, 0x8c, 0x91,
  0x99, 0xa5, 0xb8, 0xd5, 0x00, 0x19, 0x32, 0x49, 0x5c, 0x6c, 0x77, 0x7d,
  0x7e, 0x7a, 0x71, 0x63, 0x50, 0x3b, 0x23, 0x09, 0xf1, 0xd7, 0xc0, 0xab,
  0x9a, 0x8d, 0x85, 0x82, 0x84, 0x8b, 0x97, 0xa8, 0xbc, 0xd4, 0x00, 0x0f,
  0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e, 0x7e,
  0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f, 0x00,
  0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95, 0x8e, 0x88, 0x84, 0x82,
  0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6, 0xc3, 0xd2, 0xe1, 0xf1,
  0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72, 0x78, 0x7c,
  0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d, 0x2e, 0x1f,
  0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95, 0x8e, 0x88,
  0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6, 0xc3, 0xd2,
  0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72,
  0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d,
  0x2e, 0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95,
  0x8e, 0x88, 0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6,
  0xc3, 0xd2, 0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61,
  0x6b, 0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56,
  0x4a, 0x3d, 0x2e, 0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa,
  0x9f, 0x95, 0x8e, 0x88, 0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f,
  0xaa, 0xb6, 0xc3, 0xd2, 0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a,
  0x56, 0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b,
  0x61, 0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f, 0x4c, 0x5d, 0x69, 0x70, 0x75,
  0x78, 0x7a, 0x7c, 0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7d, 0x7d, 0x7c, 0x7b,
  0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e,
  0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84,
  0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6, 0x1a, 0x3c, 0x53,
  0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d,
  0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4,
  0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83, 0x83, 0x83,
  0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4, 0xe6, 0x1a,
  0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d, 0x7d, 0x7d,
  0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53, 0x3c, 0x1a,
  0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84, 0x83, 0x83,
  0x83, 0x83, 0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e, 0xad, 0xc4,
  0xe6, 0x1a, 0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b, 0x7c, 0x7d,
  0x7d, 0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c, 0x62, 0x53,
  0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87, 0x85, 0x84,
  0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 0x85, 0x87, 0x8a, 0x8e, 0x94, 0x9e,
  0xad, 0xc4, 0xe6, 0x1a, 0x3c, 0x53, 0x62, 0x6c, 0x72, 0x76, 0x79, 0x7b,
  0x7c, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76, 0x72, 0x6c,
  0x62, 0x53, 0x3c, 0x1a, 0xe6, 0xc4, 0xad, 0x9e, 0x94, 0x8e, 0x8a, 0x87,
  0x85, 0x84, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 0x84, 0x86, 0x88, 0x8c,
  0x91, 0x99, 0xa5, 0xb8, 0xd5, 0x00, 0x0a, 0x13, 0x1d, 0x26, 0x2f, 0x36,
  0x3d, 0x43, 0x48, 0x4c, 0x4e, 0x4f, 0x4f, 0x4e, 0x4c, 0x48, 0x43, 0x3d,
  0x36, 0x2f, 0x26, 0x1d, 0x13, 0x0a, 0x00, 0xf6, 0xed, 0xe3, 0xda, 0xd1,
  0xca, 0xc3, 0xbd, 0xb8, 0xb4, 0xb2, 0xb1, 0xb1, 0xb2, 0xb4, 0xb8, 0xbd,
  0xc3, 0xca, 0xd1, 0xda, 0xe3, 0xed, 0xf6, 0x00, 0x0a, 0x13, 0x1d, 0x26,
  0x2f, 0x36, 0x3d, 0x43, 0x48, 0x4c, 0x4e, 0x4f, 0x4f, 0x4e, 0x4c, 0x48,
  0x43, 0x3d, 0x36, 0x2f, 0x26, 0x1d, 0x13, 0x0a, 0x00, 0xf6, 0xed, 0xe3,
  0xda, 0xd1, 0xca, 0xc3, 0xbd, 0xb8, 0xb4, 0xb2, 0xb1, 0xb1, 0xb2, 0xb4,
  0xb8, 0xbd, 0xc3, 0xca, 0xd1, 0xda, 0xe3, 0xed, 0xf6, 0x00, 0x1a, 0x33,
  0x4a, 0x5e, 0x6d, 0x78, 0x7e, 0x7e, 0x78, 0x6d, 0x5e, 0x4a, 0x33, 0x1a,
  0x00, 0xe6, 0xcd, 0xb6, 0xa2, 0x93, 0x88, 0x82, 0x82, 0x88, 0x93, 0xa2,
  0xb6, 0xcd, 0xe6, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b,
  0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a,
  0x3d, 0x2e, 0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f,
  0x95, 0x8e, 0x88, 0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa,
  0xb6, 0xc3, 0xd2, 0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56,
  0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61,
  0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2, 0xc3, 0xb6,
  0xaa, 0x9f, 0x95, 0x8e, 0x88, 0x84, 0x82, 0x82, 0x84, 0x88, 0x8e, 0x95,
  0x9f, 0xaa, 0xb6, 0xc3, 0xd2, 0xe1, 0xf1, 0x00, 0x0f, 0x1f, 0x2e, 0x3d,
  0x4a, 0x56, 0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c, 0x78, 0x72,
  0x6b, 0x61, 0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f, 0x00, 0xf1, 0xe1, 0xd2,
  0xc3, 0xb6, 0xaa, 0x9f, 0x95, 0x8e, 0x88, 0x84, 0x82, 0x82, 0x84, 0x88,
  0x8e, 0x95, 0x9f, 0xaa, 0xb6, 0xc3, 0xd2, 0xe1, 0xf1, 0x00, 0x0f, 0x1f,
  0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e, 0x7e, 0x7c,
  0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f, 0x00, 0xf1,
  0xe1, 0xd2, 0xc3, 0xb6, 0xaa, 0x9f, 0x95, 0x8e, 0x88, 0x84, 0x82, 0x82,
  0x84, 0x88, 0x8e, 0x95, 0x9f, 0xaa, 0xb6, 0xc3, 0xd2, 0xe1, 0xf1, 0x00,
  0x0f, 0x1f, 0x2e, 0x3d, 0x4a, 0x56, 0x61, 0x6b, 0x72, 0x78, 0x7c, 0x7e,
  0x7e, 0x7c, 0x78, 0x72, 0x6b, 0x61, 0x56, 0x4a, 0x3d, 0x2e, 0x1f, 0x0f,
  0x00, 0x0d, 0x1b, 0x28, 0x34, 0x40, 0x4b, 0x54, 0x5c, 0x63, 0x68, 0x6c,
  0x6d, 0x6d, 0x6c, 0x68, 0x63, 0x5c, 0x54, 0x4b, 0x40, 0x34, 0x28, 0x1b,
  0x0d, 0x00, 0xf3, 0xe5, 0xd8, 0xcc, 0xc0, 0xb5, 0xac, 0xa4, 0x9d, 0x98,
  0x94, 0x93, 0x93, 0x94, 0x98, 0x9d, 0xa4, 0xac, 0xb5, 0xc0, 0xcc, 0xd8,
  0xe5, 0xf3, 0x00, 0x0d, 0x1b, 0x28, 0x34, 0x40, 0x4b, 0x54, 0x5c, 0x63,
  0x68, 0x6c, 0x6d, 0x6d, 0x6c, 0x68, 0x63, 0x5c, 0x54, 0x4b, 0x40, 0x34,
  0x28, 0x1b, 0x0d, 0x00, 0xf3, 0xe5, 0xd8, 0xcc, 0xc0, 0xb5, 0xac, 0xa4,
  0x9d, 0x98, 0x94, 0x93, 0x93, 0x94, 0x98, 0x9d, 0xa4, 0xac, 0xb5, 0xc0,
  0xcc, 0xd8, 0xe5, 0xf3, 0x00, 0x0d, 0x1b, 0x28, 0x34, 0x40, 0x4b, 0x54,
  0x5c, 0x63, 0x68, 0x6c, 0x6d, 0x6d, 0x6c, 0x68, 0x63, 0x5c, 0x54, 0x4b,
  0x40, 0x34, 0x28, 0x1b, 0x0d, 0x00, 0xf3, 0xe5, 0xd8, 0xcc, 0xc0, 0xb5,
  0xac, 0xa4, 0x9d, 0x98, 0x94, 0x93, 0x93, 0x94, 0x98, 0x9d, 0xa4, 0xac,
  0xb5, 0xc0, 0xcc, 0xd8, 0xe5, 0xf3, 0x00, 0x0a, 0x15, 0x1f, 0x28, 0x31,
  0x3a, 0x41, 0x47, 0x4c, 0x50, 0x53, 0x54, 0x54, 0x53, 0x50, 0x4c, 0x47,
  0x41, 0x3a, 0x31, 0x28, 0x1f, 0x15, 0x0a, 0x00, 0xf6, 0xeb, 0xe1, 0xd8,
  0xcf, 0xc6, 0xbf, 0xb9, 0xb4, 0xb0, 0xad, 0xac, 0xac, 0xad, 0xb0, 0xb4,
  0xb9, 0xbf, 0xc6, 0xcf, 0xd8, 0xe1, 0xeb, 0xf6, 0x00, 0x0a, 0x15, 0x1f,
  0x28, 0x31, 0x3a, 0x41, 0x47, 0x4c, 0x50, 0x53, 0x54, 0x54, 0x53, 0x50,
  0x4c, 0x47, 0x41, 0x3a, 0x31, 0x28, 0x1f, 0x15, 0x0a, 0x00, 0xf6, 0xeb,
  0xe1, 0xd8, 0xcf, 0xc6, 0xbf, 0xb9, 0xb4, 0xb0, 0xad, 0xac, 0xac, 0xad,
  0xb0, 0xb4, 0xb9, 0xbf, 0xc6, 0xcf, 0xd8, 0xe1, 0xeb, 0xf6, 0x00, 0x0a,
  0x15, 0x1f, 0x28, 0x31, 0x3a, 0x41, 0x47, 0x4c, 0x50, 0x53, 0x54, 0x54,
  0x53, 0x50, 0x4c, 0x47, 0x41, 0x3a, 0x31, 0x28, 0x1f, 0x15, 0x0a, 0x00,
  0xf6, 0xeb, 0xe1, 0xd8, 0xcf, 0xc6, 0xbf, 0xb9, 0xb4, 0xb0, 0xad, 0xac,
  0xac, 0xad, 0xb0, 0xb4, 0xb9, 0xbf, 0xc6, 0xcf, 0xd8, 0xe1, 0xeb, 0xf6,
  0x00, 0x0a, 0x15, 0x1f, 0x28, 0x31, 0x3a, 0x41, 0x47, 0x4c, 0x50, 0x53,
  0x54, 0x54, 0x53, 0x50, 0x4c, 0x47, 0x41, 0x3a, 0x31, 0x28, 0x1f, 0x15,
  0x0a, 0x00, 0xf6, 0xeb, 0xe1, 0xd8, 0xcf, 0xc6, 0xbf, 0xb9, 0xb4, 0xb0,
  0xad, 0xac, 0xac, 0xad, 0xb0, 0xb4, 0xb9, 0xbf, 0xc6, 0xcf, 0xd8, 0xe1,
  0xeb, 0xf6, 0x00, 0x0a, 0x15, 0x1f, 0x28, 0x31, 0x3a, 0x41, 0x47, 0x4c,
  0x50, 0x53, 0x54, 0x54, 0x53, 0x50, 0x4c, 0x47, 0x41, 0x3a, 0x31, 0x28,
  0x1f, 0x15, 0x0a, 0x00, 0xf6, 0xeb, 0xe1, 0xd8, 0xcf, 0xc6, 0xbf, 0xb9,
  0xb4, 0xb0, 0xad, 0xac, 0xac, 0xad, 0xb0, 0xb4, 0xb9, 0xbf, 0xc6, 0xcf,
  0xd8, 0xe1, 0xeb, 0xf6, 0x00, 0x05, 0x09, 0x0e, 0x13, 0x17, 0x1b, 0x1e,
  0x21, 0x24, 0x26, 0x27, 0x27, 0x27, 0x27, 0x26, 0x24, 0x21, 0x1e, 0x1b,
  0x17, 0x13, 0x0e, 0x09, 0x05, 0x00, 0xfb, 0xf7, 0xf2, 0xed, 0xe9, 0xe5,
  0xe2, 0xdf, 0xdc, 0xda, 0xd9, 0xd9, 0xd9, 0xd9, 0xda, 0xdc, 0xdf, 0xe2,
  0xe5, 0xe9, 0xed, 0xf2, 0xf7, 0xfb, 0x00, 0x05, 0x09, 0x0e, 0x13, 0x17,
  0x1b, 0x1e, 0x21, 0x24, 0x26, 0x27, 0x27, 0x27, 0x27, 0x26, 0x24, 0x21,
  0x1e, 0x1b, 0x17, 0x13, 0x0e, 0x09, 0x05, 0x00, 0xfb, 0xf7, 0xf2, 0xed,
  0xe9, 0xe5, 0xe2, 0xdf, 0xdc, 0xda, 0xd9, 0xd9, 0xd9, 0xd9, 0xda, 0xdc,
  0xdf, 0xe2, 0xe5, 0xe9, 0xed, 0xf2, 0xf7, 0xfb, 0x00, 0x05, 0x09, 0x0e,
  0x13, 0x17, 0x1b, 0x1e, 0x21, 0x24, 0x26, 0x27, 0x27, 0x27, 0x27, 0x26,
  0x24, 0x21, 0x1e, 0x1b, 0x17, 0x13, 0x0e, 0x09, 0x05, 0x00, 0xfb, 0xf7,
  0xf2, 0xed, 0xe9, 0xe5, 0xe2, 0xdf, 0xdc, 0xda, 0xd9, 0xd9, 0xd9, 0xd9,
  0xda, 0xdc, 0xdf, 0xe2, 0xe5, 0xe9, 0xed, 0xf2, 0xf7, 0xfb, 0x00, 0x05,
  0x09, 0x0e, 0x13, 0x17, 0x1b, 0x1e, 0x21, 0x24, 0x26, 0x27, 0x27, 0x27,
  0x27, 0x26, 0x24, 0x21, 0x1e, 0x1b, 0x17, 0x13, 0x0e, 0x09, 0x05, 0x00,
  0xfb, 0xf7, 0xf2, 0xed, 0xe9, 0xe5, 0xe2, 0xdf, 0xdc, 0xda, 0xd9, 0xd9,
  0xd9, 0xd9, 0xda, 0xdc, 0xdf, 0xe2, 0xe5, 0xe9, 0xed, 0xf2, 0xf7, 0xfb,
  0x00, 0x05, 0x09, 0x0e, 0x13, 0x17, 0x1b, 0x1e, 0x21, 0x24, 0x26, 0x27,
  0x27, 0x27, 0x27, 0x26, 0x24, 0x21, 0x1e, 0x1b, 0x17, 0x13, 0x0e, 0x09,
  0x05, 0x00, 0xfb, 0xf7, 0xf2, 0xed, 0xe9, 0xe5, 0xe2, 0xdf, 0xdc, 0xda,
  0xd9, 0xd9, 0xd9, 0xd9, 0xda, 0xdc, 0xdf, 0xe2, 0xe5, 0xe9, 0xed, 0xf2,
  0xf7, 0xfb,
};

static const struct aw86225_dts_info_s g_aw86225_dts_info =
{
  .f0_pre            = 2400,
  .f0_cali_percent   = 15,
  .cont_drv1_lvl     = 0x7f,
  .cont_drv2_lvl     = 0x36,
  .cont_brk_time     = 0x06,
  .cont_tset         = 0x06,
  .cont_bemf_set     = 0x02,
  .cont_drv_width    = 0x6a,
  .cont_wait_num     = 0x06,
  .cont_brk_gain     = 0x08,
  .cont_drv1_time    = 0x04,
  .cont_drv2_time    = 0x14,
  .cont_track_margin = 0x12,
  .sine_array[0]     = 0x05,
  .sine_array[1]     = 0xb2,
  .sine_array[2]     = 0xff,
  .sine_array[3]     = 0xff,
  .d2s_gain          = 0x05,
  .enable_auto_brk   = true,
};

static const struct motor_ops_s g_aw86225_ops =
{
  .setup       = aw86225_setup,
  .shutdown    = aw86225_shutdown,
  .stop        = aw86225_stop,
  .start       = aw86225_start,
  .params_set  = aw86225_setparam,
  .mode_set    = aw86225_setmode,
  .limits_set  = aw86225_setlimit,
  .fault_set   = aw86225_setfault,
  .state_get   = aw86225_getstate,
  .fault_get   = aw86225_getfault,
  .fault_clear = aw86225_clearfault,
  .ioctl       = aw86225_ioctl,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: aw86225_i2c_read
 *
 * Description:
 *   Read data
 *
 * Input Parameters
 *   priv - Device struct
 *   addr - Register address
 *   val  - Register value
 *   cnt  - Data number
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int aw86225_i2c_read(FAR struct aw86225_dev_s *priv, uint8_t addr,
                            FAR uint8_t *val, uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret;
  int retries;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &addr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->addr;
  msg[1].flags     = I2C_M_READ; /* Read data, from slave to master */
  msg[1].buffer    = val;
  msg[1].length    = cnt;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
      if (ret >= 0)
        {
          break;
        }
      else
        {
          mtrerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: aw86225_i2c_write
 *
 * Description:
 *   write data
 *
 * Input Parameters
 *   priv - Device struct
 *   addr - Register address
 *   val  - Register value
 *   cnt  - Data number
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int aw86225_i2c_write(FAR struct aw86225_dev_s *priv, uint8_t addr,
                             FAR const uint8_t *val, uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret;
  int retries;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->addr;
  msg[0].flags     = I2C_M_NOSTOP; /* Message not end with a STOP */
  msg[0].buffer    = &addr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->addr;
  msg[1].flags     = I2C_M_NOSTART; /* Message not begin with a START */
  msg[1].buffer    = (FAR uint8_t *)val;
  msg[1].length    = cnt;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
      if (ret >= 0)
        {
          break;
        }
      else
        {
          mtrerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: aw86225_i2c_writereg
 *
 * Description:
 *   Write 8-bit aw86225 register
 *
 * Input Parameters
 *   priv - Device struct
 *   addr - Register address
 *   val  - Register value
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int aw86225_i2c_writereg(FAR struct aw86225_dev_s *priv, uint8_t addr,
                                uint8_t val)
{
  return aw86225_i2c_write(priv, addr, &val, 1);
}

/****************************************************************************
 * Name: aw86225_i2c_readreg
 *
 * Description:
 *   read 8-bit aw86225 register
 *
 * Input Parameters
 *   priv - Device struct
 *   addr - Register address
 *   val  - Register value
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int aw86225_i2c_readreg(FAR struct aw86225_dev_s *priv, uint8_t addr,
                               FAR uint8_t *val)
{
  return aw86225_i2c_read(priv, addr, val, 1);
}

/****************************************************************************
 * Name: aw86225_i2c_write_bits
 *
 * Description:
 *   Write bits of aw86225 register
 *
 * Input Parameters
 *   priv  -Device struct
 *   addr  -Register address
 *   mask  -mask the bits to be written
 *   val   -value to be written
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int aw86225_i2c_write_bits(FAR struct aw86225_dev_s *priv,
                                  uint8_t addr, uint8_t mask, uint8_t val)
{
  uint8_t reg_val;
  int ret;

  ret = aw86225_i2c_readreg(priv, addr, &reg_val);
  if (ret < 0)
    {
      return ret;
    }

  reg_val &= mask;
  reg_val |= (val & (~mask));

  return aw86225_i2c_writereg(priv, addr, reg_val);
}

/* control functions */

/****************************************************************************
 * Name: aw86225_play_go
 *
 * Description:
 *  set gobit play or stop, need 2ms for state change
 *
 ****************************************************************************/

static int aw86225_play_go(FAR struct aw86225_dev_s *priv, bool flag)
{
  int ret;

  if (flag)
    {
      ret = aw86225_i2c_writereg(priv, AW86225_REG_PLAYCFG4,
                                AW86225_BIT_PLAYCFG4_GO_ON);
      if (ret < 0)
        {
          return ret;
        }
    }
  else
    {
      ret = aw86225_i2c_writereg(priv, AW86225_REG_PLAYCFG4,
                                AW86225_BIT_PLAYCFG4_STOP_ON);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* go bit need 2ms to change state */

  clock_t now = clock_systime_ticks();
  clock_t diff = now - priv->timestamp;
  diff = TICK2USEC(diff);
  if (diff < AW86225_GO_DELAY)
    {
      usleep(AW86225_GO_DELAY - diff + 1);
    }

  priv->timestamp = now;

  return OK;
}

/****************************************************************************
 * Name: aw86225_haptic_stop
 *
 * Description:
 *  force to goto standby mode, stop play
 *
 ****************************************************************************/

static int aw86225_haptic_stop(FAR struct aw86225_dev_s *priv)
{
  bool force_flag = true;
  uint8_t val;
  int cnt = 40;
  int ret;

  ret = aw86225_i2c_readreg(priv, AW86225_REG_GLBRD5, &val);
  if (ret < 0)
    {
      return ret;
    }

  if ((val & AW86225_BIT_GLBRD_STATE_MASK) == AW86225_BIT_STATE_STANDBY)
    {
      mtrinfo("already in standby!\n");
      return OK;
    }

  ret = aw86225_play_go(priv, false);
  if (ret < 0)
    {
      return ret;
    }

  while (cnt)
    {
      ret = aw86225_i2c_readreg(priv, AW86225_REG_GLBRD5, &val);
      if (ret < 0)
        {
          return ret;
        }

      if ((val & AW86225_BIT_GLBRD_STATE_MASK) == AW86225_BIT_STATE_STANDBY)
        {
          force_flag = false;
          mtrinfo("entered standby! glb_state=0x%02X\n", val);
          break;
        }
      else
        {
          cnt--;
          mtrinfo("wait for standby, glb_state=0x%02X\n", val);
        }

      usleep(AW86225_GO_DELAY);
    }

  if (force_flag)
    {
      mtrinfo("force to enter standby mode!\n");
      ret = aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL2,
                                  AW86225_BIT_SYSCTRL2_STANDBY_MASK,
                                  AW86225_BIT_SYSCTRL2_STANDBY_ON);
      if (ret < 0)
        {
          return ret;
        }

      return aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL2,
                                    AW86225_BIT_SYSCTRL2_STANDBY_MASK,
                                    AW86225_BIT_SYSCTRL2_STANDBY_OFF);
    }

  return OK;
}

/****************************************************************************
 * Name: aw86225_misc_para_init
 *
 * Description:
 *  common parameter setting
 *
 ****************************************************************************/

static int aw86225_misc_para_init(FAR struct aw86225_dev_s *priv)
{
  int ret;

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_TRIMCFG1,
                               AW86225_BIT_TRIMCFG1_RL_TRIM_SRC_MASK,
                               AW86225_BIT_TRIMCFG1_RL_TRIM_SRC_REG);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write(priv, AW86225_REG_SYSCTRL3,
                          g_aw86225_dts_info.sine_array, 4);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_writereg(priv, AW86225_REG_CONTCFG4,
                             g_aw86225_dts_info.cont_wait_num);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL7,
                               AW86225_BIT_SYSCTRL7_D2S_GAIN_MASK,
                               g_aw86225_dts_info.d2s_gain);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_writereg(priv, AW86225_REG_CONTCFG13,
                             (g_aw86225_dts_info.cont_tset << 4) |
                             g_aw86225_dts_info.cont_bemf_set);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_RTPCFG1,
                               AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
                               AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_EN);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_RTPCFG1,
                               AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
                               AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_EN);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_PLAYCFG3,
                               AW86225_BIT_PLAYCFG3_BRK_EN_MASK,
                               g_aw86225_dts_info.enable_auto_brk);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_ANACFG8,
                               (uint8_t)AW86225_BIT_ANACFG8_HDRV_MASK,
                               AW86225_BIT_ANACFG8_HDRV);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_writereg(priv, AW86225_REG_CONTCFG10,
                             g_aw86225_dts_info.cont_brk_time);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_CONTCFG5,
                               AW86225_BIT_CONTCFG5_BRK_GAIN_MASK,
                               g_aw86225_dts_info.cont_brk_gain);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL1,
                               (uint8_t)AW86225_BIT_SYSCTRL1_VBAT_MODE_MASK,
                               AW86225_BIT_SYSCTRL1_VBAT_MODE_HW);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL2,
                              AW86225_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
                              AW86225_BIT_SYSCTRL2_RATE_12K);
  if (ret < 0)
    {
      return ret;
    }

  return aw86225_i2c_write_bits(priv, AW86225_REG_WAVCFG13,
                                AW86225_BIT_WAVCFG13_WAITSLOT_MASK,
                                AW86225_BIT_WAVCFG13_WAITSLOT_DIV_64);
}

/****************************************************************************
 * Name: aw86225_raminit
 *
 * Description:
 *  raminit control, if set on, ramdata canbe changed
 *
 ****************************************************************************/

static int aw86225_raminit(FAR struct aw86225_dev_s *priv, bool flag)
{
  if (flag)
    {
      return aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL1,
                                    AW86225_BIT_SYSCTRL1_RAMINIT_MASK,
                                    AW86225_BIT_SYSCTRL1_RAMINIT_ON);
    }
  else
    {
      return aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL1,
                                    AW86225_BIT_SYSCTRL1_RAMINIT_MASK,
                                    AW86225_BIT_SYSCTRL1_RAMINIT_OFF);
    }
}

/****************************************************************************
 * Name: aw86225_offset_cali
 *
 * Description:
 *  offset calibration, used to remove the signal bias of HDP and HDN
 *
 ****************************************************************************/

static int aw86225_offset_cali(FAR struct aw86225_dev_s *priv)
{
  uint8_t reg_val = 0;
  int cont = 2000;
  int ret;

  ret = aw86225_raminit(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_DETCFG2,
                               AW86225_BIT_DETCFG2_DIAG_GO_MASK,
                               AW86225_BIT_DETCFG2_DIAG_GO_ON);
  if (ret < 0)
    {
      return ret;
    }

  while (1)
    {
      ret = aw86225_i2c_readreg(priv, AW86225_REG_DETCFG2, &reg_val);
      if (ret < 0)
        {
          return ret;
        }

      if ((reg_val & 0x01) == 0 || cont == 0)
        {
          break;
        }

      cont--;
    }

  ret = aw86225_raminit(priv, false);
  if (ret < 0)
    {
      return ret;
    }

  if (cont == 0)
    {
      mtrerr("calibration offset failed!\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: aw86225_set_trim_lra
 *
 * Description:
 *  reg 0x5a store the calibration data, if set 0, to read its initial f0,
 * after calibtaion, update the data
 *
 ****************************************************************************/

static int aw86225_set_trim_lra(FAR struct aw86225_dev_s *priv, uint8_t val)
{
  return aw86225_i2c_write_bits(priv, AW86225_REG_TRIMCFG3,
                                AW86225_BIT_TRIMCFG3_TRIM_LRA_MASK, val);
}

/****************************************************************************
 * Name: aw86225_set_wav_seq
 *
 * Description:
 *  set wavecfg, start from reg 0x0a, support maximum 8 slots, seq is the
 * pattern id used, if set 0 means setting end
 *
 ****************************************************************************/

static int aw86225_set_wav_seq(FAR struct aw86225_dev_s *priv, uint8_t wav,
                               uint8_t seq)
{
  return aw86225_i2c_writereg(priv, AW86225_REG_WAVCFG1 + wav, seq);
}

/****************************************************************************
 * Name: aw86225_set_wav_gain
 *
 * Description:
 *  set gain, input parameter should be above 0.0f and below 1.0f
 *
 ****************************************************************************/

static int aw86225_set_wav_gain(FAR struct aw86225_dev_s *priv, float gain)
{
  uint8_t reg = (gain * 10 * 128) / 10.f;
  return aw86225_i2c_writereg(priv, AW86225_REG_PLAYCFG2, reg);
}

/****************************************************************************
 * Name: aw86225_set_wav_loop
 *
 * Description:
 *  control the loop numbers of the waveform,
 *  loop: 0000-1110, play n+1 times, 1111, endless loop until GO bit set to 0
 *
 ****************************************************************************/

static int aw86225_set_wav_loop(FAR struct aw86225_dev_s *priv, uint8_t wav,
                                uint8_t loop)
{
  uint8_t tmp;

  if (wav % 2)
    {
      tmp = loop << 0;
      return aw86225_i2c_write_bits(priv, AW86225_REG_WAVCFG9 + (wav / 2),
                                    AW86225_BIT_WAVLOOP_EVEN_MASK, tmp);
    }
  else
    {
      tmp = loop << 4;
      return aw86225_i2c_write_bits(priv, AW86225_REG_WAVCFG9 + (wav / 2),
                                    (uint8_t)AW86225_BIT_WAVLOOP_ODD_MASK,
                                    tmp);
    }
}

/****************************************************************************
 * Name: aw86225_set_mainloop
 *
 * Description:
 *  loop: 0000-1110, play n+1 times, 1111, endless loop until GO bit set to 0
 *
 ****************************************************************************/

static int aw86225_set_mainloop(FAR struct aw86225_dev_s *priv, uint8_t loop)
{
  return aw86225_i2c_write_bits(priv, AW86225_REG_WAVCFG13,
                                AW86225_BIT_WAVCFG13_MAINLOOP_MASK, loop);
}

/****************************************************************************
 * Name: aw86225_read_lra_f0
 *
 * Description:
 *  read lra f0
 *
 ****************************************************************************/

static int aw86225_read_lra_f0(FAR struct aw86225_dev_s *priv)
{
  uint8_t val[2];
  uint32_t f0_reg = 0;
  uint32_t f0_tmp = 0;
  int ret;

  ret = aw86225_i2c_read(priv, AW86225_REG_CONTRD14, val, 2);
  if (ret < 0)
    {
      return ret;
    }

  f0_reg = (f0_reg | val[0]) << 8;
  f0_reg |= (val[1] << 0);
  if (!f0_reg)
    {
      mtrerr("didn't get lra f0 because f0_reg value is 0!\n");
      priv->data.f0 = g_aw86225_dts_info.f0_pre;
      return -EINVAL;
    }

  f0_tmp = AW86225_F0_FORMULA(f0_reg);
  priv->data.f0 = f0_tmp;
  mtrinfo("lra_f0=%" PRId32 "\n", priv->data.f0);
  return OK;
}

/****************************************************************************
 * Name: aw86225_cont_get_f0
 *
 * Description:
 *  start f0 calibraiton process and read its f0
 *
 ****************************************************************************/

static int aw86225_cont_get_f0(FAR struct aw86225_dev_s *priv)
{
  uint8_t brk_en_temp;
  uint8_t val[3];
  int cnt = 2;
  int ret;
  bool get_f0_flag = false;
  priv->data.f0 = g_aw86225_dts_info.f0_pre;

  /* set d2s_gain to max to get better performance when cat f0 */

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL7,
                               AW86225_BIT_SYSCTRL7_D2S_GAIN_MASK,
                               AW86225_BIT_SYSCTRL7_D2S_GAIN_40);
  if (ret < 0)
    {
      return ret;
    }

  /* enter standby mode */

  ret = aw86225_haptic_stop(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* f0 calibrate work mode */

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_PLAYCFG3,
                               AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK,
                               AW86225_BIT_PLAYCFG3_PLAY_MODE_CONT);
  if (ret < 0)
    {
      return ret;
    }

  /* enable f0 detect */

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_CONTCFG1,
                               AW86225_BIT_CONTCFG1_EN_F0_DET_MASK,
                               AW86225_BIT_CONTCFG1_F0_DET_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  /* cont config */

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_CONTCFG6,
                               (uint8_t)AW86225_BIT_CONTCFG6_TRACK_EN_MASK,
                               AW86225_BIT_CONTCFG6_TRACK_ENABLE);

  /* enable auto brake */

  ret = aw86225_i2c_readreg(priv, AW86225_REG_PLAYCFG3, &val[0]);
  if (ret < 0)
    {
      return ret;
    }

  brk_en_temp = AW86225_BIT_PLAYCFG3_BRK & val[0];

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_PLAYCFG3,
                               AW86225_BIT_PLAYCFG3_BRK_EN_MASK,
                               AW86225_BIT_PLAYCFG3_BRK_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  /* f0 driver level */

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_CONTCFG6,
                               AW86225_BIT_CONTCFG6_DRV1_LVL_MASK,
                               g_aw86225_dts_info.cont_drv1_lvl);
  if (ret < 0)
    {
      return ret;
    }

  val[0] = g_aw86225_dts_info.cont_drv2_lvl;
  val[1] = g_aw86225_dts_info.cont_drv1_time;
  val[2] = g_aw86225_dts_info.cont_drv2_time;

  ret = aw86225_i2c_write(priv, AW86225_REG_CONTCFG7, val, 3);
  if (ret < 0)
    {
      return ret;
    }

  /* TRACK_MARGIN */

  ret = aw86225_i2c_writereg(priv, AW86225_REG_CONTCFG11,
                             g_aw86225_dts_info.cont_track_margin);
  if (ret < 0)
    {
      return ret;
    }

  /* DRV_WIDTH */

  ret = aw86225_i2c_writereg(priv, AW86225_REG_CONTCFG3,
                             g_aw86225_dts_info.cont_drv_width);
  if (ret < 0)
    {
      return ret;
    }

  /* cont play go */

  ret = aw86225_play_go(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  usleep(AW86225_F0_DELAY);

  while (cnt)
    {
      ret = aw86225_i2c_readreg(priv, AW86225_REG_GLBRD5, &val[0]);
      if (ret < 0)
        {
          return ret;
        }

      if ((val[0] & AW86225_BIT_GLBRD_STATE_MASK) ==
          AW86225_BIT_STATE_STANDBY)
        {
          mtrinfo("entered standby! glb_state=0x%02X\n", val[0]);
          cnt = 0;
          get_f0_flag = true;
        }
      else
        {
          cnt--;
          mtrinfo("waitting for standby,glb_state=0x%02X\n", val[0]);
        }

      usleep(AW86225_F0_DELAY);
    }

  if (get_f0_flag)
    {
      ret = aw86225_read_lra_f0(priv);
      if (ret < 0)
        {
          return ret;
        }
    }
  else
    {
      mtrerr("enter standby mode failed, stop reading f0!\n");
      return -EINVAL;
    }

  /* disable f0 detect */

  ret = aw86225_i2c_write_bits(priv, AW86225_REG_CONTCFG1,
                               AW86225_BIT_CONTCFG1_EN_F0_DET_MASK,
                               AW86225_BIT_CONTCFG1_F0_DET_DISABLE);

  /* recover auto break config */

  ret += aw86225_i2c_write_bits(priv, AW86225_REG_PLAYCFG3,
                                AW86225_BIT_PLAYCFG3_BRK_EN_MASK,
                                brk_en_temp);

  /* set d2s_gain to default when cat f0 is finished */

  ret += aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL7,
                                AW86225_BIT_SYSCTRL7_D2S_GAIN_MASK,
                                g_aw86225_dts_info.d2s_gain);
  if (ret < 0)
    {
      mtrerr("restore config fail\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: aw86225_calculate_cali_data
 *
 * Description:
 *  f0 calibration algorithm, to get calibration data
 *
 ****************************************************************************/

static int aw86225_calculate_cali_data(FAR struct aw86225_dev_s *priv)
{
  char f0_cali_lra;
  int f0_cali_step;

  f0_cali_step = 100000 *
                 ((int)priv->data.f0 - (int)g_aw86225_dts_info.f0_pre) /
                 ((int)priv->data.f0 * AW86225_F0_CALI_ACCURACY);

  if (f0_cali_step >= 0) /* f0_cali_step >= 0 */
    {
      if (f0_cali_step % 10 >= 5)
        {
          f0_cali_step = 32 + (f0_cali_step / 10 + 1);
        }
      else
        {
          f0_cali_step = 32 + f0_cali_step / 10;
        }
    }
  else /* f0_cali_step < 0 */
    {
      if (f0_cali_step % 10 <= -5)
        {
          f0_cali_step = 32 + (f0_cali_step / 10 - 1);
        }
      else
        {
          f0_cali_step = 32 + f0_cali_step / 10;
        }
    }

  if (f0_cali_step > 31)
    {
      f0_cali_lra = (char)f0_cali_step - 32;
    }
  else
    {
      f0_cali_lra = (char)f0_cali_step + 32;
    }

  /* update cali step */

  priv->data.f0_cali_data = (int)f0_cali_lra;

  mtrinfo("f0_cali_data=0x%02X\n", priv->data.f0_cali_data);

  return priv->data.f0_cali_data;
}

/****************************************************************************
 * Name: aw86225_set_base_addr
 *
 * Description:
 *  set base address
 *
 ****************************************************************************/

static int aw86225_set_base_addr(FAR struct aw86225_dev_s *priv)
{
  uint8_t val;
  int ret;

  val = (uint8_t)AW86225_SET_BASEADDR_H(AW86225_BASERAM_ADDR);
  ret = aw86225_i2c_write_bits(priv, AW86225_REG_RTPCFG1,
                               AW86225_BIT_RTPCFG1_ADDRH_MASK, val);
  if (ret < 0)
    {
      return ret;
    }

  val = (uint8_t)AW86225_SET_BASEADDR_L(AW86225_BASERAM_ADDR);
  return aw86225_i2c_writereg(priv, AW86225_REG_RTPCFG2, val);
}

/****************************************************************************
 * Name: aw86225_set_ram_addr
 *
 * Description:
 *  set ramdata address
 *
 ****************************************************************************/

static int aw86225_set_ram_addr(FAR struct aw86225_dev_s *priv)
{
  uint8_t val[2];

  val[0] = (uint8_t)AW86225_SET_RAMADDR_H(AW86225_BASERAM_ADDR);
  val[1] = (uint8_t)AW86225_SET_RAMADDR_L(AW86225_BASERAM_ADDR);
  return aw86225_i2c_write(priv, AW86225_REG_RAMADDRH, val, 2);
}

/****************************************************************************
 * Name: aw86225_set_ram_data
 *
 * Description:
 *  update ramdata
 *
 ****************************************************************************/

static int aw86225_set_ram_data(FAR struct aw86225_dev_s *priv)
{
  int ramlen = sizeof(g_aw86225_ram_data) - AW86225_RAMDATA_SHIFT;

  if (ramlen >  AW86225_RAMDATA_MAX_SIZE)
    {
      mtrerr("ramdata size too big\n");
      return -EINVAL;
    }

  return aw86225_i2c_write(priv, AW86225_REG_RAMDATA,
                           &g_aw86225_ram_data[AW86225_RAMDATA_SHIFT],
                           ramlen);
}

/****************************************************************************
 * Name: aw86225_get_lra_resistance
 *
 * Description:
 *  detect lra resistance, used for diagnose
 *
 ****************************************************************************/

static uint32_t aw86225_get_lra_resistance(FAR struct aw86225_dev_s *priv)
{
  uint8_t reg_val;
  uint8_t d2s_gain_temp;
  uint32_t lra;
  uint32_t lra_code = 0;

  aw86225_haptic_stop(priv);
  aw86225_i2c_readreg(priv, AW86225_REG_SYSCTRL7, &reg_val);
  d2s_gain_temp = AW86225_BIT_SYSCTRL7_GAIN & reg_val;
  aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL7,
                         AW86225_BIT_SYSCTRL7_D2S_GAIN_MASK,
                         g_aw86225_dts_info.d2s_gain);
  aw86225_raminit(priv, true);
  aw86225_haptic_stop(priv);
  aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL2,
                         AW86225_BIT_SYSCTRL2_STANDBY_MASK,
                         AW86225_BIT_SYSCTRL2_STANDBY_OFF);
  aw86225_i2c_write_bits(priv, AW86225_REG_DETCFG1,
                         AW86225_BIT_DETCFG1_RL_OS_MASK,
                         AW86225_BIT_DETCFG1_RL);
  aw86225_i2c_write_bits(priv, AW86225_REG_DETCFG2,
                         AW86225_BIT_DETCFG2_DIAG_GO_MASK,
                         AW86225_BIT_DETCFG2_DIAG_GO_ON);
  usleep(AW86225_RL_DELAY);
  aw86225_i2c_readreg(priv, AW86225_REG_DET_RL, &reg_val);
  lra_code = (lra_code | reg_val) << 2;
  aw86225_i2c_readreg(priv, AW86225_REG_DET_LO, &reg_val);
  lra_code = lra_code | (reg_val & AW86225_BIT_DET_LO_RL);
  lra = AW86225_RL_FORMULA(lra_code);
  aw86225_raminit(priv, false);
  aw86225_i2c_write_bits(priv, AW86225_REG_SYSCTRL7,
                         AW86225_BIT_SYSCTRL7_D2S_GAIN_MASK, d2s_gain_temp);
  mtrinfo("lra resistance= %" PRId32 " mohm\n", lra);

  return lra;
}

/****************************************************************************
 * Name: aw86225_set_calib_param
 *
 * Description:
 *  update clibration data
 *
 ****************************************************************************/

static int aw86225_set_calib_param(FAR struct aw86225_dev_s *priv,
                                   unsigned long arg)
{
  FAR char *calibdata = (FAR char *)arg;
  uint8_t calib_finish;
  uint8_t calib_f0;

  DEBUGASSERT(priv != NULL && calibdata != NULL);

  calib_finish = atoi(&calibdata[0]);
  calib_f0 = atoi(&calibdata[4]);

  mtrinfo("finish = %d, calib_f0 = 0x%x, \n", calib_finish, calib_f0);

  if (calib_finish == 1)
    {
      aw86225_set_trim_lra(priv, calib_f0);
    }

  return OK;
}

/****************************************************************************
 * Name: aw86225_cali_judge
 *
 * Description:
 *  judge if the calibrated  f0 is in right range
 *
 ****************************************************************************/

static int aw86225_cali_judge(FAR struct aw86225_dev_s *priv)
{
  uint32_t f0_cali_min = 0;
  uint32_t f0_cali_max = 0;

  f0_cali_min = g_aw86225_dts_info.f0_pre *
                (100 - g_aw86225_dts_info.f0_cali_percent) / 100;
  f0_cali_max = g_aw86225_dts_info.f0_pre *
                (100 + g_aw86225_dts_info.f0_cali_percent) / 100;

  if ((priv->data.f0 < f0_cali_min) || (priv->data.f0 > f0_cali_max))
    {
      mtrerr("f0 out of range, lra f0 = %d!\n", priv->data.f0);
      return -EAGAIN;
    }

  return OK;
}

/****************************************************************************
 * Name: aw86225_calibration
 *
 * Description:
 *  start calibration process and get its result
 *
 ****************************************************************************/

static int aw86225_calibration(FAR struct aw86225_dev_s *priv,
                               unsigned long arg)
{
  int ret = 0;
  FAR char *calibdata = (FAR char *)arg;

  ret = aw86225_set_trim_lra(priv, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_cont_get_f0(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* if f0 not in limited range, fail */

  ret = aw86225_cali_judge(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* calculate cali step */

  priv->data.f0_cali_data = aw86225_calculate_cali_data(priv);
  priv->data.calib_finish = 1;

  /* update cali data and go to standby mode */

  aw86225_set_trim_lra(priv, priv->data.f0_cali_data);
  sprintf(calibdata, "%03d,%03d\n", priv->data.calib_finish,
          priv->data.f0_cali_data);
  aw86225_haptic_stop(priv);

  return OK;
}

/****************************************************************************
 * Name: aw86225_read_chipid
 *
 * Description:
 *  check chip id
 *
 ****************************************************************************/

static int aw86225_read_chipid(FAR struct aw86225_dev_s *priv)
{
  int ret;
  uint8_t devid = 1; /* set nonzero, because chipid is 0! */

  ret = aw86225_i2c_readreg(priv, 0x00, &devid);
  if (ret < 0 || devid != 0)
    {
      mtrerr("Wrong Device ID! %02x\n\n", devid);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: aw86225_haptic_init
 *
 * Description:
 *  haptic initialize
 *
 ****************************************************************************/

static int aw86225_haptic_init(FAR struct aw86225_dev_s *priv)
{
  int ret;
  ret = aw86225_haptic_stop(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_misc_para_init(priv);
  if (ret < 0)
    {
      return ret;
    }

  return aw86225_offset_cali(priv);
}

/****************************************************************************
 * Name: aw86225_ram_update
 *
 * Description:
 *  write ramdata into ram
 *
 ****************************************************************************/

static int aw86225_ram_update(FAR struct aw86225_dev_s *priv)
{
  int ret;

  ret = aw86225_haptic_stop(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_raminit(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_set_base_addr(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_set_ram_addr(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret =  aw86225_set_ram_data(priv);
  if (ret < 0)
    {
      return ret;
    }

  return aw86225_raminit(priv, false);
}

/****************************************************************************
 * Name: aw86225_continus_work
 *
 * Description:
 *  set sinewave play until go bit set to 0
 *
 ****************************************************************************/

static int aw86225_continus_work(FAR struct aw86225_dev_s *priv, float gain)
{
  int ret = OK;

  aw86225_haptic_stop(priv);
  ret = aw86225_set_wav_seq(priv, 0, AW86225_SINEWAVE_ID);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_set_wav_loop(priv, 0, AW86225_BIT_WAVLOOP_INIFINITELY);
  if (ret < 0)
    {
      return ret;
    }

  ret = aw86225_set_wav_seq(priv, 1, 0);
  if (ret < 0)
    {
      return ret;
    }

  return aw86225_set_wav_gain(priv, gain);
}

/****************************************************************************
 * Name: aw86225_excute_pattern
 *
 * Description:
 *  set sinewave play until go bit set to 0
 *
 ****************************************************************************/

static int aw86225_excute_pattern(FAR struct aw86225_dev_s *priv,
                                  FAR struct aw86225_pattern_s *pattern)
{
  int ret = OK;

  for (uint8_t i = 0; i < 8; i++)
    {
      mtrinfo("i %d , waveseq %0x, waveloop %d\n", i,
              pattern->patternid[i],
              pattern->waveloop[i]);

      ret = aw86225_set_wav_seq(priv, i, pattern->patternid[i]);
      if (ret < 0)
        {
          return ret;
        }

      ret = aw86225_set_wav_loop(priv, i, pattern->waveloop[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = aw86225_set_mainloop(priv, pattern->mainloop);
  if (ret < 0)
    {
      return ret;
    }

  return aw86225_set_wav_gain(priv, pattern->strength);
}

/****************************************************************************
 * Name: worker_cb
 *
 * Description:
 *  start a pattern and call more patterns
 *
 ****************************************************************************/

static void worker_cb(FAR void *arg)
{
  int ret = OK;
  FAR struct aw86225_dev_s *priv = (FAR void *)arg;
  priv->pattern_index++;
  if (priv->pattern_index == priv->patterns->count)
    {
      if (priv->patterns->repeatable)
        {
          priv->pattern_index = 0;
        }
      else
        {
          priv->pattern_play_cnt++;
          if (priv->pattern_play_cnt >= 50)
            {
              syslog(LOG_WARNING, "---aw86225 short pattern play reached 50 times---\n");
              priv->pattern_play_cnt = 0;
            }

          priv->state = MOTOR_STATE_IDLE ;
        }
    }

  /* if state idle, exit worker cb */
  /* 1. set idle state when pattern play finish */
  /* 2. set idle state via stop cmd */

  if (priv->state == MOTOR_STATE_IDLE)
    {
      aw86225_play_go(priv, false);
      return;
    }

  mtrinfo("pattern %d duration %ld\n", priv->pattern_index,
           priv->patterns->pattern[priv->pattern_index].duration);

  ret = aw86225_excute_pattern(priv,
                         &priv->patterns->pattern[priv->pattern_index]);
  if (ret < 0)
    {
      mtrerr("pattern index %d set fail \n", priv->pattern_index);
      return;
    }

  ret = aw86225_play_go(priv, true);
  if (ret < 0)
    {
      mtrerr("pattern index %d play fail \n", priv->pattern_index);
      return;
    }

  wd_start(&priv->wdog,
           MSEC2TICK(priv->patterns->pattern[priv->pattern_index].duration),
           wdog_timer_cb, (wdparm_t)priv);
}

/****************************************************************************
 * Name: wdog_timer_cb
 *
 * Description:
 *  start a workqueue
 *
 ****************************************************************************/

static void wdog_timer_cb(wdparm_t arg)
{
  FAR struct aw86225_dev_s *priv = (FAR void *)arg;
  work_queue(HPWORK, &priv->worker, worker_cb, priv, 0);
}

/****************************************************************************
 * Name: aw86225_excute_patterns
 *
 * Description:
 *  start a patten play and start a timer for more pattens
 *
 ****************************************************************************/

static int aw86225_excute_patterns(FAR struct aw86225_dev_s *priv,
                                   FAR struct aw86225_patterns_s
                                   *patterns)
{
  int ret = OK;
  DEBUGASSERT(priv != NULL && patterns != NULL);
  priv->pattern_index = 0;
  priv->patterns = patterns;

  /* if pattern play duration is longer than 1 second, print the log */

  if (patterns->pattern[0].duration > 1000 )
    {
      syslog(LOG_WARNING, "---aw86225 pattern duration %ld---\n",
             patterns->pattern[0].duration);
    }

  ret = aw86225_excute_pattern(priv, &patterns->pattern[0]);
  if (ret < 0)
    {
      return ret;
    }

  if (patterns->repeatable && patterns->count == 1)
    {
      /* let aw86225 handle same repeatable pattern */

      ret = aw86225_set_mainloop(priv, 0xf);
      if (ret < 0)
        {
          return ret;
        }
    }
  else
    {
      /* start wdtimer to loop all patterns */

      mtrinfo("start wdtimer.\n");
      wd_start(&priv->wdog, MSEC2TICK(patterns->pattern[0].duration),
               wdog_timer_cb, (wdparm_t)priv);
    }

  return ret;
}

/****************************************************************************
 * Name: aw86225_reg_dump
 *
 * Description:
 *  used for debug, dump all register values
 *
 ****************************************************************************/

static int aw86225_reg_dump(FAR struct aw86225_dev_s *priv,
                            uint8_t startaddr, uint8_t endaddr)
{
  FAR uint8_t *reg_buf;

  if (endaddr <= startaddr)
    {
      return -EINVAL;
    }

  reg_buf = kmm_zalloc(endaddr - startaddr + 1);
  if (NULL == reg_buf)
    {
      return -ENOMEM;
    }

  for (uint8_t i = 0; i <= (endaddr - startaddr); i++)
    {
      aw86225_i2c_readreg(priv, (startaddr + i), &reg_buf[i]);
      mtrinfo("[0x%2x]=0x%2x\n", (startaddr + i), reg_buf[i]);
    }

  kmm_free(reg_buf);

  return OK;
}

/* motor ops functions */

/****************************************************************************
 * Name: aw86225_setup
 *
 * Description:
 *   aw86225 initilize, power on(vsys for power, no enable pin need set)
 *   ram data set, common reg set
 *
 ****************************************************************************/

static int aw86225_setup(FAR struct motor_lowerhalf_s *dev)
{
  int ret;
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  ret = aw86225_haptic_init(priv);
  if (ret < 0)
    {
      return ret;
    }

  return aw86225_ram_update(priv);
}

/****************************************************************************
 * Name: aw86225_shutdown
 *
 * Description:
 *   aw86225 shutdown, power off(vsys for power, nothing to do)
 *
 ****************************************************************************/

static int aw86225_shutdown(FAR struct motor_lowerhalf_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: aw86225_stop
 *
 * Description:
 *   stop motor running
 *
 ****************************************************************************/

static int aw86225_stop(FAR struct motor_lowerhalf_s *dev)
{
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;
  int ret;

  DEBUGASSERT(dev != NULL);
  ret = aw86225_haptic_stop(priv);
  if (ret == 0)
    {
      priv->state = MOTOR_STATE_IDLE ;
    }
  else
    {
      mtrerr("motor stop fail\n");
    }

   wd_cancel(&priv->wdog);
   work_cancel(HPWORK, &priv->worker);

  return ret;
}

/****************************************************************************
 * Name: aw86225_start
 *
 * Description:
 *   start motor running
 *
 ****************************************************************************/

static int aw86225_start(FAR struct motor_lowerhalf_s *dev)
{
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;
  int ret;

  DEBUGASSERT(dev != NULL);

  if (priv->patterns_reload && priv->patterns)
    {
      /* start excution of patterns before set go bit. */

      ret = aw86225_excute_patterns(priv, priv->patterns);
      if (ret < 0)
        {
          merr("failed to excute patterns\n");
          return ret;
        }
      priv->patterns_reload = false;
    }

  ret = aw86225_play_go(priv, true);
  if (ret == 0)
    {
      priv->state = MOTOR_STATE_RUN ;
    }

  return ret;
}

/****************************************************************************
 * Name: aw86225_setparam
 *
 * Description:
 *   set aw86225 used parameters
 *
 ****************************************************************************/

static int aw86225_setparam(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_params_s *param)
{
  int ret = 0;
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;
  FAR struct aw86225_patterns_s *patterns = NULL;

  DEBUGASSERT(dev != NULL && param != NULL);

  priv->pattern_index = 0;
  priv->patterns_reload = priv->lower.opmode == MOTOR_OPMODE_PATTERN;
  if (priv->patterns_reload)
    {
      /* any param changed in PATTERN mode requires setting up aw86225 again */

      patterns = (FAR struct aw86225_patterns_s *)param->privdata;
    }
  priv->patterns = patterns;

  if (priv->lower.opmode == MOTOR_OPMODE_FORCE)
    {
      ret = aw86225_continus_work(priv, param->force);
    }

  return ret;
}

/****************************************************************************
 * Name: aw86225_setmode
 *
 * Description:
 *   force mode and pattern mode, both use rammode play
 *
 ****************************************************************************/

static int aw86225_setmode(FAR struct motor_lowerhalf_s *dev, uint8_t mode)
{
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  if ((mode != MOTOR_OPMODE_FORCE) && (mode != MOTOR_OPMODE_PATTERN))
    {
      mtrerr("Unsupported play mode: %d!\n", mode);
      return -EINVAL;
    }

  priv->lower.opmode = mode;

  return aw86225_i2c_write_bits(priv, AW86225_REG_PLAYCFG3,
                                AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK,
                                AW86225_BIT_PLAYCFG3_PLAY_MODE_RAM);
}

/****************************************************************************
 * Name: aw86225_setlimit
 *
 * Description:
 *   set aw86225 limit value
 *
 ****************************************************************************/

static int aw86225_setlimit(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_limits_s *limits)
{
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;

  DEBUGASSERT(dev != NULL && limits != NULL);

  /* Set limit */

  priv->lower.limits.force = limits->force;

  /* Lock limits */

  priv->lower.limits.lock = true;

  return OK;
}

/****************************************************************************
 * Name: aw86225_setfault
 *
 * Description:
 *   not used now, just keep the interface
 *
 ****************************************************************************/

static int aw86225_setfault(FAR struct motor_lowerhalf_s *dev, uint8_t fault)
{
  return OK;
}

/****************************************************************************
 * Name: aw86225_getstate
 *
 * Description:
 *   not used now, just keep the interface
 *
 ****************************************************************************/

static int aw86225_getstate(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_state_s *state)
{
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;
  FAR uint8_t *motor_state = (FAR uint8_t *)state;

  DEBUGASSERT(dev != NULL && state != NULL);

  *motor_state = priv->state;

  return OK;
}

/****************************************************************************
 * Name: aw86225_getfault
 *
 * Description:
 *   motor chip self diagnostic
 *
 ****************************************************************************/

static int aw86225_getfault(FAR struct motor_lowerhalf_s *dev,
                            FAR uint8_t *fault)
{
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;
  uint32_t lra = 0;
  uint8_t reg_val = 0;

  DEBUGASSERT(dev != NULL && fault != NULL);

  lra = aw86225_get_lra_resistance(priv);
  if (lra > AW86225_RL_MAX)
    {
      mtrerr("open circuit detected!!\n");
      *fault = -MOTOR_FAULT_OTHER;
    }

  aw86225_i2c_readreg(priv, AW86225_REG_SYSINT, &reg_val);
  if (reg_val & AW86225_BIT_SYSINT_UVLI)
    {
      mtrerr("chip uvlo error!!\n");
      *fault = -MOTOR_FAULT_OTHER;
    }

  if (reg_val & AW86225_BIT_SYSINT_OCDI)
    {
      mtrerr("chip over current error!!\n");
      *fault = -MOTOR_FAULT_OVERCURRENT;
    }

  if (reg_val & AW86225_BIT_SYSINT_OTI)
    {
      mtrerr("chip over temperature error!!\n");
      *fault = -MOTOR_FAULT_OVERTEMP;
    }

  return OK;
}

/****************************************************************************
 * Name: aw86225_clearfault
 *
 * Description:
 *   not used now, just keep the interface
 *
 ****************************************************************************/

static int aw86225_clearfault(FAR struct motor_lowerhalf_s *dev,
                              uint8_t fault)
{
  return OK;
}

/****************************************************************************
 * Name: aw86225_ioctl
 *
 * Description:
 *   ioctrl function used in aw86225
 *
 ****************************************************************************/

static int aw86225_ioctl(FAR struct motor_lowerhalf_s *dev, int cmd,
                         unsigned long arg)
{
  int ret;
  FAR struct aw86225_dev_s *priv = (FAR struct aw86225_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  switch (cmd)
    {
      case MTRIOC_SELFTEST:
        {
          ret = aw86225_reg_dump(priv, 0x00, 0x79);
          break;
        }

      case MTRIOC_SET_CALIBDATA:
        {
          ret = aw86225_set_calib_param(priv, arg);
          break;
        }

      case MTRIOC_CALIBRATE:
        {
          ret = aw86225_calibration(priv, arg);
          break;
        }

      default:
        {
          mtrerr("undefined ioctrl cmd: %d\n", cmd);
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aw86225_register
 *
 * Description:
 *   Register the aw86225 character device as 'devpath'
 *
 * Input Parameters:
 *   devname - The name of driver register. E.g., "/dev/lra0"
 *   config  - the board config of aw86225
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int aw86225_register(FAR const char *devname,
                     FAR const struct aw86225_config_s *config)
{
  int ret;
  FAR struct aw86225_dev_s *priv;

  DEBUGASSERT(devname != NULL && config != NULL);

  /* Initialize the aw86225 device structure */

  priv = kmm_zalloc(sizeof(struct aw86225_dev_s));
  if (NULL == priv)
    {
      mtrerr("Failed to allocate instance\n\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->lower.ops = &g_aw86225_ops;
  priv->state = MOTOR_OPMODE_INIT;
  priv->pattern_play_cnt = 0;

  /* Check Device ID */

  ret = aw86225_read_chipid(priv);
  if (ret < 0)
    {
      mtrerr("Check chip id failed!\n\n");
      return ret;
    }

  ret = motor_register(devname, &priv->lower);
  if (ret < 0)
    {
      mtrerr("Failed to register driver:%d\n", ret);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}

