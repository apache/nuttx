/****************************************************************************
 * drivers/input/aw86225.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>
#include <nuttx/nuttx.h>
#include <nuttx/timers/timer.h>
#include <nuttx/timers/watchdog.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/input/aw86225.h>
#include <nuttx/input/ff.h>
#include <nuttx/irq.h>
#include <nuttx/lib/lib.h>

#include "aw86225_reg.h"
#include "aw86225_internal.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void aw86225_long_vibrate_work_routine(FAR void *arg);
static void aw86225_ram_work_routine(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const char *g_aw86225_ram_name = "aw86225_haptic.bin";
#ifdef CONFIG_AW86225_RTP_FILE_SUPPORT
static FAR const char g_aw86225_rtp_name[][AW86225_RTP_NAME_MAX] =
{
  {"door_open_RTP.bin"},
  {"Guitar_RTP.bin"},
  {"aw86225_osc_rtp_24K_5s.bin"},
  {"AcousticGuitar_RTP.bin"},
  {"Carousel_RTP.bin"},
  {"Celesta_RTP.bin"},
  {"Childhood_RTP.bin"},
  {"Country_RTP.bin"},
  {"Cowboy_RTP.bin"},
  {"Echo_RTP.bin"},
  {"Fairyland_RTP.bin"},
  {"Fantasy_RTP.bin"},
  {"Field_Trip_RTP.bin"},
  {"Glee_RTP.bin"},
  {"Glockenspiel_RTP.bin"},
  {"Ice_Latte_RTP.bin"},
  {"Kung_Fu_RTP.bin"},
  {"Leisure_RTP.bin"},
  {"Lollipop_RTP.bin"},
  {"MiMix2_RTP.bin"},
  {"Mi_RTP.bin"},
  {"MiHouse_RTP.bin"},
  {"MiJazz_RTP.bin"},
  {"MiRemix_RTP.bin"},
  {"Mountain_Spring_RTP.bin"},
  {"Orange_RTP.bin"},
  {"WindChime_RTP.bin"},
  {"Space_Age_RTP.bin"},
  {"ToyRobot_RTP.bin"},
  {"Vigor_RTP.bin"},
  {"Bottle_RTP.bin"},
  {"Bubble_RTP.bin"},
  {"Bullfrog_RTP.bin"},
  {"Burst_RTP.bin"},
  {"Chirp_RTP.bin"},
  {"Clank_RTP.bin"},
  {"Crystal_RTP.bin"},
  {"FadeIn_RTP.bin"},
  {"FadeOut_RTP.bin"},
  {"Flute_RTP.bin"},
  {"Fresh_RTP.bin"},
  {"Frog_RTP.bin"},
  {"Guitar_RTP.bin"},
  {"Harp_RTP.bin"},
  {"IncomingMessage_RTP.bin"},
  {"MessageSent_RTP.bin"},
  {"Moment_RTP.bin"},
  {"NotificationXylophone_RTP.bin"},
  {"Potion_RTP.bin"},
  {"Radar_RTP.bin"},
  {"Spring_RTP.bin"},
  {"Swoosh_RTP.bin"},
  {"Gesture_UpSlide_RTP.bin"},
  {"Gesture_UpHold_RTP.bin"},
  {"Charge_Wire_RTP.bin"},
  {"Charge_Wireless_RTP.bin"},
  {"Unlock_Failed_RTP.bin"},
  {"FOD_Motion1_RTP.bin"},
  {"FOD_Motion2_RTP.bin"},
  {"FOD_Motion3_RTP.bin"},
  {"FOD_Motion4_RTP.bin"},
  {"FaceID_Wrong1_RTP.bin"},
  {"FaceID_Wrong2_RTP.bin"},
  {"uninstall_animation_rtp.bin"},
  {"uninstall_dialog_rtp.bin"},
  {"screenshot_rtp.bin"},
  {"lockscreen_camera_entry_rtp.bin"},
  {"launcher_edit_rtp.bin"},
  {"launcher_icon_selection_rtp.bin"},
  {"taskcard_remove_rtp.bin"},
  {"task_cleanall_rtp.bin"},
  {"new_iconfolder_rtp.bin"},
  {"notification_remove_rtp.bin"},
  {"notification_cleanall_rtp.bin"},
  {"notification_setting_rtp.bin"},
  {"game_turbo_rtp.bin"},
  {"NFC_card_rtp.bin"},
  {"wakeup_voice_assistant_rtp.bin"},
  {"NFC_card_slow_rtp.bin"},
  {"POCO_RTP.bin"},
  {"aw86225_rtp.bin"},
  {"offline_countdown_RTP.bin"},
  {"scene_bomb_injury_RTP.bin"},
  {"scene_bomb_RTP.bin"},
  {"door_open_RTP.bin"},
  {"aw86225_rtp.bin"},
  {"scene_step_RTP.bin"},
  {"crawl_RTP.bin"},
  {"scope_on_RTP.bin"},
  {"scope_off_RTP.bin"},
  {"magazine_quick_RTP.bin"},
  {"grenade_RTP.bin"},
  {"scene_getshot_RTP.bin"},
  {"grenade_explosion_RTP.bin"},
  {"punch_RTP.bin"},
  {"pan_RTP.bin"},
  {"bandage_RTP.bin"},
  {"aw86225_rtp.bin"},
  {"scene_jump_RTP.bin"},
  {"vehicle_plane_RTP.bin"},
  {"scene_openparachute_RTP.bin"},
  {"scene_closeparachute_RTP.bin"},
  {"vehicle_collision_RTP.bin"},
  {"vehicle_buggy_RTP.bin"},
  {"vehicle_dacia_RTP.bin"},
  {"vehicle_moto_RTP.bin"},
  {"firearms_akm_RTP.bin"},
  {"firearms_m16a4_RTP.bin"},
  {"aw86225_rtp.bin"},
  {"firearms_awm_RTP.bin"},
  {"firearms_mini14_RTP.bin"},
  {"firearms_vss_RTP.bin"},
  {"firearms_qbz_RTP.bin"},
  {"firearms_ump9_RTP.bin"},
  {"firearms_dp28_RTP.bin"},
  {"firearms_s1897_RTP.bin"},
  {"aw86225_rtp.bin"},
  {"firearms_p18c_RTP.bin"},
  {"aw86225_rtp.bin"},
  {"aw86225_rtp.bin"},
  {"CFM_KillOne_RTP.bin"},
  {"CFM_Headshot_RTP.bin"},
  {"CFM_MultiKill_RTP.bin"},
  {"CFM_KillOne_Strong_RTP.bin"},
  {"CFM_Headshot_Strong_RTP.bin"},
  {"CFM_MultiKill_Strong_RTP.bin"},
  {"CFM_Weapon_Grenade_Explode_RTP.bin"},
  {"CFM_Weapon_Grenade_KillOne_RTP.bin"},
  {"CFM_ImpactFlesh_Normal_RTP.bin"},
  {"CFM_Weapon_C4_Installed_RTP.bin"},
  {"CFM_Hero_Appear_RTP.bin"},
  {"CFM_UI_Reward_OpenBox_RTP.bin"},
  {"CFM_UI_Reward_Task_RTP.bin"},
  {"CFM_Weapon_BLT_Shoot_RTP.bin"},
  {"Atlantis_RTP.bin"},
  {"DigitalUniverse_RTP.bin"},
  {"Reveries_RTP.bin"},
  {"FOD_Motion_Triang_RTP.bin"},
  {"FOD_Motion_Flare_RTP.bin"},
  {"FOD_Motion_Ripple_RTP.bin"},
  {"FOD_Motion_Spiral_RTP.bin"},
  {"gamebox_launch_rtp.bin"},
  {"Gesture_Back_Pull_RTP.bin"},
  {"Gesture_Back_Release_RTP.bin"},
  {"alert_rtp.bin"},
  {"feedback_negative_light_rtp.bin"},
  {"feedback_neutral_rtp.bin"},
  {"feedback_positive_rtp.bin"},
  {"fingerprint_record_rtp.bin"},
  {"lockdown_rtp.bin"},
  {"sliding_damping_rtp.bin"},
  {"todo_alldone_rtp.bin"},
  {"uninstall_animation_icon_rtp.bin"},
  {"signal_button_highlight_rtp.bin"},
  {"signal_button_negative_rtp.bin"},
  {"signal_button_rtp.bin"},
  {"signal_clock_high_rtp.bin"},
  {"signal_clock_rtp.bin"},
  {"signal_clock_unit_rtp.bin"},
  {"signal_inputbox_rtp.bin"},
  {"signal_key_high_rtp.bin"},
  {"signal_key_unit_rtp.bin"},
  {"signal_list_highlight_rtp.bin"},
  {"signal_list_rtp.bin"},
  {"signal_picker_rtp.bin"},
  {"signal_popup_rtp.bin"},
  {"signal_seekbar_rtp.bin"},
  {"signal_switch_rtp.bin"},
  {"signal_tab_rtp.bin"},
  {"signal_text_rtp.bin"},
  {"signal_transition_light_rtp.bin"},
  {"signal_transition_rtp.bin"},
  {"haptics_video_rtp.bin"},
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int aw86225_i2c_write_cnt(FAR struct aw86225 *aw86225,
                                 uint8_t reg_addr,
                                 FAR const uint8_t *reg_data, uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret = -1;
  int retries = 0;

  msg[0].frequency = aw86225->freq;
  msg[0].addr      = aw86225->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;

  msg[1].frequency = aw86225->freq;
  msg[1].addr      = aw86225->addr;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (uint8_t *)reg_data;
  msg[1].length    = cnt;

  while (retries < AW86225_I2C_RETRIES)
    {
      ret = I2C_TRANSFER(aw86225->i2c, msg, 2);
      if (ret < 0)
        {
          ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          break;
        }

      retries++;
    }

  return ret;
}

static int aw86225_i2c_write(FAR struct aw86225 *aw86225,
                             uint8_t reg_addr, uint8_t reg_data)
{
  return aw86225_i2c_write_cnt(aw86225, reg_addr, &reg_data, 1);
}

static int aw86225_i2c_read_cnt(FAR struct aw86225 *aw86225,
                                uint8_t reg_addr,
                                FAR const uint8_t *reg_data,
                                uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret = -1;
  int retries = 0;

  msg[0].frequency = aw86225->freq;
  msg[0].addr      = aw86225->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;

  msg[1].frequency = aw86225->freq;
  msg[1].addr      = aw86225->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = (uint8_t *)reg_data;
  msg[1].length    = cnt;

  while (retries < AW86225_I2C_RETRIES)
    {
      ret = I2C_TRANSFER(aw86225->i2c, msg, 2);
      if (ret < 0)
        {
          ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          break;
        }

      retries++;
    }

  return ret;
}

static int aw86225_i2c_read(FAR struct aw86225 *aw86225, uint8_t reg_addr,
                            FAR uint8_t *reg_data)
{
  return aw86225_i2c_read_cnt(aw86225, reg_addr, reg_data, 1);
}

#ifdef CONFIG_AW86225_RTP_FILE_SUPPORT
static int aw86225_i2c_writes(FAR struct aw86225 *aw86225,
                              unsigned char reg_addr,
                              FAR unsigned char *buf,
                              unsigned int len)
{
  return aw86225_i2c_write_cnt(aw86225, reg_addr, buf, len);
}
#endif

static int aw86225_i2c_write_bits(FAR struct aw86225 *aw86225,
                                  unsigned char reg_addr, unsigned int mask,
                                  unsigned char reg_data)
{
  int ret;
  uint8_t reg_val = 0;

  ret = aw86225_i2c_read(aw86225, reg_addr, &reg_val);
  if (ret < 0)
    {
      ierr("i2c read error, ret: %d\n", ret);
      return ret;
    }

  reg_val &= mask;
  reg_val |= reg_data;
  ret = aw86225_i2c_write(aw86225, reg_addr, reg_val);
  if (ret < 0)
    {
      ierr("i2c write error, ret: %d\n", ret);
    }

  return ret;
}

static int aw86225_request_firmware(FAR struct aw86225_firmware *fw,
                                    FAR const char *filename)
{
  FAR char *file_path;
  struct file file;
  size_t file_size;
  int ret;

  file_path = lib_get_pathbuffer();
  if (file_path == NULL)
    {
      return -ENOMEM;
    }

  snprintf(file_path, PATH_MAX, "%s/%s",
           CONFIG_AW86225_RTP_FILE_PATH, filename);
  ret = file_open(&file, file_path, O_RDONLY);
  lib_put_pathbuffer(file_path);
  if (ret < 0)
    {
      ierr("open file failed");
      return ret;
    }

  file_size = file_seek(&file, 0, SEEK_END);
  file_seek(&file, 0, SEEK_SET);

  fw->data = kmm_malloc(file_size);
  if (fw->data != NULL)
    {
      fw->size = file_read(&file, (void *)fw->data, file_size);
    }

  file_close(&file);

  if (fw->data == NULL || fw->size <= 0)
    {
      kmm_free((void *)fw->data);
      return -1;
    }

  return 0;
}

static void aw86225_release_firmware(FAR struct aw86225_firmware *fw)
{
  kmm_free((void *)fw->data);
  fw->data = NULL;
  fw->size = 0;
}

static void aw86225_haptic_upload_lra(FAR struct aw86225 *aw86225,
                                      unsigned int flag)
{
  switch (flag)
    {
      case AW86225_WRITE_ZERO:
        {
          aw86225_i2c_write_bits(aw86225, AW86225_REG_TRIMCFG3,
                                 AW86225_BIT_TRIMCFG3_TRIM_LRA_MASK,
                                 0x00);
          break;
        }

      case AW86225_F0_CALI:
        {
          aw86225_i2c_write_bits(aw86225, AW86225_REG_TRIMCFG3,
                                 AW86225_BIT_TRIMCFG3_TRIM_LRA_MASK,
                                 (char)aw86225->f0_cali_data);
          break;
        }

      case AW86225_OSC_CALI:
        {
          aw86225_i2c_write_bits(aw86225, AW86225_REG_TRIMCFG3,
                                 AW86225_BIT_TRIMCFG3_TRIM_LRA_MASK,
                                 (char)aw86225->osc_cali_data);
          break;
        }

      default:
          ierr("%s write trim_lra error !\n", __func__);
          break;
    }
}

static void aw86225_sram_size(FAR struct aw86225 *aw86225,
                              int size_flag)
{
  if (size_flag == AW86225_HAPTIC_SRAM_2K)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG1,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_EN);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG1,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_DIS);
    }
  else if (size_flag == AW86225_HAPTIC_SRAM_1K)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG1,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_DIS);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG1,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_EN);
    }
  else if (size_flag == AW86225_HAPTIC_SRAM_3K)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG1,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_EN);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG1,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
                             AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_EN);
    }
}

static void aw86225_haptic_stop(FAR struct aw86225 *aw86225)
{
  unsigned char cnt = 40;
  unsigned char reg_val = 0;
  bool force_flag = true;

  aw86225->play_mode = AW86225_HAPTIC_STANDBY_MODE;
  aw86225_i2c_write(aw86225, AW86225_REG_PLAYCFG4, 0x02);
  while (cnt)
    {
      aw86225_i2c_read(aw86225, AW86225_REG_GLBRD5, &reg_val);
      if ((reg_val & 0x0f) == 0x00 || (reg_val & 0x0f) == 0x0a)
        {
          cnt = 0;
          force_flag = false;
          iinfo("entered standby! glb_state=0x%02X\n", reg_val);
        }
      else
        {
          cnt--;
          iinfo("wait for standby, glb_state=0x%02X\n", reg_val);
        }

      usleep(2000);
    }

  if (force_flag)
    {
      iinfo("%s force to enter standby mode!\n", __func__);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL2,
                             AW86225_BIT_SYSCTRL2_STANDBY_MASK,
                             AW86225_BIT_SYSCTRL2_STANDBY_ON);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL2,
                             AW86225_BIT_SYSCTRL2_STANDBY_MASK,
                             AW86225_BIT_SYSCTRL2_STANDBY_OFF);
    }
}

static void aw86225_haptic_raminit(FAR struct aw86225 *aw86225,
                                   bool flag)
{
  if (flag)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL1,
                             AW86225_BIT_SYSCTRL1_RAMINIT_MASK,
                             AW86225_BIT_SYSCTRL1_RAMINIT_ON);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL1,
                             AW86225_BIT_SYSCTRL1_RAMINIT_MASK,
                             AW86225_BIT_SYSCTRL1_RAMINIT_OFF);
    }
}

static void aw86225_haptic_get_vbat(FAR struct aw86225 *aw86225)
{
  unsigned char reg_val = 0;
  unsigned int vbat_code = 0;

  /* unsigned int cont = 2000; */

  aw86225_haptic_stop(aw86225);
  aw86225_haptic_raminit(aw86225, true);
  aw86225_i2c_write_bits(aw86225, AW86225_REG_DETCFG2,
                         AW86225_BIT_DETCFG2_VBAT_GO_MASK,
                         AW86225_BIT_DETCFG2_VABT_GO_ON);
  usleep(22500);
  aw86225_i2c_read(aw86225, AW86225_REG_DET_VBAT, &reg_val);
  vbat_code = (vbat_code | reg_val) << 2;
  aw86225_i2c_read(aw86225, AW86225_REG_DET_LO, &reg_val);
  vbat_code = vbat_code | ((reg_val & 0x30) >> 4);
  aw86225->vbat = 6100 * vbat_code / 1024;
  if (aw86225->vbat > AW86225_VBAT_MAX)
    {
      aw86225->vbat = AW86225_VBAT_MAX;
    }

  if (aw86225->vbat < AW86225_VBAT_MIN)
    {
      aw86225->vbat = AW86225_VBAT_MIN;
    }

  iinfo("%s aw86225->vbat=%dmV, vbat_code=0x%02X\n", __func__,
         aw86225->vbat, vbat_code);
  aw86225_haptic_raminit(aw86225, false);
}

static void aw86225_interrupt_clear(FAR struct aw86225 *aw86225)
{
  unsigned char reg_val = 0;
  aw86225_i2c_read(aw86225, AW86225_REG_SYSINT, &reg_val);
}

static int aw86225_haptic_set_gain(FAR struct aw86225 *aw86225,
                                   unsigned char gain)
{
  unsigned char comp_gain = 0;
  if (aw86225->ram_vbat_compensate == AW86225_HAPTIC_RAM_VBAT_COMP_ENABLE)
    {
      aw86225_haptic_get_vbat(aw86225);
      comp_gain = aw86225->gain * AW86225_VBAT_REFER / aw86225->vbat;
      if (comp_gain > (128 * AW86225_VBAT_REFER / AW86225_VBAT_MIN))
        {
          comp_gain = 128 * AW86225_VBAT_REFER / AW86225_VBAT_MIN;
        }

      iinfo("%s: enable vbat comp, level = %x comp level = %x", __func__,
             gain, comp_gain);
      aw86225_i2c_write(aw86225, AW86225_REG_PLAYCFG2, comp_gain);
    }
  else
    {
      iinfo("%s: disable compsensation, vbat=%d, vbat_min=%d, vbat_ref=%d",
             __func__, aw86225->vbat, AW86225_VBAT_MIN, AW86225_VBAT_REFER);
      aw86225_i2c_write(aw86225, AW86225_REG_PLAYCFG2, gain);
    }

  return 0;
}

static int aw86225_haptic_ram_vbat_compensate(FAR struct aw86225 *aw86225,
                                              bool flag)
{
  if (flag)
    {
      aw86225->ram_vbat_compensate = AW86225_HAPTIC_RAM_VBAT_COMP_ENABLE;
    }
  else
    {
      aw86225->ram_vbat_compensate = AW86225_HAPTIC_RAM_VBAT_COMP_DISABLE;
    }

  return 0;
}

static int aw86225_haptic_play_mode(FAR struct aw86225 *aw86225,
                                    unsigned char play_mode)
{
  switch (play_mode)
    {
      case AW86225_HAPTIC_STANDBY_MODE:
        {
          aw86225->play_mode = AW86225_HAPTIC_STANDBY_MODE;
          aw86225_haptic_stop(aw86225);
          break;
        }

      case AW86225_HAPTIC_RAM_MODE:
        {
          aw86225->play_mode = AW86225_HAPTIC_RAM_MODE;
          aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_RAM);
          break;
        }

      case AW86225_HAPTIC_RAM_LOOP_MODE:
        {
          aw86225->play_mode = AW86225_HAPTIC_RAM_LOOP_MODE;
          aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_RAM);
          break;
        }

      case AW86225_HAPTIC_RTP_MODE:
        {
          aw86225->play_mode = AW86225_HAPTIC_RTP_MODE;
          aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_RTP);
          break;
        }

      case AW86225_HAPTIC_TRIG_MODE:
        {
          aw86225->play_mode = AW86225_HAPTIC_TRIG_MODE;
          aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK,
                                 AW86225_BIT_PLAYCFG3_PLAY_MODE_RAM);
          break;
        }

      case AW86225_HAPTIC_CONT_MODE:
        {
          aw86225->play_mode = AW86225_HAPTIC_CONT_MODE;
          aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
              AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK,
              AW86225_BIT_PLAYCFG3_PLAY_MODE_CONT);
          break;
        }

      default:
        ierr("%s: play mode %d error", __func__, play_mode);
        break;
    }

  return 0;
}

static int aw86225_haptic_play_go(FAR struct aw86225 *aw86225, bool flag)
{
  unsigned char reg_val = 0;

  if (flag == true)
    {
      aw86225_i2c_write(aw86225, AW86225_REG_PLAYCFG4, 0x01);
      usleep(2000);
    }
  else
    {
      aw86225_i2c_write(aw86225, AW86225_REG_PLAYCFG4, 0x02);
    }

  aw86225_i2c_read(aw86225, AW86225_REG_GLBRD5, &reg_val);
  return 0;
}

static int aw86225_haptic_set_wav_seq(FAR struct aw86225 *aw86225,
      unsigned char wav, unsigned char seq)
{
  aw86225_i2c_write(aw86225, AW86225_REG_WAVCFG1 + wav, seq);
  return 0;
}

static int aw86225_haptic_set_wav_loop(FAR struct aw86225 *aw86225,
      unsigned char wav, unsigned char loop)
{
  unsigned char tmp = 0;

  if (wav % 2)
    {
      tmp = loop << 0;
      aw86225_i2c_write_bits(aw86225, AW86225_REG_WAVCFG9 + (wav / 2),
                             AW86225_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
    }
  else
    {
      tmp = loop << 4;
      aw86225_i2c_write_bits(aw86225, AW86225_REG_WAVCFG9 + (wav / 2),
          AW86225_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
    }

  return 0;
}

static int aw86225_haptic_set_main_loop(FAR struct aw86225 *aw86225,
                                        uint8_t loop)
{
  aw86225_i2c_write_bits(aw86225, AW86225_REG_WAVCFG13,
                         AW86225_BIT_WAVCFG13_MAINLOOP_MASK, loop);
  return 0;
}

static int aw86225_haptic_read_lra_f0(FAR struct aw86225 *aw86225)
{
  unsigned char reg_val = 0;
  unsigned int f0_reg = 0;
  unsigned long f0_tmp = 0;

  /* F_LRA_F0_H */

  aw86225_i2c_read(aw86225, AW86225_REG_CONTRD14, &reg_val);
  f0_reg = (f0_reg | reg_val) << 8;

  /* F_LRA_F0_L */

  aw86225_i2c_read(aw86225, AW86225_REG_CONTRD15, &reg_val);
  f0_reg |= (reg_val << 0);
  if (!f0_reg)
    {
      ierr("%s didn't get cont f0 because f0_reg value is 0!\n", __func__);
      aw86225->f0 = aw86225->config->f0_ref;
      return -1;
    }
  else
    {
      f0_tmp = 384000 * 10 / f0_reg;
      aw86225->f0 = (unsigned int)f0_tmp;
      iinfo("%s lra_f0=%d\n", __func__, aw86225->f0);
    }

  return 0;
}

static int aw86225_haptic_read_cont_f0(FAR struct aw86225 *aw86225)
{
  unsigned char reg_val = 0;
  unsigned int f0_reg = 0;
  unsigned long f0_tmp = 0;

  aw86225_i2c_read(aw86225, AW86225_REG_CONTRD16, &reg_val);
  f0_reg = (f0_reg | reg_val) << 8;
  aw86225_i2c_read(aw86225, AW86225_REG_CONTRD17, &reg_val);
  f0_reg |= (reg_val << 0);
  if (!f0_reg)
    {
      ierr("%s didn't get cont f0 because f0_reg value is 0!\n", __func__);
      aw86225->cont_f0 = aw86225->config->f0_ref;
      return -1;
    }
  else
    {
      f0_tmp = 384000 * 10 / f0_reg;
      aw86225->cont_f0 = (unsigned int)f0_tmp;
      iinfo("%s cont_f0=%d\n", __func__, aw86225->cont_f0);
    }

  return 0;
}

static int aw86225_haptic_cont_get_f0(FAR struct aw86225 *aw86225)
{
  int ret = 0;
  unsigned char reg_val = 0;
  unsigned int cnt = 200;
  bool get_f0_flag = false;
  unsigned char brk_en_temp = 0;

  aw86225->f0 = aw86225->config->f0_ref;

  /* enter standby mode */

  aw86225_haptic_stop(aw86225);

  /* f0 calibrate work mode */

  aw86225_haptic_play_mode(aw86225, AW86225_HAPTIC_CONT_MODE);

  /* enable f0 detect */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG1,
                         AW86225_BIT_CONTCFG1_EN_F0_DET_MASK,
                         AW86225_BIT_CONTCFG1_F0_DET_ENABLE);

  /* cont config */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG6,
                         AW86225_BIT_CONTCFG6_TRACK_EN_MASK,
                         AW86225_BIT_CONTCFG6_TRACK_ENABLE);

  /* enable auto brake */

  aw86225_i2c_read(aw86225, AW86225_REG_PLAYCFG3, &reg_val);
  brk_en_temp = 0x04 & reg_val;
  aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                         AW86225_BIT_PLAYCFG3_BRK_EN_MASK,
                         AW86225_BIT_PLAYCFG3_BRK_ENABLE);

  /* f0 driver level */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG6,
                         AW86225_BIT_CONTCFG6_DRV1_LVL_MASK,
                         aw86225->config->cont_drv1_lvl_dt);
  aw86225_i2c_write(aw86225, AW86225_REG_CONTCFG7,
                    aw86225->config->cont_drv2_lvl_dt);

  /* DRV1_TIME */

  aw86225_i2c_write(aw86225, AW86225_REG_CONTCFG8,
                    aw86225->config->cont_drv1_time_dt);

  /* DRV2_TIME */

  aw86225_i2c_write(aw86225, AW86225_REG_CONTCFG9,
                    aw86225->config->cont_drv2_time_dt);

  /* TRACK_MARGIN */

  if (!aw86225->config->cont_track_margin)
    {
      ierr("%s aw86225->config->cont_track_margin = 0!\n", __func__);
    }
  else
    {
      aw86225_i2c_write(aw86225, AW86225_REG_CONTCFG11,
      (unsigned char)aw86225->config->cont_track_margin);
    }

  /* cont play go */

  aw86225_haptic_play_go(aw86225, true);

  /* 300ms */

  while (cnt)
    {
      aw86225_i2c_read(aw86225, AW86225_REG_GLBRD5, &reg_val);
      if ((reg_val & 0x0f) == 0x00)
        {
          cnt = 0;
          get_f0_flag = true;
          iinfo("%s entered standby mode! glb_state=0x%02X\n",
                 __func__, reg_val);
        }
      else
        {
          cnt--;
          iinfo("%s waitting for standby, glb_state=0x%02X\n",
                 __func__, reg_val);
        }

      usleep(10000);
    }

  if (get_f0_flag)
    {
      aw86225_haptic_read_lra_f0(aw86225);
      aw86225_haptic_read_cont_f0(aw86225);
    }
  else
    {
      ierr("%s enter standby mode failed, stop reading f0!\n", __func__);
    }

  /* restore default config */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG1,
                         AW86225_BIT_CONTCFG1_EN_F0_DET_MASK,
                         AW86225_BIT_CONTCFG1_F0_DET_DISABLE);

  /* recover auto break config */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                         AW86225_BIT_PLAYCFG3_BRK_EN_MASK,
                         brk_en_temp);

  return ret;
}

#ifdef CONFIG_AW86225_RTP_FILE_SUPPORT
static uint8_t aw86225_haptic_rtp_get_fifo_afs(FAR struct aw86225 *aw86225)
{
  uint8_t reg_val = 0;

  aw86225_i2c_read(aw86225, AW86225_REG_SYSST, &reg_val);
  reg_val &= AW86225_BIT_SYSST_FF_AFS;
  return reg_val >> 3;
}

static void aw86225_haptic_set_rtp_aei(FAR struct aw86225 *aw86225,
                                       bool flag)
{
  if (flag)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSINTM,
                             AW86225_BIT_SYSINTM_FF_AEM_MASK,
                             AW86225_BIT_SYSINTM_FF_AEM_ON);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSINTM,
                             AW86225_BIT_SYSINTM_FF_AEM_MASK,
                             AW86225_BIT_SYSINTM_FF_AEM_OFF);
    }
}

static int aw86225_haptic_rtp_init(FAR struct aw86225 *aw86225)
{
  unsigned int buf_len = 0;
  unsigned char glb_state_val = 0;

  aw86225->rtp_cnt = 0;
  nxmutex_lock(&aw86225->rtp_lock);
  while ((!aw86225_haptic_rtp_get_fifo_afs(aw86225))
         && (aw86225->play_mode == AW86225_HAPTIC_RTP_MODE)
         &&  !atomic_load(&aw86225->exit_in_rtp_loop))
    {
      if (!aw86225->rtp_container)
        {
          ierr("%s:aw86225->rtp_container is null\n", __func__);
          break;
        }

      if (aw86225->rtp_cnt < aw86225->ram.base_addr)
        {
          if ((aw86225->rtp_container->len - aw86225->rtp_cnt) <
              aw86225->ram.base_addr)
            {
              buf_len = aw86225->rtp_container->len - aw86225->rtp_cnt;
            }
          else
            {
              buf_len = aw86225->ram.base_addr;
            }
        }
      else if ((aw86225->rtp_container->len - aw86225->rtp_cnt) <
               (aw86225->ram.base_addr >> 2))
        {
          buf_len = aw86225->rtp_container->len - aw86225->rtp_cnt;
        }
      else
        {
          buf_len = aw86225->ram.base_addr >> 2;
        }

      aw86225_i2c_writes(aw86225, AW86225_REG_RTPDATA,
                         &aw86225->rtp_container->data[aw86225->rtp_cnt],
                         buf_len);
      aw86225->rtp_cnt += buf_len;
      aw86225_i2c_read(aw86225, AW86225_REG_GLBRD5, &glb_state_val);
      if ((aw86225->rtp_cnt == aw86225->rtp_container->len)
          || ((glb_state_val & 0x0f) == 0x00))
        {
          if (aw86225->rtp_cnt == aw86225->rtp_container->len)
            {
              iinfo("%s: rtp load completely! glb_state_val=0x%02x \
                     aw86225->rtp_cnt=%d\n", \
                     __func__, glb_state_val, aw86225->rtp_cnt);
            }
          else
            {
              iinfo("%s rtp load failed!! glb_state_val=0x%02x \
                     aw86225->rtp_cnt=%d\n", __func__, glb_state_val, \
                     aw86225->rtp_cnt);
              aw86225->rtp_cnt = 0;
              nxmutex_unlock(&aw86225->rtp_lock);
              return 0;
            }
        }
    }

  nxmutex_unlock(&aw86225->rtp_lock);
  if (aw86225->play_mode == AW86225_HAPTIC_RTP_MODE
      && !atomic_load(&aw86225->exit_in_rtp_loop))
    {
      aw86225_haptic_set_rtp_aei(aw86225, true);
    }

  iinfo("%s exit\n", __func__);
  return 0;
}
#endif

static int aw86225_haptic_set_repeat_wav_seq(FAR struct aw86225 *aw86225,
          unsigned char seq)
{
  aw86225_haptic_set_wav_seq(aw86225, 0x00, seq);
  aw86225_haptic_set_wav_loop(aw86225, 0x00,
                              AW86225_BIT_WAVLOOP_INIFINITELY);
  return 0;
}

static void aw86225_haptic_set_pwm(FAR struct aw86225 *aw86225,
                                   unsigned char mode)
{
  switch (mode)
    {
      case AW86225_PWM_48K:
        {
          aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL2,
                                 AW86225_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
                                 AW86225_BIT_SYSCTRL2_RATE_48K);
          break;
        }

      case AW86225_PWM_24K:
        {
          aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL2,
                                 AW86225_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
                                 AW86225_BIT_SYSCTRL2_RATE_24K);
          break;
        }

      case AW86225_PWM_12K:
        {
          aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL2,
                                 AW86225_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
                                 AW86225_BIT_SYSCTRL2_RATE_12K);
          break;
        }

      default:
          break;
    }
}

static int16_t
aw86225_haptic_effect_strength(FAR struct aw86225 *aw86225)
{
  iinfo("aw86225->play.vmax_mv =0x%x\n", aw86225->play.vmax_mv);

  if (aw86225->play.vmax_mv >= 0x7fff)
    {
      aw86225->level = 0x80;
    }
  else if (aw86225->play.vmax_mv <= 0x3fff)
    {
      aw86225->level = 0x1e;
    }
  else
    {
      aw86225->level = (aw86225->play.vmax_mv - 16383) / 128;
    }

  if (aw86225->level < 0x1e)
    {
      aw86225->level = 0x1e;
    }

  iinfo("%s: aw86225->level =0x%x\n", __func__, aw86225->level);
  return 0;
}

#ifdef CONFIG_AW86225_RTP_FILE_SUPPORT
static void aw86225_rtp_work_routine(FAR void *arg)
{
  FAR struct aw86225 *aw86225 = arg;
  struct aw86225_firmware rtp_file;
  int ret = -1;
  unsigned int cnt = 200;
  unsigned char reg_val = 0;
  bool rtp_work_flag = false;

  iinfo("%s enter\n", __func__);

  if ((aw86225->effect_id < aw86225->config->effect_id_boundary) &&
      (aw86225->effect_id > aw86225->config->effect_max))
    {
      return;
    }

  iinfo("%s: effect_id =%d state = %d\n", __func__, aw86225->effect_id,
         aw86225->state);
  nxmutex_lock(&aw86225->lock);
  aw86225_haptic_upload_lra(aw86225, AW86225_OSC_CALI);
  aw86225_haptic_set_rtp_aei(aw86225, false);
  aw86225_interrupt_clear(aw86225);

  /* wait for irq to exit */

  atomic_store(&aw86225->exit_in_rtp_loop, 1);
  while (atomic_load(&aw86225->is_in_rtp_loop))
    {
      iinfo("%s:  goint to waiting irq exit\n", __func__);

      ret = nxsem_wait(&aw86225->wait_q);

      if (ret == -ERESTART)
        {
          atomic_store(&aw86225->exit_in_rtp_loop, 0);
          nxsem_post(&aw86225->stop_wait_q);
          nxmutex_unlock(&aw86225->lock);
          ierr("%s: wake up by signal return erro\n", __func__);
          return;
        }
    }

  atomic_store(&aw86225->exit_in_rtp_loop, 0);
  nxsem_post(&aw86225->stop_wait_q);
  aw86225_haptic_stop(aw86225);

  if (aw86225->state)
    {
      aw86225->wk_lock_flag = 1;
      aw86225_haptic_effect_strength(aw86225);
      aw86225->rtp_file_num = aw86225->effect_id -
                              aw86225->config->effect_id_boundary;
      iinfo("aw86225->rtp_file_num =%d\n", aw86225->rtp_file_num);
      if (aw86225->rtp_file_num < 0)
        {
          aw86225->rtp_file_num = 0;
        }

      if (aw86225->rtp_file_num >
          ((sizeof(g_aw86225_rtp_name) / AW86225_RTP_NAME_MAX) - 1))
        {
          aw86225->rtp_file_num =
            (sizeof(g_aw86225_rtp_name) / AW86225_RTP_NAME_MAX) - 1;
        }

      /* fw loaded */

      ret = aw86225_request_firmware(&rtp_file,
            g_aw86225_rtp_name[aw86225->rtp_file_num]);
      if (ret < 0)
        {
          ierr("%s: failed to read %s\n", __func__,
                 g_aw86225_rtp_name[aw86225->rtp_file_num]);
          if (aw86225->wk_lock_flag == 1)
            {
              aw86225->wk_lock_flag = 0;
            }

          nxmutex_unlock(&aw86225->lock);
          return;
        }

      aw86225->rtp_init = 0;
      kmm_free(aw86225->rtp_container);
      aw86225->rtp_container = kmm_malloc(rtp_file.size + sizeof(int));
      if (!aw86225->rtp_container)
        {
          aw86225_release_firmware(&rtp_file);
          ierr("%s: error allocating memory\n", __func__);
          if (aw86225->wk_lock_flag == 1)
            {
              aw86225->wk_lock_flag = 0;
            }

          nxmutex_unlock(&aw86225->lock);
          return;
        }

      aw86225->rtp_container->len = rtp_file.size;
      iinfo("%s: rtp file:[%s] size = %dbytes\n",
             __func__, g_aw86225_rtp_name[aw86225->rtp_file_num],
             aw86225->rtp_container->len);
      memcpy(aw86225->rtp_container->data, rtp_file.data, rtp_file.size);
      aw86225_release_firmware(&rtp_file);
      aw86225->rtp_init = 1;
      aw86225_haptic_upload_lra(aw86225, AW86225_OSC_CALI);
      aw86225_haptic_set_pwm(aw86225, AW86225_PWM_24K);

      /* gain */

      aw86225_haptic_ram_vbat_compensate(aw86225, false);
      aw86225_haptic_set_gain(aw86225, aw86225->level);

      /* rtp mode config */

      aw86225_haptic_play_mode(aw86225, AW86225_HAPTIC_RTP_MODE);

      /* haptic go */

      aw86225_haptic_play_go(aw86225, true);
      nxmutex_unlock(&aw86225->lock);
      usleep(2000);
      while (cnt)
        {
          aw86225_i2c_read(aw86225, AW86225_REG_GLBRD5, &reg_val);
          if ((reg_val & 0x0f) == 0x08)
            {
              cnt = 0;
              rtp_work_flag = true;
              iinfo("%s RTP_GO! glb_state=0x08\n", __func__);
            }
          else
            {
              cnt--;
              iinfo("wait for RTP_GO, glb_state=0x%02X\n", reg_val);
            }

          usleep(2000);
        }

      if (rtp_work_flag)
        {
          aw86225_haptic_rtp_init(aw86225);
        }
      else
        {
          aw86225_haptic_stop(aw86225);
          ierr("%s failed to enter RTP_GO status!\n", __func__);
        }
    }
  else
    {
      if (aw86225->wk_lock_flag == 1)
        {
          aw86225->wk_lock_flag = 0;
        }

      aw86225->rtp_cnt = 0;
      aw86225->rtp_init = 0;
      nxmutex_unlock(&aw86225->lock);
    }
}
#endif

static int aw86225_container_update(FAR struct aw86225 *aw86225,
                     FAR struct aw86225_container *aw86225_cont)
{
  unsigned char reg_val = 0;
  unsigned int shift = 0;
  unsigned int temp = 0;
  int i = 0;
  int ret = 0;
#ifdef AW_CHECK_RAM_DATA
  unsigned short check_sum = 0;
#endif

  iinfo("%s enter\n", __func__);
  nxmutex_lock(&aw86225->lock);
  aw86225->ram.baseaddr_shift = 2;
  aw86225->ram.ram_shift = 4;

  /* RAMINIT Enable */

  aw86225_haptic_raminit(aw86225, true);

  /* Enter standby mode */

  aw86225_haptic_stop(aw86225);

  /* base addr */

  shift = aw86225->ram.baseaddr_shift;
  aw86225->ram.base_addr = (unsigned int)
                           ((aw86225_cont->data[0 + shift] << 8) |
                           (aw86225_cont->data[1 + shift]));

  /* default 3k SRAM */

  aw86225_sram_size(aw86225, AW86225_HAPTIC_SRAM_3K);

  aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG1,
                         AW86225_BIT_RTPCFG1_ADDRH_MASK,
                         aw86225_cont->data[0 + shift]);

  aw86225_i2c_write(aw86225, AW86225_REG_RTPCFG2,
                    aw86225_cont->data[1 + shift]);

  /* FIFO_AEH */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG3,
                         AW86225_BIT_RTPCFG3_FIFO_AEH_MASK,
                         (unsigned char)
                         (((aw86225->ram.base_addr >> 1) >> 4) & 0xf0));

  /* FIFO AEL */

  aw86225_i2c_write(aw86225, AW86225_REG_RTPCFG4,
                    (unsigned char)
                    (((aw86225->ram.base_addr >> 1) & 0x00ff)));

  /* FIFO_AFH */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_RTPCFG3,
                         AW86225_BIT_RTPCFG3_FIFO_AFH_MASK,
                         (unsigned char)(((aw86225->ram.base_addr -
                         (aw86225->ram.base_addr >> 2)) >> 8) & 0x0f));

  /* FIFO_AFL */

  aw86225_i2c_write(aw86225, AW86225_REG_RTPCFG5,
                    (unsigned char)(((aw86225->ram.base_addr -
                    (aw86225->ram.base_addr >> 2)) & 0x00ff)));

  aw86225_i2c_read(aw86225, AW86225_REG_RTPCFG3, &reg_val);
  temp = ((reg_val & 0x0f) << 24) | ((reg_val & 0xf0) << 4);
  aw86225_i2c_read(aw86225, AW86225_REG_RTPCFG4, &reg_val);
  temp = temp | reg_val;
  iinfo("almost_empty_threshold = %d", (unsigned short)temp);
  aw86225_i2c_read(aw86225, AW86225_REG_RTPCFG5, &reg_val);
  temp = temp | (reg_val << 16);
  iinfo("almost_full_threshold = %d\n", temp >> 16);

  shift = aw86225->ram.baseaddr_shift;

  aw86225_i2c_write_bits(aw86225, AW86225_REG_RAMADDRH,
                         AW86225_BIT_RAMADDRH_MASK,
                         aw86225_cont->data[0 + shift]);
  aw86225_i2c_write(aw86225, AW86225_REG_RAMADDRL,
                    aw86225_cont->data[1 + shift]);
  shift = aw86225->ram.ram_shift;
  iinfo("%s: ram_len = %d\n", __func__, aw86225_cont->len - shift);
  for (i = shift; i < aw86225_cont->len; i++)
    {
      aw86225->ram_update_flag = aw86225_i2c_write(aw86225,
                                                   AW86225_REG_RAMDATA,
                                                   aw86225_cont->data[i]);
    }

#ifdef AW_CHECK_RAM_DATA
  shift = aw86225->ram.baseaddr_shift;
  aw86225_i2c_write_bits(aw86225, AW86225_REG_RAMADDRH,
                         AW86225_BIT_RAMADDRH_MASK,
                         aw86225_cont->data[0 + shift]);
  aw86225_i2c_write(aw86225, AW86225_REG_RAMADDRL,
                    aw86225_cont->data[1 + shift]);
  shift = aw86225->ram.ram_shift;
  for (i = shift; i < aw86225_cont->len; i++)
    {
      aw86225_i2c_read(aw86225, AW86225_REG_RAMDATA, &reg_val);
      if (reg_val != aw86225_cont->data[i])
        {
          iinfo("%s: ram check error addr=0x%04x, file_data=0x%02X, \
                 ram_data=0x%02X\n", \
                 __func__, i, aw86225_cont->data[i], reg_val);
          ret = -1;
          break;
        }

      check_sum += reg_val;
    }

  if (!ret)
    {
      aw86225_i2c_read(aw86225, AW86225_REG_RTPCFG1, &reg_val);
      check_sum += reg_val & 0x0f;
      aw86225_i2c_read(aw86225, AW86225_REG_RTPCFG2, &reg_val);
      check_sum += reg_val;

      if (check_sum != aw86225->ram.check_sum)
        {
          ierr("ram data check sum error, check_sum=0x%04x\n", check_sum);
          ret = -1;
        }
      else
        {
          iinfo("ram data check sum pass, check_sum=0x%04x\n", check_sum);
        }
    }
#endif

  /* RAMINIT Disable */

  aw86225_haptic_raminit(aw86225, false);
  nxmutex_unlock(&aw86225->lock);
  iinfo("%s exit\n", __func__);
  return ret;
}

static void aw86225_ram_loaded(FAR struct aw86225_firmware *cont,
                               FAR struct aw86225 *aw86225)
{
  struct aw86225_container *aw86225_fw;
  unsigned short check_sum = 0;
  int i = 0;
  int ret = 0;
#ifdef AW_READ_BIN_FLEXBALLY
  static unsigned char load_cont;
  int ram_timer_val = 1000;

  load_cont++;
#endif
  ierr("%s enter\n", __func__);
  ret = aw86225_request_firmware(cont, g_aw86225_ram_name);
  if (ret < 0)
    {
      ierr("failed to request firmware %s\n", g_aw86225_ram_name);
      return;
    }

  if (!cont)
    {
      ierr("%s: failed to read %s\n", __func__, g_aw86225_ram_name);
      aw86225_release_firmware(cont);
#ifdef AW_READ_BIN_FLEXBALLY
      if (load_cont <= 20)
        {
          work_queue(HPWORK, &aw86225->ram_work,
                     aw86225_ram_work_routine,
                     aw86225, MSEC2TICK(ram_timer_val));
          ierr("%s:start hrtimer: load_cont=%d\n", __func__, load_cont);
        }

#endif
      return;
    }

  /* check sum */

  for (i = 2; i < cont->size; i++)
    {
      check_sum += cont->data[i];
    }

  if (check_sum != (unsigned short)((cont->data[0] << 8) | (cont->data[1])))
    {
      ierr("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
      return;
    }
  else
    {
      iinfo("%s: check sum pass: 0x%04x\n", __func__, check_sum);
      aw86225->ram.check_sum = check_sum;
    }

  /* aw86225 ram update less then 128kB */

  aw86225_fw = kmm_zalloc(cont->size + sizeof(int));
  if (!aw86225_fw)
    {
      aw86225_release_firmware(cont);
      ierr("%s: Error allocating memory\n", __func__);
      return;
    }

  aw86225_fw->len = cont->size;
  memcpy(aw86225_fw->data, cont->data, cont->size);
  aw86225_release_firmware(cont);
  ret = aw86225_container_update(aw86225, aw86225_fw);
  if (ret)
    {
      kmm_free(aw86225_fw);
      aw86225->ram.len = 0;
      ierr("%s: ram firmware update failed!\n", __func__);
    }
  else
    {
      aw86225->ram_init = 1;
      aw86225->ram.len = aw86225_fw->len;
      kmm_free(aw86225_fw);
      iinfo("%s: ram firmware update complete!\n", __func__);
    }
}

static void aw86225_vibrator_timer_func(wdparm_t arg)
{
  struct aw86225 *aw86225 = (struct aw86225 *)arg;
  iinfo("%s enter\n", __func__);
  aw86225->state = 0;
  work_queue(HPWORK, &aw86225->long_vibrate_work,
             aw86225_long_vibrate_work_routine, aw86225, 0);
}

static void
aw86225_haptic_play_repeat_seq(FAR struct aw86225 *aw86225,
                               unsigned char flag)
{
  if (flag)
    {
      aw86225_haptic_play_mode(aw86225, AW86225_HAPTIC_RAM_LOOP_MODE);
      aw86225_haptic_play_go(aw86225, true);
    }
}

static void aw86225_haptic_trig_config(FAR struct aw86225 *aw86225)
{
  if (aw86225->is_used_irq == false)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL2,
                             AW86225_BIT_SYSCTRL2_INTN_PIN_MASK,
                             AW86225_BIT_SYSCTRL2_TRIG1);
    }
}

static void
aw86225_haptic_swicth_motor_protect_config(FAR struct aw86225 *aw86225,
                                           unsigned char addr,
                                           unsigned char val)
{
  if (addr == 1)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_DETCFG1,
                             AW86225_BIT_DETCFG1_PRCT_MODE_MASK,
                             AW86225_BIT_DETCFG1_PRCT_MODE_VALID);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PWMCFG1,
                             AW86225_BIT_PWMCFG1_PRC_EN_MASK,
                             AW86225_BIT_PWMCFG1_PRC_ENABLE);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PWMCFG3,
                             AW86225_BIT_PWMCFG3_PR_EN_MASK,
                             AW86225_BIT_PWMCFG3_PR_ENABLE);
    }
  else if (addr == 0)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_DETCFG1,
                             AW86225_BIT_DETCFG1_PRCT_MODE_MASK,
                             AW86225_BIT_DETCFG1_PRCT_MODE_INVALID);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PWMCFG1,
                             AW86225_BIT_PWMCFG1_PRC_EN_MASK,
                             AW86225_BIT_PWMCFG1_PRC_DISABLE);
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PWMCFG3,
                             AW86225_BIT_PWMCFG3_PR_EN_MASK,
                             AW86225_BIT_PWMCFG3_PR_DISABLE);
    }
  else if (addr == 0x2d)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PWMCFG1,
                            AW86225_BIT_PWMCFG1_PRCTIME_MASK, val);
    }
  else if (addr == 0x3e)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PWMCFG3,
                            AW86225_BIT_PWMCFG3_PRLVL_MASK, val);
    }
  else if (addr == 0x3f)
    {
      aw86225_i2c_write(aw86225, AW86225_REG_PWMCFG4, val);
    }
}

static void aw86225_haptic_f0_calibration(FAR struct aw86225 *aw86225)
{
  unsigned char reg_val = 0;
  unsigned int f0_limit = 0;
  char f0_cali_lra = 0;
  int f0_cali_step = 0;
  unsigned int f0_cali_min = aw86225->config->f0_ref *
      (100 - aw86225->config->f0_cali_percent) / 100;
  unsigned int f0_cali_max =  aw86225->config->f0_ref *
      (100 + aw86225->config->f0_cali_percent) / 100;

  if (aw86225_haptic_cont_get_f0(aw86225))
    {
      ierr("%s get f0 error, user defafult f0\n", __func__);
    }
  else
    {
      /* max and min limit */

      f0_limit = aw86225->f0;
      iinfo("%s f0_ref = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d\n",
              __func__, aw86225->config->f0_ref,
              f0_cali_min, f0_cali_max, aw86225->f0);

      if ((aw86225->f0 < f0_cali_min) || aw86225->f0 > f0_cali_max)
        {
          ierr("f0 calibration out of range = %d!\n", aw86225->f0);
          f0_limit = aw86225->config->f0_ref;
          return;
        }

      /* calculate cali step */

      f0_cali_step = 100000 * ((int)f0_limit -
                     (int)aw86225->config->f0_ref) / ((int)f0_limit * 24);
      iinfo("%s f0_cali_step = %d\n", __func__, f0_cali_step);
      if (f0_cali_step >= 0)
        {
          /* f0_cali_step >= 0 */

          if (f0_cali_step % 10 >= 5)
            {
              f0_cali_step = 32 + (f0_cali_step / 10 + 1);
            }
          else
            {
              f0_cali_step = 32 + f0_cali_step / 10;
            }
        }
      else
        {
          /* f0_cali_step < 0 */

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

      aw86225->f0_cali_data = (int)f0_cali_lra;
      aw86225_haptic_upload_lra(aw86225, AW86225_F0_CALI);
      aw86225_i2c_read(aw86225, AW86225_REG_TRIMCFG3, &reg_val);
      iinfo("%s final trim_lra=0x%02x\n", __func__, reg_val);
    }

  /* restore standby work mode */

  aw86225_haptic_stop(aw86225);
}

/****************************************************************************
 * Name: haptic cont
 ****************************************************************************/

static int aw86225_haptic_cont_config(FAR struct aw86225 *aw86225)
{
  /* work mode */

  aw86225_haptic_play_mode(aw86225, AW86225_HAPTIC_CONT_MODE);
  aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG6,
                         AW86225_BIT_CONTCFG6_TRACK_EN_MASK,
                         AW86225_BIT_CONTCFG6_TRACK_ENABLE);

  /* f0 driver level */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG6,
                         AW86225_BIT_CONTCFG6_DRV1_LVL_MASK,
                         aw86225->cont_drv1_lvl);
  aw86225_i2c_write(aw86225, AW86225_REG_CONTCFG7,
                    aw86225->cont_drv2_lvl);

  aw86225_i2c_write(aw86225, AW86225_REG_CONTCFG9, 0xff);

  aw86225_haptic_play_go(aw86225, true);
  return 0;
}

static int aw86225_haptic_activate(FAR struct aw86225 *aw86225)
{
  aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL2,
                         AW86225_BIT_SYSCTRL2_STANDBY_MASK,
                         AW86225_BIT_SYSCTRL2_STANDBY_OFF);
  aw86225_interrupt_clear(aw86225);
  aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSINTM,
                         AW86225_BIT_SYSINTM_UVLM_MASK,
                         AW86225_BIT_SYSINTM_UVLM_ON);
  return 0;
}

static int aw86225_haptic_start(FAR struct aw86225 *aw86225)
{
  aw86225_haptic_activate(aw86225);
  aw86225_haptic_play_go(aw86225, true);
  return 0;
}

static int aw86225_haptic_play_effect_seq(FAR struct aw86225 *aw86225,
                                          unsigned char flag)
{
  if ((aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RAM_MODE ||
      aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RAM_LOOP_MODE) &&
      aw86225->effect_id > aw86225->config->effect_id_boundary)
    {
      return 0;
    }

  if (aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RTP_MODE &&
      (aw86225->effect_id > aw86225->config->effect_max ||
       aw86225->effect_id < aw86225->config->effect_id_boundary))
    {
      return 0;
    }

  iinfo("%s:effect_id =%d, activate_mode = %d\n",
         __func__, aw86225->effect_id, aw86225->activate_mode);
  if (flag)
    {
      if (aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RAM_MODE)
        {
          if (aw86225->effect_id == 5 || aw86225->effect_id == 6)
            {
              aw86225->effect_id = 6;
            }
          else if (aw86225->effect_id == 1)
            {
              aw86225->effect_id = 3;
            }
          else if ((aw86225->effect_id >= 2) && (aw86225->effect_id <= 9))
            {
              aw86225->effect_id = 8;
            }

          aw86225_haptic_set_wav_seq(aw86225, 0x00,
                                    (char)aw86225->effect_id + 1);
          aw86225_haptic_set_pwm(aw86225,  AW86225_PWM_12K);
          aw86225_haptic_set_wav_seq(aw86225, 0x01, 0x00);
          aw86225_haptic_set_wav_loop(aw86225, 0x00, 0x00);
          aw86225_haptic_play_mode(aw86225, AW86225_HAPTIC_RAM_MODE);
          aw86225_haptic_effect_strength(aw86225);
          aw86225_haptic_set_gain(aw86225, aw86225->level);
          aw86225_haptic_start(aw86225);
        }

      if (aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RAM_LOOP_MODE)
        {
          aw86225_haptic_set_repeat_wav_seq(aw86225,
          (aw86225->config->effect_id_boundary + 1));

          aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL7,
          AW86225_BIT_SYSCTRL7_GAIN_BYPASS_MASK,
          AW86225_BIT_SYSCTRL7_GAIN_CHANGEABLE);

          aw86225_haptic_set_pwm(aw86225,  AW86225_PWM_12K);
          aw86225_haptic_set_gain(aw86225, aw86225->level);
          aw86225_haptic_play_repeat_seq(aw86225, true);
        }

      if (aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RTP_MODE)
        {
          int budry = aw86225->effect_id -
                      aw86225->config->effect_id_boundary;
          iinfo("budry = %d\n", budry);
          for (uint8_t i = 0; i < 8; i++)
            {
              iinfo("wav_seq[%d] = %d\n", i,
                aw86225->pattern[budry].patternid[i]);
              iinfo("wav_loop[%d] = %d\n", i,
                aw86225->pattern[budry].waveloop[i]);
              aw86225_haptic_set_wav_seq(aw86225, i,
                aw86225->pattern[budry].patternid[i] + 1);
              aw86225_haptic_set_wav_loop(aw86225, i,
                aw86225->pattern[budry].waveloop[i]);
            }

          iinfo("main_loop = %d\n", aw86225->pattern[budry].mainloop);
          aw86225_haptic_set_main_loop(aw86225,
            aw86225->pattern[budry].mainloop);
          aw86225_haptic_set_pwm(aw86225,  AW86225_PWM_12K);
          aw86225_haptic_play_mode(aw86225, AW86225_HAPTIC_RAM_LOOP_MODE);
          aw86225_haptic_effect_strength(aw86225);
          aw86225_haptic_set_gain(aw86225, aw86225->level);
          aw86225_haptic_start(aw86225);
        }
    }

  iinfo("%s: exit\n", __func__);
  return 0;
}

static void aw86225_haptic_misc_para_init(FAR struct aw86225 *aw86225)
{
  aw86225->cont_drv1_lvl = aw86225->config->cont_drv1_lvl_dt;
  aw86225->cont_drv2_lvl = aw86225->config->cont_drv2_lvl_dt;
  aw86225->cont_drv1_time = aw86225->config->cont_drv1_time_dt;
  aw86225->cont_drv2_time = aw86225->config->cont_drv2_time_dt;
  aw86225->cont_brk_time = aw86225->config->cont_brk_time_dt;
  aw86225->cont_wait_num = aw86225->config->cont_wait_num_dt;

  /* SIN_H */

  aw86225_i2c_write(aw86225, AW86225_REG_SYSCTRL3,
                    aw86225->config->sine_array[0]);

  /* SIN_L */

  aw86225_i2c_write(aw86225, AW86225_REG_SYSCTRL4,
                    aw86225->config->sine_array[1]);

  /* COS_H */

  aw86225_i2c_write(aw86225, AW86225_REG_SYSCTRL5,
                    aw86225->config->sine_array[2]);

  /* COS_L */

  aw86225_i2c_write(aw86225, AW86225_REG_SYSCTRL6,
                    aw86225->config->sine_array[3]);
  aw86225_i2c_write_bits(aw86225, AW86225_REG_TRGCFG8,
                         AW86225_BIT_TRGCFG8_TRG_TRIG1_MODE_MASK,
                         AW86225_BIT_TRGCFG8_TRIG1);
  aw86225_i2c_write_bits(aw86225, AW86225_REG_ANACFG8,
                         AW86225_BIT_ANACFG8_TRTF_CTRL_HDRV_MASK,
                         AW86225_BIT_ANACFG8_TRTF_CTRL_HDRV);

  /* d2s_gain */

  if (!aw86225->config->d2s_gain)
    {
      iinfo("%s aw86225->config->d2s_gain = 0!\n", __func__);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL7,
                             AW86225_BIT_SYSCTRL7_D2S_GAIN_MASK,
                             aw86225->config->d2s_gain);
    }

  /* cont_tset */

  if (!aw86225->config->cont_tset)
    {
      iinfo("%s aw86225->config->cont_tset = 0!\n", __func__);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG13,
                             AW86225_BIT_CONTCFG13_TSET_MASK,
                             aw86225->config->cont_tset << 4);
    }

  /* cont_bemf_set */

  if (!aw86225->config->cont_bemf_set)
    {
      iinfo("%s aw86225->config->cont_bemf_set = 0!\n", __func__);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG13,
                             AW86225_BIT_CONTCFG13_BEME_SET_MASK,
                             aw86225->config->cont_bemf_set);
    }

  /* cont_brk_time */

  if (!aw86225->cont_brk_time)
    {
      iinfo("%s aw86225->cont_brk_time = 0!\n", __func__);
    }
  else
    {
      aw86225_i2c_write(aw86225, AW86225_REG_CONTCFG10,
                        aw86225->cont_brk_time);
    }

  /* cont_brk_gain */

  if (!aw86225->config->cont_brk_gain)
    {
      iinfo("%s aw86225->config->cont_brk_gain = 0!\n", __func__);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_CONTCFG5,
                             AW86225_BIT_CONTCFG5_BRK_GAIN_MASK,
                             aw86225->config->cont_brk_gain);
    }
}

/****************************************************************************
 * Name: offset calibration
 ****************************************************************************/

static int
aw86225_haptic_offset_calibration(FAR struct aw86225 *aw86225)
{
  unsigned int cont = 2000;
  unsigned char reg_val = 0;

  aw86225_haptic_raminit(aw86225, true);
  aw86225_i2c_write_bits(aw86225, AW86225_REG_DETCFG2,
      AW86225_BIT_DETCFG2_DIAG_GO_MASK,
      AW86225_BIT_DETCFG2_DIAG_GO_ON);

  while (1)
    {
      aw86225_i2c_read(aw86225, AW86225_REG_DETCFG2, &reg_val);
      if ((reg_val & 0x01) == 0 || cont == 0)
        {
          break;
        }

      cont--;
    }

  if (cont == 0)
    {
      ierr("%s calibration offset failed!\n", __func__);
    }

  aw86225_haptic_raminit(aw86225, false);
  return 0;
}

static void
aw86225_haptic_vbat_mode_config(FAR struct aw86225 *aw86225,
                                unsigned char flag)
{
  if (flag == AW86225_HAPTIC_CONT_VBAT_HW_ADJUST_MODE)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL1,
                             AW86225_BIT_SYSCTRL1_VBAT_MODE_MASK,
                             AW86225_BIT_SYSCTRL1_VBAT_MODE_HW);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_SYSCTRL1,
                             AW86225_BIT_SYSCTRL1_VBAT_MODE_MASK,
                             AW86225_BIT_SYSCTRL1_VBAT_MODE_SW);
    }
}

static void aw86225_haptic_auto_bst_enable(FAR struct aw86225 *aw86225,
                                           unsigned char flag)
{
  if (flag)
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                             AW86225_BIT_PLAYCFG3_BRK_EN_MASK,
                             AW86225_BIT_PLAYCFG3_BRK_ENABLE);
    }
  else
    {
      aw86225_i2c_write_bits(aw86225, AW86225_REG_PLAYCFG3,
                             AW86225_BIT_PLAYCFG3_BRK_EN_MASK,
                             AW86225_BIT_PLAYCFG3_BRK_DISABLE);
    }
}

static int aw86225_read_chipid(FAR struct aw86225 *aw86225)
{
  int ret;
  uint8_t devid = 1; /* set nonzero, because chipid is 0! */

  ret = aw86225_i2c_read(aw86225, 0x00, &devid);
  if (ret < 0 || devid != 0)
    {
      ierr("Wrong Device ID! %02x\n\n", devid);
      ret = -ENODEV;
    }

  return ret;
}

static void aw86225_ram_work_routine(FAR void *arg)
{
  struct aw86225_firmware rtp_file;
  FAR struct aw86225 *aw86225 = arg;

  aw86225->ram_init = 0;
  aw86225->rtp_init = 0;

  aw86225_ram_loaded(&rtp_file, aw86225);
}

static void aw86225_long_vibrate_work_routine(FAR void *arg)
{
  FAR struct aw86225 *aw86225 = arg;

  iinfo("effect_id = %d state=%d activate_mode = %d duration = %d\n",
        aw86225->effect_id, aw86225->state,
        aw86225->activate_mode, aw86225->duration);

  nxmutex_lock(&aw86225->lock);

  /* Enter standby mode */

  aw86225_haptic_stop(aw86225);
  aw86225_haptic_upload_lra(aw86225, AW86225_F0_CALI);
  if (aw86225->state)
    {
      if (aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RAM_MODE)
        {
          aw86225_haptic_ram_vbat_compensate(aw86225, false);
          aw86225_haptic_play_effect_seq(aw86225, true);
        }
      else if (aw86225->activate_mode ==
               AW86225_HAPTIC_ACTIVATE_RAM_LOOP_MODE)
        {
          aw86225_haptic_ram_vbat_compensate(aw86225, true);
          aw86225_haptic_play_effect_seq(aw86225, true);

          /* run ms timer */

          wd_start(&aw86225->timer, MSEC2TICK(aw86225->duration),
          aw86225_vibrator_timer_func, (wdparm_t)aw86225);
          aw86225->wk_lock_flag = 1;
        }
      else if (aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_CONT_MODE)
        {
          aw86225_haptic_cont_config(aw86225);

          /* run ms timer */

          wd_start(&aw86225->timer, MSEC2TICK(aw86225->duration),
          aw86225_vibrator_timer_func, (wdparm_t)aw86225);
        }
      else if (aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RTP_MODE)
        {
          int budry = aw86225->effect_id -
                      aw86225->config->effect_id_boundary;
          iinfo("activate_mode(AW86225_HAPTIC_ACTIVATE_RTP_MODE) = %d\n",
                aw86225->activate_mode);
          aw86225_haptic_ram_vbat_compensate(aw86225, true);
          aw86225_haptic_play_effect_seq(aw86225, true);

          /* run ms timer */

          iinfo("pattern duration = %d", aw86225->pattern[budry].duration);
          wd_start(&aw86225->timer,
                   MSEC2TICK(aw86225->pattern[budry].duration),
                   aw86225_vibrator_timer_func, (wdparm_t)aw86225);
          aw86225->wk_lock_flag = 1;
        }
      else
        {
          ierr("%s: activate_mode error\n", __func__);
        }
    }
  else
    {
      if (aw86225->wk_lock_flag == 1)
        {
          aw86225->wk_lock_flag = 0;
        }
    }

  nxmutex_unlock(&aw86225->lock);
}

static int aw86225_vibrator_init(FAR struct aw86225 *aw86225)
{
  nxmutex_init(&aw86225->lock);
  nxmutex_init(&aw86225->rtp_lock);
  nxsem_init(&aw86225->wait_q, 0, 0);
  nxsem_init(&aw86225->stop_wait_q, 0, 0);

  return 0;
}

static int aw86225_ram_work_init(FAR struct aw86225 *aw86225)
{
  int ram_timer_val = 8000;

  work_queue(HPWORK, &aw86225->ram_work,
             aw86225_ram_work_routine, aw86225, MSEC2TICK(ram_timer_val));

  return 0;
}

static int aw86225_haptic_init(FAR struct aw86225 *aw86225)
{
  int ret;
  unsigned char i;
  unsigned char reg_val = 0;

  nxmutex_lock(&aw86225->lock);

  /* haptic init */

  aw86225->ram_state = 0;
  aw86225->activate_mode = aw86225->config->mode;
  ret = aw86225_i2c_read(aw86225, AW86225_REG_WAVCFG1, &reg_val);
  aw86225->index = reg_val & 0x7f;
  ret = aw86225_i2c_read(aw86225, AW86225_REG_PLAYCFG2, &reg_val);
  aw86225->gain = reg_val & 0xff;
  iinfo("%s aw86225->gain =0x%02X\n", __func__, aw86225->gain);
  for (i = 0; i < AW86225_SEQUENCER_SIZE; i++)
    {
      ret = aw86225_i2c_read(aw86225, AW86225_REG_WAVCFG1 + i, &reg_val);
      aw86225->seq[i] = reg_val;
    }

  aw86225_haptic_play_mode(aw86225, AW86225_HAPTIC_STANDBY_MODE);
  aw86225_haptic_set_pwm(aw86225, AW86225_PWM_12K);

  /* misc value init */

  aw86225_haptic_misc_para_init(aw86225);

  /* set motor protect */

  aw86225_haptic_swicth_motor_protect_config(aw86225, 0x00, 0x00);
  aw86225_haptic_trig_config(aw86225);
  aw86225_haptic_offset_calibration(aw86225);

  /* config auto_brake */

  aw86225_haptic_auto_bst_enable(aw86225,
                                 aw86225->config->is_enabled_auto_bst);

  /* vbat compensation */

  aw86225_haptic_vbat_mode_config(aw86225,
                                  AW86225_HAPTIC_CONT_VBAT_HW_ADJUST_MODE);
  aw86225->ram_vbat_compensate = AW86225_HAPTIC_RAM_VBAT_COMP_ENABLE;

  /* f0 calibration LRA trim source select register */

  aw86225_i2c_write_bits(aw86225, AW86225_REG_TRIMCFG1,
                         AW86225_BIT_TRIMCFG1_RL_TRIM_SRC_MASK,
                         AW86225_BIT_TRIMCFG1_RL_TRIM_SRC_REG);
  aw86225_haptic_upload_lra(aw86225, AW86225_WRITE_ZERO);
  aw86225_haptic_f0_calibration(aw86225);
  nxmutex_unlock(&aw86225->lock);
  return ret;
}

static int aw86225_haptics_upload_effect(FAR struct ff_lowerhalf_s *lower,
                                         FAR struct ff_effect *effect,
                                         FAR struct ff_effect *old)
{
  FAR struct aw86225 *aw86225 = (FAR struct aw86225 *)lower;
  FAR struct aw86225_hap_play_info *play = &aw86225->play;
  int16_t data[AW86225_CUSTOM_DATA_LEN];
  sclock_t time_us;
  int ret;

  time_us = wd_gettime(&aw86225->timer);
  usleep(time_us);

  iinfo("%s: effect->type=0x%x,FF_CONSTANT=0x%x,FF_PERIODIC=0x%x\n",
        __func__, effect->type, FF_CONSTANT, FF_PERIODIC);

  aw86225->effect_type = effect->type;
  nxmutex_lock(&aw86225->lock);
  while (atomic_load(&aw86225->exit_in_rtp_loop))
    {
      iinfo("%s: goint to waiting rtp exit\n", __func__);
      nxmutex_unlock(&aw86225->lock);
      ret = nxsem_post(&aw86225->stop_wait_q);
      iinfo("%s: wakeup \n", __func__);
      if (ret == -ERESTART)
        {
          ierr("%s: wake up by signal return erro\n", __func__);
          return ret;
        }

      nxmutex_lock(&aw86225->lock);
    }

  if (aw86225->effect_type == FF_CONSTANT)
    {
      /* cont mode set duration */

      aw86225->duration = effect->replay.length;
      aw86225->activate_mode = AW86225_HAPTIC_ACTIVATE_RAM_LOOP_MODE;
      aw86225->effect_id = aw86225->config->effect_id_boundary;
    }
  else if (aw86225->effect_type == FF_PERIODIC)
    {
      if (aw86225->effects_count == 0)
        {
          nxmutex_unlock(&aw86225->lock);
          return -EINVAL;
        }

      memcpy(data, effect->u.periodic.custom_data, sizeof(int16_t) * 3);

      aw86225->effect_id = data[0];
      if (aw86225->effect_id == 521)
        {
          aw86225->effect_id = 21;
        }

      iinfo("%s: aw86225->effect_id = %d \n", __func__, aw86225->effect_id);
      play->vmax_mv = effect->u.periodic.magnitude;

      if (aw86225->effect_id < 0 ||
          aw86225->effect_id > aw86225->config->effect_max)
        {
          nxmutex_unlock(&aw86225->lock);
          return 0;
        }

      if (aw86225->effect_id < aw86225->config->effect_id_boundary)
        {
          aw86225->activate_mode = AW86225_HAPTIC_ACTIVATE_RAM_MODE;
          data[1] = aw86225->predefined[aw86225->effect_id].play_rate_us
                                                            / 1000000;
          data[2] = aw86225->predefined[aw86225->effect_id].play_rate_us
                                                            / 1000;
          iinfo("aw86225->predefined[aw86225->effect_id] \
                 .play_rate_us/1000 = %d\n", \
                 aw86225->predefined[aw86225->effect_id].play_rate_us
                 / 1000);
        }

      if (aw86225->effect_id >= aw86225->config->effect_id_boundary)
        {
          aw86225->activate_mode = AW86225_HAPTIC_ACTIVATE_RTP_MODE;
          iinfo("%s: aw86225->effect_id=%d , aw86225->activate_mode = %d\n",
                 __func__, aw86225->effect_id, aw86225->activate_mode);
          int budry = aw86225->effect_id -
                      aw86225->config->effect_id_boundary;
#ifdef CONFIG_AW86225_RTP_FILE_SUPPORT
          data[1] = aw86225->config->rtp_time[budry] / 1000;
          data[2] = aw86225->config->rtp_time[budry] % 1000;
          iinfo("%s: data[1] = %d data[2] = %d, rtp_time %d\n", __func__,
                 data[1], data[2],
                 aw86225->config->rtp_time[budry]);
#else
          data[1] = 0;
          data[2] = aw86225->pattern[budry].duration;
          iinfo("data[2] = %d, waveloop time = %d\n", data[2],
                aw86225->pattern[budry].duration);
#endif
        }

      memcpy(effect->u.periodic.custom_data, data, sizeof(int16_t) * 3);
    }
  else
    {
      ierr("%s: unsupported effect type: %d\n", __func__, effect->type);
    }

  nxmutex_unlock(&aw86225->lock);
  return 0;
}

static int aw86225_haptics_playback(struct ff_lowerhalf_s *lower,
                                    int effect_id, int val)
{
  FAR struct aw86225 *aw86225 = (FAR struct aw86225 *)lower;

  iinfo("%s: aw86225->effect_id=%d , aw86225->activate_mode = %d\n",
         __func__, aw86225->effect_id, aw86225->activate_mode);

  /* for osc calibration */

  if (val > 0)
    {
      aw86225->state = 1;
    }

  if (val <= 0)
    {
      aw86225->state = 0;
    }

  wd_cancel(&aw86225->timer);
  if (aw86225->effect_type == FF_CONSTANT &&
      aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RAM_LOOP_MODE)
    {
      iinfo("%s: enter ram_loop_mode \n", __func__);
      work_queue(HPWORK, &aw86225->long_vibrate_work,
                 aw86225_long_vibrate_work_routine, aw86225, 0);
    }
  else if (aw86225->effect_type == FF_PERIODIC &&
           aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RAM_MODE)
    {
      iinfo("%s: enter  ram_mode\n", __func__);
      work_queue(HPWORK, &aw86225->long_vibrate_work,
                 aw86225_long_vibrate_work_routine, aw86225, 0);
    }
  else if (aw86225->effect_type == FF_PERIODIC &&
           aw86225->activate_mode == AW86225_HAPTIC_ACTIVATE_RTP_MODE)
    {
      iinfo("%s: enter  rtp_mode\n", __func__);
#ifdef CONFIG_AW86225_RTP_FILE_SUPPORT
      work_queue(HPWORK, &aw86225->rtp_work,
                 aw86225_rtp_work_routine, aw86225, 0);
#else
      work_queue(HPWORK, &aw86225->long_vibrate_work,
                 aw86225_long_vibrate_work_routine, aw86225, 0);
#endif
    }

  return OK;
}

static int aw86225_haptics_erase(FAR struct ff_lowerhalf_s *lower,
                                 int effect_id)
{
  FAR struct aw86225 *aw86225 = (FAR struct aw86225 *)lower;

  /* for osc calibration */

  aw86225->effect_type = 0;
  aw86225->duration = 0;
  return OK;
}

static void aw86225_haptics_set_gain_work_routine(FAR void *arg)
{
  FAR struct aw86225 *aw86225 = arg;

  if (aw86225->new_gain >= 0x7fff)
    {
      aw86225->level = 0x80;
    }
  else if (aw86225->new_gain <= 0x3fff)
    {
      aw86225->level = 0x1e;
    }
  else
    {
      aw86225->level = (aw86225->new_gain - 16383) / 128;
    }

  if (aw86225->level < 0x1e)
    {
      aw86225->level = 0x1e;
    }

  iinfo("%s: set_gain queue work, new_gain = %x level = %x \n", __func__,
          aw86225->new_gain, aw86225->level);

  if (aw86225->ram_vbat_compensate == AW86225_HAPTIC_RAM_VBAT_COMP_ENABLE
      && aw86225->vbat)
    {
      unsigned char comp_level =
      aw86225->level * AW86225_VBAT_REFER / aw86225->vbat;
      if (comp_level > (128 * AW86225_VBAT_REFER / AW86225_VBAT_MIN))
        {
          comp_level = 128 * AW86225_VBAT_REFER / AW86225_VBAT_MIN;
          iinfo("%s: comp level limit is %d ", __func__, comp_level);
        }

      iinfo("%s: enable vbat comp, level = %x comp level = %x", __func__,
             aw86225->level, comp_level);
      aw86225_i2c_write(aw86225, AW86225_REG_PLAYCFG2, comp_level);
    }
  else
    {
      iinfo("%s: disable compsensation, vbat=%d, vbat_min=%d, vbat_ref=%d",
             __func__, aw86225->vbat, AW86225_VBAT_MIN, AW86225_VBAT_REFER);
      aw86225_i2c_write(aw86225, AW86225_REG_PLAYCFG2, aw86225->level);
    }
}

static void aw86225_haptics_set_gain(FAR struct ff_lowerhalf_s *lower,
                                     uint16_t gain)
{
  FAR struct aw86225 *aw86225 = (FAR struct aw86225 *)lower;

  aw86225->new_gain = gain;
  work_queue(HPWORK, &aw86225->set_gain_work,
             aw86225_haptics_set_gain_work_routine, aw86225, 0);
}

/****************************************************************************
 * Public Functions
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
                       FAR const struct aw86225_board_config *config)
{
  FAR struct ff_lowerhalf_s *lower = NULL;
  FAR struct aw86225 *aw86225;
  int effect_count_max;
  int ret;

  DEBUGASSERT(master != NULL && ioedev != NULL && config != NULL);

  aw86225 = kmm_zalloc(sizeof(struct aw86225));
  if (NULL == aw86225)
    {
      ierr("Failed to allocate instance\n\n");
      return -ENOMEM;
    }

  aw86225->effects_count = config->effects_count;
  aw86225->is_used_irq   = config->is_used_irq;
  aw86225->predefined    = config->predefined;
  aw86225->pattern       = config->pattern;
  aw86225->config        = config->config;
  aw86225->irq           = config->irq;
  aw86225->i2c           = master;
  aw86225->addr          = config->addr;
  aw86225->freq          = config->freq;
  aw86225->level         = 0x80;

  lower                  = &aw86225->lower;
  lower->upload          = aw86225_haptics_upload_effect;
  lower->erase           = aw86225_haptics_erase;
  lower->playback        = aw86225_haptics_playback;
  lower->set_gain        = aw86225_haptics_set_gain;
  lower->set_autocenter  = NULL;
  lower->destroy         = NULL;

  /* Interrupt register. */

  ret = IOEXP_SETDIRECTION(ioedev, config->rstpin, IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      ierr("ioexpander set direction error: %d\n", ret);
      goto err;
    }

  ret = IOEXP_WRITEPIN(ioedev, config->rstpin, 0);
  if (ret < 0)
    {
      ierr("ioexpander set direction error: %d\n", ret);
      goto err;
    }

  usleep(2000);

  ret = IOEXP_WRITEPIN(ioedev, config->rstpin, 1);
  if (ret < 0)
    {
      ierr("ioexpander gpio write failed: %d\n", ret);
      goto err;
    }

  usleep(5000);

  /* int pin set */

  ret = IOEXP_SETDIRECTION(ioedev, config->intpin,
                           IOEXPANDER_DIRECTION_IN_PULLUP);
  if (ret < 0)
    {
      ierr("ioexpander set direction error: %d\n", ret);
      goto err;
    }

  ret = IOEXP_SETDIRECTION(ioedev, config->powerpin,
                           IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      ierr("ioexpander set direction error: %d\n", ret);
      goto err;
    }

  ret = IOEXP_WRITEPIN(ioedev, config->powerpin, true);
  if (ret < 0)
    {
      ierr("Error: Failed to write pin: %d\n", ret);
      return ret;
    }

  ret = aw86225_read_chipid(aw86225);
  if (ret < 0)
    {
      ierr("Check chip id failed!\n\n");
      goto err;
    }

  aw86225_vibrator_init(aw86225);
  aw86225_haptic_init(aw86225);
  aw86225_ram_work_init(aw86225);

  set_bit(FF_CUSTOM, lower->ffbit);
  set_bit(FF_GAIN, lower->ffbit);
  set_bit(FF_CONSTANT, lower->ffbit);
  set_bit(FF_PERIODIC, lower->ffbit);

  if (aw86225->effects_count + 1 > FF_EFFECT_COUNT_MAX)
    {
      effect_count_max = aw86225->effects_count + 1;
    }
  else
    {
      effect_count_max = FF_EFFECT_COUNT_MAX;
    }

  ret = ff_register(lower, config->path, effect_count_max);
  if (ret < 0)
    {
      ierr("Failed to register driver:%d\n", ret);
      nxsem_destroy(&aw86225->wait_q);
      nxsem_destroy(&aw86225->stop_wait_q);
      work_cancel(HPWORK, &aw86225->ram_work);
      goto err;
    }

  return ret;

err:
  kmm_free(aw86225);
  return ret;
}
