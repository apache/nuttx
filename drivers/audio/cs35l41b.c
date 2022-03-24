/****************************************************************************
 * drivers/audio/cs35l41b.c
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
#include <nuttx/arch.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/cs35l41b.h>

#include "cs35l41b.h"
#include "cs35l41b_fw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_RETRIES                 5

#define ARRAY_SIZE(array)           sizeof(array) / sizeof(array[0])

#define POWER_UP                    0
#define POWER_DOWN                  1
#define POWER_HIBERNATE             2
#define POWER_WAKEUP                3

#define CHANNEL_LEFT                0
#define CHANNEL_RIGHT               1
#define CHANNEL_LEFT_RIGHT          2

#define IO_SET_BYPASS               1
#define IO_CANCEL_BYPASS            2
#define IO_GET_CHIP_ID              3
#define IO_SET_CALIBRATED           4
#define IO_GET_CALIBRATED           5

#define CALIBRATED_STATUS_OK        1
#define CALIBRATED_STATUS_ERROR     2

/* Delay in ms between polling OTP_BOOT_DONE */

#define CS35L41_POLL_OTP_BOOT_DONE_MS                     (10)
#define CS35L41_POLL_OTP_BOOT_DONE_MAX                    (10)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/* Entry in OTP Map of packed bitfield entries */

typedef struct
{
  uint32_t reg;   /* Register address to trim */
  uint8_t shift;  /* Bitwise shift of register bitfield */
  uint8_t size;   /* Bitwise size of register bitfield */
} cs35l41_otp_packed_entry_t;

typedef struct
{
  unsigned long cmd;  /* Command */
  unsigned long data; /* Data */
} cs35l41b_io_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cs35l41b_reset(FAR struct cs35l41b_dev_s *priv);

static int cs35l41b_set_samplerate(FAR struct cs35l41b_dev_s *priv,
                                   uint32_t sample_rate);

static int cs35l41b_set_bit_width(FAR struct cs35l41b_dev_s *priv,
                                  uint32_t width);

static int cs35l41b_set_bclk(FAR struct cs35l41b_dev_s *priv,
                             uint32_t bclk);

static int cs35l41b_set_channel(FAR struct cs35l41b_dev_s *priv,
                                uint32_t channels);

static int cs35l41b_set_gain(FAR struct cs35l41b_dev_s *priv,
                             uint32_t gain);

static int cs35l41b_power(FAR struct cs35l41b_dev_s *priv,
                          uint8_t state);

static int cs35l41b_mute(FAR struct cs35l41b_dev_s *priv,
                         bool state);

static int cs35l41b_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *caps);

static int cs35l41b_shutdown(FAR struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs35l41b_start(FAR struct audio_lowerhalf_s *dev,
               FAR void *session);
#else
static int cs35l41b_start(FAR struct audio_lowerhalf_s *dev);
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs35l41b_configure(FAR struct audio_lowerhalf_s *dev,
                   FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int cs35l41b_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int
  cs35l41b_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#  else
static int cs35l41b_stop(FAR struct audio_lowerhalf_s *dev);
#  endif
#endif

static int cs35l41b_ioctl(FAR struct audio_lowerhalf_s *dev,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  .getcaps        = cs35l41b_getcaps,
  .configure      = cs35l41b_configure,
  .shutdown       = cs35l41b_shutdown,
  .start          = cs35l41b_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop           = cs35l41b_stop,
#endif
  .ioctl          = cs35l41b_ioctl,
};

static const uint32_t g_cs35l41_revb2_errata_patch[] =
{
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,  CS35L41_TEST_KEY_CTRL_UNLOCK_1,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,  CS35L41_TEST_KEY_CTRL_UNLOCK_2,
  0x00004100,                           0x00000000,
  0x00004310,                           0x00000000,
  0x00004400,                           0x00000000,
  0x0000381c,                           0x00000051,
  0x02bc20e0,                           0x00000000,
  0x02bc2020,                           0x00000000,

  /* Unmask IRQs */

  IRQ1_IRQ1_MASK_1_REG,                 CS35L41_INT1_MASK_DEFAULT,

  /* Set GPIO2 for INTb function */

  PAD_INTF_GPIO_PAD_CONTROL_REG,        0x04000000,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,  CS35L41_TEST_KEY_CTRL_LOCK_1,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,  CS35L41_TEST_KEY_CTRL_LOCK_2,
};

static const uint32_t g_cs35l41_mem_lock[] =
{
  XM_UNPACKED24_DSP1_MPU_LOCK_CONFIG_REG,     0x00005555,
  XM_UNPACKED24_DSP1_MPU_LOCK_CONFIG_REG,     0x0000aaaa,
  XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_0_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_0_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_0_REG, 0xffffffff,
  XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_0_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_0_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_1_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_1_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_1_REG, 0xffffffff,
  XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_1_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_1_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_2_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_2_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_2_REG, 0xffffffff,
  XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_2_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_2_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_3_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_3_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_3_REG, 0xffffffff,
  XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_3_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_3_REG,   0xffffffff,
  XM_UNPACKED24_DSP1_MPU_LOCK_CONFIG_REG,     0x00000000
};

static const uint32_t g_cs35l41_frame_sync_regs[] =
{
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX1_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX2_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX3_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX4_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX5_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX6_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX7_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_RX8_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX1_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX2_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX3_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX4_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX5_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX6_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX7_REG,
  XM_UNPACKED24_DSP1_SAMPLE_RATE_TX8_REG
};

static const uint32_t g_cs35l41_pup_patch[] =
{
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_UNLOCK_1,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_UNLOCK_2,
  0x00002084,                          0x002f1aa0,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_LOCK_1,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_LOCK_2,
};

  /* CS35L41 RevB2 OTP Map 1
   * This mapping below maps the OTP bitfields of varying sizes to the
   * Control Port register bitfields OTP is to trim.
   * addr--shift--size
   */

static const cs35l41_otp_packed_entry_t g_otp_map[] =
{
  {0x00002030,    0,      4},     /* TRIM_OSC_FREQ_TRIM */
  {0x00002030,    7,      1},     /* TRIM_OSC_TRIM_DONE */
  {0x0000208c,    24,     6},     /* TST_DIGREG_VREF_TRIM */
  {0x00002090,    14,     4},     /* TST_REF_TRIM */
  {0x00002090,    10,     4},     /* TST_REF_TEMPCO_TRIM */
  {0x0000300c,    11,     4},     /* PLL_LDOA_TST_VREF_TRIM */
  {0x0000394c,    23,     2},     /* BST_ATEST_CM_VOFF */
  {0x00003950,    0,      7},     /* BST_ATRIM_IADC_OFFSET */
  {0x00003950,    8,      7},     /* BST_ATRIM_IADC_GAIN1 */
  {0x00003950,    16,     8},     /* BST_ATRIM_IPKCOMP_OFFSET1 */
  {0x00003950,    24,     8},     /* BST_ATRIM_IPKCOMP_GAIN1 */
  {0x00003954,    0,      7},     /* BST_ATRIM_IADC_OFFSET2 */
  {0x00003954,    8,      7},     /* BST_ATRIM_IADC_GAIN2 */
  {0x00003954,    16,     8},     /* BST_ATRIM_IPKCOMP_OFFSET2 */
  {0x00003954,    24,     8},     /* BST_ATRIM_IPKCOMP_GAIN2 */
  {0x00003958,    0,      7},     /* BST_ATRIM_IADC_OFFSET3 */
  {0x00003958,    8,      7},     /* BST_ATRIM_IADC_GAIN3 */
  {0x00003958,    16,     8},     /* BST_ATRIM_IPKCOMP_OFFSET3 */
  {0x00003958,    24,     8},     /* BST_ATRIM_IPKCOMP_GAIN3 */
  {0x0000395c,    0,      7},     /* BST_ATRIM_IADC_OFFSET4 */
  {0x0000395c,    8,      7},     /* BST_ATRIM_IADC_GAIN4 */
  {0x0000395c,    16,     8},     /* BST_ATRIM_IPKCOMP_OFFSET4 */
  {0x0000395c,    24,     8},     /* BST_ATRIM_IPKCOMP_GAIN4 */
  {0x0000416c,    0,      8},     /* VMON_GAIN_OTP_VAL */
  {0x00004160,    0,      7},     /* VMON_OFFSET_OTP_VAL */
  {0x0000416c,    8,      8},     /* IMON_GAIN_OTP_VAL */
  {0x00004160,    16,     10},    /* IMON_OFFSET_OTP_VAL */
  {0x0000416c,    16,     12},    /* VMON_CM_GAIN_OTP_VAL */
  {0x0000416c,    28,     1},     /* VMON_CM_GAIN_SIGN_OTP_VAL */
  {0x00004170,    0,      6},     /* IMON_CAL_TEMPCO_OTP_VAL */
  {0x00004170,    6,      1},     /* IMON_CAL_TEMPCO_SIGN_OTP */
  {0x00004170,    8,      6},     /* IMON_CAL_TEMPCO2_OTP_VAL */
  {0x00004170,    14,     1},     /* IMON_CAL_TEMPCO2_DN_UPB_OTP_VAL */
  {0x00004170,    16,     9},     /* IMON_CAL_TEMPCO_TBASE_OTP_VAL */
  {0x00004360,    0,      5},     /* TEMP_GAIN_OTP_VAL */
  {0x00004360,    6,      9},     /* TEMP_OFFSET_OTP_VAL */
  {0x00004448,    0,      8},     /* VP_SARADC_OFFSET */
  {0x00004448,    8,      8},     /* VP_GAIN_INDEX */
  {0x00004448,    16,     8},     /* VBST_SARADC_OFFSET */
  {0x00004448,    24,     8},     /* VBST_GAIN_INDEX */
  {0x0000444c,    0,      3},     /* ANA_SELINVREF */
  {0x00006e30,    0,      5},     /* GAIN_ERR_COEFF_0 */
  {0x00006e30,    8,      5},     /* GAIN_ERR_COEFF_1 */
  {0x00006e30,    16,     5},     /* GAIN_ERR_COEFF_2 */
  {0x00006e30,    24,     5},     /* GAIN_ERR_COEFF_3 */
  {0x00006e34,    0,      5},     /* GAIN_ERR_COEFF_4 */
  {0x00006e34,    8,      5},     /* GAIN_ERR_COEFF_5 */
  {0x00006e34,    16,     5},     /* GAIN_ERR_COEFF_6 */
  {0x00006e34,    24,     5},     /* GAIN_ERR_COEFF_7 */
  {0x00006e38,    0,      5},     /* GAIN_ERR_COEFF_8 */
  {0x00006e38,    8,      5},     /* GAIN_ERR_COEFF_9 */
  {0x00006e38,    16,     5},     /* GAIN_ERR_COEFF_10 */
  {0x00006e38,    24,     5},     /* GAIN_ERR_COEFF_11 */
  {0x00006e3c,    0,      5},     /* GAIN_ERR_COEFF_12 */
  {0x00006e3c,    8,      5},     /* GAIN_ERR_COEFF_13 */
  {0x00006e3c,    16,     5},     /* GAIN_ERR_COEFF_14 */
  {0x00006e3c,    24,     5},     /* GAIN_ERR_COEFF_15 */
  {0x00006e40,    0,      5},     /* GAIN_ERR_COEFF_16 */
  {0x00006e40,    8,      5},     /* GAIN_ERR_COEFF_17 */
  {0x00006e40,    16,     5},     /* GAIN_ERR_COEFF_18 */
  {0x00006e40,    24,     5},     /* GAIN_ERR_COEFF_19 */
  {0x00006e44,    0,      5},     /* GAIN_ERR_COEFF_20 */
  {0x00006e48,    0,      10},    /* VOFF_GAIN_0 */
  {0x00006e48,    10,     10},    /* VOFF_GAIN_1 */
  {0x00006e48,    20,     10},    /* VOFF_GAIN_2 */
  {0x00006e4c,    0,      10},    /* VOFF_GAIN_3 */
  {0x00006e4c,    10,     10},    /* VOFF_GAIN_4 */
  {0x00006e4c,    20,     10},    /* VOFF_GAIN_5 */
  {0x00006e50,    0,      10},    /* VOFF_GAIN_6 */
  {0x00006e50,    10,     10},    /* VOFF_GAIN_7 */
  {0x00006e50,    20,     10},    /* VOFF_GAIN_8 */
  {0x00006e54,    0,      10},    /* VOFF_GAIN_9 */
  {0x00006e54,    10,     10},    /* VOFF_GAIN_10 */
  {0x00006e54,    20,     10},    /* VOFF_GAIN_11 */
  {0x00006e58,    0,      10},    /* VOFF_GAIN_12 */
  {0x00006e58,    10,     10},    /* VOFF_GAIN_13 */
  {0x00006e58,    20,     10},    /* VOFF_GAIN_14 */
  {0x00006e5c,    0,      10},    /* VOFF_GAIN_15 */
  {0x00006e5c,    10,     10},    /* VOFF_GAIN_16 */
  {0x00006e5c,    20,     10},    /* VOFF_GAIN_17 */
  {0x00006e60,    0,      10},    /* VOFF_GAIN_18 */
  {0x00006e60,    10,     10},    /* VOFF_GAIN_19 */
  {0x00006e60,    20,     10},    /* VOFF_GAIN_20 */
  {0x00006e64,    0,      10},    /* VOFF_INT1 */
  {0x00007418,    7,      5},     /* DS_SPK_INT1_CAP_TRIM */
  {0x0000741c,    0,      5},     /* DS_SPK_INT2_CAP_TRIM */
  {0x0000741c,    11,     4},     /* DS_SPK_LPF_CAP_TRIM */
  {0x0000741c,    19,     4},     /* DS_SPK_QUAN_CAP_TRIM */
  {0x00007434,    17,     1},     /* FORCE_CAL */
  {0x00007434,    18,     7},     /* CAL_OVERRIDE */
  {0x00007068,    0,      9},     /* MODIX */
  {0x0000410c,    7,      1},     /* VIMON_DLY_NOT_COMB */
  {0x0000400c,    0,      7},     /* VIMON_DLY */
  {0x00000000,    0,      1},     /* extra bit */
  {0x00017040,    0,      8},     /* X_COORDINATE */
  {0x00017040,    8,      8},     /* Y_COORDINATE */
  {0x00017040,    16,     8},     /* WAFER_ID */
  {0x00017040,    24,     8},     /* DVS */
  {0x00017044,    0,      24},    /* LOT_NUMBER */
};

static const uint32_t g_cs35l41_hibernate_patch[] =
{
  IRQ1_IRQ1_MASK_1_REG,                      0xFFFFFFFF,
  IRQ2_IRQ2_EINT_2_REG,                      (1 << 20),
  IRQ1_IRQ1_EINT_2_REG,                      (1 << 21),
  PWRMGT_WAKESRC_CTL,                        0x0044,
  PWRMGT_WAKESRC_CTL,                        0x0144,
  DSP_VIRTUAL1_MBOX_DSP_VIRTUAL1_MBOX_1_REG, CS35L41_DSP_MBOX_CMD_HIBERNATE,
};

static const uint32_t g_cs35l41_pdn_patch[] =
{
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_UNLOCK_1,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_UNLOCK_2,
  0x00002084,                          0x002F1AA3,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_LOCK_1,
  CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG, CS35L41_TEST_KEY_CTRL_LOCK_2,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs35l41b_ioctl
 *
 * Description:
 *   audio driver of ioctl
 *
 ****************************************************************************/

static int cs35l41b_ioctl(FAR struct audio_lowerhalf_s *dev,
                          int cmd, unsigned long arg)
{
  FAR struct cs35l41b_dev_s *priv = (FAR struct cs35l41b_dev_s *)dev;
  FAR struct audio_msg_s *audio_msg = (FAR struct audio_msg_s *)arg;

  if (cmd == AUDIOIOC_SETPARAMTER)
    {
      switch (audio_msg->msg_id)
        {
          case IO_SET_BYPASS:
            if (cs35l41b_write_register(priv,
                                        CS35L41_MIXER_DACPCM1_INPUT_REG,
                                        0x08) == ERROR)
              {
                auderr("write CS35L41_MIXER_DACPCM1_INPUT_REG error\n");
                return ERROR;
              }

            if (cs35l41b_set_gain(priv,
                                  CS35L41B_AMP_GAIN_PCM_10P5DB) == ERROR)
              {
                auderr("cs35l41b_set_gain error\n");
                return ERROR;
              }

            priv->is_bypassed = true;
            break;

          case IO_CANCEL_BYPASS:
            priv->is_bypassed = false;
            if (cs35l41b_write_register(priv,
                                        CS35L41_MIXER_DACPCM1_INPUT_REG,
                                        0x32) == ERROR)
              {
                auderr("write CS35L41_MIXER_DACPCM1_INPUT_REG error\n");
                return ERROR;
              }
            break;

          case IO_GET_CHIP_ID:
            audio_msg->msg_id = IO_GET_CHIP_ID;
            audio_msg->u.data = CS35L41_DEVID;
            break;

          case IO_SET_CALIBRATED:
            priv->is_calibrating = true;
            audio_msg->msg_id = IO_SET_CALIBRATED;

            /* pa is hibernate, wake up first */

            if (priv->is_calibrating)
            {
              if (cs35l41b_power(priv, POWER_WAKEUP) == ERROR)
                {
                  auderr("power wake up error\n");
                  return ERROR;
                }
            }

            /* reload caliberate fw */

            if (cs35l41b_reset(priv) == ERROR)
              {
                audio_msg->u.data = CALIBRATED_STATUS_ERROR;
              }

            audio_msg->u.data = CALIBRATED_STATUS_OK;

            *(FAR struct audio_msg_s *)arg = *audio_msg;

            if (cs35l41b_write_caliberate_ambient(priv, 30) == ERROR)
              {
                return ERROR;
              }
            break;

          case IO_GET_CALIBRATED:
            priv->is_calibrating = false;

            audio_msg->msg_id = IO_GET_CALIBRATED;
            audio_msg->u.data = cs35l41b_get_calibration_result();

            /* cs35l41b power down */

            if (cs35l41b_power(priv, POWER_DOWN) == ERROR)
                {
                  auderr("power wake up error\n");
                  return ERROR;
                }

            /* stop clocks to HALO DSP core */

            if (cs35l41b_write_register(priv,
                XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG, 0) == ERROR)
              {
                auderr("write XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG error\n");
                return ERROR;
              }

            /* dsp load normal fw */

            if (cs35l41_dsp_boot(priv) == ERROR)
              {
                auderr("dsp boot process error\n");
                return ERROR;
              }

            /* reset calibration values load status*/

            priv->is_calibrate_value_loaded = false;

            /* power hibernate */

            if (cs35l41b_power(priv, POWER_HIBERNATE) == ERROR)
                {
                  auderr("power wake up error\n");
                  return ERROR;
                }
            break;

          default:
            auderr("paramter invaild!\n");
            return ERROR;
            break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_getcaps
 *
 * Description:
 *   audio driver of get cs35l41b caps
 *
 ****************************************************************************/

static int cs35l41b_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              caps->ac_controls.b[0] =
                AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
                AUDIO_TYPE_PROCESSING;
              caps->ac_format.hw = (1 << (AUDIO_FMT_PCM - 1));
              break;

            case AUDIO_FMT_MIDI:

              /* We only support Format 0 */

              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.b[0] = AUDIO_SAMP_RATE_44K;
              break;

            case AUDIO_FMT_MP3:
            case AUDIO_FMT_WMA:
            case AUDIO_FMT_PCM:
              break;

            default:
              break;
          }

        break;

      /* Provide capabilities of our FEATURE units */

      case AUDIO_TYPE_FEATURE:

        /* If the sub-type is UNDEF, then report the Feature Units we
         * support.
         */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with the Feature Units we
             * have.
             */

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_BASS |
                                     AUDIO_FU_TREBLE;
            caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
          }
        else
          {
            /* TODO:  Do we need to provide specific info for the Feature
             * Units, such as volume setting ranges, etc.?
             */
          }

        break;

      /* Provide capabilities of our PROCESSING unit */

      case AUDIO_TYPE_PROCESSING:

        switch (caps->ac_subtype)
          {
            case AUDIO_PU_UNDEF:

              /* Provide the type of Processing Units we support */

              caps->ac_controls.b[0] = AUDIO_PU_STEREO_EXTENDER;
              break;

            case AUDIO_PU_STEREO_EXTENDER:

              /* Provide capabilities of our Stereo Extender */

              caps->ac_controls.b[0] =
                AUDIO_STEXT_ENABLE | AUDIO_STEXT_WIDTH;
              break;

            default:

              /* Other types of processing uint we don't support */

              break;
          }
        break;

      /* All others we don't support */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype = 0;
        caps->ac_channels = 0;

        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  return caps->ac_len;
}

/****************************************************************************
 * Name: cs36l41b_configure
 *
 * Description:
 *   audio driver configure cs35l41b
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs36l41b_configure(FAR struct audio_lowerhalf_s *dev,
                   FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int cs35l41b_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct cs35l41b_dev_s *priv = (FAR struct cs35l41b_dev_s *)dev;

  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_FEATURE:
        {
          audinfo("  AUDIO_TYPE_FEATURE\n");

          /* Process based on Feature Unit */

          switch (caps->ac_format.hw)
            {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
              case AUDIO_FU_VOLUME:
                {
                  audinfo("Volume: %d\n", caps->ac_controls.hw[0]);
                }
                break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
              case AUDIO_FU_BASS:
                {
                  audinfo("Bass: %d\n", caps->ac_controls.b[0]);
                }
                break;

              case AUDIO_FU_TREBLE:
                {
                  audinfo("Treble: %d\n", caps->ac_controls.b[0]);
                }
                break;
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

              default:
                {
                  audinfo("Unrecognized feature unit:0x%x\n",
                        caps->ac_format.hw);
                }
                break;
            }
        }
        break;

      case AUDIO_TYPE_OUTPUT:
        {
          audinfo("  AUDIO_TYPE_OUTPUT:\n");
          audinfo("    Number of channels: %u\n", caps->ac_channels);
          audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
          audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);
          audinfo("    Output channel map: 0x%x\n", caps->ac_chmap);

          /* Verify that all of the requested values are supported */

          priv->samprate  = caps->ac_controls.hw[0];
          priv->nchannels = caps->ac_channels;
          priv->bpsamp  = caps->ac_controls.b[2];
          priv->bclk    = caps->ac_controls.hw[0] *
                          (priv->lower->bclk_factor);

          /* if calibrate processing, cs35l41b_configure should not call
           * power wake up, cs35l41b_ioctl will call power wake up before
           * cs35l41b_configure
           */

          if (!priv->is_calibrating)
            {
              if (cs35l41b_power(priv, POWER_WAKEUP) == ERROR)
                {
                  auderr("power wake up error\n");
                  return ERROR;
                }
            }

          if (cs35l41b_set_channel(priv, CHANNEL_LEFT_RIGHT) == ERROR)
            {
              auderr("cs35l41b_set_channel error\n");
              return ERROR;
            }

          if (cs35l41b_set_samplerate(priv, priv->samprate) == ERROR)
            {
              auderr("cs35l41b_set_samplerate error\n");
              return ERROR;
            }

          if (cs35l41b_set_bit_width(priv, priv->bpsamp) == ERROR)
            {
              auderr("cs35l41b_set_bit_width error\n");
              return ERROR;
            }

          if (cs35l41b_set_bclk(priv,
              priv->lower->bclk_factor * priv->samprate) == ERROR)
            {
              auderr("cs35l41b_set_bclk error\n");
              return ERROR;
            }

          /* if set bypass,the gain modify of ioctl */

          if (!priv->is_bypassed)
            {
              if (cs35l41b_set_gain(priv,
                                    CS35L41B_AMP_GAIN_PCM_18P5DB) == ERROR)
                {
                  auderr("cs35l41b_set_gain error\n");
                  return ERROR;
                }
            }
        }
        break;

      case AUDIO_TYPE_PROCESSING:
        {
          audinfo("AUDIO_TYPE_PROCESSING:\n");
        }
        break;
    }

  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: cs35l41b_shutdown
 *
 * Description:
 *   aduio driver of shutdown cs35l41b
 *
 ****************************************************************************/

static int cs35l41b_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct cs35l41b_dev_s *priv = (FAR struct cs35l41b_dev_s *)dev;

  audinfo("cs35l41b shutdown\n");

  if (cs35l41b_mute(priv, true) == ERROR)
    {
      auderr("dsp mute failed\n");
      return ERROR;
    }

  if (cs35l41b_power(priv, POWER_DOWN) == ERROR)
    {
      return ERROR;
    }

  if (cs35l41b_power(priv, POWER_HIBERNATE) == ERROR)
    {
      auderr("power hibernate error!\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_start
 *
 * Description:
 *   audio driver of start cs35l41b
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs35l41b_start(FAR struct audio_lowerhalf_s *dev,
               FAR void *session)
#else
static int cs35l41b_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct cs35l41b_dev_s *priv = (FAR struct cs35l41b_dev_s *)dev;


  if (!priv->done)
    {
      return -EBUSY;
    }

  /* if dsp do not load caliberate that can be load caliberated  value */

  if ((!priv->is_calibrating) && (!priv->is_calibrate_value_loaded))
    {
      if (cs35l41b_load_calibration_value(priv) == ERROR)
        {
          auderr("dsp caliberate error\n");
          return ERROR;
        }

      priv->is_calibrate_value_loaded = true;
    }

  if (cs35l41b_power(priv, POWER_UP) == ERROR)
    {
      auderr("power process failed\n");
      return ERROR;
    }

  if (cs35l41b_mute(priv, false) == ERROR)
    {
      auderr("dsp mute failed\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_stop
 *
 * Description:
 *   audio driver of stop cs35l41b
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int
  cs35l41b_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#  else
static int cs35l41b_stop(FAR struct audio_lowerhalf_s *dev)
#  endif
#endif
{
  FAR struct cs35l41b_dev_s *priv = (FAR struct cs35l41b_dev_s *)dev;

  audinfo("cs35l41b stop!\n");

  if (cs35l41b_mute(priv, true) == ERROR)
    {
      auderr("dsp mute failed\n");
      return ERROR;
    }

  /**
   * if calibrate processing, just calibrate cs35l41b.
   * after calibration, ca35l41b_ioctl will call cs35l41b_reset to reload FW
   * therefore, cs35l41b_stop does not require call hibernate
  */

  if (priv->is_calibrating)
    {
      cs35l41b_calibrate(priv);
    }
  else
    {
      if (cs35l41b_power(priv, POWER_DOWN) == ERROR)
        {
          return ERROR;
        }

      if (cs35l41b_power(priv, POWER_HIBERNATE) == ERROR)
        {
          auderr("power hibernate error!\n");
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_write_errata
 *
 * Description:
 *   cs35l41 write otp errata
 *
 ****************************************************************************/

static int cs35l41_write_errata(FAR struct cs35l41b_dev_s *priv)
{
  int i;
  int ret;

  /* write revb2 errata data */

  for (i = 0; i < ARRAY_SIZE(g_cs35l41_revb2_errata_patch) / 2; i++)
    {
      ret = cs35l41b_write_register(priv,
                                    g_cs35l41_revb2_errata_patch[2 * i],
                                    g_cs35l41_revb2_errata_patch[2 * i + 1]);
      if (ret == ERROR)
        {
          auderr("write g_cs35l41_revb2_errata_patch[%ld] error\n", 2 * i);
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_read_entire_otp_trim_contents
 *
 * Description:
 *   cs35l41b read entire otp trim contents
 *
 ****************************************************************************/

static int
cs35l41b_read_entire_otp_trim_contents(FAR struct cs35l41b_dev_s *priv)
{
  int ret;

  ret = cs35l41b_read_block(priv, CS35L41_OTP_IF_OTP_MEM0_REG,
                            priv->otp_contents, CS35L41_OTP_SIZE_BYTES);

  if (ret != OK)
    {
      auderr("cs35l41b read block error\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_apply_trim_word
 *
 * Description:
 *   cs35l41b apply trim word
 *
 ****************************************************************************/

static int cs35l41_apply_trim_word(uint8_t *otp_mem,
                                   uint32_t bit_count,
                                   uint32_t *reg_val,
                                   uint32_t shift,
                                   uint32_t size)
{
  uint32_t bitmask;
  uint64_t otp_bits;
  uint32_t otp_mem_word_index;
  uint32_t otp_mem_msword_bit_index;

  if ((otp_mem == NULL) || (reg_val == NULL) || (size == 0))
    {
      auderr("paramter invaild\n");
      return ERROR;
    }

  /* Create bit-field mask to use on OTP contents */

  bitmask = ~(0xffffffff << size);

  /* temporary storage of bit-field */

  otp_bits = 0;

  /* Using bit_count, get index of current 32-bit word in otp_mem */

  otp_mem_word_index = bit_count >> 5; /* divide by 32 */

  /* Get position of current bit in the current word in otp_mem */

  otp_mem_msword_bit_index = bit_count - (otp_mem_word_index << 5);

  /* Skip ahead to the current 32-bit word */

  otp_mem += ((otp_mem_word_index) * sizeof(uint32_t));

  /* Shift the first 32-bit word into register - OTP bytes come over
   * I2C in Little-Endian 32-bit words!
   */

  otp_bits |= *(otp_mem++);
  otp_bits <<= 8;
  otp_bits |= *(otp_mem++);
  otp_bits <<= 8;
  otp_bits |= *(otp_mem++);
  otp_bits <<= 8;
  otp_bits |= *(otp_mem++);

  /* If there's bits to get in the second 32-bit word, get them */

  if ((size + otp_mem_msword_bit_index) > 32)
    {
      uint64_t temp_word = 0;
      temp_word |= *(otp_mem++);
      temp_word <<= 8;
      temp_word |= *(otp_mem++);
      temp_word <<= 8;
      temp_word |= *(otp_mem++);
      temp_word <<= 8;
      temp_word |= *(otp_mem++);

      otp_bits |= temp_word << 32;
    }

  /* Right-justify the bits to get from OTP */

  otp_bits >>= otp_mem_msword_bit_index;

  /* Get only required number of OTP bits */

  otp_bits &= bitmask;

  /* Mask off bits in the current register value */

  bitmask <<= shift;
  *reg_val &= ~(bitmask);

  /* Or the OTP bits into the current register value */

  *reg_val |= (otp_bits << shift);

  return OK;
}

/****************************************************************************
 * Name: cs35l41_otp_unpack
 *
 * Description:
 *   cs35l41b otp unpack
 *
 ****************************************************************************/

static int cs35l41_otp_unpack(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t i;
  uint32_t temp_reg_val;
  cs35l41_otp_packed_entry_t temp_trim_entry;
  int ret;

  /* Initialize OTP unpacking state - otp_bit_count
   * There are bits in OTP to skip to reach the trims
   */

  uint16_t otp_bit_count = CS35L41_OTP_MAP_BIT_OFFSET;

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_UNLOCK_1);
  if (ret == ERROR)
    {
      goto error_flags;
    }

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_UNLOCK_2);
  if (ret == ERROR)
    {
      goto error_flags;
    }

  for (i = 0; i < ARRAY_SIZE(g_otp_map); i++)
    {
      temp_trim_entry = g_otp_map[i];

      /* If the entry's 'reg' member is 0x0, it means skip that trim */

      if (temp_trim_entry.reg != 0x00000000)
        {
          temp_reg_val = cs35l41b_read_register(priv, temp_trim_entry.reg);
          if (temp_reg_val < 0)
            {
              ret = ERROR;
              goto error_flags;
            }

          /* Apply OTP trim bit-field to recently read trim register value.
           * OTP contents is saved in
           * cp_read_buffer + CS35L41_CP_REG_READ_LENGTH_BYTES
           */

          cs35l41_apply_trim_word(priv->otp_contents,
                                  otp_bit_count,
                                  &temp_reg_val,
                                  temp_trim_entry.shift,
                                  temp_trim_entry.size);

          /* Write new trimmed register value back */

          ret = cs35l41b_write_register(priv, temp_trim_entry.reg,
                                        temp_reg_val);
          if (ret == ERROR)
            {
              goto error_flags;
            }
        }

      /* Inrement the OTP unpacking state variable otp_bit_count */

      otp_bit_count += temp_trim_entry.size;
    }

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_LOCK_1);
  if (ret == ERROR)
    {
      goto error_flags;
    }

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_LOCK_2);
  if (ret == ERROR)
    {
      goto error_flags;
    }

error_flags:
  if (ret == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_set_samplerate
 *
 * Description:
 *   cs35l41b set sample rate
 *
 ****************************************************************************/

static int cs35l41b_set_samplerate(FAR struct cs35l41b_dev_s *priv,
                                       uint32_t sample_rate)
{
  uint32_t regval;

  switch (sample_rate)
    {
      case 16000:
        regval = CS36L41B_SAMPLE_RATE_16KHZ;
        break;

      case 44100:
        regval = CS36L41B_SAMPLE_RATE_44P100KHZ;
        break;

      case 48000:
        regval = CS36L41B_SAMPLE_RATE_48KHZ;
        break;

      default:
        regval = CS36L41B_SAMPLE_RATE_48KHZ;
        break;
    }

  if (cs35l41b_write_register(priv, CS35L41B_GLOBAL_SAMPLE_RATE,
                              regval) != OK)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_set_bclk
 *
 * Description:
 *   cs35l41b set bclk
 *
 ****************************************************************************/

static int cs35l41b_set_bclk(FAR struct cs35l41b_dev_s *priv,
                                 uint32_t bclk)
{
  uint32_t regval_control;
  uint32_t regval_ccm;
  uint32_t temp;
  int      ret;

  regval_ccm = cs35l41b_read_register(priv, CS35L41B_REFCLK_INPUT_REG);
  if (regval_ccm < 0)
    {
      auderr("read CS35L41B_REFCLK_INPUT_REG error\n");
      return ERROR;
    }

  temp = ~(CS35L41B_PLL_REFCLK_FREQ_MASK);
  temp &= regval_ccm;

  switch (bclk)
  {
    case 512000:
      regval_control = CS35L41B_ASP_BCLK_FREQ_512KHZ;
      temp |= CS35L41B_PLL_REFCLK_FREQ_512000HZ;
      break;

    case 1411200:
      regval_control = CS35L41B_ASP_BCLK_FREQ_1P4112MHZ;
      temp |= CS35L41B_PLL_REFCLK_FREQ_1411200HZ;
      break;

    case 1536000:
      regval_control = CS35L41B_ASP_BCLK_FREQ_1P536MHZ;
      temp |= CS35L41B_PLL_REFCLK_FREQ_1536000HZ;
      break;

    default:
      regval_control = CS35L41B_ASP_BCLK_FREQ_512KHZ;
      temp |= CS35L41B_PLL_REFCLK_FREQ_512000HZ;
      break;
  }

  ret = cs35l41b_write_register(priv, CS35L41B_ASP_CONTROL1_REG,
                                regval_control);
  if (ret == ERROR)
    {
      auderr("write CS35L41B_ASP_CONTROL1_REG error\n");
      return ERROR;
    }

  ret = cs35l41b_write_register(priv, CS35L41B_REFCLK_INPUT_REG, temp);
  if (ret == ERROR)
    {
      auderr("write CS35L41B_REFCLK_INPUT_REG error\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_set_bit_width
 *
 * Description:
 *   cs35l41b set width
 *
 ****************************************************************************/

static int cs35l41b_set_bit_width(FAR struct cs35l41b_dev_s *priv,
                                      uint32_t width)
{
  uint32_t regval;
  uint32_t temp;
  uint32_t temp2;
  int      ret;

  regval = cs35l41b_read_register(priv, CS35L41B_ASP_CONTROL2_REG);
  if (regval < 0)
    {
      auderr("read CS35L41B_ASP_CONTROL2_REG failed\n");
      return ERROR;
    }

  temp   = ~CS35L41B_ASP_RX_WIDTH_MASK;
  temp   &= ~CS35L41B_ASP_TX_WIDTH_MASK;
  temp   &= regval;

  regval = cs35l41b_read_register(priv, CS35L41B_ASP_DATA_CONTROL5_REG);
  if (regval < 0)
    {
      auderr("read CS35L41B_ASP_DATA_CONTROL5_REG failed\n");
      return ERROR;
    }

  temp2  = ~(CS35L41B_ASP_RX_WL_MASK);
  temp2 &= regval;

  switch (width)
    {
      case 16:
        temp |= CS35L41B_ASP_RX_WIDTH_16;
        temp |= CS35L41B_ASP_TX_WIDTH_16;

        temp2 |= CS35L41B_ASP_RX_WL_16CYCLES;
        break;

      case 32:
        temp |= CS35L41B_ASP_RX_WIDTH_32;
        temp |= CS35L41B_ASP_TX_WIDTH_32;
        temp2 |= CS35L41B_ASP_RX_WL_24CYCLES;
        break;

      default:
        temp |= CS35L41B_ASP_RX_WIDTH_32;
        temp |= CS35L41B_ASP_TX_WIDTH_32;
        temp2 |= CS35L41B_ASP_RX_WL_24CYCLES;
        break;
    }

  ret = cs35l41b_write_register(priv, CS35L41B_ASP_CONTROL2_REG, temp);
  if (ret == ERROR)
    {
      auderr("write CS35L41B_ASP_CONTROL2_REG failed\n");
      return ERROR;
    }

  ret = cs35l41b_write_register(priv, CS35L41B_ASP_DATA_CONTROL5_REG, temp2);
  if (ret == ERROR)
    {
      auderr("write CS35L41B_ASP_DATA_CONTROL5_REG failed\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_set_channel
 *
 * Description:
 *   cs35l41b set channel
 *
 ****************************************************************************/

static int cs35l41b_set_channel(FAR struct cs35l41b_dev_s *priv,
                                 uint32_t channels)
{
  uint32_t regval;
  uint32_t temp;

  regval = cs35l41b_read_register(priv, CS35L41B_ASP_ENABLES1_REG);
  if (regval < 0)
    {
      auderr("read CS35L41B_ASP_ENABLES1_REG error\n");
      return ERROR;
    }

  temp = ~CS35L41B_ASP_RX1_EN_MASK;
  temp &= ~CS35L41B_ASP_RX2_EN_MASK;
  temp &= regval;

  temp |= CS35L41B_ASP_RX2_EN_ENABLE;
  temp |= CS35L41B_ASP_RX1_EN_ENABLE;

  if (cs35l41b_write_register(priv, CS35L41B_ASP_ENABLES1_REG, temp) != OK)
    {
      auderr("write CS35L41B_ASP_ENABLES1_REG error\n");
      return ERROR;
    }

  if (channels == CHANNEL_LEFT_RIGHT)
    {
      if (cs35l41b_write_register(priv,
          CS35L41_MIXER_DSP1RX1_INPUT_REG, 0x08) != OK)
        {
          auderr("write CS35L41B_ASP_ENABLES1_REG error\n");
          return ERROR;
        }

      if (cs35l41b_write_register(priv,
          CS35L41_MIXER_DSP1RX2_INPUT_REG, 0x09) != OK)
        {
          auderr("write CS35L41B_ASP_ENABLES1_REG error\n");
          return ERROR;
        }
    }
  else if (channels == CHANNEL_LEFT)
    {
      if (cs35l41b_write_register(priv,
          CS35L41_MIXER_DSP1RX1_INPUT_REG, 0x08) != OK)
        {
          auderr("write CS35L41B_ASP_ENABLES1_REG error\n");
          return ERROR;
        }

      if (cs35l41b_write_register(priv,
          CS35L41_MIXER_DSP1RX2_INPUT_REG, 0x08) != OK)
        {
          auderr("write CS35L41B_ASP_ENABLES1_REG error\n");
          return ERROR;
        }
    }
  else if (channels == CHANNEL_RIGHT)
    {
      if (cs35l41b_write_register(priv,
          CS35L41_MIXER_DSP1RX1_INPUT_REG, 0x09) != OK)
        {
          auderr("write CS35L41B_ASP_ENABLES1_REG error\n");
          return ERROR;
        }

      if (cs35l41b_write_register(priv,
          CS35L41_MIXER_DSP1RX2_INPUT_REG, 0x09) != OK)
        {
          auderr("write CS35L41B_ASP_ENABLES1_REG error\n");
          return ERROR;
        }
    }
  else
    {
      auderr("cs35l41b_set_channel paramter error\n");
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_set_gain
 *
 * Description:
 *   cs35l41b set gain
 *
 ****************************************************************************/

static int cs35l41b_set_gain(FAR struct cs35l41b_dev_s *priv,
                                 uint32_t gain)
{
  uint32_t regval;
  uint32_t temp;

  regval = cs35l41b_read_register(priv, CS35L41B_AMP_GAIN_REG);
  if (regval < 0)
    {
      auderr("read CS35L41B_AMP_GAIN_REG error\n");
      return ERROR;
    }

  temp = ~CS35L41B_AMP_GAIN_PCM_MASK;
  temp &= regval;

  temp |= gain;

  if (cs35l41b_write_register(priv, CS35L41B_AMP_GAIN_REG, temp) == ERROR)
    {
      auderr("write CS35L41B_AMP_GAIN_REG error\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_is_mbox_status_correct
 *
 * Description:
 *   cs35l41b check mbox status
 *
 ****************************************************************************/

static bool cs35l41_is_mbox_status_correct(uint32_t cmd, uint32_t status)
{
  switch (cmd)
    {
      /* For 'NONE' - all statuses are valid */

      case CS35L41_DSP_MBOX_CMD_NONE:

      /* For 'UNKNOWN' - all statuses are valid */

      case CS35L41_DSP_MBOX_CMD_UNKNOWN:
        return true;

      /* For 'PAUSE' - only valid if status is 'PAUSED' */

      case CS35L41_DSP_MBOX_CMD_PAUSE:
        return (status == CS35L41_DSP_MBOX_STATUS_PAUSED);

      /* For 'RESUME' and 'REINIT' - only valid if status is 'RUNNING' */

      case CS35L41_DSP_MBOX_CMD_RESUME:
      case CS35L41_DSP_MBOX_CMD_REINIT:
        return (status == CS35L41_DSP_MBOX_STATUS_RUNNING);

      /* For 'STOP_PRE_REINIT' - only valid if status is 'RDY_FOR_REINIT' */

      case CS35L41_DSP_MBOX_CMD_STOP_PRE_REINIT:
        return (status == CS35L41_DSP_MBOX_STATUS_RDY_FOR_REINIT);

      default:
        return false;
    }
}

/****************************************************************************
 * Name: cs35l41_hibernate
 *
 * Description:
 *   cs35l41b power hibernate mode
 *
 ****************************************************************************/

static int cs35l41_hibernate(FAR struct cs35l41b_dev_s *priv)
{
  int ret;
  int i;

  for (i = 0; i < ARRAY_SIZE(g_cs35l41_hibernate_patch) / 2; i++)
    {
      ret = cs35l41b_write_register(priv,
                                    g_cs35l41_hibernate_patch[2 * i],
                                    g_cs35l41_hibernate_patch[2 * i + 1]);
      if (ret == ERROR)
        {
          auderr("write g_cs35l41_hibernate_patch[%ld] error\n", 2 * i);
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_wait_for_pwrmgt_sts
 *
 * Description:
 *   cs35l41b wait pwrmgt status
 *
 ****************************************************************************/

static int cs35l41_wait_for_pwrmgt_sts(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t i;
  uint32_t wrpend_sts = 0x2;

  for (i = 0;
       (i < 10) && (wrpend_sts & PWRMGT_PWRMGT_STS_WR_PENDSTS_BITMASK);
       i++)
    {
      wrpend_sts = cs35l41b_read_register(priv, PWRMGT_PWRMGT_STS);
      if (wrpend_sts < 0)
        {
          auderr("read PWRMGT_PWRMGT_STS error\n", 2 * i);
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_wake
 *
 * Description:
 *   cs35l41b wake up from hibernate
 *
 ****************************************************************************/

static int cs35l41_wake(FAR struct cs35l41b_dev_s *priv)
{
  int ret;
  int retries;
  uint32_t retval;
  uint32_t timeout;
  uint32_t mbox_cmd_drv_shift = 1 << 20;
  uint32_t mbox_cmd_fw_shift = 1 << 21;

  for (retries = 0; retries < 5; retries++)
    {
      timeout = 10;

      do
        {
          ret = cs35l41b_write_register(
                priv,
                DSP_VIRTUAL1_MBOX_DSP_VIRTUAL1_MBOX_1_REG,
                CS35L41_DSP_MBOX_CMD_OUT_OF_HIBERNATE);

          if (ret == ERROR)
            {
              auderr("write mbox dsp virtual 1 register error\n");
              return ERROR;
            }

          retval = cs35l41b_read_register(priv, DSP_MBOX_DSP_MBOX_2_REG);
          if (retval < 0)
            {
              auderr("read DSP_MBOX_DSP_MBOX_2_REG error\n");
              return ERROR;
            }

          nxsig_usleep(1000 * 2);
        } while (retval != CS35L41_DSP_MBOX_STATUS_PAUSED && --timeout);

      if (timeout != 0)
        {
          break;
        }

      ret = cs35l41_wait_for_pwrmgt_sts(priv);
      if (ret == ERROR)
        {
          auderr("cs35l41_wait_for_pwrmgt_sts error\n");
          return ERROR;
        }

      ret = cs35l41b_write_register(priv, PWRMGT_WAKESRC_CTL, 0x0044);
      if (ret == ERROR)
        {
          auderr("write PWRMGT_WAKESRC_CTL error\n");
          return ERROR;
        }

      ret = cs35l41_wait_for_pwrmgt_sts(priv);
      if (ret == ERROR)
        {
          auderr("cs35l41_wait_for_pwrmgt_sts error\n");
          return ERROR;
        }

      ret = cs35l41b_write_register(priv, PWRMGT_WAKESRC_CTL, 0x0144);
      if (ret == ERROR)
        {
          auderr("write PWRMGT_WAKESRC_CTL error\n");
          return ERROR;
        }

      ret = cs35l41_wait_for_pwrmgt_sts(priv);
      if (ret == ERROR)
        {
          auderr("cs35l41_wait_for_pwrmgt_sts error\n");
          return ERROR;
        }

      ret = cs35l41b_write_register(priv, PWRMGT_PWRMGT_CTL, 0x03);
      if (ret == ERROR)
        {
          auderr("write PWRMGT_PWRMGT_CTL error\n");
          return ERROR;
        }
    }

  ret = cs35l41b_write_register(priv,
                                IRQ2_IRQ2_EINT_2_REG,
                                mbox_cmd_drv_shift);
  if (ret == ERROR)
    {
      auderr("write IRQ2_IRQ2_EINT_2_REG error\n");
      return ERROR;
    }

  ret = cs35l41b_write_register(priv,
                                IRQ1_IRQ1_EINT_2_REG,
                                mbox_cmd_fw_shift);
  if (ret == ERROR)
    {
      auderr("write IRQ1_IRQ1_EINT_2_REG error\n");
      return ERROR;
    }

  ret = cs35l41_write_errata(priv);
  if (ret == ERROR)
    {
      auderr("cs35l41_write_errata error\n");
      return ERROR;
    }

  ret = cs35l41_otp_unpack(priv);
  if (ret == ERROR)
    {
      auderr("cs35l41_otp_unpack error\n");
      return ERROR;
    }

  /* restore boot configuration */

  cs35l41b_set_boot_configuration(priv);

  if (!priv->is_bypassed)
    {
      /* enable dsp output */

      if (cs35l41b_write_register(priv,  0x00004c00, 0x32) == ERROR)
        {
          auderr("write 0x00004c00 error\n");
          return ERROR;
        }

      /* enable dsp fadein feature */

      if (cs35l41b_write_register(priv,  0x00006000, 0x8004) == ERROR)
        {
          auderr("write 0x00006000 error\n");
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_power_down
 *
 * Description:
 *   cs35l41b power down mode
 *
 ****************************************************************************/

static int cs35l41_power_down(FAR struct cs35l41b_dev_s *priv)
{
  int ret;
  uint32_t regval;
  int i;

  /* clear HALO DSP Virtual MBOX 1 IRQ flag */

  ret = cs35l41b_write_register(
        priv,
        IRQ2_IRQ2_EINT_2_REG,
        IRQ2_IRQ2_EINT_2_DSP_VIRTUAL1_MBOX_WR_EINT2_BITMASK);

  if (ret == ERROR)
    {
      auderr("write IRQ2_IRQ2_EINT_2_REG error\n");
      return ERROR;
    }

  /* Clear HALO DSP Virtual MBOX 2 IRQ flag */

  ret = cs35l41b_write_register(
        priv,
        IRQ1_IRQ1_EINT_2_REG,
        IRQ1_IRQ1_EINT_2_DSP_VIRTUAL2_MBOX_WR_EINT1_BITMASK);

  if (ret == ERROR)
    {
      auderr("write IRQ1_IRQ1_EINT_2_REG error\n");
      return ERROR;
    }

  /* read IRQ2 mask register */

  regval = cs35l41b_read_register(
           priv,
           IRQ2_IRQ2_MASK_2_REG);

  if (regval < 0)
    {
      auderr("read IRQ2_IRQ2_MASK_2_REG error:0x%08lx\n", regval);
      return ERROR;
    }

  /* Clear HALO DSP Virtual MBOX 1 IRQ mask */

  regval &= ~(IRQ2_IRQ2_MASK_2_DSP_VIRTUAL1_MBOX_WR_MASK2_BITMASK);
  ret = cs35l41b_write_register(priv,
                                IRQ2_IRQ2_MASK_2_REG,
                                regval);

  if (ret == ERROR)
    {
      auderr("write IRQ2_IRQ2_MASK_2_REG error\n");
      return ERROR;
    }

  /* send HALO DSP MBOX 'Pause' Command */

  ret = cs35l41b_write_register(priv,
                                DSP_VIRTUAL1_MBOX_DSP_VIRTUAL1_MBOX_1_REG,
                                CS35L41_DSP_MBOX_CMD_PAUSE);

  if (regval == ERROR)
    {
      auderr("write DSP_VIRTUAL1_MBOX_DSP_VIRTUAL1_MBOX_1_REG error\n");
      return ERROR;
    }

  for (i = 0; i < 5; i++)
    {
      /* read IRQ2 mask register */

      regval = cs35l41b_read_register(priv, IRQ1_IRQ1_EINT_2_REG);

      if (regval < 0)
        {
          auderr("read IRQ1_IRQ1_EINT_2_REG error\n");
          return ERROR;
        }

      if (regval & IRQ1_IRQ1_EINT_2_DSP_VIRTUAL2_MBOX_WR_EINT1_BITMASK)
        {
          break;
        }

      /* wait for at least 2ms */

      nxsig_usleep(1000 * 2);
    }

  if (i == 5)
    {
      auderr("read IRQ2 mask register 5 times error\n");
      return ERROR;
    }

  /* Clear MBOX IRQ flag */

  ret = cs35l41b_write_register(
        priv,
        IRQ1_IRQ1_EINT_2_REG,
        IRQ1_IRQ1_EINT_2_DSP_VIRTUAL2_MBOX_WR_EINT1_BITMASK);

  if (regval == ERROR)
    {
      auderr("write IRQ1_IRQ1_EINT_2_REG error\n");
      return ERROR;
    }

  /* read IRQ2 Mask register to re-mask HALO DSP Virtual MBOX 1 IRQ */

  regval = cs35l41b_read_register(priv, IRQ2_IRQ2_MASK_2_REG);

  if (regval < 0)
    {
      auderr("read IRQ2_IRQ2_MASK_2_REG error\n");
      return ERROR;
    }

  /* Re-mask HALO DSP Virtual MBOX 1 IRQ */

  regval |= IRQ2_IRQ2_MASK_2_DSP_VIRTUAL1_MBOX_WR_MASK2_BITMASK;
  ret = cs35l41b_write_register(priv, IRQ2_IRQ2_MASK_2_REG, regval);

  if (ret == ERROR)
    {
      auderr("write IRQ2_IRQ2_MASK_2_REG error\n");
      return ERROR;
    }

  /* Read the MBOX status */

  regval = cs35l41b_read_register(priv, DSP_MBOX_DSP_MBOX_2_REG);

  if (regval < 0)
    {
      auderr("read DSP_MBOX_DSP_MBOX_2_REG error\n");
      return ERROR;
    }

  if (cs35l41_is_mbox_status_correct(CS35L41_DSP_MBOX_CMD_PAUSE, regval))
    {
      audwarn("cs35l41b mbox pause status is ok\n");
    }
  else
    {
      auderr("cs35l41b mbox pause status is error\n");
      return ERROR;
    }

  /* Read GLOBAL_EN register in order to clear GLOBAL_EN */

  regval = cs35l41b_read_register(priv, MSM_GLOBAL_ENABLES_REG);

  if (regval < 0)
    {
      auderr("read MSM_GLOBAL_ENABLES_REG error\n");
      return ERROR;
    }

  /* clear global enable */

  regval &= ~(MSM_GLOBAL_ENABLES_GLOBAL_EN_BITMASK);
  ret = cs35l41b_write_register(priv, MSM_GLOBAL_ENABLES_REG, regval);
  if (ret == ERROR)
    {
      auderr("write MSM_GLOBAL_ENABLES_REG error\n");
      return ERROR;
    }

  /* Read IRQ1 flag register to poll MSM_PDN_DONE bit */

  for (i = 0; i < 100; i++)
    {
      regval = cs35l41b_read_register(priv, IRQ1_IRQ1_EINT_1_REG);

      if (regval < 0)
        {
          auderr("read IRQ1_IRQ1_EINT_1_REG error\n");
          return ERROR;
        }

      if (regval & IRQ1_IRQ1_EINT_1_MSM_PDN_DONE_EINT1_BITMASK)
        {
          break;
        }

      /* wait for at least 1ms */

      nxsig_usleep(1000 * 2);
    }

  if (i == 100)
    {
      auderr("read IRQ1 flag register to poll MSM_PDN_DONE bit error\n");
      return ERROR;
    }

  /* Clear MSM_PDN_DONE IRQ flag */

  ret = cs35l41b_write_register(priv, IRQ1_IRQ1_EINT_1_REG,
        IRQ1_IRQ1_EINT_1_MSM_PDN_DONE_EINT1_BITMASK);
  if (ret == ERROR)
    {
      auderr("write IRQ1_IRQ1_EINT_1_REG error\n");
      return ERROR;
    }

  /* send power down patch set */

  for (i = 0; i < ARRAY_SIZE(g_cs35l41_pdn_patch) / 2; i++)
    {
      ret = cs35l41b_write_register(priv,
                                    g_cs35l41_pdn_patch[2 * i],
                                    g_cs35l41_pdn_patch[2 * i + 1]);
      if (ret == ERROR)
        {
          auderr("write g_cs35l41_pdn_patch[%ld] error\n", 2 * i);
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_power_up
 *
 * Description:
 *   cs35l41b power up mode
 *
 ****************************************************************************/

static int cs35l41_power_up(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t regval;
  uint32_t i;
  int      ret = OK;
  uint32_t mbox_cmd = CS35L41_DSP_MBOX_CMD_NONE;

  /* send HALO DSP memory lock sequence */

  for (i = 0; i < ARRAY_SIZE(g_cs35l41_mem_lock) / 2; i++)
    {
      ret = cs35l41b_write_register(priv, g_cs35l41_mem_lock[2 * i],
                                    g_cs35l41_mem_lock[2 * i + 1]);
      if (ret == ERROR)
        {
          auderr("write g_cs35l41_mem_lock[%ld] error\n", 2 * i);
          return ERROR;
        }
    }

  /* Set next HALO DSP Sample Rate register to G1R2 */

  for (i = 0; i < ARRAY_SIZE(g_cs35l41_frame_sync_regs); i++)
    {
      ret = cs35l41b_write_register(priv, g_cs35l41_frame_sync_regs[i],
                                    CS35L41_DSP1_SAMPLE_RATE_G1R2);
      if (ret == ERROR)
        {
          auderr("write g_cs35l41_frame_sync_regs[%ld] error\n", i);
          return ERROR;
        }
    }

  /* Read the HALO DSP CCM control register and
   * enable clocks to HALO DSP core
   */

  regval = cs35l41b_read_register(priv,
           XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG);
  if (regval < 0)
    {
      auderr("read XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG error\n");
      return ERROR;
    }

  regval |=
  XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_DSP1_CCM_CORE_EN_BITMASK;
  ret = cs35l41b_write_register(priv,
        XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG, regval);
  if (ret == ERROR)
    {
      auderr("write XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG error\n");
      return ERROR;
    }

  /* Send Power Up Patch */

  for (i = 0; i < ARRAY_SIZE(g_cs35l41_pup_patch) / 2; i++)
    {
      ret = cs35l41b_write_register(priv, g_cs35l41_pup_patch[2 * i],
                                    g_cs35l41_pup_patch[2 * i + 1]);
      if (ret == ERROR)
        {
          auderr("write g_cs35l41_pup_patch[%ld] error\n", 2 * i);
          return ERROR;
        }
    }

  /* set global enable */

  regval = cs35l41b_read_register(priv, MSM_GLOBAL_ENABLES_REG);
  if (regval < 0)
    {
      auderr("read MSM_GLOBAL_ENABLES_REG error\n");
      return ERROR;
    }

  regval |= MSM_GLOBAL_ENABLES_GLOBAL_EN_BITMASK;
  ret = cs35l41b_write_register(priv, MSM_GLOBAL_ENABLES_REG, regval);
  if (ret == ERROR)
    {
      auderr("write MSM_GLOBAL_ENABLES_REG error\n");
      return ERROR;
    }

  /* wait for 1ms */

  nxsig_usleep(1000 * 1);

  /* clear HALO DSP virtual MBOX 1 IRQ */

  ret = cs35l41b_write_register(priv, IRQ2_IRQ2_EINT_2_REG,
        IRQ2_IRQ2_EINT_2_DSP_VIRTUAL1_MBOX_WR_EINT2_BITMASK);
  if (ret == ERROR)
    {
      auderr("write IRQ2_IRQ2_EINT_2_REG error\n");
      return ERROR;
    }

  /* clear HALO DSP virtual MBOX 2 IRQ */

  ret = cs35l41b_write_register(priv, IRQ1_IRQ1_EINT_2_REG,
        IRQ1_IRQ1_EINT_2_DSP_VIRTUAL2_MBOX_WR_EINT1_BITMASK);
  if (ret == ERROR)
    {
      auderr("write IRQ1_IRQ1_EINT_2_REG error\n");
      return ERROR;
    }

  /* Read IRQ2 Mask register and
   * unmask IRQ for HALO DSP virtual MBOX 1
   */

  regval = cs35l41b_read_register(priv, IRQ2_IRQ2_MASK_2_REG);
  if (regval < 0)
    {
      auderr("read IRQ2_IRQ2_MASK_2_REG error\n");
       return ERROR;
    }

  regval &= ~(IRQ2_IRQ2_MASK_2_DSP_VIRTUAL1_MBOX_WR_MASK2_BITMASK);
  ret = cs35l41b_write_register(priv, IRQ2_IRQ2_MASK_2_REG, regval);
  if (ret == ERROR)
    {
      auderr("write IRQ2_IRQ2_MASK_2_REG error\n");
      return ERROR;
    }

  /* Read HALO DSP MBOX Space 2 register */

  regval = cs35l41b_read_register(priv, DSP_MBOX_DSP_MBOX_2_REG);
  if (regval < 0)
    {
      auderr("read DSP_MBOX_DSP_MBOX_2_REG error\n");
      return ERROR;
    }

  switch (regval)
    {
      case CS35L41_DSP_MBOX_STATUS_RDY_FOR_REINIT:
        mbox_cmd = CS35L41_DSP_MBOX_CMD_REINIT;
        break;

      case CS35L41_DSP_MBOX_STATUS_PAUSED:
      case CS35L41_DSP_MBOX_STATUS_RUNNING:
        mbox_cmd = CS35L41_DSP_MBOX_CMD_RESUME;
        break;

      default:
        break;
    }

  if (mbox_cmd == CS35L41_DSP_MBOX_CMD_NONE)
    {
      auderr("mbox cmd failed!!!!!\n");
      return ERROR;
    }

  /* Write MBOX command */

  ret = cs35l41b_write_register(priv,
        DSP_VIRTUAL1_MBOX_DSP_VIRTUAL1_MBOX_1_REG, mbox_cmd);
  if (ret == ERROR)
    {
      auderr("write dsp virtual mbox 1 register error\n");
      return ERROR;
    }

  for (i = 0; i < 5; i++)
    {
      regval = cs35l41b_read_register(priv, IRQ1_IRQ1_EINT_2_REG);
      if (regval < 0)
        {
          auderr("write IRQ1_IRQ1_EINT_2_REG error\n");
          return ERROR;
        }

      /* wait for 1ms */

      nxsig_usleep(1000 * 1);

      if (regval & IRQ1_IRQ1_EINT_2_DSP_VIRTUAL2_MBOX_WR_EINT1_BITMASK)
        {
          break;
        }
    }

  if (i == 5)
    {
      auderr("read IRQ1_IRQ1_EINT_2_REG error!!\n");
      return ERROR;
    }

  /* Clear MBOX IRQ */

  ret = cs35l41b_write_register(priv, IRQ1_IRQ1_EINT_2_REG,
        IRQ1_IRQ1_EINT_2_DSP_VIRTUAL2_MBOX_WR_EINT1_BITMASK);
  if (ret == ERROR)
    {
      auderr("write IRQ1_IRQ1_EINT_2_REG error\n");
      return ERROR;
    }

  /* Read IRQ2 Mask register to next re-mask the MBOX IRQ */

  regval = cs35l41b_read_register(priv, IRQ2_IRQ2_MASK_2_REG);
  if (regval < 0)
    {
      auderr("read IRQ2_IRQ2_MASK_2_REG error\n");
      return ERROR;
    }

  regval |= IRQ2_IRQ2_MASK_2_DSP_VIRTUAL1_MBOX_WR_MASK2_BITMASK;
  ret = cs35l41b_write_register(priv, IRQ2_IRQ2_MASK_2_REG, regval);
  if (ret == ERROR)
    {
      auderr("write IRQ2_IRQ2_MASK_2_REG error\n");
      return ERROR;
    }

  /* Read the HALO DSP MBOX status */

  regval = cs35l41b_read_register(priv, DSP_MBOX_DSP_MBOX_2_REG);
  if (regval < 0)
    {
      auderr("read DSP_MBOX_DSP_MBOX_2_REG error\n");
      return ERROR;
    }

  if (cs35l41_is_mbox_status_correct(mbox_cmd, regval))
    {
      audinfo("cs35l41b mbox status is ok\n");
    }
  else
    {
      auderr("cs35l41b mbox status is error\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_power
 *
 * Description:
 *   cs35l41b power process
 *
 ****************************************************************************/

static int cs35l41b_power(FAR struct cs35l41b_dev_s *priv,
                          uint8_t state)
{
  int ret;

  switch (state)
    {
      case POWER_UP:

        if (priv->power_state != CS35L41_STATE_STANDBY &&
            priv->power_state != CS35L41_STATE_DSP_STANDBY)
          {
            auderr("power status is not standby or dsp_standby!\n");
            return ERROR;
          }

        if (priv->power_state == CS35L41_STATE_STANDBY)
          {
            priv->power_state = CS35L41_STATE_POWER_UP;
          }
        else
          {
            priv->power_state = CS35L41_STATE_DSP_POWER_UP;
          }

        if (cs35l41_power_up(priv) == ERROR)
          {
            auderr("cs35l41b power up failed!\n");
            return ERROR;
          }
        break;

      case POWER_DOWN:
        if (priv->power_state != CS35L41_STATE_POWER_UP &&
            priv->power_state != CS35L41_STATE_DSP_POWER_UP)
          {
            auderr("power status is not power up or dsp_power_up!\n");
            return ERROR;
          }

          if (cs35l41_power_down(priv) == ERROR)
            {
              auderr("power down failed!\n");
              return ERROR;
            }

          priv->power_state = CS35L41_STATE_DSP_STANDBY;
        break;

      case POWER_HIBERNATE:
        if (priv->power_state == CS35L41_STATE_DSP_STANDBY)
          {
            ret = cs35l41_hibernate(priv);
            if (ret == ERROR)
              {
                auderr("cs35l41b hibernate failed!\n");
                return ERROR;
              }
            priv->power_state = CS35L41_STATE_HIBERNATE;
          }
        else
          {
            auderr("power status is not dsp standby!\n");
            return ERROR;
          }
        break;

      case POWER_WAKEUP:
        if (priv->power_state == CS35L41_STATE_HIBERNATE)
          {
            if (cs35l41_wake(priv) == ERROR)
              {
                auderr("cs35l41b wake up failed!\n");
                return ERROR;
              }
            priv->power_state = CS35L41_STATE_DSP_STANDBY;
          }
        else
          {
            auderr("power status is not hibernate!\n");
            return ERROR;
          }
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_check_id
 *
 * Description:
 *   cs35l41b check ID
 *
 ****************************************************************************/

static int cs35l41b_check_id(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t regval;
  uint32_t i;

  for (i = 0; i < CS35L41_POLL_OTP_BOOT_DONE_MAX; i++)
    {
      regval = cs35l41b_read_register(priv, CS35L41_OTP_CTRL_OTP_CTRL8_REG);
      if (regval & OTP_CTRL_OTP_CTRL8_OTP_BOOT_DONE_STS_BITMASK)
        {
          break;
        }

      if (i == CS35L41_POLL_OTP_BOOT_DONE_MAX)
        {
          auderr("cs35l41b read OTP control register failed\n");
          return ERROR;
        }

      /* wait for 10ms */

      nxsig_usleep(1000 * CS35L41_POLL_OTP_BOOT_DONE_MS);
    }

  /* DEVID */

  regval = cs35l41b_read_register(priv, CS35L41_SW_RESET_DEVID_REG);
  if ((regval & CS35L41_DEVID) != CS35L41_DEVID)
    {
      auderr("cs35l41b DEVID:0x%08lx\n error", regval);
      return ERROR;
    }

  /* REVID */

  regval = cs35l41b_read_register(priv, CS35L41_SW_RESET_REVID_REG);
  if (regval != CS35L41_REVID_B2)
    {
      auderr("cs35l41b REVID:0x%08lx\n error", regval);
      return ERROR;
    }

  /* OTPID */

  regval = cs35l41b_read_register(priv, CS35L41_SW_RESET_OTPID_REG);
  regval &= CS35L41_SW_RESET_OTPID_OTPID_BITMASK;
  if ((regval != 0x01) && (regval != 0x08))
    {
      auderr("cs35l41b OTPID:0x%08lx\n error", regval);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_reset
 *
 * Description:
 *   cs35l41b reset
 *
 ****************************************************************************/

static int cs35l41b_reset(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t i;
  int  ret;

  /* write revb2 errata data */

  for (i = 0; i < ARRAY_SIZE(g_cs35l41_revb2_errata_patch) / 2; i++)
    {
      ret = cs35l41b_write_register(priv,
                                    g_cs35l41_revb2_errata_patch[2 * i],
                                    g_cs35l41_revb2_errata_patch[2 * i + 1]);
      if (ret == ERROR)
        {
          auderr("write g_cs35l41_revb2_errata_patch[%ld] error\n", 2 * i);
          return ERROR;
        }
    }

  /* read otp contents */

  if (cs35l41b_read_entire_otp_trim_contents(priv) == ERROR)
    {
      auderr("read_entire_otp_trim_contents error\n");
      return ERROR;
    }

  /* otp unpack */

  if (cs35l41_otp_unpack(priv) == ERROR)
    {
      auderr("cs35l41_otp_unpack error\n");
      return ERROR;
    }

  /* stop clocks to HALO DSP core */

  if (cs35l41b_write_register(priv,
      XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG,
      0) == ERROR)
    {
      auderr("write XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG error\n");
      return ERROR;
    }

  /* dsp boot */

  if (cs35l41_dsp_boot(priv) == ERROR)
    {
      auderr("dsp boot process error\n");
      return ERROR;
    }

  /* cs45l41b reset and reset caliberate value load state */

  priv->is_calibrate_value_loaded = false;

  /* if pa is first initialization,
   * should be load caliberation value before power up.
   */

  if ((!priv->is_calibrating) && (!priv->is_calibrate_value_loaded))
    {
      if (cs35l41b_load_calibration_value(priv) == ERROR)
        {
          auderr("dsp caliberate value load failed\n");
          return ERROR;
        }

      priv->is_calibrate_value_loaded = true;
    }

  /* enable dsp output */

  if (cs35l41b_write_register(priv,  0x00004c00, 0x32) == ERROR)
    {
      auderr("write 0x00004c00 error\n");
      return ERROR;
    }

  /* enable dsp fadein feature */

  if (cs35l41b_write_register(priv,  0x00006000, 0x8004) == ERROR)
    {
      auderr("write 0x00006000 error\n");
      return ERROR;
    }

  priv->power_state = CS35L41_STATE_DSP_STANDBY;

  /* if calibrate processing, should not enter hibernate mode */

  if (!priv->is_calibrating)
  {
    if (cs35l41b_mute(priv, true) == ERROR)
      {
        auderr("dsp mute failed\n");
        return ERROR;
      }

    if (cs35l41b_power(priv, POWER_UP) == ERROR)
      {
        auderr("power process failed\n");
        return ERROR;
      }

    priv->power_state = CS35L41_STATE_DSP_POWER_UP;

    if (cs35l41b_power(priv, POWER_DOWN) == ERROR)
      {
        auderr("cs45l41b power down failed!\n");
        return ERROR;
      }

    if (cs35l41b_power(priv, POWER_HIBERNATE) == ERROR)
      {
        auderr("cs45l41b power hibernate failed!\n");
        return ERROR;
      }
  }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_mute
 *
 * Description:
 *   cs35l41b mute options
 *
 ****************************************************************************/

static int cs35l41b_mute(FAR struct cs35l41b_dev_s *priv,
                         bool state)
{
  uint32_t regval;

  regval = cs35l41b_read_register(priv, CS35L41B_GLOBAL_SYNC_REG);
  if (regval < 0)
    {
      auderr("cs35l41b read CS35L41B_GLOBAL_SYNC_REG  error\n");
      return ERROR;
    }

  if (state)
    {
      regval |= CS35L41B_GLOBAL_AMP_MUTE;
    }
  else
    {
      regval &= ~CS35L41B_GLOBAL_AMP_MUTE;
    }

  if (cs35l41b_write_register(priv,
                              CS35L41B_GLOBAL_SYNC_REG,
                              regval) == ERROR)
    {
      auderr("write CS35L41B_GLOBAL_SYNC_REG error\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_write_register
 *
 * Description:
 *   cs35l41b write register
 *
 ****************************************************************************/

int cs35l41b_write_register(FAR struct cs35l41b_dev_s *priv,
                            uint32_t regaddr, uint32_t regval)
{
  struct i2c_config_s config;
  int retries;
  int ret = OK;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      uint8_t data[8];

      /* Set up the data to write */

      data[0] = (regaddr >> 24) & 0xff;
      data[1] = (regaddr >> 16) & 0xff;
      data[2] = (regaddr >> 8) & 0xff;
      data[3] = regaddr & 0xff;

      data[4] = (regval >> 24) & 0xff;
      data[5] = (regval >> 16) & 0xff;
      data[6] = (regval >> 8) & 0xff;
      data[7] = regval & 0xff;

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = i2c_write(priv->i2c, &config, data, 8);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
          audinfo("retries=0x%08x regaddr=0x%08lx\n", retries, regaddr);
        }
      else
        {
          break;
        }
    }

  if (retries == MAX_RETRIES)
    {
      auderr("I2C write process failed\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_read_register
 *
 * Description:
 *   cs35l41b read register
 *
 ****************************************************************************/

int32_t cs35l41b_read_register(FAR struct cs35l41b_dev_s *priv,
                               uint32_t regaddr)
{
  int retries;
  struct i2c_config_s config;
  uint8_t buffer[4];
  uint8_t data[4];
  int ret = OK;

  buffer[0] = (regaddr >> 24) & 0xff;
  buffer[1] = (regaddr >> 16) & 0xff;
  buffer[2] = (regaddr >> 8) & 0xff;
  buffer[3] = regaddr & 0xff;

  config.address   = priv->lower->address;
  config.frequency = priv->lower->frequency;
  config.addrlen   = 7;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = i2c_writeread(priv->i2c, &config,
                         (FAR const uint8_t *)&buffer, 4, data, 4);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          int32_t regval;

          /* The I2C transfer was successful... break out of the loop and
           * return the value read.
           */

          regval = ((int32_t)data[0] << 24) | (int32_t)data[1] << 16 |
                   ((int32_t)data[2] << 8) | (int32_t)data[3];
          return regval;
        }

        audinfo("retries=0x%08x regaddr=0x%08lx\n", retries, regaddr);
    }

  return ERROR;
}

/****************************************************************************
 * Name: cs35l41b_read_block
 *
 * Description:
 *   cs35l41b read block data
 *
 ****************************************************************************/

int cs35l41b_read_block(FAR struct cs35l41b_dev_s *priv,
                        uint32_t regaddr, uint8_t *data,
                        uint32_t len)
{
  int retries;
  struct i2c_config_s config;
  uint8_t buffer[4];
  int ret = OK;

  buffer[0] = (regaddr >> 24) & 0xff;
  buffer[1] = (regaddr >> 16) & 0xff;
  buffer[2] = (regaddr >> 8) & 0xff;
  buffer[3] = regaddr & 0xff;

  config.address   = priv->lower->address;
  config.frequency = priv->lower->frequency;
  config.addrlen   = 7;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = i2c_writeread(priv->i2c, &config,
                         (FAR const uint8_t *)&buffer, 4, data, len);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          return OK;
        }

      audinfo("retries=0x%08x regaddr=0x%08lx\n", retries, regaddr);
    }

  return ERROR;
}

/****************************************************************************
 * Name: cs35l41b_write_block
 *
 * Description:
 *   cs35l41b write block data
 *
 ****************************************************************************/

int cs35l41b_write_block(FAR struct cs35l41b_dev_s *priv,
                         uint32_t waddr, uint8_t *data,
                         uint32_t len)
{
  struct i2c_config_s config;
  int retries;
  int ret = OK;
  struct i2c_msg_s msg[2];
  uint8_t temp[4];

  temp[0] = (waddr >> 24) & 0xff;
  temp[1] = (waddr >> 16) & 0xff;
  temp[2] = (waddr >> 8) & 0xff;
  temp[3] = waddr & 0xff;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  msg[0].frequency = config.frequency,
  msg[0].addr      = config.address;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = (FAR uint8_t *)temp;  /* Override const */
  msg[0].length    = 4;

  msg[1].frequency = config.frequency,
  msg[1].addr      = config.address;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (FAR uint8_t *)data;  /* Override const */
  msg[1].length    = len;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, (FAR struct i2c_msg_s *)&msg, 2);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
          audinfo("retries=0x%08x\n", retries);
        }
      else
        {
          break;
        }
    }

  if (retries == MAX_RETRIES)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs35l41b_late_initialize
 *
 * Description:
 *   Late initializa cs35l41b smart pa
 *
 ****************************************************************************/

int cs35l41b_late_initialize(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct cs35l41b_dev_s *priv = (FAR struct cs35l41b_dev_s *)dev;

  if (cs35l41b_reset(priv) == OK)
    {
      priv->done = true;
      return OK;
    }

  auderr("ERROR: cs35l41b reset failed!\n");
  return ERROR;
}

/****************************************************************************
 * Name: cs35l41b_initialize
 *
 * Description:
 *   Initializa cs35l41b smart pa
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
cs35l41b_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct cs35l41b_lower_s *lower)
{
  FAR struct cs35l41b_dev_s *priv;
  int16_t ret = OK;

  if (lower == NULL)
    {
      auderr("ERROR: lower is NULL\n");
      return NULL;
    }

  ret = lower->power_en(true);
  if (ret < 0)
    {
      auderr("ERROR: CS35L41B power_en\n");
      return NULL;
    }

  ret = lower->reset_en(true);
  if (ret < 0)
    {
      auderr("ERROR: CS35L41B reset_en\n");
      return NULL;
    }

  ret = lower->int_pin_set();
  if (ret < 0)
    {
      auderr("ERROR: CS35L41B reset_en\n");
      return NULL;
    }

  /* wait 2 ms for stable */

  up_mdelay(2);

  /* Allocate a CS35L41B device structure */

  priv = kmm_zalloc(sizeof(struct cs35l41b_dev_s));

  if (priv)
    {
      priv->dev.ops     = &g_audioops;
      priv->lower       = lower;
      priv->i2c         = i2c;
      priv->is_bypassed = false;

      if (cs35l41b_check_id(priv) != OK)
        {
          auderr("cs35l41b check id failed!\n");
          goto errout_with_dev;
        }

      return &priv->dev;
    }

errout_with_dev:
  kmm_free(priv);
  return NULL;
}
