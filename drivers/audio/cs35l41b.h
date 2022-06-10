/****************************************************************************
 * drivers/audio/cs35l41b.h
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

#ifndef __DRIVERS_AUDIO_CS35L41B_H__
#define __DRIVERS_AUDIO_CS35L41B_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CS35L41_INT1_MASK_DEFAULT                         (0x7ffd7e3f)

/* total size of CS35L41 OTP in bytes */

#define CS35L41_OTP_SIZE_BYTES                            (32 * 4)
#define CS35L41_OTP_MAP_BIT_OFFSET                        (80)

/* Software Reset and Hardware ID */

/* device id register */

#define CS35L41_SW_RESET_DEVID_REG                        (0x0)
#define CS35L41_DEVID                                     (0x35a40)

/* REVID register */

#define CS35L41_SW_RESET_REVID_REG                        (0x4)
#define CS35L41_SW_RESET_REVID_MTLREVID_BITMASK           (0xf)
#define CS35L41_SW_RESET_REVID_AREVID_BITOFFSET           (0x4)
#define CS35L41_SW_RESET_REVID_AREVID_BITMASK             (0xf0)
#define CS35L41_REVID_B2                                  (0xb2)

/* OTPID register */

#define CS35L41_SW_RESET_OTPID_REG                        (0x10)
#define CS35L41_SW_RESET_OTPID_OTPID_BITMASK              (0xf)

/* global enables register */

#define MSM_GLOBAL_ENABLES_REG                            (0x2014)
#define MSM_GLOBAL_ENABLES_GLOBAL_EN_BITMASK              (0x1)

/* block enables register */

#define MSM_BLOCK_ENABLES_REG                             (0x2018)
#define MSM_BLOCK_ENABLES_BST_EN_BITMASK                  (0x30)

/* gpio pad control register */

#define PAD_INTF_GPIO_PAD_CONTROL_REG                     (0x242c)

#define IRQ1_IRQ1_EINT_1_REG                              (0x10010)
#define IRQ1_IRQ1_EINT_1_BST_OVP_ERR_EINT1_BITMASK        (0x40)
#define IRQ1_IRQ1_EINT_1_BST_DCM_UVP_ERR_EINT1_BITMASK    (0x80)
#define IRQ1_IRQ1_EINT_1_BST_SHORT_ERR_EINT1_BITMASK      (0x100)
#define IRQ1_IRQ1_EINT_1_TEMP_WARN_RISE_EINT1_BITMASK     (0x8000)
#define IRQ1_IRQ1_EINT_1_TEMP_ERR_EINT1_BITMASK           (0x20000)
#define IRQ1_IRQ1_EINT_1_MSM_PDN_DONE_EINT1_BITMASK       (0x800000)
#define IRQ1_IRQ1_EINT_1_AMP_ERR_EINT1_BITMASK            (0x80000000)

#define IRQ1_IRQ1_EINT_2_REG                              (0x10014)
#define IRQ1_IRQ1_EINT_2_DSP_VIRTUAL2_MBOX_WR_EINT1_BITMASK \
                                                          (0x200000)

#define IRQ1_IRQ1_MASK_1_REG                              (0x10110)

#define CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG               (0x40)
#define CS35L41_TEST_KEY_CTRL_UNLOCK_1                    (0x00000055)
#define CS35L41_TEST_KEY_CTRL_UNLOCK_2                    (0x000000aa)
#define CS35L41_TEST_KEY_CTRL_LOCK_1                      (0x000000cc)
#define CS35L41_TEST_KEY_CTRL_LOCK_2                      (0x00000033)

#define CS35L41_OTP_IF_OTP_MEM0_REG                       (0x400)
#define CS35L41_OTP_CTRL_OTP_CTRL8_REG                    (0x51c)
#define OTP_CTRL_OTP_CTRL8_OTP_BOOT_DONE_STS_BITMASK      (0x4)

#define DSP_MBOX_DSP_MBOX_2_REG                           (0x13004)
#define DSP_VIRTUAL1_MBOX_DSP_VIRTUAL1_MBOX_1_REG         (0x13020)

#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX1_REG            (0x2b80080)
#define CS35L41_DSP1_SAMPLE_RATE_G1R2                     (0x00000001)

#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX2_REG            (0x2b80088)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX3_REG            (0x2b80090)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX4_REG            (0x2b80098)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX5_REG            (0x2b800a0)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX6_REG            (0x2b800a8)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX7_REG            (0x2b800b0)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_RX8_REG            (0x2b800b8)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX1_REG            (0x2b80280)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX2_REG            (0x2b80288)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX3_REG            (0x2b80290)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX4_REG            (0x2b80298)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX5_REG            (0x2b802a0)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX6_REG            (0x2b802a8)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX7_REG            (0x2b802b0)
#define XM_UNPACKED24_DSP1_SAMPLE_RATE_TX8_REG            (0x2b802b8)

#define XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_REG           (0x2bc1000)
#define XM_UNPACKED24_DSP1_CCM_CORE_CONTROL_DSP1_CCM_CORE_EN_BITMASK \
                                                          (0x1)

#define XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_0_REG          (0x2bc3000)
#define XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_0_REG          (0x2bc3004)
#define XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_0_REG        (0x2bc3008)
#define XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_0_REG          (0x2bc300c)
#define XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_0_REG          (0x2bc3014)
#define XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_1_REG          (0x2bc3018)
#define XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_1_REG          (0x2bc301c)
#define XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_1_REG        (0x2bc3020)
#define XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_1_REG          (0x2bc3024)
#define XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_1_REG          (0x2bc302c)
#define XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_2_REG          (0x2bc3030)
#define XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_2_REG          (0x2bc3034)
#define XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_2_REG        (0x2bc3038)
#define XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_2_REG          (0x2bc303c)
#define XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_2_REG          (0x2bc3044)
#define XM_UNPACKED24_DSP1_MPU_XMEM_ACCESS_3_REG          (0x2bc3048)
#define XM_UNPACKED24_DSP1_MPU_YMEM_ACCESS_3_REG          (0x2bc304c)
#define XM_UNPACKED24_DSP1_MPU_WINDOW_ACCESS_3_REG        (0x2bc3050)
#define XM_UNPACKED24_DSP1_MPU_XREG_ACCESS_3_REG          (0x2bc3054)
#define XM_UNPACKED24_DSP1_MPU_YREG_ACCESS_3_REG          (0x2bc305c)
#define XM_UNPACKED24_DSP1_MPU_LOCK_CONFIG_REG            (0x2bc3140)

#define CS35L41_MIXER_DACPCM1_INPUT_REG                   (0x4c00)
#define CS35L41_MIXER_ASPTX1_INPUT_REG                    (0x4c20)
#define CS35L41_MIXER_ASPTX2_INPUT_REG                    (0x4c24)
#define CS35L41_MIXER_ASPTX3_INPUT_REG                    (0x4c28)
#define CS35L41_MIXER_ASPTX4_INPUT_REG                    (0x4c2c)
#define CS35L41_MIXER_DSP1RX1_INPUT_REG                   (0x4c40)
#define CS35L41_MIXER_DSP1RX2_INPUT_REG                   (0x4c44)
#define CS35L41_MIXER_DSP1RX5_INPUT_REG                   (0x4c50)
#define CS35L41_MIXER_DSP1RX6_INPUT_REG                   (0x4c54)
#define CS35L41_MIXER_DSP1RX7_INPUT_REG                   (0x4c58)
#define CS35L41_MIXER_DSP1RX8_INPUT_REG                   (0x4c5c)

/* CS35L41_INPUT_SRC_
 * Settings for MIXER Source Values
 */

#define CS35L41_INPUT_SRC_ZERO_FILL                       (0x00)
#define CS35L41_INPUT_SRC_DIAG_GEN                        (0x04)
#define CS35L41_INPUT_SRC_ASPRX1                          (0x08)
#define CS35L41_INPUT_SRC_ASPRX2                          (0x09)
#define CS35L41_INPUT_SRC_VMON                            (0x18)
#define CS35L41_INPUT_SRC_IMON                            (0x19)
#define CS35L41_INPUT_SRC_ERR_VOL                         (0x20)
#define CS35L41_INPUT_SRC_CLASSH                          (0x21)
#define CS35L41_INPUT_SRC_VPMON                           (0x28)
#define CS35L41_INPUT_SRC_VBSTMON                         (0x29)
#define CS35L41_INPUT_SRC_DSP1TX1                         (0x32)
#define CS35L41_INPUT_SRC_DSP1TX2                         (0x33)
#define CS35L41_INPUT_SRC_DSP1TX3                         (0x34)
#define CS35L41_INPUT_SRC_DSP1TX4                         (0x35)
#define CS35L41_INPUT_SRC_DSP1TX5                         (0x36)
#define CS35L41_INPUT_SRC_DSP1TX6                         (0x37)
#define CS35L41_INPUT_SRC_DSP1TX7                         (0x38)
#define CS35L41_INPUT_SRC_DSP1TX8                         (0x39)
#define CS35L41_INPUT_SRC_TEMPMON                         (0x3a)
#define CS35L41_INPUT_SRC_RSVD                            (0x3b)

#define IRQ2_IRQ2_EINT_2_REG                              (0x10814)
#define IRQ2_IRQ2_EINT_2_DSP_VIRTUAL1_MBOX_WR_EINT2_BITMASK \
                                                          (0x100000)
#define IRQ2_IRQ2_MASK_2_REG                              (0x10914)
#define IRQ2_IRQ2_MASK_2_DSP_VIRTUAL1_MBOX_WR_MASK2_BITMASK \
                                                          (0x100000)

#define CS35L41_DSP_MBOX_CMD_NONE                         (0)
#define CS35L41_DSP_MBOX_CMD_PAUSE                        (1)
#define CS35L41_DSP_MBOX_CMD_RESUME                       (2)
#define CS35L41_DSP_MBOX_CMD_REINIT                       (3)
#define CS35L41_DSP_MBOX_CMD_STOP_PRE_REINIT              (4)
#define CS35L41_DSP_MBOX_CMD_HIBERNATE                    (5)
#define CS35L41_DSP_MBOX_CMD_OUT_OF_HIBERNATE             (6)
#define CS35L41_DSP_MBOX_CMD_UNKNOWN                      (-1)

/* CS35L41_DSP_MBOX_STATUS_
 * Statuses of the HALO DSP Mailbox
 */

#define CS35L41_DSP_MBOX_STATUS_RUNNING                   (0)
#define CS35L41_DSP_MBOX_STATUS_PAUSED                    (1)
#define CS35L41_DSP_MBOX_STATUS_RDY_FOR_REINIT            (2)
#define CS35L41_DSP_MBOX_STATUS_HIBERNATE                 (3)

/* global sample rate */

#define CS35L41B_GLOBAL_SAMPLE_RATE                       (0x00002c0c)
#define CS36L41B_SAMPLE_RATE_12KHZ                        (0x00000001)
#define CS36L41B_SAMPLE_RATE_24KHZ                        (0x00000002)
#define CS36L41B_SAMPLE_RATE_48KHZ                        (0x00000003)
#define CS36L41B_SAMPLE_RATE_96KHZ                        (0x00000004)
#define CS36L41B_SAMPLE_RATE_192KHZ                       (0x00000005)
#define CS36L41B_SAMPLE_RATE_11P025KHZ                    (0x00000009)
#define CS36L41B_SAMPLE_RATE_22P050KHZ                    (0x0000000a)
#define CS36L41B_SAMPLE_RATE_44P100KHZ                    (0x0000000b)
#define CS36L41B_SAMPLE_RATE_88P200KHZ                    (0x0000000c)
#define CS36L41B_SAMPLE_RATE_176P400KHZ                   (0x0000000d)
#define CS36L41B_SAMPLE_RATE_8KHZ                         (0x00000011)
#define CS36L41B_SAMPLE_RATE_16KHZ                        (0x00000012)
#define CS36L41B_SAMPLE_RATE_32KHZ                        (0x00000013)

/* ASP control2 register */

#define CS35L41B_ASP_CONTROL2_REG                         (0x00004808)

/* bit 0 */

#define CS35L41B_ASP_FSYNC_MSTR_SHIFT                     0
#define CS35L41B_ASP_FSYNC_MASTER_MODE                    \
        (1 << CS35L41B_ASP_FSYNC_MSTR_SHIFT)
#define CS35L41B_ASP_FSYNC_SLAVE_MODE                     \
        (0 << CS35L41B_ASP_FSYNC_MSTR_SHIFT)

/* bit 1 */

#define CS35L41B_ASP_FSYNC_FRC_SHIFT                      1
#define CS35L41B_ASP_FSYNC_FRC_NORMAL                     \
        (0 << CS35L41B_ASP_FSYNC_FRC_SHIFT)
#define CS35L41B_ASP_FSYNC_FRC_ENABLE                     \
        (1 << CS35L41B_ASP_FSYNC_FRC_SHIFT)

/* bit 2 */

#define CS35L41B_ASP_FSYNC_INV_SHIFT                      2
#define CS35L41B_ASP_FSYNC_INV_INVERTED                   \
        (1 << CS35L41B_ASP_FSYNC_INV_SHIFT)
#define CS35L41B_ASP_FSYNC_INV_NOTINVERTED                \
        (0 << CS35L41B_ASP_FSYNC_INV_SHIFT)

/* bit 4 */

#define CS35L41B_ASP_BCLK_MSTR_SHIFT                      4
#define CS35L41B_ASP_BCLK_SLAVE                           \
        (0 << CS35L41B_ASP_BCLK_MSTR_SHIFT)
#define CS35L41B_ASP_BCLK_MASTER                          \
        (1 << CS35L41B_ASP_BCLK_MSTR_SHIFT)

/* bit 5 */

#define CS35L41B_ASP_BCLK_FRC_SHIFT                       5
#define CS35L41B_ASP_BCLK_FRC_NORMAL                      \
        (0 << CS35L41B_ASP_BCLK_FRC_SHIFT)
#define CS35L41B_ASP_BCLK_FRC_ALWAYS                      \
        (1 << CS35L41B_ASP_BCLK_FRC_SHIFT)

/* bit6 */

#define CS35L41B_ASP_BCLK_INV_SHIFT                       6
#define CS35L41B_ASP_BCLK_INV_NINVERTED                   \
        (0 << CS35L41B_ASP_BCLK_INV_SHIFT)
#define CS35L41B_ASP_BCLK_INV_INVERTED                    \
        (1 << CS35L41B_ASP_BCLK_INV_SHIFT)

/* bit[10:8] */

#define CS35L41B_ASP_FMT_SHIFT                            8
#define CS35L41B_ASP_FMT_TDM1                             \
        (0 << CS35L41B_ASP_FMT_SHIFT)
#define CS35L41B_ASP_FMT_TDM1_5                           \
        (4 << CS35L41B_ASP_FMT_SHIFT)
#define CS35L41B_ASP_FMT_I2S                              \
        (2 << CS35L41B_ASP_FMT_SHIFT)

/* bit[23:16] */

#define CS35L41B_ASP_TX_WIDTH_SHIFT                       16
#define CS35L41B_ASP_TX_WIDTH_MASK                        \
        (0xff << CS35L41B_ASP_TX_WIDTH_SHIFT)

#define CS35L41B_ASP_TX_WIDTH_12                          \
        (0x0c << CS35L41B_ASP_TX_WIDTH_SHIFT)

#define CS35L41B_ASP_TX_WIDTH_16                          \
        (0x10 << CS35L41B_ASP_TX_WIDTH_SHIFT)

#define CS35L41B_ASP_TX_WIDTH_24                          \
        (0x18 << CS35L41B_ASP_TX_WIDTH_SHIFT)

#define CS35L41B_ASP_TX_WIDTH_32                          \
        (0x20 << CS35L41B_ASP_TX_WIDTH_SHIFT)

/* bit[31:24] */

#define CS35L41B_ASP_RX_WIDTH_SHIFT                       24
#define CS35L41B_ASP_RX_WIDTH_MASK                        \
        (0xff << CS35L41B_ASP_RX_WIDTH_SHIFT)

#define CS35L41B_ASP_RX_WIDTH_12                          \
        (0x0c << CS35L41B_ASP_RX_WIDTH_SHIFT)

#define CS35L41B_ASP_RX_WIDTH_16                          \
        (0x10 << CS35L41B_ASP_RX_WIDTH_SHIFT)

#define CS35L41B_ASP_RX_WIDTH_24                          \
        (0x18 << CS35L41B_ASP_RX_WIDTH_SHIFT)

#define CS35L41B_ASP_RX_WIDTH_32                          \
        (0x20 << CS35L41B_ASP_RX_WIDTH_SHIFT)

/* ASP control 1 */

#define CS35L41B_ASP_CONTROL1_REG                         0x00004804

/* bit[5:0] */

#define CS35L41B_ASP_BCLK_FREQ_SHIFT                      0
#define CS35L41B_ASP_BCLK_FREQ_MASK                       \
        (0x3f << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_128KHZ                     \
        (0x0c << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_176P4KHZ                   \
        (0x0d << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_192KHZ                     \
        (0x0e << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_256KHZ                     \
        (0x0f << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_384KHZ                     \
        (0x10 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_512KHZ                     \
        (0x12 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_705P6KHZ                   \
        (0x13 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_750KHZ                     \
        (0x14 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_768KHZ                     \
        (0x15 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_1MHZ                       \
        (0x16 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_1P024MHZ                   \
        (0x17 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_1P200MHZ                   \
        (0x18 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_1P4112MHZ                  \
        (0x19 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_1P500MHZ                   \
        (0x1a << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_1P536MHZ                   \
        (0x1b << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_2MHZ                       \
        (0x1c << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_2P048MHZ                   \
        (0x1d << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_2P400MHZ                   \
        (0x1e << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_2P8224MHZ                  \
        (0x1f << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_3MHZ                       \
        (0x20 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_3P072MHZ                   \
        (0x21 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_3P200MHZ                   \
        (0x22 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_4MHZ                       \
        (0x23 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_4P096MHZ                   \
        (0x24 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_4P800MHZ                   \
        (0x25 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_5P644MHZ                   \
        (0x26 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_6MHZ                       \
        (0x27 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_6P144MHZ                   \
        (0x28 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_6P250MHZ                   \
        (0x29 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_6P400MHZ                   \
        (0x2a << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_6P500MHZ                   \
        (0x2b << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_6P750MHZ                   \
        (0x2c << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_7P5264MHZ                  \
        (0x2d << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_8MHZ                       \
        (0x2e << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_8P192MHZ                   \
        (0x2f << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_9P600MHZ                   \
        (0x30 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_11P2896MHZ                 \
        (0x31 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_12MHZ                      \
        (0x32 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_12P288MHZ                  \
        (0x33 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_12P500MHZ                  \
        (0x34 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_12P800MHZ                  \
        (0x35 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_13MHZ                      \
        (0x36 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_13P500MHZ                  \
        (0x37 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_19P200MHZ                  \
        (0x38 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_22P5792MHZ                 \
        (0x39 << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_24MHZ                      \
        (0x3a << CS35L41B_ASP_BCLK_FREQ_SHIFT)

#define CS35L41B_ASP_BCLK_FREQ_24P576MHZ                  \
        (0x3b << CS35L41B_ASP_BCLK_FREQ_SHIFT)

/* REFCLK_INPUT register */

#define CS35L41B_REFCLK_INPUT_REG                         0x00002c04

/* bit[2:0] */

#define CS35L41B_PLL_REFCLK_SEL_SHIFT                     0
#define CS35L41B_PLL_REFCLK_SEL_MASK                      0x07
#define CS35L41B_PLL_REFCLK_SEL_BCLK                      \
        (0 << CS35L41B_PLL_REFCLK_SEL_SHIFT)
#define CS35L41B_PLL_REFCLK_SEL_MCLK                      \
        (5 << CS35L41B_PLL_REFCLK_SEL_SHIFT)
#define CS35L41B_PLL_REFCLK_SEL_FSNC                      \
        (1 << CS35L41B_PLL_REFCLK_SEL_SHIFT)

/* bit4 */

#define CS35L41B_PLL_REFCLK_EN_SHIFT                      4
#define CS35L41B_PLL_REFCLK_EN_MASK                       0x01
#define CS35L41B_PLL_REFCLK_ENABLE                        \
        (0 << CS35L41B_PLL_REFCLK_EN_SHIFT)
#define CS35L41B_PLL_REFCLK_DISABLE                       \
        (1 << CS35L41B_PLL_REFCLK_EN_SHIFT)

/* bit[10:5] */

#define CS35L41B_PLL_REFCLK_FREQ_SHIFT                    5
#define CS35L41B_PLL_REFCLK_FREQ_MASK                     \
        (0x3f << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_32768HZ                  \
        (0x00 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_8000HZ                   \
        (0x01 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_11025HZ                  \
        (0x02 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_12000HZ                  \
        (0x03 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_16000HZ                  \
        (0x04 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_22050HZ                  \
        (0x05 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_24000HZ                  \
        (0x06 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_32000HZ                  \
        (0x07 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_44100HZ                  \
        (0x08 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_48000HZ                  \
        (0x09 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_88200HZ                  \
        (0x0a << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_96000HZ                  \
        (0x0b << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_128000HZ                 \
        (0x0c << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_176400HZ                 \
        (0x0d << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_192000HZ                 \
        (0x0e << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_256000HZ                 \
        (0x0f << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_352800HZ                 \
        (0x10 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_384000HZ                 \
        (0x11 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_512000HZ                 \
        (0x12 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_705600HZ                 \
        (0x13 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_750000HZ                 \
        (0x14 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_768000HZ                 \
        (0x15 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_1000000HZ                \
        (0x16 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_1024000HZ                \
        (0x17 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_1200000HZ                \
        (0x18 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_1411200HZ                \
        (0x19 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_1500000HZ                \
        (0x1a << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_1536000HZ                \
        (0x1b << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_2000000HZ                \
        (0x1c << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_2048000HZ                \
        (0x1d << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_2400000HZ                \
        (0x1e << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_2822400HZ                \
        (0x1f << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_3000000HZ                \
        (0x20 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_3072000HZ                \
        (0x21 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_3200000HZ                \
        (0x22 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_4000000HZ                \
        (0x23 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_4096000HZ                \
        (0x24 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_4800000HZ                \
        (0x25 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_5644800HZ                \
        (0x26 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_6000000HZ                \
        (0x27 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_6144000HZ                \
        (0x28 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_6250000HZ                \
        (0x29 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_6400000HZ                \
        (0x2a << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_6500000HZ                \
        (0x2b << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_6750000HZ                \
        (0x2c << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_7526400HZ                \
        (0x2d << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_8000000HZ                \
        (0x2e << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_8192000HZ                \
        (0x2f << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_9600000HZ                \
        (0x30 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_112896000HZ              \
        (0x31 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_120000000HZ              \
        (0x32 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_122280000HZ              \
        (0x33 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_125000000HZ              \
        (0x34 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_128000000HZ              \
        (0x35 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_130000000HZ              \
        (0x36 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_135000000HZ              \
        (0x37 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_192000000HZ              \
        (0x38 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_225792000HZ              \
        (0x39 << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_240000000HZ              \
        (0x3a << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_245760000HZ              \
        (0x3b << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_250000000HZ              \
        (0x3c << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_256000000HZ              \
        (0x3d << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_260000000HZ              \
        (0x3e << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

#define CS35L41B_PLL_REFCLK_FREQ_270000000HZ              \
        (0x3f << CS35L41B_PLL_REFCLK_FREQ_SHIFT)

/* bit[11] */

#define CS35L41B_PLL_OPEN_LOOP_SHIFT                      11
#define CS35L41B_PLL_OPEN_LOOP_MASK                       \
        (0x01 << CS35L41B_PLL_OPEN_LOOP_SHIFT)
#define CS35L41B_PLL_OPEN_LOOP_CLOSE                      \
        (0 << CS35L41B_PLL_OPEN_LOOP_SHIFT)
#define CS35L41B_PLL_OPEN_LOOP_OPEN                       \
        (1 << CS35L41B_PLL_OPEN_LOOP_SHIFT)

/* bit[16] */

#define CS35L41B_PLL_FORCE_EN_SHIFT                       16
#define CS35L41B_PLL_FORCE_EN_MASK                        \
        (0x01 << CS35L41B_PLL_FORCE_EN_SHIFT)
#define CS35L41B_PLL_FORCE_DISEN                          \
        (0 << CS35L41B_PLL_FORCE_EN_SHIFT)
#define CS35L41B_PLL_FORCE_EN                             \
        (1 << CS35L41B_PLL_FORCE_EN_SHIFT)

/* ASP_ENABLES1 */

#define CS35L41B_ASP_ENABLES1_REG                         0x00004800

/* bit0 */

#define CS35L41B_ASP_TX1_EN_SHIFT                         0
#define CS35L41B_ASP_TX1_EN_MASK                          \
        (1 << CS35L41B_ASP_TX1_EN_SHIFT)
#define CS35L41B_ASP_TX1_EN_ENABLE                        \
        (1 << CS35L41B_ASP_TX1_EN_SHIFT)
#define CS35L41B_ASP_TX1_EN_DISENABLE                     \
        (0 << CS35L41B_ASP_TX1_EN_SHIFT)

/* bit 1 */

#define CS35L41B_ASP_TX2_EN_SHIFT                         1
#define CS35L41B_ASP_TX2_EN_MASK                          \
        (1 << CS35L41B_ASP_TX2_EN_SHIFT)
#define CS35L41B_ASP_TX2_EN_ENABLE                        \
        (1 << CS35L41B_ASP_TX2_EN_SHIFT)
#define CS35L41B_ASP_TX2_EN_DISENABLE                     \
        (0 << CS35L41B_ASP_TX2_EN_SHIFT)

/* bit 2 */

#define CS35L41B_ASP_TX3_EN_SHIFT                         2
#define CS35L41B_ASP_TX3_EN_MASK                          \
        (1 << CS35L41B_ASP_TX3_EN_SHIFT)
#define CS35L41B_ASP_TX3_EN_ENABLE                        \
        (1 << CS35L41B_ASP_TX3_EN_SHIFT)
#define CS35L41B_ASP_TX3_EN_DISENABLE                     \
        (0 << CS35L41B_ASP_TX3_EN_SHIFT)

/* bit 3 */

#define CS35L41B_ASP_TX4_EN_SHIFT                         3
#define CS35L41B_ASP_TX4_EN_MASK                          \
        (1 << CS35L41B_ASP_TX4_EN_SHIFT)
#define CS35L41B_ASP_TX4_EN_ENABLE                        \
        (1 << CS35L41B_ASP_TX4_EN_SHIFT)
#define CS35L41B_ASP_TX4_EN_DISENABLE                     \
        (0 << CS35L41B_ASP_TX4_EN_SHIFT)

/* bit 16 */

#define CS35L41B_ASP_RX1_EN_SHIFT                         16
#define CS35L41B_ASP_RX1_EN_MASK                          \
        (1 << CS35L41B_ASP_RX1_EN_SHIFT)
#define CS35L41B_ASP_RX1_EN_ENABLE                        \
        (1 << CS35L41B_ASP_RX1_EN_SHIFT)
#define CS35L41B_ASP_RX1_EN_DISENABLE                     \
        (0 << CS35L41B_ASP_RX1_EN_SHIFT)

/* bit 17 */

#define CS35L41B_ASP_RX2_EN_SHIFT                         17
#define CS35L41B_ASP_RX2_EN_MASK                          \
        (1 << CS35L41B_ASP_RX2_EN_SHIFT)
#define CS35L41B_ASP_RX2_EN_ENABLE                        \
        (1 << CS35L41B_ASP_RX2_EN_SHIFT)
#define CS35L41B_ASP_RX2_EN_DISENABLE                     \
        (0 << CS35L41B_ASP_RX2_EN_SHIFT)

/* ASP_DATA_CONTROL5 */

#define CS35L41B_ASP_DATA_CONTROL5_REG                    0x00004840

/* bit[5:0] */

#define CS35L41B_ASP_RX_WL_SHIFT                          0
#define CS35L41B_ASP_RX_WL_MASK                           \
        (0x3f << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_12CYCLES                       \
        (0x0c << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_13CYCLES                       \
        (0x0d << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_14CYCLES                       \
        (0x0e << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_15CYCLES                       \
        (0x0f << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_16CYCLES                       \
        (0x10 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_17CYCLES                       \
        (0x11 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_18CYCLES                       \
        (0x12 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_19CYCLES                       \
        (0x13 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_20CYCLES                       \
        (0x14 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_21CYCLES                       \
        (0x15 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_22CYCLES                       \
        (0x16 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_23CYCLES                       \
        (0x17 << CS35L41B_ASP_RX_WL_SHIFT)

#define CS35L41B_ASP_RX_WL_24CYCLES                       \
        (0x18 << CS35L41B_ASP_RX_WL_SHIFT)

/* AMP_GAIN register */

#define CS35L41B_AMP_GAIN_REG                             0x00006c04

/* bit[9:5] */

#define CS35L41B_AMP_GAIN_PCM_SHIFT                       5
#define CS35L41B_AMP_GAIN_PCM_MASK                        \
        (0x1f << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_0P5DB                       \
        (0x00 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_1P5DB                       \
        (0x01 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_2P5DB                       \
        (0x02 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_3P5DB                       \
        (0x03 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_4P5DB                       \
        (0x04 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_5P5DB                       \
        (0x05 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_6P5DB                       \
        (0x06 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_7P5DB                       \
        (0x07 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_8P5DB                       \
        (0x08 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_9P5DB                       \
        (0x09 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_10P5DB                      \
        (0x0a << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_11P5DB                      \
        (0x0b << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_12P5DB                      \
        (0x0c << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_13P5DB                      \
        (0x0d << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_14P5DB                      \
        (0x0e << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_15P5DB                      \
        (0x0f << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_16P5DB                      \
        (0x10 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_17P5DB                      \
        (0x11 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_18P5DB                      \
        (0x12 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_19P5DB                      \
        (0x13 << CS35L41B_AMP_GAIN_PCM_SHIFT)

#define CS35L41B_AMP_GAIN_PCM_20P5DB                      \
        (0x14 << CS35L41B_AMP_GAIN_PCM_SHIFT)

/* GLOBAL_ENABLES register */

#define CS35L41B_GLOBAL_ENABLES_REG                       0x00002014

/* bit 0 */

#define CS35L41B_GLOBAL_EN_SHIFT                          0
#define CS35L41B_GLOBAL_EN_MASK                           \
        (1 << CS35L41B_GLOBAL_EN_SHIFT)
#define CS35L41B_GLOBAL_EN_ENABLE                         \
        (1 << CS35L41B_GLOBAL_EN_SHIFT)
#define CS35L41B_GLOBAL_EN_DISABLE                        \
        (0 << CS35L41B_GLOBAL_EN_SHIFT)

/* global sync register */

#define CS35L41B_GLOBAL_SYNC_REG                          0x00002024
#define CS35L41B_GLOBAL_SYNC_CLK_PHASE                    1
#define CS35L41B_GLOBAL_AMP_MUTE_SHIFT                    4
#define CS35L41B_GLOBAL_AMP_MUTE                          \
        (1 << CS35L41B_GLOBAL_AMP_MUTE_SHIFT)
#define CS35L41B_GLOBAL_AMP_UNMUTE                        \
        (0 << CS35L41B_GLOBAL_AMP_MUTE_SHIFT)

/* Hibernation Power Management */

/* PWRMGT_CTL register */

#define PWRMGT_PWRMGT_CTL                                 (0x2900)

/* WAKESRC_CTL register */

#define PWRMGT_WAKESRC_CTL                                (0x2904)

/*  PWRMGT_STS register */

#define PWRMGT_PWRMGT_STS                                 (0x2908)
#define PWRMGT_PWRMGT_STS_WR_PENDSTS_BITMASK              (0x00000002)

/* block enables2 register */

#define CS35L41B_BLOCK_ENABLES2                           (0x0000201c)
#define CS35L41B_BLOCK_ENABLES2_CLASSH_EN                 (1 << 4)
#define CS35L41B_BLOCK_ENABLES2_SYNC_EN                   (1 << 8)
#define CS35L41B_BLOCK_ENABLES2_VPBR_EN                   (1 << 12)
#define CS35L41B_BLOCK_ENABLES2_VBBR_EN                   (1 << 13)
#define CS35L41B_BLOCK_ENABLES2_AMP_DRE_EN                (1 << 20)
#define CS35L41B_BLOCK_ENABLES2_WKFET_AMP_EN              (1 << 24)

/* IRQ1 Status Bits for Speaker Safe Mode
 *
 * If any of the bits in the mask below are set in IRQ1_EINT_1,
 * the amplifier will have entered Speaker Safe Mode.
 * - b31 - AMP_ERR_MASK1
 * - b17 - TEMP_ERR_MASK1
 * - b8  - BST_SHORT_ERR_MASK1
 * - b7  - BST_DCM_UVP_ERR_MASK1
 * - b6  - BST_OVP_ERR_MASK1
 */

#define CS35L41_INT1_SPEAKER_SAFE_MODE_IRQ_MASK           (0x800201c0)

/* IRQ1 Status Bits for Speaker Safe Mode Boost-related Events
 *
 * If any of the bits in the mask below are set in IRQ1_EINT_1,
 * the amplifier will have entered Speaker Safe Mode
 * and will require additional steps to release from Speaker Safe Mode.
 * - b8 - BST_SHORT_ERR_MASK1
 * - b7 - BST_DCM_UVP_ERR_MASK1
 * - b6 - BST_OVP_ERR_MASK1
 */

#define CS35L41_INT1_BOOST_IRQ_MASK                       (0x000001c0)

/* Toggle Mask for MSM_ERROR_RELEASE_REG to Release from Speaker Safe Mode
 *
 * The relevant fields in MSM_ERROR_RELEASE_REG that require
 * release sequence are:
 * - b6 - TEMP_ERR
 * - b5 - TEMP_WARN
 * - b4 - BST_UVP
 * - b3 - BST_OVP
 * - b2 - BST_SHORT
 * - b1 - AMP_SHORT
 */

#define CS35L41_ERR_RLS_SPEAKER_SAFE_MODE_MASK            (0x0000007e)

#define CS35L41_STATE_UNCONFIGURED                        (0)
#define CS35L41_STATE_CONFIGURED                          (1)
#define CS35L41_STATE_STANDBY                             (2)
#define CS35L41_STATE_POWER_UP                            (3)
#define CS35L41_STATE_ERROR                               (4)
#define CS35L41_STATE_DSP_POWER_UP                        (5)
#define CS35L41_STATE_DSP_STANDBY                         (6)
#define CS35L41_STATE_HIBERNATE                           (7)

#define MSM_ERROR_RELEASE_REG                             (0x2034)

#define CS35L41_EVENT_FLAG_AMP_SHORT                      (0)
#define CS35L41_EVENT_FLAG_OVERTEMP                       (1)
#define CS35L41_EVENT_FLAG_BOOST_INDUCTOR_SHORT           (2)
#define CS35L41_EVENT_FLAG_BOOST_UNDERVOLTAGE             (3)
#define CS35L41_EVENT_FLAG_BOOST_OVERVOLTAGE              (4)
#define CS35L41_EVENT_FLAG_STATE_ERROR                    (5)

/* mode type */

#define CS35L41_ASP_MODE                                  (0)
#define CS35L41_DSP_TUNE_MODE                             (1)
#define CS35L41_DSP_CAL_MODE                              (2)

/* scenario mode */

#define CS35L41B_SCENARIO_SPEAKER                         (0)
#define CS35L41B_SCENARIO_SCO                             (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cs35l41b_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the CS35L41B driver with respect to the board lower half driver).
   *
   * Terminology: Our "lower" half audio instances will be called dev for the
   * publicly visible version and "priv" for the version that only this
   * driver knows.  From the point of view of this driver, it is the board
   * lower "half" that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;       /* CS35L41B audio lower half */

  /* Our specific driver data goes here */

  /* Pointer to the board lower functions */

  const FAR struct cs35l41b_lower_s *lower;
  FAR struct i2c_master_s *i2c;      /* I2C driver to use */

  uint16_t                samprate;  /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t                balance;   /* Current balance level (b16) */
#endif  /* CONFIG_AUDIO_EXCLUDE_BALANCE */
  uint8_t                 volume;    /* Current volume level {0..63} */
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 nchannels; /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;    /* Bits per sample (8 or 16) */
  uint32_t                bclk;      /* IIS BCLK */
  struct work_s           work;      /* Work queue for load firmware */

  FAR struct ioexpander_dev_s *io_dev;      /* Ioexpander device */

  /* is pa calibration value loaded */

  bool                    is_calibrate_value_loaded;

  uint8_t otp_contents[128];         /* Cache storage for OTP contents */
  bool                    initialize;
  int                     state;
  int                     mode;
  uint32_t                asp_gain;
  uint32_t                dsp_gain;
  int                     scenario_mode;
  uint32_t                event_flags;
  bool                    is_running;
  sem_t                   pendsem;

#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
  bool                    dump_dsp_info;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int cs35l41b_read_register(FAR struct cs35l41b_dev_s *priv,
                           uint32_t *regval, uint32_t regaddr);
int cs35l41b_write_register(FAR struct cs35l41b_dev_s *priv,
                            uint32_t regaddr, uint32_t regval);
int cs35l41b_read_block(FAR struct cs35l41b_dev_s *priv,
                        uint32_t regaddr, uint8_t *data,
                        uint32_t len);
int cs35l41b_write_block(FAR struct cs35l41b_dev_s *priv,
                        uint32_t waddr, uint8_t *data,
                        uint32_t len);
#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
void cs35l41b_dump_registers(FAR struct cs35l41b_dev_s *priv,
                             unsigned long arg);
int cs35l41b_debug_set_gain(FAR struct cs35l41b_dev_s *priv, uint32_t gain);
int cs35l41b_debug_get_gain(FAR struct cs35l41b_dev_s *priv, uint32_t *gain);
int cs35l41b_debug_get_mode(FAR struct cs35l41b_dev_s *priv, int *mode);
#endif

int cs35l41b_start_tuning_switch(FAR struct cs35l41b_dev_s *priv);
int cs35l41b_finish_tuning_switch(FAR struct cs35l41b_dev_s *priv);

#endif
