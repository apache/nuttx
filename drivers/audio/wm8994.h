/****************************************************************************
 * drivers/audio/wm8994.h
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

#ifndef __DRIVERS_AUDIO_WM8994_H
#define __DRIVERS_AUDIO_WM8994_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>

#include <nuttx/compiler.h>

#include <pthread.h>

#include <nuttx/mqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* So far, I have not been able to get FLL lock interrupts. Worse, I have
 * been able to get the FLL to claim that it is locked at all even when
 * polling.  What am I doing wrong?
 *
 * Hmmm.. seems unnecessary anyway
 */

#undef WM8994_USE_FFLOCK_INT
#undef WM8994_USE_FFLOCK_POLL

/* Registers Addresses */

#define WM8994_SWRST                              0x00                                        /* SW Reset and ID */
#define WM8994_ID                                 0x00                                        /* SW Reset and ID */

#define WM8994_PM1                                0x01                                        /* Power Management */
#define WM8994_PM2                                0x02                                        /* Power Management */
#define WM8994_PM3                                0x03                                        /* Power Management */
#define WM8994_PM4                                0x04                                        /* Power Management */
#define WM8994_PM5                                0x05                                        /* Power Management */
#define WM8994_PM6                                0x06                                        /* Power Management */

#define WM8994_INPUT_MIXER1                       0x15                                        /* Input Mixer (1) */

#define WM8994_LEFTLINE_12_VOL                    0x18                                        /* Left Line Input 1&2 Volume */
#define WM8994_LEFTLINE_34_VOL                    0x19                                        /* Left Line Input 3&4 Volume */
#define WM8994_RIGHTLINE_12_VOL                   0x1A                                        /* Right Line Input 1&2 Volume */
#define WM8994_RIGHTLINE_34_VOL                   0x1B                                        /* Right Line Input 3&4 Volume */

#define WM8994_LEFT_OUTPUT_VOL                    0x1C                                        /* Left Output Volume */
#define WM8994_RIGHT_OUTPUT_VOL                   0x1D                                        /* Right Output Volume */
#define WM8994_LINE_OUTPUTS_VOL                   0x1E                                        /* Line Outputs Volume */
#define WM8994_HPOUT2_VOL                         0x1F                                        /* HPOUT2 Volume */
#define WM8994_LEFT_OPGA_VOL                      0x20                                        /* Left OPGA Volume */
#define WM8994_RIGHT_OPGA_VOL                     0x21                                        /* Right OPGA Volume */

#define WM8994_SPKMIXL_ATT                        0x22                                        /* SPKMIXL Attenuation */
#define WM8994_SPKMIXR_ATT                        0x23                                        /* SPKMIXR Attenuation */
#define WM8994_SPKOUT_MIXERS                      0x24                                        /* SPKOUT Mixers */

#define WM8994_CLASS_D                            0x25                                        /* ClassD */
#define WM8994_SPEAKER_VOL_LEFT                   0x26                                        /* Speaker Volume left */
#define WM8994_SPEAKER_VOL_RIGHT                  0x27                                        /* Speaker Volume right */

#define WM8994_INPUT_MIXER2                       0x28                                        /* Input Mixer (2) */
#define WM8994_INPUT_MIXER3                       0x29                                        /* Input Mixer (3) */
#define WM8994_INPUT_MIXER4                       0x2A                                        /* Input Mixer (4) */
#define WM8994_INPUT_MIXER5                       0x2B                                        /* Input Mixer (5) */
#define WM8994_INPUT_MIXER6                       0x2C                                        /* Input Mixer (6) */

#define WM8994_OUTPUT_MIXER1                      0x2D                                        /* Output Mixer (1) */
#define WM8994_OUTPUT_MIXER2                      0x2E                                        /* Output Mixer (2) */
#define WM8994_OUTPUT_MIXER3                      0x2F                                        /* Output Mixer (3) */
#define WM8994_OUTPUT_MIXER4                      0x30                                        /* Output Mixer (4) */
#define WM8994_OUTPUT_MIXER5                      0x31                                        /* Output Mixer (5) */
#define WM8994_OUTPUT_MIXER6                      0x32                                        /* Output Mixer (6) */

#define WM8994_HPOUT2_MIXER                       0x33                                        /* HPOUT2 Mixer */

#define WM8994_LINE_MIXER1                        0x34                                        /* Line Mixer (1) */
#define WM8994_LINE_MIXER2                        0x35                                        /* Line Mixer (2) */

#define WM8994_SPEAKER_MIXER                      0x36                                        /* Speaker Mixer */

#define WM8994_ADDITIONAL_CTL                     0x37                                        /* Additional Control */
#define WM8994_ANTI_POP1                          0x38                                        /* AntiPOP (1) */
#define WM8994_ANTI_POP2                          0x39                                        /* AntiPOP (2) */

#define WM8994_MIC_BIAS                           0x3A                                        /* MICBIAS */

#define WM8994_LDO_1                              0x3B                                        /* LDO 1 */
#define WM8994_LDO_2                              0x3C                                        /* LDO 2 */

#define WM8994_CHARGE_PUMP1                       0x4C                                        /* Charge Pump 1 */
#define WM8994_CHARGE_PUMP2                       0x4D                                        /* Charge Pump 2 */

#define WM8994_CLASS_W_1                          0x51                                        /* Class W (1) */

#define WM8994_DC_SERVO1                          0x54                                        /* DC Servo (1) */
#define WM8994_DC_SERVO2                          0x55                                        /* DC Servo (2) */
#define WM8994_DC_SERVO_RB                        0x58                                        /* DC Servo Readback */
#define WM8994_DC_SERVO4                          0x59                                        /* DC Servo (4) */

#define WM8994_ANA_HP1                            0x60                                        /* Analogue HP (1) */
#define WM8994_CHIP_REV                           0x100                                       /* Chip Revision */

#define WM8994_CTL_IF                             0x101                                       /* Control Interface */
#define WM8994_WR_CTL_SEQ1                        0x110                                       /* Write Sequencer Ctrl (1) */
#define WM8994_WR_CTL_SEQ2                        0x111                                       /* Write Sequencer Ctrl (2) */

#define WM8994_AIF1_CLK1                          0x200                                       /* AIF1 Clocking (1) */
#define WM8994_AIF1_CLK2                          0x201                                       /* AIF1 Clocking (2) */
#define WM8994_AIF2_CLK1                          0x204                                       /* AIF2 Clocking (1) */
#define WM8994_AIF2_CLK2                          0x205                                       /* AIF2 Clocking (2) */
#define WM8994_CLK1                               0x208                                       /* Clocking (1) */
#define WM8994_CLK2                               0x209                                       /* Clocking (2) */

#define WM8994_AIF1_RATE                          0x210                                       /* AIF1 Rate */
#define WM8994_AIF2_RATE                          0x211                                       /* AIF2 Rate */
#define WM8994_RATE_STATUS                        0x212                                       /* Rate Status */

#define WM8994_PLL1_CTL1                          0x220                                       /* PLL1 Control (1) */
#define WM8994_PLL1_CTL2                          0x221                                       /* PLL1 Control (2) */
#define WM8994_PLL1_CTL3                          0x222                                       /* PLL1 Control (3) */
#define WM8994_PLL1_CTL4                          0x223                                       /* PLL1 Control (4) */
#define WM8994_PLL1_CTL5                          0x224                                       /* PLL1 Control (5) */

#define WM8994_PLL2_CTL1                          0x240                                       /* PLL2 Control (1) */
#define WM8994_PLL2_CTL2                          0x241                                       /* PLL2 Control (2) */
#define WM8994_PLL2_CTL3                          0x242                                       /* PLL2 Control (3) */
#define WM8994_PLL2_CTL4                          0x243                                       /* PLL2 Control (4) */
#define WM8994_PLL2_CTL5                          0x244                                       /* PLL2 Control (5) */

#define WM8994_AIF1_CTL1                          0x300                                       /* AIF1 Control (1) */
#define WM8994_AIF1_CTL2                          0x301                                       /* AIF1 Control (2) */

#define WM8994_AIF1_MASTER_SLAVE                  0x302                                       /* AIF1 Master/Slave */
#define WM8994_AIF1_BCLK                          0x303                                       /* AIF1 BCLK */
#define WM8994_AIF1_ADC_LRCLK                     0x304                                       /* AIF1 ADC LRCLK */
#define WM8994_AIF1_DAC_LRCLK                     0x305                                       /* AIF1 DAC LRCLK */
#define WM8994_AIF1_DAC_DATA                      0x306                                       /* AIF1 DAC DATA */
#define WM8994_AIF1_ADC_DATA                      0x307                                       /* AIF1 ADC DATA */

#define WM8994_AIF2_CTL1                          0x310                                       /* AIF2 Control (1) */
#define WM8994_AIF2_CTL2                          0x311                                       /* AIF2 Control (2) */
#define WM8994_AIF2_MASTER_SLAVE                  0x312                                       /* AIF2 Master/Slave */
#define WM8994_AIF2_BCLK                          0x313                                       /* AIF2 BCLK */
#define WM8994_AIF2_ADC_LRCLK                     0x314                                       /* AIF2 ADC LRCLK */
#define WM8994_AIF2_DAC_LRCLK                     0x315                                       /* AIF2 DAC LRCLK */
#define WM8994_AIF2_DAC_DATA                      0x316                                       /* AIF2 DAC DATA */
#define WM8994_AIF2_ADC_DATA                      0x317                                       /* AIF2 ADC DATA */

#define WM8994_AIF1_ADC1_LEFT_VOL                 0x400                                       /* AIF1 ADC1 Left Volume */
#define WM8994_AIF1_ADC1_RIGHT_VOL                0x401                                       /* AIF1 ADC1 Right Volume */
#define WM8994_AIF1_DAC1_LEFT_VOL                 0x402                                       /* AIF1 DAC1 Left Volume */
#define WM8994_AIF1_DAC1_RIGHT_VOL                0x403                                       /* AIF1 DAC1 Right Volume */

#define WM8994_AIF1_ADC2_LEFT_VOL                 0x404                                       /* AIF1 ADC2 Left Volume */
#define WM8994_AIF1_ADC2_RIGHT_VOL                0x405                                       /* AIF1 ADC2 Right Volume */
#define WM8994_AIF1_DAC2_LEFT_VOL                 0x406                                       /* AIF1 DAC2 Left Volume */
#define WM8994_AIF1_DAC2_RIGHT_VOL                0x407                                       /* AIF1 DAC2 Right Volume */

#define WM8994_AIF1_ADC1_FILTERS                  0x410                                       /* AIF1 ADC1 Filters */
#define WM8994_AIF1_ADC2_FILTERS                  0x411                                       /* AIF1 ADC2 Filters */

#define WM8994_AIF1_DAC1_FILTERS1                 0x420                                       /* AIF1 DAC1 Filters (1) */
#define WM8994_AIF1_DAC1_FILTERS2                 0x421                                       /* AIF1 DAC1 Filters (2) */
#define WM8994_AIF1_DAC2_FILTERS1                 0x422                                       /* AIF1 DAC2 Filters (1) */
#define WM8994_AIF1_DAC2_FILTERS2                 0x423                                       /* AIF1 DAC2 Filters (2) */

#define WM8994_AIF1_DRC1_1                        0x440                                       /* AIF1 DRC1 (1) */
#define WM8994_AIF1_DRC1_2                        0x441                                       /* AIF1 DRC1 (2) */
#define WM8994_AIF1_DRC1_3                        0x442                                       /* AIF1 DRC1 (3) */
#define WM8994_AIF1_DRC1_4                        0x443                                       /* AIF1 DRC1 (4) */
#define WM8994_AIF1_DRC1_5                        0x444                                       /* AIF1 DRC1 (5) */

#define WM8994_AIF1_DRC2_1                        0x450                                       /* AIF1 DRC2 (1) */
#define WM8994_AIF1_DRC2_2                        0x451                                       /* AIF1 DRC2 (2) */
#define WM8994_AIF1_DRC2_3                        0x452                                       /* AIF1 DRC2 (3) */
#define WM8994_AIF1_DRC2_4                        0x453                                       /* AIF1 DRC2 (4) */
#define WM8994_AIF1_DRC2_5                        0x454                                       /* AIF1 DRC2 (5) */

#define WM8994_AIF1_DAC1_EQ_GAINS_1               0x480                                       /* AIF1 DAC1 EQ Gains (1) */
#define WM8994_AIF1_DAC1_EQ_GAINS_2               0x481                                       /* AIF1 DAC1 EQ Gains (2) */
#define WM8994_AIF1_DAC1_EQ_BAND_1A               0x482                                       /* AIF1 DAC1 EQ Band 1 A */
#define WM8994_AIF1_DAC1_EQ_BAND_1B               0x483                                       /* AIF1 DAC1 EQ Band 1 B */
#define WM8994_AIF1_DAC1_EQ_BAND_1PG              0x484                                       /* AIF1 DAC1 EQ Band 1 PG */
#define WM8994_AIF1_DAC1_EQ_BAND_2A               0x485                                       /* AIF1 DAC1 EQ Band 2 A */
#define WM8994_AIF1_DAC1_EQ_BAND_2B               0x486                                       /* AIF1 DAC1 EQ Band 2 B */
#define WM8994_AIF1_DAC1_EQ_BAND_2C               0x487                                       /* AIF1 DAC1 EQ Band 2 C */
#define WM8994_AIF1_DAC1_EQ_BAND_2PG              0x488                                       /* AIF1 DAC1 EQ Band 2 PG */
#define WM8994_AIF1_DAC1_EQ_BAND_3A               0x489                                       /* AIF1 DAC1 EQ Band 3 A */
#define WM8994_AIF1_DAC1_EQ_BAND_3B               0x48A                                       /* AIF1 DAC1 EQ Band 3 B */
#define WM8994_AIF1_DAC1_EQ_BAND_3C               0x48B                                       /* AIF1 DAC1 EQ Band 3 C */
#define WM8994_AIF1_DAC1_EQ_BAND_3PG              0x48C                                       /* AIF1 DAC1 EQ Band 3 PG */
#define WM8994_AIF1_DAC1_EQ_BAND_4A               0x48D                                       /* AIF1 DAC1 EQ Band 4 A */
#define WM8994_AIF1_DAC1_EQ_BAND_4B               0x48E                                       /* AIF1 DAC1 EQ Band 4 B */
#define WM8994_AIF1_DAC1_EQ_BAND_4C               0x48F                                       /* AIF1 DAC1 EQ Band 4 C */
#define WM8994_AIF1_DAC1_EQ_BAND_4PG              0x490                                       /* AIF1 DAC1 EQ Band 4 PG */
#define WM8994_AIF1_DAC1_EQ_BAND_5A               0x491                                       /* AIF1 DAC1 EQ Band 5 A */
#define WM8994_AIF1_DAC1_EQ_BAND_5B               0x492                                       /* AIF1 DAC1 EQ Band 5 B */
#define WM8994_AIF1_DAC1_EQ_BAND_5PG              0x493                                       /* AIF1 DAC1 EQ Band 5 PG */

#define WM8994_AIF1_DAC2_EQ_GAINS_1               0x4A0                                       /* AIF1 DAC2 EQ Gains (1) */
#define WM8994_AIF1_DAC2_EQ_GAINS_2               0x4A1                                       /* AIF1 DAC2 EQ Gains (2) */
#define WM8994_AIF1_DAC2_EQ_BAND_1A               0x4A2                                       /* AIF1 DAC2 EQ Band 1 A */
#define WM8994_AIF1_DAC2_EQ_BAND_1B               0x4A3                                       /* AIF1 DAC2 EQ Band 1 B */
#define WM8994_AIF1_DAC2_EQ_BAND_1PG              0x4A4                                       /* AIF1 DAC2 EQ Band 1 PG */
#define WM8994_AIF1_DAC2_EQ_BAND_2A               0x4A5                                       /* AIF1 DAC2 EQ Band 2 A */
#define WM8994_AIF1_DAC2_EQ_BAND_2B               0x4A6                                       /* AIF1 DAC2 EQ Band 2 B */
#define WM8994_AIF1_DAC2_EQ_BAND_2C               0x4A7                                       /* AIF1 DAC2 EQ Band 2 C */
#define WM8994_AIF1_DAC2_EQ_BAND_2PG              0x4A8                                       /* AIF1 DAC2 EQ Band 2 PG */
#define WM8994_AIF1_DAC2_EQ_BAND_3A               0x4A9                                       /* AIF1 DAC2 EQ Band 3 A */
#define WM8994_AIF1_DAC2_EQ_BAND_3B               0x4AA                                       /* AIF1 DAC2 EQ Band 3 B */
#define WM8994_AIF1_DAC2_EQ_BAND_3C               0x4AB                                       /* AIF1 DAC2 EQ Band 3 C */
#define WM8994_AIF1_DAC2_EQ_BAND_3PG              0x4AC                                       /* AIF1 DAC2 EQ Band 3 PG */
#define WM8994_AIF1_DAC2_EQ_BAND_4A               0x4AD                                       /* AIF1 DAC2 EQ Band 4 A */
#define WM8994_AIF1_DAC2_EQ_BAND_4B               0x4AE                                       /* AIF1 DAC2 EQ Band 4 B */
#define WM8994_AIF1_DAC2_EQ_BAND_4C               0x4AF                                       /* AIF1 DAC2 EQ Band 4 C */
#define WM8994_AIF1_DAC2_EQ_BAND_4PG              0x4B0                                       /* AIF1 DAC2 EQ Band 4 PG */
#define WM8994_AIF1_DAC2_EQ_BAND_5A               0x4B1                                       /* AIF1 DAC2 EQ Band 5 A */
#define WM8994_AIF1_DAC2_EQ_BAND_5B               0x4B2                                       /* AIF1 DAC2 EQ Band 5 B */
#define WM8994_AIF1_DAC2_EQ_BAND_5PG              0x4B3                                       /* AIF1 DAC2 EQ Band 5 PG */

/* AIF2 */
#define WM8994_AIF2_ADC1_LEFT_VOL                 0x500                                       /* AIF2 ADC1 Left Volume */
#define WM8994_AIF2_ADC1_RIGHT_VOL                0x501                                       /* AIF2 ADC1 Right Volume */
#define WM8994_AIF2_DAC1_LEFT_VOL                 0x502                                       /* AIF2 DAC1 Left Volume */
#define WM8994_AIF2_DAC1_RIGHT_VOL                0x503                                       /* AIF2 DAC1 Right Volume */

#define WM8994_AIF2_ADC_FILTERS                   0x510                                       /* AIF2 ADC Filters */
#define WM8994_AIF2_DAC_FILTERS1                  0x520                                       /* AIF2 DAC Filters (1) */
#define WM8994_AIF2_DAC_FILTERS2                  0x521                                       /* AIF2 DAC Filters (2) */

#define WM8994_AIF2_DRC_1                         0x540                                       /* AIF2 DRC (1) */
#define WM8994_AIF2_DRC_2                         0x541                                       /* AIF2 DRC (2) */
#define WM8994_AIF2_DRC_3                         0x542                                       /* AIF2 DRC (3) */
#define WM8994_AIF2_DRC_4                         0x543                                       /* AIF2 DRC (4) */
#define WM8994_AIF2_DRC_5                         0x544                                       /* AIF2 DRC (5) */

#define WM8994_AIF2_EQ_GAINS_1                    0x580                                       /* AIF2 EQ Gains (1) */
#define WM8994_AIF2_EQ_GAINS_2                    0x581                                       /* AIF2 EQ Gains (2) */
#define WM8994_AIF2_EQ_BAND_1A                    0x582                                       /* AIF2 EQ Band 1 A */
#define WM8994_AIF2_EQ_BAND_1B                    0x583                                       /* AIF2 EQ Band 1 B */
#define WM8994_AIF2_EQ_BAND_1PG                   0x584                                       /* AIF2 EQ Band 1 PG */
#define WM8994_AIF2_EQ_BAND_2A                    0x585                                       /* AIF2 EQ Band 2 A */
#define WM8994_AIF2_EQ_BAND_2B                    0x586                                       /* AIF2 EQ Band 2 B */
#define WM8994_AIF2_EQ_BAND_2C                    0x587                                       /* AIF2 EQ Band 2 C */
#define WM8994_AIF2_EQ_BAND_2PG                   0x588                                       /* AIF2 EQ Band 2 PG */
#define WM8994_AIF2_EQ_BAND_3A                    0x589                                       /* AIF2 EQ Band 3 A */
#define WM8994_AIF2_EQ_BAND_3B                    0x58A                                       /* AIF2 EQ Band 3 B */
#define WM8994_AIF2_EQ_BAND_3C                    0x58B                                       /* AIF2 EQ Band 3 C */
#define WM8994_AIF2_EQ_BAND_3PG                   0x58C                                       /* AIF2 EQ Band 3 PG */
#define WM8994_AIF2_EQ_BAND_4A                    0x58D                                       /* AIF2 EQ Band 4 A */
#define WM8994_AIF2_EQ_BAND_4B                    0x58E                                       /* AIF2 EQ Band 4 B */
#define WM8994_AIF2_EQ_BAND_4C                    0x58F                                       /* AIF2 EQ Band 4 C */
#define WM8994_AIF2_EQ_BAND_4PG                   0x590                                       /* AIF2 EQ Band 4 PG */
#define WM8994_AIF2_EQ_BAND_5A                    0x591                                       /* AIF2 EQ Band 5 A */
#define WM8994_AIF2_EQ_BAND_5B                    0x592                                       /* AIF2 EQ Band 5 B */
#define WM8994_AIF2_EQ_BAND_5PG                   0x593                                       /* AIF2 EQ Band 5 PG */

#define WM8994_DAC1_MIXER_VOLS                    0x600                                       /* DAC1 Mixer Volumes */
#define WM8994_DAC1_LEFT_MIXER_ROUTING            0x601                                       /* DAC1 Left Mixer Routing */
#define WM8994_DAC1_RIGHT_MIXER_ROUTING           0x602                                       /* DAC1 Right Mixer Routing */

#define WM8994_DAC2_MIXER_VOLS                    0x603                                       /* DAC2 Mixer Volumes */
#define WM8994_DAC2_LEFT_MIXER_ROUTING            0x604                                       /* DAC2 Left Mixer Routing */
#define WM8994_DAC2_RIGHT_MIXER_ROUTING           0x605                                       /* DAC2 Right Mixer Routing */

#define WM8994_ADC1_LEFT_MIXER_ROUTING            0x606                                       /* ADC1 Left Mixer Routing */
#define WM8994_ADC1_RIGHT_MIXER_ROUTING           0x607                                       /* ADC1 Right Mixer Routing */

#define WM8994_ADC2_LEFT_MIXER_ROUTING            0x608                                       /* ADC2 Left Mixer Routing */
#define WM8994_ADC2_RIGHT_MIXER_ROUTING           0x609                                       /* ADC2 Right Mixer Routing */

#define WM8994_DAC1_LEFT_VOL                      0x610                                       /* DAC1 Left Volume */
#define WM8994_DAC1_RIGHT_VOL                     0x611                                       /* DAC1 Right Volume */

#define WM8994_DAC2_LEFT_VOL                      0x612                                       /* DAC2 Left Volume */
#define WM8994_DAC2_RIGHT_VOL                     0x613                                       /* DAC2 Right Volume */

#define WM8994_DAC_SOFT_MUTE                      0x614                                       /* DAC Softmute */

#define WM8994_OVER_SAMPLING                      0x620                                       /* Oversampling */
#define WM8994_SIDE_TONE                          0x621                                       /* Sidetone */

#define WM8994_GPIO1                              0x700                                       /* GPIO 1 */
#define WM8994_GPIO2                              0x701                                       /* GPIO 2 */
#define WM8994_GPIO3                              0x702                                       /* GPIO 3 */
#define WM8994_GPIO4                              0x703                                       /* GPIO 4 */
#define WM8994_GPIO5                              0x704                                       /* GPIO 5 */
#define WM8994_GPIO6                              0x705                                       /* GPIO 6 */
#define WM8994_GPIO7                              0x706                                       /* GPIO 7 */
#define WM8994_GPIO8                              0x707                                       /* GPIO 8 */
#define WM8994_GPIO9                              0x708                                       /* GPIO 9 */
#define WM8994_GPIO10                             0x709                                       /* GPIO 10 */
#define WM8994_GPIO11                             0x70A                                       /* GPIO 11 */

#define WM8994_PULL_CTL1                          0x720                                       /* Pull Control (1) */
#define WM8994_PULL_CTL2                          0x721                                       /* Pull Control (2) */

#define WM8994_INT_STATUS1                        0x730                                       /* Interrupt Status 1 */
#define WM8994_INT_STATUS2                        0x731                                       /* Interrupt Status 2 */
#define WM8994_INT_RAW_STATUS2                    0x732                                       /* Interrupt Raw Status 2 */
#define WM8994_INT_STATUS1_MASK                   0x738                                       /* Interrupt Status 1 Mask */
#define WM8994_INT_STATUS2_MASK                   0x739                                       /* Interrupt Status 2 Mask */
#define WM8994_INT_CTL                            0x740                                       /* Interrupt Control */
#define WM8994_INT_DEBOUNCE                       0x748                                       /* IRQ Debounce */

#if 0
#define WM8994_WR_SEQ0                            0x3000                                      /* Write Sequencer 0 */
#define WM8994_WR_SEQ1                            0x3001                                      /* Write Sequencer 1 */
#define WM8994_WR_SEQ2                            0x3002                                      /* Write Sequencer 2 */
#define WM8994_WR_SEQ3                            0x3003                                      /* Write Sequencer 3 */

#define WM8994_WR_SEQ508                          0x31FC                                      /* Write Sequencer 508 */
#define WM8994_WR_SEQ509                          0x31FD                                      /* Write Sequencer 509 */
#define WM8994_WR_SEQ510                          0x31FE                                      /* Write Sequencer 510 */
#define WM8994_WR_SEQ511                          0x31FF                                      /* Write Sequencer 511 */
#endif

#define WM8994_WR_SEQ(x)                          (0x3000+(x))

#define WM8994_WR_SEQ_NUM                         (511)

#define WM8994_REGISTER_COUNT                     736
#define WM8994_MAX_REGISTER                       0x31FF
#define WM8994_MAX_CACHED_REGISTER                0x749

/* Field Definitions.
 */

/* R0 (0x00) - Software Reset
 */

#define WM8994_SW_RESET                           (0)                                         /* Bits 0-15: SW_RESET - [15:0] */

/* R1 (0x01) - Power Management (1)
 */

#define WM8994_BIAS_ENA                           (1 << 0)                                    /* Bit 0: Enables the Normal bias current generator (for all analogue functions */
#define   WM8994_BIAS_ENA_DISABLE                 (0)                                         /* Disabled */
#define   WM8994_BIAS_ENA_ENABLE                  WM8994_BIAS_ENA                             /* Enabled */
#define WM8994_VMID_SEL_SHIFT                     (1)                                         /* Bits 1-2: VMID Divider Enable and Select */
#define   WM8994_VMID_SEL_DISABLE                 (0 << WM8994_VMID_SEL_SHIFT)                /* VMID disabled (for OFF mode) */
#define   WM8994_VMID_SEL_2X40K                   (1 << WM8994_VMID_SEL_SHIFT)                /* 2*40k divider (for normal operation */
#define   WM8994_VMID_SEL_2X240K                  (2 << WM8994_VMID_SEL_SHIFT)                /* 2*240k divider (for low power standby*/
                                                                                              /* Bit 3: Reserved */

#define WM8994_MICB1_ENA                          (1 << 4)                                    /* Bit 4; Microphone Bias 1 Enable */
#define   WM8994_MICB1_ENA_DISABLE                (0)                                         /* Disabled */
#define   WM8994_MICB1_ENA_ENABLE                 (WM8994_MICB1_ENA)                          /* Enabled */
#define WM8994_MICB2_ENA                          (1 << 5)                                    /* Bit 5; Microphone Bias 2 Enable */
#define   WM8994_MICB2_ENA_DISABLE                (0)                                         /* Disabled */
#define   WM8994_MICB2_ENA_ENABLE                 (WM8994_MICB2_ENA)                          /* Enabled */
                                                                                              /* Bits 6-7: Reserved */
#define WM8994_HPOUT1R_ENA                        (1 << 8)                                    /* Bit 8: Enables HPOUT1R input stage */
#define   WM8994_HPOUT1R_ENA_DISABLE              (0)                                         /* Disabled */ 
#define   WM8994_HPOUT1R_ENA_ENABLE               (WM8994_HPOUT1R_ENA)                        /* Enabled */
#define WM8994_HPOUT1L_ENA                        (1 << 9)                                    /* Bit 9: Enables HPOUT1L input stage */
#define   WM8994_HPOUT1L_ENA_DISABLE              (0)                                         /* Disabled */ 
#define   WM8994_HPOUT1L_ENA_ENABLE               (WM8994_HPOUT1L_ENA)                        /* Enabled */
                                                                                              /* Bit 10: Reserved */
#define WM8994_HPOUT2_ENA                         (1 << 11)                                   /* Bit 11: Enables HPOUT2 input stage */
#define   WM8994_HPOUT2_ENA_DISABLE               (0)                                         /* Disabled */ 
#define   WM8994_HPOUT2_ENA_ENABLE                (WM8994_HPOUT2_ENA)                         /* Enabled */
#define WM8994_SPKOUTL_ENA                        (1 << 12)                                   /* Bit 12: SPKMIXL Mixer, SPKLVOL PGA and SPKOUTL Output Enable */
#define   WM8994_SPKOUTL_ENA_DISABLE              (0)                                         /* Disabled */
#define   WM8994_SPKOUTL_ENA_ENABLE               (WM8994_SPKOUTL_ENA)                        /* Enabled */
#define WM8994_SPKOUTR_ENA                        (1 << 13)                                   /* Bit 13: SPKMIXR Mixer, SPKRVOL PGA and SPKOUTR Output Enable */
#define   WM8994_SPKOUTR_ENA_DISABLE              (0)                                         /* Disabled */
#define   WM8994_SPKOUTR_ENA_ENABLE               (WM8994_SPKOUTR_ENA)                        /* Enabled */
                                                                                              /* Bits 14-15: Reserved */

/* R2 (0x02) - Power Management (2)
 */

                                                  /* Bits 0-3: Reserved */

#define WM8994_IN1R_ENA                           (1 << 4)                                    /* Bit 4: IN1R Input PGA Enable */
#define   WM8994_IN1R_ENA_DISABLE                 (0)                                         /* Disabled */
#define   WM8994_IN1R_ENA_ENABLE                  (WM8994_IN1R_ENA)                           /* Enabled */
#define WM8994_IN2R_ENA                           (1 << 5)                                    /* Bit 5: IN2R Input PGA Enable */
#define   WM8994_IN2R_ENA_DISABLE                 (0)                                         /* Disabled */
#define   WM8994_IN2R_ENA_ENABLE                  (WM8994_IN2R_ENA)                           /* Enabled */
#define WM8994_IN1L_ENA                           (1 << 6)                                    /* Bit 6: IN1L Input PGA Enable */
#define   WM8994_IN1L_ENA_DISABLE                 (0)                                         /* Disabled */
#define   WM8994_IN1L_ENA_ENABLE                  (WM8994_IN1L_ENA)                           /* Enabled */
#define WM8994_IN2L_ENA                           (1 << 7)                                    /* Bit 7: IN2L Input PGA Enable */
#define   WM8994_IN2L_ENA_DISABLE                 (0)                                         /* Disabled */
#define   WM8994_IN2L_ENA_ENABLE                  (WM8994_IN2L_ENA)                           /* Enabled */
#define WM8994_MIXINR_ENA                         (1 << 8)                                    /* Bit 8: Right Input Mixer Enable */
#define   WM8994_MIXINR_ENA_DISABLE               (0)                                         /* Disabled */
#define   WM8994_MIXINR_ENA_ENABLE                (WM8994_MIXINR_ENA)                         /* Enabled */
#define WM8994_MIXINL_ENA                         (1 << 9)                                    /* Bit 9: Left Input Mixer Enable */
#define   WM8994_MIXINL_ENA_DISABLE               (0)                                         /* Disabled */
#define   WM8994_MIXINL_ENA_ENABLE                (WM8994_MIXINL_ENA)                         /* Enabled */
                                                                                              /* Bit 10: Reserved */
#define WM8994_OPCLK_ENA                          (1 << 11)                                   /* Bit 11: GPIO Clock Output(OPCLK) Enable */
#define   WM8994_OPCLK_ENA_DISABLE                (0)                                         /* Disabled */
#define   WM8994_OPCLK_ENA_ENABLE                 (WM8994_OPCLK_ENA)                          /* Enabled */
                                                                                              /* Bit 12: Reserved */
#define WM8994_TSHUT_OPDIS                        (1 << 13)                                   /* Bit 13: Thermal shutdown control */
#define   WM8994_TSHUT_OPDIS_DISABLE              (0)                                         /* Disabled */ 
#define   WM8994_TSHUT_OPDIS_ENABLE               (WM8994_TSHUT_OPDIS)                        /* Enabled */       
#define WM8994_TSHUT_ENA                          (1 << 14)                                   /* Bit 14: Thermal sensor enable */
#define   WM8994_TSHUT_ENA_DISABLE                (0)                                         /* Disabled */ 
#define   WM8994_TSHUT_ENA_ENABLE                 (WM8994_TSHUT_ENA)                          /* Enabled */       
                                                                                              /* Bit 15: Reserved */

/* R3 (0x03) - Power Management (3)
 */

                                                  /* Bits 0-3: Reserved */
#define WM8994_MIXOUTR_ENA                        (1 << 4)                                    /* Bit 4: MIXOUTR Right Output Mixer Enable */
#define   WM8994_MIXOUTR_ENA_DISABLE              (0)                                         /* Disabled */
#define   WM8994_MIXOUTR_ENA_ENABLE               (WM8994_MIXOUTR_ENA)                        /* Enabled */ 
#define WM8994_MIXOUTL_ENA                        (1 << 5)                                    /* Bit 5: MIXOUTL Left Output Mixer Enable */
#define   WM8994_MIXOUTL_ENA_DISABLE              (0)                                         /* Disabled */
#define   WM8994_MIXOUTL_ENA_ENABLE               (WM8994_MIXOUTL_ENA)                        /* Enabled */ 
#define WM8994_MIXOUTRVOL_ENA                     (1 << 6)                                    /* Bit 6: MIXOUTR Right Volume Control Enable */
#define   WM8994_MIXOUTRVOL_ENA_DISABLE           (0)                                         /* Disabled */
#define   WM8994_MIXOUTRVOL_ENA_ENABLE            (WM8994_MIXOUTRVOL_ENA)                     /* Enabled */ 
#define WM8994_MIXOUTLVOL_ENA                     (1 << 7)                                    /* Bit 7: MIXOUTL Left Volume Control Enable */
#define   WM8994_MIXOUTLVOL_ENA_DISABLE           (0)                                         /* Disabled */
#define   WM8994_MIXOUTLVOL_ENA_ENABLE            (WM8994_MIXOUTLVOL_ENA)                     /* Enabled */ 
#define WM8994_SPKLVOL_ENA                        (1 << 8)                                    /* Bit 8: SPKMIXL Mixer and SPKLVOL PGA Enable */
#define   WM8994_SPKLVOL_ENA_DISABLE              (0)                                         /* Disabled */
#define   WM8994_SPKLVOL_ENA_ENABLE               (WM8994_SPKLVOL_ENA)                        /* Enabled */ 
#define WM8994_SPKRVOL_ENA                        (1 << 9)                                    /* Bit 9: SPKMIXR Mixer and SPKRVOL PGA Enable */
#define   WM8994_SPKRVOL_ENA_DISABLE              (0)                                         /* Disabled */
#define   WM8994_SPKRVOL_ENA_ENABLE               (WM8994_SPKRVOL_ENA)                        /* Enabled */ 
#define WM8994_LINEOUT2P_ENA                      (1 << 10)                                   /* Bit 10: LINEOUT2P Line Out and LINEOUT2PMIX Enable */
#define   WM8994_LINEOUT2P_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_LINEOUT2P_ENA_ENABLE             (WM8994_LINEOUT2P_ENA)                      /* Enabled */
#define WM8994_LINEOUT2N_ENA                      (1 << 11)                                   /* Bit 11: LINEOUT2N Line Out and LINEOUT2NMIX Enable */
#define   WM8994_LINEOUT2N_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_LINEOUT2N_ENA_ENABLE             (WM8994_LINEOUT2N_ENA)                      /* Enabled */
#define WM8994_LINEOUT1P_ENA                      (1 << 12)                                   /* Bit 12: LINEOUT1P Line Out and LINEOUT1PMIX Enable */
#define   WM8994_LINEOUT1P_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_LINEOUT1P_ENA_ENABLE             (WM8994_LINEOUT1P_ENA)                      /* Enabled */
#define WM8994_LINEOUT1N_ENA                      (1 << 13)                                   /* Bit 13: LINEOUT1N Line Out and LINEOUT1NMIX Enable */
#define   WM8994_LINEOUT1N_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_LINEOUT1N_ENA_ENABLE             (WM8994_LINEOUT1N_ENA)                      /* Enabled */
                                                                                              /* Bits 14-15: Reserved */

/* R4 (0x04) - Power Management (4)
 */

#define WM8994_ADCR_ENA                           (1 << 0)                                    /* Bit 0: Right ADC Enable */
#define   WM8994_ADCR_ENA_DISABLE                 (0)                                         /* Disabled */
#define   WM8994_ADCR_ENA_ENABLE                  (WM8994_ADCR_ENA)                           /* Enabled */
#define WM8994_ADCL_ENA                           (1 << 1)                                    /* Bit 1: Left ADC Enable */
#define   WM8994_ADCL_ENA_DISABLE                 (0)                                         /* Disabled */
#define   WM8994_ADCL_ENA_ENABLE                  (WM8994_ADCL_ENA)                           /* Enabled */
#define WM8994_DMIC1R_ENA                         (1 << 2)                                    /* Bit 2: Digital microphone DMICDAT1 Right channel enable */
#define   WM8994_DMIC1R_ENA_DISABLE               (0)                                         /* Disabled */
#define   WM8994_DMIC1R_ENA_ENABLE                (WM8994_DMIC1R_ENA)                         /* Enabled */ 
#define WM8994_DMIC1L_ENA                         (1 << 3)                                    /* Bit 3: Digital microphone DMICDAT1 Left channel enable */
#define   WM8994_DMIC1L_ENA_DISABLE               (0)                                         /* Disabled */
#define   WM8994_DMIC1L_ENA_ENABLE                (WM8994_DMIC1L_ENA)                         /* Enabled */ 
#define WM8994_DMIC2R_ENA                         (1 << 4)                                    /* Bit 4: Digital microphone DMICDAT2 Right channel enable */
#define   WM8994_DMIC2R_ENA_DISABLE               (0)                                         /* Disabled */
#define   WM8994_DMIC2R_ENA_ENABLE                (WM8994_DMIC2R_ENA)                         /* Enabled */ 
#define WM8994_DMIC2L_ENA                         (1 << 5)                                    /* Bit 5: Digital microphone DMICDAT2 Left channel enable */
#define   WM8994_DMIC2L_ENA_DISABLE               (0)                                         /* Disabled */
#define   WM8994_DMIC2L_ENA_ENABLE                (WM8994_DMIC2L_ENA)                         /* Enabled */ 
                                                                                              /* Bits 6-7: Reserved */
#define WM8994_AIF1ADC1R_ENA                      (1 << 8)                                    /* Bit 8: Enable AIF1ADC1(Right) output path (AIF1, Timeslot 0) */
#define   WM8994_AIF1ADC1R_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_AIF1ADC1R_ENA_ENABLE             (WM8994_AIF1ADC1R_ENA)                      /* Enabled */
#define WM8994_AIF1ADC1L_ENA                      (1 << 9)                                    /* Bit 9: Enable AIF1ADC1(Left) output path (AIF1, Timeslot 0) */
#define   WM8994_AIF1ADC1L_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_AIF1ADC1L_ENA_ENABLE             (WM8994_AIF1ADC1L_ENA)                      /* Enabled */
#define WM8994_AIF1ADC2R_ENA                      (1 << 10)                                   /* Bit 10: Enable AIF1ADC2(Right) output path (AIF1, Timeslot 1) */
#define   WM8994_AIF1ADC2R_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_AIF1ADC2R_ENA_ENABLE             (WM8994_AIF1ADC2L_ENA)                      /* Enabled */
#define WM8994_AIF1ADC2L_ENA                      (1 << 11)                                   /* Bit 11: Enable AIF1ADC2(Left) output path (AIF1, Timeslot 1) */
#define   WM8994_AIF1ADC2L_ENA_DISABLE            (0)                                         /* Disabled */
#define   WM8994_AIF1ADC2L_ENA_ENABLE             (WM8994_AIF1ADC2L_ENA)                      /* Enabled */
#define WM8994_AIF2ADCR_ENA                       (1 << 12)                                   /* Bit 12: Enable AIF2ADC(Right) output path */
#define   WM8994_AIF2ADCR_ENA_DISABLE             (0)                                         /* Disabled */
#define   WM8994_AIF2ADCR_ENA_ENABLE              (WM8994_AIF2ADCL_ENA)                       /* Enabled */
#define WM8994_AIF2ADCL_ENA                       (1 << 13)                                   /* Bit 13: Enable AIF2ADC(Left) output path */
#define   WM8994_AIF2ADCL_ENA_DISABLE             (0)                                         /* Disabled */
#define   WM8994_AIF2ADCL_ENA_ENABLE              (WM8994_AIF2ADCL_ENA)                       /* Enabled */
                                                                                              /* Bits 14-15: Reserved */

/* R5 (0x05) - Power Management (5)
 */
#define WM8994_DAC1R_ENA                          (1 << 0)                                    /* Bit 0: Right DAC1 Enable */
#define WM8994_DAC1L_ENA                          (1 << 1)                                    /* Bit 1: Left DAC1 Enable */
#define WM8994_DAC2R_ENA                          (1 << 2)                                    /* Bit 2: Right DAC2 Enable */
#define WM8994_DAC2L_ENA                          (1 << 3)                                    /* Bit 3: Left DAC2 Enable */
#define WM8994_AIF1DAC1R_ENA                      (1 << 8)                                    /* Bit 8: Enable AIF1DAC1(Right) input path (AIF1, Timeslot 0) */
#define WM8994_AIF1DAC1L_ENA                      (1 << 9)                                    /* Bit 9: Enable AIF1DAC1(Left) input path (AIF1, Timeslot 0) */
#define WM8994_AIF1DAC2R_ENA                      (1 << 10)                                   /* Bit 10: Enable AIF1DAC2(Right) input path (AIF1, Timeslot 1) */
#define WM8994_AIF1DAC2L_ENA                      (1 << 11)                                   /* Bit 11: Enable AIF1DAC2(Left) input path (AIF1, Timeslot 1) */
#define WM8994_AIF2DACR_ENA                       (1 << 12)                                   /* Bit 12: Enable AIF2DAC(Right) input path */
#define WM8994_AIF2DACL_ENA                       (1 << 13)                                   /* Bit 13: Enable AIF2DAC(Left) input path */
                                                                                              /* Bits 14-15: Reserved */

/* R6 (0x06) - Power Management (6)
 */
#define WM8994_AIF1_DACDAT_SRC                    (1 << 0)                                    /* Bit 0: AIF1 DACDAT Source Select */
#define   WM8994_AIF1_DACDAT_SRC_DACDAT1          (0)                                         /* DACDAT1 */
#define   WM8994_AIF1_DACDAT_SRC_GPIO8_DACDAT3    (WM8994_AIF1_DACDAT_SRC)                    /* GPIO8/DACDAT3 */
#define WM8994_AIF2_DACDAT_SRC                    (1 << 1)                                    /* Bit 1: AIF2 DACDAT Source Select */
#define   WM8994_AIF2_DACDAT_SRC_GPIO5_DACDAT1    (0)                                         /* GPIO5/DACDAT2 */
#define   WM8994_AIF2_DACDAT_SRC_GPIO8_DACDAT3    (WM8994_AIF2_DACDAT_SRC)                    /* GPIO8/DACDAT3 */
#define WM8994_AIF2_ADCDAT_SRC                    (1 << 2)                                    /* Bit 2: GPIO7/ADCDAT Source Select */
#define   WM8994_AIF2_ADCDAT_SRC_ADCDAT2          (0)                                         /* AIF2 ADCDAT2 */
#define   WM8994_AIF2_ADCDAT_SRC_GPIO8_DACDAT3    (WM8994_AIF2_ADCDAT_SRC)                    /* GPIO8/DACDAT3 */
#define WM8994_AIF3_ADCDAT_SRC                    (1 << 3)                                    /* Bits 3-4: GPIO9/ADCDAT3 Source Select */
#define   WM8994_AIF3_ADCDAR_SRC_AIF1_ADCDAT1     (0)                                         /* AIF1 ADCDAT1 */
#define   WM8994_AIF3_ADCDAR_SRC_AIF2_ADCDAT1     (1 << 3)                                    /* AIF2 ADCDAT2 */
#define   WM8994_AIF3_ADCDAR_SRC_GPIO5_DACDAT2    (2 << 3)                                    /* GPIO5/DACDAT2 */
#define WM8994_AIF3_TRI                           (1 << 5)                                    /* Bit 5: AIF3 Audio Interface tri-state */
#define   WM8994_AIF3_TRI_NO                      (0)                                         /* AIF3 pins operate normally */
#define   WM8994_AIF3_TRI_YES                     (WM8994_AIF3_TRI)                           /* Tri-State all AIF3 interface pins */

/* R21 (0x15) - Input Mixer (1)
 */

                                                  /* Bits 0-5: Reserved */
#define WM8994_INPUTS_CLAMP                       (1 << 6)                                    /* Bit 6: Input pad VMID clamp */
#define   WM8994_INPUTS_CLAMP_DE_ACTIVATED        (0)                                         /* Clamp de-activated */
#define   WM8994_INPUTS_CLAMP_ACTIVATED           (WM8994_INPUTS_CLAMP)                       /* Clamp activated */
#define WM8994_IN1LP_MIXINL_BOOST                 (1 << 7)                                    /* Bit 7: IN1LP Pin (PGA Bypass) to MIXINL Gain Boost. The bit seletcs the maximum gain setting of the IN1LP_MIXINL_VOL register. */
#define  WM8994_IN1LP_MIXINL_BOOST_P_6dB          (0)                                         /* Maximum gain is +6dB */
#define  WM8994_IN1LP_MIXINL_BOOST_P_15dB         (WM8994_IN1LP_MIXINL_BOOST)                 /* Maximu gain is +15dB */
#define WM8994_IN1RP_MIXINR_BOOST                 (1 << 8)                                    /* Bit 8: IN1RP Pin (PGA Bypass) to MIXINR Gain Boost. The bit seletcs the maximum gain setting of the IN1RP_MIXINR_VOL register. */
#define  WM8994_IN1RP_MIXINR_BOOST_P_6dB          (0)                                         /* Maximum gain is +6dB */
#define  WM8994_IN1RP_MIXINR_BOOST_P_15dB         (WM8994_IN1RP_MIXINR_BOOST)                 /* Maximu gain is +15dB */
                                                                                              /* Bits 9-15: Reserved */

/* R24 (0x18) - Left Line Input 1&2 Volume
 */

#define WM8994_IN1L_VOL                           (0)                                         /* Bits 0-4: IN1L Volume */
#define   WM8994_IN1L_VOL_MIN                     (0 << 0)                                    /* -16.5dB */
#define   WM8994_IN1L_VOL_DEFAULT                 (11 << 0)                                   /* -16.5dB to +30dB in 1.5dB steps */
#define   WM8994_IN1L_VOL_MAX                     (31 << 0)                                   /* +30dB */
                                                                                              /* Bit 5: Reserved */
#define WM8994_IN1L_ZC                            (1 << 6)                                    /* Bit 6: IN1L PGA Zero Cross Detector */ 
#define   WM8994_IN1L_ZC_NO                       (0)                                         /* Change gain immediately */
#define   WM8994_IN1L_ZC_YES                      (WM8994_IN1L_ZC)                            /* Change gain on zero cross only */
#define WM8994_IN1L_MUTE                          (1 << 7)                                    /* Bit 7: IN1L PGA Mute */
#define   WM8994_IN1L_MUTE_DISABLE                (0)                                         /* Disabled */
#define   WM8994_IN1L_MUTE_ENABLE                 (WM8994_IN1L_MUTE)                          /* Enabled */
#define WM8994_IN1_VU                             (1 << 8)                                    /* Bit 8: Input PGA Voluem Update. Writing a 1 to this bit cause IN1L and IN1R input PGA volumes to updated simultaneously */

                                                  /* Bits 9-15: Reserved */

/* R25 (0x19) - Left Line Input 3&4 Volume
 */

#define WM8994_IN2L_VOL_SHIFT                     (0)                                         /* Bits 0-4: IN2L Volume */
#define   WM8994_IN2L_VOL_MIN                     (0 << WM8994_IN2L_VOL_SHIFT)                /* -16.5dB */
#define   WM8994_IN2L_VOL_DEFAULT                 (11 << WM8994_IN2L_VOL_SHIFT)               /* -16.5dB to +30dB in 1.5dB steps */
#define   WM8994_IN2L_VOL_MAX                     (31 << WM8994_IN2L_VOL_SHIFT)               /* +30dB */
                                                                                              /* Bit 5: Reserved */
#define WM8994_IN2L_ZC                            (1 << 6)                                    /* Bit 6: IN2L PGA Zero Cross Detector */ 
#define   WM8994_IN2L_ZC_NO                       (0)                                         /* Change gain immediately */
#define   WM8994_IN2L_ZC_YES                      (WM8994_IN2L_ZC)                            /* Change gain on zero cross only */
#define WM8994_IN2L_MUTE                          (1 << 7)                                    /* Bit 7: IN2L PGA Mute */
#define   WM8994_IN2L_MUTE_DISABLE                (0)                                         /* Disabled */
#define   WM8994_IN2L_MUTE_ENABLE                 (WM8994_IN2L_MUTE)                          /* Enabled */
#define WM8994_IN2_VU                             (1 << 8)                                    /* Bit 8: Input PGA Voluem Update. Writing a 1 to this bit cause IN2L and IN2R input PGA volumes to updated simultaneously */

                                                  /* Bits 9-15: Reserved */

/* R26 (0x1A) - Right Line Input 1&2 Volume
 */

#define WM8994_IN1R_VOL_SHIFT                     (0)                                         /* Bits 0-4: IN1R Volume */
#define   WM8994_IN1R_VOL_MIN                     (0 << WM8994_IN1R_VOL_SHIFT)                /* -16.5dB */
#define   WM8994_IN1R_VOL_DEFAULT                 (11 << WM8994_IN1R_VOL_SHIFT)               /* -16.5dB to +30dB in 1.5dB steps */
#define   WM8994_IN1R_VOL_MAX                     (31 << WM8994_IN1R_VOL_SHIFT)               /* +30dB */
                                                                                              /* Bit 5: Reserved */
#define WM8994_IN1R_ZC_SHIFT                      (6)                                         /* Bit 6: IN1R PGA Zero Cross Detector */ 
#define   WM8994_IN1R_ZC_NO                       (0)                                         /* Change gain immediately */
#define   WM8994_IN1R_ZC_YES                      (1 << WM8994_IN1R_ZC_SHIFT)                 /* Change gain on zero cross only */
#define WM8994_IN1R_MUTE_SHIFT                    (7)                                         /* Bit 7: IN1R PGA Mute */
#define   WM8994_IN1R_MUTE_DISABLE                (0)                                         /* Disabled */
#define   WM8994_IN1R_MUTE_ENABLE                 (WM8994_IN1R_MUTE_SHIFT)                    /* Enabled */
#if 0
#define WM8994_IN1_VU                             (1 << 8)                                    /* Bit 8: Input PGA Voluem Update. Writing a 1 to this bit cause IN1L and IN1R input PGA volumes to updated simultaneously */
#endif
                                                  /* Bits 9-15: Reserved */

/* R27 (0x1B) - Right Line Input 3&4 Volume
 */

#define WM8994_IN2R_VOL_SHIFT                     (0)                                         /* Bits 0-4: IN2R Volume */
#define   WM8994_IN2R_VOL_MIN                     (0 << WM8994_IN2R_VOL_SHIFT)                /* -16.5dB */
#define   WM8994_IN2R_VOL_DEFAULT                 (11 << WM8994_IN2R_VOL_SHIFT)               /* -16.5dB to +30dB in 1.5dB steps */
#define   WM8994_IN2R_VOL_MAX                     (31 << WM8994_IN2R_VOL_SHIFT)               /* +30dB */
                                                                                              /* Bit 5: Reserved */
#define WM8994_IN2R_ZC_SHIFT                      (6)                                         /* Bit 6: IN2R PGA Zero Cross Detector */ 
#define   WM8994_IN2R_ZC_NO                       (0)                                         /* Change gain immediately */
#define   WM8994_IN2R_ZC_YES                      (1 << WM8994_IN2R_ZC_SHIFT)                 /* Change gain on zero cross only */
#define WM8994_IN2R_MUTE_SHIFT                    (7)                                         /* Bit 7: IN2R PGA Mute */
#define   WM8994_IN2R_MUTE_DISABLE                (0)                                         /* Disabled */
#define   WM8994_IN2R_MUTE_ENABLED                (1 << WM8994_IN2R_MUTE_SHIFT)               /* Enabled */
#if 0
#define WM8994_IN2_VU                             (1 << 8)                                    /* Bit 8: Input PGA Voluem Update. Writing a 1 to this bit cause IN2L and IN2R input PGA volumes to updated simultaneously */
#endif

/* R28 (0x1C) - Left Output Volume
 */

#define WM8994_HPOUT1L_VOL_SHIFT                  (0)                                         /* Bits 0-5: HPOUT1LVOL (Left Headphone Output PGA) Volume */
#define   WM8994_HPOUT1L_VOL_MIN                  (0 << WM8994_HPOUT1L_VOL_SHIFT)             /* -57dB */
#define   WM8994_HPOUT1L_VOL_DEFAULT              (0 << WM8994_HPOUT1L_VOL_SHIFT)             /* -57dB to +6dB in 1 dB steps*/
#define   WM8994_HPOUT1L_VOL_MAX                  (0x3F << WM8994_HPOUT1L_VOL_SHIFT)          /* +6dB */
#  define WM8994_HPOUT1L_VOL(n)                   ((uint16_t)(n) << WM8994_HPOUT1L_VOL_SHIFT) /* Set volume to defined value */
#define WM8994_HPOUT1L_MUTE_N_SHIFT               (6)                                         /* Bit 6: HPOUT1LVOL (Left Headphone Output PGA) Mute */
#define   WM8994_HPOUT1L_MUTE_N_YES               (0)                                         /* Mute */
#define   WM8994_HPOUT1L_MUTE_N_NO                (1 << WM8994_HPOUT1L_MUTE_N_SHIFT)          /* Un-Mute */
#define WM8994_HPOUT1L_ZC_SHIFT                   (7)                                         /* Bit 7: HPOUT1LVOL (Left Headphone Output PGA) Zero Cross */
#define   WM8994_HPOUT1L_ZC_DIABLED               (0)                                         /* Zero cross disabled */
#define   WM8994_HPOUT1L_ZC_ENABLED               (1 << WM8994_HPOUT1L_ZC_SHIFT)              /* Zero cross enabled */
#define WM8994_HPOUT1_VU_SHIFT                    (8)                                         /* Bit 8: Headphone Output PGA Volume Update */
#define   WM8994_HPOUT1_VU_DISABLE                (0) 
#define   WM8994_HPOUT1_VU_ENABLED                (1 << WM8994_HPOUT1_VU_SHIFT)               /* Writing a 1 to this bit will update HPOUT1LVOL and
                                                                                               * HPOUT1RVOL volumes simultaneously */

/* R29 (0x1D) - Right Output Volume
 */

#define WM8994_HPOUT1R_VOL_SHIFT                  (0)                                         /* Bits 0-5: HPOUT1RVOL (Right Headphone Output PGA) Volume */
#define   WM8994_HPOUT1R_VOL_MIN                  (0 << WM8994_HPOUT1R_VOL_SHIFT)             /* -57dB */
#define   WM8994_HPOUT1R_VOL_DEFAULT              (0 << WM8994_HPOUT1R_VOL_SHIFT)             /* -57dB to +6dB in 1 dB steps*/
#define   WM8994_HPOUT1R_VOL_MAX                  (0x3F << WM8994_HPOUT1R_VOL_SHIFT)          /* +6dB */
#  define WM8994_HPOUT1R_VOL(n)                   ((uint16_t)(n) << WM8994_HPOUT1R_VOL_SHIFT) /* Set volume to defined value */
#define WM8994_HPOUT1R_MUTE_N_SHIFT               (6)                                         /* Bit 6: HPOUT1RVOL (Left Headphone Output PGA) Mute */
#define   WM8994_HPOUT1R_MUTE_N_YES               (0)                                         /* Mute */
#define   WM8994_HPOUT1R_MUTE_N_NO                (1 << WM8994_HPOUT1R_MUTE_N_SHIFT)          /* Un-Mute */
#define WM8994_HPOUT1R_ZC_SHIFT                   (7)                                         /* Bit 7: HPOUT1RVOL (Left Headphone Output PGA) Zero Cross */
#define   WM8994_HPOUT1R_ZC_DIABLED               (0)                                         /* Zero cross disabled */
#define   WM8994_HPOUT1R_ZC_ENABLED               (1 << WM8994_HPOUT1R_ZC_SHIFT)              /* Zero cross enabled */
#if 0
#define WM8994_HPOUT1_VU_SHIFT                    (8)                                         /* Bit 8: Headphone Output PGA Volume Update */
#define   WM8994_HPOUT1_VU_DISABLE                (0)
#define   WM8994_HPOUT1_VU_ENABLED                (1 << WM8994_HPOUT1_VU_SHIFT)               /*  Writing a 1 to this bit will update HPOUT1LVOL and
                                                                                               *  HPOUT1RVOL volumes simultaneously */
#endif

/* R30 (0x1E) - Line Outputs Volume
 */

#define WM8994_LINEOUT2_VOL_SHIFT                 (0)                                         /* LINEOUT2 Line Output Volume */
#define   WM8994_LINEOUT2_VOL_0dB                 (0)                                         /* 0dB */
#define   WM8994_LINEOUT2_VOL_n6dB                (1 << WM8994_LINEOUT2_VOL_SHIFT)            /* -6dB */
#define WM8994_LINEOUT2P_MUTE                     (1)                                         /* LINEOUT2P Line Output Mute */

/* R31 (0x1F) - HPOUT2 Volume
 */

/* R32 (0x20) - Left OPGA Volume
 */

/* R33 (0x21) - Right OPGA Volume
 */

/* R34 (0x22) - SPKMIXL Attenuation
 */

/* R35 (0x23) - SPKMIXR Attenuation
 */

/* R36 (0x24) - SPKOUT Mixers
 */

/* R37 (0x25) - ClassD
 */

/* R38 (0x26) - Speaker Volume Left
 */

/* R39 (0x27) - Speaker Volume Right
 */

/* R40 (0x28) - Input Mixer (2)
 */

/* R41 (0x29) - Input Mixer (3)
 */

/* R42 (0x2A) - Input Mixer (4)
 */

/* R43 (0x2B) - Input Mixer (5)
 */

/* R44 (0x2C) - Input Mixer (6)
 */

/* R45 (0x2D) - Output Mixer (1)
 */

#define WM8994_DAC1L_TO_MIXOUTL                   (1 << 0)                                    /* Bit 0: Left DAC1 to MIXOUTL Mute */
#define   WM8994_DAC1L_TO_MIXOUTL_MUTE            (0)                                         /* Mute */
#define   WM8994_DAC1L_TO_MIXOUTL_UNMUTE          (WM8994_DAC1L_TO_MIXOUTL)                   /* Un-mute */
#define WM8994_IN2LP_TO_MIXOUTL                   (1 << 1)                                    /* Bit 1: IN2LP to MIXOUTL Mute */
#define   WM8994_IN2LP_TO_MIXOUTL_MUTE            (0)                                         /* Mute */
#define   WM8994_IN2LP_TO_MIXOUTL_UNMUTE          (WM8994_IN2LP_TO_MIXOUTL)                   /* Un-mute */
#define WM8994_IN1L_TO_MIXOUTL                    (1 << 2)                                    /* Bit 2: IN1L PGA Output to MIXOUTL Mute */
#define   WM8994_IN1L_TO_MIXOUTL_MUTE             (0)                                         /* Mute */
#define   WM8994_IN1L_TO_MIXOUTL_UNMUTE           (WM8994_IN1L_TO_MIXOUTL_MUTE)               /* Un-mute */                 
#define WM8994_IN1R_TO_MIXOUTL                    (1 << 3)                                    /* Bit 3: IN1R PGA Output to MIXOUTL Mute */
#define   WM8994_IN1R_TO_MIXOUTL_MUTE             (0)                                         /* Mute */
#define   WM8994_IN1R_TO_MIXOUTL_UNMUTE           (WM8994_IN1R_TO_MIXOUTL_MUTE)               /* Un-mute */            
#define WM8994_IN2LN_TO_MIXOUTL                   (1 << 4)                                    /* Bit 4: IN2LN to MIXOUTL Mute */
#define   WM8994_IN2LN_TO_MIXOUTL_MUTE            (0)                                         /* Mute */
#define   WM8994_IN2LN_TO_MIXOUTL_UNMUTE          (WM8994_IN2LN_TO_MIXOUTL)                   /* Un-mute */
#define WM8994_IN2RN_TO_MIXOUTL                   (1 << 5)                                    /* Bit 5: IN2RN to MIXOUTL Mute */
#define   WM8994_IN2RN_TO_MIXOUTL_MUTE            (0)                                         /* Mute */
#define   WM8994_IN2RN_TO_MIXOUTL_UNMUTE          (WM8994_IN2RN_TO_MIXOUTL)                   /* Un-mute */
#define WM8994_MIXINL_TO_MIXOUTL                  (1 << 6)                                    /* Bit 6: MIXINL Output(Left ADC bypass) to MIXOUTL Mute */
#define   WM8994_MIXINL_TO_MIXOUTL_MUTE           (0)                                         /* mute */
#define   WM8994_MIXINL_TO_MIXOUTL_UNMUTE         (WM8994_MIXINL_TO_MIXOUTL)                  /* Un-mute */
#define WM8994_MIXINR_TO_MIXOUTL                  (1 << 7)                                    /* Bit 7: MIXINR Output(Left ADC bypass) to MIXOUTL Mute */
#define   WM8994_MIXINR_TO_MIXOUTL_MUTE           (0)                                         /* mute */
#define   WM8994_MIXINR_TO_MIXOUTL_UNMUTE         (WM8994_MIXINR_TO_MIXOUTL)                  /* Un-mute */
#define WM8994_DAC1L_TO_HPOUT1L                   (1 << 8)                                    /* Bit 8: HPOUT1LVOL(Left Headphone Output PGA) Input Select */
#define   WM8994_DAC1L_TO_HPOUT1L_MIXOUTL         (0)                                         /* MIXOUTL */
#define   WM8994_DAC1L_TO_HPOUT1L_DAC1L           (WM8994_DAC1L_TO_HPOUT1L)                   /* DAC1L */
                                                                                              /* Bits 9-15: Reserved */

/* R46 (0x2E) - Output Mixer (2)
 */

#define WM8994_DAC1R_TO_MIXOUTR                   (1 << 0)                                    /* Bit 0: Right DAC1 to MIXOUTR Mute */
#define   WM8994_DAC1R_TO_MIXOUTR_MUTE            (0)                                         /* Mute */
#define   WM8994_DAC1R_TO_MIXOUTR_UNMUTE          (WM8994_DAC1R_TO_MIXOUTR)                   /* Un-mute */
#define WM8994_IN2RP_TO_MIXOUTR                   (1 << 1)                                    /* Bit 1: IN2RP to MIXOUTR Mute */
#define   WM8994_IN2RP_TO_MIXOUTR_MUTE            (0)                                         /* Mute */
#define   WM8994_IN2RP_TO_MIXOUTR_UNMUTE          (WM8994_IN2RP_TO_MIXOUTR)                   /* Un-mute */
#define WM8994_IN1R_TO_MIXOUTR                    (1 << 2)                                    /* Bit 2: IN1R PGA Output to MIXOUTR Mute */
#define   WM8994_IN1R_TO_MIXOUTR_MUTE             (0)                                         /* Mute */
#define   WM8994_IN1R_TO_MIXOUTR_UNMUTE           (WM8994_IN1R_TO_MIXOUTR_MUTE)               /* Un-mute */
#define WM8994_IN1L_TO_MIXOUTR                    (1 << 3)                                    /* Bit 3: IN1L PGA Output to MIXOUTR Mute */
#define   WM8994_IN1L_TO_MIXOUTR_MUTE             (0)                                         /* Mute */
#define   WM8994_IN1L_TO_MIXOUTR_UNMUTE           (WM8994_IN1L_TO_MIXOUTR_MUTE)               /* Un-mute */
#define WM8994_IN2RN_TO_MIXOUTR                   (1 << 4)                                    /* Bit 4: IN2RN to MIXOUTR Mute */
#define   WM8994_IN2RN_TO_MIXOUTR_MUTE            (0)                                         /* Mute */
#define   WM8994_IN2RN_TO_MIXOUTR_UNMUTE          (WM8994_IN2RN_TO_MIXOUTR)                   /* Un-mute */
#define WM8994_IN2LN_TO_MIXOUTR                   (1 << 5)                                    /* Bit 5: IN2LN to MIXOUTR Mute */
#define   WM8994_IN2LN_TO_MIXOUTR_MUTE            (0)                                         /* Mute */
#define   WM8994_IN2LN_TO_MIXOUTR_UNMUTE          (WM8994_IN2LN_TO_MIXOUTR)                   /* Un-mute */
#define WM8994_MIXINR_TO_MIXOUTR                  (1 << 6)                                    /* Bit 6: MIXINR Output(Left ADC bypass) to MIXOUTR Mute */
#define   WM8994_MIXINR_TO_MIXOUTR_MUTE           (0)                                         /* mute */
#define   WM8994_MIXINR_TO_MIXOUTR_UNMUTE         (WM8994_MIXINR_TO_MIXOUTR)                  /* Un-mute */
#define WM8994_MIXINL_TO_MIXOUTR                  (1 << 7)                                    /* Bit 7: MIXINL Output(Left ADC bypass) to MIXOUTR Mute */
#define   WM8994_MIXINL_TO_MIXOUTR_MUTE           (0)                                         /* mute */
#define   WM8994_MIXINL_TO_MIXOUTR_UNMUTE         (WM8994_MIXINL_TO_MIXOUTR)                  /* Un-mute */
#define WM8994_DAC1R_TO_HPOUT1R                   (1 << 8)                                    /* Bit 8: HPOUT1RVOL(Left Headphone Output PGA) Input Select */
#define   WM8994_DAC1R_TO_HPOUT1R_MIXOUTL         (0)                                         /* MIXOUTR */
#define   WM8994_DAC1R_TO_HPOUT1R_DAC1L           (WM8994_DAC1R_TO_HPOUT1R)                   /* DAC1R */

/* R47 (0x2F) - Output Mixer (3)
 */

/* R48 (0x30) - Output Mixer (4)
 */

/* R49 (0x31) - Output Mixer (5)
 */

/* R50 (0x32) - Output Mixer (6)
 */

/* R51 (0x33) - HPOUT2 Mixer
 */

/* R52 (0x34) - Line Mixer (1)
 */

/* R53 (0x35) - Line Mixer (2)
 */

/* R54 (0x36) - Speaker Mixer
 */

#define WM8994_DAC2L_TO_SPKMIXL                   (1 << 9)                                    /* Bit 9: Left DAC2 to SPKMXL Mute */
#define   WM8994_DAC2L_TO_SPKMIXL_MUTE            (0)                                         /* Mute */
#define   WM8994_DAC2L_TO_SPKMIXL_UNMUTE          (WM8994_DAC2L_TO_SPKMIXL)                   /* Un-mute */
#define WM8994_DAC2R_TO_SPKMIXR                   (1 << 8)                                    /* Bit 8: Right DAC2 to SPKMXL Mute */
#define   WM8994_DAC2R_TO_SPKMIXL_MUTE            (0)                                         /* Mute */
#define   WM8994_DAC2R_TO_SPKMIXL_UNMUTE          (WM8994_DAC2R_TO_SPKMIXL)                   /* Un-mute */
#define WM8994_MIXINL_TO_SPKMIXL                  (1 << 7)                                    /* Bit 7: MIXINL (Left ADC bypass) to SPKMIXL Mute */
#define   WM8994_MIXINL_TO_SPKMIXL_MUTE           (0)                                         /* Mute */
#define   WM8994_MIXINL_TO_SPKMIXL_UNMUTE         (WM8994_MIXINL_TO_SPKMIXL)                  /* Un-mute */
#define WM8994_MIXINR_TO_SPKMIXR                  (1 << 6)                                    /* Bit 6: MIXINR (Right ADC bypass) to SPKMIXR Mute */
#define   WM8994_MIXINR_TO_SPKMIXR_MUTE           (0)                                         /* Mute */
#define   WM8994_MIXINR_TO_SPKMIXR_UNMUTE         (WM8994_MIXINR_TO_SPKMIXR)                  /* Un-mute */
#define WM8994_IN1LP_TO_SPKMIXL                   (1 << 5)                                    /* Bit 5: IN1LP to SPKMIXL Mute */
#define   WM8994_IN1LP_TO_SPKMIXL_MUTE            (0)                                         /* Mute */
#define   WM8994_IN1LP_TO_SPKMIXL_UNMUTE          (WM8994_IN1LP_TO_SPKMIXL)                   /* Un-mute */
#define WM8994_IN1RP_TO_SPKMIXR                   (1 << 4)                                    /* Bit 4: IN1RP to SPKMIXR Mute */
#define   WM8994_IN1RP_TO_SPKMIXR_MUTE            (0)                                         /* Mute */
#define   WM8994_IN1RP_TO_SPKMIXR_UNMUTE          (WM8994_IN1RP_TO_SPKMIXR)                   /* Un-mute */
#define WM8994_MIXOUTL_TO_SPKMIXL                 (1 << 3)                                    /* Bit 3: MIXOUTL to SPKMIXL Mute */
#define   WM8994_MIXOUTL_TO_SPKMIXL_MUTE          (0)                                         /* Mute */
#define   WM8994_MIXOUTL_TO_SPKMIXL_UNMUTE        (WM8994_MIXOUTL_TO_SPKMIXL)                 /* Un-mute */
#define WM8994_MIXOUTR_TO_SPKMIXR                 (1 << 2)                                    /* Bit 2: MIXOUTR to SPKMIXR Mute */
#define   WM8994_MIXOUTR_TO_SPKMIXR_MUTE          (0)                                         /* Mute */
#define   WM8994_MIXOUTR_TO_SPKMIXR_UNMUTE        (WM8994_MIXOUTR_TO_SPKMIXR)                 /* Un-mute */
#define WM8994_DAC1L_TO_SPKMIXL                   (1 << 1)                                    /* Bit 1: DAC1L to SPKMIXL Mute */
#define   WM8994_DAC1L_TO_SPKMIXL_MUTE            (0)                                         /* Mute */
#define   WM8994_DAC1L_TO_SPKMIXL_UNMUTE          (WM8994_DAC1L_TO_SPKMIXL)                   /* Un-mute */
#define WM8994_DAC1R_TO_SPKMIXR                   (1 << 0)                                    /* Bit 0: DAC1R to SPKMIXR Mute */
#define   WM8994_DAC1R_TO_SPKMIXR_MUTE            (0)                                         /* Mute */
#define   WM8994_DAC1R_TO_SPKMIXR_UNMUTE          (WM8994_DAC1R_TO_SPKMIXR)                   /* Un-mute */

/* R55 (0x37) - Additional Control
 */

/* R56 (0x38) - AntiPOP (1)
 */

/* R57 (0x39) - AntiPOP (2)
 */

                                                  /* Bits 8-15: Reserved */
#define WM8994_MICB2_DISCH                        (1 << 8)                                    /* Bit 7: Microphone Bias 2 Discharge */
#define   WM8994_MICB2_DISCH_FLOAT                (0)                                         /* MICBIAS2 floating when disabled */
#define   WM8994_MICB2_DISCH_DISCHARGED           WM8994_MICB2_DISCH                          /* MICBIAS2 disharged when disabled */
#define WM8994_MICB1_DISCH                        (1 << 7)                                    /* Bit 7: Microphone Bias 1 Discharge */
#define   WM8994_MICB1_DISCH_FLOAT                (0)                                         /* MICBIAS1 floating when disabled */
#define   WM8994_MICB1_DISCH_DISCHARGED           WM8994_MICB1_DISCH                          /* MICBIAS1 disharged when disabled */

#define WM8994_VMID_DISCH                         (1 << 0)                                    /* Bit 0:Connects VMID to ground */
#define   WM8994_VMID_DISCH_DISABLE               (0)                                         /* Disabled */
#define   WM8994_VMID_DISCH_ENABLE                WM8994_VMID_DISCH                           /* Enabled */
#define WM8994_BIAS_SRC                           (1 << 1)                                    /* Bit 1: Selects the bias current source */
#  define WM8994_BIAS_SRC_NORMAL_BIAS             (0)                                         /* Normal bias */ 
#  define WM8994_BIAS_SRC_STARTUP_BIAS            WM8994_BIAS_SRC                             /* Start-Up bias */ 
#define WM8994_STARTUP_BIAS_ENA                   (1 << 2)                                    /* Bit 2: Enables the Start-Up bias current generator */
#define WM8994_VMID_BUF_ENA                       (1 << 3)                                    /* Bit 3: VMID Buffer Enable */
#define WM8994_VMID_RAMP_SHIFT                    (5)                                         /* Bits 5-6: VMID soft start enable/slew rate control */
#define   WM8994_VMID_RAMP_MASK                   (3 << WM8994_VMID_RAMP_SHIFT)
#define   WM8994_VMID_RAMP_NORMAL_SLOW_START      (0 << WM8994_VMID_RAMP_SHIFT)               /* Normal slow start */
#define   WM8994_VMID_RAMP_NORMAL_FAST_START      (1 << WM8994_VMID_RAMP_SHIFT)               /* Normal fast start */
#define   WM8994_VMID_RAMP_SOFT_SLOW_START        (2 << WM8994_VMID_RAMP_SHIFT)               /* Soft slow start */
#define   WM8994_VMID_RAMP_SOFT_FAST_START        (3 << WM8994_VMID_RAMP_SHIFT)               /* Soft fast start */

/* R58 (0x3A) - MICBIAS
 */

/* R59 (0x3B) - LDO 1
 */

/* R60 (0x3C) - LDO 2
 */

/* R61 (0x3D) - MICBIAS1
 */

/* R62 (0x3E) - MICBIAS2
 */

/* R210 (0xD2) - Mic Detect 3
 */

/* R76 (0x4C) - Charge Pump (1)
 */

#define WM8994_CP_ENA                             (1 << 15)                                   /* Bit 15: Enable charge-pump digits */
#define   WM8994_CP_ENA_DISABLE                   (0)                                         /* Diable */
#define   WM8994_CP_ENA_ENABLE                    (WM8994_CP_ENA)                             /* Enable */

/* R77 (0x4D) - Charge Pump (2)
 */

#define WM8994_CP_DISCH                           (1 << 15)                                   /* Bit 15: Charge Pump Discharge Select */
#define   WM8994_CP_DISCH_FLOAT                   (0)                                         /* Charge Pump outputs floating when disabled */
#define   WM8994_CP_DISCH_DISCHARGE               (WM8994_CP_DISCH)                           /* Charge Pump outputs discharged when disabled */

/* R81 (0x51) - Class W (1)
 */

#define WM8994_CP_DYN_SRC_SEL_SHIFT               8                                           /* Bits 8-9: Selects the digitial audio source for
                                                                                               * envelope tracking */
#define   WM8994_CP_DYN_SRC_SEL_MASK              (3 << WM8994_CP_DYN_SRC_SEL_SHIFT)
#define   WM8994_CP_DYN_SRC_SEL_AIF1_TS0          (0 << WM8994_CP_DYN_SRC_SEL_SHIFT)          /* AIF1, DAC Timeslot 0 */
#define   WM8994_CP_DYN_SRC_SEL_AIF1_TS1          (1 << WM8994_CP_DYN_SRC_SEL_SHIFT)          /* AIF1, DAC Timeslot 1 */
#define   WM8994_CP_DYN_SRC_SEL_AIF2_DATA         (2 << WM8994_CP_DYN_SRC_SEL_SHIFT)          /* AIF2, DAC data */

#define WM8994_CP_DYN_PWR                         (1 << 0)                                    /* Bit 0: Enable dynamic charge pump power control */
#define   WM8994_CP_DYN_PWR_CG                    (0)                                         /* Charge pump controlled by volume register (Class G) */
#define   WM8994_CP_DYN_PWR_CW                    (WM8994_CP_DYN_PWR)                         /* Charge pump controlled by real-time audio lev. (Class W) */

/* R84 (0x54) - DC Servo (1)
 */

#define WM8994_DCS_TRIG_SINGLE_1                  (1 << 13)                                   /* Bit 13: Writing 1 to this bit selects a single DC offset
                                                                                               * correction for HPOUT1R. In readback, a value of 1
                                                                                               * indicates that the DC Servo single correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_TRIG_SINGLE_0                  (1 << 12)                                   /* Bit 12: Writing 1 to this bit selects a single DC offset
                                                                                               * correction for HPOUT1L. In readback, a value of 1
                                                                                               * indicates that the DC Servo single correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_TRIG_SERIES_1                  (1 << 9)                                    /* Bit 9: Writing 1 to this bit selects a series of DC offset
                                                                                               * corrections for HPOUT1R. In readback, a value of 1
                                                                                               * indicates that the DC Servo DAC write correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_TRIG_SERIES_0                  (1 << 8)                                    /* Bit 8: Writing 1 to this bit selects a series of DC offset
                                                                                               * corrections for HPOUT1L. In readback, a value of 1
                                                                                               * indicates that the DC Servo DAC write correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_TRIG_STARTUP_1                 (1 << 5)                                    /* Bit 5: Writing 1 to this bit selects Start-Up DC
                                                                                               * Servo mode for HPOUT1R. In readback, a value of 1
                                                                                               * indicates that the DC Servo Start-Up correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_TRIG_STARTUP_0                 (1 << 4)                                    /* Bit 4: Writing 1 to this bit selects Start-Up DC
                                                                                               * Servo mode for HPOUT1L. In readback, a value of 1
                                                                                               * indicates that the DC Servo Start-Up correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_TRIG_DAC_WR_1                  (1 << 3)                                    /* Bit 3: Writing 1 to this bit selects DAC Write
                                                                                               * DC Servo mode for HPOUT1R. In readback, a value of 1
                                                                                               * indicates that the DC Servo DAC Write correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_TRIG_DAC_WR_0                  (1 << 2)                                    /* Bit 2: Writing 1 to this bit selects DAC Write
                                                                                               * DC Servo mode for HPOUT1L. In readback, a value of 1
                                                                                               * indicates that the DC Servo DAC Write correction is
                                                                                               * in progress
                                                                                               */
#define WM8994_DCS_ENA_CHAN_1                     (1 << 1)                                    /* Bit 1: DC Servo enable for HPOUT1R */
#define   WM8994_DCS_ENA_CHAN_1_DISABLE           (0)                                         /* Diable */
#define   WM8994_DCS_ENA_CHAN_1_ENABLE            (WM8994_DCS_ENA_CHAN_1)                     /* Enable */
#define WM8994_DCS_ENA_CHAN_0                     (1 << 0)                                    /* Bit 0: DC Servo enable for HPOUT1L */
#define   WM8994_DCS_ENA_CHAN_0_DISABLE           (0)                                         /* Diable */
#define   WM8994_DCS_ENA_CHAN_0_ENABLE            (WM8994_DCS_ENA_CHAN_0)                     /* Enable */

/* R85 (0x55) - DC Servo (2)
 */

/* R87 (0x57) - DC Servo (4)
 */

/* R88 (0x58) - DC Servo Readback
 */

/* R96 (0x60) - Analogue HP (1)
 */

#define WM8994_HPOUT1L_RMV_SHORT                  (1 << 7)                                    /* Bit 7: Removes HPOUT1L short */
#define   WM8994_HPOUT1L_RMV_SHORT_DISABLE        (0)                                         /* HPOUT1L short diabled */
#define   WM8994_HPOUT1L_RMV_SHORT_ENABLE         (WM8994_HPOUT1L_RMV_SHORT)                  /* HPOUT1L short enabled */
#define WM8994_HPOUT1L_OUTP                       (1 << 6)                                    /* Bit 6: Enables HPOUT1L output stage */
#define   WM8994_HPOUT1L_OUTP_DISABLE             (0)                                         /* Diable */
#define   WM8994_HPOUT1L_OUTP_ENABLE              (WM8994_HPOUT1L_OUTP)                       /* Enable */
#define WM8994_HPOUT1L_DLY                        (1 << 5)                                    /* Bit 5: Enables HPOUT1L intermediate stage */
#define   WM8994_HPOUT1L_DLY_DISABLE              (0)                                         /* Diable */
#define   WM8994_HPOUT1L_DLY_ENABLE               (WM8994_HPOUT1L_DLY)                        /* Enable */
#define WM8994_HPOUT1R_RMV_SHORT                  (1 << 3)                                    /* Bit 3: Removes HPOUT1R short */
#define   WM8994_HPOUT1R_RMV_SHORT_DISABLE        (0)                                         /* HPOUT1R short diabled */
#define   WM8994_HPOUT1R_RMV_SHORT_ENABLE         (WM8994_HPOUT1R_RMV_SHORT)                  /* HPOUT1R short enabled */
#define WM8994_HPOUT1R_OUTP                       (1 << 2)                                    /* Bit 2: Enables HPOUT1R output stage */
#define   WM8994_HPOUT1R_OUTP_DISABLE             (0)                                         /* Diable */
#define   WM8994_HPOUT1R_OUTP_ENABLE              (WM8994_HPOUT1R_OUTP)                       /* Enable */
#define WM8994_HPOUT1R_DLY                        (1 << 1)                                    /* Bit 1: Enables HPOUT1R intermediate stage */
#define   WM8994_HPOUT1R_DLY_DISABLE              (0)                                         /* Diable */
#define   WM8994_HPOUT1R_DLY_ENABLE               (WM8994_HPOUT1R_DLY)                        /* Enable */

/* R208 (0xD0) - Mic Detect 1
 */

/* R209 (0xD1) - Mic Detect 2
 */

/* R210 (0xD2) - Mic Detect 3
 */

/* R256 (0x100) - Chip Revision
 */

/** R257 (0x101) - Control Interface
 */

/* R272 (0x110) - Write Sequencer Ctrl (1)
 */

#define WM8994_WSEQ_ENA                           (1 << 15)                                   /* Bit 15: Write Sequencer Enable */
#define   WM8994_WSEQ_ENA_DISABLE                 (0)                                         /* Diable */
#define   WM8994_WSEQ_ENA_ENABLE                  (WM8994_WSEQ_ENA)                           /* Enable */
#define WM8994_WSEQ_ABORT                         (1 << 9)                                    /* Bit 9: Writing 1 to this bit aborts the current seq. */
#define WM8994_WSEQ_START                         (1 << 8)                                    /* Bit 8: Writing 1 to this bit starts the seq. */
#define WM8994_WSEQ_START_INDEX_SHIFT             (0)                                         /* Bits 0-6: Sequence start index */
#define WM8994_WSEQ_START_INDEX_MASK              (0x7F << WM8994_WSEQ_START_INDEX_SHIFT)

/* R273 (0x111) - Write Sequencer Ctrl (2)
 */

#define WM8994_WSEQ_BUSY                          (1 << 8)                                    /* Bit 8: Sequencer busy flag (read only) */
#define WM8994_WSEQ_CURRENT_INDEX_SHIFT           (0)                                         /* Bits 0-6: Sequence current index */
#define   WM8994_WSEQ_CURRENT_INDEX_MASK          (0x7F << W8994_WSEQ_CURRENT_INDEX_SHIFT)

/* R512 (0x200) - AIF1 Clocking (1)
 */

#define WM8994_AIF1CLK_ENA                        (1 << 0)                                    /* Bit 0: AIF1CLK Enable */ 
#define WM8994_AIF1CLK_DIV                        (1 << 1)                                    /* Bit 1: AIF1CLK Divider */
#define   WM8994_AIF1CLK_DIV_0                    (0)                                         /* AIF1CLK/1 */
#define   WM8994_AIF1CLK_DIV_1                    (WM8994_AIF1CLK_DIV)                        /* AIF1CLK/2 */
#define WM8994_AIF1CLK_INV                        (1 << 2)                                    /* Bit 2: AIF1CLK Invert */
#define   WM8994_AIF1CLK_INV_NOT                  (0)                                         /* AIF1CLK not inverted */
#define   WM8994_AIF1CLK_INV_YES                  (WM8994_AIF1CLK_INV)                        /* AIF1CLK inverted */
#define WM8994_AIF1CLK_SRC_SHIFT                  (3)                                         /* Bit 3-4: AIF1CLK Source Select */
#define WM8994_AIF1CLK_SRC_MASK                   (3 << WM8994_AIF1CLK_SRC_SHIFT)
#define   WM8994_AIF1CLK_SRC_MCLK1                (0 << WM8994_AIF1CLK_SRC_SHIFT)             /* MCLK1 */
#define   WM8994_AIF1CLK_SRC_MCLK2                (1 << WM8994_AIF1CLK_SRC_SHIFT)             /* MCLK2 */
#define   WM8994_AIF1CLK_SRC_PLL1                 (2 << WM8994_AIF1CLK_SRC_SHIFT)             /* PLL1 */
#define   WM8994_AIF1CLK_SRC_PLL2                 (3 << WM8994_AIF1CLK_SRC_SHIFT)             /* PLL2 */
                                                                                              /* Bits 5-15: Reserved */

/* R513 (0x201) - AIF1 Clocking (2)
 */

/* R516 (0x204) - AIF2 Clocking (1)
 */

/* R517 (0x205) - AIF2 Clocking (2)
 */

/* R520 (0x208) - Clocking (1)
 */

#define WM8994_SYSCLK_SRC                         (1 << 0)                                    /* Bit 0: SYSCLK Source Select */
#define   WM8994_SYSCLK_SRC_AIF1CLK               (0)                                         /* AIF1CLK */
#define   WM8994_SYSCLK_SRC_AIF2CLK               (WM8994_SYSCLK_SRC)                         /* AIF2CLK */
#define WM8994_SYSDSPCLK_ENA                      (1 << 1)                                    /* Bit 1: Digital Mixing Processor Clock Enable */
#define WM8994_AIF2DSPCLK_ENA                     (1 << 2)                                    /* Bit 2: AIF2 Processor Clock Enable */
#define WM8994_AIF1DSPCLK_ENA                     (1 << 3)                                    /* Bit 3: AIF1 Processor Clock Enable */
#define WM8994_TOCLK_ENA                          (1 << 4)                                    /* Bit 4: Slow Clock(TOCLK) Enable */
                                                                                              /* Bits 5-15: Reserved */

/* R521 (0x209) - Clocking (2)
 */

/* R528 (0x210) - AIF1 Rate
 */

#define WM8994_AIF1CLK_RATE_SHIFT                 (0)                                         /* Bits 0-3: Selects the AIF1CLK/fs ratio */
#define WM8994_AIF1CLK_RATE_MASK                  (0xf << WM8994_AIF1CLK_RATE_SHIFT) 
#define   WM8994_AIF1CLK_RATE_0                   (0 << WM8994_AIF1CLK_RATE_SHIFT)            /* Reserved */
#define   WM8994_AIF1CLK_RATE_1                   (1 << WM8994_AIF1CLK_RATE_SHIFT)            /* 128 */
#define   WM8994_AIF1CLK_RATE_2                   (2 << WM8994_AIF1CLK_RATE_SHIFT)            /* 192 */
#define   WM8994_AIF1CLK_RATE_3                   (3 << WM8994_AIF1CLK_RATE_SHIFT)            /* 256 */
#define   WM8994_AIF1CLK_RATE_4                   (4 << WM8994_AIF1CLK_RATE_SHIFT)            /* 384 */
#define   WM8994_AIF1CLK_RATE_5                   (5 << WM8994_AIF1CLK_RATE_SHIFT)            /* 512 */
#define   WM8994_AIF1CLK_RATE_6                   (6 << WM8994_AIF1CLK_RATE_SHIFT)            /* 768 */
#define   WM8994_AIF1CLK_RATE_7                   (7 << WM8994_AIF1CLK_RATE_SHIFT)            /* 1024 */
#define   WM8994_AIF1CLK_RATE_8                   (8 << WM8994_AIF1CLK_RATE_SHIFT)            /* 1408 */
#define   WM8994_AIF1CLK_RATE_9                   (9 << WM8994_AIF1CLK_RATE_SHIFT)            /* 1536 */
#define WM8994_AIF1_SR_SHIFT                      (4)                                         /* Bits 4-7: Selects the AIF1 Sample Rate (fs) */
#define WM8994_AIF1_SR_MASK                       (0xf << WM8994_AIF1_SR_SHIFT)
#define   WM8994_AIF1_SR_8K                       (0 << WM8994_AIF1_SR_SHIFT)                 /* 8kHz */
#define   WM8994_AIF1_SR_11K                      (1 << WM8994_AIF1_SR_SHIFT)                 /* 11.025kHz */
#define   WM8994_AIF1_SR_12K                      (2 << WM8994_AIF1_SR_SHIFT)                 /* 12kHz */
#define   WM8994_AIF1_SR_16K                      (3 << WM8994_AIF1_SR_SHIFT)                 /* 16kHz */
#define   WM8994_AIF1_SR_22K                      (4 << WM8994_AIF1_SR_SHIFT)                 /* 22.05kHz */
#define   WM8994_AIF1_SR_24K                      (5 << WM8994_AIF1_SR_SHIFT)                 /* 24kHz */
#define   WM8994_AIF1_SR_32K                      (6 << WM8994_AIF1_SR_SHIFT)                 /* 32kHz */
#define   WM8994_AIF1_SR_44K                      (7 << WM8994_AIF1_SR_SHIFT)                 /* 44.1kHz */
#define   WM8994_AIF1_SR_48K                      (8 << WM8994_AIF1_SR_SHIFT)                 /* 48kHz */
#define   WM8994_AIF1_SR_88K                      (9 << WM8994_AIF1_SR_SHIFT)                 /* 88.2kHz */
#define   WM8994_AIF1_SR_96K                      (10 << WM8994_AIF1_SR_SHIFT)                /* 96kHz */
                                                                                              /* Bits 8-15: Reserved */

/* R529 (0x211) - AIF2 Rate
 */

/* R530 (0x212) - Rate Status
 */

/* R544 (0x220) - FLL1 Control (1)
 */

/* R545 (0x221) - FLL1 Control (2)
 */

/* R546 (0x222) - FLL1 Control (3)
 */

/* R547 (0x223) - FLL1 Control (4)
 */

/* R548 (0x224) - FLL1 Control (5)
 */

/* R550 (0x226) - FLL1 EFS 1
 */

/* R551 (0x227) - FLL1 EFS 2
 */

/* R576 (0x240) - FLL2 Control (1)
 */

/* R577 (0x241) - FLL2 Control (2)
 */

/* R578 (0x242) - FLL2 Control (3)
 */

/* R579 (0x243) - FLL2 Control (4)
 */

/* R580 (0x244) - FLL2 Control (5)
 */

/* R582 (0x246) - FLL2 EFS 1
 */

/* R583 (0x247) - FLL2 EFS 2
 */

/* R768 (0x300) - AIF1 Control (1)
 */

                                                  /* Bits 0-2: Reserved */
#define WM8994_AIF1_FMT_SHIFT                     (3)                                         /* Bits 3-4: AIF1 Digital Audio Interface Format */
#define WM8994_AIF1_FMT_MASK                      (3 << WM8994_AIF1_FMT_SHIFT) 
#define   WM8994_AIF1_FMT_RIGHT                   (0 << WM8994_AIF1_FMT_SHIFT)                /* Right justified */ 
#define   WM8994_AIF1_FMT_LEFT                    (1 << WM8994_AIF1_FMT_SHIFT)                /* Left justified */
#define   WM8994_AIF1_FMT_I2S                     (2 << WM8994_AIF1_FMT_SHIFT)                /* I2S Format */
#define   WM8994_AIF1_FMT_DSP                     (3 << WM8994_AIF1_FMT_SHIFT)                /* DSP Mode */
#define WM8994_AIF1_WL_SHIFT                      (5)                                         /* Bits 5-6: AIF1 Digital Audio Interface Word Length */
#define WM8994_AIF1_WL_MASK                       (3 << WM8994_AIF1_WL_SHIFT)
#define   WM8994_AIF1_WL_16BITS                   (0 << WM8994_AIF1_WL_SHIFT)                 /* 16 bits */
#define   WM8994_AIF1_WL_20BITS                   (1 << WM8994_AIF1_WL_SHIFT)                 /* 20 bits */
#define   WM8994_AIF1_WL_24BITS                   (2 << WM8994_AIF1_WL_SHIFT)                 /* 24 bits */
#define   WM8994_AIF1_WL_32BITS                   (3 << WM8994_AIF1_WL_SHIFT)                 /* 32 bits */
#define WM8994_AIF1ADC_TDM                        (1 << 13)                                   /* Bit 13: AIF1 transmit (ADC) TDM control */
#define   WM8994_AIF1ADC_TDM_0                    (0)                                         /* ADCDAT1 driver logic '0' when not transmit data */
#define   WM8994_AIF1ADC_TDM_TRI                  (WM8994_AIF1ADC_TDM)                        /* ADCDAT1 is tri-stated when not transmit data */
#define WM8994_AIF1ADCR_SRC                       (1 << 14)                                   /* Bit 14: AIF1 Right Audio Interface Source */                                           
#define   WM8994_AIF1ADCR_LEFT_ADC                (0)                                         /* Left ADC data is output on right channel */
#define   WM8994_AIF1ADCR_RIGHT_ADC               (WM8994_AIF1ADCR_SRC)                       /* Right ADC data is output on right channel */
#define WM8994_AIF1ADCL_SRC                       (1 << 15)                                   /* Bit 15: AIF1 Left Audio Interface Source */                                           
#define   WM8994_AIF1ADCL_LEFT_ADC                (0)                                         /* Left ADC data is output on left channel */
#define   WM8994_AIF1ADCL_RIGHT_ADC               (WM8994_AIF1ADCL_SRC)                       /* Right ADC data is output on left channel */

/* R769 (0x301) - AIF1 Control (2)
 */

/* R770 (0x302) - AIF1 Master/Slave
 */

                                                  /* Bits 0-11: Reserved */
#define WM8994_AIF1_LRCLK_FRC                     (1 << 12)                                   /* Bit 12: Forces LRCLK1 and ADCLRCLK1 to enabled when all AIF1 audio channels are disabled */
#define   WM8994_AIF1_LRCLK_FRC_NORMAL            (0)                                         /* Normal */
#define   WM8994_AIF1_LRCLK_FRC_YES               (WM8994_AIF1_LRCLK_FRC)                     /* LRCLK1 and ADCLRCLK1 always enabled in Master Mode */
#define WM8994_AIF1_CLK_FRC                       (1 << 13)                                   /* Bit 13: Forces BCLK1, LRCLK1 and ADCLRCLK1 to enabled when all AIF1 audio channels are disabled */
#define   WM8994_AIF1_CLK_FRC_NORMAL              (0)                                         /* Normal */
#define   WM8994_AIF1_CLK_FRC_YES                 (WM8994_AIF1_CLK_FRC)                       /* BLCK1, LRCLK1 and ADCLRCLK1 always enabled in Master Mode */
#define WM8994_AIF1_MSTR                          (1 << 14)                                   /* Bit 14: AIF1 Audio Interface Master Mode Select */
#define   WM8994_AIF1_MSTR_SLAVE_MODE             (0)                                         /* Slave Mode */
#define   WM8994_AIF1_MSTR_MASTER_MODE            (WM8994_AIF1_MSTR)                          /* Master Mode */
#define WM8994_AIF1_TRI                           (1 << 15)                                   /* Bit 15: AIF1 Audio Interface tri-state */
#define   WM8994_AIF1_TRI_NORMAL                  (0)                                         /* AIF1 pins operate normally */ 
#define   WM8994_AIF1_TRI_TRI                     (WM8994_AIF1_TRI)                           /* Tri-state all AIF1 interface pins */

/* R771 (0x303) - AIF1 BCLK
 */

/* R772 (0x304) - AIF1ADC LRCLK
 */

/* R773 (0x305) - AIF1DAC LRCLK
 */

/* R774 (0x306) - AIF1DAC Data
 */

/* R775 (0x307) - AIF1ADC Data
 */

/* R784 (0x310) - AIF2 Control (1)
 */

/* R785 (0x311) - AIF2 Control (2)
 */

/* R786 (0x312) - AIF2 Master/Slave
 */

/* R787 (0x313) - AIF2 BCLK
 */

/* R788 (0x314) - AIF2ADC LRCLK
 */

/* R789 (0x315) - AIF2DAC LRCLK
 */

/* R790 (0x316) - AIF2DAC Data
 */

/* R791 (0x317) - AIF2ADC Data
 */

/* R800 (0x320) - AIF3 Control (1)
 */

/* R801 (0x321) - AIF3 Control (2)
 */

/* R802 (0x322) - AIF3DAC Data
 */

/* R803 (0x323) - AIF3ADC Data
 */

/* R1024 (0x400) - AIF1 ADC1 Left Volume
 */

/* R1025 (0x401) - AIF1 ADC1 Right Volume
 */

/* R1026 (0x402) - AIF1 DAC1 Left Volume
 */

#define WM8994_AIF1DAC1_VU                        (1 << 8)                                    /* Bit 8:  AIF1DAC1 input path (AIF1, TS 0) Vol Update */
#define WM8994_AIF1DAC1L_VOL_SHIFT                (0)                                         /* Bits 0-7: AIF1DAC1 (Left) input path, Digital Vol. */
#define WM8994_AIF1DAC1L_VOL_MASK                 (0xFF << WM8994_AIF1DAC1L_VOL_SHIFT)

/* R1027 (0x403) - AIF1 DAC1 Right Volume
 */

#define WM8994_AIF1DAC1R_VOL_SHIFT                (0)                                         /* Bits 0-7: AIF1DAC1 (Right) input path, Digital Vol. */
#define WM8994_AIF1DAC1R_VOL_MASK                 (0xFF << WM8994_AIF1DAC1R_VOL_SHIFT)

/* R1028 (0x404) - AIF1 ADC2 Left Volume
 */

/* R1029 (0x405) - AIF1 ADC2 Right Volume
 */

/* R1030 (0x406) - AIF1 DAC2 Left Volume
 */

#define WM8994_AIF1DAC2_VU                        (1 << 8)                                    /* Bit 8:  AIF1DAC2 input path (AIF1, TS 1) Vol Update */
#define WM8994_AIF1DAC2L_VOL_SHIFT                (0)                                         /* Bits 0-7: AIF1DAC2 (Left) input path, Digital Vol. */
#define WM8994_AIF1DAC2L_VOL_MASK                 (0xFF << WM8994_AIF1DAC2L_VOL_SHIFT)

/* R1031 (0x407) - AIF1 DAC2 Right Volume
 */

#define WM8994_AIF1DAC2R_VOL_SHIFT                (0)                                         /* Bits 0-7: AIF1DAC2 (Right) input path, Digital Vol. */
#define WM8994_AIF1DAC2R_VOL_MASK                 (0xFF << WM8994_AIF1DAC2R_VOL_SHIFT)

/* R1040 (0x410) - AIF1 ADC1 Filters
 */

/* R1041 (0x411) - AIF1 ADC2 Filters
 */

#define WM8994_AIF1ADC2_HPF_CUT_SHIFT             (13)                                        /* Bits 13-14: AIF1ADC2 output path (AIF1, TS 1), HPF CO */
#define WM8994_AIF1ADC2_HPF_CUT_MASK              (3 << WM8994_AIF1ADC2_HPF_CUT_SHIFT)
#define WM8994_AIF1ADC2_HPF_CUT_HIFI              (0 << WM8994_AIF1ADC2_HPF_CUT_SHIFT)        /* Hi-fi mode (fc = 4 Hz at fs = 48kHz) */
#define WM8994_AIF1ADC2_HPF_CUT_VOICE1            (1 << WM8994_AIF1ADC2_HPF_CUT_SHIFT)        /* Voice mode 1 (fc = 127 Hz at fs = 8kHz) */
#define WM8994_AIF1ADC2_HPF_CUT_VOICE2            (2 << WM8994_AIF1ADC2_HPF_CUT_SHIFT)        /* Voice mode 2 (fc = 130 Hz at fs = 8kHz) */
#define WM8994_AIF1ADC2_HPF_CUT_VOICE3            (3 << WM8994_AIF1ADC2_HPF_CUT_SHIFT)        /* Voice mode 3 (fc = 267 Hz at fs = 8kHz) */
#define WM8994_AIF1ADC2L_HPF                      (1 << 12)                                   /* Bit 12: AIF1ADC2 (Left) output path (AIF1, TS 1) Dig. HPF */
#define   WM8994_AIF1ADC2L_HPF_DISABLE            (0)                                         /* Disable */
#define   WM8994_AIF1ADC2L_HPF_ENABLE             (WM8994_AIF1ADC2L_HPF)                      /* Enable */
#define WM8994_AIF1ADC2R_HPF                      (1 << 11)                                   /* Bit 11: AIF1ADC2 (Right) output path (AIF1, TS 1) Dig. HPF */
#define   WM8994_AIF1ADC2R_HPF_DISABLE            (0)                                         /* Disable */
#define   WM8994_AIF1ADC2R_HPF_ENABLE             (WM8994_AIF1ADC2R_HPF)                      /* Enable */

/* R1056 (0x420) - AIF1 DAC1 Filters (1)
 */

#define WM8994_AIF1DAC1_MUTE                      (1 << 9)                                    /* Bit 9: AIF1DAC1 input path (AIF1, TS 0) Soft Mute Control */
#define   WM8994_AIF1DAC1_MUTE_UNMUTE             (0)                                         /* Un-mute */
#define   WM8994_AIF1DAC1_MUTE_MUTE               (WM8994_AIF1DAC1_MUTE)                      /* Mute */
#define WM8994_AIF1DAC1_MONO                      (1 << 7)                                    /* Bit 7: AIF1DAC1 input path (AIF1, TS 0) Mono Mix Control */
#define   WM8994_AIF1DAC1_MONO_DISABLE            (0)                                         /* Disabled */
#define   WM8994_AIF1DAC1_MONO_ENABLE             (WM8994_AIF1DAC1_MONO)                      /* Enabled */
#define WM8994_AIF1DAC1_MUTERATE                  (1 << 5)                                    /* Bit 5: AIF1DAC1 input path (AIF1, TS 0) Soft Mute Ramp Rate */
#define   WM8994_AIF1DAC1_MUTERATE_FAST           (0)                                         /* Fast ramp (fs/2, maximum ramp time is 10.7ms at fs=48kHz */
#define   WM8994_AIF1DAC1_MUTERATE_SLOW           (WM8994_AIF1DAC1_MUTERATE)                  /* Slow ramp (fs/32, maximum ramp time is 171ms at fs=48kHz */
#define WM8994_AIF1DAC1_UNMUTE_RAMP               (1 << 4)                                    /* Bit 4: AIF1DAC1 input path (AIF1, TS 0) Unmute Ramp select */
#define   WM8994_AIF1DAC1_UNMUTE_RAMP_IMMEDIATE   (0)                                         /* Volume change immediately to AIF1DAC1L_VOL */
#define   WM8994_AIF1DAC1_UNMUTE_RAMP_GRADUAL     (WM8994_AIF1DAC1_UNMUTE_RAMP)               /* Volume change gradually to AIF1DAC1R_VOL */
#define WM8994_AIF1DAC1_DEEMP_SHIFT               (1)                                         /* Bit 1-2: AIF1DAC1 input path (AIF1, TS 0), De-Emphasis */
#define WM8994_AIF1DAC1_DEEMP_MASK                (3 << WM8994_AIF1DAC1_DEEMP_SHIFT)
#define   WM8994_AIF1DAC1_DEEMP_NO                (0 << WM8994_AIF1DAC1_DEEMP_SHIFT)          /* No de-emphasis */
#define   WM8994_AIF1DAC1_DEEMP_32KHZ             (1 << WM8994_AIF1DAC1_DEEMP_SHIFT)          /* 32kHz sample rate */
#define   WM8994_AIF1DAC1_DEEMP_44KHZ             (2 << WM8994_AIF1DAC1_DEEMP_SHIFT)          /* 44.1kHz sample rate */
#define   WM8994_AIF1DAC1_DEEMP_48KHZ             (3 << WM8994_AIF1DAC1_DEEMP_SHIFT)          /* 48kHz sample rate*/

/* R1057 (0x421) - AIF1 DAC1 Filters (2)
 */

/* R1058 (0x422) - AIF1 DAC2 Filters (1)
 */

#define WM8994_AIF1DAC2_MUTE                      (1 << 9)                                    /* Bit 9: AIF1DAC2 input path (AIF1, TS 1) Soft Mute Control */
#define   WM8994_AIF1DAC2_MUTE_UNMUTE             (0)                                         /* Un-mute */
#define   WM8994_AIF1DAC2_MUTE_MUTE               (WM8994_AIF1DAC2_MUTE)                      /* Mute */
#define WM8994_AIF1DAC2_MONO                      (1 << 7)                                    /* Bit 7: AIF1DAC2 input path (AIF1, TS 1) Mono Mix Control */
#define   WM8994_AIF1DAC2_MONO_DISABLE            (0)                                         /* Disabled */
#define   WM8994_AIF1DAC2_MONO_ENABLE             (WM8994_AIF1DAC2_MONO)                      /* Enabled */
#define WM8994_AIF1DAC2_MUTERATE                  (1 << 5)                                    /* Bit 5: AIF1DAC2 input path (AIF1, TS 1) Soft Mute Ramp Rate */
#define   WM8994_AIF1DAC2_MUTERATE_FAST           (0)                                         /* Fast ramp (fs/2, maximum ramp time is 10.7ms at fs=48kHz */
#define   WM8994_AIF1DAC2_MUTERATE_SLOW           (WM8994_AIF1DAC2_MUTERATE)                  /* Slow ramp (fs/32, maximum ramp time is 171ms at fs=48kHz */
#define WM8994_AIF1DAC2_UNMUTE_RAMP               (1 << 4)                                    /* Bit 4: AIF1DAC2 input path (AIF1, TS 1) Unmute Ramp select */
#define   WM8994_AIF1DAC2_UNMUTE_RAMP_IMMEDIATE   (0)                                         /* Volume change immediately to AIF1DAC1L_VOL */
#define   WM8994_AIF1DAC2_UNMUTE_RAMP_GRADUAL     (WM8994_AIF1DAC2_UNMUTE_RAMP)               /* Volume change gradually to AIF1DAC1R_VOL */
#define WM8994_AIF1DAC2_DEEMP_SHIFT               (1)                                         /* Bit 1-2: AIF1DAC2 input path (AIF1, TS 1), De-Emphasis */
#define WM8994_AIF1DAC2_DEEMP_MASK                (3 << WM8994_AIF1DAC2_DEEMP_SHIFT)
#define   WM8994_AIF1DAC2_DEEMP_NO                (0 << WM8994_AIF1DAC2_DEEMP_SHIFT)          /* No de-emphasis */
#define   WM8994_AIF1DAC2_DEEMP_32KHZ             (1 << WM8994_AIF1DAC2_DEEMP_SHIFT)          /* 32kHz sample rate */
#define   WM8994_AIF1DAC2_DEEMP_44KHZ             (2 << WM8994_AIF1DAC2_DEEMP_SHIFT)          /* 44.1kHz sample rate */
#define   WM8994_AIF1DAC2_DEEMP_48KHZ             (3 << WM8994_AIF1DAC2_DEEMP_SHIFT)          /* 48kHz sample rate*/

/* R1059 (0x423) - AIF1 DAC2 Filters (2)
 */

/* R1072 (0x430) - AIF1 DAC1 Noise Gate
 */

/* R1073 (0x431) - AIF1 DAC2 Noise Gate
 */

/* R1088 (0x440) - AIF1 DRC1 (1)
 */

/* R1089 (0x441) - AIF1 DRC1 (2)
 */

/* R1090 (0x442) - AIF1 DRC1 (3)
 */

/* R1091 (0x443) - AIF1 DRC1 (4)
 */

/* R1092 (0x444) - AIF1 DRC1 (5)
 */

/* R1104 (0x450) - AIF1 DRC2 (1)
 */

/* R1105 (0x451) - AIF1 DRC2 (2)
 */

/* R1106 (0x452) - AIF1 DRC2 (3)
 */

/* R1107 (0x453) - AIF1 DRC2 (4)
 */

/* R1108 (0x454) - AIF1 DRC2 (5)
 */

/* R1152 (0x480) - AIF1 DAC1 EQ Gains (1)
 */

/* R1153 (0x481) - AIF1 DAC1 EQ Gains (2)
 */

/* R1154 (0x482) - AIF1 DAC1 EQ Band 1 A
 */

/* R1155 (0x483) - AIF1 DAC1 EQ Band 1 B
 */

/* R1156 (0x484) - AIF1 DAC1 EQ Band 1 PG
 */

/* R1157 (0x485) - AIF1 DAC1 EQ Band 2 A
 */

/* R1158 (0x486) - AIF1 DAC1 EQ Band 2 B
 */

/* R1159 (0x487) - AIF1 DAC1 EQ Band 2 C
 */

/* R1160 (0x488) - AIF1 DAC1 EQ Band 2 PG
 */

/* R1161 (0x489) - AIF1 DAC1 EQ Band 3 A
 */

/* R1162 (0x48A) - AIF1 DAC1 EQ Band 3 B
 */

/* R1163 (0x48B) - AIF1 DAC1 EQ Band 3 C
 */

/* R1164 (0x48C) - AIF1 DAC1 EQ Band 3 PG
 */

/* R1165 (0x48D) - AIF1 DAC1 EQ Band 4 A
 */

/* R1166 (0x48E) - AIF1 DAC1 EQ Band 4 B
 */

/* R1167 (0x48F) - AIF1 DAC1 EQ Band 4 C
 */

/* R1168 (0x490) - AIF1 DAC1 EQ Band 4 PG
 */

/* R1169 (0x491) - AIF1 DAC1 EQ Band 5 A
 */

/* R1170 (0x492) - AIF1 DAC1 EQ Band 5 B
 */

/* R1171 (0x493) - AIF1 DAC1 EQ Band 5 PG
 */

/* R1184 (0x4A0) - AIF1 DAC2 EQ Gains (1)
 */

/* R1185 (0x4A1) - AIF1 DAC2 EQ Gains (2)
 */

/* R1186 (0x4A2) - AIF1 DAC2 EQ Band 1 A
 */

/* R1187 (0x4A3) - AIF1 DAC2 EQ Band 1 B
 */

/* R1188 (0x4A4) - AIF1 DAC2 EQ Band 1 PG
 */

/* R1189 (0x4A5) - AIF1 DAC2 EQ Band 2 A
 */

/* R1190 (0x4A6) - AIF1 DAC2 EQ Band 2 B
 */

/* R1191 (0x4A7) - AIF1 DAC2 EQ Band 2 C
 */

/* R1192 (0x4A8) - AIF1 DAC2 EQ Band 2 PG
 */

/* R1193 (0x4A9) - AIF1 DAC2 EQ Band 3 A
 */

/* R1194 (0x4AA) - AIF1 DAC2 EQ Band 3 B
 */

/* R1195 (0x4AB) - AIF1 DAC2 EQ Band 3 C
 */

/* R1196 (0x4AC) - AIF1 DAC2 EQ Band 3 PG
 */

/* R1197 (0x4AD) - AIF1 DAC2 EQ Band 4 A
 */

/* R1198 (0x4AE) - AIF1 DAC2 EQ Band 4 B
 */

/* R1199 (0x4AF) - AIF1 DAC2 EQ Band 4 C
 */

/* R1200 (0x4B0) - AIF1 DAC2 EQ Band 4 PG
 */

/* R1201 (0x4B1) - AIF1 DAC2 EQ Band 5 A
 */

/* R1202 (0x4B2) - AIF1 DAC2 EQ Band 5 B
 */

/* R1203 (0x4B3) - AIF1 DAC2 EQ Band 5 PG
 */

/* R1280 (0x500) - AIF2 ADC Left Volume
 */

/* R1281 (0x501) - AIF2 ADC Right Volume
 */

/* R1282 (0x502) - AIF2 DAC Left Volume
 */

/* R1283 (0x503) - AIF2 DAC Right Volume
 */

/* R1296 (0x510) - AIF2 ADC Filters
 */

/* R1312 (0x520) - AIF2 DAC Filters (1)
 */

/* R1313 (0x521) - AIF2 DAC Filters (2)
 */

/* R1328 (0x530) - AIF2 DAC Noise Gate
 */

/* R1344 (0x540) - AIF2 DRC (1)
 */

/* R1345 (0x541) - AIF2 DRC (2)
 */

/* R1346 (0x542) - AIF2 DRC (3)
 */

/* R1347 (0x543) - AIF2 DRC (4)
 */

/* R1348 (0x544) - AIF2 DRC (5)
 */

/* R1408 (0x580) - AIF2 EQ Gains (1)
 */

/* R1409 (0x581) - AIF2 EQ Gains (2)
 */

/* R1410 (0x582) - AIF2 EQ Band 1 A
 */

/* R1411 (0x583) - AIF2 EQ Band 1 B
 */

/* R1412 (0x584) - AIF2 EQ Band 1 PG
 */

/* R1413 (0x585) - AIF2 EQ Band 2 A
 */

/* R1414 (0x586) - AIF2 EQ Band 2 B
 */

/* R1415 (0x587) - AIF2 EQ Band 2 C
 */

/* R1416 (0x588) - AIF2 EQ Band 2 PG
 */

/* R1417 (0x589) - AIF2 EQ Band 3 A
 */

/* R1418 (0x58A) - AIF2 EQ Band 3 B
 */

/* R1419 (0x58B) - AIF2 EQ Band 3 C
 */

/* R1420 (0x58C) - AIF2 EQ Band 3 PG
 */

/* R1421 (0x58D) - AIF2 EQ Band 4 A
 */

/* R1422 (0x58E) - AIF2 EQ Band 4 B
 */

/* R1423 (0x58F) - AIF2 EQ Band 4 C
 */

/* R1424 (0x590) - AIF2 EQ Band 4 PG
 */

/* R1425 (0x591) - AIF2 EQ Band 5 A
 */

/* R1426 (0x592) - AIF2 EQ Band 5 B
 */

/* R1427 (0x593) - AIF2 EQ Band 5 PG
 */

/* R1536 (0x600) - DAC1 Mixer Volumes
 */

/* R1537 (0x601) - DAC1 Left Mixer Routing
 */

#define WM8994_AIF1DAC1L_TO_DAC1L_ENA             (1 << 0)                                    /* Bit 0: Enable AIF1(Timeslot 0, Left) to DAC1L */
#define WM8994_AIF1DAC2L_TO_DAC1L_ENA             (1 << 1)                                    /* Bit 1: Enable AIF1(Timeslot 1, Left) to DAC1L */
#define WM8994_AIF2DACL_TO_DAC1L_ENA              (1 << 2)                                    /* Bit 2: Enable AIF2(Left) to DAC1L */
                                                                                              /* Bit 3: Reserved */
#define WM8994_ADCL_TO_DAC1L_ENA                  (1 << 4)                                    /* Bit 4: Enable Sidetone STL to DAC1L */
#define WM8994_ADCR_TO_DAC1L_ENA                  (1 << 5)                                    /* Bit 5: Enable Sidetone STR to DAC1L */
                                                                                              /* Bits 6-15: Reserved */

/* R1538 (0x602) - DAC1 Right Mixer Routing
 */

#define WM8994_AIF1DAC1R_TO_DAC1R_ENA             (1 << 0)                                    /* Bit 0: Enable AIF1(Timeslot 0, Right) to DAC1R */
#define WM8994_AIF1DAC2R_TO_DAC1R_ENA             (1 << 1)                                    /* Bit 1: Enable AIF1(Timeslot 1, Right) to DAC1R */
#define WM8994_AIF2DACR_TO_DAC1R_ENA              (1 << 2)                                    /* Bit 2: Enable AIF2(Right) to DAC1R */
                                                                                              /* Bit 3: Reserved */
#define WM8994_ADCL_TO_DAC1R_ENA                  (1 << 4)                                    /* Bit 4: Enable Sidetone STL to DAC1R */
#define WM8994_ADCR_TO_DAC1R_ENA                  (1 << 5)                                    /* Bit 5: Enable Sidetone STR to DAC1R */
                                                                                              /* Bits 6-15: Reserved */

/* R1539 (0x603) - DAC2 Mixer Volumes
 */

#define WM8994_ADCR_DAC2_VOL_SHIFT                (5)                                         /* Bits 5-8: Sidetone STR to DAC2L and DAC2R Volume */
#define WM8994_ADCR_DAC2_VOL_MASK                 (0xF << WM8994_ADCR_DAC2_VOL_SHIFT)         /* 0000 = -36 DB, 1100 = 0dB */
#define WM8994_ADCL_DAC2_VOL_SHIFT                (0)                                         /* Bits 0-3: Sidetone STL to DAC2L and DAC2R Volume */
#define WM8994_ADCL_DAC2_VOL_MASK                 (0xF << WM8994_ADCL_DAC2_VOL_SHIFT          /* 0000 = -36 DB, 1100 = 0dB */

/* R1540 (0x604) - DAC2 Left Mixer Routing
 */

#define WM8994_AIF1DAC1L_TO_DAC2L_ENA             (1 << 0)                                    /* Bit 0: Enable AIF1(Timeslot 0, Left) to DAC2L */
#define WM8994_AIF1DAC2L_TO_DAC2L_ENA             (1 << 1)                                    /* Bit 1: Enable AIF1(Timeslot 1, Left) to DAC2L */
#define WM8994_AIF2DACL_TO_DAC2L_ENA              (1 << 2)                                    /* Bit 2: Enable AIF2(Left) to DAC2L */
                                                                                              /* Bit 3: Reserved */
#define WM8994_ADCL_TO_DAC2L_ENA                  (1 << 4)                                    /* Bit 4: Enable Sidetone STL to DAC2L */
#define WM8994_ADCR_TO_DAC2L_ENA                  (1 << 5)                                    /* Bit 5: Enable Sidetone STR to DAC2L */
                                                                                              /* Bits 6-15: Reserved */

/* R1541 (0x605) - DAC2 Right Mixer Routing
 */

#define WM8994_AIF1DAC1R_TO_DAC2R_ENA             (1 << 0)                                    /* Bit 0: Enable AIF1(Timeslot 0, Right) to DAC2R */
#define WM8994_AIF1DAC2R_TO_DAC2R_ENA             (1 << 1)                                    /* Bit 1: Enable AIF1(Timeslot 1, Right) to DAC2R */
#define WM8994_AIF2DACR_TO_DAC2R_ENA              (1 << 2)                                    /* Bit 2: Enable AIF2(Right) to DAC2R */
                                                                                              /* Bit 3: Reserved */
#define WM8994_ADCL_TO_DAC2R_ENA                  (1 << 4)                                    /* Bit 4: Enable Sidetone STL to DAC2R */
#define WM8994_ADCR_TO_DAC2R_ENA                  (1 << 5)                                    /* Bit 5: Enable Sidetone STR to DAC2R */
                                                                                              /* Bits 6-15: Reserved */

/* R1542 (0x606) - AIF1 ADC1 Left Mixer Routing
 */

/* R1543 (0x607) - AIF1 ADC1 Right Mixer Routing
 */

/* R1544 (0x608) - AIF1 ADC2 Left Mixer Routing
 */

/* R1545 (0x609) - AIF1 ADC2 Right mixer Routing
 */

/* R1552 (0x610) - DAC1 Left Volume
 */

#define WM8994_DAC1L_MUTE                         (1 << 9)                                    /* Bit 9: DAC1L Soft Mute Control */
#define   WM8994_DAC1L_MUTE_UNMUTE                (0)                                         /* DAC Un-mute */
#define   WM8994_DAC1L_MUTE_MUTE                  (WM8994_DAC1L_MUTE)                         /* DAC Mute */
#define WM8994_DAC1_VU                            (1 << 8)                                    /* Bit 8: DAC1L and DAC1R Volume Update */
#define WM8994_DAC1L_VOL_SHIFT                    (0)                                         /* Bits 0-7: DAC1L Digital Volume */
#define WM8994_DAC1L_VOL_MASK                     (0xFF << WM8994_DAC1L_VOL_SHIFT)

/* R1553 (0x611) - DAC1 Right Volume
 */

#define WM8994_DAC1R_MUTE                         (1 << 9)                                    /* Bit 9: DAC1R Soft Mute Control */
#define   WM8994_DAC1R_MUTE_UNMUTE                (0)                                         /* DAC Un-mute */
#define   WM8994_DAC1R_MUTE_MUTE                  (WM8994_DAC1L_MUTE)                         /* DAC Mute */
#define WM8994_DAC1R_VOL_SHIFT                    (0)                                         /* Bits 0-7: DAC1R Digital Volume */
#define WM8994_DAC1R_VOL_MASK                     (0xFF << WM8994_DAC1R_VOL_SHIFT)

/* R1554 (0x612) - DAC2 Left Volume
 */

#define WM8994_DAC2L_MUTE                         (1 << 9)                                    /* Bit 9: DAC2L Soft Mute Control */
#define   WM8994_DAC2L_MUTE_UNMUTE                (0)                                         /* DAC Un-mute */
#define   WM8994_DAC2L_MUTE_MUTE                  (WM8994_DAC2L_MUTE)                         /* DAC Mute */
#define WM8994_DAC2_VU                            (1 << 8)                                    /* Bit 8: DAC2L and DAC2R Volume Update */
#define WM8994_DAC2L_VOL_SHIFT                    (0)                                         /* Bits 0-7: DAC2L Digital Volume */
#define WM8994_DAC2L_VOL_MASK                     (0xFF << WM8994_DAC2L_VOL_SHIFT)

/* R1555 (0x613) - DAC2 Right Volume
 */

#define WM8994_DAC2R_MUTE                         (1 << 9)                                    /* Bit 9: DAC2R Soft Mute Control */
#define   WM8994_DAC2R_MUTE_UNMUTE                (0)                                         /* DAC Un-mute */
#define   WM8994_DAC2R_MUTE_MUTE                  (WM8994_DAC2R_MUTE)                         /* DAC Mute */
#define WM8994_DAC2R_VOL_SHIFT                    (0)                                         /* Bits 0-7: DAC2R Digital Volume */
#define WM8994_DAC2R_VOL_MASK                     (0xFF << WM8994_DAC2R_VOL_SHIFT)

/* R1556 (0x614) - DAC Softmute
 */

/* R1568 (0x620) - Oversampling
 */

/* R1569 (0x621) - Sidetone
 */

/* R1797 (0x705) - JACKDET Ctrl
 */

/* R1824 (0x720) - Pull Control (1)
 */

/* R1825 (0x721) - Pull Control (2)
 */

/* R1840 (0x730) - Interrupt Status 1
 */

/* R1841 (0x731) - Interrupt Status 2
 */

/* R1842 (0x732) - Interrupt Raw Status 2
 */

/* R1848 (0x738) - Interrupt Status 1 Mask
 */

/* R1849 (0x739) - Interrupt Status 2 Mask
 */

/* R1856 (0x740) - Interrupt Control
 */

/* R1864 (0x748) - IRQ Debounce
 */

/* Register Default Values */

/* Registers have some undocumented bits set on power up.  These probably
 * should be retained on writes (?).
 */

/* Register Bit Definitions */

/* 0x00 SW Reset and ID */

#define WM8994_SW_RST_DEV_ID1        0x8994

#define WM8994_AUDIO_FREQUENCY_8K                8000UL
#define WM8994_AUDIO_FREQUENCY_11_025K          11025UL
#define WM8994_AUDIO_FREQUENCY_16K              16000UL
#define WM8994_AUDIO_FREQUENCY_22_050K          22050UL
#define WM8994_AUDIO_FREQUENCY_32K              32000UL
#define WM8994_AUDIO_FREQUENCY_44_100K          44100UL
#define WM8994_AUDIO_FREQUENCY_48K              48000UL
#define WM8994_AUDIO_FREQUENCY_96K              96000UL
#define WM8994_AUDIO_FREQUENCY_192K            192000UL

#define WM8994_DEFAULT_SAMPRATE                 (WM8994_AUDIO_FREQUENCY_48K) 

#define WM8994_DEFAULT_NCHANNELS      0
#define WM8994_DEFAULT_BPSAMP      0
#define WM8994_BCLK_MAXDIV  30
#define WM8994_NFLLRATIO  30

#define WM8994_FRAMELEN8              16        /* Bits per frame for 8-bit data */
#define WM8994_FRAMELEN16             32        /* Bits per frame for 16-bit data */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct wm8994_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the WM8994 driver with respect to the board lower half driver).
   *
   * Terminology: Our "lower" half audio instances will be called dev for the
   * publicly visible version and "priv" for the version that only this
   * driver knows.  From the point of view of this driver, it is the board
   * lower "half" that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;             /* WM8994 audio lower half (this device) */

  /* Our specific driver data goes here */

  FAR const struct wm8994_lower_s *lower;   /* Pointer to the board lower functions */
  FAR struct i2c_master_s *i2c;             /* I2C driver to use */
  FAR struct i2s_dev_s   *i2s;              /* I2S driver to use */
  struct dq_queue_s       pendq;            /* Queue of pending buffers to be sent */
  struct dq_queue_s       doneq;            /* Queue of sent buffers to be returned */
  struct file             mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  uint32_t                bitrate;          /* Actual programmed bit rate */
  mutex_t                 pendlock;         /* Protect pendq */
#ifdef WM8994_USE_FFLOCK_INT
  struct work_s           work;             /* Interrupt work */
#endif
  uint16_t                samprate;         /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t                balance;          /* Current balance level (b16) */
#endif  /* CONFIG_AUDIO_EXCLUDE_BALANCE */
  uint8_t                 volume;           /* Current volume level {0..63} */
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 nchannels;        /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;           /* Bits per sample (8 or 16) */
  volatile uint8_t        inflight;         /* Number of audio buffers in-flight */
#ifdef WM8994_USE_FFLOCK_INT
  volatile bool           locked;           /* FLL is locked */
#endif
  bool                    running;          /* True: Worker thread is running */
  bool                    paused;           /* True: Playing is paused */
  bool                    mute;             /* True: Output is muted */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  bool                    terminating;      /* True: Stop requested */
#endif
  bool                    reserved;         /* True: Device is reserved */
  volatile int            result;           /* The result of the last transfer */
  uint16_t power_mgnt_reg_1;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_WM8994_CLKDEBUG
extern const uint8_t g_sysclk_scaleb1[WM8994_BCLK_MAXDIV + 1];
extern const uint8_t g_fllratio[WM8994_NFLLRATIO];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: wm8994_readreg
 *
 * Description:
 *    Read the specified 16-bit register from the WM8994 device.
 *
 ****************************************************************************/

#if defined(CONFIG_WM8994_REGDUMP) || defined(CONFIG_WM8994_CLKDEBUG)
struct wm8994_dev_s;
uint16_t wm8994_readreg(FAR struct wm8994_dev_s *priv, uint16_t regaddr);
#endif

#endif /* CONFIG_AUDIO */
#endif /* __DRIVERS_AUDIO_WM8994_H */
