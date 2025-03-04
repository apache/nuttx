/****************************************************************************
 * drivers/input/aw86225_reg.h
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

#ifndef __DRIVERS_INPUT_AW86225_REG_H
#define __DRIVERS_INPUT_AW86225_REG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AW86225_REG_ID                            (0x00)
#define AW86225_REG_SYSST                         (0x01)
#define AW86225_REG_SYSINT                        (0x02)
#define AW86225_REG_SYSINTM                       (0x03)
#define AW86225_REG_SYSST2                        (0x04)
#define AW86225_REG_SYSER                         (0x05)
#define AW86225_REG_PLAYCFG2                      (0x07)
#define AW86225_REG_PLAYCFG3                      (0x08)
#define AW86225_REG_PLAYCFG4                      (0x09)
#define AW86225_REG_WAVCFG1                       (0x0A)
#define AW86225_REG_WAVCFG2                       (0x0B)
#define AW86225_REG_WAVCFG3                       (0x0C)
#define AW86225_REG_WAVCFG4                       (0x0D)
#define AW86225_REG_WAVCFG5                       (0x0E)
#define AW86225_REG_WAVCFG6                       (0x0F)
#define AW86225_REG_WAVCFG7                       (0x10)
#define AW86225_REG_WAVCFG8                       (0x11)
#define AW86225_REG_WAVCFG9                       (0x12)
#define AW86225_REG_WAVCFG10                      (0x13)
#define AW86225_REG_WAVCFG11                      (0x14)
#define AW86225_REG_WAVCFG12                      (0x15)
#define AW86225_REG_WAVCFG13                      (0x16)
#define AW86225_REG_WAVCFG14                      (0x17)
#define AW86225_REG_CONTCFG1                      (0x18)
#define AW86225_REG_CONTCFG2                      (0x19)
#define AW86225_REG_CONTCFG3                      (0x1A)
#define AW86225_REG_CONTCFG4                      (0x1B)
#define AW86225_REG_CONTCFG5                      (0x1C)
#define AW86225_REG_CONTCFG6                      (0x1D)
#define AW86225_REG_CONTCFG7                      (0x1E)
#define AW86225_REG_CONTCFG8                      (0x1F)
#define AW86225_REG_CONTCFG9                      (0x20)
#define AW86225_REG_CONTCFG10                     (0x21)
#define AW86225_REG_CONTCFG11                     (0x22)
#define AW86225_REG_CONTCFG12                     (0x23)
#define AW86225_REG_CONTCFG13                     (0x24)
#define AW86225_REG_CONTRD14                      (0x25)
#define AW86225_REG_CONTRD15                      (0x26)
#define AW86225_REG_CONTRD16                      (0x27)
#define AW86225_REG_CONTRD17                      (0x28)
#define AW86225_REG_CONTRD18                      (0x29)
#define AW86225_REG_CONTRD19                      (0x2A)
#define AW86225_REG_CONTRD20                      (0x2B)
#define AW86225_REG_CONTRD21                      (0x2C)
#define AW86225_REG_RTPCFG1                       (0x2D)
#define AW86225_REG_RTPCFG2                       (0x2E)
#define AW86225_REG_RTPCFG3                       (0x2F)
#define AW86225_REG_RTPCFG4                       (0x30)
#define AW86225_REG_RTPCFG5                       (0x31)
#define AW86225_REG_RTPDATA                       (0x32)
#define AW86225_REG_TRGCFG1                       (0x33)
#define AW86225_REG_TRGCFG2                       (0x34)
#define AW86225_REG_TRGCFG3                       (0x35)
#define AW86225_REG_TRGCFG4                       (0x36)
#define AW86225_REG_TRGCFG5                       (0x37)
#define AW86225_REG_TRGCFG6                       (0x38)
#define AW86225_REG_TRGCFG7                       (0x39)
#define AW86225_REG_TRGCFG8                       (0x3A)
#define AW86225_REG_GLBCFG1                       (0x3B)
#define AW86225_REG_GLBCFG2                       (0x3C)
#define AW86225_REG_GLBCFG3                       (0x3D)
#define AW86225_REG_GLBCFG4                       (0x3E)
#define AW86225_REG_GLBRD5                        (0x3F)
#define AW86225_REG_RAMADDRH                      (0x40)
#define AW86225_REG_RAMADDRL                      (0x41)
#define AW86225_REG_RAMDATA                       (0x42)
#define AW86225_REG_SYSCTRL1                      (0x43)
#define AW86225_REG_SYSCTRL2                      (0x44)
#define AW86225_REG_SYSCTRL3                      (0x45)
#define AW86225_REG_SYSCTRL4                      (0x46)
#define AW86225_REG_SYSCTRL5                      (0x47)
#define AW86225_REG_SYSCTRL6                      (0x48)
#define AW86225_REG_SYSCTRL7                      (0x49)
#define AW86225_REG_PWMCFG1                       (0x4C)
#define AW86225_REG_PWMCFG2                       (0x4D)
#define AW86225_REG_PWMCFG3                       (0x4E)
#define AW86225_REG_PWMCFG4                       (0x4F)
#define AW86225_REG_TMCFG                         (0x50)
#define AW86225_REG_DETCFG1                       (0x51)
#define AW86225_REG_DETCFG2                       (0x52)
#define AW86225_REG_DET_RL                        (0x53)
#define AW86225_REG_DET_OS                        (0x54)
#define AW86225_REG_DET_VBAT                      (0x55)
#define AW86225_REG_DET_TEST                      (0x56)
#define AW86225_REG_DET_LO                        (0x57)
#define AW86225_REG_TRIMCFG1                      (0x58)
#define AW86225_REG_TRIMCFG3                      (0x5A)
#define AW86225_REG_TRIMCFG4                      (0x5B)
#define AW86225_REG_EFCFG1                        (0x5C)
#define AW86225_REG_EFCFG2                        (0x5D)
#define AW86225_REG_EFCFG3                        (0x5E)
#define AW86225_REG_EFCFG4                        (0x5F)
#define AW86225_REG_EFCFG5                        (0x60)
#define AW86225_REG_EFCFG6                        (0x61)
#define AW86225_REG_EFRD9                         (0x64)
#define AW86225_REG_EFRD10                        (0x65)
#define AW86225_REG_ANACFG1                       (0x6A)
#define AW86225_REG_ANACFG2                       (0x6B)
#define AW86225_REG_ANACFG3                       (0x6C)
#define AW86225_REG_ANACFG4                       (0x73)
#define AW86225_REG_ANACFG5                       (0x74)
#define AW86225_REG_ANACFG6                       (0x75)
#define AW86225_REG_ANACFG7                       (0x76)
#define AW86225_REG_ANACFG8                       (0x77)
#define AW86225_REG_SPACE                         (0x78)
#define AW86225_REG_MBIST                         (0x79)

#define REG_NONE_ACCESS                           (0)
#define REG_RD_ACCESS                             (1 << 0)
#define REG_WR_ACCESS                             (1 << 1)
#define AW86225_REG_MAX                           0xff

/* SYSST: reg 0x01 RO */

#define AW86225_BIT_SYSST_UVLS                    (1<<5)
#define AW86225_BIT_SYSST_FF_AES                  (1<<4)
#define AW86225_BIT_SYSST_FF_AFS                  (1<<3)
#define AW86225_BIT_SYSST_OCDS                    (1<<2)
#define AW86225_BIT_SYSST_OTS                     (1<<1)
#define AW86225_BIT_SYSST_DONES                   (1<<0)

/* SYSINT: reg 0x02 RC */

#define AW86225_BIT_SYSINT_UVLI                   (1<<5)
#define AW86225_BIT_SYSINT_FF_AEI                 (1<<4)
#define AW86225_BIT_SYSINT_FF_AFI                 (1<<3)
#define AW86225_BIT_SYSINT_OCDI                   (1<<2)
#define AW86225_BIT_SYSINT_OTI                    (1<<1)
#define AW86225_BIT_SYSINT_DONEI                  (1<<0)

/* SYSINTM: reg 0x03 RW */

#define AW86225_BIT_SYSINTM_UVLM_MASK             (~(1<<5))
#define AW86225_BIT_SYSINTM_UVLM_OFF              (1<<5)
#define AW86225_BIT_SYSINTM_UVLM_ON               (0<<5)
#define AW86225_BIT_SYSINTM_FF_AEM_MASK           (~(1<<4))
#define AW86225_BIT_SYSINTM_FF_AEM_OFF            (1<<4)
#define AW86225_BIT_SYSINTM_FF_AEM_ON             (0<<4)
#define AW86225_BIT_SYSINTM_FF_AFM_MASK           (~(1<<3))
#define AW86225_BIT_SYSINTM_FF_AFM_OFF            (1<<3)
#define AW86225_BIT_SYSINTM_FF_AFM_ON             (0<<3)
#define AW86225_BIT_SYSINTM_OCDM_MASK             (~(1<<2))
#define AW86225_BIT_SYSINTM_OCDM_OFF              (1<<2)
#define AW86225_BIT_SYSINTM_OCDM_ON               (0<<2)
#define AW86225_BIT_SYSINTM_OTM_MASK              (~(1<<1))
#define AW86225_BIT_SYSINTM_OTM_OFF               (1<<1)
#define AW86225_BIT_SYSINTM_OTM_ON                (0<<1)
#define AW86225_BIT_SYSINTM_DONEM_MASK            (~(1<<0))
#define AW86225_BIT_SYSINTM_DONEM_OFF             (1<<0)
#define AW86225_BIT_SYSINTM_DONEM_ON              (0<<0)

/* SYSST2: reg 0x04 RO */

#define AW86225_BIT_SYSST2_RAM_ADDR_ER            (1<<7)
#define AW86225_BIT_SYSST2_TRG_ADDR_ER            (1<<6)
#define AW86225_BIT_SYSST2_VBG_OK                 (1<<3)
#define AW86225_BIT_SYSST2_LDO_OK                 (1<<2)
#define AW86225_BIT_SYSST2_FF_FULL                (1<<1)
#define AW86225_BIT_SYSST2_FF_EMPTY               (1<<0)

/* SYSER: reg 0x05 RC */

#define AW86225_BIT_SYSER_I2S_ERR                 (1<<7)
#define AW86225_BIT_SYSER_TRIG1_EVENT             (1<<6)
#define AW86225_BIT_SYSER_TRIG2_EVENT             (1<<5)
#define AW86225_BIT_SYSER_TRIG3_EVENT             (1<<4)
#define AW86225_BIT_SYSER_OV                      (1<<3)
#define AW86225_BIT_SYSER_ADDR_ER                 (1<<2)
#define AW86225_BIT_SYSER_FF_ER                   (1<<1)
#define AW86225_BIT_SYSER_PLL_REF_ER              (1<<0)

/* PLAYCFG3: reg 0x08 RW */

#define AW86225_BIT_PLAYCFG3_STOP_MODE_MASK       (~(1<<5))
#define AW86225_BIT_PLAYCFG3_STOP_MODE_NOW        (1<<5)
#define AW86225_BIT_PLAYCFG3_STOP_MODE_LATER      (0<<5)
#define AW86225_BIT_PLAYCFG3_BRK_EN_MASK          (~(1<<2))
#define AW86225_BIT_PLAYCFG3_BRK_ENABLE           (1<<2)
#define AW86225_BIT_PLAYCFG3_BRK_DISABLE          (0<<2)
#define AW86225_BIT_PLAYCFG3_PLAY_MODE_MASK       (~(3<<0))
#define AW86225_BIT_PLAYCFG3_PLAY_MODE_STOP       (3<<0)
#define AW86225_BIT_PLAYCFG3_PLAY_MODE_CONT       (2<<0)
#define AW86225_BIT_PLAYCFG3_PLAY_MODE_RTP        (1<<0)
#define AW86225_BIT_PLAYCFG3_PLAY_MODE_RAM        (0<<0)

/* PLAYCFG4: reg 0x09 RW */

#define AW86225_BIT_PLAYCFG4_STOP_MASK            (~(1<<1))
#define AW86225_BIT_PLAYCFG4_STOP_ON              (1<<1)
#define AW86225_BIT_PLAYCFG4_STOP_OFF             (0<<1)
#define AW86225_BIT_PLAYCFG4_GO_MASK              (~(1<<0))
#define AW86225_BIT_PLAYCFG4_GO_ON                (1<<0)
#define AW86225_BIT_PLAYCFG4_GO_OFF               (0<<0)

/* WAVCFG1-8: reg 0x0A - reg 0x11 RW */

#define AW86225_BIT_WAVCFG_SEQWAIT_MASK           (~(1<<7))
#define AW86225_BIT_WAVCFG_SEQWAIT_TIME           (1<<7)
#define AW86225_BIT_WAVCFG_SEQWAIT_NUMBER         (0<<7)

/* WAVCFG9-12: reg 0x12 - reg 0x15 RW */

#define AW86225_BIT_WAVLOOP_SEQ_ODD_MASK          (~(0x0F<<4))
#define AW86225_BIT_WAVLOOP_SEQ_ODD_INIFINITELY   (0x0F<<4)
#define AW86225_BIT_WAVLOOP_SEQ_EVEN_MASK         (~(0x0F<<0))
#define AW86225_BIT_WAVLOOP_SEQ_EVEN_INIFINITELY  (0x0F<<0)
#define AW86225_BIT_WAVLOOP_INIFINITELY           (0x0F<<0)

/* WAVCFG9: reg 0x12 RW */

#define AW86225_BIT_WAVCFG9_SEQ1LOOP_MASK         (~(0x0F<<4))
#define AW86225_BIT_WAVCFG9_SEQ1LOOP_INIFINITELY  (0x0F<<4)
#define AW86225_BIT_WAVCFG9_SEQ2LOOP_MASK         (~(0x0F<<0))
#define AW86225_BIT_WAVCFG9_SEQ2LOOP_INIFINITELY  (0x0F<<0)

/* WAVCFG10: reg 0x13 RW */

#define AW86225_BIT_WAVCFG10_SEQ3LOOP_MASK        (~(0x0F<<4))
#define AW86225_BIT_WAVCFG10_SEQ3LOOP_INIFINITELY (0x0F<<4)
#define AW86225_BIT_WAVCFG10_SEQ4LOOP_MASK        (~(0x0F<<0))
#define AW86225_BIT_WAVCFG10_SEQ4LOOP_INIFINITELY (0x0F<<0)

/* WAVCFG11: reg 0x14 RW */

#define AW86225_BIT_WAVCFG11_SEQ5LOOP_MASK        (~(0x0F<<4))
#define AW86225_BIT_WAVCFG11_SEQ5LOOP_INIFINITELY (0x0F<<4)
#define AW86225_BIT_WAVCFG11_SEQ6LOOP_MASK        (~(0x0F<<0))
#define AW86225_BIT_WAVCFG11_SEQ6LOOP_INIFINITELY (0x0F<<0)

/* WAVCFG12: reg 0x15 RW */
#define AW86225_BIT_WAVCFG12_SEQ7LOOP_MASK        (~(0x0F<<4))
#define AW86225_BIT_WAVCFG12_SEQ7LOOP_INIFINITELY (0x0F<<4)
#define AW86225_BIT_WAVCFG12_SEQ8LOOP_MASK        (~(0x0F<<0))
#define AW86225_BIT_WAVCFG12_SEQ8LOOP_INIFINITELY (0x0F<<0)

/* WAVCFG13: reg 0x16 RW */

#define AW86225_BIT_WAVCFG13_WAITSLOT_MASK        (~(3<<5))
#define AW86225_BIT_WAVCFG13_WAITSLOT_DIV_1       (0<<5)
#define AW86225_BIT_WAVCFG13_WAITSLOT_DIV_8       (1<<5)
#define AW86225_BIT_WAVCFG13_WAITSLOT_DIV_64      (2<<5)
#define AW86225_BIT_WAVCFG13_WAITSLOT_DIV_512     (3<<5)
#define AW86225_BIT_WAVCFG13_AUTO_MD_MASK         (~(1<<4))
#define AW86225_BIT_WAVCFG13_AUTO_MD_CONT_MODE    (1<<4)
#define AW86225_BIT_WAVCFG13_AUTO_MD_SIN_WAV      (0<<4)
#define AW86225_BIT_WAVCFG13_MAINLOOP_MASK        (~(0x0F<<0))
#define AW86225_BIT_WAVCFG13_MAINLOOP_INIFINITELY (0x0F<<0)

/* CONTCFG1: reg 0x18 RW */

#define AW86225_BIT_CONTCFG1_EDGE_FRE_MASK        (~(0x0F<<4))
#define AW86225_BIT_CONTCFG1_EN_F0_DET_MASK       (~(1<<3))
#define AW86225_BIT_CONTCFG1_F0_DET_ENABLE        (1<<3)
#define AW86225_BIT_CONTCFG1_F0_DET_DISABLE       (0<<3)
#define AW86225_BIT_CONTCFG1_SIN_MODE_MASK        (~(1<<0))
#define AW86225_BIT_CONTCFG1_SIN_MODE_COS         (1<<0)
#define AW86225_BIT_CONTCFG1_SIN_MODE_SINE        (0<<0)

/* CONTCFG5: reg 0x1C RW */

#define AW86225_BIT_CONTCFG5_BRK_GAIN_MASK        (~(0x0F<<0))

/* CONTCFG6: reg 0x1D RW */

#define AW86225_BIT_CONTCFG6_TRACK_EN_MASK        (~(1<<7))
#define AW86225_BIT_CONTCFG6_TRACK_ENABLE         (1<<7)
#define AW86225_BIT_CONTCFG6_TRACK_DISABLE        (0<<7)
#define AW86225_BIT_CONTCFG6_DRV1_LVL_MASK        (~(0x7F<<0))

/* CONTCFG7: reg 0x1E RW */

#define AW86225_BIT_CONTCFG7_DRV2_LVL_MASK        (~(0x7F<<0))

/* CONTCFG13: reg 0x24 RW */

#define AW86225_BIT_CONTCFG13_TSET_MASK           (~(0x0F<<4))
#define AW86225_BIT_CONTCFG13_BEME_SET_MASK       (~(0x0F<<0))

/* RTPCFG1: reg 0x2D RW */

#define AW86225_BIT_RTPCFG1_ADDRH_MASK            (~(0x0F<<0))

#define AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_MASK     (~(1<<5))
#define AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_EN       (1<<5)
#define AW86225_BIT_RTPCFG1_SRAM_SIZE_2K_DIS      (0<<5)

#define AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_MASK     (~(1<<4))
#define AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_EN       (1<<4)
#define AW86225_BIT_RTPCFG1_SRAM_SIZE_1K_DIS      (0<<4)

/* RTPCFG3: reg 0x2F RW */

#define AW86225_BIT_RTPCFG3_FIFO_AEH_MASK         (~(0x0F<<4))
#define AW86225_BIT_RTPCFG3_FIFO_AFH_MASK         (~(0x0F<<0))

#define AW86225_BIT_TRG_ENABLE_MASK               (~(1<<7))
#define AW86225_BIT_TRG_ENABLE                    (1<<7)
#define AW86225_BIT_TRG_DISABLE                   (0<<7)
#define AW86225_BIT_TRG_SEQ_MASK                  (~(0x7F<<0))

/* TRGCFG1: reg 0x33 RW */

#define AW86225_BIT_TRGCFG1_TRG1_POS_MASK         (~(1<<7))
#define AW86225_BIT_TRGCFG1_TRG1_POS_ENABLE       (1<<7)
#define AW86225_BIT_TRGCFG1_TRG1_POS_DISABLE      (0<<7)
#define AW86225_BIT_TRGCFG1_TRG1SEQ_P_MASK        (~(0x7F<<0))

/* TRGCFG2: reg 0x34 RW */

#define AW86225_BIT_TRGCFG2_TRG2_POS_MASK         (~(1<<7))
#define AW86225_BIT_TRGCFG2_TRG2_POS_ENABLE       (1<<7)
#define AW86225_BIT_TRGCFG2_TRG2_POS_DISABLE      (0<<7)
#define AW86225_BIT_TRGCFG2_TRG2SEQ_P_MASK        (~(0x7F<<0))

/* TRGCFG3: reg 0x35 RW */

#define AW86225_BIT_TRGCFG3_TRG3_POS_MASK         (~(1<<7))
#define AW86225_BIT_TRGCFG3_TRG3_POS_ENABLE       (1<<7)
#define AW86225_BIT_TRGCFG3_TRG3_POS_DISABLE      (0<<7)
#define AW86225_BIT_TRGCFG3_TRG3SEQ_P_MASK        (~(0x7F<<0))

/* TRGCFG4: reg 0x36 RW */

#define AW86225_BIT_TRGCFG4_TRG1_NEG_MASK         (~(1<<7))
#define AW86225_BIT_TRGCFG4_TRG1_NEG_ENABLE       (1<<7)
#define AW86225_BIT_TRGCFG4_TRG1_NEG_DISABLE      (0<<7)
#define AW86225_BIT_TRGCFG4_TRG1SEQ_N_MASK        (~(0x7F<<0))

/* TRGCFG5: reg 0x37 RW */

#define AW86225_BIT_TRGCFG5_TRG2_NEG_MASK         (~(1<<7))
#define AW86225_BIT_TRGCFG5_TRG2_NEG_ENABLE       (1<<7)
#define AW86225_BIT_TRGCFG5_TRG2_NEG_DISABLE      (0<<7)
#define AW86225_BIT_TRGCFG5_TRG2SEQ_N_MASK        (~(0x7F<<0))

/* TRGCFG6: reg 0x38 RW */

#define AW86225_BIT_TRGCFG6_TRG3_NEG_MASK         (~(1<<7))
#define AW86225_BIT_TRGCFG6_TRG3_NEG_ENABLE       (1<<7)
#define AW86225_BIT_TRGCFG6_TRG3_NEG_DISABLE      (0<<7)
#define AW86225_BIT_TRGCFG6_TRG3SEQ_N_MASK        (~(0x7F<<0))

/* TRGCFG7: reg 0x39 RW */

#define AW86225_BIT_TRGCFG7_TRG1_POR_LEV_BRK_MASK (~(7<<5))
#define AW86225_BIT_TRGCFG7_TRG2_POR_LEV_BRK_MASK (~(7<<1))
#define AW86225_BIT_TRGCFG7_TRG1_POLAR_MASK       (~(1<<7))
#define AW86225_BIT_TRGCFG7_TRG1_POLAR_NEG        (1<<7)
#define AW86225_BIT_TRGCFG7_TRG1_POLAR_POS        (0<<7)
#define AW86225_BIT_TRGCFG7_TRG1_MODE_MASK        (~(1<<6))
#define AW86225_BIT_TRGCFG7_TRG1_MODE_LEVEL       (1<<6)
#define AW86225_BIT_TRGCFG7_TRG1_MODE_EDGE        (0<<6)
#define AW86225_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK    (~(1<<5))
#define AW86225_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE  (1<<5)
#define AW86225_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE (0<<5)
#define AW86225_BIT_TRGCFG7_TRG2_POLAR_MASK       (~(1<<3))
#define AW86225_BIT_TRGCFG7_TRG2_POLAR_NEG        (1<<3)
#define AW86225_BIT_TRGCFG7_TRG2_POLAR_POS        (0<<3)
#define AW86225_BIT_TRGCFG7_TRG2_MODE_MASK        (~(1<<2))
#define AW86225_BIT_TRGCFG7_TRG2_MODE_LEVEL       (1<<2)
#define AW86225_BIT_TRGCFG7_TRG2_MODE_EDGE        (0<<2)
#define AW86225_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK    (~(1<<1))
#define AW86225_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE  (1<<1)
#define AW86225_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE (0<<1)

/* TRGCFG8: reg 0x3A RW */
#define AW86225_BIT_TRGCFG8_TRG3_POR_LEV_BRK_MASK (~(7<<5))
#define AW86225_BIT_TRGCFG8_TRG3_POLAR_MASK       (~(1<<7))
#define AW86225_BIT_TRGCFG8_TRG3_POLAR_NEG        (1<<7)
#define AW86225_BIT_TRGCFG8_TRG3_POLAR_POS        (0<<7)
#define AW86225_BIT_TRGCFG8_TRG3_MODE_MASK        (~(1<<6))
#define AW86225_BIT_TRGCFG8_TRG3_MODE_LEVEL       (1<<6)
#define AW86225_BIT_TRGCFG8_TRG3_MODE_EDGE        (0<<6)
#define AW86225_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK    (~(1<<5))
#define AW86225_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE  (1<<5)
#define AW86225_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE (0<<5)
#define AW86225_BIT_TRGCFG8_TRG_TRIG1_MODE_MASK   (~(3<<3))
#define AW86225_BIT_TRGCFG8_PWM_LRA               (0<<3)
#define AW86225_BIT_TRGCFG8_PWM_ERA               (1<<3)
#define AW86225_BIT_TRGCFG8_TRIG1                 (2<<3)
#define AW86225_BIT_TRGCFG8_DISABLE               (3<<3)
#define AW86225_BIT_TRGCFG8_TRG1_STOP_MASK        (~(1<<2))
#define AW86225_BIT_TRGCFG8_TRG1_STOP             (1<<2)
#define AW86225_BIT_TRGCFG8_TRG2_STOP_MASK        (~(1<<1))
#define AW86225_BIT_TRGCFG8_TRG2_STOP             (1<<1)
#define AW86225_BIT_TRGCFG8_TRG3_STOP_MASK        (~(1<<0))
#define AW86225_BIT_TRGCFG8_TRG3_STOP             (1<<0)

/* GLBCFG4: reg 0x3E RW */

#define AW86225_BIT_GLBCFG4_GO_PRIO_MASK          (~(3<<6))
#define AW86225_BIT_GLBCFG4_TRG3_PRIO_MASK        (~(3<<4))
#define AW86225_BIT_GLBCFG4_TRG2_PRIO_MASK        (~(3<<2))
#define AW86225_BIT_GLBCFG4_TRG1_PRIO_MASK        (~(3<<0))

/* GLBRD5: reg 0x3F R0 */

#define AW86225_BIT_GLBRD5_STATE_MASK             (~(15<<0))
#define AW86225_BIT_GLBRD5_STATE_STANDBY          (0<<0)
#define AW86225_BIT_GLBRD5_STATE_WAKEUP           (1<<0)
#define AW86225_BIT_GLBRD5_STATE_STARTUP          (2<<0)
#define AW86225_BIT_GLBRD5_STATE_WAIT             (3<<0)
#define AW86225_BIT_GLBRD5_STATE_CONT_GO          (6<<0)
#define AW86225_BIT_GLBRD5_STATE_RAM_GO           (7<<0)
#define AW86225_BIT_GLBRD5_STATE_RTP_GO           (8<<0)
#define AW86225_BIT_GLBRD5_STATE_TRIG_GO          (9<<0)
#define AW86225_BIT_GLBRD5_STATE_I2S_GO           (10<<0)
#define AW86225_BIT_GLBRD5_STATE_BRAKE            (11<<0)
#define AW86225_BIT_GLBRD5_STATE_END              (12<<0)

/* RAMADDRH: reg 0x40 RWS */

#define AW86225_BIT_RAMADDRH_MASK                 (~(63<<0))

/* SYSCTRL1: reg 0x43 RW */

#define AW86225_BIT_SYSCTRL1_VBAT_MODE_MASK       (~(1<<7))
#define AW86225_BIT_SYSCTRL1_VBAT_MODE_HW         (1<<7)
#define AW86225_BIT_SYSCTRL1_VBAT_MODE_SW         (0<<7)
#define AW86225_BIT_SYSCTRL1_PERP_MASK            (~(1<<6))
#define AW86225_BIT_SYSCTRL1_PERP_ON              (1<<6)
#define AW86225_BIT_SYSCTRL1_PERP_OFF             (0<<6)
#define AW86225_BIT_SYSCTRL1_CLK_SEL_MASK         (~(3<<4))
#define AW86225_BIT_SYSCTRL1_CLK_SEL_OSC          (1<<4)
#define AW86225_BIT_SYSCTRL1_CLK_SEL_AUTO         (0<<4)
#define AW86225_BIT_SYSCTRL1_RAMINIT_MASK         (~(1<<3))
#define AW86225_BIT_SYSCTRL1_RAMINIT_ON           (1<<3)
#define AW86225_BIT_SYSCTRL1_RAMINIT_OFF          (0<<3)
#define AW86225_BIT_SYSCTRL1_EN_FIR_MASK          (~(1<<2))
#define AW86225_BIT_SYSCTRL1_FIR_ENABLE           (0<<2)
#define AW86225_BIT_SYSCTRL1_WAKE_MODE_MASK       (~(1<<1))
#define AW86225_BIT_SYSCTRL1_WAKE_MODE_WAKEUP     (1<<1)
#define AW86225_BIT_SYSCTRL1_WAKE_MODE_BST        (0<<1)
#define AW86225_BIT_SYSCTRL1_RTP_CLK_MASK         (~(1<<0))
#define AW86225_BIT_SYSCTRL1_RTP_PLL              (1<<0)
#define AW86225_BIT_SYSCTRL1_RTP_OSC              (0<<0)

/* SYSCTRL2: reg 0x44 RW */

#define AW86225_BIT_SYSCTRL2_WAKE_MASK            (~(1<<7))
#define AW86225_BIT_SYSCTRL2_WAKE_ON              (1<<7)
#define AW86225_BIT_SYSCTRL2_WAKE_OFF             (0<<7)
#define AW86225_BIT_SYSCTRL2_STANDBY_MASK         (~(1<<6))
#define AW86225_BIT_SYSCTRL2_STANDBY_ON           (1<<6)
#define AW86225_BIT_SYSCTRL2_STANDBY_OFF          (0<<6)
#define AW86225_BIT_SYSCTRL2_RTP_DLY_MASK         (~(3<<4))
#define AW86225_BIT_SYSCTRL2_INTN_PIN_MASK        (~(1<<3))
#define AW86225_BIT_SYSCTRL2_INTN                 (1<<3)
#define AW86225_BIT_SYSCTRL2_TRIG1                (0<<3)
#define AW86225_BIT_SYSCTRL2_WCK_PIN_MASK         (~(1<<2))
#define AW86225_BIT_SYSCTRL2_ENABLE_TRIG2         (1<<2)
#define AW86225_BIT_SYSCTRL2_DISENABLE_TRIG2      (0<<2)
#define AW86225_BIT_SYSCTRL2_WAVDAT_MODE_MASK     (~(3<<0))
#define AW86225_BIT_SYSCTRL2_RATE_12K             (2<<0)
#define AW86225_BIT_SYSCTRL2_RATE_24K             (0<<0)
#define AW86225_BIT_SYSCTRL2_RATE_48K             (1<<0)

/* SYSCTRL7: reg 0x49 RW */

#define AW86225_BIT_SYSCTRL7_GAIN_BYPASS_MASK     (~(1<<6))
#define AW86225_BIT_SYSCTRL7_GAIN_CHANGEABLE      (1<<6)
#define AW86225_BIT_SYSCTRL7_GAIN_FIXED           (0<<6)

#define AW86225_BIT_SYSCTRL7_INT_EDGE_MODE_MASK   (~(1<<5))
#define AW86225_BIT_SYSCTRL7_INT_EDGE_MODE_POS    (0<<5)
#define AW86225_BIT_SYSCTRL7_INT_EDGE_MODE_BOTH   (1<<5)
#define AW86225_BIT_SYSCTRL7_INT_MODE_MASK        (~(1<<4))
#define AW86225_BIT_SYSCTRL7_INT_MODE_EDGE        (1<<4)
#define AW86225_BIT_SYSCTRL7_INT_MODE_LEVEL       (0<<4)

#define AW86225_BIT_SYSCTRL7_INTP_MASK            (~(1<<3))
#define AW86225_BIT_SYSCTRL7_INTP_HIGH            (1<<3)
#define AW86225_BIT_SYSCTRL7_INTP_LOW             (0<<3)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_MASK        (~(7<<0))
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_1           (0<<0)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_2           (1<<0)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_4           (2<<0)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_5           (3<<0)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_8           (4<<0)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_10          (5<<0)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_20          (6<<0)
#define AW86225_BIT_SYSCTRL7_D2S_GAIN_40          (7<<0)

/* I2SCFG1: reg 0x4A RW */

#define AW86225_BIT_I2SCFG1_I2SMD_MASK            (~(3<<6))
#define AW86225_BIT_I2SCFG1_I2SFS_MASK            (~(3<<4))
#define AW86225_BIT_I2SCFG1_I2SFS_16BIT           (0<<4)
#define AW86225_BIT_I2SCFG1_I2SFS_20BIT           (1<<4)
#define AW86225_BIT_I2SCFG1_I2SFS_24BIT           (2<<4)
#define AW86225_BIT_I2SCFG1_I2SFS_32BIT           (3<<4)
#define AW86225_BIT_I2SCFG1_I2SBCK_MASK           (~(3<<2))
#define AW86225_BIT_I2SCFG1_I2SBCK_32FS           (0<<2)
#define AW86225_BIT_I2SCFG1_I2SBCK_48FS           (1<<2)
#define AW86225_BIT_I2SCFG1_I2SBCK_64FS           (2<<2)
#define AW86225_BIT_I2SCFG1_RX_THRS_MASK          (~(3<<0))

/* I2SCFG2: reg 0x4B RW */

#define AW86225_BIT_I2SCFG2_WSINV_MASK            (~(1<<4))
#define AW86225_BIT_I2SCFG2_WSINV_SWITCH          (1<<4)
#define AW86225_BIT_I2SCFG2_WSINV_NO_SWITCH       (0<<4)
#define AW86225_BIT_I2SCFG2_BCKINV_MASK           (~(1<<3))
#define AW86225_BIT_I2SCFG2_BCKINV_INVERT         (1<<3)
#define AW86225_BIT_I2SCFG2_BCKINV_NOTINVT        (0<<3)
#define AW86225_BIT_I2SCFG2_CHSEL_MASK            (~(1<<2))
#define AW86225_BIT_I2SCFG2_CHSEL_LEFT            (1<<2)
#define AW86225_BIT_I2SCFG2_CHSEL_RIGHT           (0<<2)
#define AW86225_BIT_I2SCFG2_I2S_INT_MASK          (~(1<<1))
#define AW86225_BIT_I2SCFG2_I2S_INT_ON            (1<<1)
#define AW86225_BIT_I2SCFG2_I2S_INT_OFF           (0<<1)
#define AW86225_BIT_I2SCFG2_I2S_EN_MASK           (~(1<<0))
#define AW86225_BIT_I2SCFG2_I2S_ENABLE            (1<<0)
#define AW86225_BIT_I2SCFG2_I2S_DISABLE           (0<<0)

/* PWMCFG1: reg 0x4C RW */

#define AW86225_BIT_PWMCFG1_PRC_EN_MASK           (~(1<<7))
#define AW86225_BIT_PWMCFG1_PRC_ENABLE            (1<<7)
#define AW86225_BIT_PWMCFG1_PRC_DISABLE           (0<<7)
#define AW86225_BIT_PWMCFG1_PRCTIME_MASK          (~(0x7F<<0))

/* PWMCFG2: reg 0x4D RW */

#define AW86225_BIT_PWMCFG2_REF_SEL_MASK          (~(1<<5))
#define AW86225_BIT_PWMCFG2_REF_SEL_TRIANGLE      (1<<5)
#define AW86225_BIT_PWMCFG2_REF_SEL_SAWTOOTH      (0<<5)
#define AW86225_BIT_PWMCFG2_PD_HWM_MASK           (~(1<<4))
#define AW86225_BIT_PWMCFG2_PD_HWM_ON             (1<<4)
#define AW86225_BIT_PWMCFG2_PWMOE_MASK            (~(1<<3))
#define AW86225_BIT_PWMCFG2_PWMOE_ON              (1<<3)
#define AW86225_BIT_PWMCFG2_PWMFRC_MASK           (~(7<<0))

/* PWMCFG3: reg 0x4E RW */

#define AW86225_BIT_PWMCFG3_PR_EN_MASK            (~(1<<7))
#define AW86225_BIT_PWMCFG3_PR_ENABLE             (1<<7)
#define AW86225_BIT_PWMCFG3_PR_DISABLE            (0<<7)
#define AW86225_BIT_PWMCFG3_PRLVL_MASK            (~(0x7F<<0))

/* DETCFG1: reg 0x51 RW */

#define AW86225_BIT_DETCFG1_FTS_GO_MASK           (~(1<<7))
#define AW86225_BIT_DETCFG1_FTS_GO_ENABLE         (1<<7)
#define AW86225_BIT_DETCFG1_TEST_GO_MASK          (~(1<<6))
#define AW86225_BIT_DETCFG1_TEST_GO_ENABLE        (1<<6)
#define AW86225_BIT_DETCFG1_ADO_SLOT_MODE_MASK    (~(1<<5))
#define AW86225_BIT_DETCFG1_ADO_SLOT_ADC_32       (1<<5)
#define AW86225_BIT_DETCFG1_ADO_SLOT_ADC_256      (0<<5)
#define AW86225_BIT_DETCFG1_RL_OS_MASK            (~(1<<4))
#define AW86225_BIT_DETCFG1_RL                    (1<<4)
#define AW86225_BIT_DETCFG1_OS                    (0<<4)
#define AW86225_BIT_DETCFG1_PRCT_MODE_MASK        (~(1<<3))
#define AW86225_BIT_DETCFG1_PRCT_MODE_INVALID     (1<<3)
#define AW86225_BIT_DETCFG1_PRCT_MODE_VALID       (0<<3)
#define AW86225_BIT_DETCFG1_CLK_ADC_MASK          (~(7<<0))
#define AW86225_BIT_DETCFG1_CLK_ADC_12M           (0<<0)
#define AW86225_BIT_DETCFG1_CLK_ADC_6M            (1<<0)
#define AW86225_BIT_DETCFG1_CLK_ADC_3M            (2<<0)
#define AW86225_BIT_DETCFG1_CLK_ADC_1M5           (3<<0)
#define AW86225_BIT_DETCFG1_CLK_ADC_M75           (4<<0)
#define AW86225_BIT_DETCFG1_CLK_ADC_M37           (5<<0)
#define AW86225_BIT_DETCFG1_CLK_ADC_M18           (6<<0)
#define AW86225_BIT_DETCFG1_CLK_ADC_M09           (7<<0)

/* DETCFG2: reg 0x52 RW */

#define AW86225_BIT_DETCFG2_VBAT_GO_MASK          (~(1<<1))
#define AW86225_BIT_DETCFG2_VABT_GO_ON            (1<<1)
#define AW86225_BIT_DETCFG2_DIAG_GO_MASK          (~(1<<0))
#define AW86225_BIT_DETCFG2_DIAG_GO_ON            (1<<0)

/* DET_LO: reg 0x57 RW */

#define AW86225_BIT_DET_LO_TEST_MASK              (~(3<<6))
#define AW86225_BIT_DET_LO_VBAT_MASK              (~(3<<4))
#define AW86225_BIT_DET_LO_OS_MASK                (~(3<<2))
#define AW86225_BIT_DET_LO_RL_MASK                (~(3<<0))

/* TRIMCFG1: reg:0x58 RW */

#define AW86225_BIT_TRIMCFG1_RL_TRIM_SRC_MASK     (~(1<<6))
#define AW86225_BIT_TRIMCFG1_RL_TRIM_SRC_REG      (1<<6)
#define AW86225_BIT_TRIMCFG1_RL_TRIM_SRC_EFUSE    (0<<6)
#define AW86225_BIT_TRIMCFG1_TRIM_RL_MASK         (~(63<<0))

/* TRIMCFG3: reg:0x5A RW */
#define AW86225_BIT_TRIMCFG3_OSC_TRIM_SRC_MASK    (~(1<<7))
#define AW86225_BIT_TRIMCFG3_OSC_TRIM_SRC_REG     (1<<7)
#define AW86225_BIT_TRIMCFG3_OSC_TRIM_SRC_EFUSE   (0<<7)
#define AW86225_BIT_TRIMCFG3_TRIM_LRA_MASK        (~(63<<0))

/* PLLCFG1: reg:0x68 RW */

#define AW86225_BIT_PLLCFG1_PLL_TEST_EN_MASK      (~(1<<6))
#define AW86225_BIT_PLLCFG1_PLL_TEST_ENABLE       (1<<6)
#define AW86225_BIT_PLLCFG1_PLL_TEST_DIV_MASK     (~(3<<4))
#define AW86225_BIT_PLLCFG1_PLL_TEST_DIV_1        (0<<4)
#define AW86225_BIT_PLLCFG1_PLL_TEST_DIV_2        (1<<4)
#define AW86225_BIT_PLLCFG1_PLL_TEST_DIV_4        (2<<4)
#define AW86225_BIT_PLLCFG1_PLL_TEST_DIV_8        (3<<4)
#define AW86225_BIT_PLLCFG1_PLL_BIAS_CP1_IEN_MASK (~(1<<3))
#define AW86225_BIT_PLLCFG1_PLL_BIAS_CP1_IENABLE  (1<<3)
#define AW86225_BIT_PLLCFG1_PLL_VTI_CP1_IEN_MASK  (~(1<<2))
#define AW86225_BIT_PLLCFG1_PLL_VTI_CP1_IENABLE   (1<<2)
#define AW86225_BIT_PLLCFG1_PLL_DELAY_SEL_MASK    (~(1<<1))
#define AW86225_BIT_PLLCFG1_PLL_R1_SEL_MASK       (~(1<<0))

/* PLLCFG2: reg:0x69 RW */

#define AW86225_BIT_PLLCFG2_PLL_CP1_SEL_MASK      (~(0x0F<<4))
#define AW86225_BIT_PLLCFG2_PLL_CP1_40UA          (4<<4)
#define AW86225_BIT_PLLCFG2_PLL_CP1_50UA          (5<<4)
#define AW86225_BIT_PLLCFG2_PLL_CP1_80UA          (8<<4)
#define AW86225_BIT_PLLCFG2_PLL_CP1_100UA         (10<<4)
#define AW86225_BIT_PLLCFG2_PLL_CP2_SEL_MASK      (~(0x0F<<0))
#define AW86225_BIT_PLLCFG2_PLL_CP2_40NA          (1<<0)
#define AW86225_BIT_PLLCFG2_PLL_CP2_600NA         (8<<0)
#define AW86225_BIT_PLLCFG2_PLL_CP2_800NA         (10<<0)
#define AW86225_BIT_PLLCFG2_PLL_CP2_1200NA        (12<<0)

/* HDRVCFG1: reg:0x6A RW */

#define AW86225_BIT_HDRVCFG1_EN_HD_LOW_MASK       (~(1<<7))
#define AW86225_BIT_HDRVCFG1_EN_HD_HZ             (0<<7)
#define AW86225_BIT_HDRVCFG1_EN_HD_PD             (1<<7)

/* IOCFG1: reg:0x6B RW */

#define AW86225_BIT_IOCFG1_HSEN_MASK              (~(1<<6))
#define AW86225_BIT_IOCFG1_HS_ENABLE              (1<<6)
#define AW86225_BIT_IOCFG1_HS_DISABLE             (0<<6)
#define AW86225_BIT_IOCFG1_IO_FAST_MASK           (~(3<<4))
#define AW86225_BIT_IOCFG1_ALL_IO_FAST_ENABLE     (3<<4)
#define AW86225_BIT_IOCFG1_IIS_IO_FAST_ENABLE     (2<<4)
#define AW86225_BIT_IOCFG1_IIC_IO_FAST_ENABLE     (1<<4)
#define AW86225_BIT_IOCFG1_IO_FAST_DISABLE        (0<<4)

/* OCCFG1: reg:0x74 RW */

#define AW86225_BIT_OCCFG1_HS_IOC_MASK            (~(3<<6))
#define AW86225_BIT_OCCFG1_HS_IOC_3A15            (0<<6)
#define AW86225_BIT_OCCFG1_HS_IOC_3A65            (1<<6)
#define AW86225_BIT_OCCFG1_HS_IOC_4A15            (2<<6)
#define AW86225_BIT_OCCFG1_HS_IOC_4A65            (3<<6)
#define AW86225_BIT_OCCFG1_LS_IOC_MASK            (~(3<<4))
#define AW86225_BIT_OCCFG1_LS_IOC_3A15            (0<<4)
#define AW86225_BIT_OCCFG1_LS_IOC_3A65            (1<<4)
#define AW86225_BIT_OCCFG1_LS_IOC_4A15            (2<<4)
#define AW86225_BIT_OCCFG1_LS_IOC_4A65            (3<<4)
#define AW86225_BIT_OCCFG1_OCDT_MASK              (~(3<<2))

#define AW86225_BIT_OCCLK_MODE_MASK               (~(3<<0))

/* ADCCFG1: reg:0x75 RW */

#define AW86225_BIT_ADCCFG1_EN_OS_DET_MASK        (~(1<<7))
#define AW86225_BIT_ADCCFG1_EN_RL_DET_MASK        (~(1<<6))
#define AW86225_BIT_ADCCFG1_D2S_SEL_HDRV_MASK     (~(1<<5))
#define AW86225_BIT_ADCCFG1_AD_SEL_HDP_MASK       (~(1<<4))
#define AW86225_BIT_ADCCFG1_AD_SEL_HDN_MASK       (~(1<<3))
#define AW86225_BIT_ADCCFG1_AD_SEL_VBAT_MASK      (~(1<<2))
#define AW86225_BIT_ADCCFG1_AD_SEL_TEST_MASK      (~(1<<1))
#define AW86225_BIT_ADCCFG1_AD_SEL_D2S_MASK       (~(1<<0))

/* D2SCFG1: reg:0x76 RW */

#define AW86225_BIT_D2SCFG1_CLK_TRIM_MODE_MASK    (~(7<<0))
#define AW86225_BIT_D2SCFG1_CLK_TRIM_MODE_48K     (0<<0)
#define AW86225_BIT_D2SCFG1_CLK_TRIM_MODE_24K     (1<<0)
#define AW86225_BIT_D2SCFG1_CLK_TRIM_MODE_12K     (2<<0)
#define AW86225_BIT_D2SCFG1_CLK_TRIM_MODE_6K      (3<<0)
#define AW86225_BIT_D2SCFG1_CLK_TRIM_MODE_12M     (4<<0)

/* ANACFG8: reg:0x77 RW */

#define AW86225_BIT_ANACFG8_TRTF_CTRL_HDRV_MASK   (~(1<<6))
#define AW86225_BIT_ANACFG8_TRTF_CTRL_HDRV        (3<<6)

#endif /* __DRIVERS_INPUT_AW86225_REG_H */
