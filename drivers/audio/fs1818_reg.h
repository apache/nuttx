/****************************************************************************
 * drivers/audio/fs1818_reg.h
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

#ifndef __DRIVERS_AUDIO_FS1818_REG_H__
#define __DRIVERS_AUDIO_FS1818_REG_H__

#define FS1818_OTP_ACC_KEY           (0xCA00)
#define FS1818_OTP_ACC_KEY2          (0xCA91)
#define FS1818_OTP_BST_CFG           (0xC5FC)

#define FS1818_STATUS_OK_MASK        (FS1818_STATUS_REG_CLKS|     \
                                      FS1818_STATUS_REG_UVDS      \
                                      | FS1818_STATUS_REG_OVDS |  \
                                      FS1818_STATUS_REG_OTDS      \
                                      | FS1818_STATUS_REG_PLLS |  \
                                      FS1818_STATUS_REG_BOVDS)

#define FS1818_DSP_ENABLE_ALL_MASK   (FS1818_DSPCTRL_REG_DSPEN |  \
                                      FS1818_DSPCTRL_REG_POSTEQEN \
                                      | FS1818_DSPCTRL_REG_NOFILTEN | \
                                      FS1601S_DSP_DCCOEF_DEFAULT)

#define FS1818_DSP_BYPASS_ALL_MASK   0x04

#define FS1818_ACSCTRL_REG_DEFAULT   0x9880

#define FS1818_ADCCTRL_REG_DEFAULT   0x1300
#define FS1818_ADCCTRL_REG_DISABLE   0x0300

#define FS1818_PLLCTRL_EN_ALL        (FS1818_PLLCTRL4_REG_PLLEN |     \
                                      FS1818_PLLCTRL4_REG_OSCEN       \
                                      | FS1818_PLLCTRL4_REG_ZMEN |    \
                                      FS1818_PLLCTRL4_REG_VBGEN)
#define FS1818_ANACTRL_BYPASS_OV_PLL (FS1818_ANACTRL_REG_BPOV_MSK |   \
                                      FS1818_ANACTRL_REG_BPCLKCK_MSK)
#define FS1818_ANACTRL_DEFAULT       (FS1818_ANACTRL_REG_BPOV_MSK)

#define FS1818_CHIPOTPLOAD_OK        (FS1818_CHIPINI_REG_INIOK |      \
                                      FS1818_CHIPINI_REG_INIFINISH)

/* STATUS (0x00) */

#define FS1818_STATUS_REG                    0x00

#define FS1818_STATUS_REG_BOVDS              (0x01<<0)
#define FS1818_STATUS_REG_BOVDS_POS          0
#define FS1818_STATUS_REG_BOVDS_LEN          1
#define FS1818_STATUS_REG_BOVDS_MSK          0x1
#define FS1818_STATUS_REG_PLLS               (0x01<<1)
#define FS1818_STATUS_REG_PLLS_POS           1
#define FS1818_STATUS_REG_PLLS_LEN           1
#define FS1818_STATUS_REG_PLLS_MSK           0x2
#define FS1818_STATUS_REG_OTDS               (0x01<<2)
#define FS1818_STATUS_REG_OTDS_POS           2
#define FS1818_STATUS_REG_OTDS_LEN           1
#define FS1818_STATUS_REG_OTDS_MSK           0x4
#define FS1818_STATUS_REG_OVDS               (0x01<<3)
#define FS1818_STATUS_REG_OVDS_POS           3
#define FS1818_STATUS_REG_OVDS_LEN           1
#define FS1818_STATUS_REG_OVDS_MSK           0x8
#define FS1818_STATUS_REG_UVDS               (0x01<<4)
#define FS1818_STATUS_REG_UVDS_POS           4
#define FS1818_STATUS_REG_UVDS_LEN           1
#define FS1818_STATUS_REG_UVDS_MSK           0x10
#define FS1818_STATUS_REG_OCDS               (0x01<<5)
#define FS1818_STATUS_REG_OCDS_POS           5
#define FS1818_STATUS_REG_OCDS_LEN           1
#define FS1818_STATUS_REG_OCDS_MSK           0x20
#define FS1818_STATUS_REG_CLKS               (0x01<<6)
#define FS1818_STATUS_REG_CLKS_POS           6
#define FS1818_STATUS_REG_CLKS_LEN           1
#define FS1818_STATUS_REG_CLKS_MSK           0x40
#define FS1818_STATUS_REG_SPKS               (0x01<<10)
#define FS1818_STATUS_REG_SPKS_POS           10
#define FS1818_STATUS_REG_SPKS_LEN           1
#define FS1818_STATUS_REG_SPKS_MSK           0x400
#define FS1818_STATUS_REG_SPKT               (0x01<<11)
#define FS1818_STATUS_REG_SPKT_POS           11
#define FS1818_STATUS_REG_SPKT_LEN           1
#define FS1818_STATUS_REG_SPKT_MSK           0x800

/* BATS (0x01) */

#define FS1818_BATS_REG                      0x01

#define FS1818_BATS_REG_BATV                 (0x01<<0)
#define FS1818_BATS_REG_BATV_POS             0
#define FS1818_BATS_REG_BATV_LEN             10
#define FS1818_BATS_REG_BATV_MSK             0x3ff

/* TEMPS (0x02) */

#define FS1818_TEMPS_REG                     0x02

#define FS1818_TEMPS_REG_TEMPV               (0x01<<0)
#define FS1818_TEMPS_REG_TEMPV_POS           0
#define FS1818_TEMPS_REG_TEMPV_LEN           9
#define FS1818_TEMPS_REG_TEMPV_MSK           0x1ff

/* ID (0x03) */

#define FS1818_ID_REG                         0x03

#define FS1818_ID_REG_REV                    (0x01<<0)
#define FS1818_ID_REG_REV_POS                0
#define FS1818_ID_REG_REV_LEN                8
#define FS1818_ID_REG_REV_MSK                0xff
#define FS1818_ID_REG_DEVICE_ID              (0x01<<8)
#define FS1818_ID_REG_DEVICE_ID_POS          8
#define FS1818_ID_REG_DEVICE_ID_LEN          8
#define FS1818_ID_REG_DEVICE_ID_MSK          0xff00

/* I2SCTRL (0x04) */

#define FS1818_I2SCTRL_REG                   0x04

#define FS1818_I2SCTRL_MONO_AEC              0x381B//0x881B->0x381B(16k)->0x781B(44.1k)
#define FS1818_I2SCTRL_REG_I2SF              (0x01<<0)
#define FS1818_I2SCTRL_REG_I2SF_POS          0
#define FS1818_I2SCTRL_REG_I2SF_LEN          3
#define FS1818_I2SCTRL_REG_I2SF_MSK          0x7
#define FS1818_I2SCTRL_REG_CHS12             (0x01<<3)
#define FS1818_I2SCTRL_REG_CHS12_POS         3
#define FS1818_I2SCTRL_REG_CHS12_LEN         2
#define FS1818_I2SCTRL_REG_CHS12_MSK         0x18
#define FS1818_I2SCTRL_REG_DISP              (0x01<<10)
#define FS1818_I2SCTRL_REG_DISP_POS          10
#define FS1818_I2SCTRL_REG_DISP_LEN          1
#define FS1818_I2SCTRL_REG_DISP_MSK          0x400
#define FS1818_I2SCTRL_REG_I2SDOE            (0x01<<11)
#define FS1818_I2SCTRL_REG_I2SDOE_POS        11
#define FS1818_I2SCTRL_REG_I2SDOE_LEN        1
#define FS1818_I2SCTRL_REG_I2SDOE_MSK        0x800
#define FS1818_I2SCTRL_REG_I2SSR             (0x01<<12)
#define FS1818_I2SCTRL_REG_I2SSR_POS         12
#define FS1818_I2SCTRL_REG_I2SSR_LEN         4
#define FS1818_I2SCTRL_REG_I2SSR_MSK         0xf000
#define FS1818_I2SCTRL_CHS12                 0x1304
#define FS1818_I2SCTRL_I2SF                  0x3004
#define FS1818_I2SCTRL_I2SSR                 0x3C04
#define FS1818_I2SCTRL_I2SDOE                0x0B04

/* ANASTAT (0x05) */

#define FS1818_ANASTAT_REG                   0x05

#define FS1818_ANASTAT_REG_VBGS              (0x01<<0)
#define FS1818_ANASTAT_REG_VBGS_POS          0
#define FS1818_ANASTAT_REG_VBGS_LEN          1
#define FS1818_ANASTAT_REG_VBGS_MSK          0x1
#define FS1818_ANASTAT_REG_PLLS              (0x01<<1)
#define FS1818_ANASTAT_REG_PLLS_POS          1
#define FS1818_ANASTAT_REG_PLLS_LEN          1
#define FS1818_ANASTAT_REG_PLLS_MSK          0x2
#define FS1818_ANASTAT_REG_BSTS              (0x01<<2)
#define FS1818_ANASTAT_REG_BSTS_POS          2
#define FS1818_ANASTAT_REG_BSTS_LEN          1
#define FS1818_ANASTAT_REG_BSTS_MSK          0x4
#define FS1818_ANASTAT_REG_AMPS              (0x01<<3)
#define FS1818_ANASTAT_REG_AMPS_POS          3
#define FS1818_ANASTAT_REG_AMPS_LEN          1
#define FS1818_ANASTAT_REG_AMPS_MSK          0x8
#define FS1818_ANASTAT_REG_OTPRDE            (0x01<<8)
#define FS1818_ANASTAT_REG_OTPRDE_POS        8
#define FS1818_ANASTAT_REG_OTPRDE_LEN        1
#define FS1818_ANASTAT_REG_OTPRDE_MSK        0x100

/* AUDIOCTRL (0x06) */

#define FS1818_AUDIOCTRL_REG                 0x06

#define FS1818_AUDIOCTRL_DEFAULT             0xFF00
#define FS1818_AUDIOCTRL_REG_VOL             (0x01<<8)
#define FS1818_AUDIOCTRL_REG_VOL_POS         8
#define FS1818_AUDIOCTRL_REG_VOL_LEN         8
#define FS1818_AUDIOCTRL_REG_VOL_MSK         0xff00

/* TEMPSEL (0x08) */

#define FS1818_TEMPSEL_REG                   0x08

#define FS1818_TEMPSEL_REG_EXTTS             (0x01<<1)
#define FS1818_TEMPSEL_REG_EXTTS_POS         1
#define FS1818_TEMPSEL_REG_EXTTS_LEN         9
#define FS1818_TEMPSEL_REG_EXTTS_MSK         0x3fe

/* SYSCTRL (0x09) */

#define FS1818_SYSCTRL_REG                   0x09

#define FS1818_SYSCTRL_REG_PWON              0x0000
#define FS1818_SYSCTRL_REG_PWDN              (0x01<<0)
#define FS1818_SYSCTRL_REG_PWDN_POS          0
#define FS1818_SYSCTRL_REG_PWDN_LEN          1
#define FS1818_SYSCTRL_REG_PWDN_MSK          0x1
#define FS1818_SYSCTRL_REG_I2CR              (0x01<<1)
#define FS1818_SYSCTRL_REG_I2CR_POS          1
#define FS1818_SYSCTRL_REG_I2CR_LEN          1
#define FS1818_SYSCTRL_REG_I2CR_MSK          0x2
#define FS1818_SYSCTRL_REG_AMPE              (0x01<<3)
#define FS1818_SYSCTRL_REG_AMPE_POS          3
#define FS1818_SYSCTRL_REG_AMPE_LEN          1
#define FS1818_SYSCTRL_REG_AMPE_MSK          0x8

/* SPKSET (0x0A) */

#define FS1818_SPKSET_REG                    0x0A

#define FS1818_SPKSET_REG_SPKR               (0x01<<9)
#define FS1818_SPKSET_REG_SPKR_POS           9
#define FS1818_SPKSET_REG_SPKR_LEN           2
#define FS1818_SPKSET_REG_SPKR_MSK           0x600

/* OTPACC (0x0B) */

#define FS1818_OTPACC_REG                    0x0B

#define FS1818_OTPACC_ENABLE                 0xCA91
#define FS1818_OTPACC_DISABLE              0x0000
#define FS1818_OTPACC_REG_TRIMKEY            (0x01<<0)
#define FS1818_OTPACC_REG_TRIMKEY_POS        0
#define FS1818_OTPACC_REG_TRIMKEY_LEN        8
#define FS1818_OTPACC_REG_TRIMKEY_MSK        0xff
#define FS1818_OTPACC_REG_REKEY              (0x01<<8)
#define FS1818_OTPACC_REG_REKEY_POS          8
#define FS1818_OTPACC_REG_REKEY_LEN          8
#define FS1818_OTPACC_REG_REKEY_MSK          0xff00

/* SPKSTAUTS (0x80) */

#define FS1818_SPKSTAUTS_REG                 0x80

#define FS1818_SPKSTAUTS_REG_STATUS          (0x01<<1)
#define FS1818_SPKSTAUTS_REG_STATUS_POS      1
#define FS1818_SPKSTAUTS_REG_STATUS_LEN      1
#define FS1818_SPKSTAUTS_REG_STATUS_MSK      0x2

/* ACSCTRL (0x89) */

#define FS1818_ACSCTRL_REG                   0x89

#define FS1818_ACSCTRL_DEFAULT               0x9000
#define FS1818_ACSCTRL_MUSIC                 0x9080
#define FS1818_ACSCTRL_VOICE                 0x9090
#define FS1818_ACSCTRL_REG_ACS_COE_SEL       (0x01<<4)
#define FS1818_ACSCTRL_REG_ACS_COE_SEL_POS   4
#define FS1818_ACSCTRL_REG_ACS_COE_SEL_LEN   1
#define FS1818_ACSCTRL_REG_ACS_COE_SEL_MSK   0x10
#define FS1818_ACSCTRL_REG_MBDRC_EN          (0x01<<7)
#define FS1818_ACSCTRL_REG_MBDRC_EN_POS      7
#define FS1818_ACSCTRL_REG_MBDRC_EN_LEN      1
#define FS1818_ACSCTRL_REG_MBDRC_EN_MSK      0x80
#define FS1818_ACSCTRL_REG_EQEN              (0x01<<12)
#define FS1818_ACSCTRL_REG_EQEN_POS          12
#define FS1818_ACSCTRL_REG_EQEN_LEN          1
#define FS1818_ACSCTRL_REG_EQEN_MSK          0x1000
#define FS1818_ACSCTRL_REG_ACS_EN            (0x01<<15)
#define FS1818_ACSCTRL_REG_ACS_EN_POS        15
#define FS1818_ACSCTRL_REG_ACS_EN_LEN        1
#define FS1818_ACSCTRL_REG_ACS_EN_MSK        0x8000

/* ACSDRC (0x8A) */

#define FS1818_ACSDRC_REG                    0x8A

#define FS1818_ACSDRC_REG_DRCS1_EN           (0x01<<0)
#define FS1818_ACSDRC_REG_DRCS1_EN_POS       0
#define FS1818_ACSDRC_REG_DRCS1_EN_LEN       1
#define FS1818_ACSDRC_REG_DRCS1_EN_MSK       0x1
#define FS1818_ACSDRC_REG_DRCS2_EN           (0x01<<1)
#define FS1818_ACSDRC_REG_DRCS2_EN_POS       1
#define FS1818_ACSDRC_REG_DRCS2_EN_LEN       1
#define FS1818_ACSDRC_REG_DRCS2_EN_MSK       0x2
#define FS1818_ACSDRC_REG_DRCS3_EN           (0x01<<2)
#define FS1818_ACSDRC_REG_DRCS3_EN_POS       2
#define FS1818_ACSDRC_REG_DRCS3_EN_LEN       1
#define FS1818_ACSDRC_REG_DRCS3_EN_MSK       0x4
#define FS1818_ACSDRC_REG_DRC_ENV_W          (0x01<<8)
#define FS1818_ACSDRC_REG_DRC_ENV_W_POS      8
#define FS1818_ACSDRC_REG_DRC_ENV_W_LEN      2
#define FS1818_ACSDRC_REG_DRC_ENV_W_MSK      0x300

/* ACSDRCS (0x8B) */

#define FS1818_ACSDRCS_REG                   0x8B

#define FS1818_ACSDRCS_REG_DRCS11_EN         (0x01<<2)
#define FS1818_ACSDRCS_REG_DRCS11_EN_POS     2
#define FS1818_ACSDRCS_REG_DRCS11_EN_LEN     1
#define FS1818_ACSDRCS_REG_DRCS11_EN_MSK     0x4
#define FS1818_ACSDRCS_REG_DRCS12_EN         (0x01<<4)
#define FS1818_ACSDRCS_REG_DRCS12_EN_POS     4
#define FS1818_ACSDRCS_REG_DRCS12_EN_LEN     1
#define FS1818_ACSDRCS_REG_DRCS12_EN_MSK     0x10
#define FS1818_ACSDRCS_REG_DRCS21_EN         (0x01<<7)
#define FS1818_ACSDRCS_REG_DRCS21_EN_POS     7
#define FS1818_ACSDRCS_REG_DRCS21_EN_LEN     1
#define FS1818_ACSDRCS_REG_DRCS21_EN_MSK     0x80
#define FS1818_ACSDRCS_REG_DRCS22_EN         (0x01<<9)
#define FS1818_ACSDRCS_REG_DRCS22_EN_POS     9
#define FS1818_ACSDRCS_REG_DRCS22_EN_LEN     1
#define FS1818_ACSDRCS_REG_DRCS22_EN_MSK     0x200
#define FS1818_ACSDRCS_REG_DRCS31_EN         (0x01<<12)
#define FS1818_ACSDRCS_REG_DRCS31_EN_POS     12
#define FS1818_ACSDRCS_REG_DRCS31_EN_LEN     1
#define FS1818_ACSDRCS_REG_DRCS31_EN_MSK     0x1000
#define FS1818_ACSDRCS_REG_DRCS32_EN         (0x01<<14)
#define FS1818_ACSDRCS_REG_DRCS32_EN_POS     14
#define FS1818_ACSDRCS_REG_DRCS32_EN_LEN     1
#define FS1818_ACSDRCS_REG_DRCS32_EN_MSK     0x4000

/* CHIPINI (0x90) */

#define FS1818_CHIPINI_REG                   0x90

#define FS1818_CHIPINI_REG_INIFINISH         (0x01<<0)
#define FS1818_CHIPINI_REG_INIFINISH_POS     0
#define FS1818_CHIPINI_REG_INIFINISH_LEN     1
#define FS1818_CHIPINI_REG_INIFINISH_MSK     0x1
#define FS1818_CHIPINI_REG_INIOK             (0x01<<1)
#define FS1818_CHIPINI_REG_INIOK_POS         1
#define FS1818_CHIPINI_REG_INIOK_LEN         1
#define FS1818_CHIPINI_REG_INIOK_MSK         0x2

/* I2CADD (0x91) */

#define FS1818_I2CADD_REG                    0x91

#define FS1818_I2CADD_REG_ADR                (0x01<<0)
#define FS1818_I2CADD_REG_ADR_POS            0
#define FS1818_I2CADD_REG_ADR_LEN            7
#define FS1818_I2CADD_REG_ADR_MSK            0x7f

/* BISTCTL1 (0x9C) */

#define FS1818_BISTCTL1_REG                  0x9C

#define FS1818_BISTCTL1_REG_GO               (0x01<<0)
#define FS1818_BISTCTL1_REG_GO_POS           0
#define FS1818_BISTCTL1_REG_GO_LEN           1
#define FS1818_BISTCTL1_REG_GO_MSK           0x1
#define FS1818_BISTCTL1_REG_MODE             (0x01<<1)
#define FS1818_BISTCTL1_REG_MODE_POS         1
#define FS1818_BISTCTL1_REG_MODE_LEN         1
#define FS1818_BISTCTL1_REG_MODE_MSK         0x2
#define FS1818_BISTCTL1_REG_TRAMSEL          (0x01<<2)
#define FS1818_BISTCTL1_REG_TRAMSEL_POS      2
#define FS1818_BISTCTL1_REG_TRAMSEL_LEN      1
#define FS1818_BISTCTL1_REG_TRAMSEL_MSK      0x4
#define FS1818_BISTCTL1_REG_HCLKEN           (0x01<<3)
#define FS1818_BISTCTL1_REG_HCLKEN_POS       3
#define FS1818_BISTCTL1_REG_HCLKEN_LEN       1
#define FS1818_BISTCTL1_REG_HCLKEN_MSK       0x8
#define FS1818_BISTCTL1_REG_RAM1_SKIP        (0x01<<4)
#define FS1818_BISTCTL1_REG_RAM1_SKIP_POS    4
#define FS1818_BISTCTL1_REG_RAM1_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM1_SKIP_MSK    0x10
#define FS1818_BISTCTL1_REG_RAM2_SKIP        (0x01<<5)
#define FS1818_BISTCTL1_REG_RAM2_SKIP_POS    5
#define FS1818_BISTCTL1_REG_RAM2_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM2_SKIP_MSK    0x20
#define FS1818_BISTCTL1_REG_RAM3_SKIP        (0x01<<6)
#define FS1818_BISTCTL1_REG_RAM3_SKIP_POS    6
#define FS1818_BISTCTL1_REG_RAM3_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM3_SKIP_MSK    0x40
#define FS1818_BISTCTL1_REG_RAM4_SKIP        (0x01<<7)
#define FS1818_BISTCTL1_REG_RAM4_SKIP_POS    7
#define FS1818_BISTCTL1_REG_RAM4_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM4_SKIP_MSK    0x80
#define FS1818_BISTCTL1_REG_RAM5_SKIP        (0x01<<8)
#define FS1818_BISTCTL1_REG_RAM5_SKIP_POS    8
#define FS1818_BISTCTL1_REG_RAM5_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM5_SKIP_MSK    0x100
#define FS1818_BISTCTL1_REG_RAM6_SKIP        (0x01<<9)
#define FS1818_BISTCTL1_REG_RAM6_SKIP_POS    9
#define FS1818_BISTCTL1_REG_RAM6_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM6_SKIP_MSK    0x200
#define FS1818_BISTCTL1_REG_RAM7_SKIP        (0x01<<10)
#define FS1818_BISTCTL1_REG_RAM7_SKIP_POS    10
#define FS1818_BISTCTL1_REG_RAM7_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM7_SKIP_MSK    0x400
#define FS1818_BISTCTL1_REG_RAM8_SKIP        (0x01<<11)
#define FS1818_BISTCTL1_REG_RAM8_SKIP_POS    11
#define FS1818_BISTCTL1_REG_RAM8_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM8_SKIP_MSK    0x800
#define FS1818_BISTCTL1_REG_RAM9_SKIP        (0x01<<12)
#define FS1818_BISTCTL1_REG_RAM9_SKIP_POS    12
#define FS1818_BISTCTL1_REG_RAM9_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAM9_SKIP_MSK    0x1000
#define FS1818_BISTCTL1_REG_RAMA_SKIP        (0x01<<13)
#define FS1818_BISTCTL1_REG_RAMA_SKIP_POS    13
#define FS1818_BISTCTL1_REG_RAMA_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAMA_SKIP_MSK    0x2000
#define FS1818_BISTCTL1_REG_RAMB_SKIP        (0x01<<14)
#define FS1818_BISTCTL1_REG_RAMB_SKIP_POS    14
#define FS1818_BISTCTL1_REG_RAMB_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAMB_SKIP_MSK    0x4000
#define FS1818_BISTCTL1_REG_RAMC_SKIP        (0x01<<15)
#define FS1818_BISTCTL1_REG_RAMC_SKIP_POS    15
#define FS1818_BISTCTL1_REG_RAMC_SKIP_LEN    1
#define FS1818_BISTCTL1_REG_RAMC_SKIP_MSK    0x8000

/* BISTSTAT1 (0x9D) */

#define FS1818_BISTSTAT1_REG                 0x9D

#define FS1818_BISTSTAT1_REG_RAM             (0x01<<0)
#define FS1818_BISTSTAT1_REG_RAM_POS         0
#define FS1818_BISTSTAT1_REG_RAM_LEN         4
#define FS1818_BISTSTAT1_REG_RAM_MSK         0xf
#define FS1818_BISTSTAT1_REG_FAIL            (0x01<<4)
#define FS1818_BISTSTAT1_REG_FAIL_POS        4
#define FS1818_BISTSTAT1_REG_FAIL_LEN        1
#define FS1818_BISTSTAT1_REG_FAIL_MSK        0x10
#define FS1818_BISTSTAT1_REG_ACTIVE          (0x01<<6)
#define FS1818_BISTSTAT1_REG_ACTIVE_POS      6
#define FS1818_BISTSTAT1_REG_ACTIVE_LEN      1
#define FS1818_BISTSTAT1_REG_ACTIVE_MSK      0x40
#define FS1818_BISTSTAT1_REG_DONE            (0x01<<7)
#define FS1818_BISTSTAT1_REG_DONE_POS        7
#define FS1818_BISTSTAT1_REG_DONE_LEN        1
#define FS1818_BISTSTAT1_REG_DONE_MSK        0x80
#define FS1818_BISTSTAT1_REG_ADDR            (0x01<<8)
#define FS1818_BISTSTAT1_REG_ADDR_POS        8
#define FS1818_BISTSTAT1_REG_ADDR_LEN        8
#define FS1818_BISTSTAT1_REG_ADDR_MSK        0xff00

/* BISTSTAT2 (0x9E) */

#define FS1818_BISTSTAT2_REG                 0x9E

#define FS1818_BISTSTAT2_REG_BITS            (0x01<<0)
#define FS1818_BISTSTAT2_REG_BITS_POS        0
#define FS1818_BISTSTAT2_REG_BITS_LEN        16
#define FS1818_BISTSTAT2_REG_BITS_MSK        0xffff

/* BISTSTAT3 (0x9F) */

#define FS1818_BISTSTAT3_REG                 0x9F

#define FS1818_BISTSTAT3_REG_BITS            (0x01<<0)
#define FS1818_BISTSTAT3_REG_BITS_POS        0
#define FS1818_BISTSTAT3_REG_BITS_LEN        8
#define FS1818_BISTSTAT3_REG_BITS_MSK        0xff
#define FS1818_BISTSTAT3_REG_ADDRH           (0x01<<8)
#define FS1818_BISTSTAT3_REG_ADDRH_POS       8
#define FS1818_BISTSTAT3_REG_ADDRH_LEN       2
#define FS1818_BISTSTAT3_REG_ADDRH_MSK       0x300

/* I2SSET (0xA0) */

#define FS1818_I2SSET_REG                    0xA0

#define FS1818_I2SSET_REG_AECRS              (0x01<<0)
#define FS1818_I2SSET_REG_AECRS_POS          0
#define FS1818_I2SSET_REG_AECRS_LEN          2
#define FS1818_I2SSET_REG_AECRS_MSK          0x3
#define FS1818_I2SSET_REG_BCMP               (0x01<<3)
#define FS1818_I2SSET_REG_BCMP_POS           3
#define FS1818_I2SSET_REG_BCMP_LEN           2
#define FS1818_I2SSET_REG_BCMP_MSK           0x18
#define FS1818_I2SSET_REG_LRCLKP             (0x01<<5)
#define FS1818_I2SSET_REG_LRCLKP_POS         5
#define FS1818_I2SSET_REG_LRCLKP_LEN         1
#define FS1818_I2SSET_REG_LRCLKP_MSK         0x20
#define FS1818_I2SSET_REG_BCLKP              (0x01<<6)
#define FS1818_I2SSET_REG_BCLKP_POS          6
#define FS1818_I2SSET_REG_BCLKP_LEN          1
#define FS1818_I2SSET_REG_BCLKP_MSK          0x40
#define FS1818_I2SSET_REG_I2SOSWAP           (0x01<<7)
#define FS1818_I2SSET_REG_I2SOSWAP_POS       7
#define FS1818_I2SSET_REG_I2SOSWAP_LEN       1
#define FS1818_I2SSET_REG_I2SOSWAP_MSK       0x80
#define FS1818_I2SSET_REG_AECSELL            (0x01<<8)
#define FS1818_I2SSET_REG_AECSELL_POS        8
#define FS1818_I2SSET_REG_AECSELL_LEN        3
#define FS1818_I2SSET_REG_AECSELL_MSK        0x700
#define FS1818_I2SSET_REG_AECSELR            (0x01<<12)
#define FS1818_I2SSET_REG_AECSELR_POS        12
#define FS1818_I2SSET_REG_AECSELR_LEN        3
#define FS1818_I2SSET_REG_AECSELR_MSK        0x7000
#define FS1818_I2SSET_REG_BCLKSTA            (0x01<<15)
#define FS1818_I2SSET_REG_BCLKSTA_POS        15
#define FS1818_I2SSET_REG_BCLKSTA_LEN        1
#define FS1818_I2SSET_REG_BCLKSTA_MSK        0x8000

/* DSPCTRL (0xA1) */

#define FS1818_DSPCTRL_REG                   0xA1

#define FS1818_DSPCTRL_DEFAULT               0x1812
#define FS1818_DSPCTRL_COEF_SELECT           0x1815
#define FS1818_DSPCTRL_REG_DCCOEF            (0x01<<0)
#define FS1818_DSPCTRL_REG_DCCOEF_POS        0
#define FS1818_DSPCTRL_REG_DCCOEF_LEN        3
#define FS1818_DSPCTRL_REG_DCCOEF_MSK        0x7
#define FS1818_DSPCTRL_REG_NOFILTEN          (0x01<<4)
#define FS1818_DSPCTRL_REG_NOFILTEN_POS      4
#define FS1818_DSPCTRL_REG_NOFILTEN_LEN      1
#define FS1818_DSPCTRL_REG_NOFILTEN_MSK      0x10
#define FS1818_DSPCTRL_REG_POSTEQEN          (0x01<<11)
#define FS1818_DSPCTRL_REG_POSTEQEN_POS      11
#define FS1818_DSPCTRL_REG_POSTEQEN_LEN      1
#define FS1818_DSPCTRL_REG_POSTEQEN_MSK      0x800
#define FS1818_DSPCTRL_REG_DSPEN             (0x01<<12)
#define FS1818_DSPCTRL_REG_DSPEN_POS         12
#define FS1818_DSPCTRL_REG_DSPEN_LEN         1
#define FS1818_DSPCTRL_REG_DSPEN_MSK         0x1000
#define FS1818_DSPCTRL_REG_EQCOEFSEL         (0x01<<13)
#define FS1818_DSPCTRL_REG_EQCOEFSEL_POS     13
#define FS1818_DSPCTRL_REG_EQCOEFSEL_LEN     1
#define FS1818_DSPCTRL_REG_EQCOEFSEL_MSK     0x2000

/* DACEQWL (0xA2) */

#define FS1818_DACEQWL_REG                   0xA2

#define FS1818_DACEQWL_REG_DL                (0x01<<0)
#define FS1818_DACEQWL_REG_DL_POS            0
#define FS1818_DACEQWL_REG_DL_LEN            8
#define FS1818_DACEQWL_REG_DL_MSK            0xff
#define FS1818_DACEQWL_REG_DM                (0x01<<8)
#define FS1818_DACEQWL_REG_DM_POS            8
#define FS1818_DACEQWL_REG_DM_LEN            8
#define FS1818_DACEQWL_REG_DM_MSK            0xff00

/* DACEQWH (0xA3) */

#define FS1818_DACEQWH_REG                   0xA3

#define FS1818_DACEQWH_REG_DH                (0x01<<0)
#define FS1818_DACEQWH_REG_DH_POS            0
#define FS1818_DACEQWH_REG_DH_LEN            8
#define FS1818_DACEQWH_REG_DH_MSK            0xff

/* DACEQRL (0xA4) */

#define FS1818_DACEQRL_REG                   0xA4

#define FS1818_DACEQRL_REG_DL                (0x01<<0)
#define FS1818_DACEQRL_REG_DL_POS            0
#define FS1818_DACEQRL_REG_DL_LEN            8
#define FS1818_DACEQRL_REG_DL_MSK            0xff
#define FS1818_DACEQRL_REG_DM                (0x01<<8)
#define FS1818_DACEQRL_REG_DM_POS            8
#define FS1818_DACEQRL_REG_DM_LEN            8
#define FS1818_DACEQRL_REG_DM_MSK            0xff00

/* DACEQRH (0xA5) */

#define FS1818_DACEQRH_REG                   0xA5

#define FS1818_DACEQRH_REG_DH                (0x01<<0)
#define FS1818_DACEQRH_REG_DH_POS            0
#define FS1818_DACEQRH_REG_DH_LEN            8
#define FS1818_DACEQRH_REG_DH_MSK            0xff

/* DACEQA (0xA6) */

#define FS1818_DACEQA_REG                    0xA6

#define FS1818_DACEQA_REG_ACS0               0x0000
#define FS1818_DACEQA_REG_ACS1               0x0069
#define FS1818_DACEQA_REG_SPKMDL             0x00D2
#define FS1818_DACEQA_REG_ADDR               (0x01<<0)
#define FS1818_DACEQA_REG_ADDR_POS           0
#define FS1818_DACEQA_REG_ADDR_LEN           9
#define FS1818_DACEQA_REG_ADDR_MSK           0x1ff

/* BFLCTRL (0xA7) */

#define FS1818_BFLCTRL_REG                   0xA7

#define FS1818_BFLCTRL_REG_EN                (0x01<<2)
#define FS1818_BFLCTRL_REG_EN_POS            2
#define FS1818_BFLCTRL_REG_EN_LEN            1
#define FS1818_BFLCTRL_REG_EN_MSK            0x4

/* BFLSET (0xA8) */

#define FS1818_BFLSET_REG                    0xA8

#define FS1818_BFLSET_REG_BFL_SET            (0x01<<0)
#define FS1818_BFLSET_REG_BFL_SET_POS        0
#define FS1818_BFLSET_REG_BFL_SET_LEN        8
#define FS1818_BFLSET_REG_BFL_SET_MSK        0xff

/* SQC (0xA9) */

#define FS1818_SQC_REG                       0xA9

#define FS1818_SQC_REG_SQC                   (0x01<<3)
#define FS1818_SQC_REG_SQC_POS               3
#define FS1818_SQC_REG_SQC_LEN               13
#define FS1818_SQC_REG_SQC_MSK               0xfff8

/* AGC (0xAA) */

#define FS1818_AGC_REG                       0xAA

#define FS1818_AGC_REG_GAIN                  (0x01<<0)
#define FS1818_AGC_REG_GAIN_POS              0
#define FS1818_AGC_REG_GAIN_LEN              8
#define FS1818_AGC_REG_GAIN_MSK              0xff

/* DACPEAKRD (0xAB) */

#define FS1818_DACPEAKRD_REG                 0xAB

#define FS1818_DACPEAKRD_REG_SEL             (0x01<<0)
#define FS1818_DACPEAKRD_REG_SEL_POS         0
#define FS1818_DACPEAKRD_REG_SEL_LEN         5
#define FS1818_DACPEAKRD_REG_SEL_MSK         0x1f

/* PDCCTRL (0xAC) */

#define FS1818_PDCCTRL_REG                   0xAC

#define FS1818_PDCCTRL_REG_RMEN              (0x01<<8)
#define FS1818_PDCCTRL_REG_RMEN_POS          8
#define FS1818_PDCCTRL_REG_RMEN_LEN          1
#define FS1818_PDCCTRL_REG_RMEN_MSK          0x100
#define FS1818_PDCCTRL_REG_TIME              (0x01<<9)
#define FS1818_PDCCTRL_REG_TIME_POS          9
#define FS1818_PDCCTRL_REG_TIME_LEN          3
#define FS1818_PDCCTRL_REG_TIME_MSK          0xe00
#define FS1818_PDCCTRL_REG_THD               (0x01<<12)
#define FS1818_PDCCTRL_REG_THD_POS           12
#define FS1818_PDCCTRL_REG_THD_LEN           3
#define FS1818_PDCCTRL_REG_THD_MSK           0x7000

/* DRPARA (0xAD) */

#define FS1818_DRPARA_REG                    0xAD

#define FS1818_DRPARA_REG_DRC                (0x01<<0)
#define FS1818_DRPARA_REG_DRC_POS            0
#define FS1818_DRPARA_REG_DRC_LEN            13
#define FS1818_DRPARA_REG_DRC_MSK            0x1fff

/* DACCTRL (0xAE) */

#define FS1818_DACCTRL_REG                   0xAE

#define FS1818_DACCTRL_UNMUTE                0x0210
#define FS1818_DACCTRL_REG_SDMSTLBYP         (0x01<<4)
#define FS1818_DACCTRL_REG_SDMSTLBYP_POS     4
#define FS1818_DACCTRL_REG_SDMSTLBYP_LEN     1
#define FS1818_DACCTRL_REG_SDMSTLBYP_MSK     0x10
#define FS1818_DACCTRL_REG_MUTE              (0x01<<8)
#define FS1818_DACCTRL_REG_MUTE_POS          8
#define FS1818_DACCTRL_REG_MUTE_LEN          1
#define FS1818_DACCTRL_REG_MUTE_MSK          0x100
#define FS1818_DACCTRL_REG_FADE              (0x01<<9)
#define FS1818_DACCTRL_REG_FADE_POS          9
#define FS1818_DACCTRL_REG_FADE_LEN          1
#define FS1818_DACCTRL_REG_FADE_MSK          0x200

/* TSCTRL (0xAF) */

#define FS1818_TSCTRL_REG                    0xAF

#define FS1818_TSCTRL_TS_NOFF                0x162B
#define FS1818_TSCTRL_TSAUTO_OFF             0x362B
#define FS1818_TSCTRL_REG_GAIN               (0x01<<0)
#define FS1818_TSCTRL_REG_GAIN_POS           0
#define FS1818_TSCTRL_REG_GAIN_LEN           3
#define FS1818_TSCTRL_REG_GAIN_MSK           0x7
#define FS1818_TSCTRL_REG_EN                 (0x01<<3)
#define FS1818_TSCTRL_REG_EN_POS             3
#define FS1818_TSCTRL_REG_EN_LEN             1
#define FS1818_TSCTRL_REG_EN_MSK             0x8
#define FS1818_TSCTRL_REG_OFF_THD            (0x01<<4)
#define FS1818_TSCTRL_REG_OFF_THD_POS        4
#define FS1818_TSCTRL_REG_OFF_THD_LEN        3
#define FS1818_TSCTRL_REG_OFF_THD_MSK        0x70
#define FS1818_TSCTRL_REG_OFF_DELAY          (0x01<<8)
#define FS1818_TSCTRL_REG_OFF_DELAY_POS      8
#define FS1818_TSCTRL_REG_OFF_DELAY_LEN      3
#define FS1818_TSCTRL_REG_OFF_DELAY_MSK      0x700
#define FS1818_TSCTRL_REG_OFF_ZEROCRS        (0x01<<12)
#define FS1818_TSCTRL_REG_OFF_ZEROCRS_POS    12
#define FS1818_TSCTRL_REG_OFF_ZEROCRS_LEN    1
#define FS1818_TSCTRL_REG_OFF_ZEROCRS_MSK    0x1000
#define FS1818_TSCTRL_REG_OFF_AUTOEN         (0x01<<13)
#define FS1818_TSCTRL_REG_OFF_AUTOEN_POS     13
#define FS1818_TSCTRL_REG_OFF_AUTOEN_LEN     1
#define FS1818_TSCTRL_REG_OFF_AUTOEN_MSK     0x2000
#define FS1818_TSCTRL_REG_OFFSTA             (0x01<<14)
#define FS1818_TSCTRL_REG_OFFSTA_POS         14
#define FS1818_TSCTRL_REG_OFFSTA_LEN         1
#define FS1818_TSCTRL_REG_OFFSTA_MSK         0x4000

/* MODCTRL (0xB0) */

#define FS1818_MODCTRL_REG                   0xB0

#define FS1818_MODCTRL_REG_G1_SEL            (0x01<<0)
#define FS1818_MODCTRL_REG_G1_SEL_POS        0
#define FS1818_MODCTRL_REG_G1_SEL_LEN        3
#define FS1818_MODCTRL_REG_G1_SEL_MSK        0x7
#define FS1818_MODCTRL_REG_G2_SEL            (0x01<<3)
#define FS1818_MODCTRL_REG_G2_SEL_POS        3
#define FS1818_MODCTRL_REG_G2_SEL_LEN        3
#define FS1818_MODCTRL_REG_G2_SEL_MSK        0x38
#define FS1818_MODCTRL_REG_DEMBYP            (0x01<<7)
#define FS1818_MODCTRL_REG_DEMBYP_POS        7
#define FS1818_MODCTRL_REG_DEMBYP_LEN        1
#define FS1818_MODCTRL_REG_DEMBYP_MSK        0x80
#define FS1818_MODCTRL_REG_DITHPOS           (0x01<<8)
#define FS1818_MODCTRL_REG_DITHPOS_POS       8
#define FS1818_MODCTRL_REG_DITHPOS_LEN       5
#define FS1818_MODCTRL_REG_DITHPOS_MSK       0x1f00
#define FS1818_MODCTRL_REG_DITHRANGE         (0x01<<13)
#define FS1818_MODCTRL_REG_DITHRANGE_POS     13
#define FS1818_MODCTRL_REG_DITHRANGE_LEN     1
#define FS1818_MODCTRL_REG_DITHRANGE_MSK     0x2000
#define FS1818_MODCTRL_REG_DITHCLR           (0x01<<14)
#define FS1818_MODCTRL_REG_DITHCLR_POS       14
#define FS1818_MODCTRL_REG_DITHCLR_LEN       1
#define FS1818_MODCTRL_REG_DITHCLR_MSK       0x4000
#define FS1818_MODCTRL_REG_DITHEN            (0x01<<15)
#define FS1818_MODCTRL_REG_DITHEN_POS        15
#define FS1818_MODCTRL_REG_DITHEN_LEN        1
#define FS1818_MODCTRL_REG_DITHEN_MSK        0x8000

/* DTINI (0xB1) */

#define FS1818_DTINI_REG                     0xB1

#define FS1818_DTINI_REG_INI_VAL             (0x01<<0)
#define FS1818_DTINI_REG_INI_VAL_POS         0
#define FS1818_DTINI_REG_INI_VAL_LEN         16
#define FS1818_DTINI_REG_INI_VAL_MSK         0xffff

/* DTFD (0xB2) */

#define FS1818_DTFD_REG                      0xB2

#define FS1818_DTFD_REG_COEFF                (0x01<<0)
#define FS1818_DTFD_REG_COEFF_POS            0
#define FS1818_DTFD_REG_COEFF_LEN            16
#define FS1818_DTFD_REG_COEFF_MSK            0xffff

/* ADCCTRL (0xB3) */

#define FS1818_ADCCTRL_REG                   0xB3

#define FS1818_ADCCTRL_REG_EQB1EN_R          (0x01<<8)
#define FS1818_ADCCTRL_REG_EQB1EN_R_POS      8
#define FS1818_ADCCTRL_REG_EQB1EN_R_LEN      1
#define FS1818_ADCCTRL_REG_EQB1EN_R_MSK      0x100
#define FS1818_ADCCTRL_REG_EQB0EN_R          (0x01<<9)
#define FS1818_ADCCTRL_REG_EQB0EN_R_POS      9
#define FS1818_ADCCTRL_REG_EQB0EN_R_LEN      1
#define FS1818_ADCCTRL_REG_EQB0EN_R_MSK      0x200
#define FS1818_ADCCTRL_REG_ADCRGAIN          (0x01<<10)
#define FS1818_ADCCTRL_REG_ADCRGAIN_POS      10
#define FS1818_ADCCTRL_REG_ADCRGAIN_LEN      2
#define FS1818_ADCCTRL_REG_ADCRGAIN_MSK      0xc00
#define FS1818_ADCCTRL_REG_ADCREN            (0x01<<12)
#define FS1818_ADCCTRL_REG_ADCREN_POS        12
#define FS1818_ADCCTRL_REG_ADCREN_LEN        1
#define FS1818_ADCCTRL_REG_ADCREN_MSK        0x1000
#define FS1818_ADCCTRL_REG_ADCRSEL           (0x01<<13)
#define FS1818_ADCCTRL_REG_ADCRSEL_POS       13
#define FS1818_ADCCTRL_REG_ADCRSEL_LEN       1
#define FS1818_ADCCTRL_REG_ADCRSEL_MSK       0x2000

/* ADCEQWL (0xB4) */

#define FS1818_ADCEQWL_REG                   0xB4

#define FS1818_ADCEQWL_REG_DL                (0x01<<0)
#define FS1818_ADCEQWL_REG_DL_POS            0
#define FS1818_ADCEQWL_REG_DL_LEN            8
#define FS1818_ADCEQWL_REG_DL_MSK            0xff
#define FS1818_ADCEQWL_REG_DM                (0x01<<8)
#define FS1818_ADCEQWL_REG_DM_POS            8
#define FS1818_ADCEQWL_REG_DM_LEN            8
#define FS1818_ADCEQWL_REG_DM_MSK            0xff00

/* ADCEQWH (0xB5) */

#define FS1818_ADCEQWH_REG                   0xB5

#define FS1818_ADCEQWH_REG_DH                (0x01<<0)
#define FS1818_ADCEQWH_REG_DH_POS            0
#define FS1818_ADCEQWH_REG_DH_LEN            8
#define FS1818_ADCEQWH_REG_DH_MSK            0xff

/* ADCEQRL (0xB6) */

#define FS1818_ADCEQRL_REG                   0xB6

#define FS1818_ADCEQRL_REG_DL                (0x01<<0)
#define FS1818_ADCEQRL_REG_DL_POS            0
#define FS1818_ADCEQRL_REG_DL_LEN            8
#define FS1818_ADCEQRL_REG_DL_MSK            0xff
#define FS1818_ADCEQRL_REG_DM                (0x01<<8)
#define FS1818_ADCEQRL_REG_DM_POS            8
#define FS1818_ADCEQRL_REG_DM_LEN            8
#define FS1818_ADCEQRL_REG_DM_MSK            0xff00

/* ADCEQRH (0xB7) */

#define FS1818_ADCEQRH_REG                   0xB7

#define FS1818_ADCEQRH_REG_DH                (0x01<<0)
#define FS1818_ADCEQRH_REG_DH_POS            0
#define FS1818_ADCEQRH_REG_DH_LEN            8
#define FS1818_ADCEQRH_REG_DH_MSK            0xff

/* ADCEQA (0xB8) */

#define FS1818_ADCEQA_REG                    0xB8

#define FS1818_ADCEQA_REG_ADDR               (0x01<<0)
#define FS1818_ADCEQA_REG_ADDR_POS           0
#define FS1818_ADCEQA_REG_ADDR_LEN           8
#define FS1818_ADCEQA_REG_ADDR_MSK           0xff

/* ADCENV (0xB9) */

#define FS1818_ADCENV_REG                    0xB9

#define FS1818_ADCENV_REG_AMP_DTEN_ALL       0xffff
#define FS1818_ADCENV_REG_AMP_DT_A           (0x01<<0)
#define FS1818_ADCENV_REG_AMP_DT_A_POS       0
#define FS1818_ADCENV_REG_AMP_DT_A_LEN       13
#define FS1818_ADCENV_REG_AMP_DT_A_MSK       0x1fff
#define FS1818_ADCENV_REG_AVG_NUM            (0x01<<13)
#define FS1818_ADCENV_REG_AVG_NUM_POS        13
#define FS1818_ADCENV_REG_AVG_NUM_LEN        2
#define FS1818_ADCENV_REG_AVG_NUM_MSK        0x6000
#define FS1818_ADCENV_REG_AMP_DT_EN          (0x01<<15)
#define FS1818_ADCENV_REG_AMP_DT_EN_POS      15
#define FS1818_ADCENV_REG_AMP_DT_EN_LEN      1
#define FS1818_ADCENV_REG_AMP_DT_EN_MSK      0x8000

/* ADCTIME (0xBA) */

#define FS1818_ADCTIME_REG                   0xBA

#define FS1818_ADCTIME_REG_VALUE             0x0029
#define FS1818_ADCTIME_REG_STABLE_TM         (0x01<<0)
#define FS1818_ADCTIME_REG_STABLE_TM_POS     0
#define FS1818_ADCTIME_REG_STABLE_TM_LEN     3
#define FS1818_ADCTIME_REG_STABLE_TM_MSK     0x7
#define FS1818_ADCTIME_REG_CHK_TM            (0x01<<3)
#define FS1818_ADCTIME_REG_CHK_TM_POS        3
#define FS1818_ADCTIME_REG_CHK_TM_LEN        3
#define FS1818_ADCTIME_REG_CHK_TM_MSK        0x38

/* ZMDATA (0xBB) */

#define FS1818_ZMDATA_REG                    0xBB

#define FS1818_ZMDATA_REG_XAVGM              (0x01<<0)
#define FS1818_ZMDATA_REG_XAVGM_POS          0
#define FS1818_ZMDATA_REG_XAVGM_LEN          8
#define FS1818_ZMDATA_REG_XAVGM_MSK          0xff
#define FS1818_ZMDATA_REG_XAVGH              (0x01<<8)
#define FS1818_ZMDATA_REG_XAVGH_POS          8
#define FS1818_ZMDATA_REG_XAVGH_LEN          8
#define FS1818_ZMDATA_REG_XAVGH_MSK          0xff00

/* DACENV (0xBC) */

#define FS1818_DACENV_REG                    0xBC

#define FS1818_DACENV_REG_MBYTE              (0x01<<0)
#define FS1818_DACENV_REG_MBYTE_POS          0
#define FS1818_DACENV_REG_MBYTE_LEN          8
#define FS1818_DACENV_REG_MBYTE_MSK          0xff
#define FS1818_DACENV_REG_HBYTE              (0x01<<8)
#define FS1818_DACENV_REG_HBYTE_POS          8
#define FS1818_DACENV_REG_HBYTE_LEN          8
#define FS1818_DACENV_REG_HBYTE_MSK          0xff00

/* DIGSTAT (0xBD) */

#define FS1818_DIGSTAT_REG                   0xBD

#define FS1818_DIGSTAT_REG_ADCRUN            (0x01<<0)
#define FS1818_DIGSTAT_REG_ADCRUN_POS        0
#define FS1818_DIGSTAT_REG_ADCRUN_LEN        1
#define FS1818_DIGSTAT_REG_ADCRUN_MSK        0x1
#define FS1818_DIGSTAT_REG_DACRUN            (0x01<<1)
#define FS1818_DIGSTAT_REG_DACRUN_POS        1
#define FS1818_DIGSTAT_REG_DACRUN_LEN        1
#define FS1818_DIGSTAT_REG_DACRUN_MSK        0x2
#define FS1818_DIGSTAT_REG_DSPFLAG           (0x01<<3)
#define FS1818_DIGSTAT_REG_DSPFLAG_POS       3
#define FS1818_DIGSTAT_REG_DSPFLAG_LEN       1
#define FS1818_DIGSTAT_REG_DSPFLAG_MSK       0x8
#define FS1818_DIGSTAT_REG_SPKM24            (0x01<<4)
#define FS1818_DIGSTAT_REG_SPKM24_POS        4
#define FS1818_DIGSTAT_REG_SPKM24_LEN        1
#define FS1818_DIGSTAT_REG_SPKM24_MSK        0x10
#define FS1818_DIGSTAT_REG_SPKM6             (0x01<<5)
#define FS1818_DIGSTAT_REG_SPKM6_POS         5
#define FS1818_DIGSTAT_REG_SPKM6_LEN         1
#define FS1818_DIGSTAT_REG_SPKM6_MSK         0x20
#define FS1818_DIGSTAT_REG_SPKRE             (0x01<<6)
#define FS1818_DIGSTAT_REG_SPKRE_POS         6
#define FS1818_DIGSTAT_REG_SPKRE_LEN         1
#define FS1818_DIGSTAT_REG_SPKRE_MSK         0x40
#define FS1818_DIGSTAT_REG_SPKFSM            (0x01<<12)
#define FS1818_DIGSTAT_REG_SPKFSM_POS        12
#define FS1818_DIGSTAT_REG_SPKFSM_LEN        4
#define FS1818_DIGSTAT_REG_SPKFSM_MSK        0xf000

/* I2SPINC (0xBE) */

#define FS1818_I2SPINC_REG                   0xBE

#define FS1818_I2SPINC_REG_BCPDD             (0x01<<0)
#define FS1818_I2SPINC_REG_BCPDD_POS         0
#define FS1818_I2SPINC_REG_BCPDD_LEN         1
#define FS1818_I2SPINC_REG_BCPDD_MSK         0x1
#define FS1818_I2SPINC_REG_LRPDD             (0x01<<1)
#define FS1818_I2SPINC_REG_LRPDD_POS         1
#define FS1818_I2SPINC_REG_LRPDD_LEN         1
#define FS1818_I2SPINC_REG_LRPDD_MSK         0x2
#define FS1818_I2SPINC_REG_SDOPDD            (0x01<<2)
#define FS1818_I2SPINC_REG_SDOPDD_POS        2
#define FS1818_I2SPINC_REG_SDOPDD_LEN        1
#define FS1818_I2SPINC_REG_SDOPDD_MSK        0x4
#define FS1818_I2SPINC_REG_SDIPDD            (0x01<<3)
#define FS1818_I2SPINC_REG_SDIPDD_POS        3
#define FS1818_I2SPINC_REG_SDIPDD_LEN        1
#define FS1818_I2SPINC_REG_SDIPDD_MSK        0x8

/* BSTCTRL (0xC0) */

#define FS1818_BSTCTRL_REG                   0xC0

#define FS1818_BSTCTRL_BST_DISABLE_ALL       0x0000
#define FS1818_BSTCTRL_BST_BIT_EN            0x0008
#define FS1818_BSTCTRL_DEFAULT               0x15EE
#define FS1818_BSTCTRL_BST_DISABLE           0x3DF0
#define FS1818_BSTCTRL_BST_ENABLE            0x95DE
#define FS1818_BSTCTRL_REG_DISCHARGE         (0x01<<0)
#define FS1818_BSTCTRL_REG_DISCHARGE_POS     0
#define FS1818_BSTCTRL_REG_DISCHARGE_LEN     1
#define FS1818_BSTCTRL_REG_DISCHARGE_MSK     0x1
#define FS1818_BSTCTRL_REG_DAC_GAIN          (0x01<<1)
#define FS1818_BSTCTRL_REG_DAC_GAIN_POS      1
#define FS1818_BSTCTRL_REG_DAC_GAIN_LEN      2
#define FS1818_BSTCTRL_REG_DAC_GAIN_MSK      0x6
#define FS1818_BSTCTRL_REG_BSTEN             (0x01<<3)
#define FS1818_BSTCTRL_REG_BSTEN_POS         3
#define FS1818_BSTCTRL_REG_BSTEN_LEN         1
#define FS1818_BSTCTRL_REG_BSTEN_MSK         0x8
#define FS1818_BSTCTRL_REG_MODE_CTRL         (0x01<<4)
#define FS1818_BSTCTRL_REG_MODE_CTRL_POS     4
#define FS1818_BSTCTRL_REG_MODE_CTRL_LEN     2
#define FS1818_BSTCTRL_REG_MODE_CTRL_MSK     0x30
#define FS1818_BSTCTRL_REG_ILIM_SEL          (0x01<<6)
#define FS1818_BSTCTRL_REG_ILIM_SEL_POS      6
#define FS1818_BSTCTRL_REG_ILIM_SEL_LEN      4
#define FS1818_BSTCTRL_REG_ILIM_SEL_MSK      0x3c0
#define FS1818_BSTCTRL_REG_VOUT_SEL          (0x01<<10)
#define FS1818_BSTCTRL_REG_VOUT_SEL_POS      10
#define FS1818_BSTCTRL_REG_VOUT_SEL_LEN      4
#define FS1818_BSTCTRL_REG_VOUT_SEL_MSK      0x3c00
#define FS1818_BSTCTRL_REG_SSEND             (0x01<<15)
#define FS1818_BSTCTRL_REG_SSEND_POS         15
#define FS1818_BSTCTRL_REG_SSEND_LEN         1
#define FS1818_BSTCTRL_REG_SSEND_MSK         0x8000

/* PLLCTRL1 (0xC1) */

#define FS1818_PLLCTRL1_REG                  0xC1

#define FS1818_PLLCTRL1_INIT                 0x01A0 // 0x01A0(44.1khz)->0x01A0(16khz)
#define FS1818_PLLCTRL1_REG_POSTSEL          (0x01<<2)
#define FS1818_PLLCTRL1_REG_POSTSEL_POS      2
#define FS1818_PLLCTRL1_REG_POSTSEL_LEN      2
#define FS1818_PLLCTRL1_REG_POSTSEL_MSK      0xc
#define FS1818_PLLCTRL1_REG_FINSEL           (0x01<<4)
#define FS1818_PLLCTRL1_REG_FINSEL_POS       4
#define FS1818_PLLCTRL1_REG_FINSEL_LEN       2
#define FS1818_PLLCTRL1_REG_FINSEL_MSK       0x30
#define FS1818_PLLCTRL1_REG_BWSEL            (0x01<<6)
#define FS1818_PLLCTRL1_REG_BWSEL_POS        6
#define FS1818_PLLCTRL1_REG_BWSEL_LEN        2
#define FS1818_PLLCTRL1_REG_BWSEL_MSK        0xc0
#define FS1818_PLLCTRL1_REG_ICPSEL           (0x01<<8)
#define FS1818_PLLCTRL1_REG_ICPSEL_POS       8
#define FS1818_PLLCTRL1_REG_ICPSEL_LEN       2
#define FS1818_PLLCTRL1_REG_ICPSEL_MSK       0x300

/* PLLCTRL2 (0xC2) */

#define FS1818_PLLCTRL2_REG                  0xC2

#define FS1818_PLLCTRL2_INIT                 0x0100 // 0x0100(44.1khz)->0x0180(16khz)
#define FS1818_PLLCTRL2_REG_FBDIV            (0x01<<0)
#define FS1818_PLLCTRL2_REG_FBDIV_POS        0
#define FS1818_PLLCTRL2_REG_FBDIV_LEN        15
#define FS1818_PLLCTRL2_REG_FBDIV_MSK        0x7fff

/* PLLCTRL3 (0xC3) */

#define FS1818_PLLCTRL3_REG                  0xC3

#define FS1818_PLLCTRL3_INIT                 0x0004 // 0x0004(44.1khz)->0x0002(16khz)
#define FS1818_PLLCTRL3_REG_FINDIV           (0x01<<0)
#define FS1818_PLLCTRL3_REG_FINDIV_POS       0
#define FS1818_PLLCTRL3_REG_FINDIV_LEN       9
#define FS1818_PLLCTRL3_REG_FINDIV_MSK       0x1ff

/* PLLCTRL4 (0xC4) */

#define FS1818_PLLCTRL4_REG                  0xC4

#define FS1818_PLLCTRL4_ENABLE               0x000F
#define FS1818_PLLCTRL4_DISABLE_OSC          0x000A
#define FS1818_PLLCTRL4_DISABLE_ZMADC        0x000B
#define FS1818_PLLCTRL4_DISABLE_PLL          0x000E
#define FS1818_PLLCTRL4_DISABLE              0x0000
#define FS1818_PLLCTRL4_REG_PLLEN            (0x01<<0)
#define FS1818_PLLCTRL4_REG_PLLEN_POS        0
#define FS1818_PLLCTRL4_REG_PLLEN_LEN        1
#define FS1818_PLLCTRL4_REG_PLLEN_MSK        0x1
#define FS1818_PLLCTRL4_REG_OSCEN            (0x01<<1)
#define FS1818_PLLCTRL4_REG_OSCEN_POS        1
#define FS1818_PLLCTRL4_REG_OSCEN_LEN        1
#define FS1818_PLLCTRL4_REG_OSCEN_MSK        0x2
#define FS1818_PLLCTRL4_REG_ZMEN             (0x01<<2)
#define FS1818_PLLCTRL4_REG_ZMEN_POS         2
#define FS1818_PLLCTRL4_REG_ZMEN_LEN         1
#define FS1818_PLLCTRL4_REG_ZMEN_MSK         0x4
#define FS1818_PLLCTRL4_REG_VBGEN            (0x01<<3)
#define FS1818_PLLCTRL4_REG_VBGEN_POS        3
#define FS1818_PLLCTRL4_REG_VBGEN_LEN        1
#define FS1818_PLLCTRL4_REG_VBGEN_MSK        0x8

/* OCCTRL (0xC5) */

#define FS1818_OCCTRL_REG                    0xC5

#define FS1818_OCCTRL_REG_OCNUM              (0x01<<0)
#define FS1818_OCCTRL_REG_OCNUM_POS          0
#define FS1818_OCCTRL_REG_OCNUM_LEN          8
#define FS1818_OCCTRL_REG_OCNUM_MSK          0xff

/* OTCTRL (0xC6) */

#define FS1818_OTCTRL_REG                    0xC6

#define FS1818_OTCTRL_REG_OTTHD_L            (0x01<<0)
#define FS1818_OTCTRL_REG_OTTHD_L_POS        0
#define FS1818_OTCTRL_REG_OTTHD_L_LEN        8
#define FS1818_OTCTRL_REG_OTTHD_L_MSK        0xff
#define FS1818_OTCTRL_REG_OTTHD_H            (0x01<<8)
#define FS1818_OTCTRL_REG_OTTHD_H_POS        8
#define FS1818_OTCTRL_REG_OTTHD_H_LEN        8
#define FS1818_OTCTRL_REG_OTTHD_H_MSK        0xff00

/* UVCTRL (0xC7) */

#define FS1818_UVCTRL_REG                    0xC7

#define FS1818_UVCTRL_REG_UVTHD_L            (0x01<<0)
#define FS1818_UVCTRL_REG_UVTHD_L_POS        0
#define FS1818_UVCTRL_REG_UVTHD_L_LEN        8
#define FS1818_UVCTRL_REG_UVTHD_L_MSK        0xff
#define FS1818_UVCTRL_REG_UVTHD_H            (0x01<<8)
#define FS1818_UVCTRL_REG_UVTHD_H_POS        8
#define FS1818_UVCTRL_REG_UVTHD_H_LEN        8
#define FS1818_UVCTRL_REG_UVTHD_H_MSK        0xff00

/* OVCTRL (0xC8) */

#define FS1818_OVCTRL_REG                    0xC8

#define FS1818_OVCTRL_REG_OVTHD_L            (0x01<<0)
#define FS1818_OVCTRL_REG_OVTHD_L_POS        0
#define FS1818_OVCTRL_REG_OVTHD_L_LEN        8
#define FS1818_OVCTRL_REG_OVTHD_L_MSK        0xff
#define FS1818_OVCTRL_REG_OVTHD_H            (0x01<<8)
#define FS1818_OVCTRL_REG_OVTHD_H_POS        8
#define FS1818_OVCTRL_REG_OVTHD_H_LEN        8
#define FS1818_OVCTRL_REG_OVTHD_H_MSK        0xff00

/* SPKERR (0xC9) */

#define FS1818_SPKERR_REG                    0xC9

#define FS1818_SPKERR_REG_CLEAR              0x0000
#define FS1818_SPKERR_REG_THRD               (0x01<<0)
#define FS1818_SPKERR_REG_THRD_POS           0
#define FS1818_SPKERR_REG_THRD_LEN           16
#define FS1818_SPKERR_REG_THRD_MSK           0xffff

/* SPKM24 (0xCA) */

#define FS1818_SPKM24_REG                    0xCA

#define FS1818_SPKM24_REG_CLEAR              0x0000
#define FS1818_SPKM24_REG_THRD               (0x01<<0)
#define FS1818_SPKM24_REG_THRD_POS           0
#define FS1818_SPKM24_REG_THRD_LEN           16
#define FS1818_SPKM24_REG_THRD_MSK           0xffff

/* SPKM6 (0xCB) */

#define FS1818_SPKM6_REG                     0xCB

#define FS1818_SPKM6_REG_CLEAR               0x0000
#define FS1818_SPKM6_REG_THRD                (0x01<<0)
#define FS1818_SPKM6_REG_THRD_POS            0
#define FS1818_SPKM6_REG_THRD_LEN            16
#define FS1818_SPKM6_REG_THRD_MSK            0xffff

/* SPKRE (0xCC) */

#define FS1818_SPKRE_REG                     0xCC

#define FS1818_SPKRE_REG_CLEAR               0x0000
#define FS1818_SPKRE_REG_THRD                (0x01<<0)
#define FS1818_SPKRE_REG_THRD_POS            0
#define FS1818_SPKRE_REG_THRD_LEN            16
#define FS1818_SPKRE_REG_THRD_MSK            0xffff

/* SPKMDB (0xCD) */

#define FS1818_SPKMDB_REG                    0xCD

#define FS1818_SPKMDB_SET                    0x2004
#define FS1818_SPKMDB_REG_INIT               0x1604
#define FS1818_SPKMDB_REG_DBVSPKM6           (0x01<<0)
#define FS1818_SPKMDB_REG_DBVSPKM6_POS       0
#define FS1818_SPKMDB_REG_DBVSPKM6_LEN       8
#define FS1818_SPKMDB_REG_DBVSPKM6_MSK       0xff
#define FS1818_SPKMDB_REG_DBVSPKM24          (0x01<<8)
#define FS1818_SPKMDB_REG_DBVSPKM24_POS      8
#define FS1818_SPKMDB_REG_DBVSPKM24_LEN      8
#define FS1818_SPKMDB_REG_DBVSPKM24_MSK      0xff00

/* ADPBST (0xCF) */

#define FS1818_ADPBST_REG                    0xCF

#define FS1818_ADPBST_REG_TFSEL              (0x01<<2)
#define FS1818_ADPBST_REG_TFSEL_POS          2
#define FS1818_ADPBST_REG_TFSEL_LEN          2
#define FS1818_ADPBST_REG_TFSEL_MSK          0xc
#define FS1818_ADPBST_REG_TRSEL              (0x01<<4)
#define FS1818_ADPBST_REG_TRSEL_POS          4
#define FS1818_ADPBST_REG_TRSEL_LEN          2
#define FS1818_ADPBST_REG_TRSEL_MSK          0x30
#define FS1818_ADPBST_REG_DISCHARGESEL       (0x01<<14)
#define FS1818_ADPBST_REG_DISCHARGESEL_POS   14
#define FS1818_ADPBST_REG_DISCHARGESEL_LEN   2
#define FS1818_ADPBST_REG_DISCHARGESEL_MSK   0xc000

/* ANACTRL (0xD0) */

#define FS1818_ANACTRL_REG                   0xD0

#define FS1818_ANACTRL_REG_BPCLKCK           (0x01<<0)
#define FS1818_ANACTRL_REG_BPCLKCK_POS       0
#define FS1818_ANACTRL_REG_BPCLKCK_LEN       1
#define FS1818_ANACTRL_REG_BPCLKCK_MSK       0x1
#define FS1818_ANACTRL_REG_BPUVOV            (0x01<<1)
#define FS1818_ANACTRL_REG_BPUVOV_POS        1
#define FS1818_ANACTRL_REG_BPUVOV_LEN        1
#define FS1818_ANACTRL_REG_BPUVOV_MSK        0x2
#define FS1818_ANACTRL_REG_BPOT              (0x01<<2)
#define FS1818_ANACTRL_REG_BPOT_POS          2
#define FS1818_ANACTRL_REG_BPOT_LEN          1
#define FS1818_ANACTRL_REG_BPOT_MSK          0x4
#define FS1818_ANACTRL_REG_BPOC              (0x01<<3)
#define FS1818_ANACTRL_REG_BPOC_POS          3
#define FS1818_ANACTRL_REG_BPOC_LEN          1
#define FS1818_ANACTRL_REG_BPOC_MSK          0x8
#define FS1818_ANACTRL_REG_BPOV              (0x01<<4)
#define FS1818_ANACTRL_REG_BPOV_POS          4
#define FS1818_ANACTRL_REG_BPOV_LEN          1
#define FS1818_ANACTRL_REG_BPOV_MSK          0x10
#define FS1818_ANACTRL_REG_BPSPKOT           (0x01<<5)
#define FS1818_ANACTRL_REG_BPSPKOT_POS       5
#define FS1818_ANACTRL_REG_BPSPKOT_LEN       1
#define FS1818_ANACTRL_REG_BPSPKOT_MSK       0x20
#define FS1818_ANACTRL_REG_PTSEQBP           (0x01<<6)
#define FS1818_ANACTRL_REG_PTSEQBP_POS       6
#define FS1818_ANACTRL_REG_PTSEQBP_LEN       1
#define FS1818_ANACTRL_REG_PTSEQBP_MSK       0x40
#define FS1818_ANACTRL_REG_HWSEQEN           (0x01<<8)
#define FS1818_ANACTRL_REG_HWSEQEN_POS       8
#define FS1818_ANACTRL_REG_HWSEQEN_LEN       1
#define FS1818_ANACTRL_REG_HWSEQEN_MSK       0x100
#define FS1818_ANACTRL_REG_FSWSDLY           (0x01<<12)
#define FS1818_ANACTRL_REG_FSWSDLY_POS       12
#define FS1818_ANACTRL_REG_FSWSDLY_LEN       3
#define FS1818_ANACTRL_REG_FSWSDLY_MSK       0x7000
#define FS1818_ANACTRL_REG_FSWSDLYEN         (0x01<<15)
#define FS1818_ANACTRL_REG_FSWSDLYEN_POS     15
#define FS1818_ANACTRL_REG_FSWSDLYEN_LEN     1
#define FS1818_ANACTRL_REG_FSWSDLYEN_MSK     0x8000

/* REFGEN (0xD3) */

#define FS1818_CLDCTRL_REG                   0xD3
#define FS1818_CLDCTRL_DEFAULT               0x0100
#define FS1818_CLDCTRL_DISABLE               0x0000

/* REFGEN (0xD4) */

#define FS1818_REFGEN_REG                    0xD4

#define FS1818_REFGEN_REG_TMSELVIN3          (0x01<<12)
#define FS1818_REFGEN_REG_TMSELVIN3_POS      12
#define FS1818_REFGEN_REG_TMSELVIN3_LEN      1
#define FS1818_REFGEN_REG_TMSELVIN3_MSK      0x1000
#define FS1818_REFGEN_REG_TMSELVBG           (0x01<<13)
#define FS1818_REFGEN_REG_TMSELVBG_POS       13
#define FS1818_REFGEN_REG_TMSELVBG_LEN       1
#define FS1818_REFGEN_REG_TMSELVBG_MSK       0x2000

/* ZMCONFIG (0xD5) */

#define FS1818_ZMCONFIG_REG                  0xD5

#define FS1818_ZMCONFIG_REG_CALIB_BIT        0x0010
#define FS1818_ZMCONFIG_REG_NORMAL_BIT       0x0000
#define FS1818_ZMCONFIG_REG_CALIBEN          (0x01<<4)
#define FS1818_ZMCONFIG_REG_CALIBEN_POS      4
#define FS1818_ZMCONFIG_REG_CALIBEN_LEN      1
#define FS1818_ZMCONFIG_REG_CALIBEN_MSK      0x10

/* AUXCFG (0xD7) */

#define FS1818_AUXCFG_REG                    0xD7

#define FS1818_AUXCFG_REG_CRAMACKSEL         (0x01<<1)
#define FS1818_AUXCFG_REG_CRAMACKSEL_POS     1
#define FS1818_AUXCFG_REG_CRAMACKSEL_LEN     1
#define FS1818_AUXCFG_REG_CRAMACKSEL_MSK     0x2
#define FS1818_AUXCFG_REG_CLKSWFORCESEL      (0x01<<4)
#define FS1818_AUXCFG_REG_CLKSWFORCESEL_POS  4
#define FS1818_AUXCFG_REG_CLKSWFORCESEL_LEN  1
#define FS1818_AUXCFG_REG_CLKSWFORCESEL_MSK  0x10
#define FS1818_AUXCFG_REG_CLKSWFORCE         (0x01<<5)
#define FS1818_AUXCFG_REG_CLKSWFORCE_POS     5
#define FS1818_AUXCFG_REG_CLKSWFORCE_LEN     1
#define FS1818_AUXCFG_REG_CLKSWFORCE_MSK     0x20
#define FS1818_AUXCFG_REG_DSPRST             (0x01<<8)
#define FS1818_AUXCFG_REG_DSPRST_POS         8
#define FS1818_AUXCFG_REG_DSPRST_LEN         1
#define FS1818_AUXCFG_REG_DSPRST_MSK         0x100
#define FS1818_AUXCFG_REG_DCBP               (0x01<<12)
#define FS1818_AUXCFG_REG_DCBP_POS           12
#define FS1818_AUXCFG_REG_DCBP_LEN           1
#define FS1818_AUXCFG_REG_DCBP_MSK           0x1000

/* I2CCTRL (0xD9) */

#define FS1818_I2CCTRL_REG                   0xD9

#define FS1818_I2CCTRL_REG_TIMEOUTEN         (0x01<<0)
#define FS1818_I2CCTRL_REG_TIMEOUTEN_POS     0
#define FS1818_I2CCTRL_REG_TIMEOUTEN_LEN     1
#define FS1818_I2CCTRL_REG_TIMEOUTEN_MSK     0x1
#define FS1818_I2CCTRL_REG_BPDEGLITCH        (0x01<<1)
#define FS1818_I2CCTRL_REG_BPDEGLITCH_POS    1
#define FS1818_I2CCTRL_REG_BPDEGLITCH_LEN    1
#define FS1818_I2CCTRL_REG_BPDEGLITCH_MSK    0x2

/* OTPCMD (0xDC) */

#define FS1818_OTPCMD_REG                    0xDC

#define FS1818_OTPCMD_CMD1                   0x0000
#define FS1818_OTPCMD_CMD2                   0x0001
#define FS1818_OTPCMD_CMD3                   0x2000
#define FS1818_OTPCMD_CMD4                   0x2002
#define FS1818_OTPCMD_REG_R                  (0x01<<0)
#define FS1818_OTPCMD_REG_R_POS              0
#define FS1818_OTPCMD_REG_R_LEN              1
#define FS1818_OTPCMD_REG_R_MSK              0x1
#define FS1818_OTPCMD_REG_W                  (0x01<<1)
#define FS1818_OTPCMD_REG_W_POS              1
#define FS1818_OTPCMD_REG_W_LEN              1
#define FS1818_OTPCMD_REG_W_MSK              0x2
#define FS1818_OTPCMD_REG_BUSY               (0x01<<2)
#define FS1818_OTPCMD_REG_BUSY_POS           2
#define FS1818_OTPCMD_REG_BUSY_LEN           1
#define FS1818_OTPCMD_REG_BUSY_MSK           0x4
#define FS1818_OTPCMD_REG_EPROM_LD           (0x01<<8)
#define FS1818_OTPCMD_REG_EPROM_LD_POS       8
#define FS1818_OTPCMD_REG_EPROM_LD_LEN       1
#define FS1818_OTPCMD_REG_EPROM_LD_MSK       0x100
#define FS1818_OTPCMD_REG_PW                 (0x01<<12)
#define FS1818_OTPCMD_REG_PW_POS             12
#define FS1818_OTPCMD_REG_PW_LEN             1
#define FS1818_OTPCMD_REG_PW_MSK             0x1000
#define FS1818_OTPCMD_REG_WMODE              (0x01<<13)
#define FS1818_OTPCMD_REG_WMODE_POS          13
#define FS1818_OTPCMD_REG_WMODE_LEN          1
#define FS1818_OTPCMD_REG_WMODE_MSK          0x2000

/* OTPADDR (0xDD) */

#define FS1818_OTPADDR_REG                 0xDD
#define FS1818_OTPADDR_ADDR                0x0010

/* OTPWDATA (0xDE) */

#define FS1818_OTPWDATA_REG                0xDE

/* OTPRDATA (0xDF) */

#define FS1818_OTPRDATA_REG                0xDF

/* OTPPG0W0 (0xE0) */

#define FS1818_OTPPG0W0_REG                0xE0

/* OTPPG0W1 (0xE1) */

#define FS1818_OTPPG0W1_REG                0xE1

/* OTPPG0W2 (0xE2) */

#define FS1818_OTPPG0W2_REG                0xE2

/* OTPPG0W3 (0xE3) */

#define FS1818_OTPPG0W3_REG                0xE3

/* OTPPG1W0 (0xE4) */

#define FS1818_OTPPG1W0_REG                0xE4

/* OTPPG1W1 (0xE5) */

#define FS1818_OTPPG1W1_REG                0xE5

/* OTPPG1W2 (0xE6) */

#define FS1818_OTPPG1W2_REG                0xE6

/* OTPPG1W3 (0xE7) */

#define FS1818_OTPPG1W3_REG                0xE7

/* OTPPG2 (0xE8) */

#define FS1818_OTPPG2_REG                  0xE8

#define FS1818_OTPPG2_REG_DEFAULT          0xFF10

#endif