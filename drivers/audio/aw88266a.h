/****************************************************************************
 * drivers/audio/aw88266a.h
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

#ifndef __DRIVERS_AUDIO_AW88266AH__
#define __DRIVERS_AUDIO_AW88266AH__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* registers list */

#define AW88266A_ID_REG                             (0x00)
#define AW88266A_SYSST_REG                          (0x01)
#define AW88266A_SYSINT_REG                         (0x02)
#define AW88266A_SYSINTM_REG                        (0x03)
#define AW88266A_SYSCTRL_REG                        (0x04)
#define AW88266A_SYSCTRL2_REG                       (0x05)
#define AW88266A_I2SCTRL_REG                        (0x06)
#define AW88266A_I2SCFG1_REG                        (0x07)
#define AW88266A_I2SCFG2_REG                        (0x08)
#define AW88266A_HAGCCFG1_REG                       (0x09)
#define AW88266A_HAGCCFG2_REG                       (0x0A)
#define AW88266A_HAGCCFG3_REG                       (0x0B)
#define AW88266A_HAGCCFG4_REG                       (0x0C)
#define AW88266A_HAGCCFG5_REG                       (0x0D)
#define AW88266A_HAGCCFG6_REG                       (0x0E)
#define AW88266A_HAGCCFG7_REG                       (0x0F)
#define AW88266A_HAGCST_REG                         (0x10)
#define AW88266A_PRODID_REG                         (0x11)
#define AW88266A_VBAT_REG                           (0x12)
#define AW88266A_TEMP_REG                           (0x13)
#define AW88266A_PVDD_REG                           (0x14)
#define AW88266A_DBGCTRL_REG                        (0x20)
#define AW88266A_I2SINT_REG                         (0x21)
#define AW88266A_I2SCAPCNT_REG                      (0x22)
#define AW88266A_CRCIN_REG                          (0x38)
#define AW88266A_CRCOUT_REG                         (0x39)
#define AW88266A_VSNCTRL1_REG                       (0x50)
#define AW88266A_ISNCTRL1_REG                       (0x52)
#define AW88266A_ISNCTRL2_REG                       (0x53)
#define AW88266A_VTMCTRL1_REG                       (0x54)
#define AW88266A_VTMCTRL2_REG                       (0x55)
#define AW88266A_VTMCTRL3_REG                       (0x56)
#define AW88266A_ISNDAT_REG                         (0x57)
#define AW88266A_VSNDAT_REG                         (0x58)
#define AW88266A_PWMCTRL_REG                        (0x59)
#define AW88266A_PWMCTRL2_REG                       (0x5A)
#define AW88266A_BSTCTRL1_REG                       (0x60)
#define AW88266A_BSTCTRL2_REG                       (0x61)
#define AW88266A_BSTCTRL3_REG                       (0x62)
#define AW88266A_BSTDBG1_REG                        (0x63)
#define AW88266A_BSTDBG2_REG                        (0x64)
#define AW88266A_BSTDBG3_REG                        (0x65)
#define AW88266A_PLLCTRL1_REG                       (0x66)
#define AW88266A_PLLCTRL2_REG                       (0x67)
#define AW88266A_PLLCTRL3_REG                       (0x68)
#define AW88266A_CDACTRL1_REG                       (0x69)
#define AW88266A_CDACTRL2_REG                       (0x6A)
#define AW88266A_SADCCTRL_REG                       (0x6B)
#define AW88266A_TESTCTRL1_REG                      (0x70)
#define AW88266A_TESTCTRL2_REG                      (0x71)
#define AW88266A_EFCTRL1_REG                        (0x72)
#define AW88266A_EFCTRL2_REG                        (0x73)
#define AW88266A_EFWH_REG                           (0x74)
#define AW88266A_EFWM2_REG                          (0x75)
#define AW88266A_EFWM1_REG                          (0x76)
#define AW88266A_EFWL_REG                           (0x77)
#define AW88266A_EFRH_REG                           (0x78)
#define AW88266A_EFRM2_REG                          (0x79)
#define AW88266A_EFRM1_REG                          (0x7A)
#define AW88266A_EFRL_REG                           (0x7B)
#define AW88266A_TESTDET_REG                        (0x7C)

#define AW88266A_REG_MAX                            (0x7D)

#define REG_NONE_ACCESS                             (0)
#define REG_RD_ACCESS                               (1 << 0)
#define REG_WR_ACCESS                               (1 << 1)
#define AW88266A_GAIN_MAX                           (767)
#define AW88266A_VOL_6DB_STEP                       (6 * 8)

/* detail information of registers begin
 * ID (0x00) detail
 * IDCODE bit 15:0 (ID 0x00)
 */

#define AW88266A_IDCODE_START_BIT                   (0)
#define AW88266A_IDCODE_BITS_LEN                    (16)
#define AW88266A_IDCODE_MASK                        \
  (~(((1 << AW88266A_IDCODE_BITS_LEN) - 1 ) << AW88266A_IDCODE_START_BIT))

#define AW88266A_IDCODE_DEFAULT_VALUE               (0x1852)
#define AW88266A_IDCODE_DEFAULT                     \
  (AW88266A_IDCODE_DEFAULT_VALUE << AW88266A_IDCODE_START_BIT)

/* default value of ID (0x00)
 * #define  AW88266A_ID_DEFAULT (0x1852)
 * SYSST (0x01) detail
 * OVP2S bit 15 (SYSST 0x01)
 */

#define AW88266A_OVP2S_START_BIT                    (15)
#define AW88266A_OVP2S_BITS_LEN                     (1)
#define AW88266A_OVP2S_MASK                         \
  (~(((1 << AW88266A_OVP2S_BITS_LEN) - 1) << AW88266A_OVP2S_START_BIT))

#define AW88266A_OVP2S_NONE                         (0)
#define AW88266A_OVP2S_NONE_VALUE                   \
  (AW88266A_OVP2S_NONE << AW88266A_OVP2S_START_BIT)
#define AW88266A_OVP2S_TRIG                         (1)
#define AW88266A_OVP2S_TRIG_VALUE                   \
  (AW88266A_OVP2S_TRIG << AW88266A_OVP2S_START_BIT)

#define AW88266A_OVP2S_DEFAULT_VALUE                (0)
#define AW88266A_OVP2S_DEFAULT                      \
  (AW88266A_OVP2S_DEFAULT_VALUE << AW88266A_OVP2S_START_BIT)

/* UVLS bit 14 (SYSST 0x01) */

#define AW88266A_UVLS_START_BIT                     (14)
#define AW88266A_UVLS_BITS_LEN                      (1)
#define AW88266A_UVLS_MASK                          \
  (~(((1 << AW88266A_UVLS_BITS_LEN) - 1) << AW88266A_UVLS_START_BIT))

#define AW88266A_UVLS_VDD_ABOVE_2P8V                (0)
#define AW88266A_UVLS_VDD_ABOVE_2P8V_VALUE          \
  (AW88266A_UVLS_VDD_ABOVE_2P8V << AW88266A_UVLS_START_BIT)

#define AW88266A_UVLS_VDD_BELOW_2P8V                (1)
#define AW88266A_UVLS_VDD_BELOW_2P8V_VALUE          \
  (AW88266A_UVLS_VDD_BELOW_2P8V << AW88266A_UVLS_START_BIT)

#define AW88266A_UVLS_DEFAULT_VALUE                 (0)
#define AW88266A_UVLS_DEFAULT                       \
  (AW88266A_UVLS_DEFAULT_VALUE << AW88266A_UVLS_START_BIT)

/* ADPS bit 13 (SYSST 0x01) */

#define AW88266A_ADPS_START_BIT                     (13)
#define AW88266A_ADPS_BITS_LEN                      (1)
#define AW88266A_ADPS_MASK                          \
  (~(((1 << AW88266A_ADPS_BITS_LEN) - 1) << AW88266A_ADPS_START_BIT))

#define AW88266A_ADPS_TRANSPARENT                   (0)
#define AW88266A_ADPS_TRANSPARENT_VALUE             \
  (AW88266A_ADPS_TRANSPARENT << AW88266A_ADPS_START_BIT)
#define AW88266A_ADPS_BOOST                         (1)
#define AW88266A_ADPS_BOOST_VALUE                   \
  (AW88266A_ADPS_BOOST << AW88266A_ADPS_START_BIT)

#define AW88266A_ADPS_DEFAULT_VALUE                 (0)
#define AW88266A_ADPS_DEFAULT                       \
  (AW88266A_ADPS_DEFAULT_VALUE << AW88266A_ADPS_START_BIT)

/* BSTOCS bit 11 (SYSST 0x01) */

#define AW88266A_BSTOCS_START_BIT                   (11)
#define AW88266A_BSTOCS_BITS_LEN                    (1)
#define AW88266A_BSTOCS_MASK                        \
  (~(((1 << AW88266A_BSTOCS_BITS_LEN) - 1) << AW88266A_BSTOCS_START_BIT))

#define AW88266A_BSTOCS_NONE                        (0)
#define AW88266A_BSTOCS_NONE_VALUE                  \
  (AW88266A_BSTOCS_NONE << AW88266A_BSTOCS_START_BIT)
#define AW88266A_BSTOCS_TRIG                        (1)
#define AW88266A_BSTOCS_TRIG_VALUE                  \
  (AW88266A_BSTOCS_TRIG << AW88266A_BSTOCS_START_BIT)

#define AW88266A_BSTOCS_DEFAULT_VALUE               (0)
#define AW88266A_BSTOCS_DEFAULT                     \
  (AW88266A_BSTOCS_DEFAULT_VALUE << AW88266A_BSTOCS_START_BIT)

/* OVPS bit 10 (SYSST 0x01) */

#define AW88266A_OVPS_START_BIT                     (10)
#define AW88266A_OVPS_BITS_LEN                      (1)
#define AW88266A_OVPS_MASK                          \
  (~(((1 << AW88266A_OVPS_BITS_LEN) - 1) << AW88266A_OVPS_START_BIT))

#define AW88266A_OVPS_NONE                          (0)
#define AW88266A_OVPS_NONE_VALUE                    \
  (AW88266A_OVPS_NONE << AW88266A_OVPS_START_BIT)
#define AW88266A_OVPS_TRIG                          (1)
#define AW88266A_OVPS_NONE_VALUE                    \
  (AW88266A_OVPS_NONE << AW88266A_OVPS_START_BIT)

#define AW88266A_OVPS_DEFAULT_VALUE                 (0)
#define AW88266A_OVPS_DEFAULT                       \
  (AW88266A_OVPS_DEFAULT_VALUE << AW88266A_OVPS_START_BIT)

/* BSTS bit 9 (SYSST 0x01) */

#define AW88266A_BSTS_START_BIT                     (9)
#define AW88266A_BSTS_BITS_LEN                      (1)
#define AW88266A_BSTS_MASK                          \
  (~(((1 << AW88266A_BSTS_BITS_LEN) - 1) << AW88266A_BSTS_START_BIT))

#define AW88266A_BSTS_NOT_FINISHED                  (0)
#define AW88266A_BSTS_NOT_FINISHED_VALUE            \
  (AW88266A_BSTS_NOT_FINISHED << AW88266A_BSTS_START_BIT)
#define AW88266A_BSTS_FINISHED                      (1)
#define AW88266A_BSTS_FINISHED_VALUE                \
  (AW88266A_BSTS_FINISHED << AW88266A_BSTS_START_BIT)

#define AW88266A_BSTS_DEFAULT_VALUE                 (0)
#define AW88266A_BSTS_DEFAULT                       \
  (AW88266A_BSTS_DEFAULT_VALUE << AW88266A_BSTS_START_BIT)

/* SWS bit 8 (SYSST 0x01) */

#define AW88266A_SWS_START_BIT                      (8)
#define AW88266A_SWS_BITS_LEN                       (1)
#define AW88266A_SWS_MASK                           \
  (~(((1 << AW88266A_SWS_BITS_LEN) - 1) << AW88266A_SWS_START_BIT))

#define AW88266A_SWS_NOT_SWITCHING                  (0)
#define AW88266A_SWS_NOT_SWITCHING_VALUE            \
  (AW88266A_SWS_NOT_SWITCHING << AW88266A_SWS_START_BIT)
#define AW88266A_SWS_SWITCHING                      (1)
#define AW88266A_SWS_SWITCHING_VALUE                \
  (AW88266A_SWS_SWITCHING << AW88266A_SWS_START_BIT)

#define AW88266A_SWS_DEFAULT_VALUE                  (0)
#define AW88266A_SWS_DEFAULT                        \
  (AW88266A_SWS_DEFAULT_VALUE << AW88266A_SWS_START_BIT)

/* CLIPS bit 7 (SYSST 0x01) */

#define AW88266A_CLIPS_START_BIT                    (7)
#define AW88266A_CLIPS_BITS_LEN                     (1)
#define AW88266A_CLIPS_MASK                         \
  (~(((1 << AW88266A_CLIPS_BITS_LEN) - 1) << AW88266A_CLIPS_START_BIT))

#define AW88266A_CLIPS_NOT_CLIPPING                 (0)
#define AW88266A_CLIPS_NOT_CLIPPING_VALUE           \
  (AW88266A_CLIPS_NOT_CLIPPING << AW88266A_CLIPS_START_BIT)

#define AW88266A_CLIPS_CLIPPING                     (1)
#define AW88266A_CLIPS_CLIPPING_VALUE               \
  (AW88266A_CLIPS_CLIPPING << AW88266A_CLIPS_START_BIT)

#define AW88266A_CLIPS_DEFAULT_VALUE                (0)
#define AW88266A_CLIPS_DEFAULT                      \
  (AW88266A_CLIPS_DEFAULT_VALUE << AW88266A_CLIPS_START_BIT)

/* NOCLKS bit 5 (SYSST 0x01) */

#define AW88266A_NOCLKS_START_BIT                   (5)
#define AW88266A_NOCLKS_BITS_LEN                    (1)
#define AW88266A_NOCLKS_MASK                        \
  (~(((1 << AW88266A_NOCLKS_BITS_LEN) - 1) << AW88266A_NOCLKS_START_BIT))

#define AW88266A_NOCLKS_NONE                        (0)
#define AW88266A_NOCLKS_NONE_VALUE                  \
  (AW88266A_NOCLKS_NONE << AW88266A_NOCLKS_START_BIT)

#define AW88266A_NOCLKS_TRIG                        (1)
#define AW88266A_NOCLKS_TRIG_VALUE                  \
  (AW88266A_NOCLKS_NONE << AW88266A_NOCLKS_START_BIT)

#define AW88266A_NOCLKS_DEFAULT_VALUE               (0)
#define AW88266A_NOCLKS_DEFAULT                     \
  (AW88266A_NOCLKS_DEFAULT_VALUE << AW88266A_NOCLKS_START_BIT)

/* CLKS bit 4 (SYSST 0x01) */

#define AW88266A_CLKS_START_BIT                     (4)
#define AW88266A_CLKS_BITS_LEN                      (1)
#define AW88266A_CLKS_MASK                          \
  (~(((1 << AW88266A_CLKS_BITS_LEN) - 1) << AW88266A_CLKS_START_BIT))

#define AW88266A_CLKS_NONE                          (0)
#define AW88266A_CLKS_NONE_VALUE                    \
  (AW88266A_CLKS_NONE << AW88266A_CLKS_START_BIT)
#define AW88266A_CLKS_TRIG                          (1)
#define AW88266A_CLKS_TRIG_VALUE                    \
  (AW88266A_CLKS_TRIG << AW88266A_CLKS_START_BIT)

#define AW88266A_CLKS_DEFAULT_VALUE                 (0)
#define AW88266A_CLKS_DEFAULT                       \
  (AW88266A_CLKS_DEFAULT_VALUE << AW88266A_CLKS_START_BIT)

/* OCDS bit 3 (SYSST 0x01) */

#define AW88266A_OCDS_START_BIT                     (3)
#define AW88266A_OCDS_BITS_LEN                      (1)
#define AW88266A_OCDS_MASK                          \
  (~(((1 << AW88266A_OCDS_BITS_LEN) - 1) << AW88266A_OCDS_START_BIT))

#define AW88266A_OCDS_NONE                          (0)
#define AW88266A_OCDS_NONE_VALUE                    \
  (AW88266A_OCDS_NONE << AW88266A_OCDS_START_BIT)
#define AW88266A_OCDS_TRIG                          (1)
#define AW88266A_OCDS_TRIG_VALUE                    \
  (AW88266A_OCDS_TRIG << AW88266A_OCDS_START_BIT)

#define AW88266A_OCDS_DEFAULT_VALUE                 (0)
#define AW88266A_OCDS_DEFAULT                       \
  (AW88266A_OCDS_DEFAULT_VALUE << AW88266A_OCDS_START_BIT)

/* CLIP_PRES bit 2 (SYSST 0x01) */

#define AW88266A_CLIP_PRES_START_BIT                (2)
#define AW88266A_CLIP_PRES_BITS_LEN                 (1)
#define AW88266A_CLIP_PRES_MASK                     \
  (~(((1 << AW88266A_CLIP_PRES_BITS_LEN) - 1) << AW88266A_CLIP_PRES_START_BIT))

#define AW88266A_CLIP_PRES_NONE                     (0)
#define AW88266A_CLIP_PRES_NONE_VALUE               \
  (AW88266A_CLIP_PRES_NONE << AW88266A_CLIP_PRES_START_BIT)
#define AW88266A_CLIP_PRES_TRIG                     (1)
#define AW88266A_CLIP_PRES_TRIG_VALUE               \
  (AW88266A_CLIP_PRES_TRIG << AW88266A_CLIP_PRES_START_BIT)

#define AW88266A_CLIP_PRES_DEFAULT_VALUE            (0)
#define AW88266A_CLIP_PRES_DEFAULT                  \
  (AW88266A_CLIP_PRES_DEFAULT_VALUE << AW88266A_CLIP_PRES_START_BIT)

/* OTHS bit 1 (SYSST 0x01) */

#define AW88266A_OTHS_START_BIT                     (1)
#define AW88266A_OTHS_BITS_LEN                      (1)
#define AW88266A_OTHS_MASK                          \
  (~(((1 << AW88266A_OTHS_BITS_LEN) - 1) << AW88266A_OTHS_START_BIT))

#define AW88266A_OTHS_NONE                          (0)
#define AW88266A_OTHS_NONE_VALUE                    \
  (AW88266A_OTHS_NONE << AW88266A_OTHS_START_BIT)
#define AW88266A_OTHS_TRIG                          (1)
#define AW88266A_OTHS_TRIG_VALUE                    \
  (AW88266A_OTHS_TRIG << AW88266A_OTHS_START_BIT)

#define AW88266A_OTHS_DEFAULT_VALUE                 (0)
#define AW88266A_OTHS_DEFAULT                       \
  (AW88266A_OTHS_DEFAULT_VALUE<<AW88266A_OTHS_START_BIT)

/* PLLS bit 0 (SYSST 0x01) */

#define AW88266A_PLLS_START_BIT                     (0)
#define AW88266A_PLLS_BITS_LEN                      (1)
#define AW88266A_PLLS_MASK                          \
  (~(((1 << AW88266A_PLLS_BITS_LEN) - 1) << AW88266A_PLLS_START_BIT))

#define AW88266A_PLLS_UNLOCKED                      (0)
#define AW88266A_PLLS_UNLOCKED_VALUE                \
  (AW88266A_PLLS_UNLOCKED << AW88266A_PLLS_START_BIT)
#define AW88266A_PLLS_LOCKED                        (1)
#define AW88266A_PLLS_LOCKED_VALUE                  \
  (AW88266A_PLLS_LOCKED << AW88266A_PLLS_START_BIT)

#define AW88266A_PLLS_DEFAULT_VALUE                 (0)
#define AW88266A_PLLS_DEFAULT                       \
  (AW88266A_PLLS_DEFAULT_VALUE << AW88266A_PLLS_START_BIT)

#define AW88266A_SYSST_CHECK_MASK                   \
  (~(AW88266A_UVLS_VDD_BELOW_2P8V_VALUE |           \
  AW88266A_BSTOCS_TRIG_VALUE |                      \
  AW88266A_BSTS_FINISHED_VALUE |                    \
  AW88266A_SWS_SWITCHING_VALUE |                    \
  AW88266A_NOCLKS_TRIG_VALUE |                      \
  AW88266A_CLKS_TRIG_VALUE |                        \
  AW88266A_OCDS_TRIG_VALUE |                        \
  AW88266A_OTHS_TRIG_VALUE |                        \
  AW88266A_PLLS_LOCKED_VALUE))

#define AW88266A_SYSST_CHECK                        \
  (AW88266A_CLKS_TRIG_VALUE |                       \
  AW88266A_PLLS_LOCKED_VALUE)

#define AW88266A_IIS_CHECK_MASK                     \
  (~(AW88266A_UVLS_VDD_BELOW_2P8V_VALUE |           \
  AW88266A_NOCLKS_TRIG_VALUE |                      \
  AW88266A_CLKS_TRIG_VALUE |                        \
  AW88266A_OCDS_TRIG_VALUE |                        \
  AW88266A_OTHS_TRIG_VALUE |                        \
  AW88266A_PLLS_LOCKED_VALUE))

#define AW88266A_IIS_CHECK                          \
  (AW88266A_CLKS_TRIG_VALUE |                       \
  AW88266A_PLLS_LOCKED_VALUE)

/* default value of SYSST (0x01)
 * #define  AW88266A_SYSST_DEFAULT (0x0000)
 * SYSINT (0x02) detail
 * OVP2I bit 15 (SYSINT 0x02)
 */

#define AW88266A_OVP2I_START_BIT                    (15)
#define AW88266A_OVP2I_BITS_LEN                     (1)
#define AW88266A_OVP2I_MASK                         \
  (~(((1 << AW88266A_OVP2I_BITS_LEN) - 1) << AW88266A_OVP2I_START_BIT))

#define AW88266A_OVP2I_DEFAULT_VALUE                (0)
#define AW88266A_OVP2I_DEFAULT                      \
  (AW88266A_OVP2I_DEFAULT_VALUE << AW88266A_OVP2I_START_BIT)

/* UVLI bit 14 (SYSINT 0x02) */

#define AW88266A_UVLI_START_BIT                     (14)
#define AW88266A_UVLI_BITS_LEN                      (1)
#define AW88266A_UVLI_MASK                          \
  (~(((1 << AW88266A_UVLI_BITS_LEN) - 1) << AW88266A_UVLI_START_BIT))

#define AW88266A_UVLI_DEFAULT_VALUE                 (0)
#define AW88266A_UVLI_DEFAULT                       \
  (AW88266A_UVLI_DEFAULT_VALUE << AW88266A_UVLI_START_BIT)

/* ADPI bit 13 (SYSINT 0x02) */

#define AW88266A_ADPI_START_BIT                     (13)
#define AW88266A_ADPI_BITS_LEN                      (1)
#define AW88266A_ADPI_MASK                          \
  (~(((1 << AW88266A_ADPI_BITS_LEN) - 1) << AW88266A_ADPI_START_BIT))

#define AW88266A_ADPI_DEFAULT_VALUE                 (0)
#define AW88266A_ADPI_DEFAULT                       \
  (AW88266A_ADPI_DEFAULT_VALUE << AW88266A_ADPI_START_BIT)

/* BSTOCI bit 11 (SYSINT 0x02) */

#define AW88266A_BSTOCI_START_BIT                   (11)
#define AW88266A_BSTOCI_BITS_LEN                    (1)
#define AW88266A_BSTOCI_MASK                        \
  (~(((1 << AW88266A_BSTOCI_BITS_LEN) - 1) << AW88266A_BSTOCI_START_BIT))

#define AW88266A_BSTOCI_DEFAULT_VALUE               (0)
#define AW88266A_BSTOCI_DEFAULT                     \
  (AW88266A_BSTOCI_DEFAULT_VALUE << AW88266A_BSTOCI_START_BIT)

/* OVPI bit 10 (SYSINT 0x02) */

#define AW88266A_OVPI_START_BIT                     (10)
#define AW88266A_OVPI_BITS_LEN                      (1)
#define AW88266A_OVPI_MASK                          \
  (~(((1 << AW88266A_OVPI_BITS_LEN) - 1) << AW88266A_OVPI_START_BIT))

#define AW88266A_OVPI_DEFAULT_VALUE                 (0)
#define AW88266A_OVPI_DEFAULT                       \
  (AW88266A_OVPI_DEFAULT_VALUE << AW88266A_OVPI_START_BIT)

/* BSTI bit 9 (SYSINT 0x02) */

#define AW88266A_BSTI_START_BIT                     (9)
#define AW88266A_BSTI_BITS_LEN                      (1)
#define AW88266A_BSTI_MASK                          \
  (~(((1 << AW88266A_BSTI_BITS_LEN) - 1) << AW88266A_BSTI_START_BIT))

#define AW88266A_BSTI_DEFAULT_VALUE                 (0)
#define AW88266A_BSTI_DEFAULT                       \
  (AW88266A_BSTI_DEFAULT_VALUE << AW88266A_BSTI_START_BIT)

/* SWI bit 8 (SYSINT 0x02) */

#define AW88266A_SWI_START_BIT                      (8)
#define AW88266A_SWI_BITS_LEN                       (1)
#define AW88266A_SWI_MASK                           \
  (~(((1 << AW88266A_SWI_BITS_LEN) - 1) << AW88266A_SWI_START_BIT))

#define AW88266A_SWI_DEFAULT_VALUE                  (0)
#define AW88266A_SWI_DEFAULT                        \
  (AW88266A_SWI_DEFAULT_VALUE << AW88266A_SWI_START_BIT)

/* CLIPI bit 7 (SYSINT 0x02) */

#define AW88266A_CLIPI_START_BIT                    (7)
#define AW88266A_CLIPI_BITS_LEN                     (1)
#define AW88266A_CLIPI_MASK                         \
  (~(((1 << AW88266A_CLIPI_BITS_LEN) - 1) << AW88266A_CLIPI_START_BIT))

#define AW88266A_CLIPI_DEFAULT_VALUE                (0)
#define AW88266A_CLIPI_DEFAULT                      \
  (AW88266A_CLIPI_DEFAULT_VALUE << AW88266A_CLIPI_START_BIT)

/* NOCLKI bit 5 (SYSINT 0x02) */

#define AW88266A_NOCLKI_START_BIT                   (5)
#define AW88266A_NOCLKI_BITS_LEN                    (1)
#define AW88266A_NOCLKI_MASK                        \
  (~(((1 << AW88266A_NOCLKI_BITS_LEN) - 1) << AW88266A_NOCLKI_START_BIT))

#define AW88266A_NOCLKI_DEFAULT_VALUE               (0)
#define AW88266A_NOCLKI_DEFAULT                     \
  (AW88266A_NOCLKI_DEFAULT_VALUE << AW88266A_NOCLKI_START_BIT)

/* CLKI bit 4 (SYSINT 0x02) */

#define AW88266A_CLKI_START_BIT                     (4)
#define AW88266A_CLKI_BITS_LEN                      (1)
#define AW88266A_CLKI_MASK                          \
  (~(((1 << AW88266A_CLKI_BITS_LEN) - 1) << AW88266A_CLKI_START_BIT))

#define AW88266A_CLKI_DEFAULT_VALUE                 (0)
#define AW88266A_CLKI_DEFAULT                       \
  (AW88266A_CLKI_DEFAULT_VALUE << AW88266A_CLKI_START_BIT)

/* OCDI bit 3 (SYSINT 0x02) */
#define AW88266A_OCDI_START_BIT                     (3)
#define AW88266A_OCDI_BITS_LEN                      (1)
#define AW88266A_OCDI_MASK                          \
  (~(((1 << AW88266A_OCDI_BITS_LEN) - 1) << AW88266A_OCDI_START_BIT))

#define AW88266A_OCDI_DEFAULT_VALUE                 (0)
#define AW88266A_OCDI_DEFAULT                       \
  (AW88266A_OCDI_DEFAULT_VALUE << AW88266A_OCDI_START_BIT)

/* CLIP_PREI bit 2 (SYSINT 0x02) */

#define AW88266A_CLIP_PREI_START_BIT                (2)
#define AW88266A_CLIP_PREI_BITS_LEN                 (1)
#define AW88266A_CLIP_PREI_MASK                     \
  (~(((1 << AW88266A_CLIP_PREI_BITS_LEN) - 1) << AW88266A_CLIP_PREI_START_BIT))

#define AW88266A_CLIP_PREI_DEFAULT_VALUE            (0)
#define AW88266A_CLIP_PREI_DEFAULT                  \
  (AW88266A_CLIP_PREI_DEFAULT_VALUE << AW88266A_CLIP_PREI_START_BIT)

/* OTHI bit 1 (SYSINT 0x02) */

#define AW88266A_OTHI_START_BIT                     (1)
#define AW88266A_OTHI_BITS_LEN                      (1)
#define AW88266A_OTHI_MASK                          \
  (~(((1 << AW88266A_OTHI_BITS_LEN) - 1) << AW88266A_OTHI_START_BIT))

#define AW88266A_OTHI_DEFAULT_VALUE                 (0)
#define AW88266A_OTHI_DEFAULT                       \
  (AW88266A_OTHI_DEFAULT_VALUE << AW88266A_OTHI_START_BIT)

/* PLLI bit 0 (SYSINT 0x02) */

#define AW88266A_PLLI_START_BIT                     (0)
#define AW88266A_PLLI_BITS_LEN                      (1)
#define AW88266A_PLLI_MASK                          \
  (~(((1 << AW88266A_PLLI_BITS_LEN) - 1) << AW88266A_PLLI_START_BIT))

#define AW88266A_PLLI_DEFAULT_VALUE                 (0)
#define AW88266A_PLLI_DEFAULT                       \
  (AW88266A_PLLI_DEFAULT_VALUE << AW88266A_PLLI_START_BIT)

/* default value of SYSINT (0x02)
 * #define  AW88266A_SYSINT_DEFAULT (0x0000)
 * SYSINTM (0x03) detail
 * OVP2M bit 15 (SYSINTM 0x03)
 */

#define AW88266A_OVP2M_START_BIT                    (15)
#define AW88266A_OVP2M_BITS_LEN                     (1)
#define AW88266A_OVP2M_MASK                         \
  (~(((1 << AW88266A_OVP2M_BITS_LEN) - 1) << AW88266A_OVP2M_START_BIT))

#define AW88266A_OVP2M_DEFAULT_VALUE                (1)
#define AW88266A_OVP2M_DEFAULT                      \
  (AW88266A_OVP2M_DEFAULT_VALUE << AW88266A_OVP2M_START_BIT)

/* UVLM bit 14 (SYSINTM 0x03) */

#define AW88266A_UVLM_START_BIT                     (14)
#define AW88266A_UVLM_BITS_LEN                      (1)
#define AW88266A_UVLM_MASK                          \
  (~(((1 << AW88266A_UVLM_BITS_LEN) - 1) << AW88266A_UVLM_START_BIT))

#define AW88266A_UVLM_DEFAULT_VALUE                 (1)
#define AW88266A_UVLM_DEFAULT                       \
  (AW88266A_UVLM_DEFAULT_VALUE << AW88266A_UVLM_START_BIT)

/* ADPM bit 13 (SYSINTM 0x03) */

#define AW88266A_ADPM_START_BIT                     (13)
#define AW88266A_ADPM_BITS_LEN                      (1)
#define AW88266A_ADPM_MASK                          \
  (~(((1 << AW88266A_ADPM_BITS_LEN) - 1) << AW88266A_ADPM_START_BIT))

#define AW88266A_ADPM_DEFAULT_VALUE                 (1)
#define AW88266A_ADPM_DEFAULT                       \
  (AW88266A_ADPM_DEFAULT_VALUE<<AW88266A_ADPM_START_BIT)

/* BSTOCM bit 11 (SYSINTM 0x03) */

#define AW88266A_BSTOCM_START_BIT                   (11)
#define AW88266A_BSTOCM_BITS_LEN                    (1)
#define AW88266A_BSTOCM_MASK                        \
  (~(((1 << AW88266A_BSTOCM_BITS_LEN) - 1) << AW88266A_BSTOCM_START_BIT))

#define AW88266A_BSTOCM_DEFAULT_VALUE               (1)
#define AW88266A_BSTOCM_DEFAULT                     \
  (AW88266A_BSTOCM_DEFAULT_VALUE << AW88266A_BSTOCM_START_BIT)

/* OVPM bit 10 (SYSINTM 0x03) */

#define AW88266A_OVPM_START_BIT                     (10)
#define AW88266A_OVPM_BITS_LEN                      (1)
#define AW88266A_OVPM_MASK                          \
  (~(((1 << AW88266A_OVPM_BITS_LEN) - 1) << AW88266A_OVPM_START_BIT))

#define AW88266A_OVPM_DEFAULT_VALUE                 (1)
#define AW88266A_OVPM_DEFAULT                       \
  (AW88266A_OVPM_DEFAULT_VALUE << AW88266A_OVPM_START_BIT)

/* BSTM bit 9 (SYSINTM 0x03) */

#define AW88266A_BSTM_START_BIT                     (9)
#define AW88266A_BSTM_BITS_LEN                      (1)
#define AW88266A_BSTM_MASK                          \
  (~(((1 << AW88266A_BSTM_BITS_LEN) - 1) << AW88266A_BSTM_START_BIT))

#define AW88266A_BSTM_DEFAULT_VALUE                 (1)
#define AW88266A_BSTM_DEFAULT                       \
  (AW88266A_BSTM_DEFAULT_VALUE << AW88266A_BSTM_START_BIT)

/* SWM bit 8 (SYSINTM 0x03) */

#define AW88266A_SWM_START_BIT                      (8)
#define AW88266A_SWM_BITS_LEN                       (1)
#define AW88266A_SWM_MASK                           \
  (~(((1 << AW88266A_SWM_BITS_LEN) - 1) << AW88266A_SWM_START_BIT))

#define AW88266A_SWM_DEFAULT_VALUE                  (1)
#define AW88266A_SWM_DEFAULT                        \
  (AW88266A_SWM_DEFAULT_VALUE << AW88266A_SWM_START_BIT)

/* CLIPM bit 7 (SYSINTM 0x03) */

#define AW88266A_CLIPM_START_BIT                    (7)
#define AW88266A_CLIPM_BITS_LEN                     (1)
#define AW88266A_CLIPM_MASK                         \
  (~(((1 << AW88266A_CLIPM_BITS_LEN) - 1) << AW88266A_CLIPM_START_BIT))

#define AW88266A_CLIPM_DEFAULT_VALUE                (1)
#define AW88266A_CLIPM_DEFAULT                      \
  (AW88266A_CLIPM_DEFAULT_VALUE << AW88266A_CLIPM_START_BIT)

/* NOCLKM bit 5 (SYSINTM 0x03) */
#define AW88266A_NOCLKM_START_BIT                   (5)
#define AW88266A_NOCLKM_BITS_LEN                    (1)
#define AW88266A_NOCLKM_MASK                        \
  (~(((1 << AW88266A_NOCLKM_BITS_LEN) - 1) << AW88266A_NOCLKM_START_BIT))

#define AW88266A_NOCLKM_DEFAULT_VALUE               (1)
#define AW88266A_NOCLKM_DEFAULT                     \
  (AW88266A_NOCLKM_DEFAULT_VALUE << AW88266A_NOCLKM_START_BIT)

/* CLKM bit 4 (SYSINTM 0x03) */

#define AW88266A_CLKM_START_BIT                     (4)
#define AW88266A_CLKM_BITS_LEN                      (1)
#define AW88266A_CLKM_MASK                          \
  (~(((1 << AW88266A_CLKM_BITS_LEN) - 1) << AW88266A_CLKM_START_BIT))

#define AW88266A_CLKM_DEFAULT_VALUE                 (1)
#define AW88266A_CLKM_DEFAULT                       \
  (AW88266A_CLKM_DEFAULT_VALUE << AW88266A_CLKM_START_BIT)

/* OCDM bit 3 (SYSINTM 0x03) */

#define AW88266A_OCDM_START_BIT                     (3)
#define AW88266A_OCDM_BITS_LEN                      (1)
#define AW88266A_OCDM_MASK                          \
  (~(((1 << AW88266A_OCDM_BITS_LEN) - 1) << AW88266A_OCDM_START_BIT))

#define AW88266A_OCDM_DEFAULT_VALUE                 (1)
#define AW88266A_OCDM_DEFAULT                       \
  (AW88266A_OCDM_DEFAULT_VALUE << AW88266A_OCDM_START_BIT)

/* CLIP_PREM bit 2 (SYSINTM 0x03) */

#define AW88266A_CLIP_PREM_START_BIT                (2)
#define AW88266A_CLIP_PREM_BITS_LEN                 (1)
#define AW88266A_CLIP_PREM_MASK                     \
  (~(((1 << AW88266A_CLIP_PREM_BITS_LEN) - 1) << AW88266A_CLIP_PREM_START_BIT))

#define AW88266A_CLIP_PREM_DEFAULT_VALUE            (1)
#define AW88266A_CLIP_PREM_DEFAULT                  \
  (AW88266A_CLIP_PREM_DEFAULT_VALUE << AW88266A_CLIP_PREM_START_BIT)

/* OTHM bit 1 (SYSINTM 0x03) */

#define AW88266A_OTHM_START_BIT                     (1)
#define AW88266A_OTHM_BITS_LEN                      (1)
#define AW88266A_OTHM_MASK                          \
  (~(((1 << AW88266A_OTHM_BITS_LEN) - 1) << AW88266A_OTHM_START_BIT))

#define AW88266A_OTHM_DEFAULT_VALUE                 (1)
#define AW88266A_OTHM_DEFAULT                       \
  (AW88266A_OTHM_DEFAULT_VALUE << AW88266A_OTHM_START_BIT)

/* PLLM bit 0 (SYSINTM 0x03) */

#define AW88266A_PLLM_START_BIT                     (0)
#define AW88266A_PLLM_BITS_LEN                      (1)
#define AW88266A_PLLM_MASK                          \
  (~(((1 << AW88266A_PLLM_BITS_LEN) - 1) << AW88266A_PLLM_START_BIT))

#define AW88266A_PLLM_DEFAULT_VALUE                 (1)
#define AW88266A_PLLM_DEFAULT                       \
  (AW88266A_PLLM_DEFAULT_VALUE << AW88266A_PLLM_START_BIT)

/* default value of SYSINTM (0x03)
 * #define  AW88266A_SYSINTM_DEFAULT (0xEFBF)
 * SYSCTRL (0x04) detail
 * SPK_GAIN bit 14:12 (SYSCTRL 0x04)
 */

#define AW88266A_SPK_GAIN_START_BIT                 (12)
#define AW88266A_SPK_GAIN_BITS_LEN                  (3)
#define AW88266A_SPK_GAIN_MASK                      \
  (~((( 1 << AW88266A_SPK_GAIN_BITS_LEN) - 1) << AW88266A_SPK_GAIN_START_BIT))

#define AW88266A_SPK_GAIN_AV7                       (0)
#define AW88266A_SPK_GAIN_AV7_VALUE                 \
  (AW88266A_SPK_GAIN_AV7 << AW88266A_SPK_GAIN_START_BIT)
#define AW88266A_SPK_GAIN_AV8                       (1)
#define AW88266A_SPK_GAIN_AV8_VALUE                 \
  (AW88266A_SPK_GAIN_AV8 << AW88266A_SPK_GAIN_START_BIT)
#define AW88266A_SPK_GAIN_AV10                      (2)
#define AW88266A_SPK_GAIN_AV10_VALUE                \
  (AW88266A_SPK_GAIN_AV10 << AW88266A_SPK_GAIN_START_BIT)
#define AW88266A_SPK_GAIN_AV14                      (3)
#define AW88266A_SPK_GAIN_AV14_VALUE                \
  (AW88266A_SPK_GAIN_AV14 << AW88266A_SPK_GAIN_START_BIT)
#define AW88266A_SPK_GAIN_AV16                      (4)
#define AW88266A_SPK_GAIN_AV16_VALUE                \
  (AW88266A_SPK_GAIN_AV16 << AW88266A_SPK_GAIN_START_BIT)
#define AW88266A_SPK_GAIN_AV20                      (5)
#define AW88266A_SPK_GAIN_AV20_VALUE                \
  (AW88266A_SPK_GAIN_AV20 << AW88266A_SPK_GAIN_START_BIT)

#define AW88266A_SPK_GAIN_DEFAULT_VALUE             (0x4)
#define AW88266A_SPK_GAIN_DEFAULT                   \
  (AW88266A_SPK_GAIN_DEFAULT_VALUE << AW88266A_SPK_GAIN_START_BIT)

/* RCV_GAIN bit 11:10 (SYSCTRL 0x04) */

#define AW88266A_RCV_GAIN_START_BIT                 (10)
#define AW88266A_RCV_GAIN_BITS_LEN                  (2)
#define AW88266A_RCV_GAIN_MASK                      \
  (~(((1 << AW88266A_RCV_GAIN_BITS_LEN) - 1) << AW88266A_RCV_GAIN_START_BIT))

#define AW88266A_RCV_GAIN_AV4P5                     (0)
#define AW88266A_RCV_GAIN_AV4P5_VALUE               \
  (AW88266A_RCV_GAIN_AV4P5 << AW88266A_RCV_GAIN_START_BIT)
#define AW88266A_RCV_GAIN_AV5                       (1)
#define AW88266A_RCV_GAIN_AV5_VALUE                 \
  (AW88266A_RCV_GAIN_AV5 << AW88266A_RCV_GAIN_START_BIT)
#define AW88266A_RCV_GAIN_AV5P5                     (2)
#define AW88266A_RCV_GAIN_AV5P5_VALUE               \
  (AW88266A_RCV_GAIN_AV5P5 << AW88266A_RCV_GAIN_START_BIT)
#define AW88266A_RCV_GAIN_AV7P5                     (3)
#define AW88266A_RCV_GAIN_AV7P5_VALUE               \
  (AW88266A_RCV_GAIN_AV7P5 << AW88266A_RCV_GAIN_START_BIT)

#define AW88266A_RCV_GAIN_DEFAULT_VALUE             (0)
#define AW88266A_RCV_GAIN_DEFAULT                   \
  (AW88266A_RCV_GAIN_DEFAULT_VALUE << AW88266A_RCV_GAIN_START_BIT)

/* INTMODE bit 9 (SYSCTRL 0x04) */

#define AW88266A_INTMODE_START_BIT                  (9)
#define AW88266A_INTMODE_BITS_LEN                   (1)
#define AW88266A_INTMODE_MASK                       \
  (~(((1 << AW88266A_INTMODE_BITS_LEN) - 1) << AW88266A_INTMODE_START_BIT))

#define AW88266A_INTMODE_OPENDRAIN                  (0)
#define AW88266A_INTMODE_OPENDRAIN_VALUE            \
  (AW88266A_INTMODE_OPENDRAIN << AW88266A_INTMODE_START_BIT)
#define AW88266A_INTMODE_PUSHPULL                   (1)
#define AW88266A_INTMODE_PUSHPULL_VALUE             \
  (AW88266A_INTMODE_PUSHPULL << AW88266A_INTMODE_START_BIT)

#define AW88266A_INTMODE_DEFAULT_VALUE              (0)
#define AW88266A_INTMODE_DEFAULT                    \
  (AW88266A_INTMODE_DEFAULT_VALUE<<AW88266A_INTMODE_START_BIT)

/* INTN bit 8 (SYSCTRL 0x04) */

#define AW88266A_INTN_START_BIT                     (8)
#define AW88266A_INTN_BITS_LEN                      (1)
#define AW88266A_INTN_MASK                          \
  (~(((1 << AW88266A_INTN_BITS_LEN) - 1) << AW88266A_INTN_START_BIT))

#define AW88266A_INTN_SYSINT                        (0)
#define AW88266A_INTN_SYSINT_VALUE                  \
  (AW88266A_INTN_SYSINT << AW88266A_INTN_START_BIT)
#define AW88266A_INTN_SYSST                         (1)
#define AW88266A_INTN_SYSST_VALUE                   \
  (AW88266A_INTN_SYSST << AW88266A_INTN_START_BIT)

#define AW88266A_INTN_DEFAULT_VALUE                 (0)
#define AW88266A_INTN_DEFAULT                       \
  (AW88266A_INTN_DEFAULT_VALUE << AW88266A_INTN_START_BIT)

/* RCV_MODE bit 7 (SYSCTRL 0x04) */

#define AW88266A_RCV_MODE_START_BIT                 (7)
#define AW88266A_RCV_MODE_BITS_LEN                  (1)
#define AW88266A_RCV_MODE_MASK                      \
  (~(((1 << AW88266A_RCV_MODE_BITS_LEN) - 1) << AW88266A_RCV_MODE_START_BIT))

#define AW88266A_RCV_MODE_SPEAKER                   (0)
#define AW88266A_RCV_MODE_SPEAKER_MODE              \
  (AW88266A_RCV_MODE_SPEAKER_MODE << AW88266A_RCV_MODE_START_BIT)
#define AW88266A_RCV_MODE_RECEIVER                  (1)
#define AW88266A_RCV_MODE_RECEIVER_MODE             \
  (AW88266A_RCV_MODE_RECEIVER_MODE << AW88266A_RCV_MODE_START_BIT)

#define AW88266A_RCV_MODE_DEFAULT_VALUE             (0)
#define AW88266A_RCV_MODE_DEFAULT                   \
  (AW88266A_RCV_MODE_DEFAULT_VALUE<<AW88266A_RCV_MODE_START_BIT)

/* I2SEN bit 6 (SYSCTRL 0x04) */

#define AW88266A_I2SEN_START_BIT                    (6)
#define AW88266A_I2SEN_BITS_LEN                     (1)
#define AW88266A_I2SEN_MASK                         \
  (~(((1<<AW88266A_I2SEN_BITS_LEN)-1)<<AW88266A_I2SEN_START_BIT))

#define AW88266A_I2SEN_DISABLE                      (0)
#define AW88266A_I2SEN_DISABLE_VALUE                \
  (AW88266A_I2SEN_DISABLE << AW88266A_I2SEN_START_BIT)
#define AW88266A_I2SEN_ENABLE                       (1)
#define AW88266A_I2SEN_ENABLE_VALUE                 \
  (AW88266A_I2SEN_ENABLE << AW88266A_I2SEN_START_BIT)

#define AW88266A_I2SEN_DEFAULT_VALUE                (0)
#define AW88266A_I2SEN_DEFAULT                      \
  (AW88266A_I2SEN_DEFAULT_VALUE<<AW88266A_I2SEN_START_BIT)

/* WSINV bit 5 (SYSCTRL 0x04) */

#define AW88266A_WSINV_START_BIT                    (5)
#define AW88266A_WSINV_BITS_LEN                     (1)
#define AW88266A_WSINV_MASK                         \
  (~(((1<<AW88266A_WSINV_BITS_LEN)-1)<<AW88266A_WSINV_START_BIT))

#define AW88266A_WSINV_NO_SWITCH                    (0)
#define AW88266A_WSINV_NO_SWITCH_VALUE              \
  (AW88266A_WSINV_NO_SWITCH << AW88266A_WSINV_START_BIT)
#define AW88266A_WSINV_LEFTRIGHT_SWITCH             (1)
#define AW88266A_WSINV_LEFTRIGHT_SWITCH_VALUE       \
  (AW88266A_WSINV_LEFTRIGHT_SWITCH << AW88266A_WSINV_START_BIT)

#define AW88266A_WSINV_DEFAULT_VALUE                (0)
#define AW88266A_WSINV_DEFAULT                      \
  (AW88266A_WSINV_DEFAULT_VALUE << AW88266A_WSINV_START_BIT)

/* BCKINV bit 4 (SYSCTRL 0x04) */

#define AW88266A_BCKINV_START_BIT                   (4)
#define AW88266A_BCKINV_BITS_LEN                    (1)
#define AW88266A_BCKINV_MASK                        \
  (~(((1 << AW88266A_BCKINV_BITS_LEN) - 1) << AW88266A_BCKINV_START_BIT))

#define AW88266A_BCKINV_NOT_INVERT                  (0)
#define AW88266A_BCKINV_NOT_INVERT_VALUE            \
  (AW88266A_BCKINV_NOT_INVERT << AW88266A_BCKINV_START_BIT)
#define AW88266A_BCKINV_INVERTED                    (1)
#define AW88266A_BCKINV_INVERTED_VALUE              \
  (AW88266A_BCKINV_INVERTED << AW88266A_BCKINV_START_BIT)

#define AW88266A_BCKINV_DEFAULT_VALUE               (0)
#define AW88266A_BCKINV_DEFAULT                     \
  (AW88266A_BCKINV_DEFAULT_VALUE << AW88266A_BCKINV_START_BIT)

/* IPLL bit 3 (SYSCTRL 0x04) */

#define AW88266A_IPLL_START_BIT                     (3)
#define AW88266A_IPLL_BITS_LEN                      (1)
#define AW88266A_IPLL_MASK                          \
  (~(((1 << AW88266A_IPLL_BITS_LEN) - 1) << AW88266A_IPLL_START_BIT))

#define AW88266A_IPLL_BCK                           (0)
#define AW88266A_IPLL_BCK_VALUE                     \
  (AW88266A_IPLL_BCK << AW88266A_IPLL_START_BIT)
#define AW88266A_IPLL_WCK                           (1)
#define AW88266A_IPLL_WCK_VALUE                     \
  (AW88266A_IPLL_WWCK << AW88266A_IPLL_START_BIT)

#define AW88266A_IPLL_DEFAULT_VALUE                 (0)
#define AW88266A_IPLL_DEFAULT                       \
  (AW88266A_IPLL_DEFAULT_VALUE << AW88266A_IPLL_START_BIT)

/* AMPPD bit 1 (SYSCTRL 0x04) */

#define AW88266A_AMPPD_START_BIT                    (1)
#define AW88266A_AMPPD_BITS_LEN                     (1)
#define AW88266A_AMPPD_MASK                         \
  (~(((1 << AW88266A_AMPPD_BITS_LEN) - 1) << AW88266A_AMPPD_START_BIT))

#define AW88266A_AMPPD_NORMAL_WORKING               (0)
#define AW88266A_AMPPD_NORMAL_WORKING_VALUE         \
  (AW88266A_AMPPD_NORMAL_WORKING << AW88266A_AMPPD_START_BIT)
#define AW88266A_AMPPD_POWER_DOWN                   (1)
#define AW88266A_AMPPD_POWER_DOWN_VALUE             \
  (AW88266A_AMPPD_POWER_DOWN << AW88266A_AMPPD_START_BIT)

#define AW88266A_AMPPD_DEFAULT_VALUE                (1)
#define AW88266A_AMPPD_DEFAULT                      \
  (AW88266A_AMPPD_DEFAULT_VALUE << AW88266A_AMPPD_START_BIT)

/* PWDN bit 0 (SYSCTRL 0x04) */

#define AW88266A_PWDN_START_BIT                     (0)
#define AW88266A_PWDN_BITS_LEN                      (1)
#define AW88266A_PWDN_MASK                          \
  (~(((1 << AW88266A_PWDN_BITS_LEN) - 1) << AW88266A_PWDN_START_BIT))

#define AW88266A_PWDN_NORMAL_WORKING                (0)
#define AW88266A_PWDN_NORMAL_WORKING_VALUE          \
  (AW88266A_PWDN_NORMAL_WORKING << AW88266A_PWDN_START_BIT)
#define AW88266A_PWDN_POWER_DOWN                    (1)
#define AW88266A_PWDN_POWER_DOWN_VALUE              \
  (AW88266A_PWDN_POWER_DOWN << AW88266A_PWDN_START_BIT)

#define AW88266A_PWDN_DEFAULT_VALUE                 (1)
#define AW88266A_PWDN_DEFAULT                       \
  (AW88266A_PWDN_DEFAULT_VALUE << AW88266A_PWDN_START_BIT)

/* default value of SYSCTRL (0x04)
 * #define  AW88266A_SYSCTRL_DEFAULT (0x4003)
 * SYSCTRL2 (0x05) detail
 * RMSE bit 7 (SYSCTRL2 0x05)
 */

#define AW88266A_RMSE_START_BIT                     (7)
#define AW88266A_RMSE_BITS_LEN                      (1)
#define AW88266A_RMSE_MASK                          \
  (~(((1 << AW88266A_RMSE_BITS_LEN) - 1) << AW88266A_RMSE_START_BIT))

#define AW88266A_RMSE_DISABLE                       (0)
#define AW88266A_RMSE_DISABLE_VALUE                 \
  (AW88266A_RMSE_DISABLE << AW88266A_RMSE_START_BIT)
#define AW88266A_RMSE_ENABLE                        (1)
#define AW88266A_RMSE_ENABLE_VALUE                  \
  (AW88266A_RMSE_ENABLE << AW88266A_RMSE_START_BIT)

#define AW88266A_RMSE_DEFAULT_VALUE                 (0)
#define AW88266A_RMSE_DEFAULT                       \
  (AW88266A_RMSE_DEFAULT_VALUE << AW88266A_RMSE_START_BIT)

/* HAGCE bit 6 (SYSCTRL2 0x05) */

#define AW88266A_HAGCE_START_BIT                    (6)
#define AW88266A_HAGCE_BITS_LEN                     (1)
#define AW88266A_HAGCE_MASK                         \
  (~(((1 << AW88266A_HAGCE_BITS_LEN) - 1) << AW88266A_HAGCE_START_BIT))

#define AW88266A_HAGCE_DISABLE                      (0)
#define AW88266A_HAGCE_DISABLE_VALUE                \
  (AW88266A_HAGCE_DISABLE << AW88266A_HAGCE_START_BIT)
#define AW88266A_HAGCE_ENABLE                       (1)
#define AW88266A_HAGCE_ENABLE_VALUE                 \
  (AW88266A_HAGCE_ENABLE << AW88266A_HAGCE_START_BIT)

#define AW88266A_HAGCE_DEFAULT_VALUE                (0)
#define AW88266A_HAGCE_DEFAULT                      \
  (AW88266A_HAGCE_DEFAULT_VALUE << AW88266A_HAGCE_START_BIT)

/* HDCCE bit 5 (SYSCTRL2 0x05) */

#define AW88266A_HDCCE_START_BIT                    (5)
#define AW88266A_HDCCE_BITS_LEN                     (1)
#define AW88266A_HDCCE_MASK                         \
  (~(((1 << AW88266A_HDCCE_BITS_LEN) - 1) << AW88266A_HDCCE_START_BIT))

#define AW88266A_HDCCE_DISABLE                      (0)
#define AW88266A_HDCCE_DISABLE_VALUE                \
  (AW88266A_HDCCE_DISABLE << AW88266A_HDCCE_START_BIT)
#define AW88266A_HDCCE_ENABLE                       (1)
#define AW88266A_HDCCE_ENABLE_VALUE                 \
  (AW88266A_HDCCE_ENABLE << AW88266A_HDCCE_START_BIT)

#define AW88266A_HDCCE_DEFAULT_VALUE                (1)
#define AW88266A_HDCCE_DEFAULT                      \
  (AW88266A_HDCCE_DEFAULT_VALUE << AW88266A_HDCCE_START_BIT)

/* HMUTE bit 4 (SYSCTRL2 0x05) */

#define AW88266A_HMUTE_START_BIT                    (4)
#define AW88266A_HMUTE_BITS_LEN                     (1)
#define AW88266A_HMUTE_MASK                         \
  (~(((1 << AW88266A_HMUTE_BITS_LEN) - 1) << AW88266A_HMUTE_START_BIT))

#define AW88266A_HMUTE_DISABLE                      (0)
#define AW88266A_HMUTE_DISABLE_VALUE                \
  (AW88266A_HMUTE_DISABLE << AW88266A_HMUTE_START_BIT)
#define AW88266A_HMUTE_ENABLE                       (1)
#define AW88266A_HMUTE_ENABLE_VALUE                 \
  (AW88266A_HMUTE_ENABLE << AW88266A_HMUTE_START_BIT)

#define AW88266A_HMUTE_DEFAULT_VALUE                (1)
#define AW88266A_HMUTE_DEFAULT                      \
  (AW88266A_HMUTE_DEFAULT_VALUE << AW88266A_HMUTE_START_BIT)

/* BST_IPEAK bit 3:0 (SYSCTRL2 0x05) */

#define AW88266A_BST_IPEAK_START_BIT                (0)
#define AW88266A_BST_IPEAK_BITS_LEN                 (4)
#define AW88266A_BST_IPEAK_MASK                     \
  (~(((1 << AW88266A_BST_IPEAK_BITS_LEN) - 1) << AW88266A_BST_IPEAK_START_BIT))

#define AW88266A_BST_IPEAK_1P5A                     (0)
#define AW88266A_BST_IPEAK_1P5A_VALUE               \
  (AW88266A_BST_IPEAK_1P5A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_1P75A                    (1)
#define AW88266A_BST_IPEAK_1P75A_VALUE              \
  (AW88266A_BST_IPEAK_1P75A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_2P0A                     (2)
#define AW88266A_BST_IPEAK_2P0A_VALUE               \
  (AW88266A_BST_IPEAK_2P0A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_2P25A                    (3)
#define AW88266A_BST_IPEAK_2P25A_VALUE              \
  (AW88266A_BST_IPEAK_2P25A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_2P5A                     (4)
#define AW88266A_BST_IPEAK_2P5A_VALUE               \
  (AW88266A_BST_IPEAK_2P5A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_2P75A                    (5)
#define AW88266A_BST_IPEAK_2P75A_VALUE              \
  (AW88266A_BST_IPEAK_2P75A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_3P0A                     (6)
#define AW88266A_BST_IPEAK_3P0A_VALUE               \
  (AW88266A_BST_IPEAK_3P0A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_3P25A                    (7)
#define AW88266A_BST_IPEAK_3P25A_VALUE              \
  (AW88266A_BST_IPEAK_3P25A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_3P5A                     (8)
#define AW88266A_BST_IPEAK_3P5A_VALUE               \
  (AW88266A_BST_IPEAK_3P5A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_3P75A                    (9)
#define AW88266A_BST_IPEAK_3P75A_VALUE              \
  (AW88266A_BST_IPEAK_3P75A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_4A                       (10)
#define AW88266A_BST_IPEAK_4A_VALUE                 \
  (AW88266A_BST_IPEAK_4A << AW88266A_BST_IPEAK_START_BIT)
#define AW88266A_BST_IPEAK_4P25A                    (11)
#define AW88266A_BST_IPEAK_4P25A_VALUE              \
  (AW88266A_BST_IPEAK_4P25A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_DEFAULT_VALUE            (8)
#define AW88266A_BST_IPEAK_DEFAULT                  \
  (AW88266A_BST_IPEAK_DEFAULT_VALUE << AW88266A_BST_IPEAK_START_BIT)

/* default value of SYSCTRL2 (0x05)
 * #define  AW88266A_SYSCTRL2_DEFAULT (0x0038)
 * I2SCTRL (0x06) detail
 * INPLEV bit 13 (I2SCTRL 0x06)
 */

#define AW88266A_INPLEV_START_BIT                   (13)
#define AW88266A_INPLEV_BITS_LEN                    (1)
#define AW88266A_INPLEV_MASK                        \
  (~(((1 << AW88266A_INPLEV_BITS_LEN) - 1) << AW88266A_INPLEV_START_BIT))

#define AW88266A_INPLEV_NOT_ATTENUATED              (0)
#define AW88266A_INPLEV_NOT_ATTENUATED_VALUE        \
  (AW88266A_INPLEV_NOT_ATTENUATED << AW88266A_INPLEV_START_BIT)
#define AW88266A_INPLEV_ATTENUATED_MINUS_6DB        (1)
#define AW88266A_INPLEV_ATTENUATED_MINUS_6DB_VALUE  \
  (AW88266A_INPLEV_ATTENUATED_MINUS_6DB << AW88266A_INPLEV_START_BIT)

#define AW88266A_INPLEV_DEFAULT_VALUE               (0)
#define AW88266A_INPLEV_DEFAULT                     \
  (AW88266A_INPLEV_DEFAULT_VALUE << AW88266A_INPLEV_START_BIT)

/* I2SRXEN bit 12 (I2SCTRL 0x06) */

#define AW88266A_I2SRXEN_START_BIT                  (12)
#define AW88266A_I2SRXEN_BITS_LEN                   (1)
#define AW88266A_I2SRXEN_MASK                       \
  (~(((1 << AW88266A_I2SRXEN_BITS_LEN) - 1) << AW88266A_I2SRXEN_START_BIT))

#define AW88266A_I2SRXEN_DISABLE                    (0)
#define AW88266A_I2SRXEN_DISABLE_VALUE              \
  (AW88266A_I2SRXEN_DISABLE << AW88266A_I2SRXEN_START_BIT)
#define AW88266A_I2SRXEN_ENABLE                     (1)
#define AW88266A_I2SRXEN_ENABLE_VALUE               \
  (AW88266A_I2SRXEN_ENABLE << AW88266A_I2SRXEN_START_BIT)

#define AW88266A_I2SRXEN_DEFAULT_VALUE              (1)
#define AW88266A_I2SRXEN_DEFAULT                    \
  (AW88266A_I2SRXEN_DEFAULT_VALUE << AW88266A_I2SRXEN_START_BIT)

/* CHSEL bit 11:10 (I2SCTRL 0x06) */

#define AW88266A_CHSEL_START_BIT                    (10)
#define AW88266A_CHSEL_BITS_LEN                     (2)
#define AW88266A_CHSEL_MASK                         \
  (~(((1 << AW88266A_CHSEL_BITS_LEN) - 1) << AW88266A_CHSEL_START_BIT))

#define AW88266A_CHSEL_RESERVED                     (0)
#define AW88266A_CHSEL_RESERVED_VALUE               \
  (AW88266A_CHSEL_RESERVED << AW88266A_CHSEL_START_BIT)
#define AW88266A_CHSEL_LEFT                         (1)
#define AW88266A_CHSEL_LEFT_VALUE                   \
  (AW88266A_CHSEL_LEFT << AW88266A_CHSEL_START_BIT)
#define AW88266A_CHSEL_RIGHT                        (2)
#define AW88266A_CHSEL_RIGHT_VALUE                  \
  (AW88266A_CHSEL_RIGHT << AW88266A_CHSEL_START_BIT)
#define AW88266A_CHSEL_MONO_LR2                     (3)
#define AW88266A_CHSEL_MONO_LR2_VALUE               \
  (AW88266A_CHSEL_MONO_LR2 << AW88266A_CHSEL_START_BIT)

#define AW88266A_CHSEL_DEFAULT_VALUE                (1)
#define AW88266A_CHSEL_DEFAULT                      \
  (AW88266A_CHSEL_DEFAULT_VALUE << AW88266A_CHSEL_START_BIT)

/* I2SMD bit 9:8 (I2SCTRL 0x06) */

#define AW88266A_I2SMD_START_BIT                    (8)
#define AW88266A_I2SMD_BITS_LEN                     (2)
#define AW88266A_I2SMD_MASK                         \
  (~(((1 << AW88266A_I2SMD_BITS_LEN) - 1) << AW88266A_I2SMD_START_BIT))

#define AW88266A_I2SMD_STANDARD_I2S                 (0)
#define AW88266A_I2SMD_STANDARD_I2S_VALUE           \
  (AW88266A_I2SMD_STANDARD_I2S << AW88266A_I2SMD_START_BIT)
#define AW88266A_I2SMD_MSB_JUSTIFIED                (1)
#define AW88266A_I2SMD_MSB_JUSTIFIED_VALUE          \
  (AW88266A_I2SMD_MSB_JUSTIFIED << AW88266A_I2SMD_START_BIT)
#define AW88266A_I2SMD_LSB_JUSTIFIED                (2)
#define AW88266A_I2SMD_LSB_JUSTIFIED_VALUE          \
  (AW88266A_I2SMD_LSB_JUSTIFIED << AW88266A_I2SMD_START_BIT)
#define AW88266A_I2SMD_RESERVED                     (3)
#define AW88266A_I2SMD_RESERVED_VALUE               \
  (AW88266A_I2SMD_RESERVED << AW88266A_I2SMD_START_BIT)

#define AW88266A_I2SMD_DEFAULT_VALUE                (0)
#define AW88266A_I2SMD_DEFAULT                      \
  (AW88266A_I2SMD_DEFAULT_VALUE << AW88266A_I2SMD_START_BIT)

/* I2SFS bit 7:6 (I2SCTRL 0x06) */

#define AW88266A_I2SFS_START_BIT                    (6)
#define AW88266A_I2SFS_BITS_LEN                     (2)
#define AW88266A_I2SFS_MASK                         \
  (~(((1 << AW88266A_I2SFS_BITS_LEN) - 1) << AW88266A_I2SFS_START_BIT))

#define AW88266A_I2SFS_16_BITS                      (0)
#define AW88266A_I2SFS_16_BITS_VALUE                \
  (AW88266A_I2SFS_16_BITS << AW88266A_I2SFS_START_BIT)
#define AW88266A_I2SFS_20_BITS                      (1)
#define AW88266A_I2SFS_20_BITS_VALUE                \
  (AW88266A_I2SFS_20_BITS << AW88266A_I2SFS_START_BIT)
#define AW88266A_I2SFS_24_BITS                      (2)
#define AW88266A_I2SFS_24_BITS_VALUE                \
  (AW88266A_I2SFS_24_BITS << AW88266A_I2SFS_START_BIT)
#define AW88266A_I2SFS_32_BITS                      (3)
#define AW88266A_I2SFS_32_BITS_VALUE                \
  (AW88266A_I2SFS_32_BITS << AW88266A_I2SFS_START_BIT)

#define AW88266A_I2SFS_DEFAULT_VALUE                (3)
#define AW88266A_I2SFS_DEFAULT                      \
  (AW88266A_I2SFS_DEFAULT_VALUE << AW88266A_I2SFS_START_BIT)

/* I2SBCK bit 5:4 (I2SCTRL 0x06) */

#define AW88266A_I2SBCK_START_BIT                   (4)
#define AW88266A_I2SBCK_BITS_LEN                    (2)
#define AW88266A_I2SBCK_MASK                        \
  (~(((1 << AW88266A_I2SBCK_BITS_LEN) - 1) << AW88266A_I2SBCK_START_BIT))

#define AW88266A_I2SBCK_32FS                        (0)
#define AW88266A_I2SBCK_32FS_VALUE                  \
  (AW88266A_I2SBCK_32FS << AW88266A_I2SBCK_START_BIT)
#define AW88266A_I2SBCK_48FS                        (1)
#define AW88266A_I2SBCK_48FS_VALUE                  \
  (AW88266A_I2SBCK_48FS << AW88266A_I2SBCK_START_BIT)
#define AW88266A_I2SBCK_64FS                        (2)
#define AW88266A_I2SBCK_64FS_VALUE                  \
  (AW88266A_I2SBCK_64FS << AW88266A_I2SBCK_START_BIT)
#define AW88266A_I2SBCK_RESERVED                    (3)
#define AW88266A_I2SBCK_RESERVED_VALUE              \
  (AW88266A_I2SBCK_RESERVED << AW88266A_I2SBCK_START_BIT)

#define AW88266A_I2SBCK_DEFAULT_VALUE	              (2)
#define AW88266A_I2SBCK_DEFAULT			                \
  (AW88266A_I2SBCK_DEFAULT_VALUE<<AW88266A_I2SBCK_START_BIT)

/* I2SSR bit 3:0 (I2SCTRL 0x06) */

#define AW88266A_I2SSR_START_BIT                    (0)
#define AW88266A_I2SSR_BITS_LEN                     (4)
#define AW88266A_I2SSR_MASK                         \
  (~(((1 << AW88266A_I2SSR_BITS_LEN) - 1) << AW88266A_I2SSR_START_BIT))

/* I2S CCO_UMX */

#define AW88266A_I2S_CCO_MUX_START_BIT              (14)
#define AW88266A_I2S_CCO_MUX_BITS_LEN               (1)
#define AW88266A_I2S_CCO_MUX_MASK                   \
 (~(((1 << AW88266A_I2S_CCO_MUX_BITS_LEN) - 1) << AW88266A_I2S_CCO_MUX_START_BIT))

#define AW88266A_I2S_CCO_MUX_8_16_32KHZ_BIT_VALUE   (0)
#define AW88266A_I2S_CCO_MUX_8_16_32KHZ_VALUE       \
  (AW88266A_I2S_CCO_MUX_8_16_32KHZ_BIT_VALUE << AW88266A_I2S_CCO_MUX_START_BIT)

#define AW88266A_I2S_CCO_MUX_EXC_8_16_32KHZ_BIT_VALUE (1)
#define AW88266A_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE     \
  (AW88266A_I2S_CCO_MUX_EXC_8_16_32KHZ_BIT_VALUE <<   \
  AW88266A_I2S_CCO_MUX_START_BIT)

#define AW88266A_I2SSR_8KHZ                         (0)
#define AW88266A_I2SSR_8KHZ_VALUE                   \
  (AW88266A_I2SSR_8KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_11P025KHZ                    (1)
#define AW88266A_I2SSR_11P025KHZ_VALUE              \
  (AW88266A_I2SSR_11P025KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_12KHZ                        (2)
#define AW88266A_I2SSR_12KHZ_VALUE                  \
  (AW88266A_I2SSR_12KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_16KHZ                        (3)
#define AW88266A_I2SSR_16KHZ_VALUE                  \
  (AW88266A_I2SSR_16KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_22P05KHZ                     (4)
#define AW88266A_I2SSR_22P05KHZ_VALUE               \
  (AW88266A_I2SSR_22P05KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_24KHZ                        (5)
#define AW88266A_I2SSR_24KHZ_VALUE                  \
  (AW88266A_I2SSR_24KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_32KHZ                        (6)
#define AW88266A_I2SSR_32KHZ_VALUE                  \
  (AW88266A_I2SSR_32KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_44P1KHZ                      (7)
#define AW88266A_I2SSR_44P1KHZ_VALUE                \
  (AW88266A_I2SSR_44P1KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_48KHZ                        (8)
#define AW88266A_I2SSR_48KHZ_VALUE                  \
  (AW88266A_I2SSR_48KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_96KHZ                        (9)
#define AW88266A_I2SSR_96KHZ_VALUE                  \
  (AW88266A_I2SSR_96KHZ << AW88266A_I2SSR_START_BIT)
#define AW88266A_I2SSR_192KHZ                       (10)
#define AW88266A_I2SSR_192KHZ_VALUE                 \
  (AW88266A_I2SSR_192KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_DEFAULT_VALUE                (8)
#define AW88266A_I2SSR_DEFAULT                      \
  (AW88266A_I2SSR_DEFAULT_VALUE<<AW88266A_I2SSR_START_BIT)

/* default value of I2SCTRL (0x06)
 * #define  AW88266A_I2SCTRL_DEFAULT  (0x14E8)
 * I2SCFG1 (0x07) detail
 * I2S_TX_SLOTVLD bit 13:12 (I2SCFG1 0x07)
 */

#define AW88266A_I2S_TX_SLOTVLD_START_BIT           (12)
#define AW88266A_I2S_TX_SLOTVLD_BITS_LEN            (2)
#define AW88266A_I2S_TX_SLOTVLD_MASK                \
  (~(((1 << AW88266A_I2S_TX_SLOTVLD_BITS_LEN) - 1) << \
  AW88266A_I2S_TX_SLOTVLD_START_BIT))

#define AW88266A_I2S_TX_SLOTVLD_SLOT_0              (0)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_0_VALUE        \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_0 << AW88266A_I2S_TX_SLOTVLD_START_BIT)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_1              (1)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_1_VALUE        \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_1 << AW88266A_I2S_TX_SLOTVLD_START_BIT)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_2              (2)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_2_VALUE        \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_2 << AW88266A_I2S_TX_SLOTVLD_START_BIT)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_3              (3)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_3_VALUE        \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_3 << AW88266A_I2S_TX_SLOTVLD_START_BIT)

#define AW88266A_I2S_TX_SLOTVLD_DEFAULT_VALUE       (0)
#define AW88266A_I2S_TX_SLOTVLD_DEFAULT             \
  (AW88266A_I2S_TX_SLOTVLD_DEFAULT_VALUE << AW88266A_I2S_TX_SLOTVLD_START_BIT)

/* I2S_RX_SLOTVLD bit 11:8 (I2SCFG1 0x07) */

#define AW88266A_I2S_RX_SLOTVLD_START_BIT           (8)
#define AW88266A_I2S_RX_SLOTVLD_BITS_LEN            (4)
#define AW88266A_I2S_RX_SLOTVLD_MASK                \
  (~(((1 << AW88266A_I2S_RX_SLOTVLD_BITS_LEN) - 1) << \
  AW88266A_I2S_RX_SLOTVLD_START_BIT))

#define AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_1       (3)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_1_VALUE \
  (AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_1 <<         \
  AW88266A_I2S_RX_SLOTVLD_START_BIT)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_2       (5)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_2_VALUE \
  (AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_2 <<         \
  AW88266A_I2S_RX_SLOTVLD_START_BIT)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_1_AND_2       (6)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_1_AND_2_VALUE \
  (AW88266A_I2S_RX_SLOTVLD_SLOTS_1_AND_2 <<         \
  AW88266A_I2S_RX_SLOTVLD_START_BIT)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_3       (9)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_3_VALUE \
  (AW88266A_I2S_RX_SLOTVLD_SLOTS_0_AND_3 <<         \
  AW88266A_I2S_RX_SLOTVLD_START_BIT)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_1_AND_3       (10)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_1_AND_3_VALUE \
  (AW88266A_I2S_RX_SLOTVLD_SLOTS_1_AND_3 <<         \
  AW88266A_I2S_RX_SLOTVLD_START_BIT)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_2_AND_3       (12)
#define AW88266A_I2S_RX_SLOTVLD_SLOTS_2_AND_3_VALUE \
  (AW88266A_I2S_RX_SLOTVLD_SLOTS_2_AND_3 <<         \
  AW88266A_I2S_RX_SLOTVLD_START_BIT)

#define AW88266A_I2S_RX_SLOTVLD_DEFAULT_VALUE       (3)
#define AW88266A_I2S_RX_SLOTVLD_DEFAULT             \
  (AW88266A_I2S_RX_SLOTVLD_DEFAULT_VALUE <<         \
  AW88266A_I2S_RX_SLOTVLD_START_BIT)

/* CFSEL bit 7:6 (I2SCFG1 0x07) */

#define AW88266A_CFSEL_START_BIT                    (6)
#define AW88266A_CFSEL_BITS_LEN                     (2)
#define AW88266A_CFSEL_MASK                         \
  (~(((1 << AW88266A_CFSEL_BITS_LEN) - 1) << AW88266A_CFSEL_START_BIT))

#define AW88266A_CFSEL_HAGC_DATA                    (0)
#define AW88266A_CFSEL_HAGC_DATA_VALUE              \
  (AW88266A_CFSEL_HAGC_DATA << AW88266A_CFSEL_START_BIT)
#define AW88266A_CFSEL_IV_SENSE_DATA                (1)
#define AW88266A_CFSEL_IV_SENSE_DATA_VALUE          \
  (AW88266A_CFSEL_IV_SENSE_DATA << AW88266A_CFSEL_START_BIT)
#define AW88266A_CFSEL_IVT_DATA_MAPPING_ACCORDING_TO_I2S_AUDIO_RES        (2)
#define AW88266A_CFSEL_IVT_DATA_MAPPING_ACCORDING_TO_I2S_AUDIO_RES_VALUE  \
  (AW88266A_CFSEL_IVT_DATA_MAPPING_ACCORDING_TO_I2S_AUDIO_RES <<          \
  AW88266A_CFSEL_START_BIT)
#define AW88266A_CFSEL_IVT_DATA_MAPPING_AS__I_PVDD_VBAT_V_T_SYNC          (3)
#define AW88266A_CFSEL_IVT_DATA_MAPPING_AS__I_PVDD_VBAT_V_T_SYNC_VALUE    \
  (AW88266A_CFSEL_IVT_DATA_MAPPING_AS__I_PVDD_VBAT_V_T_SYNC <<            \
  AW88266A_CFSEL_START_BIT)

#define AW88266A_CFSEL_DEFAULT_VALUE                (0)
#define AW88266A_CFSEL_DEFAULT                      \
  (AW88266A_CFSEL_DEFAULT_VALUE << AW88266A_CFSEL_START_BIT)

/* DRVSTREN bit 5 (I2SCFG1 0x07) */

#define AW88266A_DRVSTREN_START_BIT                 (5)
#define AW88266A_DRVSTREN_BITS_LEN                  (1)
#define AW88266A_DRVSTREN_MASK                      \
  (~(((1 << AW88266A_DRVSTREN_BITS_LEN) - 1) << AW88266A_DRVSTREN_START_BIT))

#define AW88266A_DRVSTREN_2MA                       (0)
#define AW88266A_DRVSTREN_2MA_VALUE                 \
  (AW88266A_DRVSTREN_2MA << AW88266A_DRVSTREN_START_BIT)
#define AW88266A_DRVSTREN_8MA                       (1)
#define AW88266A_DRVSTREN_8MA_VALUE                 \
  (AW88266A_DRVSTREN_8MA << AW88266A_DRVSTREN_START_BIT)

#define AW88266A_DRVSTREN_DEFAULT_VALUE             (1)
#define AW88266A_DRVSTREN_DEFAULT                   \
  (AW88266A_DRVSTREN_DEFAULT_VALUE << AW88266A_DRVSTREN_START_BIT)

/* DOHZ bit 4 (I2SCFG1 0x07) */

#define AW88266A_DOHZ_START_BIT                     (4)
#define AW88266A_DOHZ_BITS_LEN                      (1)
#define AW88266A_DOHZ_MASK                          \
  (~(((1 << AW88266A_DOHZ_BITS_LEN) - 1) << AW88266A_DOHZ_START_BIT))

#define AW88266A_DOHZ_ALL_AVAILABLE                 (0)
#define AW88266A_DOHZ_ALL_AVAILABLE_VALUE           \
  (AW88266A_DOHZ_ALL_AVAILABLE << AW88266A_DOHZ_START_BIT)
#define AW88266A_DOHZ_HIZ                           (1)
#define AW88266A_DOHZ_HIZ_VALUE                     \
  (AW88266A_DOHZ_HIZ << AW88266A_DOHZ_START_BIT)

#define AW88266A_DOHZ_DEFAULT_VALUE                 (1)
#define AW88266A_DOHZ_DEFAULT                       \
  (AW88266A_DOHZ_DEFAULT_VALUE << AW88266A_DOHZ_START_BIT)

/* FSYNC_TYPE bit 3 (I2SCFG1 0x07) */

#define AW88266A_FSYNC_TYPE_START_BIT               (3)
#define AW88266A_FSYNC_TYPE_BITS_LEN                (1)
#define AW88266A_FSYNC_TYPE_MASK                    \
  (~(((1 << AW88266A_FSYNC_TYPE_BITS_LEN) - 1) << AW88266A_FSYNC_TYPE_START_BIT))

#define AW88266A_FSYNC_TYPE_ONE_SLOT                (0)
#define AW88266A_FSYNC_TYPE_ONE_SLOT_VALUE          \
  (AW88266A_FSYNC_TYPE_ONE_SLOT << AW88266A_FSYNC_TYPE_START_BIT)
#define AW88266A_FSYNC_TYPE_ONE_BCK                 (1)
#define AW88266A_FSYNC_TYPE_ONE_BCK_VALUE           \
  (AW88266A_FSYNC_TYPE_ONE_BCK << AW88266A_FSYNC_TYPE_START_BIT)

#define AW88266A_FSYNC_TYPE_DEFAULT_VALUE           (0)
#define AW88266A_FSYNC_TYPE_DEFAULT                 \
  (AW88266A_FSYNC_TYPE_DEFAULT_VALUE << AW88266A_FSYNC_TYPE_START_BIT)

/* SLOT_NUM bit 2 (I2SCFG1 0x07) */

#define AW88266A_SLOT_NUM_START_BIT                 (2)
#define AW88266A_SLOT_NUM_BITS_LEN                  (1)
#define AW88266A_SLOT_NUM_MASK                      \
  (~(((1 << AW88266A_SLOT_NUM_BITS_LEN) - 1) << AW88266A_SLOT_NUM_START_BIT))

#define AW88266A_SLOT_NUM_2_SLOTS                   (0)
#define AW88266A_SLOT_NUM_2_SLOTS_VALUE             \
  (AW88266A_SLOT_NUM_2_SLOTS << AW88266A_SLOT_NUM_START_BIT)
#define AW88266A_SLOT_NUM_4_SLOTS                   (1)
#define AW88266A_SLOT_NUM_4_SLOTS_VALUE             \
  (AW88266A_SLOT_NUM_4_SLOTS << AW88266A_SLOT_NUM_START_BIT)

#define AW88266A_SLOT_NUM_DEFAULT_VALUE             (0)
#define AW88266A_SLOT_NUM_DEFAULT                   \
  (AW88266A_SLOT_NUM_DEFAULT_VALUE << AW88266A_SLOT_NUM_START_BIT)

/* I2SCHS bit 1 (I2SCFG1 0x07) */

#define AW88266A_I2SCHS_START_BIT                   (1)
#define AW88266A_I2SCHS_BITS_LEN                    (1)
#define AW88266A_I2SCHS_MASK                        \
  (~(((1 << AW88266A_I2SCHS_BITS_LEN) - 1) << AW88266A_I2SCHS_START_BIT))

#define AW88266A_I2SCHS_LEFT_CHANNEL                (0)
#define AW88266A_I2SCHS_LEFT_CHANNEL_VALUE          \
  (AW88266A_I2SCHS_LEFT_CHANNEL << AW88266A_I2SCHS_START_BIT)
#define AW88266A_I2SCHS_RIGHT_CHANNEL               (1)
#define AW88266A_I2SCHS_RIGHT_CHANNEL_VALUE         \
  (AW88266A_I2SCHS_RIGHT_CHANNEL << AW88266A_I2SCHS_START_BIT)

#define AW88266A_I2SCHS_DEFAULT_VALUE               (0)
#define AW88266A_I2SCHS_DEFAULT                     \
  (AW88266A_I2SCHS_DEFAULT_VALUE << AW88266A_I2SCHS_START_BIT)

/* I2STXEN bit 0 (I2SCFG1 0x07) */

#define AW88266A_I2STXEN_START_BIT                  (0)
#define AW88266A_I2STXEN_BITS_LEN                   (1)
#define AW88266A_I2STXEN_MASK                       \
  (~(((1 << AW88266A_I2STXEN_BITS_LEN) - 1) << AW88266A_I2STXEN_START_BIT))

#define AW88266A_I2STXEN_DISABLE                    (0)
#define AW88266A_I2STXEN_DISABLE_VALUE              \
  (AW88266A_I2STXEN_DISABLE << AW88266A_I2STXEN_START_BIT)
#define AW88266A_I2STXEN_ENABLE                     (1)
#define AW88266A_I2STXEN_ENABLE_VALUE               \
  (AW88266A_I2STXEN_ENABLE << AW88266A_I2STXEN_START_BIT)

#define AW88266A_I2STXEN_DEFAULT_VALUE              (0)
#define AW88266A_I2STXEN_DEFAULT                    \
  (AW88266A_I2STXEN_DEFAULT_VALUE << AW88266A_I2STXEN_START_BIT)

/* default value of I2SCFG1 (0x07)
 * #define  AW88266A_I2SCFG1_DEFAULT (0x0330)
 * I2SCFG2 (0x08) detail
 * RX_FLS bit 15 (I2SCFG2 0x08)
 */

#define AW88266A_RX_FLS_START_BIT                   (15)
#define AW88266A_RX_FLS_BITS_LEN                    (1)
#define AW88266A_RX_FLS_MASK                        \
  (~(((1 << AW88266A_RX_FLS_BITS_LEN) - 1) << AW88266A_RX_FLS_START_BIT))

#define AW88266A_RX_FLS_NORMAL_WORK_MODE            (0)
#define AW88266A_RX_FLS_NORMAL_WORK_MODE_VALUE      \
  (AW88266A_RX_FLS_NORMAL_WORK_MODE << AW88266A_RX_FLS_START_BIT)
#define AW88266A_RX_FLS_FLUSH_FIFO                  (1)
#define AW88266A_RX_FLS_FLUSH_FIFO_VALUE            \
  (AW88266A_RX_FLS_FLUSH_FIFO << AW88266A_RX_FLS_START_BIT)

#define AW88266A_RX_FLS_DEFAULT_VALUE               (0)
#define AW88266A_RX_FLS_DEFAULT                     \
  (AW88266A_RX_FLS_DEFAULT_VALUE << AW88266A_RX_FLS_START_BIT)

/* RX_THRS bit 13:12 (I2SCFG2 0x08) */

#define AW88266A_RX_THRS_START_BIT                  (12)
#define AW88266A_RX_THRS_BITS_LEN                   (2)
#define AW88266A_RX_THRS_MASK                       \
  (~(((1 << AW88266A_RX_THRS_BITS_LEN) - 1) << AW88266A_RX_THRS_START_BIT))

#define AW88266A_RX_THRS_DEFAULT_VALUE              (2)
#define AW88266A_RX_THRS_DEFAULT                    \
  (AW88266A_RX_THRS_DEFAULT_VALUE << AW88266A_RX_THRS_START_BIT)

/* TX_FLS bit 11 (I2SCFG2 0x08) */

#define AW88266A_TX_FLS_START_BIT                   (11)
#define AW88266A_TX_FLS_BITS_LEN                    (1)
#define AW88266A_TX_FLS_MASK                        \
 (~(((1 << AW88266A_TX_FLS_BITS_LEN) - 1) << AW88266A_TX_FLS_START_BIT))

#define AW88266A_TX_FLS_NORMAL_WORK_MODE            (0)
#define AW88266A_TX_FLS_NORMAL_WORK_MODE_VALUE      \
  (AW88266A_TX_FLS_NORMAL_WORK_MODE << AW88266A_TX_FLS_START_BIT)
#define AW88266A_TX_FLS_FLUSH_FIFO                  (1)
#define AW88266A_TX_FLS_FLUSH_FIFO_VALUE            \
  (AW88266A_TX_FLS_FLUSH_FIFO << AW88266A_TX_FLS_START_BIT)

#define AW88266A_TX_FLS_DEFAULT_VALUE               (0)
#define AW88266A_TX_FLS_DEFAULT                     \
  (AW88266A_TX_FLS_DEFAULT_VALUE << AW88266A_TX_FLS_START_BIT)

/* TX_THRS bit 9:8 (I2SCFG2 0x08) */

#define AW88266A_TX_THRS_START_BIT                  (8)
#define AW88266A_TX_THRS_BITS_LEN                   (2)
#define AW88266A_TX_THRS_MASK                       \
  (~(((1 << AW88266A_TX_THRS_BITS_LEN) - 1) << AW88266A_TX_THRS_START_BIT))

#define AW88266A_TX_THRS_DEFAULT_VALUE              (1)
#define AW88266A_TX_THRS_DEFAULT                    \
  (AW88266A_TX_THRS_DEFAULT_VALUE << AW88266A_TX_THRS_START_BIT)

/* VDSEL bit 3 (I2SCFG2 0x08) */

#define AW88266A_VDSEL_START_BIT                    (3)
#define AW88266A_VDSEL_BITS_LEN                     (1)
#define AW88266A_VDSEL_MASK                         \
  (~(((1 << AW88266A_VDSEL_BITS_LEN) - 1) << AW88266A_VDSEL_START_BIT))

#define AW88266A_VDSEL_DAC_DATA                     (0)
#define AW88266A_VDSEL_DAC_DATA_VALUE               \
  (AW88266A_VDSEL_DAC_DATA << AW88266A_VDSEL_START_BIT)
#define AW88266A_VDSEL_VSENSE_DATA                  (1)
#define AW88266A_VDSEL_VSENSE_DATA_VALUE            \
  (AW88266A_VDSEL_VSENSE_DATA << AW88266A_VDSEL_START_BIT)

#define AW88266A_VDSEL_DEFAULT_VALUE                (0)
#define AW88266A_VDSEL_DEFAULT                      \
  (AW88266A_VDSEL_DEFAULT_VALUE << AW88266A_VDSEL_START_BIT)

/* IV2CH bit 2 (I2SCFG2 0x08) */

#define AW88266A_IV2CH_START_BIT                    (2)
#define AW88266A_IV2CH_BITS_LEN                     (1)
#define AW88266A_IV2CH_MASK                         \
  (~(((1 << AW88266A_IV2CH_BITS_LEN) - 1) << AW88266A_IV2CH_START_BIT))

#define AW88266A_IV2CH_LEGACY_MODE                  (0)
#define AW88266A_IV2CH_LEGACY_MODE_VALUE            \
  (AW88266A_IV2CH_LEGACY_MODE << AW88266A_IV2CH_START_BIT)
#define AW88266A_IV2CH_16BITS_PER_CHANNEL           (1)
#define AW88266A_IV2CH_16BITS_PER_CHANNEL_VALUE     \
  (AW88266A_IV2CH_16BITS_PER_CHANNEL << AW88266A_IV2CH_START_BIT)

#define AW88266A_IV2CH_DEFAULT_VALUE                (0)
#define AW88266A_IV2CH_DEFAULT                      \
  (AW88266A_IV2CH_DEFAULT_VALUE << AW88266A_IV2CH_START_BIT)

/* default value of I2SCFG2 (0x08)
 * #define  AW88266A_I2SCFG2_DEFAULT (0x2100)
 * HAGCCFG4 (0x0C) detail
 * VOL bit 15:8 (HAGCCFG4 0x0C)
 */

#define AW88266A_VOL_START_BIT                      (0)
#define AW88266A_VOL_BITS_LEN                       (10)
#define AW88266A_VOL_MASK                           \
  (~(((1 << AW88266A_VOL_BITS_LEN) - 1) << AW88266A_VOL_START_BIT))

#define AW88266A_VOLUME_MAX                         (0)
#define AW88266A_VOLUME_MIN                         (-255)

#define AW88266A_VOL_DEFAULT_VALUE                  (0)
#define AW88266A_VOL_DEFAULT                        \
  (AW88266A_VOL_DEFAULT_VALUE << AW88266A_VOL_START_BIT)

/* HOLDTH bit 7:0 (HAGCCFG4 0x0C) */

#define AW88266A_HOLDTH_START_BIT                   (0)
#define AW88266A_HOLDTH_BITS_LEN                    (8)
#define AW88266A_HOLDTH_MASK                        \
  (~(((1 << AW88266A_HOLDTH_BITS_LEN) - 1) << AW88266A_HOLDTH_START_BIT))

#define AW88266A_HOLDTH_RESERVED                    (0)
#define AW88266A_HOLDTH_RESERVED_VALUE              \
  (AW88266A_HOLDTH_RESERVED << AW88266A_HOLDTH_START_BIT)

#define AW88266A_HOLDTH_DEFAULT_VALUE               (0x64)
#define AW88266A_HOLDTH_DEFAULT                     \
  (AW88266A_HOLDTH_DEFAULT_VALUE << AW88266A_HOLDTH_START_BIT)

/* default value of HAGCCFG4 (0x0C)
 * #define  AW88266A_HAGCCFG4_DEFAULT (0x0064)
 * #define  AW88266A_TESTDET_DEFAULT (0x0000)
 */

#define AW88266A_EF_VSN_GESLP_MASK                  (0x03ff)
#define AW88266A_EF_VSN_GESLP_SIGN_MASK             (0x0200)
#define AW88266A_EF_VSN_GESLP_NEG                   (0xfc00)

#define AW88266A_EF_ISN_GESLP_MASK                  (0x03ff)
#define AW88266A_EF_ISN_GESLP_SIGN_MASK             (0x0200)
#define AW88266A_EF_ISN_GESLP_NEG                   (0xfc00)

#define AW88266A_CABL_BASE_VALUE                    (1000)
#define AW88266A_ICABLK_FACTOR                      (1)
#define AW88266A_VCABLK_FACTOR                      (1)

#define AW88266A_VCAL_FACTOR                        (1<<13)
#endif