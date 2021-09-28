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
#define AW88266A_I2SCTRL1_REG                       (0x06)
#define AW88266A_I2SCTRL2_REG                       (0x07)
#define AW88266A_DACCFG1_REG                        (0x08)
#define AW88266A_DACCFG2_REG                        (0x09)
#define AW88266A_DACCFG3_REG                        (0x0A)
#define AW88266A_DACCFG4_REG                        (0x0B)
#define AW88266A_DACCFG5_REG                        (0x0C)
#define AW88266A_DACCFG6_REG                        (0x0D)
#define AW88266A_DACCFG7_REG                        (0x0E)
#define AW88266A_PWMCTRL_REG                        (0x10)
#define AW88266A_I2SCFG1_REG                        (0x11)
#define AW88266A_DBGCTRL_REG                        (0x12)
#define AW88266A_DACST_REG                          (0x20)
#define AW88266A_VBAT_REG                           (0x21)
#define AW88266A_TEMP_REG                           (0x22)
#define AW88266A_PVDD_REG                           (0x23)
#define AW88266A_ISNDAT_REG                         (0x24)
#define AW88266A_VSNDAT_REG                         (0x25)
#define AW88266A_I2SINT_REG                         (0x26)
#define AW88266A_I2SCAPCNT_REG                      (0x27)
#define AW88266A_ANASTA1_REG                        (0x28)
#define AW88266A_ANASTA2_REG                        (0x29)
#define AW88266A_ANASTA3_REG                        (0x2A)
#define AW88266A_TESTDET_REG                        (0x2B)
#define AW88266A_TESTIN_REG                         (0x38)
#define AW88266A_TESTOUT_REG                        (0x39)
#define AW88266A_VSNTM1_REG                         (0x50)
#define AW88266A_VSNTM2_REG                         (0x51)
#define AW88266A_ISNCTRL1_REG                       (0x52)
#define AW88266A_PLLCTRL1_REG                       (0x53)
#define AW88266A_PLLCTRL2_REG                       (0x54)
#define AW88266A_PLLCTRL3_REG                       (0x55)
#define AW88266A_CDACTRL1_REG                       (0x56)
#define AW88266A_CDACTRL2_REG                       (0x57)
#define AW88266A_SADCCTRL1_REG                      (0x58)
#define AW88266A_BSTCTRL1_REG                       (0x60)
#define AW88266A_BSTCTRL2_REG                       (0x61)
#define AW88266A_BSTCTRL3_REG                       (0x62)
#define AW88266A_BSTCTRL4_REG                       (0x63)
#define AW88266A_BSTCTRL5_REG                       (0x64)
#define AW88266A_BSTCTRL6_REG                       (0x65)
#define AW88266A_DSMCFG1_REG                        (0x66)
#define AW88266A_DSMCFG2_REG                        (0x67)
#define AW88266A_DSMCFG3_REG                        (0x68)
#define AW88266A_DSMCFG4_REG                        (0x69)
#define AW88266A_DSMCFG5_REG                        (0x6A)
#define AW88266A_DSMCFG6_REG                        (0x6B)
#define AW88266A_DSMCFG7_REG                        (0x6C)
#define AW88266A_DSMCFG8_REG                        (0x6D)
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
#define AW88266A_TM_REG                             (0x7C)

#define AW88266A_REG_MAX                            (0x7D)

#define AW88266A_REG_NONE_ACCESS                    (0)
#define AW88266A_REG_RD_ACCESS                      (1 << 0)
#define AW88266A_REG_WR_ACCESS                      (1 << 1)
#define AW88266A_GAIN_MAX                           (767)

/* detail information of registers begin
 * ID (0x00) detail
 * IDCODE bit 15:0 (ID 0x00)
 */

#define AW88266A_IDCODE_START_BIT                   (0)
#define AW88266A_IDCODE_BITS_LEN                    (16)
#define AW88266A_IDCODE_MASK                        \
  (~(((1 << AW88266A_IDCODE_BITS_LEN) - 1 ) << AW88266A_IDCODE_START_BIT))

#define AW88266A_IDCODE_DEFAULT_VALUE               (0x2013)
#define AW88266A_IDCODE_DEFAULT                     \
  (AW88266A_IDCODE_DEFAULT_VALUE << AW88266A_IDCODE_START_BIT)

/* default value of ID (0x00)
 * #define AW88266A_ID_DEFAULT (0x2013)
 */

/* SYSST (0x01) detail
 * OVP2S bit 15 (SYSST 0x01)
 */

#define AW88266A_OVP2S_START_BIT                    (15)
#define AW88266A_OVP2S_BITS_LEN                     (1)
#define AW88266A_OVP2S_MASK                         \
  (~(((1 << AW88266A_OVP2S_BITS_LEN) - 1) << AW88266A_OVP2S_START_BIT))

#define AW88266A_OVP2S_NORMAL                       (0)
#define AW88266A_OVP2S_NORMAL_VALUE                 \
  (AW88266A_OVP2S_NORMAL << AW88266A_OVP2S_START_BIT)

#define AW88266A_OVP2S_OVP                          (1)
#define AW88266A_OVP2S_OVP_VALUE                    \
  (AW88266A_OVP2S_OVP << AW88266A_OVP2S_START_BIT)

#define AW88266A_OVP2S_DEFAULT_VALUE                (0)
#define AW88266A_OVP2S_DEFAULT                      \
  (AW88266A_OVP2S_DEFAULT_VALUE << AW88266A_OVP2S_START_BIT)

/* UVLS bit 14 (SYSST 0x01) */

#define AW88266A_UVLS_START_BIT                     (14)
#define AW88266A_UVLS_BITS_LEN                      (1)
#define AW88266A_UVLS_MASK                          \
  (~(((1 << AW88266A_UVLS_BITS_LEN) - 1) << AW88266A_UVLS_START_BIT))

#define AW88266A_UVLS_NORMAL                        (0)
#define AW88266A_UVLS_NORMAL_VALUE                  \
  (AW88266A_UVLS_NORMAL << AW88266A_UVLS_START_BIT)

#define AW88266A_UVLS_UVLO                          (1)
#define AW88266A_UVLS_UVLO_VALUE                    \
  (AW88266A_UVLS_UVLO << AW88266A_UVLS_START_BIT)

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

#define AW88266A_BSTOCS_NORMAL                      (0)
#define AW88266A_BSTOCS_NORMAL_VALUE                \
  (AW88266A_BSTOCS_NORMAL << AW88266A_BSTOCS_START_BIT)

#define AW88266A_BSTOCS_OVER_CURRENT                (1)
#define AW88266A_BSTOCS_OVER_CURRENT_VALUE          \
  (AW88266A_BSTOCS_OVER_CURRENT << AW88266A_BSTOCS_START_BIT)

#define AW88266A_BSTOCS_DEFAULT_VALUE               (0)
#define AW88266A_BSTOCS_DEFAULT                     \
  (AW88266A_BSTOCS_DEFAULT_VALUE << AW88266A_BSTOCS_START_BIT)

/* OVPS bit 10 (SYSST 0x01) */

#define AW88266A_OVPS_START_BIT                     (10)
#define AW88266A_OVPS_BITS_LEN                      (1)
#define AW88266A_OVPS_MASK                          \
  (~(((1 << AW88266A_OVPS_BITS_LEN) - 1) << AW88266A_OVPS_START_BIT))

#define AW88266A_OVPS_NORMAL                        (0)
#define AW88266A_OVPS_NORMAL_VALUE                  \
  (AW88266A_OVPS_NORMAL << AW88266A_OVPS_START_BIT)

#define AW88266A_OVPS_OVP                           (1)
#define AW88266A_OVPS_OVP_VALUE                     \
  (AW88266A_OVPS_OVP << AW88266A_OVPS_START_BIT)

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

/* NOCLKS bit 5 (SYSST 0x01) */

#define AW88266A_NOCLKS_START_BIT                   (5)
#define AW88266A_NOCLKS_BITS_LEN                    (1)
#define AW88266A_NOCLKS_MASK                        \
  (~(((1 << AW88266A_NOCLKS_BITS_LEN) - 1) << AW88266A_NOCLKS_START_BIT))

#define AW88266A_NOCLKS_CLOCK_OK                    (0)
#define AW88266A_NOCLKS_CLOCK_OK_VALUE              \
  (AW88266A_NOCLKS_CLOCK_OK << AW88266A_NOCLKS_START_BIT)

#define AW88266A_NOCLKS_NO_CLOCK                    (1)
#define AW88266A_NOCLKS_NO_CLOCK_VALUE              \
  (AW88266A_NOCLKS_NO_CLOCK << AW88266A_NOCLKS_START_BIT)

#define AW88266A_NOCLKS_DEFAULT_VALUE               (0)
#define AW88266A_NOCLKS_DEFAULT                     \
  (AW88266A_NOCLKS_DEFAULT_VALUE << AW88266A_NOCLKS_START_BIT)

/* CLKS bit 4 (SYSST 0x01) */

#define AW88266A_CLKS_START_BIT                     (4)
#define AW88266A_CLKS_BITS_LEN                      (1)
#define AW88266A_CLKS_MASK                          \
  (~(((1 << AW88266A_CLKS_BITS_LEN) - 1) << AW88266A_CLKS_START_BIT))

#define AW88266A_CLKS_NOT_STABLE                    (0)
#define AW88266A_CLKS_NOT_STABLE_VALUE              \
  (AW88266A_CLKS_NOT_STABLE << AW88266A_CLKS_START_BIT)

#define AW88266A_CLKS_STABLE                        (1)
#define AW88266A_CLKS_STABLE_VALUE                  \
  (AW88266A_CLKS_STABLE << AW88266A_CLKS_START_BIT)

#define AW88266A_CLKS_DEFAULT_VALUE                 (0)
#define AW88266A_CLKS_DEFAULT                       \
  (AW88266A_CLKS_DEFAULT_VALUE << AW88266A_CLKS_START_BIT)

/* OCDS bit 3 (SYSST 0x01) */

#define AW88266A_OCDS_START_BIT                     (3)
#define AW88266A_OCDS_BITS_LEN                      (1)
#define AW88266A_OCDS_MASK                          \
  (~(((1 << AW88266A_OCDS_BITS_LEN) - 1) << AW88266A_OCDS_START_BIT))

#define AW88266A_OCDS_NORAML                        (0)
#define AW88266A_OCDS_NORAML_VALUE                  \
  (AW88266A_OCDS_NORAML << AW88266A_OCDS_START_BIT)

#define AW88266A_OCDS_OC                            (1)
#define AW88266A_OCDS_OC_VALUE                      \
  (AW88266A_OCDS_OC << AW88266A_OCDS_START_BIT)

#define AW88266A_OCDS_DEFAULT_VALUE                 (0)
#define AW88266A_OCDS_DEFAULT                       \
  (AW88266A_OCDS_DEFAULT_VALUE << AW88266A_OCDS_START_BIT)

/* UVL_DVDDS bit 2 (SYSST 0x01) */

#define AW88266A_UVL_DVDDS_START_BIT                (2)
#define AW88266A_UVL_DVDDS_BITS_LEN                 (1)
#define AW88266A_UVL_DVDDS_MASK                     \
  (~(((1 << AW88266A_UVL_DVDDS_BITS_LEN) - 1) <<    \
  AW88266A_UVL_DVDDS_START_BIT))

#define AW88266A_UVL_DVDDS_NORMAL                   (0)
#define AW88266A_UVL_DVDDS_NORMAL_VALUE             \
  (AW88266A_UVL_DVDDS_NORMAL << AW88266A_UVL_DVDDS_START_BIT)

#define AW88266A_UVL_DVDDS_UVLO                     (1)
#define AW88266A_UVL_DVDDS_UVLO_VALUE               \
  (AW88266A_UVL_DVDDS_UVLO << AW88266A_UVL_DVDDS_START_BIT)

#define AW88266A_UVL_DVDDS_DEFAULT_VALUE            (0)
#define AW88266A_UVL_DVDDS_DEFAULT                  \
  (AW88266A_UVL_DVDDS_DEFAULT_VALUE << AW88266A_UVL_DVDDS_START_BIT)

/* OTHS bit 1 (SYSST 0x01) */

#define AW88266A_OTHS_START_BIT                     (1)
#define AW88266A_OTHS_BITS_LEN                      (1)
#define AW88266A_OTHS_MASK                          \
  (~(((1 << AW88266A_OTHS_BITS_LEN) - 1) << AW88266A_OTHS_START_BIT))

#define AW88266A_OTHS_NORMAL                        (0)
#define AW88266A_OTHS_NORMAL_VALUE                  \
  (AW88266A_OTHS_NORMAL << AW88266A_OTHS_START_BIT)

#define AW88266A_OTHS_OT                            (1)
#define AW88266A_OTHS_OT_VALUE                      \
  (AW88266A_OTHS_OT << AW88266A_OTHS_START_BIT)

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
  (~(AW88266A_UVLS_UVLO_VALUE |                     \
  AW88266A_BSTOCS_OVER_CURRENT_VALUE |              \
  AW88266A_BSTS_FINISHED_VALUE |                    \
  AW88266A_SWS_SWITCHING_VALUE |                    \
  AW88266A_NOCLKS_NO_CLOCK_VALUE |                  \
  AW88266A_CLKS_STABLE_VALUE |                      \
  AW88266A_OCDS_OC_VALUE |                          \
  AW88266A_OTHS_OT_VALUE |                          \
  AW88266A_PLLS_LOCKED_VALUE))

#define AW88266A_SYSST_CHECK                        \
  (AW88266A_BSTS_FINISHED_VALUE |                   \
  AW88266A_SWS_SWITCHING_VALUE |                    \
  AW88266A_CLKS_STABLE_VALUE |                      \
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

/* UVL_DVDDI bit 2 (SYSINT 0x02) */

#define AW88266A_UVL_DVDDI_START_BIT                (2)
#define AW88266A_UVL_DVDDI_BITS_LEN                 (1)
#define AW88266A_UVL_DVDDI_MASK                     \
  (~((( 1<< AW88266A_UVL_DVDDI_BITS_LEN) - 1) <<    \
  AW88266A_UVL_DVDDI_START_BIT))

#define AW88266A_UVL_DVDDI_DEFAULT_VALUE            (0)
#define AW88266A_UVL_DVDDI_DEFAULT                  \
  (AW88266A_UVL_DVDDI_DEFAULT_VALUE << AW88266A_UVL_DVDDI_START_BIT)

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

/* DSPM bit 12 (SYSINTM 0x03) */

#define AW88266A_DSPM_START_BIT                     (12)
#define AW88266A_DSPM_BITS_LEN                      (1)
#define AW88266A_DSPM_MASK                          \
  (~(((1 << AW88266A_DSPM_BITS_LEN) - 1) << AW88266A_DSPM_START_BIT))

#define AW88266A_DSPM_DEFAULT_VALUE                 (1)
#define AW88266A_DSPM_DEFAULT                       \
  (AW88266A_DSPM_DEFAULT_VALUE << AW88266A_DSPM_START_BIT)

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

/* WDM bit 6 (SYSINTM 0x03) */

#define AW88266A_WDM_START_BIT                      (6)
#define AW88266A_WDM_BITS_LEN                       (1)
#define AW88266A_WDM_MASK                           \
  (~(((1 << AW88266A_WDM_BITS_LEN) - 1) << AW88266A_WDM_START_BIT))

#define AW88266A_WDM_DEFAULT_VALUE                  (1)
#define AW88266A_WDM_DEFAULT                        \
  (AW88266A_WDM_DEFAULT_VALUE << AW88266A_WDM_START_BIT)

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

/* UVL_DVDDM bit 2 (SYSINTM 0x03) */

#define AW88266A_UVL_DVDDM_START_BIT                (2)
#define AW88266A_UVL_DVDDM_BITS_LEN                 (1)
#define AW88266A_UVL_DVDDM_MASK                     \
  (~(((1 << AW88266A_UVL_DVDDM_BITS_LEN) - 1) <<    \
  AW88266A_UVL_DVDDM_START_BIT))

#define AW88266A_UVL_DVDDM_DEFAULT_VALUE            (1)
#define AW88266A_UVL_DVDDM_DEFAULT                  \
  (AW88266A_UVL_DVDDM_DEFAULT_VALUE << AW88266A_UVL_DVDDM_START_BIT)

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

/* default value of SYSINTM (0x03) */

#define AW88266A_SYSINTM_DEFAULT                    (0xFFFF)

/* SYSCTRL (0x04) detail
 * DRVSTREN bit 15 (SYSCTRL 0x04)
 */

#define AW88266A_DRVSTREN_START_BIT                 (15)
#define AW88266A_DRVSTREN_BITS_LEN                  (1)
#define AW88266A_DRVSTREN_MASK                      \
  (~(((1 << AW88266A_DRVSTREN_BITS_LEN) - 1) << AW88266A_DRVSTREN_START_BIT))

#define AW88266A_DRVSTREN_4MA                       (0)
#define AW88266A_DRVSTREN_4MA_VALUE                 \
  (AW88266A_DRVSTREN_4MA << AW88266A_DRVSTREN_START_BIT)

#define AW88266A_DRVSTREN_12MA                      (1)
#define AW88266A_DRVSTREN_12MA_VALUE                \
  (AW88266A_DRVSTREN_12MA << AW88266A_DRVSTREN_START_BIT)

#define AW88266A_DRVSTREN_DEFAULT_VALUE             (1)
#define AW88266A_DRVSTREN_DEFAULT                   \
  (AW88266A_DRVSTREN_DEFAULT_VALUE << AW88266A_DRVSTREN_START_BIT)

/* SET_GAIN bit 14:12 (SYSCTRL 0x04) */

#define AW88266A_SET_GAIN_START_BIT                 (12)
#define AW88266A_SET_GAIN_BITS_LEN                  (3)
#define AW88266A_SET_GAIN_MASK                      \
  (~(((1 << AW88266A_SET_GAIN_BITS_LEN) - 1) << AW88266A_SET_GAIN_START_BIT))

#define AW88266A_SET_GAIN_4P5_AV                    (0)
#define AW88266A_SET_GAIN_4P5_AV_VALUE              \
  (AW88266A_SET_GAIN_4P5_AV << AW88266A_SET_GAIN_START_BIT)

#define AW88266A_SET_GAIN_5P0_AV                    (1)
#define AW88266A_SET_GAIN_5P0_AV_VALUE              \
  (AW88266A_SET_GAIN_5P0_AV << AW88266A_SET_GAIN_START_BIT)

#define AW88266A_SET_GAIN_6P0_AV                    (2)
#define AW88266A_SET_GAIN_6P0_AV_VALUE              \
  (AW88266A_SET_GAIN_6P0_AV << AW88266A_SET_GAIN_START_BIT)

#define AW88266A_SET_GAIN_6P7_AV                    (3)
#define AW88266A_SET_GAIN_6P7_AV_VALUE              \
  (AW88266A_SET_GAIN_6P7_AV << AW88266A_SET_GAIN_START_BIT)

#define AW88266A_SET_GAIN_10_AV                     (4)
#define AW88266A_SET_GAIN_10_AV_VALUE               \
  (AW88266A_SET_GAIN_10_AV << AW88266A_SET_GAIN_START_BIT)

#define AW88266A_SET_GAIN_12_AV                     (5)
#define AW88266A_SET_GAIN_12_AV_VALUE               \
  (AW88266A_SET_GAIN_12_AV << AW88266A_SET_GAIN_START_BIT)

#define AW88266A_SET_GAIN_DEFAULT_VALUE             (0x5)
#define AW88266A_SET_GAIN_DEFAULT                   \
  (AW88266A_SET_GAIN_DEFAULT_VALUE << AW88266A_SET_GAIN_START_BIT)

/* RMSE bit 11 (SYSCTRL 0x04) */

#define AW88266A_RMSE_START_BIT                     (11)
#define AW88266A_RMSE_BITS_LEN                      (1)
#define AW88266A_RMSE_MASK                          \
  (~(((1 << AW88266A_RMSE_BITS_LEN) - 1) << AW88266A_RMSE_START_BIT))

#define AW88266A_RMSE_PEAK_AGC                      (0)
#define AW88266A_RMSE_PEAK_AGC_VALUE                \
  (AW88266A_RMSE_PEAK_AGC << AW88266A_RMSE_START_BIT)

#define AW88266A_RMSE_RMS_AGC                       (1)
#define AW88266A_RMSE_RMS_AGC_VALUE                 \
  (AW88266A_RMSE_RMS_AGC << AW88266A_RMSE_START_BIT)

#define AW88266A_RMSE_DEFAULT_VALUE                 (0)
#define AW88266A_RMSE_DEFAULT                       \
  (AW88266A_RMSE_DEFAULT_VALUE << AW88266A_RMSE_START_BIT)

/* HAGCE bit 10 (SYSCTRL 0x04) */

#define AW88266A_HAGCE_START_BIT                    (10)
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

/* HDCCE bit 9 (SYSCTRL 0x04) */

#define AW88266A_HDCCE_START_BIT                    (9)
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

/* HMUTE bit 8 (SYSCTRL 0x04) */

#define AW88266A_HMUTE_START_BIT                    (8)
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

/* EN_TRAN bit 7 (SYSCTRL 0x04) */

#define AW88266A_EN_TRAN_START_BIT                  (7)
#define AW88266A_EN_TRAN_BITS_LEN                   (1)
#define AW88266A_EN_TRAN_MASK                       \
  (~(((1 << AW88266A_EN_TRAN_BITS_LEN) - 1) << AW88266A_EN_TRAN_START_BIT))

#define AW88266A_EN_TRAN_SPK                        (0)
#define AW88266A_EN_TRAN_SPK_VALUE                  \
  (AW88266A_EN_TRAN_SPK << AW88266A_EN_TRAN_START_BIT)

#define AW88266A_EN_TRAN_RCV                        (1)
#define AW88266A_EN_TRAN_RCV_VALUE                  \
  (AW88266A_EN_TRAN_RCV << AW88266A_EN_TRAN_START_BIT)

#define AW88266A_EN_TRAN_DEFAULT_VALUE              (0)
#define AW88266A_EN_TRAN_DEFAULT                    \
  (AW88266A_EN_TRAN_DEFAULT_VALUE << AW88266A_EN_TRAN_START_BIT)

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

#define AW88266A_WSINV_NOT_SWITCH                   (0)
#define AW88266A_WSINV_NOT_SWITCH_VALUE             \
  (AW88266A_WSINV_NOT_SWITCH << AW88266A_WSINV_START_BIT)

#define AW88266A_WSINV_SWITCH                       (1)
#define AW88266A_WSINV_SWITCH_VALUE                 \
  (AW88266A_WSINV_SWITCH << AW88266A_WSINV_START_BIT)

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
  (AW88266A_IPLL_WCK << AW88266A_IPLL_START_BIT)

#define AW88266A_IPLL_DEFAULT_VALUE                 (0)
#define AW88266A_IPLL_DEFAULT                       \
  (AW88266A_IPLL_DEFAULT_VALUE << AW88266A_IPLL_START_BIT)

/* AMPPD bit 1 (SYSCTRL 0x04) */

#define AW88266A_AMPPD_START_BIT                    (1)
#define AW88266A_AMPPD_BITS_LEN                     (1)
#define AW88266A_AMPPD_MASK                         \
  (~(((1 << AW88266A_AMPPD_BITS_LEN) - 1) << AW88266A_AMPPD_START_BIT))

#define AW88266A_AMPPD_WORKING                      (0)
#define AW88266A_AMPPD_WORKING_VALUE                \
  (AW88266A_AMPPD_WORKING << AW88266A_AMPPD_START_BIT)

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

#define AW88266A_PWDN_WORKING                       (0)
#define AW88266A_PWDN_WORKING_VALUE                 \
  (AW88266A_PWDN_WORKING << AW88266A_PWDN_START_BIT)

#define AW88266A_PWDN_POWER_DOWN                    (1)
#define AW88266A_PWDN_POWER_DOWN_VALUE              \
  (AW88266A_PWDN_POWER_DOWN << AW88266A_PWDN_START_BIT)

#define AW88266A_PWDN_DEFAULT_VALUE                 (1)
#define AW88266A_PWDN_DEFAULT                       \
  (AW88266A_PWDN_DEFAULT_VALUE << AW88266A_PWDN_START_BIT)

/* default value of SYSCTRL (0x04)
 * #define AW88266A_SYSCTRL_DEFAULT (0xD303)
 */

/* SYSCTRL2 (0x05) detail
 * IV2CH bit 15 (SYSCTRL2 0x05)
 */

#define AW88266A_IV2CH_START_BIT                    (15)
#define AW88266A_IV2CH_BITS_LEN                     (1)
#define AW88266A_IV2CH_MASK                         \
  (~(((1 << AW88266A_IV2CH_BITS_LEN) - 1) << AW88266A_IV2CH_START_BIT))

#define AW88266A_IV2CH_LEGACY                       (0)
#define AW88266A_IV2CH_LEGACY_VALUE                 \
  (AW88266A_IV2CH_LEGACY << AW88266A_IV2CH_START_BIT)

#define AW88266A_IV2CH_SPECIAL                      (1)
#define AW88266A_IV2CH_SPECIAL_VALUE                \
  (AW88266A_IV2CH_SPECIAL << AW88266A_IV2CH_START_BIT)

#define AW88266A_IV2CH_DEFAULT_VALUE                (0)
#define AW88266A_IV2CH_DEFAULT                      \
  (AW88266A_IV2CH_DEFAULT_VALUE << AW88266A_IV2CH_START_BIT)

/* I2SDOSEL bit 14 (SYSCTRL2 0x05) */

#define AW88266A_I2SDOSEL_START_BIT                 (14)
#define AW88266A_I2SDOSEL_BITS_LEN                  (1)
#define AW88266A_I2SDOSEL_MASK                      \
  (~(((1 << AW88266A_I2SDOSEL_BITS_LEN) - 1) << AW88266A_I2SDOSEL_START_BIT))

#define AW88266A_I2SDOSEL_ZEROS                     (0)
#define AW88266A_I2SDOSEL_ZEROS_VALUE               \
  (AW88266A_I2SDOSEL_ZEROS << AW88266A_I2SDOSEL_START_BIT)

#define AW88266A_I2SDOSEL_TXDATA                    (1)
#define AW88266A_I2SDOSEL_TXDATA_VALUE              \
  (AW88266A_I2SDOSEL_TXDATA << AW88266A_I2SDOSEL_START_BIT)

#define AW88266A_I2SDOSEL_DEFAULT_VALUE             (1)
#define AW88266A_I2SDOSEL_DEFAULT                   \
  (AW88266A_I2SDOSEL_DEFAULT_VALUE << AW88266A_I2SDOSEL_START_BIT)

/* DOHZ bit 13 (SYSCTRL2 0x05) */

#define AW88266A_DOHZ_START_BIT                     (13)
#define AW88266A_DOHZ_BITS_LEN                      (1)
#define AW88266A_DOHZ_MASK                          \
  (~(((1 << AW88266A_DOHZ_BITS_LEN) - 1) << AW88266A_DOHZ_START_BIT))

#define AW88266A_DOHZ_ALL                           (0)
#define AW88266A_DOHZ_ALL_VALUE                     \
  (AW88266A_DOHZ_ALL << AW88266A_DOHZ_START_BIT)

#define AW88266A_DOHZ_HIZ                           (1)
#define AW88266A_DOHZ_HIZ_VALUE                     \
  (AW88266A_DOHZ_HIZ << AW88266A_DOHZ_START_BIT)

#define AW88266A_DOHZ_DEFAULT_VALUE                 (1)
#define AW88266A_DOHZ_DEFAULT                       \
  (AW88266A_DOHZ_DEFAULT_VALUE << AW88266A_DOHZ_START_BIT)

/* I2SCHS bit 12 (SYSCTRL2 0x05) */

#define AW88266A_I2SCHS_START_BIT                   (12)
#define AW88266A_I2SCHS_BITS_LEN                    (1)
#define AW88266A_I2SCHS_MASK                        \
  (~(((1 << AW88266A_I2SCHS_BITS_LEN) - 1) << AW88266A_I2SCHS_START_BIT))

#define AW88266A_I2SCHS_LEFT                        (0)
#define AW88266A_I2SCHS_LEFT_VALUE                  \
  (AW88266A_I2SCHS_LEFT << AW88266A_I2SCHS_START_BIT)

#define AW88266A_I2SCHS_RIGHT                       (1)
#define AW88266A_I2SCHS_RIGHT_VALUE                 \
  (AW88266A_I2SCHS_RIGHT << AW88266A_I2SCHS_START_BIT)

#define AW88266A_I2SCHS_DEFAULT_VALUE               (0)
#define AW88266A_I2SCHS_DEFAULT                     \
  (AW88266A_I2SCHS_DEFAULT_VALUE << AW88266A_I2SCHS_START_BIT)

/* INTMODE bit 11 (SYSCTRL2 0x05) */

#define AW88266A_INTMODE_START_BIT                  (11)
#define AW88266A_INTMODE_BITS_LEN                   (1)
#define AW88266A_INTMODE_MASK                       \
  (~(((1 << AW88266A_INTMODE_BITS_LEN) - 1) << AW88266A_INTMODE_START_BIT))

#define AW88266A_INTMODE_OPENMINUS_DRAIN            (0)
#define AW88266A_INTMODE_OPENMINUS_DRAIN_VALUE      \
  (AW88266A_INTMODE_OPENMINUS_DRAIN << AW88266A_INTMODE_START_BIT)

#define AW88266A_INTMODE_PUSHPULL                   (1)
#define AW88266A_INTMODE_PUSHPULL_VALUE             \
  (AW88266A_INTMODE_PUSHPULL << AW88266A_INTMODE_START_BIT)

#define AW88266A_INTMODE_DEFAULT_VALUE              (0)
#define AW88266A_INTMODE_DEFAULT                    \
  (AW88266A_INTMODE_DEFAULT_VALUE << AW88266A_INTMODE_START_BIT)

/* INTN bit 10 (SYSCTRL2 0x05) */

#define AW88266A_INTN_START_BIT                     (10)
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

/* VOL bit 9:0 (SYSCTRL2 0x05) */

#define AW88266A_VOL_START_BIT                      (0)
#define AW88266A_VOL_BITS_LEN                       (10)
#define AW88266A_VOL_MASK                           \
  (~(((1 << AW88266A_VOL_BITS_LEN) - 1) << AW88266A_VOL_START_BIT))

#define AW88266A_MUTE_VOL                           (90 * 8)
#define AW88266A_VOL_STEP_DB                        (6 * 8)

#define AW88266A_VOL_DEFAULT_VALUE                  (0)
#define AW88266A_VOL_DEFAULT                        \
  (AW88266A_VOL_DEFAULT_VALUE << AW88266A_VOL_START_BIT)

/* default value of SYSCTRL2 (0x05)
 * #define AW88266A_SYSCTRL2_DEFAULT  (0x6000)
 */

/* I2SCTRL1 (0x06) detail
 * I2SRXEN bit 15 (I2SCTRL1 0x06)
 */

#define AW88266A_I2SRXEN_START_BIT                  (15)
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

/* I2STXEN bit 14 (I2SCTRL1 0x06) */

#define AW88266A_I2STXEN_START_BIT                  (14)
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

/* CFSEL bit 13:12 (I2SCTRL1 0x06) */

#define AW88266A_CFSEL_START_BIT                    (12)
#define AW88266A_CFSEL_BITS_LEN                     (2)
#define AW88266A_CFSEL_MASK                         \
  (~(((1 << AW88266A_CFSEL_BITS_LEN) - 1) << AW88266A_CFSEL_START_BIT))

#define AW88266A_CFSEL_HAGC                         (0)
#define AW88266A_CFSEL_HAGC_VALUE                   \
  (AW88266A_CFSEL_HAGC << AW88266A_CFSEL_START_BIT)

#define AW88266A_CFSEL_IV                           (1)
#define AW88266A_CFSEL_IV_VALUE                     \
  (AW88266A_CFSEL_IV << AW88266A_CFSEL_START_BIT)

#define AW88266A_CFSEL_IVT_IPVT                     (2)
#define AW88266A_CFSEL_IVT_IPVT_VALUE               \
  (AW88266A_CFSEL_IVT_IPVT << AW88266A_CFSEL_START_BIT)

#define AW88266A_CFSEL_RESERVED                     (3)
#define AW88266A_CFSEL_RESERVED_VALUE               \
  (AW88266A_CFSEL_RESERVED << AW88266A_CFSEL_START_BIT)

#define AW88266A_CFSEL_DEFAULT_VALUE                (0)
#define AW88266A_CFSEL_DEFAULT                      \
  (AW88266A_CFSEL_DEFAULT_VALUE << AW88266A_CFSEL_START_BIT)

/* CHSEL bit 11:10 (I2SCTRL1 0x06) */

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

#define AW88266A_CHSEL_MONO                         (3)
#define AW88266A_CHSEL_MONO_VALUE                   \
  (AW88266A_CHSEL_MONO << AW88266A_CHSEL_START_BIT)

#define AW88266A_CHSEL_DEFAULT_VALUE                (1)
#define AW88266A_CHSEL_DEFAULT                      \
  (AW88266A_CHSEL_DEFAULT_VALUE << AW88266A_CHSEL_START_BIT)

/* I2SMD bit 9:8 (I2SCTRL1 0x06) */

#define AW88266A_I2SMD_START_BIT                    (8)
#define AW88266A_I2SMD_BITS_LEN                     (2)
#define AW88266A_I2SMD_MASK                         \
  (~(((1 << AW88266A_I2SMD_BITS_LEN) - 1) << AW88266A_I2SMD_START_BIT))

#define AW88266A_I2SMD_PHILIP_STANDARD              (0)
#define AW88266A_I2SMD_PHILIP_STANDARD_VALUE        \
  (AW88266A_I2SMD_PHILIP_STANDARD << AW88266A_I2SMD_START_BIT)

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

/* I2SFS bit 7:6 (I2SCTRL1 0x06) */

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

/* I2SBCK bit 5:4 (I2SCTRL1 0x06) */

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

/* I2SSR bit 3:0 (I2SCTRL1 0x06) */

#define AW88266A_I2SSR_START_BIT                    (0)
#define AW88266A_I2SSR_BITS_LEN                     (4)
#define AW88266A_I2SSR_MASK                         \
  (~(((1 << AW88266A_I2SSR_BITS_LEN) - 1) << AW88266A_I2SSR_START_BIT))

#define AW88266A_I2SSR_8_KHZ                        (0)
#define AW88266A_I2SSR_8_KHZ_VALUE                  \
  (AW88266A_I2SSR_8_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_11_KHZ                       (1)
#define AW88266A_I2SSR_11_KHZ_VALUE                 \
  (AW88266A_I2SSR_11_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_12_KHZ                       (2)
#define AW88266A_I2SSR_12_KHZ_VALUE                 \
  (AW88266A_I2SSR_12_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_16_KHZ                       (3)
#define AW88266A_I2SSR_16_KHZ_VALUE                 \
  (AW88266A_I2SSR_16_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_22_KHZ                       (4)
#define AW88266A_I2SSR_22_KHZ_VALUE                 \
  (AW88266A_I2SSR_22_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_24_KHZ                       (5)
#define AW88266A_I2SSR_24_KHZ_VALUE                 \
  (AW88266A_I2SSR_24_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_32_KHZ                       (6)
#define AW88266A_I2SSR_32_KHZ_VALUE                 \
  (AW88266A_I2SSR_32_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_44_KHZ                       (7)
#define AW88266A_I2SSR_44_KHZ_VALUE                 \
  (AW88266A_I2SSR_44_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_48_KHZ                       (8)
#define AW88266A_I2SSR_48_KHZ_VALUE                 \
  (AW88266A_I2SSR_48_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_96_KHZ                       (9)
#define AW88266A_I2SSR_96_KHZ_VALUE                 \
  (AW88266A_I2SSR_96_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_192_KHZ                      (10)
#define AW88266A_I2SSR_192_KHZ_VALUE                \
  (AW88266A_I2SSR_192_KHZ << AW88266A_I2SSR_START_BIT)

#define AW88266A_I2SSR_DEFAULT_VALUE                (8)
#define AW88266A_I2SSR_DEFAULT                      \
  (AW88266A_I2SSR_DEFAULT_VALUE<<AW88266A_I2SSR_START_BIT)

/* default value of I2SCTRL1 (0x06)
 * #define AW88266A_I2SCTRL1_DEFAULT  (0x84E8)
 */

/* I2SCTRL2 (0x07) detail
 * FSYNC_TYPE bit 15 (I2SCTRL2 0x07)
 */

#define AW88266A_FSYNC_TYPE_START_BIT               (15)
#define AW88266A_FSYNC_TYPE_BITS_LEN                (1)
#define AW88266A_FSYNC_TYPE_MASK                    \
  (~(((1 << AW88266A_FSYNC_TYPE_BITS_LEN) - 1) <<   \
  AW88266A_FSYNC_TYPE_START_BIT))

#define AW88266A_FSYNC_TYPE_ONEMINUS_SLOT           (0)
#define AW88266A_FSYNC_TYPE_ONEMINUS_SLOT_VALUE     \
  (AW88266A_FSYNC_TYPE_ONEMINUS_SLOT << AW88266A_FSYNC_TYPE_START_BIT)

#define AW88266A_FSYNC_TYPE_ONEMINUS_BCK            (1)
#define AW88266A_FSYNC_TYPE_ONEMINUS_BCK_VALUE      \
  (AW88266A_FSYNC_TYPE_ONEMINUS_BCK << AW88266A_FSYNC_TYPE_START_BIT)

#define AW88266A_FSYNC_TYPE_DEFAULT_VALUE           (0)
#define AW88266A_FSYNC_TYPE_DEFAULT                 \
  (AW88266A_FSYNC_TYPE_DEFAULT_VALUE << AW88266A_FSYNC_TYPE_START_BIT)

/* SLOT_NUM bit 14:12 (I2SCTRL2 0x07) */

#define AW88266A_SLOT_NUM_START_BIT                 (12)
#define AW88266A_SLOT_NUM_BITS_LEN                  (3)
#define AW88266A_SLOT_NUM_MASK                      \
  (~(((1 << AW88266A_SLOT_NUM_BITS_LEN) - 1) << AW88266A_SLOT_NUM_START_BIT))

#define AW88266A_SLOT_NUM_I2S_MODE                  (0)
#define AW88266A_SLOT_NUM_I2S_MODE_VALUE            \
  (AW88266A_SLOT_NUM_I2S_MODE << AW88266A_SLOT_NUM_START_BIT)

#define AW88266A_SLOT_NUM_TDM1S                     (1)
#define AW88266A_SLOT_NUM_TDM1S_VALUE               \
  (AW88266A_SLOT_NUM_TDM1S << AW88266A_SLOT_NUM_START_BIT)

#define AW88266A_SLOT_NUM_TDM2S                     (2)
#define AW88266A_SLOT_NUM_TDM2S_VALUE               \
  (AW88266A_SLOT_NUM_TDM2S << AW88266A_SLOT_NUM_START_BIT)

#define AW88266A_SLOT_NUM_TDM4S                     (3)
#define AW88266A_SLOT_NUM_TDM4S_VALUE               \
  (AW88266A_SLOT_NUM_TDM4S << AW88266A_SLOT_NUM_START_BIT)

#define AW88266A_SLOT_NUM_TDM6S                     (4)
#define AW88266A_SLOT_NUM_TDM6S_VALUE               \
  (AW88266A_SLOT_NUM_TDM6S << AW88266A_SLOT_NUM_START_BIT)

#define AW88266A_SLOT_NUM_TDM8S                     (5)
#define AW88266A_SLOT_NUM_TDM8S_VALUE               \
  (AW88266A_SLOT_NUM_TDM8S << AW88266A_SLOT_NUM_START_BIT)

#define AW88266A_SLOT_NUM_TDM16S                    (6)
#define AW88266A_SLOT_NUM_TDM16S_VALUE              \
  (AW88266A_SLOT_NUM_TDM16S << AW88266A_SLOT_NUM_START_BIT)

/* #define AW88266A_SLOT_NUM_TDM16S  (7)
 * #define AW88266A_SLOT_NUM_TDM16S_VALUE
 * (AW88266A_SLOT_NUM_TDM16S << AW88266A_SLOT_NUM_START_BIT)
 */

#define AW88266A_SLOT_NUM_DEFAULT_VALUE             (0)
#define AW88266A_SLOT_NUM_DEFAULT                   \
  (AW88266A_SLOT_NUM_DEFAULT_VALUE << AW88266A_SLOT_NUM_START_BIT)

/* I2S_TX_SLOTVLD bit 11:8 (I2SCTRL2 0x07) */

#define AW88266A_I2S_TX_SLOTVLD_START_BIT           (8)
#define AW88266A_I2S_TX_SLOTVLD_BITS_LEN            (4)
#define AW88266A_I2S_TX_SLOTVLD_MASK                \
  (~(((1 << AW88266A_I2S_TX_SLOTVLD_BITS_LEN) - 1)  \
  << AW88266A_I2S_TX_SLOTVLD_START_BIT))

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

#define AW88266A_I2S_TX_SLOTVLD_SLOT_4              (4)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_4_VALUE        \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_4 << AW88266A_I2S_TX_SLOTVLD_START_BIT)

#define AW88266A_I2S_TX_SLOTVLD_SLOT_5              (5)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_5_VALUE        \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_5 << AW88266A_I2S_TX_SLOTVLD_START_BIT)

#define AW88266A_I2S_TX_SLOTVLD_SLOT_6              (6)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_6_VALUE        \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_6 << AW88266A_I2S_TX_SLOTVLD_START_BIT)

#define AW88266A_I2S_TX_SLOTVLD_SLOT_15             (15)
#define AW88266A_I2S_TX_SLOTVLD_SLOT_15_VALUE       \
  (AW88266A_I2S_TX_SLOTVLD_SLOT_15 << AW88266A_I2S_TX_SLOTVLD_START_BIT)

#define AW88266A_I2S_TX_SLOTVLD_DEFAULT_VALUE       (0)
#define AW88266A_I2S_TX_SLOTVLD_DEFAULT             \
  (AW88266A_I2S_TX_SLOTVLD_DEFAULT_VALUE << AW88266A_I2S_TX_SLOTVLD_START_BIT)

/* I2S_RXR_SLOTVLD bit 7:4 (I2SCTRL2 0x07) */

#define AW88266A_I2S_RXR_SLOTVLD_START_BIT          (4)
#define AW88266A_I2S_RXR_SLOTVLD_BITS_LEN           (4)
#define AW88266A_I2S_RXR_SLOTVLD_MASK               \
  (~(((1 << AW88266A_I2S_RXR_SLOTVLD_BITS_LEN) - 1) \
  << AW88266A_I2S_RXR_SLOTVLD_START_BIT))

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_0             (0)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_0_VALUE       \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_0 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_1             (1)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_1_VALUE       \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_1 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_2             (2)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_2_VALUE       \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_2 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_3             (3)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_3_VALUE       \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_3 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_4             (4)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_4_VALUE       \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_4 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_5             (5)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_5_VALUE       \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_5 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_6             (6)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_6_VALUE       \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_6 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_SLOT_15            (15)
#define AW88266A_I2S_RXR_SLOTVLD_SLOT_15_VALUE      \
  (AW88266A_I2S_RXR_SLOTVLD_SLOT_15 << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXR_SLOTVLD_DEFAULT_VALUE      (1)
#define AW88266A_I2S_RXR_SLOTVLD_DEFAULT            \
  (AW88266A_I2S_RXR_SLOTVLD_DEFAULT_VALUE << AW88266A_I2S_RXR_SLOTVLD_START_BIT)

/* I2S_RXL_SLOTVLD bit 3:0 (I2SCTRL2 0x07) */

#define AW88266A_I2S_RXL_SLOTVLD_START_BIT          (0)
#define AW88266A_I2S_RXL_SLOTVLD_BITS_LEN           (4)
#define AW88266A_I2S_RXL_SLOTVLD_MASK               \
  (~(((1 << AW88266A_I2S_RXL_SLOTVLD_BITS_LEN) - 1) \
  << AW88266A_I2S_RXL_SLOTVLD_START_BIT))

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_0             (0)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_0_VALUE       \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_0 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_1             (1)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_1_VALUE       \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_1 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_2             (2)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_2_VALUE       \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_2 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_3             (3)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_3_VALUE       \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_3 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_4             (4)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_4_VALUE       \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_4 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_5             (5)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_5_VALUE       \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_5 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_6             (6)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_6_VALUE       \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_6 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_SLOT_15            (15)
#define AW88266A_I2S_RXL_SLOTVLD_SLOT_15_VALUE      \
  (AW88266A_I2S_RXL_SLOTVLD_SLOT_15 << AW88266A_I2S_RXL_SLOTVLD_START_BIT)

#define AW88266A_I2S_RXL_SLOTVLD_DEFAULT_VALUE      (0)
#define AW88266A_I2S_RXL_SLOTVLD_DEFAULT            \
  (AW88266A_I2S_RXL_SLOTVLD_DEFAULT_VALUE <<        \
  AW88266A_I2S_RXL_SLOTVLD_START_BIT)

/* default value of I2SCTRL2 (0x07)
 * #define AW88266A_I2SCTRL2_DEFAULT  (0x0010)
 */

/* DACCFG1 (0x08) detail
 * RVTH bit 15:8 (DACCFG1 0x08)
 */

#define AW88266A_RVTH_START_BIT                     (8)
#define AW88266A_RVTH_BITS_LEN                      (8)
#define AW88266A_RVTH_MASK                          \
  (~(((1 << AW88266A_RVTH_BITS_LEN) - 1) << AW88266A_RVTH_START_BIT))

#define AW88266A_RVTH_DEFAULT_VALUE                 (0x39)
#define AW88266A_RVTH_DEFAULT                       \
  (AW88266A_RVTH_DEFAULT_VALUE << AW88266A_RVTH_START_BIT)

/* AVTH bit 7:0 (DACCFG1 0x08) */

#define AW88266A_AVTH_START_BIT                     (0)
#define AW88266A_AVTH_BITS_LEN                      (8)
#define AW88266A_AVTH_MASK                          \
  (~(((1 << AW88266A_AVTH_BITS_LEN) - 1) << AW88266A_AVTH_START_BIT))

#define AW88266A_AVTH_DEFAULT_VALUE                 (0x40)
#define AW88266A_AVTH_DEFAULT                       \
  (AW88266A_AVTH_DEFAULT_VALUE << AW88266A_AVTH_START_BIT)

/* default value of DACCFG1 (0x08)
 * #define AW88266A_DACCFG1_DEFAULT (0x3940)
 */

/* DACCFG2 (0x09) detail
 * ATTH bit 15:0 (DACCFG2 0x09)
 */

#define AW88266A_ATTH_START_BIT                     (0)
#define AW88266A_ATTH_BITS_LEN                      (16)
#define AW88266A_ATTH_MASK                          \
  (~(((1 << AW88266A_ATTH_BITS_LEN) - 1) << AW88266A_ATTH_START_BIT))

#define AW88266A_ATTH_RESERVED                      (0)
#define AW88266A_ATTH_RESERVED_VALUE                \
  (AW88266A_ATTH_RESERVED << AW88266A_ATTH_START_BIT)

#define AW88266A_ATTH_DEFAULT_VALUE                 (0x0030)
#define AW88266A_ATTH_DEFAULT                       \
  (AW88266A_ATTH_DEFAULT_VALUE << AW88266A_ATTH_START_BIT)

/* default value of DACCFG2 (0x09)
 * #define AW88266A_DACCFG2_DEFAULT (0x0030)
 */

/* DACCFG3 (0x0A) detail
 * RTTH bit 15:0 (DACCFG3 0x0A)
 */

#define AW88266A_RTTH_START_BIT                     (0)
#define AW88266A_RTTH_BITS_LEN                      (16)
#define AW88266A_RTTH_MASK                          \
  (~(((1 << AW88266A_RTTH_BITS_LEN) - 1) << AW88266A_RTTH_START_BIT))

#define AW88266A_RTTH_RESERVED                      (0)
#define AW88266A_RTTH_RESERVED_VALUE                \
  (AW88266A_RTTH_RESERVED << AW88266A_RTTH_START_BIT)

#define AW88266A_RTTH_DEFAULT_VALUE                 (0x01E0)
#define AW88266A_RTTH_DEFAULT                       \
  (AW88266A_RTTH_DEFAULT_VALUE << AW88266A_RTTH_START_BIT)

/* default value of DACCFG3 (0x0A)
 * #define AW88266A_DACCFG3_DEFAULT (0x01E0)
 */

/* DACCFG4 (0x0B) detail
 * IIC_GEN_ADDR bit 15:9 (DACCFG4 0x0B)
 */

#define AW88266A_IIC_GEN_ADDR_START_BIT             (9)
#define AW88266A_IIC_GEN_ADDR_BITS_LEN              (7)
#define AW88266A_IIC_GEN_ADDR_MASK                  \
  (~(((1 << AW88266A_IIC_GEN_ADDR_BITS_LEN) - 1) << \
  AW88266A_IIC_GEN_ADDR_START_BIT))

#define AW88266A_IIC_GEN_ADDR_DEFAULT_VALUE         (0x0E)
#define AW88266A_IIC_GEN_ADDR_DEFAULT               \
  (AW88266A_IIC_GEN_ADDR_DEFAULT_VALUE << AW88266A_IIC_GEN_ADDR_START_BIT)

/* IIC_GEN_EN bit 8 (DACCFG4 0x0B) */

#define AW88266A_IIC_GEN_EN_START_BIT               (8)
#define AW88266A_IIC_GEN_EN_BITS_LEN                (1)
#define AW88266A_IIC_GEN_EN_MASK                    \
 (~(((1 << AW88266A_IIC_GEN_EN_BITS_LEN) - 1) <<    \
 AW88266A_IIC_GEN_EN_START_BIT))

#define AW88266A_IIC_GEN_EN_DISABLE                 (0)
#define AW88266A_IIC_GEN_EN_DISABLE_VALUE           \
  (AW88266A_IIC_GEN_EN_DISABLE << AW88266A_IIC_GEN_EN_START_BIT)

#define AW88266A_IIC_GEN_EN_ENABLE                  (1)
#define AW88266A_IIC_GEN_EN_ENABLE_VALUE            \
  (AW88266A_IIC_GEN_EN_ENABLE << AW88266A_IIC_GEN_EN_START_BIT)

#define AW88266A_IIC_GEN_EN_DEFAULT_VALUE           (0)
#define AW88266A_IIC_GEN_EN_DEFAULT                 \
  (AW88266A_IIC_GEN_EN_DEFAULT_VALUE << AW88266A_IIC_GEN_EN_START_BIT)

/* HOLDTH bit 7:0 (DACCFG4 0x0B) */

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

/* default value of DACCFG4 (0x0B)
 * #define AW88266A_DACCFG4_DEFAULT (0x1C64)
 */

/* DACST (0x20) detail
 * SET_GAIN_ST bit 10:8 (DACST 0x20)
 */

#define AW88266A_SET_GAIN_ST_START_BIT              (8)
#define AW88266A_SET_GAIN_ST_BITS_LEN               (3)
#define AW88266A_SET_GAIN_ST_MASK                   \
  (~(((1 << AW88266A_SET_GAIN_ST_BITS_LEN) - 1) <<  \
  AW88266A_SET_GAIN_ST_START_BIT))

#define AW88266A_SET_GAIN_ST_4P5_AV                 (0)
#define AW88266A_SET_GAIN_ST_4P5_AV_VALUE           \
  (AW88266A_SET_GAIN_ST_4P5_AV << AW88266A_SET_GAIN_ST_START_BIT)

#define AW88266A_SET_GAIN_ST_5P0_AV                 (1)
#define AW88266A_SET_GAIN_ST_5P0_AV_VALUE           \
  (AW88266A_SET_GAIN_ST_5P0_AV << AW88266A_SET_GAIN_ST_START_BIT)

#define AW88266A_SET_GAIN_ST_6P0_AV                 (2)
#define AW88266A_SET_GAIN_ST_6P0_AV_VALUE           \
  (AW88266A_SET_GAIN_ST_6P0_AV << AW88266A_SET_GAIN_ST_START_BIT)

#define AW88266A_SET_GAIN_ST_6P7_AV                 (3)
#define AW88266A_SET_GAIN_ST_6P7_AV_VALUE           \
  (AW88266A_SET_GAIN_ST_6P7_AV << AW88266A_SET_GAIN_ST_START_BIT)

#define AW88266A_SET_GAIN_ST_10_AV                  (4)
#define AW88266A_SET_GAIN_ST_10_AV_VALUE            \
  (AW88266A_SET_GAIN_ST_10_AV << AW88266A_SET_GAIN_ST_START_BIT)

#define AW88266A_SET_GAIN_ST_12_AV                  (5)
#define AW88266A_SET_GAIN_ST_12_AV_VALUE            \
  (AW88266A_SET_GAIN_ST_12_AV << AW88266A_SET_GAIN_ST_START_BIT)

#define AW88266A_SET_GAIN_ST_DEFAULT_VALUE          (5)
#define AW88266A_SET_GAIN_ST_DEFAULT                \
  (AW88266A_SET_GAIN_ST_DEFAULT_VALUE << AW88266A_SET_GAIN_ST_START_BIT)

/* BSTVOUT_ST bit 3:0 (DACST 0x20) */

#define AW88266A_BSTVOUT_ST_START_BIT               (0)
#define AW88266A_BSTVOUT_ST_BITS_LEN                (4)
#define AW88266A_BSTVOUT_ST_MASK                    \
  (~(((1 << AW88266A_BSTVOUT_ST_BITS_LEN) - 1) <<   \
  AW88266A_BSTVOUT_ST_START_BIT))

#define AW88266A_BSTVOUT_ST_3P25V                   (0)
#define AW88266A_BSTVOUT_ST_3P25V_VALUE             \
  (AW88266A_BSTVOUT_ST_3P25V << AW88266A_BSTVOUT_ST_START_BIT)

#define AW88266A_BSTVOUT_ST_3P50V                   (1)
#define AW88266A_BSTVOUT_ST_3P50V_VALUE             \
  (AW88266A_BSTVOUT_ST_3P50V << AW88266A_BSTVOUT_ST_START_BIT)

#define AW88266A_BSTVOUT_ST_3P75V                   (2)
#define AW88266A_BSTVOUT_ST_3P75V_VALUE             \
  (AW88266A_BSTVOUT_ST_3P75V << AW88266A_BSTVOUT_ST_START_BIT)

#define AW88266A_BSTVOUT_ST_7P00V                   (15)
#define AW88266A_BSTVOUT_ST_7P00V_VALUE             \
  (AW88266A_BSTVOUT_ST_7P00V << AW88266A_BSTVOUT_ST_START_BIT)

#define AW88266A_BSTVOUT_ST_DEFAULT_VALUE           (0)
#define AW88266A_BSTVOUT_ST_DEFAULT                 \
  (AW88266A_BSTVOUT_ST_DEFAULT_VALUE << AW88266A_BSTVOUT_ST_START_BIT)

/* default value of DACST (0x20)
 * #define AW88266A_DACST_DEFAULT (0x0500)
 */

/* VBAT (0x21) detail
 * VBAT_DET bit 9:0 (VBAT 0x21)
 */

#define AW88266A_VBAT_DET_START_BIT                 (0)
#define AW88266A_VBAT_DET_BITS_LEN                  (10)
#define AW88266A_VBAT_DET_MASK                      \
  (~(((1 << AW88266A_VBAT_DET_BITS_LEN) - 1) << AW88266A_VBAT_DET_START_BIT))

#define AW88266A_VBAT_DET_DEFAULT_VALUE             (0x263)
#define AW88266A_VBAT_DET_DEFAULT                   \
  (AW88266A_VBAT_DET_DEFAULT_VALUE << AW88266A_VBAT_DET_START_BIT)

/* default value of VBAT (0x21)
 * #define AW88266A_VBAT_DEFAULT (0x0263)
 */

/* TEMP (0x22) detail
 * TEMP_DET bit 9:0 (TEMP 0x22)
 */

#define AW88266A_TEMP_DET_START_BIT                 (0)
#define AW88266A_TEMP_DET_BITS_LEN                  (10)
#define AW88266A_TEMP_DET_MASK                      \
  (~(((1 << AW88266A_TEMP_DET_BITS_LEN) - 1) << AW88266A_TEMP_DET_START_BIT))

#define AW88266A_TEMP_DET_MINUS_40_DEGREE           (0x3D8)
#define AW88266A_TEMP_DET_MINUS_40_DEGREE_VALUE     \
  (AW88266A_TEMP_DET_MINUS_40_DEGREE << AW88266A_TEMP_DET_START_BIT)

#define AW88266A_TEMP_DET_0_DEGREE                  (0x00)
#define AW88266A_TEMP_DET_0_DEGREE_VALUE            \
  (AW88266A_TEMP_DET_0_DEGREE << AW88266A_TEMP_DET_START_BIT)

#define AW88266A_TEMP_DET_1_DEGREE                  (0x01)
#define AW88266A_TEMP_DET_1_DEGREE_VALUE            \
  (AW88266A_TEMP_DET_1_DEGREE << AW88266A_TEMP_DET_START_BIT)

#define AW88266A_TEMP_DET_25_DEGREE                 (0x19)
#define AW88266A_TEMP_DET_25_DEGREE_VALUE           \
  (AW88266A_TEMP_DET_25_DEGREE << AW88266A_TEMP_DET_START_BIT)

#define AW88266A_TEMP_DET_55_DEGREE                 (0x37)
#define AW88266A_TEMP_DET_55_DEGREE_VALUE           \
  (AW88266A_TEMP_DET_55_DEGREE << AW88266A_TEMP_DET_START_BIT)

#define AW88266A_TEMP_DET_DEFAULT_VALUE             (0x019)
#define AW88266A_TEMP_DET_DEFAULT                   \
  (AW88266A_TEMP_DET_DEFAULT_VALUE << AW88266A_TEMP_DET_START_BIT)

/* default value of TEMP (0x22)
 * #define AW88266A_TEMP_DEFAULT  (0x0019)
 */

/* PVDD (0x23) detail
 * PVDD_DET bit 9:0 (PVDD 0x23)
 */

#define AW88266A_PVDD_DET_START_BIT                 (0)
#define AW88266A_PVDD_DET_BITS_LEN                  (10)
#define AW88266A_PVDD_DET_MASK                      \
  (~(((1 << AW88266A_PVDD_DET_BITS_LEN) - 1) << AW88266A_PVDD_DET_START_BIT))

#define AW88266A_PVDD_DET_DEFAULT_VALUE             (0x263)
#define AW88266A_PVDD_DET_DEFAULT                   \
  (AW88266A_PVDD_DET_DEFAULT_VALUE << AW88266A_PVDD_DET_START_BIT)

/* default value of PVDD (0x23)
 * #define AW88266A_PVDD_DEFAULT  (0x0263)
 */

/* CCO_MUX bit 2 (PLLCTRL3 0x55) */

#define AW88266A_CCO_MUX_START_BIT                  (2)
#define AW88266A_CCO_MUX_BITS_LEN                   (1)
#define AW88266A_CCO_MUX_MASK                       \
  (~(((1 << AW88266A_CCO_MUX_BITS_LEN) - 1) << AW88266A_CCO_MUX_START_BIT))

#define AW88266A_CCO_MUX_DIVIDED                    (0)
#define AW88266A_CCO_MUX_DIVIDED_VALUE              \
  (AW88266A_CCO_MUX_DIVIDED << AW88266A_CCO_MUX_START_BIT)

#define AW88266A_CCO_MUX_BYPASS                     (1)
#define AW88266A_CCO_MUX_BYPASS_VALUE               \
  (AW88266A_CCO_MUX_BYPASS << AW88266A_CCO_MUX_START_BIT)

/* BSTCTRL1 (0x60) detail
 * BURST_OVPCTR bit 15 (BSTCTRL1 0x60)
 */

#define AW88266A_BURST_OVPCTR_START_BIT             (15)
#define AW88266A_BURST_OVPCTR_BITS_LEN              (1)
#define AW88266A_BURST_OVPCTR_MASK                  \
  (~(((1 << AW88266A_BURST_OVPCTR_BITS_LEN) - 1) << \
  AW88266A_BURST_OVPCTR_START_BIT))

#define AW88266A_BURST_OVPCTR_DISABLE               (0)
#define AW88266A_BURST_OVPCTR_DISABLE_VALUE         \
  (AW88266A_BURST_OVPCTR_DISABLE << AW88266A_BURST_OVPCTR_START_BIT)

#define AW88266A_BURST_OVPCTR_ENABLE                (1)
#define AW88266A_BURST_OVPCTR_ENABLE_VALUE          \
  (AW88266A_BURST_OVPCTR_ENABLE << AW88266A_BURST_OVPCTR_START_BIT)

#define AW88266A_BURST_OVPCTR_DEFAULT_VALUE         (1)
#define AW88266A_BURST_OVPCTR_DEFAULT               \
  (AW88266A_BURST_OVPCTR_DEFAULT_VALUE << AW88266A_BURST_OVPCTR_START_BIT)

/* BST_RTH bit 13:8 (BSTCTRL1 0x60) */

#define AW88266A_BST_RTH_START_BIT                  (8)
#define AW88266A_BST_RTH_BITS_LEN                   (6)
#define AW88266A_BST_RTH_MASK                       \
  (~(((1 << AW88266A_BST_RTH_BITS_LEN) - 1) << AW88266A_BST_RTH_START_BIT))

#define AW88266A_BST_RTH_DEFAULT_VALUE              (4)
#define AW88266A_BST_RTH_DEFAULT                    \
  (AW88266A_BST_RTH_DEFAULT_VALUE << AW88266A_BST_RTH_START_BIT)

/* BST_ATH bit 5:0 (BSTCTRL1 0x60) */

#define AW88266A_BST_ATH_START_BIT                  (0)
#define AW88266A_BST_ATH_BITS_LEN                   (6)
#define AW88266A_BST_ATH_MASK                       \
  (~(((1 << AW88266A_BST_ATH_BITS_LEN) - 1) << AW88266A_BST_ATH_START_BIT))

#define AW88266A_BST_ATH_DEFAULT_VALUE              (2)
#define AW88266A_BST_ATH_DEFAULT                    \
  (AW88266A_BST_ATH_DEFAULT_VALUE << AW88266A_BST_ATH_START_BIT)

/* default value of BSTCTRL1 (0x60)
 * #define AW88266A_BSTCTRL1_DEFAULT  (0x8402)
 */

/* BSTCTRL2 (0x61) detail
 * BST_VOUT6_SEL bit 15 (BSTCTRL2 0x61)
 */

#define AW88266A_BST_VOUT6_SEL_START_BIT            (15)
#define AW88266A_BST_VOUT6_SEL_BITS_LEN             (1)
#define AW88266A_BST_VOUT6_SEL_MASK                 \
  (~(((1 << AW88266A_BST_VOUT6_SEL_BITS_LEN) - 1) <<\
  AW88266A_BST_VOUT6_SEL_START_BIT))

#define AW88266A_BST_VOUT6_SEL_6P0V                 (0)
#define AW88266A_BST_VOUT6_SEL_6P0V_VALUE           \
  (AW88266A_BST_VOUT6_SEL_6P0V << AW88266A_BST_VOUT6_SEL_START_BIT)

#define AW88266A_BST_VOUT6_SEL_6P1V                 (1)
#define AW88266A_BST_VOUT6_SEL_6P1V_VALUE           \
  (AW88266A_BST_VOUT6_SEL_6P1V << AW88266A_BST_VOUT6_SEL_START_BIT)

#define AW88266A_BST_VOUT6_SEL_DEFAULT_VALUE        (0)
#define AW88266A_BST_VOUT6_SEL_DEFAULT              \
  (AW88266A_BST_VOUT6_SEL_DEFAULT_VALUE << AW88266A_BST_VOUT6_SEL_START_BIT)

/* BST_IPEAK bit 14:12 (BSTCTRL2 0x61) */

#define AW88266A_BST_IPEAK_START_BIT                (12)
#define AW88266A_BST_IPEAK_BITS_LEN                 (3)
#define AW88266A_BST_IPEAK_MASK                     \
  (~(((1 << AW88266A_BST_IPEAK_BITS_LEN) - 1) <<    \
  AW88266A_BST_IPEAK_START_BIT))

#define AW88266A_BST_IPEAK_1P20A                    (0)
#define AW88266A_BST_IPEAK_1P20A_VALUE              \
  (AW88266A_BST_IPEAK_1P20A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_1P40A                    (1)
#define AW88266A_BST_IPEAK_1P40A_VALUE              \
  (AW88266A_BST_IPEAK_1P40A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_1P60A                    (2)
#define AW88266A_BST_IPEAK_1P60A_VALUE              \
  (AW88266A_BST_IPEAK_1P60A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_1P80A                    (3)
#define AW88266A_BST_IPEAK_1P80A_VALUE              \
  (AW88266A_BST_IPEAK_1P80A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_2P00A                    (4)
#define AW88266A_BST_IPEAK_2P00A_VALUE              \
  (AW88266A_BST_IPEAK_2P00A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_2P25A                    (5)
#define AW88266A_BST_IPEAK_2P25A_VALUE              \
  (AW88266A_BST_IPEAK_2P25A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_2P50A                    (6)
#define AW88266A_BST_IPEAK_2P50A_VALUE              \
  (AW88266A_BST_IPEAK_2P50A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_2P75A                    (7)
#define AW88266A_BST_IPEAK_2P75A_VALUE              \
  (AW88266A_BST_IPEAK_2P75A << AW88266A_BST_IPEAK_START_BIT)

#define AW88266A_BST_IPEAK_DEFAULT_VALUE            (6)
#define AW88266A_BST_IPEAK_DEFAULT                  \
  (AW88266A_BST_IPEAK_DEFAULT_VALUE << AW88266A_BST_IPEAK_START_BIT)

/* BST_TDEG bit 11:8 (BSTCTRL2 0x61) */

#define AW88266A_BST_TDEG_START_BIT                 (8)
#define AW88266A_BST_TDEG_BITS_LEN                  (4)
#define AW88266A_BST_TDEG_MASK                      \
  (~(((1 << AW88266A_BST_TDEG_BITS_LEN) - 1) <<     \
  AW88266A_BST_TDEG_START_BIT))

#define AW88266A_BST_TDEG_0P50_MS                   (0)
#define AW88266A_BST_TDEG_0P50_MS_VALUE             \
  (AW88266A_BST_TDEG_0P50_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_1P00_MS                   (1)
#define AW88266A_BST_TDEG_1P00_MS_VALUE             \
  (AW88266A_BST_TDEG_1P00_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_2P00_MS                   (2)
#define AW88266A_BST_TDEG_2P00_MS_VALUE             \
  (AW88266A_BST_TDEG_2P00_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_4P00_MS                   (3)
#define AW88266A_BST_TDEG_4P00_MS_VALUE             \
  (AW88266A_BST_TDEG_4P00_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_8P00_MS                   (4)
#define AW88266A_BST_TDEG_8P00_MS_VALUE             \
  (AW88266A_BST_TDEG_8P00_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_10P7_MS                   (5)
#define AW88266A_BST_TDEG_10P7_MS_VALUE             \
  (AW88266A_BST_TDEG_10P7_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_13P3_MS                   (6)
#define AW88266A_BST_TDEG_13P3_MS_VALUE             \
  (AW88266A_BST_TDEG_13P3_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_16P0_MS                   (7)
#define AW88266A_BST_TDEG_16P0_MS_VALUE             \
  (AW88266A_BST_TDEG_16P0_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_18P6_MS                   (8)
#define AW88266A_BST_TDEG_18P6_MS_VALUE             \
  (AW88266A_BST_TDEG_18P6_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_21P3_MS                   (9)
#define AW88266A_BST_TDEG_21P3_MS_VALUE             \
  (AW88266A_BST_TDEG_21P3_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_24P0_MS                   (10)
#define AW88266A_BST_TDEG_24P0_MS_VALUE             \
  (AW88266A_BST_TDEG_24P0_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_32P0_MS                   (11)
#define AW88266A_BST_TDEG_32P0_MS_VALUE             \
  (AW88266A_BST_TDEG_32P0_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_64P0_MS                   (12)
#define AW88266A_BST_TDEG_64P0_MS_VALUE             \
  (AW88266A_BST_TDEG_64P0_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_128_MS                    (13)
#define AW88266A_BST_TDEG_128_MS_VALUE              \
  (AW88266A_BST_TDEG_128_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_256_MS                    (14)
#define AW88266A_BST_TDEG_256_MS_VALUE              \
  (AW88266A_BST_TDEG_256_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_1200_MS                   (15)
#define AW88266A_BST_TDEG_1200_MS_VALUE             \
  (AW88266A_BST_TDEG_1200_MS << AW88266A_BST_TDEG_START_BIT)

#define AW88266A_BST_TDEG_DEFAULT_VALUE             (11)
#define AW88266A_BST_TDEG_DEFAULT                   \
  (AW88266A_BST_TDEG_DEFAULT_VALUE << AW88266A_BST_TDEG_START_BIT)

/* BURST_FRE bit 7 (BSTCTRL2 0x61) */

#define AW88266A_BURST_FRE_START_BIT                (7)
#define AW88266A_BURST_FRE_BITS_LEN                 (1)
#define AW88266A_BURST_FRE_MASK                     \
  (~(((1 << AW88266A_BURST_FRE_BITS_LEN) - 1) <<    \
  AW88266A_BURST_FRE_START_BIT))

#define AW88266A_BURST_FRE_20KHZ                    (0)
#define AW88266A_BURST_FRE_20KHZ_VALUE              \
  (AW88266A_BURST_FRE_20KHZ << AW88266A_BURST_FRE_START_BIT)

#define AW88266A_BURST_FRE_30KHZ                    (1)
#define AW88266A_BURST_FRE_30KHZ_VALUE              \
  (AW88266A_BURST_FRE_30KHZ << AW88266A_BURST_FRE_START_BIT)

#define AW88266A_BURST_FRE_DEFAULT_VALUE            (0)
#define AW88266A_BURST_FRE_DEFAULT                  \
  (AW88266A_BURST_FRE_DEFAULT_VALUE << AW88266A_BURST_FRE_START_BIT)

/* RSQN_DLY_EN bit 6 (BSTCTRL2 0x61) */

#define AW88266A_RSQN_DLY_EN_START_BIT              (6)
#define AW88266A_RSQN_DLY_EN_BITS_LEN               (1)
#define AW88266A_RSQN_DLY_EN_MASK                   \
  (~(((1 << AW88266A_RSQN_DLY_EN_BITS_LEN) - 1) <<  \
  AW88266A_RSQN_DLY_EN_START_BIT))

#define AW88266A_RSQN_DLY_EN_6NS                    (0)
#define AW88266A_RSQN_DLY_EN_6NS_VALUE              \
  (AW88266A_RSQN_DLY_EN_6NS << AW88266A_RSQN_DLY_EN_START_BIT)

#define AW88266A_RSQN_DLY_EN_12NS                   (1)
#define AW88266A_RSQN_DLY_EN_12NS_VALUE             \
  (AW88266A_RSQN_DLY_EN_12NS << AW88266A_RSQN_DLY_EN_START_BIT)

#define AW88266A_RSQN_DLY_EN_DEFAULT_VALUE          (0)
#define AW88266A_RSQN_DLY_EN_DEFAULT                \
  (AW88266A_RSQN_DLY_EN_DEFAULT_VALUE << AW88266A_RSQN_DLY_EN_START_BIT)

/* BST_MODE bit 5:4 (BSTCTRL2 0x61) */

#define AW88266A_BST_MODE_START_BIT                 (4)
#define AW88266A_BST_MODE_BITS_LEN                  (2)
#define AW88266A_BST_MODE_MASK                      \
  (~(((1 << AW88266A_BST_MODE_BITS_LEN) - 1) << AW88266A_BST_MODE_START_BIT))

#define AW88266A_BST_MODE_TRANSPARENT               (0)
#define AW88266A_BST_MODE_TRANSPARENT_VALUE         \
  (AW88266A_BST_MODE_TRANSPARENT << AW88266A_BST_MODE_START_BIT)

#define AW88266A_BST_MODE_FORCE_BOOST               (1)
#define AW88266A_BST_MODE_FORCE_BOOST_VALUE         \
  (AW88266A_BST_MODE_FORCE_BOOST << AW88266A_BST_MODE_START_BIT)

#define AW88266A_BST_MODE_CLASS_G                   (2)
#define AW88266A_BST_MODE_CLASS_G_VALUE             \
  (AW88266A_BST_MODE_CLASS_G << AW88266A_BST_MODE_START_BIT)

#define AW88266A_BST_MODE_CLASS_H                   (3)
#define AW88266A_BST_MODE_CLASS_H_VALUE             \
  (AW88266A_BST_MODE_CLASS_H << AW88266A_BST_MODE_START_BIT)

#define AW88266A_BST_MODE_DEFAULT_VALUE             (0x3)
#define AW88266A_BST_MODE_DEFAULT                   \
  (AW88266A_BST_MODE_DEFAULT_VALUE << AW88266A_BST_MODE_START_BIT)

/* VOUT_VREFSET bit 3:0 (BSTCTRL2 0x61) */

#define AW88266A_VOUT_VREFSET_START_BIT             (0)
#define AW88266A_VOUT_VREFSET_BITS_LEN              (4)
#define AW88266A_VOUT_VREFSET_MASK                  \
  (~(((1 << AW88266A_VOUT_VREFSET_BITS_LEN) - 1) << \
  AW88266A_VOUT_VREFSET_START_BIT))

#define AW88266A_VOUT_VREFSET_3P25V                 (0)
#define AW88266A_VOUT_VREFSET_3P25V_VALUE           \
  (AW88266A_VOUT_VREFSET_3P25V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_3P50V                 (1)
#define AW88266A_VOUT_VREFSET_3P50V_VALUE           \
  (AW88266A_VOUT_VREFSET_3P50V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_3P75V                 (2)
#define AW88266A_VOUT_VREFSET_3P75V_VALUE           \
  (AW88266A_VOUT_VREFSET_3P75V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_4P00V                 (3)
#define AW88266A_VOUT_VREFSET_4P00V_VALUE           \
  (AW88266A_VOUT_VREFSET_4P00V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_4P25V                 (4)
#define AW88266A_VOUT_VREFSET_4P25V_VALUE           \
  (AW88266A_VOUT_VREFSET_4P25V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_4P50V                 (5)
#define AW88266A_VOUT_VREFSET_4P50V_VALUE           \
  (AW88266A_VOUT_VREFSET_4P50V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_4P75V                 (6)
#define AW88266A_VOUT_VREFSET_4P75V_VALUE           \
  (AW88266A_VOUT_VREFSET_4P75V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_5P00V                 (7)
#define AW88266A_VOUT_VREFSET_5P00V_VALUE           \
  (AW88266A_VOUT_VREFSET_5P00V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_5P25V                 (8)
#define AW88266A_VOUT_VREFSET_5P25V_VALUE           \
  (AW88266A_VOUT_VREFSET_5P25V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_5P50V                 (9)
#define AW88266A_VOUT_VREFSET_5P50V_VALUE           \
  (AW88266A_VOUT_VREFSET_5P50V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_5P75V                 (10)
#define AW88266A_VOUT_VREFSET_5P75V_VALUE           \
  (AW88266A_VOUT_VREFSET_5P75V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_6P00V                 (11)
#define AW88266A_VOUT_VREFSET_6P00V_VALUE           \
  (AW88266A_VOUT_VREFSET_6P00V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_6P25V                 (12)
#define AW88266A_VOUT_VREFSET_6P25V_VALUE           \
  (AW88266A_VOUT_VREFSET_6P25V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_6P50V                 (13)
#define AW88266A_VOUT_VREFSET_6P50V_VALUE           \
  (AW88266A_VOUT_VREFSET_6P50V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_6P75V                 (14)
#define AW88266A_VOUT_VREFSET_6P75V_VALUE           \
  (AW88266A_VOUT_VREFSET_6P75V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_7P00V                 (15)
#define AW88266A_VOUT_VREFSET_7P00V_VALUE           \
  (AW88266A_VOUT_VREFSET_7P00V << AW88266A_VOUT_VREFSET_START_BIT)

#define AW88266A_VOUT_VREFSET_DEFAULT_VALUE         (0xB)
#define AW88266A_VOUT_VREFSET_DEFAULT               \
  (AW88266A_VOUT_VREFSET_DEFAULT_VALUE << AW88266A_VOUT_VREFSET_START_BIT)

/* default value of BSTCTRL2 (0x61) */

/* #define AW88266A_BSTCTRL2_DEFAULT  (0x6B3B) */

/* EF_ISN_GESLP bit 9:0 (EFRH 0x78) */

#define AW88266A_EF_ISN_GESLP_START_BIT             (0)
#define AW88266A_EF_ISN_GESLP_BITS_LEN              (10)
#define AW88266A_EF_ISN_GESLP_MASK                  \
  (~(((1 << AW88266A_EF_ISN_GESLP_BITS_LEN) - 1) << \
  AW88266A_EF_ISN_GESLP_START_BIT))

/* EF_VSN_GESLP bit 9:0 (EFRM2 0x79) */

#define AW88266A_EF_VSN_GESLP_START_BIT             (0)
#define AW88266A_EF_VSN_GESLP_BITS_LEN              (10)
#define AW88266A_EF_VSN_GESLP_MASK                  \
  (~(((1 << AW88266A_EF_VSN_GESLP_BITS_LEN) - 1) << \
  AW88266A_EF_VSN_GESLP_START_BIT))

/* detail information of registers end */

#define AW88266A_EFVER_CHECK                        (0x0007)
#define AW88266A_EFVERH_START_BIT                   (13)
#define AW88266A_EFVERL_START_BIT                   (10)

#define AW88266A_EFVERH_MASK                        (~(7 << 13))
#define AW88266A_EFVERL_MASK                        (~(7 << 10))

/* Volume Coefficient */

#define AW88266A_VOL_STEP	                          (6 * 8)

/* Vcalb */

#define AW88266A_EF_ISN_GESLP2_SIGN_MASK            (~0x0200)
#define AW88266A_EF_ISN_GESLP2_NEG                  (~0xFC00)

#define AW88266A_EF_ISN_GESLP_SIGN_MASK             (~0x0200)
#define AW88266A_EF_ISN_GESLP_NEG                   (~0xFC00)

#define AW88266A_CABL_BASE_VALUE                    (1000)
#define AW88266A_ICABLK_FACTOR                      (1)
#define AW88266A_VCABLK_FACTOR                      (1)

#define AW88266A_VCAL_FACTOR                        (1<<13)

#define AW88266A_MONITOR_VBAT_RANGE                 (6025)
#define AW88266A_MONITOR_INT_10BIT                  (1023)
#define AW88266A_MONITOR_TEMP_SIGN_MASK             (~(1<<9))
#define AW88266A_MONITOR_TEMP_NEG_MASK              (0XFC00)

#endif
