/******************************************************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_vadc.h
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
 ******************************************************************************************************************/

/******************************************************************************************************************
 * May include some logic from sample code provided by Infineon:
 *
 * Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS,
 * IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************************************************/

/* Reference: XMC4[78]00 Reference Manual V1.3 2016-07 Microcontrollers.
 * Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers.
 */

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_VADC_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_VADC_H

/******************************************************************************************************************
 * Included Files
 ******************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/xmc4_memorymap.h"

#include <stdint.h>

/******************************************************************************************************************
 * Preprocessor Definitions
 ******************************************************************************************************************/

/* Define XMC4[78] and XMC45 series VADC module specifications */

#if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
#   define XMC_VADC_BOUNDARY_FLAG_SELECT    (1U) /*  Defines the availability of boundary flags in a device*/
#   define XMC_VADC_EMUX_CH_SEL_STYLE       (1U) /*  Defines the external multiplexer channel selection mode of
                                                  *  operation for a particular device */
#   define XMC_VADC_GROUP_SRCREG_AVAILABLE  (1U) /*  Define the availability of a source specific result register */
#elif defined(CONFIG_ARCH_CHIP_XMC4500)
#   define XMC_VADC_BOUNDARY_FLAG_SELECT    (0U) /*  Defines the availability of boundary flags in a device*/
#   define XMC_VADC_EMUX_CH_SEL_STYLE       (0U) /*  Defines the external multiplexer channel selection mode of
                                                  *  operation for a particular device */
#   define XMC_VADC_GROUP_SRCREG_AVAILABLE  (0U) /*  Define the availability of a source specific result register */
#else
#  error "Unsupported XMC4xxx chip"
#endif

#define XMC_VADC_MAXIMUM_NUM_GROUPS      (4U)  /* Defines the maximum number of groups available in a device */
#define XMC_VADC_NUM_RESULT_REGISTERS    (16U) /* Defines the number of result holding registers per ADC group */
#define XMC_VADC_NUM_CHANNELS_PER_GROUP  (8U)  /* Defines the number of ADC channels per group */

/* Register Offsets */

/* Structures to handle VADC global registers */

typedef struct                                   /* (@ 0x40004000) VADC Structure */
{
  volatile uint32_t  CLC;                        /* (@ 0x40004000) Clock Control Register */
  volatile uint32_t  RESERVED;
  volatile uint32_t  ID;                         /* (@ 0x40004008) Module Identification Register */
  volatile uint32_t  RESERVED1[7];
  volatile uint32_t  OCS;                        /* (@ 0x40004028) OCDS Control and Status Register */
  volatile uint32_t  RESERVED2[21];
  volatile uint32_t  GLOBCFG;                    /* (@ 0x40004080) Global Configuration Register */
  volatile uint32_t  RESERVED3[7];
  volatile uint32_t  GLOBICLASS[2];              /* (@ 0x400040A0) Input Class Register, Global */
  volatile uint32_t  RESERVED4[4];
  volatile uint32_t  GLOBBOUND;                  /* (@ 0x400040B8) Global Boundary Select Register */
  volatile uint32_t  RESERVED5[9];
  volatile uint32_t  GLOBEFLAG;                  /* (@ 0x400040E0) Global Event Flag Register */
  volatile uint32_t  RESERVED6[23];
  volatile uint32_t  GLOBEVNP;                   /* (@ 0x40004140) Global Event Node Pointer Register */
  volatile uint32_t  RESERVED7[7];
  volatile uint32_t  GLOBTF;                     /* (@ 0x40004160) Global Test Functions Register */
  volatile uint32_t  RESERVED8[7];
  volatile uint32_t  BRSSEL[4];                  /* (@ 0x40004180) Background Request Source Channel Select Register */
  volatile uint32_t  RESERVED9[12];
  volatile uint32_t  BRSPND[4];                  /* (@ 0x400041C0) Background Request Source Pending Register */
  volatile uint32_t  RESERVED10[12];
  volatile uint32_t  BRSCTRL;                    /* (@ 0x40004200) Background Request Source Control Register */
  volatile uint32_t  BRSMR;                      /* (@ 0x40004204) Background Request Source Mode Register */
  volatile uint32_t  RESERVED11[30];
  volatile uint32_t  GLOBRCR;                    /* (@ 0x40004280) Global Result Control Register */
  volatile uint32_t  RESERVED12[31];
  volatile uint32_t  GLOBRES;                    /* (@ 0x40004300) Global Result Register */
  volatile uint32_t  RESERVED13[31];
  volatile uint32_t  GLOBRESD;                   /* (@ 0x40004380) Global Result Register, Debug */
  volatile uint32_t  RESERVED14[27];
  volatile uint32_t  EMUXSEL;                    /* (@ 0x400043F0) External Multiplexer Select Register */
} vadc_global_t;

/* Structures to handle VADC groups registers */

typedef struct                                   /* (@ 0x40004400) */
{
  volatile uint32_t  RESERVED[32];
  volatile uint32_t  ARBCFG;                     /* (@ 0x40004480) Arbitration Configuration Register */
  volatile uint32_t  ARBPR;                      /* (@ 0x40004484) Arbitration Priority Register */
  volatile uint32_t  CHASS;                      /* (@ 0x40004488) Channel Assignment Register */
  volatile uint32_t  RESERVED1[5];
  volatile uint32_t  ICLASS[2];                  /* (@ 0x400044A0) Input Class Register */
  volatile uint32_t  RESERVED2[2];
  volatile uint32_t  ALIAS;                      /* (@ 0x400044B0) Alias Register */
  volatile uint32_t  RESERVED3;
  volatile uint32_t  BOUND;                      /* (@ 0x400044B8) Boundary Select Register */
  volatile uint32_t  RESERVED4;
  volatile uint32_t  SYNCTR;                     /* (@ 0x400044C0) Synchronization Control Register */
  volatile uint32_t  RESERVED5;
  volatile uint32_t  BFL;                        /* (@ 0x400044C8) Boundary Flag Register */
  #if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
    volatile uint32_t  BFLS;                     /* (@ 0x400044CC) Boundary Flag Software Register */
    volatile uint32_t  BFLC;                     /* (@ 0x400044D0) Boundary Flag Control Register */
    volatile uint32_t  BFLNP;                    /* (@ 0x400044D4) Boundary Flag Node Pointer Register */
    volatile uint32_t  RESERVED6[10];
  #elif defined(CONFIG_ARCH_CHIP_XMC4500)
    volatile uint32_t  RESERVED6[13];
  #endif
  volatile uint32_t  QCTRL0;                     /* (@ 0x40004500) Queue 0 Source Control Register */
  volatile uint32_t  QMR0;                       /* (@ 0x40004504) Queue 0 Mode Register */
  volatile uint32_t  QSR0;                       /* (@ 0x40004508) Queue 0 Status Register */
  volatile uint32_t  Q0R0;                       /* (@ 0x4000450C) Queue 0 Register 0 */

  union
  {
    volatile uint32_t  QBUR0;                    /* (@ 0x40004510) Queue 0 Backup Register */
    volatile uint32_t  QINR0;                    /* (@ 0x40004510) Queue 0 Input Register */
  };

  volatile uint32_t  RESERVED7[3];
  volatile uint32_t  ASCTRL;                     /* (@ 0x40004520) Autoscan Source Control Register */
  volatile uint32_t  ASMR;                       /* (@ 0x40004524) Autoscan Source Mode Register */
  volatile uint32_t  ASSEL;                      /* (@ 0x40004528) Autoscan Source Channel Select Register */
  volatile uint32_t  ASPND;                      /* (@ 0x4000452C) Autoscan Source Pending Register */
  volatile uint32_t  RESERVED8[20];
  volatile uint32_t  CEFLAG;                     /* (@ 0x40004580) Channel Event Flag Register */
  volatile uint32_t  REFLAG;                     /* (@ 0x40004584) Result Event Flag Register */
  volatile uint32_t  SEFLAG;                     /* (@ 0x40004588) Source Event Flag Register */
  volatile uint32_t  RESERVED9;
  volatile uint32_t  CEFCLR;                     /* (@ 0x40004590) Channel Event Flag Clear Register */
  volatile uint32_t  REFCLR;                     /* (@ 0x40004594) Result Event Flag Clear Register */
  volatile uint32_t  SEFCLR;                     /* (@ 0x40004598) Source Event Flag Clear Register */
  volatile uint32_t  RESERVED10;
  volatile uint32_t  CEVNP0;                     /* (@ 0x400045A0) Channel Event Node Pointer Register 0 */
  volatile uint32_t  RESERVED11[3];
  volatile uint32_t  REVNP0;                     /* (@ 0x400045B0) Result Event Node Pointer Register 0 */
  volatile uint32_t  REVNP1;                     /* (@ 0x400045B4) Result Event Node Pointer Register 1 */
  volatile uint32_t  RESERVED12[2];
  volatile uint32_t  SEVNP;                      /* (@ 0x400045C0) Source Event Node Pointer Register */
  volatile uint32_t  RESERVED13;
  volatile uint32_t  SRACT;                      /* (@ 0x400045C8) Service Request Software Activation Trigger */
  volatile uint32_t  RESERVED14[9];
  volatile uint32_t  EMUXCTR;                    /* (@ 0x400045F0) E0ternal Multiplexer Control Register */
  volatile uint32_t  RESERVED15;
  volatile uint32_t  VFR;                        /* (@ 0x400045F8) Valid Flag Register */
  volatile uint32_t  RESERVED16;
  volatile uint32_t  CHCTR[8];                   /* (@ 0x40004600) Channel Ctrl. Reg */
  volatile uint32_t  RESERVED17[24];
  volatile uint32_t  RCR[16];                    /* (@ 0x40004680) Result Control Register */
  volatile uint32_t  RESERVED18[16];
  volatile uint32_t  RES[16];                    /* (@ 0x40004700) Result Register */
  volatile uint32_t  RESERVED19[16];
  volatile uint32_t  RESD[16];                   /* (@ 0x40004780) Result Register, Debug */
} vadc_group_t;

/* Define pointers for VADC global and group stuctures */
#define VADC      ((vadc_global_t *) XMC4_VADC_BASE)
#define VADC_G0   ((vadc_group_t *) XMC4_VADC_G0_BASE)
#define VADC_G1   ((vadc_group_t *) XMC4_VADC_G1_BASE)
#define VADC_G2   ((vadc_group_t *) XMC4_VADC_G2_BASE)
#define VADC_G3   ((vadc_group_t *) XMC4_VADC_G3_BASE)

/* Register Bit-Field Definitions */

/* ----------------------------------  VADC_CLC  ---------------------------------- */
#define VADC_CLC_DISR_SHIFT                     (0UL)
#define VADC_CLC_DISR_MASK                      (0x1 << VADC_CLC_DISR_SHIFT)
#define VADC_CLC_DISS_SHIFT                     (2UL)
#define VADC_CLC_DISS_MASK                      (0x1 << VADC_CLC_DISS_SHIFT)
#define VADC_CLC_EDIS_SHIFT                     (3UL)
#define VADC_CLC_EDIS_MASK                      (0x1 << VADC_CLC_EDIS_SHIFT)

/* -----------------------------------  VADC_ID  ---------------------------------- */
#define VADC_ID_MOD_REV_MASK                    (0xff << VADC_ID_MOD_REV_SHIFT)
#define VADC_ID_MOD_TYPE_SHIFT                  (8UL)
#define VADC_ID_MOD_TYPE_MASK                   (0xff << VADC_ID_MOD_TYPE_SHIFT)
#define VADC_ID_MOD_REV_SHIFT                   (0UL)
#define VADC_ID_MOD_NUMBER_SHIFT                (16UL)
#define VADC_ID_MOD_NUMBER_MASK                 (0xffff << VADC_ID_MOD_NUMBER_SHIFT)

/* ----------------------------------  VADC_OCS  ---------------------------------- */
#define VADC_OCS_TGS_SHIFT                      (0UL)
#define VADC_OCS_TGS_MASK                       (0x3 << VADC_OCS_TGS_SHIFT)
#define VADC_OCS_TGB_SHIFT                      (2UL)
#define VADC_OCS_TGB_MASK                       (0x1 << VADC_OCS_TGB_SHIFT)
#define VADC_OCS_TG_P_SHIFT                     (3UL)
#define VADC_OCS_TG_P_MASK                      (0x1 << VADC_OCS_TG_P_SHIFT)
#define VADC_OCS_SUS_SHIFT                      (24UL)
#define VADC_OCS_SUS_MASK                       (0xf << VADC_OCS_SUS_SHIFT)
#define VADC_OCS_SUS_P_SHIFT                    (28UL)
#define VADC_OCS_SUS_P_MASK                     (0x1 << VADC_OCS_SUS_P_SHIFT)
#define VADC_OCS_SUSSTA_SHIFT                   (29UL)
#define VADC_OCS_SUSSTA_MASK                    (0x1 << VADC_OCS_SUSSTA_SHIFT)

/* --------------------------------  VADC_GLOBCFG  -------------------------------- */
#define VADC_GLOBCFG_DIVA_SHIFT                 (0UL)
#define VADC_GLOBCFG_DIVA_MASK                  (0x1f << VADC_GLOBCFG_DIVA_SHIFT)
#define VADC_GLOBCFG_DCMSB_SHIFT                (7UL)
#define VADC_GLOBCFG_DCMSB_MASK                 (0x1 << VADC_GLOBCFG_DCMSB_SHIFT)
#define VADC_GLOBCFG_DIVD_SHIFT                 (8UL)
#define VADC_GLOBCFG_DIVD_MASK                  (0x3 << VADC_GLOBCFG_DIVD_SHIFT)
#define VADC_GLOBCFG_DIVWC_SHIFT                (15UL)
#define VADC_GLOBCFG_DIVWC_MASK                 (0x1 << VADC_GLOBCFG_DIVWC_SHIFT)
#define VADC_GLOBCFG_DPCAL0_SHIFT               (16UL)
#define VADC_GLOBCFG_DPCAL0_MASK                (0x1 << VADC_GLOBCFG_DPCAL0_SHIFT)
#define VADC_GLOBCFG_DPCAL1_SHIFT               (17UL)
#define VADC_GLOBCFG_DPCAL1_MASK                (0x1 << VADC_GLOBCFG_DPCAL1_SHIFT)
#define VADC_GLOBCFG_DPCAL2_SHIFT               (18UL)
#define VADC_GLOBCFG_DPCAL2_MASK                (0x1 << VADC_GLOBCFG_DPCAL2_SHIFT)
#define VADC_GLOBCFG_DPCAL3_SHIFT               (19UL)
#define VADC_GLOBCFG_DPCAL3_MASK                (0x1 << VADC_GLOBCFG_DPCAL3_SHIFT)
#define VADC_GLOBCFG_SUCAL_SHIFT                (31UL)
#define VADC_GLOBCFG_SUCAL_MASK                 (0x1 << VADC_GLOBCFG_SUCAL_SHIFT)

/* -------------------------------  VADC_GLOBICLASS  ------------------------------ */
#define VADC_GLOBICLASS_STCS_MASK               (0x1f << VADC_GLOBICLASS_STCS_SHIFT)
#define VADC_GLOBICLASS_CMS_SHIFT               (8UL)
#define VADC_GLOBICLASS_STCS_SHIFT              (0UL)
#define VADC_GLOBICLASS_CMS_MASK                (0x7 << VADC_GLOBICLASS_CMS_SHIFT)
#define VADC_GLOBICLASS_STCE_SHIFT              (16UL)
#define VADC_GLOBICLASS_STCE_MASK               (0x1f << VADC_GLOBICLASS_STCE_SHIFT)
#define VADC_GLOBICLASS_CME_SHIFT               (24UL)
#define VADC_GLOBICLASS_CME_MASK                (0x7 << VADC_GLOBICLASS_CME_SHIFT)

/* -------------------------------  VADC_GLOBBOUND  ------------------------------- */
#define VADC_GLOBBOUND_BOUNDARY0_SHIFT          (0UL)
#define VADC_GLOBBOUND_BOUNDARY0_MASK           (0xfff << VADC_GLOBBOUND_BOUNDARY0_SHIFT)
#define VADC_GLOBBOUND_BOUNDARY1_SHIFT          (16UL)
#define VADC_GLOBBOUND_BOUNDARY1_MASK           (0xfff << VADC_GLOBBOUND_BOUNDARY1_SHIFT)

/* -------------------------------  VADC_GLOBEFLAG  ------------------------------- */
#define VADC_GLOBEFLAG_SEVGLB_SHIFT             (0UL)
#define VADC_GLOBEFLAG_SEVGLB_MASK              (0x1 << VADC_GLOBEFLAG_SEVGLB_SHIFT)
#define VADC_GLOBEFLAG_REVGLB_SHIFT             (8UL)
#define VADC_GLOBEFLAG_REVGLB_MASK              (0x1 << VADC_GLOBEFLAG_REVGLB_SHIFT)
#define VADC_GLOBEFLAG_SEVGLBCLR_SHIFT          (16UL)
#define VADC_GLOBEFLAG_SEVGLBCLR_MASK           (0x1 << VADC_GLOBEFLAG_SEVGLBCLR_SHIFT)
#define VADC_GLOBEFLAG_REVGLBCLR_SHIFT          (24UL)
#define VADC_GLOBEFLAG_REVGLBCLR_MASK           (0x1 << VADC_GLOBEFLAG_REVGLBCLR_SHIFT)

/* --------------------------------  VADC_GLOBEVNP  ------------------------------- */
#define VADC_GLOBEVNP_SEV0NP_SHIFT              (0UL)
#define VADC_GLOBEVNP_SEV0NP_MASK               (0xf << VADC_GLOBEVNP_SEV0NP_SHIFT)
#define VADC_GLOBEVNP_REV0NP_SHIFT              (16UL)
#define VADC_GLOBEVNP_REV0NP_MASK               (0xf << VADC_GLOBEVNP_REV0NP_SHIFT)

/* ---------------------------------  VADC_GLOBTF  -------------------------------- */
#define VADC_GLOBTF_CDGR_SHIFT                  (4UL)
#define VADC_GLOBTF_CDGR_MASK                   (0xf << VADC_GLOBTF_CDGR_SHIFT)
#define VADC_GLOBTF_CDEN_SHIFT                  (8UL)
#define VADC_GLOBTF_CDEN_MASK                   (0x1 << VADC_GLOBTF_CDEN_SHIFT)
#define VADC_GLOBTF_CDSEL_SHIFT                 (9UL)
#define VADC_GLOBTF_CDSEL_MASK                  (0x3 << VADC_GLOBTF_CDSEL_SHIFT)
#define VADC_GLOBTF_CDWC_SHIFT                  (15UL)
#define VADC_GLOBTF_CDWC_MASK                   (0x1 << VADC_GLOBTF_CDWC_SHIFT)
#define VADC_GLOBTF_PDD_SHIFT                   (16UL)
#define VADC_GLOBTF_PDD_MASK                    (0x1 << VADC_GLOBTF_PDD_SHIFT)
#define VADC_GLOBTF_MDWC_SHIFT                  (23UL)
#define VADC_GLOBTF_MDWC_MASK                   (0x1 << VADC_GLOBTF_MDWC_SHIFT)

/* ---------------------------------  VADC_BRSSELx  -------------------------------- */
#define VADC_BRSSEL_CHSELG0_SHIFT               (0UL)
#define VADC_BRSSEL_CHSELG0_MASK                (0x1 << VADC_BRSSEL_CHSELG0_SHIFT)
#define VADC_BRSSEL_CHSELG1_SHIFT               (1UL)
#define VADC_BRSSEL_CHSELG1_MASK                (0x1 << VADC_BRSSEL_CHSELG1_SHIFT)
#define VADC_BRSSEL_CHSELG2_SHIFT               (2UL)
#define VADC_BRSSEL_CHSELG2_MASK                (0x1 << VADC_BRSSEL_CHSELG2_SHIFT)
#define VADC_BRSSEL_CHSELG3_SHIFT               (3UL)
#define VADC_BRSSEL_CHSELG3_MASK                (0x1 << VADC_BRSSEL_CHSELG3_SHIFT)
#define VADC_BRSSEL_CHSELG4_SHIFT               (4UL)
#define VADC_BRSSEL_CHSELG4_MASK                (0x1 << VADC_BRSSEL_CHSELG4_SHIFT)
#define VADC_BRSSEL_CHSELG5_SHIFT               (5UL)
#define VADC_BRSSEL_CHSELG5_MASK                (0x1 << VADC_BRSSEL_CHSELG5_SHIFT)
#define VADC_BRSSEL_CHSELG6_SHIFT               (6UL)
#define VADC_BRSSEL_CHSELG6_MASK                (0x1 << VADC_BRSSEL_CHSELG6_SHIFT)
#define VADC_BRSSEL_CHSELG7_SHIFT               (7UL)
#define VADC_BRSSEL_CHSELG7_MASK                (0x1 << VADC_BRSSEL_CHSELG7_SHIFT)

/* ---------------------------------  VADC_BRSPNDx  -------------------------------- */
#define VADC_BRSPND_CHPNDG0_SHIFT               (0UL)
#define VADC_BRSPND_CHPNDG0_MASK                (0x1 << VADC_BRSPND_CHPNDG0_SHIFT)
#define VADC_BRSPND_CHPNDG1_SHIFT               (1UL)
#define VADC_BRSPND_CHPNDG1_MASK                (0x1 << VADC_BRSPND_CHPNDG1_SHIFT)
#define VADC_BRSPND_CHPNDG2_SHIFT               (2UL)
#define VADC_BRSPND_CHPNDG2_MASK                (0x1 << VADC_BRSPND_CHPNDG2_SHIFT)
#define VADC_BRSPND_CHPNDG3_SHIFT               (3UL)
#define VADC_BRSPND_CHPNDG3_MASK                (0x1 << VADC_BRSPND_CHPNDG3_SHIFT)
#define VADC_BRSPND_CHPNDG4_SHIFT               (4UL)
#define VADC_BRSPND_CHPNDG4_MASK                (0x1 << VADC_BRSPND_CHPNDG4_SHIFT)
#define VADC_BRSPND_CHPNDG5_SHIFT               (5UL)
#define VADC_BRSPND_CHPNDG5_MASK                (0x1 << VADC_BRSPND_CHPNDG5_SHIFT)
#define VADC_BRSPND_CHPNDG6_SHIFT               (6UL)
#define VADC_BRSPND_CHPNDG6_MASK                (0x1 << VADC_BRSPND_CHPNDG6_SHIFT)
#define VADC_BRSPND_CHPNDG7_SHIFT               (7UL)
#define VADC_BRSPND_CHPNDG7_MASK                (0x1 << VADC_BRSPND_CHPNDG7_SHIFT)

/* --------------------------------  VADC_BRSCTRL  -------------------------------- */

/* XMC4800 SPECIFIC */
#if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
    #define VADC_BRSCTRL_SRCRESREG_SHIFT         (0UL)
    #define VADC_BRSCTRL_SRCRESREG_MASK          (0xf << VADC_BRSCTRL_SRCRESREG_SHIFT)
#endif/* XMC4[78]00 SPECIFIC */

#define VADC_BRSCTRL_XTSEL_SHIFT                (8UL)
#define VADC_BRSCTRL_XTSEL_MASK                 (0xf << VADC_BRSCTRL_XTSEL_SHIFT)
#define VADC_BRSCTRL_XTLVL_SHIFT                (12UL)
#define VADC_BRSCTRL_XTLVL_MASK                 (0x1 << VADC_BRSCTRL_XTLVL_SHIFT)
#define VADC_BRSCTRL_XTMODE_SHIFT               (13UL)
#define VADC_BRSCTRL_XTMODE_MASK                (0x3 << VADC_BRSCTRL_XTMODE_SHIFT)
#define VADC_BRSCTRL_XTWC_SHIFT                 (15UL)
#define VADC_BRSCTRL_XTWC_MASK                  (0x1 << VADC_BRSCTRL_XTWC_SHIFT)
#define VADC_BRSCTRL_GTSEL_SHIFT                (16UL)
#define VADC_BRSCTRL_GTSEL_MASK                 (0xf << VADC_BRSCTRL_GTSEL_SHIFT)
#define VADC_BRSCTRL_GTLVL_SHIFT                (20UL)
#define VADC_BRSCTRL_GTLVL_MASK                 (0x1 << VADC_BRSCTRL_GTLVL_SHIFT)
#define VADC_BRSCTRL_GTWC_SHIFT                 (23UL)
#define VADC_BRSCTRL_GTWC_MASK                  (0x1 << VADC_BRSCTRL_GTWC_SHIFT)

/* ---------------------------------  VADC_BRSMR  --------------------------------- */
#define VADC_BRSMR_ENGT_SHIFT                   (0UL)
#define VADC_BRSMR_ENGT_MASK                    (0x03 << VADC_BRSMR_ENGT_SHIFT)
#define VADC_BRSMR_ENTR_SHIFT                   (2UL)
#define VADC_BRSMR_ENTR_MASK                    (0x01 << VADC_BRSMR_ENTR_SHIFT)
#define VADC_BRSMR_ENSI_SHIFT                   (3UL)
#define VADC_BRSMR_ENSI_MASK                    (0x01 << VADC_BRSMR_ENSI_SHIFT)
#define VADC_BRSMR_SCAN_SHIFT                   (4UL)
#define VADC_BRSMR_SCAN_MASK                    (0x01 << VADC_BRSMR_SCAN_SHIFT)
#define VADC_BRSMR_LDM_SHIFT                    (5UL)
#define VADC_BRSMR_LDM_MASK                     (0x01 << VADC_BRSMR_LDM_SHIFT)
#define VADC_BRSMR_REQGT_SHIFT                  (7UL)
#define VADC_BRSMR_REQGT_MASK                   (0x01 << VADC_BRSMR_REQGT_SHIFT)
#define VADC_BRSMR_CLRPND_SHIFT                 (8UL)
#define VADC_BRSMR_CLRPND_MASK                  (0x01 << VADC_BRSMR_CLRPND_SHIFT)
#define VADC_BRSMR_LDEV_SHIFT                   (9UL)
#define VADC_BRSMR_LDEV_MASK                    (0x01 << VADC_BRSMR_LDEV_SHIFT)
#define VADC_BRSMR_RPTDIS_SHIFT                 (16UL)
#define VADC_BRSMR_RPTDIS_MASK                  (0x01 << VADC_BRSMR_RPTDIS_SHIFT)

/* --------------------------------  VADC_GLOBRCR  -------------------------------- */
#define VADC_GLOBRCR_DRCTR_MASK                 (0x0f << VADC_GLOBRCR_DRCTR_SHIFT)
#define VADC_GLOBRCR_WFR_SHIFT                  (24UL)
#define VADC_GLOBRCR_WFR_MASK                   (0x01 << VADC_GLOBRCR_WFR_SHIFT)
#define VADC_GLOBRCR_DRCTR_SHIFT                (16UL)
#define VADC_GLOBRCR_SRGEN_SHIFT                (31UL)
#define VADC_GLOBRCR_SRGEN_MASK                 (0x01 << VADC_GLOBRCR_SRGEN_SHIFT)

/* --------------------------------  VADC_GLOBRES  -------------------------------- */
#define VADC_GLOBRES_RESULT_SHIFT               (0UL)
#define VADC_GLOBRES_RESULT_MASK                (0xffff << VADC_GLOBRES_RESULT_SHIFT)
#define VADC_GLOBRES_GNR_SHIFT                  (16UL)
#define VADC_GLOBRES_GNR_MASK                   (0x0f << VADC_GLOBRES_GNR_SHIFT)
#define VADC_GLOBRES_CHNR_SHIFT                 (20UL)
#define VADC_GLOBRES_CHNR_MASK                  (0x1f << VADC_GLOBRES_CHNR_SHIFT)
#define VADC_GLOBRES_EMUX_SHIFT                 (25UL)
#define VADC_GLOBRES_EMUX_MASK                  (0x07 << VADC_GLOBRES_EMUX_SHIFT)
#define VADC_GLOBRES_CRS_SHIFT                  (28UL)
#define VADC_GLOBRES_CRS_MASK                   (0x03 << VADC_GLOBRES_CRS_SHIFT)
#define VADC_GLOBRES_FCR_SHIFT                  (30UL)
#define VADC_GLOBRES_FCR_MASK                   (0x01 << VADC_GLOBRES_FCR_SHIFT)
#define VADC_GLOBRES_VF_SHIFT                   (31UL)
#define VADC_GLOBRES_VF_MASK                    (0x01 << VADC_GLOBRES_VF_SHIFT)

/* --------------------------------  VADC_GLOBRESD  ------------------------------- */
#define VADC_GLOBRESD_RESULT_SHIFT              (0UL)
#define VADC_GLOBRESD_RESULT_MASK               (0xffff << VADC_GLOBRESD_RESULT_SHIFT)
#define VADC_GLOBRESD_GNR_SHIFT                 (16UL)
#define VADC_GLOBRESD_GNR_MASK                  (0x0f << VADC_GLOBRESD_GNR_SHIFT)
#define VADC_GLOBRESD_CHNR_SHIFT                (20UL)
#define VADC_GLOBRESD_CHNR_MASK                 (0x1f << VADC_GLOBRESD_CHNR_SHIFT)
#define VADC_GLOBRESD_EMUX_SHIFT                (25UL)
#define VADC_GLOBRESD_EMUX_MASK                 (0x07 << VADC_GLOBRESD_EMUX_SHIFT)
#define VADC_GLOBRESD_CRS_SHIFT                 (28UL)
#define VADC_GLOBRESD_CRS_MASK                  (0x03 << VADC_GLOBRESD_CRS_SHIFT)
#define VADC_GLOBRESD_FCR_SHIFT                 (30UL)
#define VADC_GLOBRESD_FCR_MASK                  (0x01 << VADC_GLOBRESD_FCR_SHIFT)
#define VADC_GLOBRESD_VF_SHIFT                  (31UL)
#define VADC_GLOBRESD_VF_MASK                   (0x01 << VADC_GLOBRESD_VF_SHIFT)

/* --------------------------------  VADC_EMUXSEL  -------------------------------- */
#define VADC_EMUXSEL_EMUXGRP0_SHIFT             (0UL)
#define VADC_EMUXSEL_EMUXGRP0_MASK              (0xf << VADC_EMUXSEL_EMUXGRP0_SHIFT)
#define VADC_EMUXSEL_EMUXGRP1_SHIFT             (4UL)
#define VADC_EMUXSEL_EMUXGRP1_MASK              (0xf << VADC_EMUXSEL_EMUXGRP1_SHIFT)

/* Group 'VADC_GX' Position & Mask */

/* --------------------------------  VADC_GXARBCFG  ------------------------------- */
#define VADC_GXARBCFG_ANONC_SHIFT               (0UL)
#define VADC_GXARBCFG_ANONC_MASK                (0x03 <<  VADC_GXARBCFG_ANONC_SHIFT)
#define VADC_GXARBCFG_ARBRND_SHIFT              (4UL)
#define VADC_GXARBCFG_ARBRND_MASK               (0x03 << VADC_GXARBCFG_ARBRND_SHIFT)
#define VADC_GXARBCFG_ARBM_SHIFT                (7UL)
#define VADC_GXARBCFG_ARBM_MASK                 (0x01 << VADC_GXARBCFG_ARBM_SHIFT)
#define VADC_GXARBCFG_ANONS_SHIFT               (16UL)
#define VADC_GXARBCFG_ANONS_MASK                (0x03 << VADC_GXARBCFG_ANONS_SHIFT)
#define VADC_GXARBCFG_CAL_SHIFT                 (28UL)
#define VADC_GXARBCFG_CAL_MASK                  (0x01 << VADC_GXARBCFG_CAL_SHIFT)
#define VADC_GXARBCFG_BUSY_SHIFT                (30UL)
#define VADC_GXARBCFG_BUSY_MASK                 (0x01 << VADC_GXARBCFG_BUSY_SHIFT)
#define VADC_GXARBCFG_SAMPLE_SHIFT              (31UL)
#define VADC_GXARBCFG_SAMPLE_MASK               (0x01 << VADC_GXARBCFG_SAMPLE_SHIFT)

/* --------------------------------  VADC_GXARBPR  -------------------------------- */
#define VADC_GXARBPR_PRIO0_SHIFT                (0UL)
#define VADC_GXARBPR_PRIO0_MASK                 (0x03 << VADC_GXARBPR_PRIO0_SHIFT)
#define VADC_GXARBPR_CSM0_SHIFT                 (3UL)
#define VADC_GXARBPR_CSM0_MASK                  (0x01 << VADC_GXARBPR_CSM0_SHIFT)
#define VADC_GXARBPR_PRIO1_SHIFT                (4UL)
#define VADC_GXARBPR_PRIO1_MASK                 (0x03 << VADC_GXARBPR_PRIO1_SHIFT)
#define VADC_GXARBPR_CSM1_SHIFT                 (7UL)
#define VADC_GXARBPR_CSM1_MASK                  (0x01 << VADC_GXARBPR_CSM1_SHIFT)
#define VADC_GXARBPR_PRIO2_SHIFT                (8UL)
#define VADC_GXARBPR_PRIO2_MASK                 (0x03 << VADC_GXARBPR_PRIO2_SHIFT)
#define VADC_GXARBPR_CSM2_SHIFT                 (11UL)
#define VADC_GXARBPR_CSM2_MASK                  (0x01 << VADC_GXARBPR_CSM2_SHIFT)
#define VADC_GXARBPR_ASEN0_SHIFT                (24UL)
#define VADC_GXARBPR_ASEN0_MASK                 (0x01 << VADC_GXARBPR_ASEN0_SHIFT)
#define VADC_GXARBPR_ASEN1_SHIFT                (25UL)
#define VADC_GXARBPR_ASEN1_MASK                 (0x01 << VADC_GXARBPR_ASEN1_SHIFT)
#define VADC_GXARBPR_ASEN2_SHIFT                (26UL)
#define VADC_GXARBPR_ASEN2_MASK                 (0x01 << VADC_GXARBPR_ASEN2_SHIFT)

/* --------------------------------  VADC_GXCHASS  -------------------------------- */
#define VADC_GXCHASS_ASSCH0_SHIFT               (0UL)
#define VADC_GXCHASS_ASSCH0_MASK                (0x01 << VADC_GXCHASS_ASSCH0_SHIFT)
#define VADC_GXCHASS_ASSCH1_SHIFT               (1UL)
#define VADC_GXCHASS_ASSCH1_MASK                (0x01 << VADC_GXCHASS_ASSCH1_SHIFT)
#define VADC_GXCHASS_ASSCH2_SHIFT               (2UL)
#define VADC_GXCHASS_ASSCH2_MASK                (0x01 << VADC_GXCHASS_ASSCH2_SHIFT)
#define VADC_GXCHASS_ASSCH3_SHIFT               (3UL)
#define VADC_GXCHASS_ASSCH3_MASK                (0x01 << VADC_GXCHASS_ASSCH3_SHIFT)
#define VADC_GXCHASS_ASSCH4_SHIFT               (4UL)
#define VADC_GXCHASS_ASSCH4_MASK                (0x01 << VADC_GXCHASS_ASSCH4_SHIFT)
#define VADC_GXCHASS_ASSCH5_SHIFT               (5UL)
#define VADC_GXCHASS_ASSCH5_MASK                (0x01 << VADC_GXCHASS_ASSCH5_SHIFT)
#define VADC_GXCHASS_ASSCH6_SHIFT               (6UL)
#define VADC_GXCHASS_ASSCH6_MASK                (0x01 << VADC_GXCHASS_ASSCH6_SHIFT)
#define VADC_GXCHASS_ASSCH7_SHIFT               (7UL)
#define VADC_GXCHASS_ASSCH7_MASK                (0x01 << VADC_GXCHASS_ASSCH7_SHIFT)

/* --------------------------------  VADC_GXICLASS  ------------------------------- */
#define VADC_GXICLASS_STCS_SHIFT                (0UL)
#define VADC_GXICLASS_STCS_MASK                 (0x1f << VADC_GXICLASS_STCS_SHIFT)
#define VADC_GXICLASS_CMS_SHIFT                 (8UL)
#define VADC_GXICLASS_CMS_MASK                  (0x07 << VADC_GXICLASS_CMS_SHIFT)
#define VADC_GXICLASS_STCE_SHIFT                (16UL)
#define VADC_GXICLASS_STCE_MASK                 (0x1f << VADC_GXICLASS_STCE_SHIFT)
#define VADC_GXICLASS_CME_SHIFT                 (24UL)
#define VADC_GXICLASS_CME_MASK                  (0x07 << VADC_GXICLASS_CME_SHIFT)

/* --------------------------------  VADC_GXALIAS  -------------------------------- */
#define VADC_GXALIAS_ALIAS0_SHIFT               (0UL)
#define VADC_GXALIAS_ALIAS0_MASK                (0x1f << VADC_GXALIAS_ALIAS0_SHIFT)
#define VADC_GXALIAS_ALIAS1_SHIFT               (8UL)
#define VADC_GXALIAS_ALIAS1_MASK                (0x1f << VADC_GXALIAS_ALIAS1_SHIFT)

/* --------------------------------  VADC_GXBOUND  -------------------------------- */
#define VADC_GXBOUND_BOUNDARY0_SHIFT            (0UL)
#define VADC_GXBOUND_BOUNDARY0_MASK             (0xfff << VADC_GXBOUND_BOUNDARY0_SHIFT)
#define VADC_GXBOUND_BOUNDARY1_SHIFT            (16UL)
#define VADC_GXBOUND_BOUNDARY1_MASK             (0xfff << VADC_GXBOUND_BOUNDARY1_SHIFT)

/* --------------------------------  VADC_GXSYNCTR  ------------------------------- */
#define VADC_GXSYNCTR_STSEL_SHIFT               (0UL)
#define VADC_GXSYNCTR_STSEL_MASK                (0x03 << VADC_GXSYNCTR_STSEL_SHIFT)
#define VADC_GXSYNCTR_EVALR1_SHIFT              (4UL)
#define VADC_GXSYNCTR_EVALR1_MASK               (0x01 << VADC_GXSYNCTR_EVALR1_SHIFT)
#define VADC_GXSYNCTR_EVALR2_SHIFT              (5UL)
#define VADC_GXSYNCTR_EVALR2_MASK               (0x01 << VADC_GXSYNCTR_EVALR2_SHIFT)
#define VADC_GXSYNCTR_EVALR3_SHIFT              (6UL)
#define VADC_GXSYNCTR_EVALR3_MASK               (0x01 << VADC_GXSYNCTR_EVALR3_SHIFT)

/* ---------------------------------  VADC_GXBFL  --------------------------------- */
#define VADC_GXBFL_BFL0_SHIFT                   (0UL)
#define VADC_GXBFL_BFL0_MASK                    (0x01 << VADC_GXBFL_BFL0_SHIFT)
#define VADC_GXBFL_BFL1_SHIFT                   (1UL)
#define VADC_GXBFL_BFL1_MASK                    (0x01 << VADC_GXBFL_BFL1_SHIFT)
#define VADC_GXBFL_BFL2_SHIFT                   (2UL)
#define VADC_GXBFL_BFL2_MASK                    (0x01 << VADC_GXBFL_BFL2_SHIFT)
#define VADC_GXBFL_BFL3_SHIFT                   (3UL)
#define VADC_GXBFL_BFL3_MASK                    (0x01 << VADC_GXBFL_BFL3_SHIFT)

#if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
    #define VADC_GXBFL_BFA0_SHIFT               (8UL)
    #define VADC_GXBFL_BFA0_MASK                (0x01 << VADC_GXBFL_BFA0_SHIFT)
    #define VADC_GXBFL_BFA1_SHIFT               (9UL)
    #define VADC_GXBFL_BFA1_MASK                (0x01 << VADC_GXBFL_BFA1_SHIFT)
    #define VADC_GXBFL_BFA2_SHIFT               (10UL)
    #define VADC_GXBFL_BFA2_MASK                (0x01 << VADC_GXBFL_BFA2_SHIFT)
    #define VADC_GXBFL_BFA3_SHIFT               (11UL)
    #define VADC_GXBFL_BFA3_MASK                (0x01 << VADC_GXBFL_BFA3_SHIFT)
    #define VADC_GXBFL_BFI0_SHIFT               (16UL)
    #define VADC_GXBFL_BFI0_MASK                (0x01 << VADC_GXBFL_BFI0_SHIFT)
    #define VADC_GXBFL_BFI1_SHIFT               (17UL)
    #define VADC_GXBFL_BFI1_MASK                (0x01 << VADC_GXBFL_BFI1_SHIFT)
    #define VADC_GXBFL_BFI2_SHIFT               (18UL)
    #define VADC_GXBFL_BFI2_MASK                (0x01 << VADC_GXBFL_BFI2_SHIFT)
    #define VADC_GXBFL_BFI3_SHIFT               (19UL)
    #define VADC_GXBFL_BFI3_MASK                (0x01 << VADC_GXBFL_BFI3_SHIFT)
#endif /* XMC48700 SPECIFIC */

#if defined(CONFIG_ARCH_CHIP_XMC4500)
    #define VADC_GXBFL_BFE0_SHIFT                  (16UL)
    #define VADC_GXBFL_BFE0_MASK                   (0x01 << VADC_GXBFL_BFE0_SHIFT)
    #define VADC_GXBFL_BFE1_SHIFT                  (17UL)
    #define VADC_GXBFL_BFE1_MASK                   (0x01 << VADC_GXBFL_BFE1_SHIFT)
    #define VADC_GXBFL_BFE2_SHIFT                  (18UL)
    #define VADC_GXBFL_BFE2_MASK                   (0x01 << VADC_GXBFL_BFE2_SHIFT)
    #define VADC_GXBFL_BFE3_SHIFT                  (19UL)
    #define VADC_GXBFL_BFE3_MASK                   (0x01 << VADC_GXBFL_BFE3_SHIFT)
#endif /* XMC4500 SPECIFIC */

#if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
    /* ---------------------------------  VADC_GXBFLS  -------------------------------- */
    #define VADC_GXBFLS_BFC0_SHIFT                  (0UL)
    #define VADC_GXBFLS_BFC0_MASK                   (0x01 << VADC_GXBFLS_BFC0_SHIFT)
    #define VADC_GXBFLS_BFC1_SHIFT                  (1UL)
    #define VADC_GXBFLS_BFC1_MASK                   (0x01 << VADC_GXBFLS_BFC1_SHIFT)
    #define VADC_GXBFLS_BFC2_SHIFT                  (2UL)
    #define VADC_GXBFLS_BFC2_MASK                   (0x01 << VADC_GXBFLS_BFC2_SHIFT)
    #define VADC_GXBFLS_BFC3_SHIFT                  (3UL)
    #define VADC_GXBFLS_BFC3_MASK                   (0x01 << VADC_GXBFLS_BFC3_SHIFT)
    #define VADC_GXBFLS_BFS0_SHIFT                  (16UL)
    #define VADC_GXBFLS_BFS0_MASK                   (0x01 << VADC_GXBFLS_BFS0_SHIFT)
    #define VADC_GXBFLS_BFS1_SHIFT                  (17UL)
    #define VADC_GXBFLS_BFS1_MASK                   (0x01 << VADC_GXBFLS_BFS1_SHIFT)
    #define VADC_GXBFLS_BFS2_SHIFT                  (18UL)
    #define VADC_GXBFLS_BFS2_MASK                   (0x01 << VADC_GXBFLS_BFS2_SHIFT)
    #define VADC_GXBFLS_BFS3_SHIFT                  (19UL)
    #define VADC_GXBFLS_BFS3_MASK                   (0x01 << VADC_GXBFLS_BFS3_SHIFT)

    /* ---------------------------------  VADC_GXBFLC  -------------------------------- */
    #define VADC_GXBFLC_BFM0_SHIFT                  (0UL)
    #define VADC_GXBFLC_BFM0_MASK                   (0x0f << VADC_GXBFLC_BFM0_SHIFT)
    #define VADC_GXBFLC_BFM1_SHIFT                  (4UL)
    #define VADC_GXBFLC_BFM1_MASK                   (0x0f << VADC_GXBFLC_BFM1_SHIFT)
    #define VADC_GXBFLC_BFM2_SHIFT                  (8UL)
    #define VADC_GXBFLC_BFM2_MASK                   (0x0f << VADC_GXBFLC_BFM2_SHIFT)
    #define VADC_GXBFLC_BFM3_SHIFT                  (12UL)
    #define VADC_GXBFLC_BFM3_MASK                   (0x0f << VADC_GXBFLC_BFM3_SHIFT)

    /* --------------------------------  VADC_GXBFLNP  -------------------------------- */
    #define VADC_GXBFLNP_BFL0NP_SHIFT               (0UL)
    #define VADC_GXBFLNP_BFL0NP_MASK                (0x0f << VADC_GXBFLNP_BFL0NP_SHIFT)
    #define VADC_GXBFLNP_BFL1NP_SHIFT               (4UL)
    #define VADC_GXBFLNP_BFL1NP_MASK                (0x0f << VADC_GXBFLNP_BFL1NP_SHIFT)
    #define VADC_GXBFLNP_BFL2NP_SHIFT               (8UL)
    #define VADC_GXBFLNP_BFL2NP_MASK                (0x0f << VADC_GXBFLNP_BFL2NP_SHIFT)
    #define VADC_GXBFLNP_BFL3NP_SHIFT               (12UL)
    #define VADC_GXBFLNP_BFL3NP_MASK                (0x0f << VADC_GXBFLNP_BFL3NP_SHIFT)
#endif /* XMC48700 SPECIFIC */

/* --------------------------------  VADC_GXQCTRL0  ------------------------------- */
#if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
    #define VADC_GXQCTRL0_SRCRESREG_SHIFT           (0UL)
    #define VADC_GXQCTRL0_SRCRESREG_MASK            (0x0f << VADC_GXQCTRL0_SRCRESREG_SHIFT)
#endif/* XMC48700 SPECIFIC */

#define VADC_GXQCTRL0_XTSEL_SHIFT               (8UL)
#define VADC_GXQCTRL0_XTSEL_MASK                (0x0f << VADC_GXQCTRL0_XTSEL_SHIFT)
#define VADC_GXQCTRL0_XTLVL_SHIFT               (12UL)
#define VADC_GXQCTRL0_XTLVL_MASK                (0x01 << VADC_GXQCTRL0_XTLVL_SHIFT)
#define VADC_GXQCTRL0_XTMODE_SHIFT              (13UL)
#define VADC_GXQCTRL0_XTMODE_MASK               (0x03 << VADC_GXQCTRL0_XTMODE_SHIFT)
#define VADC_GXQCTRL0_XTWC_SHIFT                (15UL)
#define VADC_GXQCTRL0_XTWC_MASK                 (0x01 << VADC_GXQCTRL0_XTWC_SHIFT)
#define VADC_GXQCTRL0_GTSEL_SHIFT               (16UL)
#define VADC_GXQCTRL0_GTSEL_MASK                (0x0f << VADC_GXQCTRL0_GTSEL_SHIFT)
#define VADC_GXQCTRL0_GTLVL_SHIFT               (20UL)
#define VADC_GXQCTRL0_GTLVL_MASK                (0x01 << VADC_GXQCTRL0_GTLVL_SHIFT)
#define VADC_GXQCTRL0_GTWC_SHIFT                (23UL)
#define VADC_GXQCTRL0_GTWC_MASK                 (0x01 << VADC_GXQCTRL0_GTWC_SHIFT)
#define VADC_GXQCTRL0_TMEN_SHIFT                (28UL)
#define VADC_GXQCTRL0_TMEN_MASK                 (0x01 << VADC_GXQCTRL0_TMEN_SHIFT)
#define VADC_GXQCTRL0_TMWC_SHIFT                (31UL)
#define VADC_GXQCTRL0_TMWC_MASK                 (0x01 << VADC_GXQCTRL0_TMWC_SHIFT)

/* ---------------------------------  VADC_GXQMR0  -------------------------------- */
#define VADC_GXQMR0_ENGT_SHIFT                  (0UL)
#define VADC_GXQMR0_ENGT_MASK                   (0x03 << VADC_GXQMR0_ENGT_SHIFT)
#define VADC_GXQMR0_ENTR_SHIFT                  (2UL)
#define VADC_GXQMR0_ENTR_MASK                   (0x01 << VADC_GXQMR0_ENTR_SHIFT)
#define VADC_GXQMR0_CLRV_SHIFT                  (8UL)
#define VADC_GXQMR0_CLRV_MASK                   (0x01 << VADC_GXQMR0_CLRV_SHIFT)
#define VADC_GXQMR0_TREV_SHIFT                  (9UL)
#define VADC_GXQMR0_TREV_MASK                   (0x01 << VADC_GXQMR0_TREV_SHIFT)
#define VADC_GXQMR0_FLUSH_SHIFT                 (10UL)
#define VADC_GXQMR0_FLUSH_MASK                  (0x01 << VADC_GXQMR0_FLUSH_SHIFT)
#define VADC_GXQMR0_CEV_SHIFT                   (11UL)
#define VADC_GXQMR0_CEV_MASK                    (0x01 << VADC_GXQMR0_CEV_SHIFT)
#define VADC_GXQMR0_RPTDIS_SHIFT                (16UL)
#define VADC_GXQMR0_RPTDIS_MASK                 (0x01 << VADC_GXQMR0_RPTDIS_SHIFT)

/* ---------------------------------  VADC_GXQSR0  -------------------------------- */
#define VADC_GXQSR0_FILL_SHIFT                  (0UL)
#define VADC_GXQSR0_FILL_MASK                   (0x0f << VADC_GXQSR0_FILL_SHIFT)
#define VADC_GXQSR0_EMPTY_SHIFT                 (5UL)
#define VADC_GXQSR0_EMPTY_MASK                  (0x01 << VADC_GXQSR0_EMPTY_SHIFT)
#define VADC_GXQSR0_REQGT_SHIFT                 (7UL)
#define VADC_GXQSR0_REQGT_MASK                  (0x01 << VADC_GXQSR0_REQGT_SHIFT)
#define VADC_GXQSR0_EV_SHIFT                    (8UL)
#define VADC_GXQSR0_EV_MASK                     (0x01 << VADC_GXQSR0_EV_SHIFT)

/* ---------------------------------  VADC_GXQ0R0  -------------------------------- */
#define VADC_GXQ0R0_REQCHNR_SHIFT               (0UL)
#define VADC_GXQ0R0_REQCHNR_MASK                (0x1f << VADC_GXQ0R0_REQCHNR_SHIFT)
#define VADC_GXQ0R0_RF_SHIFT                    (5UL)
#define VADC_GXQ0R0_RF_MASK                     (0x01 << VADC_GXQ0R0_RF_SHIFT)
#define VADC_GXQ0R0_ENSI_SHIFT                  (6UL)
#define VADC_GXQ0R0_ENSI_MASK                   (0x01 << VADC_GXQ0R0_ENSI_SHIFT)
#define VADC_GXQ0R0_EXTR_SHIFT                  (7UL)
#define VADC_GXQ0R0_EXTR_MASK                   (0x01 << VADC_GXQ0R0_EXTR_SHIFT)
#define VADC_GXQ0R0_V_SHIFT                     (8UL)
#define VADC_GXQ0R0_V_MASK                      (0x01 << VADC_GXQ0R0_V_SHIFT)

/* --------------------------------  VADC_GXQINR0  -------------------------------- */
#define VADC_GXQINR0_REQCHNR_SHIFT              (0UL)
#define VADC_GXQINR0_REQCHNR_MASK               (0x1f << VADC_GXQINR0_REQCHNR_SHIFT)
#define VADC_GXQINR0_RF_SHIFT                   (5UL)
#define VADC_GXQINR0_RF_MASK                    (0x01 << VADC_GXQINR0_RF_SHIFT)
#define VADC_GXQINR0_ENSI_SHIFT                 (6UL)
#define VADC_GXQINR0_ENSI_MASK                  (0x01 << VADC_GXQINR0_ENSI_SHIFT)
#define VADC_GXQINR0_EXTR_SHIFT                 (7UL)
#define VADC_GXQINR0_EXTR_MASK                  (0x01 << VADC_GXQINR0_EXTR_SHIFT)

/* --------------------------------  VADC_GXQBUR0  -------------------------------- */
#define VADC_GXQBUR0_REQCHNR_SHIFT              (0UL)
#define VADC_GXQBUR0_REQCHNR_MASK               (0x1f << VADC_GXQBUR0_REQCHNR_SHIFT)
#define VADC_GXQBUR0_RF_SHIFT                   (5UL)
#define VADC_GXQBUR0_RF_MASK                    (0x01 << VADC_GXQBUR0_RF_SHIFT)
#define VADC_GXQBUR0_ENSI_SHIFT                 (6UL)
#define VADC_GXQBUR0_ENSI_MASK                  (0x01 << VADC_GXQBUR0_ENSI_SHIFT)
#define VADC_GXQBUR0_EXTR_SHIFT                 (7UL)
#define VADC_GXQBUR0_EXTR_MASK                  (0x01 << VADC_GXQBUR0_EXTR_SHIFT)
#define VADC_GXQBUR0_V_SHIFT                    (8UL)
#define VADC_GXQBUR0_V_MASK                     (0x01 << VADC_GXQBUR0_V_SHIFT)

/* --------------------------------  VADC_GXASCTRL  ------------------------------- */
#if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
    #define VADC_GXASCTRL_SRCRESREG_SHIFT       (0UL)
    #define VADC_GXASCTRL_SRCRESREG_MASK        (0x0f << VADC_GXASCTRL_SRCRESREG_SHIFT)
#endif /* XMC48700 SPECIFIC */

#define VADC_GXASCTRL_XTSEL_SHIFT               (8UL)
#define VADC_GXASCTRL_XTSEL_MASK                (0x0f << VADC_GXASCTRL_XTSEL_SHIFT)
#define VADC_GXASCTRL_XTLVL_SHIFT               (12UL)
#define VADC_GXASCTRL_XTLVL_MASK                (0x01 << VADC_GXASCTRL_XTLVL_SHIFT)
#define VADC_GXASCTRL_XTMODE_SHIFT              (13UL)
#define VADC_GXASCTRL_XTMODE_MASK               (0x03 << VADC_GXASCTRL_XTMODE_SHIFT)
#define VADC_GXASCTRL_XTWC_SHIFT                (15UL)
#define VADC_GXASCTRL_XTWC_MASK                 (0x01 << VADC_GXASCTRL_XTWC_SHIFT)
#define VADC_GXASCTRL_GTSEL_SHIFT               (16UL)
#define VADC_GXASCTRL_GTSEL_MASK                (0x0f << VADC_GXASCTRL_GTSEL_SHIFT)
#define VADC_GXASCTRL_GTLVL_SHIFT               (20UL)
#define VADC_GXASCTRL_GTLVL_MASK                (0x01 << VADC_GXASCTRL_GTLVL_SHIFT)
#define VADC_GXASCTRL_GTWC_SHIFT                (23UL)
#define VADC_GXASCTRL_GTWC_MASK                 (0x01 << VADC_GXASCTRL_GTWC_SHIFT)
#define VADC_GXASCTRL_TMEN_SHIFT                (28UL)
#define VADC_GXASCTRL_TMEN_MASK                 (0x01 << VADC_GXASCTRL_TMEN_SHIFT)
#define VADC_GXASCTRL_TMWC_SHIFT                (31UL)
#define VADC_GXASCTRL_TMWC_MASK                 (0x01 << VADC_GXASCTRL_TMWC_SHIFT)

/* ---------------------------------  VADC_GXASMR  -------------------------------- */
#define VADC_GXASMR_ENGT_SHIFT                  (0UL)
#define VADC_GXASMR_ENGT_MASK                   (0x03 << VADC_GXASMR_ENGT_SHIFT)
#define VADC_GXASMR_ENTR_SHIFT                  (2UL)
#define VADC_GXASMR_ENTR_MASK                   (0x01 << VADC_GXASMR_ENTR_SHIFT)
#define VADC_GXASMR_ENSI_SHIFT                  (3UL)
#define VADC_GXASMR_ENSI_MASK                   (0x01 << VADC_GXASMR_ENSI_SHIFT)
#define VADC_GXASMR_SCAN_SHIFT                  (4UL)
#define VADC_GXASMR_SCAN_MASK                   (0x01 << VADC_GXASMR_SCAN_SHIFT)
#define VADC_GXASMR_LDM_SHIFT                   (5UL)
#define VADC_GXASMR_LDM_MASK                    (0x01 << VADC_GXASMR_LDM_SHIFT)
#define VADC_GXASMR_REQGT_SHIFT                 (7UL)
#define VADC_GXASMR_REQGT_MASK                  (0x01 << VADC_GXASMR_REQGT_SHIFT)
#define VADC_GXASMR_CLRPND_SHIFT                (8UL)
#define VADC_GXASMR_CLRPND_MASK                 (0x01 << VADC_GXASMR_CLRPND_SHIFT)
#define VADC_GXASMR_LDEV_SHIFT                  (9UL)
#define VADC_GXASMR_LDEV_MASK                   (0x01 << VADC_GXASMR_LDEV_SHIFT)
#define VADC_GXASMR_RPTDIS_SHIFT                (16UL)
#define VADC_GXASMR_RPTDIS_MASK                 (0x01 << VADC_GXASMR_RPTDIS_SHIFT)

/* --------------------------------  VADC_GXASSEL  -------------------------------- */
#define VADC_GXASSEL_CHSEL0_SHIFT               (0UL)
#define VADC_GXASSEL_CHSEL0_MASK                (0x01 << VADC_GXASSEL_CHSEL0_SHIFT)
#define VADC_GXASSEL_CHSEL1_SHIFT               (1UL)
#define VADC_GXASSEL_CHSEL1_MASK                (0x01 << VADC_GXASSEL_CHSEL1_SHIFT)
#define VADC_GXASSEL_CHSEL2_SHIFT               (2UL)
#define VADC_GXASSEL_CHSEL2_MASK                (0x01 << VADC_GXASSEL_CHSEL2_SHIFT)
#define VADC_GXASSEL_CHSEL3_SHIFT               (3UL)
#define VADC_GXASSEL_CHSEL3_MASK                (0x01 << VADC_GXASSEL_CHSEL3_SHIFT)
#define VADC_GXASSEL_CHSEL4_SHIFT               (4UL)
#define VADC_GXASSEL_CHSEL4_MASK                (0x01 << VADC_GXASSEL_CHSEL4_SHIFT)
#define VADC_GXASSEL_CHSEL5_SHIFT               (5UL)
#define VADC_GXASSEL_CHSEL5_MASK                (0x01 << VADC_GXASSEL_CHSEL5_SHIFT)
#define VADC_GXASSEL_CHSEL6_SHIFT               (6UL)
#define VADC_GXASSEL_CHSEL6_MASK                (0x01 << VADC_GXASSEL_CHSEL6_SHIFT)
#define VADC_GXASSEL_CHSEL7_SHIFT               (7UL)
#define VADC_GXASSEL_CHSEL7_MASK                (0x01 << VADC_GXASSEL_CHSEL7_SHIFT)

/* --------------------------------  VADC_GXASPND  -------------------------------- */
#define VADC_GXASPND_CHPND0_SHIFT               (0UL)
#define VADC_GXASPND_CHPND0_MASK                (0x01 << VADC_GXASPND_CHPND0_SHIFT)
#define VADC_GXASPND_CHPND1_SHIFT               (1UL)
#define VADC_GXASPND_CHPND1_MASK                (0x01 << VADC_GXASPND_CHPND1_SHIFT)
#define VADC_GXASPND_CHPND2_SHIFT               (2UL)
#define VADC_GXASPND_CHPND2_MASK                (0x01 << VADC_GXASPND_CHPND2_SHIFT)
#define VADC_GXASPND_CHPND3_SHIFT               (3UL)
#define VADC_GXASPND_CHPND3_MASK                (0x01 << VADC_GXASPND_CHPND3_SHIFT)
#define VADC_GXASPND_CHPND4_SHIFT               (4UL)
#define VADC_GXASPND_CHPND4_MASK                (0x01 << VADC_GXASPND_CHPND4_SHIFT)
#define VADC_GXASPND_CHPND5_SHIFT               (5UL)
#define VADC_GXASPND_CHPND5_MASK                (0x01 << VADC_GXASPND_CHPND5_SHIFT)
#define VADC_GXASPND_CHPND6_SHIFT               (6UL)
#define VADC_GXASPND_CHPND6_MASK                (0x01 << VADC_GXASPND_CHPND6_SHIFT)
#define VADC_GXASPND_CHPND7_SHIFT               (7UL)
#define VADC_GXASPND_CHPND7_MASK                (0x01 << VADC_GXASPND_CHPND7_SHIFT)

/* --------------------------------  VADC_GXCEFLAG  ------------------------------- */
#define VADC_GXCEFLAG_CEV0_SHIFT                (0UL)
#define VADC_GXCEFLAG_CEV0_MASK                 (0x01 << VADC_GXCEFLAG_CEV0_SHIFT)
#define VADC_GXCEFLAG_CEV1_SHIFT                (1UL)
#define VADC_GXCEFLAG_CEV1_MASK                 (0x01 << VADC_GXCEFLAG_CEV1_SHIFT)
#define VADC_GXCEFLAG_CEV2_SHIFT                (2UL)
#define VADC_GXCEFLAG_CEV2_MASK                 (0x01 << VADC_GXCEFLAG_CEV2_SHIFT)
#define VADC_GXCEFLAG_CEV3_SHIFT                (3UL)
#define VADC_GXCEFLAG_CEV3_MASK                 (0x01 << VADC_GXCEFLAG_CEV3_SHIFT)
#define VADC_GXCEFLAG_CEV4_SHIFT                (4UL)
#define VADC_GXCEFLAG_CEV4_MASK                 (0x01 << VADC_GXCEFLAG_CEV4_SHIFT)
#define VADC_GXCEFLAG_CEV5_SHIFT                (5UL)
#define VADC_GXCEFLAG_CEV5_MASK                 (0x01 << VADC_GXCEFLAG_CEV5_SHIFT)
#define VADC_GXCEFLAG_CEV6_SHIFT                (6UL)
#define VADC_GXCEFLAG_CEV6_MASK                 (0x01 << VADC_GXCEFLAG_CEV6_SHIFT)
#define VADC_GXCEFLAG_CEV7_SHIFT                (7UL)
#define VADC_GXCEFLAG_CEV7_MASK                 (0x01 << VADC_GXCEFLAG_CEV7_SHIFT)

/* --------------------------------  VADC_GXREFLAG  ------------------------------- */
#define VADC_GXREFLAG_REV0_SHIFT                (0UL)
#define VADC_GXREFLAG_REV0_MASK                 (0x01 << VADC_GXREFLAG_REV0_SHIFT)
#define VADC_GXREFLAG_REV1_SHIFT                (1UL)
#define VADC_GXREFLAG_REV1_MASK                 (0x01 << VADC_GXREFLAG_REV1_SHIFT)
#define VADC_GXREFLAG_REV2_SHIFT                (2UL)
#define VADC_GXREFLAG_REV2_MASK                 (0x01 << VADC_GXREFLAG_REV2_SHIFT)
#define VADC_GXREFLAG_REV3_SHIFT                (3UL)
#define VADC_GXREFLAG_REV3_MASK                 (0x01 << VADC_GXREFLAG_REV3_SHIFT)
#define VADC_GXREFLAG_REV4_SHIFT                (4UL)
#define VADC_GXREFLAG_REV4_MASK                 (0x01 << VADC_GXREFLAG_REV4_SHIFT)
#define VADC_GXREFLAG_REV5_SHIFT                (5UL)
#define VADC_GXREFLAG_REV5_MASK                 (0x01 << VADC_GXREFLAG_REV5_SHIFT)
#define VADC_GXREFLAG_REV6_SHIFT                (6UL)
#define VADC_GXREFLAG_REV6_MASK                 (0x01 << VADC_GXREFLAG_REV6_SHIFT)
#define VADC_GXREFLAG_REV7_SHIFT                (7UL)
#define VADC_GXREFLAG_REV7_MASK                 (0x01 << VADC_GXREFLAG_REV7_SHIFT)
#define VADC_GXREFLAG_REV8_SHIFT                (8UL)
#define VADC_GXREFLAG_REV8_MASK                 (0x01 << VADC_GXREFLAG_REV8_SHIFT)
#define VADC_GXREFLAG_REV9_SHIFT                (9UL)
#define VADC_GXREFLAG_REV9_MASK                 (0x01 << VADC_GXREFLAG_REV9_SHIFT)
#define VADC_GXREFLAG_REV10_SHIFT               (10UL)
#define VADC_GXREFLAG_REV10_MASK                (0x01 << VADC_GXREFLAG_REV10_SHIFT)
#define VADC_GXREFLAG_REV11_SHIFT               (11UL)
#define VADC_GXREFLAG_REV11_MASK                (0x01 << VADC_GXREFLAG_REV11_SHIFT)
#define VADC_GXREFLAG_REV12_SHIFT               (12UL)
#define VADC_GXREFLAG_REV12_MASK                (0x01 << VADC_GXREFLAG_REV12_SHIFT)
#define VADC_GXREFLAG_REV13_SHIFT               (13UL)
#define VADC_GXREFLAG_REV13_MASK                (0x01 << VADC_GXREFLAG_REV13_SHIFT)
#define VADC_GXREFLAG_REV14_SHIFT               (14UL)
#define VADC_GXREFLAG_REV14_MASK                (0x01 << VADC_GXREFLAG_REV14_SHIFT)
#define VADC_GXREFLAG_REV15_SHIFT               (15UL)
#define VADC_GXREFLAG_REV15_MASK                (0x01 << VADC_GXREFLAG_REV15_SHIFT)

/* --------------------------------  VADC_GXSEFLAG  ------------------------------- */
#define VADC_GXSEFLAG_SEV0_SHIFT                (0UL)
#define VADC_GXSEFLAG_SEV0_MASK                 (0x01 << VADC_GXSEFLAG_SEV0_SHIFT)
#define VADC_GXSEFLAG_SEV1_SHIFT                (1UL)
#define VADC_GXSEFLAG_SEV1_MASK                 (0x01 << VADC_GXSEFLAG_SEV1_SHIFT)

/* --------------------------------  VADC_GXCEFCLR  ------------------------------- */
#define VADC_GXCEFCLR_CEV0_SHIFT                (0UL)
#define VADC_GXCEFCLR_CEV0_MASK                 (0x01 << VADC_GXCEFCLR_CEV0_SHIFT)
#define VADC_GXCEFCLR_CEV1_SHIFT                (1UL)
#define VADC_GXCEFCLR_CEV1_MASK                 (0x01 << VADC_GXCEFCLR_CEV1_SHIFT)
#define VADC_GXCEFCLR_CEV2_SHIFT                (2UL)
#define VADC_GXCEFCLR_CEV2_MASK                 (0x01 << VADC_GXCEFCLR_CEV2_SHIFT)
#define VADC_GXCEFCLR_CEV3_SHIFT                (3UL)
#define VADC_GXCEFCLR_CEV3_MASK                 (0x01 << VADC_GXCEFCLR_CEV3_SHIFT)
#define VADC_GXCEFCLR_CEV4_SHIFT                (4UL)
#define VADC_GXCEFCLR_CEV4_MASK                 (0x01 << VADC_GXCEFCLR_CEV4_SHIFT)
#define VADC_GXCEFCLR_CEV5_SHIFT                (5UL)
#define VADC_GXCEFCLR_CEV5_MASK                 (0x01 << VADC_GXCEFCLR_CEV5_SHIFT)
#define VADC_GXCEFCLR_CEV6_SHIFT                (6UL)
#define VADC_GXCEFCLR_CEV6_MASK                 (0x01 << VADC_GXCEFCLR_CEV6_SHIFT)
#define VADC_GXCEFCLR_CEV7_SHIFT                (7UL)
#define VADC_GXCEFCLR_CEV7_MASK                 (0x01 << VADC_GXCEFCLR_CEV7_SHIFT)

/* --------------------------------  VADC_GXREFCLR  ------------------------------- */
#define VADC_GXREFCLR_REV0_SHIFT                (0UL)
#define VADC_GXREFCLR_REV0_MASK                 (0x01 << VADC_GXREFCLR_REV0_SHIFT)
#define VADC_GXREFCLR_REV1_SHIFT                (1UL)
#define VADC_GXREFCLR_REV1_MASK                 (0x01 << VADC_GXREFCLR_REV1_SHIFT)
#define VADC_GXREFCLR_REV2_SHIFT                (2UL)
#define VADC_GXREFCLR_REV2_MASK                 (0x01 << VADC_GXREFCLR_REV2_SHIFT)
#define VADC_GXREFCLR_REV3_SHIFT                (3UL)
#define VADC_GXREFCLR_REV3_MASK                 (0x01 << VADC_GXREFCLR_REV3_SHIFT)
#define VADC_GXREFCLR_REV4_SHIFT                (4UL)
#define VADC_GXREFCLR_REV4_MASK                 (0x01 << VADC_GXREFCLR_REV4_SHIFT)
#define VADC_GXREFCLR_REV5_SHIFT                (5UL)
#define VADC_GXREFCLR_REV5_MASK                 (0x01 << VADC_GXREFCLR_REV5_SHIFT)
#define VADC_GXREFCLR_REV6_SHIFT                (6UL)
#define VADC_GXREFCLR_REV6_MASK                 (0x01 << VADC_GXREFCLR_REV6_SHIFT)
#define VADC_GXREFCLR_REV7_SHIFT                (7UL)
#define VADC_GXREFCLR_REV7_MASK                 (0x01 << VADC_GXREFCLR_REV7_SHIFT)
#define VADC_GXREFCLR_REV8_SHIFT                (8UL)
#define VADC_GXREFCLR_REV8_MASK                 (0x01 << VADC_GXREFCLR_REV8_SHIFT)
#define VADC_GXREFCLR_REV9_SHIFT                (9UL)
#define VADC_GXREFCLR_REV9_MASK                 (0x01 << VADC_GXREFCLR_REV9_SHIFT)
#define VADC_GXREFCLR_REV10_SHIFT               (10UL)
#define VADC_GXREFCLR_REV10_MASK                (0x01 << VADC_GXREFCLR_REV10_SHIFT)
#define VADC_GXREFCLR_REV11_SHIFT               (11UL)
#define VADC_GXREFCLR_REV11_MASK                (0x01 << VADC_GXREFCLR_REV11_SHIFT)
#define VADC_GXREFCLR_REV12_SHIFT               (12UL)
#define VADC_GXREFCLR_REV12_MASK                (0x01 << VADC_GXREFCLR_REV12_SHIFT)
#define VADC_GXREFCLR_REV13_SHIFT               (13UL)
#define VADC_GXREFCLR_REV13_MASK                (0x01 << VADC_GXREFCLR_REV13_SHIFT)
#define VADC_GXREFCLR_REV14_SHIFT               (14UL)
#define VADC_GXREFCLR_REV14_MASK                (0x01 << VADC_GXREFCLR_REV14_SHIFT)
#define VADC_GXREFCLR_REV15_SHIFT               (15UL)
#define VADC_GXREFCLR_REV15_MASK                (0x01 << VADC_GXREFCLR_REV15_SHIFT)

/* --------------------------------  VADC_GXSEFCLR  ------------------------------- */
#define VADC_GXSEFCLR_SEV0_SHIFT                (0UL)
#define VADC_GXSEFCLR_SEV0_MASK                 (0x01 << VADC_GXSEFCLR_SEV0_SHIFT)
#define VADC_GXSEFCLR_SEV1_SHIFT                (1UL)
#define VADC_GXSEFCLR_SEV1_MASK                 (0x01 << VADC_GXSEFCLR_SEV1_SHIFT)

/* --------------------------------  VADC_GXCEVNP0  ------------------------------- */
#define VADC_GXCEVNP0_CEV0NP_SHIFT              (0UL)
#define VADC_GXCEVNP0_CEV0NP_MASK               (0x0f << VADC_GXCEVNP0_CEV0NP_SHIFT)
#define VADC_GXCEVNP0_CEV1NP_SHIFT              (4UL)
#define VADC_GXCEVNP0_CEV1NP_MASK               (0x0f << VADC_GXCEVNP0_CEV1NP_SHIFT)
#define VADC_GXCEVNP0_CEV2NP_SHIFT              (8UL)
#define VADC_GXCEVNP0_CEV2NP_MASK               (0x0f << VADC_GXCEVNP0_CEV2NP_SHIFT)
#define VADC_GXCEVNP0_CEV3NP_SHIFT              (12UL)
#define VADC_GXCEVNP0_CEV3NP_MASK               (0x0f << VADC_GXCEVNP0_CEV3NP_SHIFT)
#define VADC_GXCEVNP0_CEV4NP_SHIFT              (16UL)
#define VADC_GXCEVNP0_CEV4NP_MASK               (0x0f << VADC_GXCEVNP0_CEV4NP_SHIFT)
#define VADC_GXCEVNP0_CEV5NP_SHIFT              (20UL)
#define VADC_GXCEVNP0_CEV5NP_MASK               (0x0f << VADC_GXCEVNP0_CEV5NP_SHIFT)
#define VADC_GXCEVNP0_CEV6NP_SHIFT              (24UL)
#define VADC_GXCEVNP0_CEV6NP_MASK               (0x0f << VADC_GXCEVNP0_CEV6NP_SHIFT)
#define VADC_GXCEVNP0_CEV7NP_SHIFT              (28UL)
#define VADC_GXCEVNP0_CEV7NP_MASK               (0x0f << VADC_GXCEVNP0_CEV7NP_SHIFT)

/* --------------------------------  VADC_GXREVNP0  ------------------------------- */
#define VADC_GXREVNP0_REV0NP_SHIFT              (0UL)
#define VADC_GXREVNP0_REV0NP_MASK               (0x0f << VADC_GXREVNP0_REV0NP_SHIFT)
#define VADC_GXREVNP0_REV1NP_SHIFT              (4UL)
#define VADC_GXREVNP0_REV1NP_MASK               (0x0f << VADC_GXREVNP0_REV1NP_SHIFT)
#define VADC_GXREVNP0_REV2NP_SHIFT              (8UL)
#define VADC_GXREVNP0_REV2NP_MASK               (0x0f << VADC_GXREVNP0_REV2NP_SHIFT)
#define VADC_GXREVNP0_REV3NP_SHIFT              (12UL)
#define VADC_GXREVNP0_REV3NP_MASK               (0x0f << VADC_GXREVNP0_REV3NP_SHIFT)
#define VADC_GXREVNP0_REV4NP_SHIFT              (16UL)
#define VADC_GXREVNP0_REV4NP_MASK               (0x0f << VADC_GXREVNP0_REV4NP_SHIFT)
#define VADC_GXREVNP0_REV5NP_SHIFT              (20UL)
#define VADC_GXREVNP0_REV5NP_MASK               (0x0f << VADC_GXREVNP0_REV5NP_SHIFT)
#define VADC_GXREVNP0_REV6NP_SHIFT              (24UL)
#define VADC_GXREVNP0_REV6NP_MASK               (0x0f << VADC_GXREVNP0_REV6NP_SHIFT)
#define VADC_GXREVNP0_REV7NP_SHIFT              (28UL)
#define VADC_GXREVNP0_REV7NP_MASK               (0x0f << VADC_GXREVNP0_REV7NP_SHIFT)

/* --------------------------------  VADC_GXREVNP1  ------------------------------- */
#define VADC_GXREVNP1_REV8NP_SHIFT              (0UL)
#define VADC_GXREVNP1_REV8NP_MASK               (0x0f << VADC_GXREVNP1_REV8NP_SHIFT)
#define VADC_GXREVNP1_REV9NP_SHIFT              (4UL)
#define VADC_GXREVNP1_REV9NP_MASK               (0x0f << VADC_GXREVNP1_REV9NP_SHIFT)
#define VADC_GXREVNP1_REV10NP_SHIFT             (8UL)
#define VADC_GXREVNP1_REV10NP_MASK              (0x0f << VADC_GXREVNP1_REV10NP_SHIFT)
#define VADC_GXREVNP1_REV11NP_SHIFT             (12UL)
#define VADC_GXREVNP1_REV11NP_MASK              (0x0f << VADC_GXREVNP1_REV11NP_SHIFT)
#define VADC_GXREVNP1_REV12NP_SHIFT             (16UL)
#define VADC_GXREVNP1_REV12NP_MASK              (0x0f << VADC_GXREVNP1_REV12NP_SHIFT)
#define VADC_GXREVNP1_REV13NP_SHIFT             (20UL)
#define VADC_GXREVNP1_REV13NP_MASK              (0x0f << VADC_GXREVNP1_REV13NP_SHIFT)
#define VADC_GXREVNP1_REV14NP_SHIFT             (24UL)
#define VADC_GXREVNP1_REV14NP_MASK              (0x0f << VADC_GXREVNP1_REV14NP_SHIFT)
#define VADC_GXREVNP1_REV15NP_SHIFT             (28UL)
#define VADC_GXREVNP1_REV15NP_MASK              (0x0f << VADC_GXREVNP1_REV15NP_SHIFT)

/* --------------------------------  VADC_GXSEVNP  -------------------------------- */
#define VADC_GXSEVNP_SEV0NP_SHIFT               (0UL)
#define VADC_GXSEVNP_SEV0NP_MASK                (0x0f << VADC_GXSEVNP_SEV0NP_SHIFT)
#define VADC_GXSEVNP_SEV1NP_SHIFT               (4UL)
#define VADC_GXSEVNP_SEV1NP_MASK                (0x0f << VADC_GXSEVNP_SEV1NP_SHIFT)

/* --------------------------------  VADC_GXSRACT  -------------------------------- */
#define VADC_GXSRACT_AGSR0_SHIFT                (0UL)
#define VADC_GXSRACT_AGSR0_MASK                 (0x01 << VADC_GXSRACT_AGSR0_SHIFT)
#define VADC_GXSRACT_AGSR1_SHIFT                (1UL)
#define VADC_GXSRACT_AGSR1_MASK                 (0x01 << VADC_GXSRACT_AGSR1_SHIFT)
#define VADC_GXSRACT_AGSR2_SHIFT                (2UL)
#define VADC_GXSRACT_AGSR2_MASK                 (0x01 << VADC_GXSRACT_AGSR2_SHIFT)
#define VADC_GXSRACT_AGSR3_SHIFT                (3UL)
#define VADC_GXSRACT_AGSR3_MASK                 (0x01 << VADC_GXSRACT_AGSR3_SHIFT)
#define VADC_GXSRACT_ASSR0_SHIFT                (8UL)
#define VADC_GXSRACT_ASSR0_MASK                 (0x01 << VADC_GXSRACT_ASSR0_SHIFT)
#define VADC_GXSRACT_ASSR1_SHIFT                (9UL)
#define VADC_GXSRACT_ASSR1_MASK                 (0x01 << VADC_GXSRACT_ASSR1_SHIFT)
#define VADC_GXSRACT_ASSR2_SHIFT                (10UL)
#define VADC_GXSRACT_ASSR2_MASK                 (0x01 << VADC_GXSRACT_ASSR2_SHIFT)
#define VADC_GXSRACT_ASSR3_SHIFT                (11UL)
#define VADC_GXSRACT_ASSR3_MASK                 (0x01 << VADC_GXSRACT_ASSR3_SHIFT)

/* -------------------------------  VADC_GXEMUXCTR  ------------------------------- */
#define VADC_GXEMUXCTR_EMUXSET_SHIFT            (0UL)
#define VADC_GXEMUXCTR_EMUXSET_MASK             (0x07 << VADC_GXEMUXCTR_EMUXSET_SHIFT)
#define VADC_GXEMUXCTR_EMUXACT_SHIFT            (8UL)
#define VADC_GXEMUXCTR_EMUXACT_MASK             (0x07 << VADC_GXEMUXCTR_EMUXSET_SHIFT)
#define VADC_GXEMUXCTR_EMUXCH_SHIFT             (16UL)

#if defined(CONFIG_ARCH_CHIP_XMC4700) || defined(CONFIG_ARCH_CHIP_XMC4800)
    #define VADC_GXEMUXCTR_EMUXCH_MASK          (0x3ff << VADC_GXEMUXCTR_EMUXCH_SHIFT)
#endif /* XMC48700 SPECIFIC */

#if defined(CONFIG_ARCH_CHIP_XMC4500)
    #define VADC_GXEMUXCTR_EMUXCH_MASK          (0x1f << VADC_GXEMUXCTR_EMUXCH_SHIFT)
#endif /* XMC4500 SPECIFIC */

#define VADC_GXEMUXCTR_EMUXMODE_SHIFT           (26UL)
#define VADC_GXEMUXCTR_EMUXMODE_MASK            (0x03 << VADC_GXEMUXCTR_EMUXMODE_SHIFT)
#define VADC_GXEMUXCTR_EMXCOD_SHIFT             (28UL)
#define VADC_GXEMUXCTR_EMXCOD_MASK              (0x01 << VADC_GXEMUXCTR_EMXCOD_SHIFT)
#define VADC_GXEMUXCTR_EMXST_SHIFT              (29UL)
#define VADC_GXEMUXCTR_EMXST_MASK               (0x01 << VADC_GXEMUXCTR_EMXST_SHIFT)
#define VADC_GXEMUXCTR_EMXCSS_SHIFT             (30UL)
#define VADC_GXEMUXCTR_EMXCSS_MASK              (0x01 << VADC_GXEMUXCTR_EMXCSS_SHIFT)
#define VADC_GXEMUXCTR_EMXWC_SHIFT              (31UL)
#define VADC_GXEMUXCTR_EMXWC_MASK               (0x01 << VADC_GXEMUXCTR_EMXWC_SHIFT)

/* ---------------------------------  VADC_GXVFR  --------------------------------- */
#define VADC_GXVFR_VF0_SHIFT                    (0UL)
#define VADC_GXVFR_VF0_MASK                     (0x01 << VADC_GXVFR_VF0_SHIFT)
#define VADC_GXVFR_VF1_SHIFT                    (1UL)
#define VADC_GXVFR_VF1_MASK                     (0x01 << VADC_GXVFR_VF1_SHIFT)
#define VADC_GXVFR_VF2_SHIFT                    (2UL)
#define VADC_GXVFR_VF2_MASK                     (0x01 << VADC_GXVFR_VF2_SHIFT)
#define VADC_GXVFR_VF3_SHIFT                    (3UL)
#define VADC_GXVFR_VF3_MASK                     (0x01 << VADC_GXVFR_VF3_SHIFT)
#define VADC_GXVFR_VF4_SHIFT                    (4UL)
#define VADC_GXVFR_VF4_MASK                     (0x01 << VADC_GXVFR_VF4_SHIFT)
#define VADC_GXVFR_VF5_SHIFT                    (5UL)
#define VADC_GXVFR_VF5_MASK                     (0x01 << VADC_GXVFR_VF5_SHIFT)
#define VADC_GXVFR_VF6_SHIFT                    (6UL)
#define VADC_GXVFR_VF6_MASK                     (0x01 << VADC_GXVFR_VF6_SHIFT)
#define VADC_GXVFR_VF7_SHIFT                    (7UL)
#define VADC_GXVFR_VF7_MASK                     (0x01 << VADC_GXVFR_VF7_SHIFT)
#define VADC_GXVFR_VF8_SHIFT                    (8UL)
#define VADC_GXVFR_VF8_MASK                     (0x01 << VADC_GXVFR_VF8_SHIFT)
#define VADC_GXVFR_VF9_SHIFT                    (9UL)
#define VADC_GXVFR_VF9_MASK                     (0x01 << VADC_GXVFR_VF9_SHIFT)
#define VADC_GXVFR_VF10_SHIFT                   (10UL)
#define VADC_GXVFR_VF10_MASK                    (0x01 << VADC_GXVFR_VF10_SHIFT)
#define VADC_GXVFR_VF11_SHIFT                   (11UL)
#define VADC_GXVFR_VF11_MASK                    (0x01 << VADC_GXVFR_VF11_SHIFT)
#define VADC_GXVFR_VF12_SHIFT                   (12UL)
#define VADC_GXVFR_VF12_MASK                    (0x01 << VADC_GXVFR_VF12_SHIFT)
#define VADC_GXVFR_VF13_SHIFT                   (13UL)
#define VADC_GXVFR_VF13_MASK                    (0x01 << VADC_GXVFR_VF13_SHIFT)
#define VADC_GXVFR_VF14_SHIFT                   (14UL)
#define VADC_GXVFR_VF14_MASK                    (0x01 << VADC_GXVFR_VF14_SHIFT)
#define VADC_GXVFR_VF15_SHIFT                   (15UL)
#define VADC_GXVFR_VF15_MASK                    (0x01 << VADC_GXVFR_VF15_SHIFT)

/* --------------------------------  VADC_GXCHCTR  -------------------------------- */
#define VADC_GXCHCTR_ICLSEL_SHIFT               (0UL)
#define VADC_GXCHCTR_ICLSEL_MASK                (0x03 << VADC_GXCHCTR_ICLSEL_SHIFT)
#define VADC_GXCHCTR_BNDSELL_SHIFT              (4UL)
#define VADC_GXCHCTR_BNDSELL_MASK               (0x03 << VADC_GXCHCTR_BNDSELL_SHIFT)
#define VADC_GXCHCTR_BNDSELU_SHIFT              (6UL)
#define VADC_GXCHCTR_BNDSELU_MASK               (0x03 << VADC_GXCHCTR_BNDSELU_SHIFT)
#define VADC_GXCHCTR_CHEVMODE_SHIFT             (8UL)
#define VADC_GXCHCTR_CHEVMODE_MASK              (0x03 << VADC_GXCHCTR_CHEVMODE_SHIFT)
#define VADC_GXCHCTR_SYNC_SHIFT                 (10UL)
#define VADC_GXCHCTR_SYNC_MASK                  (0x01 << VADC_GXCHCTR_SYNC_SHIFT)
#define VADC_GXCHCTR_REFSEL_SHIFT               (11UL)
#define VADC_GXCHCTR_REFSEL_MASK                (0x01 << VADC_GXCHCTR_REFSEL_SHIFT)
#define VADC_GXCHCTR_RESREG_SHIFT               (16UL)
#define VADC_GXCHCTR_RESREG_MASK                (0x0f << VADC_GXCHCTR_RESREG_SHIFT)
#define VADC_GXCHCTR_RESTBS_SHIFT               (20UL)
#define VADC_GXCHCTR_RESTBS_MASK                (0x01 << VADC_GXCHCTR_RESTBS_SHIFT)
#define VADC_GXCHCTR_RESPOS_SHIFT               (21UL)
#define VADC_GXCHCTR_RESPOS_MASK                (0x01 << VADC_GXCHCTR_RESPOS_SHIFT)
#define VADC_GXCHCTR_BWDCH_SHIFT                (28UL)
#define VADC_GXCHCTR_BWDCH_MASK                 (0x03 << VADC_GXCHCTR_BWDCH_SHIFT)
#define VADC_GXCHCTR_BWDEN_SHIFT                (30UL)
#define VADC_GXCHCTR_BWDEN_MASK                 (0x01 << VADC_GXCHCTR_BWDEN_SHIFT)

/* ---------------------------------  VADC_GXRCR  --------------------------------- */
#define VADC_GXRCR_DRCTR_SHIFT                  (16UL)
#define VADC_GXRCR_DRCTR_MASK                   (0x0f << VADC_GXRCR_DRCTR_SHIFT)
#define VADC_GXRCR_DMM_SHIFT                    (20UL)
#define VADC_GXRCR_DMM_MASK                     (0x03 << VADC_GXRCR_DMM_SHIFT)
#define VADC_GXRCR_WFR_SHIFT                    (24UL)
#define VADC_GXRCR_WFR_MASK                     (0x01 << VADC_GXRCR_WFR_SHIFT)
#define VADC_GXRCR_FEN_SHIFT                    (25UL)
#define VADC_GXRCR_FEN_MASK                     (0x03 << VADC_GXRCR_FEN_SHIFT)
#define VADC_GXRCR_SRGEN_SHIFT                  (31UL)
#define VADC_GXRCR_SRGEN_MASK                   (0x01 << VADC_GXRCR_SRGEN_SHIFT)

/* ---------------------------------  VADC_GXRES  --------------------------------- */
#define VADC_GXRES_RESULT_SHIFT                 (0UL)
#define VADC_GXRES_RESULT_MASK                  (0xffff << VADC_GXRES_RESULT_SHIFT)
#define VADC_GXRES_DRC_SHIFT                    (16UL)
#define VADC_GXRES_DRC_MASK                     (0x0f << VADC_GXRES_DRC_SHIFT)
#define VADC_GXRES_CHNR_SHIFT                   (20UL)
#define VADC_GXRES_CHNR_MASK                    (0x1f << VADC_GXRES_CHNR_SHIFT)
#define VADC_GXRES_EMUX_SHIFT                   (25UL)
#define VADC_GXRES_EMUX_MASK                    (0x07 << VADC_GXRES_EMUX_SHIFT)
#define VADC_GXRES_CRS_SHIFT                    (28UL)
#define VADC_GXRES_CRS_MASK                     (0x03 << VADC_GXRES_CRS_SHIFT)
#define VADC_GXRES_FCR_SHIFT                    (30UL)
#define VADC_GXRES_FCR_MASK                     (0x01 << VADC_GXRES_FCR_SHIFT)
#define VADC_GXRES_VF_SHIFT                     (31UL)
#define VADC_GXRES_VF_MASK                      (0x01 << VADC_GXRES_VF_SHIFT)

/* ---------------------------------  VADC_GXRESD  -------------------------------- */
#define VADC_GXRESD_RESULT_SHIFT                (0UL)
#define VADC_GXRESD_RESULT_MASK                 (0xffff << VADC_GXRESD_RESULT_SHIFT)
#define VADC_GXRESD_DRC_SHIFT                   (16UL)
#define VADC_GXRESD_DRC_MASK                    (0x0f << VADC_GXRESD_DRC_SHIFT)
#define VADC_GXRESD_CHNR_SHIFT                  (20UL)
#define VADC_GXRESD_CHNR_MASK                   (0x1f << VADC_GXRESD_CHNR_SHIFT)
#define VADC_GXRESD_EMUX_SHIFT                  (25UL)
#define VADC_GXRESD_EMUX_MASK                   (0x07 << VADC_GXRESD_EMUX_SHIFT)
#define VADC_GXRESD_CRS_SHIFT                   (28UL)
#define VADC_GXRESD_CRS_MASK                    (0x03 << VADC_GXRESD_CRS_SHIFT)
#define VADC_GXRESD_FCR_SHIFT                   (30UL)
#define VADC_GXRESD_FCR_MASK                    (0x01 << VADC_GXRESD_FCR_SHIFT)
#define VADC_GXRESD_VF_SHIFT                    (31UL)
#define VADC_GXRESD_VF_MASK                     (0x01 << VADC_GXRESD_VF_SHIFT)

#endif /* __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_VADC_H */
