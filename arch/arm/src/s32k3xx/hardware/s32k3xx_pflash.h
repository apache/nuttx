/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_pflash.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PFLASH_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PFLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PFLASH Register Offsets **************************************************/

#define S32K3XX_PFLASH_PFCR0_OFFSET                  (0x0000) /* Platform Flash Memory Configuration Register 0 (PFCR0) */
#define S32K3XX_PFLASH_PFCR1_OFFSET                  (0x0004) /* Platform Flash Memory Configuration Register 1 (PFCR1) */
#define S32K3XX_PFLASH_PFCR2_OFFSET                  (0x0008) /* Platform Flash Memory Configuration Register 2 (PFCR2) */
#define S32K3XX_PFLASH_PFCR4_OFFSET                  (0x0010) /* Platform Flash Memory Configuration Register 4 (PFCR4) */
#define S32K3XX_PFLASH_PFAPR_OFFSET                  (0x0014) /* Platform Flash Memory Access Protection Register (PFAPR) */
#define S32K3XX_PFLASH_PFCPGM_PEADR_L_OFFSET         (0x0300) /* Platform Flash Memory Program Erase Address Logical (PFCPGM_PEADR_L) */
#define S32K3XX_PFLASH_PFCPGM_PEADR_P_OFFSET         (0x0304) /* Platform Flash Memory Program Erase Address Physical (PFCPGM_PEADR_P) */
#define S32K3XX_PFLASH_PFCPGM_XPEADR_L_OFFSET        (0x0308) /* Platform Flash Memory Express Program Erase Address Logical (PFCPGM_XPEADR_L) */
#define S32K3XX_PFLASH_PFCPGM_XPEADR_P_OFFSET        (0x030c) /* Platform Flash Memory Express Program Erase Address Physical (PFCPGM_XPEADR_P) */
#define S32K3XX_PFLASH_PFCBLK0_SPELOCK_OFFSET        (0x0340) /* Block 0 Sector Program Erase Lock (PFCBLK0_SPELOCK) */
#define S32K3XX_PFLASH_PFCBLK1_SPELOCK_OFFSET        (0x0344) /* Block 1 Sector Program Erase Lock (PFCBLK1_SPELOCK) */
#define S32K3XX_PFLASH_PFCBLK2_SPELOCK_OFFSET        (0x0348) /* Block 2 Sector Program Erase Lock (PFCBLK2_SPELOCK) */
#define S32K3XX_PFLASH_PFCBLK3_SPELOCK_OFFSET        (0x034c) /* Block 3 Sector Program Erase Lock (PFCBLK3_SPELOCK) */
#define S32K3XX_PFLASH_PFCBLK4_SPELOCK_OFFSET        (0x0350) /* Block 4 Sector Program Erase Lock (PFCBLK4_SPELOCK) */
#define S32K3XX_PFLASH_PFCBLKU_SPELOCK_OFFSET        (0x0358) /* Block UTEST Sector Program Erase Lock (PFCBLKU_SPELOCK) */
#define S32K3XX_PFLASH_PFCBLK0_SSPELOCK_OFFSET       (0x035c) /* Block 0 Super Sector Program Erase Lock (PFCBLK0_SSPELOCK) */
#define S32K3XX_PFLASH_PFCBLK1_SSPELOCK_OFFSET       (0x0360) /* Block 1 Super Sector Program Erase Lock (PFCBLK1_SSPELOCK) */
#define S32K3XX_PFLASH_PFCBLK2_SSPELOCK_OFFSET       (0x0364) /* Block 2 Super Sector Program Erase Lock (PFCBLK2_SSPELOCK) */
#define S32K3XX_PFLASH_PFCBLK3_SSPELOCK_OFFSET       (0x0368) /* Block 3 Super Sector Program Erase Lock (PFCBLK3_SSPELOCK) */
#define S32K3XX_PFLASH_PFCBLK0_SETSLOCK_OFFSET       (0x0380) /* Block 0 Set Sector Lock (PFCBLK0_SETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK1_SETSLOCK_OFFSET       (0x0384) /* Block 1 Set Sector Lock (PFCBLK1_SETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK2_SETSLOCK_OFFSET       (0x0388) /* Block 2 Set Sector Lock (PFCBLK2_SETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK3_SETSLOCK_OFFSET       (0x038c) /* Block 3 Set Sector Lock (PFCBLK3_SETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK4_SETSLOCK_OFFSET       (0x0390) /* Block 4 Set Sector Lock (PFCBLK4_SETSLOCK) */
#define S32K3XX_PFLASH_PFCBLKU_SETSLOCK_OFFSET       (0x0398) /* Block UTEST Set Sector Lock (PFCBLKU_SETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK0_SSETSLOCK_OFFSET      (0x039c) /* Block 0 Set Super Sector Lock (PFCBLK0_SSETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK1_SSETSLOCK_OFFSET      (0x03a0) /* Block 1 Set Super Sector Lock (PFCBLK1_SSETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK2_SSETSLOCK_OFFSET      (0x03a4) /* Block 2 Set Super Sector Lock (PFCBLK2_SSETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK3_SSETSLOCK_OFFSET      (0x03a8) /* Block 3 Set Super Sector Lock (PFCBLK3_SSETSLOCK) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S0_OFFSET  (0x03c0) /* Block 0 Lock Master Sector 0 (PFCBLK0_LOCKMASTER_S0) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S1_OFFSET  (0x03c4) /* Block 0 Lock Master Sector 1 (PFCBLK0_LOCKMASTER_S1) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S2_OFFSET  (0x03c8) /* Block 0 Lock Master Sector 2 (PFCBLK0_LOCKMASTER_S2) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S3_OFFSET  (0x03cc) /* Block 0 Lock Master Sector 3 (PFCBLK0_LOCKMASTER_S3) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S4_OFFSET  (0x03d0) /* Block 0 Lock Master Sector 4 (PFCBLK0_LOCKMASTER_S4) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S5_OFFSET  (0x03d4) /* Block 0 Lock Master Sector 5 (PFCBLK0_LOCKMASTER_S5) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S6_OFFSET  (0x03d8) /* Block 0 Lock Master Sector 6 (PFCBLK0_LOCKMASTER_S6) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S7_OFFSET  (0x03dc) /* Block 0 Lock Master Sector 7 (PFCBLK0_LOCKMASTER_S7) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S0_OFFSET  (0x03e0) /* Block 1 Lock Master Sector 0 (PFCBLK1_LOCKMASTER_S0) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S1_OFFSET  (0x03e4) /* Block 1 Lock Master Sector 1 (PFCBLK1_LOCKMASTER_S1) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S2_OFFSET  (0x03e8) /* Block 1 Lock Master Sector 2 (PFCBLK1_LOCKMASTER_S2) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S3_OFFSET  (0x03ec) /* Block 1 Lock Master Sector 3 (PFCBLK1_LOCKMASTER_S3) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S4_OFFSET  (0x03f0) /* Block 1 Lock Master Sector 4 (PFCBLK1_LOCKMASTER_S4) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S5_OFFSET  (0x03f4) /* Block 1 Lock Master Sector 5 (PFCBLK1_LOCKMASTER_S5) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S6_OFFSET  (0x03f8) /* Block 1 Lock Master Sector 6 (PFCBLK1_LOCKMASTER_S6) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S7_OFFSET  (0x03fc) /* Block 1 Lock Master Sector 7 (PFCBLK1_LOCKMASTER_S7) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S0_OFFSET  (0x0400) /* Block 2 Lock Master Sector 0 (PFCBLK2_LOCKMASTER_S0) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S1_OFFSET  (0x0404) /* Block 2 Lock Master Sector 1 (PFCBLK2_LOCKMASTER_S1) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S2_OFFSET  (0x0408) /* Block 2 Lock Master Sector 2 (PFCBLK2_LOCKMASTER_S2) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S3_OFFSET  (0x040c) /* Block 2 Lock Master Sector 3 (PFCBLK2_LOCKMASTER_S3) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S4_OFFSET  (0x0410) /* Block 2 Lock Master Sector 4 (PFCBLK2_LOCKMASTER_S4) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S5_OFFSET  (0x0414) /* Block 2 Lock Master Sector 5 (PFCBLK2_LOCKMASTER_S5) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S6_OFFSET  (0x0418) /* Block 2 Lock Master Sector 6 (PFCBLK2_LOCKMASTER_S6) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S7_OFFSET  (0x041c) /* Block 2 Lock Master Sector 7 (PFCBLK2_LOCKMASTER_S7) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S0_OFFSET  (0x0420) /* Block 3 Lock Master Sector 0 (PFCBLK3_LOCKMASTER_S0) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S1_OFFSET  (0x0424) /* Block 3 Lock Master Sector 1 (PFCBLK3_LOCKMASTER_S1) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S2_OFFSET  (0x0428) /* Block 3 Lock Master Sector 2 (PFCBLK3_LOCKMASTER_S2) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S3_OFFSET  (0x042c) /* Block 3 Lock Master Sector 3 (PFCBLK3_LOCKMASTER_S3) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S4_OFFSET  (0x0430) /* Block 3 Lock Master Sector 4 (PFCBLK3_LOCKMASTER_S4) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S5_OFFSET  (0x0434) /* Block 3 Lock Master Sector 5 (PFCBLK3_LOCKMASTER_S5) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S6_OFFSET  (0x0438) /* Block 3 Lock Master Sector 6 (PFCBLK3_LOCKMASTER_S6) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S7_OFFSET  (0x043c) /* Block 3 Lock Master Sector 7 (PFCBLK3_LOCKMASTER_S7) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S0_OFFSET  (0x0440) /* Block 4 Lock Master Sector 0 (PFCBLK4_LOCKMASTER_S0) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S1_OFFSET  (0x0444) /* Block 4 Lock Master Sector 1 (PFCBLK4_LOCKMASTER_S1) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S2_OFFSET  (0x0448) /* Block 4 Lock Master Sector 2 (PFCBLK4_LOCKMASTER_S2) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S3_OFFSET  (0x044c) /* Block 4 Lock Master Sector 3 (PFCBLK4_LOCKMASTER_S3) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S4_OFFSET  (0x0450) /* Block 4 Lock Master Sector 4 (PFCBLK4_LOCKMASTER_S4) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S5_OFFSET  (0x0454) /* Block 4 Lock Master Sector 5 (PFCBLK4_LOCKMASTER_S5) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S6_OFFSET  (0x0458) /* Block 4 Lock Master Sector 6 (PFCBLK4_LOCKMASTER_S6) */
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S7_OFFSET  (0x045c) /* Block 4 Lock Master Sector 7 (PFCBLK4_LOCKMASTER_S7) */
#define S32K3XX_PFLASH_PFCBLKU_LOCKMASTER_S_OFFSET   (0x0480) /* Block UTEST Lock Master Sector (PFCBLKU_LOCKMASTER_S) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS0_OFFSET (0x0484) /* Block 0 Lock Master Super Sector 0 (PFCBLK0_LOCKMASTER_SS0) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS1_OFFSET (0x0488) /* Block 0 Lock Master Super Sector 1 (PFCBLK0_LOCKMASTER_SS1) */
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS2_OFFSET (0x048c) /* Block 0 Lock Master Super Sector 2 (PFCBLK0_LOCKMASTER_SS2) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS0_OFFSET (0x0494) /* Block 1 Lock Master Super Sector 0 (PFCBLK1_LOCKMASTER_SS0) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS1_OFFSET (0x0498) /* Block 1 Lock Master Super Sector 1 (PFCBLK1_LOCKMASTER_SS1) */
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS2_OFFSET (0x049c) /* Block 1 Lock Master Super Sector 2 (PFCBLK1_LOCKMASTER_SS2) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS0_OFFSET (0x04a4) /* Block 2 Lock Master Super Sector 0 (PFCBLK2_LOCKMASTER_SS0) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS1_OFFSET (0x04a8) /* Block 2 Lock Master Super Sector 1 (PFCBLK2_LOCKMASTER_SS1) */
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS2_OFFSET (0x04ac) /* Block 2 Lock Master Super Sector 2 (PFCBLK2_LOCKMASTER_SS2) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS0_OFFSET (0x04b4) /* Block 3 Lock Master Super Sector 0 (PFCBLK3_LOCKMASTER_SS0) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS1_OFFSET (0x04b8) /* Block 3 Lock Master Super Sector 1 (PFCBLK3_LOCKMASTER_SS1) */
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS2_OFFSET (0x04bc) /* Block 3 Lock Master Super Sector 2 (PFCBLK3_LOCKMASTER_SS2) */

/* Flash Management Unit Register Offsets ***********************************/

#define S32K3XX_FMU_MCR_OFFSET                       (0x0000) /* Module Configuration (MCR) */
#define S32K3XX_FMU_MCRS_OFFSET                      (0x0004) /* Module Configuration Status (MCRS) */
#define S32K3XX_FMU_PD_OFFSET                        (0x0100) /* Program data (PD) */

/* PFLASH Register Addresses ************************************************/

#define S32K3XX_PFLASH_PFCR0                         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCR0_OFFSET)
#define S32K3XX_PFLASH_PFCR1                         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCR1_OFFSET)
#define S32K3XX_PFLASH_PFCR2                         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCR2_OFFSET)
#define S32K3XX_PFLASH_PFCR4                         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCR4_OFFSET)
#define S32K3XX_PFLASH_PFAPR                         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFAPR_OFFSET)
#define S32K3XX_PFLASH_PFCPGM_PEADR_L                (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCPGM_PEADR_L_OFFSET)
#define S32K3XX_PFLASH_PFCPGM_PEADR_P                (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCPGM_PEADR_P_OFFSET)
#define S32K3XX_PFLASH_PFCPGM_XPEADR_L               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCPGM_XPEADR_L_OFFSET)
#define S32K3XX_PFLASH_PFCPGM_XPEADR_P               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCPGM_XPEADR_P_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_SPELOCK               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_SPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_SPELOCK               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_SPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_SPELOCK               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_SPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_SPELOCK               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_SPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_SPELOCK               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_SPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLKU_SPELOCK               (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLKU_SPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_SSPELOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_SSPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_SSPELOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_SSPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_SSPELOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_SSPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_SSPELOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_SSPELOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_SETSLOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_SETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_SETSLOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_SETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_SETSLOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_SETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_SETSLOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_SETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_SETSLOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_SETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLKU_SETSLOCK              (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLKU_SETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_SSETSLOCK             (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_SSETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_SSETSLOCK             (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_SSETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_SSETSLOCK             (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_SSETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_SSETSLOCK             (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_SSETSLOCK_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S0         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S1         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S2         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S3         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S3_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S4         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S4_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S5         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S5_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S6         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S6_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S7         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_S7_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S0         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S1         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S2         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S3         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S3_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S4         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S4_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S5         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S5_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S6         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S6_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S7         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_S7_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S0         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S1         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S2         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S3         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S3_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S4         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S4_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S5         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S5_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S6         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S6_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S7         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_S7_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S0         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S1         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S2         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S3         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S3_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S4         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S4_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S5         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S5_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S6         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S6_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S7         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_S7_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S0         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S1         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S2         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S3         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S3_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S4         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S4_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S5         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S5_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S6         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S6_OFFSET)
#define S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S7         (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK4_LOCKMASTER_S7_OFFSET)
#define S32K3XX_PFLASH_PFCBLKU_LOCKMASTER_S          (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLKU_LOCKMASTER_S_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS0        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS1        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS2        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK0_LOCKMASTER_SS2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS0        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS1        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS2        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK1_LOCKMASTER_SS2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS0        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS1        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS2        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK2_LOCKMASTER_SS2_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS0        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS0_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS1        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS1_OFFSET)
#define S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS2        (S32K3XX_PFLASH_BASE + S32K3XX_PFLASH_PFCBLK3_LOCKMASTER_SS2_OFFSET)

/* Flash Management Unit Register Addresses *********************************/

#define S32K3XX_FMU_MCR                              (S32K3XX_FMU_BASE + S32K3XX_FMU_MCR_OFFSET)
#define S32K3XX_FMU_MCRS                             (S32K3XX_FMU_BASE + S32K3XX_FMU_MCRS_OFFSET)
#define S32K3XX_FMU_PD(n)                            (S32K3XX_FMU_BASE + S32K3XX_FMU_PD_OFFSET + (n * 4))

/* PFLASH Register Bitfield Definitions *************************************/

/* Platform Flash Memory Configuration Register 0 (PFCR0) */

#define PFLASH_PFCR0_P0_CBFEN             (1 << 0)  /* Bit 0: Port 0 PFLASH Line Read Code Buffers Enable (P0_CBFEN) */
#define PFLASH_PFCR0_P0_DBFEN             (1 << 1)  /* Bit 1: Port 0 PFLASH Line Read Data Buffers Enable (P0_DBFEN) */
                                                    /* Bit 2-3: Reserved */
#define PFLASH_PFCR0_P0_CPFEN             (1 << 4)  /* Bit 4: Port 0 Code Prefetch Enable (P0_CPFEN) */
#define PFLASH_PFCR0_P0_DPFEN             (1 << 5)  /* Bit 5: Port 0 Data Prefetch Enable (P0_DPFEN) */
                                                    /* Bits 6-31: Reserved */

/* Platform Flash Memory Configuration Register 1 (PFCR1) */

#define PFLASH_PFCR1_P1_CBFEN             (1 << 0)  /* Bit 0: Port 1 PFLASH Line Read Code Buffers Enable (P1_CBFEN) */
#define PFLASH_PFCR1_P1_DBFEN             (1 << 1)  /* Bit 1: Port 1 PFLASH Line Read Data Buffers Enable (P1_DBFEN) */
                                                    /* Bit 2-3: Reserved */
#define PFLASH_PFCR1_P1_CPFEN             (1 << 4)  /* Bit 4: Port 1 Code Prefetch Enable (P1_CPFEN) */
#define PFLASH_PFCR1_P1_DPFEN             (1 << 5)  /* Bit 5: Port 1 Data Prefetch Enable (P1_DPFEN) */
                                                    /* Bits 6-31: Reserved */

/* Platform Flash Memory Configuration Register 2 (PFCR2) */

#define PFLASH_PFCR2_P2_CBFEN             (1 << 0)  /* Bit 0: Port 2 PFLASH Line Read Code Buffers Enable (P2_CBFEN) */
#define PFLASH_PFCR2_P2_DBFEN             (1 << 1)  /* Bit 1: Port 2 PFLASH Line Read Data Buffers Enable (P2_DBFEN) */
                                                    /* Bit 2-3: Reserved */
#define PFLASH_PFCR2_P2_CPFEN             (1 << 4)  /* Bit 4: Port 2 Code Prefetch Enable (P2_CPFEN) */
#define PFLASH_PFCR2_P2_DPFEN             (1 << 5)  /* Bit 5: Port 2 Data Prefetch Enable (P2_DPFEN) */
                                                    /* Bits 6-31: Reserved */

/* Platform Flash Memory Configuration Register 4 (PFCR4) */

#define PFLASH_PFCR4_DERR_SUP             (1 << 0)  /* Bit 0: Data Error Suppression (DERR_SUP) */
#define PFLASH_PFCR4_BLK4_PS_SHIFT        (1)       /* Bits 1-3: Block 4 Pipe Select (BLK4_PS) */
#define PFLASH_PFCR4_BLK4_PS_MASK         (0x07 << PFLASH_PFCR4_BLK4_PS_SHIFT)
                                                    /* Bits 4-6: Reserved */
#define PFLASH_PFCR4_DMEEE                (1 << 7)  /* Bit 7: Disable Multi-Bit ECC Error Exception (DMEEE) */
                                                    /* Bits 8-31: Reserved */

/* Platform Flash Memory Access Protection Register (PFAPR) */

#define PFLASH_PFAPR_M15AP_SHIFT          (0)       /* Bits 0-1: Master 15 Access Protection (M15AP) */
#define PFLASH_PFAPR_M15AP_MASK           (0x03 << PFLASH_PFAPR_M15AP_SHIFT)
#define PFLASH_PFAPR_M14AP_SHIFT          (2)       /* Bits 2-3: Master 14 Access Protection (M14AP) */
#define PFLASH_PFAPR_M14AP_MASK           (0x03 << PFLASH_PFAPR_M14AP_SHIFT)
#define PFLASH_PFAPR_M13AP_SHIFT          (4)       /* Bits 4-5: Master 13 Access Protection (M13AP) */
#define PFLASH_PFAPR_M13AP_MASK           (0x03 << PFLASH_PFAPR_M13AP_SHIFT)
#define PFLASH_PFAPR_M12AP_SHIFT          (6)       /* Bits 6-7: Master 12 Access Protection (M12AP) */
#define PFLASH_PFAPR_M12AP_MASK           (0x03 << PFLASH_PFAPR_M12AP_SHIFT)
#define PFLASH_PFAPR_M11AP_SHIFT          (8)       /* Bits 8-9: Master 11 Access Protection (M11AP) */
#define PFLASH_PFAPR_M11AP_MASK           (0x03 << PFLASH_PFAPR_M11AP_SHIFT)
#define PFLASH_PFAPR_M10AP_SHIFT          (10)      /* Bits 10-11: Master 10 Access Protection (M10AP) */
#define PFLASH_PFAPR_M10AP_MASK           (0x03 << PFLASH_PFAPR_M10AP_SHIFT)
#define PFLASH_PFAPR_M9AP_SHIFT           (12)      /* Bits 12-13: Master 9 Access Protection (M9AP) */
#define PFLASH_PFAPR_M9AP_MASK            (0x03 << PFLASH_PFAPR_M9AP_SHIFT)
#define PFLASH_PFAPR_M8AP_SHIFT           (14)      /* Bits 14-15: Master 8 Access Protection (M8AP) */
#define PFLASH_PFAPR_M8AP_MASK            (0x03 << PFLASH_PFAPR_M8AP_SHIFT)
#define PFLASH_PFAPR_M7AP_SHIFT           (16)      /* Bits 16-17: Master 7 Access Protection (M7AP) */
#define PFLASH_PFAPR_M7AP_MASK            (0x03 << PFLASH_PFAPR_M7AP_SHIFT)
#define PFLASH_PFAPR_M6AP_SHIFT           (18)      /* Bits 18-19: Master 6 Access Protection (M6AP) */
#define PFLASH_PFAPR_M6AP_MASK            (0x03 << PFLASH_PFAPR_M6AP_SHIFT)
#define PFLASH_PFAPR_M5AP_SHIFT           (20)      /* Bits 20-21: Master 5 Access Protection (M5AP) */
#define PFLASH_PFAPR_M5AP_MASK            (0x03 << PFLASH_PFAPR_M5AP_SHIFT)
#define PFLASH_PFAPR_M4AP_SHIFT           (22)      /* Bits 22-23: Master 4 Access Protection (M4AP) */
#define PFLASH_PFAPR_M4AP_MASK            (0x03 << PFLASH_PFAPR_M4AP_SHIFT)
                                                    /* Bits 24-25: Reserved */
#define PFLASH_PFAPR_M2AP_SHIFT           (26)      /* Bits 26-27: Master 2 Access Protection (M2AP) */
#define PFLASH_PFAPR_M2AP_MASK            (0x03 << PFLASH_PFAPR_M2AP_SHIFT)
#define PFLASH_PFAPR_M1AP_SHIFT           (28)      /* Bits 28-29: Master 1 Access Protection (M1AP) */
#define PFLASH_PFAPR_M1AP_MASK            (0x03 << PFLASH_PFAPR_M1AP_SHIFT)
#define PFLASH_PFAPR_M0AP_SHIFT           (30)      /* Bits 30-31: Master 0 Access Protection (M0AP) */
#define PFLASH_PFAPR_M0AP_MASK            (0x03 << PFLASH_PFAPR_M0AP_SHIFT)

/* Platform Flash Memory Program Erase Address Logical (PFCPGM_PEADR_L) */

#define PFLASH_PFCPGM_PEADR_L_SHIFT       (0)       /* Bits 0-31: Program Erase Address Logical (PEADR_L) */
#define PFLASH_PFCPGM_PEADR_L_MASK        (0xffffffff << PFLASH_PFCPGM_PEADR_L_SHIFT)

/* Platform Flash Memory Program Erase Address Physical (PFCPGM_PEADR_P) */

#define PFLASH_PFCPGM_PEADR_P_SHIFT       (0)       /* Bits 0-31: Program Erase Address Physical (PEADR_P) */
#define PFLASH_PFCPGM_PEADR_P_MASK        (0xffffffff << PFLASH_PFCPGM_PEADR_P_SHIFT)

/* Platform Flash Memory Express Program Erase Address Logica
 * (PFCPGM_XPEADR_L)
 */

#define PFLASH_PFCPGM_XPEADR_L_SHIFT      (0)       /* Bits 0-31: Express Program Erase Address Logical (XPEADR_L) */
#define PFLASH_PFCPGM_XPEADR_L_MASK       (0xffffffff << PFLASH_PFCPGM_XPEADR_L_SHIFT)

/* Platform Flash Memory Express Program Erase Address Physical
 * (PFCPGM_XPEADR_P)
 */

#define PFLASH_PFCPGM_XPEADR_P_SHIFT      (0)       /* Bits 0-31: Express Program Erase Address Physical (XPEADR_P) */
#define PFLASH_PFCPGM_XPEADR_P_MASK       (0xffffffff << PFLASH_PFCPGM_XPEADR_P_SHIFT)

/* Block n Sector Program Erase Lock (PFCBLKn_SPELOCK) */

#define PFLASH_PFCBLK_SPELOCK_SHIFT       (0)       /* Bits 0-31: Sector Lock (SLCK) */
#define PFLASH_PFCBLK_SPELOCK_MASK        (0xffffffff << PFLASH_PFCBLK_SPELOCK_SLCK_SHIFT)

/* Block UTEST Sector Program Erase Lock (PFCBLKU_SPELOCK) */

#define PFLASH_PFCBLKU_SPELOCK_SLCK       (1 << 0)  /* Bit 0: Sector Lock (SLCK) */
                                                    /* Bits 1-31: Reserved */

/* Block n Super Sector Program Erase Lock (PFCBLKn_SSPELOCK) */

#define PFLASH_PFCBLK_SSPELOCK_SHIFT      (0)       /* Bits 0-11: Super Sector Lock (SSLCK) */
#define PFLASH_PFCBLK_SSPELOCK_MASK       (0x0fff << PFLASH_PFCBLK_SSPELOCK_SSLCK_SHIFT)
                                                    /* Bits 12-31: Reserved */

/* Block n Set Sector Lock (PFCBLKn_SETSLOCK) */

#define PFLASH_PFCBLK_SETSLOCK_SHIFT      (0)       /* Bits 0-31: Set Sector Lock (SETSLCK) */
#define PFLASH_PFCBLK_SETSLOCK_MASK       (0xffffffff << PFLASH_PFCBLK_SETSLOCK_SHIFT)

/* Block UTEST Set Sector Lock (PFCBLKU_SETSLOCK) */

#define PFLASH_PFCBLKU_SETSLOCK           (1 << 0)  /* Bit 0: Set Sector Lock (SETSLCK) */
                                                    /* Bits 1-31: Reserved */

/* Block n Set Super Sector Lock (PFCBLKn_SSETSLOCK) */

#define PFLASH_PFCBLK_SSETSLOCK_SHIFT     (0)       /* Bits 0-11: Set Super Sector Lock (SSETSLCK) */
#define PFLASH_PFCBLK_SSETSLOCK_MASK      (0x0fff << PFLASH_FCBLK_SSETSLOCK_SHIFT)
                                                    /* Bits 12-31: Reserved */

/* Block n Lock Master Sector m (PFCBLKn_LOCKMASTER_Sm) */

#define PFLASH_PFCBLK_LOCKMASTER_S_SHIFT  (0)       /* Bits 0-31: Block n Lock Master Sector m (LOCKMASTER_S) */
#define PFLASH_PFCBLK_LOCKMASTER_S_MASK   (0xffffffff << PFLASH_PFCBLK_LOCKMASTER_S_SHIFT)

/* Block UTEST Lock Master Sector (PFCBLKU_LOCKMASTER_S) */

#define PFLASH_PFCBLKU_LOCKMASTER_S_SHIFT (0)       /* Bits 0-7: Block n Lock Master Sector m (LOCKMASTER_S) */
#define PFLASH_PFCBLKU_LOCKMASTER_S_MASK  (0xff << PFLASH_PFCBLKU_LOCKMASTER_S_SHIFT)
                                                    /* Bits 8-31: Reserved */

/* Block n Lock Master Super Sector m (PFCBLKn_LOCKMASTER_SSm) */

#define PFLASH_PFCBLK_LOCKMASTER_SS_SHIFT (0)       /* Bits 0-31: Block n Lock Master Super Sector m (LOCKMASTER_SS) */
#define PFLASH_PFCBLK_LOCKMASTER_SS_MASK  (0xffffffff << PFLASH_PFCBLK_LOCKMASTER_SS_SHIFT)

/* Flash Management Unit Bitfield Definitions *******************************/

/* Module Configuration (MCR) */

#define FMU_MCR_EHV_SHIFT                 (0)       /* Bit 0: Enable High Voltage (EHV) */
#define FMU_MCR_EHV_MASK                  (0x01 << FMU_MCR_EHV_SHIFT)
#define FMU_MCR_ERS_SHIFT                 (4)       /* Bit 4: Erase (ERS) */
#define FMU_MCR_ERS_MASK                  (0x01 << FMU_MCR_ERS_SHIFT)
#define FMU_MCR_ESS_SHIFT                 (5)       /* Bit 5: Erase Size Select (ESS) */
#define FMU_MCR_ESS_MASK                  (0x01 << FMU_MCR_ESS_SHIFT)
#define FMU_MCR_PGM_SHIFT                 (8)       /* Bit 8: Program (PGM) */
#define FMU_MCR_PGM_MASK                  (0x01 << FMU_MCR_PGM_SHIFT)
#define FMU_MCR_WDIE_SHIFT                (12)      /* Bit 12: Watch Dog Interrupt Enable (WDIE) */
#define FMU_MCR_WDIE_MASK                 (0x01 << FMU_MCR_WDIE_SHIFT)
#define FMU_MCR_PECIE_SHIFT               (15)      /* Bit 15: Program/Erase Complete Interrupt Enable (PECIE) */
#define FMU_MCR_PECIE_MASK                (0x01 << FMU_MCR_PECIE_SHIFT)
#define FMU_MCR_PEID_SHIFT                (16)      /* Bit 16-23: Program and Erase Master/Domain ID (PEID) */
#define FMU_MCR_PEID_MASK                 (0xFF << FMU_MCR_PEID_SHIFT)
#define FMU_MCR_PEID(n)                   ((n & FMU_MCR_PEID_MASK) >> FMU_MCR_PEID_SHIFT)

#define FMU_MCRS_PEG_SHIFT                (14)      /* Bit 14: Program/Erase Good (PEG) */
#define FMU_MCRS_PEG_MASK                 (0x01 << FMU_MCRS_PEG_SHIFT)
#define FMU_MCRS_DONE_SHIFT               (15)      /* Bit 15: State machine status (DONE) */
#define FMU_MCRS_DONE_MASK                (0x01 << FMU_MCRS_DONE_SHIFT)
#define FMU_MCRS_PES_SHIFT                (16)      /* Bit 16: Program and erase Protection error (PEP) */
#define FMU_MCRS_PES_MASK                 (0x01 << FMU_MCRS_PES_SHIFT)
#define FMU_MCRS_PEP_SHIFT                (17)      /* Bit 17: Program and erase sequence error (PES) */
#define FMU_MCRS_PEP_MASK                 (0x01 << FMU_MCRS_PEP_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PFLASH_H */
