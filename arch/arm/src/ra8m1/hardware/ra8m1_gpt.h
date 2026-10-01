/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_gpt.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_GPT_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_GPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_GPT_GTWP_OFFSET                   0x0000  /* General PWM Timer Write-Protection Register (32-bits) */
#define R_GPT_GTSTR_OFFSET                  0x0004  /* General PWM Timer Software Start Register (32-bits) */
#define R_GPT_GTSTP_OFFSET                  0x0008  /* General PWM Timer Software Stop Register (32-bits) */
#define R_GPT_GTCLR_OFFSET                  0x000c  /* General PWM Timer Software Clear Register (32-bits) */
#define R_GPT_GTSSR_OFFSET                  0x0010  /* General PWM Timer Start Source Select Register (32-bits) */
#define R_GPT_GTPSR_OFFSET                  0x0014  /* General PWM Timer Stop Source Select Register (32-bits) */
#define R_GPT_GTCSR_OFFSET                  0x0018  /* General PWM Timer Clear Source Select Register (32-bits) */
#define R_GPT_GTUPSR_OFFSET                 0x001c  /* General PWM Timer Up Count Source Select Register (32-bits) */
#define R_GPT_GTDNSR_OFFSET                 0x0020  /* General PWM Timer Down Count Source Select Register (32-bits) */
#define R_GPT_GTICASR_OFFSET                0x0024  /* General PWM Timer Input Capture Source Select Register A (32-bits) */
#define R_GPT_GTICBSR_OFFSET                0x0028  /* General PWM Timer Input Capture Source Select Register B (32-bits) */
#define R_GPT_GTCR_OFFSET                   0x002c  /* General PWM Timer Control Register (32-bits) */
#define R_GPT_GTUDDTYC_OFFSET               0x0030  /* General PWM Timer Count Direction and Duty Setting Register (32-bits) */
#define R_GPT_GTIOR_OFFSET                  0x0034  /* General PWM Timer I/O Control Register (32-bits) */
#define R_GPT_GTINTAD_OFFSET                0x0038  /* General PWM Timer Interrupt Output Setting Register (32-bits) */
#define R_GPT_GTST_OFFSET                   0x003c  /* General PWM Timer Status Register (32-bits) */
#define R_GPT_GTBER_OFFSET                  0x0040  /* General PWM Timer Buffer Enable Register (32-bits) */
#define R_GPT_GTCNT_OFFSET                  0x0048  /* General PWM Timer Counter (32-bits) */
#define R_GPT_GTCCRA_OFFSET                 0x004c  /* General PWM Timer Compare Capture Register A (32-bits) */
#define R_GPT_GTCCRB_OFFSET                 0x0050  /* General PWM Timer Compare Capture Register B (32-bits) */
#define R_GPT_GTCCRC_OFFSET                 0x0054  /* General PWM Timer Compare Capture Register C (32-bits) */
#define R_GPT_GTCCRE_OFFSET                 0x0058  /* General PWM Timer Compare Capture Register E (32-bits) */
#define R_GPT_GTCCRD_OFFSET                 0x005c  /* General PWM Timer Compare Capture Register D (32-bits) */
#define R_GPT_GTCCRF_OFFSET                 0x0060  /* General PWM Timer Compare Capture Register F (32-bits) */
#define R_GPT_GTPR_OFFSET                   0x0064  /* General PWM Timer Cycle Setting Register (32-bits) */
#define R_GPT_GTPBR_OFFSET                  0x0068  /* General PWM Timer Cycle Setting Buffer Register (32-bits) */
#define R_GPT_GTADTRA_OFFSET                0x0070  /* A/D Converter Start Request Timing Register A (32-bits) */
#define R_GPT_GTADTBRA_OFFSET               0x0074  /* A/D Converter Start Request Timing Buffer Register A (32-bits) */
#define R_GPT_GTADTDBRA_OFFSET              0x0078  /* A/D Converter Start Request Timing Double-Buffer Register A (32-bits) */
#define R_GPT_GTADTRB_OFFSET                0x007c  /* A/D Converter Start Request Timing Register B (32-bits) */
#define R_GPT_GTADTBRB_OFFSET               0x0080  /* A/D Converter Start Request Timing Buffer Register B (32-bits) */
#define R_GPT_GTADTDBRB_OFFSET              0x0084  /* A/D Converter Start Request Timing Double-Buffer Register B (32-bits) */
#define R_GPT_GTDTCR_OFFSET                 0x0088  /* General PWM Timer Dead Time Control Register (32-bits) */
#define R_GPT_GTDVU_OFFSET                  0x008c  /* General PWM Timer Dead Time Value Register U (32-bits) */
#define R_GPT_GTADSMR_OFFSET                0x00a4  /* General PWM Timer A/D Conversion Start Request Signal Monitoring Register (32-bits) */
#define R_GPT_GTICLF_OFFSET                 0x00b8  /* General PWM Timer Inter Channel Logical Operation Function Setting Register (32-bits) */
#define R_GPT_GTPC_OFFSET                   0x00bc  /* General PWM Timer Period Count Register (32-bits) */
#define R_GPT_GTSECSR_OFFSET                0x00d0  /* General PWM Timer Operation Enable Bit Simultaneous Control Channel Select Register (32-bits) */
#define R_GPT_GTSECR_OFFSET                 0x00d4  /* General PWM Timer Operation Enable Bit Simultaneous Control Register (32-bits) */

/* Register Addresses *******************************************************/

/* GPT0 Registers */

#define R_GPT0_GTWP                        (R_GPT0_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT0_GTSTR                       (R_GPT0_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT0_GTSTP                       (R_GPT0_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT0_GTCLR                       (R_GPT0_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT0_GTSSR                       (R_GPT0_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT0_GTPSR                       (R_GPT0_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT0_GTCSR                       (R_GPT0_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT0_GTUPSR                      (R_GPT0_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT0_GTDNSR                      (R_GPT0_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT0_GTICASR                     (R_GPT0_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT0_GTICBSR                     (R_GPT0_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT0_GTCR                        (R_GPT0_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT0_GTUDDTYC                    (R_GPT0_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT0_GTIOR                       (R_GPT0_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT0_GTINTAD                     (R_GPT0_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT0_GTST                        (R_GPT0_BASE + R_GPT_GTST_OFFSET)
#define R_GPT0_GTBER                       (R_GPT0_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT0_GTCNT                       (R_GPT0_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT0_GTCCRA                      (R_GPT0_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT0_GTCCRB                      (R_GPT0_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT0_GTCCRC                      (R_GPT0_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT0_GTCCRE                      (R_GPT0_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT0_GTCCRD                      (R_GPT0_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT0_GTCCRF                      (R_GPT0_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT0_GTPR                        (R_GPT0_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT0_GTPBR                       (R_GPT0_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT0_GTADTRA                     (R_GPT0_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT0_GTADTBRA                    (R_GPT0_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT0_GTADTDBRA                   (R_GPT0_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT0_GTADTRB                     (R_GPT0_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT0_GTADTBRB                    (R_GPT0_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT0_GTADTDBRB                   (R_GPT0_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT0_GTDTCR                      (R_GPT0_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT0_GTDVU                       (R_GPT0_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT0_GTADSMR                     (R_GPT0_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT0_GTICLF                      (R_GPT0_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT0_GTPC                        (R_GPT0_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT0_GTSECSR                     (R_GPT0_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT0_GTSECR                      (R_GPT0_BASE + R_GPT_GTSECR_OFFSET)

/* GPT1 Registers */

#define R_GPT1_GTWP                        (R_GPT1_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT1_GTSTR                       (R_GPT1_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT1_GTSTP                       (R_GPT1_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT1_GTCLR                       (R_GPT1_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT1_GTSSR                       (R_GPT1_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT1_GTPSR                       (R_GPT1_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT1_GTCSR                       (R_GPT1_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT1_GTUPSR                      (R_GPT1_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT1_GTDNSR                      (R_GPT1_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT1_GTICASR                     (R_GPT1_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT1_GTICBSR                     (R_GPT1_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT1_GTCR                        (R_GPT1_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT1_GTUDDTYC                    (R_GPT1_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT1_GTIOR                       (R_GPT1_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT1_GTINTAD                     (R_GPT1_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT1_GTST                        (R_GPT1_BASE + R_GPT_GTST_OFFSET)
#define R_GPT1_GTBER                       (R_GPT1_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT1_GTCNT                       (R_GPT1_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT1_GTCCRA                      (R_GPT1_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT1_GTCCRB                      (R_GPT1_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT1_GTCCRC                      (R_GPT1_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT1_GTCCRE                      (R_GPT1_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT1_GTCCRD                      (R_GPT1_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT1_GTCCRF                      (R_GPT1_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT1_GTPR                        (R_GPT1_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT1_GTPBR                       (R_GPT1_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT1_GTADTRA                     (R_GPT1_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT1_GTADTBRA                    (R_GPT1_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT1_GTADTDBRA                   (R_GPT1_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT1_GTADTRB                     (R_GPT1_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT1_GTADTBRB                    (R_GPT1_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT1_GTADTDBRB                   (R_GPT1_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT1_GTDTCR                      (R_GPT1_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT1_GTDVU                       (R_GPT1_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT1_GTADSMR                     (R_GPT1_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT1_GTICLF                      (R_GPT1_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT1_GTPC                        (R_GPT1_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT1_GTSECSR                     (R_GPT1_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT1_GTSECR                      (R_GPT1_BASE + R_GPT_GTSECR_OFFSET)

/* GPT2 Registers */

#define R_GPT2_GTWP                        (R_GPT2_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT2_GTSTR                       (R_GPT2_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT2_GTSTP                       (R_GPT2_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT2_GTCLR                       (R_GPT2_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT2_GTSSR                       (R_GPT2_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT2_GTPSR                       (R_GPT2_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT2_GTCSR                       (R_GPT2_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT2_GTUPSR                      (R_GPT2_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT2_GTDNSR                      (R_GPT2_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT2_GTICASR                     (R_GPT2_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT2_GTICBSR                     (R_GPT2_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT2_GTCR                        (R_GPT2_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT2_GTUDDTYC                    (R_GPT2_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT2_GTIOR                       (R_GPT2_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT2_GTINTAD                     (R_GPT2_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT2_GTST                        (R_GPT2_BASE + R_GPT_GTST_OFFSET)
#define R_GPT2_GTBER                       (R_GPT2_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT2_GTCNT                       (R_GPT2_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT2_GTCCRA                      (R_GPT2_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT2_GTCCRB                      (R_GPT2_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT2_GTCCRC                      (R_GPT2_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT2_GTCCRE                      (R_GPT2_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT2_GTCCRD                      (R_GPT2_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT2_GTCCRF                      (R_GPT2_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT2_GTPR                        (R_GPT2_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT2_GTPBR                       (R_GPT2_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT2_GTADTRA                     (R_GPT2_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT2_GTADTBRA                    (R_GPT2_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT2_GTADTDBRA                   (R_GPT2_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT2_GTADTRB                     (R_GPT2_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT2_GTADTBRB                    (R_GPT2_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT2_GTADTDBRB                   (R_GPT2_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT2_GTDTCR                      (R_GPT2_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT2_GTDVU                       (R_GPT2_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT2_GTADSMR                     (R_GPT2_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT2_GTICLF                      (R_GPT2_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT2_GTPC                        (R_GPT2_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT2_GTSECSR                     (R_GPT2_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT2_GTSECR                      (R_GPT2_BASE + R_GPT_GTSECR_OFFSET)

/* GPT3 Registers */

#define R_GPT3_GTWP                        (R_GPT3_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT3_GTSTR                       (R_GPT3_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT3_GTSTP                       (R_GPT3_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT3_GTCLR                       (R_GPT3_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT3_GTSSR                       (R_GPT3_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT3_GTPSR                       (R_GPT3_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT3_GTCSR                       (R_GPT3_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT3_GTUPSR                      (R_GPT3_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT3_GTDNSR                      (R_GPT3_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT3_GTICASR                     (R_GPT3_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT3_GTICBSR                     (R_GPT3_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT3_GTCR                        (R_GPT3_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT3_GTUDDTYC                    (R_GPT3_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT3_GTIOR                       (R_GPT3_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT3_GTINTAD                     (R_GPT3_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT3_GTST                        (R_GPT3_BASE + R_GPT_GTST_OFFSET)
#define R_GPT3_GTBER                       (R_GPT3_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT3_GTCNT                       (R_GPT3_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT3_GTCCRA                      (R_GPT3_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT3_GTCCRB                      (R_GPT3_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT3_GTCCRC                      (R_GPT3_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT3_GTCCRE                      (R_GPT3_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT3_GTCCRD                      (R_GPT3_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT3_GTCCRF                      (R_GPT3_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT3_GTPR                        (R_GPT3_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT3_GTPBR                       (R_GPT3_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT3_GTADTRA                     (R_GPT3_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT3_GTADTBRA                    (R_GPT3_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT3_GTADTDBRA                   (R_GPT3_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT3_GTADTRB                     (R_GPT3_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT3_GTADTBRB                    (R_GPT3_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT3_GTADTDBRB                   (R_GPT3_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT3_GTDTCR                      (R_GPT3_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT3_GTDVU                       (R_GPT3_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT3_GTADSMR                     (R_GPT3_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT3_GTICLF                      (R_GPT3_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT3_GTPC                        (R_GPT3_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT3_GTSECSR                     (R_GPT3_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT3_GTSECR                      (R_GPT3_BASE + R_GPT_GTSECR_OFFSET)

/* GPT4 Registers */

#define R_GPT4_GTWP                        (R_GPT4_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT4_GTSTR                       (R_GPT4_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT4_GTSTP                       (R_GPT4_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT4_GTCLR                       (R_GPT4_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT4_GTSSR                       (R_GPT4_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT4_GTPSR                       (R_GPT4_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT4_GTCSR                       (R_GPT4_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT4_GTUPSR                      (R_GPT4_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT4_GTDNSR                      (R_GPT4_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT4_GTICASR                     (R_GPT4_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT4_GTICBSR                     (R_GPT4_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT4_GTCR                        (R_GPT4_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT4_GTUDDTYC                    (R_GPT4_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT4_GTIOR                       (R_GPT4_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT4_GTINTAD                     (R_GPT4_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT4_GTST                        (R_GPT4_BASE + R_GPT_GTST_OFFSET)
#define R_GPT4_GTBER                       (R_GPT4_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT4_GTCNT                       (R_GPT4_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT4_GTCCRA                      (R_GPT4_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT4_GTCCRB                      (R_GPT4_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT4_GTCCRC                      (R_GPT4_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT4_GTCCRE                      (R_GPT4_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT4_GTCCRD                      (R_GPT4_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT4_GTCCRF                      (R_GPT4_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT4_GTPR                        (R_GPT4_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT4_GTPBR                       (R_GPT4_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT4_GTADTRA                     (R_GPT4_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT4_GTADTBRA                    (R_GPT4_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT4_GTADTDBRA                   (R_GPT4_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT4_GTADTRB                     (R_GPT4_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT4_GTADTBRB                    (R_GPT4_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT4_GTADTDBRB                   (R_GPT4_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT4_GTDTCR                      (R_GPT4_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT4_GTDVU                       (R_GPT4_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT4_GTADSMR                     (R_GPT4_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT4_GTICLF                      (R_GPT4_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT4_GTPC                        (R_GPT4_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT4_GTSECSR                     (R_GPT4_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT4_GTSECR                      (R_GPT4_BASE + R_GPT_GTSECR_OFFSET)

/* GPT5 Registers */

#define R_GPT5_GTWP                        (R_GPT5_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT5_GTSTR                       (R_GPT5_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT5_GTSTP                       (R_GPT5_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT5_GTCLR                       (R_GPT5_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT5_GTSSR                       (R_GPT5_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT5_GTPSR                       (R_GPT5_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT5_GTCSR                       (R_GPT5_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT5_GTUPSR                      (R_GPT5_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT5_GTDNSR                      (R_GPT5_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT5_GTICASR                     (R_GPT5_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT5_GTICBSR                     (R_GPT5_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT5_GTCR                        (R_GPT5_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT5_GTUDDTYC                    (R_GPT5_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT5_GTIOR                       (R_GPT5_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT5_GTINTAD                     (R_GPT5_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT5_GTST                        (R_GPT5_BASE + R_GPT_GTST_OFFSET)
#define R_GPT5_GTBER                       (R_GPT5_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT5_GTCNT                       (R_GPT5_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT5_GTCCRA                      (R_GPT5_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT5_GTCCRB                      (R_GPT5_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT5_GTCCRC                      (R_GPT5_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT5_GTCCRE                      (R_GPT5_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT5_GTCCRD                      (R_GPT5_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT5_GTCCRF                      (R_GPT5_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT5_GTPR                        (R_GPT5_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT5_GTPBR                       (R_GPT5_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT5_GTADTRA                     (R_GPT5_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT5_GTADTBRA                    (R_GPT5_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT5_GTADTDBRA                   (R_GPT5_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT5_GTADTRB                     (R_GPT5_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT5_GTADTBRB                    (R_GPT5_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT5_GTADTDBRB                   (R_GPT5_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT5_GTDTCR                      (R_GPT5_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT5_GTDVU                       (R_GPT5_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT5_GTADSMR                     (R_GPT5_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT5_GTICLF                      (R_GPT5_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT5_GTPC                        (R_GPT5_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT5_GTSECSR                     (R_GPT5_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT5_GTSECR                      (R_GPT5_BASE + R_GPT_GTSECR_OFFSET)

/* GPT6 Registers */

#define R_GPT6_GTWP                        (R_GPT6_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT6_GTSTR                       (R_GPT6_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT6_GTSTP                       (R_GPT6_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT6_GTCLR                       (R_GPT6_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT6_GTSSR                       (R_GPT6_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT6_GTPSR                       (R_GPT6_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT6_GTCSR                       (R_GPT6_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT6_GTUPSR                      (R_GPT6_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT6_GTDNSR                      (R_GPT6_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT6_GTICASR                     (R_GPT6_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT6_GTICBSR                     (R_GPT6_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT6_GTCR                        (R_GPT6_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT6_GTUDDTYC                    (R_GPT6_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT6_GTIOR                       (R_GPT6_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT6_GTINTAD                     (R_GPT6_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT6_GTST                        (R_GPT6_BASE + R_GPT_GTST_OFFSET)
#define R_GPT6_GTBER                       (R_GPT6_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT6_GTCNT                       (R_GPT6_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT6_GTCCRA                      (R_GPT6_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT6_GTCCRB                      (R_GPT6_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT6_GTCCRC                      (R_GPT6_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT6_GTCCRE                      (R_GPT6_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT6_GTCCRD                      (R_GPT6_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT6_GTCCRF                      (R_GPT6_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT6_GTPR                        (R_GPT6_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT6_GTPBR                       (R_GPT6_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT6_GTADTRA                     (R_GPT6_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT6_GTADTBRA                    (R_GPT6_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT6_GTADTDBRA                   (R_GPT6_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT6_GTADTRB                     (R_GPT6_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT6_GTADTBRB                    (R_GPT6_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT6_GTADTDBRB                   (R_GPT6_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT6_GTDTCR                      (R_GPT6_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT6_GTDVU                       (R_GPT6_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT6_GTADSMR                     (R_GPT6_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT6_GTICLF                      (R_GPT6_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT6_GTPC                        (R_GPT6_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT6_GTSECSR                     (R_GPT6_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT6_GTSECR                      (R_GPT6_BASE + R_GPT_GTSECR_OFFSET)

/* GPT7 Registers */

#define R_GPT7_GTWP                        (R_GPT7_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT7_GTSTR                       (R_GPT7_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT7_GTSTP                       (R_GPT7_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT7_GTCLR                       (R_GPT7_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT7_GTSSR                       (R_GPT7_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT7_GTPSR                       (R_GPT7_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT7_GTCSR                       (R_GPT7_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT7_GTUPSR                      (R_GPT7_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT7_GTDNSR                      (R_GPT7_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT7_GTICASR                     (R_GPT7_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT7_GTICBSR                     (R_GPT7_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT7_GTCR                        (R_GPT7_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT7_GTUDDTYC                    (R_GPT7_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT7_GTIOR                       (R_GPT7_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT7_GTINTAD                     (R_GPT7_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT7_GTST                        (R_GPT7_BASE + R_GPT_GTST_OFFSET)
#define R_GPT7_GTBER                       (R_GPT7_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT7_GTCNT                       (R_GPT7_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT7_GTCCRA                      (R_GPT7_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT7_GTCCRB                      (R_GPT7_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT7_GTCCRC                      (R_GPT7_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT7_GTCCRE                      (R_GPT7_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT7_GTCCRD                      (R_GPT7_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT7_GTCCRF                      (R_GPT7_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT7_GTPR                        (R_GPT7_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT7_GTPBR                       (R_GPT7_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT7_GTADTRA                     (R_GPT7_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT7_GTADTBRA                    (R_GPT7_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT7_GTADTDBRA                   (R_GPT7_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT7_GTADTRB                     (R_GPT7_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT7_GTADTBRB                    (R_GPT7_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT7_GTADTDBRB                   (R_GPT7_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT7_GTDTCR                      (R_GPT7_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT7_GTDVU                       (R_GPT7_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT7_GTADSMR                     (R_GPT7_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT7_GTICLF                      (R_GPT7_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT7_GTPC                        (R_GPT7_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT7_GTSECSR                     (R_GPT7_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT7_GTSECR                      (R_GPT7_BASE + R_GPT_GTSECR_OFFSET)

/* GPT8 Registers */

#define R_GPT8_GTWP                        (R_GPT8_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT8_GTSTR                       (R_GPT8_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT8_GTSTP                       (R_GPT8_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT8_GTCLR                       (R_GPT8_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT8_GTSSR                       (R_GPT8_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT8_GTPSR                       (R_GPT8_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT8_GTCSR                       (R_GPT8_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT8_GTUPSR                      (R_GPT8_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT8_GTDNSR                      (R_GPT8_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT8_GTICASR                     (R_GPT8_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT8_GTICBSR                     (R_GPT8_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT8_GTCR                        (R_GPT8_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT8_GTUDDTYC                    (R_GPT8_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT8_GTIOR                       (R_GPT8_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT8_GTINTAD                     (R_GPT8_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT8_GTST                        (R_GPT8_BASE + R_GPT_GTST_OFFSET)
#define R_GPT8_GTBER                       (R_GPT8_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT8_GTCNT                       (R_GPT8_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT8_GTCCRA                      (R_GPT8_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT8_GTCCRB                      (R_GPT8_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT8_GTCCRC                      (R_GPT8_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT8_GTCCRE                      (R_GPT8_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT8_GTCCRD                      (R_GPT8_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT8_GTCCRF                      (R_GPT8_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT8_GTPR                        (R_GPT8_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT8_GTPBR                       (R_GPT8_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT8_GTADTRA                     (R_GPT8_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT8_GTADTBRA                    (R_GPT8_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT8_GTADTDBRA                   (R_GPT8_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT8_GTADTRB                     (R_GPT8_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT8_GTADTBRB                    (R_GPT8_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT8_GTADTDBRB                   (R_GPT8_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT8_GTDTCR                      (R_GPT8_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT8_GTDVU                       (R_GPT8_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT8_GTADSMR                     (R_GPT8_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT8_GTICLF                      (R_GPT8_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT8_GTPC                        (R_GPT8_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT8_GTSECSR                     (R_GPT8_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT8_GTSECR                      (R_GPT8_BASE + R_GPT_GTSECR_OFFSET)

/* GPT9 Registers */

#define R_GPT9_GTWP                        (R_GPT9_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT9_GTSTR                       (R_GPT9_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT9_GTSTP                       (R_GPT9_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT9_GTCLR                       (R_GPT9_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT9_GTSSR                       (R_GPT9_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT9_GTPSR                       (R_GPT9_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT9_GTCSR                       (R_GPT9_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT9_GTUPSR                      (R_GPT9_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT9_GTDNSR                      (R_GPT9_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT9_GTICASR                     (R_GPT9_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT9_GTICBSR                     (R_GPT9_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT9_GTCR                        (R_GPT9_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT9_GTUDDTYC                    (R_GPT9_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT9_GTIOR                       (R_GPT9_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT9_GTINTAD                     (R_GPT9_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT9_GTST                        (R_GPT9_BASE + R_GPT_GTST_OFFSET)
#define R_GPT9_GTBER                       (R_GPT9_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT9_GTCNT                       (R_GPT9_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT9_GTCCRA                      (R_GPT9_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT9_GTCCRB                      (R_GPT9_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT9_GTCCRC                      (R_GPT9_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT9_GTCCRE                      (R_GPT9_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT9_GTCCRD                      (R_GPT9_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT9_GTCCRF                      (R_GPT9_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT9_GTPR                        (R_GPT9_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT9_GTPBR                       (R_GPT9_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT9_GTADTRA                     (R_GPT9_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT9_GTADTBRA                    (R_GPT9_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT9_GTADTDBRA                   (R_GPT9_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT9_GTADTRB                     (R_GPT9_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT9_GTADTBRB                    (R_GPT9_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT9_GTADTDBRB                   (R_GPT9_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT9_GTDTCR                      (R_GPT9_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT9_GTDVU                       (R_GPT9_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT9_GTADSMR                     (R_GPT9_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT9_GTICLF                      (R_GPT9_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT9_GTPC                        (R_GPT9_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT9_GTSECSR                     (R_GPT9_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT9_GTSECR                      (R_GPT9_BASE + R_GPT_GTSECR_OFFSET)

/* GPT10 Registers */

#define R_GPT10_GTWP                       (R_GPT10_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT10_GTSTR                      (R_GPT10_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT10_GTSTP                      (R_GPT10_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT10_GTCLR                      (R_GPT10_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT10_GTSSR                      (R_GPT10_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT10_GTPSR                      (R_GPT10_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT10_GTCSR                      (R_GPT10_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT10_GTUPSR                     (R_GPT10_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT10_GTDNSR                     (R_GPT10_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT10_GTICASR                    (R_GPT10_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT10_GTICBSR                    (R_GPT10_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT10_GTCR                       (R_GPT10_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT10_GTUDDTYC                   (R_GPT10_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT10_GTIOR                      (R_GPT10_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT10_GTINTAD                    (R_GPT10_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT10_GTST                       (R_GPT10_BASE + R_GPT_GTST_OFFSET)
#define R_GPT10_GTBER                      (R_GPT10_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT10_GTCNT                      (R_GPT10_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT10_GTCCRA                     (R_GPT10_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT10_GTCCRB                     (R_GPT10_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT10_GTCCRC                     (R_GPT10_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT10_GTCCRE                     (R_GPT10_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT10_GTCCRD                     (R_GPT10_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT10_GTCCRF                     (R_GPT10_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT10_GTPR                       (R_GPT10_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT10_GTPBR                      (R_GPT10_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT10_GTADTRA                    (R_GPT10_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT10_GTADTBRA                   (R_GPT10_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT10_GTADTDBRA                  (R_GPT10_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT10_GTADTRB                    (R_GPT10_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT10_GTADTBRB                   (R_GPT10_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT10_GTADTDBRB                  (R_GPT10_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT10_GTDTCR                     (R_GPT10_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT10_GTDVU                      (R_GPT10_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT10_GTADSMR                    (R_GPT10_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT10_GTICLF                     (R_GPT10_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT10_GTPC                       (R_GPT10_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT10_GTSECSR                    (R_GPT10_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT10_GTSECR                     (R_GPT10_BASE + R_GPT_GTSECR_OFFSET)

/* GPT11 Registers */

#define R_GPT11_GTWP                       (R_GPT11_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT11_GTSTR                      (R_GPT11_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT11_GTSTP                      (R_GPT11_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT11_GTCLR                      (R_GPT11_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT11_GTSSR                      (R_GPT11_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT11_GTPSR                      (R_GPT11_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT11_GTCSR                      (R_GPT11_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT11_GTUPSR                     (R_GPT11_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT11_GTDNSR                     (R_GPT11_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT11_GTICASR                    (R_GPT11_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT11_GTICBSR                    (R_GPT11_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT11_GTCR                       (R_GPT11_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT11_GTUDDTYC                   (R_GPT11_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT11_GTIOR                      (R_GPT11_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT11_GTINTAD                    (R_GPT11_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT11_GTST                       (R_GPT11_BASE + R_GPT_GTST_OFFSET)
#define R_GPT11_GTBER                      (R_GPT11_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT11_GTCNT                      (R_GPT11_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT11_GTCCRA                     (R_GPT11_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT11_GTCCRB                     (R_GPT11_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT11_GTCCRC                     (R_GPT11_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT11_GTCCRE                     (R_GPT11_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT11_GTCCRD                     (R_GPT11_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT11_GTCCRF                     (R_GPT11_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT11_GTPR                       (R_GPT11_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT11_GTPBR                      (R_GPT11_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT11_GTADTRA                    (R_GPT11_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT11_GTADTBRA                   (R_GPT11_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT11_GTADTDBRA                  (R_GPT11_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT11_GTADTRB                    (R_GPT11_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT11_GTADTBRB                   (R_GPT11_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT11_GTADTDBRB                  (R_GPT11_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT11_GTDTCR                     (R_GPT11_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT11_GTDVU                      (R_GPT11_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT11_GTADSMR                    (R_GPT11_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT11_GTICLF                     (R_GPT11_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT11_GTPC                       (R_GPT11_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT11_GTSECSR                    (R_GPT11_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT11_GTSECR                     (R_GPT11_BASE + R_GPT_GTSECR_OFFSET)

/* GPT12 Registers */

#define R_GPT12_GTWP                       (R_GPT12_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT12_GTSTR                      (R_GPT12_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT12_GTSTP                      (R_GPT12_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT12_GTCLR                      (R_GPT12_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT12_GTSSR                      (R_GPT12_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT12_GTPSR                      (R_GPT12_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT12_GTCSR                      (R_GPT12_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT12_GTUPSR                     (R_GPT12_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT12_GTDNSR                     (R_GPT12_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT12_GTICASR                    (R_GPT12_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT12_GTICBSR                    (R_GPT12_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT12_GTCR                       (R_GPT12_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT12_GTUDDTYC                   (R_GPT12_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT12_GTIOR                      (R_GPT12_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT12_GTINTAD                    (R_GPT12_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT12_GTST                       (R_GPT12_BASE + R_GPT_GTST_OFFSET)
#define R_GPT12_GTBER                      (R_GPT12_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT12_GTCNT                      (R_GPT12_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT12_GTCCRA                     (R_GPT12_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT12_GTCCRB                     (R_GPT12_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT12_GTCCRC                     (R_GPT12_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT12_GTCCRE                     (R_GPT12_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT12_GTCCRD                     (R_GPT12_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT12_GTCCRF                     (R_GPT12_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT12_GTPR                       (R_GPT12_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT12_GTPBR                      (R_GPT12_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT12_GTADTRA                    (R_GPT12_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT12_GTADTBRA                   (R_GPT12_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT12_GTADTDBRA                  (R_GPT12_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT12_GTADTRB                    (R_GPT12_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT12_GTADTBRB                   (R_GPT12_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT12_GTADTDBRB                  (R_GPT12_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT12_GTDTCR                     (R_GPT12_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT12_GTDVU                      (R_GPT12_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT12_GTADSMR                    (R_GPT12_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT12_GTICLF                     (R_GPT12_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT12_GTPC                       (R_GPT12_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT12_GTSECSR                    (R_GPT12_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT12_GTSECR                     (R_GPT12_BASE + R_GPT_GTSECR_OFFSET)

/* GPT13 Registers */

#define R_GPT13_GTWP                       (R_GPT13_BASE + R_GPT_GTWP_OFFSET)
#define R_GPT13_GTSTR                      (R_GPT13_BASE + R_GPT_GTSTR_OFFSET)
#define R_GPT13_GTSTP                      (R_GPT13_BASE + R_GPT_GTSTP_OFFSET)
#define R_GPT13_GTCLR                      (R_GPT13_BASE + R_GPT_GTCLR_OFFSET)
#define R_GPT13_GTSSR                      (R_GPT13_BASE + R_GPT_GTSSR_OFFSET)
#define R_GPT13_GTPSR                      (R_GPT13_BASE + R_GPT_GTPSR_OFFSET)
#define R_GPT13_GTCSR                      (R_GPT13_BASE + R_GPT_GTCSR_OFFSET)
#define R_GPT13_GTUPSR                     (R_GPT13_BASE + R_GPT_GTUPSR_OFFSET)
#define R_GPT13_GTDNSR                     (R_GPT13_BASE + R_GPT_GTDNSR_OFFSET)
#define R_GPT13_GTICASR                    (R_GPT13_BASE + R_GPT_GTICASR_OFFSET)
#define R_GPT13_GTICBSR                    (R_GPT13_BASE + R_GPT_GTICBSR_OFFSET)
#define R_GPT13_GTCR                       (R_GPT13_BASE + R_GPT_GTCR_OFFSET)
#define R_GPT13_GTUDDTYC                   (R_GPT13_BASE + R_GPT_GTUDDTYC_OFFSET)
#define R_GPT13_GTIOR                      (R_GPT13_BASE + R_GPT_GTIOR_OFFSET)
#define R_GPT13_GTINTAD                    (R_GPT13_BASE + R_GPT_GTINTAD_OFFSET)
#define R_GPT13_GTST                       (R_GPT13_BASE + R_GPT_GTST_OFFSET)
#define R_GPT13_GTBER                      (R_GPT13_BASE + R_GPT_GTBER_OFFSET)
#define R_GPT13_GTCNT                      (R_GPT13_BASE + R_GPT_GTCNT_OFFSET)
#define R_GPT13_GTCCRA                     (R_GPT13_BASE + R_GPT_GTCCRA_OFFSET)
#define R_GPT13_GTCCRB                     (R_GPT13_BASE + R_GPT_GTCCRB_OFFSET)
#define R_GPT13_GTCCRC                     (R_GPT13_BASE + R_GPT_GTCCRC_OFFSET)
#define R_GPT13_GTCCRE                     (R_GPT13_BASE + R_GPT_GTCCRE_OFFSET)
#define R_GPT13_GTCCRD                     (R_GPT13_BASE + R_GPT_GTCCRD_OFFSET)
#define R_GPT13_GTCCRF                     (R_GPT13_BASE + R_GPT_GTCCRF_OFFSET)
#define R_GPT13_GTPR                       (R_GPT13_BASE + R_GPT_GTPR_OFFSET)
#define R_GPT13_GTPBR                      (R_GPT13_BASE + R_GPT_GTPBR_OFFSET)
#define R_GPT13_GTADTRA                    (R_GPT13_BASE + R_GPT_GTADTRA_OFFSET)
#define R_GPT13_GTADTBRA                   (R_GPT13_BASE + R_GPT_GTADTBRA_OFFSET)
#define R_GPT13_GTADTDBRA                  (R_GPT13_BASE + R_GPT_GTADTDBRA_OFFSET)
#define R_GPT13_GTADTRB                    (R_GPT13_BASE + R_GPT_GTADTRB_OFFSET)
#define R_GPT13_GTADTBRB                   (R_GPT13_BASE + R_GPT_GTADTBRB_OFFSET)
#define R_GPT13_GTADTDBRB                  (R_GPT13_BASE + R_GPT_GTADTDBRB_OFFSET)
#define R_GPT13_GTDTCR                     (R_GPT13_BASE + R_GPT_GTDTCR_OFFSET)
#define R_GPT13_GTDVU                      (R_GPT13_BASE + R_GPT_GTDVU_OFFSET)
#define R_GPT13_GTADSMR                    (R_GPT13_BASE + R_GPT_GTADSMR_OFFSET)
#define R_GPT13_GTICLF                     (R_GPT13_BASE + R_GPT_GTICLF_OFFSET)
#define R_GPT13_GTPC                       (R_GPT13_BASE + R_GPT_GTPC_OFFSET)
#define R_GPT13_GTSECSR                    (R_GPT13_BASE + R_GPT_GTSECSR_OFFSET)
#define R_GPT13_GTSECR                     (R_GPT13_BASE + R_GPT_GTSECR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* General PWM Timer Write-Protection Register (32-bits) ********************/

#define R_GPT_GTWP_WP (1 <<  0)                                   /* 01: Register Write Disable */
#define R_GPT_GTWP_STRWP (1 <<  1)                                /* 02: GTSTR.CSTRT Bit Write Disabled */
#define R_GPT_GTWP_STPWP (1 <<  2)                                /* 04: GTSTP.CSTOP Bit Write Disabled */
#define R_GPT_GTWP_CLRWP (1 <<  3)                                /* 08: GTCLR.CCLR Bit Write Disabled */
#define R_GPT_GTWP_CMNWP (1 <<  4)                                /* 10: Common Register Write Disabled */
#define R_GPT_GTWP_PRKEY_SHIFT (8)
#define R_GPT_GTWP_PRKEY_MASK (0xff)
#  define R_GPT_GTWP_PRKEY_V0XA5 (165 << R_GPT_GTWP_PRKEY_SHIFT)  /* Written to these bits, the WP bits write is permitted. */

/* General PWM Timer Software Start Register (32-bits) **********************/

#define R_GPT_GTSTR_CSTRT0 (1 <<  0)   /* 01: Channel 0 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT1 (1 <<  1)   /* 02: Channel 1 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT2 (1 <<  2)   /* 04: Channel 2 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT3 (1 <<  3)   /* 08: Channel 3 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT4 (1 <<  4)   /* 10: Channel 4 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT5 (1 <<  5)   /* 20: Channel 5 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT6 (1 <<  6)   /* 40: Channel 6 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT7 (1 <<  7)   /* 80: Channel 7 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT8 (1 <<  8)   /* 100: Channel 8 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT9 (1 <<  9)   /* 200: Channel 9 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT10 (1 << 10)  /* 400: Channel 10 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT11 (1 << 11)  /* 800: Channel 11 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT12 (1 << 12)  /* 1000: Channel 12 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */
#define R_GPT_GTSTR_CSTRT13 (1 << 13)  /* 2000: Channel 13 GTCNT Count StartRead data shows each channel's counter status (GTCR.CST bit). 0 means counter stop. 1 means counter running. */

/* General PWM Timer Software Stop Register (32-bits) ***********************/

#define R_GPT_GTSTP_CSTOP0 (1 <<  0)   /* 01: Channel 0 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP1 (1 <<  1)   /* 02: Channel 1 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP2 (1 <<  2)   /* 04: Channel 2 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP3 (1 <<  3)   /* 08: Channel 3 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP4 (1 <<  4)   /* 10: Channel 4 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP5 (1 <<  5)   /* 20: Channel 5 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP6 (1 <<  6)   /* 40: Channel 6 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP7 (1 <<  7)   /* 80: Channel 7 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP8 (1 <<  8)   /* 100: Channel 8 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP9 (1 <<  9)   /* 200: Channel 9 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP10 (1 << 10)  /* 400: Channel 10 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP11 (1 << 11)  /* 800: Channel 11 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP12 (1 << 12)  /* 1000: Channel 12 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */
#define R_GPT_GTSTP_CSTOP13 (1 << 13)  /* 2000: Channel 13 GTCNT Count StopRead data shows each channel's counter status (GTCR.CST bit). 0 means counter running. 1 means counter stop. */

/* General PWM Timer Software Clear Register (32-bits) **********************/

#define R_GPT_GTCLR_CCLR0 (1 <<  0)   /* 01: Channel 0 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR1 (1 <<  1)   /* 02: Channel 1 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR2 (1 <<  2)   /* 04: Channel 2 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR3 (1 <<  3)   /* 08: Channel 3 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR4 (1 <<  4)   /* 10: Channel 4 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR5 (1 <<  5)   /* 20: Channel 5 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR6 (1 <<  6)   /* 40: Channel 6 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR7 (1 <<  7)   /* 80: Channel 7 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR8 (1 <<  8)   /* 100: Channel 8 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR9 (1 <<  9)   /* 200: Channel 9 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR10 (1 << 10)  /* 400: Channel 10 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR11 (1 << 11)  /* 800: Channel 11 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR12 (1 << 12)  /* 1000: Channel 12 GTCNT Count Clear */
#define R_GPT_GTCLR_CCLR13 (1 << 13)  /* 2000: Channel 13 GTCNT Count Clear */

/* General PWM Timer Start Source Select Register (32-bits) *****************/

#define R_GPT_GTSSR_SSGTRGAR (1 <<  0)  /* 01: GTETRGA Pin Rising Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSGTRGAF (1 <<  1)  /* 02: GTETRGA Pin Falling Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSGTRGBR (1 <<  2)  /* 04: GTETRGB Pin Rising Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSGTRGBF (1 <<  3)  /* 08: GTETRGB Pin Falling Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSGTRGCR (1 <<  4)  /* 10: GTETRGC Pin Rising Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSGTRGCF (1 <<  5)  /* 20: GTETRGC Pin Falling Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSGTRGDR (1 <<  6)  /* 40: GTETRGD Pin Rising Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSGTRGDF (1 <<  7)  /* 80: GTETRGD Pin Falling Input Source Counter Start Enable */
#define R_GPT_GTSSR_SSCARBL (1 <<  8)   /* 100: GTIOCA Pin Rising Input during GTIOCB Value Low Source Counter Start Enable */
#define R_GPT_GTSSR_SSCARBH (1 <<  9)   /* 200: GTIOCA Pin Rising Input during GTIOCB Value High Source Counter Start Enable */
#define R_GPT_GTSSR_SSCAFBL (1 << 10)   /* 400: GTIOCA Pin Falling Input during GTIOCB Value Low Source Counter Start Enable */
#define R_GPT_GTSSR_SSCAFBH (1 << 11)   /* 800: GTIOCA Pin Falling Input during GTIOCB Value High Source Counter Start Enable */
#define R_GPT_GTSSR_SSCBRAL (1 << 12)   /* 1000: GTIOCB Pin Rising Input during GTIOCA Value Low Source Counter Start Enable */
#define R_GPT_GTSSR_SSCBRAH (1 << 13)   /* 2000: GTIOCB Pin Rising Input during GTIOCA Value High Source Counter Start Enable */
#define R_GPT_GTSSR_SSCBFAL (1 << 14)   /* 4000: GTIOCB Pin Falling Input during GTIOCA Value Low Source Counter Start Enable */
#define R_GPT_GTSSR_SSCBFAH (1 << 15)   /* 8000: GTIOCB Pin Falling Input during GTIOCA Value High Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCA (1 << 16)    /* 10000: ELC_GPTA Event Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCB (1 << 17)    /* 20000: ELC_GPTB Event Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCC (1 << 18)    /* 40000: ELC_GPTC Event Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCD (1 << 19)    /* 80000: ELC_GPTD Event Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCE (1 << 20)    /* 100000: ELC_GPTE Event Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCF (1 << 21)    /* 200000: ELC_GPTF Event Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCG (1 << 22)    /* 400000: ELC_GPTG Event Source Counter Start Enable */
#define R_GPT_GTSSR_SSELCH (1 << 23)    /* 800000: ELC_GPTH Event Source Counter Start Enable */
#define R_GPT_GTSSR_CSTRT (1 << 31)     /* 80000000: Software Source Counter Start Enable */

/* General PWM Timer Stop Source Select Register (32-bits) ******************/

#define R_GPT_GTPSR_PSGTRGAR (1 <<  0)  /* 01: GTETRGA Pin Rising Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSGTRGAF (1 <<  1)  /* 02: GTETRGA Pin Falling Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSGTRGBR (1 <<  2)  /* 04: GTETRGB Pin Rising Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSGTRGBF (1 <<  3)  /* 08: GTETRGB Pin Falling Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSGTRGCR (1 <<  4)  /* 10: GTETRGC Pin Rising Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSGTRGCF (1 <<  5)  /* 20: GTETRGC Pin Falling Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSGTRGDR (1 <<  6)  /* 40: GTETRGD Pin Rising Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSGTRGDF (1 <<  7)  /* 80: GTETRGD Pin Falling Input Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCARBL (1 <<  8)   /* 100: GTIOCA Pin Rising Input during GTIOCB Value Low Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCARBH (1 <<  9)   /* 200: GTIOCA Pin Rising Input during GTIOCB Value High Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCAFBL (1 << 10)   /* 400: GTIOCA Pin Falling Input during GTIOCB Value Low Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCAFBH (1 << 11)   /* 800: GTIOCA Pin Falling Input during GTIOCB Value High Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCBRAL (1 << 12)   /* 1000: GTIOCB Pin Rising Input during GTIOCA Value Low Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCBRAH (1 << 13)   /* 2000: GTIOCB Pin Rising Input during GTIOCA Value High Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCBFAL (1 << 14)   /* 4000: GTIOCB Pin Falling Input during GTIOCA Value Low Source Counter Stop Enable */
#define R_GPT_GTPSR_PSCBFAH (1 << 15)   /* 8000: GTIOCB Pin Falling Input during GTIOCA Value High Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCA (1 << 16)    /* 10000: ELC_GPTA Event Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCB (1 << 17)    /* 20000: ELC_GPTB Event Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCC (1 << 18)    /* 40000: ELC_GPTC Event Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCD (1 << 19)    /* 80000: ELC_GPTD Event Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCE (1 << 20)    /* 100000: ELC_GPTE Event Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCF (1 << 21)    /* 200000: ELC_GPTF Event Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCG (1 << 22)    /* 400000: ELC_GPTG Event Source Counter Stop Enable */
#define R_GPT_GTPSR_PSELCH (1 << 23)    /* 800000: ELC_GPTH Event Source Counter Stop Enable */
#define R_GPT_GTPSR_CSTOP (1 << 31)     /* 80000000: Software Source Counter Stop Enable */

/* General PWM Timer Clear Source Select Register (32-bits) *****************/

#define R_GPT_GTCSR_CSGTRGAR (1 <<  0)  /* 01: GTETRGA Pin Rising Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSGTRGAF (1 <<  1)  /* 02: GTETRGA Pin Falling Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSGTRGBR (1 <<  2)  /* 04: GTETRGB Pin Rising Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSGTRGBF (1 <<  3)  /* 08: GTETRGB Pin Falling Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSGTRGCR (1 <<  4)  /* 10: GTETRGC Pin Rising Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSGTRGCF (1 <<  5)  /* 20: GTETRGC Pin Falling Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSGTRGDR (1 <<  6)  /* 40: GTETRGD Pin Rising Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSGTRGDF (1 <<  7)  /* 80: GTETRGD Pin Falling Input Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCARBL (1 <<  8)   /* 100: GTIOCA Pin Rising Input during GTIOCB Value Low Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCARBH (1 <<  9)   /* 200: GTIOCA Pin Rising Input during GTIOCB Value High Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCAFBL (1 << 10)   /* 400: GTIOCA Pin Falling Input during GTIOCB Value Low Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCAFBH (1 << 11)   /* 800: GTIOCA Pin Falling Input during GTIOCB Value High Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCBRAL (1 << 12)   /* 1000: GTIOCB Pin Rising Input during GTIOCA Value Low Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCBRAH (1 << 13)   /* 2000: GTIOCB Pin Rising Input during GTIOCA Value High Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCBFAL (1 << 14)   /* 4000: GTIOCB Pin Falling Input during GTIOCA Value Low Source Counter Clear Enable */
#define R_GPT_GTCSR_CSCBFAH (1 << 15)   /* 8000: GTIOCB Pin Falling Input during GTIOCA Value High Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCA (1 << 16)    /* 10000: ELC_GPTA Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCB (1 << 17)    /* 20000: ELC_GPTB Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCC (1 << 18)    /* 40000: ELC_GPTC Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCD (1 << 19)    /* 80000: ELC_GPTD Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCE (1 << 20)    /* 100000: ELC_GPTE Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCF (1 << 21)    /* 200000: ELC_GPTF Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCG (1 << 22)    /* 400000: ELC_GPTG Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CSELCH (1 << 23)    /* 800000: ELC_GPTH Event Source Counter Clear Enable */
#define R_GPT_GTCSR_CCLR (1 << 31)      /* 80000000: Software Source Counter Clear Enable */

/* General PWM Timer Up Count Source Select Register (32-bits) **************/

#define R_GPT_GTUPSR_USGTRGAR (1 <<  0)  /* 01: GTETRGA Pin Rising Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USGTRGAF (1 <<  1)  /* 02: GTETRGA Pin Falling Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USGTRGBR (1 <<  2)  /* 04: GTETRGB Pin Rising Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USGTRGBF (1 <<  3)  /* 08: GTETRGB Pin Falling Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USGTRGCR (1 <<  4)  /* 10: GTETRGC Pin Rising Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USGTRGCF (1 <<  5)  /* 20: GTETRGC Pin Falling Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USGTRGDR (1 <<  6)  /* 40: GTETRGD Pin Rising Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USGTRGDF (1 <<  7)  /* 80: GTETRGD Pin Falling Input Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCARBL (1 <<  8)   /* 100: GTIOCA Pin Rising Input during GTIOCB Value Low Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCARBH (1 <<  9)   /* 200: GTIOCA Pin Rising Input during GTIOCB Value High Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCAFBL (1 << 10)   /* 400: GTIOCA Pin Falling Input during GTIOCB Value Low Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCAFBH (1 << 11)   /* 800: GTIOCA Pin Falling Input during GTIOCB Value High Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCBRAL (1 << 12)   /* 1000: GTIOCB Pin Rising Input during GTIOCA Value Low Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCBRAH (1 << 13)   /* 2000: GTIOCB Pin Rising Input during GTIOCA Value High Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCBFAL (1 << 14)   /* 4000: GTIOCB Pin Falling Input during GTIOCA Value Low Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USCBFAH (1 << 15)   /* 8000: GTIOCB Pin Falling Input during GTIOCA Value High Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCA (1 << 16)    /* 10000: ELC_GPTA Event Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCB (1 << 17)    /* 20000: ELC_GPTB Event Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCC (1 << 18)    /* 40000: ELC_GPTC Event Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCD (1 << 19)    /* 80000: ELC_GPTD Event Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCE (1 << 20)    /* 100000: ELC_GPTE Event Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCF (1 << 21)    /* 200000: ELC_GPTF Event Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCG (1 << 22)    /* 400000: ELC_GPTG Event Source Counter Count Up Enable */
#define R_GPT_GTUPSR_USELCH (1 << 23)    /* 800000: ELC_GPTH Event Source Counter Count Up Enable */

/* General PWM Timer Down Count Source Select Register (32-bits) ************/

#define R_GPT_GTDNSR_DSGTRGAR (1 <<  0)  /* 01: GTETRGA Pin Rising Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSGTRGAF (1 <<  1)  /* 02: GTETRGA Pin Falling Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSGTRGBR (1 <<  2)  /* 04: GTETRGB Pin Rising Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSGTRGBF (1 <<  3)  /* 08: GTETRGB Pin Falling Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSGTRGCR (1 <<  4)  /* 10: GTETRGC Pin Rising Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSGTRGCF (1 <<  5)  /* 20: GTETRGC Pin Falling Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSGTRGDR (1 <<  6)  /* 40: GTETRGD Pin Rising Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSGTRGDF (1 <<  7)  /* 80: GTETRGD Pin Falling Input Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCARBL (1 <<  8)   /* 100: GTIOCA Pin Rising Input during GTIOCB Value Low Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCARBH (1 <<  9)   /* 200: GTIOCA Pin Rising Input during GTIOCB Value High Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCAFBL (1 << 10)   /* 400: GTIOCA Pin Falling Input during GTIOCB Value Low Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCAFBH (1 << 11)   /* 800: GTIOCA Pin Falling Input during GTIOCB Value High Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCBRAL (1 << 12)   /* 1000: GTIOCB Pin Rising Input during GTIOCA Value Low Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCBRAH (1 << 13)   /* 2000: GTIOCB Pin Rising Input during GTIOCA Value High Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCBFAL (1 << 14)   /* 4000: GTIOCB Pin Falling Input during GTIOCA Value Low Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSCBFAH (1 << 15)   /* 8000: GTIOCB Pin Falling Input during GTIOCA Value High Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCA (1 << 16)    /* 10000: ELC_GPTA Event Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCB (1 << 17)    /* 20000: ELC_GPTB Event Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCC (1 << 18)    /* 40000: ELC_GPTC Event Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCD (1 << 19)    /* 80000: ELC_GPTD Event Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCE (1 << 20)    /* 100000: ELC_GPTE Event Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCF (1 << 21)    /* 200000: ELC_GPTF Event Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCG (1 << 22)    /* 400000: ELC_GPTG Event Source Counter Count Down Enable */
#define R_GPT_GTDNSR_DSELCH (1 << 23)    /* 800000: ELC_GPTH Event Source Counter Count Down Enable */

/* General PWM Timer Input Capture Source Select Register A (32-bits) *******/

#define R_GPT_GTICASR_ASGTRGAR (1 <<  0)  /* 01: GTETRGA Pin Rising Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASGTRGAF (1 <<  1)  /* 02: GTETRGA Pin Falling Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASGTRGBR (1 <<  2)  /* 04: GTETRGB Pin Rising Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASGTRGBF (1 <<  3)  /* 08: GTETRGB Pin Falling Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASGTRGCR (1 <<  4)  /* 10: GTETRGC Pin Rising Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASGTRGCF (1 <<  5)  /* 20: GTETRGC Pin Falling Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASGTRGDR (1 <<  6)  /* 40: GTETRGD Pin Rising Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASGTRGDF (1 <<  7)  /* 80: GTETRGD Pin Falling Input Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCARBL (1 <<  8)   /* 100: GTIOCA Pin Rising Input during GTIOCB Value Low Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCARBH (1 <<  9)   /* 200: GTIOCA Pin Rising Input during GTIOCB Value High Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCAFBL (1 << 10)   /* 400: GTIOCA Pin Falling Input during GTIOCB Value Low Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCAFBH (1 << 11)   /* 800: GTIOCA Pin Falling Input during GTIOCB Value High Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCBRAL (1 << 12)   /* 1000: GTIOCB Pin Rising Input during GTIOCA Value Low Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCBRAH (1 << 13)   /* 2000: GTIOCB Pin Rising Input during GTIOCA Value High Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCBFAL (1 << 14)   /* 4000: GTIOCB Pin Falling Input during GTIOCA Value Low Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASCBFAH (1 << 15)   /* 8000: GTIOCB Pin Falling Input during GTIOCA Value High Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCA (1 << 16)    /* 10000: ELC_GPTA Event Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCB (1 << 17)    /* 20000: ELC_GPTB Event Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCC (1 << 18)    /* 40000: ELC_GPTC Event Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCD (1 << 19)    /* 80000: ELC_GPTD Event Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCE (1 << 20)    /* 100000: ELC_GPTE Event Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCF (1 << 21)    /* 200000: ELC_GPTF Event Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCG (1 << 22)    /* 400000: ELC_GPTG Event Source GTCCRA Input Capture Enable */
#define R_GPT_GTICASR_ASELCH (1 << 23)    /* 800000: ELC_GPTH Event Source GTCCRA Input Capture Enable */

/* General PWM Timer Input Capture Source Select Register B (32-bits) *******/

#define R_GPT_GTICBSR_BSGTRGAR (1 <<  0)  /* 01: GTETRGA Pin Rising Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSGTRGAF (1 <<  1)  /* 02: GTETRGA Pin Falling Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSGTRGBR (1 <<  2)  /* 04: GTETRGB Pin Rising Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSGTRGBF (1 <<  3)  /* 08: GTETRGB Pin Falling Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSGTRGCR (1 <<  4)  /* 10: GTETRGC Pin Rising Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSGTRGCF (1 <<  5)  /* 20: GTETRGC Pin Falling Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSGTRGDR (1 <<  6)  /* 40: GTETRGD Pin Rising Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSGTRGDF (1 <<  7)  /* 80: GTETRGD Pin Falling Input Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCARBL (1 <<  8)   /* 100: GTIOCA Pin Rising Input during GTIOCB Value Low Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCARBH (1 <<  9)   /* 200: GTIOCA Pin Rising Input during GTIOCB Value High Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCAFBL (1 << 10)   /* 400: GTIOCA Pin Falling Input during GTIOCB Value Low Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCAFBH (1 << 11)   /* 800: GTIOCA Pin Falling Input during GTIOCB Value High Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCBRAL (1 << 12)   /* 1000: GTIOCB Pin Rising Input during GTIOCA Value Low Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCBRAH (1 << 13)   /* 2000: GTIOCB Pin Rising Input during GTIOCA Value High Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCBFAL (1 << 14)   /* 4000: GTIOCB Pin Falling Input during GTIOCA Value Low Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSCBFAH (1 << 15)   /* 8000: GTIOCB Pin Falling Input during GTIOCA Value High Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCA (1 << 16)    /* 10000: ELC_GPTA Event Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCB (1 << 17)    /* 20000: ELC_GPTB Event Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCC (1 << 18)    /* 40000: ELC_GPTC Event Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCD (1 << 19)    /* 80000: ELC_GPTD Event Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCE (1 << 20)    /* 100000: ELC_GPTE Event Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCF (1 << 21)    /* 200000: ELC_GPTF Event Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCG (1 << 22)    /* 400000: ELC_GPTG Event Source GTCCRB Input Capture Enable */
#define R_GPT_GTICBSR_BSELCH (1 << 23)    /* 800000: ELC_GPTH Event Source GTCCRB Input Capture Enable */

/* General PWM Timer Control Register (32-bits) *****************************/

#define R_GPT_GTCR_CST (1 <<  0)                                              /* 01: Count Start */
#define R_GPT_GTCR_MD_SHIFT (16)
#define R_GPT_GTCR_MD_MASK (0x7)
#  define R_GPT_GTCR_MD_V000 (0 << R_GPT_GTCR_MD_SHIFT)                       /* Saw-wave PWM mode (single buffer or double buffer possible) */
#  define R_GPT_GTCR_MD_V001 (1 << R_GPT_GTCR_MD_SHIFT)                       /* Saw-wave one-shot pulse mode (fixed buffer operation) */
#  define R_GPT_GTCR_MD_SETTING_PROHIBITED_2 (2 << R_GPT_GTCR_MD_SHIFT)       /* Setting prohibited */
#  define R_GPT_GTCR_MD_SETTING_PROHIBITED_3 (3 << R_GPT_GTCR_MD_SHIFT)       /* Setting prohibited */
#  define R_GPT_GTCR_MD_V100 (4 << R_GPT_GTCR_MD_SHIFT)                       /* Triangle-wave PWM mode 1 (32-bit transfer at crest) (single buffer or double buffer possible) */
#  define R_GPT_GTCR_MD_V101 (5 << R_GPT_GTCR_MD_SHIFT)                       /* Triangle-wave PWM mode 2 (32-bit transfer at crest and trough) (single buffer or double buffer possible) */
#  define R_GPT_GTCR_MD_V110 (6 << R_GPT_GTCR_MD_SHIFT)                       /* Triangle-wave PWM mode 3 (64-bit transfer at trough) fixed buffer operation) */
#  define R_GPT_GTCR_MD_SETTING_PROHIBITED_7 (7 << R_GPT_GTCR_MD_SHIFT)       /* Setting prohibited */
#define R_GPT_GTCR_TPCS_SHIFT (23)
#define R_GPT_GTCR_TPCS_MASK (0xf)
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_1 (0 << R_GPT_GTCR_TPCS_SHIFT)        /* PCLKGPTnPCLKC/1 */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_2 (1 << R_GPT_GTCR_TPCS_SHIFT)        /* PCLKGPTnPCLKC/2 */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_4 (2 << R_GPT_GTCR_TPCS_SHIFT)        /* PCLKGPTnPCLKC/4 */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_8 (3 << R_GPT_GTCR_TPCS_SHIFT)        /* PCLKGPTnPCLKC/8 */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_16 (4 << R_GPT_GTCR_TPCS_SHIFT)       /* PCLKGPTnPCLKC/16 */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_32 (5 << R_GPT_GTCR_TPCS_SHIFT)       /* PCLKGPTnPCLKC/32 */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_64 (6 << R_GPT_GTCR_TPCS_SHIFT)       /* PCLKGPTnPCLKC/64 */
#  define R_GPT_GTCR_TPCS_V0111 (7 << R_GPT_GTCR_TPCS_SHIFT)                  /* Setting prohibited(PCLKGPTn) */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_256 (8 << R_GPT_GTCR_TPCS_SHIFT)      /* PCLKGPTnPCLKC/256 */
#  define R_GPT_GTCR_TPCS_V1001 (9 << R_GPT_GTCR_TPCS_SHIFT)                  /* Setting prohibited(PCLKGPTn) */
#  define R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_1024 (10 << R_GPT_GTCR_TPCS_SHIFT)    /* PCLKGPTnPCLKC/1024 */
#  define R_GPT_GTCR_TPCS_V1011 (11 << R_GPT_GTCR_TPCS_SHIFT)                 /* Setting prohibited(PCLKGPTn) */
#  define R_GPT_GTCR_TPCS_GTETRGA_VIA_THE_POEG (12 << R_GPT_GTCR_TPCS_SHIFT)  /* GTETRGA (via the POEG) */
#  define R_GPT_GTCR_TPCS_GTETRGB_VIA_THE_POEG (13 << R_GPT_GTCR_TPCS_SHIFT)  /* GTETRGB (via the POEG) */
#  define R_GPT_GTCR_TPCS_GTETRGC_VIA_THE_POEG (14 << R_GPT_GTCR_TPCS_SHIFT)  /* GTETRGC (via the POEG) */
#  define R_GPT_GTCR_TPCS_GTETRGD_VIA_THE_POEG (15 << R_GPT_GTCR_TPCS_SHIFT)  /* GTETRGD (via the POEG) */

/* General PWM Timer Count Direction and Duty Setting Register (32-bits) ****/

#define R_GPT_GTUDDTYC_UD (1 <<  0)                                                        /* 01: Count Direction Setting */
#define R_GPT_GTUDDTYC_UDF (1 <<  1)                                                       /* 02: Forcible Count Direction Setting */
#define R_GPT_GTUDDTYC_OADTY_SHIFT (16)
#define R_GPT_GTUDDTYC_OADTY_MASK (0x3)
#  define R_GPT_GTUDDTYC_OADTY_V00 (0 << R_GPT_GTUDDTYC_OADTY_SHIFT)                       /* GTIOCA pin duty is depend on compare match */
#  define R_GPT_GTUDDTYC_OADTY_V01 (1 << R_GPT_GTUDDTYC_OADTY_SHIFT)                       /* GTIOCA pin duty is depend on compare match */
#  define R_GPT_GTUDDTYC_OADTY_V10 (2 << R_GPT_GTUDDTYC_OADTY_SHIFT)                       /* GTIOCA pin duty 0 percent */
#  define R_GPT_GTUDDTYC_OADTY_V11 (3 << R_GPT_GTUDDTYC_OADTY_SHIFT)                       /* GTIOCA pin duty 100 percent */
#define R_GPT_GTUDDTYC_OADTYF (1 << 18)                                                    /* 40000: Forcible GTIOCA Output Duty Setting */
#define R_GPT_GTUDDTYC_OADTYR (1 << 19)                                                    /* 80000: GTIOCA Output Value Selecting after Releasing 0 percent/100 percent Duty Setting */
#define R_GPT_GTUDDTYC_OBDTY_SHIFT (24)
#define R_GPT_GTUDDTYC_OBDTY_MASK (0x3)
#  define R_GPT_GTUDDTYC_OBDTY_V00 (0 << R_GPT_GTUDDTYC_OBDTY_SHIFT)                       /* GTIOCB pin duty is depend on compare match */
#  define R_GPT_GTUDDTYC_OBDTY_V01 (1 << R_GPT_GTUDDTYC_OBDTY_SHIFT)                       /* GTIOCB pin duty is depend on compare match */
#  define R_GPT_GTUDDTYC_OBDTY_GTIOCB_PIN_DUTY_0PERCENT (2 << R_GPT_GTUDDTYC_OBDTY_SHIFT)  /* GTIOCB pin duty 0percent */
#  define R_GPT_GTUDDTYC_OBDTY_V11 (3 << R_GPT_GTUDDTYC_OBDTY_SHIFT)                       /* GTIOCB pin duty 100percent */
#define R_GPT_GTUDDTYC_OBDTYF (1 << 26)                                                    /* 4000000: Forcible GTIOCB Output Duty Setting */
#define R_GPT_GTUDDTYC_OBDTYR (1 << 27)                                                    /* 8000000: GTIOCB Output Value Selecting after Releasing 0 percent/100 percent Duty Setting */

/* General PWM Timer I/O Control Register (32-bits) *************************/

#define R_GPT_GTIOR_GTIOA_SHIFT (0)
#define R_GPT_GTIOR_GTIOA_MASK (0x1f)
#  define R_GPT_GTIOR_GTIOA_V00000 (0 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Output retained at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V00001 (1 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Output retained at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V00010 (2 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Output retained at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V00011 (3 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Output retained at cycle end. Output toggled at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V00100 (4 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Low output at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V00101 (5 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Low output at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V00110 (6 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Low output at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V00111 (7 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. Low output at cycle end. Output toggled at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01000 (8 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. High output at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01001 (9 << R_GPT_GTIOR_GTIOA_SHIFT)                 /* Initial output is Low. High output at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01010 (10 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is Low. High output at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01011 (11 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is Low. High output at cycle end. Output toggled at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01100 (12 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is Low. Output toggled at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01101 (13 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is Low. Output toggled at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01110 (14 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is Low. Output toggled at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V01111 (15 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is Low. Output toggled at cycle end. Output toggled at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10000 (16 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output retained at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10001 (17 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output retained at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10010 (18 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output retained at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10011 (19 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output retained at cycle end. Output toggled at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10100 (20 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Low output at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10101 (21 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Low output at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10110 (22 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Low output at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V10111 (23 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Low output at cycle end. Output toggled at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11000 (24 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. High output at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11001 (25 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. High output at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11010 (26 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. High output at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11011 (27 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. High output at cycle end. Output toggled at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11100 (28 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output toggled at cycle end. Output retained at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11101 (29 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output toggled at cycle end. Low output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11110 (30 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output toggled at cycle end. High output at GTCCRA compare match. */
#  define R_GPT_GTIOR_GTIOA_V11111 (31 << R_GPT_GTIOR_GTIOA_SHIFT)                /* Initial output is High. Output toggled at cycle end. Output toggled at GTCCRA compare match. */
#define R_GPT_GTIOR_OADFLT (1 <<  6)                                              /* 40: GTIOCA Pin Output Value Setting at the Count Stop */
#define R_GPT_GTIOR_OAHLD (1 <<  7)                                               /* 80: GTIOCA Pin Output Setting at the Start/Stop Count */
#define R_GPT_GTIOR_OAE (1 <<  8)                                                 /* 100: GTIOCA Pin Output Enable */
#define R_GPT_GTIOR_OADF_SHIFT (9)
#define R_GPT_GTIOR_OADF_MASK (0x3)
#  define R_GPT_GTIOR_OADF_PROHIBIT_OUTPUT_DISABLE (0 << R_GPT_GTIOR_OADF_SHIFT)  /* Prohibit output disable */
#  define R_GPT_GTIOR_OADF_V01 (1 << R_GPT_GTIOR_OADF_SHIFT)                      /* Set GTIOCA pin to Hi-Z on output disable */
#  define R_GPT_GTIOR_OADF_V10 (2 << R_GPT_GTIOR_OADF_SHIFT)                      /* Set GTIOCA pin to 0 on output disable */
#  define R_GPT_GTIOR_OADF_V11 (3 << R_GPT_GTIOR_OADF_SHIFT)                      /* Set GTIOCA pin to 1 on output disable. */
#define R_GPT_GTIOR_NFAEN (1 << 13)                                               /* 2000: Noise Filter A Enable */
#define R_GPT_GTIOR_NFCSA_SHIFT (14)
#define R_GPT_GTIOR_NFCSA_MASK (0x3)
#  define R_GPT_GTIOR_NFCSA_PCLKGPTN_1 (0 << R_GPT_GTIOR_NFCSA_SHIFT)             /* PCLKGPTn/1 */
#  define R_GPT_GTIOR_NFCSA_PCLKGPTN_4 (1 << R_GPT_GTIOR_NFCSA_SHIFT)             /* PCLKGPTn/4 */
#  define R_GPT_GTIOR_NFCSA_PCLKGPTN_16 (2 << R_GPT_GTIOR_NFCSA_SHIFT)            /* PCLKGPTn/16 */
#  define R_GPT_GTIOR_NFCSA_PCLKGPTN_64 (3 << R_GPT_GTIOR_NFCSA_SHIFT)            /* PCLKGPTn/64 */
#define R_GPT_GTIOR_GTIOB_SHIFT (16)
#define R_GPT_GTIOR_GTIOB_MASK (0x1f)
#  define R_GPT_GTIOR_GTIOB_V00000 (0 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Output retained at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V00001 (1 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Output retained at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V00010 (2 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Output retained at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V00011 (3 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Output retained at cycle end. Output toggled at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V00100 (4 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Low output at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V00101 (5 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Low output at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V00110 (6 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Low output at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V00111 (7 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. Low output at cycle end. Output toggled at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01000 (8 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. High output at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01001 (9 << R_GPT_GTIOR_GTIOB_SHIFT)                 /* Initial output is Low. High output at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01010 (10 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is Low. High output at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01011 (11 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is Low. High output at cycle end. Output toggled at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01100 (12 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is Low. Output toggled at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01101 (13 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is Low. Output toggled at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01110 (14 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is Low. Output toggled at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V01111 (15 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is Low. Output toggled at cycle end. Output toggled at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10000 (16 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output retained at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10001 (17 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output retained at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10010 (18 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output retained at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10011 (19 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output retained at cycle end. Output toggled at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10100 (20 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Low output at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10101 (21 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Low output at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10110 (22 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Low output at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V10111 (23 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Low output at cycle end. Output toggled at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11000 (24 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. High output at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11001 (25 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. High output at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11010 (26 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. High output at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11011 (27 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. High output at cycle end. Output toggled at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11100 (28 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output toggled at cycle end. Output retained at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11101 (29 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output toggled at cycle end. Low output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11110 (30 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output toggled at cycle end. High output at GTCCRB compare match. */
#  define R_GPT_GTIOR_GTIOB_V11111 (31 << R_GPT_GTIOR_GTIOB_SHIFT)                /* Initial output is High. Output toggled at cycle end. Output toggled at GTCCRB compare match. */
#define R_GPT_GTIOR_OBDFLT (1 << 22)                                              /* 400000: GTIOCB Pin Output Value Setting at the Count Stop */
#define R_GPT_GTIOR_OBHLD (1 << 23)                                               /* 800000: GTIOCB Pin Output Setting at the Start/Stop Count */
#define R_GPT_GTIOR_OBE (1 << 24)                                                 /* 1000000: GTIOCB Pin Output Enable */
#define R_GPT_GTIOR_OBDF_SHIFT (25)
#define R_GPT_GTIOR_OBDF_MASK (0x3)
#  define R_GPT_GTIOR_OBDF_PROHIBIT_OUTPUT_DISABLE (0 << R_GPT_GTIOR_OBDF_SHIFT)  /* Prohibit output disable */
#  define R_GPT_GTIOR_OBDF_V01 (1 << R_GPT_GTIOR_OBDF_SHIFT)                      /* Set GTIOCB pin to Hi-Z on output disable */
#  define R_GPT_GTIOR_OBDF_V10 (2 << R_GPT_GTIOR_OBDF_SHIFT)                      /* Set GTIOCB pin to 0 on output disable */
#  define R_GPT_GTIOR_OBDF_V11 (3 << R_GPT_GTIOR_OBDF_SHIFT)                      /* Set GTIOCB pin to 1 on output disable. */
#define R_GPT_GTIOR_NFBEN (1 << 29)                                               /* 20000000: Noise Filter B Enable */
#define R_GPT_GTIOR_NFCSB_SHIFT (30)
#define R_GPT_GTIOR_NFCSB_MASK (0x3)
#  define R_GPT_GTIOR_NFCSB_PCLKGPTN_1 (0 << R_GPT_GTIOR_NFCSB_SHIFT)             /* PCLKGPTn/1 */
#  define R_GPT_GTIOR_NFCSB_PCLKGPTN_4 (1 << R_GPT_GTIOR_NFCSB_SHIFT)             /* PCLKGPTn/4 */
#  define R_GPT_GTIOR_NFCSB_PCLKGPTN_16 (2 << R_GPT_GTIOR_NFCSB_SHIFT)            /* PCLKGPTn/16 */
#  define R_GPT_GTIOR_NFCSB_PCLKGPTN_64 (3 << R_GPT_GTIOR_NFCSB_SHIFT)            /* PCLKGPTn/64 */

/* General PWM Timer Interrupt Output Setting Register (32-bits) ************/

#define R_GPT_GTINTAD_ADTRAUEN (1 << 16)                        /* 10000: GTADTRA Compare Match (Up-Counting) A/D Converter Start Request Interrupt Enable */
#define R_GPT_GTINTAD_ADTRADEN (1 << 17)                        /* 20000: GTADTRA Compare Match (Down-Counting) A/D Converter Start Request Interrupt Enable */
#define R_GPT_GTINTAD_ADTRBUEN (1 << 18)                        /* 40000: GTADTRB Compare Match (Up-Counting) A/D Converter Start Request Interrupt Enable */
#define R_GPT_GTINTAD_ADTRBDEN (1 << 19)                        /* 80000: GTADTRB Compare Match (Down-Counting) A/D Converter Start Request Interrupt Enable */
#define R_GPT_GTINTAD_GRP_SHIFT (24)
#define R_GPT_GTINTAD_GRP_MASK (0x3)
#  define R_GPT_GTINTAD_GRP_V00 (0 << R_GPT_GTINTAD_GRP_SHIFT)  /* Select Group A output disable request */
#  define R_GPT_GTINTAD_GRP_V01 (1 << R_GPT_GTINTAD_GRP_SHIFT)  /* Select Group B output disable request */
#  define R_GPT_GTINTAD_GRP_V10 (2 << R_GPT_GTINTAD_GRP_SHIFT)  /* Select Group C output disable request */
#  define R_GPT_GTINTAD_GRP_V11 (3 << R_GPT_GTINTAD_GRP_SHIFT)  /* Select Group D output disable request. */
#define R_GPT_GTINTAD_GRPABH (1 << 29)                          /* 20000000: Same Time Output Level High Disable Request Enable */
#define R_GPT_GTINTAD_GRPABL (1 << 30)                          /* 40000000: Same Time Output Level Low Disable Request Enable */

/* General PWM Timer Status Register (32-bits) ******************************/

#define R_GPT_GTST_TCFA (1 <<  0)     /* 01: Input Capture/Compare Match Flag A */
#define R_GPT_GTST_TCFB (1 <<  1)     /* 02: Input Capture/Compare Match Flag B */
#define R_GPT_GTST_TCFC (1 <<  2)     /* 04: Input Compare Match Flag C */
#define R_GPT_GTST_TCFD (1 <<  3)     /* 08: Input Compare Match Flag D */
#define R_GPT_GTST_TCFE (1 <<  4)     /* 10: Input Compare Match Flag E */
#define R_GPT_GTST_TCFF (1 <<  5)     /* 20: Input Compare Match Flag F */
#define R_GPT_GTST_TCFPO (1 <<  6)    /* 40: Overflow Flag */
#define R_GPT_GTST_TCFPU (1 <<  7)    /* 80: Underflow Flag */
#define R_GPT_GTST_TUCF (1 << 15)     /* 8000: Count Direction Flag */
#define R_GPT_GTST_ADTRAUF (1 << 16)  /* 10000: GTADTRA Register Compare Match(Up-Counting) A/D Converter Start Request Flag */
#define R_GPT_GTST_ADTRADF (1 << 17)  /* 20000: GTADTRA Register Compare Match(Down-Counting) A/D Converter Start Request Flag */
#define R_GPT_GTST_ADTRBUF (1 << 18)  /* 40000: GTADTRB Register Compare Match(Up-Counting) A/D Converter Start Request Flag */
#define R_GPT_GTST_ADTRBDF (1 << 19)  /* 80000: GTADTRB Register Compare Match(Down-Counting) A/D Converter Start Request Flag */
#define R_GPT_GTST_ODF (1 << 24)      /* 1000000: Output Disable Flag */
#define R_GPT_GTST_OABHF (1 << 29)    /* 20000000: Same Time Output Level High Disable Request Enable */
#define R_GPT_GTST_OABLF (1 << 30)    /* 40000000: Same Time Output Level Low Disable Request Enable */
#define R_GPT_GTST_PCF (1 << 31)      /* 80000000: Period Count Function Finish Flag */

/* General PWM Timer Buffer Enable Register (32-bits) ***********************/

#define R_GPT_GTBER_BD_SHIFT (0)
#define R_GPT_GTBER_BD_MASK (0x7)
#  define R_GPT_GTBER_BD_ENABLE_BUFFER_OPERATION (0 << R_GPT_GTBER_BD_SHIFT)   /* Enable buffer operation */
#  define R_GPT_GTBER_BD_DISABLE_BUFFER_OPERATION (1 << R_GPT_GTBER_BD_SHIFT)  /* Disable buffer operation */
#define R_GPT_GTBER_CCRA_SHIFT (16)
#define R_GPT_GTBER_CCRA_MASK (0x3)
#  define R_GPT_GTBER_CCRA_V00 (0 << R_GPT_GTBER_CCRA_SHIFT)                   /* Buffer operation is not performed */
#  define R_GPT_GTBER_CCRA_V01 (1 << R_GPT_GTBER_CCRA_SHIFT)                   /* Single buffer operation (GTCCRA register to GTCCRC register) */
#define R_GPT_GTBER_CCRB_SHIFT (18)
#define R_GPT_GTBER_CCRB_MASK (0x3)
#  define R_GPT_GTBER_CCRB_V00 (0 << R_GPT_GTBER_CCRB_SHIFT)                   /* Buffer operation is not performed */
#  define R_GPT_GTBER_CCRB_V01 (1 << R_GPT_GTBER_CCRB_SHIFT)                   /* Single buffer operation (GTCCRB register to GTCCRE register) */
#define R_GPT_GTBER_PR_SHIFT (20)
#define R_GPT_GTBER_PR_MASK (0x3)
#  define R_GPT_GTBER_PR_V00 (0 << R_GPT_GTBER_PR_SHIFT)                       /* Buffer operation is not performed */
#  define R_GPT_GTBER_PR_V01 (1 << R_GPT_GTBER_PR_SHIFT)                       /* Buffer operation (GTPBR register to GTPR register) */
#define R_GPT_GTBER_CCRSWT (1 << 22)                                           /* 400000: GTCCRA and GTCCRB Forcible Buffer OperationThis bit is read as 0. */
#define R_GPT_GTBER_ADTTA_SHIFT (24)
#define R_GPT_GTBER_ADTTA_MASK (0x3)
#  define R_GPT_GTBER_ADTTA_NO_TRANSFER (0 << R_GPT_GTBER_ADTTA_SHIFT)         /* No transfer */
#  define R_GPT_GTBER_ADTTA_TRANSFER_AT_CREST (1 << R_GPT_GTBER_ADTTA_SHIFT)   /* Transfer at crest */
#  define R_GPT_GTBER_ADTTA_TRANSFER_AT_TROUGH (2 << R_GPT_GTBER_ADTTA_SHIFT)  /* Transfer at trough */
#  define R_GPT_GTBER_ADTTA_V11 (3 << R_GPT_GTBER_ADTTA_SHIFT)                 /* Transfer at both crest and trough */
#define R_GPT_GTBER_ADTDA (1 << 26)                                            /* 4000000: GTADTRA Double Buffer Operation */
#define R_GPT_GTBER_ADTTB_SHIFT (28)
#define R_GPT_GTBER_ADTTB_MASK (0x3)
#  define R_GPT_GTBER_ADTTB_NO_TRANSFER (0 << R_GPT_GTBER_ADTTB_SHIFT)         /* No transfer */
#  define R_GPT_GTBER_ADTTB_TRANSFER_AT_CREST (1 << R_GPT_GTBER_ADTTB_SHIFT)   /* Transfer at crest */
#  define R_GPT_GTBER_ADTTB_TRANSFER_AT_TROUGH (2 << R_GPT_GTBER_ADTTB_SHIFT)  /* Transfer at trough */
#  define R_GPT_GTBER_ADTTB_V11 (3 << R_GPT_GTBER_ADTTB_SHIFT)                 /* Transfer at both crest and trough */
#define R_GPT_GTBER_ADTDB (1 << 30)                                            /* 40000000: GTADTRB Double Buffer Operation */

/* General PWM Timer Counter (32-bits) **************************************/

#define R_GPT_GTCNT_GTCNT_SHIFT (0)
#define R_GPT_GTCNT_GTCNT_MASK (0xffffffff)

/* General PWM Timer Compare Capture Register A (32-bits) *******************/

#define R_GPT_GTCCRA_GTCCRA_SHIFT (0)
#define R_GPT_GTCCRA_GTCCRA_MASK (0xffffffff)

/* General PWM Timer Compare Capture Register B (32-bits) *******************/

#define R_GPT_GTCCRB_GTCCRB_SHIFT (0)
#define R_GPT_GTCCRB_GTCCRB_MASK (0xffffffff)

/* General PWM Timer Compare Capture Register C (32-bits) *******************/

#define R_GPT_GTCCRC_GTCCRC_SHIFT (0)
#define R_GPT_GTCCRC_GTCCRC_MASK (0xffffffff)

/* General PWM Timer Compare Capture Register E (32-bits) *******************/

#define R_GPT_GTCCRE_GTCCRE_SHIFT (0)
#define R_GPT_GTCCRE_GTCCRE_MASK (0xffffffff)

/* General PWM Timer Compare Capture Register D (32-bits) *******************/

#define R_GPT_GTCCRD_GTCCRD_SHIFT (0)
#define R_GPT_GTCCRD_GTCCRD_MASK (0xffffffff)

/* General PWM Timer Compare Capture Register F (32-bits) *******************/

#define R_GPT_GTCCRF_GTCCRF_SHIFT (0)
#define R_GPT_GTCCRF_GTCCRF_MASK (0xffffffff)

/* General PWM Timer Cycle Setting Register (32-bits) ***********************/

#define R_GPT_GTPR_GTPR_SHIFT (0)
#define R_GPT_GTPR_GTPR_MASK (0xffffffff)

/* General PWM Timer Cycle Setting Buffer Register (32-bits) ****************/

#define R_GPT_GTPBR_GTPBR_SHIFT (0)
#define R_GPT_GTPBR_GTPBR_MASK (0xffffffff)

/* A/D Converter Start Request Timing Register A (32-bits) ******************/

#define R_GPT_GTADTRA_GTADTRA_SHIFT (0)
#define R_GPT_GTADTRA_GTADTRA_MASK (0xffffffff)

/* A/D Converter Start Request Timing Buffer Register A (32-bits) ***********/

#define R_GPT_GTADTBRA_GTADTBRA_SHIFT (0)
#define R_GPT_GTADTBRA_GTADTBRA_MASK (0xffffffff)

/* A/D Converter Start Request Timing Double-Buffer Register A (32-bits) ****/

#define R_GPT_GTADTDBRA_GTADTDBRA_SHIFT (0)
#define R_GPT_GTADTDBRA_GTADTDBRA_MASK (0xffffffff)

/* A/D Converter Start Request Timing Register B (32-bits) ******************/

#define R_GPT_GTADTRB_GTADTRB_SHIFT (0)
#define R_GPT_GTADTRB_GTADTRB_MASK (0xffffffff)

/* A/D Converter Start Request Timing Buffer Register B (32-bits) ***********/

#define R_GPT_GTADTBRB_GTADTBRB_SHIFT (0)
#define R_GPT_GTADTBRB_GTADTBRB_MASK (0xffffffff)

/* A/D Converter Start Request Timing Double-Buffer Register B (32-bits) ****/

#define R_GPT_GTADTDBRB_GTADTDBRB_SHIFT (0)
#define R_GPT_GTADTDBRB_GTADTDBRB_MASK (0xffffffff)

/* General PWM Timer Dead Time Control Register (32-bits) *******************/

#define R_GPT_GTDTCR_TDE (1 <<  0)  /* 01: Negative-Phase Waveform Setting */

/* General PWM Timer Dead Time Value Register U (32-bits) *******************/

#define R_GPT_GTDVU_GTDVU_SHIFT (0)
#define R_GPT_GTDVU_GTDVU_MASK (0xffffffff)

/* General PWM Timer A/D Conversion Start Request Signal Monitoring Register
 * (32-bits)
 */

#define R_GPT_GTADSMR_ADSMS0_SHIFT (0)
#define R_GPT_GTADSMR_ADSMS0_MASK (0x3)
#  define R_GPT_GTADSMR_ADSMS0_V00 (0 << R_GPT_GTADSMR_ADSMS0_SHIFT)  /* A/D conversion start request signal generated by the GTADTRA register during up-counting */
#  define R_GPT_GTADSMR_ADSMS0_V01 (1 << R_GPT_GTADSMR_ADSMS0_SHIFT)  /* A/D conversion start request signal generated by the GTADTRA register during down-counting */
#  define R_GPT_GTADSMR_ADSMS0_V10 (2 << R_GPT_GTADSMR_ADSMS0_SHIFT)  /* A/D conversion start request signal generated by the GTADTRB register during up-counting */
#  define R_GPT_GTADSMR_ADSMS0_V11 (3 << R_GPT_GTADSMR_ADSMS0_SHIFT)  /* A/D conversion start request signal generated by the GTADTRB register during down-counting */
#define R_GPT_GTADSMR_ADSMEN0 (1 <<  8)                               /* 100: A/D Conversion Start Request Signal Monitor 0 Output Enabling */
#define R_GPT_GTADSMR_ADSMS1_SHIFT (16)
#define R_GPT_GTADSMR_ADSMS1_MASK (0x3)
#  define R_GPT_GTADSMR_ADSMS1_V00 (0 << R_GPT_GTADSMR_ADSMS1_SHIFT)  /* A/D conversion start request signal generated by the GTADTRA register during up-counting */
#  define R_GPT_GTADSMR_ADSMS1_V01 (1 << R_GPT_GTADSMR_ADSMS1_SHIFT)  /* A/D conversion start request signal generated by the GTADTRA register during down-counting */
#  define R_GPT_GTADSMR_ADSMS1_V10 (2 << R_GPT_GTADSMR_ADSMS1_SHIFT)  /* A/D conversion start request signal generated by the GTADTRB register during up-counting */
#  define R_GPT_GTADSMR_ADSMS1_V11 (3 << R_GPT_GTADSMR_ADSMS1_SHIFT)  /* A/D conversion start request signal generated by the GTADTRB register during down-counting */
#define R_GPT_GTADSMR_ADSMEN1 (1 << 24)                               /* 1000000: A/D Conversion Start Request Signal Monitor 1 Output Enabling */

/* General PWM Timer Inter Channel Logical Operation Function Setting
 * Register (32-bits)
 */

#define R_GPT_GTICLF_ICLFA_SHIFT (0)
#define R_GPT_GTICLF_ICLFA_MASK (0x7)
#  define R_GPT_GTICLF_ICLFA_A_NO_DELAY (0 << R_GPT_GTICLF_ICLFA_SHIFT)              /* A (no delay) */
#  define R_GPT_GTICLF_ICLFA_NOT_A_NO_DELAY (1 << R_GPT_GTICLF_ICLFA_SHIFT)          /* NOT A (no delay) */
#  define R_GPT_GTICLF_ICLFA_C_1PCLKGPTN_DELAY (2 << R_GPT_GTICLF_ICLFA_SHIFT)       /* C (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFA_NOT_C_1PCLKGPTN_DELAY (3 << R_GPT_GTICLF_ICLFA_SHIFT)   /* NOT C (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFA_A_AND_C_1PCLKGPTNDELAY (4 << R_GPT_GTICLF_ICLFA_SHIFT)  /* A AND C (1PCLKGPTndelay) */
#  define R_GPT_GTICLF_ICLFA_A_OR_C_1PCLKGPTNDELAY (5 << R_GPT_GTICLF_ICLFA_SHIFT)   /* A OR C (1PCLKGPTndelay) */
#  define R_GPT_GTICLF_ICLFA_V110 (6 << R_GPT_GTICLF_ICLFA_SHIFT)                    /* A EXOR C (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFA_A_NOR_C_1PCLKGPTNDELAY (7 << R_GPT_GTICLF_ICLFA_SHIFT)  /* A NOR C (1PCLKGPTndelay) */
#define R_GPT_GTICLF_ICLFSELC_SHIFT (4)
#define R_GPT_GTICLF_ICLFSELC_MASK (0x3f)
#  define R_GPT_GTICLF_ICLFSELC_GTIOC0A (0 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC0A */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC0B (1 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC0B */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC1A (2 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC1A */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC1B (3 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC1B */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC2A (4 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC2A */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC2B (5 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC2B */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC3A (6 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC3A */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC3B (7 << R_GPT_GTICLF_ICLFSELC_SHIFT)           /* GTIOC3B */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC31A (62 << R_GPT_GTICLF_ICLFSELC_SHIFT)         /* GTIOC31A */
#  define R_GPT_GTICLF_ICLFSELC_GTIOC31B (63 << R_GPT_GTICLF_ICLFSELC_SHIFT)         /* GTIOC31B */
#define R_GPT_GTICLF_ICLFB_SHIFT (16)
#define R_GPT_GTICLF_ICLFB_MASK (0x7)
#  define R_GPT_GTICLF_ICLFB_B_NO_DELAY (0 << R_GPT_GTICLF_ICLFB_SHIFT)              /* B (no delay) */
#  define R_GPT_GTICLF_ICLFB_NOT_B_NO_DELAY (1 << R_GPT_GTICLF_ICLFB_SHIFT)          /* NOT B (no delay) */
#  define R_GPT_GTICLF_ICLFB_D_1PCLKGPTN_DELAY (2 << R_GPT_GTICLF_ICLFB_SHIFT)       /* D (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFB_NOT_D_1PCLKGPTN_DELAY (3 << R_GPT_GTICLF_ICLFB_SHIFT)   /* NOT D (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFB_V100 (4 << R_GPT_GTICLF_ICLFB_SHIFT)                    /* B AND D (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFB_B_OR_D_1PCLKGPTN_DELAY (5 << R_GPT_GTICLF_ICLFB_SHIFT)  /* B OR D (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFB_V110 (6 << R_GPT_GTICLF_ICLFB_SHIFT)                    /* B EXOR D (1PCLKGPTn delay) */
#  define R_GPT_GTICLF_ICLFB_V111 (7 << R_GPT_GTICLF_ICLFB_SHIFT)                    /* B NOR D (1PCLKGPTn delay) */
#define R_GPT_GTICLF_ICLFSELD_SHIFT (20)
#define R_GPT_GTICLF_ICLFSELD_MASK (0x3f)
#  define R_GPT_GTICLF_ICLFSELD_GTIOC0A (0 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC0A */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC0B (1 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC0B */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC1A (2 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC1A */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC1B (3 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC1B */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC2A (4 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC2A */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC2B (5 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC2B */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC3A (6 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC3A */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC3B (7 << R_GPT_GTICLF_ICLFSELD_SHIFT)           /* GTIOC3B */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC31A (62 << R_GPT_GTICLF_ICLFSELD_SHIFT)         /* GTIOC31A */
#  define R_GPT_GTICLF_ICLFSELD_GTIOC31B (63 << R_GPT_GTICLF_ICLFSELD_SHIFT)         /* GTIOC31B */

/* General PWM Timer Period Count Register (32-bits) ************************/

#define R_GPT_GTPC_PCEN (1 <<  0)  /* 01: Period Count Function Enable */
#define R_GPT_GTPC_ASTP (1 <<  8)  /* 100: Automatic Stop Function Enable */
#define R_GPT_GTPC_PCNT_SHIFT (16)
#define R_GPT_GTPC_PCNT_MASK (0xfff)

/* General PWM Timer Operation Enable Bit Simultaneous Control Channel Select
 * Register (32-bits)
 */

#define R_GPT_GTSECSR_SECSEL0 (1 <<  0)   /* 01: Channel 0 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL1 (1 <<  1)   /* 02: Channel 1 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL2 (1 <<  2)   /* 04: Channel 2 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL3 (1 <<  3)   /* 08: Channel 3 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL4 (1 <<  4)   /* 10: Channel 4 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL5 (1 <<  5)   /* 20: Channel 5 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL6 (1 <<  6)   /* 40: Channel 6 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL7 (1 <<  7)   /* 80: Channel 7 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL8 (1 <<  8)   /* 100: Channel 8 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL9 (1 <<  9)   /* 200: Channel 9 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL10 (1 << 10)  /* 400: Channel 10 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL11 (1 << 11)  /* 800: Channel 11 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL12 (1 << 12)  /* 1000: Channel 12 Operation Enable BitSimultaneous Control Channel Select */
#define R_GPT_GTSECSR_SECSEL13 (1 << 13)  /* 2000: Channel 13 Operation Enable BitSimultaneous Control Channel Select */

/* General PWM Timer Operation Enable Bit Simultaneous Control Register
 * (32-bits)
 */

#define R_GPT_GTSECR_SBDCE (1 <<  0)  /* 01: GTCCR Register Buffer Operation Simultaneous Enable */
#define R_GPT_GTSECR_SBDPE (1 <<  1)  /* 02: GTPR Register Buffer Operation Simultaneous Enable */
#define R_GPT_GTSECR_SBDAE (1 <<  2)  /* 04: GTADTR Register Buffer Operation Simultaneous Enable */
#define R_GPT_GTSECR_SBDCD (1 <<  8)  /* 100: GTCCR Register Buffer Operation Simultaneous Disable */
#define R_GPT_GTSECR_SBDPD (1 <<  9)  /* 200: GTPR Register Buffer Operation Simultaneous Disable */
#define R_GPT_GTSECR_SBDAD (1 << 10)  /* 400: GTADTR Register Buffer Operation Simultaneous Disable */
#define R_GPT_GTSECR_SPCE (1 << 16)   /* 10000: Period Count Function Simultaneous Enable */
#define R_GPT_GTSECR_SPCD (1 << 24)   /* 1000000: Period Count Function Simultaneous Disable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_GPT_H */
