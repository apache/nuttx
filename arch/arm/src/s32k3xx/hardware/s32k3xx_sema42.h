/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_sema42.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SEMA42_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SEMA42_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SEMA42 Register Offsets **************************************************/

#define S32K3XX_SEMA42_GATE3_OFFSET  (0x00) /* Gate Register 3 (GATE3) */
#define S32K3XX_SEMA42_GATE2_OFFSET  (0x01) /* Gate Register 2 (GATE2) */
#define S32K3XX_SEMA42_GATE1_OFFSET  (0x02) /* Gate Register 1 (GATE1) */
#define S32K3XX_SEMA42_GATE0_OFFSET  (0x03) /* Gate Register 0 (GATE0) */
#define S32K3XX_SEMA42_GATE7_OFFSET  (0x04) /* Gate Register 7 (GATE7) */
#define S32K3XX_SEMA42_GATE6_OFFSET  (0x05) /* Gate Register 6 (GATE6) */
#define S32K3XX_SEMA42_GATE5_OFFSET  (0x06) /* Gate Register 5 (GATE5) */
#define S32K3XX_SEMA42_GATE4_OFFSET  (0x07) /* Gate Register 4 (GATE4) */
#define S32K3XX_SEMA42_GATE11_OFFSET (0x08) /* Gate Register 11 (GATE11) */
#define S32K3XX_SEMA42_GATE10_OFFSET (0x09) /* Gate Register 10 (GATE19) */
#define S32K3XX_SEMA42_GATE9_OFFSET  (0x0a) /* Gate Register 9 (GATE9) */
#define S32K3XX_SEMA42_GATE8_OFFSET  (0x0b) /* Gate Register 8 (GATE8) */
#define S32K3XX_SEMA42_GATE15_OFFSET (0x0c) /* Gate Register 15 (GATE15) */
#define S32K3XX_SEMA42_GATE14_OFFSET (0x0d) /* Gate Register 14 (GATE14) */
#define S32K3XX_SEMA42_GATE13_OFFSET (0x0e) /* Gate Register 13 (GATE13) */
#define S32K3XX_SEMA42_GATE12_OFFSET (0x0f) /* Gate Register 12 (GATE12) */
#define S32K3XX_SEMA42_RSTGT_OFFSET  (0x42) /* Reset Gate Register (RSTGT) */

/* SEMA42 Register Addresses ************************************************/

#define S32K3XX_SEMA42_GATE3         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE3)
#define S32K3XX_SEMA42_GATE2         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE2)
#define S32K3XX_SEMA42_GATE1         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE1)
#define S32K3XX_SEMA42_GATE0         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE0)
#define S32K3XX_SEMA42_GATE7         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE7)
#define S32K3XX_SEMA42_GATE6         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE6)
#define S32K3XX_SEMA42_GATE5         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE5)
#define S32K3XX_SEMA42_GATE4         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE4)
#define S32K3XX_SEMA42_GATE11        (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE11)
#define S32K3XX_SEMA42_GATE10        (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE10)
#define S32K3XX_SEMA42_GATE9         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE9)
#define S32K3XX_SEMA42_GATE8         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE8)
#define S32K3XX_SEMA42_GATE15        (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE15)
#define S32K3XX_SEMA42_GATE14        (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE14)
#define S32K3XX_SEMA42_GATE13        (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE13)
#define S32K3XX_SEMA42_GATE12        (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_GATE12)
#define S32K3XX_SEMA42_RSTGT         (S32K3XX_SEMA42_BASE + S32K3XX_SEMA42_RSTGT)

/* SEMA42 Register Bitfield Definitions *************************************/

/* Gate Register n (GATEn) */

#define SEMA42_GATE_GTFSM_SHIFT      (0)  /* Bits 0-3: Gate finite state machine (GTFSM) */
#define SEMA42_GATE_GTFSM_MASK       (0x0f << SEMA42_GATE_GTFSM_SHIFT)
#define SEMA42_GATE_GTFSM_FREE       (0x00 << SEMA42_GATE_GTFSM_SHIFT) /* Gate is unlocked (free) */
#define SEMA42_GATE_GTFSM_DOM0       (0x01 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 0 locked the gate */
#define SEMA42_GATE_GTFSM_DOM1       (0x02 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 1 locked the gate */
#define SEMA42_GATE_GTFSM_DOM2       (0x03 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 2 locked the gate */
#define SEMA42_GATE_GTFSM_DOM3       (0x04 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 3 locked the gate */
#define SEMA42_GATE_GTFSM_DOM4       (0x05 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 4 locked the gate */
#define SEMA42_GATE_GTFSM_DOM5       (0x06 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 5 locked the gate */
#define SEMA42_GATE_GTFSM_DOM6       (0x07 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 6 locked the gate */
#define SEMA42_GATE_GTFSM_DOM7       (0x08 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 7 locked the gate */
#define SEMA42_GATE_GTFSM_DOM8       (0x09 << SEMA42_GATE_GTFSM_SHIFT) /* Domain 8 locked the gate */
#define SEMA42_GATE_GTFSM_DOM9       (0x0a << SEMA42_GATE_GTFSM_SHIFT) /* Domain 9 locked the gate */
#define SEMA42_GATE_GTFSM_DOM10      (0x0b << SEMA42_GATE_GTFSM_SHIFT) /* Domain 10 locked the gate */
#define SEMA42_GATE_GTFSM_DOM11      (0x0c << SEMA42_GATE_GTFSM_SHIFT) /* Domain 11 locked the gate */
#define SEMA42_GATE_GTFSM_DOM12      (0x0d << SEMA42_GATE_GTFSM_SHIFT) /* Domain 12 locked the gate */
#define SEMA42_GATE_GTFSM_DOM13      (0x0e << SEMA42_GATE_GTFSM_SHIFT) /* Domain 13 locked the gate */
#define SEMA42_GATE_GTFSM_DOM14      (0x0f << SEMA42_GATE_GTFSM_SHIFT) /* Domain 14 locked the gate */

                                          /* Bits 4-7: Reserved */

/* Reset Gate Register (RSTGT) */

#define SEMA42_RSTGT_RSTGTN_SHIFT    (0)  /* Bits 0-7: Reset Gate Number (RSTGTN) */
#define SEMA42_RSTGT_RSTGTN_MASK     (0xff << SEMA42_RSTGT_RSTGTN_SHIFT)
#define SEMA42_RSTGT_RSTGDP_SHIFT    (8)  /* Bits 8-15: Reset Gate Data Pattern (RSTGDP) */
#define SEMA42_RSTGT_RSTGDP_MASK     (0xff << SEMA42_RSTGT_RSTGDP_SHIFT)
#define SEMA42_RSTGT_RSTGMS_SHIFT    (8)  /* Bits 8-11: Reset Gate Domain (RSTGMS) */
#define SEMA42_RSTGT_RSTGMS_MASK     (0x0f << SEMA42_RSTGT_RSTGMS_SHIFT)
#define SEMA42_RSTGT_RSTGSM_SHIFT    (12) /* Bits 12-13: Reset Gate Finite State Machine (RSTGSM) */
#define SEMA42_RSTGT_RSTGSM_MASK     (0x03 << SEMA42_RSTGT_RSTGSM_SHIFT)
#define SEMA42_RSTGT_RSTGSM_FIRST    (0x03 << SEMA42_RSTGT_RSTGSM_SHIFT) /* Idle, waiting for the first data pattern write */
#define SEMA42_RSTGT_RSTGSM_SECOND   (0x03 << SEMA42_RSTGT_RSTGSM_SHIFT) /* Waiting for the second data pattern write */
#define SEMA42_RSTGT_RSTGSM_COMPLETE (0x03 << SEMA42_RSTGT_RSTGSM_SHIFT) /* The 2-write sequence has completed */

#define SEMA42_RSTGT_ROZ_SHIFT       (14) /* Bits 14-15: ROZ */
#define SEMA42_RSTGT_ROZ_MASK        (0x03 << SEMA42_RSTGT_ROZ_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SEMA42_H */
