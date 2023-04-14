/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_flexio.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXIO_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_FLEXIO_VERID_OFFSET         0x0000 /* Version ID Register, offset: 0x0 */
#define IMXRT_FLEXIO_PARAM_OFFSET         0x0004 /* Parameter Register, offset: 0x4 */
#define IMXRT_FLEXIO_CTRL_OFFSET          0x0008 /* FlexIO Control Register, offset: 0x8 */
#define IMXRT_FLEXIO_PIN_OFFSET           0x000c /* Pin State Register, offset: 0xC */
#define IMXRT_FLEXIO_SHIFTSTAT_OFFSET     0x0010 /* Shifter Status Register, offset: 0x10 */
#define IMXRT_FLEXIO_SHIFTERR_OFFSET      0x0014 /* Shifter Error Register, offset: 0x14 */
#define IMXRT_FLEXIO_TIMSTAT_OFFSET       0x0018 /* Timer Status Register, offset: 0x18 */
#define IMXRT_FLEXIO_SHIFTSIEN_OFFSET     0x0020 /* Shift Enable, offset: 0x20 */ 
#define IMXRT_FLEXIO_SHIFTEIEN_OFFSET     0x0024 /* Shifter Error Interrupt Enable, offset: 0x24 */
#define IMXRT_FLEXIO_TIMIEN_OFFSET        0x0028 /* Timer Interrupt Enable Register, offset: 0x28 */
#define IMXRT_FLEXIO_SHIFTSDEN_OFFSET     0x0030 /* Shifter Status DMA Enable, offset: 0x30 */
#define IMXRT_FLEXIO_SHIFTSTATE_OFFSET    0x0040 /* Shifter State Register, offset: 0x40 */
#define IMXRT_FLEXIO_SHIFTCTL0_OFFSET     0x0080 /* Shifter Control N Register, array offset: 0x80, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTCTL1_OFFSET     0x0084
#define IMXRT_FLEXIO_SHIFTCTL2_OFFSET     0x0088
#define IMXRT_FLEXIO_SHIFTCTL3_OFFSET     0x008c
#define IMXRT_FLEXIO_SHIFTCTL4_OFFSET     0x0090
#define IMXRT_FLEXIO_SHIFTCTL5_OFFSET     0x0094
#define IMXRT_FLEXIO_SHIFTCTL6_OFFSET     0x0098
#define IMXRT_FLEXIO_SHIFTCTL7_OFFSET     0x009c
#define IMXRT_FLEXIO_SHIFTCFG0_OFFSET     0x0100 /* Shifter Configuration N Register, array offset: 0x100, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTCFG1_OFFSET     0x0104
#define IMXRT_FLEXIO_SHIFTCFG2_OFFSET     0x0108
#define IMXRT_FLEXIO_SHIFTCFG3_OFFSET     0x010c
#define IMXRT_FLEXIO_SHIFTCFG4_OFFSET     0x0110
#define IMXRT_FLEXIO_SHIFTCFG5_OFFSET     0x0114
#define IMXRT_FLEXIO_SHIFTCFG6_OFFSET     0x0118
#define IMXRT_FLEXIO_SHIFTCFG7_OFFSET     0x011c
#define IMXRT_FLEXIO_SHIFTBUF0_OFFSET     0x0200 /* Shifter Buffer N Register, array offset: 0x200, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTBUF1_OFFSET     0x0204
#define IMXRT_FLEXIO_SHIFTBUF2_OFFSET     0x0208
#define IMXRT_FLEXIO_SHIFTBUF3_OFFSET     0x020c
#define IMXRT_FLEXIO_SHIFTBUF4_OFFSET     0x0210
#define IMXRT_FLEXIO_SHIFTBUF5_OFFSET     0x0214
#define IMXRT_FLEXIO_SHIFTBUF6_OFFSET     0x0218
#define IMXRT_FLEXIO_SHIFTBUF7_OFFSET     0x021c
#define IMXRT_FLEXIO_SHIFTBUFBIS0_OFFSET  0x0280 /* Shifter Buffer N Bit Swapped Register, array offset: 0x280, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTBUFBIS1_OFFSET  0x0284
#define IMXRT_FLEXIO_SHIFTBUFBIS2_OFFSET  0x0288
#define IMXRT_FLEXIO_SHIFTBUFBIS3_OFFSET  0x028c
#define IMXRT_FLEXIO_SHIFTBUFBIS4_OFFSET  0x0290
#define IMXRT_FLEXIO_SHIFTBUFBIS5_OFFSET  0x0294
#define IMXRT_FLEXIO_SHIFTBUFBIS6_OFFSET  0x0298
#define IMXRT_FLEXIO_SHIFTBUFBIS7_OFFSET  0x029c
#define IMXRT_FLEXIO_SHIFTBUFBYS0_OFFSET  0x0300 /* Shifter Buffer N Byte Swapped Register, array offset: 0x300, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTBUFBYS1_OFFSET  0x0304
#define IMXRT_FLEXIO_SHIFTBUFBYS2_OFFSET  0x0308
#define IMXRT_FLEXIO_SHIFTBUFBYS3_OFFSET  0x030c
#define IMXRT_FLEXIO_SHIFTBUFBYS4_OFFSET  0x0310
#define IMXRT_FLEXIO_SHIFTBUFBYS5_OFFSET  0x0314
#define IMXRT_FLEXIO_SHIFTBUFBYS6_OFFSET  0x0318
#define IMXRT_FLEXIO_SHIFTBUFBYS7_OFFSET  0x031c
#define IMXRT_FLEXIO_SHIFTBUFBBS0_OFFSET  0x0380 /* Shifter Buffer N Bit Byte Swapped Register, array offset: 0x380, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTBUFBBS1_OFFSET  0x0384
#define IMXRT_FLEXIO_SHIFTBUFBBS2_OFFSET  0x0388
#define IMXRT_FLEXIO_SHIFTBUFBBS3_OFFSET  0x038c
#define IMXRT_FLEXIO_SHIFTBUFBBS4_OFFSET  0x0390
#define IMXRT_FLEXIO_SHIFTBUFBBS5_OFFSET  0x0394
#define IMXRT_FLEXIO_SHIFTBUFBBS6_OFFSET  0x0398
#define IMXRT_FLEXIO_SHIFTBUFBBS7_OFFSET  0x039c
#define IMXRT_FLEXIO_TIMCTL0_OFFSET       0x0400 /* Timer Control N Register, array offset: 0x400, array step: 0x4 */
#define IMXRT_FLEXIO_TIMCTL1_OFFSET       0x0404
#define IMXRT_FLEXIO_TIMCTL2_OFFSET       0x0408
#define IMXRT_FLEXIO_TIMCTL3_OFFSET       0x040c
#define IMXRT_FLEXIO_TIMCTL4_OFFSET       0x0410
#define IMXRT_FLEXIO_TIMCTL5_OFFSET       0x0414
#define IMXRT_FLEXIO_TIMCTL6_OFFSET       0x0418
#define IMXRT_FLEXIO_TIMCTL7_OFFSET       0x041c
#define IMXRT_FLEXIO_TIMCFG0_OFFSET       0x0480 /* Timer Configuration N Register, array offset: 0x480, array step: 0x4 */
#define IMXRT_FLEXIO_TIMCFG1_OFFSET       0x0484
#define IMXRT_FLEXIO_TIMCFG2_OFFSET       0x0488
#define IMXRT_FLEXIO_TIMCFG3_OFFSET       0x048c
#define IMXRT_FLEXIO_TIMCFG4_OFFSET       0x0490
#define IMXRT_FLEXIO_TIMCFG5_OFFSET       0x0494
#define IMXRT_FLEXIO_TIMCFG6_OFFSET       0x0498
#define IMXRT_FLEXIO_TIMCFG7_OFFSET       0x049c
#define IMXRT_FLEXIO_TIMCMP0_OFFSET       0x0500 /* Timer Compare N Register, array offset: 0x500, array step: 0x4 */
#define IMXRT_FLEXIO_TIMCMP1_OFFSET       0x0504
#define IMXRT_FLEXIO_TIMCMP2_OFFSET       0x0508
#define IMXRT_FLEXIO_TIMCMP3_OFFSET       0x050c
#define IMXRT_FLEXIO_TIMCMP4_OFFSET       0x0510
#define IMXRT_FLEXIO_TIMCMP5_OFFSET       0x0514
#define IMXRT_FLEXIO_TIMCMP6_OFFSET       0x0518
#define IMXRT_FLEXIO_TIMCMP7_OFFSET       0x051c
#define IMXRT_FLEXIO_SHIFTBUFNBS0_OFFSET  0x0680 /* Shifter Buffer N Nibble Byte Swapped Register, array offset: 0x680, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTBUFNBS1_OFFSET  0x0684
#define IMXRT_FLEXIO_SHIFTBUFNBS2_OFFSET  0x0688
#define IMXRT_FLEXIO_SHIFTBUFNBS3_OFFSET  0x068c
#define IMXRT_FLEXIO_SHIFTBUFNBS4_OFFSET  0x0690
#define IMXRT_FLEXIO_SHIFTBUFNBS5_OFFSET  0x0694
#define IMXRT_FLEXIO_SHIFTBUFNBS6_OFFSET  0x0698
#define IMXRT_FLEXIO_SHIFTBUFNBS7_OFFSET  0x069c
#define IMXRT_FLEXIO_SHIFTBUFHWS0_OFFSET  0x0700 /* Shifter Buffer N Half Word Swapped Register, array offset: 0x700, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTBUFHWS1_OFFSET  0x0704
#define IMXRT_FLEXIO_SHIFTBUFHWS2_OFFSET  0x0708
#define IMXRT_FLEXIO_SHIFTBUFHWS3_OFFSET  0x070c
#define IMXRT_FLEXIO_SHIFTBUFHWS4_OFFSET  0x0710
#define IMXRT_FLEXIO_SHIFTBUFHWS5_OFFSET  0x0714
#define IMXRT_FLEXIO_SHIFTBUFHWS6_OFFSET  0x0718
#define IMXRT_FLEXIO_SHIFTBUFHWS7_OFFSET  0x071c
#define IMXRT_FLEXIO_SHIFTBUFNIS0_OFFSET  0x0780 /* Shifter Buffer N Nibble Swapped Register, array offset: 0x780, array step: 0x4 */
#define IMXRT_FLEXIO_SHIFTBUFNIS1_OFFSET  0x0784
#define IMXRT_FLEXIO_SHIFTBUFNIS2_OFFSET  0x0788
#define IMXRT_FLEXIO_SHIFTBUFNIS3_OFFSET  0x078c
#define IMXRT_FLEXIO_SHIFTBUFNIS4_OFFSET  0x0790
#define IMXRT_FLEXIO_SHIFTBUFNIS5_OFFSET  0x0794
#define IMXRT_FLEXIO_SHIFTBUFNIS6_OFFSET  0x0798
#define IMXRT_FLEXIO_SHIFTBUFNIS7_OFFSET  0x079c

/* VERID - Version ID Register */

#define FLEXIO_VERID_FEATURE_MASK                (0xffffu)
#define FLEXIO_VERID_FEATURE_SHIFT               (0u)

/* FEATURE - Feature Specification Number
 *  0b0000000000000000..Standard features implemented.
 *  0b0000000000000001..Supports state, logic and parallel modes.
 */

#define FLEXIO_VERID_FEATURE(x)                  ((((uint32_t)(x)) << FLEXIO_VERID_FEATURE_SHIFT) & FLEXIO_VERID_FEATURE_MASK)

#define FLEXIO_VERID_MINOR_MASK                  (0xff0000u)
#define FLEXIO_VERID_MINOR_SHIFT                 (16u)

/* MINOR - Minor Version Number */

#define FLEXIO_VERID_MINOR(x)                    ((((uint32_t)(x)) << FLEXIO_VERID_MINOR_SHIFT) & FLEXIO_VERID_MINOR_MASK)

#define FLEXIO_VERID_MAJOR_MASK                  (0xff000000u)
#define FLEXIO_VERID_MAJOR_SHIFT                 (24u)

/* MAJOR - Major Version Number */
#define FLEXIO_VERID_MAJOR(x)                    ((((uint32_t)(x)) << FLEXIO_VERID_MAJOR_SHIFT) & FLEXIO_VERID_MAJOR_MASK)

/* PARAM - Parameter Register */

#define FLEXIO_PARAM_SHIFTER_MASK                (0xffu)
#define FLEXIO_PARAM_SHIFTER_SHIFT               (0u)

/* SHIFTER - Shifter Number */
#define FLEXIO_PARAM_SHIFTER(x)                  ((((uint32_t)(x)) << FLEXIO_PARAM_SHIFTER_SHIFT) & FLEXIO_PARAM_SHIFTER_MASK)

#define FLEXIO_PARAM_TIMER_MASK                  (0xff00u)
#define FLEXIO_PARAM_TIMER_SHIFT                 (8u)

/* TIMER - Timer Number */

#define FLEXIO_PARAM_TIMER(x)                    ((((uint32_t)(x)) << FLEXIO_PARAM_TIMER_SHIFT) & FLEXIO_PARAM_TIMER_MASK)

#define FLEXIO_PARAM_PIN_MASK                    (0xff0000u)
#define FLEXIO_PARAM_PIN_SHIFT                   (16u)

/* PIN - Pin Number */
#define FLEXIO_PARAM_PIN(x)                      ((((uint32_t)(x)) << FLEXIO_PARAM_PIN_SHIFT) & FLEXIO_PARAM_PIN_MASK)

#define FLEXIO_PARAM_TRIGGER_MASK                (0xff000000u)
#define FLEXIO_PARAM_TRIGGER_SHIFT               (24u)

/* TRIGGER - Trigger Number */

#define FLEXIO_PARAM_TRIGGER(x)                  ((((uint32_t)(x)) << FLEXIO_PARAM_TRIGGER_SHIFT) & FLEXIO_PARAM_TRIGGER_MASK)

/* CTRL - FlexIO Control Register */

#define FLEXIO_CTRL_FLEXEN_MASK                  (0x1u)
#define FLEXIO_CTRL_FLEXEN_SHIFT                 (0u)

/* FLEXEN - FlexIO Enable
 *  0b0..FlexIO module is disabled.
 *  0b1..FlexIO module is enabled.
 */

#define FLEXIO_CTRL_FLEXEN(x)                    ((((uint32_t)(x)) << FLEXIO_CTRL_FLEXEN_SHIFT) & FLEXIO_CTRL_FLEXEN_MASK)

#define FLEXIO_CTRL_SWRST_MASK                   (0x2u)
#define FLEXIO_CTRL_SWRST_SHIFT                  (1u)

/* SWRST - Software Reset
 *  0b0..Software reset is disabled
 *  0b1..Software reset is enabled, all FlexIO registers except the Control
 *       Register are reset.
 */

#define FLEXIO_CTRL_SWRST(x)                     ((((uint32_t)(x)) << FLEXIO_CTRL_SWRST_SHIFT) & FLEXIO_CTRL_SWRST_MASK)

#define FLEXIO_CTRL_FASTACC_MASK                 (0x4u)
#define FLEXIO_CTRL_FASTACC_SHIFT                (2u)

/* FASTACC - Fast Access
 *  0b0..Configures for normal register accesses to FlexIO
 *  0b1..Configures for fast register accesses to FlexIO
 */

#define FLEXIO_CTRL_FASTACC(x)                   ((((uint32_t)(x)) << FLEXIO_CTRL_FASTACC_SHIFT) & FLEXIO_CTRL_FASTACC_MASK)

#define FLEXIO_CTRL_DBGE_MASK                    (0x40000000u)
#define FLEXIO_CTRL_DBGE_SHIFT                   (30u)

/* DBGE - Debug Enable
 *  0b0..FlexIO is disabled in debug modes.
 *  0b1..FlexIO is enabled in debug modes
 */

#define FLEXIO_CTRL_DBGE(x)                      ((((uint32_t)(x)) << FLEXIO_CTRL_DBGE_SHIFT) & FLEXIO_CTRL_DBGE_MASK)

#define FLEXIO_CTRL_DOZEN_MASK                   (0x80000000u)
#define FLEXIO_CTRL_DOZEN_SHIFT                  (31u)

/* DOZEN - Doze Enable
 *  0b0..FlexIO enabled in Doze modes.
 *  0b1..FlexIO disabled in Doze modes.
 */

#define FLEXIO_CTRL_DOZEN(x)                     ((((uint32_t)(x)) << FLEXIO_CTRL_DOZEN_SHIFT) & FLEXIO_CTRL_DOZEN_MASK)

/* PIN - Pin State Register */

#define FLEXIO_PIN_PDI_MASK                      (0xffffffffu)  /* Merged from fields with different position or width, of widths (16, 32), largest definition used */
#define FLEXIO_PIN_PDI_SHIFT                     (0u)

/* PDI - Pin Data Input */

#define FLEXIO_PIN_PDI(x)                        ((((uint32_t)(x)) << FLEXIO_PIN_PDI_SHIFT) & FLEXIO_PIN_PDI_MASK)  /* Merged from fields with different position or width, of widths (16, 32), largest definition used */

/* SHIFTSTAT - Shifter Status Register */

#define FLEXIO_SHIFTSTAT_SSF_MASK                (0xffu)
#define FLEXIO_SHIFTSTAT_SSF_SHIFT               (0u)

/* SSF - Shifter Status Flag */

#define FLEXIO_SHIFTSTAT_SSF(x)                  ((((uint32_t)(x)) << FLEXIO_SHIFTSTAT_SSF_SHIFT) & FLEXIO_SHIFTSTAT_SSF_MASK)

/* SHIFTERR - Shifter Error Register */

#define FLEXIO_SHIFTERR_SEF_MASK                 (0xffu)
#define FLEXIO_SHIFTERR_SEF_SHIFT                (0u)

/* SEF - Shifter Error Flags */

#define FLEXIO_SHIFTERR_SEF(x)                   ((((uint32_t)(x)) << FLEXIO_SHIFTERR_SEF_SHIFT) & FLEXIO_SHIFTERR_SEF_MASK)

/* TIMSTAT - Timer Status Register */

#define FLEXIO_TIMSTAT_TSF_MASK                  (0xffu)
#define FLEXIO_TIMSTAT_TSF_SHIFT                 (0u)

/* TSF - Timer Status Flags */

#define FLEXIO_TIMSTAT_TSF(x)                    ((((uint32_t)(x)) << FLEXIO_TIMSTAT_TSF_SHIFT) & FLEXIO_TIMSTAT_TSF_MASK)

/* SHIFTSIEN - Shifter Status Interrupt Enable */

#define FLEXIO_SHIFTSIEN_SSIE_MASK               (0xffu)
#define FLEXIO_SHIFTSIEN_SSIE_SHIFT              (0u)

/* SSIE - Shifter Status Interrupt Enable */

#define FLEXIO_SHIFTSIEN_SSIE(x)                 ((((uint32_t)(x)) << FLEXIO_SHIFTSIEN_SSIE_SHIFT) & FLEXIO_SHIFTSIEN_SSIE_MASK)

/* SHIFTEIEN - Shifter Error Interrupt Enable */

#define FLEXIO_SHIFTEIEN_SEIE_MASK               (0xffu)
#define FLEXIO_SHIFTEIEN_SEIE_SHIFT              (0u)

/* SEIE - Shifter Error Interrupt Enable */

#define FLEXIO_SHIFTEIEN_SEIE(x)                 ((((uint32_t)(x)) << FLEXIO_SHIFTEIEN_SEIE_SHIFT) & FLEXIO_SHIFTEIEN_SEIE_MASK)

/* TIMIEN - Timer Interrupt Enable Register */

#define FLEXIO_TIMIEN_TEIE_MASK                  (0xffu)
#define FLEXIO_TIMIEN_TEIE_SHIFT                 (0u)

/* TEIE - Timer Status Interrupt Enable */

#define FLEXIO_TIMIEN_TEIE(x)                    ((((uint32_t)(x)) << FLEXIO_TIMIEN_TEIE_SHIFT) & FLEXIO_TIMIEN_TEIE_MASK)

/* SHIFTSDEN - Shifter Status DMA Enable */

#define FLEXIO_SHIFTSDEN_SSDE_MASK               (0xffu)
#define FLEXIO_SHIFTSDEN_SSDE_SHIFT              (0u)

/* SSDE - Shifter Status DMA Enable */

#define FLEXIO_SHIFTSDEN_SSDE(x)                 ((((uint32_t)(x)) << FLEXIO_SHIFTSDEN_SSDE_SHIFT) & FLEXIO_SHIFTSDEN_SSDE_MASK)

/* SHIFTSTATE - Shifter State Register */

#define FLEXIO_SHIFTSTATE_STATE_MASK             (0x7u)
#define FLEXIO_SHIFTSTATE_STATE_SHIFT            (0u)

/* STATE - Current State Pointer */

#define FLEXIO_SHIFTSTATE_STATE(x)               ((((uint32_t)(x)) << FLEXIO_SHIFTSTATE_STATE_SHIFT) & FLEXIO_SHIFTSTATE_STATE_MASK)

/* SHIFTCTL - Shifter Control N Register */

#define FLEXIO_SHIFTCTL_SMOD_MASK                (0x7u)
#define FLEXIO_SHIFTCTL_SMOD_SHIFT               (0u)

/* SMOD - Shifter Mode
 *  0b000..Disabled.
 *  0b001..Receive mode. Captures the current Shifter content into the
 *         SHIFTBUF on expiration of the Timer.
 *  0b010..Transmit mode. Load SHIFTBUF contents into the Shifter on
 *         expiration of the Timer.
 *  0b011..Reserved.
 *  0b100..Match Store mode. Shifter data is compared to SHIFTBUF content on
 *         expiration of the Timer.
 *  0b101..Match Continuous mode. Shifter data is continuously compared to
 *         SHIFTBUF contents.
 *  0b110..State mode. SHIFTBUF contents are used for storing programmable
 *         state attributes.
 *  0b111..Logic mode. SHIFTBUF contents are used for implementing
 *         programmable logic look up table.
 */

#define FLEXIO_SHIFTCTL_SMOD(x)                  ((((uint32_t)(x)) << FLEXIO_SHIFTCTL_SMOD_SHIFT) & FLEXIO_SHIFTCTL_SMOD_MASK)

#define FLEXIO_SHIFTCTL_PINPOL_MASK              (0x80u)
#define FLEXIO_SHIFTCTL_PINPOL_SHIFT             (7u)

/* PINPOL - Shifter Pin Polarity
 *  0b0..Pin is active high
 *  0b1..Pin is active low
 */

#define FLEXIO_SHIFTCTL_PINPOL(x)                ((((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINPOL_SHIFT) & FLEXIO_SHIFTCTL_PINPOL_MASK)

#define FLEXIO_SHIFTCTL_PINSEL_MASK              (0x1f00u)  /* Merged from fields with different position or width, of widths (4, 5), largest definition used */
#define FLEXIO_SHIFTCTL_PINSEL_SHIFT             (8u)

/* PINSEL - Shifter Pin Select */

#define FLEXIO_SHIFTCTL_PINSEL(x)                ((((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINSEL_SHIFT) & FLEXIO_SHIFTCTL_PINSEL_MASK)  /* Merged from fields with different position or width, of widths (4, 5), largest definition used */

#define FLEXIO_SHIFTCTL_PINCFG_MASK              (0x30000u)
#define FLEXIO_SHIFTCTL_PINCFG_SHIFT             (16u)

/* PINCFG - Shifter Pin Configuration
 *  0b00..Shifter pin output disabled
 *  0b01..Shifter pin open drain or bidirectional output enable
 *  0b10..Shifter pin bidirectional output data
 *  0b11..Shifter pin output
 */

#define FLEXIO_SHIFTCTL_PINCFG(x)                ((((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINCFG_SHIFT) & FLEXIO_SHIFTCTL_PINCFG_MASK)

#define FLEXIO_SHIFTCTL_TIMPOL_MASK              (0x800000u)
#define FLEXIO_SHIFTCTL_TIMPOL_SHIFT             (23u)

/* TIMPOL - Timer Polarity
 *  0b0..Shift on posedge of Shift clock
 *  0b1..Shift on negedge of Shift clock
 */

#define FLEXIO_SHIFTCTL_TIMPOL(x)                ((((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMPOL_SHIFT) & FLEXIO_SHIFTCTL_TIMPOL_MASK)

#define FLEXIO_SHIFTCTL_TIMSEL_MASK              (0x7000000u)
#define FLEXIO_SHIFTCTL_TIMSEL_SHIFT             (24u)

/* TIMSEL - Timer Select */

#define FLEXIO_SHIFTCTL_TIMSEL(x)                ((((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMSEL_SHIFT) & FLEXIO_SHIFTCTL_TIMSEL_MASK)

/* The count of FLEXIO_SHIFTCTL */

#define FLEXIO_SHIFTCTL_COUNT                    (8u)

/* SHIFTCFG - Shifter Configuration N Register */

#define FLEXIO_SHIFTCFG_SSTART_MASK              (0x3u)
#define FLEXIO_SHIFTCFG_SSTART_SHIFT             (0u)

/* SSTART - Shifter Start bit
 *  0b00..Start bit disabled for transmitter/receiver/match store,
 *        transmitter loads data on enable
 *  0b01..Start bit disabled for transmitter/receiver/match store,
 *        transmitter loads data on first shift
 *  0b10..Transmitter outputs start bit value 0 before loading data on first
 *        shift, receiver/match store sets error flag if start bit is not 0
 *  0b11..Transmitter outputs start bit value 1 before loading data on first
 *        shift, receiver/match store sets error flag if start bit is not 1
 */

#define FLEXIO_SHIFTCFG_SSTART(x)                ((((uint32_t)(x)) << FLEXIO_SHIFTCFG_SSTART_SHIFT) & FLEXIO_SHIFTCFG_SSTART_MASK)

#define FLEXIO_SHIFTCFG_SSTOP_MASK               (0x30u)
#define FLEXIO_SHIFTCFG_SSTOP_SHIFT              (4u)

/* SSTOP - Shifter Stop bit
 *  0b00..Stop bit disabled for transmitter/receiver/match store
 *  0b01..Reserved for transmitter/receiver/match store
 *  0b10..Transmitter outputs stop bit value 0 on store, receiver/match store
 *        sets error flag if stop bit is not 0
 *  0b11..Transmitter outputs stop bit value 1 on store, receiver/match store
 *        sets error flag if stop bit is not 1
 */

#define FLEXIO_SHIFTCFG_SSTOP(x)                 ((((uint32_t)(x)) << FLEXIO_SHIFTCFG_SSTOP_SHIFT) & FLEXIO_SHIFTCFG_SSTOP_MASK)

#define FLEXIO_SHIFTCFG_INSRC_MASK               (0x100u)
#define FLEXIO_SHIFTCFG_INSRC_SHIFT              (8u)

/* INSRC - Input Source
 *  0b0..Pin
 *  0b1..Shifter N+1 Output
 */

#define FLEXIO_SHIFTCFG_INSRC(x)                 ((((uint32_t)(x)) << FLEXIO_SHIFTCFG_INSRC_SHIFT) & FLEXIO_SHIFTCFG_INSRC_MASK)

#define FLEXIO_SHIFTCFG_PWIDTH_MASK              (0x1f0000u)  /* Merged from fields with different position or width, of widths (4, 5), largest definition used */
#define FLEXIO_SHIFTCFG_PWIDTH_SHIFT             (16u)

/* PWIDTH - Parallel Width */

#define FLEXIO_SHIFTCFG_PWIDTH(x)                ((((uint32_t)(x)) << FLEXIO_SHIFTCFG_PWIDTH_SHIFT) & FLEXIO_SHIFTCFG_PWIDTH_MASK)  /* Merged from fields with different position or width, of widths (4, 5), largest definition used */

/* The count of FLEXIO_SHIFTCFG */

#define FLEXIO_SHIFTCFG_COUNT                    (8u)

/* SHIFTBUF - Shifter Buffer N Register */

#define FLEXIO_SHIFTBUF_SHIFTBUF_MASK            (0xffffffffu)
#define FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT           (0u)

/* SHIFTBUF - Shift Buffer */

#define FLEXIO_SHIFTBUF_SHIFTBUF(x)              ((((uint32_t)(x)) << FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT) & FLEXIO_SHIFTBUF_SHIFTBUF_MASK)

/* The count of FLEXIO_SHIFTBUF */

#define FLEXIO_SHIFTBUF_COUNT                    (8u)

/* SHIFTBUFBIS - Shifter Buffer N Bit Swapped Register */

#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK      (0xffffffffu)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT     (0u)

/* SHIFTBUFBIS - Shift Buffer */

#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS(x)        ((((uint32_t)(x)) << FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT) & FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK)

/* The count of FLEXIO_SHIFTBUFBIS */

#define FLEXIO_SHIFTBUFBIS_COUNT                 (8u)

/* SHIFTBUFBYS - Shifter Buffer N Byte Swapped Register */

#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK      (0xffffffffu)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT     (0u)

/* SHIFTBUFBYS - Shift Buffer */

#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS(x)        ((((uint32_t)(x)) << FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT) & FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK)

/* The count of FLEXIO_SHIFTBUFBYS */

#define FLEXIO_SHIFTBUFBYS_COUNT                 (8u)

/* SHIFTBUFBBS - Shifter Buffer N Bit Byte Swapped Register */

#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK      (0xffffffffu)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT     (0u)

/* SHIFTBUFBBS - Shift Buffer */

#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS(x)        ((((uint32_t)(x)) << FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT) & FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK)

/* The count of FLEXIO_SHIFTBUFBBS */

#define FLEXIO_SHIFTBUFBBS_COUNT                 (8u)

/* TIMCTL - Timer Control N Register */

#define FLEXIO_TIMCTL_TIMOD_MASK                 (0x3u)
#define FLEXIO_TIMCTL_TIMOD_SHIFT                (0u)

/* TIMOD - Timer Mode
 *  0b00..Timer Disabled.
 *  0b01..Dual 8-bit counters baud mode.
 *  0b10..Dual 8-bit counters PWM high mode.
 *  0b11..Single 16-bit counter mode.
 */

#define FLEXIO_TIMCTL_TIMOD(x)                   ((((uint32_t)(x)) << FLEXIO_TIMCTL_TIMOD_SHIFT) & FLEXIO_TIMCTL_TIMOD_MASK)

#define FLEXIO_TIMCTL_PINPOL_MASK                (0x80u)
#define FLEXIO_TIMCTL_PINPOL_SHIFT               (7u)

/* PINPOL - Timer Pin Polarity
 *  0b0..Pin is active high
 *  0b1..Pin is active low
 */

#define FLEXIO_TIMCTL_PINPOL(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCTL_PINPOL_SHIFT) & FLEXIO_TIMCTL_PINPOL_MASK)

#define FLEXIO_TIMCTL_PINSEL_MASK                (0x1f00u)  /* Merged from fields with different position or width, of widths (4, 5), largest definition used */
#define FLEXIO_TIMCTL_PINSEL_SHIFT               (8u)

/* PINSEL - Timer Pin Select */

#define FLEXIO_TIMCTL_PINSEL(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCTL_PINSEL_SHIFT) & FLEXIO_TIMCTL_PINSEL_MASK)  /* Merged from fields with different position or width, of widths (4, 5), largest definition used */

#define FLEXIO_TIMCTL_PINCFG_MASK                (0x30000u)
#define FLEXIO_TIMCTL_PINCFG_SHIFT               (16u)

/* PINCFG - Timer Pin Configuration
 *  0b00..Timer pin output disabled
 *  0b01..Timer pin open drain or bidirectional output enable
 *  0b10..Timer pin bidirectional output data
 *  0b11..Timer pin output
 */

#define FLEXIO_TIMCTL_PINCFG(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCTL_PINCFG_SHIFT) & FLEXIO_TIMCTL_PINCFG_MASK)

#define FLEXIO_TIMCTL_TRGSRC_MASK                (0x400000u)
#define FLEXIO_TIMCTL_TRGSRC_SHIFT               (22u)

/* TRGSRC - Trigger Source
 *  0b0..External trigger selected
 *  0b1..Internal trigger selected
 */

#define FLEXIO_TIMCTL_TRGSRC(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCTL_TRGSRC_SHIFT) & FLEXIO_TIMCTL_TRGSRC_MASK)

#define FLEXIO_TIMCTL_TRGPOL_MASK                (0x800000u)
#define FLEXIO_TIMCTL_TRGPOL_SHIFT               (23u)

/* TRGPOL - Trigger Polarity
 *  0b0..Trigger active high
 *  0b1..Trigger active low
 */

#define FLEXIO_TIMCTL_TRGPOL(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCTL_TRGPOL_SHIFT) & FLEXIO_TIMCTL_TRGPOL_MASK)

#define FLEXIO_TIMCTL_TRGSEL_MASK                (0x3f000000u)  /* Merged from fields with different position or width, of widths (5, 6), largest definition used */
#define FLEXIO_TIMCTL_TRGSEL_SHIFT               (24u)

/* TRGSEL - Trigger Select */

#define FLEXIO_TIMCTL_TRGSEL(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCTL_TRGSEL_SHIFT) & FLEXIO_TIMCTL_TRGSEL_MASK)  /* Merged from fields with different position or width, of widths (5, 6), largest definition used */

/* The count of FLEXIO_TIMCTL */

#define FLEXIO_TIMCTL_COUNT                      (8u)

/* TIMCFG - Timer Configuration N Register */

#define FLEXIO_TIMCFG_TSTART_MASK                (0x2u)
#define FLEXIO_TIMCFG_TSTART_SHIFT               (1u)

/* TSTART - Timer Start Bit
 *  0b0..Start bit disabled
 *  0b1..Start bit enabled
 */

#define FLEXIO_TIMCFG_TSTART(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCFG_TSTART_SHIFT) & FLEXIO_TIMCFG_TSTART_MASK)

#define FLEXIO_TIMCFG_TSTOP_MASK                 (0x30u)
#define FLEXIO_TIMCFG_TSTOP_SHIFT                (4u)

/* TSTOP - Timer Stop Bit
 *  0b00..Stop bit disabled
 *  0b01..Stop bit is enabled on timer compare
 *  0b10..Stop bit is enabled on timer disable
 *  0b11..Stop bit is enabled on timer compare and timer disable
 */

#define FLEXIO_TIMCFG_TSTOP(x)                   ((((uint32_t)(x)) << FLEXIO_TIMCFG_TSTOP_SHIFT) & FLEXIO_TIMCFG_TSTOP_MASK)

#define FLEXIO_TIMCFG_TIMENA_MASK                (0x700u)
#define FLEXIO_TIMCFG_TIMENA_SHIFT               (8u)

/* TIMENA - Timer Enable
 *  0b000..Timer always enabled
 *  0b001..Timer enabled on Timer N-1 enable
 *  0b010..Timer enabled on Trigger high
 *  0b011..Timer enabled on Trigger high and Pin high
 *  0b100..Timer enabled on Pin rising edge
 *  0b101..Timer enabled on Pin rising edge and Trigger high
 *  0b110..Timer enabled on Trigger rising edge
 *  0b111..Timer enabled on Trigger rising or falling edge
 */

#define FLEXIO_TIMCFG_TIMENA(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCFG_TIMENA_SHIFT) & FLEXIO_TIMCFG_TIMENA_MASK)

#define FLEXIO_TIMCFG_TIMDIS_MASK                (0x7000u)
#define FLEXIO_TIMCFG_TIMDIS_SHIFT               (12u)

/* TIMDIS - Timer Disable
 *  0b000..Timer never disabled
 *  0b001..Timer disabled on Timer N-1 disable
 *  0b010..Timer disabled on Timer compare (upper 8-bits match and decrement)
 *  0b011..Timer disabled on Timer compare (upper 8-bits match and decrement)
 *         and Trigger Low
 *  0b100..Timer disabled on Pin rising or falling edge
 *  0b101..Timer disabled on Pin rising or falling edge provided Trigger is
 *         high
 *  0b110..Timer disabled on Trigger falling edge
 *  0b111..Reserved
 */

#define FLEXIO_TIMCFG_TIMDIS(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCFG_TIMDIS_SHIFT) & FLEXIO_TIMCFG_TIMDIS_MASK)

#define FLEXIO_TIMCFG_TIMRST_MASK                (0x70000u)
#define FLEXIO_TIMCFG_TIMRST_SHIFT               (16u)

/* TIMRST - Timer Reset
 *  0b000..Timer never reset
 *  0b001..Reserved
 *  0b010..Timer reset on Timer Pin equal to Timer Output
 *  0b011..Timer reset on Timer Trigger equal to Timer Output
 *  0b100..Timer reset on Timer Pin rising edge
 *  0b101..Reserved
 *  0b110..Timer reset on Trigger rising edge
 *  0b111..Timer reset on Trigger rising or falling edge
 */

#define FLEXIO_TIMCFG_TIMRST(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCFG_TIMRST_SHIFT) & FLEXIO_TIMCFG_TIMRST_MASK)

#define FLEXIO_TIMCFG_TIMDEC_MASK                (0x300000u)
#define FLEXIO_TIMCFG_TIMDEC_SHIFT               (20U)

/* TIMDEC - Timer Decrement
 *  0b00..Decrement counter on FlexIO clock, Shift clock equals Timer output.
 *  0b01..Decrement counter on Trigger input (both edges), Shift clock equals
 *        Timer output.
 *  0b10..Decrement counter on Pin input (both edges), Shift clock equals Pin
 *        input.
 *  0b11..Decrement counter on Trigger input (both edges), Shift clock equals
 *        Trigger input.
 */

#define FLEXIO_TIMCFG_TIMDEC(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCFG_TIMDEC_SHIFT) & FLEXIO_TIMCFG_TIMDEC_MASK)

#define FLEXIO_TIMCFG_TIMOUT_MASK                (0x3000000u)
#define FLEXIO_TIMCFG_TIMOUT_SHIFT               (24u)

/* TIMOUT - Timer Output
 *  0b00..Timer output is logic one when enabled and is not affected by timer
 *        reset
 *  0b01..Timer output is logic zero when enabled and is not affected by
 *        timer reset
 *  0b10..Timer output is logic one when enabled and on timer reset
 *  0b11..Timer output is logic zero when enabled and on timer reset
 */

#define FLEXIO_TIMCFG_TIMOUT(x)                  ((((uint32_t)(x)) << FLEXIO_TIMCFG_TIMOUT_SHIFT) & FLEXIO_TIMCFG_TIMOUT_MASK)

/* The count of FLEXIO_TIMCFG */

#define FLEXIO_TIMCFG_COUNT                      (8u)

/* TIMCMP - Timer Compare N Register */

#define FLEXIO_TIMCMP_CMP_MASK                   (0xffffu)
#define FLEXIO_TIMCMP_CMP_SHIFT                  (0u)

/* CMP - Timer Compare Value */

#define FLEXIO_TIMCMP_CMP(x)                     ((((uint32_t)(x)) << FLEXIO_TIMCMP_CMP_SHIFT) & FLEXIO_TIMCMP_CMP_MASK)

/* The count of FLEXIO_TIMCMP */

#define FLEXIO_TIMCMP_COUNT                      (8u)

/* SHIFTBUFNBS - Shifter Buffer N Nibble Byte Swapped Register */

#define FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_MASK      (0xffffffffu)
#define FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_SHIFT     (0u)

/* SHIFTBUFNBS - Shift Buffer */

#define FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS(x)        ((((uint32_t)(x)) << FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_SHIFT) & FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_MASK)

/* The count of FLEXIO_SHIFTBUFNBS */

#define FLEXIO_SHIFTBUFNBS_COUNT                 (8u)

/* SHIFTBUFHWS - Shifter Buffer N Half Word Swapped Register */

#define FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_MASK      (0xffffffffu)
#define FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_SHIFT     (0u)

/* SHIFTBUFHWS - Shift Buffer */

#define FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS(x)        ((((uint32_t)(x)) << FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_SHIFT) & FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_MASK)

/* The count of FLEXIO_SHIFTBUFHWS */

#define FLEXIO_SHIFTBUFHWS_COUNT                 (8u)

/* SHIFTBUFNIS - Shifter Buffer N Nibble Swapped Register */

#define FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_MASK      (0xffffffffu)
#define FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_SHIFT     (0u)

/* SHIFTBUFNIS - Shift Buffer */

#define FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS(x)        ((((uint32_t)(x)) << FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_SHIFT) & FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_MASK)

/* The count of FLEXIO_SHIFTBUFNIS */

#define FLEXIO_SHIFTBUFNIS_COUNT                 (8u)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXIO_H */
