/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_flexio.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLEXIO_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLEXIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/s32k1xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FlexIO Register Offsets **************************************************/

#define S32K1XX_FLEXIO_VERID_OFFSET          (0x0000) /* Version ID Register (VERID) */
#define S32K1XX_FLEXIO_PARAM_OFFSET          (0x0004) /* Parameter Register (PARAM) */
#define S32K1XX_FLEXIO_CTRL_OFFSET           (0x0008) /* FlexIO Control Register (CTRL) */
#define S32K1XX_FLEXIO_PIN_OFFSET            (0x000c) /* Pin State Register (PIN) */
#define S32K1XX_FLEXIO_SHIFTSTAT_OFFSET      (0x0010) /* Shifter Status Register (SHIFTSTAT) */
#define S32K1XX_FLEXIO_SHIFTERR_OFFSET       (0x0014) /* Shifter Error Register (SHIFTERR) */
#define S32K1XX_FLEXIO_TIMSTAT_OFFSET        (0x0018) /* Timer Status Register (TIMSTAT) */
#define S32K1XX_FLEXIO_SHIFTSIEN_OFFSET      (0x0020) /* Shifter Status Interrupt Enable (SHIFTSIEN) */
#define S32K1XX_FLEXIO_SHIFTEIEN_OFFSET      (0x0024) /* Shifter Error Interrupt Enable (SHIFTEIEN) */
#define S32K1XX_FLEXIO_TIMIEN_OFFSET         (0x0028) /* Timer Interrupt Enable Register (TIMIEN) */
#define S32K1XX_FLEXIO_SHIFTSDEN_OFFSET      (0x0030) /* Shifter Status DMA Enable (SHIFTSDEN) */
#define S32K1XX_FLEXIO_SHIFTCTL0_OFFSET      (0x0080) /* Shifter Control 0 Register (SHIFTCTL0) */
#define S32K1XX_FLEXIO_SHIFTCTL1_OFFSET      (0x0084) /* Shifter Control 1 Register (SHIFTCTL1) */
#define S32K1XX_FLEXIO_SHIFTCTL2_OFFSET      (0x0088) /* Shifter Control 2 Register (SHIFTCTL2) */
#define S32K1XX_FLEXIO_SHIFTCTL3_OFFSET      (0x008c) /* Shifter Control 3 Register (SHIFTCTL3) */
#define S32K1XX_FLEXIO_SHIFTCFG0_OFFSET      (0x0100) /* Shifter Configuration 0 Register (SHIFTCFG0) */
#define S32K1XX_FLEXIO_SHIFTCFG1_OFFSET      (0x0104) /* Shifter Configuration 1 Register (SHIFTCFG1) */
#define S32K1XX_FLEXIO_SHIFTCFG2_OFFSET      (0x0108) /* Shifter Configuration 2 Register (SHIFTCFG2) */
#define S32K1XX_FLEXIO_SHIFTCFG3_OFFSET      (0x010c) /* Shifter Configuration 3 Register (SHIFTCFG3) */
#define S32K1XX_FLEXIO_SHIFTBUF0_OFFSET      (0x0200) /* Shifter Buffer 0 Register (SHIFTBUF0) */
#define S32K1XX_FLEXIO_SHIFTBUF1_OFFSET      (0x0204) /* Shifter Buffer 1 Register (SHIFTBUF1) */
#define S32K1XX_FLEXIO_SHIFTBUF2_OFFSET      (0x0208) /* Shifter Buffer 2 Register (SHIFTBUF2) */
#define S32K1XX_FLEXIO_SHIFTBUF3_OFFSET      (0x020c) /* Shifter Buffer 3 Register (SHIFTBUF3) */
#define S32K1XX_FLEXIO_SHIFTBUFBIS0_OFFSET   (0x0280) /* Shifter Buffer 0 Bit Swapped Register (SHIFTBUFBIS0) */
#define S32K1XX_FLEXIO_SHIFTBUFBIS1_OFFSET   (0x0284) /* Shifter Buffer 1 Bit Swapped Register (SHIFTBUFBIS1) */
#define S32K1XX_FLEXIO_SHIFTBUFBIS2_OFFSET   (0x0288) /* Shifter Buffer 2 Bit Swapped Register (SHIFTBUFBIS2) */
#define S32K1XX_FLEXIO_SHIFTBUFBIS3_OFFSET   (0x028c) /* Shifter Buffer 3 Bit Swapped Register (SHIFTBUFBIS3) */
#define S32K1XX_FLEXIO_SHIFTBUFBYS0_OFFSET   (0x0300) /* Shifter Buffer 0 Byte Swapped Register (SHIFTBUFBYS0) */
#define S32K1XX_FLEXIO_SHIFTBUFBYS1_OFFSET   (0x0304) /* Shifter Buffer 1 Byte Swapped Register (SHIFTBUFBYS1) */
#define S32K1XX_FLEXIO_SHIFTBUFBYS2_OFFSET   (0x0308) /* Shifter Buffer 2 Byte Swapped Register (SHIFTBUFBYS2) */
#define S32K1XX_FLEXIO_SHIFTBUFBYS3_OFFSET   (0x030c) /* Shifter Buffer 3 Byte Swapped Register (SHIFTBUFBYS3) */
#define S32K1XX_FLEXIO_SHIFTBUFBBS0_OFFSET   (0x0380) /* Shifter Buffer 0 Bit Byte Swapped Register (SHIFTBUFBBS0) */
#define S32K1XX_FLEXIO_SHIFTBUFBBS1_OFFSET   (0x0384) /* Shifter Buffer 1 Bit Byte Swapped Register (SHIFTBUFBBS1) */
#define S32K1XX_FLEXIO_SHIFTBUFBBS2_OFFSET   (0x0388) /* Shifter Buffer 2 Bit Byte Swapped Register (SHIFTBUFBBS2) */
#define S32K1XX_FLEXIO_SHIFTBUFBBS3_OFFSET   (0x038c) /* Shifter Buffer 3 Bit Byte Swapped Register (SHIFTBUFBBS3) */
#define S32K1XX_FLEXIO_TIMCTL0_OFFSET        (0x0400) /* Timer Control 0 Register (TIMCTL0) */
#define S32K1XX_FLEXIO_TIMCTL1_OFFSET        (0x0404) /* Timer Control 1 Register (TIMCTL1) */
#define S32K1XX_FLEXIO_TIMCTL2_OFFSET        (0x0408) /* Timer Control 2 Register (TIMCTL2) */
#define S32K1XX_FLEXIO_TIMCTL3_OFFSET        (0x040c) /* Timer Control 3 Register (TIMCTL3) */
#define S32K1XX_FLEXIO_TIMCFG0_OFFSET        (0x0480) /* Timer Configuration 0 Register (TIMCFG0) */
#define S32K1XX_FLEXIO_TIMCFG1_OFFSET        (0x0484) /* Timer Configuration 1 Register (TIMCFG1) */
#define S32K1XX_FLEXIO_TIMCFG2_OFFSET        (0x0488) /* Timer Configuration 2 Register (TIMCFG2) */
#define S32K1XX_FLEXIO_TIMCFG3_OFFSET        (0x048c) /* Timer Configuration 3 Register (TIMCFG3) */
#define S32K1XX_FLEXIO_TIMCMP0_OFFSET        (0x0500) /* Timer Compare 0 Register (TIMCMP0) */
#define S32K1XX_FLEXIO_TIMCMP1_OFFSET        (0x0504) /* Timer Compare 1 Register (TIMCMP1) */
#define S32K1XX_FLEXIO_TIMCMP2_OFFSET        (0x0508) /* Timer Compare 2 Register (TIMCMP2) */
#define S32K1XX_FLEXIO_TIMCMP3_OFFSET        (0x050c) /* Timer Compare 3 Register (TIMCMP3) */

#define S32K1XX_FLEXIO_SHIFTCTL_OFFSET(n)    (0x0080 + ((n) << 2)) /* Shifter Control n Register (SHIFTCTLn) */
#define S32K1XX_FLEXIO_SHIFTCFG_OFFSET(n)    (0x0100 + ((n) << 2)) /* Shifter Configuration n Register (SHIFTCFGn) */
#define S32K1XX_FLEXIO_SHIFTBUF_OFFSET(n)    (0x0200 + ((n) << 2)) /* Shifter Buffer n Register (SHIFTBUFn) */
#define S32K1XX_FLEXIO_SHIFTBUFBIS_OFFSET(n) (0x0280 + ((n) << 2)) /* Shifter Buffer n Bit Swapped Register (SHIFTBUFBISn) */
#define S32K1XX_FLEXIO_SHIFTBUFBYS_OFFSET(n) (0x0300 + ((n) << 2)) /* Shifter Buffer n Byte Swapped Register (SHIFTBUFBYSn) */
#define S32K1XX_FLEXIO_SHIFTBUFBBS_OFFSET(n) (0x0380 + ((n) << 2)) /* Shifter Buffer n Bit Byte Swapped Register (SHIFTBUFBBSn) */
#define S32K1XX_FLEXIO_TIMCTL_OFFSET(n)      (0x0400 + ((n) << 2)) /* Timer Control n Register (TIMCTLn) */
#define S32K1XX_FLEXIO_TIMCFG_OFFSET(n)      (0x0480 + ((n) << 2)) /* Timer Configuration n Register (TIMCFGn) */
#define S32K1XX_FLEXIO_TIMCMP_OFFSET(n)      (0x0500 + ((n) << 2)) /* Timer Compare n Register (TIMCMPn) */

/* FlexIO Register Addresses ************************************************/

#define S32K1XX_FLEXIO_VERID                 (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_VERID_OFFSET)
#define S32K1XX_FLEXIO_PARAM                 (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_PARAM_OFFSET)
#define S32K1XX_FLEXIO_CTRL                  (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_CTRL_OFFSET)
#define S32K1XX_FLEXIO_PIN                   (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_PIN_OFFSET)
#define S32K1XX_FLEXIO_SHIFTSTAT             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTSTAT_OFFSET)
#define S32K1XX_FLEXIO_SHIFTERR              (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTERR_OFFSET)
#define S32K1XX_FLEXIO_TIMSTAT               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMSTAT_OFFSET)
#define S32K1XX_FLEXIO_SHIFTSIEN             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTSIEN_OFFSET)
#define S32K1XX_FLEXIO_SHIFTEIEN             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTEIEN_OFFSET)
#define S32K1XX_FLEXIO_TIMIEN                (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMIEN_OFFSET)
#define S32K1XX_FLEXIO_SHIFTSDEN             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTSDEN_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCTL0             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCTL0_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCTL1             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCTL1_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCTL2             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCTL2_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCTL3             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCTL3_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCFG0             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCFG0_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCFG1             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCFG1_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCFG2             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCFG2_OFFSET)
#define S32K1XX_FLEXIO_SHIFTCFG3             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCFG3_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUF0             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUF0_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUF1             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUF1_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUF2             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUF2_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUF3             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUF3_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBIS0          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBIS0_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBIS1          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBIS1_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBIS2          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBIS2_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBIS3          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBIS3_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBYS0          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBYS0_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBYS1          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBYS1_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBYS2          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBYS2_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBYS3          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBYS3_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBBS0          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBBS0_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBBS1          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBBS1_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBBS2          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBBS2_OFFSET)
#define S32K1XX_FLEXIO_SHIFTBUFBBS3          (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBBS3_OFFSET)
#define S32K1XX_FLEXIO_TIMCTL0               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCTL0_OFFSET)
#define S32K1XX_FLEXIO_TIMCTL1               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCTL1_OFFSET)
#define S32K1XX_FLEXIO_TIMCTL2               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCTL2_OFFSET)
#define S32K1XX_FLEXIO_TIMCTL3               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCTL3_OFFSET)
#define S32K1XX_FLEXIO_TIMCFG0               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCFG0_OFFSET)
#define S32K1XX_FLEXIO_TIMCFG1               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCFG1_OFFSET)
#define S32K1XX_FLEXIO_TIMCFG2               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCFG2_OFFSET)
#define S32K1XX_FLEXIO_TIMCFG3               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCFG3_OFFSET)
#define S32K1XX_FLEXIO_TIMCMP0               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCMP0_OFFSET)
#define S32K1XX_FLEXIO_TIMCMP1               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCMP1_OFFSET)
#define S32K1XX_FLEXIO_TIMCMP2               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCMP2_OFFSET)
#define S32K1XX_FLEXIO_TIMCMP3               (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCMP3_OFFSET)

#define S32K1XX_FLEXIO_SHIFTCTL(n)           (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCTL_OFFSET(n))
#define S32K1XX_FLEXIO_SHIFTCFG(n)           (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTCFG_OFFSET(n))
#define S32K1XX_FLEXIO_SHIFTBUF(n)           (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUF_OFFSET(n))
#define S32K1XX_FLEXIO_SHIFTBUFBIS(n)        (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBIS_OFFSET(n))
#define S32K1XX_FLEXIO_SHIFTBUFBYS(n)        (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBYS_OFFSET(n))
#define S32K1XX_FLEXIO_SHIFTBUFBBS(n)        (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_SHIFTBUFBBS_OFFSET(n))
#define S32K1XX_FLEXIO_TIMCTL(n)             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCTL(n))
#define S32K1XX_FLEXIO_TIMCFG(n)             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCFG(n))
#define S32K1XX_FLEXIO_TIMCMP(n)             (S32K1XX_FLEXIO_BASE + S32K1XX_FLEXIO_TIMCMP(n))

/* FlexIO Register Bit Definitions ******************************************/

/* Version ID Register (VERID) */

#define FLEXIO_VERID_FEATURE_SHIFT        (0)       /* Bits 0-15: Feature Specification Number (FEATURE) */
#define FLEXIO_VERID_FEATURE_MASK         (0xffff << FLEXIO_VERID_FEATURE_SHIFT)
#define FLEXIO_VERID_MINOR_SHIFT          (16)      /* Bits 16-23: Minor Version Number (MINOR) */
#define FLEXIO_VERID_MINOR_MASK           (0xff << FLEXIO_VERID_MINOR_SHIFT)
#define FLEXIO_VERID_MAJOR_SHIFT          (24)      /* Bits 24-31: Major Version Number (MAJOR) */
#define FLEXIO_VERID_MAJOR_MASK           (0xff << FLEXIO_VERID_MAJOR_SHIFT)

/* Parameter Register (PARAM) */

#define FLEXIO_PARAM_SHIFTER_SHIFT        (0)       /* Bits 0-7: Shifter Number (SHIFTER) */
#define FLEXIO_PARAM_SHIFTER_MASK         (0xff << FLEXIO_PARAM_SHIFTER_SHIFT)
#define FLEXIO_PARAM_TIMER_SHIFT          (8)       /* Bits 8-15: Timer Number (TIMER) */
#define FLEXIO_PARAM_TIMER_MASK           (0xff << FLEXIO_PARAM_TIMER_SHIFT)
#define FLEXIO_PARAM_PIN_SHIFT            (16)      /* Bits 16-23: Pin Number (PIN) */
#define FLEXIO_PARAM_PIN_MASK             (0xff << FLEXIO_PARAM_PIN_SHIFT)
#define FLEXIO_PARAM_TRIGGER_SHIFT        (24)      /* Bits 24-31: Trigger Number (TRIGGER) */
#define FLEXIO_PARAM_TRIGGER_MASK         (0xff << FLEXIO_PARAM_TRIGGER_SHIFT)

/* FlexIO Control Register (CTRL) */

#define FLEXIO_CTRL_FLEXEN                (1 << 0)  /* Bit 0: FlexIO Enable (FLEXEN) */
#  define FLEXIO_CTRL_FLEXEN_DIS          (0 << 0)  /*        FlexIO module is disabled */
#  define FLEXIO_CTRL_FLEXEN_ENA          (1 << 0)  /*        FlexIO module is enabled */
#define FLEXIO_CTRL_SWRST                 (1 << 1)  /* Bit 1: Software Reset (SWRST) */
#  define FLEXIO_CTRL_SWRST_DIS           (0 << 1)  /*        Software reset is disabled */
#  define FLEXIO_CTRL_SWRST_ENA           (1 << 1)  /*        Software reset is enabled */
#define FLEXIO_CTRL_FASTACC               (1 << 2)  /* Bit 2: Fast Access (FASTACC) */
#  define FLEXIO_CTRL_FASTACC_NORMAL      (0 << 2)  /*        Configures for normal register accesses to FlexIO */
#  define FLEXIO_CTRL_FASTACC_FAST        (1 << 2)  /*        Configures for fast register accesses to FlexIO */
                                                    /* Bits 3-29: Reserved */
#define FLEXIO_CTRL_DBGE                  (1 << 30) /* Bit 30: Debug Enable (DBGE) */
#  define FLEXIO_CTRL_DBGE_DIS            (0 << 30) /*         FlexIO is disabled in debug modes */
#  define FLEXIO_CTRL_DBGE_ENA            (1 << 30) /*         FlexIO is enabled in debug modes */
#define FLEXIO_CTRL_DOZEN                 (1 << 31) /* Bit 31: Doze Enable (DOZEN) */
#  define FLEXIO_CTRL_DOZEN_ENA           (0 << 31) /*         FlexIO enabled in Doze modes */
#  define FLEXIO_CTRL_DOZEN_DIS           (1 << 31) /*         FlexIO disabled in Doze modes */

/* Pin State Register (PIN) */

#define FLEXIO_PIN_PDI_SHIFT              (0)       /* Bits 0-7: Pin Data Input (PDI) */
#define FLEXIO_PIN_PDI_MASK               (0xff << FLEXIO_PIN_PDI_SHIFT)
                                                    /* Bits 8-31: Reserved */

/* Shifter Status Register (SHIFTSTAT) */

#define FLEXIO_SHIFTSTAT_SSF_SHIFT        (0)       /* Bits 0-3: Shifter Status Flag (SSF) */
#define FLEXIO_SHIFTSTAT_SSF_MASK         (0x0f << FLEXIO_SHIFTSTAT_SSF_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* Shifter Error Register (SHIFTERR) */

#define FLEXIO_SHIFTERR_SEF_SHIFT         (0)       /* Bits 0-3: Shifter Error Flags (SEF) */
#define FLEXIO_SHIFTERR_SEF_MASK          (0x0f << FLEXIO_SHIFTERR_SEF_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* Timer Status Register (TIMSTAT) */

#define FLEXIO_TIMSTAT_TSF_SHIFT          (0)       /* Bits 0-3: Timer Status Flags (TSF) */
#define FLEXIO_TIMSTAT_TSF_MASK           (0x0f << FLEXIO_TIMSTAT_TSF_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* Shifter Status Interrupt Enable (SHIFTSIEN) */

#define FLEXIO_SHIFTSIEN_SSIE_SHIFT       (0)       /* Bits 0-3: Shifter Status Interrupt Enable (SSIE) */
#define FLEXIO_SHIFTSIEN_SSIE_MASK        (0x0f << FLEXIO_SHIFTSIEN_SSIE_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* Shifter Error Interrupt Enable (SHIFTEIEN) */

#define FLEXIO_SHIFTEIEN_SEIE_SHIFT       (0)       /* Bits 0-3: Shifter Error Interrupt Enable (SEIE) */
#define FLEXIO_SHIFTEIEN_SEIE_MASK        (0x0f << FLEXIO_SHIFTEIEN_SEIE_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* Timer Interrupt Enable Register (TIMIEN) */

#define FLEXIO_TIMIEN_TEIE_SHIFT          (0)       /* Bits 0-3: Timer Status Interrupt Enable (TEIE) */
#define FLEXIO_TIMIEN_TEIE_MASK           (0x0f << FLEXIO_TIMIEN_TEIE_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* Shifter Status DMA Enable (SHIFTSDEN) */

#define FLEXIO_SHIFTSDEN_SSDE_SHIFT       (0)       /* Bits 0-3: Shifter Status DMA Enable (SSDE) */
#define FLEXIO_SHIFTSDEN_SSDE_MASK        (0x0f << FLEXIO_SHIFTSDEN_SSDE_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* Shifter Control n Register (SHIFTCTLn) */

#define FLEXIO_SHIFTCTL_SMOD_SHIFT        (0)       /* Bits 0-2: Shifter Mode (SMOD) */
#define FLEXIO_SHIFTCTL_SMOD_MASK         (0x07 << FLEXIO_SHIFTCTL_SMOD_SHIFT)
#  define FLEXIO_SHIFTCTL_SMOD_DIS        (0x00 << FLEXIO_SHIFTCTL_SMOD_SHIFT) /* Disabled */
#  define FLEXIO_SHIFTCTL_SMOD_RX         (0x01 << FLEXIO_SHIFTCTL_SMOD_SHIFT) /* Receive mode */
#  define FLEXIO_SHIFTCTL_SMOD_TX         (0x02 << FLEXIO_SHIFTCTL_SMOD_SHIFT) /* Transmit mode */
#  define FLEXIO_SHIFTCTL_SMOD_STORE      (0x04 << FLEXIO_SHIFTCTL_SMOD_SHIFT) /* Match Store mode */
#  define FLEXIO_SHIFTCTL_SMOD_CONT       (0x05 << FLEXIO_SHIFTCTL_SMOD_SHIFT) /* Match Continuous mode */

                                                    /* Bits 3-6: Reserved */
#define FLEXIO_SHIFTCTL_PINPOL            (1 << 7)  /* Bit 7: Shifter Pin Polarity (PINPOL) */
#  define FLEXIO_SHIFTCTL_PINPOL_HI       (0 << 7)  /*        Pin is active high */
#  define FLEXIO_SHIFTCTL_PINPOL_LO       (1 << 7)  /*        Pin is active low */
#define FLEXIO_SHIFTCTL_PINSEL_SHIFT      (8)       /* Bits 8-10: Shifter Pin Select (PINSEL) */
#define FLEXIO_SHIFTCTL_PINSEL_MASK       (0x07 << FLEXIO_SHIFTCTL_PINSEL_SHIFT)
#  define FLEXIO_SHIFTCTL_PINSEL(n)       (((n) << FLEXIO_SHIFTCTL_PINSEL_SHIFT) & FLEXIO_SHIFTCTL_PINSEL_MASK)
                                                    /* Bits 11-15: Reserved */
#define FLEXIO_SHIFTCTL_PINCFG_SHIFT      (16)      /* Bits 16-17: Shifter Pin Configuration (PINCFG) */
#define FLEXIO_SHIFTCTL_PINCFG_MASK       (0x03 << FLEXIO_SHIFTCTL_PINCFG_SHIFT)
#  define FLEXIO_SHIFTCTL_PINCFG_DIS      (0x00 << FLEXIO_SHIFTCTL_PINCFG_SHIFT) /* Shifter pin output disabled */
#  define FLEXIO_SHIFTCTL_PINCFG_OD       (0x01 << FLEXIO_SHIFTCTL_PINCFG_SHIFT) /* Shifter pin open drain or bidirectional output enable */
#  define FLEXIO_SHIFTCTL_PINCFG_BID      (0x02 << FLEXIO_SHIFTCTL_PINCFG_SHIFT) /* Shifter pin bidirectional output data */
#  define FLEXIO_SHIFTCTL_PINCFG_OUT      (0x03 << FLEXIO_SHIFTCTL_PINCFG_SHIFT) /* Shifter pin output */

                                                    /* Bits 18-22: Reserved */
#define FLEXIO_SHIFTCTL_TIMPOL            (1 << 23) /* Bit 23: Timer Polarity (TIMPOL) */
#  define FLEXIO_SHIFTCTL_TIMPOL_PE       (0 << 23) /*         Shift on posedge of Shift clock */
#  define FLEXIO_SHIFTCTL_TIMPOL_NE       (1 << 23) /*         Shift on negedge of Shift clock */
#define FLEXIO_SHIFTCTL_TIMSEL_SHIFT      (24)      /* Bit 24-25: Timer Select (TIMSEL) */
#define FLEXIO_SHIFTCTL_TIMSEL_MASK       (0x03 << FLEXIO_SHIFTCTL_TIMSEL_SHIFT)
#  define FLEXIO_SHIFTCTL_TIMSEL(n)       (((n) << FLEXIO_SHIFTCTL_TIMSEL_SHIFT) & FLEXIO_SHIFTCTL_TIMSEL_MASK)
                                                    /* Bits 26-31: Reserved */

/* Shifter Configuration n Register (SHIFTCFGn) */

#define FLEXIO_SHIFTCFG_SSTART_SHIFT      (0)       /* Bits 0-1: Shifter Start bit (SSTART) */
#define FLEXIO_SHIFTCFG_SSTART_MASK       (0x03 << FLEXIO_SHIFTCFG_SSTART_SHIFT)
#  define FLEXIO_SHIFTCFG_SSTART_DIS      (0x00 << FLEXIO_SHIFTCFG_SSTART_SHIFT) /* Start bit disabled for transmitter/receiver/match store, transmitter loads data on enable */
#  define FLEXIO_SHIFTCFG_SSTART_DIS_SH   (0x01 << FLEXIO_SHIFTCFG_SSTART_SHIFT) /* Start bit disabled for transmitter/receiver/match store, transmitter loads data on first shift */
#  define FLEXIO_SHIFTCFG_SSTART_ZERO     (0x02 << FLEXIO_SHIFTCFG_SSTART_SHIFT) /* Transmitter outputs start bit value 0 before loading data on first shift, receiver/match store sets error flag if start bit is not 0 */
#  define FLEXIO_SHIFTCFG_SSTART_ONE      (0x03 << FLEXIO_SHIFTCFG_SSTART_SHIFT) /* Transmitter outputs start bit value 1 before loading data on first shift, receiver/match store sets error flag if start bit is not 1 */

                                                    /* Bits 2-3: Reserved */
#define FLEXIO_SHIFTCFG_SSTOP_SHIFT       (4)       /* Bits 4-5: Shifter Stop bit (SSTOP) */
#define FLEXIO_SHIFTCFG_SSTOP_MASK        (0x03 << FLEXIO_SHIFTCFG_SSTOP_SHIFT)
#  define FLEXIO_SHIFTCFG_SSTOP_DIS       (0x00 << FLEXIO_SHIFTCFG_SSTOP_SHIFT) /* Stop bit disabled for transmitter/receiver/match store */
#  define FLEXIO_SHIFTCFG_SSTOP_ZERO      (0x02 << FLEXIO_SHIFTCFG_SSTOP_SHIFT) /* Transmitter outputs stop bit value 0 on store, receiver/match store sets error flag if stop bit is not 0 */
#  define FLEXIO_SHIFTCFG_SSTOP_ONE       (0x03 << FLEXIO_SHIFTCFG_SSTOP_SHIFT) /* Transmitter outputs stop bit value 1 on store, receiver/match store sets error flag if stop bit is not 1 */

                                                    /* Bits 6-7: Reserved */
#define FLEXIO_SHIFTCFG_INSRC             (1 << 8)  /* Bit 8: Input Source (INSRC) */
#  define FLEXIO_SHIFTCFG_INSRC_PIN       (0 << 8)  /*        Pin */
#  define FLEXIO_SHIFTCFG_INSRC_SHIFTER   (1 << 8)  /*        Shifter N+1 Output */
                                                    /* Bits 9-31: Reserved */

/* Shifter Buffer n Register (SHIFTBUFn) */

#define FLEXIO_SHIFTBUF_SHIFT             (0)       /* Bits 0-31: Shift Buffer (SHIFTBUF) */
#define FLEXIO_SHIFTBUF_MASK              (0xffffffff << FLEXIO_SHIFTBUF_SHIFT)

/* Shifter Buffer n Bit Swapped Register (SHIFTBUFBISn) */

#define FLEXIO_SHIFTBUFBIS_SHIFT          (0)       /* Bits 0-31: Shift Buffer (bit swapped) (SHIFTBUFBIS) */
#define FLEXIO_SHIFTBUFBIS_MASK           (0xffffffff << FLEXIO_SHIFTBUFBIS_SHIFT)

/* Shifter Buffer n Byte Swapped Register (SHIFTBUFBYSn) */

#define FLEXIO_SHIFTBUFBYS_SHIFT          (0)       /* Bits 0-31: Shift Buffer (byte swapped) (SHIFTBUFBYS) */
#define FLEXIO_SHIFTBUFBYS_MASK           (0xffffffff << FLEXIO_SHIFTBUFBYS_SHIFT)

/* Shifter Buffer n Bit & Byte Swapped Register (SHIFTBUFBBSn) */

#define FLEXIO_SHIFTBUFBBS_SHIFT          (0)       /* Bits 0-31: Shift Buffer (bit and byte swapped) (SHIFTBUFBBS) */
#define FLEXIO_SHIFTBUFBBS_MASK           (0xffffffff << FLEXIO_SHIFTBUFBBS_SHIFT)

/* Timer Control n Register (TIMCTLn) */

#define FLEXIO_TIMCTL_TIMOD_SHIFT         (0)       /* Bits 0-1: Timer Mode (TIMOD) */
#define FLEXIO_TIMCTL_TIMOD_MASK          (0x03 << FLEXIO_TIMCTL_TIMOD_SHIFT)
#  define FLEXIO_TIMCTL_TIMOD_DIS         (0x00 << FLEXIO_TIMCTL_TIMOD_SHIFT) /* Timer Disabled */
#  define FLEXIO_TIMCTL_TIMOD_8BBAUD      (0x01 << FLEXIO_TIMCTL_TIMOD_SHIFT) /* Dual 8-bit counters baud mode */
#  define FLEXIO_TIMCTL_TIMOD_8BPWMHI     (0x02 << FLEXIO_TIMCTL_TIMOD_SHIFT) /* Dual 8-bit counters PWM high mode */
#  define FLEXIO_TIMCTL_TIMOD_16BCNT      (0x03 << FLEXIO_TIMCTL_TIMOD_SHIFT) /* Single 16-bit counter mode */

                                                    /* Bits 2-6: Reserved */
#define FLEXIO_TIMCTL_PINPOL              (1 << 7)  /* Bit 7: Timer Pin Polarity (PINPOL) */
#  define FLEXIO_TIMCTL_PINPOL_HI         (0 << 7)  /*        Pin is active high */
#  define FLEXIO_TIMCTL_PINPOL_LO         (1 << 7)  /*        Pin is active low */
#define FLEXIO_TIMCTL_PINSEL_SHIFT        (8)       /* Bits 8-10: Timer Pin Select (PINSEL) */ 
#define FLEXIO_TIMCTL_PINSEL_MASK         (0x07 << FLEXIO_TIMCTL_PINSEL_SHIFT)
#  define FLEXIO_TIMCTL_PINSEL(n)         (((n) << FLEXIO_TIMCTL_PINSEL_SHIFT) & FLEXIO_TIMCTL_PINSEL_MASK)
                                                    /* Bits 11-15: Reserved */
#define FLEXIO_TIMCTL_PINCFG_SHIFT        (16)      /* Bits 16-17: Timer Pin Configuration (PINCFG) */
#define FLEXIO_TIMCTL_PINCFG_MASK         (0x03 << FLEXIO_TIMCTL_PINCFG_SHIFT)
#  define FLEXIO_TIMCTL_PINCFG_DIS        (0x00 << FLEXIO_TIMCTL_PINCFG_SHIFT) /* Timer pin output disabled */
#  define FLEXIO_TIMCTL_PINCFG_OD         (0x01 << FLEXIO_TIMCTL_PINCFG_SHIFT) /* Timer pin open drain or bidirectional output enable */
#  define FLEXIO_TIMCTL_PINCFG_BID        (0x02 << FLEXIO_TIMCTL_PINCFG_SHIFT) /* Timer pin bidirectional output data */
#  define FLEXIO_TIMCTL_PINCFG_OUT        (0x03 << FLEXIO_TIMCTL_PINCFG_SHIFT) /* Timer pin output */

                                                    /* Bits 18-21: Reserved */
#define FLEXIO_TIMCTL_TRGSRC              (1 << 22) /* Bit 22: Trigger Source (TRGSRC) */
#  define FLEXIO_TIMCTL_TRGSRC_EXT        (0 << 22) /*         External trigger selected */
#  define FLEXIO_TIMCTL_TRGSRC_INT        (1 << 22) /*         Internal trigger selected */
#define FLEXIO_TIMCTL_TRGPOL              (1 << 23) /* Bit 23: Trigger Polarity (TRGPOL) */
#  define FLEXIO_TIMCTL_TRGPOL_HI         (0 << 23) /*         Trigger active high */
#  define FLEXIO_TIMCTL_TRGPOL_LO         (1 << 23) /*         Trigger active low */
#define FLEXIO_TIMCTL_TRGSEL_SHIFT        (24)      /* Bits 24-27: Trigger Select (TRGSEL) */
#define FLEXIO_TIMCTL_TRGSEL_MASK         (0x0f << FLEXIO_TIMCTL_TRGSEL_SHIFT)
#  define FLEXIO_TIMCTL_TRGSEL_EXT(n)     (((n) << FLEXIO_TIMCTL_TRGSEL_SHIFT) & FLEXIO_TIMCTL_TRGSEL_MASK)       /* External trigger n input */
#  define FLEXIO_TIMCTL_TRGSEL_PIN(n)     (((2*(n)) << FLEXIO_TIMCTL_TRGSEL_SHIFT) & FLEXIO_TIMCTL_TRGSEL_MASK)   /* Pin n input */
#  define FLEXIO_TIMCTL_TRGSEL_SHIFTER(n) (((4*(n)+1) << FLEXIO_TIMCTL_TRGSEL_SHIFT) & FLEXIO_TIMCTL_TRGSEL_MASK) /* Shifter n status flag */
#  define FLEXIO_TIMCTL_TRGSEL_TIMER(n)   (((4*(n)+3) << FLEXIO_TIMCTL_TRGSEL_SHIFT) & FLEXIO_TIMCTL_TRGSEL_MASK) /* Timer n trigger output */

                                                    /* Bits 28-31: Reserved */

/* Timer Configuration n Register (TIMCFGn) */

                                                    /* Bit 0: Reserved */
#define FLEXIO_TIMCFG_TSTART              (1 << 1)  /* Bit 1: Timer Start Bit (TSTART) */
#  define FLEXIO_TIMCFG_TSTART_DIS        (0 << 1)  /*        Start bit disabled */
#  define FLEXIO_TIMCFG_TSTART_ENA        (1 << 1)  /*        Start bit enabled */
                                                    /* Bits 2-3: Reserved */
#define FLEXIO_TIMCFG_TSTOP_SHIFT         (4)       /* Bits 4-5: Timer Stop Bit (TSTOP) */ 
#define FLEXIO_TIMCFG_TSTOP_MASK          (0x03 << FLEXIO_TIMCFG_TSTOP_SHIFT)
#  define FLEXIO_TIMCFG_TSTOP_DIS         (0x00 << FLEXIO_TIMCFG_TSTOP_SHIFT) /* Stop bit disabled */
#  define FLEXIO_TIMCFG_TSTOP_TIMCMP      (0x01 << FLEXIO_TIMCFG_TSTOP_SHIFT) /* Stop bit is enabled on timer compare */
#  define FLEXIO_TIMCFG_TSTOP_TIMDIS      (0x02 << FLEXIO_TIMCFG_TSTOP_SHIFT) /* Stop bit is enabled on timer disable */
#  define FLEXIO_TIMCFG_TSTOP_BOTH        (0x03 << FLEXIO_TIMCFG_TSTOP_SHIFT) /* Stop bit is enabled on timer compare and timer disable */

                                                    /* Bits 6-7: Reserved */
#define FLEXIO_TIMCFG_TIMENA_SHIFT        (8)       /* Bits 8-10: Timer Enable (TIMENA) */ 
#define FLEXIO_TIMCFG_TIMENA_MASK         (0x07 << FLEXIO_TIMCFG_TIMENA_SHIFT)
#  define FLEXIO_TIMCFG_TIMENA_ALWAYS     (0x00 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer always enabled */
#  define FLEXIO_TIMCFG_TIMENA_TIMENA     (0x01 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer enabled on Timer N-1 enable */
#  define FLEXIO_TIMCFG_TIMENA_TRGHI      (0x02 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer enabled on Trigger high */
#  define FLEXIO_TIMCFG_TIMENA_TRGHIPIN   (0x03 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer enabled on Trigger high and Pin high */
#  define FLEXIO_TIMCFG_TIMENA_PINRIS     (0x04 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer enabled on Pin rising edge */
#  define FLEXIO_TIMCFG_TIMENA_PINTRG     (0x05 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer enabled on Pin rising edge and Trigger high */
#  define FLEXIO_TIMCFG_TIMENA_TRGRIS     (0x06 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer enabled on Trigger rising edge */
#  define FLEXIO_TIMCFG_TIMENA_TRGBOTH    (0x07 << FLEXIO_TIMCFG_TIMENA_SHIFT) /* Timer enabled on Trigger rising or falling edge */

                                                    /* Bit 11: Reserved */
#define FLEXIO_TIMCFG_TIMDIS_SHIFT        (12)      /* Bits 12-14: Timer Disable (TIMDIS) */
#define FLEXIO_TIMCFG_TIMDIS_MASK         (0x07 << FLEXIO_TIMCFG_TIMDIS_SHIFT)
#  define FLEXIO_TIMCFG_TIMDIS_NEVER      (0x00 << FLEXIO_TIMCFG_TIMDIS_SHIFT) /* Timer never disabled */
#  define FLEXIO_TIMCFG_TIMDIS_TIMDIS     (0x01 << FLEXIO_TIMCFG_TIMDIS_SHIFT) /* Timer disabled on Timer N-1 disable */
#  define FLEXIO_TIMCFG_TIMDIS_TIMCMP     (0x02 << FLEXIO_TIMCFG_TIMDIS_SHIFT) /* Timer disabled on Timer compare (upper 8-bits match and decrement) */
#  define FLEXIO_TIMCFG_TIMDIS_CMPTRGLO   (0x03 << FLEXIO_TIMCFG_TIMDIS_SHIFT) /* Timer disabled on Timer compare (upper 8-bits match and decrement) and Trigger Low */
#  define FLEXIO_TIMCFG_TIMDIS_PIN        (0x04 << FLEXIO_TIMCFG_TIMDIS_SHIFT) /* Timer disabled on Pin rising or falling edge */
#  define FLEXIO_TIMCFG_TIMDIS_PINTRGHI   (0x05 << FLEXIO_TIMCFG_TIMDIS_SHIFT) /* Timer disabled on Pin rising or falling edge provided Trigger is high */
#  define FLEXIO_TIMCFG_TIMDIS_TRG        (0x06 << FLEXIO_TIMCFG_TIMDIS_SHIFT) /* Timer disabled on Trigger falling edge */

                                                    /* Bit 15: Reserved */
#define FLEXIO_TIMCFG_TIMRST_SHIFT        (16)      /* Bits 16-18: Timer Reset (TIMRST) */
#define FLEXIO_TIMCFG_TIMRST_MASK         (0x07 << FLEXIO_TIMCFG_TIMRST_SHIFT)
#  define FLEXIO_TIMCFG_TIMRST_NEVER      (0x00 << FLEXIO_TIMCFG_TIMRST_SHIFT) /* Timer never reset */
#  define FLEXIO_TIMCFG_TIMRST_PINOUT     (0x02 << FLEXIO_TIMCFG_TIMRST_SHIFT) /* Timer reset on Timer Pin equal to Timer Output */
#  define FLEXIO_TIMCFG_TIMRST_TRGOUT     (0x03 << FLEXIO_TIMCFG_TIMRST_SHIFT) /* Timer reset on Timer Trigger equal to Timer Output */
#  define FLEXIO_TIMCFG_TIMRST_PINRIS     (0x04 << FLEXIO_TIMCFG_TIMRST_SHIFT) /* Timer reset on Timer Pin rising edge */
#  define FLEXIO_TIMCFG_TIMRST_TRGRIS     (0x06 << FLEXIO_TIMCFG_TIMRST_SHIFT) /* Timer reset on Trigger rising edge */
#  define FLEXIO_TIMCFG_TIMRST_TRGBOTH    (0x07 << FLEXIO_TIMCFG_TIMRST_SHIFT) /* Timer reset on Trigger rising or falling edge */

                                                    /* Bit 19: Reserved */
#define FLEXIO_TIMCFG_TIMDEC_SHIFT             (20) /* Bits 20-21: Timer Decrement (TIMDEC) */ 
#define FLEXIO_TIMCFG_TIMDEC_MASK              (0x03 << FLEXIO_TIMCFG_TIMDEC_SHIFT)
#  define FLEXIO_TIMCFG_TIMDEC_CLKTIMOUT       (0x00 << FLEXIO_TIMCFG_TIMDEC_SHIFT) /* Decrement counter on FlexIO clock, Shift clock equals Timer output */
#  define FLEXIO_TIMCFG_TIMDEC_TRGINBOTHTIMOUT (0x01 << FLEXIO_TIMCFG_TIMDEC_SHIFT) /* Decrement counter on Trigger input (both edges), Shift clock equals Timer output */
#  define FLEXIO_TIMCFG_TIMDEC_PINBOTHPIN      (0x02 << FLEXIO_TIMCFG_TIMDEC_SHIFT) /* Decrement counter on Pin input (both edges), Shift clock equals Pin input */
#  define FLEXIO_TIMCFG_TIMDEC_TRGINBOTHTRGIN  (0x03 << FLEXIO_TIMCFG_TIMDEC_SHIFT) /* Decrement counter on Trigger input (both edges), Shift clock equals Trigger input */

                                                    /* Bit 23: Reserved */
#define FLEXIO_TIMCFG_TIMOUT_SHIFT        (24)      /* Bits 24-25: Timer Output (TIMOUT) */ 
#define FLEXIO_TIMCFG_TIMOUT_MASK         (0x03 << FLEXIO_TIMCFG_TIMOUT_SHIFT)
#  define FLEXIO_TIMCFG_TIMOUT_ONE        (0x00 << FLEXIO_TIMCFG_TIMOUT_SHIFT) /* Timer output is logic one when enabled and is not affected by timer reset */
#  define FLEXIO_TIMCFG_TIMOUT_ZERO       (0x01 << FLEXIO_TIMCFG_TIMOUT_SHIFT) /* Timer output is logic zero when enabled and is not affected by timer reset */
#  define FLEXIO_TIMCFG_TIMOUT_ONERST     (0x02 << FLEXIO_TIMCFG_TIMOUT_SHIFT) /* Timer output is logic one when enabled and on timer reset */
#  define FLEXIO_TIMCFG_TIMOUT_ZERORST    (0x03 << FLEXIO_TIMCFG_TIMOUT_SHIFT) /* Timer output is logic zero when enabled and on timer reset */

                                                    /* Bits 26-31: Reserved */

/* Timer Compare n Register (TIMCMPn) */

#define FLEXIO_TIMCMP_CMP_SHIFT           (0)       /* Bits 0-15: Timer Compare Value (CMP) */
#define FLEXIO_TIMCMP_CMP_MASK            (0xffff << FLEXIO_TIMCMP_CMP_SHIFT)
                                                    /* Bits 16-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLEXIO_H */
