/****************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_supc.h
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

/* References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_SUPC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_SUPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SUPC register offsets ****************************************************/

#define SAM_SUPC_INTENCLR_OFFSET   0x0000  /* Interrupt enable clear */
#define SAM_SUPC_INTENSET_OFFSET   0x0004  /* Interrupt enable set */
#define SAM_SUPC_INTFLAG_OFFSET    0x0008  /* Interrupt flag status and clear */
#define SAM_SUPC_STATUS_OFFSET     0x000c  /* Status */
#define SAM_SUPC_BOD33_OFFSET      0x0010  /* 3.3V brown-out detector control */

#define SAM_SUPC_BOD12_OFFSET      0x0014  /* 1.2V brown-out detctor control */
#define SAM_SUPC_VREG_OFFSET       0x0018  /* Voltage regulator system control */

#define SAM_SUPC_VREF_OFFSET       0x001c  /* Voltage references system control */

#define SAM_SUPC_BBPS_OFFSET       0x0020  /* Battery backup power switch control */
#define SAM_SUPC_BKOUT_OFFSET      0x0024  /* Backup output control */
#define SAM_SUPC_BKIN_OFFSET       0x0028  /* Backup input value */

/* SUPC register addresses **************************************************/

#define SAM_SUPC_INTENCLR          (SAM_SUPC_BASE+SAM_SUPC_INTENCLR_OFFSET)
#define SAM_SUPC_INTENSET          (SAM_SUPC_BASE+SAM_SUPC_INTENSET_OFFSET)
#define SAM_SUPC_INTFLAG           (SAM_SUPC_BASE+SAM_SUPC_INTFLAG_OFFSET)
#define SAM_SUPC_STATUS            (SAM_SUPC_BASE+SAM_SUPC_STATUS_OFFSET)
#define SAM_SUPC_BOD33             (SAM_SUPC_BASE+SAM_SUPC_BOD33_OFFSET)
#define SAM_SUPC_BOD12             (SAM_SUPC_BASE+SAM_SUPC_BOD12_OFFSET)
#define SAM_SUPC_VREG              (SAM_SUPC_BASE+SAM_SUPC_VREG_OFFSET)
#define SAM_SUPC_VREF              (SAM_SUPC_BASE+SAM_SUPC_VREF_OFFSET)
#define SAM_SUPC_BBPS              (SAM_SUPC_BASE+SAM_SUPC_BBPS_OFFSET)
#define SAM_SUPC_BKOUT             (SAM_SUPC_BASE+SAM_SUPC_BKOUT_OFFSET)
#define SAM_SUPC_BKIN              (SAM_SUPC_BASE+SAM_SUPC_BKIN_OFFSET)

/* SUPC register bit definitions ********************************************/

/* Interrupt enable clear, Interrupt enable set, Interrupt flag status and
 * clear, and Status registers.
 */

#define SUPC_INT_BOD33RDY          (1 << 0)  /* Bit 0:  BOD33 ready interrupt */
#define SUPC_INT_BOD33DET          (1 << 1)  /* Bit 1:  BOD33 detection interrupt */
#define SUPC_INT_B33SRDY           (1 << 2)  /* Bit 2:  BOD33 synchronization ready interrupt */
#define SUPC_INT_BOD12RDY          (1 << 3)  /* Bit 3:  BOD12 ready interrupt */
#define SUPC_INT_BOD12DET          (1 << 4)  /* Bit 4:  BOD12 detection interrupt */
#define SUPC_INT_B12SRDY           (1 << 5)  /* Bit 5:  BOD12 synchronization ready interrupt */
#define SUPC_INT_VREGRDY           (1 << 8)  /* Bit 8:  Voltage regulator ready interrupt */
#define SUPC_INT_APWSRDY           (1 << 9)  /* Bit 9:  Automatic power switch ready interrupt */
#define SUPC_INT_VCORERDY          (1 << 10) /* Bit 10: VDDCORE voltage ready interrupt */

#define SUPC_INT_ALL               (0x0000073f)

/* 3.3V brown-out detector control register */

#define SUPC_BOD33_ENABLE          (1 << 1)  /* Bit 1: Enable */
#define SUPC_BOD33_HYST            (1 << 2)  /* Bit 2: Hysteresis */
#define SUPC_BOD33_ACTION_SHIFT    (3)       /* Bits 3-4: BOD33 action */
#define SUPC_BOD33_ACTION_MASK     (3 << SUPC_BOD33_ACTION_SHIFT)
#  define SUPC_BOD33_ACTION(n)     ((n) << SUPC_BOD33_ACTION_SHIFT)
#  define SUPC_BOD33_ACTION_NONE   (0 << SUPC_BOD33_ACTION_SHIFT) /* No action */
#  define SUPC_BOD33_ACTION_RESET  (1 << SUPC_BOD33_ACTION_SHIFT) /* BOD33 generates reset */
#  define SUPC_BOD33_ACTION_INTR   (2 << SUPC_BOD33_ACTION_SHIFT) /* BOD33 generates interrupt */
#  define SUPC_BOD33_ACTION_BKUP   (3 << SUPC_BOD33_ACTION_SHIFT) /* BOD33 backup sleep mode */

#define SUPC_BOD33_STDBYCFG        (1 << 5)  /* Bit 5:  BOD33 configuration in standby sleep mode */
#define SUPC_BOD33_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define SUPC_BOD33_RUNBKUP         (1 << 7)  /* Bit 7:  BOD33 configuration in backup sleep */
#define SUPC_BOD33_ACTCFG          (1 << 8)  /* Bit 8:  BOD33 configuration in active sleep */
#define SUPC_BOD33_VMON            (1 << 10) /* Bit 10: Voltage monitored in active and standby */
#define SUPC_BOD33_PSEL_SHIFT      (12)      /* Bits 12-15: Prescaler select */
#define SUPC_BOD33_PSEL_MASK       (15 << SUPC_BOD33_PSEL_SHIFT)
#  define SUPC_BOD33_PSEL(n)       ((uint32_t)(n) << SUPC_BOD33_PSEL_SHIFT)
#  define SUPC_BOD33_PSEL_DIV2     (0 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 2 */
#  define SUPC_BOD33_PSEL_DIV4     (1 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 4 */
#  define SUPC_BOD33_PSEL_DIV8     (2 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 8 */
#  define SUPC_BOD33_PSEL_DIV16    (3 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 16 */
#  define SUPC_BOD33_PSEL_DIV32    (4 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 32 */
#  define SUPC_BOD33_PSEL_DIV64    (5 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 64 */
#  define SUPC_BOD33_PSEL_DIV128   (6 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 128 */
#  define SUPC_BOD33_PSEL_DIV256   (7 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 256 */
#  define SUPC_BOD33_PSEL_DIV512   (8 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 512 */
#  define SUPC_BOD33_PSEL_DIV1K    (9 << SUPC_BOD33_PSEL_SHIFT)  /* Divide clock by 1024 */
#  define SUPC_BOD33_PSEL_DIV2K    (10 << SUPC_BOD33_PSEL_SHIFT) /* Divide clock by 2048 */
#  define SUPC_BOD33_PSEL_DIV4K    (11 << SUPC_BOD33_PSEL_SHIFT) /* Divide clock by 4096 */
#  define SUPC_BOD33_PSEL_DIV8K    (12 << SUPC_BOD33_PSEL_SHIFT) /* Divide clock by 8192 */
#  define SUPC_BOD33_PSEL_DIV16K   (13 << SUPC_BOD33_PSEL_SHIFT) /* Divide clock by 16384 */
#  define SUPC_BOD33_PSEL_DIV32K   (14 << SUPC_BOD33_PSEL_SHIFT) /* Divide clock by 32768 */
#  define SUPC_BOD33_PSEL_DIV64K   (15 << SUPC_BOD33_PSEL_SHIFT) /* Divide clock by 65536 */

#define SUPC_BOD33_LEVEL_SHIFT     (16)      /* Bits 16-21: BOD33 threshold level VDD */
#define SUPC_BOD33_LEVEL_MASK      (0x3f << SUPC_BOD33_LEVEL_SHIFT)
#  define SUPC_BOD33_LEVEL(n)      ((uint32_t)(n) << SUPC_BOD33_LEVEL_SHIFT)
#define SUPC_BOD33_BKUPLEVEL_SHIFT (24)      /* Bits 24-29: BOD33 threshold VBAT/backup sleep level */
#define SUPC_BOD33_BKUPLEVEL_MASK  (0x3f << SUPC_BOD33_BKUPLEVEL_SHIFT)
#  define SUPC_BOD33_BKUPLEVEL(n)  ((uint32_t)(n) << SUPC_BOD33_BKUPLEVEL_SHIFT)

/* 1.2V brown-out detctor control */

#define SUPC_BOD12_ENABLE          (1 << 1)  /* Bit 1: Enable */
#define SUPC_BOD12_HYST            (1 << 2)  /* Bit 2: Hysteresis */
#define SUPC_BOD12_ACTION_SHIFT    (3)       /* Bits 3-4: BOD12 action */
#define SUPC_BOD12_ACTION_MASK     (3 << SUPC_BOD12_ACTION_SHIFT)
#  define SUPC_BOD12_ACTION(n)     ((n) << SUPC_BOD12_ACTION_SHIFT)
#  define SUPC_BOD12_ACTION_NONE   (0 << SUPC_BOD12_ACTION_SHIFT) /* No action */
#  define SUPC_BOD12_ACTION_RESET  (1 << SUPC_BOD12_ACTION_SHIFT) /* BOD12 generates reset */
#  define SUPC_BOD12_ACTION_INTR   (2 << SUPC_BOD12_ACTION_SHIFT) /* BOD12 generates interrupt */

#define SUPC_BOD12_STDBYCFG        (1 << 5)  /* Bit 5:  BOD12 configuration in standby sleep mode */
#define SUPC_BOD12_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define SUPC_BOD12_ACTCFG          (1 << 8)  /* Bit 8:  BOD12 configuration in active sleep */
#define SUPC_BOD12_PSEL_SHIFT      (12)      /* Bits 12-15: Prescaler select */
#define SUPC_BOD12_PSEL_MASK       (15 << SUPC_BOD12_PSEL_SHIFT)
#  define SUPC_BOD12_PSEL(n)       ((uint32_t)(n) << SUPC_BOD12_PSEL_SHIFT)
#  define SUPC_BOD12_PSEL_DIV2     (0 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 2 */
#  define SUPC_BOD12_PSEL_DIV4     (1 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 4 */
#  define SUPC_BOD12_PSEL_DIV8     (2 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 8 */
#  define SUPC_BOD12_PSEL_DIV16    (3 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 16 */
#  define SUPC_BOD12_PSEL_DIV32    (4 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 32 */
#  define SUPC_BOD12_PSEL_DIV64    (5 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 64 */
#  define SUPC_BOD12_PSEL_DIV128   (6 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 128 */
#  define SUPC_BOD12_PSEL_DIV256   (7 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 256 */
#  define SUPC_BOD12_PSEL_DIV512   (8 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 512 */
#  define SUPC_BOD12_PSEL_DIV1K    (9 << SUPC_BOD12_PSEL_SHIFT)  /* Divide clock by 1024 */
#  define SUPC_BOD12_PSEL_DIV2K    (10 << SUPC_BOD12_PSEL_SHIFT) /* Divide clock by 2048 */
#  define SUPC_BOD12_PSEL_DIV4K    (11 << SUPC_BOD12_PSEL_SHIFT) /* Divide clock by 4096 */
#  define SUPC_BOD12_PSEL_DIV8K    (12 << SUPC_BOD12_PSEL_SHIFT) /* Divide clock by 8192 */
#  define SUPC_BOD12_PSEL_DIV16K   (13 << SUPC_BOD12_PSEL_SHIFT) /* Divide clock by 16384 */
#  define SUPC_BOD12_PSEL_DIV32K   (14 << SUPC_BOD12_PSEL_SHIFT) /* Divide clock by 32768 */
#  define SUPC_BOD12_PSEL_DIV64K   (15 << SUPC_BOD12_PSEL_SHIFT) /* Divide clock by 65536 */

#define SUPC_BOD12_LEVEL_SHIFT     (16)      /* Bits 16-21: BOD12 threshold level */
#define SUPC_BOD12_LEVEL_MASK      (0x3f << SUPC_BOD12_LEVEL_SHIFT)
#  define SUPC_BOD12_LEVEL(n)      ((uint32_t)(n) << SUPC_BOD12_LEVEL_SHIFT)

/* Voltage regulator system control */

#define SUPC_VREG_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define SUPC_VREG_SEL              (1 << 2)  /* Bit 2:  Voltage regulator selection */
#define SUPC_VREG_RUNDSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define SUPC_VREG_LPEFF            (1 << 8)  /* Bit 8:  Low power mode efficiency */
#define SUPC_VREG_VSVSTEP_SHIFT    (16)      /* Bits 16-19: Voltage scaling step */
#define SUPC_VREG_VSVSTEP_MASK     (15 << SUPC_VREG_VSVSTEP_SHIFT)
#  define SUPC_VREG_VSVSTEP(n)     ((uint32_t)(n) << SUPC_VREG_VSVSTEP_SHIFT)
#define SUPC_VREG_VSPER_SHIFT      (24)      /* Bits 24-31: Voltage scaling period */
#define SUPC_VREG_VSPER_MASK       (0xff << SUPC_VREG_VSPER_SHIFT)
#  define SUPC_VREG_VSPER(n)       ((uint32_t)(n) << SUPC_VREG_VSPER_SHIFT)

/* Voltage references system control register */

#define SUPC_VREF_TSEN             (1 << 1)  /* Bit 1:  Temperature sensor enable */
#define SUPC_VREF_VREFOE           (1 << 2)  /* Bit 2:  Voltage reference output enable */
#define SUPC_VREF_RUNSTDBY         (1 << 6)  /* Bit 6:  Run in standby */
#define SUPC_VREF_ONDEMAND         (1 << 7)  /* Bit 7:  On demand control */
#define SUPC_VREF_SEL_SHIFT        (16)      /* Bits 16-19: Voltage reference selection */
#define SUPC_VREF_SEL_MASK         (15 << SUPC_VREF_SEL_SHIFT)
#  define SUPC_VREF_SEL_1V0        (0 << SUPC_VREF_SEL_SHIFT) /* 1.0V voltage reference typical value */
#  define SUPC_VREF_SEL_1V1        (1 << SUPC_VREF_SEL_SHIFT) /* 1.1V voltage reference typical value */
#  define SUPC_VREF_SEL_1V2        (2 << SUPC_VREF_SEL_SHIFT) /* 1.2V voltage reference typical value */
#  define SUPC_VREF_SEL_1V25       (3 << SUPC_VREF_SEL_SHIFT) /* 1.25V voltage reference typical value */
#  define SUPC_VREF_SEL_2V0        (4 << SUPC_VREF_SEL_SHIFT) /* 2.0V voltage reference typical value */
#  define SUPC_VREF_SEL_2V2        (5 << SUPC_VREF_SEL_SHIFT) /* 2.2V voltage reference typical value */
#  define SUPC_VREF_SEL_2V4        (6 << SUPC_VREF_SEL_SHIFT) /* 2.4V voltage reference typical value */
#  define SUPC_VREF_SEL_2V5        (7 << SUPC_VREF_SEL_SHIFT) /* 5.5V voltage reference typical value */

/* Battery backup power switch control */

#define SUPC_BBPS_CONFIG_SHIFT     (0)        /* Bits 0-1: Battery backup power switch configuration */
#define SUPC_BBPS_CONFIG_MASK      (3 << SUPC_BBPS_CONFIG_SHIFT)
#  define SUPC_BBPS_CONFIG_NONE    (0 << SUPC_BBPS_CONFIG_SHIFT) /* Backup domain from main power */
#  define SUPC_BBPS_CONFIG_APWS    (1 << SUPC_BBPS_CONFIG_SHIFT) /* Automatic power switch */
#  define SUPC_BBPS_CONFIG_FORCED  (2 << SUPC_BBPS_CONFIG_SHIFT) /* Backup domain from batter backup power */
#  define SUPC_BBPS_CONFIG_BOD33   (3 << SUPC_BBPS_CONFIG_SHIFT) /* Power switch handled by BOD33 */

#define SUPC_BBPS_WAKEEN           (1 << 2)  /* Bit 2: Wake enable */
#define SUPC_BBPS_PSOKEN           (1 << 3)  /* Bit 3: Power supply OK enable */

/* Backup output control */

#define SUPC_BKOUT_EN_SHIFT        (0)       /* Bits 0-1: Output enable */
#define SUPC_BKOUT_EN_MASK         (3 << SUPC_BKOUT_EN_SHIFT)
#  define SUPC_BKOUT_DISABLE       (0 << SUPC_BKOUT_EN_SHIFT)
#  define SUPC_BKOUT_ENABLE        (1 << SUPC_BKOUT_EN_SHIFT)
#define SUPC_BKOUT_CLR_SHIFT       (8)       /* Bits 8-9: Clear output */
#define SUPC_BKOUT_CLR_MASK        (3 << SUPC_BKOUT_CLR_SHIFT)
#  define SUPC_BKOUT_CLR           (1 << SUPC_BKOUT_CLR_SHIFT)
#define SUPC_BKOUT_SET_SHIFT       (16)      /* Bits 16-17: Set output */
#define SUPC_BKOUT_SET_MASK        (3 << SUPC_BKOUT_SET_SHIFT)
#  define SUPC_BKOUT_SET           (1 << SUPC_BKOUT_SET_SHIFT)
#define SUPC_BKOUT_RTCTGL_SHIFT    (24)      /* Bits 24-25: RTC toggle output */
#define SUPC_BKOUT_RTCTGL_MASK     (3 << SUPC_BKOUT_RTCTGL_SHIFT)
#  define SUPC_BKOUT_RTCTGL_NONE   (0 << SUPC_BKOUT_EN_SHIFT) /* No toggle */
#  define SUPC_BKOUT_RTCTGL_TOGGLE (1 << SUPC_BKOUT_EN_SHIFT) /* Toggle */

/* Backup input value */

#define SUPC_BKIN_SHIFT            (0)       /* Bits 0-2: Backup I/O data input value */
#define SUPC_BKIN_MASK             (7 << SUPC_BKIN_SHIFT)
#  define SUPC_BKIN_PSOK           (1 << SUPC_BKIN_SHIFT) /* Input value of PSOCK pin */
#  define SUPC_BKIN_OUT0           (2 << SUPC_BKIN_SHIFT) /* Input value of OUT[0] pin */
#  define SUPC_BKIN_OUT1           (4 << SUPC_BKIN_SHIFT) /* Input value of OUT[1] pin */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_SUPC_H */
