/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_dac.h
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_DAC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DAC register offsets *****************************************************/

#define SAM_DAC_CTRLA_OFFSET       0x0000 /* Control A Register */
#define SAM_DAC_CTRLB_OFFSET       0x0001 /* Control B Register */
#define SAM_DAC_EVCTRL_OFFSET      0x0002 /* Event Control Register */
#define SAM_DAC_INTENCLR_OFFSET    0x0004 /* Interrupt Enable Clear Register */
#define SAM_DAC_INTENSET_OFFSET    0x0005 /* Interrupt Enable Set Register */
#define SAM_DAC_INTFLAG_OFFSET     0x0006 /* Interrupt Flag Status and Clear Register */
#define SAM_DAC_STATUS_OFFSET      0x0007 /* Status Register */
#define SAM_DAC_DATA0_OFFSET       0x0008 /* Data DAC0 Register */
#define SAM_DAC_DATA1_OFFSET       0x0009 /* Data DAC1 Register */
#define SAM_DAC_DATABUF0_OFFSET    0x000C /* Data Buffer DAC0 Register */
#define SAM_DAC_DATABUF1_OFFSET    0x000D /* Data Buffer DAC1 Register */

/* DAC register addresses ***************************************************/

#define SAM_DAC_CTRLA              (SAM_DAC_BASE+SAM_DAC_CTRLA_OFFSET)
#define SAM_DAC_CTRLB              (SAM_DAC_BASE+SAM_DAC_CTRLB_OFFSET)
#define SAM_DAC_EVCTRL             (SAM_DAC_BASE+SAM_DAC_EVCTRL_OFFSET)
#define SAM_DAC_INTENCLR           (SAM_DAC_BASE+SAM_DAC_INTENCLR_OFFSET)
#define SAM_DAC_INTENSET           (SAM_DAC_BASE+SAM_DAC_INTENSET_OFFSET)
#define SAM_DAC_INTFLAG            (SAM_DAC_BASE+SAM_DAC_INTFLAG_OFFSET)
#define SAM_DAC_STATUS             (SAM_DAC_BASE+SAM_DAC_STATUS_OFFSET)
#define SAM_DAC_DATA0              (SAM_DAC_BASE+SAM_DAC_DATA0_OFFSET)
#define SAM_DAC_DATA1              (SAM_DAC_BASE+SAM_DAC_DATA1_OFFSET)
#define SAM_DAC_DATABUF0           (SAM_DAC_BASE+SAM_DAC_DATABUF0_OFFSET)
#define SAM_DAC_DATABUF1           (SAM_DAC_BASE+SAM_DAC_DATABUF1_OFFSET)

/* DAC register bit definitions *********************************************/

/* Control A Register */

#define DAC_CTRLA_SWRTS            (1 << 0)  /* Bit 0:  Software reset */
#define DAC_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable DAC controller */
#define DAC_CTRLA_RUNSTDBY         (1 << 2)  /* Bit 1:  Run in standby */

/* Control B Register */

#define DAC_CTRLB_EOEN             (1 << 0)  /* Bit 0:  External Output Enable (to Vout) */
#define DAC_CTRLB_IOEN             (1 << 1)  /* Bit 1:  Internal Output Enable (to analog comparator) */
#define DAC_CTRLB_LEFTADJ          (1 << 2)  /* Bit 2:  Left-Adjusted Data */
#define DAC_CTRLB_VPD              (1 << 3)  /* Bit 3:  Voltage Pump Disabled */
#define DAC_CTRLB_BDWP             (1 << 4)  /* Bit 4:  Bypass DATABUF Write protection */
#define DAC_CTRLB_REFSEL_SHIFT     (6)       /* Bit 7:6: Reference selection */
#define DAC_CTRLB_REFSEL_MASK      (3 << DAC_CTRLB_REFSEL_SHIFT)
#  define DAC_CTRLB_REFSEL_INTREF  (0 << DAC_CTRLB_REFSEL_SHIFT) /* Internal voltage reference */
#  define DAC_CTRLB_REFSEL_VDDANA  (1 << DAC_CTRLB_REFSEL_SHIFT) /* Analog voltage supply */
#  define DAC_CTRLB_REFSEL_VREFA   (2 << DAC_CTRLB_REFSEL_SHIFT) /* External voltage reference */

/* Event Control Register */

#define DAC_EVCTRL_STARTEI         (1 << 0)  /* Bit 0:  Start conversion event input */
#define DAC_EVCTRL_EMPTYEO         (1 << 1)  /* Bit 1:  Data buffer empty event output */

/* Common bit definitions for Interrupt Enable Clear Register,
 * Interrupt Enable Set Register,
 * and Interrupt Flag Status and Clear Register
 */

#define DAC_INT_UNDERRUN           (1 << 0)  /* Bit 0:  Underrun interrupt */
#define DAC_INT_EMPTY              (1 << 1)  /* Bit 1:  Data buffer empty interrupt */
#define DAC_INT_SYNCRDY            (1 << 2)  /* Bit 2:  Sync ready */
#define DAC_INT_ALL                0x07

/* Status Register */

#define DAC_STATUS_SYNCBUSY        (1 << 7)  /* Bit 0:  Sync busy */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_DAC_H */
