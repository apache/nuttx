/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_utmi.h
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
 *   SAMV7D3 Series Data Sheet
 */

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UTMI_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UTMI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define SAM_UTMI_OHCIICR_OFFSET            0x0010            /* OHCI Interrupt Configuration Register */
#define SAM_UTMI_CKTRIM_OFFSET             0x0030            /* UTMI Clock Trimming Register */

/* Register addresses *******************************************************/

#define SAM_UTMI_OHCIICR                   (SAM_UTMI_BASE+SAM_UTMI_OHCIICR_OFFSET)
#define SAM_UTMI_CKTRIM                    (SAM_UTMI_BASE+SAM_UTMI_CKTRIM_OFFSET)

/* Register bit-field definitions *******************************************/

/* OHCI Interrupt Configuration Register */

#define UTMI_OHCIICR_RES0                  (1 << 0)          /* Bit 0:  USB PORT0 Reset */
#define UTMI_OHCIICR_ARIE                  (1 << 4)          /* Bit 4:  OHCI Asynchronous Resume Interrupt Enable */
#define UTMI_OHCIICR_APPSTART              (0 << 5)          /* Bit 5:  Reserved, must be zero */
#define UTMI_OHCIICR_UDPPUDIS              (1 << 23)         /* Bit 23: USB Device Pull-up Disable */

/* UTMI Clock Trimming Register */

#define UTMI_CKTRIM_FREQ_SHIFT             (0)               /* Bits 0-1: UTMI Reference Clock Frequency */
#define UTMI_CKTRIM_FREQ_MASK              (3 << UTMI_CKTRIM_FREQ_SHIFT)
#  define UTMI_CKTRIM_FREQ_XTAL12          (0 << UTMI_CKTRIM_FREQ_SHIFT) /* 12 MHz reference clock */
#  define UTMI_CKTRIM_FREQ_XTAL16          (1 << UTMI_CKTRIM_FREQ_SHIFT) /* 16 MHz reference clock */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UTMI_H */
