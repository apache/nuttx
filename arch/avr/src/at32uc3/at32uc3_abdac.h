/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_abdac.h
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
 ************************************************************************************/

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_ABDAC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_ABDAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_ABDAC_SDR_OFFSET    0x00 /* Sample Data Register */
#define AVR32_ABDAC_CR_OFFSET     0x08 /* Control Register */
#define AVR32_ABDAC_IMR_OFFSET    0x0c /* Interrupt Mask Register */
#define AVR32_ABDAC_IER_OFFSET    0x10 /* Interrupt Enable Register */
#define AVR32_ABDAC_IDR_OFFSET    0x14 /* Interrupt Disable Register */
#define AVR32_ABDAC_ICR_OFFSET    0x18 /* Interrupt Clear Register */
#define AVR32_ABDAC_ISR_OFFSET    0x1c /* Interrupt Status Register */

/* Register Addresses ***************************************************************/

#define AVR32_ABDAC_SDR           (AVR32_ABDAC_BASE+AVR32_ABDAC_SDR_OFFSET)
#define AVR32_ABDAC_CR            (AVR32_ABDAC_BASE+AVR32_ABDAC_CR_OFFSET)
#define AVR32_ABDAC_IMR           (AVR32_ABDAC_BASE+AVR32_ABDAC_IMR_OFFSET)
#define AVR32_ABDAC_IER           (AVR32_ABDAC_BASE+AVR32_ABDAC_IER_OFFSET)
#define AVR32_ABDAC_IDR           (AVR32_ABDAC_BASE+AVR32_ABDAC_IDR_OFFSET)
#define AVR32_ABDAC_ICR           (AVR32_ABDAC_BASE+AVR32_ABDAC_ICR_OFFSET)
#define AVR32_ABDAC_ISR           (AVR32_ABDAC_BASE+AVR32_ABDAC_ISR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Sample Data Register Bit-field Definitions */

/* This register contains a 32-bit data and, hence, has no bit-fiels */

/* Control Register Bit-field Definitions */

#define ABDAC_CR_SWAP             (1 << 30) /* Bit 30: Swap Channels */
#define ABDAC_CR_EN               (1 << 31) /* Bit 31: Enable Audio Bitstream DAC */

/* Interrupt Mask Register Bit-field Definitions */

/* Interrupt Enable Register Bit-field Definitions */

/* Interrupt Disable Register Bit-field Definitions */

/* Interrupt Clear Register Bit-field Definitions */

/* Interrupt Status Register Bit-field Definitions */

#define ABDAC_INT_UNDERRUN        (1 << 28)  /* Bit 28:  Underrun Interrupt Status */
#define ABDAC_INT_TXREADY         (1 << 29)  /* Bit 29  TX Ready Interrupt Status */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions Prototypes
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_ABDAC_H */
