/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_flexcom.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common Flexcom Register Offsets ******************************************/

#define SAM_FLEX_MR_OFFSET     0x000 /* FLEXCOM Mode Register */
                                     /* 0x0004–0x000c Reserved */
#define SAM_FLEX_RHR_OFFSET    0x010 /* FLEXCOM Receive Holding Register */
                                     /* 0x0014–0x001c Reserved */
#define SAM_FLEX_THR_OFFSET    0x020 /* FLEXCOM Transmit Holding Register */
                                     /* 0x0024–0x01fc Reserved */
                                     /* 0x0200-0x03ff: Flexcom USART register */
                                     /* 0x0400-0x05ff: Flexcom USART register */
                                     /* 0x0600-0x07ff: Flexcom USART register */

/* Common Flexcom Register Addresses ****************************************/

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM0
#  define SAM_FLEX0_MR         (SAM_FLEXCOM0_VBASE+SAM_FLEX_MR_OFFSET)
#  define SAM_FLEX0_RHR        (SAM_FLEXCOM0_VBASE+SAM_FLEX_RHR_OFFSET)
#  define SAM_FLEX0_THR        (SAM_FLEXCOM0_VBASE+SAM_FLEX_THR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM1
#  define SAM_FLEX1_MR         (SAM_FLEXCOM1_VBASE+SAM_FLEX_MR_OFFSET)
#  define SAM_FLEX1_RHR        (SAM_FLEXCOM1_VBASE+SAM_FLEX_RHR_OFFSET)
#  define SAM_FLEX1_THR        (SAM_FLEXCOM1_VBASE+SAM_FLEX_THR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM2
#  define SAM_FLEX2_MR         (SAM_FLEXCOM2_VBASE+SAM_FLEX_MR_OFFSET)
#  define SAM_FLEX2_RHR        (SAM_FLEXCOM2_VBASE+SAM_FLEX_RHR_OFFSET)
#  define SAM_FLEX2_THR        (SAM_FLEXCOM2_VBASE+SAM_FLEX_THR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM3
#  define SAM_FLEX3_MR         (SAM_FLEXCOM3_VBASE+SAM_FLEX_MR_OFFSET)
#  define SAM_FLEX3_RHR        (SAM_FLEXCOM3_VBASE+SAM_FLEX_RHR_OFFSET)
#  define SAM_FLEX3_THR        (SAM_FLEXCOM3_VBASE+SAM_FLEX_THR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM4
#  define SAM_FLEX4_MR         (SAM_FLEXCOM4_VBASE+SAM_FLEX_MR_OFFSET)
#  define SAM_FLEX4_RHR        (SAM_FLEXCOM4_VBASE+SAM_FLEX_RHR_OFFSET)
#  define SAM_FLEX4_THR        (SAM_FLEXCOM4_VBASE+SAM_FLEX_THR_OFFSET)
#endif

/* Common Flexcom Register Bit Field Definitions ****************************/

/* FLEXCOM Mode Register */

#define FLEX_MR_OPMODE_SHIFT   (0)  /* Bits 0-1: Operating mode */
#define FLEX_MR_OPMODE_MASK    (3 << FLEX_MR_OPMODE_SHIFT)
#  define FLEX_MR_OPMODE_NOCOM (0 << FLEX_MR_OPMODE_SHIFT) /* No communication */
#  define FLEX_MR_OPMODE_USART (1 << FLEX_MR_OPMODE_SHIFT) /* All UART related protocols selected */
#  define FLEX_MR_OPMODE_SPI   (2 << FLEX_MR_OPMODE_SHIFT) /* SPI operating mode is selected */
#  define FLEX_MR_OPMODE_TWI   (3 << FLEX_MR_OPMODE_SHIFT) /* All TWI related protocols are selected */

/* FLEXCOM Receive Holding Register */

#define FLEX_RHR_MASK          (0xffff)

/* FLEXCOM Transmit Holding Register */

#define FLEX_THR_MASK          (0xffff)

/* Flexcom USART Register Definitions ***************************************/

#include "hardware/sam_flexcom_usart.h"

/* Flexcom SPI Register Definitions *****************************************/

#include "hardware/sam_flexcom_spi.h"

/* Flexcom TWI Register Definitions *****************************************/

#include "hardware/sam_flexcom_twi.h"

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_H */
