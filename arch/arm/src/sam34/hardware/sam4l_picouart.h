/****************************************************************************
 * arch/arm/src/sam34/hardware/sam4l_picouart.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PICOUART_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PICOUART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PICOUART register offsets ************************************************/

#define SAM_PICOUART_CR_OFFSET       0x0000 /* Control Register */
#define SAM_PICOUART_CFG_OFFSET      0x0004 /* Configuration Register */
#define SAM_PICOUART_SR_OFFSET       0x0008 /* Status Register */
#define SAM_PICOUART_RHR_OFFSET      0x000c /* Receive Holding Register */
#define SAM_PICOUART_VERSION_OFFSET  0x0020 /* Version Register */

/* PICOUART register addresses **********************************************/

#define SAM_PICOUART_CR_OFFSET       0x0000 /* Control Register */
#define SAM_PICOUART_CR_OFFSET       0x0000 /* Control Register */
#define SAM_PICOUART_CFG_OFFSET      0x0004 /* Configuration Register */
#define SAM_PICOUART_CFG_OFFSET      0x0004 /* Configuration Register */
#define SAM_PICOUART_SR_OFFSET       0x0008 /* Status Register */
#define SAM_PICOUART_SR_OFFSET       0x0008 /* Status Register */
#define SAM_PICOUART_RHR_OFFSET      0x000c /* Receive Holding Register */
#define SAM_PICOUART_RHR_OFFSET      0x000c /* Receive Holding Register */
#define SAM_PICOUART_VERSION_OFFSET  0x0020 /* Version Register */
#define SAM_PICOUART_VERSION_OFFSET  0x0020 /* Version Register */

/* PICOUART register bit definitions ****************************************/

/* Control Register */

#define PICOUART_CR_EN               (1 << 0)  /* Bit 0:  Enable */
#define PICOUART_CR_DIS              (1 << 1)  /* Bit 1:  Disable */

/* Configuration Register */

#define PICOUART_CFG_SOURCE_SHIFT    (0)       /* Bit 0-1: Source Enable Mode */
#define PICOUART_CFG_SOURCE_MASK     (3 << PICOUART_CFG_SOURCE_SHIFT)
#  define PICOUART_CFG_SOURCE_WE     (0 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up and event disable */
#  define PICOUART_CFG_SOURCE_WESB   (1 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up or event enable on start bit detection */
#  define PICOUART_CFG_SOURCE_WEFF   (2 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up or event enable on full frame reception */
#  define PICOUART_CFG_SOURCE_WECH   (3 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up or event enable on character recognition */

#define PICOUART_CFG_ACTION          (1 << 0)  /* Bit 0: Action to perform */
#define PICOUART_CFG_MATCH_SHIFT     (8)       /* Bit 8-15: Data Match */
#define PICOUART_CFG_MATCH_SHIFT     (8)       /* Bit 8-15: Data Match */
#define PICOUART_CFG_MATCH_MASK      (0xff << PICOUART_CFG_MATCH_SHIFT)

/* Status Register */

#define PICOUART_SR_EN               (1 << 0)  /* Bit 0:  Enable Status */
#define PICOUART_SR_DRDY             (1 << 1)  /* Bit 1:  Data Ready */

/* Receive Holding Register */

#define PICOUART_RHR_MASK            0xff

/* Version Register */

#define PICOUART_VERSION_SHIFT       (0)       /* Bits 0-11: Macrocell version number */
#define PICOUART_VERSION_MASK        (0xfff << PICOUART_VERSION_SHIFT)
#define PICOUART_VARIANT_SHIFT       (16)      /* Bits 16-18: Reserved */
#define PICOUART_VARIANT_MASK        (7 << PICOUART_VARIANT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PICOUART_H */
