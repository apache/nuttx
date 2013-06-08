/****************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_picouart.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_PICOUART_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_PICOUART_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PICOUART register offsets ************************************************************/

#define SAM_PICOUART_CR_OFFSET       0x0000 /* Control Register */
#define SAM_PICOUART_CFG_OFFSET      0x0004 /* Configuration Register */
#define SAM_PICOUART_SR_OFFSET       0x0008 /* Status Register */
#define SAM_PICOUART_RHR_OFFSET      0x000c /* Receive Holding Register */
#define SAM_PICOUART_VERSION_OFFSET  0x0020 /* Version Register */

/* PICOUART register adresses ***********************************************************/

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

/* PICOUART register bit definitions ****************************************************/

/* Control Register */

#define PICOUART_CR_EN               (1 << 0)  /* Bit 0:  Enable */
#define PICOUART_CR_DIS              (1 << 1)  /* Bit 1:  Disable */

/* Configuration Register */

#define PICOUART_CFG_SOURCE_SHIFT    (0)       /* Bit 0-1: Source Enable Mode */
#define PICOUART_CFG_SOURCE_MASK     (3 << PICOUART_CFG_SOURCE_SHIFT)
# define PICOUART_CFG_SOURCE_WE      (0 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up and event disable */
# define PICOUART_CFG_SOURCE_WESB    (1 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up or event enable on start bit detection */
# define PICOUART_CFG_SOURCE_WEFF    (2 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up or event enable on full frame reception */
# define PICOUART_CFG_SOURCE_WECH    (3 << PICOUART_CFG_SOURCE_SHIFT) /* Wake up or event enable on character recognition */
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

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_PICOUART_H */
