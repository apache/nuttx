/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_i2s.h
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
 *   "Microchip SAMD21 datasheet"
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2S_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S register offsets *****************************************************/

#define SAM_I2S_CTRLA_OFFSET          0x0000  /* Control A register */
#define SAM_I2S_CLKCTRL0_OFFSET       0x0004  /* Clock Control 0 register */
#define SAM_I2S_CLKCTRL1_OFFSET       0x0008  /* Clock Control 1 register */
#define SAM_I2S_INTENCLR_OFFSET       0x000C  /* Interrupt enable clear register */
#define SAM_I2S_INTENSET_OFFSET       0x0010  /* Interrupt enable set register */
#define SAM_I2S_INTFLAG_OFFSET        0x0014  /* Interrupt flag register */
#define SAM_I2S_SYNCBUSY_OFFSET       0x0018  /* Sync Busy register */
#define SAM_I2S_SERCTRL0_OFFSET       0x0020  /* Serializer 0 Control register */
#define SAM_I2S_SERCTRL1_OFFSET       0x0024  /* Serializer 1 Control register */
#define SAM_I2S_DATA0_OFFSET          0x0030  /* Data 0 register */
#define SAM_I2S_DATA1_OFFSET          0x0034  /* Data 1 register */

/* I2S register addresses ***************************************************/

#define SAM_I2S_CTRLA                 (SAM_I2S_BASE+SAM_I2S_CTRLA_OFFSET)
#define SAM_I2S_CLKCTRL0              (SAM_I2S_BASE+SAM_I2S_CLKCTRL0_OFFSET)
#define SAM_I2S_CLKCTRL1              (SAM_I2S_BASE+SAM_I2S_CLKCTRL1_OFFSET)
#define SAM_I2S_INTENCLR              (SAM_I2S_BASE+SAM_I2S_INTENCLR_OFFSET)
#define SAM_I2S_INTENSET              (SAM_I2S_BASE+SAM_I2S_INTENSET_OFFSET)
#define SAM_I2S_INTFLAG               (SAM_I2S_BASE+SAM_I2S_INTFLAG_OFFSET)
#define SAM_I2S_SYNCBUSY              (SAM_I2S_BASE+SAM_I2S_SYNCBUSY_OFFSET)
#define SAM_I2S_SERCTRL0              (SAM_I2S_BASE+SAM_I2S_SERCTRL0_OFFSET)
#define SAM_I2S_SERCTRL1              (SAM_I2S_BASE+SAM_I2S_SERCTRL1_OFFSET)
#define SAM_I2S_DATA0                 (SAM_I2S_BASE+SAM_I2S_DATA0_OFFSET)
#define SAM_I2S_DATA1                 (SAM_I2S_BASE+SAM_I2S_DATA1_OFFSET)

/* I2S register bit definitions *********************************************/

/* Control A register */

#define I2S_CTRLA_SWRST               (1 << 0)  /* Bit 0:  Software Reset */
#define I2S_CTRLA_ENABLE              (1 << 1)  /* Bit 1:  Enable */
#define I2S_CTRLA_CKEN0               (1 << 2)  /* Bit 2:  Clock Unit 0 Enable */
#define I2S_CTRLA_CKEN1               (1 << 3)  /* Bit 3:  Clock Unit 1 Enable */
#define I2S_CTRLA_SEREN0              (1 << 4)  /* Bit 4:  Serializer 0 Enable */
#define I2S_CTRLA_SEREN1              (1 << 5)  /* Bit 5:  Seriailier 1 Enable */

/* Clock Unit Control Register */

#define I2S_CLKCTRL_SLOTSIZE_SHIFT    (0)       /* Bits [1:0]: Slot Size */
#define I2S_CLKCTRL_SLOTSIZE_MASK     (3 << I2S_CLKCTRL_SLOTSIZE_SHIFT)
#  define I2S_CLKCTRL_SLOTSIZE_8      (0 << I2S_CLKCTRL_SLOTSIZE_SHIFT)
#  define I2S_CLKCTRL_SLOTSIZE_16     (1 << I2S_CLKCTRL_SLOTSIZE_SHIFT)
#  define I2S_CLKCTRL_SLOTSIZE_24     (2 << I2S_CLKCTRL_SLOTSIZE_SHIFT)
#  define I2S_CLKCTRL_SLOTSIZE_32     (3 << I2S_CLKCTRL_SLOTSIZE_SHIFT)
#define I2S_CLKCTRL_NBSLOTS_SHIFT     (2)       /* Bit 2: Number of Slots in Frame */
#define I2S_CLKCTRL_NBSLOTS_MASK      (7 << I2S_CLKCTRL_NBSLOTS_SHIFT)
#define I2S_CLKCTRL_NBSLOTS(n)        (((n) & 0x7) << I2S_CLKCTRL_NBSLOTS_SHIFT)
#define I2S_CLKCTRL_FSWIDTH_SHIFT     (5)       /* Bits [6:5]: Frame Sync Width */
#define I2S_CLKCTRL_FSWIDTH_MASK      (3 << I2S_CLKCTRL_FSWIDTH_SHIFT)
#  define I2S_CLKCTRL_FSWIDTH_SLOT    (0 << I2S_CLKCTRL_FSWIDTH_SHIFT)
#  define I2S_CLKCTRL_FSWIDTH_HALF    (1 << I2S_CLKCTRL_FSWIDTH_SHIFT)
#  define I2S_CLKCTRL_FSWIDTH_BIT     (2 << I2S_CLKCTRL_FSWIDTH_SHIFT)
#  define I2S_CLKCTRL_FSWIDTH_BURST   (3 << I2S_CLKCTRL_FSWIDTH_SHIFT)
#define I2S_CLKCTRL_BITDELAY          (1 << 7)  /* Bit 7: Data Delay from Frame Sync */
#define I2S_CLKCTRL_FSSEL             (1 << 8)  /* Bit 8: Frame Sync Select */
#define I2S_CLKCTRL_FSINV             (1 << 11) /* Bit 11: Frame Sync Invert */
#define I2S_CLKCTRL_SCKSEL            (1 << 12) /* Bit 12: Serial Clock Select. 0: Divided Master clock, 1: SCKn input pin */
#define I2S_CLKCTRL_MCKSEL            (1 << 16) /* Bit 16: Master Clock Select. 0: GCLK, 1: MCKn input pin */
#define I2S_CLKCTRL_MCKEN             (1 << 18) /* Bit 18: Master Clock Enable */
#define I2S_CLKCTRL_MCKDIV_SHIFT      (19)      /* Bits [23:19]: Master Clock Division Factor */
#define I2S_CLKCTRL_MCKDIV_MASK       (0x1f << I2S_CLKCTRL_MCKDIV_SHIFT)
#define I2S_CLKCTRL_MCKDIV(n)         (((n) & 0x1f) << I2S_CLKCTRL_MCKDIV_SHIFT)
#define I2S_CLKCTRL_MCKOUTDIV_SHIFT   (24)      /* Bits [28:24]: Master Clock Output Division Factor */
#define I2S_CLKCTRL_MCKOUTDIV_MASK    (0x1f << I2S_CLKCTRL_MCKOUTDIV_SHIFT)
#define I2S_CLKCTRL_MCKOUTDIV(n)      (((n) & 0x1f) << I2S_CLKCTRL_MCKOUTDIV_SHIFT)
#define I2S_CLKCTRL_FSOUTINV          (1 << 29) /* Bit 29: Frame Sync Output Invert */
#define I2S_CLKCTRL_SCKOUTINV         (1 << 30) /* Bit 30: Serial Clock Output Invert */
#define I2S_CLKCTRL_MCKOUTINV         (1 << 31) /* Bit 31: Master Clock Output Invert */

/* Interrupt register bits */

#define I2S_INT_RXRDY0                (1 << 0)  /* Bit 0: Receive Ready 0 */
#define I2S_INT_RXRDY1                (1 << 1)  /* Bit 1: Receive Ready 1 */
#define I2S_INT_RXOR0                 (1 << 4)  /* Bit 4: Receive Overrun 0 */
#define I2S_INT_RXOR1                 (1 << 5)  /* Bit 5: Receive Overrun 1 */
#define I2S_INT_TXRDY0                (1 << 8)  /* Bit 8: Transmit Ready 0 */
#define I2S_INT_TXRDY1                (1 << 9)  /* Bit 9: Transmit Ready 1 */
#define I2S_INT_TXUR0                 (1 << 12) /* Bit 12: Transmit Underrun 0 */
#define I2S_INT_TXUR1                 (1 << 13) /* Bit 13: Transmit Underrun 1 */
#define I2S_INT_ALL                   (0x3333)

/* Sync Busy register bits */

#define I2S_SYNCBUSY_SWRST            (1 << 0)  /* Bit 0: Software Reset Sync Status */
#define I2S_SYNCBUSY_ENABLE           (1 << 1)  /* Bit 1: Enable Sync Status */
#define I2S_SYNCBUSY_CKEN0            (1 << 2)  /* Bit 2: Clock Unit 0 Sync Status */
#define I2S_SYNCBUSY_CKEN1            (1 << 3)  /* Bit 3: Clock Unit 1 Sync Status */
#define I2S_SYNCBUSY_SEREN0           (1 << 4)  /* Bit 4: Serializer 0 Enable Sync Status */
#define I2S_SYNCBUSY_SEREN1           (1 << 5)  /* Bit 5: Seriaiizer 1 Enable Sync Status */
#define I2S_SYNCBUSY_DATA0            (1 << 8)  /* Bit 8: Data 0 Sync Status */
#define I2S_SYNCBUSY_DATA1            (1 << 9)  /* Bit 9: Data 1 Sync Status */

/* Serializer Control register bits */

#define I2S_SERCTRL_SERMODE_SHIFT     (0)       /* Bits [1:0]: Seriailizer Mode */
#define I2S_SERCTRL_SERMODE_MASK      (3 << I2S_SERCTRL_SERMODE_SHIFT)
#  define I2S_SERCTRL_SERMODE_RX      (0 << I2S_SERCTRL_SERMODE_SHIFT)
#  define I2S_SERCTRL_SERMODE_TX      (1 << I2S_SERCTRL_SERMODE_SHIFT)
#  define I2S_SERCTRL_SERMODE_PDM2    (2 << I2S_SERCTRL_SERMODE_SHIFT)
#define I2S_SERCTRL_TXDEFAULT_SHIFT   (2)       /* Bits [3:2]: Line Default when Slot Disabled */
#define I2S_SERCTRL_TXDEFAULT_MASK    (3 << I2S_SERCTRL_TXDEFAULT_SHIFT)
#  define I2S_SERCTRL_TXDEFAULT_ZERO  (0 << I2S_SERCTRL_TXDEFAULT_SHIFT)
#  define I2S_SERCTRL_TXDEFAULT_ONE   (1 << I2S_SERCTRL_TXDEFAULT_SHIFT)
#  define I2S_SERCTRL_TXDEFAULT_HIZ   (3 << I2S_SERCTRL_TXDEFAULT_SHIFT)
#define I2S_SERCTRL_TXSAME            (1 << 4)  /* Bit 4: Transmit last data when Underrun */
#define I2S_SERCTRL_CLKSEL            (1 << 5)  /* Bit 5: Clock Unit Selection. 0: Use Clock 0, 1: Use Clock 1 */
#define I2S_SERCTRL_SLOTADJ           (1 << 7)  /* Bit 7: Data slot formatting. 0: Right justified, 1: Left justified */
#define I2S_SERCTRL_DATASIZE_SHIFT    (8)       /* Bits [10:8]: Data Word Size */
#define I2S_SERCTRL_DATASIZE_MASK     (7 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_32     (0 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_24     (1 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_20     (2 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_18     (3 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_16     (4 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_16C    (5 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_8      (6 << I2S_SERCTRL_DATASIZE_SHIFT)
#  define I2S_SERCTRL_DATASIZE_8C     (7 << I2S_SERCTRL_DATASIZE_SHIFT)
#define I2S_SERCTRL_WORDADJ           (1 << 12) /* Bit 12: Data word formatting. 0: Right justified, 1: Left justified */
#define I2S_SERCTRL_EXTEND_SHIFT      (13)      /* Bits [14:13]: Data formatting bit extension */
#define I2S_SERCTRL_EXTEND_MASK       (3 << I2S_SERCTRL_EXTEND_SHIFT)
#  define I2S_SERCTRL_EXTEND_ZERO     (0 << I2S_SERCTRL_EXTEND_SHIFT)
#  define I2S_SERCTRL_EXTEND_ONE      (1 << I2S_SERCTRL_EXTEND_SHIFT)
#  define I2S_SERCTRL_EXTEND_MSBIT    (2 << I2S_SERCTRL_EXTEND_SHIFT)
#  define I2S_SERCTRL_EXTEND_LSBIT    (3 << I2S_SERCTRL_EXTEND_SHIFT)
#define I2S_SERCTRL_BITREV            (1 << 15) /* Bit 15: Data formatting bit reverse */
#define I2S_SERCTRL_SLOTDIS_SHIFT     (16)      /* Bits [23:16]: Slot x Disabled */
#define I2S_SERCTRL_SLOTDIS_MASK      (0xff << I2S_SERCTRL_SLOTDIS_SHIFT)
#define I2S_SERCTRL_SLOTDIS(n)        (((n) & 0xff) << I2C_SERCTRL_SLOTDIS_SHIFT)
#define I2S_SERCTRL_MONO              (1 << 24) /* Bit 24: Mono Mode */
#define I2S_SERCTRL_DMA               (1 << 25) /* Bit 25: Single or Multiple DMA channels */
#define I2S_SERCTRL_RXLOOP            (1 << 26) /* Bit 26: RX Loopback Test Mode */

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
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2S_H */
