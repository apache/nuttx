/****************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32_dma2d.h
 *
 *   Copyright (C) 2014-2015 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_DMA2D_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_DMA2D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_DMA2D_NCLUT           256    /* Number of entries in the CLUT */

/* DMA2D Register Offsets ****************************************************/

#define STM32_DMA2D_CR_OFFSET       0x0000 /* DMA2D Control Register */
#define STM32_DMA2D_ISR_OFFSET      0x0004 /* DMA2D Interrupt Status Register */
#define STM32_DMA2D_IFCR_OFFSET     0x0008 /* DMA2D Interrupt Flag Clear Register */
#define STM32_DMA2D_FGMAR_OFFSET    0x000C /* DMA2D Foreground Memory Address Register */
#define STM32_DMA2D_FGOR_OFFSET     0x0010 /* DMA2D Foreground Offset Register */
#define STM32_DMA2D_BGMAR_OFFSET    0x0014 /* DMA2D Background Memory Address Register */
#define STM32_DMA2D_BGOR_OFFSET     0x0018 /* DMA2D Background Offset Register */
#define STM32_DMA2D_FGPFCCR_OFFSET  0x001C /* DMA2D Foreground PFC Control Register */
#define STM32_DMA2D_FGCOLR_OFFSET   0x0020 /* DMA2D Foreground Color Register */
#define STM32_DMA2D_BGPFCCR_OFFSET  0x0024 /* DMA2D Background PFC Control Register */
#define STM32_DMA2D_BGCOLR_OFFSET   0x0028 /* DMA2D Background Color Register */
#define STM32_DMA2D_FGCMAR_OFFSET   0x002C /* DMA2D Foreground CLUT Memory Address Register */
#define STM32_DMA2D_BGCMAR_OFFSET   0x0030 /* DMA2D Background CLUT Memory Address Register */
#define STM32_DMA2D_OPFCCR_OFFSET   0x0034 /* DMA2D Output PFC Control Register */
#define STM32_DMA2D_OCOLR_OFFSET    0x0038 /* DMA2D Output Color Register */
#define STM32_DMA2D_OMAR_OFFSET     0x003C /* DMA2D Output Memory Address Register */
#define STM32_DMA2D_OOR_OFFSET      0x0040 /* DMA2D Output Offset Register */
#define STM32_DMA2D_NLR_OFFSET      0x0044 /* DMA2D Number Of Line Register */
#define STM32_DMA2D_LWR_OFFSET      0x0048 /* DMA2D Line Watermark Register */
#define STM32_DMA2D_AMTCR_OFFSET    0x004c /* DMA2D AHB Master Time Configuration Register */

/* DMA2D Register Addresses **************************************************/

#define STM32_DMA2D_CR              (STM32_DMA2D_BASE + STM32_DMA2D_CR_OFFSET)
#define STM32_DMA2D_ISR             (STM32_DMA2D_BASE + STM32_DMA2D_ISR_OFFSET)
#define STM32_DMA2D_IFCR            (STM32_DMA2D_BASE + STM32_DMA2D_IFCR_OFFSET)
#define STM32_DMA2D_FGMAR           (STM32_DMA2D_BASE + STM32_DMA2D_FGMAR_OFFSET)
#define STM32_DMA2D_FGOR            (STM32_DMA2D_BASE + STM32_DMA2D_FGOR_OFFSET)
#define STM32_DMA2D_BGMAR           (STM32_DMA2D_BASE + STM32_DMA2D_BGMAR_OFFSET)
#define STM32_DMA2D_BGOR            (STM32_DMA2D_BASE + STM32_DMA2D_BGOR_OFFSET)
#define STM32_DMA2D_FGPFCCR         (STM32_DMA2D_BASE + STM32_DMA2D_FGPFCCR_OFFSET)
#define STM32_DMA2D_FGCOLR          (STM32_DMA2D_BASE + STM32_DMA2D_FGCOLR_OFFSET)
#define STM32_DMA2D_BGPFCCR         (STM32_DMA2D_BASE + STM32_DMA2D_BGPFCCR_OFFSET)
#define STM32_DMA2D_BGCOLR          (STM32_DMA2D_BASE + STM32_DMA2D_BGCOLR_OFFSET)
#define STM32_DMA2D_FGCMAR          (STM32_DMA2D_BASE + STM32_DMA2D_FGCMAR_OFFSET)
#define STM32_DMA2D_BGCMAR          (STM32_DMA2D_BASE + STM32_DMA2D_BGCMAR_OFFSET)
#define STM32_DMA2D_OPFCCR          (STM32_DMA2D_BASE + STM32_DMA2D_OPFCCR_OFFSET)
#define STM32_DMA2D_OCOLR           (STM32_DMA2D_BASE + STM32_DMA2D_OCOLR_OFFSET)
#define STM32_DMA2D_OMAR            (STM32_DMA2D_BASE + STM32_DMA2D_OMAR_OFFSET)
#define STM32_DMA2D_OOR             (STM32_DMA2D_BASE + STM32_DMA2D_OOR_OFFSET)
#define STM32_DMA2D_NLR             (STM32_DMA2D_BASE + STM32_DMA2D_NLR_OFFSET)
#define STM32_DMA2D_LWR             (STM32_DMA2D_BASE + STM32_DMA2D_LWR_OFFSET)

/* DMA2D Register Bit Definitions ********************************************/

/* DMA2D Control Register */

#define DMA2D_CR_START              (1 << 0)  /* Start Bit */
#define DMA2D_CR_SUSP               (1 << 1)  /* Suspend Bit */
#define DMA2D_CR_ABORT              (1 << 2)  /* Abort Bit */
#define DMA2D_CR_TEIE               (1 << 8)  /* Transfer Error Interrupt Enable Bit */
#define DMA2D_CR_TCIE               (1 << 9)  /* Transfer Complete Interrupt Enable Bit */
#define DMA2D_CR_TWIE               (1 << 10) /* Transfer Watermark Interrupt Enable Bit */
#define DMA2D_CR_CAEIE              (1 << 11) /* CLUT Access Error Interrupt Enable Bit */
#define DMA2D_CR_CTCIE              (1 << 12) /* CLUT Transfer Complete Interrupt Enable Bit */
#define DMA2D_CR_CEIE               (1 << 13) /* Configuration Error Interrupt Enable Bit */
#define DMA2D_CR_MODE_SHIFT         (16) /* Bits 16-17 DMA2D mode Bits */
#define DMA2D_CR_MODE_MASK          (3 << DMA2D_CR_MODE_SHIFT)
#define DMA2D_CR_MODE(n)            ((uint32_t)(n) << DMA2D_CR_MODE_SHIFT)

/* DMA2D Interrupt Status Register */

#define DMA2D_ISR_TEIF              (1 << 0)  /* Transfer error interrupt flag */
#define DMA2D_ISR_TCIF              (1 << 1)  /* Transfer Complete Interrupt flag */
#define DMA2D_ISR_TWIF              (1 << 2)  /* Transfer Watermark Interrupt flag */
#define DMA2D_ISR_CAEIF             (1 << 3)  /* CLUT Access Error Interrupt flag */
#define DMA2D_ISR_CTCIF             (1 << 4)  /* CLUT Transfer Complete Interrupt flag */
#define DMA2D_ISR_CEIF              (1 << 5)  /* Configuration Error Interrupt flag */

/* DMA2D Interrupt Flag Clear Register */

#define DMA2D_IFCR_CTEIF            (1 << 0)  /* Clear Transfer Interrupt Flag */
#define DMA2D_IFCR_CTCIF            (1 << 1)  /* Clear Transfer Complete Interrupt Flag */
#define DMA2D_IFCR_CTWIF            (1 << 2)  /* Clear Transfer Watermark Interrupt Flag */
#define DMA2D_IFCR_CAECIF           (1 << 3)  /* Clear CLUT Access Error Interrupt Flag */
#define DMA2D_IFCR_CCTCIF           (1 << 4)  /* Clear CLUT Transfer Complete Interrupt Flag */
#define DMA2D_IFCR_CCEIF            (1 << 5)  /* Clear Configuration Error Interrupt Flag */

/* DMA2D Foreground Memory Access Register */

/* DMA2D Background Memory Access Register */

/* DMA2D Foreground/Background Offset Register */

#define DMA2D_xGOR_SHIFT            (0)  /* Bits 0-13 Line Offset */
#define DMA2D_xGOR_MASK             (0x3FFF << DMA2D_xGOR_SHIFT)
#define DMA2D_xGOR(n)               ((uint32_t)(n) << DMA2D_xGOR_SHIFT)

/* DMA2D Foreground/Background PFC Control Register */

#define DMA2D_xGPFCCR_CM_SHIFT      (0)  /* Bits 0-3 Color Mode */
#define DMA2D_xGPFCCR_CM_MASK       (0xF << DMA2D_xGPFCCR_CM_SHIFT)
#define DMA2D_xGPFCCR_CM(n)         ((uint32_t)(n) << DMA2D_xGPFCCR_CM_SHIFT)
#define DMA2D_xGPFCCR_CCM           (1 << 4)  /* CLUT Color Mode */
#define DMA2D_xGPFCCR_START         (1 << 5)  /* Start */
#define DMA2D_xGPFCCR_CS_SHIFT      (8)  /* Bits 8-15 CLUT Size */
#define DMA2D_xGPFCCR_CS_MASK       (0xFF << DMA2D_xGPFCCR_CS_SHIFT)
#define DMA2D_xGPFCCR_CS(n)         ((uint32_t)(n) << DMA2D_xGPFCCR_CS_SHIFT)
#define DMA2D_xGPFCCR_AM_SHIFT      (16)  /* Bits 16-17 Alpha Mode */
#define DMA2D_xGPFCCR_AM_MASK       (3 << DMA2D_xGPFCCR_AM_SHIFT)
#define DMA2D_xGPFCCR_AM(n)         ((uint32_t)(n) << DMA2D_xGPFCCR_AM_SHIFT)
#define DMA2D_xGPFCCR_ALPHA_SHIFT   (24)  /* Bits 24-31 Alpha Value */
#define DMA2D_xGPFCCR_ALPHA_MASK    (0xFF << DMA2D_xGPFCCR_ALPHA_SHIFT)
#define DMA2D_xGPFCCR_ALPHA(n)      ((uint32_t)(n) << DMA2D_xGPFCCR_ALPHA_SHIFT)

/* DMA2D PFC alpha mode */

#define STM32_DMA2D_PFCCR_AM_NONE   0
#define STM32_DMA2D_PFCCR_AM_CONST  1
#define STM32_DMA2D_PFCCR_AM_PIXEL  2

/* DMA2D Foreground/Background Color Register */

#define DMA2D_xGCOLR_BLUE_SHIFT     (0)  /* Bits 0-7 Blue Value */
#define DMA2D_xGCOLR_BLUE_MASK      (0xFF << DMA2D_xGCOLR_BLUE_SHIFT)
#define DMA2D_xGCOLR_BLUE(n)        ((uint32_t)(n) << DMA2D_xGCOLR_BLUE_SHIFT)
#define DMA2D_xGCOLR_GREEN_SHIFT    (8)  /* Bits 8-15 Green Value */
#define DMA2D_xGCOLR_GREEN_MASK     (0xFF << DMA2D_xGCOLR_GREEN_SHIFT)
#define DMA2D_xGCOLR_GREEN(n)       ((uint32_t)(n) << DMA2D_xGCOLR_GREEN_SHIFT)
#define DMA2D_xGCOLR_RED_SHIFT      (16)  /* Bits 16-23 Red Value */
#define DMA2D_xGCOLR_RED_MASK       (0xFF << DMA2D_xGCOLR_RED_SHIFT)
#define DMA2D_xGCOLR_RED(n)         ((uint32_t)(n) << DMA2D_xGCOLR_RED_SHIFT)

/* DMA2D Foreground CLUT Memory Address Register */

/* DMA2D Background CLUT Memory Address Register */

/* DMA2D Output PFC Control Register */

#define DMA2D_OPFCCR_CM_SHIFT       (0)  /* Bits 0-2 Color Mode */
#define DMA2D_OPFCCR_CM_MASK        (7 << DMA2D_OPFCCR_CM_SHIFT)
#define DMA2D_OPFCCR_CM(n)          ((uint32_t)(n) << DMA2D_OPFCCR_CM_SHIFT)

/* DMA2D PFC Pixel Format */

#define DMA2D_PF_ARGB8888           0
#define DMA2D_PF_RGB888             1
#define DMA2D_PF_RGB565             2
#define DMA2D_PF_ARGB1555           3
#define DMA2D_PF_ARGB14444          4
#define DMA2D_PF_L8                 5
#define DMA2D_PF_AL44               6
#define DMA2D_PF_AL88               7
#define DMA2D_PF_L4                 8
#define DMA2D_PF_A8                 9
#define DMA2D_PF_A4                 10

/* DMA2D Output Color Register */

#define DMA2D_OCOLR_BLUE_SHIFT      (0)  /* Bits 0-7 Blue Value */
#define DMA2D_OCOLR_BLUE_MASK       (0xFF << DMA2D_OCOLR_BLUE_SHIFT)
#define DMA2D_OCOLR_BLUE(n)         ((uint32_t)(n) << DMA2D_OCOLR_BLUE_SHIFT)
#define DMA2D_OCOLR_GREEN_SHIFT     (8)  /* Bits 8-15 Green Value */
#define DMA2D_OCOLR_GREEN_MASK      (0xFF << DMA2D_OCOLR_GREEN_SHIFT)
#define DMA2D_OCOLR_GREEN(n)        ((uint32_t)(n) << DMA2D_OCOLR_GREEN_SHIFT)
#define DMA2D_OCOLR_RED_SHIFT       (16)  /* Bits 16-23 Red Value */
#define DMA2D_OCOLR_RED_MASK        (0xFF << DMA2D_OCOLR_RED_SHIFT)
#define DMA2D_OCOLR_RED(n)          ((uint32_t)(n) << DMA2D_OCOLR_RED_SHIFT)
#define DMA2D_OCOLR_ALPHA_SHIFT     (24)  /* Bits 24-31 Alpha Value */
#define DMA2D_OCOLR_ALPHA_MASK      (0xFF << DMA2D_OCOLR_ALPHA_SHIFT)
#define DMA2D_OCOLR_ALPHA(n)        ((uint32_t)(n) << DMA2D_OCOLR_ALPHA_SHIFT)

/* DMA2D Output Memory Address Register */

/* DMA2D Output Offset Register */

#define DMA2D_OOR_LO_SHIFT          (0)  /* Bits 0-13 Line Offset */
#define DMA2D_OOR_LO_MASK           (0x3FFF << DMA2D_OOR_LO_SHIFT)
#define DMA2D_OOR_LO(n)             ((uint32_t)(n) << DMA2D_OOR_LO_SHIFT)

/* DMA2D Number Of Line Register */

#define DMA2D_NLR_NL_SHIFT          (0)  /* Bits 0-15 Number Of Lines */
#define DMA2D_NLR_NL_MASK           (0xFFFF << DMA2D_NLR_NL_SHIFT)
#define DMA2D_NLR_NL(n)             ((uint32_t)(n) << DMA2D_NLR_NL_SHIFT)
#define DMA2D_NLR_PL_SHIFT          (16) /* Bits 16-29 Pixel per Lines */
#define DMA2D_NLR_PL_MASK           (0x3FFF << DMA2D_NLR_PL_SHIFT)
#define DMA2D_NLR_PL(n)             ((uint32_t)(n) << DMA2D_NLR_PL_SHIFT)

/* DMA2D Line Watermark Register */

#define DMA2D_LWR_LW_SHIFT          (0)  /* Bits 0-15 Line Watermark */
#define DMA2D_LWR_LW_MASK           (0xFFFF << DMA2D_LWR_LW_SHIFT)
#define DMA2D_LWR_LW(n)             ((uint32_t)(n) << DMA2D_LWR_LW_SHIFT)

/* DMA2D AHB Master Timer Configuration Register */

#define DMA2D_AMTCR_EN              (1 << 0)  /* Enable */
#define DMA2D_AMTCR_DT_SHIFT        (0)  /* Bits 8-15 Dead Time */
#define DMA2D_AMTCR_DT_MASK         (0xFF << DMA2D_AMTCR_DT_SHIFT)
#define DMA2D_AMTCR_DT(n)           ((uint32_t)(n) << DMA2D_AMTCR_DT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_DMA2D_H */
