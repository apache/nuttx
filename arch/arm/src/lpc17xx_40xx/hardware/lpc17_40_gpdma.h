/************************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_gpdma.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_GPDMA_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_GPDMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register offsets *****************************************************************/

/* General registers (see also LPC17_40_SYSCON_DMAREQSEL_OFFSET in lpc17_40_syscon.h) */

#define LPC17_40_DMA_INTST_OFFSET         0x0000 /* DMA Interrupt Status Register */
#define LPC17_40_DMA_INTTCST_OFFSET       0x0004 /* DMA Interrupt Terminal Count Request Status Register */
#define LPC17_40_DMA_INTTCCLR_OFFSET      0x0008 /* DMA Interrupt Terminal Count Request Clear Register */
#define LPC17_40_DMA_INTERRST_OFFSET      0x000c /* DMA Interrupt Error Status Register */
#define LPC17_40_DMA_INTERRCLR_OFFSET     0x0010 /* DMA Interrupt Error Clear Register */
#define LPC17_40_DMA_RAWINTTCST_OFFSET    0x0014 /* DMA Raw Interrupt Terminal Count Status Register */
#define LPC17_40_DMA_RAWINTERRST_OFFSET   0x0018 /* DMA Raw Error Interrupt Status Register */
#define LPC17_40_DMA_ENBLDCHNS_OFFSET     0x001c /* DMA Enabled Channel Register */
#define LPC17_40_DMA_SOFTBREQ_OFFSET      0x0020 /* DMA Software Burst Request Register */
#define LPC17_40_DMA_SOFTSREQ_OFFSET      0x0024 /* DMA Software Single Request Register */
#define LPC17_40_DMA_SOFTLBREQ_OFFSET     0x0028 /* DMA Software Last Burst Request Register */
#define LPC17_40_DMA_SOFTLSREQ_OFFSET     0x002c /* DMA Software Last Single Request Register */
#define LPC17_40_DMA_CONFIG_OFFSET        0x0030 /* DMA Configuration Register */
#define LPC17_40_DMA_SYNC_OFFSET          0x0034 /* DMA Synchronization Register */

/* Channel Registers */

#define LPC17_40_NDMACH                   8      /* Eight DMA channels */
#define LPC17_40_DMA_CHAN_OFFSET(n)       (0x0100 + ((n) << 5)) /* n=0,1,...,(LPC17_40_NDMACH-1) */

#define LPC17_40_DMACH_SRCADDR_OFFSET     0x0000 /* DMA Channel Source Address Register */
#define LPC17_40_DMACH_DESTADDR_OFFSET    0x0004 /* DMA Channel Destination Address Register */
#define LPC17_40_DMACH_LLI_OFFSET         0x0008 /* DMA Channel Linked List Item Register */
#define LPC17_40_DMACH_CONTROL_OFFSET     0x000c /* DMA Channel Control Register */
#define LPC17_40_DMACH_CONFIG_OFFSET      0x0010 /* DMA Channel Configuration Register */

#define LPC17_40_DMACH0_SRCADDR_OFFSET    (0x100+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH0_DESTADDR_OFFSET   (0x100+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH0_LLI_OFFSET        (0x100+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH0_CONTROL_OFFSET    (0x100+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH0_CONFIG_OFFSET     (0x100+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH1_SRCADDR_OFFSET    (0x120+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH1_DESTADDR_OFFSET   (0x120+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH1_LLI_OFFSET        (0x120+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH1_CONTROL_OFFSET    (0x120+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH1_CONFIG_OFFSET     (0x120+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH2_SRCADDR_OFFSET    (0x140+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH2_DESTADDR_OFFSET   (0x140+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH2_LLI_OFFSET        (0x140+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH2_CONTROL_OFFSET    (0x140+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH2_CONFIG_OFFSET     (0x140+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH3_SRCADDR_OFFSET    (0x160+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH3_DESTADDR_OFFSET   (0x160+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH3_LLI_OFFSET        (0x160+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH3_CONTROL_OFFSET    (0x160+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH3_CONFIG_OFFSET     (0x160+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH4_SRCADDR_OFFSET    (0x180+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH4_DESTADDR_OFFSET   (0x180+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH4_LLI_OFFSET        (0x180+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH4_CONTROL_OFFSET    (0x180+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH4_CONFIG_OFFSET     (0x180+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH5_SRCADDR_OFFSET    (0x1a0+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH5_DESTADDR_OFFSET   (0x1a0+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH5_LLI_OFFSET        (0x1a0+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH5_CONTROL_OFFSET    (0x1a0+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH5_CONFIG_OFFSET     (0x1a0+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH6_SRCADDR_OFFSET    (0x1c0+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH6_DESTADDR_OFFSET   (0x1c0+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH6_LLI_OFFSET        (0x1c0+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH6_CONTROL_OFFSET    (0x1c0+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH6_CONFIG_OFFSET     (0x1c0+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH7_SRCADDR_OFFSET    (0x1e0+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH7_DESTADDR_OFFSET   (0x1e0+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH7_LLI_OFFSET        (0x1e0+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH7_CONTROL_OFFSET    (0x1e0+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH7_CONFIG_OFFSET     (0x1e0+LPC17_40_DMACH_CONFIG_OFFSET)

/* Register addresses ***************************************************************/
/* General registers (see also LPC17_40_SYSCON_DMAREQSEL in lpc17_40_syscon.h) */

#define LPC17_40_DMA_INTST                (LPC17_40_GPDMA_BASE+LPC17_40_DMA_INTST_OFFSET)
#define LPC17_40_DMA_INTTCST              (LPC17_40_GPDMA_BASE+LPC17_40_DMA_INTTCST_OFFSET)
#define LPC17_40_DMA_INTTCCLR             (LPC17_40_GPDMA_BASE+LPC17_40_DMA_INTTCCLR_OFFSET)
#define LPC17_40_DMA_INTERRST             (LPC17_40_GPDMA_BASE+LPC17_40_DMA_INTERRST_OFFSET)
#define LPC17_40_DMA_INTERRCLR            (LPC17_40_GPDMA_BASE+LPC17_40_DMA_INTERRCLR_OFFSET)
#define LPC17_40_DMA_RAWINTTCST           (LPC17_40_GPDMA_BASE+LPC17_40_DMA_RAWINTTCST_OFFSET)
#define LPC17_40_DMA_RAWINTERRST          (LPC17_40_GPDMA_BASE+LPC17_40_DMA_RAWINTERRST_OFFSET)
#define LPC17_40_DMA_ENBLDCHNS            (LPC17_40_GPDMA_BASE+LPC17_40_DMA_ENBLDCHNS_OFFSET)
#define LPC17_40_DMA_SOFTBREQ             (LPC17_40_GPDMA_BASE+LPC17_40_DMA_SOFTBREQ_OFFSET)
#define LPC17_40_DMA_SOFTSREQ             (LPC17_40_GPDMA_BASE+LPC17_40_DMA_SOFTSREQ_OFFSET)
#define LPC17_40_DMA_SOFTLBREQ            (LPC17_40_GPDMA_BASE+LPC17_40_DMA_SOFTLBREQ_OFFSET)
#define LPC17_40_DMA_SOFTLSREQ            (LPC17_40_GPDMA_BASE+LPC17_40_DMA_SOFTLSREQ_OFFSET)
#define LPC17_40_DMA_CONFIG               (LPC17_40_GPDMA_BASE+LPC17_40_DMA_CONFIG_OFFSET)
#define LPC17_40_DMA_SYNC                 (LPC17_40_GPDMA_BASE+LPC17_40_DMA_SYNC_OFFSET)

/* Channel Registers */

#define LPC17_40_DMACH_BASE(n)            (LPC17_40_GPDMA_BASE+LPC17_40_DMA_CHAN_OFFSET(n))

#define LPC17_40_DMACH_SRCADDR(n)         (LPC17_40_DMACH_BASE(n)+LPC17_40_DMACH_SRCADDR_OFFSET)
#define LPC17_40_DMACH_DESTADDR(n)        (LPC17_40_DMACH_BASE(n)+LPC17_40_DMACH_DESTADDR_OFFSET)
#define LPC17_40_DMACH_LLI(n)             (LPC17_40_DMACH_BASE(n)+LPC17_40_DMACH_LLI_OFFSET)
#define LPC17_40_DMACH_CONTROL(n)         (LPC17_40_DMACH_BASE(n)+LPC17_40_DMACH_CONTROL_OFFSET)
#define LPC17_40_DMACH_CONFIG(n)          (LPC17_40_DMACH_BASE(n)+LPC17_40_DMACH_CONFIG_OFFSET)

#define LPC17_40_DMACH0_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH0_SRCADDR_OFFSET)
#define LPC17_40_DMACH0_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH0_DESTADDR_OFFSET)
#define LPC17_40_DMACH0_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH0_LLI_OFFSET)
#define LPC17_40_DMACH0_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH0_CONTROL_OFFSET)
#define LPC17_40_DMACH0_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH0_CONFIG_OFFSET)

#define LPC17_40_DMACH1_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH1_SRCADDR_OFFSET)
#define LPC17_40_DMACH1_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH1_DESTADDR_OFFSET)
#define LPC17_40_DMACH1_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH1_LLI_OFFSET)
#define LPC17_40_DMACH1_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH1_CONTROL_OFFSET)
#define LPC17_40_DMACH1_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH1_CONFIG_OFFSET)

#define LPC17_40_DMACH2_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH2_SRCADDR_OFFSET)
#define LPC17_40_DMACH2_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH2_DESTADDR_OFFSET)
#define LPC17_40_DMACH2_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH2_LLI_OFFSET)
#define LPC17_40_DMACH2_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH2_CONTROL_OFFSET)
#define LPC17_40_DMACH2_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH2_CONFIG_OFFSET)

#define LPC17_40_DMACH3_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH3_SRCADDR_OFFSET)
#define LPC17_40_DMACH3_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH3_DESTADDR_OFFSET)
#define LPC17_40_DMACH3_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH3_LLI_OFFSET)
#define LPC17_40_DMACH3_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH3_CONTROL_OFFSET)
#define LPC17_40_DMACH3_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH3_CONFIG_OFFSET)

#define LPC17_40_DMACH4_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH4_SRCADDR_OFFSET)
#define LPC17_40_DMACH4_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH4_DESTADDR_OFFSET)
#define LPC17_40_DMACH4_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH4_LLI_OFFSET)
#define LPC17_40_DMACH4_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH4_CONTROL_OFFSET)
#define LPC17_40_DMACH4_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH4_CONFIG_OFFSET)

#define LPC17_40_DMACH5_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH5_SRCADDR_OFFSET)
#define LPC17_40_DMACH5_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH5_DESTADDR_OFFSET)
#define LPC17_40_DMACH5_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH5_LLI_OFFSET)
#define LPC17_40_DMACH5_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH5_CONTROL_OFFSET)
#define LPC17_40_DMACH5_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH5_CONFIG_OFFSET)

#define LPC17_40_DMACH6_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH6_SRCADDR_OFFSET)
#define LPC17_40_DMACH6_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH6_DESTADDR_OFFSET)
#define LPC17_40_DMACH6_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH6_LLI_OFFSET)
#define LPC17_40_DMACH6_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH6_CONTROL_OFFSET)
#define LPC17_40_DMACH6_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH6_CONFIG_OFFSET)

#define LPC17_40_DMACH7_SRCADDR           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH7_SRCADDR_OFFSET)
#define LPC17_40_DMACH7_DESTADDR          (LPC17_40_GPDMA_BASE+LPC17_40_DMACH7_DESTADDR_OFFSET)
#define LPC17_40_DMACH7_LLI               (LPC17_40_GPDMA_BASE+LPC17_40_DMACH7_LLI_OFFSET)
#define LPC17_40_DMACH7_CONTROL           (LPC17_40_GPDMA_BASE+LPC17_40_DMACH7_CONTROL_OFFSET)
#define LPC17_40_DMACH7_CONFIG            (LPC17_40_GPDMA_BASE+LPC17_40_DMACH7_CONFIG_OFFSET)

/* Register bit definitions *********************************************************/
/* DMA Request Connections **********************************************************/

#define LPC17_40_NDMAREQ                  (16) /* The number of DMA requests */
#if defined(LPC176x)
/* Request Numbers */

#  define DMA_REQ_SSP0TX                  (0)
#  define DMA_REQ_SSP0RX                  (1)
#  define DMA_REQ_SSP1TX                  (2)
#  define DMA_REQ_SSP1RX                  (3)

#  define DMA_REQ_ADC                     (4)

#  define DMA_REQ_I2SCH0                  (5)
#  define DMA_REQ_I2SCH1                  (6)

#  define DMA_REQ_DAC                     (7)

#  define DMA_REQ_UART0TX                 (8)  /* DMASEL08=0*/
#  define DMA_REQ_UART0RX                 (9)  /* DMASEL09=0*/
#  define DMA_REQ_UART1TX                 (10) /* DMASEL010=0*/
#  define DMA_REQ_UART1RX                 (11) /* DMASEL011=0*/
#  define DMA_REQ_UART2TX                 (12) /* DMASEL012=0*/
#  define DMA_REQ_UART2RX                 (13) /* DMASEL013=0*/
#  define DMA_REQ_UART3TX                 (14) /* DMASEL014=0*/
#  define DMA_REQ_UART3RX                 (15) /* DMASEL015=0*/

#  define DMA_REQ_MAT0p0                  (8)  /* DMASEL08=1 */
#  define DMA_REQ_MAT0p1                  (9)  /* DMASEL09=1 */
#  define DMA_REQ_MAT1p0                  (10) /* DMASEL010=1 */
#  define DMA_REQ_MAT1p1                  (11) /* DMASEL011=1 */
#  define DMA_REQ_MAT2p0                  (12) /* DMASEL012=1 */
#  define DMA_REQ_MAT2p1                  (13) /* DMASEL013=1 */
#  define DMA_REQ_MAT3p0                  (14) /* DMASEL014=1 */
#  define DMA_REQ_MAT3p1                  (15) /* DMASEL015=1 */

/* DMASEL values.  For the LPC176x family, only request numbers 8-15 have
 * DMASEL bits.
 */

#  define DMA_DMASEL_SSP0TX               (0)  /* Not applicable */
#  define DMA_DMASEL_SSP0RX               (0)  /* Not applicable */
#  define DMA_DMASEL_SSP1TX               (0)  /* Not applicable */
#  define DMA_DMASEL_SSP1RX               (0)  /* Not applicable */

#  define DMA_DMASEL_ADC                  (0)  /* Not applicable */

#  define DMA_DMASEL_I2SCH0               (0)  /* Not applicable */
#  define DMA_DMASEL_I2SCH1               (0)  /* Not applicable */

#  define DMA_DMASEL_DAC                  (0)  /* Not applicable */

#  define DMA_DMASEL_UART0TX              (0)
#  define DMA_DMASEL_UART0RX              (0)
#  define DMA_DMASEL_UART1TX              (0)
#  define DMA_DMASEL_UART1RX              (0)
#  define DMA_DMASEL_UART2TX              (0)
#  define DMA_DMASEL_UART2RX              (0)
#  define DMA_DMASEL_UART3TX              (0)
#  define DMA_DMASEL_UART3RX              (0)

#  define DMA_DMASEL_MAT0p0               (1)
#  define DMA_DMASEL_MAT0p1               (1)
#  define DMA_DMASEL_MAT1p0               (1)
#  define DMA_DMASEL_MAT1p1               (1)
#  define DMA_DMASEL_MAT2p0               (1)
#  define DMA_DMASEL_MAT2p1               (1)
#  define DMA_DMASEL_MAT3p0               (1)
#  define DMA_DMASEL_MAT3p1               (1)

#elif defined(LPC178x_40xx)
/* Request Numbers */

#  define DMA_REQ_SDCARD                  (1)  /* DMASEL01=0 */

#  define DMA_REQ_SSP0TX                  (2)  /* DMASEL02=0 */
#  define DMA_REQ_SSP0RX                  (3)  /* DMASEL03=0 */
#  define DMA_REQ_SSP1TX                  (4)  /* DMASEL04=0 */
#  define DMA_REQ_SSP1RX                  (5)  /* DMASEL05=0 */
#  define DMA_REQ_SSP2TX                  (6)  /* DMASEL06=0 */
#  define DMA_REQ_SSP2RX                  (7)  /* DMASEL07=0 */

#  define DMA_REQ_MAT0p0                  (0)  /* DMASEL00=1 */
#  define DMA_REQ_MAT0p1                  (1)  /* DMASEL01=1 */
#  define DMA_REQ_MAT1p0                  (2)  /* DMASEL02=1 */
#  define DMA_REQ_MAT1p1                  (3)  /* DMASEL03=1 */
#  define DMA_REQ_MAT2p0                  (4)  /* DMASEL04=1 */
#  define DMA_REQ_MAT2p1                  (5)  /* DMASEL05=1 */
#  define DMA_REQ_MAT3p0                  (14) /* DMASEL14=1 */
#  define DMA_REQ_MAT3p1                  (15) /* DMASEL15=1 */

#  define DMA_REQ_I2SCH0                  (6)  /* DMASEL06=1 */
#  define DMA_REQ_I2SCH1                  (7)  /* DMASEL07=1 */

#  define DMA_REQ_ADC                     (8)  /* Not applicable */
#  define DMA_REQ_DAC                     (9)  /* Not applicable */

#  define DMA_REQ_UART0TX                 (10)  /* DMASEL10=0 */
#  define DMA_REQ_UART0RX                 (11)  /* DMASEL11=0 */
#  define DMA_REQ_UART1TX                 (12)  /* DMASEL12=0 */
#  define DMA_REQ_UART1RX                 (13)  /* DMASEL13=0 */
#  define DMA_REQ_UART2TX                 (14)  /* DMASEL14=0 */
#  define DMA_REQ_UART2RX                 (15)  /* DMASEL15=0 */
#  define DMA_REQ_UART3TX                 (10)  /* DMASEL10=1 */
#  define DMA_REQ_UART3RX                 (11)  /* DMASEL11=1 */
#  define DMA_REQ_UART4TX                 (12)  /* DMASEL12=1 */
#  define DMA_REQ_UART4RX                 (13)  /* DMASEL13=1 */

/* DMASEL values */

#  define DMA_DMASEL_SDCARD               (0)

#  define DMA_DMASEL_SSP0TX               (0)
#  define DMA_DMASEL_SSP0RX               (0)
#  define DMA_DMASEL_SSP1TX               (0)
#  define DMA_DMASEL_SSP1RX               (0)
#  define DMA_DMASEL_SSP2TX               (0)
#  define DMA_DMASEL_SSP2RX               (0)

#  define DMA_DMASEL_MAT0p0               (1)
#  define DMA_DMASEL_MAT0p1               (1)
#  define DMA_DMASEL_MAT1p0               (1)
#  define DMA_DMASEL_MAT1p1               (1)
#  define DMA_DMASEL_MAT2p0               (1)
#  define DMA_DMASEL_MAT2p1               (1)
#  define DMA_DMASEL_MAT3p0               (1)
#  define DMA_DMASEL_MAT3p1               (1)

#  define DMA_DMASEL_I2SCH0               (1)
#  define DMA_DMASEL_I2SCH1               (1)

#  define DMA_DMASEL_ADC                  (0)  /* Not applicable */
#  define DMA_DMASEL_DAC                  (0)  /* Not applicable */

#  define DMA_DMASEL_UART0TX              (0)
#  define DMA_DMASEL_UART0RX              (0)
#  define DMA_DMASEL_UART1TX              (0)
#  define DMA_DMASEL_UART1RX              (0)
#  define DMA_DMASEL_UART2TX              (0)
#  define DMA_DMASEL_UART2RX              (0)
#  define DMA_DMASEL_UART3TX              (1)
#  define DMA_DMASEL_UART3RX              (1)
#  define DMA_DMASEL_UART4TX              (1)
#  define DMA_DMASEL_UART4RX              (1)
#endif

/* General registers (see also LPC17_40_SYSCON_DMAREQSEL in lpc17_40_syscon.h) */
/* Fach of the following registers, bits 0-7 controls DMA channels 9-7,
 * respectively.  Bits 8-31 are reserved.
 *
 *   DMA Interrupt Status Register
 *   DMA Interrupt Terminal Count Request Status Register
 *   DMA Interrupt Terminal Count Request Clear Register
 *   DMA Interrupt Error Status Register
 *   DMA Interrupt Error Clear Register
 *   DMA Raw Interrupt Terminal Count Status Register
 *   DMA Raw Error Interrupt Status Register
 *   DMA Enabled Channel Register
 */

#define DMACH(n)                          (1 << (n)) /* n=0,1,...7 */
#define DMACH_ALL                         (0xff)

/* For each of the following registers, bits 0-15 represent a set of encoded
 * DMA sources. Bits 16-31 are reserved in each case.
 *
 *   DMA Software Burst Request Register
 *   DMA Software Single Request Register
 *   DMA Software Last Burst Request Register
 *   DMA Software Last Single Request Register
 *   DMA Synchronization Register
 */

#if defined(LPC176x)
#  define DMA_REQ_SSP0TX_BIT              (1 << DMA_REQ_SSP0TX)
#  define DMA_REQ_SSP0RX_BIT              (1 << DMA_REQ_SSP0RX)
#  define DMA_REQ_SSP1TX_BIT              (1 << DMA_REQ_SSP1TX)
#  define DMA_REQ_SSP1RX_BIT              (1 << DMA_REQ_SSP0RX)
#  define DMA_REQ_ADC_BIT                 (1 << DMA_REQ_ADC)
#  define DMA_REQ_I2SCH0_BIT              (1 << DMA_REQ_I2SCH0)
#  define DMA_REQ_I2SCH1_BIT              (1 << DMA_REQ_I2SCH1)
#  define DMA_REQ_DAC_BIT                 (1 << DMA_REQ_DAC)

#  define DMA_REQ_UART0TX_BIT             (1 << DMA_REQ_UART0TX)
#  define DMA_REQ_UART0RX_BIT             (1 << DMA_REQ_UART0RX)
#  define DMA_REQ_UART1TX_BIT             (1 << DMA_REQ_UART1TX)
#  define DMA_REQ_UART1RX_BIT             (1 << DMA_REQ_UART1RX)
#  define DMA_REQ_UART2TX_BIT             (1 << DMA_REQ_UART2TX)
#  define DMA_REQ_UART2RX_BIT             (1 << DMA_REQ_UART2RX)
#  define DMA_REQ_UART3TX_BIT             (1 << DMA_REQ_UART3TX)
#  define DMA_REQ_UART3RX_BIT             (1 << DMA_REQ_UART3RX)

#  define DMA_REQ_MAT0p0_BIT              (1 << DMA_REQ_MAT0p0)
#  define DMA_REQ_MAT0p1_BIT              (1 << DMA_REQ_MAT0p1)
#  define DMA_REQ_MAT1p0_BIT              (1 << DMA_REQ_MAT1p0)
#  define DMA_REQ_MAT1p1_BIT              (1 << DMA_REQ_MAT1p1)
#  define DMA_REQ_MAT2p0_BIT              (1 << DMA_REQ_MAT2p0)
#  define DMA_REQ_MAT2p1_BIT              (1 << DMA_REQ_MAT2p1)
#  define DMA_REQ_MAT3p0_BIT              (1 << DMA_REQ_MAT3p0)
#  define DMA_REQ_MAT3p1_BIT              (1 << DMA_REQ_MAT3p1)
#elif defined(LPC178x_40xx)
#  define DMA_REQ_SDCARD_BIT              (1 << DMA_REQ_SDCARD)

#  define DMA_REQ_SSP0TX_BIT              (1 << DMA_REQ_SSP0TX)
#  define DMA_REQ_SSP0RX_BIT              (1 << DMA_REQ_SSP0RX)
#  define DMA_REQ_SSP1TX_BIT              (1 << DMA_REQ_SSP1TX)
#  define DMA_REQ_SSP1RX_BIT              (1 << DMA_REQ_SSP1RX)
#  define DMA_REQ_SSP2TX_BIT              (1 << DMA_REQ_SSP2TX)
#  define DMA_REQ_SSP2RX_BIT              (1 << DMA_REQ_SSP2RX)

#  define DMA_REQ_MAT0p0_BIT              (1 << DMA_REQ_MAT0p0)
#  define DMA_REQ_MAT0p1_BIT              (1 << DMA_REQ_MAT0p1)
#  define DMA_REQ_MAT1p0_BIT              (1 << DMA_REQ_MAT1p0)
#  define DMA_REQ_MAT1p1_BIT              (1 << DMA_REQ_MAT1p1)
#  define DMA_REQ_MAT2p0_BIT              (1 << DMA_REQ_MAT2p0)
#  define DMA_REQ_MAT2p1_BIT              (1 << DMA_REQ_MAT2p1)
#  define DMA_REQ_MAT3p0_BIT              (1 << DMA_REQ_MAT3p0)
#  define DMA_REQ_MAT3p1_BIT              (1 << DMA_REQ_MAT3p1)

#  define DMA_REQ_I2SCH0_BIT              (1 << DMA_REQ_I2SCH0)
#  define DMA_REQ_I2SCH1_BIT              (1 << DMA_REQ_I2SCH1)

#  define DMA_REQ_ADC_BIT                 (1 << DMA_REQ_ADC)
#  define DMA_REQ_DAC_BIT                 (1 << DMA_REQ_DAC)

#  define DMA_REQ_UART0TX_BIT             (1 << DMA_REQ_UART0TX)
#  define DMA_REQ_UART0RX_BIT             (1 << DMA_REQ_UART0RX)
#  define DMA_REQ_UART1TX_BIT             (1 << DMA_REQ_UART1TX)
#  define DMA_REQ_UART1RX_BIT             (1 << DMA_REQ_UART1RX)
#  define DMA_REQ_UART2TX_BIT             (1 << DMA_REQ_UART2TX)
#  define DMA_REQ_UART2RX_BIT             (1 << DMA_REQ_UART2RX)
#  define DMA_REQ_UART3TX_BIT             (1 << DMA_REQ_UART3TX)
#  define DMA_REQ_UART3RX_BIT             (1 << DMA_REQ_UART3RX)
#  define DMA_REQ_UART4TX_BIT             (1 << DMA_REQ_UART4TX)
#  define DMA_REQ_UART4RX_BIT             (1 << DMA_REQ_UART4RX)
#endif

/* DMA Configuration Register */

#define DMA_CONFIG_E                      (1 << 0)  /* Bit 0:  DMA Controller enable */
#define DMA_CONFIG_M                      (1 << 1)  /* Bit 1:  AHB Master endianness configuration */
                                                    /* Bits 2-31: Reserved */
/* Channel Registers */

/* DMA Channel Source Address Register (Bits 0-31: Source Address) */
/* DMA Channel Destination Address Register Bits 0-31: Destination Address) */
/* DMA Channel Linked List Item Register (Bits 0-31: Address of next link list
 * item.  Bits 0-1 must be zero.
 */

/* DMA Channel Control Register */

#define DMACH_CONTROL_XFRSIZE_SHIFT       (0)       /* Bits 0-11: Transfer size */
#define DMACH_CONTROL_XFRSIZE_MASK        (0x0fff << DMACH_CONTROL_XFRSIZE_SHIFT)
#  define DMACH_CONTROL_XFRSIZE(n)        ((n) << DMACH_CONTROL_XFRSIZE_SHIFT)
#define DMACH_CONTROL_SBSIZE_SHIFT        (12)      /* Bits 12-14: Source burst size */
#define DMACH_CONTROL_SBSIZE_MASK         (7 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_1          (0 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_4          (1 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_8          (2 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_16         (3 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_32         (4 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_64         (5 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_128        (6 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_256        (7 << DMACH_CONTROL_SBSIZE_SHIFT)
#define DMACH_CONTROL_DBSIZE_SHIFT        (15)      /* Bits 15-17: Destination burst size */
#define DMACH_CONTROL_DBSIZE_MASK         (7 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_1          (0 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_4          (1 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_8          (2 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_16         (3 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_32         (4 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_64         (5 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_128        (6 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_256        (7 << DMACH_CONTROL_DBSIZE_SHIFT)
#define DMACH_CONTROL_SWIDTH_SHIFT        (18)      /* Bits 18-20: Source transfer width */
#define DMACH_CONTROL_SWIDTH_MASK         (7 << DMACH_CONTROL_SWIDTH_SHIFT)
#  define DMACH_CONTROL_SWIDTH_8BIT       (0 << DMACH_CONTROL_SWIDTH_SHIFT) /* Byte (8-bit) */
#  define DMACH_CONTROL_SWIDTH_16BIT      (1 << DMACH_CONTROL_SWIDTH_SHIFT) /* Halfword (16-bit) */
#  define DMACH_CONTROL_SWIDTH_32BIT      (2 << DMACH_CONTROL_SWIDTH_SHIFT) /* Word (32-bit) */
#define DMACH_CONTROL_DWIDTH_SHIFT        (21)      /* Bits 21-23: Destination transfer width */
#define DMACH_CONTROL_DWIDTH_MASK         (7 << DMACH_CONTROL_DWIDTH_SHIFT)
#  define DMACH_CONTROL_DWIDTH_8BIT       (0 << DMACH_CONTROL_DWIDTH_SHIFT) /* Byte (8-bit) */
#  define DMACH_CONTROL_DWIDTH_16BIT      (1 << DMACH_CONTROL_DWIDTH_SHIFT) /* Halfword (16-bit) */
#  define DMACH_CONTROL_DWIDTH_32BIT      (2 << DMACH_CONTROL_DWIDTH_SHIFT) /* Word (32-bit) */
#define DMACH_CONTROL_SI                  (1 << 26) /* Bit 26: Source increment */
#define DMACH_CONTROL_DI                  (1 << 27) /* Bit 27: Destination increment */
#define DMACH_CONTROL_PROT1               (1 << 28) /* Bit 28: User/privileged mode */
#define DMACH_CONTROL_PROT2               (1 << 29) /* Bit 29: Bufferable */
#define DMACH_CONTROL_PROT3               (1 << 30) /* Bit 30: Cacheable */
#define DMACH_CONTROL_I                   (1 << 31) /* Bit 31: Terminal count interrupt enable */

/* DMA Channel Configuration Register */

#define DMACH_CONFIG_E                    (1 << 0) /* Bit 0: Channel enable */
#define DMACH_CONFIG_SRCPER_SHIFT         (1)      /* Bits 1-5: Source peripheral */
#define DMACH_CONFIG_SRCPER_MASK          (31 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SDCARD      (DMA_REQ_SDCARD << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP0TX      (DMA_REQ_SSP0TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP0RX      (DMA_REQ_SSP0RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP1TX      (DMA_REQ_SSP1TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP1RX      (DMA_REQ_SSP1RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP2TX      (DMA_REQ_SSP2TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP2RX      (DMA_REQ_SSP2RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_I2SCH0      (DMA_REQ_I2SCH0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_I2SCH1      (DMA_REQ_I2SCH1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_ADC         (DMA_REQ_ADC << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_DAC         (DMA_REQ_DAC << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART0TX     (DMA_REQ_UART0TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART0RX     (DMA_REQ_UART0RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART1TX     (DMA_REQ_UART1TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART1RX     (DMA_REQ_UART1RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART2TX     (DMA_REQ_UART2TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART2RX     (DMA_REQ_UART2RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART3TX     (DMA_REQ_UART3TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART3RX     (DMA_REQ_UART3RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART4TX     (DMA_REQ_UART4TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART4RX     (DMA_REQ_UART4RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT0p0      (DMA_REQ_MAT0p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT0p1      (DMA_REQ_MAT0p1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT1p0      (DMA_REQ_MAT1p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT1p1      (DMA_REQ_MAT1p1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT2p0      (DMA_REQ_MAT2p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT2p1      (DMA_REQ_MAT2p1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT3p0      (DMA_REQ_MAT3p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT3p1      (DMA_REQ_MAT3p1 << DMACH_CONFIG_SRCPER_SHIFT)
#define DMACH_CONFIG_DSTPER_SHIFT         (6)      /* Bits 6-10: Destination peripheral */
#define DMACH_CONFIG_DSTPER_MASK          (31 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SDCARD      (DMA_REQ_SDCARD << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP0TX      (DMA_REQ_SSP0TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP0RX      (DMA_REQ_SSP0RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP1TX      (DMA_REQ_SSP1TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP1RX      (DMA_REQ_SSP1RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP2TX      (DMA_REQ_SSP2TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP2RX      (DMA_REQ_SSP2RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_I2SCH0      (DMA_REQ_I2SCH0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_I2SCH1      (DMA_REQ_I2SCH1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_ADC         (DMA_REQ_ADC << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_DAC         (DMA_REQ_DAC << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART0TX     (DMA_REQ_UART0TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART0RX     (DMA_REQ_UART0RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART1TX     (DMA_REQ_UART1TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART1RX     (DMA_REQ_UART1RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART2TX     (DMA_REQ_UART2TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART2RX     (DMA_REQ_UART2RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART3TX     (DMA_REQ_UART3TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART3RX     (DMA_REQ_UART3RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART4TX     (DMA_REQ_UART4TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART4RX     (DMA_REQ_UART4RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT0p0      (DMA_REQ_MAT0p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT0p1      (DMA_REQ_MAT0p1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT1p0      (DMA_REQ_MAT1p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT1p1      (DMA_REQ_MAT1p1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT2p0      (DMA_REQ_MAT2p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT2p1      (DMA_REQ_MAT2p1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT3p0      (DMA_REQ_MAT3p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT3p1      (DMA_REQ_MAT3p1 << DMACH_CONFIG_DSTPER_SHIFT)
#define DMACH_CONFIG_XFRTYPE_SHIFT        (11)      /* Bits 11-13: Type of transfer */
#define DMACH_CONFIG_XFRTYPE_MASK         (7 << DMACH_CONFIG_XFRTYPE_SHIFT)
                                                                            /* Flow controller = DMA controller */
#  define DMACH_CONFIG_XFRTYPE_M2M        (0 << DMACH_CONFIG_XFRTYPE_SHIFT) /*   Memory to memory */
#  define DMACH_CONFIG_XFRTYPE_M2P        (1 << DMACH_CONFIG_XFRTYPE_SHIFT) /*   Memory to peripheral */
#  define DMACH_CONFIG_XFRTYPE_P2M        (2 << DMACH_CONFIG_XFRTYPE_SHIFT) /*   Peripheral to memory */
#  define DMACH_CONFIG_XFRTYPE_P2P        (3 << DMACH_CONFIG_XFRTYPE_SHIFT) /*   Peripheral to peripheral */
#ifdef LPC178x_40xx
                                                                            /* Flow controller = Dest peripheral */
#  define DMACH_CONFIG_XFRTYPE_M2M_DC     (4 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Memory to memory */
#  define DMACH_CONFIG_XFRTYPE_M2P_DC     (5 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Memory to peripheral */
                                                                            /* Flow controller = Source peripheral */
#  define DMACH_CONFIG_XFRTYPE_P2M_SC     (6 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Peripheral to memory */
#  define DMACH_CONFIG_XFRTYPE_P2P_SC     (7 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Peripheral to peripheral */
#endif
#define DMACH_CONFIG_IE                   (1 << 14) /* Bit 14: Interrupt error mask */
#define DMACH_CONFIG_ITC                  (1 << 15) /* Bit 15: Terminal count interrupt mask */
#define DMACH_CONFIG_L                    (1 << 16) /* Bit 16: Lock */
#define DMACH_CONFIG_A                    (1 << 17) /* Bit 17: Active */
#define DMACH_CONFIG_H                    (1 << 18) /* Bit 18: Halt */
                                                    /* Bits 19-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_GPDMA_H */
