/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_spi.h
 *
 *   Copyright (C) 2015, 2019 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_SPI_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/pic32mz/chip.h>

#include "hardware/pic32mz_memorymap.h"

#if CHIP_NSPI > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI Peripheral Offsets ***************************************************/

#define PIC32MZ_SPIn_OFFSET(n)    ((n) << 9)
#  define PIC32MZ_SPI1_OFFSET     0x0000
#  define PIC32MZ_SPI2_OFFSET     0x0200
#  define PIC32MZ_SPI3_OFFSET     0x0400
#  define PIC32MZ_SPI4_OFFSET     0x0600
#  define PIC32MZ_SPI5_OFFSET     0x0800
#  define PIC32MZ_SPI6_OFFSET     0x0a00

/* SPI Register Offsets *****************************************************/

#define PIC32MZ_SPI_CON_OFFSET     0x0000 /* SPI control register */
#define PIC32MZ_SPI_CONCLR_OFFSET  0x0004 /* SPI control clear register */
#define PIC32MZ_SPI_CONSET_OFFSET  0x0008 /* SPI control set register */
#define PIC32MZ_SPI_CONINV_OFFSET  0x000c /* SPI control invert register */

#define PIC32MZ_SPI_STAT_OFFSET    0x0010 /* SPI status register */
#define PIC32MZ_SPI_STATCLR_OFFSET 0x0014 /* SPI status clear register */

#define PIC32MZ_SPI_BUF_OFFSET     0x0020 /* SPI buffer register */

#define PIC32MZ_SPI_BRG_OFFSET     0x0030 /* SPI baud rate register */
#define PIC32MZ_SPI_BRGCLR_OFFSET  0x0034 /* SPI baud rate clear register */
#define PIC32MZ_SPI_BRGSET_OFFSET  0x0038 /* SPI baud rate set register */
#define PIC32MZ_SPI_BRGINV_OFFSET  0x003c /* SPI baud rate invert register */

#define PIC32MZ_SPI_CON2_OFFSET    0x0040 /* SPI control register 2 */
#define PIC32MZ_SPI_CON2CLR_OFFSET 0x0044 /* SPI control clear register 2 */
#define PIC32MZ_SPI_CON2SET_OFFSET 0x0048 /* SPI control set register 2 */
#define PIC32MZ_SPI_CON2INV_OFFSET 0x004c /* SPI control invert register 2 */

/* SPI Peripheral Addresses *************************************************/

#define PIC32MZ_SPIn_K1BASE(n)    (PIC32MZ_SPI_K1BASE + PIC32MZ_SPIn_OFFSET(n))
#  define PIC32MZ_SPI1_K1BASE     (PIC32MZ_SPI_K1BASE + PIC32MZ_SPI1_OFFSET)
#  define PIC32MZ_SPI2_K1BASE     (PIC32MZ_SPI_K1BASE + PIC32MZ_SPI2_OFFSET)
#  define PIC32MZ_SPI3_K1BASE     (PIC32MZ_SPI_K1BASE + PIC32MZ_SPI3_OFFSET)
#  define PIC32MZ_SPI4_K1BASE     (PIC32MZ_SPI_K1BASE + PIC32MZ_SPI4_OFFSET)
#  define PIC32MZ_SPI5_K1BASE     (PIC32MZ_SPI_K1BASE + PIC32MZ_SPI5_OFFSET)
#  define PIC32MZ_SPI6_K1BASE     (PIC32MZ_SPI_K1BASE + PIC32MZ_SPI6_OFFSET)

/* SPI Register Addresses ***************************************************/

#define PIC32MZ_SPI1_CON          (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CON_OFFSET)
#define PIC32MZ_SPI1_CONCLR       (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CONCLR_OFFSET)
#define PIC32MZ_SPI1_CONSET       (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CONSET_OFFSET)
#define PIC32MZ_SPI1_CONINV       (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CONINV_OFFSET)
#define PIC32MZ_SPI1_STAT         (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_STAT_OFFSET)
#define PIC32MZ_SPI1_STATCLR      (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_STATCLR_OFFSET)
#define PIC32MZ_SPI1_BUF          (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_BUF_OFFSET)
#define PIC32MZ_SPI1_BRG          (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_BRG_OFFSET)
#define PIC32MZ_SPI1_BRGCLR       (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_BRGCLR_OFFSET)
#define PIC32MZ_SPI1_BRGSET       (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_BRGSET_OFFSET)
#define PIC32MZ_SPI1_BRGINV       (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_BRGINV_OFFSET)
#define PIC32MZ_SPI1_CON2         (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CON2_OFFSET)
#define PIC32MZ_SPI1_CON2CLR      (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CON2CLR_OFFSET)
#define PIC32MZ_SPI1_CON2SET      (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CON2SET_OFFSET)
#define PIC32MZ_SPI1_CON2INV      (PIC32MZ_SPI1_K1BASE+PIC32MZ_SPI_CON2INV_OFFSET)

#if CHIP_NSPI > 1
#  define PIC32MZ_SPI2_CON         (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CON_OFFSET)
#  define PIC32MZ_SPI2_CONCLR      (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CONCLR_OFFSET)
#  define PIC32MZ_SPI2_CONSET      (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CONSET_OFFSET)
#  define PIC32MZ_SPI2_CONINV      (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CONINV_OFFSET)
#  define PIC32MZ_SPI2_STAT        (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_STAT_OFFSET)
#  define PIC32MZ_SPI2_STATCLR     (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_STATCLR_OFFSET)
#  define PIC32MZ_SPI2_BUF         (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_BUF_OFFSET)
#  define PIC32MZ_SPI2_BRG         (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_BRG_OFFSET)
#  define PIC32MZ_SPI2_BRGCLR      (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_BRGCLR_OFFSET)
#  define PIC32MZ_SPI2_BRGSET      (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_BRGSET_OFFSET)
#  define PIC32MZ_SPI2_BRGINV      (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_BRGINV_OFFSET)
#  define PIC32MZ_SPI2_CON2        (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CON2_OFFSET)
#  define PIC32MZ_SPI2_CON2CLR     (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CON2CLR_OFFSET)
#  define PIC32MZ_SPI2_CON2SET     (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CON2SET_OFFSET)
#  define PIC32MZ_SPI2_CON2INV     (PIC32MZ_SPI2_K1BASE+PIC32MZ_SPI_CON2INV_OFFSET)
#endif

#if CHIP_NSPI > 2
#  define PIC32MZ_SPI3_CON         (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_CON_OFFSET)
#  define PIC32MZ_SPI3_CONCLR      (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_CONCLR_OFFSET)
#  define PIC32MZ_SPI3_CONSET      (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_CONSET_OFFSET)
#  define PIC32MZ_SPI3_CONINV      (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_CONINV_OFFSET)
#  define PIC32MZ_SPI3_STAT        (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_STAT_OFFSET)
#  define PIC32MZ_SPI3_STATCLR     (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_STATCLR_OFFSET)
#  define PIC32MZ_SPI3_BUF         (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_BUF_OFFSET)
#  define PIC32MZ_SPI3_BRG         (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_BRG_OFFSET)
#  define PIC32MZ_SPI3_BRGCLR      (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_BRGCLR_OFFSET)
#  define PIC32MZ_SPI3_BRGSET      (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_BRGSET_OFFSET)
#  define PIC32MZ_SPI3_BRGINV      (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_BRGINV_OFFSET)
#  define PIC32MZ_SPI3_CON2CLR     (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_CON2CLR_OFFSET)
#  define PIC32MZ_SPI3_CON2SET     (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_CON2SET_OFFSET)
#  define PIC32MZ_SPI3_CON2INV     (PIC32MZ_SPI3_K1BASE+PIC32MZ_SPI_CON2INV_OFFSET)
#endif

#if CHIP_NSPI > 3
#  define PIC32MZ_SPI4_CON         (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_CON_OFFSET)
#  define PIC32MZ_SPI4_CONCLR      (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_CONCLR_OFFSET)
#  define PIC32MZ_SPI4_CONSET      (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_CONSET_OFFSET)
#  define PIC32MZ_SPI4_CONINV      (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_CONINV_OFFSET)
#  define PIC32MZ_SPI4_STAT        (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_STAT_OFFSET)
#  define PIC32MZ_SPI4_STATCLR     (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_STATCLR_OFFSET)
#  define PIC32MZ_SPI4_BUF         (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_BUF_OFFSET)
#  define PIC32MZ_SPI4_BRG         (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_BRG_OFFSET)
#  define PIC32MZ_SPI4_BRGCLR      (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_BRGCLR_OFFSET)
#  define PIC32MZ_SPI4_BRGSET      (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_BRGSET_OFFSET)
#  define PIC32MZ_SPI4_BRGINV      (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_BRGINV_OFFSET)
#  define PIC32MZ_SPI4_CON2CLR     (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_CON2CLR_OFFSET)
#  define PIC32MZ_SPI4_CON2SET     (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_CON2SET_OFFSET)
#  define PIC32MZ_SPI4_CON2INV     (PIC32MZ_SPI4_K1BASE+PIC32MZ_SPI_CON2INV_OFFSET)
#endif

#if CHIP_NSPI > 4
#  define PIC32MZ_SPI5_CON         (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_CON_OFFSET)
#  define PIC32MZ_SPI5_CONCLR      (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_CONCLR_OFFSET)
#  define PIC32MZ_SPI5_CONSET      (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_CONSET_OFFSET)
#  define PIC32MZ_SPI5_CONINV      (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_CONINV_OFFSET)
#  define PIC32MZ_SPI5_STAT        (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_STAT_OFFSET)
#  define PIC32MZ_SPI5_STATCLR     (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_STATCLR_OFFSET)
#  define PIC32MZ_SPI5_BUF         (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_BUF_OFFSET)
#  define PIC32MZ_SPI5_BRG         (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_BRG_OFFSET)
#  define PIC32MZ_SPI5_BRGCLR      (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_BRGCLR_OFFSET)
#  define PIC32MZ_SPI5_BRGSET      (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_BRGSET_OFFSET)
#  define PIC32MZ_SPI5_BRGINV      (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_BRGINV_OFFSET)
#  define PIC32MZ_SPI5_CON2CLR     (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_CON2CLR_OFFSET)
#  define PIC32MZ_SPI5_CON2SET     (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_CON2SET_OFFSET)
#  define PIC32MZ_SPI5_CON2INV     (PIC32MZ_SPI5_K1BASE+PIC32MZ_SPI_CON2INV_OFFSET)
#endif

#if CHIP_NSPI > 5
#  define PIC32MZ_SPI6_CON         (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_CON_OFFSET)
#  define PIC32MZ_SPI6_CONCLR      (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_CONCLR_OFFSET)
#  define PIC32MZ_SPI6_CONSET      (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_CONSET_OFFSET)
#  define PIC32MZ_SPI6_CONINV      (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_CONINV_OFFSET)
#  define PIC32MZ_SPI6_STAT        (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_STAT_OFFSET)
#  define PIC32MZ_SPI6_STATCLR     (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_STATCLR_OFFSET)
#  define PIC32MZ_SPI6_BUF         (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_BUF_OFFSET)
#  define PIC32MZ_SPI6_BRG         (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_BRG_OFFSET)
#  define PIC32MZ_SPI6_BRGCLR      (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_BRGCLR_OFFSET)
#  define PIC32MZ_SPI6_BRGSET      (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_BRGSET_OFFSET)
#  define PIC32MZ_SPI6_BRGINV      (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_BRGINV_OFFSET)
#  define PIC32MZ_SPI6_CON2CLR     (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_CON2CLR_OFFSET)
#  define PIC32MZ_SPI6_CON2SET     (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_CON2SET_OFFSET)
#  define PIC32MZ_SPI6_CON2INV     (PIC32MZ_SPI6_K1BASE+PIC32MZ_SPI_CON2INV_OFFSET)
#endif

/* Register Bit-Field Definitions *******************************************/

/* SPI control register */

#define SPI_CON_SRXISEL_SHIFT      (0)       /* Bits 0-1: SPI Receive Buffer Full Interrupt Mode */
#define SPI_CON_SRXISEL_MASK       (3 << SPI_CON_SRXISEL_SHIFT)
#  define SPI_CON_SRXISEL_EMPTY    (0 << SPI_CON_SRXISEL_SHIFT) /* Buffer empty */
#  define SPI_CON_SRXISEL_NEMPTY   (1 << SPI_CON_SRXISEL_SHIFT) /* Buffer not empty */
#  define SPI_CON_SRXISEL_HALF     (2 << SPI_CON_SRXISEL_SHIFT) /* Buffer half full or more */
#  define SPI_CON_SRXISEL_FULL     (3 << SPI_CON_SRXISEL_SHIFT) /* Buffer full */
#define SPI_CON_STXISEL_SHIFT      (2)       /* Bits 2-3: SPI Transmit Buffer Empty Interrupt Mode */
#define SPI_CON_STXISEL_MASK       (3 << SPI_CON_STXISEL_SHIFT)
#  define SPI_CON_STXISEL_DONE     (0 << SPI_CON_STXISEL_SHIFT) /* Buffer empty (and data shifted out) */
#  define SPI_CON_STXISEL_EMPTY    (1 << SPI_CON_STXISEL_SHIFT) /* Buffer empty */
#  define SPI_CON_STXISEL_HALF     (2 << SPI_CON_STXISEL_SHIFT) /* Buffer half empty or more */
#  define SPI_CON_STXISEL_NFULL    (3 << SPI_CON_STXISEL_SHIFT) /* Buffer not full */
#define SPI_CON_DISSDI             (1 << 4)  /* Bit 4: Disable SDI */
#define SPI_CON_MSTEN              (1 << 5)  /* Bit 5: Master mode enable */
#define SPI_CON_CKP                (1 << 6)  /* Bit 6: Clock polarity select */
#define SPI_CON_SSEN               (1 << 7)  /* Bit 7: Slave select enable (slave mode) */
#define SPI_CON_CKE                (1 << 8)  /* Bit 8: SPI clock edge select */
#define SPI_CON_SMP                (1 << 9)  /* Bit 9: SPI data input sample phase */
#define SPI_CON_MODE_SHIFT         (10)      /* Bits 10-11: 32/16-Bit Communication Select */
#define SPI_CON_MODE_MASK          (3 << SPI_CON_MODE_SHIFT)
                                                             /* With AUDEN=0: */
#  define SPI_CON_MODE_8BIT        (0 << SPI_CON_MODE_SHIFT) /* 8-bit data width */
#  define SPI_CON_MODE_16BIT       (1 << SPI_CON_MODE_SHIFT) /* 16-bit data width */
#  define SPI_CON_MODE_32BIT       (2 << SPI_CON_MODE_SHIFT) /* 32-bit data width */
                                                             /* With AUDEN=1: */
#  define SPI_CON_MODE_161616      (0 << SPI_CON_MODE_SHIFT) /* 16-bit data, 16-bit FIFO, 16-bit channel */
#  define SPI_CON_MODE_161632      (1 << SPI_CON_MODE_SHIFT) /* 16-bit data, 16-bit FIFO, 32-bit channel */
#  define SPI_CON_MODE_323232      (2 << SPI_CON_MODE_SHIFT) /* 32-bit data, 32-bit FIFO, 32-bit channel */
#  define SPI_CON_MODE_243232      (3 << SPI_CON_MODE_SHIFT) /* 24-bit data, 32-bit FIFO, 32-bit channel */
#define SPI_CON_DISSDO             (1 << 12) /* Bit 12: Disable SDOx pin */
#define SPI_CON_SIDL               (1 << 13) /* Bit 13: Stop in idle mode */
#define SPI_CON_ON                 (1 << 15) /* Bit 15: SPI peripheral on */
#define SPI_CON_ENHBUF             (1 << 16) /* Bit 16: Enhanced buffer enable */
#define SPI_CON_SPIFE              (1 << 17) /* Bit 17: Frame sync pulse edge select */
                                             /* Bits 18-23: Reserved */
#define SPI_CON_MCLKSEL            (1 << 23) /* Bit 23: Master clock enable */
#define SPI_CON_FRMCNT_SHIFT       (24)      /* Bits 24-26: Frame Sync Pulse Counter bits */
#define SPI_CON_FRMCNT_MASK        (7 << SPI_CON_FRMCNT_SHIFT)
#  define SPI_CON_FRMCNT_CHAR1     (0 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse each char */
#  define SPI_CON_FRMCNT_CHAR2     (1 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 2 chars */
#  define SPI_CON_FRMCNT_CHAR4     (2 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 4 chars */
#  define SPI_CON_FRMCNT_CHAR8     (3 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 8 chars */
#  define SPI_CON_FRMCNT_CHAR16    (4 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 16 chars */
#  define SPI_CON_FRMCNT_CHAR32    (5 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 32 chars */
#define SPI_CON_FRMSYPW            (1 << 27) /* Bits 27: Frame sync pulse width */
#define SPI_CON_MSSEN              (1 << 28) /* Bits 28: Master mode slave select enable */
#define SPI_CON_FRMPOL             (1 << 29) /* Bits 29: Frame sync polarity */
#define SPI_CON_FRMSYNC            (1 << 30) /* Bits 30: Frame sync pulse direction control on SSx pin */
#define SPI_CON_FRMEN              (1 << 31) /* Bits 31: Framed SPI support */

/* SPI control register 2 */

#define SPI_CON2_AUDMOD_SHIFT     (0)       /* Bits 0-1: Audio Protocol Mode */
#define SPI_CON2_AUDMOD_MASK      (3 << SPI2_CON2_AUDMOD_SHIFT)
#  define SPI_CON2_AUDMOD_I2S     (0 << SPI2_CON2_AUDMOD_SHIFT) /* I2S mode */
#  define SPI_CON2_AUDMOD_LJ      (1 << SPI2_CON2_AUDMOD_SHIFT) /* Left Justified mode */
#  define SPI_CON2_AUDMOD_RJ      (2 << SPI2_CON2_AUDMOD_SHIFT) /* Right Justified mode */
#  define SPI_CON2_AUDMOD_PCM     (3 << SPI2_CON2_AUDMOD_SHIFT) /* PCM/DSP mode */
                                             /* Bit 2: Reserved */
#define SPI_CON2_AUDMONO          (1 << 3)  /* Bit 3:  Transmit Audio Data Format */
                                             /* Bits 5-6: Reserved */
#define SPI_CON2_AUDEN            (1 << 7)  /* Bit 7:  Enable Audio CODEC Support */
#define SPI_CON2_IGNTUR           (1 << 8)  /* Bit 8:  Ignore Transmit Underrun bit */
#define SPI_CON2_IGNROV           (1 << 9)  /* Bit 9:  Ignore Receive Overflow */
#define SPI_CON2_SPITUREN         (1 << 10) /* Bit 10: Enable Interrupt Events via SPITUR */
#define SPI_CON2_SPIROVEN         (1 << 11) /* Bit 11: Enable Interrupt Events via SPIROV */
#define SPI_CON2_FRMERREN         (1 << 12) /* Bit 12: Enable Interrupt Events via FRMERR */
                                             /* Bits 13-14: Reserved */
#define SPI_CON2_SPISGNEXT        (1 << 15) /* Bit 15 : Sign Extend Read Data from the RX FIFO */
                                             /* Bits 16-31: Reserved */

/* SPI status register */

#define SPI_STAT_SPIRBF            (1 << 0)  /* Bit 0: SPI receive buffer full status */
#define SPI_STAT_SPITBF            (1 << 1)  /* Bit 1: SPI transmit buffer full status */
                                             /* Bit 2: Reserved */
#define SPI_STAT_SPITBE            (1 << 3)  /* Bit 3: SPI transmit buffer empty status */
                                             /* Bit 4: Reserved */
#define SPI_STAT_SPIRBE            (1 << 5)  /* Bit 5: RX FIFO Empty */
#define SPI_STAT_SPIROV            (1 << 6)  /* Bit 6: Receive overflow flag */
#define SPI_STAT_SRMT              (1 << 7)  /* Bit 6: Shift Register Empty */
#define SPI_STAT_SPITUR            (1 << 8)  /* Bit 8: Transmit under run */
                                             /* Bits 9-10: Reserved */
#define SPI_STAT_SPIBUSY           (1 << 11) /* Bit 11: SPI activity status */
#define SPI_STAT_FRMERR            (1 << 12) /* Bit 12: SPI Frame Error status */
                                             /* Bits 13-15: Reserved */
#define SPI_STAT_TXBUFELM_SHIFT    (16)      /* Bits 16-20: Transmit Buffer Element Count bits */
#define SPI_STAT_TXBUFELM_MASK     (31 << SPI_STAT_TXBUFELM_SHIFT)
#  define SPI_STAT_TXBUFELM(n)     ((uint32_t)(n) << SPI_STAT_TXBUFELM_SHIFT)
                                             /* Bits 21-23: Reserved */
#define SPI_STAT_RXBUFELM_SHIFT    (24)      /* Bits 24-28: Receive Buffer Element Count bits */
#define SPI_STAT_RXBUFELM_MASK     (31 << SPI_STAT_RXBUFELM_SHIFT)
#  define SPI_STAT_RXBUFELM(n)     ((uint32_t)(n) << SPI_STAT_RXBUFELM_SHIFT)

/* SPI buffer register (32-bits wide) */

/* SPI baud rate register */

#define SPI_BRG_MASK               0x00001fff

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CHIP_NSPI > 0 */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_SPI_H */
