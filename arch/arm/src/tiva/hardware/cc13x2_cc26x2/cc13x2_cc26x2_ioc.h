/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_ioc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_IOC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_IOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIVA_NDIO                  32      /* DIO0-31 */

/* IOC register offsets *****************************************************/

#define TIVA_IOC_IOCFG_OFFSET(n)   ((n) << 2)
#  define TIVA_IOC_IOCFG0_OFFSET   0x0000  /* Configuration of DIO0 */
#  define TIVA_IOC_IOCFG1_OFFSET   0x0004  /* Configuration of DIO1 */
#  define TIVA_IOC_IOCFG2_OFFSET   0x0008  /* Configuration of DIO2 */
#  define TIVA_IOC_IOCFG3_OFFSET   0x000c  /* Configuration of DIO3 */
#  define TIVA_IOC_IOCFG4_OFFSET   0x0010  /* Configuration of DIO4 */
#  define TIVA_IOC_IOCFG5_OFFSET   0x0014  /* Configuration of DIO5 */
#  define TIVA_IOC_IOCFG6_OFFSET   0x0018  /* Configuration of DIO6 */
#  define TIVA_IOC_IOCFG7_OFFSET   0x001c  /* Configuration of DIO7 */
#  define TIVA_IOC_IOCFG8_OFFSET   0x0020  /* Configuration of DIO8 */
#  define TIVA_IOC_IOCFG9_OFFSET   0x0024  /* Configuration of DIO9 */
#  define TIVA_IOC_IOCFG10_OFFSET  0x0028  /* Configuration of DIO10 */
#  define TIVA_IOC_IOCFG11_OFFSET  0x002c  /* Configuration of DIO11 */
#  define TIVA_IOC_IOCFG12_OFFSET  0x0030  /* Configuration of DIO12 */
#  define TIVA_IOC_IOCFG13_OFFSET  0x0034  /* Configuration of DIO13 */
#  define TIVA_IOC_IOCFG14_OFFSET  0x0038  /* Configuration of DIO14 */
#  define TIVA_IOC_IOCFG15_OFFSET  0x003c  /* Configuration of DIO15 */
#  define TIVA_IOC_IOCFG16_OFFSET  0x0040  /* Configuration of DIO16 */
#  define TIVA_IOC_IOCFG17_OFFSET  0x0044  /* Configuration of DIO17 */
#  define TIVA_IOC_IOCFG18_OFFSET  0x0048  /* Configuration of DIO18 */
#  define TIVA_IOC_IOCFG19_OFFSET  0x004c  /* Configuration of DIO19 */
#  define TIVA_IOC_IOCFG20_OFFSET  0x0050  /* Configuration of DIO20 */
#  define TIVA_IOC_IOCFG21_OFFSET  0x0054  /* Configuration of DIO21 */
#  define TIVA_IOC_IOCFG22_OFFSET  0x0058  /* Configuration of DIO22 */
#  define TIVA_IOC_IOCFG23_OFFSET  0x005c  /* Configuration of DIO23 */
#  define TIVA_IOC_IOCFG24_OFFSET  0x0060  /* Configuration of DIO24 */
#  define TIVA_IOC_IOCFG25_OFFSET  0x0064  /* Configuration of DIO25 */
#  define TIVA_IOC_IOCFG26_OFFSET  0x0068  /* Configuration of DIO26 */
#  define TIVA_IOC_IOCFG27_OFFSET  0x006c  /* Configuration of DIO27 */
#  define TIVA_IOC_IOCFG28_OFFSET  0x0070  /* Configuration of DIO28 */
#  define TIVA_IOC_IOCFG29_OFFSET  0x0074  /* Configuration of DIO29 */
#  define TIVA_IOC_IOCFG30_OFFSET  0x0078  /* Configuration of DIO30 */
#  define TIVA_IOC_IOCFG31_OFFSET  0x007c  /* Configuration of DIO31 */

/* IOC register addresses ***************************************************/

#define TIVA_IOC_IOCFG(n)          (TIVA_IOC_BASE + TIVA_IOC_IOCFG_OFFSET(n))
#  define TIVA_IOC_IOCFG0          (TIVA_IOC_BASE + TIVA_IOC_IOCFG0_OFFSET)
#  define TIVA_IOC_IOCFG1          (TIVA_IOC_BASE + TIVA_IOC_IOCFG1_OFFSET)
#  define TIVA_IOC_IOCFG2          (TIVA_IOC_BASE + TIVA_IOC_IOCFG2_OFFSET)
#  define TIVA_IOC_IOCFG3          (TIVA_IOC_BASE + TIVA_IOC_IOCFG3_OFFSET)
#  define TIVA_IOC_IOCFG4          (TIVA_IOC_BASE + TIVA_IOC_IOCFG4_OFFSET)
#  define TIVA_IOC_IOCFG5          (TIVA_IOC_BASE + TIVA_IOC_IOCFG5_OFFSET)
#  define TIVA_IOC_IOCFG6          (TIVA_IOC_BASE + TIVA_IOC_IOCFG6_OFFSET)
#  define TIVA_IOC_IOCFG7          (TIVA_IOC_BASE + TIVA_IOC_IOCFG7_OFFSET)
#  define TIVA_IOC_IOCFG8          (TIVA_IOC_BASE + TIVA_IOC_IOCFG8_OFFSET)
#  define TIVA_IOC_IOCFG9          (TIVA_IOC_BASE + TIVA_IOC_IOCFG9_OFFSET)
#  define TIVA_IOC_IOCFG10         (TIVA_IOC_BASE + TIVA_IOC_IOCFG10_OFFSET)
#  define TIVA_IOC_IOCFG11         (TIVA_IOC_BASE + TIVA_IOC_IOCFG11_OFFSET)
#  define TIVA_IOC_IOCFG12         (TIVA_IOC_BASE + TIVA_IOC_IOCFG12_OFFSET)
#  define TIVA_IOC_IOCFG13         (TIVA_IOC_BASE + TIVA_IOC_IOCFG13_OFFSET)
#  define TIVA_IOC_IOCFG14         (TIVA_IOC_BASE + TIVA_IOC_IOCFG14_OFFSET)
#  define TIVA_IOC_IOCFG15         (TIVA_IOC_BASE + TIVA_IOC_IOCFG15_OFFSET)
#  define TIVA_IOC_IOCFG16         (TIVA_IOC_BASE + TIVA_IOC_IOCFG16_OFFSET)
#  define TIVA_IOC_IOCFG17         (TIVA_IOC_BASE + TIVA_IOC_IOCFG17_OFFSET)
#  define TIVA_IOC_IOCFG18         (TIVA_IOC_BASE + TIVA_IOC_IOCFG18_OFFSET)
#  define TIVA_IOC_IOCFG19         (TIVA_IOC_BASE + TIVA_IOC_IOCFG19_OFFSET)
#  define TIVA_IOC_IOCFG20         (TIVA_IOC_BASE + TIVA_IOC_IOCFG20_OFFSET)
#  define TIVA_IOC_IOCFG21         (TIVA_IOC_BASE + TIVA_IOC_IOCFG21_OFFSET)
#  define TIVA_IOC_IOCFG22         (TIVA_IOC_BASE + TIVA_IOC_IOCFG22_OFFSET)
#  define TIVA_IOC_IOCFG23         (TIVA_IOC_BASE + TIVA_IOC_IOCFG23_OFFSET)
#  define TIVA_IOC_IOCFG24         (TIVA_IOC_BASE + TIVA_IOC_IOCFG24_OFFSET)
#  define TIVA_IOC_IOCFG25         (TIVA_IOC_BASE + TIVA_IOC_IOCFG25_OFFSET)
#  define TIVA_IOC_IOCFG26         (TIVA_IOC_BASE + TIVA_IOC_IOCFG26_OFFSET)
#  define TIVA_IOC_IOCFG27         (TIVA_IOC_BASE + TIVA_IOC_IOCFG27_OFFSET)
#  define TIVA_IOC_IOCFG28         (TIVA_IOC_BASE + TIVA_IOC_IOCFG28_OFFSET)
#  define TIVA_IOC_IOCFG29         (TIVA_IOC_BASE + TIVA_IOC_IOCFG29_OFFSET)
#  define TIVA_IOC_IOCFG30         (TIVA_IOC_BASE + TIVA_IOC_IOCFG30_OFFSET)
#  define TIVA_IOC_IOCFG31         (TIVA_IOC_BASE + TIVA_IOC_IOCFG31_OFFSET)

/* IOC register bit settings ************************************************/

#define IOC_IOCFG_PORTID_SHIFT     (0)       /* Bits 0-5:  Selects DIO usage */
#define IOC_IOCFG_PORTID_MASK      (0x3f << IOC_IOCFG_PORTID_SHIFT)
#  define IOC_IOCFG_PORTID(n)      ((uint32_t)(n) << IOC_IOCFG_PORTID_SHIFT) /* See PORT ID definitions */

#define IOC_IOCFG_IOEV_MCU_WUEN    (1 << 6)  /* Bit 6:  Input edge asserts MCU_WU event */
#define IOC_IOCFG_IOEV_RTCEN       (1 << 7)  /* Bit 7:  Input edge asserts RTC event */
#define IOC_IOCFG_IOSTR_SHIFT      (8)       /* Bits 8-9: I/O drive strength */
#define IOC_IOCFG_IOSTR_MASK       (3 << IOC_IOCFG_IOSTR_SHIFT)
#  define IOC_IOCFG_IOSTR_AUTO     (0 << IOC_IOCFG_IOSTR_SHIFT) /* Automatic drive strength */
#  define IOC_IOCFG_IOSTR_MIN      (1 << IOC_IOCFG_IOSTR_SHIFT) /* Minimum drive strength */
#  define IOC_IOCFG_IOSTR_MED      (2 << IOC_IOCFG_IOSTR_SHIFT) /* Medium drive strength */
#  define IOC_IOCFG_IOSTR_MAX      (3 << IOC_IOCFG_IOSTR_SHIFT) /* Maximum drive strength */

#define IOC_IOCFG_IOCURR_SHIFT     (10)      /* Bits 10-11: I/O current mode */
#define IOC_IOCFG_IOCURR_MASK      (3 << IOC_IOCFG_IOCURR_SHIFT)
#  define IOC_IOCFG_IOCURR_2MA     (0 << IOC_IOCFG_IOCURR_SHIFT) /* Extended-Current (EC) mode */
#  define IOC_IOCFG_IOCURR_4MA     (1 << IOC_IOCFG_IOCURR_SHIFT) /* High-Current (HC) mode */
#  define IOC_IOCFG_IOCURR_8MA     (2 << IOC_IOCFG_IOCURR_SHIFT) /* Low-Current (LC) mode */

#define IOC_IOCFG_SLEW_RED         (1 << 12) /* Bit 12:  Reduces output slew rate */
#define IOC_IOCFG_PULLCTL_SHIFT    (13)      /* Bits 13-14: Pull Control */
#define IOC_IOCFG_PULLCTL_MASK     (3 << IOC_IOCFG_PULLCTL_SHIFT)
#  define IOC_IOCFG_PULLCTL_DIS    (3 << IOC_IOCFG_PULLCTL_SHIFT) /* No pull */
#  define IOC_IOCFG_PULLCTL_DWN    (1 << IOC_IOCFG_PULLCTL_SHIFT) /* Pull down */
#  define IOC_IOCFG_PULLCTL_UP     (2 << IOC_IOCFG_PULLCTL_SHIFT) /* Pull up */

#define IOC_IOCFG_EDGEDET_SHIFT    (16)      /* Bits 16-17: Enable edge events generation */
#define IOC_IOCFG_EDGEDET_MASK     (3 << IOC_IOCFG_EDGEDET_SHIFT)
#  define IOC_IOCFG_EDGEDET_NONE   (0 << IOC_IOCFG_EDGEDET_SHIFT) /* No edge detection */
#  define IOC_IOCFG_EDGEDET_NEG    (1 << IOC_IOCFG_EDGEDET_SHIFT) /* Negative edge detection */
#  define IOC_IOCFG_EDGEDET_POS    (2 << IOC_IOCFG_EDGEDET_SHIFT) /* Positive edge detection */
#  define IOC_IOCFG_EDGEDET_BOTH   (3 << IOC_IOCFG_EDGEDET_SHIFT) /* Both edge detection */

#define IOC_IOCFG_EDGE_IRQEN       (1 << 18) /* Bit 18: Enable interrupt generation */
#define IOC_IOCFG_IOEV_AON_PROG0   (1 << 21) /* Bit 21: Input edge asserts AON_PROG0 */
#define IOC_IOCFG_IOEV_AON_PROG1   (1 << 22) /* Bit 22: Input edge asserts AON_PROG1 */
#define IOC_IOCFG_IOEV_AON_PROG2   (1 << 23) /* Bit 23: Input edge assert AON_PROG2 */
#define IOC_IOCFG_IOMODE_SHIFT     (24)      /* Bits 24-26:  I/O Mode */
#define IOC_IOCFG_IOMODE_MASK      (7 << IOC_IOCFG_IOMODE_SHIFT)
#  define IOC_IOCFG_IOMODE_NORMAL     (0 << IOC_IOCFG_IOMODE_SHIFT) /* Normal I/O */
#  define IOC_IOCFG_IOMODE_INV        (1 << IOC_IOCFG_IOMODE_SHIFT) /* Inverted I/O */
#  define IOC_IOCFG_IOMODE_OPENDR     (4 << IOC_IOCFG_IOMODE_SHIFT) /* Open drain */
#  define IOC_IOCFG_IOMODE_OPENDRINV  (5 << IOC_IOCFG_IOMODE_SHIFT) /* Open drain, inverted I/O */
#  define IOC_IOCFG_IOMODE_OPENSRC    (6 << IOC_IOCFG_IOMODE_SHIFT) /* Open source */
#  define IOC_IOCFG_IOMODE_OPENSRCINV (7 << IOC_IOCFG_IOMODE_SHIFT) /* Open source, inverted I/O */

#define IOC_IOCFG_WUCFG_SHIFT      (27)      /* Bits 27-28:  Wakeup Configuration */
#define IOC_IOCFG_WUCFG_MASK       (3 << IOC_IOCFG_WUCFG_SHIFT)
#  define IOC_IOCFG_WUCFG_NONE     (0 << IOC_IOCFG_WUCFG_SHIFT) /* 0, 1: Wakeup disabled */
#  define IOC_IOCFG_WUCFG_ENABLE   (2 << IOC_IOCFG_WUCFG_SHIFT) /* 2, 3: Wakeup enabled */
#  define IOC_IOCFG_WUCFG_WAKEUPL  (2 << IOC_IOCFG_WUCFG_SHIFT) /* 2: Wakeup on transition low */
#  define IOC_IOCFG_WUCFG_WEKUPH   (3 << IOC_IOCFG_WUCFG_SHIFT) /* 3: Wakeup on transition high */

#define IOC_IOCFG_IE               (1 << 29) /* Bit 29: Input enable */
#define IOC_IOCFG_HYSTEN           (1 << 30) /* Bit 30: Input hysteresis enable */

/* PORTID definitions */

#define IOC_IOCFG_PORTID_GPIO          0x00
#define IOC_IOCFG_PORTID_AON_CLK32K    0x07
#define IOC_IOCFG_PORTID_AUX_IO        0x08
#define IOC_IOCFG_PORTID_SSI0_RX       0x09
#define IOC_IOCFG_PORTID_SSI0_TX       0x0a
#define IOC_IOCFG_PORTID_SSI0_FSS      0x0b
#define IOC_IOCFG_PORTID_SSI0_CLK      0x0c
#define IOC_IOCFG_PORTID_I2C_MSSDA     0x0d
#define IOC_IOCFG_PORTID_I2C_MSSCL     0x0e
#define IOC_IOCFG_PORTID_UART0_RX      0x0f
#define IOC_IOCFG_PORTID_UART0_TX      0x10
#define IOC_IOCFG_PORTID_UART0_CTS     0x11
#define IOC_IOCFG_PORTID_UART0_RTS     0x12
#define IOC_IOCFG_PORTID_UART1_RX      0x13
#define IOC_IOCFG_PORTID_UART1_TX      0x14
#define IOC_IOCFG_PORTID_UART1_CTS     0x15
#define IOC_IOCFG_PORTID_UART1_RTS     0x16
#define IOC_IOCFG_PORTID_PORT_EVENT0   0x17
#define IOC_IOCFG_PORTID_PORT_EVENT1   0x18
#define IOC_IOCFG_PORTID_PORT_EVENT2   0x19
#define IOC_IOCFG_PORTID_PORT_EVENT3   0x1a
#define IOC_IOCFG_PORTID_PORT_EVENT4   0x1b
#define IOC_IOCFG_PORTID_PORT_EVENT5   0x1c
#define IOC_IOCFG_PORTID_PORT_EVENT6   0x1d
#define IOC_IOCFG_PORTID_PORT_EVENT7   0x1e
#define IOC_IOCFG_PORTID_CPU_SWV       0x20
#define IOC_IOCFG_PORTID_SSI1_RX       0x21
#define IOC_IOCFG_PORTID_SSI1_TX       0x22
#define IOC_IOCFG_PORTID_SSI1_FSS      0x23
#define IOC_IOCFG_PORTID_SSI1_CLK      0x24
#define IOC_IOCFG_PORTID_I2S_AD0       0x25
#define IOC_IOCFG_PORTID_I2S_AD1       0x26
#define IOC_IOCFG_PORTID_I2S_WCLK      0x27
#define IOC_IOCFG_PORTID_I2S_BCLK      0x28
#define IOC_IOCFG_PORTID_I2S_MCLK      0x29
#define IOC_IOCFG_PORTID_RFC_TRC       0x2e
#define IOC_IOCFG_PORTID_RFC_GPO0      0x2f
#define IOC_IOCFG_PORTID_RFC_GPO1      0x30
#define IOC_IOCFG_PORTID_RFC_GPO2      0x31
#define IOC_IOCFG_PORTID_RFC_GPO3      0x32
#define IOC_IOCFG_PORTID_RFC_GPI0      0x33
#define IOC_IOCFG_PORTID_RFC_GPI1      0x34
#define IOC_IOCFG_PORTID_RFC_SMI_DLOUT 0x35
#define IOC_IOCFG_PORTID_RFC_SMI_DLIN  0x36
#define IOC_IOCFG_PORTID_RFC_SMI_CLOUT 0x37
#define IOC_IOCFG_PORTID_RFC_SMI_CLIN  0x38

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_IOC_H */
