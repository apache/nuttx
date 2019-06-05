/************************************************************************************
 * arch/arm/src/am335x/hardware/am335x_gpio.h
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_GPIO_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/am335x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define GPIO0                     0      /* Port 0 index */
#define GPIO1                     1      /* Port 1 index */
#define GPIO2                     2      /* Port 2 index */
#define GPIO3                     3      /* Port 3 index */

#define AM335X_GPIO_NPORTS        4      /* Seven total ports */
#define AM335X_GPIO_NPINS        32      /* Up to 32 pins per port */

/* GPIO Register Offsets ************************************************************/

#define AM335X_GPIO_REV_OFFSET   0x0000  /* Module Revision Register */
#define AM335X_GPIO_SCR_OFFSET   0x0010  /* System Configuration Register */
#define AM335X_GPIO_EOIR_OFFSET  0x0020  /* End of Event Register */
#define AM335X_GPIO_ISRR0_OFFSET 0x0024  /* Interrupt Status Raw 0 Register */
#define AM335X_GPIO_ISRR1_OFFSET 0x0028  /* Interrupt Status Raw 0 Register */
#define AM335X_GPIO_ISR0_OFFSET  0x002C  /* Interrupt Status 0 Register */
#define AM335X_GPIO_ISR1_OFFSET  0x0030  /* Interrupt Status 1 Register */
#define AM335X_GPIO_ISSR0_OFFSET 0x0034  /* Interrupt Status Set 0 Register */
#define AM335X_GPIO_ISSR1_OFFSET 0x0038  /* Interrupt Status Set 1 Register */
#define AM335X_GPIO_ISCR0_OFFSET 0x003C  /* Interrupt Status Clear 0 Register */
#define AM335X_GPIO_ISCR1_OFFSET 0x0040  /* Interrupt Status Clear 1 Register */
#define AM335X_GPIO_IWER0_OFFSET 0x0044  /* Interrupt Wake-up Enable 0 Register */
#define AM335X_GPIO_IWER1_OFFSET 0x0048  /* Interrupt Wake-up Enable 1 Register */
#define AM335X_GPIO_SSR_OFFSET   0x0114  /* System Status Register */
#define AM335X_GPIO_CTRL_OFFSET  0x0130  /* Control Register */
#define AM335X_GPIO_OER_OFFSET   0x0134  /* Output Enable Register */
#define AM335X_GPIO_DIR_OFFSET   0x0138  /* Data Input Register */
#define AM335X_GPIO_DOR_OFFSET   0x013C  /* Data Output Register */
#define AM335X_GPIO_LDR0_OFFSET  0x0140  /* Level Detect 0 Register */
#define AM335X_GPIO_LDR1_OFFSET  0x0144  /* Level Detect 1 Register */
#define AM335X_GPIO_RDR_OFFSET   0x0148  /* Rising-edge Detection Register */
#define AM335X_GPIO_FDR_OFFSET   0x014C  /* Falling-edge Detection Register */
#define AM335X_GPIO_DER_OFFSET   0x0150  /* Debouncing Enable Register */
#define AM335X_GPIO_DTR_OFFSET   0x0154  /* Debouncing Time Register */
#define AM335X_GPIO_CDOR_OFFSET  0x0190  /* Clear Data Output Register */
#define AM335X_GPIO_SDOR_OFFSET  0x0194  /* Set Data Output Register */

/* GPIO Register Addresses **********************************************************/

#define AM335X_GPIO_SCR(v)       ((v) + AM335X_GPIO_SCR_OFFSET)
#define AM335X_GPIO_EOIR(v)      ((v) + AM335X_GPIO_EOIR_OFFSET)
#define AM335X_GPIO_ISRR0(v)     ((v) + AM335X_GPIO_ISRR0_OFFSET)
#define AM335X_GPIO_ISRR1(v)     ((v) + AM335X_GPIO_ISRR1_OFFSET)
#define AM335X_GPIO_ISR0(v)      ((v) + AM335X_GPIO_ISR0_OFFSET)
#define AM335X_GPIO_ISR1(v)      ((v) + AM335X_GPIO_ISR1_OFFSET)
#define AM335X_GPIO_ISSR0(v)     ((v) + AM335X_GPIO_ISSR0_OFFSET)
#define AM335X_GPIO_ISSR1(v)     ((v) + AM335X_GPIO_ISSR1_OFFSET)
#define AM335X_GPIO_ISCR0(v)     ((v) + AM335X_GPIO_ISCR0_OFFSET)
#define AM335X_GPIO_ISCR1(v)     ((v) + AM335X_GPIO_ISCR1_OFFSET)
#define AM335X_GPIO_IWER0(v)     ((v) + AM335X_GPIO_IWER0_OFFSET)
#define AM335X_GPIO_IWER1(v)     ((v) + AM335X_GPIO_IWER1_OFFSET)
#define AM335X_GPIO_SSR(v)       ((v) + AM335X_GPIO_SSR_OFFSET)
#define AM335X_GPIO_CTRL(v)      ((v) + AM335X_GPIO_CTRL_OFFSET)
#define AM335X_GPIO_OER(v)       ((v) + AM335X_GPIO_OER_OFFSET)
#define AM335X_GPIO_DIR(v)       ((v) + AM335X_GPIO_DIR_OFFSET)
#define AM335X_GPIO_DOR(v)       ((v) + AM335X_GPIO_DOR_OFFSET)
#define AM335X_GPIO_LDR0(v)      ((v) + AM335X_GPIO_LDR0_OFFSET)
#define AM335X_GPIO_LDR1(v)      ((v) + AM335X_GPIO_LDR1_OFFSET)
#define AM335X_GPIO_RDR(v)       ((v) + AM335X_GPIO_RDR_OFFSET)
#define AM335X_GPIO_FDR(v)       ((v) + AM335X_GPIO_FDR_OFFSET)
#define AM335X_GPIO_DER(v)       ((v) + AM335X_GPIO_DER_OFFSET)
#define AM335X_GPIO_DTR(v)       ((v) + AM335X_GPIO_DTR_OFFSET)
#define AM335X_GPIO_CDOR(v)      ((v) + AM335X_GPIO_CDOR_OFFSET)
#define AM335X_GPIO_SDOR(v)      ((v) + AM335X_GPIO_SDOR_OFFSET)

#define AM335X_GPIO0_SCR         (AM335X_GPIO0_VADDR + AM335X_GPIO_SCR_OFFSET)
#define AM335X_GPIO0_EOIR        (AM335X_GPIO0_VADDR + AM335X_GPIO_EOIR_OFFSET)
#define AM335X_GPIO0_ISRR0       (AM335X_GPIO0_VADDR + AM335X_GPIO_ISRR0_OFFSET)
#define AM335X_GPIO0_ISRR1       (AM335X_GPIO0_VADDR + AM335X_GPIO_ISRR1_OFFSET)
#define AM335X_GPIO0_ISR0        (AM335X_GPIO0_VADDR + AM335X_GPIO_ISR0_OFFSET)
#define AM335X_GPIO0_ISR1        (AM335X_GPIO0_VADDR + AM335X_GPIO_ISR1_OFFSET)
#define AM335X_GPIO0_ISSR0       (AM335X_GPIO0_VADDR + AM335X_GPIO_ISSR0_OFFSET)
#define AM335X_GPIO0_ISSR1       (AM335X_GPIO0_VADDR + AM335X_GPIO_ISSR1_OFFSET)
#define AM335X_GPIO0_ISCR0       (AM335X_GPIO0_VADDR + AM335X_GPIO_ISCR0_OFFSET)
#define AM335X_GPIO0_ISCR1       (AM335X_GPIO0_VADDR + AM335X_GPIO_ISCR1_OFFSET)
#define AM335X_GPIO0_IWER0       (AM335X_GPIO0_VADDR + AM335X_GPIO_IWER0_OFFSET)
#define AM335X_GPIO0_IWER1       (AM335X_GPIO0_VADDR + AM335X_GPIO_IWER1_OFFSET)
#define AM335X_GPIO0_SSR         (AM335X_GPIO0_VADDR + AM335X_GPIO_SSR_OFFSET)
#define AM335X_GPIO0_CTRL        (AM335X_GPIO0_VADDR + AM335X_GPIO_CTRL_OFFSET)
#define AM335X_GPIO0_OER         (AM335X_GPIO0_VADDR + AM335X_GPIO_OER_OFFSET)
#define AM335X_GPIO0_DIR         (AM335X_GPIO0_VADDR + AM335X_GPIO_DIR_OFFSET)
#define AM335X_GPIO0_DOR         (AM335X_GPIO0_VADDR + AM335X_GPIO_DOR_OFFSET)
#define AM335X_GPIO0_LDR0        (AM335X_GPIO0_VADDR + AM335X_GPIO_LDR0_OFFSET)
#define AM335X_GPIO0_LDR1        (AM335X_GPIO0_VADDR + AM335X_GPIO_LDR1_OFFSET)
#define AM335X_GPIO0_RDR         (AM335X_GPIO0_VADDR + AM335X_GPIO_RDR_OFFSET)
#define AM335X_GPIO0_FDR         (AM335X_GPIO0_VADDR + AM335X_GPIO_FDR_OFFSET)
#define AM335X_GPIO0_DER         (AM335X_GPIO0_VADDR + AM335X_GPIO_DER_OFFSET)
#define AM335X_GPIO0_DTR         (AM335X_GPIO0_VADDR + AM335X_GPIO_DTR_OFFSET)
#define AM335X_GPIO0_CDOR        (AM335X_GPIO0_VADDR + AM335X_GPIO_CDOR_OFFSET)
#define AM335X_GPIO0_SDOR        (AM335X_GPIO0_VADDR + AM335X_GPIO_SDOR_OFFSET)

#define AM335X_GPIO1_SCR         (AM335X_GPIO1_VADDR + AM335X_GPIO_SCR_OFFSET)
#define AM335X_GPIO1_EOIR        (AM335X_GPIO1_VADDR + AM335X_GPIO_EOIR_OFFSET)
#define AM335X_GPIO1_ISRR0       (AM335X_GPIO1_VADDR + AM335X_GPIO_ISRR0_OFFSET)
#define AM335X_GPIO1_ISRR1       (AM335X_GPIO1_VADDR + AM335X_GPIO_ISRR1_OFFSET)
#define AM335X_GPIO1_ISR0        (AM335X_GPIO1_VADDR + AM335X_GPIO_ISR0_OFFSET)
#define AM335X_GPIO1_ISR1        (AM335X_GPIO1_VADDR + AM335X_GPIO_ISR1_OFFSET)
#define AM335X_GPIO1_ISSR0       (AM335X_GPIO1_VADDR + AM335X_GPIO_ISSR0_OFFSET)
#define AM335X_GPIO1_ISSR1       (AM335X_GPIO1_VADDR + AM335X_GPIO_ISSR1_OFFSET)
#define AM335X_GPIO1_ISCR0       (AM335X_GPIO1_VADDR + AM335X_GPIO_ISCR0_OFFSET)
#define AM335X_GPIO1_ISCR1       (AM335X_GPIO1_VADDR + AM335X_GPIO_ISCR1_OFFSET)
#define AM335X_GPIO1_IWER0       (AM335X_GPIO1_VADDR + AM335X_GPIO_IWER0_OFFSET)
#define AM335X_GPIO1_IWER1       (AM335X_GPIO1_VADDR + AM335X_GPIO_IWER1_OFFSET)
#define AM335X_GPIO1_SSR         (AM335X_GPIO1_VADDR + AM335X_GPIO_SSR_OFFSET)
#define AM335X_GPIO1_CTRL        (AM335X_GPIO1_VADDR + AM335X_GPIO_CTRL_OFFSET)
#define AM335X_GPIO1_OER         (AM335X_GPIO1_VADDR + AM335X_GPIO_OER_OFFSET)
#define AM335X_GPIO1_DIR         (AM335X_GPIO1_VADDR + AM335X_GPIO_DIR_OFFSET)
#define AM335X_GPIO1_DOR         (AM335X_GPIO1_VADDR + AM335X_GPIO_DOR_OFFSET)
#define AM335X_GPIO1_LDR0        (AM335X_GPIO1_VADDR + AM335X_GPIO_LDR0_OFFSET)
#define AM335X_GPIO1_LDR1        (AM335X_GPIO1_VADDR + AM335X_GPIO_LDR1_OFFSET)
#define AM335X_GPIO1_RDR         (AM335X_GPIO1_VADDR + AM335X_GPIO_RDR_OFFSET)
#define AM335X_GPIO1_FDR         (AM335X_GPIO1_VADDR + AM335X_GPIO_FDR_OFFSET)
#define AM335X_GPIO1_DER         (AM335X_GPIO1_VADDR + AM335X_GPIO_DER_OFFSET)
#define AM335X_GPIO1_DTR         (AM335X_GPIO1_VADDR + AM335X_GPIO_DTR_OFFSET)
#define AM335X_GPIO1_CDOR        (AM335X_GPIO1_VADDR + AM335X_GPIO_CDOR_OFFSET)
#define AM335X_GPIO1_SDOR        (AM335X_GPIO1_VADDR + AM335X_GPIO_SDOR_OFFSET)

#define AM335X_GPIO2_SCR         (AM335X_GPIO2_VADDR + AM335X_GPIO_SCR_OFFSET)
#define AM335X_GPIO2_EOIR        (AM335X_GPIO2_VADDR + AM335X_GPIO_EOIR_OFFSET)
#define AM335X_GPIO2_ISRR0       (AM335X_GPIO2_VADDR + AM335X_GPIO_ISRR0_OFFSET)
#define AM335X_GPIO2_ISRR1       (AM335X_GPIO2_VADDR + AM335X_GPIO_ISRR1_OFFSET)
#define AM335X_GPIO2_ISR0        (AM335X_GPIO2_VADDR + AM335X_GPIO_ISR0_OFFSET)
#define AM335X_GPIO2_ISR1        (AM335X_GPIO2_VADDR + AM335X_GPIO_ISR1_OFFSET)
#define AM335X_GPIO2_ISSR0       (AM335X_GPIO2_VADDR + AM335X_GPIO_ISSR0_OFFSET)
#define AM335X_GPIO2_ISSR1       (AM335X_GPIO2_VADDR + AM335X_GPIO_ISSR1_OFFSET)
#define AM335X_GPIO2_ISCR0       (AM335X_GPIO2_VADDR + AM335X_GPIO_ISCR0_OFFSET)
#define AM335X_GPIO2_ISCR1       (AM335X_GPIO2_VADDR + AM335X_GPIO_ISCR1_OFFSET)
#define AM335X_GPIO2_IWER0       (AM335X_GPIO2_VADDR + AM335X_GPIO_IWER0_OFFSET)
#define AM335X_GPIO2_IWER1       (AM335X_GPIO2_VADDR + AM335X_GPIO_IWER1_OFFSET)
#define AM335X_GPIO2_SSR         (AM335X_GPIO2_VADDR + AM335X_GPIO_SSR_OFFSET)
#define AM335X_GPIO2_CTRL        (AM335X_GPIO2_VADDR + AM335X_GPIO_CTRL_OFFSET)
#define AM335X_GPIO2_OER         (AM335X_GPIO2_VADDR + AM335X_GPIO_OER_OFFSET)
#define AM335X_GPIO2_DIR         (AM335X_GPIO2_VADDR + AM335X_GPIO_DIR_OFFSET)
#define AM335X_GPIO2_DOR         (AM335X_GPIO2_VADDR + AM335X_GPIO_DOR_OFFSET)
#define AM335X_GPIO2_LDR0        (AM335X_GPIO2_VADDR + AM335X_GPIO_LDR0_OFFSET)
#define AM335X_GPIO2_LDR1        (AM335X_GPIO2_VADDR + AM335X_GPIO_LDR1_OFFSET)
#define AM335X_GPIO2_RDR         (AM335X_GPIO2_VADDR + AM335X_GPIO_RDR_OFFSET)
#define AM335X_GPIO2_FDR         (AM335X_GPIO2_VADDR + AM335X_GPIO_FDR_OFFSET)
#define AM335X_GPIO2_DER         (AM335X_GPIO2_VADDR + AM335X_GPIO_DER_OFFSET)
#define AM335X_GPIO2_DTR         (AM335X_GPIO2_VADDR + AM335X_GPIO_DTR_OFFSET)
#define AM335X_GPIO2_CDOR        (AM335X_GPIO2_VADDR + AM335X_GPIO_CDOR_OFFSET)
#define AM335X_GPIO2_SDOR        (AM335X_GPIO2_VADDR + AM335X_GPIO_SDOR_OFFSET)

#define AM335X_GPIO3_SCR         (AM335X_GPIO3_VADDR + AM335X_GPIO_SCR_OFFSET)
#define AM335X_GPIO3_EOIR        (AM335X_GPIO3_VADDR + AM335X_GPIO_EOIR_OFFSET)
#define AM335X_GPIO3_ISRR0       (AM335X_GPIO3_VADDR + AM335X_GPIO_ISRR0_OFFSET)
#define AM335X_GPIO3_ISRR1       (AM335X_GPIO3_VADDR + AM335X_GPIO_ISRR1_OFFSET)
#define AM335X_GPIO3_ISR0        (AM335X_GPIO3_VADDR + AM335X_GPIO_ISR0_OFFSET)
#define AM335X_GPIO3_ISR1        (AM335X_GPIO3_VADDR + AM335X_GPIO_ISR1_OFFSET)
#define AM335X_GPIO3_ISSR0       (AM335X_GPIO3_VADDR + AM335X_GPIO_ISSR0_OFFSET)
#define AM335X_GPIO3_ISSR1       (AM335X_GPIO3_VADDR + AM335X_GPIO_ISSR1_OFFSET)
#define AM335X_GPIO3_ISCR0       (AM335X_GPIO3_VADDR + AM335X_GPIO_ISCR0_OFFSET)
#define AM335X_GPIO3_ISCR1       (AM335X_GPIO3_VADDR + AM335X_GPIO_ISCR1_OFFSET)
#define AM335X_GPIO3_IWER0       (AM335X_GPIO3_VADDR + AM335X_GPIO_IWER0_OFFSET)
#define AM335X_GPIO3_IWER1       (AM335X_GPIO3_VADDR + AM335X_GPIO_IWER1_OFFSET)
#define AM335X_GPIO3_SSR         (AM335X_GPIO3_VADDR + AM335X_GPIO_SSR_OFFSET)
#define AM335X_GPIO3_CTRL        (AM335X_GPIO3_VADDR + AM335X_GPIO_CTRL_OFFSET)
#define AM335X_GPIO3_OER         (AM335X_GPIO3_VADDR + AM335X_GPIO_OER_OFFSET)
#define AM335X_GPIO3_DIR         (AM335X_GPIO3_VADDR + AM335X_GPIO_DIR_OFFSET)
#define AM335X_GPIO3_DOR         (AM335X_GPIO3_VADDR + AM335X_GPIO_DOR_OFFSET)
#define AM335X_GPIO3_LDR0        (AM335X_GPIO3_VADDR + AM335X_GPIO_LDR0_OFFSET)
#define AM335X_GPIO3_LDR1        (AM335X_GPIO3_VADDR + AM335X_GPIO_LDR1_OFFSET)
#define AM335X_GPIO3_RDR         (AM335X_GPIO3_VADDR + AM335X_GPIO_RDR_OFFSET)
#define AM335X_GPIO3_FDR         (AM335X_GPIO3_VADDR + AM335X_GPIO_FDR_OFFSET)
#define AM335X_GPIO3_DER         (AM335X_GPIO3_VADDR + AM335X_GPIO_DER_OFFSET)
#define AM335X_GPIO3_DTR         (AM335X_GPIO3_VADDR + AM335X_GPIO_DTR_OFFSET)
#define AM335X_GPIO3_CDOR        (AM335X_GPIO3_VADDR + AM335X_GPIO_CDOR_OFFSET)
#define AM335X_GPIO3_SDOR        (AM335X_GPIO3_VADDR + AM335X_GPIO_SDOR_OFFSET)

/* GPIO Register Bit Definitions ****************************************************/

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)              (1 << ((n) & 0x1f)) /* Bit n: Pin n, n=0-31 */

/* GPIO interrupt configuration register 1/2 */

#define GPIO_RDR_REN(n)          (1 << ((n) & 0x1f)) /* Interrupt is rising-edge sensitive */
#define GPIO_FDR_FEN(n)          (1 << ((n) & 0x1f))
#define GPIO_LDR0_HEN(n)         (1 << ((n) & 0x1f))
#define GPIO_LDR1_LEN(n)         (1 << ((n) & 0x1f))

#define GPIO_ICR_INDEX(n)        (((n) >> 4) & 1)
#define GPIO_ICR_OFFSET(n)       (GPIO_ICR1_OFFSET + (GPIO_ICR_INDEX(n) << 2))

#define GPIO_ICR_LOWLEVEL        0          /* Interrupt is low-level sensitive */
#define GPIO_ICR_HIGHLEVEL       1          /* Interrupt is high-level sensitive */
#define GPIO_ICR_RISINGEDGE      2          /* Interrupt is rising-edge sensitive */
#define GPIO_ICR_FALLINGEDGE     3          /* Interrupt is falling-edge sensitive */

#define GPIO_ICR_SHIFT(n)        (((n) & 15) << 1)
#define GPIO_ICR_MASK(n)         (3 << GPIO_ICR_SHIFT(n))
#define GPIO_ICR(i,n)            ((uint32_t)(n) << GPIO_ICR_SHIFT(n))

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_GPIO_H */
