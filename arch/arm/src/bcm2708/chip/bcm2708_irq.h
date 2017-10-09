/************************************************************************************
 * arch/arm/src/bcm2708/chip/bcm2708_irq.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_BCM2708_CHIP_BCM2780_IRQ_H
#define __ARCH_ARM_SRC_BCM2708_CHIP_BCM2780_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/bcm2708/irq.h>

#include "chip/bcm2708_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IRQ Register Offsets ************************************************************/

#define BCM_IRQ_BPR_OFFSET        0x0200  /* IRQ basic pending register */
#define BCM_IRQ_IPR1_OFSET        0x0204  /* IRQ pending 1 register */
#define BCM_IRQ_IPR2_OFFSET       0x0208  /* IRQ pending 2 register */
#define BCM_IRQ_FIQC_OFFSET       0x020c  /* FIQ control register */
#define BCM_IRQ_EIR1_OFFSET       0x0210  /* Enable IRQs 1 register */
#define BCM_IRQ_EIR2_OFFSET       0x0214  /* Enable IRQs 2 register */
#define BCM_IRQ_EBR_OFFSET        0x0218  /* Enable Basic IRQs register */
#define BCM_IRQ_DIR1_OFFSET       0x021c  /* Disable IRQs 1 register */
#define BCM_IRQ_DIR2_OFFSET       0x0220  /* Disable IRQs 2 register */
#define BCM_IRQ_DBR_OFFSET        0x0224  /* Disable Basic IRQs */

/* IRQ Register Addresses **********************************************************/

#define BCM_IRQ_BPR               (BCM_IRQ_OFFSET + BCM_IRQ_BPR_OFFSET)
#define BCM_IRQ_IPR1              (BCM_IRQ_OFFSET + BCM_IRQ_IPR1_OFSET)
#define BCM_IRQ_IPR2              (BCM_IRQ_OFFSET + BCM_IRQ_IPR2_OFFSET)
#define BCM_IRQ_FIQC              (BCM_IRQ_OFFSET + BCM_IRQ_FIQC_OFFSET)
#define BCM_IRQ_EIR1              (BCM_IRQ_OFFSET + BCM_IRQ_EIR1_OFFSET)
#define BCM_IRQ_EIR2              (BCM_IRQ_OFFSET + BCM_IRQ_EIR2_OFFSET)
#define BCM_IRQ_EBR               (BCM_IRQ_OFFSET + BCM_IRQ_EBR_OFFSET)
#define BCM_IRQ_DIR1              (BCM_IRQ_OFFSET + BCM_IRQ_DIR1_OFFSET)
#define BCM_IRQ_DIR2              (BCM_IRQ_OFFSET + BCM_IRQ_DIR2_OFFSET)
#define BCM_IRQ_DBR               (BCM_IRQ_OFFSET + BCM_IRQ_DBR_OFFSET)

/* IRQ Register Bit Definitions ****************************************************/

/* IRQ basic pending, IRQ pending 1, and IRQ pending 2 registers:
 * (See arch/arm/include/bcm2708/irq.h)
 */

#define IRQ_BPR_ALLINTS           BPR_BIT_IRQMASK
#define IRQ_IPR1_ALLINTS          IPR1_BIT_IRQMASK
#define IRQ_IPR2_ALLINTS          IPR2_BIT_IRQMASK

/* The following provide interrupt numbers that may be used with the FIQC register.
 */

#define IPR1_IRQ_AUX_INT          29        /* Aux interrupt */
#define IPR2_IRQ_I2C_SPI_SLV      43        /* I2C/SPI slave */
#define IPR2_IRQ_PWA0             45        /* PWA0 */
#define IPR2_IRQ_PWA1             46        /* PWA1 */
#define IPR2_IRQ_SMI              48        /* SMI */
#define IPR2_IRQ_GPIO0            49        /* GPIO interrupt 0 */
#define IPR2_IRQ_GPIO1            50        /* GPIO interrupt 1 */
#define IPR2_IRQ_GPIO2            51        /* GPIO interrupt 2 */
#define IPR2_IRQ_GPIO3            52        /* GPIO interrupt 3 */
#define IPR2_IRQ_I2C              53        /* I2C interrupt */
#define IPR2_IRQ_SPI              54        /* SPI interrupt */
#define IPR2_IRQ_PCM              55        /* PCM interrupt */
#define IPR2_IRQ_UART             57        /* UART interrupt */
#define BPR_IRQ_ARM_TIMER         64        /* ARM Timer interrupt */
#define BPR_IRQ_ARM_MAILBOX       65        /* ARM Mailbox interrupt */
#define BPR_IRQ_ARM_DOORBELL_0    66        /* ARM Doorbell 0 interrupt */
#define BPR_IRQ_ARM_DOORBELL_1    67        /* ARM Doorbell 1 interrupt */
#define BPR_IRQ_GPU0_HALTED       68        /* GPU0 Halted interrupt (or GPU1) */
#define BPR_IRQ_GPU1_HALTED       69        /* GPU1 Halted interrupt */
#define BPR_IRQ_ILLEGAL_ACCESS_1  70        /* Illegal access type-1 interrupt */
#define BPR_IRQ_ILLEGAL_ACCESS_0  71        /* Illegal access type-0 interrupt */

/* FIQ control register */

#define IRQ_FIQC_SOURCE_SHIFT     0        /* Bits 0-6: FIQ source */
#define IRQ_FIQC_SOURCE_MASK     (0x3f << IRQ_FIQC_SOURCE_SHIFT)
#  define IRQ_FIQC_SOURCE(n)     (0x3f << IRQ_FIQC_SOURCE_SHIFT)
#define IRQ_FIQC_ENABLE          (1 << 7)  /* Bit 7: FIQ enable */

/* IRQ basic enable/disable, IRQ enable/disable 1, and IRQ enable/disable 2
 * registers:  Same as the interrupt pending registers except that only bits
 * 0-7 of the BPR are available (see arch/arm/include/bcm2708/irq.h).
 */

#define IRQ_EBR_ALLINTS          0x000000ff
#define IRQ_EIR1_ALLINTS         IPR1_BIT_IRQMASK
#define IRQ_EIR2_ALLINTS         IPR2_BIT_IRQMASK

#define IRQ_DBR_ALLINTS          0x000000ff
#define IRQ_DIR1_ALLINTS         IPR1_BIT_IRQMASK
#define IRQ_DIR2_ALLINTS         IPR2_BIT_IRQMASK

#endif /* __ARCH_ARM_SRC_BCM2708_CHIP_BCM2780_IRQ_H */

