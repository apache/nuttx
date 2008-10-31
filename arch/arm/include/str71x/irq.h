/************************************************************************************
 * arch/arm/include/str71x/irq.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STR71X_IRQ_H
#define __ARCH_ARM_INCLUDE_STR71X_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ channels */

#define STR71X_IRQ_T0TIMI     (0)
#define STR71X_IRQ_FLASH      (1)
#define STR71X_IRQ_RCCU       (2)
#define STR71X_IRQ_RTC        (3)
#define STR71X_IRQ_WDG        (4)
#define STR71X_IRQ_XTI        (5)
#define STR71X_IRQ_USBHP      (6)
#define STR71X_IRQ_I2C0ITERR  (7)
#define STR71X_IRQ_I2C1ITERR  (8)
#define STR71X_IRQ_UART0      (9)
#define STR71X_IRQ_UART1     (10)
#define STR71X_IRQ_UART2     (11)
#define STR71X_IRQ_UART3     (12)
#define STR71X_IRQ_SPI0      (13)
#define STR71X_IRQ_SPI1      (14)
#define STR71X_IRQ_I2C0      (15)
#define STR71X_IRQ_I2C1      (16)
#define STR71X_IRQ_CAN       (17)
#define STR71X_IRQ_ADC       (18)
#define STR71X_IRQ_T1TIMI    (19)
#define STR71X_IRQ_T2TIMI    (20)
#define STR71X_IRQ_T3TIMI    (21)
#define STR71X_IRQ_HDLC      (25)
#define STR71X_IRQ_USBLP     (26)
#define STR71X_IRQ_T0TOI     (29)
#define STR71X_IRQ_T0OC1     (30)
#define STR71X_IRQ_T0OC2     (31)

#define LPC214X_IRQ_SYSTIMER  STR71X_IRQ_T0TIMI
#define NR_IRQS               32

/* FIQ channels */

#define STR71X_FIQ_T0TIMI     (0X00000001)
#define STR71X_FIQ_WDG        (0X00000002)
#define STR71X_FIQ_WDGT0TIMIS (0X00000003)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

EXTERN int up_irqpriority(int irq, ubyte priority); /* Set interrupt priority (0-15) */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_LPC214X_IRQ_H */

