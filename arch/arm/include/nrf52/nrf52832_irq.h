/****************************************************************************************************
 * arch/arm/include/nrf52/nrf52_irq.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_NRF52_NRF52_IRQ_H
#define __ARCH_ARM_INCLUDE_NRF52_NRF52_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Cortex-M4 External interrupts (vectors >= 16) */

#define NRF52_IRQ_POWER_CLOCK   (NRF52_IRQ_EXTINT+0)   /* VOD Windowed watchdog timer, Brownout detect */
#define NRF52_IRQ_RADIO         (NRF52_IRQ_EXTINT+1)   /* DMA controller */
#define NRF52_IRQ_UART0         (NRF52_IRQ_EXTINT+2)   /* GPIO group 0 */
#define NRF52_IRQ_SPI_TWI_0     (NRF52_IRQ_EXTINT+3)   /* GPIO group 1 */
#define NRF52_IRQ_SPI_TWI_1     (NRF52_IRQ_EXTINT+4)   /* Pin interrupt 0 or pattern match engine slice 0 */
#define NRF52_IRQ_NFCT          (NRF52_IRQ_EXTINT+5)   /* Pin interrupt 1 or pattern match engine slice 1 */
#define NRF52_IRQ_GPIOTE        (NRF52_IRQ_EXTINT+6)   /* Pin interrupt 2 or pattern match engine slice 2 */
#define NRF52_IRQ_SAADC         (NRF52_IRQ_EXTINT+7)   /* Pin interrupt 3 or pattern match engine slice 3 */
#define NRF52_IRQ_TIMER0        (NRF52_IRQ_EXTINT+8)   /* Micro-tick Timer */
#define NRF52_IRQ_TIMER1        (NRF52_IRQ_EXTINT+9)   /* Multi-rate timer */
#define NRF52_IRQ_TIMER2        (NRF52_IRQ_EXTINT+10)  /* Standard counter/timer CTIMER0 */
#define NRF52_IRQ_RTC0          (NRF52_IRQ_EXTINT+11)  /* Standard counter/timer CTIMER1 */
#define NRF52_IRQ_TEMP          (NRF52_IRQ_EXTINT+12)  /* SCTimer/PWM0 */
#define NRF52_IRQ_RNG           (NRF52_IRQ_EXTINT+12)  /* SCTimer/PWM0 */
#define NRF52_IRQ_ECB           (NRF52_IRQ_EXTINT+13)  /* CTIMER3 Standard counter/timer CTIMER3 */
#define NRF52_IRQ_CCM_AAR       (NRF52_IRQ_EXTINT+14)  /* Flexcomm Interface 0 (USART, SPI, I2C) */
#define NRF52_IRQ_WDT           (NRF52_IRQ_EXTINT+15)  /* Flexcomm Interface 1 (USART, SPI, I2C) */
#define NRF52_IRQ_RTC1          (NRF52_IRQ_EXTINT+16)  /* Flexcomm Interface 2 (USART, SPI, I2C) */
#define NRF52_IRQ_QDEC          (NRF52_IRQ_EXTINT+17)  /* Flexcomm Interface 3 (USART, SPI, I2C) */
#define NRF52_IRQ_COMP_LPCOMP   (NRF52_IRQ_EXTINT+18)  /* Flexcomm Interface 4 (USART, SPI, I2C) */
#define NRF52_IRQ_SWI0_EGU0     (NRF52_IRQ_EXTINT+19)  /* Flexcomm Interface 5 (USART, SPI, I2C) */
#define NRF52_IRQ_SWI1_EGU1     (NRF52_IRQ_EXTINT+20)  /* Flexcomm Interface 6 (USART, SPI, I2C, I2S) */
#define NRF52_IRQ_SWI2_EGU2     (NRF52_IRQ_EXTINT+21)  /* Flexcomm Interface 7 (USART, SPI, I2C, I2S) */
#define NRF52_IRQ_SWI3_EGU3     (NRF52_IRQ_EXTINT+22)  /* ADC0 sequence A completion */
#define NRF52_IRQ_SWI4_EGU4     (NRF52_IRQ_EXTINT+23)  /* ADC0 sequence B completion */
#define NRF52_IRQ_SWI5_EGU5     (NRF52_IRQ_EXTINT+24)  /* ADC0 threshold compare and error */
#define NRF52_IRQ_TIMER3        (NRF52_IRQ_EXTINT+25)  /* Digital microphone and audio subsystem */
#define NRF52_IRQ_TIMER4        (NRF52_IRQ_EXTINT+26)  /* Hardware Voice Activity Detection */
#define NRF52_IRQ_PWM0          (NRF52_IRQ_EXTINT+27)  /* USB0 Activity Interrupt */
#define NRF52_IRQ_PDM           (NRF52_IRQ_EXTINT+28)  /* USB0 host and device */
                                                       /* 29-30 Reserved */
#define NRF52_IRQ_MWU           (NRF52_IRQ_EXTINT+31)  /* Pin interrupt 4 or pattern match engine slice 4 */
#define NRF52_IRQ_PWM1          (NRF52_IRQ_EXTINT+32)  /* Pin interrupt 5 or pattern match engine slice 5 */
#define NRF52_IRQ_PWM2          (NRF52_IRQ_EXTINT+33)  /* Pin interrupt 6 or pattern match engine slice 6 */
#define NRF52_IRQ_SPI2          (NRF52_IRQ_EXTINT+34)  /* Pin interrupt 7 or pattern match engine slice 7 */
#define NRF52_IRQ_RTC2          (NRF52_IRQ_EXTINT+35)  /* Standard counter/timer CTIMER2 */
#define NRF52_IRQ_I2S           (NRF52_IRQ_EXTINT+36)  /* Standard counter/timer CTIMER4 */
#define NRF52_IRQ_FPU           (NRF52_IRQ_EXTINT+37)  /* Repetitive Interrupt Timer */

#define NRF52_IRQ_NEXTINT       (38)
#define NRF52_IRQ_NIRQS         (NRF52_IRQ_EXTINT+NRF52_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_VECTORS              NRF52_IRQ_NIRQS
#define NR_IRQS                 NRF52_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_NRF52_NRF52_IRQ_H */
