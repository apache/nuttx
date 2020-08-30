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

#define NRF52_IRQ_POWER_CLOCK   (NRF52_IRQ_EXTINT+0)   /* Power, Clock, Bprot */
#define NRF52_IRQ_RADIO         (NRF52_IRQ_EXTINT+1)   /* Radio controller */
#define NRF52_IRQ_UART0         (NRF52_IRQ_EXTINT+2)   /* UART/UARTE 0 */
#define NRF52_IRQ_SPI_TWI_0     (NRF52_IRQ_EXTINT+3)   /* SPI / TWI 0 */
#define NRF52_IRQ_SPI_TWI_1     (NRF52_IRQ_EXTINT+4)   /* SPI / TWI 1 */
#define NRF52_IRQ_NFCT          (NRF52_IRQ_EXTINT+5)   /* NFCT */
#define NRF52_IRQ_GPIOTE        (NRF52_IRQ_EXTINT+6)   /* GPIO Task & Event */
#define NRF52_IRQ_SAADC         (NRF52_IRQ_EXTINT+7)   /* Analog to Digital Converter */
#define NRF52_IRQ_TIMER0        (NRF52_IRQ_EXTINT+8)   /* Timer 0 */
#define NRF52_IRQ_TIMER1        (NRF52_IRQ_EXTINT+9)   /* Timer 1 */
#define NRF52_IRQ_TIMER2        (NRF52_IRQ_EXTINT+10)  /* Timer 2 */
#define NRF52_IRQ_RTC0          (NRF52_IRQ_EXTINT+11)  /* Real-time counter 0 */
#define NRF52_IRQ_TEMP          (NRF52_IRQ_EXTINT+12)  /* Temperature Sensor */
#define NRF52_IRQ_RNG           (NRF52_IRQ_EXTINT+13)  /* Random Number Generator */
#define NRF52_IRQ_ECB           (NRF52_IRQ_EXTINT+14)  /* AES ECB Mode Encryption */
#define NRF52_IRQ_CCM_AAR       (NRF52_IRQ_EXTINT+15)  /* AES CCM Mode Encryption/Accel. Address Resolve */
#define NRF52_IRQ_WDT           (NRF52_IRQ_EXTINT+16)  /* Watchdog Timer */
#define NRF52_IRQ_RTC1          (NRF52_IRQ_EXTINT+17)  /* Real-time counter 1 */
#define NRF52_IRQ_QDEC          (NRF52_IRQ_EXTINT+18)  /* Quadrature decoder */
#define NRF52_IRQ_COMP_LPCOMP   (NRF52_IRQ_EXTINT+19)  /* Low power comparator */
#define NRF52_IRQ_SWI0_EGU0     (NRF52_IRQ_EXTINT+20)  /* Software interrupt 0 / Event Gen. Unit 0 */
#define NRF52_IRQ_SWI1_EGU1     (NRF52_IRQ_EXTINT+21)  /* Software interrupt 1 / Event Gen. Unit 1 */
#define NRF52_IRQ_SWI2_EGU2     (NRF52_IRQ_EXTINT+22)  /* Software interrupt 2 / Event Gen. Unit 2 */
#define NRF52_IRQ_SWI3_EGU3     (NRF52_IRQ_EXTINT+23)  /* Software interrupt 3 / Event Gen. Unit 3 */
#define NRF52_IRQ_SWI4_EGU4     (NRF52_IRQ_EXTINT+24)  /* Software interrupt 4 / Event Gen. Unit 4 */
#define NRF52_IRQ_SWI5_EGU5     (NRF52_IRQ_EXTINT+25)  /* Software interrupt 5 / Event Gen. Unit 5 */
#define NRF52_IRQ_TIMER3        (NRF52_IRQ_EXTINT+26)  /* Timer 3 */
#define NRF52_IRQ_TIMER4        (NRF52_IRQ_EXTINT+27)  /* Timer 4 */
#define NRF52_IRQ_PWM0          (NRF52_IRQ_EXTINT+28)  /* Pulse Width Modulation Unit 0 */
#define NRF52_IRQ_PDM           (NRF52_IRQ_EXTINT+29)  /* Pulse Density Modulation (Digital Mic) Interface */
#define NRF52_IRQ_NVMC          (NRF52_IRQ_EXTINT+30)  /* Non Volatile Memory Controller */
#define NRF52_IRQ_PPI           (NRF52_IRQ_EXTINT+31)  /* PPI controller */
#define NRF52_IRQ_MWU           (NRF52_IRQ_EXTINT+32)  /* Memory Watch Unit */
#define NRF52_IRQ_PWM1          (NRF52_IRQ_EXTINT+33)  /* Pulse Width Modulation Unit 1 */
#define NRF52_IRQ_PWM2          (NRF52_IRQ_EXTINT+34)  /* Pulse Width Modulation Unit 2 */
#define NRF52_IRQ_SPI2          (NRF52_IRQ_EXTINT+35)  /* SPI master 2 / SPI slave 2 */
#define NRF52_IRQ_RTC2          (NRF52_IRQ_EXTINT+36)  /* Real-time counter 2 */
#define NRF52_IRQ_I2S           (NRF52_IRQ_EXTINT+37)  /* Inter-IC Sound interface */
#define NRF52_IRQ_FPU           (NRF52_IRQ_EXTINT+38)  /* FPU interrupt */

#ifdef CONFIG_NRF52_HAVE_USBDEV
#  define NRF52_IRQ_USBD        (NRF52_IRQ_EXTINT+39)  /* USB device */
#endif
#ifdef CONFIG_NRF52_HAVE_UART1
#  define NRF52_IRQ_UART1       (NRF52_IRQ_EXTINT+40)  /* UART/UARTE 1 */
#endif
#ifdef CONFIG_NRF52_HAVE_QSPI
#  define NRF52_IRQ_QSPI        (NRF52_IRQ_EXTINT+41)  /* Quad SPI */
#endif
#ifdef CONFIG_NRF52_HAVE_PWM3
#  define NRF52_IRQ_PWM3        (NRF52_IRQ_EXTINT+45)  /* Pulse Width Modulation Unit 3 */
#endif
#ifdef CONFIG_NRF52_HAVE_SPI3_MASTER
#  define NRF52_IRQ_SPIM3       (NRF52_IRQ_EXTINT+47)  /* SPI Master 3 */
#endif

#if defined(CONFIG_ARCH_CHIP_NRF52832)
#  define NRF52_IRQ_NEXTINT     (39)
#elif defined(CONFIG_ARCH_CHIP_NRF52833)
#  define NRF52_IRQ_NEXTINT     (48)
#elif defined(CONFIG_ARCH_CHIP_NRF52840)
#  define NRF52_IRQ_NEXTINT     (48)
#else
#  error Unknown NRF52 chip !
#endif

#define NRF52_IRQ_NIRQS         (NRF52_IRQ_EXTINT+NRF52_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS                 NRF52_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_NRF52_NRF52_IRQ_H */
