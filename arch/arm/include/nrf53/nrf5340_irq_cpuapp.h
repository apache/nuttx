/****************************************************************************
 * arch/arm/include/nrf53/nrf5340_irq_cpuapp.h
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

#ifndef __ARCH_ARM_INCLUDE_NRF53_NRF5340_IRQ_CPUAPP_H
#define __ARCH_ARM_INCLUDE_NRF53_NRF5340_IRQ_CPUAPP_H

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Cortex-M33 External interrupts (vectors >= 16) */

/* Application core */

#define NRF53_IRQ_FPU           (NRF53_IRQ_EXTINT+0)   /* FPU interrupt */
#define NRF53_IRQ_CACHE         (NRF53_IRQ_EXTINT+1)   /* CACHE interrupt */
#define NRF53_IRQ_SPU           (NRF53_IRQ_EXTINT+3)   /* SPU interrupt */
#define NRF53_IRQ_POWER_CLOCK   (NRF53_IRQ_EXTINT+5)   /* Power, Clock, Bprot */
#define NRF53_IRQ_UART0         (NRF53_IRQ_EXTINT+8)   /* UART/UARTE 0 */
#define NRF53_IRQ_UART1         (NRF53_IRQ_EXTINT+9)   /* UART/UARTE 1 */
#define NRF53_IRQ_SPIM4         (NRF53_IRQ_EXTINT+10)  /* SPIM4 */
#define NRF53_IRQ_UART2         (NRF53_IRQ_EXTINT+11)  /* UART/UARTE 2 */
#define NRF53_IRQ_UART3         (NRF53_IRQ_EXTINT+12)  /* UART/UARTE 3 */
#define NRF53_IRQ_GPIOTE0       (NRF53_IRQ_EXTINT+13)  /* GPIO Task & Event 0 */
#define NRF53_IRQ_SAADC         (NRF53_IRQ_EXTINT+14)  /* Analog to Digital Converter */
#define NRF53_IRQ_TIMER0        (NRF53_IRQ_EXTINT+15)  /* Timer 0 */
#define NRF53_IRQ_TIMER1        (NRF53_IRQ_EXTINT+16)  /* Timer 1 */
#define NRF53_IRQ_TIMER2        (NRF53_IRQ_EXTINT+17)  /* Timer 2 */
#define NRF53_IRQ_RTC0          (NRF53_IRQ_EXTINT+20)  /* Real-time counter 0 */
#define NRF53_IRQ_RTC1          (NRF53_IRQ_EXTINT+21)  /* Real-time counter 1 */
#define NRF53_IRQ_WDT0          (NRF53_IRQ_EXTINT+24)  /* Watchdog Timer 0 */
#define NRF53_IRQ_WDT1          (NRF53_IRQ_EXTINT+25)  /* Watchdog Timer 1 */
#define NRF53_IRQ_COMP_LPCOMP   (NRF53_IRQ_EXTINT+26)  /* Low power comparator */
#define NRF53_IRQ_EGU0          (NRF53_IRQ_EXTINT+27)  /* Event Gen. Unit 0 */
#define NRF53_IRQ_EGU1          (NRF53_IRQ_EXTINT+28)  /* Event Gen. Unit 1 */
#define NRF53_IRQ_EGU2          (NRF53_IRQ_EXTINT+29)  /* Event Gen. Unit 2 */
#define NRF53_IRQ_EGU3          (NRF53_IRQ_EXTINT+30)  /* Event Gen. Unit 3 */
#define NRF53_IRQ_EGU4          (NRF53_IRQ_EXTINT+31)  /* Event Gen. Unit 4 */
#define NRF53_IRQ_EGU5          (NRF53_IRQ_EXTINT+32)  /* Event Gen. Unit 5 */
#define NRF53_IRQ_PWM0          (NRF53_IRQ_EXTINT+33)  /* Pulse Width Modulation Unit 0 */
#define NRF53_IRQ_PWM1          (NRF53_IRQ_EXTINT+34)  /* Pulse Width Modulation Unit 1 */
#define NRF53_IRQ_PWM2          (NRF53_IRQ_EXTINT+35)  /* Pulse Width Modulation Unit 2 */
#define NRF53_IRQ_PWM3          (NRF53_IRQ_EXTINT+36)  /* Pulse Width Modulation Unit 3 */
#define NRF53_IRQ_PDM           (NRF53_IRQ_EXTINT+38)  /* Pulse Density Modulation (Digital Mic) Interface */
#define NRF53_IRQ_I2S           (NRF53_IRQ_EXTINT+40)  /* Inter-IC Sound interface */
#define NRF53_IRQ_IPC           (NRF53_IRQ_EXTINT+42)  /* IPC */
#define NRF53_IRQ_QSPI          (NRF53_IRQ_EXTINT+43)  /* QSPI */
#define NRF53_IRQ_NFCT          (NRF53_IRQ_EXTINT+45)  /* NFCT */
#define NRF53_IRQ_GPIOTE1       (NRF53_IRQ_EXTINT+47)  /* GPIO Task & Event 1 */
#define NRF53_IRQ_QDEC0         (NRF53_IRQ_EXTINT+51)  /* Quadrature decoder 0 */
#define NRF53_IRQ_QDEC1         (NRF53_IRQ_EXTINT+52)  /* Quadrature decoder 1 */
#define NRF53_IRQ_USBD          (NRF53_IRQ_EXTINT+54)  /* USBD */
#define NRF53_IRQ_USBREG        (NRF53_IRQ_EXTINT+55)  /* USBREGULATOR */
#define NRF53_IRQ_KMU           (NRF53_IRQ_EXTINT+57)  /* KMU */
#define NRF53_IRQ_CRYPTOCELL    (NRF53_IRQ_EXTINT+68)  /* CRYPTOCELL */

#define NRF53_IRQ_NEXTINT       (69)

#define NRF53_IRQ_NIRQS         (NRF53_IRQ_EXTINT+NRF53_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS                 NRF53_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_NRF53_NRF5340_IRQ_CPUAPP_H */
