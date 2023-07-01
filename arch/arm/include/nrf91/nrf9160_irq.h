/****************************************************************************
 * arch/arm/include/nrf91/nrf9160_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_NRF9160_IRQ_H
#define __ARCH_ARM_INCLUDE_NRF9160_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Cortex-M33 External interrupts (vectors >= 16) */

/* Application core */

#define NRF91_IRQ_SPU           (NRF91_IRQ_EXTINT+3)   /* SPU interrupt */
#define NRF91_IRQ_REGULATORS    (NRF91_IRQ_EXTINT+4)   /* Regulators */
#define NRF91_IRQ_POWER_CLOCK   (NRF91_IRQ_EXTINT+5)   /* Power, Clock */
#define NRF91_IRQ_SERIAL0       (NRF91_IRQ_EXTINT+8)   /* UART/SPI/TWI 0 */
#define NRF91_IRQ_SERIAL1       (NRF91_IRQ_EXTINT+9)   /* UART/SPI/TWI 1 */
#define NRF91_IRQ_SERIAL2       (NRF91_IRQ_EXTINT+10)  /* UART/SPI/TWI 2 */
#define NRF91_IRQ_SERIAL3       (NRF91_IRQ_EXTINT+11)  /* UART/SPI/TWI 3 */
#define NRF91_IRQ_GPIOTE0       (NRF91_IRQ_EXTINT+13)  /* GPIO Task & Event 0 */
#define NRF91_IRQ_SAADC         (NRF91_IRQ_EXTINT+14)  /* Analog to Digital Converter */
#define NRF91_IRQ_TIMER0        (NRF91_IRQ_EXTINT+15)  /* Timer 0 */
#define NRF91_IRQ_TIMER1        (NRF91_IRQ_EXTINT+16)  /* Timer 1 */
#define NRF91_IRQ_TIMER2        (NRF91_IRQ_EXTINT+17)  /* Timer 2 */
#define NRF91_IRQ_RTC0          (NRF91_IRQ_EXTINT+20)  /* Real-time counter 0 */
#define NRF91_IRQ_RTC1          (NRF91_IRQ_EXTINT+21)  /* Real-time counter 1 */
#define NRF91_IRQ_WDT0          (NRF91_IRQ_EXTINT+24)  /* Watchdog Timer 0 */
#define NRF91_IRQ_EGU0          (NRF91_IRQ_EXTINT+27)  /* Event Gen. Unit 0 */
#define NRF91_IRQ_EGU1          (NRF91_IRQ_EXTINT+28)  /* Event Gen. Unit 1 */
#define NRF91_IRQ_EGU2          (NRF91_IRQ_EXTINT+29)  /* Event Gen. Unit 2 */
#define NRF91_IRQ_EGU3          (NRF91_IRQ_EXTINT+30)  /* Event Gen. Unit 3 */
#define NRF91_IRQ_EGU4          (NRF91_IRQ_EXTINT+31)  /* Event Gen. Unit 4 */
#define NRF91_IRQ_EGU5          (NRF91_IRQ_EXTINT+32)  /* Event Gen. Unit 5 */
#define NRF91_IRQ_PWM0          (NRF91_IRQ_EXTINT+33)  /* Pulse Width Modulation Unit 0 */
#define NRF91_IRQ_PWM1          (NRF91_IRQ_EXTINT+34)  /* Pulse Width Modulation Unit 1 */
#define NRF91_IRQ_PWM2          (NRF91_IRQ_EXTINT+35)  /* Pulse Width Modulation Unit 2 */
#define NRF91_IRQ_PWM3          (NRF91_IRQ_EXTINT+36)  /* Pulse Width Modulation Unit 3 */
#define NRF91_IRQ_PDM           (NRF91_IRQ_EXTINT+38)  /* Pulse Density Modulation (Digital Mic) Interface */
#define NRF91_IRQ_I2S           (NRF91_IRQ_EXTINT+40)  /* Inter-IC Sound interface */
#define NRF91_IRQ_IPC           (NRF91_IRQ_EXTINT+42)  /* IPC */
#define NRF91_IRQ_GPIOTE1       (NRF91_IRQ_EXTINT+49)  /* GPIO Task & Event 1 */
#define NRF91_IRQ_KMU           (NRF91_IRQ_EXTINT+57)  /* KMU */
#define NRF91_IRQ_CRYPTOCELL    (NRF91_IRQ_EXTINT+64)  /* CRYPTOCELL */

#define NRF91_IRQ_NEXTINT       (66)

#define NRF91_IRQ_NIRQS         (NRF91_IRQ_EXTINT+NRF91_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS                 NRF91_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_NRF9160_IRQ_H */
