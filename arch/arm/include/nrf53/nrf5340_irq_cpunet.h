/****************************************************************************
 * arch/arm/include/nrf53/nrf5340_irq_cpunet.h
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

#ifndef __ARCH_ARM_INCLUDE_NRF53_NRF5340_IRQ_CPUNET_H
#define __ARCH_ARM_INCLUDE_NRF53_NRF5340_IRQ_CPUNET_H

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Cortex-M33 External interrupts (vectors >= 16) */

/* Network core */

#define NRF53_IRQ_POWER_CLOCK   (NRF53_IRQ_EXTINT+5)   /* Power, Clock, Bprot */
#define NRF53_IRQ_RADIO         (NRF53_IRQ_EXTINT+8)   /* Radio controller */
#define NRF53_IRQ_RNG           (NRF53_IRQ_EXTINT+9)   /* Random Number Generator */
#define NRF53_IRQ_GPIOTE        (NRF53_IRQ_EXTINT+10)  /* GPIO Task & Event */
#define NRF53_IRQ_WDT           (NRF53_IRQ_EXTINT+11)  /* Watchdog Timer */
#define NRF53_IRQ_TIMER0        (NRF53_IRQ_EXTINT+12)  /* Timer 0 */
#define NRF53_IRQ_ECB           (NRF53_IRQ_EXTINT+13)  /* AES ECB Mode Encryption */
#define NRF53_IRQ_CCM_AAR       (NRF53_IRQ_EXTINT+14)  /* AES CCM Mode Encryption/Accel. Address Resolve */
#define NRF53_IRQ_TEMP          (NRF53_IRQ_EXTINT+16)  /* Temperature Sensor */
#define NRF53_IRQ_RTC0          (NRF53_IRQ_EXTINT+17)  /* Real-time counter 0 */
#define NRF53_IRQ_IPC           (NRF53_IRQ_EXTINT+18)  /* IPC */
#define NRF53_IRQ_UART0         (NRF53_IRQ_EXTINT+19)  /* UART/UARTE 0 */
#define NRF53_IRQ_EGU0          (NRF53_IRQ_EXTINT+20)  /* Event Gen. Unit 0 */
#define NRF53_IRQ_RTC1          (NRF53_IRQ_EXTINT+22)  /* Real-time counter 1 */
#define NRF53_IRQ_TIMER1        (NRF53_IRQ_EXTINT+24)  /* Timer 1 */
#define NRF53_IRQ_TIMER2        (NRF53_IRQ_EXTINT+25)  /* Timer 2 */
#define NRF53_IRQ_SWI0          (NRF53_IRQ_EXTINT+26)  /* Software interrupt 0 */
#define NRF53_IRQ_SWI1          (NRF53_IRQ_EXTINT+27)  /* Software interrupt 1  */
#define NRF53_IRQ_SWI2          (NRF53_IRQ_EXTINT+28)  /* Software interrupt 2  */
#define NRF53_IRQ_SWI3          (NRF53_IRQ_EXTINT+29)  /* Software interrupt 3 / Event Gen. Unit 3 */

#define NRF53_IRQ_NEXTINT       (30)

#define NRF53_IRQ_NIRQS         (NRF53_IRQ_EXTINT+NRF53_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS                 NRF53_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_NRF53_NRF5340_IRQ_CPUNET_H */
