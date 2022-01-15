/****************************************************************************
 * arch/arm/include/lpc54xx/lpc546x_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_LPC54XX_LPC546X_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC54XX_LPC546X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Cortex-M4 External interrupts (vectors >= 16) */

#define LPC54_IRQ_WDT           (LPC54_IRQ_EXTINT+0)   /* VOD Windowed watchdog timer, Brownout detect */
#define LPC54_IRQ_DMA           (LPC54_IRQ_EXTINT+1)   /* DMA controller */
#define LPC54_IRQ_GINT0         (LPC54_IRQ_EXTINT+2)   /* GPIO group 0 */
#define LPC54_IRQ_GINT1         (LPC54_IRQ_EXTINT+3)   /* GPIO group 1 */
#define LPC54_IRQ_PININT0       (LPC54_IRQ_EXTINT+4)   /* Pin interrupt 0 or pattern match engine slice 0 */
#define LPC54_IRQ_PININT1       (LPC54_IRQ_EXTINT+5)   /* Pin interrupt 1 or pattern match engine slice 1 */
#define LPC54_IRQ_PININT2       (LPC54_IRQ_EXTINT+6)   /* Pin interrupt 2 or pattern match engine slice 2 */
#define LPC54_IRQ_PININT3       (LPC54_IRQ_EXTINT+7)   /* Pin interrupt 3 or pattern match engine slice 3 */
#define LPC54_IRQ_UTICK         (LPC54_IRQ_EXTINT+8)   /* Micro-tick Timer */
#define LPC54_IRQ_MRT           (LPC54_IRQ_EXTINT+9)   /* Multi-rate timer */
#define LPC54_IRQ_CTIMER0       (LPC54_IRQ_EXTINT+10)  /* Standard counter/timer CTIMER0 */
#define LPC54_IRQ_CTIMER1       (LPC54_IRQ_EXTINT+11)  /* Standard counter/timer CTIMER1 */
#define LPC54_IRQ_SCTIMER       (LPC54_IRQ_EXTINT+12)  /* SCTimer/PWM0 */
#define LPC54_IRQ_PWM0          (LPC54_IRQ_EXTINT+12)  /* SCTimer/PWM0 */
#define LPC54_IRQ_CTIMER3       (LPC54_IRQ_EXTINT+13)  /* CTIMER3 Standard counter/timer CTIMER3 */
#define LPC54_IRQ_FLEXCOMM0     (LPC54_IRQ_EXTINT+14)  /* Flexcomm Interface 0 (USART, SPI, I2C) */
#define LPC54_IRQ_FLEXCOMM1     (LPC54_IRQ_EXTINT+15)  /* Flexcomm Interface 1 (USART, SPI, I2C) */
#define LPC54_IRQ_FLEXCOMM2     (LPC54_IRQ_EXTINT+16)  /* Flexcomm Interface 2 (USART, SPI, I2C) */
#define LPC54_IRQ_FLEXCOMM3     (LPC54_IRQ_EXTINT+17)  /* Flexcomm Interface 3 (USART, SPI, I2C) */
#define LPC54_IRQ_FLEXCOMM4     (LPC54_IRQ_EXTINT+18)  /* Flexcomm Interface 4 (USART, SPI, I2C) */
#define LPC54_IRQ_FLEXCOMM5     (LPC54_IRQ_EXTINT+19)  /* Flexcomm Interface 5 (USART, SPI, I2C) */
#define LPC54_IRQ_FLEXCOMM6     (LPC54_IRQ_EXTINT+20)  /* Flexcomm Interface 6 (USART, SPI, I2C, I2S) */
#define LPC54_IRQ_FLEXCOMM7     (LPC54_IRQ_EXTINT+21)  /* Flexcomm Interface 7 (USART, SPI, I2C, I2S) */
#define LPC54_IRQ_ADC0SEQA      (LPC54_IRQ_EXTINT+22)  /* ADC0 sequence A completion */
#define LPC54_IRQ_ADC0SEQB      (LPC54_IRQ_EXTINT+23)  /* ADC0 sequence B completion */
#define LPC54_IRQ_ADC0THCMP     (LPC54_IRQ_EXTINT+24)  /* ADC0 threshold compare and error */
#define LPC54_IRQ_DMIC          (LPC54_IRQ_EXTINT+25)  /* Digital microphone and audio subsystem */
#define LPC54_IRQ_HWVAD         (LPC54_IRQ_EXTINT+26)  /* Hardware Voice Activity Detection */
#define LPC54_IRQ_USB0NEEDCLK   (LPC54_IRQ_EXTINT+27)  /* USB0 Activity Interrupt */
#define LPC54_IRQ_USB0          (LPC54_IRQ_EXTINT+28)  /* USB0 host and device */
#define LPC54_IRQ_RTC           (LPC54_IRQ_EXTINT+29)  /* RTC alarm and wake-up interrupts */
                                                       /* 30-31 Reserved */
#define LPC54_IRQ_PININT4       (LPC54_IRQ_EXTINT+32)  /* Pin interrupt 4 or pattern match engine slice 4 */
#define LPC54_IRQ_PININT5       (LPC54_IRQ_EXTINT+33)  /* Pin interrupt 5 or pattern match engine slice 5 */
#define LPC54_IRQ_PININT6       (LPC54_IRQ_EXTINT+34)  /* Pin interrupt 6 or pattern match engine slice 6 */
#define LPC54_IRQ_PININT7       (LPC54_IRQ_EXTINT+35)  /* Pin interrupt 7 or pattern match engine slice 7 */
#define LPC54_IRQ_CTIMER2       (LPC54_IRQ_EXTINT+36)  /* Standard counter/timer CTIMER2 */
#define LPC54_IRQ_CTIMER4       (LPC54_IRQ_EXTINT+37)  /* Standard counter/timer CTIMER4 */
#define LPC54_IRQ_RIT           (LPC54_IRQ_EXTINT+38)  /* Repetitive Interrupt Timer */
#define LPC54_IRQ_SPIFI         (LPC54_IRQ_EXTINT+39)  /* SPI flash interface */
#define LPC54_IRQ_FLEXCOMM8     (LPC54_IRQ_EXTINT+40)  /* Flexcomm Interface 8 (USART, SPI, I2C) */
#define LPC54_IRQ_FLEXCOMM9     (LPC54_IRQ_EXTINT+41)  /* Flexcomm Interface 9 (USART, SPI, I2C) */
#define LPC54_IRQ_SDMMC         (LPC54_IRQ_EXTINT+42)  /* SD/MMC interrupt */
#define LPC54_IRQ_CAN0IRQ0      (LPC54_IRQ_EXTINT+43)  /* CAN0 interrupt 0 */
#define LPC54_IRQ_CAN0IRQ1      (LPC54_IRQ_EXTINT+44)  /* CAN0 interrupt 1 */
#define LPC54_IRQ_CAN1IRQ0      (LPC54_IRQ_EXTINT+45)  /* CAN1 interrupt 0 */
#define LPC54_IRQ_CAN1IRQ1      (LPC54_IRQ_EXTINT+46)  /* CAN1 interrupt 1 */
#define LPC54_IRQ_USB1          (LPC54_IRQ_EXTINT+47)  /* USB1 interrupt */
#define LPC54_IRQ_USB1NEEDCLK   (LPC54_IRQ_EXTINT+48)  /* USB1 activity */
#define LPC54_IRQ_ETHERNET      (LPC54_IRQ_EXTINT+49)  /* Ethernet */
#define LPC54_IRQ_ETHERNETPMT   (LPC54_IRQ_EXTINT+50)  /* Ethernet power management interrupt */
#define LPC54_IRQ_ETHERNETMACLP (LPC54_IRQ_EXTINT+51)  /* Ethernet MAC interrupt */
#define LPC54_IRQ_EEPROM        (LPC54_IRQ_EXTINT+52)  /* EEPROM interrupt */
#define LPC54_IRQ_LCD           (LPC54_IRQ_EXTINT+53)  /* LCD interrupt */
#define LPC54_IRQ_SHA           (LPC54_IRQ_EXTINT+54)  /* SHA interrupt */
#define LPC54_IRQ_SMARTCARD0    (LPC54_IRQ_EXTINT+55)  /* Smart card 0 interrupt */
#define LPC54_IRQ_SMARTCARD1    (LPC54_IRQ_EXTINT+56)  /* Smart card 1 interrupt */

#define LPC54_IRQ_NEXTINT       (57)
#define LPC54_IRQ_NIRQS         (LPC54_IRQ_EXTINT+LPC54_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS                 LPC54_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_LPC54XX_LPC546X_IRQ_H */
