/****************************************************************************************************
 * arch/arm/include/lpc55xx/lpc55s6x_irq.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_LPC55XX_LPC55S6X_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC55XX_LPC55S6X_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Cortex-M33 External interrupts (vectors >= 16) */

#define LPC55_IRQ_WDT           (LPC55_IRQ_EXTINT + 0)   /* VOD Windowed watchdog timer, Brownout detect */
#define LPC55_IRQ_DMA0          (LPC55_IRQ_EXTINT + 1)   /* DMA0 controller */
#define LPC55_IRQ_GINT0         (LPC55_IRQ_EXTINT + 2)   /* GPIO group 0 */
#define LPC55_IRQ_GINT1         (LPC55_IRQ_EXTINT + 3)   /* GPIO group 1 */
#define LPC55_IRQ_PININT0       (LPC55_IRQ_EXTINT + 4)   /* Pin interrupt 0 or pattern match engine slice 0 */
#define LPC55_IRQ_PININT1       (LPC55_IRQ_EXTINT + 5)   /* Pin interrupt 1 or pattern match engine slice 1 */
#define LPC55_IRQ_PININT2       (LPC55_IRQ_EXTINT + 6)   /* Pin interrupt 2 or pattern match engine slice 2 */
#define LPC55_IRQ_PININT3       (LPC55_IRQ_EXTINT + 7)   /* Pin interrupt 3 or pattern match engine slice 3 */
#define LPC55_IRQ_UTICK         (LPC55_IRQ_EXTINT + 8)   /* Micro-tick Timer */
#define LPC55_IRQ_MRT           (LPC55_IRQ_EXTINT + 9)   /* Multi-rate timer */
#define LPC55_IRQ_CTIMER0       (LPC55_IRQ_EXTINT + 10)  /* Standard counter/timer CTIMER0 */
#define LPC55_IRQ_CTIMER1       (LPC55_IRQ_EXTINT + 11)  /* Standard counter/timer CTIMER1 */
#define LPC55_IRQ_SCTIMER       (LPC55_IRQ_EXTINT + 12)  /* SCTimer */
#define LPC55_IRQ_CTIMER3       (LPC55_IRQ_EXTINT + 13)  /* CTIMER3 Standard counter/timer CTIMER3 */
#define LPC55_IRQ_FLEXCOMM0     (LPC55_IRQ_EXTINT + 14)  /* Flexcomm Interface 0 (USART, SPI, I2C) */
#define LPC55_IRQ_FLEXCOMM1     (LPC55_IRQ_EXTINT + 15)  /* Flexcomm Interface 1 (USART, SPI, I2C) */
#define LPC55_IRQ_FLEXCOMM2     (LPC55_IRQ_EXTINT + 16)  /* Flexcomm Interface 2 (USART, SPI, I2C) */
#define LPC55_IRQ_FLEXCOMM3     (LPC55_IRQ_EXTINT + 17)  /* Flexcomm Interface 3 (USART, SPI, I2C) */
#define LPC55_IRQ_FLEXCOMM4     (LPC55_IRQ_EXTINT + 18)  /* Flexcomm Interface 4 (USART, SPI, I2C) */
#define LPC55_IRQ_FLEXCOMM5     (LPC55_IRQ_EXTINT + 19)  /* Flexcomm Interface 5 (USART, SPI, I2C) */
#define LPC55_IRQ_FLEXCOMM6     (LPC55_IRQ_EXTINT + 20)  /* Flexcomm Interface 6 (USART, SPI, I2C, I2S) */
#define LPC55_IRQ_FLEXCOMM7     (LPC55_IRQ_EXTINT + 21)  /* Flexcomm Interface 7 (USART, SPI, I2C, I2S) */
#define LPC55_IRQ_ADC0SEQA      (LPC55_IRQ_EXTINT + 22)  /* ADC0 sequence A completion */
#define LPC55_IRQ_ADC0SEQB      (LPC55_IRQ_EXTINT + 23)  /* ADC0 sequence B completion */
#define LPC55_IRQ_ADC0THCMP     (LPC55_IRQ_EXTINT + 24)  /* ADC0 threshold compare and error */
#define LPC55_IRQ_USB0NEEDCLK   (LPC55_IRQ_EXTINT + 27)  /* USB0 Activity Interrupt */
#define LPC55_IRQ_USB0          (LPC55_IRQ_EXTINT + 28)  /* USB0 host and device */
#define LPC55_IRQ_RTC           (LPC55_IRQ_EXTINT + 29)  /* RTC alarm and wake-up interrupts */
#define LPC55_IRQ_MAILBOX       (LPC55_IRQ_EXTINT + 31)  /* System IRQ for Mailbox */
#define LPC55_IRQ_PININT4       (LPC55_IRQ_EXTINT + 32)  /* Pin interrupt 4 or pattern match engine slice 4 */
#define LPC55_IRQ_PININT5       (LPC55_IRQ_EXTINT + 33)  /* Pin interrupt 5 or pattern match engine slice 5 */
#define LPC55_IRQ_PININT6       (LPC55_IRQ_EXTINT + 34)  /* Pin interrupt 6 or pattern match engine slice 6 */
#define LPC55_IRQ_PININT7       (LPC55_IRQ_EXTINT + 35)  /* Pin interrupt 7 or pattern match engine slice 7 */
#define LPC55_IRQ_CTIMER2       (LPC55_IRQ_EXTINT + 36)  /* Standard counter/timer CTIMER2 */
#define LPC55_IRQ_CTIMER4       (LPC55_IRQ_EXTINT + 37)  /* Standard counter/timer CTIMER4 */
#define LPC55_IRQ_OSEVTIMER     (LPC55_IRQ_EXTINT + 38)  /* OSTIMER0 */
#define LPC55_IRQ_SDMMC         (LPC55_IRQ_EXTINT + 42)  /* SD/MMC interrupt */
#define LPC55_IRQ_USB1PHY       (LPC55_IRQ_EXTINT + 46)  /* USB1 PHY interrupt */
#define LPC55_IRQ_USB1          (LPC55_IRQ_EXTINT + 47)  /* USB1 interrupt */
#define LPC55_IRQ_USB1NEEDCLK   (LPC55_IRQ_EXTINT + 48)  /* USB1 activity */
#define LPC55_IRQ_HYPERVISOR    (LPC55_IRQ_EXTINT + 49)  /* Hypervisor */
#define LPC55_IRQ_SGPIOINT0IRQ0 (LPC55_IRQ_EXTINT + 50)  /* Secure GPIO 0 interrupt */
#define LPC55_IRQ_SGPIOINT0IRQ1 (LPC55_IRQ_EXTINT + 51)  /* Secure GPIO 1 interrupt */
#define LPC55_IRQ_PLU           (LPC55_IRQ_EXTINT + 52)  /* PLU interrupt */
#define LPC55_IRQ_SECVIO        (LPC55_IRQ_EXTINT + 53)  /* Secure violation interrupt */
#define LPC55_IRQ_HASH          (LPC55_IRQ_EXTINT + 54)  /* HASH interrupt */
#define LPC55_IRQ_CASPER        (LPC55_IRQ_EXTINT + 55)  /* CASPER interrupt */
#define LPC55_IRQ_PUF           (LPC55_IRQ_EXTINT + 56)  /* PUF controller interrupt */
#define LPC55_IRQ_PQ            (LPC55_IRQ_EXTINT + 57)  /* Power quad interrupt */
#define LPC55_IRQ_SDMA1         (LPC55_IRQ_EXTINT + 58)  /* SDMA1 interrupt */
#define LPC55_IRQ_HSSPI         (LPC55_IRQ_EXTINT + 59)  /* HS SPI interrupt */

#define LPC55_IRQ_NEXTINT       (59)
#define LPC55_IRQ_NIRQS         (LPC55_IRQ_EXTINT + LPC55_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS                 LPC55_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_LPC55XX_LPC55S6X_IRQ_H */
