/****************************************************************************
 * arch/arm/include/max326xx/max32690_irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_INCLUDE_MAX326XX_MAX32690_IRQ_H
#define __ARCH_ARM_INCLUDE_MAX326XX_MAX32690_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* External interrupts (vectors >= 16) */

#define MAX32690_IRQ_PF                  16    /*  Power Fail */
#define MAX32690_IRQ_WDT0                17    /*  Watchdog 0 Interrupt */
#define MAX32690_IRQ_USB                 18    /*  USB */
#define MAX32690_IRQ_RTC                 19    /*  Real-time Clock */
#define MAX32690_IRQ_TRNG                20    /*  True Random Number Generator */
#define MAX32690_IRQ_TMR0                21    /*  Timer 0  */
#define MAX32690_IRQ_TMR1                22    /*  Timer 1  */
#define MAX32690_IRQ_TMR2                23    /*  Timer 2  */
#define MAX32690_IRQ_TMR3                24    /*  Timer 3  */
#define MAX32690_IRQ_TMR4                25    /*  Timer 4  Low-Power Timer 0 */
#define MAX32690_IRQ_TMR5                26    /*  Timer 5  Low-Power Timer 1 */
                                               /*  27 - 28 Reserved */
#define MAX32690_IRQ_I2C0                29    /*  I2C Port 0 */
#define MAX32690_IRQ_UART0               30    /*  UART 0 */
#define MAX32690_IRQ_UART1               31    /*  UART 1 */
#define MAX32690_IRQ_SPI0                32    /*  SPI 0 */
#define MAX32690_IRQ_SPI1                33    /*  SPI 1 */
#define MAX32690_IRQ_SPI2                34    /*  SPI 2 */
                                               /*  35 Reserved */
#define MAX32690_IRQ_ADC                 36    /*  ADC */
                                               /*  37 - 38 Reserved */
#define MAX32690_IRQ_FLASH0              39    /*  Flash Controller 0 */
#define MAX32690_IRQ_GPIO0               40    /*  GPIO 0 */
#define MAX32690_IRQ_GPIO1               41    /*  GPIO 1 */
#define MAX32690_IRQ_GPIO2               42    /*  GPIO 2 */
#define MAX32690_IRQ_CRYPTO              43    /*  Crypto Tool Box */
#define MAX32690_IRQ_DMA0                44    /*  DMA 0 */
#define MAX32690_IRQ_DMA1                45    /*  DMA 1 */
#define MAX32690_IRQ_DMA2                46    /*  DMA 2 */
#define MAX32690_IRQ_DMA3                47    /*  DMA 3 */
                                               /*  48 - 49 Reserved */
#define MAX32690_IRQ_UART2               50    /*  UART 2 */
                                               /*  51 Reserved */
#define MAX32690_IRQ_I2C1                52    /*  I2C Port 1 */ */
                                               /*  53 Reserved */
#define MAX32690_IRQ_SPIX                54    /*  SPI-XiP */
#define MAX32690_IRQ_BTLE_TX_DONE        55    /*  Bluetooth Transmitter Done */
#define MAX32690_IRQ_BTLE_RX_RCVD        56    /*  Bluetooth Receive Data  */
#define MAX32690_IRQ_BTLE_RX_ENG_DET     57    /*  Bluetooth Receive Energy Detected  */
#define MAX32690_IRQ_BTLE_SFD_DET        58    /*  BTLE SFD Detected */
#define MAX32690_IRQ_BTLE_SFD_TO         59    /*  BTLE SFD Timeout */
#define MAX32690_IRQ_BTLE_GP_EVENT       60    /*  BTLE Timestamp */
#define MAX32690_IRQ_BTLE_CFO            61    /*  BTLE CFO Done */
#define MAX32690_IRQ_BTLE_SIG_DET        62    /*  BTLE Signal Detected */
#define MAX32690_IRQ_BTLE_AGC_EVENT      63    /*  BTLE AGC  */
#define MAX32690_IRQ_BTLE_RFFE_SPIM      64    /*  BTLE RFFE SPIM Done */
#define MAX32690_IRQ_BTLE_TX_AES         65    /*  BTLE TX AES Done */
#define MAX32690_IRQ_BTLE_RX_AES         66    /*  BTLE RX AES Done */
#define MAX32690_IRQ_BTLE_INV_APB_ADDR   67    /*  BTLE Invalid APB Address */
#define MAX32690_IRQ_BTLE_IQ_DATA_VALID  68    /*  BTLE IQ Data Valid  */
#define MAX32690_IRQ_WUT0                69    /*  Wake-up Timer 0  */
#define MAX32690_IRQ_GPIOWAKE            70    /*  GPIO Wake-up  */
                                               /*  71 Reserved */
#define MAX32690_IRQ_SPI3                72    /*  SPI 3  */
#define MAX32690_IRQ_WDT1                73    /*  Low-Power Watchdog Timer 0 (WDT1) */
#define MAX32690_IRQ_GPIO3               74    /*  GPIO 3 */
#define MAX32690_IRQ_PT                  75    /*  Pulse Train */
                                               /*  76 Reserved */
#define MAX32690_IRQ_HPB                 77    /*  HyperBus  */
#define MAX32690_IRQ_I2C2                78    /*  I2C Port 2 */
#define MAX32690_IRQ_RISCV               79    /*  RV32  */
                                               /*  80 - 82 Reserved */
#define MAX32690_IRQ_OWM                 83    /*  1-Wire Controller */                                               
#define MAX32690_IRQ_DMA4                84    /*  DMA 4 */
#define MAX32690_IRQ_DMA5                85    /*  DMA 5 */
#define MAX32690_IRQ_DMA6                86    /*  DMA 6 */
#define MAX32690_IRQ_DMA7                87    /*  DMA 7 */
#define MAX32690_IRQ_DMA8                88    /*  DMA 8 */
#define MAX32690_IRQ_DMA9                89    /*  DMA 9 */
#define MAX32690_IRQ_DMA10               90    /*  DMA 10 */
#define MAX32690_IRQ_DMA11               91    /*  DMA 11 */
#define MAX32690_IRQ_DMA12               92    /*  DMA 12 */
#define MAX32690_IRQ_DMA13               93    /*  DMA 13 */
#define MAX32690_IRQ_DMA14               94    /*  DMA 14 */
#define MAX32690_IRQ_DMA15               95    /*  DMA 15 */
#define MAX32690_IRQ_USBDMA              96    /*  USB DMA */
                                               /*  97 Reserved */
#define MAX32690_IRQ_ECC                 98    /*  Error Correction Coding Block */
                                               /*  99 - 100 Reserved */
#define MAX32690_IRQ_SCA                 101   /*  SCA Crypto Accelerator */
                                               /*  102 Reserved */
#define MAX32690_IRQ_FLC1                103   /*  Flash Controller 1  */                                               
#define MAX32690_IRQ_UART3               104   /*  UART 3 */
                                               /*  105 - 110 Reserved */
#define MAX32690_IRQ_PUF                 111   /*  Physically Unclonable Function  */
                                               /*  112 Reserved */
#define MAX32690_IRQ_I2S                 115   /*  I2S  */
                                               /*  116 - 118 Reserved */
#define MAX32690_IRQ_LPCMP               119   /*  Low-Power Comparator  */
                                               /*  120 Reserved */
#define MAX32690_IRQ_SPI4                121   /*  SPI 4 */
                                               /*  122 Reserved */
#define MAX32690_IRQ_CAN0                123   /*  CAN 0 */
#define MAX32690_IRQ_CAN1                124   /*  CAN 1 */
#define MAX32690_IRQ_WUT1                125   /*  Wake-up Timer 1  */
                                               /*  126 - 127 Reserved */

/* Number of external interrupts and number of true interrupt vectors */

#define MAX326_IRQ_NEXTINT    112
#define MAX326_IRQ_NVECTORS   (MAX326_IRQ_EXTINT + MAX326_IRQ_NEXTINT)

/* Total number of interrupts handled by the OS */

#define NR_IRQS               (MAX326_IRQ_NVECTORS)

#endif /* __ARCH_ARM_INCLUDE_MAX326XX_MAX32690_IRQ_H */
