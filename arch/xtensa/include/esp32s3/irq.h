/****************************************************************************
 * arch/xtensa/include/esp32s3/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_XTENSA_INCLUDE_ESP32S3_IRQ_H
#define __ARCH_XTENSA_INCLUDE_ESP32S3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32S3_INT_PRIO_DEF        1

/* Interrupt Matrix
 *
 * The Interrupt Matrix embedded in the ESP32-S3 independently allocates
 * peripheral interrupt sources to the two CPUs’ peripheral interrupts, to
 * timely inform CPU0 or CPU1 to process the interrupts once the interrupt
 * signals are generated.
 * Peripheral interrupt sources must be routed to CPU0/CPU1 peripheral
 * interrupts via this interrupt matrix due to the following considerations:
 * - ESP32-S3 has 99 peripheral interrupt sources. To map them to 32 CPU0
 *   interrupts or 32 CPU1 interrupts, this matrix is needed.
 * - Through this matrix, one peripheral interrupt source can be mapped to
 *   multiple CPU0 interrupts or CPU1 interrupts according to application
 *   requirements.
 *
 * Features:
 * - Accept 99 peripheral interrupt sources as input.
 * - Generate 26 peripheral interrupts to CPU0 and 26 peripheral interrupts
 *   to CPU1 as output. Note that the remaining 6 CPU0 interrupts and 6 CPU1
 *   interrupts are internal interrupts.
 * - Support disabling CPU non-maskable interrupt (NMI) sources.
 * - Support querying current interrupt status of peripheral interrupt
 *   sources.
 */

#define ESP32S3_PERIPH_MAC                                 0
#define ESP32S3_PERIPH_MAC_NMI                             1
#define ESP32S3_PERIPH_PWR                                 2
#define ESP32S3_PERIPH_BB                                  3
#define ESP32S3_PERIPH_BT_MAC                              4
#define ESP32S3_PERIPH_BT_BB                               5
#define ESP32S3_PERIPH_BT_BB_NMI                           6
#define ESP32S3_PERIPH_RWBT                                7
#define ESP32S3_PERIPH_RWBLE                               8
#define ESP32S3_PERIPH_RWBT_NMI                            9

/* RESERVED interrupts: 12, 13, 15, 18, 19 */

#define ESP32S3_PERIPH_RWBLE_NMI                           10
#define ESP32S3_PERIPH_I2C_MST                             11
#define ESP32S3_PERIPH_UHCI0                               14
#define ESP32S3_PERIPH_GPIO_INT_CPU                        16
#define ESP32S3_PERIPH_GPIO_INT_CPU_NMI                    17

/* RESERVED interrupts: 23 */

#define ESP32S3_PERIPH_SPI1                                20
#define ESP32S3_PERIPH_SPI2                                21
#define ESP32S3_PERIPH_SPI3                                22
#define ESP32S3_PERIPH_LCD_CAM                             24
#define ESP32S3_PERIPH_I2S0                                25
#define ESP32S3_PERIPH_I2S1                                26
#define ESP32S3_PERIPH_UART0                               27
#define ESP32S3_PERIPH_UART1                               28
#define ESP32S3_PERIPH_UART2                               29

/* RESERVED interrupts: 33, 34 */

#define ESP32S3_PERIPH_SDIO_HOST                           30
#define ESP32S3_PERIPH_PWM0                                31
#define ESP32S3_PERIPH_PWM1                                32
#define ESP32S3_PERIPH_LEDC                                35
#define ESP32S3_PERIPH_EFUSE                               36
#define ESP32S3_PERIPH_CAN                                 37
#define ESP32S3_PERIPH_USB                                 38
#define ESP32S3_PERIPH_RTC_CORE                            39

/* RESERVED interrupts: 44, 45, 46, 47, 48, 49 */

#define ESP32S3_PERIPH_RMT                                 40
#define ESP32S3_PERIPH_PCNT                                41
#define ESP32S3_PERIPH_I2C_EXT0                            42
#define ESP32S3_PERIPH_I2C_EXT1                            43

#define ESP32S3_PERIPH_TG_T0_LEVEL                         50
#define ESP32S3_PERIPH_TG_T1_LEVEL                         51
#define ESP32S3_PERIPH_TG_WDT_LEVEL                        52
#define ESP32S3_PERIPH_TG1_T0_LEVEL                        53
#define ESP32S3_PERIPH_TG1_T1_LEVEL                        54
#define ESP32S3_PERIPH_TG1_WDT_LEVEL                       55
#define ESP32S3_PERIPH_CACHE_IA                            56
#define ESP32S3_PERIPH_SYSTIMER_TARGET0                    57
#define ESP32S3_PERIPH_SYSTIMER_TARGET1                    58
#define ESP32S3_PERIPH_SYSTIMER_TARGET2                    59

#define ESP32S3_PERIPH_SPI_MEM_REJECT                      60
#define ESP32S3_PERIPH_DCACHE_PRELOAD                      61
#define ESP32S3_PERIPH_ICACHE_PRELOAD                      62
#define ESP32S3_PERIPH_DCACHE_SYNC                         63
#define ESP32S3_PERIPH_ICACHE_SYNC                         64
#define ESP32S3_PERIPH_APB_ADC                             65
#define ESP32S3_PERIPH_DMA_IN_CH0                          66
#define ESP32S3_PERIPH_DMA_IN_CH1                          67
#define ESP32S3_PERIPH_DMA_IN_CH2                          68
#define ESP32S3_PERIPH_DMA_IN_CH3                          69

#define ESP32S3_PERIPH_DMA_IN_CH4                          70
#define ESP32S3_PERIPH_DMA_OUT_CH0                         71
#define ESP32S3_PERIPH_DMA_OUT_CH1                         72
#define ESP32S3_PERIPH_DMA_OUT_CH2                         73
#define ESP32S3_PERIPH_DMA_OUT_CH3                         74
#define ESP32S3_PERIPH_DMA_OUT_CH4                         75
#define ESP32S3_PERIPH_RSA                                 76
#define ESP32S3_PERIPH_AES                                 77
#define ESP32S3_PERIPH_SHA                                 78
#define ESP32S3_PERIPH_INT_FROM_CPU0                       79

#define ESP32S3_PERIPH_INT_FROM_CPU1                       80
#define ESP32S3_PERIPH_INT_FROM_CPU2                       81
#define ESP32S3_PERIPH_INT_FROM_CPU3                       82
#define ESP32S3_PERIPH_ASSIST_DEBUG                        83
#define ESP32S3_PERIPH_DMA_APB_PMS_MONITOR_VIOLATE         84
#define ESP32S3_PERIPH_CORE_0_IRAM0_PMS_MONITOR_VIOLATE    85
#define ESP32S3_PERIPH_CORE_0_DRAM0_PMS_MONITOR_VIOLATE    86
#define ESP32S3_PERIPH_CORE_0_PIF_PMS_MONITOR_VIOLATE      87
#define ESP32S3_PERIPH_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE 88
#define ESP32S3_PERIPH_CORE_1_IRAM0_PMS_MONITOR_VIOLATE    89

#define ESP32S3_PERIPH_CORE_1_DRAM0_PMS_MONITOR_VIOLATE    90
#define ESP32S3_PERIPH_CORE_1_PIF_PMS_MONITOR_VIOLATE      91
#define ESP32S3_PERIPH_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE 92
#define ESP32S3_PERIPH_BACKUP_PMS_VIOLATE                  93
#define ESP32S3_PERIPH_CACHE_CORE0_ACS                     94
#define ESP32S3_PERIPH_CACHE_CORE1_ACS                     95
#define ESP32S3_PERIPH_USB_DEVICE                          96
#define ESP32S3_PERIPH_PERIPH_BACKUP                       97
#define ESP32S3_PERIPH_DMA_EXTMEM_REJECT                   98

/* Total number of peripherals */

#define ESP32S3_NPERIPHERALS          99

/* Exceptions
 *
 * IRAM Offset  Description
 *   0x0000     Windows
 *   0x0180     Level 2 interrupt
 *   0x01c0     Level 3 interrupt
 *   0x0200     Level 4 interrupt
 *   0x0240     Level 5 interrupt
 *   0x0280     Debug exception
 *   0x02c0     NMI exception
 *   0x0300     Kernel exception
 *   0x0340     User exception
 *   0x03c0     Double exception
 */

/* IRQ numbers for internal interrupts that are dispatched like peripheral
 * interrupts.
 */

#define XTENSA_IRQ_TIMER0           0  /* INTERRUPT, bit 6 */
#define XTENSA_IRQ_TIMER1           1  /* INTERRUPT, bit 15 */
#define XTENSA_IRQ_TIMER2           2  /* INTERRUPT, bit 16 */
#define XTENSA_IRQ_SYSCALL          3  /* User interrupt w/EXCCAUSE=syscall */
#define XTENSA_IRQ_SWINT            4  /* Software interrupt */

#define XTENSA_NIRQ_INTERNAL        5  /* Number of dispatch internal interrupts */
#define XTENSA_IRQ_FIRSTPERIPH      5  /* First peripheral IRQ number */

/* IRQ numbers for peripheral interrupts coming through the Interrupt
 * Matrix.
 */

#define ESP32S3_IRQ2PERIPH(irq)                         ((irq) - XTENSA_IRQ_FIRSTPERIPH)
#define ESP32S3_PERIPH2IRQ(id)                          ((id) + XTENSA_IRQ_FIRSTPERIPH)

#define ESP32S3_IRQ_MAC                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_MAC)
#define ESP32S3_IRQ_MAC_NMI                             (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_MAC_NMI)
#define ESP32S3_IRQ_PWR                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_PWR)
#define ESP32S3_IRQ_BB                                  (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_BB)
#define ESP32S3_IRQ_BT_MAC                              (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_BT_MAC)
#define ESP32S3_IRQ_BT_BB                               (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_BB)
#define ESP32S3_IRQ_BT_BB_NMI                           (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_BB_NMI)
#define ESP32S3_IRQ_RWBT                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_RWBT)
#define ESP32S3_IRQ_RWBLE                               (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_RWBLE)
#define ESP32S3_IRQ_RWBT_NMI                            (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_RWBT_NMI)

#define ESP32S3_IRQ_RWBLE_NMI                           (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_RWBLE_NMI)
#define ESP32S3_IRQ_I2C_MST                             (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_I2C_MST)
#define ESP32S3_IRQ_UHCI0                               (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_UHCI0)
#define ESP32S3_IRQ_GPIO_INT_CPU                        (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_GPIO_INT_CPU)
#define ESP32S3_IRQ_GPIO_INT_CPU_NMI                    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_GPIO_INT_CPU_NMI)

#define ESP32S3_IRQ_SPI1                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SPI1)
#define ESP32S3_IRQ_SPI2                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SPI2)
#define ESP32S3_IRQ_SPI3                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SPI3)
#define ESP32S3_IRQ_LCD_CAM                             (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_LCD_CAM)
#define ESP32S3_IRQ_I2S0                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_I2S0)
#define ESP32S3_IRQ_I2S1                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_I2S1)
#define ESP32S3_IRQ_UART0                               (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_UART0)
#define ESP32S3_IRQ_UART1                               (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_UART1)
#define ESP32S3_IRQ_UART2                               (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_UART2)

#define ESP32S3_IRQ_SDIO_HOST                           (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SDIO_HOST)
#define ESP32S3_IRQ_PWM0                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_PWM0)

#define ESP32S3_IRQ_SREG0                               ESP32S3_IRQ_MAC
#define ESP32S3_NIRQS_SREG0                             32

#define ESP32S3_IRQ_PWM1                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_PWM1)
#define ESP32S3_IRQ_LEDC                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_LEDC)
#define ESP32S3_IRQ_EFUSE                               (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_EFUSE)
#define ESP32S3_IRQ_CAN                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CAN)
#define ESP32S3_IRQ_USB                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_USB)
#define ESP32S3_IRQ_RTC_CORE                            (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_RTC_CORE)

#define ESP32S3_IRQ_RMT                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_RMT)
#define ESP32S3_IRQ_PCNT                                (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_PCNT)
#define ESP32S3_IRQ_I2C_EXT0                            (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_I2C_EXT0)
#define ESP32S3_IRQ_I2C_EXT1                            (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_I2C_EXT1)

#define ESP32S3_IRQ_TG_T0_LEVEL                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_TG_T0_LEVEL)
#define ESP32S3_IRQ_TG_T1_LEVEL                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_TG_T1_LEVEL)
#define ESP32S3_IRQ_TG_WDT_LEVEL                        (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_TG_WDT_LEVEL)
#define ESP32S3_IRQ_TG1_T0_LEVEL                        (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_TG1_T0_LEVEL)
#define ESP32S3_IRQ_TG1_T1_LEVEL                        (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_TG1_T1_LEVEL)
#define ESP32S3_IRQ_TG1_WDT_LEVEL                       (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_TG1_WDT_LEVEL)
#define ESP32S3_IRQ_CACHE_IA                            (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CACHE_IA)
#define ESP32S3_IRQ_SYSTIMER_TARGET0                    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SYSTIMER_TARGET0)
#define ESP32S3_IRQ_SYSTIMER_TARGET1                    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SYSTIMER_TARGET1)
#define ESP32S3_IRQ_SYSTIMER_TARGET2                    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SYSTIMER_TARGET2)

#define ESP32S3_IRQ_SPI_MEM_REJECT                      (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SPI_MEM_REJECT)
#define ESP32S3_IRQ_DCACHE_PRELOAD                      (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DCACHE_PRELOAD)
#define ESP32S3_IRQ_ICACHE_PRELOAD                      (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_ICACHE_PRELOAD)
#define ESP32S3_IRQ_DCACHE_SYNC                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DCACHE_SYNC)

#define ESP32S3_IRQ_SREG1                               ESP32S3_IRQ_PWM1
#define ESP32S3_NIRQS_SREG1                             32

#define ESP32S3_IRQ_ICACHE_SYNC                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_ICACHE_SYNC)
#define ESP32S3_IRQ_APB_ADC                             (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_APB_ADC)
#define ESP32S3_IRQ_DMA_IN_CH0                          (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_IN_CH0)
#define ESP32S3_IRQ_DMA_IN_CH1                          (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_IN_CH1)
#define ESP32S3_IRQ_DMA_IN_CH2                          (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_IN_CH2)
#define ESP32S3_IRQ_DMA_IN_CH3                          (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_IN_CH3)

#define ESP32S3_IRQ_DMA_IN_CH4                          (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_IN_CH4)
#define ESP32S3_IRQ_DMA_OUT_CH0                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_OUT_CH0)
#define ESP32S3_IRQ_DMA_OUT_CH1                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_OUT_CH1)
#define ESP32S3_IRQ_DMA_OUT_CH2                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_OUT_CH2)
#define ESP32S3_IRQ_DMA_OUT_CH3                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_OUT_CH3)
#define ESP32S3_IRQ_DMA_OUT_CH4                         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_OUT_CH4)
#define ESP32S3_IRQ_RSA                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_RSA)
#define ESP32S3_IRQ_AES                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_AES)
#define ESP32S3_IRQ_SHA                                 (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_SHA)
#define ESP32S3_IRQ_INT_FROM_CPU0                       (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_INT_FROM_CPU0)

#define ESP32S3_IRQ_INT_FROM_CPU1                       (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_INT_FROM_CPU1)
#define ESP32S3_IRQ_INT_FROM_CPU2                       (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_INT_FROM_CPU2)
#define ESP32S3_IRQ_INT_FROM_CPU3                       (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_INT_FROM_CPU3)
#define ESP32S3_IRQ_ASSIST_DEBUG                        (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_ASSIST_DEBUG)
#define ESP32S3_IRQ_DMA_APB_PMS_MONITOR_VIOLATE         (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_APB_PMS_MONITOR_VIOLATE)
#define ESP32S3_IRQ_CORE_0_IRAM0_PMS_MONITOR_VIOLATE    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_0_IRAM0_PMS_MONITOR_VIOLATE)
#define ESP32S3_IRQ_CORE_0_DRAM0_PMS_MONITOR_VIOLATE    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_0_DRAM0_PMS_MONITOR_VIOLATE)
#define ESP32S3_IRQ_CORE_0_PIF_PMS_MONITOR_VIOLATE      (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_0_PIF_PMS_MONITOR_VIOLATE)
#define ESP32S3_IRQ_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE)
#define ESP32S3_IRQ_CORE_1_IRAM0_PMS_MONITOR_VIOLATE    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_1_IRAM0_PMS_MONITOR_VIOLATE)

#define ESP32S3_IRQ_CORE_1_DRAM0_PMS_MONITOR_VIOLATE    (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_1_DRAM0_PMS_MONITOR_VIOLATE)
#define ESP32S3_IRQ_CORE_1_PIF_PMS_MONITOR_VIOLATE      (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_1_PIF_PMS_MONITOR_VIOLATE)
#define ESP32S3_IRQ_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE)
#define ESP32S3_IRQ_BACKUP_PMS_VIOLATE                  (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_BACKUP_PMS_VIOLATE)
#define ESP32S3_IRQ_CACHE_CORE0_ACS                     (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CACHE_CORE0_ACS)
#define ESP32S3_IRQ_CACHE_CORE1_ACS                     (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_CACHE_CORE1_ACS)

#define ESP32S3_IRQ_SREG2                               ESP32S3_IRQ_ICACHE_SYNC
#define ESP32S3_NIRQS_SREG2                             32

#define ESP32S3_IRQ_USB_DEVICE                          (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_USB_DEVICE)
#define ESP32S3_IRQ_PERIPH_BACKUP                       (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_PERIPH_BACKUP)
#define ESP32S3_IRQ_DMA_EXTMEM_REJECT                   (XTENSA_IRQ_FIRSTPERIPH + ESP32S3_PERIPH_DMA_EXTMEM_REJECT)

#define ESP32S3_IRQ_SREG3                               ESP32S3_IRQ_USB_DEVICE
#define ESP32S3_NIRQS_SREG3                             3

#define ESP32S3_NIRQ_PERIPH                             ESP32S3_NPERIPHERALS

#ifdef CONFIG_ESP32S3_GPIO_IRQ

/* Second level GPIO interrupts. GPIO interrupts are decoded and dispatched
 * as a second level of decoding: The first level dispatches to the GPIO
 * interrupt handler. The second to the decoded GPIO interrupt handler.
 */

#  define ESP32S3_NIRQ_GPIO             49
#  define ESP32S3_FIRST_GPIOIRQ         (XTENSA_NIRQ_INTERNAL + ESP32S3_NIRQ_PERIPH)
#  define ESP32S3_LAST_GPIOIRQ          (ESP32S3_FIRST_GPIOIRQ + ESP32S3_NIRQ_GPIO - 1)
#  define ESP32S3_PIN2IRQ(p)            ((p) + ESP32S3_FIRST_GPIOIRQ)
#  define ESP32S3_IRQ2PIN(i)            ((i) - ESP32S3_FIRST_GPIOIRQ)
#else
#  define ESP32S3_NIRQ_GPIO             0
#endif

#ifdef CONFIG_ESP32S3_RTCIO_IRQ

/* Second level RTC interrupts.  RTC interrupts are decoded and dispatched
 * as a second level of decoding:  The first level dispatches to the RTC
 * interrupt handler.  The second to the decoded RTC interrupt handler.
 * A third level might be required to be implemented on the driver (e.g.
 * Touch pads)
 */

#  define ESP32S3_NIRQ_RTCIO_PERIPH                 21
#  define ESP32S3_NIRQ_RTCIO_TOUCHPAD               15
#  define ESP32S3_NIRQ_RTCIO                        (ESP32S3_NIRQ_RTCIO_PERIPH+ESP32S3_NIRQ_RTCIO_TOUCHPAD)

#  define ESP32S3_FIRST_RTCIOIRQ_PERIPH             (XTENSA_NIRQ_INTERNAL+ESP32S3_NIRQ_PERIPH+ESP32S3_NIRQ_GPIO)
#  define ESP32S3_LAST_RTCIOIRQ_PERIPH              (ESP32S3_FIRST_RTCIOIRQ_PERIPH+ESP32S3_NIRQ_RTCIO_PERIPH-1)
#  define ESP32S3_IRQ_RTC_SLP_WAKEUP                (ESP32S3_FIRST_RTCIOIRQ_PERIPH+0)
#  define ESP32S3_IRQ_RTC_SLP_REJECT                (ESP32S3_FIRST_RTCIOIRQ_PERIPH+1)
#  define ESP32S3_IRQ_RTC_SDIO_IDLE                 (ESP32S3_FIRST_RTCIOIRQ_PERIPH+2)
#  define ESP32S3_IRQ_RTC_WDT                       (ESP32S3_FIRST_RTCIOIRQ_PERIPH+3)
#  define ESP32S3_IRQ_RTC_TOUCH_SCAN_DONE           (ESP32S3_FIRST_RTCIOIRQ_PERIPH+4)
#  define ESP32S3_IRQ_RTC_ULP_CP                    (ESP32S3_FIRST_RTCIOIRQ_PERIPH+5)
#  define ESP32S3_IRQ_RTC_TOUCH_DONE                (ESP32S3_FIRST_RTCIOIRQ_PERIPH+6)
#  define ESP32S3_IRQ_RTC_TOUCH_ACTIVE              (ESP32S3_FIRST_RTCIOIRQ_PERIPH+7)
#  define ESP32S3_IRQ_RTC_TOUCH_INACTIVE            (ESP32S3_FIRST_RTCIOIRQ_PERIPH+8)
#  define ESP32S3_IRQ_RTC_BROWN_OUT                 (ESP32S3_FIRST_RTCIOIRQ_PERIPH+9)
#  define ESP32S3_IRQ_RTC_MAIN_TIMER                (ESP32S3_FIRST_RTCIOIRQ_PERIPH+10)
#  define ESP32S3_IRQ_RTC_SARADC1                   (ESP32S3_FIRST_RTCIOIRQ_PERIPH+11)
#  define ESP32S3_IRQ_RTC_TSENS                     (ESP32S3_FIRST_RTCIOIRQ_PERIPH+12)
#  define ESP32S3_IRQ_RTC_COCPU                     (ESP32S3_FIRST_RTCIOIRQ_PERIPH+13)
#  define ESP32S3_IRQ_RTC_SARADC2                   (ESP32S3_FIRST_RTCIOIRQ_PERIPH+14)
#  define ESP32S3_IRQ_RTC_SWD                       (ESP32S3_FIRST_RTCIOIRQ_PERIPH+15)
#  define ESP32S3_IRQ_RTC_XTAL32K_DEAD              (ESP32S3_FIRST_RTCIOIRQ_PERIPH+16)
#  define ESP32S3_IRQ_RTC_COCPU_TRAP                (ESP32S3_FIRST_RTCIOIRQ_PERIPH+17)
#  define ESP32S3_IRQ_RTC_TOUCH_TIMEOUT             (ESP32S3_FIRST_RTCIOIRQ_PERIPH+18)
#  define ESP32S3_IRQ_RTC_GLITCH_DET                (ESP32S3_FIRST_RTCIOIRQ_PERIPH+19)
#  define ESP32S3_IRQ_RTC_TOUCH_APPROACH_LOOP_DONE  (ESP32S3_FIRST_RTCIOIRQ_PERIPH+20)

#  define ESP32S3_FIRST_RTCIOIRQ_TOUCHPAD           (ESP32S3_LAST_RTCIOIRQ_PERIPH+1)
#  define ESP32S3_LAST_RTCIOIRQ_TOUCHPAD            (ESP32S3_FIRST_RTCIOIRQ_TOUCHPAD+ESP32S3_NIRQ_RTCIO_TOUCHPAD-1)
#  define ESP32S3_TOUCHPAD2IRQ(t)                   ((t) + ESP32S3_FIRST_RTCIOIRQ_TOUCHPAD)
#  define ESP32S3_IRQ2TOUCHPAD(i)                   ((i) - ESP32S3_FIRST_RTCIOIRQ_TOUCHPAD)
#else
#  define ESP32S3_NIRQ_RTCIO                        0
#endif

/* Total number of interrupts */

#define NR_IRQS                     (XTENSA_NIRQ_INTERNAL + ESP32S3_NIRQ_PERIPH + ESP32S3_NIRQ_GPIO + ESP32S3_NIRQ_RTCIO)

/* Xtensa CPU Interrupts.
 *
 * Each of the two CPUs (PRO and APP) have 32 interrupts each, of which
 * 26 can be mapped to peripheral interrupts:
 *
 *   Level triggered peripherals (21 total):
 *     0-5, 8-9, 12-13, 17-18 - Priority 1
 *     19-21                  - Priority 2
 *     23, 27                 - Priority 3
 *     24-25                  - Priority 4
 *     26, 31                 - Priority 5
 *   Edge triggered peripherals (4 total):
 *     10                     - Priority 1
 *     22                     - Priority 3
 *     28, 30                 - Priority 4
 *   NMI (1 total):
 *     14                     - NMI
 *
 * CPU peripheral interrupts can be a assigned to a CPU interrupt using the
 * PRO_*_MAP_REG or APP_*_MAP_REG.  There are a pair of these registers for
 * each peripheral source.  Multiple peripheral interrupt sources can be
 * mapped to the same CPU interrupt.
 *
 * The remaining, six, internal CPU interrupts are:
 *
 *   6   Timer0    - Priority 1
 *   7   Software  - Priority 1
 *   11  Profiling - Priority 3
 *   15  Timer1    - Priority 3
 *   16  Timer2    - Priority 5
 *   29  Software  - Priority 3
 *
 * A peripheral interrupt can be disabled
 */

#define ESP32S3_CPUINT_LEVELPERIPH_0  0
#define ESP32S3_CPUINT_LEVELPERIPH_1  1
#define ESP32S3_CPUINT_LEVELPERIPH_2  2
#define ESP32S3_CPUINT_LEVELPERIPH_3  3
#define ESP32S3_CPUINT_LEVELPERIPH_4  4
#define ESP32S3_CPUINT_LEVELPERIPH_5  5
#define ESP32S3_CPUINT_LEVELPERIPH_6  8
#define ESP32S3_CPUINT_LEVELPERIPH_7  9
#define ESP32S3_CPUINT_LEVELPERIPH_8  12
#define ESP32S3_CPUINT_LEVELPERIPH_9  13
#define ESP32S3_CPUINT_LEVELPERIPH_10 17
#define ESP32S3_CPUINT_LEVELPERIPH_11 18
#define ESP32S3_CPUINT_LEVELPERIPH_12 19
#define ESP32S3_CPUINT_LEVELPERIPH_13 20
#define ESP32S3_CPUINT_LEVELPERIPH_14 21
#define ESP32S3_CPUINT_LEVELPERIPH_15 23
#define ESP32S3_CPUINT_LEVELPERIPH_16 24
#define ESP32S3_CPUINT_LEVELPERIPH_17 25
#define ESP32S3_CPUINT_LEVELPERIPH_18 26
#define ESP32S3_CPUINT_LEVELPERIPH_19 27
#define ESP32S3_CPUINT_LEVELPERIPH_20 31

#define ESP32S3_CPUINT_NLEVELPERIPHS  21
#define ESP32S3_CPUINT_LEVELSET       0x8fbe333f

#define ESP32S3_CPUINT_EDGEPERIPH_0   10
#define ESP32S3_CPUINT_EDGEPERIPH_1   22
#define ESP32S3_CPUINT_EDGEPERIPH_2   28
#define ESP32S3_CPUINT_EDGEPERIPH_3   30

#define ESP32S3_CPUINT_NEDGEPERIPHS   4
#define ESP32S3_CPUINT_EDGESET        0x50400400

#define ESP32S3_CPUINT_NNMIPERIPHS    1
#define ESP32S3_CPUINT_NMISET         0x00004000

#define ESP32S3_CPUINT_MAC            0
#define ESP32S3_CPUINT_TIMER0         6
#define ESP32S3_CPUINT_SOFTWARE0      7
#define ESP32S3_CPUINT_PROFILING      11
#define ESP32S3_CPUINT_TIMER1         15
#define ESP32S3_CPUINT_TIMER2         16
#define ESP32S3_CPUINT_SOFTWARE1      29

#define ESP32S3_CPUINT_NINTERNAL      6

#define ESP32S3_NCPUINTS              32
#define ESP32S3_CPUINT_MAX            (ESP32S3_NCPUINTS - 1)
#define ESP32S3_CPUINT_PERIPHSET      0xdffe773f
#define ESP32S3_CPUINT_INTERNALSET    0x200188c0

/* Priority 1:   0-10, 12-13, 17-18    (15)
 * Priority 2:   19-21                 (3)
 * Priority 3:   11, 15, 22-23, 27, 29 (6)
 * Priority 4:   24-25, 28, 30         (4)
 * Priority 5:   16, 26, 31            (3)
 * Priority NMI: 14                    (1)
 */

#define ESP32S3_INTPRI1_MASK          0x000637ff
#define ESP32S3_INTPRI2_MASK          0x00380000
#define ESP32S3_INTPRI3_MASK          0x28c08800
#define ESP32S3_INTPRI4_MASK          0x53000000
#define ESP32S3_INTPRI5_MASK          0x84010000
#define ESP32S3_INTNMI_MASK           0x00004000

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_INCLUDE_ESP32S3_IRQ_H */
