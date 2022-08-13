/****************************************************************************
 * arch/risc-v/include/esp32c3/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_ESP32C3_IRQ_H
#define __ARCH_RISCV_INCLUDE_ESP32C3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Matrix
 *
 * The Interrupt Matrix embedded in the ESP32C3 independently allocates
 * peripheral interrupt sources to the CPUs’ peripheral interrupts.
 * This configuration is highly flexible in order to meet many different
 * needs.
 *
 * Features
 * - Accepts 62 peripheral interrupt sources as input.
 * - Generate 31 peripheral interrupts to CPU as output.
 * - Queries current interrupt status of peripheral interrupt sources.
 */

#define ESP32C3_PERIPH_WIFI_MAC            0
#define ESP32C3_PERIPH_WIFI_MAC_NMI        1
#define ESP32C3_PERIPH_WIFI_PWR            2
#define ESP32C3_PERIPH_WIFI_BB             3
#define ESP32C3_PERIPH_BT_MAC              4
#define ESP32C3_PERIPH_BT_BB               5
#define ESP32C3_PERIPH_BT_BB_NMI           6
#define ESP32C3_PERIPH_RWBT_IRQ            7
#define ESP32C3_PERIPH_RWBLE_IRQ           8
#define ESP32C3_PERIPH_RWBT_NMI            9

#define ESP32C3_PERIPH_RWBLE_NMI           10
#define ESP32C3_PERIPH_I2C_MASTER          11
#define ESP32C3_PERIPH_SLC0                12
#define ESP32C3_PERIPH_SLC1                13
#define ESP32C3_PERIPH_APB_CTRL            14
#define ESP32C3_PERIPH_UHCI0               15
#define ESP32C3_PERIPH_GPIO                16
#define ESP32C3_PERIPH_GPIO_NMI            17
#define ESP32C3_PERIPH_SPI1                18
#define ESP32C3_PERIPH_SPI2                19

#define ESP32C3_PERIPH_I2S1                20
#define ESP32C3_PERIPH_UART0               21
#define ESP32C3_PERIPH_UART1               22
#define ESP32C3_PERIPH_LEDC                23
#define ESP32C3_PERIPH_EFUSE               24
#define ESP32C3_PERIPH_TWAI                25
#define ESP32C3_PERIPH_USB                 26
#define ESP32C3_PERIPH_RTC_CORE            27
#define ESP32C3_PERIPH_RMT                 28
#define ESP32C3_PERIPH_I2C_EXT0            29

#define ESP32C3_PERIPH_TIMER1              30
#define ESP32C3_PERIPH_TIMER2              31
#define ESP32C3_PERIPH_TG0_T0              32
#define ESP32C3_PERIPH_TG0_WDT             33
#define ESP32C3_PERIPH_TG1_T0              34
#define ESP32C3_PERIPH_TG1_WDT             35
#define ESP32C3_PERIPH_CACHE_IA            36
#define ESP32C3_PERIPH_SYSTIMER_T0         37
#define ESP32C3_PERIPH_SYSTIMER_T1         38
#define ESP32C3_PERIPH_SYSTIMER_T2         39

#define ESP32C3_PERIPH_SPIMEM_REJECT_CACHE 40
#define ESP32C3_PERIPH_ICACHE_PRELOAD0     41
#define ESP32C3_PERIPH_ICACHE_SYNC0        42
#define ESP32C3_PERIPH_APB_ADC             43
#define ESP32C3_PERIPH_DMA_CH0             44
#define ESP32C3_PERIPH_DMA_CH1             45
#define ESP32C3_PERIPH_DMA_CH2             46
#define ESP32C3_PERIPH_RSA                 47
#define ESP32C3_PERIPH_AES                 48
#define ESP32C3_PERIPH_SHA                 49

#define ESP32C3_PERIPH_FROM_CPU_INT0       50
#define ESP32C3_PERIPH_FROM_CPU_INT1       51
#define ESP32C3_PERIPH_FROM_CPU_INT2       52
#define ESP32C3_PERIPH_FROM_CPU_INT3       53
#define ESP32C3_PERIPH_ASSIST_DEBUG        54
#define ESP32C3_PERIPH_DMA_APBPERI_PMS     55
#define ESP32C3_PERIPH_CORE0_IRAM0_PMS     56
#define ESP32C3_PERIPH_CORE0_DRAM0_PMS     57
#define ESP32C3_PERIPH_CORE0_PIF_PMS       58
#define ESP32C3_PERIPH_CORE0_PIF_PMS_SZIE  59

#define ESP32C3_PERIPH_BAK_PMS_VIOLATE     60
#define ESP32C3_PERIPH_CACHE_CORE0_ACS     61

/* Total number of peripherals */

#define ESP32C3_NPERIPHERALS            62

/* CPU Interrupts.
 *
 * The ESP32-C3 CPU interrupt controller accepts 31 asynchronous interrupts.
 */

#define ESP32C3_CPUINT_MIN             1
#define ESP32C3_CPUINT_MAX             31

/* Reserved CPU interrupt for specific drivers */

#define ESP32C3_CPUINT_WMAC            1  /* Wi-Fi MAC */
#define ESP32C3_CPUINT_BT_BB           5  /* BT BB */
#define ESP32C3_CPUINT_RWBLE           8  /* RW BLE */

/* IRQ numbers. */

/* ecall is dispatched like normal interrupts.  It occupies an IRQ number. */

#define RISCV_NIRQ_INTERRUPTS       16  /* Number of RISC-V dispatched interrupts. */
#define ESP32C3_IRQ_FIRSTPERIPH     16  /* First peripheral IRQ number */

/* Peripheral IRQs */

#define ESP32C3_IRQ_WIFI_MAC            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_WIFI_MAC)
#define ESP32C3_IRQ_WIFI_MAC_NMI        (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_WIFI_MAC_NMI)
#define ESP32C3_IRQ_WIFI_PWR            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_WIFI_PWR)
#define ESP32C3_IRQ_WIFI_BB             (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_WIFI_BB)
#define ESP32C3_IRQ_BT_MAC              (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_BT_MAC)
#define ESP32C3_IRQ_BT_BB               (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_BT_BB)
#define ESP32C3_IRQ_BT_BB_NMI           (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_BT_BB_NMI)
#define ESP32C3_IRQ_RWBT_IRQ            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_RWBT_IRQ)
#define ESP32C3_IRQ_RWBLE_IRQ           (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_RWBLE_IRQ)
#define ESP32C3_IRQ_RWBT_NMI            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_RWBT_NMI)
#define ESP32C3_IRQ_RWBLE_NMI           (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_RWBLE_NMI)
#define ESP32C3_IRQ_I2C_MASTER          (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_I2C_MASTER)
#define ESP32C3_IRQ_SLC0                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SLC0)
#define ESP32C3_IRQ_SLC1                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SLC1)
#define ESP32C3_IRQ_APB_CTRL            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_APB_CTRL)
#define ESP32C3_IRQ_UHCI0               (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_UHCI0)
#define ESP32C3_IRQ_GPIO                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_GPIO)
#define ESP32C3_IRQ_GPIO_NMI            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_GPIO_NMI)
#define ESP32C3_IRQ_SPI1                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SPI1)
#define ESP32C3_IRQ_SPI2                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SPI2)
#define ESP32C3_IRQ_I2S1                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_I2S1)
#define ESP32C3_IRQ_UART0               (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_UART0)
#define ESP32C3_IRQ_UART1               (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_UART1)
#define ESP32C3_IRQ_LEDC                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_LEDC)
#define ESP32C3_IRQ_EFUSE               (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_EFUSE)
#define ESP32C3_IRQ_TWAI                (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_TWAI)
#define ESP32C3_IRQ_USB                 (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_USB)
#define ESP32C3_IRQ_RTC_CORE            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_RTC_CORE)
#define ESP32C3_IRQ_RMT                 (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_RMT)
#define ESP32C3_IRQ_I2C_EXT0            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_I2C_EXT0)
#define ESP32C3_IRQ_TIMER1              (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_TIMER1)
#define ESP32C3_IRQ_TIMER2              (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_TIMER2)
#define ESP32C3_IRQ_TG0_T0              (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_TG0_T0)
#define ESP32C3_IRQ_TG0_WDT             (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_TG0_WDT)
#define ESP32C3_IRQ_TG1_T0              (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_TG1_T0)
#define ESP32C3_IRQ_TG1_WDT             (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_TG1_WDT)
#define ESP32C3_IRQ_CACHE_IA            (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_CACHE_IA)
#define ESP32C3_IRQ_SYSTIMER_T0         (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SYSTIMER_T0)
#define ESP32C3_IRQ_SYSTIMER_T1         (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SYSTIMER_T1)
#define ESP32C3_IRQ_SYSTIMER_T2         (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SYSTIMER_T2)
#define ESP32C3_IRQ_SPIMEM_REJECT_CACHE (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SPIMEM_REJECT_CACHE)
#define ESP32C3_IRQ_ICACHE_PRELOAD0     (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_ICACHE_PRELOAD0)
#define ESP32C3_IRQ_ICACHE_SYNC0        (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_ICACHE_SYNC0)
#define ESP32C3_IRQ_APB_ADC             (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_APB_ADC)
#define ESP32C3_IRQ_DMA_CH0             (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_DMA_CH0)
#define ESP32C3_IRQ_DMA_CH1             (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_DMA_CH1)
#define ESP32C3_IRQ_DMA_CH2             (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_DMA_CH2)
#define ESP32C3_IRQ_RSA                 (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_RSA)
#define ESP32C3_IRQ_AES                 (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_AES)
#define ESP32C3_IRQ_SHA                 (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_SHA)
#define ESP32C3_IRQ_FROM_CPU_INT0       (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_FROM_CPU_INT0)
#define ESP32C3_IRQ_FROM_CPU_INT1       (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_FROM_CPU_INT1)
#define ESP32C3_IRQ_FROM_CPU_INT2       (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_FROM_CPU_INT2)
#define ESP32C3_IRQ_FROM_CPU_INT3       (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_FROM_CPU_INT3)
#define ESP32C3_IRQ_ASSIST_DEBUG        (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_ASSIST_DEBUG)
#define ESP32C3_IRQ_DMA_APBPERI_PMS     (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_DMA_APBPERI_PMS)
#define ESP32C3_IRQ_CORE0_IRAM0_PMS     (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_CORE0_IRAM0_PMS)
#define ESP32C3_IRQ_CORE0_DRAM0_PMS     (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_CORE0_DRAM0_PMS)
#define ESP32C3_IRQ_CORE0_PIF_PMS       (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_CORE0_PIF_PMS)
#define ESP32C3_IRQ_CORE0_PIF_PMS_SZIE  (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_CORE0_PIF_PMS_SZIE)
#define ESP32C3_IRQ_BAK_PMS_VIOLATE     (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_BAK_PMS_VIOLATE)
#define ESP32C3_IRQ_CACHE_CORE0_ACS     (ESP32C3_IRQ_FIRSTPERIPH + ESP32C3_PERIPH_CACHE_CORE0_ACS)

#define ESP32C3_NIRQ_PERIPH             ESP32C3_NPERIPHERALS

/* Second level GPIO interrupts.  GPIO interrupts are decoded and dispatched
 * as a second level of decoding:  The first level dispatches to the GPIO
 * interrupt handler.  The second to the decoded GPIO interrupt handler.
 */

#ifdef CONFIG_ESP32C3_GPIO_IRQ
#  define ESP32C3_NIRQ_GPIO           22
#  define ESP32C3_FIRST_GPIOIRQ       (RISCV_NIRQ_INTERRUPTS + ESP32C3_NIRQ_PERIPH)
#  define ESP32C3_LAST_GPIOIRQ        (ESP32C3_FIRST_GPIOIRQ + ESP32C3_NIRQ_GPIO - 1)
#  define ESP32C3_PIN2IRQ(p)          ((p) + ESP32C3_FIRST_GPIOIRQ)
#  define ESP32C3_IRQ2PIN(i)          ((i) - ESP32C3_FIRST_GPIOIRQ)
#else
#  define ESP32C3_NIRQ_GPIO           0
#endif

/* Total number of IRQs: ecall + Number of peripheral IRQs + GPIOs IRQs. */

#define NR_IRQS  (RISCV_NIRQ_INTERRUPTS + ESP32C3_NIRQ_PERIPH + ESP32C3_NIRQ_GPIO)

#endif /* __ARCH_RISCV_INCLUDE_ESP32C3_IRQ_H */
