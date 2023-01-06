/****************************************************************************
 * arch/risc-v/include/esp32c6/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_ESP32C6_IRQ_H
#define __ARCH_RISCV_INCLUDE_ESP32C6_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <arch/csr.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Matrix
 * The Interrupt Matrix embedded in the ESP32-C6 independently allocates
 * peripheral interrupt sources to the CPUs’ peripheral interrupts.
 * This configuration is highly flexible in order to meet many different
 * needs.
 *
 * Features
 * - Accepts 77 peripheral interrupt sources as input.
 * - Generate 31 peripheral interrupts to CPU as output.
 * - Queries current interrupt status of peripheral interrupt sources.
 */

#define ESP32C6_WIFI_MAC_PERIPH             0      /* interrupt of WiFi MAC, level */
#define ESP32C6_WIFI_MAC_NMI_PERIPH         1      /* interrupt of WiFi MAC, NMI, use if MAC have bug to fix in NMI */
#define ESP32C6_WIFI_PWR_PERIPH             2
#define ESP32C6_WIFI_BB_PERIPH              3      /* interrupt of WiFi BB, level, we can do some calibartion */
#define ESP32C6_BT_MAC_PERIPH               4
#define ESP32C6_BT_BB_PERIPH                5      /* interrupt of BT BB, level */
#define ESP32C6_BT_BB_NMI_PERIPH            6      /* interrupt of BT BB, NMI, use if BB have bug to fix in NMI */
#define ESP32C6_LP_TIMER_PERIPH             7
#define ESP32C6_COEX_PERIPH                 8
#define ESP32C6_BLE_TIMER_PERIPH            9
#define ESP32C6_BLE_SEC_PERIPH              10
#define ESP32C6_I2C_MASTER_PERIPH           11      /* interrupt of I2C Master, level */
#define ESP32C6_ZB_MAC_PERIPH               12
#define ESP32C6_PMU_PERIPH                  13
#define ESP32C6_EFUSE_PERIPH                14      /* interrupt of efuse, level, not likely to use */
#define ESP32C6_LP_RTC_TIMER_PERIPH         15
#define ESP32C6_LP_UART_PERIPH              16
#define ESP32C6_LP_I2C_PERIPH               17
#define ESP32C6_LP_WDT_PERIPH               18
#define ESP32C6_LP_PERI_TIMEOUT_PERIPH      19
#define ESP32C6_LP_APM_M0_PERIPH            20
#define ESP32C6_LP_APM_M1_PERIPH            21
#define ESP32C6_FROM_CPU_PERIPH0            22      /* interrupt0 generated from a CPU, level */
#define ESP32C6_FROM_CPU_PERIPH1            23      /* interrupt1 generated from a CPU, level */
#define ESP32C6_FROM_CPU_PERIPH2            24      /* interrupt2 generated from a CPU, level */
#define ESP32C6_FROM_CPU_PERIPH3            25      /* interrupt3 generated from a CPU, level */
#define ESP32C6_ASSIST_DEBUG_PERIPH         26      /* interrupt of Assist debug module, level */
#define ESP32C6_TRACE_PERIPH                27
#define ESP32C6_CACHE_PERIPH                28
#define ESP32C6_CPU_PERI_TIMEOUT_PERIPH     29
#define ESP32C6_GPIO_PERIPH                 30      /* interrupt of GPIO, level */
#define ESP32C6_GPIO_NMI_PERIPH             31      /* interrupt of GPIO, NMI */
#define ESP32C6_PAU_PERIPH                  32
#define ESP32C6_HP_PERI_TIMEOUT_PERIPH      33
#define ESP32C6_MODEM_PERI_TIMEOUT_PERIPH   34
#define ESP32C6_HP_APM_M0_PERIPH            35
#define ESP32C6_HP_APM_M1_PERIPH            36
#define ESP32C6_HP_APM_M2_PERIPH            37
#define ESP32C6_HP_APM_M3_PERIPH            38
#define ESP32C6_LP_APM0_PERIPH              39
#define ESP32C6_MSPI_PERIPH                 40
#define ESP32C6_I2S1_PERIPH                 41      /* interrupt of I2S1, level */
#define ESP32C6_UHCI0_PERIPH                42      /* interrupt of UHCI0, level */
#define ESP32C6_UART0_PERIPH                43      /* interrupt of UART0, level */
#define ESP32C6_UART1_PERIPH                44      /* interrupt of UART1, level */
#define ESP32C6_LEDC_PERIPH                 45      /* interrupt of LED PWM, level */
#define ESP32C6_TWAI0_PERIPH                46      /* interrupt of can0, level */
#define ESP32C6_TWAI1_PERIPH                47      /* interrupt of can1, level */
#define ESP32C6_USB_SERIAL_JTAG_PERIPH      48      /* interrupt of USB, level */
#define ESP32C6_RMT_PERIPH                  49      /* interrupt of remote controller, level */
#define ESP32C6_I2C_EXT0_PERIPH             50      /* interrupt of I2C controller1, level */
#define ESP32C6_TG0_T0_LEVEL_PERIPH         51      /* interrupt of TIMER_GROUP0, TIMER0, level */
#define ESP32C6_TG0_T1_LEVEL_PERIPH         52      /* interrupt of TIMER_GROUP0, TIMER1, level */
#define ESP32C6_TG0_WDT_LEVEL_PERIPH        53      /* interrupt of TIMER_GROUP0, WATCH DOG, level */
#define ESP32C6_TG1_T0_LEVEL_PERIPH         54      /* interrupt of TIMER_GROUP1, TIMER0, level */
#define ESP32C6_TG1_T1_LEVEL_PERIPH         55      /* interrupt of TIMER_GROUP1, TIMER1, level */
#define ESP32C6_TG1_WDT_LEVEL_PERIPH        56      /* interrupt of TIMER_GROUP1, WATCHDOG, level */
#define ESP32C6_SYSTIMER_TARGET0_EDGE_PERIPH 57     /* interrupt of system timer 0, EDGE */
#define ESP32C6_SYSTIMER_TARGET1_EDGE_PERIPH 58     /* interrupt of system timer 1, EDGE */
#define ESP32C6_SYSTIMER_TARGET2_EDGE_PERIPH 59     /* interrupt of system timer 2, EDGE */
#define ESP32C6_APB_ADC_PERIPH               60     /* interrupt of APB ADC, level */
#define ESP32C6_MCPWM0_PERIPH                61     /* interrupt of MCPWM0, level */
#define ESP32C6_PCNT_PERIPH                  62 
#define ESP32C6_PARL_IO_PERIPH               63
#define ESP32C6_SLC0_PERIPH                  64 
#define ESP32C6_SLC_PERIPH                   65
#define ESP32C6_DMA_IN_CH0_PERIPH            66     /* interrupt of general DMA IN channel 0, level */
#define ESP32C6_DMA_IN_CH1_PERIPH            67     /* interrupt of general DMA IN channel 1, level */
#define ESP32C6_DMA_IN_CH2_PERIPH            68     /* interrupt of general DMA IN channel 2, level */
#define ESP32C6_DMA_OUT_CH0_PERIPH           69     /* interrupt of general DMA OUT channel 0, level */
#define ESP32C6_DMA_OUT_CH1_PERIPH           70     /* interrupt of general DMA OUT channel 1, level */
#define ESP32C6_DMA_OUT_CH2_PERIPH           71     /* interrupt of general DMA OUT channel 2, level */
#define ESP32C6_GSPI2_PERIPH                 72 
#define ESP32C6_AES_PERIPH                   73     /* interrupt of AES accelerator, level */
#define ESP32C6_SHA_PERIPH                   74     /* interrupt of SHA accelerator, level */
#define ESP32C6_RSA_PERIPH                   75     /* interrupt of RSA accelerator, level */
#define ESP32C6_ECC_PERIPH                   76     /* interrupt of ECC accelerator, level */

/* Total number of peripherals */

#define ESP32C6_NPERIPHERALS            77

/* CPU Interrupts.
 * The ESP32-C6 CPU interrupt controller accepts 31 asynchronous interrupts.
 */

#define ESP32C6_CPUINT_MIN             1
#define ESP32C6_CPUINT_MAX             31

#define ESP32C6_NCPUINTS               32

#define ESP32C6_CPUINT_PERIPHSET       0xffffffff

/* IRQ numbers. */

/* ecall is dispatched like normal interrupts.  It occupies an IRQ number. */

#define RISCV_NIRQ_INTERRUPTS       16  /* Number of RISC-V dispatched interrupts. */
#define ESP32C6_IRQ_FIRSTPERIPH     16  /* First peripheral IRQ number */

/* IRQ numbers for peripheral interrupts coming through the Interrupt
 * Matrix.
 */

#define ESP32C6_IRQ2PERIPH(irq)       ((irq) - ESP32C6_IRQ_FIRSTPERIPH)
#define ESP32C6_PERIPH2IRQ(id)        ((id) + ESP32C6_IRQ_FIRSTPERIPH)

/* Peripheral IRQs */

#define ESP32C6_IRQ_WIFI_MAC            (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_WIFI_MAC_PERIPH)
#define ESP32C6_IRQ_WIFI_MAC_NMI        (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_WIFI_MAC_NMI_PERIPH)
#define ESP32C6_IRQ_WIFI_PWR            (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_WIFI_PWR_PERIPH)
#define ESP32C6_IRQ_WIFI_BB             (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_WIFI_BB_PERIPH)

#define ESP32C6_IRQ_BT_MAC              (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_BT_MAC_PERIPH)
#define ESP32C6_IRQ_BT_BB               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_BT_BB_PERIPH)
#define ESP32C6_IRQ_BT_BB_NMI           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_BT_BB_NMI_PERIPH)
#define ESP32C6_IRQ_LP_TIMER            (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_TIMER_PERIPH)

#define ESP32C6_IRQ_COEX                (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_COEX_PERIPH)
#define ESP32C6_IRQ_BLE_TIMER           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_BLE_TIMER_PERIPH)
#define ESP32C6_IRQ_BLE_SEC             (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_BLE_SEC_PERIPH)
#define ESP32C6_IRQ_I2C_MASTER          (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_I2C_MASTER_PERIPH)

#define ESP32C6_IRQ_ZB_MAC              (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_ZB_MAC_PERIPH)
#define ESP32C6_IRQ_PMU                 (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_PMU_PERIPH)
#define ESP32C6_IRQ_EFUSE               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_EFUSE_PERIPH)
#define ESP32C6_IRQ_LP_RTC_TIMER        (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_RTC_TIMER_PERIPH)

#define ESP32C6_IRQ_LP_UART             (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_UART_PERIPH)
#define ESP32C6_IRQ_LP_I2C              (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_I2C_PERIPH)
#define ESP32C6_IRQ_LP_WDT              (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_WDT_PERIPH)
#define ESP32C6_IRQ_LP_PERI_TIMEOUT     (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_PERI_TIMEOUT_PERIPH)

#define ESP32C6_IRQ_LP_APM_M0           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_APM_M0_PERIPH)
#define ESP32C6_IRQ_LP_APM_M1           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_APM_M1_PERIPH)
#define ESP32C6_IRQ_FROM_CPU_PERIPH0    (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_FROM_CPU_PERIPH0)
#define ESP32C6_IRQ_FROM_CPU_PERIPH1    (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_FROM_CPU_PERIPH1)

#define ESP32C6_IRQ_FROM_CPU_PERIPH2    (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_FROM_CPU_PERIPH2)
#define ESP32C6_IRQ_FROM_CPU_PERIPH3    (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_FROM_CPU_PERIPH3)
#define ESP32C6_IRQ_ASSIST_DEBUG        (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_ASSIST_DEBUG_PERIPH)
#define ESP32C6_IRQ_TRACE               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TRACE_PERIPH)

#define ESP32C6_IRQ_CACHE               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_CACHE_PERIPH)
#define ESP32C6_IRQ_CPU_PERI_TIMEOUT    (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_CPU_PERI_TIMEOUT_PERIPH)
#define ESP32C6_IRQ_GPIO                (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_GPIO_PERIPH)
#define ESP32C6_IRQ_GPIO_NMI            (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_GPIO_NMI_PERIPH)

#define ESP32C6_IRQ_PAU                 (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_PAU_PERIPH)
#define ESP32C6_IRQ_HP_PERI_TIMEOUT     (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_HP_PERI_TIMEOUT_PERIPH)
#define ESP32C6_IRQ_MODEM_PERI_TIMEOUT  (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_MODEM_PERI_TIMEOUT_PERIPH)
#define ESP32C6_IRQ_HP_APM_M0           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_HP_APM_M0_PERIPH)

#define ESP32C6_IRQ_HP_APM_M1           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_HP_APM_M1_PERIPH)
#define ESP32C6_IRQ_HP_APM_M2           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_HP_APM_M2_PERIPH)
#define ESP32C6_IRQ_HP_APM_M3           (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_HP_APM_M3_PERIPH)
#define ESP32C6_IRQ_LP_APM0             (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LP_APM0_PERIPH)

#define ESP32C6_IRQ_MSPI                (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_MSPI_PERIPH)
#define ESP32C6_IRQ_I2S1                (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_I2S1_PERIPH)
#define ESP32C6_IRQ_UHCI0               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_UHCI0_PERIPH)
#define ESP32C6_IRQ_UART0               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_UART0_PERIPH)

#define ESP32C6_IRQ_UART1               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_UART1_PERIPH)
#define ESP32C6_IRQ_LEDC                (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_LEDC_PERIPH)
#define ESP32C6_IRQ_TWAI0               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TWAI0_PERIPH)
#define ESP32C6_IRQ_TWAI1               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TWAI1_PERIPH)

#define ESP32C6_IRQ_USB_SERIAL_JTAG     (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_USB_SERIAL_JTAG_PERIPH)
#define ESP32C6_IRQ_RMT                 (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_RMT_PERIPH)
#define ESP32C6_IRQ_I2C_EXT0            (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_I2C_EXT0_PERIPH)
#define ESP32C6_IRQ_TG0_T0_LEVEL        (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TG0_T0_LEVEL_PERIPH)

#define ESP32C6_IRQ_TG0_T1_LEVEL        (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TG0_T1_LEVEL_PERIPH)
#define ESP32C6_IRQ_TG0_WDT_LEVEL       (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TG0_WDT_LEVEL_PERIPH)
#define ESP32C6_IRQ_TG1_T0_LEVEL        (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TG1_T0_LEVEL_PERIPH)
#define ESP32C6_IRQ_TG1_T1_LEVEL        (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TG1_T1_LEVEL_PERIPH)

#define ESP32C6_IRQ_TG1_WDT_LEVEL             (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_TG1_WDT_LEVEL_PERIPH)
#define ESP32C6_IRQ_SYSTIMER_TARGET0_EDGE     (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_SYSTIMER_TARGET0_EDGE_PERIPH)
#define ESP32C6_IRQ_SYSTIMER_TARGET1_EDGE     (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_SYSTIMER_TARGET1_EDGE_PERIPH)
#define ESP32C6_IRQ_SYSTIMER_TARGET2_EDGE     (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_SYSTIMER_TARGET2_EDGE_PERIPH)

#define ESP32C6_IRQ_APB_ADC             (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_APB_ADC_PERIPH)
#define ESP32C6_IRQ_MCPWM0              (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_MCPWM0_PERIPH)
#define ESP32C6_IRQ_PCNT                (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_PCNT_PERIPH)
#define ESP32C6_IRQ_PARL_IO             (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_PARL_IO_PERIPH)

#define ESP32C6_IRQ_SLC0                (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_SLC0_PERIPH)
#define ESP32C6_IRQ_SLC                 (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_SLC_PERIPH)
#define ESP32C6_IRQ_DMA_IN_CH0          (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_DMA_IN_CH0_PERIPH)
#define ESP32C6_IRQ_DMA_IN_CH1          (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_DMA_IN_CH1_PERIPH)

#define ESP32C6_IRQ_DMA_IN_CH2          (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_DMA_IN_CH2_PERIPH)
#define ESP32C6_IRQ_DMA_OUT_CH0         (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_DMA_OUT_CH0_PERIPH)
#define ESP32C6_IRQ_DMA_OUT_CH1         (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_DMA_OUT_CH1_PERIPH)
#define ESP32C6_IRQ_DMA_OUT_CH2         (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_DMA_OUT_CH2_PERIPH)

#define ESP32C6_IRQ_GSPI2               (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_GSPI2_PERIPH)
#define ESP32C6_IRQ_AES                 (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_AES_PERIPH)
#define ESP32C6_IRQ_SHA                 (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_SHA_PERIPH)
#define ESP32C6_IRQ_RSA                 (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_RSA_PERIPH)

#define ESP32C6_IRQ_CACHE_CORE0_ACS     (ESP32C6_IRQ_FIRSTPERIPH + ESP32C6_ECC_PERIPH)

#define ESP32C6_NIRQ_PERIPH             ESP32C6_NPERIPHERALS

/* Total number of IRQs: ecall + Number of peripheral IRQs + GPIOs IRQs. */

#define NR_IRQS  (RISCV_NIRQ_INTERRUPTS + ESP32C6_NIRQ_PERIPH)

#endif /* __ARCH_RISCV_INCLUDE_ESP32C6_IRQ_H */
