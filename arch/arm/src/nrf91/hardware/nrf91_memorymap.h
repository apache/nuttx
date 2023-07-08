/****************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_memorymap.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_MEMORYMAP_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map */

#define NRF91_FLASH_BASE        0x00000000 /* Flash memory Start Address */
#define NRF91_SRAM_BASE         0x20000000 /* SRAM Start Address */

#define NRF91_CORTEXM33_BASE    0xe0000000 /* Cortex-M33 Private Peripheral Bus */

/* Non-secure access address */

#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  define NRF91_NS(x)             (x - 0x10000000)
#else
#  define NRF91_NS(x)             (x)
#endif

/* Non-secure FICR */

#define NRF91_NONSECURE_RAM_FICR_OFFSET 0x1000
#define NRF91_NONSECURE_RAM_FICR        (NRF91_SRAM_BASE +                \
                                         CONFIG_NRF91_CPUAPP_MEM_RAM_SIZE \
                                         - NRF91_NONSECURE_RAM_FICR_OFFSET)

/* APB Peripherals */

#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  define NRF91_SPU_BASE        0x50003000
#endif
#define NRF91_REGULATORS_BASE   NRF91_NS(0x50004000)
#define NRF91_CLOCK_BASE        NRF91_NS(0x50005000)
#define NRF91_POWER_BASE        NRF91_NS(0x50005000)
#define NRF91_CTRLAPPERI_BASE   NRF91_NS(0x50006000)
#define NRF91_SPIM0_BASE        NRF91_NS(0x50008000)
#define NRF91_SPIS0_BASE        NRF91_NS(0x50008000)
#define NRF91_TWIM0_BASE        NRF91_NS(0x50008000)
#define NRF91_TWIS0_BASE        NRF91_NS(0x50008000)
#define NRF91_UART0_BASE        NRF91_NS(0x50008000)
#define NRF91_SPIM1_BASE        NRF91_NS(0x50009000)
#define NRF91_SPIS1_BASE        NRF91_NS(0x50009000)
#define NRF91_TWIM1_BASE        NRF91_NS(0x50009000)
#define NRF91_TWIS1_BASE        NRF91_NS(0x50009000)
#define NRF91_UART1_BASE        NRF91_NS(0x50009000)
#define NRF91_SPIM2_BASE        NRF91_NS(0x5000A000)
#define NRF91_SPIS2_BASE        NRF91_NS(0x5000A000)
#define NRF91_TWIM2_BASE        NRF91_NS(0x5000A000)
#define NRF91_TWIS2_BASE        NRF91_NS(0x5000A000)
#define NRF91_UART2_BASE        NRF91_NS(0x5000A000)
#define NRF91_SPIM3_BASE        NRF91_NS(0x5000C000)
#define NRF91_SPIS3_BASE        NRF91_NS(0x5000C000)
#define NRF91_TWIM3_BASE        NRF91_NS(0x5000C000)
#define NRF91_TWIS3_BASE        NRF91_NS(0x5000C000)
#define NRF91_UART3_BASE        NRF91_NS(0x5000C000)
#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  define NRF91_GPIOTE0_BASE    0x5000D000
#endif
#define NRF91_SAADC_BASE        NRF91_NS(0x5000E000)
#define NRF91_TIMER0_BASE       NRF91_NS(0x5000F000)
#define NRF91_TIMER1_BASE       NRF91_NS(0x50010000)
#define NRF91_TIMER2_BASE       NRF91_NS(0x50011000)
#define NRF91_RTC0_BASE         NRF91_NS(0x50014000)
#define NRF91_RTC1_BASE         NRF91_NS(0x50015000)
#define NRF91_DPPIC_BASE        NRF91_NS(0x50017000)
#define NRF91_WDT0_BASE         NRF91_NS(0x50018000)
#define NRF91_WDT1_BASE         NRF91_NS(0x50019000)
#define NRF91_EGU0_BASE         NRF91_NS(0x5001B000)
#define NRF91_EGU1_BASE         NRF91_NS(0x5001C000)
#define NRF91_EGU2_BASE         NRF91_NS(0x5001D000)
#define NRF91_EGU3_BASE         NRF91_NS(0x5001E000)
#define NRF91_EGU4_BASE         NRF91_NS(0x5001F000)
#define NRF91_EGU5_BASE         NRF91_NS(0x50020000)
#define NRF91_PWM0_BASE         NRF91_NS(0x50021000)
#define NRF91_PWM1_BASE         NRF91_NS(0x50022000)
#define NRF91_PWM2_BASE         NRF91_NS(0x50023000)
#define NRF91_PWM3_BASE         NRF91_NS(0x50024000)
#define NRF91_PDM0_BASE         NRF91_NS(0x50026000)
#define NRF91_I2S0_BASE         NRF91_NS(0x50028000)
#define NRF91_IPC_BASE          NRF91_NS(0x5002A000)
#define NRF91_FPU_BASE          NRF91_NS(0x5002C000)
#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  define NRF91_GPIOTE1_BASE    0x40031000
#endif
#define NRF91_KMU_BASE          NRF91_NS(0x50039000)
#define NRF91_NVMC_BASE         NRF91_NS(0x50039000)
#define NRF91_VMC_BASE          NRF91_NS(0x5003A000)
#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  define NRF91_CCHOSTRGF_BASE  0x50840000
#  define NRF91_CRYPTOCELL_BASE 0x50840000
#endif
#define NRF91_GPIO_P0_BASE      NRF91_NS(0x50842500)
#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  define NRF91_FICR_BASE       0x00FF0000
#elif CONFIG_NRF91_FICR_NS_WORKAROUND
/* Non-secure FICR RAM copy */

#  define NRF91_FICR_BASE       NRF91_NONSECURE_RAM_FICR
#endif
#define NRF91_UICR_BASE         0x00FF8000
#define NRF91_TAD_BASE          0xE0080000

/* Peripherals IDs */

#define NRF91_SPU_ID            3
#define NRF91_REGULATORS_ID     4
#define NRF91_POWER_CLOCK_ID    5
#define NRF91_CTRLAPPERI_ID     6
#define NRF91_SERIAL0_ID        8
#define NRF91_SERIAL1_ID        9
#define NRF91_SERIAL2_ID        10
#define NRF91_SERIAL3_ID        11
#define NRF91_GPIOTE0_ID        13
#define NRF91_SAADC_ID          14
#define NRF91_TIMER0_ID         15
#define NRF91_TIMER1_ID         16
#define NRF91_TIMER2_ID         17
#define NRF91_RTC0_ID           20
#define NRF91_RTC1_ID           21
#define NRF91_DPPIC_ID          23
#define NRF91_WDT0_ID           24
#define NRF91_EGU0_ID           27
#define NRF91_EGU1_ID           28
#define NRF91_EGU2_ID           29
#define NRF91_EGU3_ID           30
#define NRF91_EGU4_ID           31
#define NRF91_EGU5_ID           32
#define NRF91_PWM0_ID           33
#define NRF91_PWM1_ID           34
#define NRF91_PWM2_ID           35
#define NRF91_PWM3_ID           36
#define NRF91_PDM_ID            38
#define NRF91_I2S_ID            40
#define NRF91_IPC_ID            42
#define NRF91_GPIOTE1_ID        49
#define NRF91_KMU_ID            57
#define NRF91_NVMC_ID           57
#define NRF91_VMC_ID            58
#define NRF91_CRUPTOCELL_ID     64
#define NRF91_GPIO0_ID          66

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_MEMORYMAP_H */
