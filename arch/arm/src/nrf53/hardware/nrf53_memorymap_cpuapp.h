/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_memorymap_cpuapp.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_MEMORYMAP_CPUAPP_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_MEMORYMAP_CPUAPP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map */

#define NRF53_FLASH_BASE        0x00000000 /* Flash memory Start Address */
#define NRF53_SRAM_BASE         0x20000000 /* SRAM Start Address */

#define NRF53_CORTEXM33_BASE    0xe0000000 /* Cortex-M33 Private Peripheral Bus */

/* APB Peripherals */

#define NRF53_DCNF_BASE         0x50000000
#define NRF53_FPU_BASE          0x50000000
#define NRF53_CACHE_BASE        0x50001000
#define NRF53_SPU_BASE          0x50003000
#define NRF53_OSCILLATORS_BASE  0x50004000
#define NRF53_REGULATORS_BASE   0x50004000
#define NRF53_CLOCK_BASE        0x50005000
#define NRF53_POWER_BASE        0x50005000
#define NRF53_RESET_BASE        0x50005000
#define NRF53_CTRLAPPERI_BASE   0x50006000
#define NRF53_SPIM0_BASE        0x50008000
#define NRF53_SPIS0_BASE        0x50008000
#define NRF53_TWIM0_BASE        0x50008000
#define NRF53_TWIS0_BASE        0x50008000
#define NRF53_UART0_BASE        0x50008000
#define NRF53_SPIM1_BASE        0x50009000
#define NRF53_SPIS1_BASE        0x50009000
#define NRF53_TWIM1_BASE        0x50009000
#define NRF53_TWIS1_BASE        0x50009000
#define NRF53_UART1_BASE        0x50009000
#define NRF53_SPIM2_BASE        0x5000A000
#define NRF53_GPIOTE0_BASE      0x5000D000
#define NRF53_SAADC_BASE        0x5000E000
#define NRF53_TIMER0_BASE       0x5000F000
#define NRF53_TIMER1_BASE       0x50010000
#define NRF53_TIMER2_BASE       0x50011000
#define NRF53_RTC0_BASE         0x50014000
#define NRF53_RTC1_BASE         0x50015000
#define NRF53_DPPIC_BASE        0x50017000
#define NRF53_WDT0_BASE         0x50018000
#define NRF53_WDT1_BASE         0x50019000
#define NRF53_COMP_BASE         0x5001A000
#define NRF53_LPCOMP_BASE       0x5001A000
#define NRF53_EGU0_BASE         0x5001B000
#define NRF53_EGU1_BASE         0x5001C000
#define NRF53_EGU2_BASE         0x5001D000
#define NRF53_EGU3_BASE         0x5001E000
#define NRF53_EGU4_BASE         0x5001F000
#define NRF53_EGU5_BASE         0x50020000
#define NRF53_PWM0_BASE         0x50021000
#define NRF53_PWM1_BASE         0x50022000
#define NRF53_PWM2_BASE         0x50023000
#define NRF53_PDM0_BASE         0x50026000
#define NRF53_I2S0_BASE         0x50028000
#define NRF53_IPC_BASE          0x5002A000
#define NRF53_QSPI_BASE         0x5002B000
#define NRF53_NFCT_BASE         0x5002D000
#define NRF53_GPIOTE1_BASE      0x4002F000
#define NRF53_MUTEX_BASE        0x50030000
#define NRF53_QDEC_BASE         0x50033000
#define NRF53_USBD_BASE         0x50036000
#define NRF53_USBREGULATOR_BASE 0x50037000
#define NRF53_KMU_BASE          0x50039000
#define NRF53_NVMC_BASE         0x50039000
#define NRF53_GPIO_P0_BASE      0x50842500
#define NRF53_GPIO_P1_BASE      0x50842800
#define NRF53_CRYPTOCELL_BASE   0x50844000
#define NRF53_VMC_BASE          0x50081000
#define NRF53_CACHEDATA_BASE    0x00F00000
#define NRF53_CACHEINFO_BASE    0x00F00000
#define NRF53_FICR_BASE         0x00FF0000
#define NRF53_UICR_BASE         0x00FF8000
#define NRF53_CTI_BASE          0xE0042000
#define NRF53_TAD_BASE          0xE0080000

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_MEMORYMAP_CPUAPP_H */
