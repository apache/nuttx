/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_memorymap_cpunet.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_MEMORYMAP_CPUNET_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_MEMORYMAP_CPUNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map */

#define NRF53_FLASH_BASE      0x01000000 /* Flash memory Start Address */
#define NRF53_SRAM_BASE       0x21000000 /* SRAM Start Address */

#define NRF53_CORTEXM33_BASE  0xe0000000 /* Cortex-M33 Private Peripheral Bus */

/* APB Peripherals */

#define NRF53_DCNF_BASE       0x41000000
#define NRF53_VREQCTRL_BASE   0x41004000
#define NRF53_CLOCK_BASE      0x41005000
#define NRF53_POWER_BASE      0x41005000
#define NRF53_RESET_BASE      0x41005000
#define NRF53_CTRLAPPERI_BASE 0x41006000
#define NRF53_RADIO_BASE      0x41008000
#define NRF53_RNG_BASE        0x41009000
#define NRF53_GPIOTE0_BASE    0x4100A000
#define NRF53_WDT_BASE        0x4100B000
#define NRF53_TIMER0_BASE     0x4100C000
#define NRF53_ECB_BASE        0x4100D000
#define NRF53_AAR_BASE        0x4100E000
#define NRF53_CCM_BASE        0x4100E000
#define NRF53_DPPIC_BASE      0x4100F000
#define NRF53_TEMP_BASE       0x41010000
#define NRF53_RTC_BASE        0x41011000
#define NRF53_IPC_BASE        0x41012000
#define NRF53_SPIM0_BASE      0x41013000
#define NRF53_SPIS0_BASE      0x41013000
#define NRF53_TWIM0_BASE      0x41013000
#define NRF53_TWIS0_BASE      0x41013000
#define NRF53_UART0_BASE      0x41013000
#define NRF53_EGU0_BASE       0x41014000
#define NRF53_RTC1_BASE       0x41016000
#define NRF53_TIMER1_BASE     0x41018000
#define NRF53_TIMER2_BASE     0x41019000
#define NRF53_SWI0_BASE       0x4101A000
#define NRF53_SWI1_BASE       0x4101B000
#define NRF53_SWI2_BASE       0x4101C000
#define NRF53_SWI3_BASE       0x4101D000
#define NRF53_ACL_BASE        0x41080000
#define NRF53_NVMC_BASE       0x41080000
#define NRF53_VMC_BASE        0x41081000
#define NRF53_GPIO_P0_BASE    0x418C0500
#define NRF53_GPIO_P1_BASE    0x418C0800
#define NRF53_FICR_BASE       0x01FF0000
#define NRF53_UICR_BASE       0x01FF8000
#define NRF53_CTI_BASE        0xE0042000

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_MEMORYMAP_CPUNET_H */
