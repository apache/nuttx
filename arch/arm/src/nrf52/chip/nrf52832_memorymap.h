/****************************************************************************
 * arch/arm/src/nrf52/chip/nrf52832_memorymap.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52832_MEMORYMAP_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52832_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map */

#define NRF52_FLASH_BASE     0x00000000  /* Flash memory (512 KB) */
#define NRF52_SRAM_BASE      0x20000000  /* SRAM bank (64 KB) */
#define NRF52_FICR_BASE      0x10000000  /* FICR */
#define NRF52_UICR_BASE      0x10001000  /* UICR */
#define NRF52_APB0_BASE      0x40000000  /* APB  */

#define NRF52_CORTEXM4_BASE  0xe0000000  /* Cortex-M4 Private Peripheral Bus */

/* APB Peripherals */

#define NRF52_CLOCK_BASE    0x40000000
#define NRF52_POWER_BASE    0x40000000
#define NRF52_BPROT_BASE    0x40000000
#define NRF52_RADIO_BASE    0x40001000
#define NRF52_UARTE0_BASE   0x40002000
#define NRF52_UART0_BASE    0x40002000
#define NRF52_SPIM0_BASE    0x40003000
#define NRF52_SPIS0_BASE    0x40003000
#define NRF52_TWIM0_BASE    0x40003000
#define NRF52_TWI0_BASE     0x40003000
#define NRF52_SPI0_BASE     0x40003000
#define NRF52_TWIS0_BASE    0x40003000
#define NRF52_SPIM1_BASE    0x40004000
#define NRF52_TWI1_BASE     0x40004000
#define NRF52_SPIS1_BASE    0x40004000
#define NRF52_TWIS1_BASE    0x40004000
#define NRF52_TWIM1_BASE    0x40004000
#define NRF52_SPI1_BASE     0x40004000
#define NRF52_NFCT_BASE     0x40005000
#define NRF52_GPIOTE_BASE   0x40006000
#define NRF52_SAADC_BASE    0x40007000
#define NRF52_TIMER0_BASE   0x40008000
#define NRF52_TIMER1_BASE   0x40009000
#define NRF52_TIMER2_BASE   0x4000A000
#define NRF52_RTC0_BASE     0x4000B000
#define NRF52_TEMP_BASE     0x4000C000
#define NRF52_RNG_BASE      0x4000D000
#define NRF52_ECB_BASE      0x4000E000
#define NRF52_CCM_BASE      0x4000F000
#define NRF52_AAR_BASE      0x4000F000
#define NRF52_WDT_BASE      0x40010000
#define NRF52_RTC1_BASE     0x40011000
#define NRF52_QDEC_BASE     0x40012000
#define NRF52_LPCOMP_BASE   0x40013000
#define NRF52_COMP_BASE     0x40013000
#define NRF52_SWI0_BASE     0x40014000
#define NRF52_EGU0_BASE     0x40014000
#define NRF52_EGU1_BASE     0x40015000
#define NRF52_SWI1_BASE     0x40015000
#define NRF52_SWI2_BASE     0x40016000
#define NRF52_EGU2_BASE     0x40016000
#define NRF52_SWI3_BASE     0x40017000
#define NRF52_EGU3_BASE     0x40017000
#define NRF52_EGU4_BASE     0x40018000
#define NRF52_SWI4_BASE     0x40018000
#define NRF52_SWI5_BASE     0x40019000
#define NRF52_EGU5_BASE     0x40019000
#define NRF52_TIMER3_BASE   0x4001A000
#define NRF52_TIMER4_BASE   0x4001B000
#define NRF52_PWM0_BASE     0x4001C000
#define NRF52_PDM_BASE      0x4001D000
#define NRF52_NVMC_BASE     0x4001E000
#define NRF52_PPI_BASE      0x4001F000
#define NRF52_MWU_BASE      0x40020000
#define NRF52_PWM1_BASE     0x40021000
#define NRF52_PWM2_BASE     0x40022000
#define NRF52_SPI2_BASE     0x40023000
#define NRF52_SPIS2_BASE    0x40023000
#define NRF52_SPIM2_BASE    0x40023000
#define NRF52_RTC2_BASE     0x40024000
#define NRF52_I2S_BASE      0x40025000
#define NRF52_FPU_BASE      0x40026000

#define NRF52_GPIO_P0_BASE  0x50000000

#endif /* __ARCH_ARM_SRC_NRF52_CHIP_NRF52832_MEMORYMAP_H */
