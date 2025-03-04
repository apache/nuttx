/****************************************************************************
 * arch/risc-v/src/hpm6000/hardware/hpm6300/hpm6300_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM6300_HPM6300_MEMORYMAP_H
#define __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM6300_HPM6300_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define HPM_PLIC_BASE        0xE4000000
#define HPM_MCHTMR_BASE      0xE6000000
#define HPM_GPIO0_BASE       0xF0000000

#define HPM_GPIOM_BASE       0xF0008000
#define HPM_ADC0_BASE        0xF0010000
#define HPM_ADC1_BASE        0xF0014000
#define HPM_ADC2_BASE        0xF0018000

#define HPM_ACMP_BASE        0xF0020000
#define HPM_DAC_BASE         0Xf0024000
#define HPM_SPI0_BASE        0xF0030000
#define HPM_SPI1_BASE        0xF0034000
#define HPM_SPI2_BASE        0xF0038000
#define HPM_SPI3_BASE        0xF003c000
#define HPM_UART0_BASE       0xF0040000
#define HPM_UART1_BASE       0xF0044000
#define HPM_UART2_BASE       0xF0048000
#define HPM_UART3_BASE       0xF004C000
#define HPM_UART4_BASE       0xF0050000
#define HPM_UART5_BASE       0xF0054000
#define HPM_UART6_BASE       0xF0058000
#define HPM_UART7_BASE       0xF005C000

#define HPM_CAN0_BASE        0xF0080000
#define HPM_CAN1_BASE        0xF0084000

#define HPM_WDG0_BASE        0xF0090000
#define HPM_WDG1_BASE        0xF0094000

#define HPM_MBX0A_BASE       0xF00A0000
#define HPM_MBX0B_BASE       0xF00A4000

#define HPM_PTPC_BASE        0xF00B0000
#define HPM_DMAMUX_BASE      0xF00C0000
#define HPM_HDMA_BASE        0xF00C4000
#define HPM_RNG_BASE         0xF00C8000
#define HPM_KEYM_BASE        0xF00CC000
#define HPM_I2S0_BASE        0xF0100000
#define HPM_I2S1_BASE        0xF0104000

#define HPM_DAO_BASE         0xF0110000
#define HPM_PDM_BASE         0xF0114000
#define HPM_PWM0_BASE        0xF0200000
#define HPM_HALL0_BASE       0xF0204000
#define HPM_QEI0_BASE        0xF0208000
#define HPM_TRGM0_BASE       0xF020C000
#define HPM_PWM1_BASE        0xF0210000
#define HPM_HALL1_BASE       0xF0214000
#define HPM_QEI1_BASE        0xF0218000
#define HPM_TRGM1_BASE       0xF021C000

#define HPM_SYNT_BASE        0xF0240000

#define HPM_ENET0_BASE       0xF2000000

#define HPM_NTMR0_BASE       0xF2010000

#define HPM_USB0_BASE        0xF2020000

#define HPM_SDXC0_BASE       0xF2030000

#define HPM_CONCTL_BASE      0xF2040000
#define HPM_GPTMR0_BASE      0xF3000000
#define HPM_GPTMR1_BASE      0xF3004000
#define HPM_GPTMR2_BASE      0xF3008000
#define HPM_GPTMR3_BASE      0xF300C000

#define HPM_I2C0_BASE        0xF3020000
#define HPM_I2C1_BASE        0xF3020000
#define HPM_I2C2_BASE        0xF3020000
#define HPM_I2C3_BASE        0xF3020000
#define HPM_XPI0_BASE        0xF3040000
#define HPM_XPI1_BASE        0xF3044000
#define HPM_XDMA_BASE        0xF3048000
#define HPM_SDP_BASE         0xF304C000
#define HPM_FEMC_BASE        0xF3050000
#define HPM_ROMC_BASE        0xF3054000
#define HPM_FFA_BASE         0xF3058000
#define HPM_SYSCTL_BASE      0xF4000000
#define HPM_IOC_BASE         0xF4040000
#define HPM_OTPSHW_BASE      0xF4080000
#define HPM_PPOR_BASE        0xF40C0000
#define HPM_PCFG_BASE        0xF40C4000
#define HPM_OTP_BASE         0xF40C8000
#define HPM_PSEC_BASE        0xF40CC000
#define HPM_PMON_BASE        0xF40D0000
#define HPM_PGPR_BASE        0xF40D4000
#define HPM_PIOC_BASE        0xF40D8000
#define HPM_PGPIO_BASE       0xF40DC000
#define HPM_PTMR_BASE        0xF40E0000
#define HPM_PUART_BASE       0xF40E4000
#define HPM_PWDG_BASE        0xF40E8000

#define HPM_PLLCTL_BASE      0xF4100000
#define HPM_TSNS_BASE        0xF4104000
#define HPM_BACC_BASE        0xF5000000
#define HPM_BPOR_BASE        0xF5004000
#define HPM_BCFG_BASE        0xF5008000
#define HPM_BUTN_BASE        0xF500C000
#define HPM_BIOC_BASE        0xF5010000
#define HPM_BGPIO_BASE       0xF5014000
#define HPM_BGPR_BASE        0xF5018000
#define HPM_RTCSHW_BASE      0xF501C000
#define HPM_BSEC_BASE        0xF5040000
#define HPM_RTC_BASE         0xF5044000
#define HPM_BKEY_BASE        0xF5048000
#define HPM_BMON_BASE        0xF504C000
#define HPM_TAMP_BASE        0xF5050000
#define HPM_MONO_BASE        0xF5054000

#endif /* __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM6300_HPM6300_MEMORYMAP_H */
