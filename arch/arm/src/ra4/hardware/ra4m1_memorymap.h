/****************************************************************************
 * arch/arm/src/ra4/hardware/ra4m1_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RA_HARDWARE_RA4M1_MEMORYMAP_H
#define __ARCH_ARM_SRC_RA_HARDWARE_RA4M1_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers Base Addresses */

 #define R_ACMPLP_BASE       0x40085E00UL
 #define R_ADC0_BASE         0x4005C000UL
 #define R_ADC1_BASE         0x4005C200UL
 #define R_BUS_BASE          0x40003000UL
 #define R_CAC_BASE          0x40044600UL
 #define R_CAN0_BASE         0x40050000UL
 #define R_CAN1_BASE         0x40051000UL
 #define R_CRC_BASE          0x40074000UL
 #define R_CTSU_BASE         0x40081000UL
 #define R_DAC_BASE          0x4005E000UL
 #define R_DAC8_BASE         0x4009E000UL
 #define R_DEBUG_BASE        0x4001B000UL
 #define R_DMA_BASE          0x40005200UL
 #define R_DMAC0_BASE        0x40005000UL
 #define R_DMAC1_BASE        0x40005040UL
 #define R_DMAC2_BASE        0x40005080UL
 #define R_DMAC3_BASE        0x400050C0UL
 #define R_DMAC4_BASE        0x40005100UL
 #define R_DMAC5_BASE        0x40005140UL
 #define R_DMAC6_BASE        0x40005180UL
 #define R_DMAC7_BASE        0x400051C0UL
 #define R_DOC_BASE          0x40054100UL
 #define R_DTC_BASE          0x40005400UL
 #define R_ELC_BASE          0x40041000UL
 #define R_FACI_LP_BASE      0x407EC000UL
 #define R_FCACHE_BASE       0x4001C000UL
 #define R_GPT0_BASE         0x40078000UL
 #define R_GPT1_BASE         0x40078100UL
 #define R_GPT2_BASE         0x40078200UL
 #define R_GPT3_BASE         0x40078300UL
 #define R_GPT4_BASE         0x40078400UL
 #define R_GPT5_BASE         0x40078500UL
 #define R_GPT6_BASE         0x40078600UL
 #define R_GPT7_BASE         0x40078700UL
 #define R_GPT8_BASE         0x40078800UL
 #define R_GPT9_BASE         0x40078900UL
 #define R_GPT10_BASE        0x40078A00UL
 #define R_GPT11_BASE        0x40078B00UL
 #define R_GPT12_BASE        0x40078C00UL
 #define R_GPT13_BASE        0x40078D00UL
 #define R_GPT_OPS_BASE      0x40078FF0UL
 #define R_GPT_POEG0_BASE    0x40042000UL
 #define R_GPT_POEG1_BASE    0x40042100UL
 #define R_GPT_POEG2_BASE    0x40042200UL
 #define R_GPT_POEG3_BASE    0x40042300UL
 #define R_ICU_BASE          0x40006000UL
 #define R_IIC0_BASE         0x40053000UL
 #define R_IIC1_BASE         0x40053100UL
 #define R_IIC2_BASE         0x40053200UL
 #define R_IWDT_BASE         0x40044400UL
 #define R_KINT_BASE         0x40080000UL
 #define R_MPU_MMPU_BASE     0x40000000UL
 #define R_MPU_SMPU_BASE     0x40000C00UL
 #define R_MPU_SPMON_BASE    0x40000D00UL
 #define R_MSTP_BASE         0x40047000UL
 #define R_OPAMP_BASE        0x40086000UL
 #define R_PORT0_BASE        0x40040000UL
 #define R_PORT1_BASE        0x40040020UL
 #define R_PORT2_BASE        0x40040040UL
 #define R_PORT3_BASE        0x40040060UL
 #define R_PORT4_BASE        0x40040080UL
 #define R_PORT5_BASE        0x400400A0UL
 #define R_PORT6_BASE        0x400400C0UL
 #define R_PORT7_BASE        0x400400E0UL
 #define R_PORT8_BASE        0x40040100UL
 #define R_PORT9_BASE        0x40040120UL
 #define R_PORT10_BASE       0x40040140UL
 #define R_PORT11_BASE       0x40040160UL
 #define R_PORT12_BASE       0x40040180UL
 #define R_PORT13_BASE       0x400401A0UL
 #define R_PORT14_BASE       0x400401C0UL
 #define R_PFS_BASE          0x40040800UL
 #define R_PMISC_BASE        0x40040D00UL
 #define R_RTC_BASE          0x40044000UL
 #define R_SCI0_BASE         0x40070000UL
 #define R_SCI1_BASE         0x40070020UL
 #define R_SCI2_BASE         0x40070040UL
 #define R_SCI3_BASE         0x40070060UL
 #define R_SCI4_BASE         0x40070080UL
 #define R_SCI5_BASE         0x400700A0UL
 #define R_SCI6_BASE         0x400700C0UL
 #define R_SCI7_BASE         0x400700E0UL
 #define R_SCI8_BASE         0x40070100UL
 #define R_SCI9_BASE         0x40070120UL
 #define R_SLCDC_BASE        0x40082000UL
 #define R_SPI0_BASE         0x40072000UL
 #define R_SPI1_BASE         0x40072100UL
 #define R_SRAM_BASE         0x40002000UL
 #define R_SSI0_BASE         0x4004E000UL
 #define R_SSI1_BASE         0x4004E100UL
 #define R_SYSTEM_BASE       0x4001E000UL
 #define R_TSN_BASE          0x407EC000UL
 #define R_USB_FS0_BASE      0x40090000UL
 #define R_WDT_BASE          0x40044200UL
 #define R_AGTX0_BASE        0x40084000UL
 #define R_AGTX1_BASE        0x40084100UL
 #define R_AGTX2_BASE        0x40084200UL
 #define R_AGTX3_BASE        0x40084300UL
 #define R_AGTX4_BASE        0x40084400UL
 #define R_AGTX5_BASE        0x40084500UL
 #define R_AGTX6_BASE        0x40084600UL
 #define R_AGTX7_BASE        0x40084700UL
 #define R_AGTX8_BASE        0x40084800UL
 #define R_AGTX9_BASE        0x40084900UL
 #define R_OFS_BASE          0x00000400UL

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_RA_HARDWARE_RA4M1_MEMORYMAP_H */
