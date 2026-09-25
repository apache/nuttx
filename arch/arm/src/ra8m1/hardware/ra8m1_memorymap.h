/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_MEMORYMAP_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers Base Addresses */

#define R_ACMPHS0_BASE                     0x40236000UL
#define R_ACMPHS1_BASE                     0x40236100UL
#define R_ACMPHS2_BASE                     0x40236200UL
#define R_ACMPHS3_BASE                     0x40236300UL
#define R_ACMPHS4_BASE                     0x40236400UL
#define R_ACMPHS5_BASE                     0x40236500UL
#define R_ADC0_BASE                        0x40332000UL
#define R_ADC1_BASE                        0x40332200UL
#define R_AGTX0_BASE                       0x40221000UL
#define R_AGTX1_BASE                       0x40221100UL
#define R_AGTX2_BASE                       0x40221200UL
#define R_AGTX3_BASE                       0x40221300UL
#define R_AGTX4_BASE                       0x40221400UL
#define R_AGTX5_BASE                       0x40221500UL
#define R_AGTX6_BASE                       0x40221600UL
#define R_AGTX7_BASE                       0x40221700UL
#define R_AGTX8_BASE                       0x40221800UL
#define R_AGTX9_BASE                       0x40221900UL
#define R_BUS_BASE                         0x40003000UL
#define R_CAC_BASE                         0x40202400UL
#define R_CANFD1_BASE                      0x40382000UL
#define R_CANFD_BASE                       0x40380000UL
#define R_CEU_BASE                         0x40348000UL
#define R_CPSCU_BASE                       0x40008000UL
#define R_CRC_BASE                         0x40310000UL
#define R_DAC_BASE                         0x40333000UL
#define R_DEBUG_BASE                       0x4001B000UL
#define R_DEBUG_OCD_BASE                   0x40011000UL
#define R_DMAC0_BASE                       0x4000A000UL
#define R_DMAC1_BASE                       0x4000A040UL
#define R_DMAC2_BASE                       0x4000A080UL
#define R_DMAC3_BASE                       0x4000A0C0UL
#define R_DMAC4_BASE                       0x4000A100UL
#define R_DMAC5_BASE                       0x4000A140UL
#define R_DMAC6_BASE                       0x4000A180UL
#define R_DMAC7_BASE                       0x4000A1C0UL
#define R_DMA_BASE                         0x4000A800UL
#define R_DOC_BASE                         0x40311000UL
#define R_DOC_B_BASE                       0x40311000UL
#define R_DOTF1_BASE                       0x40268900UL
#define R_DOTF_BASE                        0x40268800UL
#define R_DTC_BASE                         0x4000AC00UL
#define R_ECCMB0_BASE                      0x4036F200UL
#define R_ECCMB1_BASE                      0x4036F300UL
#define R_ELC_BASE                         0x40201000UL
#define R_ETHERC0_BASE                     0x40354100UL
#define R_ETHERC_EDMAC_BASE                0x40354000UL
#define R_FACI_HP_BASE                     0x4011E000UL
#define R_FACI_HP_CMD_BASE                 0x40100000UL
#define R_FCACHE_BASE                      0x4001C100UL
#define R_FLAD_BASE                        0x4011C000UL
#define R_GPT0_BASE                        0x40322000UL
#define R_GPT10_BASE                       0x40322A00UL
#define R_GPT11_BASE                       0x40322B00UL
#define R_GPT12_BASE                       0x40322C00UL
#define R_GPT13_BASE                       0x40322D00UL
#define R_GPT1_BASE                        0x40322100UL
#define R_GPT2_BASE                        0x40322200UL
#define R_GPT3_BASE                        0x40322300UL
#define R_GPT4_BASE                        0x40322400UL
#define R_GPT5_BASE                        0x40322500UL
#define R_GPT6_BASE                        0x40322600UL
#define R_GPT7_BASE                        0x40322700UL
#define R_GPT8_BASE                        0x40322800UL
#define R_GPT9_BASE                        0x40322900UL
#define R_GPT_OPS_BASE                     0x40323F00UL
#define R_GPT_POEG0_BASE                   0x40212000UL
#define R_GPT_POEG1_BASE                   0x40212100UL
#define R_GPT_POEG2_BASE                   0x40212200UL
#define R_GPT_POEG3_BASE                   0x40212300UL
#define R_I3C0_BASE                        0x4035F000UL
#define R_I3C1_BASE                        0x4035F100UL
#define R_ICU_BASE                         0x40006000UL
#define R_IIC0_BASE                        0x4025E000UL
#define R_IIC1_BASE                        0x4025E100UL
#define R_IIC2_BASE                        0x4025E200UL
#define R_IWDT_BASE                        0x40202200UL
#define R_MPU_MMPU_BASE                    0x40000000UL
#define R_MPU_SPMON_BASE                   0x40000D00UL
#define R_MSTP_BASE                        0x40203000UL
#define R_OFS_DATAFLASH_BASE               0x27030000UL
#define R_PFS_BASE                         0x40400800UL
#define R_PMISC_BASE                       0x40400D00UL
#define R_PORT0_BASE                       0x40400000UL
#define R_PORT10_BASE                      0x40400140UL
#define R_PORT11_BASE                      0x40400160UL
#define R_PORT12_BASE                      0x40400180UL
#define R_PORT13_BASE                      0x404001A0UL
#define R_PORT14_BASE                      0x404001C0UL
#define R_PORT1_BASE                       0x40400020UL
#define R_PORT2_BASE                       0x40400040UL
#define R_PORT3_BASE                       0x40400060UL
#define R_PORT4_BASE                       0x40400080UL
#define R_PORT5_BASE                       0x404000A0UL
#define R_PORT6_BASE                       0x404000C0UL
#define R_PORT7_BASE                       0x404000E0UL
#define R_PORT8_BASE                       0x40400100UL
#define R_PORT9_BASE                       0x40400120UL
#define R_PSCU_BASE                        0x40204000UL
#define R_RTC_BASE                         0x40202000UL
#define R_SCI0_BASE                        0x40358000UL
#define R_SCI1_BASE                        0x40358100UL
#define R_SCI2_BASE                        0x40358200UL
#define R_SCI3_BASE                        0x40358300UL
#define R_SCI4_BASE                        0x40358400UL
#define R_SCI5_BASE                        0x40358500UL
#define R_SCI6_BASE                        0x40358600UL
#define R_SCI7_BASE                        0x40358700UL
#define R_SCI8_BASE                        0x40358800UL
#define R_SCI9_BASE                        0x40358900UL
#define R_SCI_B0_BASE                      0x40358000UL
#define R_SCI_B1_BASE                      0x40358100UL
#define R_SCI_B2_BASE                      0x40358200UL
#define R_SCI_B3_BASE                      0x40358300UL
#define R_SCI_B4_BASE                      0x40358400UL
#define R_SCI_B5_BASE                      0x40358500UL
#define R_SCI_B6_BASE                      0x40358600UL
#define R_SCI_B7_BASE                      0x40358700UL
#define R_SCI_B8_BASE                      0x40358800UL
#define R_SCI_B9_BASE                      0x40358900UL
#define R_SDHI0_BASE                       0x40252000UL
#define R_SDHI1_BASE                       0x40252400UL
#define R_SPI0_BASE                        0x4035C000UL
#define R_SPI1_BASE                        0x4035C100UL
#define R_SPI2_BASE                        0x40072200UL
#define R_SPI_B0_BASE                      0x4035C000UL
#define R_SPI_B1_BASE                      0x4035C100UL
#define R_SRAM_BASE                        0x40002000UL
#define R_SSI0_BASE                        0x4025D000UL
#define R_SSI1_BASE                        0x4025D100UL
#define R_SYSTEM_BASE                      0x4001E000UL
#define R_TSN_CAL_BASE                     0x4011B17CUL
#define R_TSN_CTRL_BASE                    0x40235000UL
#define R_ULPT0_BASE                       0x40220000UL
#define R_ULPT1_BASE                       0x40220100UL
#define R_USB_FS0_BASE                     0x40250000UL
#define R_USB_HS0_BASE                     0x40351000UL
#define R_WDT1_BASE                        0x40044300UL
#define R_WDT_BASE                         0x40202600UL
#define R_XSPI0_BASE                       0x40268000UL
#define R_XSPI1_BASE                       0x40268400UL

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

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_MEMORYMAP_H */
