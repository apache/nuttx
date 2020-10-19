/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_memorymap.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3) Neither the name NuttX nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_C13X0_MEMORYMAP_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_C13X0_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 *
 * The following are defines for the base address of the memories and
 * peripherals on the CPU_MMAP interface
 *
 ****************************************************************************/

#define TIVA_FLASHMEM_BASE            0x00000000 /* FLASHMEM */
#define TIVA_BROM_BASE                0x10000000 /* BROM */
#define TIVA_GPRAM_BASE               0x11000000 /* GPRAM */
#define TIVA_SRAM_BASE                0x20000000 /* SRAM */
#define TIVA_RFC_RAM_BASE             0x21000000 /* RFC_RAM */
#define TIVA_SSI0_BASE                0x40000000 /* SSI */
#define TIVA_UART0_BASE               0x40001000 /* UART */
#define TIVA_I2C0_BASE                0x40002000 /* I2C */
#define TIVA_SSI1_BASE                0x40008000 /* SSI */
#define TIVA_GPT0_BASE                0x40010000 /* GPT */
#define TIVA_GPT1_BASE                0x40011000 /* GPT */
#define TIVA_GPT2_BASE                0x40012000 /* GPT */
#define TIVA_GPT3_BASE                0x40013000 /* GPT */
#define TIVA_UDMA0_BASE               0x40020000 /* UDMA */
#define TIVA_I2S0_BASE                0x40021000 /* I2S */
#define TIVA_GPIO_BASE                0x40022000 /* GPIO */
#define TIVA_CRYPTO_BASE              0x40024000 /* CRYPTO */
#define TIVA_TRNG_BASE                0x40028000 /* TRNG */
#define TIVA_FLASH_BASE               0x40030000 /* FLASH */
#define TIVA_VIMS_BASE                0x40034000 /* VIMS */
#define TIVA_RFC_PWR_BASE             0x40040000 /* RFC_PWR */
#define TIVA_RFC_DBELL_BASE           0x40041000 /* RFC_DBELL */
#define TIVA_RFC_RAT_BASE             0x40043000 /* RFC_RAT */
#define TIVA_RFC_FSCA_BASE            0x40044000 /* RFC_FSCA */
#define TIVA_WDT_BASE                 0x40080000 /* WDT */
#define TIVA_IOC_BASE                 0x40081000 /* IOC */
#define TIVA_PRCM_BASE                0x40082000 /* PRCM */
#define TIVA_EVENT_BASE               0x40083000 /* EVENT */
#define TIVA_SMPH_BASE                0x40084000 /* SMPH */
#define TIVA_ADI2_BASE                0x40086000 /* ADI */
#define TIVA_ADI3_BASE                0x40086200 /* ADI */
#define TIVA_AON_SYSCTL_BASE          0x40090000 /* AON_SYSCTL */
#define TIVA_AON_WUC_BASE             0x40091000 /* AON_WUC */
#define TIVA_AON_RTC_BASE             0x40092000 /* AON_RTC */
#define TIVA_AON_EVENT_BASE           0x40093000 /* AON_EVENT */
#define TIVA_AON_IOC_BASE             0x40094000 /* AON_IOC */
#define TIVA_AON_BATMON_BASE          0x40095000 /* AON_BATMON */
#define TIVA_AUX_AIODIO0_BASE         0x400c1000 /* AUX_AIODIO */
#define TIVA_AUX_AIODIO1_BASE         0x400c2000 /* AUX_AIODIO */
#define TIVA_AUX_TDC_BASE             0x400c4000 /* AUX_TDC */
#define TIVA_AUX_EVCTL_BASE           0x400c5000 /* AUX_EVCTL */
#define TIVA_AUX_WUC_BASE             0x400c6000 /* AUX_WUC */
#define TIVA_AUX_TIMER_BASE           0x400c7000 /* AUX_TIMER */
#define TIVA_AUX_SMPH_BASE            0x400c8000 /* AUX_SMPH */
#define TIVA_AUX_ANAIF_BASE           0x400c9000 /* AUX_ANAIF */
#define TIVA_AUX_DDI0_OSC_BASE        0x400ca000 /* DDI */
#define TIVA_AUX_ADI4_BASE            0x400cb000 /* ADI */
#define TIVA_AUX_RAM_BASE             0x400e0000 /* AUX_RAM */
#define TIVA_AUX_SCE_BASE             0x400e1000 /* AUX_SCE */
#define TIVA_FLASH_CFG_BASE           0x50000000 /* CC26_DUMMY_COMP */
#define TIVA_FCFG1_BASE               0x50001000 /* FCFG1 */
#define TIVA_FCFG2_BASE               0x50002000 /* FCFG2 */
#define TIVA_CCFG_BASE                0x50003000 /* CCFG */
#define TIVA_SSI0_NONBUF_BASE         0x60000000 /* SSI CPU nonbuf base */
#define TIVA_UART0_NONBUF_BASE        0x60001000 /* UART CPU nonbuf base */
#define TIVA_I2C0_NONBUF_BASE         0x60002000 /* I2C CPU nonbuf base */
#define TIVA_SSI1_NONBUF_BASE         0x60008000 /* SSI CPU nonbuf base */
#define TIVA_GPT0_NONBUF_BASE         0x60010000 /* GPT CPU nonbuf base */
#define TIVA_GPT1_NONBUF_BASE         0x60011000 /* GPT CPU nonbuf base */
#define TIVA_GPT2_NONBUF_BASE         0x60012000 /* GPT CPU nonbuf base */
#define TIVA_GPT3_NONBUF_BASE         0x60013000 /* GPT CPU nonbuf base */
#define TIVA_UDMA0_NONBUF_BASE        0x60020000 /* UDMA CPU nonbuf base */
#define TIVA_I2S0_NONBUF_BASE         0x60021000 /* I2S CPU nonbuf base */
#define TIVA_GPIO_NONBUF_BASE         0x60022000 /* GPIO CPU nonbuf base */
#define TIVA_CRYPTO_NONBUF_BASE       0x60024000 /* CRYPTO CPU nonbuf base */
#define TIVA_TRNG_NONBUF_BASE         0x60028000 /* TRNG CPU nonbuf base */
#define TIVA_FLASH_NONBUF_BASE        0x60030000 /* FLASH CPU nonbuf base */
#define TIVA_VIMS_NONBUF_BASE         0x60034000 /* VIMS CPU nonbuf base */
#define TIVA_RFC_PWR_NONBUF_BASE      0x60040000 /* RFC_PWR CPU nonbuf base */
#define TIVA_RFC_DBELL_NONBUF_BASE    0x60041000 /* RFC_DBELL CPU nonbuf base */
#define TIVA_RFC_RAT_NONBUF_BASE      0x60043000 /* RFC_RAT CPU nonbuf base */
#define TIVA_RFC_FSCA_NONBUF_BASE     0x60044000 /* RFC_FSCA CPU nonbuf base */
#define TIVA_WDT_NONBUF_BASE          0x60080000 /* WDT CPU nonbuf base */
#define TIVA_IOC_NONBUF_BASE          0x60081000 /* IOC CPU nonbuf base */
#define TIVA_PRCM_NONBUF_BASE         0x60082000 /* PRCM CPU nonbuf base */
#define TIVA_EVENT_NONBUF_BASE        0x60083000 /* EVENT CPU nonbuf base */
#define TIVA_SMPH_NONBUF_BASE         0x60084000 /* SMPH CPU nonbuf base */
#define TIVA_ADI2_NONBUF_BASE         0x60086000 /* ADI CPU nonbuf base */
#define TIVA_ADI3_NONBUF_BASE         0x60086200 /* ADI CPU nonbuf base */
#define TIVA_AON_SYSCTL_NONBUF_BASE   0x60090000 /* AON_SYSCTL CPU nonbuf base */
#define TIVA_AON_WUC_NONBUF_BASE      0x60091000 /* AON_WUC CPU nonbuf base */
#define TIVA_AON_RTC_NONBUF_BASE      0x60092000 /* AON_RTC CPU nonbuf base */
#define TIVA_AON_EVENT_NONBUF_BASE    0x60093000 /* AON_EVENT CPU nonbuf base */
#define TIVA_AON_IOC_NONBUF_BASE      0x60094000 /* AON_IOC CPU nonbuf base */
#define TIVA_AON_BATMON_NONBUF_BASE   0x60095000 /* AON_BATMON CPU nonbuf base */
#define TIVA_AUX_AIODIO0_NONBUF_BASE  0x600c1000 /* AUX_AIODIO CPU nonbuf base */
#define TIVA_AUX_AIODIO1_NONBUF_BASE  0x600c2000 /* AUX_AIODIO CPU nonbuf base */
#define TIVA_AUX_TDC_NONBUF_BASE      0x600c4000 /* AUX_TDC CPU nonbuf base */
#define TIVA_AUX_EVCTL_NONBUF_BASE    0x600c5000 /* AUX_EVCTL CPU nonbuf base */
#define TIVA_AUX_WUC_NONBUF_BASE      0x600c6000 /* AUX_WUC CPU nonbuf base */
#define TIVA_AUX_TIMER_NONBUF_BASE    0x600c7000 /* AUX_TIMER CPU nonbuf base */
#define TIVA_AUX_SMPH_NONBUF_BASE     0x600c8000 /* AUX_SMPH CPU nonbuf base */
#define TIVA_AUX_ANAIF_NONBUF_BASE    0x600c9000 /* AUX_ANAIF CPU nonbuf base */
#define TIVA_AUX_DDI0_OSC_NONBUF_BASE 0x600ca000 /* DDI CPU nonbuf base */
#define TIVA_AUX_ADI4_NONBUF_BASE     0x600cb000 /* ADI CPU nonbuf base */
#define TIVA_AUX_RAM_NONBUF_BASE      0x600e0000 /* AUX_RAM CPU nonbuf base */
#define TIVA_AUX_SCE_NONBUF_BASE      0x600e1000 /* AUX_SCE CPU nonbuf base */
#define TIVA_FLASHMEM_ALIAS_BASE      0xa0000000 /* FLASHMEM Alias base */
#define TIVA_CPU_ITM_BASE             0xe0000000 /* CPU_ITM */
#define TIVA_CPU_DWT_BASE             0xe0001000 /* CPU_DWT */
#define TIVA_CPU_FPB_BASE             0xe0002000 /* CPU_FPB */
#define TIVA_CPU_SCS_BASE             0xe000e000 /* CPU_SCS */
#define TIVA_CPU_TPIU_BASE            0xe0040000 /* CPU_TPIU */
#define TIVA_CPU_TIPROP_BASE          0xe00fe000 /* CPU_TIPROP */
#define TIVA_CPU_ROM_TABLE_BASE       0xe00ff000 /* CPU_ROM_TABLE */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_C13X0_MEMORYMAP_H */
