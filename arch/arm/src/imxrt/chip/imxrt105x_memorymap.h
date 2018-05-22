/************************************************************************************
 * arch/arm/src/imxrt/chip/imxrt105x_memorymap.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT105X_MEMORYMAP_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT105X_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* System memory map */

#define IMXRT_ITCM_BASE           0x00000000  /* 512KB ITCM */
                               /* 0x00080000     512KB ITCM Reserved */
                               /* 0x00100000     1MB ITCM Reserved */
#define IMXRT_ROMCP_BASE          0x00200000  /* 96KB ROMCP */
                               /* 0x00218000     416KB ROMCP Reserved */
                               /* 0x00280000     1536KB Reserved */
                               /* 0x00400000     128MB Reserved */
#define IMXRT_FLEXSPI_BASE        0x08000000  /* 128MB FlexSPI (Aliased) */
#define IMXRT_SEMCA_BASE          0x10000000  /* 256MB SEMC (Aliased) */
#define IMXRT_DTCM_BASE           0x20000000  /* 512KB DTCM */
                               /* 0x20080000     512KB DTCM Reserved */
                               /* 0x20100000     1MB Reserved */
#define IMXRT_OCRAM_BASE          0x20200000  /* 512KB OCRAM */
                               /* 0x20280000     1536KB OCRAM Reserved */
                               /* 0x20400000     252MB Reserved */
                               /* 0x30000000     256MB Reserved */
#define IMXRT_AIPS1_BASE          0x40000000  /* 1MB AIPS-1 */
#define IMXRT_AIPS2_BASE          0x40100000  /* 1MB AIPS-2 */
#define IMXRT_AIPS3_BASE          0x40200000  /* 1MB AIPS-3 */
#define IMXRT_AIPS4_BASE          0x40300000  /* 1MB AIPS-4 */
                               /* 40400000       12MB Reserved */
#define IMXRT_MAINCNF_BASE        0x41000000  /* 1MB "main" configuration port */
#define IMXRT_MCNF_BASE           0x41100000  /* 1MB "m" configuration port */
                               /* 41200000       1MB Reserved for "per" GPV */
                               /* 41300000       1MB Reserved for "ems" GPV */
#define IMXRT_CPUCNF_BASE         0x41400000  /* 1MB "cpu" configuration port */
                               /* 0x41500000     1MB GPV Reserved */
                               /* 0x41600000     1MB GPV Reserved */
                               /* 0x41700000     1MB GPV Reserved */
                               /* 0x41800000     8MB Reserved */
                               /* 0x42000000     32MB Reserved */
                               /* 0x44000000     64MB Reserved */
                               /* 0x48000000     384MB Reserved */
#define IMXRT_FLEXCIPHER_BASE     0x60000000  /* 504MB FlexSPI/ FlexSPI ciphertext */
#define IMXRT_FLEXSPITX_BASE      0x7f800000  /* 4MB FlexSPI TX FIFO */
#define IMXRT_FLEXSPIRX_BASE      0x7fc00000  /* 4MB FlexSPI RX FIFO */
#define IMXRT_EXTMEM_BASE         0x80000000  /* 1.5GB SEMC external memories shared memory space */
#define IMXRT_CM7_BASE            0xe0000000  /* 1MB CM7 PPB */
                               /* 0xe0100000     511MB Reserved */

/* AIPS-1 memory map */

                               /* 0x40000000     256KB Reserved */
                               /* 0x40040000     240KB Reserved */
#define IMXRT_AIPS1CNF_BASE       0x4007c000  /* 6KB AIPS-1 Configuration */
#define IMXRT_DCDC_BASE           0x40080000  /* 16KB DCDC */
#define IMXRT_PIT_BASE            0x40084000  /* 16KB PIT */
                               /* 0x40088000     16KB Reserved */
                               /* 0x4008c000     16KB Reserved */
#define IMXRT_MTR_BASE            0x40090000  /* 16KB MTR */
#define IMXRT_ACMP_BASE           0x40094000  /* 16KB ACMP */
                               /* 0x40098000     16KB Reserved */
                               /* 0x4009c000     16KB Reserved */
                               /* 0x400a0000     16KB Reserved */
#define IMXRT_IOMUXCSNVSGPR_BASE  0x400a4000  /* 16KB IOMUXC_SNVS_GPR */
#define IMXRT_IOMUXCSNVS_BASE     0x400a8000  /* 16KB IOMUXC_SNVS */
#define IMXRT_IOMUXCGPR_BASE      0x400ac000  /* 16KB IOMUXC_GPR */
#define IMXRT_FLEXRAM_BASE        0x400b0000  /* 16KB CM7_MX6RT(FLEXRAM) */
#define IMXRT_EWM_BASE            0x400b4000  /* 16KB EWM */
#define IMXRT_WDOG1_BASE          0x400b8000  /* 16KB WDOG1 */
#define IMXRT_WDOG3_BASE          0x400bc000  /* 16KB WDOG3 */
#define IMXRT_GPIO5_BASE          0x400c0000  /* 16KB GPIO5 */
#define IMXRT_ADC1_BASE           0x400c4000  /* 16KB ADC1 */
#define IMXRT_ADC2_BASE           0x400c8000  /* 16KB ADC2 */
#define IMXRT_TRNG_BASE           0x400cc000  /* 16KB TRNG */
#define IMXRT_WDOG2_BASE          0x400d0000  /* 16KB WDOG2 */
#define IMXRT_SNVSHP_BASE         0x400d4000  /* 16KB SNVS_HP */
#define IMXRT_ANATOP_BASE         0x400d8000  /* 16KB ANATOP */
#define IMXRT_CSU_BASE            0x400dc000  /* 16KB CSU */
                               /* 0x400e0000     16KB Reserved */
                               /* 0x400e4000     16KB Reserved */
#define IMXRT_EDMA_BASE           0x400e8000  /* 16KB EDMA */
#define IMXRT_DMAMUX_BASE         0x400ec000  /* 16KB DMA_CH_MUX */
                               /* 400f0000       16KB Reserved */
#define IMXRT_GPC_BASE            0x400f4000  /* 16KB GPC */
#define IMXRT_SRC_BASE            0x400f8000  /* 16KB SRC */
#define IMXRT_CCM_BASE            0x400fc000  /* 16KB CCM */

/* AIPS-2 memory map */

                               /* 0x40100000     256KB Reserved */
                               /* 0x40140000     240KB Reserved */
#define IMXRT_AIPS2CNF_BASE       0x4017c000  /* 16KB AIPS-2 Configuration */
#define IMXRT_ROMCPC_BASE         0x40180000  /* 16KB ROMCP controller*/
#define IMXRT_LPUART1_BASE        0x40184000  /* 16KB LPUART1 */
#define IMXRT_LPUART2_BASE        0x40188000  /* 16KB LPUART2 */
#define IMXRT_LPUART3_BASE        0x4018c000  /* 16KB LPUART3 */
#define IMXRT_LPUART4_BASE        0x40190000  /* 16KB LPUART4 */
#define IMXRT_LPUART5_BASE        0x40194000  /* 16KB LPUART5 */
#define IMXRT_LPUART6_BASE        0x40198000  /* 16KB LPUART6 */
#define IMXRT_LPUART7_BASE        0x4019c000  /* 16KB LPUART7 */
#define IMXRT_LPUART8_BASE        0x401a0000  /* 16KB LPUART8 */
                               /* 0x401a4000     16KB Reserved */
                               /* 0x401a8000     16KB Reserved */
#define IMXRT_FLEXIO1_BASE        0x401ac000  /* 16KB FlexIO1 */
#define IMXRT_FLEXIO2_BASE        0x401b0000  /* 16KB FlexIO2 */
                               /* 0x401b4000     16KB Reserved */
#define IMXRT_GPIO1_BASE          0x401b8000  /* 16KB GPIO1 */
#define IMXRT_GPIO2_BASE          0x401bc000  /* 16KB GPIO2 */
#define IMXRT_GPIO3_BASE          0x401c0000  /* 16KB GPIO3 */
#define IMXRT_GPIO4_BASE          0x401c4000  /* 16KB GPIO4 */
                               /* 0x401c8000     16KB Reserved */
                               /* 0x401cc000     16KB Reserved */
#define IMXRT_CAN1_BASE           0x401d0000  /* 16KB CAN1 */
#define IMXRT_CAN2_BASE           0x401d4000  /* 16KB CAN2 */
                               /* 0x401d8000     16KB Reserved */
#define IMXRT_QTIMER1_BASE        0x401dc000  /* 16KB QTimer1 */
#define IMXRT_QTIMER2_BASE        0x401e0000  /* 16KB QTimer2 */
#define IMXRT_QTIMER3_BASE        0x401e4000  /* 16KB QTimer3 */
#define IMXRT_QTIMER4_BASE        0x401e8000  /* 16KB QTimer4 */
#define IMXRT_GPT1_BASE           0x401ec000  /* 16KB GPT1 */
#define IMXRT_GPT2_BASE           0x401f0000  /* 16KB GPT2 */
#define IMXRT_OCOTP_BASE          0x401f4000  /* 16KB OCOTP */
#define IMXRT_IOMUXC_BASE         0x401f8000  /* 16KB IOMUXC */
#define IMXRT_KPP_BASE            0x401fc000  /* 16KB KPP */

/* AIPS-3 memory map */

                               /* 0x40200000     256KB Reserved */
                               /* 0x40240000     240KB Reserved */
#define IMXRT_AIOS3CNF_BASE       0x4027c000  /* 16KB AIPS-3 Configuration */
                               /* 0x40280000     16KB Reserved */
                               /* 0x40284000     16KB Reserved */
                               /* 0x40288000     16KB Reserved */
                               /* 0x4028c000     16KB Reserved */
                               /* 0x40290000     16KB Reserved */
                               /* 0x40294000     16KB Reserved */
                               /* 0x40298000     16KB Reserved */
                               /* 0x4029c000     16KB Reserved */
                               /* 0x402a0000     16KB Reserved */
                               /* 0x402a4000     16KB Reserved */
#define IMXRT_FLEXSPIC_BASE       0x402a8000  /* 16KB FlexSPI controller */
                               /* 0x402ac000     16KB Reserved */
                               /* 0x402b0000     16KB Reserved */
#define IMXRT_PXP_BASE            0x402b4000  /* 16KB PXP */
#define IMXRT_LCDIF_BASE          0x402b8000  /* 16KB LCDIF */
#define IMXRT_CSI_BASE            0x402bc000  /* 16KB CSI */
#define IMXRT_USDHC1_BASE         0x402c0000  /* 16KB USDHC1 */
#define IMXRT_USDHC2_BASE         0x402c4000  /* 16KB USDHC2 */
                               /* 0x402c8000     16KB Reserved */
                               /* 0x402cc000     16KB Reserved */
                               /* 0x402d0000     16KB Reserved */
                               /* 0x402d4000     16KB Reserved */
#define IMXRT_ENET_BASE           0x402d8000  /* 16KB ENET */
#define IMXRT_USBPL301_BASE       0x402dc000  /* 16KB USB(PL301) */
#define IMXRT_USB_BASE            0x402e0000  /* 16KB USB(USB) */
                               /* 0x402e4000     16KB Reserved */
                               /* 0x402e8000     16KB Reserved */
                               /* 0x402ec000     16KB Reserved */
#define IMXRT_SEMC_BASE           0x402f0000  /* 16KB SEMC */
                               /* 0x402f4000     16KB Reserved */
                               /* 0x402f8000     16KB Reserved */
#define IMXRT_DCP_BASE            0x402fc000  /* 16KB DCP */

/* AIPS-4 memory map */

                               /* 0x40300000     256KB Reserved */
                               /* 0x40340000     240KB Reserved */
#define IMXRT_AIPS4CNF_BASE       0x4037c000  /* 16KB AIPS-4 Configuration */
#define IMXRT_SPDIF_BASE          0x40380000  /* 16KB SPDIF */
#define IMXRT_SAI1_BASE           0x40384000  /* 16KB SAI1 */
#define IMXRT_SAI2_BASE           0x40388000  /* 16KB SAI2 */
#define IMXRT_SAI3_BASE           0x4038c000  /* 16KB SAI3 */
                               /* 0x40390000     16KB Reserved */
#define IMXRT_LPSPI1_BASE         0x40394000  /* 16KB LPSPI1 */
#define IMXRT_LPSPI2_BASE         0x40398000  /* 16KB LPSPI2 */
#define IMXRT_LPSPI3_BASE         0x4039c000  /* 16KB LPSPI3 */
#define IMXRT_LPSPI4_BASE         0x403a0000  /* 16KB LPSPI4 */
                               /* 0x403a4000     16KB Reserved */
                               /* 0x403a8000     16KB Reserved */
                               /* 0x403ac000     16KB Reserved */
#define IMXRT_ADCETC_BASE         0x403b0000  /* 16KB ADC_ETC */
#define IMXRT_AOI1_BASE           0x403b4000  /* 16KB AOI1 */
#define IMXRT_AOI2_BASE           0x403b8000  /* 16KB AOI2 */
#define IMXRT_XBAR1_BASE          0x403bc000  /* 16KB XBAR1 */
#define IMXRT_XBAR2_BASE          0x403c0000  /* 16KB XBAR2 */
#define IMXRT_XBAR3_BASE          0x403c4000  /* 16KB XBAR3 */
#define IMXRT_ENC1_BASE           0x403c8000  /* 16KB ENC1 */
#define IMXRT_ENC2_BASE           0x403cc000  /* 16KB ENC2 */
#define IMXRT_ENC3_BASE           0x403d0000  /* 16KB ENC3 */
#define IMXRT_ENC4_BASE           0x403d4000  /* 16KB ENC4 */
                               /* 0x403d8000     16KB Reserved */
#define IMXRT_FLEXPWM1_BASE       0x403dc000  /* 16KB FLEXPWM1 */
#define IMXRT_FLEXPWM2_BASE       0x403e0000  /* 16KB FLEXPWM2 */
#define IMXRT_FLEXPWM3_BASE       0x403e4000  /* 16KB FLEXPWM3 */
#define IMXRT_FLEXPWM4_BASE       0x403e8000  /* 16KB FLEXPWM4 */
#define IMXRT_BEE_BASE            0x403ec000  /* 16KB BEE */
#define IMXRT_LPI2C1_BASE         0x403f0000  /* 16KB  */
#define IMXRT_LPI2C2_BASE         0x403f4000  /* 16KB LPI2C2 */
#define IMXRT_LPI2C3_BASE         0x403f8000  /* 16KB LPI2C3 */
#define IMXRT_LPI2C4_BASE         0x403fc000  /* 16KB LPI2C4 */

/* PPB memory map */

#define IMXRT_TPIU_BASE           0xe0040000  /* 4KB TPIU */
#define IMXRT_ETM_BASE            0xe0041000  /* 4KB ETM */
#define IMXRT_CTI_BASE            0xe0042000  /* 4KB CTI */
#define IMXRT_TSGEN_BASE          0xe0043000  /* 4KB TSGEN */
#define IMXRT_PPBRES_BASE         0xe0044000  /* 4KB PPB RES */
                               /* 0xe0045000     236KB PPB Reserved */
#define IMXRT_MCM_BASE            0xe0080000  /* 4KB MCM */
                               /* 0xe0081000     444KB PPB Reserved */
                               /* 0xe00f0000     52KB PPB Reserved */
#define IMXRT_SYSROM_BASE         0xe00fd000  /* 4KB SYS ROM */
#define IMXRT_PROCROM_BASE        0xe00fe000  /* 4KB Processor ROM */
#define IMXRT_PPBROM_BASE         0xe00ff000  /* 4KB PPB ROM */

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT105X_MEMORYMAP_H */
