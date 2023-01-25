/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_memorymap.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_MEMORYMAP_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Memory Map ********************************************************/

#define IMXRT_ITCM_BASE            (0x00000000) /* 512 KB, CM7: ITCM (FlexRAM) */

/*                                 (0x00080000)    512 KB, Reserved
 *                                 (0x00100000)      1 MB, Reserved
 *                                 (0x00200000)    256 KB, Reserved
 *                                 (0x00240000)    256 KB, Reserved
 */

#define IMXRT_CAAM_BASE            (0x00280000) /*  64 KB, CAAM Secure RAM */

/*                                 (0x00290000)   1472 KB, Reserved
 *                                 (0x00400000)    124 MB, Reserved
 */

#define IMXRT_FLEXSPI1_ALIAS_BASE  (0x08000000) /* 256 MB, CM4: FlexSPI1 (alias) */

                                /* (0x18000000)    128 MB, Reserved */

#define IMXRT_CODE_TCM_BASE        (0x1ffe0000) /* 128 KB, CM4: Code TCM (LMEM RAM_L) */
#define IMXRT_SYSTEM_TCM_BASE      (0x20000000) /* 128 KB, CM4: System TCM (LMEM RAM_U) */
#define IMXRT_DTCM_BASE            (0x20000000) /* 512 KB, CM7: DTCM (FlexRAM) */

/*                                 (0x20080000)    512 KB, Reserved
 *                                 (0x20100000)      1 MB, Reserved
 */

#define IMXRT_OCRAM_M4_BASE        (0x20200000) /* 256 KB, OCRAM M4 (LMEM 128 KB SRAM_L + 128 KB SRAM_U backdoor) */
#define IMXRT_OCRAM_BASE           (0x20240000) /* 512 KB, OCRAM1 */
#define IMXRT_OCRAM2_BASE          (0x202c0000) /* 512 KB, OCRAM2 */
#define IMXRT_OCRAM1_ECC_BASE      (0x20340000) /*  64 KB, OCRAM1 ECC */
#define IMXRT_OCRAM2_ECC_BASE      (0x20350000) /*  64 KB, OCRAM2 ECC */
#define IMXRT_OCRAM_M7_BASE        (0x20360000) /* 128 KB, OCRAM M7 (FlexRAM) */
#define IMXRT_OCRAM_M7_ECC_BASE    (0x20380000) /* 512 KB, OCRAM M7 (FlexRAM ECC) */

                                /* (0x20400000)    244 MB, Reserved */

#define IMXRT_FLEXSPI1_TX_BASE     (0x2f800000) /*   4 MB, FlexSPI1 TX FIFO */
#define IMXRT_FLEXSPI1_RX_BASE     (0x2fc00000) /*   4 MB, FlexSPI1 RX FIFO */
#define IMXRT_FLEXSPI1_CIPHER_BASE (0x30000000) /* 256 MB, FlexSPI1 / FlexSPI1 cipher text */
#define IMXRT_AIPS1_BASE           (0x40000000) /*   4 MB, AIPS-1 */
#define IMXRT_AIPS2_BASE           (0x40400000) /*   4 MB, AIPS-2 */
#define IMXRT_AIPS3_BASE           (0x40800000) /*   4 MB, AIPS-3 */
#define IMXRT_AIPS4_BASE           (0x40c00000) /*   4 MB, AIPS-4 */
#define IMXRT_SIM_DISP_BASE        (0x41000000) /*   1 MB, SIM_DISP configuration port */
#define IMXRT_SIM_M_BASE           (0x41100000) /*   1 MB, SIM_M configuration port */

/*                                 (0x41200000)      1 MB, Reserved
 *                                 (0x41300000)      1 MB, Reserved
 */

#define IMXRT_SIM_M7_BASE          (0x41400000) /*   1 MB, SIM_M7 configuration port */

/*                                 (0x41500000)      1 MB, Reserved
 *                                 (0x41600000)      1 MB, Reserved
 *                                 (0x41700000)      1 MB, Reserved
 */

#define IMXRT_GPU2D_BASE           (0x41800000) /*   1 MB, GPU2D (Peripheral, AHB) */
#define IMXRT_CDOG_BASE            (0x41900000) /*   1 MB, CDOG (Peripheral, AHB) */

                                /* (0x41a00000)      6 MB, Reserved */

#define IMXRT_AIPS_M7_BASE         (0x42000000) /*   1 MB, CM7: AIPS M7 (Peripheral, Platform) */

/*                                 (0x42100000)      7 MB, Reserved
 *                                 (0x42800000)    472 MB, Reserved
 */

#define IMXRT_FLEXSPI2_CIPHER_BASE (0x60000000) /* 504 MB, FlexSPI2 / FlexSPI2 cipher text */
#define IMXRT_FLEXSPI2_TX_BASE     (0x7f800000) /*   4 MB, FlexSPI2 TX FIFO */
#define IMXRT_FLEXSPI2_RX_BASE     (0x7fc00000) /*   4 MB, FlexSPI2 RX FIFO */
#define IMXRT_SEMC0_BASE           (0x80000000) /* 256 MB, SEMC0 */
#define IMXRT_SEMC1_BASE           (0x90000000) /* 256 MB, SEMC1 */
#define IMXRT_SEMC2_BASE           (0xa0000000) /* 512 MB, SEMC2 */
#define IMXRT_SEMC3_BASE           (0xc0000000) /* 512 MB, SEMC3 */
#define IMXRT_PPB_M7_BASE          (0xe0000000) /*   1 MB, CM7: PPB M7 */
#define IMXRT_PPB_M4_BASE          (0xe0000000) /*   1 MB, CM4: PPB M4 */

                                /* (0xe0100000)    511 MB, Reserved */

/* AIPS-1 Memory Map ********************************************************/

/*                                 (0x40000000)     16 KB, Reserved
 *                                 (0x40004000)     16 KB, Reserved
 *                                 (0x40008000)     16 KB, Reserved
 *                                 (0x4000c000)     16 KB, Reserved
 *                                 (0x40010000)     16 KB, Reserved
 */

#define IMXRT_MECC1_BASE           (0x40014000) /*  16 KB, MECC1 */
#define IMXRT_MECC2_BASE           (0x40018000) /*  16 KB, MECC2 */
#define IMXRT_XECC_FLEXSPI1_BASE   (0x4001c000) /*  16 KB, XECC_FLEXSPI1 */
#define IMXRT_XECC_FLEXSPI2_BASE   (0x40020000) /*  16 KB, XECC_FLEXSPI2 */
#define IMXRT_XECC_SEMC            (0x40024000) /*  16 KB, XECC_SEMC */
#define IMXRT_CM7_FLEXRAM_BASE     (0x40028000) /*  16 KB, CM7 (FLEXRAM) */
#define IMXRT_EWM_BASE             (0x4002c000) /*  16 KB, EWM */
#define IMXRT_WDOG1_BASE           (0x40030000) /*  16 KB, WDOG1 */
#define IMXRT_WDOG2_BASE           (0x40034000) /*  16 KB, WDOG2 */
#define IMXRT_WDOG3_BASE           (0x40038000) /*  16 KB, WDOG3 */
#define IMXRT_XBAR1_BASE           (0x4003c000) /*  16 KB, XBAR1 */
#define IMXRT_XBAR2_BASE           (0x40040000) /*  16 KB, XBAR2 */
#define IMXRT_XBAR3_BASE           (0x40044000) /*  16 KB, XBAR3 */
#define IMXRT_ADC_ETC_BASE         (0x40048000) /*  16 KB, ADC_ETC */

                                /* (0x4004c000)     16 KB, Reserved */

#define IMXRT_LPADC1_BASE          (0x40050000) /*  16 KB, LPADC1 */
#define IMXRT_LPADC2_BASE          (0x40054000) /*  16 KB, LPADC2 */

/*                                 (0x40058000)     16 KB, Reserved
 *                                 (0x4005c000)     16 KB, Reserved
 *                                 (0x40060000)     16 KB, Reserved
 */

#define IMXRT_DAC_BASE             (0x40064000) /*  16 KB, DAC */
#define IMXRT_IEE_APC_BASE         (0x40068000) /*  16 KB, IEE_APC */
#define IMXRT_IEE_BASE             (0x4006c000) /*  16 KB, IEE */
#define IMXRT_EDMA_BASE            (0x40070000) /*  16 KB, EDMA */
#define IMXRT_DMAMUX0_BASE         (0x40074000) /*  16 KB, DMAMUX0 */

                                /* (0x40078000)     16 KB, Reserved */

#define IMXRT_LPUART1_BASE         (0x4007c000) /*  16 KB, LPUART1 */
#define IMXRT_LPUART2_BASE         (0x40080000) /*  16 KB, LPUART2 */
#define IMXRT_LPUART3_BASE         (0x40084000) /*  16 KB, LPUART3 */
#define IMXRT_LPUART4_BASE         (0x40088000) /*  16 KB, LPUART4 */
#define IMXRT_LPUART5_BASE         (0x4008c000) /*  16 KB, LPUART5 */
#define IMXRT_LPUART6_BASE         (0x40090000) /*  16 KB, LPUART6 */
#define IMXRT_LPUART7_BASE         (0x40094000) /*  16 KB, LPUART7 */
#define IMXRT_LPUART8_BASE         (0x40098000) /*  16 KB, LPUART8 */
#define IMXRT_LPUART9_BASE         (0x4009c000) /*  16 KB, LPUART9 */
#define IMXRT_LPUART10_BASE        (0x400a0000) /*  16 KB, LPUART10 */

/*                                 (0x400a4000)     16 KB, Reserved
 *                                 (0x400a8000)     16 KB, Reserved
 */

#define IMXRT_FLEXIO1_BASE         (0x400ac000) /*  16 KB, FlexIO1 */
#define IMXRT_FLEXIO2_BASE         (0x400b0000) /*  16 KB, FlexIO2 */

                                /* (0x400b4000)     16 KB, Reserved */

#define IMXRT_AOI1_BASE            (0x400b8000) /*  16 KB, AOI1 */
#define IMXRT_AOI2_BASE            (0x400bc000) /*  16 KB, AOI2 */

                                /* (0x400c0000)     16 KB, Reserved */

#define IMXRT_CAN1_BASE            (0x400c4000) /*  16 KB, CAN1 */
#define IMXRT_CAN2_BASE            (0x400c8000) /*  16 KB, CAN2 */
#define IMXRT_CAN3_BASE            (0x40c3c000) /*  16 KB, CAN3 */
#define IMXRT_FLEXSPIC_BASE        (0x400cc000) /*  16 KB, FlexSPI1 */
#define IMXRT_FLEXSPI2C_BASE       (0x400d0000) /*  16 KB, FlexSPI2 */
#define IMXRT_SEMC_BASE            (0x400d4000) /*  16 KB, SEMC */
#define IMXRT_PIT_BASE             (0x400d8000) /*  16 KB, PIT1 */

                                /* (0x400dc000)     16 KB, Reserved */

#define IMXRT_KPP_BASE             (0x400e0000) /*  16 KB, KPP */
#define IMXRT_IOMUXCGPR_BASE       (0x400e4000) /*  16 KB, IOMUXC_GPR */
#define IMXRT_IOMUXC_BASE          (0x400e8000) /*  16 KB, IOMUXC */
#define IMXRT_GPT1_BASE            (0x400ec000) /*  16 KB, GPT1 */
#define IMXRT_GPT2_BASE            (0x400f0000) /*  16 KB, GPT2 */
#define IMXRT_GPT3_BASE            (0x400f4000) /*  16 KB, GPT3 */
#define IMXRT_GPT4_BASE            (0x400f8000) /*  16 KB, GPT4 */
#define IMXRT_GPT5_BASE            (0x400fc000) /*  16 KB, GPT5 */
#define IMXRT_GPT6_BASE            (0x40100000) /*  16 KB, GPT6 */
#define IMXRT_LPI2C1_BASE          (0x40104000) /*  16 KB, LPI2C1 */
#define IMXRT_LPI2C2_BASE          (0x40108000) /*  16 KB, LPI2C2 */
#define IMXRT_LPI2C3_BASE          (0x4010c000) /*  16 KB, LPI2C3 */
#define IMXRT_LPI2C4_BASE          (0x40110000) /*  16 KB, LPI2C4 */
#define IMXRT_LPSPI1_BASE          (0x40114000) /*  16 KB, LPSPI1 */
#define IMXRT_LPSPI2_BASE          (0x40118000) /*  16 KB, LPSPI2 */
#define IMXRT_LPSPI3_BASE          (0x4011c000) /*  16 KB, LPSPI3 */
#define IMXRT_LPSPI4_BASE          (0x40120000) /*  16 KB, LPSPI4 */

/*                                 (0x40124000)     16 KB, Reserved
 *                                 (0x40128000)     16 KB, Reserved
 */

#define IMXRT_GPIO1_BASE           (0x4012c000) /*  16 KB, GPIO1 */
#define IMXRT_GPIO2_BASE           (0x40130000) /*  16 KB, GPIO2 */
#define IMXRT_GPIO3_BASE           (0x40134000) /*  16 KB, GPIO3 */
#define IMXRT_GPIO4_BASE           (0x40138000) /*  16 KB, GPIO4 */
#define IMXRT_GPIO5_BASE           (0x4013c000) /*  16 KB, GPIO5 */
#define IMXRT_GPIO6_BASE           (0x40140000) /*  16 KB, GPIO6 */

/*                                 (0x40144000)     16 KB, Reserved
 *                                 (0x40148000)     16 KB, Reserved
 *                                 (0x4014c000)     16 KB, Reserved
 *                                 (0x40150000)     16 KB, Reserved
 */

#define IMXRT_EMVSIM1_BASE         (0x40154000) /*  16 KB, EMVSIM1 */
#define IMXRT_EMVSIM2_BASE         (0x40158000) /*  16 KB, EMVSIM2 */
#define IMXRT_TMR1_BASE            (0x4015c000) /*  16 KB, TMR1 */
#define IMXRT_TMR2_BASE            (0x40160000) /*  16 KB, TMR2 */
#define IMXRT_TMR3_BASE            (0x40164000) /*  16 KB, TMR3 */
#define IMXRT_TMR4_BASE            (0x40168000) /*  16 KB, TMR4 */

/*                                 (0x4016c000)     16 KB, Reserved
 *                                 (0x40170000)     16 KB, Reserved
 */

#define IMXRT_QDC1_BASE            (0x40174000) /*  16 KB, QDC1 */
#define IMXRT_QDC2_BASE            (0x40178000) /*  16 KB, QDC2 */
#define IMXRT_QDC3_BASE            (0x4017c000) /*  16 KB, QDC3 */
#define IMXRT_QDC4_BASE            (0x40180000) /*  16 KB, QDC4 */

/*                                 (0x40184000)     16 KB, Reserved
 *                                 (0x40188000)     16 KB, Reserved
 */

#define IMXRT_FLEXPWM1_BASE        (0x4018c000) /*  16 KB, FLEXPWM1 */
#define IMXRT_FLEXPWM2_BASE        (0x40190000) /*  16 KB, FLEXPWM2 */
#define IMXRT_FLEXPWM3_BASE        (0x40194000) /*  16 KB, FLEXPWM3 */
#define IMXRT_FLEXPWM4_BASE        (0x40198000) /*  16 KB, FLEXPWM4 */

/*                                 (0x4019c000)     16 KB, Reserved
 *                                 (0x401a0000)     16 KB, Reserved
 */

#define IMXRT_ACMP1_BASE           (0x401a4000) /*  16 KB, ACMP1 */
#define IMXRT_ACMP2_BASE           (0x401a8000) /*  16 KB, ACMP2 */
#define IMXRT_ACMP3_BASE           (0x401ac000) /*  16 KB, ACMP3 */
#define IMXRT_ACMP4_BASE           (0x401b0000) /*  16 KB, ACMP4 */

/*                                 (0x401b4000)     16 KB, Reserved
 *                                 (0x401b8000)     16 KB, Reserved
 *                                 (0x401bc000)     16 KB, Reserved
 *                                 (0x401c0000)     16 KB, Reserved
 *                                 (0x401c4000)     16 KB, Reserved
 *                                 (0x401c8000)     16 KB, Reserved
 *                                 (0x401cc000)     16 KB, Reserved
 *                                 (0x401d0000)     16 KB, Reserved
 *                                 (0x401d4000)     16 KB, Reserved
 *                                 (0x401d8000)     16 KB, Reserved
 *                                 (0x401dc000)     16 KB, Reserved
 *                                 (0x401e0000)     16 KB, Reserved
 *                                 (0x401e4000)     16 KB, Reserved
 *                                 (0x401e8000)     16 KB, Reserved
 *                                 (0x401ec000)     16 KB, Reserved
 *                                 (0x401f0000)     16 KB, Reserved
 *                                 (0x401f4000)     16 KB, Reserved
 *                                 (0x401f8000)     16 KB, Reserved
 *                                 (0x401fc000)     16 KB, Reserved
 *                                 (0x40200000)      1 MB, Reserved
 *                                 (0x40300000)      1 MB, Reserved
 */

/* AIPS-2 Memory Map ********************************************************/

#define IMXRT_SPDIF_BASE           (0x40400000) /*  16 KB, SPDIF */
#define IMXRT_SAI1_BASE            (0x40404000) /*  16 KB, SAI1 */
#define IMXRT_SAI2_BASE            (0x40408000) /*  16 KB, SAI2 */
#define IMXRT_SAI3_BASE            (0x4040c000) /*  16 KB, SAI3 */

                                /* (0x40410000)     16 KB, Reserved */

#define IMXRT_ASRC_BASE            (0x40414000) /*  16 KB, ASRC */
#define IMXRT_USDHC1_BASE          (0x40418000) /*  16 KB, USDHC1 */
#define IMXRT_USDHC2_BASE          (0x4041c000) /*  16 KB, USDHC2 */
#define IMXRT_ENET_1G_BASE         (0x40420000) /*  16 KB, ENET_1G */
#define IMXRT_ENET_BASE            (0x40424000) /*  16 KB, ENET */

                                /* (0x40428000)     16 KB, Reserved */

#define IMXRT_USBOTG2_BASE         (0x4042c000) /*  16 KB, USBOTG2 */
#define IMXRT_USB_BASE             (0x40430000) /*  16 KB, USBOTG1 */
#define IMXRT_USBPHY1_BASE         (0x40434000) /*  16 KB, USBPHY1 */
#define IMXRT_USBPHY2_BASE         (0x40438000) /*  16 KB, USBPHY2 */
#define IMXRT_ENET_QOS_BASE        (0x4043c000) /*  16 KB, ENET_QOS */

/*                                 (0x40440000)     16 KB, CAAM (General)
 *                                 (0x40444000)     16 KB, CAAM (General)
 *                                 (0x40448000)     16 KB, CAAM (General)
 *                                 (0x4044c000)     16 KB, CAAM (General)
 *                                 (0x40450000)     16 KB, CAAM (JR0)
 *                                 (0x40454000)     16 KB, CAAM (JR0)
 *                                 (0x40458000)     16 KB, CAAM (JR0)
 *                                 (0x4045c000)     16 KB, CAAM (JR0)
 *                                 (0x40460000)     16 KB, CAAM (JR1)
 *                                 (0x40464000)     16 KB, CAAM (JR1)
 *                                 (0x40468000)     16 KB, CAAM (JR1)
 *                                 (0x4046c000)     16 KB, CAAM (JR1)
 *                                 (0x40470000)     16 KB, CAAM (JR2)
 *                                 (0x40474000)     16 KB, CAAM (JR2)
 *                                 (0x40478000)     16 KB, CAAM (JR2)
 *                                 (0x4047c000)     16 KB, CAAM (JR2)
 *                                 (0x40480000)     16 KB, CAAM (JR3)
 *                                 (0x40484000)     16 KB, CAAM (JR3)
 *                                 (0x40488000)     16 KB, CAAM (JR3)
 *                                 (0x4048c000)     16 KB, CAAM (JR3)
 *                                 (0x40490000)     16 KB, CAAM (Reserved)
 *                                 (0x40494000)     16 KB, CAAM (Reserved)
 *                                 (0x40498000)     16 KB, CAAM (Reserved)
 *                                 (0x4049c000)     16 KB, CAAM (Reserved)
 *                                 (0x404a0000)     16 KB, CAAM (RTIC)
 *                                 (0x404a4000)     16 KB, CAAM (RTIC)
 *                                 (0x404a8000)     16 KB, CAAM (RTIC)
 *                                 (0x404ac000)     16 KB, CAAM (RTIC)
 *                                 (0x404b0000)     16 KB, CAAM (Reserved)
 *                                 (0x404b4000)     16 KB, CAAM (Reserved)
 *                                 (0x404b8000)     16 KB, CAAM (Reserved)
 *                                 (0x404bc000)     16 KB, CAAM (Reserved)
 *                                 (0x404c0000)     16 KB, CAAM (Debug)
 *                                 (0x404c4000)     16 KB, CAAM (Debug)
 *                                 (0x404c8000)     16 KB, CAAM (Debug)
 *                                 (0x404cc000)     16 KB, CAAM (Debug)
 *                                 (0x404d0000)    448 KB, CAAM (Reserved)
 *                                 (0x40540000)    768 KB, Reserved
 *                                 (0x40600000)      1 MB, Reserved
 *                                 (0x40700000)      1 MB, Reserved
 */

/* AIPS-3 Memory Map ********************************************************/

#define IMXRT_CSI_BASE             (0x40800000) /*  16 KB, CSI */
#define IMXRT_ELCDIF_BASE          (0x40804000) /*  16 KB, eLCDIF */
#define IMXRT_LCDIFV2_BASE         (0x40808000) /*  16 KB, LCDIFv2 */
#define IMXRT_MIPI_DSI_BASE        (0x4080c000) /*  16 KB, MIPI_DSI */
#define IMXRT_MIPI_CSI_BASE        (0x40810000) /*  16 KB, MIPI_CSI */
#define IMXRT_PXP_BASE             (0x40814000) /*  16 KB, PXP */
#define IMXRT_VIDEO_MUX_BASE       (0x40818000) /*  16 KB, VIDEO_MUX */

/*                                 (0x4081c000)     16 KB, Reserved
 *                                 (0x40a00000)      1 MB, Reserved
 *                                 (0x40b00000)      1 MB, Reserved
 */

/* AIPS-4 Memory Map ********************************************************/

#define IMXRT_GPC_BASE             (0x40c00000) /*  16 KB, GPC */
#define IMXRT_SRC_BASE             (0x40c04000) /*  16 KB, SRC */
#define IMXRT_IOMUXCLPSR_BASE      (0x40c08000) /*  16 KB, IOMUXC_LPSR */
#define IMXRT_IOMUXCLPSRGPR_BASE   (0x40c0c000) /*  16 KB, IOMUXC_LPSR_GPR */
#define IMXRT_WDOG4_BASE           (0x40c10000) /*  16 KB, WDOG4 */
#define IMXRT_EDMA_LPSR_BASE       (0x40c14000) /*  16 KB, EDMA_LPSR */
#define IMXRT_DMAMUX1_BASE         (0x40c18000) /*  16 KB, DMAMUX1 (LPSR) */

                                /* (0x40c1c000)     16 KB, Reserved */

#define IMXRT_PDM_BASE             (0x40c20000) /*  16 KB, PDM */
#define IMXRT_LPUART11_BASE        (0x40c24000) /*  16 KB, LPUART11 */
#define IMXRT_LPUART12_BASE        (0x40c28000) /*  16 KB, LPUART12 */
#define IMXRT_LPSPI5_BASE          (0x40c2c000) /*  16 KB, LPSPI5 */
#define IMXRT_LPSPI6_BASE          (0x40c30000) /*  16 KB, LPSPI6 */
#define IMXRT_LPI2C5_BASE          (0x40c34000) /*  16 KB, LPI2C5 */
#define IMXRT_LPI2C6_BASE          (0x40c38000) /*  16 KB, LPI2C6 */
#define IMXRT_CAN3                 (0x40c3c000) /*  16 KB, CAN3 */
#define IMXRT_SAI4                 (0x40c40000) /*  16 KB, SAI4 */
#define IMXRT_RDC_SEMAPHORE1_BASE  (0x40c44000) /*  16 KB, RDC_SEMAPHORE1 */
#define IMXRT_MU_A                 (0x40c48000) /*  16 KB, MU-A */
#define IMXRT_MU_B                 (0x40c4c000) /*  16 KB, MU-B */

/*                                 (0x40c50000)     16 KB, Reserved
 *                                 (0x40c54000)     16 KB, Reserved
 *                                 (0x40c58000)     16 KB, Reserved
 */

#define IMXRT_GPIO7_BASE           (0x40c5c000) /*  16 KB, GPIO7 */
#define IMXRT_GPIO8_BASE           (0x40c60000) /*  16 KB, GPIO8 */
#define IMXRT_GPIO9_BASE           (0x40c64000) /*  16 KB, GPIO9 */
#define IMXRT_GPIO10_BASE          (0x40c68000) /*  16 KB, GPIO10 */
#define IMXRT_GPIO11_BASE          (0x40c6c000) /*  16 KB, GPIO11 */
#define IMXRT_GPIO12_BASE          (0x40c70000) /*  16 KB, GPIO12 */

                                /* (0x40c74000)     16 KB, Reserved */

#define IMXRT_RDC_BASE             (0x40c78000) /*  16 KB, RDC */

                                /* (0x40c7c000)     16 KB, Reserved */

#define IMXRT_KEYMGR_BASE          (0x40c80000) /*  16 KB, KEYMGR */
#define IMXRT_ANADIG_BASE          (0x40c84000) /*  16 KB, ANALOG / ANADIG */
#define IMXRT_PGMC_BASE            (0x40c88000) /*  16 KB, PGMC */

                                /* (0x40c8c000)     16 KB, Reserved */

#define IMXRT_SNVSHP_BASE          (0x40c90000) /*  16 KB, SNVS */
#define IMXRT_IOMUXCSNVS_BASE      (0x40c94000) /*  16 KB, IOMUXC_SNVS */
#define IMXRT_IOMUXCSNVSGPR_BASE   (0x40c98000) /*  16 KB, IOMUXC_SNVS_GPR */
#define IMXRT_SNVS_SRAM_BASE       (0x40c9c000) /*  16 KB, SNVS_SRAM */
#define IMXRT_GPIO13_BASE          (0x40ca0000) /*  16 KB, GPIO13 */
#define IMXRT_ROMCP_BASE           (0x40ca4000) /*  16 KB, ROMCP */
#define IMXRT_DCDC_BASE            (0x40ca8000) /*  16 KB, DCDC */
#define IMXRT_OCOTP_BASE           (0x40cac000) /*  16 KB, OCOTP */
#define IMXRT_PIT2_BASE            (0x40cb0000) /*  16 KB, PIT2 */
#define IMXRT_SSARC_HP_BASE        (0x40cb4000) /*  16 KB, SSARC_HP (SRAM) */
#define IMXRT_SSARC_LP_BASE        (0x40cb8000) /*  16 KB, SSARC_LP */

                                /* (0x40cbc000)     16 KB, Reserved */

#define IMXRT_CCM_BASE             (0x40cc0000) /*  32 KB, CCM */
#define IMXRT_SEMA4_BASE           (0x40cc8000) /*  16 KB, SEMA4 */
#define IMXRT_RDC_SEMAPHORE2_BASE  (0x40ccc000) /*  16 KB, RDC_SEMAPHORE2 */

/*                                 (0x40cd0000)     16 KB, XRDC2 MGR M4
 *                                 (0x40cd4000)     16 KB, XRDC2 MGR M4
 *                                 (0x40cd8000)     16 KB, XRDC2 MGR M4
 *                                 (0x40cdc000)     16 KB, XRDC2 MGR M4
 *                                 (0x40ce0000)     16 KB, XRDC2 MGR M7
 *                                 (0x40ce4000)     16 KB, XRDC2 MGR M7
 *                                 (0x40ce8000)     16 KB, XRDC2 MGR M7
 *                                 (0x40cec000)     16 KB, XRDC2 MGR M7
 *                                 (0x40cf0000)   1088 KB, Reserved
 *                                 (0x40e00000)      1 MB, Reserved
 *                                 (0x40f00000)      1 MB, Reserved
 */

/* AIPS M7 Memory Map *******************************************************/

                                /* (0x42000000)     32 KB, Reserved */

#define IMXRT_CM7_GPIO2_BASE       (0x42008000) /*  16 KB, CM7_GPIO2 */
#define IMXRT_CM7_GPIO3_BASE       (0x4200c000) /*  16 KB, CM7_GPIO3 */

                                /* (0x42010000)    960 KB, Reserved */

/* PPB Memory Map ***********************************************************/

/*                                 (0xe0000000)    256 KB, M7/M4 Internal Use
 *                                 (0xe0040000)      4 KB, PPB Reserved
 */

#define IMXRT_ETM_BASE             (0xe0041000) /*   4 KB, ETM */
#define IMXRT_CTI_BASE             (0xe0042000) /*   4 KB, CTI */
#define IMXRT_ATB_BASE             (0xe0043000) /*   4 KB, ATB Funnel */
#define IMXRT_CSSYS_CTI_BASE       (0xe0044000) /*   4 KB, CSSYS CTI */
#define IMXRT_CSSYS_ATB_BASE       (0xe0045000) /*   4 KB, CSSYS ATB Funnel */
#define IMXRT_CSSYS_TPIU_BASE      (0xe0046000) /*   4 KB, CSSYS TPIU */
#define IMXRT_CSSYS_TSGEN_BASE     (0xe0047000) /*   4 KB, CSSYS TSGEN */
#define IMXRT_CSSYS_SWO_BASE       (0xe0048000) /*   4 KB, CSSYS SWO */

                                /* (0xe0049000)    220 KB, PPB Reserved */

#define IMXRT_MCM_BASE             (0xe0080000) /*   4 KB, MCM */
#define IMXRT_MMCAU_BASE           (0xe0081000) /*   4 KB, M4: MMCAU */
#define IMXRT_AHB_LMEM_BASE        (0xe0082000) /*   4 KB, M4: AHB_LMEM */

                                /* (0xe0083000)    488 KB, PPB Reserved */

#define IMXRT_SYSROM_BASE          (0xe00fd000) /*   4 KB, SYS ROM */
#define IMXRT_PROCROM_BASE         (0xe00fe000) /*   4 KB, Processor ROM */
#define IMXRT_PPBROM_BASE          (0xe00ff000) /*   4 KB, PPB */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_MEMORYMAP_H */
