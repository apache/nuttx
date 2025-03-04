/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx93/imx93_memorymap.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_MEMORYMAP_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_GIC_DISTRIBUTOR_BASE          (0x48000000UL)
#define IMX9_GIC_REDISTRIBUTOR_BASE        (0x48040000UL)
#define IMX9_ANA_OSC_BASE                  (0x44480000UL)
#define IMX9_AXBS_BASE                     (0x44510000UL)
#define IMX9_BBNSM_BASE                    (0x44440000UL)
#define IMX9_BLK_CTRL_BBSMMIX1_BASE        (0x44410000UL)
#define IMX9_BLK_CTRL_MLMIX_BASE           (0x4A810000UL)
#define IMX9_BLK_CTRL_NIC_WRAPPER1_BASE    (0x49000000UL)
#define IMX9_BLK_CTRL_NS_AONMIX1_BASE      (0x44210000UL)
#define IMX9_BLK_CTRL_S_AONMIX2_BASE       (0x444F0000UL)
#define IMX9_BLK_CTRL_WAKEUPMIX1_BASE      (0x42420000UL)
#define IMX9_CAN1_BASE                     (0x443A0000UL)
#define IMX9_CAN2_BASE                     (0x425B0000UL)
#define IMX9_CCM_CTRL_BASE                 (0x44450000UL)
#define IMX9_CM33_MCM_BASE                 (0x44420000UL)
#define IMX9_DDR_CTRL_BASE                 (0x4E300000UL)
#define IMX9_BLK_CTRL_DDRMIX_BASE          (0x4E010000UL)
#define IMX9_DMA3_BASE                     (0x44000000UL)
#define IMX9_DMA4_BASE                     (0x42000000UL)
#define IMX9_PMRO_BASE                     (0x44484000UL)
#define IMX9_ENET_BASE                     (0x42890000UL)
#define IMX9_ENET_QOS_BASE                 (0x428A0000UL)
#define IMX9_FLEXIO1_BASE                  (0x425C0000UL)
#define IMX9_FLEXIO2_BASE                  (0x425D0000UL)
#define IMX9_FLEXSPI_BASE                  (0x425E0000UL)
#define IMX9_FLEXSPI_ARDF_BASE             (0x47420000UL)
#define IMX9_FLEXSPI_ATDF_BASE             (0x47430000UL)
#define IMX9_GPC_CTRL_CM33_BASE            (0x44470000UL)
#define IMX9_GPC_CTRL_CA55_0_BASE          (0x44470800UL)
#define IMX9_GPC_CTRL_CA55_1_BASE          (0x44471000UL)
#define IMX9_GPC_CTRL_CA55_CLUSTER_BASE    (0x44471800UL)
#define IMX9_SAI1_BASE                     (0x443B0000UL)
#define IMX9_SAI2_BASE                     (0x42650000UL)
#define IMX9_SAI3_BASE                     (0x42660000UL)
#define IMX9_I3C1_BASE                     (0x44330000UL)
#define IMX9_I3C2_BASE                     (0x42520000UL)
#define IMX9_IOMUXC1_BASE                  (0x443C0000UL)
#define IMX9_ISI_BASE                      (0x4AE40000UL)
#define IMX9_LCDIF_BASE                    (0x4AE30000UL)
#define IMX9_LPI2C1_BASE                   (0x44340000UL)
#define IMX9_LPI2C2_BASE                   (0x44350000UL)
#define IMX9_LPI2C3_BASE                   (0x42530000UL)
#define IMX9_LPI2C4_BASE                   (0x42540000UL)
#define IMX9_LPI2C5_BASE                   (0x426B0000UL)
#define IMX9_LPI2C6_BASE                   (0x426C0000UL)
#define IMX9_LPI2C7_BASE                   (0x426D0000UL)
#define IMX9_LPI2C8_BASE                   (0x426E0000UL)
#define IMX9_LPIT1_BASE                    (0x442F0000UL)
#define IMX9_LPIT2_BASE                    (0x424C0000UL)
#define IMX9_LPSPI1_BASE                   (0x44360000UL)
#define IMX9_LPSPI2_BASE                   (0x44370000UL)
#define IMX9_LPSPI3_BASE                   (0x42550000UL)
#define IMX9_LPSPI4_BASE                   (0x42560000UL)
#define IMX9_LPSPI5_BASE                   (0x426F0000UL)
#define IMX9_LPSPI6_BASE                   (0x42700000UL)
#define IMX9_LPSPI7_BASE                   (0x42710000UL)
#define IMX9_LPSPI8_BASE                   (0x42720000UL)
#define IMX9_LPTMR1_BASE                   (0x44300000UL)
#define IMX9_LPTMR2_BASE                   (0x424D0000UL)
#define IMX9_LPUART1_BASE                  (0x44380000UL)
#define IMX9_LPUART2_BASE                  (0x44390000UL)
#define IMX9_LPUART3_BASE                  (0x42570000UL)
#define IMX9_LPUART4_BASE                  (0x42580000UL)
#define IMX9_LPUART5_BASE                  (0x42590000UL)
#define IMX9_LPUART6_BASE                  (0x425A0000UL)
#define IMX9_LPUART7_BASE                  (0x42690000UL)
#define IMX9_LPUART8_BASE                  (0x426A0000UL)
#define IMX9_M33_CACHE_MCM_BASE            (0x44401000UL)
#define IMX9_BLK_CTRL_MEDIAMIX_BASE        (0x4AC10000UL)
#define IMX9_MIPI_CSI_CSR_BASE             (0x4AE00000UL)
#define IMX9_MIPI_DSI_BASE                 (0x4AE10000UL)
#define IMX9_MU1__MUB_BASE                 (0x44230000UL)
#define IMX9_MU2__MUB_BASE                 (0x42440000UL)
#define IMX9_S3MUA_BASE                    (0x47520000UL)
#define IMX9_TRDC_BASE                     (0x49010000UL)
#define IMX9_NPU_BASE                      (0x4A900000UL)
#define IMX9_OCOTP_BASE                    (0x47518000UL)
#define IMX9_OCRAM_MECC1_BASE              (0x490A0000UL)
#define IMX9_FLEXSPI_OTFAD1_BASE           (0x425E0C00UL)
#define IMX9_PDM_BASE                      (0x44520000UL)
#define IMX9_ARMPLL_BASE                   (0x44481000UL)
#define IMX9_AUDIOPLL_BASE                 (0x44481200UL)
#define IMX9_DRAMPLL_BASE                  (0x44481300UL)
#define IMX9_SYSPLL_BASE                   (0x44481100UL)
#define IMX9_VIDEOPLL_BASE                 (0x44481400UL)
#define IMX9_PXP_BASE                      (0x4AE20000UL)
#define IMX9_GPIO1_BASE                    (0x47400000UL)
#define IMX9_GPIO2_BASE                    (0x43810000UL)
#define IMX9_GPIO3_BASE                    (0x43820000UL)
#define IMX9_GPIO4_BASE                    (0x43830000UL)
#define IMX9_ROMCP1_BASE                   (0x44430000UL)
#define IMX9_ROMCP2_BASE                   (0x42640000UL)
#define IMX9_ADC1_BASE                     (0x44530000UL)
#define IMX9_SEMA42_1_BASE                 (0x44260000UL)
#define IMX9_SEMA42_2_BASE                 (0x42450000UL)
#define IMX9_SFA_BASE                      (0x44483000UL)
#define IMX9_SPDIF_BASE                    (0x42680000UL)
#define IMX9_SRC_GENERAL_REG_BASE          (0x44460000UL)
#define IMX9_SRC_SENTINEL_SLICE_BASE       (0x44460400UL)
#define IMX9_SRC_AON_SLICE_BASE            (0x44460800UL)
#define IMX9_SRC_WKUP_SLICE_BASE           (0x44460C00UL)
#define IMX9_SRC_DDR_SLICE_BASE            (0x44461000UL)
#define IMX9_SRC_DPHY_SLICE_BASE           (0x44461400UL)
#define IMX9_SRC_ML_SLICE_BASE             (0x44461800UL)
#define IMX9_SRC_NIC_SLICE_BASE            (0x44461C00UL)
#define IMX9_SRC_HSIO_SLICE_BASE           (0x44462000UL)
#define IMX9_SRC_MEDIA_SLICE_BASE          (0x44462400UL)
#define IMX9_SRC_M33P_SLICE_BASE           (0x44462800UL)
#define IMX9_SRC_A55C0_SLICE_BASE          (0x44462C00UL)
#define IMX9_SRC_A55C1_SLICE_BASE          (0x44463000UL)
#define IMX9_SRC_A55P_SLICE_BASE           (0x44463400UL)
#define IMX9_SRC_MEDIA_MEM_BASE            (0x44465800UL)
#define IMX9_SRC_ML_MEM_BASE               (0x44464800UL)
#define IMX9_M33_PCF1_BASE                 (0x443E0000UL)
#define IMX9_M33_PSF1_BASE                 (0x443F0000UL)
#define IMX9_SYS_CTR_COMPARE_BASE          (0x442A0000UL)
#define IMX9_SYS_CTR_CONTROL_BASE          (0x44290000UL)
#define IMX9_SYS_CTR_READ_BASE             (0x442B0000UL)
#define IMX9_TMU_BASE                      (0x44482000UL)
#define IMX9_TPM1_BASE                     (0x44310000UL)
#define IMX9_TPM2_BASE                     (0x44320000UL)
#define IMX9_TPM3_BASE                     (0x424E0000UL)
#define IMX9_TPM4_BASE                     (0x424F0000UL)
#define IMX9_TPM5_BASE                     (0x42500000UL)
#define IMX9_TPM6_BASE                     (0x42510000UL)
#define IMX9_TRDC1_BASE                    (0x44270000UL)
#define IMX9_TRDC2_BASE                    (0x42460000UL)
#define IMX9_TRGMUX_BASE                   (0x44531000UL)
#define IMX9_TSTMR1_BASE                   (0x442C0000UL)
#define IMX9_TSTMR2_BASE                   (0x42480000UL)
#define IMX9_USB_OTG1_BASE                 (0x4C100000UL)
#define IMX9_USB_OTG2_BASE                 (0x4C200000UL)
#define IMX9_USBNC_OTG1_BASE               (0x4C100200UL)
#define IMX9_USBNC_OTG2_BASE               (0x4C200200UL)
#define IMX9_USDHC1_BASE                   (0x42850000UL)
#define IMX9_USDHC2_BASE                   (0x42860000UL)
#define IMX9_USDHC3_BASE                   (0x428B0000UL)
#define IMX9_WDOG1_BASE                    (0x442D0000UL)
#define IMX9_WDOG2_BASE                    (0x442E0000UL)
#define IMX9_WDOG3_BASE                    (0x42490000UL)
#define IMX9_WDOG4_BASE                    (0x424A0000UL)
#define IMX9_WDOG5_BASE                    (0x424B0000UL)
#define IMX9_LPCAC_PC_BASE                 (0x44400000UL)
#define IMX9_LPCAC_PS_BASE                 (0x44400800UL)

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_MEMORYMAP_H */
