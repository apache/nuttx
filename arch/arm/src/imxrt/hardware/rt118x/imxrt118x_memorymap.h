/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_MEMORYMAP_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* i.MX RT118x memory map (IMXRT1180RM Ch. 3).  Peripheral bases are non-
 * secure aliases; the corresponding secure alias is at NS + 0x1000_0000
 * (e.g. TRDC1 non-secure 0x4427_0000, secure 0x5427_0000).
 *
 * The organisation follows the reference-manual "system memory map" table:
 * memory regions first, then peripherals grouped by mix domain (AONMIX,
 * WAKEUPMIX, MEGAMIX-in-WAKEUPMIX, NETCMIX) in ascending address order.
 */

/* System Memory Map ********************************************************/

#define IMXRT_ITCM_BASE                 (0x00000000ul)  /* M7 ITCM              256 KB */
#define IMXRT_FLEXSPI2_XIP_BASE         (0x04000000ul)  /* FlexSPI2 XIP          64 MB */
#define IMXRT_DTCM_BASE                 (0x20000000ul)  /* M7 DTCM              256 KB */
#define IMXRT_OCRAM1_BASE               (0x20480000ul)  /* OCRAM1               512 KB */
#define IMXRT_OCRAM2_BASE               (0x20500000ul)  /* OCRAM2               256 KB */
#define IMXRT_FLEXSPI1_XIP_BASE         (0x28000000ul)  /* FlexSPI1 XIP         128 MB */

/* System-view aliases (accessible from either core via the AXI backbone) */

#define IMXRT_M7_ITCM_ALIAS_BASE        (0x303c0000ul)  /* M7 ITCM (M33 view)   256 KB */
#define IMXRT_M7_DTCM_ALIAS_BASE        (0x30400000ul)  /* M7 DTCM (M33 view)   256 KB */

/* The first 16 KB of OCRAM1 is reserved by the boot ROM */

#define IMXRT_OCRAM_BASE                (IMXRT_OCRAM1_BASE + 0x4000)
#define IMXRT_OCRAM_SIZE                (512 * 1024 - 0x4000)

/* WAKEUPMIX Peripheral Domain (0x4200_0000 - 0x42FF_FFFF) ******************/

#define IMXRT_DMA4_BASE                 (0x42000000ul)  /* eDMA4 (channels + TCDs) */

#define IMXRT_BLK_CTRL_WAKEUPMIX_BASE   (0x42420000ul)  /* Wakeup block control */
#define IMXRT_MU2_MUB_BASE              (0x42440000ul)  /* Message Unit 2 side B */
#define IMXRT_SEMA2_BASE                (0x42450000ul)  /* Semaphore 2 */
#define IMXRT_TRDC2_BASE                (0x42460000ul)  /* TRDC2 (Wakeup/Mega) */

#define IMXRT_TSTMR2_BASE               (0x42480000ul)  /* Time Stamp Timer 2 */
#define IMXRT_RTWDOG3_BASE              (0x42490000ul)  /* WDog 3 */
#define IMXRT_RTWDOG4_BASE              (0x424a0000ul)  /* WDog 4 */
#define IMXRT_RTWDOG5_BASE              (0x424b0000ul)  /* WDog 5 */
#define IMXRT_LPIT2_BASE                (0x424c0000ul)  /* Periodic Interrupt Timer 2 */
#define IMXRT_LPTMR2_BASE               (0x424d0000ul)  /* Low Power Timer 2 */
#define IMXRT_TPM3_BASE                 (0x424e0000ul)  /* Timer PWM Module 3 */
#define IMXRT_TPM4_BASE                 (0x424f0000ul)  /* Timer PWM Module 4 */
#define IMXRT_TPM5_BASE                 (0x42500000ul)  /* Timer PWM Module 5 */
#define IMXRT_TPM6_BASE                 (0x42510000ul)  /* Timer PWM Module 6 */
#define IMXRT_I3C2_BASE                 (0x42520000ul)  /* I3C 2 */
#define IMXRT_LPI2C3_BASE               (0x42530000ul)
#define IMXRT_LPI2C4_BASE               (0x42540000ul)
#define IMXRT_LPSPI3_BASE               (0x42550000ul)
#define IMXRT_LPSPI4_BASE               (0x42560000ul)
#define IMXRT_LPUART3_BASE              (0x42570000ul)
#define IMXRT_LPUART4_BASE              (0x42580000ul)
#define IMXRT_LPUART5_BASE              (0x42590000ul)
#define IMXRT_LPUART6_BASE              (0x425a0000ul)
#define IMXRT_CAN2_BASE                 (0x425b0000ul)  /* FlexCAN 2 */
#define IMXRT_FLEXIO1_BASE              (0x425c0000ul)
#define IMXRT_FLEXIO2_BASE              (0x425d0000ul)
#define IMXRT_FLEXSPIC_BASE             (0x425e0000ul)  /* FlexSPI1 controller */
#define IMXRT_OTFAD1_BASE               (0x425e0000ul)  /* On-The-Fly AES Decrypt 1 */

#define IMXRT_ADC1_BASE                 (0x42600000ul)  /* LPADC 1 */
#define IMXRT_FLEXPWM1_BASE             (0x42650000ul)  /* FlexPWM 1 */
#define IMXRT_FLEXPWM2_BASE             (0x42660000ul)  /* FlexPWM 2 */
#define IMXRT_FLEXPWM3_BASE             (0x42670000ul)  /* FlexPWM 3 */
#define IMXRT_FLEXPWM4_BASE             (0x42680000ul)  /* FlexPWM 4 */
#define IMXRT_TMR1_BASE                 (0x42690000ul)  /* Quad Timer 1 */
#define IMXRT_TMR2_BASE                 (0x426a0000ul)
#define IMXRT_TMR3_BASE                 (0x426b0000ul)
#define IMXRT_TMR4_BASE                 (0x426c0000ul)
#define IMXRT_TMR5_BASE                 (0x426d0000ul)
#define IMXRT_TMR6_BASE                 (0x426e0000ul)
#define IMXRT_TMR7_BASE                 (0x426f0000ul)
#define IMXRT_TMR8_BASE                 (0x42700000ul)
#define IMXRT_ENC1_BASE                 (0x42710000ul)  /* Enhanced Quad Decoder 1 (eQDC) */
#define IMXRT_ENC2_BASE                 (0x42720000ul)
#define IMXRT_ENC3_BASE                 (0x42730000ul)
#define IMXRT_ENC4_BASE                 (0x42740000ul)
#define IMXRT_XBAR1_BASE                (0x42750000ul)  /* Inter-Peripheral Crossbar 1 */
#define IMXRT_XBAR2_BASE                (0x42760000ul)
#define IMXRT_XBAR3_BASE                (0x42770000ul)
#define IMXRT_AOI1_BASE                 (0x42780000ul)  /* AND/OR Invert 1 */
#define IMXRT_AOI2_BASE                 (0x42790000ul)
#define IMXRT_EWM_BASE                  (0x427b0000ul)  /* External Watchdog Monitor */
#define IMXRT_AOI3_BASE                 (0x427e0000ul)
#define IMXRT_AOI4_BASE                 (0x427f0000ul)

#define IMXRT_TRDC3_BASE                (0x42810000ul)  /* TRDC3 (NIX/Media) */
#define IMXRT_USDHC1_BASE               (0x42850000ul)
#define IMXRT_USDHC2_BASE               (0x42860000ul)

#define IMXRT_MSGINTR1_BASE             (0x428a0000ul)  /* Message-signalled INTR 1 */
#define IMXRT_MSGINTR2_BASE             (0x428b0000ul)
#define IMXRT_MSGINTR3_BASE             (0x428c0000ul)
#define IMXRT_MSGINTR4_BASE             (0x428d0000ul)
#define IMXRT_MSGINTR5_BASE             (0x428e0000ul)
#define IMXRT_MSGINTR6_BASE             (0x428f0000ul)

#define IMXRT_FLEXSPI_SLV_BASE          (0x42900000ul)  /* FlexSPI slave */
#define IMXRT_SEMC_BASE                 (0x42910000ul)  /* Smart External Mem Ctlr */
#define IMXRT_SEMC0_BASE                (0x80000000ul)  /* SEMC external memory window */
#define IMXRT_MECC1_BASE                (0x42920000ul)  /* Memory ECC 1 */
#define IMXRT_MECC2_BASE                (0x42930000ul)
#define IMXRT_ASRC_BASE                 (0x429a0000ul)  /* Async Sample Rate Conv */

#define IMXRT_KPP_BASE                  (0x42a00000ul)  /* Keypad Port */
#define IMXRT_IOMUXC_WKUP_BASE          (0x42a10000ul)  /* IOMUX (wakeup pads) */
#define IMXRT_ECAT_BASE                 (0x42a80000ul)  /* EtherCAT slave controller */

#define IMXRT_SPDIF_BASE                (0x42ba0000ul)  /* SPDIF Tx/Rx */
#define IMXRT_SAI2_BASE                 (0x42bb0000ul)  /* Sync Audio Interface 2 */
#define IMXRT_SAI3_BASE                 (0x42bc0000ul)
#define IMXRT_SAI4_BASE                 (0x42bd0000ul)
#define IMXRT_PDM_BASE                  (0x42be0000ul)  /* Pulse Density Modulation */
#define IMXRT_SINC1_BASE                (0x42bf0000ul)  /* Sinc Filter 1 */
#define IMXRT_SINC2_BASE                (0x42c00000ul)
#define IMXRT_SINC3_BASE                (0x42c10000ul)

/* IMXRT_USB_BASE is the generic name used by the shared USB device driver
 * (arch/arm/src/imxrt/imxrt_usbdev.c) and hardware/imxrt_usbotg.h.  On
 * RT118x this is USB OTG1 (the only USB device-capable controller).
 */

#define IMXRT_USB_BASE                  (0x42c80000ul)  /* USB OTG1 controller */
#define IMXRT_USBNC_OTG1_BASE           (0x42c80200ul)  /* USB OTG1 non-core */
#define IMXRT_USB_OTG2_BASE             (0x42c90000ul)
#define IMXRT_USBNC_OTG2_BASE           (0x42c90200ul)

#define IMXRT_USBPHY1_BASE              (0x42ca0000ul)  /* USB PHY 1 */
#define IMXRT_USBHSDCD1_BASE            (0x42ca0800ul)  /* USB HS charger detect 1 */
#define IMXRT_USBPHY2_BASE              (0x42cb0000ul)
#define IMXRT_USBHSDCD2_BASE            (0x42cb0800ul)

#define IMXRT_LPIT3_BASE                (0x42cc0000ul)
#define IMXRT_LPTMR3_BASE               (0x42cd0000ul)

#define IMXRT_LPI2C5_BASE               (0x42d30000ul)
#define IMXRT_LPI2C6_BASE               (0x42d40000ul)
#define IMXRT_LPSPI5_BASE               (0x42d50000ul)
#define IMXRT_LPSPI6_BASE               (0x42d60000ul)
#define IMXRT_LPUART9_BASE              (0x42d70000ul)
#define IMXRT_LPUART10_BASE             (0x42d80000ul)
#define IMXRT_LPUART11_BASE             (0x42d90000ul)
#define IMXRT_LPUART8_BASE              (0x42da0000ul)  /* Note: LPUART8 base > 9..11 */
#define IMXRT_CMP1_BASE                 (0x42dc0000ul)  /* Analog Comparator 1 */
#define IMXRT_CMP2_BASE                 (0x42dd0000ul)
#define IMXRT_CMP3_BASE                 (0x42de0000ul)

/* WAKEUPMIX GPIO ports (RGPIO2..RGPIO6) */

#define IMXRT_GPIO2_BASE                (0x43810000ul)
#define IMXRT_GPIO3_BASE                (0x43820000ul)
#define IMXRT_GPIO4_BASE                (0x43830000ul)
#define IMXRT_GPIO5_BASE                (0x43840000ul)
#define IMXRT_GPIO6_BASE                (0x43850000ul)

/* AONMIX Peripheral Domain (0x4420_0000 - 0x447F_FFFF) *********************/

#define IMXRT_DMA3_BASE                 (0x44000000ul)  /* eDMA3 (AON) */

#define IMXRT_BLK_CTRL_NS_AON_BASE      (0x44210000ul)  /* AON non-secure block control */

#define IMXRT_LPUART1_BASE              (0x44380000ul)
#define IMXRT_LPUART2_BASE              (0x44390000ul)

#define IMXRT_IOMUXC_AON_BASE           (0x443c0000ul)  /* IOMUX (AON pads) */

#define IMXRT_CCM_BASE                  (0x44450000ul)  /* Clock Controller Module */
#define IMXRT_SRC_BASE                  (0x44460000ul)  /* System Reset Controller */

/* SRC power-slice sub-blocks (per mix, non-secure aliases).  IMXRT1180RM
 * Ch. 27.  Each slice is 0x400 bytes.
 */

#define IMXRT_AON_MIX_SLICE_BASE        (0x44460800ul)
#define IMXRT_WAKEUP_MIX_SLICE_BASE     (0x44460c00ul)
#define IMXRT_MEGA_MIX_SLICE_BASE       (0x44461000ul)
#define IMXRT_NETC_MIX_SLICE_BASE       (0x44461400ul)
#define IMXRT_CM33PLATFORM_MIX_SLICE    (0x44461800ul)
#define IMXRT_CM7PLATFORM_MIX_SLICE     (0x44461c00ul)

#define IMXRT_GPC_CPU_CTRL_BASE         (0x44470000ul)  /* GPC CPU control */
#define IMXRT_GPC_SM_BASE               (0x44471000ul)  /* GPC state machine */

/* AONMIX ANADIG (analog / PLL / OSC).  See imxrt118x_anadig.h for the
 * per-register offsets and bit fields.
 */

#define IMXRT_ANADIG_PLL_BASE           (0x44480000ul)  /* Analog PLL registers */
#define IMXRT_ANADIG_OSC_BASE           (0x44481000ul)  /* Analog OSC registers */
#define IMXRT_ANADIG_TEMPSENSOR_BASE    (0x44482000ul)
#define IMXRT_ANADIG_MISC_BASE          (0x44483000ul)
#define IMXRT_ANADIG_LDO_BASE           (0x44484000ul)  /* Analog PMU LDOs incl. PHY_LDO */
#define IMXRT_DCDC_BASE                 (0x44520000ul)  /* On-chip DCDC */

/* AONMIX BLK_CTRL (block-control fabric) - secure alias.  See
 * IMXRT_BLK_CTRL_NS_AON_BASE above for the non-secure alias.
 */

#define IMXRT_BLK_CTRL_S_AONMIX_BASE    (0x444f0000ul)  /* AON secure block control */

/* AONMIX common serial + timers */

#define IMXRT_TSTMR1_BASE               (0x442c0000ul)  /* Time Stamp Timer 1 */
#define IMXRT_RTWDOG1_BASE              (0x442d0000ul)  /* WDog 1 */
#define IMXRT_RTWDOG2_BASE              (0x442e0000ul)
#define IMXRT_LPIT1_BASE                (0x442f0000ul)
#define IMXRT_LPTMR1_BASE               (0x44300000ul)
#define IMXRT_TPM1_BASE                 (0x44310000ul)
#define IMXRT_TPM2_BASE                 (0x44320000ul)
#define IMXRT_I3C1_BASE                 (0x44330000ul)
#define IMXRT_LPI2C1_BASE               (0x44340000ul)
#define IMXRT_LPI2C2_BASE               (0x44350000ul)
#define IMXRT_LPSPI1_BASE               (0x44360000ul)
#define IMXRT_LPSPI2_BASE               (0x44370000ul)
#define IMXRT_CAN1_BASE                 (0x443a0000ul)
#define IMXRT_LPUART7_BASE              (0x44570000ul)
#define IMXRT_LPUART12_BASE             (0x44580000ul)
#define IMXRT_CAN3_BASE                 (0x445b0000ul)
#define IMXRT_FLEXSPI2C_BASE            (0x445e0000ul)  /* FlexSPI2 controller */
#define IMXRT_OTFAD2_BASE               (0x445e0000ul)  /* On-The-Fly AES Decrypt 2 */

/* AONMIX secure services (S3MU, ELE, OTP, RGPIO1) */

#define IMXRT_GPIO1_BASE                (0x47400000ul)  /* RGPIO1 (AON) */
#define IMXRT_MU_APPS_S3MUA_BASE        (0x47520000ul)  /* Applications MU to ELE (NS alias) */
#define IMXRT_S3MUA_BASE                (0x57540000ul)  /* System 3 MU (ELE), secure alias */
#define IMXRT_S3MUA_NS_BASE             (0x47540000ul)  /* System 3 MU (ELE), non-secure alias */

/* TRDC1 (AON) instance */

#define IMXRT_TRDC1_BASE                (0x44270000ul)  /* TRDC1 (AON) */

/* NETCMIX Domain (0x6000_0000 - 0x60FF_FFFF) *******************************/

#define IMXRT_ENETC0_SI0_BASE           (0x60b00000ul)  /* ENETC0 station interface 0 */
#define IMXRT_ENETC0_BASE               (0x60b10000ul)  /* ENETC0 common regs */
#define IMXRT_ENETC0_PORT_BASE          (0x60b14000ul)  /* ENETC0 port regs */
#define IMXRT_ENETC0_ETH_MAC_BASE       (0x60b15000ul)  /* ENETC0 MAC regs */
#define IMXRT_ENETC0_GLOBAL_BASE        (0x60b20000ul)
#define IMXRT_ENETC1_SI0_BASE           (0x60b40000ul)
#define IMXRT_ENETC1_BASE               (0x60b50000ul)
#define IMXRT_ENETC1_PORT_BASE          (0x60b54000ul)
#define IMXRT_ENETC1_PSEUDO_MAC_BASE    (0x60b55000ul)
#define IMXRT_ENETC1_GLOBAL_BASE        (0x60b60000ul)
#define IMXRT_ENETC1_SI1_BASE           (0x60c10000ul)
#define IMXRT_EMDIO_BASE                (0x60ba0000ul)  /* External MDIO */
#define IMXRT_TMR0_BASE                 (0x60b80000ul)  /* NETC 1588 Timer */
#define IMXRT_SW0_GLOBAL_BASE           (0x60a80000ul)  /* Switch 0 global */

/* CM7 Private Peripheral Bus (0xE000_0000 - 0xE00F_FFFF) *******************/

#define IMXRT_M7_MCM_BASE               (0xe0080000ul)  /* CM7 memory control module */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_MEMORYMAP_H */
