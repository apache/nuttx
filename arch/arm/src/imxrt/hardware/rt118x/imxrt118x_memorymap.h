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

/* Cortex-M33 memory view */

#define IMXRT_ITCM_BASE             0x1ffe0000u
#define IMXRT_ITCM_SIZE             (128u * 1024u)
#define IMXRT_DTCM_BASE             0x30000000u
#define IMXRT_DTCM_SIZE             (128u * 1024u)
#define IMXRT_OCRAM_BASE            0x30480000u
#define IMXRT_OCRAM2_BASE           IMXRT_OCRAM_BASE
/* Non-secure FlexSPI AHB windows. */

#define IMXRT_FLEXSPI1_BASE         0x28000000u
#define IMXRT_FLEXSPI2_BASE         0x04000000u

/* Non-secure peripheral aliases used by this initial NuttX port. */

#define IMXRT_LPUART1_BASE          0x44380000u
#define IMXRT_IOMUXC_AON_BASE       0x443c0000u
#define IMXRT_IOMUXC_BASE           0x42a10000u
#define IMXRT_GPC_CPU_CTRL_BASE      0x44470000u
#define IMXRT_GPC_GLOBAL_BASE        0x44472000u
#define IMXRT_GPC_SYS_SLEEP_BASE     0x44472800u
#define IMXRT_ANADIG_BASE            0x44480000u
#define IMXRT_ANADIG_PLL_BASE        IMXRT_ANADIG_BASE
#define IMXRT_ANADIG_OSC_BASE        IMXRT_ANADIG_BASE
#define IMXRT_ANADIG_PMU_BASE        IMXRT_ANADIG_BASE
#define IMXRT_ETHERNET_PLL_BASE      0x44484180u
#define IMXRT_AUDIO_PLL_BASE         0x44484280u
#define IMXRT_OSC_RC_400M_BASE       0x44484380u
#define IMXRT_PHY_LDO_BASE           0x44484680u
#define IMXRT_CCM_BASE              0x44450000u
#define IMXRT_GPIO1_BASE            0x47400000u
#define IMXRT_GPT1_BASE             0x446c0000u
#define IMXRT_LPIT1_BASE            0x442f0000u
#define IMXRT_WDOG1_BASE            0x442d0000u
#define IMXRT_OCOTP_BASE            0x47510000u
#define IMXRT_XBAR1_BASE            0x47520000u
#define IMXRT_MU_RT_S3MUA_BASE      0x57540000u

/* USB OTG1 / USBNC / USBPHY1 non-secure bases from MIMXRT1186 CMSIS. */

#define IMXRT_USB_BASE              0x42c80000u
#define IMXRT_USBOTG1_BASE          IMXRT_USB_BASE
#define IMXRT_USBNC_OTG1_BASE       0x42c80200u
#define IMXRT_USBPHY1_BASE          0x42ca0000u

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_MEMORYMAP_H */
