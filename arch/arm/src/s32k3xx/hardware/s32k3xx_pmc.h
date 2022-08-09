/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_pmc.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PMC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMC Register Offsets *****************************************************/

#define S32K3XX_PMC_LVSC_OFFSET   (0x00) /* Low Voltage Status and Control Register (LVSC) */
#define S32K3XX_PMC_CONFIG_OFFSET (0x04) /* PMC Configuration Register (CONFIG) */
#define S32K3XX_PMC_VERID_OFFSET  (0x0c) /* Version ID Register (VERID) */

/* PMC Register Addresses ***************************************************/

#define S32K3XX_PMC_LVSC          (S32K3XX_PMC_BASE + S32K3XX_PMC_LVSC_OFFSET)
#define S32K3XX_PMC_CONFIG        (S32K3XX_PMC_BASE + S32K3XX_PMC_CONFIG_OFFSET)
#define S32K3XX_PMC_VERID         (S32K3XX_PMC_BASE + S32K3XX_PMC_VERID_OFFSET)

/* PMC Register Bitfield Definitions ****************************************/

/* Low Voltage Status and Control Register (LVSC) */

#define PMC_LVSC_HVDAF            (1 << 0)  /* Bit 0: HVDA flag on VDD_HV_A domain in FPM (HVDAF) */
#define PMC_LVSC_HVDBF            (1 << 1)  /* Bit 1: HVDB flag on VDD_HV_B domain in FPM (HVDBF) */
#define PMC_LVSC_HVD25F           (1 << 2)  /* Bit 2: HVD25 flag on V25 domain in FPM (HVD25F) */
#define PMC_LVSC_HVD11F           (1 << 3)  /* Bit 3: HVD11 flag on V11 domain in FPM (HVD11F) */
#define PMC_LVSC_LVD5AF           (1 << 4)  /* Bit 4: LVD5A flag on VDD_HV_A domain in FPM (LVD5AF) */
#define PMC_LVSC_LVD15F           (1 << 5)  /* Bit 5: LVD15 flag on V15 domain in FPM (LVD15F) */
                                            /* Bits 6-7: Reserved */
#define PMC_LVSC_HVDAS            (1 << 8)  /* Bit 8: HVDA status on VDD_HV_A domain in FPM (HVDAS) */
#define PMC_LVSC_HVDBS            (1 << 9)  /* Bit 9: HVDB status on VDD_HV_B domain in FPM (HVDBS) */
#define PMC_LVSC_HVD25S           (1 << 10) /* Bit 10: HVD25 status on V25 domain in FPM (HVD25S) */
#define PMC_LVSC_HVD11S           (1 << 11) /* Bit 11: HVD11 status on V11 domain in FPM (HVD11S) */
#define PMC_LVSC_LVD5AS           (1 << 12) /* Bit 12: LVD5A status on VDD_HV_A domain in FPM (LVD5AS) */
#define PMC_LVSC_LVD15S           (1 << 13) /* Bit 13: LVD15 status on V15 domain in FPM (LVD15S) */
                                            /* Bits 14-15: Reserved */
#define PMC_LVSC_LVRAF            (1 << 16) /* Bit 16: LVRA flag on VDD_HV_A domain in FPM (LVRAF) */
#define PMC_LVSC_LVRALPF          (1 << 17) /* Bit 17: LVRALP flag on VDD_HV_A domain (LVRALPF) */
#define PMC_LVSC_LVRBF            (1 << 18) /* Bit 18: LVRB flag on VDD_HV_B domain in FPM (LVRBF) */
#define PMC_LVSC_LVRBLPF          (1 << 19) /* Bit 19: LVRBLP flag on VDD_HV_B domain (LVRBLPF) */
#define PMC_LVSC_LVR25F           (1 << 20) /* Bit 20: LVR25 flag on V25 domain in FPM (LVR25F) */
#define PMC_LVSC_LVR25LPF         (1 << 21) /* Bit 21: LVR25LP flag on V25 domain (LVR25LPF) */
#define PMC_LVSC_LVR11F           (1 << 22) /* Bit 22: LVR11 flag on V11 domain in FPM (LVR11F) */
#define PMC_LVSC_LVR11LPF         (1 << 23) /* Bit 23: LVR11LP flag on V11 domain (LVR11LPF) */
#define PMC_LVSC_GNG25OSCF        (1 << 24) /* Bit 24: GO/NoGo detect flag on Osc part of V25 domain (GNG25OSCF) */
#define PMC_LVSC_GNG11OSCF        (1 << 25) /* Bit 25: GO/NoGo detect flag on Osc part of V11 domain (GNG11OSCF) */
                                            /* Bits 26-30: Reserved */
#define PMC_LVSC_PORF             (1 << 31) /* Bit 31: POR flag (PORF) */

/* PMC Configuration Register (CONFIG) */

#define PMC_CONFIG_LMEN           (1 << 0)  /* Bit 0: Last Mile regulator enable bit (LMEN) */
#define PMC_CONFIG_LMBCTLEN       (1 << 1)  /* Bit 1: Last Mile regulator base control enable bit (LMBCTLEN) */
#define PMC_CONFIG_FASTREC        (1 << 2)  /* Bit 2: Fast recovery from LPM enable bit (FASTREC) */
#define PMC_CONFIG_LPM25EN        (1 << 3)  /* Bit 3: V25 domain enable bit during LPM (LPM25EN) */
#define PMC_CONFIG_LVRBLPEN       (1 << 4)  /* Bit 4: LVRBLP enable bit during LPM (LVRBLPEN) */
                                            /* Bits 5-7: Reserved */
#define PMC_CONFIG_HVDIE          (1 << 8)  /* Bit 8: High voltage detect interrupt enable (HVDIE) */
#define PMC_CONFIG_LVDIE          (1 << 9)  /* Bit 9: Low voltage detect interrupt enable (LVDIE) */
                                            /* Bit 10-15: Reserved */
#define PMC_CONFIG_LMAUTOEN       (1 << 16) /* Bit 16: Last Mile regulator auto turn over bit (LMAUTOEN) */
#define PMC_CONFIG_LMSTAT         (1 << 17) /* Bit 17: Last Mile regulator status bit (LMSTAT) */
                                            /* Bits 18-31: Reserved */

/* Version ID Register (VERID) */

#define PMC_VERID_LMFEAT          (1 << 0)  /* Bit 0: Last Mile Regulator Feature (LMFEAT) */
                                            /* Bits 1-15: Reserved */
#define PMC_VERID_MINOR_SHIFT     (16)      /* Bits 16-23: Minor version number (MINOR) */
#define PMC_VERID_MINOR_MASK      (0xff << PMC_VERID_MINOR_SHIFT)
#define PMC_VERID_MAJOR_SHIFT     (24)      /* Bits 14-31: Major version number (MAJOR) */
#define PMC_VERID_MAJOR_MASK      (0xff << PMC_VERID_MAJOR_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PMC_H */
