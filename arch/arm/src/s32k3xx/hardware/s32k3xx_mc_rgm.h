/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_mc_rgm.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_RGM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_RGM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MC_RGM Register Offsets **************************************************/

#define S32K3XX_MC_RGM_DES_OFFSET     (0x00) /* Destructive Event Status Register (DES) */
#define S32K3XX_MC_RGM_FES_OFFSET     (0x08) /* Functional/External Reset Status Register (FES) */
#define S32K3XX_MC_RGM_FERD_OFFSET    (0x0c) /* Functional Event Reset Disable Register (FERD) */
#define S32K3XX_MC_RGM_FBRE_OFFSET    (0x10) /* Functional Bidirectional Reset Enable Register (FBRE) */
#define S32K3XX_MC_RGM_FREC_OFFSET    (0x14) /* Functional Reset Escalation Counter Register (FREC) */
#define S32K3XX_MC_RGM_FRET_OFFSET    (0x18) /* Functional Reset Escalation Threshold Register (FRET) */
#define S32K3XX_MC_RGM_DRET_OFFSET    (0x1c) /* Destructive Reset Escalation Threshold Register (DRET) */
#define S32K3XX_MC_RGM_ERCTRL_OFFSET  (0x20) /* External Reset Control Register (ERCTRL) */
#define S32K3XX_MC_RGM_RDSS_OFFSET    (0x24) /* Reset During Standby Status Register (RDSS) */
#define S32K3XX_MC_RGM_FRENTC_OFFSET  (0x28) /* Functional Reset Entry Timeout Control Register (FRENTC) */
#define S32K3XX_MC_RGM_LPDEBUG_OFFSET (0x2c) /* Low Power Debug Control Register (LPDEBUG) */

/* MC_RGM Register Addresses ************************************************/

#define S32K3XX_MC_RGM_DES            (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_DES_OFFSET)
#define S32K3XX_MC_RGM_FES            (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_FES_OFFSET)
#define S32K3XX_MC_RGM_FERD           (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_FERD_OFFSET)
#define S32K3XX_MC_RGM_FBRE           (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_FBRE_OFFSET)
#define S32K3XX_MC_RGM_FREC           (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_FREC_OFFSET)
#define S32K3XX_MC_RGM_FRET           (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_FRET_OFFSET)
#define S32K3XX_MC_RGM_DRET           (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_DRET_OFFSET)
#define S32K3XX_MC_RGM_ERCTRL         (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_ERCTRL_OFFSET)
#define S32K3XX_MC_RGM_RDSS           (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_RDSS_OFFSET)
#define S32K3XX_MC_RGM_FRENTC         (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_FRENTC_OFFSET)
#define S32K3XX_MC_RGM_LPDEBUG        (S32K3XX_MC_RGM_BASE + S32K3XX_MC_RGM_LPDEBUG_OFFSET)

/* MC_RGM Register Bitfield Definitions *************************************/

/* Destructive Event Status Register (DES) */

#define MC_RGM_DES_F_POR                 (1 << 0)  /* Bit 0: Flag for power-on reset (F_POR) */
                                                   /* Bits 1-2: Reserved */
#define MC_RGM_DES_FCCU_FTR              (1 << 3)  /* Bit 3: Flag for 'Destructive' Reset FCCU_FTR */
#define MC_RGM_DES_STCU_URF              (1 << 4)  /* Bit 4: Flag for 'Destructive' Reset STCU_URF */
                                                   /* Bit 5: Reserved */
#define MC_RGM_DES_MC_RGM_FRE            (1 << 6)  /* Bit 6: Flag for 'Destructive' Reset MC_RGM_FRE */
                                                   /* Bit 7: Reserved */
#define MC_RGM_DES_FXOSC_FAIL            (1 << 8)  /* Bit 8: Flag for 'Destructive' Reset FXOSC_FAIL */
#define MC_RGM_DES_PLL_LOL               (1 << 9)  /* Bit 9: Flag for 'Destructive' Reset PLL_LOL */
#define MC_RGM_DES_CORE_CLK_FAIL         (1 << 10) /* Bit 10: Flag for 'Destructive' Reset CORE_CLK_FAIL */
                                                   /* Bit 11: Reserved */
#define MC_RGM_DES_AIPS_PLAT_CLK_FAIL    (1 << 12) /* Bit 12: Flag for 'Destructive' Reset AIPS_PLAT_CLK_FAIL */
                                                   /* Bit 13: Reserved */
#define MC_RGM_DES_HSE_CLK_FAIL          (1 << 14) /* Bit 14: Flag for 'Destructive' Reset HSE_CLK_FAIL */
#define MC_RGM_DES_SYS_DIV_FAIL          (1 << 15) /* Bit 15: Flag for 'Destructive' Reset SYS_DIV_FAIL */
                                                   /* Bit 16: Reserved */
#define MC_RGM_DES_HSE_TMPR_RST          (1 << 17) /* Bit 17: Flag for 'Destructive' Reset HSE_TMPR_RST */
#define MC_RGM_DES_HSE_SNVS_RST          (1 << 18) /* Bit 18: Flag for 'Destructive' Reset HSE_SNVS_RST */
                                                   /* Bits 19-28: Reserved */
#define MC_RGM_DES_SW_DEST               (1 << 29) /* Bit 29: Flag for 'Destructive' Reset SW_DEST */
#define MC_RGM_DES_DEBUG_DEST            (1 << 30) /* Bit 30: Flag for 'Destructive' Reset DEBUG_DEST */
                                                   /* Bit 31: Reserved */

/* Functional/External Reset Status Register (FES) */

#define MC_RGM_FES_F_EXR                 (1 << 0)  /* Bit 0: Flag for External Rest (F_EXR) */
                                                   /* Bits 1-2: Reserved */
#define MC_RGM_FES_FCCU_RST              (1 << 3)  /* Bit 3: Flag for 'Functional' Reset FCCU_RST */
#define MC_RGM_FES_ST_DONE               (1 << 4)  /* Bit 4: Flag for 'Functional' Reset ST_DONE */
                                                   /* Bit 5: Reserved */
#define MC_RGM_FES_SWT0_RST              (1 << 6)  /* Bit 6: Flag for 'Functional' Reset SWT0_RST */
#define MC_RGM_FES_SWT1_RST              (1 << 7)  /* Bit 7: Flag for 'Functional' Reset SWT1_RST */
                                                   /* Bit 8: Reserved */
#define MC_RGM_FES_JTAG_RST              (1 << 9)  /* Bit 9: Flag for 'Functional' Reset JTAG_RST */
                                                   /* Bits 10-15: Reserved */
#define MC_RGM_FES_HSE_SWT_RST           (1 << 16) /* Bit 16: Flag for 'Functional' Reset HSE_SWT_RST */
                                                   /* Bits 17-19: Reserved */
#define MC_RGM_FES_HSE_BOOT_RST          (1 << 20) /* Bit 20: Flag for 'Functional' Reset HSE_BOOT_RST */
                                                   /* Bits 21-28: Reserved */
#define MC_RGM_FES_SW_FUNC               (1 << 29) /* Bit 29: Flag for 'Functional' Reset SW_FUNC */
#define MC_RGM_FES_DEBUG_FUNC            (1 << 30) /* Bit 30: Flag for 'Functional' Reset DEBUG_FUNC */
                                                   /* Bit 31: Reserved */

/* Functional Event Reset Disable Register (FERD) */

                                                   /* Bits 0-2: Reserved */
#define MC_RGM_FERD_D_FCCU_RST           (1 << 3)  /* Bit 3: FCCU_RST Disable Control (D_FCCU_RST) */
                                                   /* Bits 4-5: Reserved */
#define MC_RGM_FERD_D_SWT0_RST           (1 << 6)  /* Bit 6: SWT0_RST Disable Control (D_SWT0_RST) */
#define MC_RGM_FERD_D_SWT1_RST           (1 << 7)  /* Bit 7: SWT1_RST Disable Control (D_SWT1_RST) */
                                                   /* Bit 8: Reserved */
#define MC_RGM_FERD_D_JTAG_RST           (1 << 9)  /* Bit 9: JTAG_RST Disable Control (D_JTAG_RST) */
                                                   /* Bits 10-29: Reserved */
#define MC_RGM_FERD_D_DEBUG_FUNC         (1 << 30) /* Bit 30: DEBUG_FUNC Disable Control (D_DEBUG_FUNC) */
                                                   /* Bit 31: Reserved */

/* Functional Bidirectional Reset Enable Register (FBRE) */

                                                /* Bits 0-2: Reserved */
#define MC_RGM_FBRE_BE_FCCU_RST          (1 << 3)  /* Bit 3: Bidirectional Reset Enable for 'Functional' Reset FCCU_RST (BE_FCCU_RST) */
#define MC_RGM_FBRE_BE_ST_DONE           (1 << 4)  /* Bit 4: Bidirectional Reset Enable for 'Functional' Reset ST_DONE (BE_ST_DONE) */
                                                   /* Bit 5: Reserved */
#define MC_RGM_FBRE_BE_SWT0_RST          (1 << 6)  /* Bit 6: Bidirectional Reset Enable for 'Functional' Reset SWT0_RST (BE_SWT0_RST) */
#define MC_RGM_FBRE_BE_SWT1_RST          (1 << 7)  /* Bit 7: Bidirectional Reset Enable for 'Functional' Reset SWT1_RST (BE_SWT1_RST) */
                                                   /* Bit 8: Reserved */
#define MC_RGM_FBRE_BE_JTAG_RST          (1 << 9)  /* Bit 9: Bidirectional Reset Enable for 'Functional' Reset JTAG_RST (BE_JTAG_RST) */
                                                   /* Bits 10-15: Reserved */
#define MC_RGM_FBRE_BE_HSE_SWT_RST       (1 << 16) /* Bit 16: Bidirectional Reset Enable for 'Functional' Reset HSE_SWT_RST (BE_HSE_SWT_RST) */
                                                   /* Bits 17-19: Reserved */
#define MC_RGM_FBRE_BE_HSE_BOOT_RST      (1 << 20) /* Bit 20: Bidirectional Rest Enable for 'Functional' Reset HSE_BOOT_RST (BE_HSE_BOOT_RST) */
                                                   /* Bits 21-28: Reserved */
#define MC_RGM_FBRE_BE_SW_FUNC           (1 << 29) /* Bit 29: Bidirectional Rest Enable for 'Functional' Reset SW_FUNC (BE_SW_FUNC) */
#define MC_RGM_FBRE_BE_DEBUG_FUNC        (1 << 30) /* Bit 30: Bidirectional Rest Enable for 'Functional' Reset DEBUG_FUNC (BE_DEBUG_FUNC) */
                                                   /* Bit 31: Reserved */

/* Functional Reset Escalation Counter Register (FREC) */

#define MC_RGM_FREC_SHIFT                (0)       /* Bits 0-3: 'Functional' Reset Escalation Counter (FREC) */
#define MC_RGM_FREC_MASK                 (0x0f << MC_RGM_FREC_SHIFT)
                                                   /* Bits 4-31: Reserved */

/* Functional Reset Escalation Threshold Register (FRET) */

#define MC_RGM_FRET_SHIFT                (0)       /* Bits 0-3: 'Functional' Reset Escalation Threshold (FRET) */
#define MC_RGM_FRET_MASK                 (0x0f << MC_RGM_FRET_SHIFT)
                                                   /* Bits 4-31: Reserved */

/* Destructive Reset Escalation Threshold Register (DRET) */

#define MC_RGM_DRET_SHIFT                (0)       /* Bits 0-3: 'Destructive' Reset Escalation Threshold (DRET) */
#define MC_RGM_DRET_MASK                 (0x0f << MC_RGM_DRET_SHIFT)
                                                   /* Bits 4-31: Reserved */

/* External Reset Control Register (ERCTRL) */

#define MC_RGM_ERCTRL_ERASSERT           (1 << 0)  /* Bit 0: External reset is asserted (ERASSERT) */
                                                   /* Bits 1-31: Reserved */

/* Reset During Standby Status Register (RDSS) */

#define MC_RGM_RDSS_DES_RES              (1 << 0)  /* Bit 0: Destructive reset event occurred during standby mode (DES_RES) */
#define MC_RGM_RDSS_FES_RES              (1 << 1)  /* Bit 1: Functional reset event occurred during standby mode (FES_RES) */
                                                   /* Bits 2-31: Reserved */

/* Functional Reset Entry Timeout Control Register (FRENTC) */

#define MC_RGM_FRENTC_FRET_EN            (1 << 0)  /* Bit 0: Functional Reset Entry Timer Enable (FRET_EN) */
#define MC_RGM_FRENTC_FRET_TIMEOUT_SHIFT (1)       /* Bits 1-31: Functional Reset Entry Timer Value (FRET_TIMEOUT) */
#define MC_RGM_FRENTC_FRET_TIMEOUT_MASK  (0x7fffffff << MC_RGM_FRENTC_FRET_TIMEOUT_SHIFT)

/* Low Power Debug Control Register (LPDEBUG) */

#define MC_RGM_LPDEBUG_LP_DBG_EN         (1 << 0)  /* Bit 0: Low-Power Debug Enable (LP_DBG_EN) */
                                                   /* Bits 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_RGM_H */
