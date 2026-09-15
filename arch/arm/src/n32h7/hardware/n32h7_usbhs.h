/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7_usbhs.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_USBHS_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_USBHS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/n32h7_memorymap.h"

/* ==========================================================================
 * Register Offsets
 * ==========================================================================
 */

/* Core global registers */
#define N32_USBHS_GCTRLSTS_OFF        0x0000UL
#define N32_USBHS_GAHBCFG_OFF         0x0008UL
#define N32_USBHS_GCFG_OFF            0x000CUL
#define N32_USBHS_GRSTCTRL_OFF        0x0010UL
#define N32_USBHS_GINTSTS_OFF         0x0014UL
#define N32_USBHS_GINTEN_OFF          0x0018UL
#define N32_USBHS_GRXSTS_OFF          0x001CUL
#define N32_USBHS_GRXSTSP_OFF         0x0020UL
#define N32_USBHS_GRXFSIZ_OFF         0x0024UL
#define N32_USBHS_GNPTXFSIZ_OFF       0x0028UL
#define N32_USBHS_GNPTXFSTS_OFF       0x002CUL
#define N32_USBHS_CID_OFF             0x003CUL
#define N32_USBHS_GPD_OFF             0x0058UL
#define N32_USBHS_HPTXFSIZ_OFF        0x0100UL
#define N32_USBHS_DINEPPTXFSIZ_OFF    0x0104UL        /* x = 1..8, offset = 0x0104 + (x1)4 */

/* Host mode registers */
#define N32_USBHS_HCFG_OFF            0x0400UL
#define N32_USBHS_HFRI_OFF            0x0404UL
#define N32_USBHS_HFNUM_OFF           0x0408UL
#define N32_USBHS_HPTXFQSTS_OFF       0x0410UL
#define N32_USBHS_HACHINT_OFF         0x0414UL
#define N32_USBHS_HACHINTEN_OFF       0x0418UL
#define N32_USBHS_HPCS_OFF            0x0440UL

/* Host channel registers (each 0x20) */
#define N32_USBHS_HCH_OFF(n)          (0x0500UL + ((n) << 5))
#define N32_USBHS_HCHCTRL_OFF(n)      (N32_USBHS_HCH_OFF(n) + 0x0000UL)
#define N32_USBHS_HCSCTRL_OFF(n)      (N32_USBHS_HCH_OFF(n) + 0x0004UL)
#define N32_USBHS_HCHINTSTS_OFF(n)    (N32_USBHS_HCH_OFF(n) + 0x0008UL)
#define N32_USBHS_HCHINTEN_OFF(n)     (N32_USBHS_HCH_OFF(n) + 0x000CUL)
#define N32_USBHS_HCHTXSIZ_OFF(n)     (N32_USBHS_HCH_OFF(n) + 0x0010UL)
#define N32_USBHS_HCHDMADD_OFF(n)     (N32_USBHS_HCH_OFF(n) + 0x0014UL)

/* Device mode registers */
#define N32_USBHS_DCFG_OFF            0x0800UL
#define N32_USBHS_DCTRL_OFF           0x0804UL
#define N32_USBHS_DSTS_OFF            0x0808UL
#define N32_USBHS_DINEPINTEN_OFF      0x0810UL
#define N32_USBHS_DOUTEPINTEN_OFF     0x0814UL
#define N32_USBHS_DAEPINTSTS_OFF      0x0818UL
#define N32_USBHS_DAEPINTEN_OFF       0x081CUL
#define N32_USBHS_DTHRCTRL_OFF        0x0830UL
#define N32_USBHS_DINEPFEINTEN_OFF    0x0834UL
#define N32_USBHS_DEEPINTSTS_OFF      0x0838UL
#define N32_USBHS_DEEPINTEN_OFF       0x083CUL
#define N32_USBHS_DINEPXINTEN_OFF     0x0840UL        /* x = 0..8, offset = 0x0840 + x4 */
#define N32_USBHS_DOUTEPXINTEN_OFF    0x0880UL        /* x = 0..8, offset = 0x0880 + x4 */

/* Device IN endpoint registers (each 0x20) */
#define N32_USBHS_DINEP_OFF(n)        (0x0900UL + ((n) << 5))
#define N32_USBHS_DINEPCTRL_OFF(n)    (N32_USBHS_DINEP_OFF(n) + 0x0000UL)
#define N32_USBHS_DINEPINTSTS_OFF(n)  (N32_USBHS_DINEP_OFF(n) + 0x0008UL)
#define N32_USBHS_DINEPTXSIZ_OFF(n)   (N32_USBHS_DINEP_OFF(n) + 0x0010UL)
#define N32_USBHS_DINEPDMADD_OFF(n)   (N32_USBHS_DINEP_OFF(n) + 0x0014UL)
#define N32_USBHS_DINEPTXFSTS_OFF(n)  (N32_USBHS_DINEP_OFF(n) + 0x0018UL)

/* Device OUT endpoint registers (each 0x20) */
#define N32_USBHS_DOUTEP_OFF(n)       (0x0B00UL + ((n) << 5))
#define N32_USBHS_DOUTEPCTRL_OFF(n)   (N32_USBHS_DOUTEP_OFF(n) + 0x0000UL)
#define N32_USBHS_DOUTEPINTSTS_OFF(n) (N32_USBHS_DOUTEP_OFF(n) + 0x0008UL)
#define N32_USBHS_DOUTEPTXSIZ_OFF(n)  (N32_USBHS_DOUTEP_OFF(n) + 0x0010UL)
#define N32_USBHS_DOUTEPDMADD_OFF(n)  (N32_USBHS_DOUTEP_OFF(n) + 0x0014UL)

/* Power control registers */
#define N32_USBHS_PWRCTRL_OFF         0x0E00UL
#define N32_USBHS_PWRCTRL1_OFF        0x0E04UL

/* Wrapper registers (separate base) */
#define N32_USBHS_WRPCTRL_OFF         0x0000UL
#define N32_USBHS_WRPCFG_OFF          0x0004UL

/* ==========================================================================
 * Register Addresses
 * ==========================================================================
 */
#define N32_USBHS1_GCTRLSTS            (N32_USBCTRL1_BASE + N32_USBHS_GCTRLSTS_OFF)
#define N32_USBHS1_GAHBCFG             (N32_USBCTRL1_BASE + N32_USBHS_GAHBCFG_OFF)
#define N32_USBHS1_GCFG                (N32_USBCTRL1_BASE + N32_USBHS_GCFG_OFF)
#define N32_USBHS1_GRSTCTRL            (N32_USBCTRL1_BASE + N32_USBHS_GRSTCTRL_OFF)
#define N32_USBHS1_GINTSTS             (N32_USBCTRL1_BASE + N32_USBHS_GINTSTS_OFF)
#define N32_USBHS1_GINTEN              (N32_USBCTRL1_BASE + N32_USBHS_GINTEN_OFF)
#define N32_USBHS1_GRXSTS              (N32_USBCTRL1_BASE + N32_USBHS_GRXSTS_OFF)
#define N32_USBHS1_GRXSTSP             (N32_USBCTRL1_BASE + N32_USBHS_GRXSTSP_OFF)
#define N32_USBHS1_GRXFSIZ             (N32_USBCTRL1_BASE + N32_USBHS_GRXFSIZ_OFF)
#define N32_USBHS1_GNPTXFSIZ           (N32_USBCTRL1_BASE + N32_USBHS_GNPTXFSIZ_OFF)
#define N32_USBHS1_GNPTXFSTS           (N32_USBCTRL1_BASE + N32_USBHS_GNPTXFSTS_OFF)
#define N32_USBHS1_CID                 (N32_USBCTRL1_BASE + N32_USBHS_CID_OFF)
#define N32_USBHS1_GPD                 (N32_USBCTRL1_BASE + N32_USBHS_GPD_OFF)
#define N32_USBHS1_HPTXFSIZ            (N32_USBCTRL1_BASE + N32_USBHS_HPTXFSIZ_OFF)

#define N32_USBHS1_DINEPPTXFSIZ(n)     (N32_USBCTRL1_BASE + N32_USBHS_DINEPPTXFSIZ_OFF + (((n)-1) << 2))

#define N32_USBHS2_GCTRLSTS            (N32_USBCTRL2_BASE + N32_USBHS_GCTRLSTS_OFF)
#define N32_USBHS2_GAHBCFG             (N32_USBCTRL2_BASE + N32_USBHS_GAHBCFG_OFF)
#define N32_USBHS2_GCFG                (N32_USBCTRL2_BASE + N32_USBHS_GCFG_OFF)
#define N32_USBHS2_GRSTCTRL            (N32_USBCTRL2_BASE + N32_USBHS_GRSTCTRL_OFF)
#define N32_USBHS2_GINTSTS             (N32_USBCTRL2_BASE + N32_USBHS_GINTSTS_OFF)
#define N32_USBHS2_GINTEN              (N32_USBCTRL2_BASE + N32_USBHS_GINTEN_OFF)
#define N32_USBHS2_GRXSTS              (N32_USBCTRL2_BASE + N32_USBHS_GRXSTS_OFF)
#define N32_USBHS2_GRXSTSP             (N32_USBCTRL2_BASE + N32_USBHS_GRXSTSP_OFF)
#define N32_USBHS2_GRXFSIZ             (N32_USBCTRL2_BASE + N32_USBHS_GRXFSIZ_OFF)
#define N32_USBHS2_GNPTXFSIZ           (N32_USBCTRL2_BASE + N32_USBHS_GNPTXFSIZ_OFF)
#define N32_USBHS2_GNPTXFSTS           (N32_USBCTRL2_BASE + N32_USBHS_GNPTXFSTS_OFF)
#define N32_USBHS2_CID                 (N32_USBCTRL2_BASE + N32_USBHS_CID_OFF)
#define N32_USBHS2_GPD                 (N32_USBCTRL2_BASE + N32_USBHS_GPD_OFF)
#define N32_USBHS2_HPTXFSIZ            (N32_USBCTRL2_BASE + N32_USBHS_HPTXFSIZ_OFF)

#define N32_USBHS2_DINEPPTXFSIZ(n)     (N32_USBCTRL2_BASE + N32_USBHS_DINEPPTXFSIZ_OFF + (((n)-1) << 2))

/* Host mode */
#define N32_USBHS1_HCFG                (N32_USBCTRL1_BASE + N32_USBHS_HCFG_OFF)
#define N32_USBHS1_HFRI                (N32_USBCTRL1_BASE + N32_USBHS_HFRI_OFF)
#define N32_USBHS1_HFNUM               (N32_USBCTRL1_BASE + N32_USBHS_HFNUM_OFF)
#define N32_USBHS1_HPTXFQSTS           (N32_USBCTRL1_BASE + N32_USBHS_HPTXFQSTS_OFF)
#define N32_USBHS1_HACHINT             (N32_USBCTRL1_BASE + N32_USBHS_HACHINT_OFF)
#define N32_USBHS1_HACHINTEN           (N32_USBCTRL1_BASE + N32_USBHS_HACHINTEN_OFF)
#define N32_USBHS1_HPCS                (N32_USBCTRL1_BASE + N32_USBHS_HPCS_OFF)

#define N32_USBHS1_HCHCTRL(n)          (N32_USBCTRL1_BASE + N32_USBHS_HCHCTRL_OFF(n))
#define N32_USBHS1_HCSCTRL(n)          (N32_USBCTRL1_BASE + N32_USBHS_HCSCTRL_OFF(n))
#define N32_USBHS1_HCHINTSTS(n)        (N32_USBCTRL1_BASE + N32_USBHS_HCHINTSTS_OFF(n))
#define N32_USBHS1_HCHINTEN(n)         (N32_USBCTRL1_BASE + N32_USBHS_HCHINTEN_OFF(n))
#define N32_USBHS1_HCHTXSIZ(n)         (N32_USBCTRL1_BASE + N32_USBHS_HCHTXSIZ_OFF(n))
#define N32_USBHS1_HCHDMADD(n)         (N32_USBCTRL1_BASE + N32_USBHS_HCHDMADD_OFF(n))

#define N32_USBHS2_HCFG                (N32_USBCTRL2_BASE + N32_USBHS_HCFG_OFF)
#define N32_USBHS2_HFRI                (N32_USBCTRL2_BASE + N32_USBHS_HFRI_OFF)
#define N32_USBHS2_HFNUM               (N32_USBCTRL2_BASE + N32_USBHS_HFNUM_OFF)
#define N32_USBHS2_HPTXFQSTS           (N32_USBCTRL2_BASE + N32_USBHS_HPTXFQSTS_OFF)
#define N32_USBHS2_HACHINT             (N32_USBCTRL2_BASE + N32_USBHS_HACHINT_OFF)
#define N32_USBHS2_HACHINTEN           (N32_USBCTRL2_BASE + N32_USBHS_HACHINTEN_OFF)
#define N32_USBHS2_HPCS                (N32_USBCTRL2_BASE + N32_USBHS_HPCS_OFF)

#define N32_USBHS2_HCHCTRL(n)          (N32_USBCTRL2_BASE + N32_USBHS_HCHCTRL_OFF(n))
#define N32_USBHS2_HCSCTRL(n)          (N32_USBCTRL2_BASE + N32_USBHS_HCSCTRL_OFF(n))
#define N32_USBHS2_HCHINTSTS(n)        (N32_USBCTRL2_BASE + N32_USBHS_HCHINTSTS_OFF(n))
#define N32_USBHS2_HCHINTEN(n)         (N32_USBCTRL2_BASE + N32_USBHS_HCHINTEN_OFF(n))
#define N32_USBHS2_HCHTXSIZ(n)         (N32_USBCTRL2_BASE + N32_USBHS_HCHTXSIZ_OFF(n))
#define N32_USBHS2_HCHDMADD(n)         (N32_USBCTRL2_BASE + N32_USBHS_HCHDMADD_OFF(n))

/* Device mode */
#define N32_USBHS1_DCFG                (N32_USBCTRL1_BASE + N32_USBHS_DCFG_OFF)
#define N32_USBHS1_DCTRL               (N32_USBCTRL1_BASE + N32_USBHS_DCTRL_OFF)
#define N32_USBHS1_DSTS                (N32_USBCTRL1_BASE + N32_USBHS_DSTS_OFF)
#define N32_USBHS1_DINEPINTEN          (N32_USBCTRL1_BASE + N32_USBHS_DINEPINTEN_OFF)
#define N32_USBHS1_DOUTEPINTEN         (N32_USBCTRL1_BASE + N32_USBHS_DOUTEPINTEN_OFF)
#define N32_USBHS1_DAEPINTSTS          (N32_USBCTRL1_BASE + N32_USBHS_DAEPINTSTS_OFF)
#define N32_USBHS1_DAEPINTEN           (N32_USBCTRL1_BASE + N32_USBHS_DAEPINTEN_OFF)
#define N32_USBHS1_DTHRCTRL            (N32_USBCTRL1_BASE + N32_USBHS_DTHRCTRL_OFF)
#define N32_USBHS1_DINEPFEINTEN        (0x40040000N32_USBCTRL1_BASE + N32_USBHS_DINEPFEINTEN_OFF)
#define N32_USBHS1_DEEPINTSTS          (N32_USBCTRL1_BASE + N32_USBHS_DEEPINTSTS_OFF)
#define N32_USBHS1_DEEPINTEN           (N32_USBCTRL1_BASE + N32_USBHS_DEEPINTEN_OFF)

#define N32_USBHS1_DINEPXINTEN(n)      (N32_USBCTRL1_BASE + N32_USBHS_DINEPXINTEN_OFF + ((n) << 2))
#define N32_USBHS1_DOUTEPXINTEN(n)     (N32_USBCTRL1_BASE + N32_USBHS_DOUTEPXINTEN_OFF + ((n) << 2))

#define N32_USBHS1_DINEPCTRL(n)        (N32_USBCTRL1_BASE + N32_USBHS_DINEPCTRL_OFF(n))
#define N32_USBHS1_DINEPINTSTS(n)      (N32_USBCTRL1_BASE + N32_USBHS_DINEPINTSTS_OFF(n))
#define N32_USBHS1_DINEPTXSIZ(n)       (N32_USBCTRL1_BASE + N32_USBHS_DINEPTXSIZ_OFF(n))
#define N32_USBHS1_DINEPDMADD(n)       (N32_USBCTRL1_BASE + N32_USBHS_DINEPDMADD_OFF(n))
#define N32_USBHS1_DINEPTXFSTS(n)      (N32_USBCTRL1_BASE + N32_USBHS_DINEPTXFSTS_OFF(n))

#define N32_USBHS1_DOUTEPCTRL(n)       (N32_USBCTRL1_BASE + N32_USBHS_DOUTEPCTRL_OFF(n))
#define N32_USBHS1_DOUTEPINTSTS(n)     (N32_USBCTRL1_BASE + N32_USBHS_DOUTEPINTSTS_OFF(n))
#define N32_USBHS1_DOUTEPTXSIZ(n)      (N32_USBCTRL1_BASE + N32_USBHS_DOUTEPTXSIZ_OFF(n))
#define N32_USBHS1_DOUTEPDMADD(n)      (N32_USBCTRL1_BASE + N32_USBHS_DOUTEPDMADD_OFF(n))

#define N32_USBHS2_DCFG                (N32_USBCTRL2_BASE + N32_USBHS_DCFG_OFF)
#define N32_USBHS2_DCTRL               (N32_USBCTRL2_BASE + N32_USBHS_DCTRL_OFF)
#define N32_USBHS2_DSTS                (N32_USBCTRL2_BASE + N32_USBHS_DSTS_OFF)
#define N32_USBHS2_DINEPINTEN          (N32_USBCTRL2_BASE + N32_USBHS_DINEPINTEN_OFF)
#define N32_USBHS2_DOUTEPINTEN         (N32_USBCTRL2_BASE + N32_USBHS_DOUTEPINTEN_OFF)
#define N32_USBHS2_DAEPINTSTS          (N32_USBCTRL2_BASE + N32_USBHS_DAEPINTSTS_OFF)
#define N32_USBHS2_DAEPINTEN           (N32_USBCTRL2_BASE + N32_USBHS_DAEPINTEN_OFF)
#define N32_USBHS2_DTHRCTRL            (N32_USBCTRL2_BASE + N32_USBHS_DTHRCTRL_OFF)
#define N32_USBHS2_DINEPFEINTEN        (N32_USBCTRL2_BASE + N32_USBHS_DINEPFEINTEN_OFF)
#define N32_USBHS2_DEEPINTSTS          (N32_USBCTRL2_BASE + N32_USBHS_DEEPINTSTS_OFF)
#define N32_USBHS2_DEEPINTEN           (N32_USBCTRL2_BASE + N32_USBHS_DEEPINTEN_OFF)

#define N32_USBHS2_DINEPXINTEN(n)      (N32_USBCTRL2_BASE + N32_USBHS_DINEPXINTEN_OFF + ((n) << 2))
#define N32_USBHS2_DOUTEPXINTEN(n)     (N32_USBCTRL2_BASE + N32_USBHS_DOUTEPXINTEN_OFF + ((n) << 2))

#define N32_USBHS2_DINEPCTRL(n)        (N32_USBCTRL2_BASE + N32_USBHS_DINEPCTRL_OFF(n))
#define N32_USBHS2_DINEPINTSTS(n)      (N32_USBCTRL2_BASE + N32_USBHS_DINEPINTSTS_OFF(n))
#define N32_USBHS2_DINEPTXSIZ(n)       (N32_USBCTRL2_BASE + N32_USBHS_DINEPTXSIZ_OFF(n))
#define N32_USBHS2_DINEPDMADD(n)       (N32_USBCTRL2_BASE + N32_USBHS_DINEPDMADD_OFF(n))
#define N32_USBHS2_DINEPTXFSTS(n)      (N32_USBCTRL2_BASE + N32_USBHS_DINEPTXFSTS_OFF(n))

#define N32_USBHS2_DOUTEPCTRL(n)       (N32_USBCTRL2_BASE + N32_USBHS_DOUTEPCTRL_OFF(n))
#define N32_USBHS2_DOUTEPINTSTS(n)     (N32_USBCTRL2_BASE + N32_USBHS_DOUTEPINTSTS_OFF(n))
#define N32_USBHS2_DOUTEPTXSIZ(n)      (N32_USBCTRL2_BASE + N32_USBHS_DOUTEPTXSIZ_OFF(n))
#define N32_USBHS2_DOUTEPDMADD(n)      (N32_USBCTRL2_BASE + N32_USBHS_DOUTEPDMADD_OFF(n))

/* Power control */
#define N32_USBHS1_PWRCTRL             (N32_USBCTRL1_BASE + N32_USBHS_PWRCTRL_OFF)
#define N32_USBHS1_PWRCTRL1            (N32_USBCTRL1_BASE + N32_USBHS_PWRCTRL1_OFF)

#define N32_USBHS2_PWRCTRL             (N32_USBCTRL2_BASE + N32_USBHS_PWRCTRL_OFF)
#define N32_USBHS2_PWRCTRL1            (N32_USBCTRL2_BASE + N32_USBHS_PWRCTRL1_OFF)

/* Wrapper (separate base) */
#define N32_USBHS1_WRPCTRL             (N32_USBCTRL1_WRAPPER_BASE + N32_USBHS_WRPCTRL_OFF)
#define N32_USBHS1_WRPCFG              (N32_USBCTRL1_WRAPPER_BASE + N32_USBHS_WRPCFG_OFF)

#define N32_USBHS2_WRPCTRL             (N32_USBCTRL2_WRAPPER_BASE + N32_USBHS_WRPCTRL_OFF)
#define N32_USBHS2_WRPCFG              (N32_USBCTRL2_WRAPPER_BASE + N32_USBHS_WRPCFG_OFF)

/* ==========================================================================
 * Bitfield Definitions (no _POS macros, only _SHIFT and _MASK where needed)
 * ==========================================================================
 */

/* GCTRLSTS (0x000) *********************************************************/
#define N32_USBHS_GCTRLSTS_VBVALOVEN    (1UL << 2)  /* VBUS valid override enable */
#define N32_USBHS_GCTRLSTS_VBVALOVAL    (1UL << 3)  /* VBUS valid override value */
#define N32_USBHS_GCTRLSTS_IDSTS        (1UL << 16) /* Connector ID status */
#define N32_USBHS_GCTRLSTS_DETIM        (1UL << 17) /* Long/short debounce time */
#define N32_USBHS_GCTRLSTS_CMODE        (1UL << 21) /* Current mode (0:Device, 1:Host) */

/* GAHBCFG (0x008) **********************************************************/
#define N32_USBHS_GAHBCFG_GINTEN            (1UL << 0)  /* Global interrupt mask */
#define N32_USBHS_GAHBCFG_BURSTTYP_SHIFT    1
#define N32_USBHS_GAHBCFG_BURSTTYP_MASK     (0xFUL << N32_USBHS_GAHBCFG_BURSTTYP_SHIFT)
#  define N32_USBHS_GAHBCFG_BURSTTYP_SINGLE (0UL << N32_USBHS_GAHBCFG_BURSTTYP_SHIFT)
#  define N32_USBHS_GAHBCFG_BURSTTYP_INCR   (1UL << N32_USBHS_GAHBCFG_BURSTTYP_SHIFT)
#  define N32_USBHS_GAHBCFG_BURSTTYP_INCR4  (3UL << N32_USBHS_GAHBCFG_BURSTTYP_SHIFT)
#  define N32_USBHS_GAHBCFG_BURSTTYP_INCR8  (5UL << N32_USBHS_GAHBCFG_BURSTTYP_SHIFT)
#  define N32_USBHS_GAHBCFG_BURSTTYP_INCR16 (7UL << N32_USBHS_GAHBCFG_BURSTTYP_SHIFT)
#define N32_USBHS_GAHBCFG_DMAEN             (1UL << 5)  /* DMA enable */
#define N32_USBHS_GAHBCFG_NPTXFETH          (1UL << 7)  /* Non-periodic TxFIFO empty level */
#define N32_USBHS_GAHBCFG_PTXFETH           (1UL << 8)  /* Periodic TxFIFO empty level */

/* GCFG (0x00C) *************************************************************/
#define N32_USBHS_GCFG_TOCAL_SHIFT      0
#define N32_USBHS_GCFG_TOCAL_MASK       (7UL << N32_USBHS_GCFG_TOCAL_SHIFT)    /* FS timeout calibration */
#define N32_USBHS_GCFG_PHYIF            (1UL << 3)                             /* PHY interface (0:8bit, 1:16bit) */
#define N32_USBHS_GCFG_PHYSEL           (1UL << 6)                             /* PHY select (0:USB2.0 HS, 1:USB1.1 FS) */
#define N32_USBHS_GCFG_TRDTIM_SHIFT     10
#define N32_USBHS_GCFG_TRDTIM_MASK      (0xFUL << N32_USBHS_GCFG_TRDTIM_SHIFT) /* USB turnaround time */
#define N32_USBHS_GCFG_FHMODE           (1UL << 29)                            /* Force host mode */
#define N32_USBHS_GCFG_FDMODE           (1UL << 30)                            /* Force device mode */

/* GRSTCTRL (0x010) *********************************************************/
#define N32_USBHS_GRSTCTRL_CSRST        (1UL << 0)                                  /* Core soft reset */
#define N32_USBHS_GRSTCTRL_PFSSRST      (1UL << 1)                                  /* PIU FS-specific controller reset */
#define N32_USBHS_GRSTCTRL_HFCRST       (1UL << 2)                                  /* Host frame counter reset */
#define N32_USBHS_GRSTCTRL_RXFFLSH      (1UL << 4)                                  /* RxFIFO flush */
#define N32_USBHS_GRSTCTRL_TXFFLSH      (1UL << 5)                                  /* TxFIFO flush */
#define N32_USBHS_GRSTCTRL_TXFNUM_SHIFT 6
#define N32_USBHS_GRSTCTRL_TXFNUM_MASK  (0x1FUL << N32_USBHS_GRSTCTRL_TXFNUM_SHIFT) /* TxFIFO number */
#define N32_USBHS_GRSTCTRL_SRSTDNE      (1UL << 29)                                 /* Software reset done */
#define N32_USBHS_GRSTCTRL_DMAREQ       (1UL << 30)                                 /* DMA request signal */
#define N32_USBHS_GRSTCTRL_AHBIDLE      (1UL << 31)                                 /* AHB master idle */

/* GINTSTS (0x014) **********************************************************/
#define N32_USBHS_GINTSTS_CMODE         (1UL << 0)     /* Current mode (0:Device, 1:Host) */
#define N32_USBHS_GINTSTS_MODMISIF      (1UL << 1)     /* Mode mismatch interrupt */
#define N32_USBHS_GINTSTS_SOFIF         (1UL << 3)     /* Start of frame */
#define N32_USBHS_GINTSTS_RXFNEIF       (1UL << 4)     /* RxFIFO non-empty */
#define N32_USBHS_GINTSTS_NPTXFEIF      (1UL << 5)     /* Non-periodic TxFIFO empty */
#define N32_USBHS_GINTSTS_GINNPNAKEIF   (1UL << 6)     /* Global IN non-periodic NAK effective */
#define N32_USBHS_GINTSTS_GOUTNAKEIF    (1UL << 7)     /* Global OUT NAK effective */
#define N32_USBHS_GINTSTS_ESUSPIF       (1UL << 10)    /* Early suspend */
#define N32_USBHS_GINTSTS_USBSUSPIF     (1UL << 11)    /* USB suspend */
#define N32_USBHS_GINTSTS_USBRSTIF      (1UL << 12)    /* USB reset */
#define N32_USBHS_GINTSTS_ENUMDIF       (1UL << 13)    /* Enumeration done */
#define N32_USBHS_GINTSTS_ISOUTPDIF     (1UL << 14)    /* Isochronous OUT packet dropped */
#define N32_USBHS_GINTSTS_EOPFIF        (1UL << 15)    /* End of periodic frame */
#define N32_USBHS_GINTSTS_INEPIF        (1UL << 18)    /* IN endpoint interrupt */
#define N32_USBHS_GINTSTS_OUTEPIF       (1UL << 19)    /* OUT endpoint interrupt */
#define N32_USBHS_GINTSTS_ISOINCIF      (1UL << 20)    /* Incomplete isochronous IN transfer */
#define N32_USBHS_GINTSTS_PTNCIF_ISOUTNCIF (1UL << 21) /* Incomplete periodic / isochronous OUT */
#define N32_USBHS_GINTSTS_FETSUSPIF     (1UL << 22)    /* Data fetch suspended */
#define N32_USBHS_GINTSTS_RSTDIF        (1UL << 23)    /* Reset detected */
#define N32_USBHS_GINTSTS_HPIF          (1UL << 24)    /* Host port interrupt */
#define N32_USBHS_GINTSTS_HCHIF         (1UL << 25)    /* Host channels interrupt */
#define N32_USBHS_GINTSTS_PTXFEIF       (1UL << 26)    /* Periodic TxFIFO empty */
#define N32_USBHS_GINTSTS_IDSTSCIF      (1UL << 28)    /* ID pin status change */
#define N32_USBHS_GINTSTS_DISCIF        (1UL << 29)    /* Disconnect detected */
#define N32_USBHS_GINTSTS_WKUPIF        (1UL << 31)    /* Resume/remote wakeup detected */

/* GINTEN (0x018) ***********************************************************/
#define N32_USBHS_GINTEN_MODMISIEN      (1UL << 1)      /* Mode mismatch interrupt enable */
#define N32_USBHS_GINTEN_USBHSIEN       (1UL << 2)      /* USBHS interrupt enable */
#define N32_USBHS_GINTEN_SOFIEN         (1UL << 3)      /* SOF interrupt enable */
#define N32_USBHS_GINTEN_RXFNEIEN       (1UL << 4)      /* RxFIFO non-empty interrupt enable */
#define N32_USBHS_GINTEN_NPTXFEIEN      (1UL << 5)      /* Non-periodic TxFIFO empty interrupt enable */
#define N32_USBHS_GINTEN_GINNPNAKEIEN   (1UL << 6)      /* Global IN non-periodic NAK effective interrupt enable */
#define N32_USBHS_GINTEN_GOUTNAKEIEN    (1UL << 7)      /* Global OUT NAK effective interrupt enable */
#define N32_USBHS_GINTEN_ESUSPIEN       (1UL << 10)     /* Early suspend interrupt enable */
#define N32_USBHS_GINTEN_USBSUSPIEN     (1UL << 11)     /* USB suspend interrupt enable */
#define N32_USBHS_GINTEN_USBRSTIEN      (1UL << 12)     /* USB reset interrupt enable */
#define N32_USBHS_GINTEN_ENUMDIEN       (1UL << 13)     /* Enumeration done interrupt enable */
#define N32_USBHS_GINTEN_ISOUTPDIEN     (1UL << 14)     /* Isochronous OUT packet dropped interrupt enable */
#define N32_USBHS_GINTEN_EOPFIEN        (1UL << 15)     /* End of periodic frame interrupt enable */
#define N32_USBHS_GINTEN_INEPIEN        (1UL << 18)     /* IN endpoint interrupt enable */
#define N32_USBHS_GINTEN_OUTEPIEN       (1UL << 19)     /* OUT endpoint interrupt enable */
#define N32_USBHS_GINTEN_ISOINCIEN      (1UL << 20)     /* Incomplete isochronous IN interrupt enable */
#define N32_USBHS_GINTEN_PTNCIEN_ISOUTNCIEN (1UL << 21) /* Incomplete periodic / isochronous OUT interrupt enable */
#define N32_USBHS_GINTEN_FETSUSPIEN     (1UL << 22)     /* Data fetch suspended interrupt enable */
#define N32_USBHS_GINTEN_RSTDIEN        (1UL << 23)     /* Reset detected interrupt enable */
#define N32_USBHS_GINTEN_HPIEN          (1UL << 24)     /* Host port interrupt enable */
#define N32_USBHS_GINTEN_HCHIEN         (1UL << 25)     /* Host channels interrupt enable */
#define N32_USBHS_GINTEN_PTXFEIEN       (1UL << 26)     /* Periodic TxFIFO empty interrupt enable */
#define N32_USBHS_GINTEN_IDSTSCIEN      (1UL << 28)     /* ID pin status change interrupt enable */
#define N32_USBHS_GINTEN_DISCIEN        (1UL << 29)     /* Disconnect interrupt enable */
#define N32_USBHS_GINTEN_VBUSVIF        (1UL << 30)     /* VBUS valid interrupt enable */
#define N32_USBHS_GINTEN_WKUPIEN        (1UL << 31)     /* Resume/remote wakeup interrupt enable */

/* GRXSTS / GRXSTSP (0x01C / 0x020) *****************************************/
#define N32_USBHS_GRXSTS_CHEPNUM_SHIFT 0
#define N32_USBHS_GRXSTS_CHEPNUM_MASK  (0xFUL << N32_USBHS_GRXSTS_CHEPNUM_SHIFT) /* Channel/endpoint number */
#define N32_USBHS_GRXSTS_BCNT_SHIFT    4
#define N32_USBHS_GRXSTS_BCNT_MASK     (0x7FFUL << N32_USBHS_GRXSTS_BCNT_SHIFT)  /* Byte count */
#define N32_USBHS_GRXSTS_DPID_SHIFT    15
#define N32_USBHS_GRXSTS_DPID_MASK     (3UL << N32_USBHS_GRXSTS_DPID_SHIFT)      /* Data PID */
#  define N32_USBHS_GRXSTS_DPID_0      (0UL << N32_USBHS_GRXSTS_DPID_SHIFT)      /* Data0 */
#  define N32_USBHS_GRXSTS_DPID_2      (1UL << N32_USBHS_GRXSTS_DPID_SHIFT)      /* Data2 */
#  define N32_USBHS_GRXSTS_DPID_1      (2UL << N32_USBHS_GRXSTS_DPID_SHIFT)      /* Data1 */
#  define N32_USBHS_GRXSTS_DPID_M      (3UL << N32_USBHS_GRXSTS_DPID_SHIFT)      /* MData */
#define N32_USBHS_GRXSTS_PKTSTS_SHIFT  17
#define N32_USBHS_GRXSTS_PKTSTS_MASK   (0xFUL << N32_USBHS_GRXSTS_PKTSTS_SHIFT)  /* Packet status */

/* Host mode */
#  define N32_USBHS_GRXSTS_IN_RCVD     (2UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT) /* Received IN packet */
#  define N32_USBHS_GRXSTS_IN_CPLT     (3UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT) /* Completed IN packet (Trigger interrupt) */
#  define N32_USBHS_GRXSTS_DATA_TOGERR (5UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT) /* Data toggle error (Trigger interrupt) */
#  define N32_USBHS_GRXSTS_CHAN_TERMIN (7UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT) /* Channel terminated (Trigger interrupt) */

/* Device mode */
#  define N32_USBHS_GRXSTS_OUT_NAK     (1UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT)  /* Global OUT NAK (Trigger interrupt) */
#  define N32_USBHS_GRXSTS_OUT_RCVD    (2UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT)  /* Received OUT packet */
#  define N32_USBHS_GRXSTS_OUT_CPLT    (3UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT)  /* Completed OUT packet (Trigger interrupt) */
#  define N32_USBHS_GRXSTS_SETUP_DNE   (4UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT)  /* Setup done (Trigger interrupt) */
#  define N32_USBHS_GRXSTS_SETUP_RCVD  (6UL << N32_USBHS_GRXSTS_PKTSTS_SHIFT)  /* Received setup packet (Trigger interrupt) */
#define N32_USBHS_GRXSTS_FUNM_SHIFT     21
#define N32_USBHS_GRXSTS_FUNM_MASK      (0xFUL << N32_USBHS_GRXSTS_FUNM_SHIFT) /* Frame number (device) */

/* GRXFSIZ (0x024) **********************************************************/
#define N32_USBHS_GRXFSIZ_RXFDEP_MASK   (0x7FFUL)                              /* Rx FIFO depth (words) */

/* GNPTXFSIZ (0x028) ********************************************************/

/* Host mode */
#define N32_USBHS_GNPTXFSIZ_NPTXFSADD_SHIFT    0
#define N32_USBHS_GNPTXFSIZ_NPTXFSADD_MASK     (0x7FFUL << N32_USBHS_GNPTXFSIZ_NPTXFSADD_SHIFT)
#define N32_USBHS_GNPTXFSIZ_NPTXFDEP_SHIFT     16
#define N32_USBHS_GNPTXFSIZ_NPTXFDEP_MASK      (0x7FFUL << N32_USBHS_GNPTXFSIZ_NPTXFDEP_SHIFT)

/* Device mode (EP0) */
#define N32_USBHS_GNPTXFSIZ_IEP0TXFRSADD_SHIFT 0
#define N32_USBHS_GNPTXFSIZ_IEP0TXFRSADD_MASK  (0x7FFUL << N32_USBHS_GNPTXFSIZ_IEP0TXFRSADD_SHIFT)
#define N32_USBHS_GNPTXFSIZ_IEP0TXFDEP_SHIFT   16
#define N32_USBHS_GNPTXFSIZ_IEP0TXFDEP_MASK    (0x7FFUL << N32_USBHS_GNPTXFSIZ_IEP0TXFDEP_SHIFT)

/* GNPTXFSTS (0x02C) ********************************************************/
#define N32_USBHS_GNPTXFSTS_NPTXFSAV_SHIFT            0
#define N32_USBHS_GNPTXFSTS_NPTXFSAV_MASK             (0xFFFFUL << N32_USBHS_GNPTXFSTS_NPTXFSAV_SHIFT)
#define N32_USBHS_GNPTXFSTS_NPTXRQSAV_SHIFT           16
#define N32_USBHS_GNPTXFSTS_NPTXRQSAV_MASK            (0xFFUL << N32_USBHS_GNPTXFSTS_NPTXRQSAV_SHIFT)
#define N32_USBHS_GNPTXFSTS_NPTXRQTOP_SHIFT           24
#define N32_USBHS_GNPTXFSTS_NPTXRQTOP_MASK            (0x7FUL << N32_USBHS_GNPTXFSTS_NPTXRQTOP_SHIFT)
#  define N32_USBHS_GNPTXFSTS_NPTXRQTOP_END           (1UL << 24)                                        /* Terminate */
#  define N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_SHIFT   25
#  define N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_MASK    (3UL << N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_SHIFT)
#    define N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_INOUT (0UL << N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_SHIFT) /* IN/OUT token */
#    define N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_0LEN  (1UL << N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_SHIFT) /* 0-length packet token */
#    define N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_DISCH (3UL << N32_USBHS_GNPTXFSTS_NPTXRQTOP_TOKEN_SHIFT) /* Disable channel command */
#  define N32_USBHS_GNPTXFSTS_NPTXRQTOP_CHNUM_SHIFT   27
#  define N32_USBHS_GNPTXFSTS_NPTXRQTOP_CHNUM_MASK    (0xFUL << N32_USBHS_GNPTXFSTS_NPTXRQTOP_CHNUM_SHIFT)

/* CID (0x03C) **************************************************************/
#define N32_USBHS_CID_MASK              (0xFFFFFFFFUL)                                                   /* User ID */

/* GPD (0x058) **************************************************************/
#define N32_USBHS_GPD_LINSTSCHNGSTS     (1UL << 7)  /* Line status change status */
#define N32_USBHS_GPD_LINSTSCHNGINTEN   (1UL << 8)  /* Line status change interrupt enable */
#define N32_USBHS_GPD_RESTDETSTS        (1UL << 9)  /* Reset detected status */
#define N32_USBHS_GPD_RESTDETINTEN      (1UL << 10) /* Reset detected interrupt enable */
#define N32_USBHS_GPD_DISCONNDETSTS     (1UL << 11) /* Disconnect detected status */
#define N32_USBHS_GPD_DISCONNDETINTEN   (1UL << 12) /* Disconnect detected interrupt enable */
#define N32_USBHS_GPD_CONNDETSTS        (1UL << 13) /* Connect detected status */
#define N32_USBHS_GPD_CONNDETINTEN      (1UL << 14) /* Connect detected interrupt enable */
#define N32_USBHS_GPD_STSCHNGINTSTS     (1UL << 17) /* Status change interrupt status */
#define N32_USBHS_GPD_STSCHNGINTEN      (1UL << 18) /* Status change interrupt enable */
#define N32_USBHS_GPD_LINSTS_SHIFT      19
#define N32_USBHS_GPD_LINSTS_MASK       (3UL << N32_USBHS_GPD_LINSTS_SHIFT) /* Line status */
#  define N32_USBHS_GPD_LINSTS_DM0DP0   (0UL << N32_USBHS_GPD_LINSTS_SHIFT) /* DM=0, DP=0 */
#  define N32_USBHS_GPD_LINSTS_DM0DP1   (1UL << N32_USBHS_GPD_LINSTS_SHIFT) /* DM=0, DP=1 */
#  define N32_USBHS_GPD_LINSTS_DM1DP0   (2UL << N32_USBHS_GPD_LINSTS_SHIFT) /* DM=1, DP=0 */
#define N32_USBHS_GPD_IDDIG             (1UL << 21)                         /* IDDIG signal state (0:Host, 1:Device) */

/* HPTXFSIZ (0x100) *********************************************************/
#define N32_USBHS_HPTXFSIZ_HPTXFSADD_SHIFT 0
#define N32_USBHS_HPTXFSIZ_HPTXFSADD_MASK  (0xFFFFUL << N32_USBHS_HPTXFSIZ_HPTXFSADD_SHIFT)
#define N32_USBHS_HPTXFSIZ_HPTXFDEP_SHIFT  16
#define N32_USBHS_HPTXFSIZ_HPTXFDEP_MASK   (0xFFFFUL << N32_USBHS_HPTXFSIZ_HPTXFDEP_SHIFT)

/* DINEPPTXFSIZ[1..8] (0x104 + (x1)4) ***************************************/
#define N32_USBHS_DINEPPTXFSIZ_INEPTXSADD_SHIFT 0
#define N32_USBHS_DINEPPTXFSIZ_INEPTXSADD_MASK  (0xFFFFUL << N32_USBHS_DINEPPTXFSIZ_INEPTXSADD_SHIFT)
#define N32_USBHS_DINEPPTXFSIZ_INEPTXFDEP_SHIFT 16
#define N32_USBHS_DINEPPTXFSIZ_INEPTXFDEP_MASK  (0xFFFFUL << N32_USBHS_DINEPPTXFSIZ_INEPTXFDEP_SHIFT)

/* ==========================================================================
 * Host Mode Registers
 * ==========================================================================
 */

/* HCFG (0x400) *************************************************************/
#define N32_USBHS_HCFG_SPSEL            (1UL << 2)  /* USB speed select (0:HS/FS/LS, 1:FS/LS only) */

/* HFRI (0x404) *************************************************************/
#define N32_USBHS_HFRI_FRI_SHIFT        0
#define N32_USBHS_HFRI_FRI_MASK         (0xFFFFUL << N32_USBHS_HFRI_FRI_SHIFT) /* Frame interval */
#define N32_USBHS_HFRI_DRLDEN           (1UL << 16)                            /* Dynamic reload enable */

/* HFNUM (0x408) ************************************************************/
#define N32_USBHS_HFNUM_FRNUM_SHIFT     0
#define N32_USBHS_HFNUM_FRNUM_MASK      (0xFFFFUL << N32_USBHS_HFNUM_FRNUM_SHIFT) /* Frame number */
#define N32_USBHS_HFNUM_FRT_SHIFT       16
#define N32_USBHS_HFNUM_FRT_MASK        (0xFFFFUL << N32_USBHS_HFNUM_FRT_SHIFT)   /* Frame time remaining */

/* HPTXFQSTS (0x410) ********************************************************/
#define N32_USBHS_HPTXFQSTS_PTXFSAVL_SHIFT            0
#define N32_USBHS_HPTXFQSTS_PTXFSAVL_MASK             (0xFFFFUL << N32_USBHS_HPTXFQSTS_PTXFSAVL_SHIFT)
#define N32_USBHS_HPTXFQSTS_PTXRQSAVL_SHIFT           16
#define N32_USBHS_HPTXFQSTS_PTXRQSAVL_MASK            (0x7FUL << N32_USBHS_HPTXFQSTS_PTXRQSAVL_SHIFT)
#define N32_USBHS_HPTXFQSTS_PTXRQTOP_SHIFT            23
#define N32_USBHS_HPTXFQSTS_PTXRQTOP_MASK             (0xFFUL << N32_USBHS_HPTXFQSTS_PTXRQTOP_SHIFT)
#  define N32_USBHS_HPTXFQSTS_PTXRQTOP_END            (1UL << 23) /* Terminate */
#  define N32_USBHS_HPTXFQSTS_PTXRQTOP_PEND           (1UL << 24) /* Pending */
#  define N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_SHIFT    25
#  define N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_MASK     (3UL << N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_SHIFT)
#    define N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_INOUT  (0UL << N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_SHIFT) /* IN/OUT token */
#    define N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_0LEN   (1UL << N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_SHIFT) /* 0-length packet token */
#    define N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_CSPLIT (2UL << N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_SHIFT) /* CSPLIT */
#    define N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_DISCH  (3UL << N32_USBHS_HPTXFQSTS_PTXRQTOP_TOKEN_SHIFT) /* Disable channel command */
#  define N32_USBHS_HPTXFQSTS_PTXRQTOP_CHNUM_SHIFT    27
#  define N32_USBHS_HPTXFQSTS_PTXRQTOP_CHNUM_MASK     (0xFUL << N32_USBHS_HPTXFQSTS_PTXRQTOP_CHNUM_SHIFT)
#  define N32_USBHS_HPTXFQSTS_PTXRQTOP_OEFRM          (1UL << 31)                                       /* Odd/even frame */

/* HACHINT (0x414) **********************************************************/
#define N32_USBHS_HACHINT_MASK          (0xFFFFUL)  /* Channel interrupt bits */

/* HACHINTEN (0x418) ********************************************************/
#define N32_USBHS_HACHINTEN_MASK        (0xFFFFUL)  /* Channel interrupt enable bits */

/* HPCS (0x440) *************************************************************/
#define N32_USBHS_HPCS_PCSTS            (1UL << 0)  /* Port connect status */
#define N32_USBHS_HPCS_PCDET            (1UL << 1)  /* Port connect detected */
#define N32_USBHS_HPCS_PEN              (1UL << 2)  /* Port enable */
#define N32_USBHS_HPCS_PENC             (1UL << 3)  /* Port enable/disable change */
#define N32_USBHS_HPCS_POCA             (1UL << 4)  /* Port overcurrent active */
#define N32_USBHS_HPCS_POCC             (1UL << 5)  /* Port overcurrent change */
#define N32_USBHS_HPCS_PRES             (1UL << 6)  /* Port resume */
#define N32_USBHS_HPCS_PSUSP            (1UL << 7)  /* Port suspend */
#define N32_USBHS_HPCS_PRST             (1UL << 8)  /* Port reset */
#define N32_USBHS_HPCS_PLSTS_SHIFT      10
#define N32_USBHS_HPCS_PLSTS_MASK       (3UL << N32_USBHS_HPCS_PLSTS_SHIFT) /* Port line status */
#define N32_USBHS_HPCS_PPWR             (1UL << 12)                         /* Port power */
#define N32_USBHS_HPCS_PTCTRL_SHIFT     13
#define N32_USBHS_HPCS_PTCTRL_MASK      (0xFUL << N32_USBHS_HPCS_PTCTRL_SHIFT) /* Port test control */
#  define N32_USBHS_HPCS_PTCTRL_TEST_NO (0UL << N32_USBHS_HPCS_PTCTRL_SHIFT)   /* No test */
#  define N32_USBHS_HPCS_PTCTRL_TEST_J  (1UL << N32_USBHS_HPCS_PTCTRL_SHIFT)   /* Test_J mode */
#  define N32_USBHS_HPCS_PTCTRL_TEST_K  (2UL << N32_USBHS_HPCS_PTCTRL_SHIFT)   /* Test_K mode */
#  define N32_USBHS_HPCS_PTCTRL_TEST_SN (3UL << N32_USBHS_HPCS_PTCTRL_SHIFT)   /* Test_SE0_NAK mode */
#  define N32_USBHS_HPCS_PTCTRL_TEST_PK (4UL << N32_USBHS_HPCS_PTCTRL_SHIFT)   /* Test_Packet mode */
#  define N32_USBHS_HPCS_PTCTRL_TEST_FE (5UL << N32_USBHS_HPCS_PTCTRL_SHIFT)   /* Test forced enable */
#define N32_USBHS_HPCS_PSPD_SHIFT       17
#define N32_USBHS_HPCS_PSPD_MASK        (3UL << N32_USBHS_HPCS_PSPD_SHIFT)     /* Port speed */
#  define N32_USBHS_HPCS_PSPD_HS        (0UL << N32_USBHS_HPCS_PSPD_SHIFT)     /* High-speed */
#  define N32_USBHS_HPCS_PSPD_FS        (1UL << N32_USBHS_HPCS_PSPD_SHIFT)     /* Full-speed */
#  define N32_USBHS_HPCS_PSPD_LS        (2UL << N32_USBHS_HPCS_PSPD_SHIFT)     /* Low-speed */

/* HCHCTRL (0x500 + n0x20) **************************************************/
#define N32_USBHS_HCHCTRL_MPS_SHIFT     0
#define N32_USBHS_HCHCTRL_MPS_MASK      (0x7FFUL << N32_USBHS_HCHCTRL_MPS_SHIFT) /* Max packet size */
#define N32_USBHS_HCHCTRL_EPNUM_SHIFT   11
#define N32_USBHS_HCHCTRL_EPNUM_MASK    (0xFUL << N32_USBHS_HCHCTRL_EPNUM_SHIFT) /* Endpoint number */
#define N32_USBHS_HCHCTRL_EPDIR         (1UL << 15)                              /* Endpoint direction (0:OUT, 1:IN) */
#define N32_USBHS_HCHCTRL_LSPDDEV       (1UL << 17)                              /* Low-speed device */
#define N32_USBHS_HCHCTRL_EPTYPE_SHIFT  18
#define N32_USBHS_HCHCTRL_EPTYPE_MASK   (3UL << N32_USBHS_HCHCTRL_EPTYPE_SHIFT)  /* Endpoint type */
#  define N32_USBHS_HCHCTRL_EPTYPE_CTRL (0UL << N32_USBHS_HCHCTRL_EPTYPE_SHIFT)
#  define N32_USBHS_HCHCTRL_EPTYPE_ISOC (1UL << N32_USBHS_HCHCTRL_EPTYPE_SHIFT)
#  define N32_USBHS_HCHCTRL_EPTYPE_BULK (2UL << N32_USBHS_HCHCTRL_EPTYPE_SHIFT)
#  define N32_USBHS_HCHCTRL_EPTYPE_INTR (3UL << N32_USBHS_HCHCTRL_EPTYPE_SHIFT)
#define N32_USBHS_HCHCTRL_DEVADDR_SHIFT 22
#define N32_USBHS_HCHCTRL_DEVADDR_MASK  (0x7FUL << N32_USBHS_HCHCTRL_DEVADDR_SHIFT) /* Device address */
#define N32_USBHS_HCHCTRL_ODDFRM        (1UL << 29)                                 /* Odd frame */
#define N32_USBHS_HCHCTRL_CHDIS         (1UL << 30)                                 /* Channel disable */
#define N32_USBHS_HCHCTRL_CHEN          (1UL << 31)                                 /* Channel enable */

/* HCSCTRL (0x504 + n0x20) **************************************************/
#define N32_USBHS_HCSCTRL_PRTADD_SHIFT  0
#define N32_USBHS_HCSCTRL_PRTADD_MASK   (0x7FUL << N32_USBHS_HCSCTRL_PRTADD_SHIFT) /* Port address */
#define N32_USBHS_HCSCTRL_HUBADD_SHIFT  7
#define N32_USBHS_HCSCTRL_HUBADD_MASK   (0x7FUL << N32_USBHS_HCSCTRL_HUBADD_SHIFT) /* Hub address */
#define N32_USBHS_HCSCTRL_TRANPOS_SHIFT 14
#define N32_USBHS_HCSCTRL_TRANPOS_MASK  (3UL << N32_USBHS_HCSCTRL_TRANPOS_SHIFT)   /* Transaction position */
#  define N32_USBHS_HCSCTRL_TRANPOS_MID (0UL << N32_USBHS_HCSCTRL_TRANPOS_SHIFT)   /* Middle */
#  define N32_USBHS_HCSCTRL_TRANPOS_END (1UL << N32_USBHS_HCSCTRL_TRANPOS_SHIFT)   /* Last */
#  define N32_USBHS_HCSCTRL_TRANPOS_ALL (2UL << N32_USBHS_HCSCTRL_TRANPOS_SHIFT)   /* All */
#  define N32_USBHS_HCSCTRL_TRANPOS_BGN (3UL << N32_USBHS_HCSCTRL_TRANPOS_SHIFT)   /* First */
#define N32_USBHS_HCSCTRL_COMPSPLF      (1UL << 16)                                /* Complete split */
#define N32_USBHS_HCSCTRL_SPLEN         (1UL << 31)                                /* Split enable */

/* HCHINTSTS (0x508 + n0x20) ************************************************/
#define N32_USBHS_HCHINTSTS_TXCFIF      (1UL << 0)  /* Transfer completed */
#define N32_USBHS_HCHINTSTS_CHHTDIF     (1UL << 1)  /* Channel halted */
#define N32_USBHS_HCHINTSTS_AHBERRIF    (1UL << 2)  /* AHB error */
#define N32_USBHS_HCHINTSTS_STALLIF     (1UL << 3)  /* STALL response received */
#define N32_USBHS_HCHINTSTS_NAKIF       (1UL << 4)  /* NAK response received */
#define N32_USBHS_HCHINTSTS_ACKIF       (1UL << 5)  /* ACK response received/transmitted */
#define N32_USBHS_HCHINTSTS_NYETIF      (1UL << 6)  /* NYET response received */
#define N32_USBHS_HCHINTSTS_TXERRIF     (1UL << 7)  /* Transaction error */
#define N32_USBHS_HCHINTSTS_BBERRIF     (1UL << 8)  /* Babble error */
#define N32_USBHS_HCHINTSTS_FOVRIF      (1UL << 9)  /* Frame overrun */
#define N32_USBHS_HCHINTSTS_DTERRIF     (1UL << 10) /* Data toggle error */

/* HCHINTEN (0x50C + n0x20) *************************************************/
#define N32_USBHS_HCHINTEN_TXCIEN       (1UL << 0)  /* Transfer completed interrupt enable */
#define N32_USBHS_HCHINTEN_CHHTDIEN     (1UL << 1)  /* Channel halted interrupt enable */
#define N32_USBHS_HCHINTEN_AHBERRIEN    (1UL << 2)  /* AHB error interrupt enable */
#define N32_USBHS_HCHINTEN_STALLIEN     (1UL << 3)  /* STALL response interrupt enable */
#define N32_USBHS_HCHINTEN_NAKIEN       (1UL << 4)  /* NAK response interrupt enable */
#define N32_USBHS_HCHINTEN_ACKIEN       (1UL << 5)  /* ACK response interrupt enable */
#define N32_USBHS_HCHINTEN_NYETIEN      (1UL << 6)  /* NYET response interrupt enable */
#define N32_USBHS_HCHINTEN_TXERRIEN     (1UL << 7)  /* Transaction error interrupt enable */
#define N32_USBHS_HCHINTEN_BBERRIEN     (1UL << 8)  /* Babble error interrupt enable */
#define N32_USBHS_HCHINTEN_FOVRIEN      (1UL << 9)  /* Frame overrun interrupt enable */
#define N32_USBHS_HCHINTEN_DTERRIEN     (1UL << 10) /* Data toggle error interrupt enable */

/* HCHTXSIZ (0x510 + n0x20) *************************************************/
#define N32_USBHS_HCHTXSIZ_TXSIZ_SHIFT  0
#define N32_USBHS_HCHTXSIZ_TXSIZ_MASK   (0x7FFFFUL << N32_USBHS_HCHTXSIZ_TXSIZ_SHIFT) /* Transfer size */
#define N32_USBHS_HCHTXSIZ_PKCNT_SHIFT  19
#define N32_USBHS_HCHTXSIZ_PKCNT_MASK   (0x3FFUL << N32_USBHS_HCHTXSIZ_PKCNT_SHIFT)   /* Packet count */
#define N32_USBHS_HCHTXSIZ_PID_SHIFT    29
#define N32_USBHS_HCHTXSIZ_PID_MASK     (3UL << N32_USBHS_HCHTXSIZ_PID_SHIFT)         /* Data PID */
#  define N32_USBHS_HCHTXSIZ_PID_0      (0UL << N32_USBHS_HCHTXSIZ_PID_SHIFT)         /* Data0 */
#  define N32_USBHS_HCHTXSIZ_PID_2      (1UL << N32_USBHS_HCHTXSIZ_PID_SHIFT)         /* Data2 */
#  define N32_USBHS_HCHTXSIZ_PID_1      (2UL << N32_USBHS_HCHTXSIZ_PID_SHIFT)         /* Data1 */
#  define N32_USBHS_HCHTXSIZ_PID_M_S    (3UL << N32_USBHS_HCHTXSIZ_PID_SHIFT)         /* MData/Setup */
#define N32_USBHS_HCHTXSIZ_DPING        (1UL << 31)                                   /* PING token request */

/* HCHDMADD (0x514 + n0x20) *************************************************/
#define N32_USBHS_HCHDMADD_MASK         (0xFFFFFFFFUL)                                /* DMA address */

/* ==========================================================================
 * Device Mode Registers
 * ==========================================================================
 */

/* DCFG (0x800) *************************************************************/
#define N32_USBHS_DCFG_DEVSPD_SHIFT     0
#define N32_USBHS_DCFG_DEVSPD_MASK      (3UL << N32_USBHS_DCFG_DEVSPD_SHIFT) /* Device speed (00:High speed, 01:Full speed) */
#define N32_USBHS_DCFG_NZLSOUTHSK       (1UL << 2)                           /* Non-zero-length status OUT handshake */
#define N32_USBHS_DCFG_DEVARR_SHIFT     4
#define N32_USBHS_DCFG_DEVARR_MASK      (0x7FUL << N32_USBHS_DCFG_DEVARR_SHIFT) /* Device address */
#define N32_USBHS_DCFG_PFRITVL_SHIFT    11
#define N32_USBHS_DCFG_PFRITVL_MASK     (3UL << N32_USBHS_DCFG_PFRITVL_SHIFT)   /* Periodic frame interval */
#  define N32_USBHS_DCFG_PFRITVL_80     (0UL << N32_USBHS_DCFG_PFRITVL_SHIFT)   /* 80% frame interval */
#  define N32_USBHS_DCFG_PFRITVL_85     (1UL << N32_USBHS_DCFG_PFRITVL_SHIFT)   /* 85% frame interval */
#  define N32_USBHS_DCFG_PFRITVL_90     (2UL << N32_USBHS_DCFG_PFRITVL_SHIFT)   /* 90% frame interval */
#  define N32_USBHS_DCFG_PFRITVL_95     (3UL << N32_USBHS_DCFG_PFRITVL_SHIFT)   /* 95% frame interval */
#define N32_USBHS_DCFG_XCVDRLY          (1UL << 14)                             /* Transceiver delay */
#define N32_USBHS_DCFG_EERRAIEN         (1UL << 15)                             /* Erratic error interrupt enable */
#define N32_USBHS_DCFG_PSITVL_SHIFT     24
#define N32_USBHS_DCFG_PSITVL_MASK      (3UL << N32_USBHS_DCFG_PSITVL_SHIFT)    /* Periodic schedule interval */
#  define N32_USBHS_DCFG_PSITVL_25      (0UL << N32_USBHS_DCFG_PSITVL_SHIFT)    /* 25% frame interval */
#  define N32_USBHS_DCFG_PSITVL_50      (1UL << N32_USBHS_DCFG_PSITVL_SHIFT)    /* 50% frame interval */
#  define N32_USBHS_DCFG_PSITVL_75      (2UL << N32_USBHS_DCFG_PSITVL_SHIFT)    /* 75% frame interval */

/* DCTRL (0x804) ************************************************************/
#define N32_USBHS_DCTRL_RMWKUP           (1UL << 0)  /* Remote wakeup signaling */
#define N32_USBHS_DCTRL_SFTDIS           (1UL << 1)  /* Soft disconnect */
#define N32_USBHS_DCTRL_GINAKSTS         (1UL << 2)  /* Global IN NAK status */
#define N32_USBHS_DCTRL_GONAKSTS         (1UL << 3)  /* Global OUT NAK status */
#define N32_USBHS_DCTRL_TSCTRL_SHIFT     4
#define N32_USBHS_DCTRL_TSCTRL_MASK      (7UL << N32_USBHS_DCTRL_TSCTRL_SHIFT) /* Test control */
#  define N32_USBHS_DCTRL_TSCTRL_TEST_NO (0UL << N32_USBHS_DCTRL_TSCTRL_SHIFT) /* No test */
#  define N32_USBHS_DCTRL_TSCTRL_TEST_J  (1UL << N32_USBHS_DCTRL_TSCTRL_SHIFT) /* Test_J mode */
#  define N32_USBHS_DCTRL_TSCTRL_TEST_K  (2UL << N32_USBHS_DCTRL_TSCTRL_SHIFT) /* Test_K mode */
#  define N32_USBHS_DCTRL_TSCTRL_TEST_SN (3UL << N32_USBHS_DCTRL_TSCTRL_SHIFT) /* Test_SE0_NAK mode */
#  define N32_USBHS_DCTRL_TSCTRL_TEST_PK (4UL << N32_USBHS_DCTRL_TSCTRL_SHIFT) /* Test_Packet mode */
#  define N32_USBHS_DCTRL_TSCTRL_TEST_FE (5UL << N32_USBHS_DCTRL_TSCTRL_SHIFT) /* Test forced enable */
#define N32_USBHS_DCTRL_SGINAK           (1UL << 7)                            /* Set global IN NAK */
#define N32_USBHS_DCTRL_CGNPINAK         (1UL << 8)                            /* Clear global IN NAK */
#define N32_USBHS_DCTRL_SGONAK           (1UL << 9)                            /* Set global OUT NAK */
#define N32_USBHS_DCTRL_CGONAK           (1UL << 10)                           /* Clear global OUT NAK */
#define N32_USBHS_DCTRL_POPDNE           (1UL << 11)                           /* Power-on programming done */
#define N32_USBHS_DCTRL_NAKOBBLE         (1UL << 16)                           /* NAK on babble error */

/* DSTS (0x808) *************************************************************/
#define N32_USBHS_DSTS_SUSPF            (1UL << 0)                            /* Suspend status */
#define N32_USBHS_DSTS_ENUMSPD_SHIFT    1
#define N32_USBHS_DSTS_ENUMSPD_MASK     (3UL << N32_USBHS_DSTS_ENUMSPD_SHIFT) /* Enumerated speed (00:High speed, 01:Full speed, 10:Low speed, 11:Low speed) */
#define N32_USBHS_DSTS_ERERRF           (1UL << 3)                            /* Erratic error */
#define N32_USBHS_DSTS_SOFFN_SHIFT      8
#define N32_USBHS_DSTS_SOFFN_MASK       (0x3FFFUL << N32_USBHS_DSTS_SOFFN_SHIFT)    /* SOF frame number */
#define N32_USBHS_DSTS_DEVLINSTS_SHIFT  22
#define N32_USBHS_DSTS_DEVLINSTS_MASK   (3UL << N32_USBHS_DSTS_DEVLINSTS_SHIFT)     /* Device line status */
#  define N32_USBHS_DSTS_DEVLINSTS_DM0DP0   (0UL << N32_USBHS_DSTS_DEVLINSTS_SHIFT) /* DM=0, DP=0 */
#  define N32_USBHS_DSTS_DEVLINSTS_DM0DP1   (1UL << N32_USBHS_DSTS_DEVLINSTS_SHIFT) /* DM=0, DP=1 */
#  define N32_USBHS_DSTS_DEVLINSTS_DM1DP0   (2UL << N32_USBHS_DSTS_DEVLINSTS_SHIFT) /* DM=1, DP=0 */

/* DINEPINTEN (0x810) *******************************************************/
#define N32_USBHS_DINEPINTEN_TXCIEN         (1UL << 0)  /* Transfer completed interrupt enable */
#define N32_USBHS_DINEPINTEN_EPDIEN         (1UL << 1)  /* Endpoint disabled interrupt enable */
#define N32_USBHS_DINEPINTEN_AHBERRIEN      (1UL << 2)  /* AHB error interrupt enable */
#define N32_USBHS_DINEPINTEN_TOIEN          (1UL << 3)  /* Timeout interrupt enable */
#define N32_USBHS_DINEPINTEN_TXFERINTKIEN   (1UL << 4)  /* IN token when TxFIFO empty */
#define N32_USBHS_DINEPINTEN_INTREPMISIEN   (1UL << 5)  /* IN token with EP mismatch */
#define N32_USBHS_DINEPINTEN_INEPNAKEIEN    (1UL << 6)  /* IN endpoint NAK effective */
#define N32_USBHS_DINEPINTEN_TXFUDIEN       (1UL << 8)  /* Tx FIFO underrun interrupt enable */
#define N32_USBHS_DINEPINTEN_NAKIEN         (1UL << 13) /* NAK interrupt enable */

/* DOUTEPINTEN (0x814) ******************************************************/
#define N32_USBHS_DOUTEPINTEN_TXCIEN        (1UL << 0)  /* Transfer completed interrupt enable */
#define N32_USBHS_DOUTEPINTEN_EPDIEN        (1UL << 1)  /* Endpoint disabled interrupt enable */
#define N32_USBHS_DOUTEPINTEN_AHBERRIEN     (1UL << 2)  /* AHB error interrupt enable */
#define N32_USBHS_DOUTEPINTEN_STUPDNEIEN    (1UL << 3)  /* SETUP phase done interrupt enable */
#define N32_USBHS_DOUTEPINTEN_EPDISROTIEN   (1UL << 4)  /* OUT token when EP disabled */
#define N32_USBHS_DOUTEPINTEN_B2BSTUPIEN    (1UL << 6)  /* Back-to-back SETUP packets */
#define N32_USBHS_DOUTEPINTEN_OPERRIEN      (1UL << 8)  /* OUT packet error interrupt enable */
#define N32_USBHS_DOUTEPINTEN_BERRIEN       (1UL << 12) /* Babble error interrupt enable */
#define N32_USBHS_DOUTEPINTEN_NAKIEN        (1UL << 13) /* NAK interrupt enable */
#define N32_USBHS_DOUTEPINTEN_NYETIEN       (1UL << 14) /* NYET interrupt enable */

/* DAEPINTSTS (0x818) *******************************************************/
#define N32_USBHS_DAEPINTSTS_INEPINT_SHIFT  0
#define N32_USBHS_DAEPINTSTS_INEPINT_MASK   (0x1FFUL << N32_USBHS_DAEPINTSTS_INEPINT_SHIFT)  /* IN EP bits 0..8 */
#define N32_USBHS_DAEPINTSTS_OUTEPINT_SHIFT 16
#define N32_USBHS_DAEPINTSTS_OUTEPINT_MASK  (0x1FFUL << N32_USBHS_DAEPINTSTS_OUTEPINT_SHIFT) /* OUT EP bits 0..8 */

/* DAEPINTEN (0x81C) ********************************************************/
#define N32_USBHS_DAEPINTEN_INEPIEN_SHIFT  0
#define N32_USBHS_DAEPINTEN_INEPIEN_MASK   (0x1FFUL << N32_USBHS_DAEPINTEN_INEPIEN_SHIFT)
#define N32_USBHS_DAEPINTEN_OUTEPIEN_SHIFT 16
#define N32_USBHS_DAEPINTEN_OUTEPIEN_MASK  (0x1FFUL << N32_USBHS_DAEPINTEN_OUTEPIEN_SHIFT)

/* DTHRCTRL (0x830) *********************************************************/
#define N32_USBHS_DTHRCTRL_NISOINEPTHREN  (1UL << 0)  /* Non-isochronous IN EP threshold enable */
#define N32_USBHS_DTHRCTRL_ISOINEPTHREN   (1UL << 1)  /* Isochronous IN EP threshold enable */
#define N32_USBHS_DTHRCTRL_TXTHRLEN_SHIFT 2
#define N32_USBHS_DTHRCTRL_TXTHRLEN_MASK  (0x1FFUL << N32_USBHS_DTHRCTRL_TXTHRLEN_SHIFT) /* Tx threshold length */
#define N32_USBHS_DTHRCTRL_RXTHREN        (1UL << 16)                                    /* Receive threshold enable */
#define N32_USBHS_DTHRCTRL_RXTHRLEN_SHIFT 17
#define N32_USBHS_DTHRCTRL_RXTHRLEN_MASK  (0x1FFUL << N32_USBHS_DTHRCTRL_RXTHRLEN_SHIFT) /* Rx threshold length */
#define N32_USBHS_DTHRCTRL_ARPEN          (1UL << 27)                                    /* Arbiter register enable */

/* DINEPFEINTEN (0x834) *****************************************************/
#define N32_USBHS_DINEPFEINTEN_INEPTXFEIEN_MASK (0xFFFFUL)                               /* IN EP Tx FIFO empty interrupt enable */

/* DEEPINTSTS (0x838) *******************************************************/
#define N32_USBHS_DEEPINTSTS_INEPINT_SHIFT  0
#define N32_USBHS_DEEPINTSTS_INEPINT_MASK   (0xFFUL << N32_USBHS_DEEPINTSTS_INEPINT_SHIFT)  /* IN EP 0..7 */
#define N32_USBHS_DEEPINTSTS_OUTEPINT_SHIFT 16
#define N32_USBHS_DEEPINTSTS_OUTEPINT_MASK  (0xFFUL << N32_USBHS_DEEPINTSTS_OUTEPINT_SHIFT) /* OUT EP 0..7 */

/* DEEPINTEN (0x83C) ********************************************************/
#define N32_USBHS_DEEPINTEN_INEPIEN_SHIFT  0
#define N32_USBHS_DEEPINTEN_INEPIEN_MASK   (0xFFUL << N32_USBHS_DEEPINTEN_INEPIEN_SHIFT)
#define N32_USBHS_DEEPINTEN_OUTEPIEN_SHIFT 16
#define N32_USBHS_DEEPINTEN_OUTEPIEN_MASK  (0xFFUL << N32_USBHS_DEEPINTEN_OUTEPIEN_SHIFT)

/* DINEPXINTEN (0x840 + x4) (x=0..8) ****************************************/
#define N32_USBHS_DINEPXINTEN_TXCIEN        (1UL << 0)  /* Transfer completed interrupt enable */
#define N32_USBHS_DINEPXINTEN_EPDISIEN      (1UL << 1)  /* Endpoint disabled interrupt enable */
#define N32_USBHS_DINEPXINTEN_AHBERRIEN     (1UL << 2)  /* AHB error interrupt enable */
#define N32_USBHS_DINEPXINTEN_TOIEN         (1UL << 3)  /* Timeout interrupt enable */
#define N32_USBHS_DINEPXINTEN_TXFERINTKIEN  (1UL << 4)  /* IN token when TxFIFO empty */
#define N32_USBHS_DINEPXINTEN_INTREPMISIEN  (1UL << 5)  /* EP mismatch interrupt enable */
#define N32_USBHS_DINEPXINTEN_INEPNAKEIEN   (1UL << 6)  /* IN EP NAK effective interrupt enable */
#define N32_USBHS_DINEPXINTEN_TXFUDIEN      (1UL << 8)  /* Tx FIFO underrun interrupt enable */
#define N32_USBHS_DINEPXINTEN_NAKIEN        (1UL << 13) /* NAK interrupt enable */

/* DOUTEPXINTEN (0x880 + x4) (x=0..8) ***************************************/
#define N32_USBHS_DOUTEPXINTEN_TXCIEN       (1UL << 0)  /* Transfer completed interrupt enable */
#define N32_USBHS_DOUTEPXINTEN_EPDIEN       (1UL << 1)  /* Endpoint disabled interrupt enable */
#define N32_USBHS_DOUTEPXINTEN_AHBERRIEN    (1UL << 2)  /* AHB error interrupt enable */
#define N32_USBHS_DOUTEPXINTEN_STUPDNEIEN   (1UL << 3)  /* SETUP phase done interrupt enable */
#define N32_USBHS_DOUTEPXINTEN_EPDISROTIEN  (1UL << 4)  /* OUT token when EP disabled */
#define N32_USBHS_DOUTEPXINTEN_B2BSTUPIEN   (1UL << 6)  /* Back-to-back SETUP packets */
#define N32_USBHS_DOUTEPXINTEN_OPERRIEN     (1UL << 8)  /* OUT packet error interrupt enable */
#define N32_USBHS_DOUTEPXINTEN_BERRIEN      (1UL << 12) /* Babble error interrupt enable */
#define N32_USBHS_DOUTEPXINTEN_NAKIEN       (1UL << 13) /* NAK interrupt enable */
#define N32_USBHS_DOUTEPXINTEN_NYETIEN      (1UL << 14) /* NYET interrupt enable */

/* ==========================================================================
 * Device IN Endpoint Registers (0x900 + ep*0x20)
 * ==========================================================================
 */

/* DINEPCTRL (EP0) **********************************************************/
#define N32_USBHS_DINEP0CTRL_MPLEN_SHIFT   0
#define N32_USBHS_DINEP0CTRL_MPLEN_MASK    (3UL << N32_USBHS_DINEP0CTRL_MPLEN_SHIFT) /* Max packet size */
#  define N32_USBHS_DINEP0CTRL_MPLEN_64    (0UL << N32_USBHS_DINEP0CTRL_MPLEN_SHIFT) /* 64 bytes */
#  define N32_USBHS_DINEP0CTRL_MPLEN_32    (1UL << N32_USBHS_DINEP0CTRL_MPLEN_SHIFT) /* 32 bytes */
#  define N32_USBHS_DINEP0CTRL_MPLEN_16    (2UL << N32_USBHS_DINEP0CTRL_MPLEN_SHIFT) /* 16 bytes */
#  define N32_USBHS_DINEP0CTRL_MPLEN_8     (3UL << N32_USBHS_DINEP0CTRL_MPLEN_SHIFT) /* 8 bytes */
#define N32_USBHS_DINEP0CTRL_EPACT         (1UL << 15)                               /* USB active endpoint */
#define N32_USBHS_DINEP0CTRL_NAKSTS        (1UL << 17)                               /* NAK status */
#define N32_USBHS_DINEP0CTRL_EPTYPE_MASK   (3UL << 18)                               /* Endpoint type (must be 00:CTRL) */
#define N32_USBHS_DINEP0CTRL_STALL         (1UL << 21)                               /* STALL handshake */
#define N32_USBHS_DINEP0CTRL_TXFNUM_SHIFT  22
#define N32_USBHS_DINEP0CTRL_TXFNUM_MASK   (0xFUL << N32_USBHS_DINEP0CTRL_TXFNUM_SHIFT) /* Tx FIFO number */
#define N32_USBHS_DINEP0CTRL_CNAK          (1UL << 26)                                  /* Clear NAK */
#define N32_USBHS_DINEP0CTRL_SNAK          (1UL << 27)                                  /* Set NAK */
#define N32_USBHS_DINEP0CTRL_EPDIS         (1UL << 30)                                  /* Endpoint disable */
#define N32_USBHS_DINEP0CTRL_EPEN          (1UL << 31)                                  /* Endpoint enable */

/* DINEPCTRL (EP1..8) *******************************************************/
#define N32_USBHS_DINEPCTRL_MPLEN_SHIFT    0
#define N32_USBHS_DINEPCTRL_MPLEN_MASK     (0x7FFUL << N32_USBHS_DINEPCTRL_MPLEN_SHIFT) /* Max packet size */
#define N32_USBHS_DINEPCTRL_EPACT          (1UL << 15)                                  /* USB active endpoint */
#define N32_USBHS_DINEPCTRL_EPDPID_EPEOFRM (1UL << 16)                                  /* Data PID / Even/odd frame */
#define N32_USBHS_DINEPCTRL_NAKSTS         (1UL << 17)                                  /* NAK status */
#define N32_USBHS_DINEPCTRL_EPTYPE_SHIFT   18
#define N32_USBHS_DINEPCTRL_EPTYPE_MASK    (3UL << N32_USBHS_DINEPCTRL_EPTYPE_SHIFT)    /* Endpoint type */
#  define N32_USBHS_DINEPCTRL_EPTYPE_CTRL  (0UL << N32_USBHS_DINEPCTRL_EPTYPE_SHIFT)    /* CTRL */
#  define N32_USBHS_DINEPCTRL_EPTYPE_ISOC  (1UL << N32_USBHS_DINEPCTRL_EPTYPE_SHIFT)    /* ISOC */
#  define N32_USBHS_DINEPCTRL_EPTYPE_BULK  (2UL << N32_USBHS_DINEPCTRL_EPTYPE_SHIFT)    /* BULK */
#  define N32_USBHS_DINEPCTRL_EPTYPE_INTR  (3UL << N32_USBHS_DINEPCTRL_EPTYPE_SHIFT)    /* INTR */
#define N32_USBHS_DINEPCTRL_STALL          (1UL << 21)                                  /* STALL handshake */
#define N32_USBHS_DINEPCTRL_TXFNUM_SHIFT   22
#define N32_USBHS_DINEPCTRL_TXFNUM_MASK    (0xFUL << N32_USBHS_DINEPCTRL_TXFNUM_SHIFT)  /* Tx FIFO number */
#define N32_USBHS_DINEPCTRL_CNAK           (1UL << 26)                                  /* Clear NAK */
#define N32_USBHS_DINEPCTRL_SNAK           (1UL << 27)                                  /* Set NAK */
#define N32_USBHS_DINEPCTRL_SD0PID_SEVNFRM (1UL << 28)                                  /* Set DATA0 / Even frame */
#define N32_USBHS_DINEPCTRL_SD1PID_SODDFRM (1UL << 29)                                  /* Set DATA1 / Odd frame */
#define N32_USBHS_DINEPCTRL_EPDIS          (1UL << 30)                                  /* Endpoint disable */
#define N32_USBHS_DINEPCTRL_EPEN           (1UL << 31)                                  /* Endpoint enable */

/* DINEPINTSTS (0x908 + ep0x20) *********************************************/
#define N32_USBHS_DINEPINTSTS_TXCIF        (1UL << 0)  /* Transfer completed interrupt */
#define N32_USBHS_DINEPINTSTS_EPDISIF      (1UL << 1)  /* Endpoint disabled interrupt */
#define N32_USBHS_DINEPINTSTS_AHBERRIF     (1UL << 2)  /* AHB error interrupt */
#define N32_USBHS_DINEPINTSTS_TOUTIF       (1UL << 3)  /* Timeout condition */
#define N32_USBHS_DINEPINTSTS_TXFERINTIF   (1UL << 4)  /* IN token when TxFIFO empty */
#define N32_USBHS_DINEPINTSTS_INEPMISIF    (1UL << 5)  /* IN endpoint mismatch */
#define N32_USBHS_DINEPINTSTS_INEPNAKEIF   (1UL << 6)  /* IN endpoint NAK effective */
#define N32_USBHS_DINEPINTSTS_TXFEIF       (1UL << 7)  /* Transmit FIFO empty */
#define N32_USBHS_DINEPINTSTS_TXFUDRIF     (1UL << 8)  /* Tx FIFO underrun */
#define N32_USBHS_DINEPINTSTS_PKDRPSTS     (1UL << 11) /* Packet dropped status */
#define N32_USBHS_DINEPINTSTS_BBERRIF      (1UL << 12) /* Babble error */
#define N32_USBHS_DINEPINTSTS_NAKIF        (1UL << 13) /* NAK interrupt */
#define N32_USBHS_DINEPINTSTS_NYETIF       (1UL << 14) /* NYET interrupt */

/* DINEPTXSIZ (0x910 + ep0x20) EP0 vs others ********************************/

/* EP0 */
#define N32_USBHS_DINEP0TXSIZ_TLEN_SHIFT   0
#define N32_USBHS_DINEP0TXSIZ_TLEN_MASK    (0x7FUL << N32_USBHS_DINEP0TXSIZ_TLEN_SHIFT) /* Transfer size */
#define N32_USBHS_DINEP0TXSIZ_PKTCNT_SHIFT 19
#define N32_USBHS_DINEP0TXSIZ_PKTCNT_MASK  (3UL << N32_USBHS_DINEP0TXSIZ_PKTCNT_SHIFT)  /* Packet count */

/* EP1..8 */
#define N32_USBHS_DINEPTXSIZ_TLEN_SHIFT    0
#define N32_USBHS_DINEPTXSIZ_TLEN_MASK     (0x7FFFFUL << N32_USBHS_DINEPTXSIZ_TLEN_SHIFT)
#define N32_USBHS_DINEPTXSIZ_PKTCNT_SHIFT  19
#define N32_USBHS_DINEPTXSIZ_PKTCNT_MASK   (0x3FFUL << N32_USBHS_DINEPTXSIZ_PKTCNT_SHIFT)
#define N32_USBHS_DINEPTXSIZ_MCNT_SHIFT    29
#define N32_USBHS_DINEPTXSIZ_MCNT_MASK     (3UL << N32_USBHS_DINEPTXSIZ_MCNT_SHIFT)     /* Multi count */

/* DINEPDMADD (0x914 + ep0x20) **********************************************/
#define N32_USBHS_DINEPDMADD_MASK          (0xFFFFFFFFUL)                               /* DMA address */

/* DINEPTXFSTS (0x918 + ep0x20) *********************************************/
#define N32_USBHS_DINEPTXFSTS_TXFSPCAVL_SHIFT 0
#define N32_USBHS_DINEPTXFSTS_TXFSPCAVL_MASK (0xFFFFUL << N32_USBHS_DINEPTXFSTS_TXFSPCAVL_SHIFT)

/* ==========================================================================
 * Device OUT Endpoint Registers (0xB00 + ep*0x20)
 * ==========================================================================
 */

/* DOUTEPCTRL (EP0) *********************************************************/
#define N32_USBHS_DOUTEP0CTRL_MPLEN_SHIFT  0
#define N32_USBHS_DOUTEP0CTRL_MPLEN_MASK   (3UL << N32_USBHS_DOUTEP0CTRL_MPLEN_SHIFT) /* Max packet size */
#  define N32_USBHS_DOUTEP0CTRL_MPLEN_64   (0UL << N32_USBHS_DOUTEP0CTRL_MPLEN_SHIFT) /* 64 bytes */
#  define N32_USBHS_DOUTEP0CTRL_MPLEN_32   (1UL << N32_USBHS_DOUTEP0CTRL_MPLEN_SHIFT) /* 32 bytes */
#  define N32_USBHS_DOUTEP0CTRL_MPLEN_16   (2UL << N32_USBHS_DOUTEP0CTRL_MPLEN_SHIFT) /* 16 bytes */
#  define N32_USBHS_DOUTEP0CTRL_MPLEN_8    (3UL << N32_USBHS_DOUTEP0CTRL_MPLEN_SHIFT) /* 8 bytes */
#define N32_USBHS_DOUTEP0CTRL_EPACT        (1UL << 15)                                /* USB active endpoint */
#define N32_USBHS_DOUTEP0CTRL_NAKSTS       (1UL << 17)                                /* NAK status */
#define N32_USBHS_DOUTEP0CTRL_EPTYPE_MASK  (3UL << 18)                                /* Endpoint type (must be 00:CTRL) */
#define N32_USBHS_DOUTEP0CTRL_STALL        (1UL << 21)                                /* STALL handshake */
#define N32_USBHS_DOUTEP0CTRL_CNAK         (1UL << 26)                                /* Clear NAK */
#define N32_USBHS_DOUTEP0CTRL_SNAK         (1UL << 27)                                /* Set NAK */
#define N32_USBHS_DOUTEP0CTRL_EPDIS        (1UL << 30)                                /* Endpoint disable */
#define N32_USBHS_DOUTEP0CTRL_EPEN         (1UL << 31)                                /* Endpoint enable */

/* DOUTEPCTRL (EP1..8) ******************************************************/
#define N32_USBHS_DOUTEPCTRL_MPLEN_SHIFT   0
#define N32_USBHS_DOUTEPCTRL_MPLEN_MASK    (0x7FFUL << N32_USBHS_DOUTEPCTRL_MPLEN_SHIFT) /* Max packet size */
#define N32_USBHS_DOUTEPCTRL_EPACT         (1UL << 15)                                   /* USB active endpoint */
#define N32_USBHS_DOUTEPCTRL_EPDPID_EPEOFRM (1UL << 16)                                  /* Data PID / Even/odd frame */
#define N32_USBHS_DOUTEPCTRL_NAKSTS        (1UL << 17)                                   /* NAK status */
#define N32_USBHS_DOUTEPCTRL_EPTYPE_SHIFT  18
#define N32_USBHS_DOUTEPCTRL_EPTYPE_MASK   (3UL << N32_USBHS_DOUTEPCTRL_EPTYPE_SHIFT)    /* Endpoint type */
#  define N32_USBHS_DOUTEPCTRL_EPTYPE_CTRL (0UL << N32_USBHS_DOUTEPCTRL_EPTYPE_SHIFT)    /* CTRL */
#  define N32_USBHS_DOUTEPCTRL_EPTYPE_ISOC (1UL << N32_USBHS_DOUTEPCTRL_EPTYPE_SHIFT)    /* ISOOC */
#  define N32_USBHS_DOUTEPCTRL_EPTYPE_BULK (2UL << N32_USBHS_DOUTEPCTRL_EPTYPE_SHIFT)    /* BULK */
#  define N32_USBHS_DOUTEPCTRL_EPTYPE_INTR (3UL << N32_USBHS_DOUTEPCTRL_EPTYPE_SHIFT)    /* INTR */
#define N32_USBHS_DOUTEPCTRL_STALL         (1UL << 21)                                   /* STALL handshake */
#define N32_USBHS_DOUTEPCTRL_CNAK          (1UL << 26)                                   /* Clear NAK */
#define N32_USBHS_DOUTEPCTRL_SNAK          (1UL << 27)                                   /* Set NAK */
#define N32_USBHS_DOUTEPCTRL_SD0PID_SEVNFRM (1UL << 28)                                  /* Set DATA0 / Even frame */
#define N32_USBHS_DOUTEPCTRL_SD1PID_SODDFRM (1UL << 29)                                  /* Set DATA1 / Odd frame */
#define N32_USBHS_DOUTEPCTRL_EPDIS         (1UL << 30)                                   /* Endpoint disable */
#define N32_USBHS_DOUTEPCTRL_EPEN          (1UL << 31)                                   /* Endpoint enable */

/* DOUTEPINTSTS (0xB08 + ep0x20) ********************************************/
#define N32_USBHS_DOUTEPINTSTS_TXCIF       (1UL << 0)   /* Transfer completed interrupt */
#define N32_USBHS_DOUTEPINTSTS_EPDISIF     (1UL << 1)   /* Endpoint disabled interrupt */
#define N32_USBHS_DOUTEPINTSTS_AHBERRIF    (1UL << 2)   /* AHB error interrupt */
#define N32_USBHS_DOUTEPINTSTS_STUPPDNEIF  (1UL << 3)   /* SETUP phase done */
#define N32_USBHS_DOUTEPINTSTS_OUTTRXEPDISIF (1UL << 4) /* OUT token when EP disabled */
#define N32_USBHS_DOUTEPINTSTS_STSPRXIF    (1UL << 5)   /* Status phase received */
#define N32_USBHS_DOUTEPINTSTS_B2BSTUPRIF  (1UL << 6)   /* Back-to-back SETUP packets */
#define N32_USBHS_DOUTEPINTSTS_OUTPCKERRIF (1UL << 8)   /* OUT packet error */
#define N32_USBHS_DOUTEPINTSTS_PKDRPSTS    (1UL << 11)  /* Packet dropped status */
#define N32_USBHS_DOUTEPINTSTS_BBERRIF     (1UL << 12)  /* Babble error */
#define N32_USBHS_DOUTEPINTSTS_NAKIF       (1UL << 13)  /* NAK interrupt */
#define N32_USBHS_DOUTEPINTSTS_NYETIF      (1UL << 14)  /* NYET interrupt */
#define N32_USBHS_DOUTEPINTSTS_STUPPRXIF   (1UL << 15)  /* SETUP packet received (DMA mode) */

/* DOUTEPTXSIZ (0xB10 + ep0x20) EP0 vs others *******************************/

/* EP0 */
#define N32_USBHS_DOUTEP0TXSIZ_TLEN_SHIFT     0
#define N32_USBHS_DOUTEP0TXSIZ_TLEN_MASK      (0x7FUL << N32_USBHS_DOUTEP0TXSIZ_TLEN_SHIFT)  /* Transfer size */
#define N32_USBHS_DOUTEP0TXSIZ_PKTCNT_SHIFT   19
#define N32_USBHS_DOUTEP0TXSIZ_PKTCNT_MASK    (1UL << N32_USBHS_DOUTEP0TXSIZ_PKTCNT_SHIFT)   /* Packet count */
#define N32_USBHS_DOUTEP0TXSIZ_STUPPCNT_SHIFT 29
#define N32_USBHS_DOUTEP0TXSIZ_STUPPCNT_MASK  (3UL << N32_USBHS_DOUTEP0TXSIZ_STUPPCNT_SHIFT) /* SETUP packet count */

/* EP1..8 */
#define N32_USBHS_DOUTEPTXSIZ_TLEN_SHIFT            0
#define N32_USBHS_DOUTEPTXSIZ_TLEN_MASK             (0x7FFFFUL << N32_USBHS_DOUTEPTXSIZ_TLEN_SHIFT)
#define N32_USBHS_DOUTEPTXSIZ_PKTCNT_SHIFT          19
#define N32_USBHS_DOUTEPTXSIZ_PKTCNT_MASK           (0x3FFUL << N32_USBHS_DOUTEPTXSIZ_PKTCNT_SHIFT)
#define N32_USBHS_DOUTEPTXSIZ_STUPPCNT_RXDPID_SHIFT 29
#define N32_USBHS_DOUTEPTXSIZ_STUPPCNT_RXDPID_MASK  (3UL << N32_USBHS_DOUTEPTXSIZ_STUPPCNT_RXDPID_SHIFT) /* SETUP count / RX Data PID */
#  define N32_USBHS_DOUTEPTXSIZ_RXDPID_0            (0UL << N32_USBHS_DOUTEPTXSIZ_STUPPCNT_RXDPID_SHIFT) /* Data0 */
#  define N32_USBHS_DOUTEPTXSIZ_RXDPID_2            (1UL << N32_USBHS_DOUTEPTXSIZ_STUPPCNT_RXDPID_SHIFT) /* Data2 */
#  define N32_USBHS_DOUTEPTXSIZ_RXDPID_1            (2UL << N32_USBHS_DOUTEPTXSIZ_STUPPCNT_RXDPID_SHIFT) /* Data1 */
#  define N32_USBHS_DOUTEPTXSIZ_RXDPID_M            (3UL << N32_USBHS_DOUTEPTXSIZ_STUPPCNT_RXDPID_SHIFT) /* MData */

/* DOUTEPDMADD (0xB14 + ep0x20) *********************************************/
#define N32_USBHS_DOUTEPDMADD_MASK         (0xFFFFFFFFUL) /* DMA address */

/* ==========================================================================
 * Power Control Registers
 * ==========================================================================
 */

/* PWRCTRL (0xE00) **********************************************************/
#define N32_USBHS_PWRCTRL_PHYSTP          (1UL << 0)  /* Stop PHY clock */
#define N32_USBHS_PWRCTRL_GATEHCLK        (1UL << 1)  /* Gate HCLK */
#define N32_USBHS_PWRCTRL_PDMRST          (1UL << 3)  /* Reset power-down module */
#define N32_USBHS_PWRCTRL_PHYSLEEP        (1UL << 6)  /* PHY sleep */
#define N32_USBHS_PWRCTRL_DSLEEP          (1UL << 7)  /* Deep sleep */

/* PWRCTRL1 (0xE04) *********************************************************/
#define N32_USBHS_PWRCTRL1_GATEN          (1UL << 0)  /* Enable activity clock gating */
#define N32_USBHS_PWRCTRL1_CNT_SHIFT      1
#define N32_USBHS_PWRCTRL1_CNT_MASK       (3UL << N32_USBHS_PWRCTRL1_CNT_SHIFT) /* Gating clock count */
#  define N32_USBHS_PWRCTRL1_CNT_64       (0UL << N32_USBHS_PWRCTRL1_CNT_SHIFT) /* 64 clocks */
#  define N32_USBHS_PWRCTRL1_CNT_128      (1UL << N32_USBHS_PWRCTRL1_CNT_SHIFT) /* 128 clocks */
#define N32_USBHS_PWRCTRL1_RAMCLKEN       (1UL << 3)                            /* RAM clock gating enable */

/* ==========================================================================
 * Wrapper Registers (separate base)
 * ==========================================================================
 */

/* WRPCTRL (0x000) **********************************************************/
#define N32_USBHS_WRPCTRL_PINDETEN        (1UL << 16) /* Pin detection enable */
#define N32_USBHS_WRPCTRL_VBRMDETEN       (1UL << 17) /* VBUS removal detect enable */
#define N32_USBHS_WRPCTRL_HDISCEN         (1UL << 18) /* Host disconnect detect enable */
#define N32_USBHS_WRPCTRL_IDDETEN         (1UL << 19) /* ID detect enable */
#define N32_USBHS_WRPCTRL_SUSPWKEN        (1UL << 20) /* Suspend wake-up enable */
#define N32_USBHS_WRPCTRL_LSCHGEN         (1UL << 21) /* Line status change detection enable */

/* WRPCFG (0x004) ***********************************************************/
#define N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT  0
#define N32_USBHS_WRPCFG_PHYCLKSEL_MASK   (7UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* PHY clock select */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_10   (0UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 10MHz */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_12   (1UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 12MHz */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_25   (2UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 25MHz */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_30   (3UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 30MHz */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_19_2 (4UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 19.2MHz */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_24   (5UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 24MHz */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_27   (6UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 27MHz */
#  define N32_USBHS_WRPCFG_PHYCLKSEL_40   (7UL << N32_USBHS_WRPCFG_PHYCLKSEL_SHIFT) /* 40MHz */
#define N32_USBHS_WRPCFG_PLLEN            (1UL << 3)                                /* PLL enable */
#define N32_USBHS_WRPCFG_IDEN             (1UL << 9)                                /* ID pin enable */
#define N32_USBHS_WRPCFG_LSEN             (1UL << 10)                               /* Low-swing enable */
#define N32_USBHS_WRPCFG_SOFDEN           (1UL << 11)                               /* SOF detection enable */
#define N32_USBHS_WRPCFG_IDSIG            (1UL << 12)                               /* Internal ID signal (when IDEN=0) */

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_USBHS_H */
