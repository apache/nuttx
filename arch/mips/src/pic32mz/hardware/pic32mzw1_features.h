/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzw1_features.h
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

/* PIC32MZ-W1 does NOT use the classic PIC32MZ EC/EF DEVCFG0-3 fuse model.
 * Its boot-flash configuration words (FUSERID, DEVCFG4, DEVCFG2, DEVCFG1,
 * DEVCFG0, FBCFG0, FCPN0, FSIGN0 - note there is no DEVCFG3) are copied
 * into runtime registers at reset (DEVCFG0 -> CFGCON0, DEVCFG1 -> CFGCON1,
 * DEVCFG2 -> CFGCON2, DEVCFG4 -> CFGCON4, FBCFG0 -> BCFG0; DS70005425
 * Table 6-2 note 1).  Note that the data sheet numbers the boot-flash words
 * differently (BFDEVCFG0..5); the names here follow the DFP.
 *
 * The words themselves are emitted by pic32mz_head.S from the values
 * composed in pic32mz_config.h.  Register addresses and field layouts are
 * from the Microchip PIC32MZ-W_DFP (Apache-2.0), proc/pwfi32e01.h and
 * xc32/WFI32E01/configuration.data, cross-checked with DS70005425 chapters
 * 6 and 38.
 */

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_FEATURES_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_FEATURES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets (SFR PIC32MZ_CONFIG_K1BASE) *****************************/

#define PIC32MZ_CFGCON0_OFFSET   0x0000
#define PIC32MZ_CFGCON1_OFFSET   0x0010
#define PIC32MZ_CFGCON2_OFFSET   0x0020
#define PIC32MZ_CFGCON3_OFFSET   0x0030
#define PIC32MZ_DEVID_OFFSET     0x0060
#define PIC32MZ_SYSKEY_OFFSET    0x0080
#define PIC32MZ_PMD1_OFFSET      0x0090
#define PIC32MZ_PMD2_OFFSET      0x00a0
#define PIC32MZ_PMD3_OFFSET      0x00b0

/* Register Addresses *******************************************************/

#define PIC32MZ_CFGCON0          (PIC32MZ_CONFIG_K1BASE + PIC32MZ_CFGCON0_OFFSET)
#define PIC32MZ_CFGCON1          (PIC32MZ_CONFIG_K1BASE + PIC32MZ_CFGCON1_OFFSET)
#define PIC32MZ_CFGCON2          (PIC32MZ_CONFIG_K1BASE + PIC32MZ_CFGCON2_OFFSET)
#define PIC32MZ_CFGCON3          (PIC32MZ_CONFIG_K1BASE + PIC32MZ_CFGCON3_OFFSET)
#define PIC32MZ_DEVID            (PIC32MZ_CONFIG_K1BASE + PIC32MZ_DEVID_OFFSET)
#define PIC32MZ_SYSKEY           (PIC32MZ_CONFIG_K1BASE + PIC32MZ_SYSKEY_OFFSET)
#define PIC32MZ_PMD1             (PIC32MZ_CONFIG_K1BASE + PIC32MZ_PMD1_OFFSET)
#define PIC32MZ_PMD2             (PIC32MZ_CONFIG_K1BASE + PIC32MZ_PMD2_OFFSET)
#define PIC32MZ_PMD3             (PIC32MZ_CONFIG_K1BASE + PIC32MZ_PMD3_OFFSET)

/* CFGCON0 bitfields (identical layout to DEVCFG0) **************************/

#define CFGCON0_TDOEN            (1 << 0)
#define CFGCON0_TROEN            (1 << 2)
#define CFGCON0_JTAGEN           (1 << 3)
#define CFGCON0_USBSSEN          (1 << 8)
#define CFGCON0_PMULOCK          (1 << 10)
#define CFGCON0_PGLOCK           (1 << 11)
#define CFGCON0_PMDLOCK          (1 << 12)
#define CFGCON0_IOLOCK           (1 << 13)
#define CFGCON0_CANFDDIV_SHIFT   (20)
#define CFGCON0_CANFDDIV_MASK    (3 << CFGCON0_CANFDDIV_SHIFT)
#define CFGCON0_UPLLHWMD         (1 << 24)
#define CFGCON0_SPLLHWMD         (1 << 25)
#define CFGCON0_BTPLLHWMD        (1 << 26)
#define CFGCON0_ETHPLLHWMD       (1 << 27)

/* DEVID bitfields (Device identification register) *************************/

#define DEVID_PARTNUM_SHIFT      (20)
#define DEVID_PARTNUM_MASK       (0xff << DEVID_PARTNUM_SHIFT)
#define DEVID_VER_SHIFT          (28)
#define DEVID_VER_MASK           (0xf << DEVID_VER_SHIFT)

/* Known WFI32E01/PIC32MZW1 silicon revisions (DEVID bits 20-27).  The
 * data sheet (DS70005425P Reg 38-7) only documents DEVID[27:0] as a whole;
 * these values and their A1/B0/G meaning come from Microchip's
 * WFI32_Ethernet_Wi-Fi_Bridge_OOB example firmware, which uses them to
 * select the PMU/clock bring-up sequence (see pic32mz_wfi32_pwrclk.c).
 */

#define DEVID_PARTNUM_PIC32MZW1_A1 0x8c
#define DEVID_PARTNUM_PIC32MZW1_B0 0xa4
#define DEVID_PARTNUM_PIC32MZW1_G  0xa6

/* SYSKEY unlock/lock sequence (standard across the whole PIC32 family) */

#define UNLOCK_SYSKEY_0          (0xaa996655ul)
#define UNLOCK_SYSKEY_1          (0x556699aaul)
#define LOCK_SYSKEY              (0x33333333ul)

/* Boot-flash configuration words *******************************************/

/* Primary copy at 0xbfc55f88; offsets from the start of the block.  (An
 * alternate copy lives at 0xbfc55e88; programming tools write it with DFP
 * defaults, so NuttX does not emit it.)
 */

#define W1CFG_FUSERID_OFFSET     0x00
#define W1CFG_DEVCFG4_OFFSET     0x04
#define W1CFG_DEVCFG2_OFFSET     0x08
#define W1CFG_DEVCFG1_OFFSET     0x0c
#define W1CFG_DEVCFG0_OFFSET     0x10
#define W1CFG_FBCFG0_OFFSET      0x14
#define W1CFG_FCPN0_OFFSET       0x34
#define W1CFG_FSIGN0_OFFSET      0x54

/* FUSERID */

#define FUSERID_USERID_SHIFT     (0)       /* Bits 0-15 */
#define FUSERID_USERID_MASK      (0xffff << FUSERID_USERID_SHIFT)

/* DEVCFG4 (-> CFGCON4) */

#define DEVCFG4_SOSCCFG_SHIFT    (0)       /* Bits 0-7: SOSC configuration */
#define DEVCFG4_VBZPBOREN        (1 << 19) /* VBAT zero-power BOR enable */
#define DEVCFG4_DSZPBOREN        (1 << 20) /* Deep Sleep zero-power BOR enable */
#define DEVCFG4_DSWDTPS_SHIFT    (21)      /* Bits 21-25: DSWDT postscaler */
#define DEVCFG4_DSWDTOSC         (1 << 26) /* DSWDT clock: 1 = LPRC */
#define DEVCFG4_DSWDTEN          (1 << 27) /* Deep Sleep WDT enable */
#define DEVCFG4_DSEN             (1 << 28) /* Deep Sleep enable */
#define DEVCFG4_SOSCEN           (1 << 30) /* SOSC enable */

/* DEVCFG2 (-> CFGCON2) */

#define DEVCFG2_DMTINTV_SHIFT    (3)       /* Bits 3-5: DMT window interval */
#define DEVCFG2_POSCMOD_SHIFT    (8)       /* Bits 8-9: POSC mode */
#  define DEVCFG2_POSCMOD_HS     (2 << DEVCFG2_POSCMOD_SHIFT)
#  define DEVCFG2_POSCMOD_OFF    (3 << DEVCFG2_POSCMOD_SHIFT)
#define DEVCFG2_WDTRMCS_SHIFT    (10)      /* Bits 10-11: WDT run-mode clock */
#define DEVCFG2_SOSCSEL          (1 << 12) /* SOSC: 1 = crystal */
#define DEVCFG2_WAKE2SPD         (1 << 13) /* Wake to speed */
#define DEVCFG2_CKSWEN           (1 << 14) /* Software clock switching */
#define DEVCFG2_FSCMEN           (1 << 15) /* Fail-safe clock monitor */
#define DEVCFG2_WDTPS_SHIFT      (16)      /* Bits 16-20: WDT postscaler */
#define DEVCFG2_WDTSPGM          (1 << 21) /* WDT stop during flash prog. */
#define DEVCFG2_WINDIS           (1 << 22) /* WDT window disable */
#define DEVCFG2_WDTEN            (1 << 23) /* WDT enable */
#define DEVCFG2_WDTWINSZ_SHIFT   (24)      /* Bits 24-25: WDT window size */
#define DEVCFG2_DMTCNT_SHIFT     (26)      /* Bits 26-30: DMT count */

/* DMT (deadman timer) enable */

#define DEVCFG2_DMTEN            (1u << 31)

/* DEVCFG1 (-> CFGCON1) */

#define DEVCFG1_DEBUG_SHIFT      (0)       /* Bits 0-1: background debugger */
#  define DEVCFG1_DEBUG_ENABLED  (0 << DEVCFG1_DEBUG_SHIFT)
#  define DEVCFG1_DEBUG_DISABLED (3 << DEVCFG1_DEBUG_SHIFT)
#define DEVCFG1_ICESEL_SHIFT     (3)       /* Bits 3-4: ICE/debug channel */
#  define DEVCFG1_ICESEL_PGX4    (0 << DEVCFG1_ICESEL_SHIFT)
#  define DEVCFG1_ICESEL_PGX2    (2 << DEVCFG1_ICESEL_SHIFT)
#  define DEVCFG1_ICESEL_PGX1    (3 << DEVCFG1_ICESEL_SHIFT)
#define DEVCFG1_TRCEN            (1 << 5)  /* CPU trace enable */
#define DEVCFG1_FMIIEN           (1 << 8)  /* Ethernet: 1 = MII, 0 = RMII */
#define DEVCFG1_ETHEXEREF        (1 << 9)  /* PHY ref clock on exclusive pin */
#define DEVCFG1_CLASSBDIS        (1 << 10) /* Class B features disabled */
#define DEVCFG1_USBIDIO          (1 << 11) /* USBID pin owned by port */
#define DEVCFG1_VBUSIO           (1 << 12) /* VBUSON pin owned by port */
#define DEVCFG1_HSSPIEN          (1 << 13) /* SPI1 on dedicated pins */
#define DEVCFG1_SMCLR            (1 << 14) /* MCLR: legacy mode */
#define DEVCFG1_HSUARTEN         (1 << 15) /* UART1 on dedicated pins */

/* DEVCFG0 (-> CFGCON0): same layout as the CFGCON0 bits below, plus */

#define DEVCFG0_TDOEN            (1 << 0)
#define DEVCFG0_JTAGEN           (1 << 3)
#define DEVCFG0_PCM              (1 << 23) /* Prefetch I/D always cacheable */
#define DEVCFG0_FECCCON_SHIFT    (28)      /* Bits 28-29: flash ECC control */

/* FBCFG0 (-> BCFG0) */

#define FBCFG0_BUHSWEN           (1 << 0)  /* Buck mode switching control */
#define FBCFG0_PCSCMODE          (1 << 1)  /* Must be 0 (DS Reg 38-10) */
#define FBCFG0_BOOTISA           (1 << 3)  /* 1 = MIPS32, 0 = microMIPS */

/* BINFOVALID must be programmed to 0 (DS70005425 Reg 38-10) */

#define FBCFG0_BINFOVALID        (1u << 31)

/* FCPN0: CP, bit 28.  FSIGN0: SIGN, bit 31.  Emitted with the values found
 * on factory-programmed parts (0xffffffff and 0x7fffffff).
 */

#define W1CFG_FCPN0_VALUE        0xffffffff
#define W1CFG_FSIGN0_VALUE       0x7fffffff

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_FEATURES_H */
