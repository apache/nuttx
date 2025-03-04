/****************************************************************************
 * drivers/coresight/coresight_etm4.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/coresight/coresight_etm4.h>

#include "coresight_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device registers:
 * 0x000 - 0x2fc: Trace      registers
 * 0x300 - 0x314: Management registers
 * 0x318 - 0xefc: Trace      registers
 * 0xf00        : Management registers
 * 0xfa0 - 0xfa4: Trace      registers
 * 0xfa8 - 0xffc: Management registers
 *
 * Trace registers (0x000-0x2fc)
 * Main control and configuration registers
 */

#define TRCPRGCTLR                    0x004
#define TRCPROCSELR                   0x008
#define TRCSTATR                      0x00c
#define TRCCONFIGR                    0x010
#define TRCAUXCTLR                    0x018
#define TRCEVENTCTL0R                 0x020
#define TRCEVENTCTL1R                 0x024
#define TRCRSR                        0x028
#define TRCSTALLCTLR                  0x02c
#define TRCTSCTLR                     0x030
#define TRCSYNCPR                     0x034
#define TRCCCCTLR                     0x038
#define TRCBBCTLR                     0x03c
#define TRCTRACEIDR                   0x040
#define TRCQCTLR                      0x044

/* Filtering control registers */

#define TRCVICTLR                     0x080
#define TRCVIIECTLR                   0x084
#define TRCVISSCTLR                   0x088
#define TRCVIPCSSCTLR                 0x08c
#define TRCVDCTLR                     0x0a0
#define TRCVDSACCTLR                  0x0a4
#define TRCVDARCCTLR                  0x0a8

/* Derived resources registers */

#define TRCSEQEVRN(n)                 (0x100 + (n * 4)) /* n = 0-2 */
#define TRCSEQRSTEVR                  0x118
#define TRCSEQSTR                     0x11c
#define TRCEXTINSELR                  0x120
#define TRCEXTINSELRN(n)              (0x120 + (n * 4)) /* n = 0-3 */
#define TRCCNTRLDVRN(n)               (0x140 + (n * 4)) /* n = 0-3 */
#define TRCCNTCTLRN(n)                (0x150 + (n * 4)) /* n = 0-3 */
#define TRCCNTVRN(n)                  (0x160 + (n * 4)) /* n = 0-3 */

/* ID registers */

#define TRCIDR8                       0x180
#define TRCIDR9                       0x184
#define TRCIDR10                      0x188
#define TRCIDR11                      0x18c
#define TRCIDR12                      0x190
#define TRCIDR13                      0x194
#define TRCIMSPEC0                    0x1c0
#define TRCIMSPECN(n)                 (0x1c0 + (n * 4)) /* n = 1-7 */
#define TRCIDR0                       0x1e0
#define TRCIDR1                       0x1e4
#define TRCIDR2                       0x1e8
#define TRCIDR3                       0x1eC
#define TRCIDR4                       0x1f0
#define TRCIDR5                       0x1f4
#define TRCIDR6                       0x1f8
#define TRCIDR7                       0x1fc

/* Resource selection registers, n = 2-31.
 * First pair (regs 0, 1) is always present and is reserved.
 */

#define TRCRSCTLRN(n)                 (0x200 + (n * 4))

/* Single-shot comparator registers, n = 0-7 */

#define TRCSSCCRN(n)                  (0x280 + (n * 4))
#define TRCSSCSRN(n)                  (0x2a0 + (n * 4))
#define TRCSSPCICRN(n)                (0x2c0 + (n * 4))

/* Management registers (0x300-0x314) */

#define TRCOSLAR                      0x300
#define TRCOSLSR                      0x304
#define TRCPDCR                       0x310
#define TRCPDSR                       0x314

/* Trace registers (0x318-0xefc)
 * Address Comparator registers n = 0-15
 */

#define TRCACVRN(n)                   (0x400 + (n * 8))
#define TRCACATRN(n)                  (0x480 + (n * 8))

/* Data Value Comparator Value registers, n = 0-7 */

#define TRCDVCVRN(n)                  (0x500 + (n * 16))
#define TRCDVCMRN(n)                  (0x580 + (n * 16))

/* ContextID/Virtual ContextID comparators, n = 0-7 */

#define TRCCIDCVRN(n)                 (0x600 + (n * 8))
#define TRCVMIDCVRN(n)                (0x640 + (n * 8))
#define TRCCIDCCTLR0                  0x680
#define TRCCIDCCTLR1                  0x684
#define TRCVMIDCCTLR0                 0x688
#define TRCVMIDCCTLR1                 0x68c

/* Management register (0xf00)
 * Integration control registers
 */

#define TRCITCTRL                     0xf00

/* Trace registers (0xfa0-0xfa4)
 * Claim tag registers
 */

#define TRCCLAIMSET                   0xfa0
#define TRCCLAIMCLR                   0xfa4

/* Management registers (0xfa8-0xffc) */

#define TRCDEVAFF0                    0xfa8
#define TRCDEVAFF1                    0xfac
#define TRCLAR                        0xfb0
#define TRCLSR                        0xfb4
#define TRCAUTHSTATUS                 0xfb8
#define TRCDEVARCH                    0xfbc
#define TRCDEVID                      0xfc8
#define TRCDEVTYPE                    0xfcc
#define TRCPIDR4                      0xfd0
#define TRCPIDR5                      0xfd4
#define TRCPIDR6                      0xfd8
#define TRCPIDR7                      0xfdc
#define TRCPIDR0                      0xfe0
#define TRCPIDR1                      0xfe4
#define TRCPIDR2                      0xfe8
#define TRCPIDR3                      0xfec
#define TRCCIDR0                      0xff0
#define TRCCIDR1                      0xff4
#define TRCCIDR2                      0xff8
#define TRCCIDR3                      0xffc

/* Bit positions of registers that are defined above, in the sysreg.h style
 * of _MASK for multi bit fields and BIT() for single bits.
 */

#define TRCRSR_TA                     BIT(12)
#define TRCIDR0_INSTP0_MASK           GENMASK(2, 1)
#define TRCIDR0_TRCBB                 BIT(5)
#define TRCIDR0_TRCCOND               BIT(6)
#define TRCIDR0_TRCCCI                BIT(7)
#define TRCIDR0_RETSTACK              BIT(9)
#define TRCIDR0_NUMEVENT_MASK         GENMASK(11, 10)
#define TRCIDR0_QSUPP_MASK            GENMASK(16, 15)
#define TRCIDR0_TSSIZE_MASK           GENMASK(28, 24)
#define TRCIDR2_CIDSIZE_MASK          GENMASK(9, 5)
#define TRCIDR2_VMIDSIZE_MASK         GENMASK(14, 10)
#define TRCIDR2_CCSIZE_MASK           GENMASK(28, 25)
#define TRCIDR3_CCITMIN_MASK          GENMASK(11, 0)
#define TRCIDR3_EXLEVEL_S_MASK        GENMASK(19, 16)
#define TRCIDR3_EXLEVEL_NS_MASK       GENMASK(23, 20)
#define TRCIDR3_TRCERR                BIT(24)
#define TRCIDR3_SYNCPR                BIT(25)
#define TRCIDR3_STALLCTL              BIT(26)
#define TRCIDR3_SYSSTALL              BIT(27)
#define TRCIDR3_NUMPROC_LO_MASK       GENMASK(30, 28)
#define TRCIDR3_NUMPROC_HI_MASK       GENMASK(13, 12)
#define TRCIDR3_NOOVERFLOW            BIT(31)
#define TRCIDR4_NUMACPAIRS_MASK       GENMASK(3, 0)
#define TRCIDR4_NUMPC_MASK            GENMASK(15, 12)
#define TRCIDR4_NUMRSPAIR_MASK        GENMASK(19, 16)
#define TRCIDR4_NUMSSCC_MASK          GENMASK(23, 20)
#define TRCIDR4_NUMCIDC_MASK          GENMASK(27, 24)
#define TRCIDR4_NUMVMIDC_MASK         GENMASK(31, 28)
#define TRCIDR5_NUMEXTIN_MASK         GENMASK(8, 0)
#define TRCIDR5_TRACEIDSIZE_MASK      GENMASK(21, 16)
#define TRCIDR5_ATBTRIG               BIT(22)
#define TRCIDR5_LPOVERRIDE            BIT(23)
#define TRCIDR5_NUMSEQSTATE_MASK      GENMASK(27, 25)
#define TRCIDR5_NUMCNTR_MASK          GENMASK(30, 28)
#define TRCCONFIGR_INSTP0_LOAD        BIT(1)
#define TRCCONFIGR_INSTP0_STORE       BIT(2)
#define TRCCONFIGR_INSTP0_LOAD_STORE  (TRCCONFIGR_INSTP0_LOAD | TRCCONFIGR_INSTP0_STORE)
#define TRCCONFIGR_BB                 BIT(3)
#define TRCCONFIGR_CCI                BIT(4)
#define TRCCONFIGR_CID                BIT(6)
#define TRCCONFIGR_VMID               BIT(7)
#define TRCCONFIGR_COND_MASK          GENMASK(10, 8)
#define TRCCONFIGR_TS                 BIT(11)
#define TRCCONFIGR_RS                 BIT(12)
#define TRCCONFIGR_QE_W_COUNTS        BIT(13)
#define TRCCONFIGR_QE_WO_COUNTS       BIT(14)
#define TRCCONFIGR_VMIDOPT            BIT(15)
#define TRCCONFIGR_DA                 BIT(16)
#define TRCCONFIGR_DV                 BIT(17)
#define TRCEVENTCTL1R_INSTEN_MASK     GENMASK(3, 0)
#define TRCEVENTCTL1R_INSTEN_0        BIT(0)
#define TRCEVENTCTL1R_INSTEN_1        BIT(1)
#define TRCEVENTCTL1R_INSTEN_2        BIT(2)
#define TRCEVENTCTL1R_INSTEN_3        BIT(3)
#define TRCEVENTCTL1R_ATB             BIT(11)
#define TRCEVENTCTL1R_LPOVERRIDE      BIT(12)
#define TRCSTALLCTLR_ISTALL           BIT(8)
#define TRCSTALLCTLR_INSTPRIORITY     BIT(10)
#define TRCSTALLCTLR_NOOVERFLOW       BIT(13)
#define TRCVICTLR_EVENT_MASK          GENMASK(7, 0)
#define TRCVICTLR_SSSTATUS            BIT(9)
#define TRCVICTLR_TRCRESET            BIT(10)
#define TRCVICTLR_TRCERR              BIT(11)
#define TRCVICTLR_EXLEVEL_MASK        GENMASK(22, 16)
#define TRCVICTLR_EXLEVEL_S_MASK      GENMASK(19, 16)
#define TRCVICTLR_EXLEVEL_NS_MASK     GENMASK(22, 20)
#define TRCACATRN_TYPE_MASK           GENMASK(1, 0)
#define TRCACATRN_CONTEXTTYPE_MASK    GENMASK(3, 2)
#define TRCACATRN_CONTEXTTYPE_CTXID   BIT(2)
#define TRCACATRN_CONTEXTTYPE_VMID    BIT(3)
#define TRCACATRN_CONTEXT_MASK        GENMASK(6, 4)
#define TRCACATRN_EXLEVEL_MASK        GENMASK(14, 8)
#define TRCSSCSRN_STATUS              BIT(31)
#define TRCSSCCRN_SAC_ARC_RST_MASK    GENMASK(24, 0)
#define TRCSSPCICRN_PC_MASK           GENMASK(7, 0)
#define TRCBBCTLR_MODE                BIT(8)
#define TRCBBCTLR_RANGE_MASK          GENMASK(7, 0)
#define TRCRSCTLRN_PAIRINV            BIT(21)
#define TRCRSCTLRN_INV                BIT(20)
#define TRCRSCTLRN_GROUP_MASK         GENMASK(19, 16)
#define TRCRSCTLRN_SELECT_MASK        GENMASK(15, 0)

#define TRFCR_ELX_TS_SHIFT            5
#define TRFCR_ELX_TS_VIRTUAL          ((0x1UL) << TRFCR_ELX_TS_SHIFT)
#define TRFCR_EL2_CX                  BIT(3)
#define TRFCR_ELX_E1TRE               BIT(1)
#define TRFCR_ELX_E0TRE               BIT(0)
#define CURRENTEL_EL2                 (2 << 2)

/* TRCDEVARCH Bit field definitions
 * Bits[31:21]  - ARCHITECT = Always Arm Ltd.
 *                * Bits[31:28] = 0x4
 *                * Bits[27:21] = 0b0111011
 * Bit[20]  - PRESENT,  Indicates the presence of this register.
 *
 * Bit[19:16]  - REVISION, Revision of the architecture.
 *
 * Bit[15:0]  - ARCHID, Identifies this component as an ETM
 *                * Bits[15:12] - architecture version of ETM
 *                *             = 4 for ETMv4
 *                * Bits[11:0] = 0xA13, architecture part number for ETM.
 */

#define TRCDEVARCH_REVISION_SHIFT           16
#define TRCDEVARCH_REVISION_MASK            GENMASK(19, 16)
#define TRCDEVARCH_REVISION(x) \
        (((x) & TRCDEVARCH_REVISION_MASK) >> TRCDEVARCH_REVISION_SHIFT)

#define TRCDEVARCH_ARCHID_ARCH_VER_SHIFT    12
#define TRCDEVARCH_ARCHID_ARCH_VER_MASK     GENMASK(15, 12)
#define TRCDEVARCH_ARCHID_ARCH_VER(x) \
        (((x) & TRCDEVARCH_ARCHID_ARCH_VER_MASK) >> TRCDEVARCH_ARCHID_ARCH_VER_SHIFT)

#define ID_AA64DFR0_EL1_TRACEFILT_SHIFT     40
#define ID_AA64DFR0_EL1_TRACEFILT_MASK      GENMASK(43, 40)
#define ID_AA64DFR0_EL1_TRACEFILT(x) \
        (((x) & ID_AA64DFR0_EL1_TRACEFILT_MASK) >> ID_AA64DFR0_EL1_TRACEFILT_SHIFT)

#define TRCSTATR_IDLE_BIT                   BIT(0)
#define TRCSTATR_PMSTABLE_BIT               BIT(1)
#define TRCSSCSRN_PC                        BIT(3)

/* PowerDown Control Register bits */

#define TRCPDCR_PU                          BIT(3)

/* Driver representation of the ETM architecture.
 * The version of an ETM component can be detected from
 *
 * TRCDEVARCH  - CoreSight architected register
 *                - Bits[15:12] - Major version
 *                - Bits[19:16] - Minor version
 *
 * We must rely only on TRCDEVARCH for the version information. Even though,
 * TRCIDR1 also provides the architecture version, it is a "Trace" register
 * and as such must be accessed only with Trace power domain ON. This may
 * not be available at probe time.
 *
 * Now to make certain decisions easier based on the version
 * we use an internal representation of the version in the
 * driver, as follows :
 *
 * ETM4_ARCH_VERSION[7:0], where :
 *      Bits[7:4] - Major version
 *      Bits[3:0] - Minro version
 */

#define ETM4_ARCH_VERSION(major, minor)     ((((major) & 0xf) << 4) | (((minor) & 0xf)))

#define ETM4_ARCH_V4                        ETM4_ARCH_VERSION(4, 0)
#define ETM4_ARCH_V4_3                      ETM4_ARCH_VERSION(4, 3)
#define ETM4_ARCH_ETE                       ETM4_ARCH_VERSION(5, 0)
#define ETM4_ARCH_V3_3                      ETM4_ARCH_VERSION(3, 3)
#define ETM4_ARCH_V3_5                      ETM4_ARCH_VERSION(3, 5)
#define PFT_ARCH_V1_0                       ETM4_ARCH_VERSION(1, 0)
#define PFT_ARCH_V1_1                       ETM4_ARCH_VERSION(1, 1)

/* Below are the definition of bit offsets for perf option */

#define ETM4_OPT_BRANCH_BROADCAST           8
#define ETM4_OPT_CYCACC                     12
#define ETM4_OPT_RETSTK                     29

/* ETMv4 CONFIGR programming bits for the ETM OPTs */

#define ETM4_CFG_BIT_BB                     3

/* ETMv4 programming modes */

#define ETM4_MODE_VIEWINST_STARTSTOP        BIT(27)

/* System instructions to access ETM registers */

#define ETM4_READ_CASE(res, x, y)                      \
  case (x):                                            \
    (res) = read_sysreg(y);                            \
    break;

#define ETM4_WRITE_CASE(val, x, y)                     \
  case (x):                                            \
    write_sysreg((val), y);                            \
    break;

/* List of registers accessible via System instructions */

#define ETM4_READ_ONLY_SYSREG_CASES(res)               \
  ETM4_READ_CASE(res, TRCIDR9, trcidr9)                \
  ETM4_READ_CASE(res, TRCIDR12, trcidr12)              \
  ETM4_READ_CASE(res, TRCIDR1, trcidr1)                \
  ETM4_READ_CASE(res, TRCIDR4, trcidr4)                \
  ETM4_READ_CASE(res, TRCIDR7, trcidr7)                \
  ETM4_READ_CASE(res, TRCOSLSR, trcoslsr)              \
  ETM4_READ_CASE(res, TRCSTATR, trcstatr)              \
  ETM4_READ_CASE(res, TRCIDR8, trcidr8)                \
  ETM4_READ_CASE(res, TRCIDR10, trcidr10)              \
  ETM4_READ_CASE(res, TRCIDR11, trcidr11)              \
  ETM4_READ_CASE(res, TRCIDR13, trcidr13)              \
  ETM4_READ_CASE(res, TRCIDR0, trcidr0)                \
  ETM4_READ_CASE(res, TRCIDR2, trcidr2)                \
  ETM4_READ_CASE(res, TRCIDR3, trcidr3)                \
  ETM4_READ_CASE(res, TRCIDR5, trcidr5)                \
  ETM4_READ_CASE(res, TRCIDR6, trcidr6)

#define ETM4_WRITE_ONLY_SYSREG_CASES(val)              \
  ETM4_WRITE_CASE(val, TRCOSLAR, trcoslar)

#define ETM4_READ_WRITE_SYSREG_CASES(op, val)          \
  ETM4_##op##_CASE(val, TRCPRGCTLR, trcprgctlr)        \
  ETM4_##op##_CASE(val, TRCCONFIGR, trcconfigr)        \
  ETM4_##op##_CASE(val, TRCAUXCTLR, trcauxctlr)        \
  ETM4_##op##_CASE(val, TRCEVENTCTL0R, trceventctl0r)  \
  ETM4_##op##_CASE(val, TRCEVENTCTL1R, trceventctl1r)  \
  ETM4_##op##_CASE(val, TRCSTALLCTLR, trcstallctlr)    \
  ETM4_##op##_CASE(val, TRCTSCTLR, trctsctlr)          \
  ETM4_##op##_CASE(val, TRCSYNCPR, trcsyncpr)          \
  ETM4_##op##_CASE(val, TRCCCCTLR, trcccctlr)          \
  ETM4_##op##_CASE(val, TRCBBCTLR, trcbbctlr)          \
  ETM4_##op##_CASE(val, TRCTRACEIDR, trctraceidr)      \
  ETM4_##op##_CASE(val, TRCQCTLR, trcqctlr)            \
  ETM4_##op##_CASE(val, TRCVICTLR, trcvictlr)          \
  ETM4_##op##_CASE(val, TRCVIIECTLR, trcviiectlr)      \
  ETM4_##op##_CASE(val, TRCVISSCTLR, trcvissctlr)      \
  ETM4_##op##_CASE(val, TRCVIPCSSCTLR, trcvipcssctlr)  \
  ETM4_##op##_CASE(val, TRCSEQEVRN(0), trcseqevr0)     \
  ETM4_##op##_CASE(val, TRCSEQEVRN(1), trcseqevr1)     \
  ETM4_##op##_CASE(val, TRCSEQEVRN(2), trcseqevr2)     \
  ETM4_##op##_CASE(val, TRCSEQRSTEVR, trcseqrstevr)    \
  ETM4_##op##_CASE(val, TRCSEQSTR, trcseqstr)          \
  ETM4_##op##_CASE(val, TRCEXTINSELR, trcextinselr)    \
  ETM4_##op##_CASE(val, TRCCNTRLDVRN(0), trccntrldvr0) \
  ETM4_##op##_CASE(val, TRCCNTRLDVRN(1), trccntrldvr1) \
  ETM4_##op##_CASE(val, TRCCNTRLDVRN(2), trccntrldvr2) \
  ETM4_##op##_CASE(val, TRCCNTRLDVRN(3), trccntrldvr3) \
  ETM4_##op##_CASE(val, TRCCNTCTLRN(0), trccntctlr0)   \
  ETM4_##op##_CASE(val, TRCCNTCTLRN(1), trccntctlr1)   \
  ETM4_##op##_CASE(val, TRCCNTCTLRN(2), trccntctlr2)   \
  ETM4_##op##_CASE(val, TRCCNTCTLRN(3), trccntctlr3)   \
  ETM4_##op##_CASE(val, TRCCNTVRN(0), trccntvr0)       \
  ETM4_##op##_CASE(val, TRCCNTVRN(1), trccntvr1)       \
  ETM4_##op##_CASE(val, TRCCNTVRN(2), trccntvr2)       \
  ETM4_##op##_CASE(val, TRCCNTVRN(3), trccntvr3)       \
  ETM4_##op##_CASE(val, TRCIMSPECN(0), trcimspec0)     \
  ETM4_##op##_CASE(val, TRCIMSPECN(1), trcimspec1)     \
  ETM4_##op##_CASE(val, TRCIMSPECN(2), trcimspec2)     \
  ETM4_##op##_CASE(val, TRCIMSPECN(3), trcimspec3)     \
  ETM4_##op##_CASE(val, TRCIMSPECN(4), trcimspec4)     \
  ETM4_##op##_CASE(val, TRCIMSPECN(5), trcimspec5)     \
  ETM4_##op##_CASE(val, TRCIMSPECN(6), trcimspec6)     \
  ETM4_##op##_CASE(val, TRCIMSPECN(7), trcimspec7)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(2), trcrsctlr2)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(3), trcrsctlr3)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(4), trcrsctlr4)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(5), trcrsctlr5)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(6), trcrsctlr6)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(7), trcrsctlr7)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(8), trcrsctlr8)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(9), trcrsctlr9)     \
  ETM4_##op##_CASE(val, TRCRSCTLRN(10), trcrsctlr10)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(11), trcrsctlr11)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(12), trcrsctlr12)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(13), trcrsctlr13)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(14), trcrsctlr14)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(15), trcrsctlr15)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(16), trcrsctlr16)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(17), trcrsctlr17)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(18), trcrsctlr18)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(19), trcrsctlr19)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(20), trcrsctlr20)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(21), trcrsctlr21)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(22), trcrsctlr22)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(23), trcrsctlr23)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(24), trcrsctlr24)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(25), trcrsctlr25)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(26), trcrsctlr26)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(27), trcrsctlr27)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(28), trcrsctlr28)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(29), trcrsctlr29)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(30), trcrsctlr30)   \
  ETM4_##op##_CASE(val, TRCRSCTLRN(31), trcrsctlr31)   \
  ETM4_##op##_CASE(val, TRCSSCCRN(0), trcssccr0)       \
  ETM4_##op##_CASE(val, TRCSSCCRN(1), trcssccr1)       \
  ETM4_##op##_CASE(val, TRCSSCCRN(2), trcssccr2)       \
  ETM4_##op##_CASE(val, TRCSSCCRN(3), trcssccr3)       \
  ETM4_##op##_CASE(val, TRCSSCCRN(4), trcssccr4)       \
  ETM4_##op##_CASE(val, TRCSSCCRN(5), trcssccr5)       \
  ETM4_##op##_CASE(val, TRCSSCCRN(6), trcssccr6)       \
  ETM4_##op##_CASE(val, TRCSSCCRN(7), trcssccr7)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(0), trcsscsr0)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(1), trcsscsr1)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(2), trcsscsr2)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(3), trcsscsr3)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(4), trcsscsr4)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(5), trcsscsr5)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(6), trcsscsr6)       \
  ETM4_##op##_CASE(val, TRCSSCSRN(7), trcsscsr7)       \
  ETM4_##op##_CASE(val, TRCSSPCICRN(0), trcsspcicr0)   \
  ETM4_##op##_CASE(val, TRCSSPCICRN(1), trcsspcicr1)   \
  ETM4_##op##_CASE(val, TRCSSPCICRN(2), trcsspcicr2)   \
  ETM4_##op##_CASE(val, TRCSSPCICRN(3), trcsspcicr3)   \
  ETM4_##op##_CASE(val, TRCSSPCICRN(4), trcsspcicr4)   \
  ETM4_##op##_CASE(val, TRCSSPCICRN(5), trcsspcicr5)   \
  ETM4_##op##_CASE(val, TRCSSPCICRN(6), trcsspcicr6)   \
  ETM4_##op##_CASE(val, TRCSSPCICRN(7), trcsspcicr7)   \
  ETM4_##op##_CASE(val, TRCACVRN(0), trcacvr0)         \
  ETM4_##op##_CASE(val, TRCACVRN(1), trcacvr1)         \
  ETM4_##op##_CASE(val, TRCACVRN(2), trcacvr2)         \
  ETM4_##op##_CASE(val, TRCACVRN(3), trcacvr3)         \
  ETM4_##op##_CASE(val, TRCACVRN(4), trcacvr4)         \
  ETM4_##op##_CASE(val, TRCACVRN(5), trcacvr5)         \
  ETM4_##op##_CASE(val, TRCACVRN(6), trcacvr6)         \
  ETM4_##op##_CASE(val, TRCACVRN(7), trcacvr7)         \
  ETM4_##op##_CASE(val, TRCACVRN(8), trcacvr8)         \
  ETM4_##op##_CASE(val, TRCACVRN(9), trcacvr9)         \
  ETM4_##op##_CASE(val, TRCACVRN(10), trcacvr10)       \
  ETM4_##op##_CASE(val, TRCACVRN(11), trcacvr11)       \
  ETM4_##op##_CASE(val, TRCACVRN(12), trcacvr12)       \
  ETM4_##op##_CASE(val, TRCACVRN(13), trcacvr13)       \
  ETM4_##op##_CASE(val, TRCACVRN(14), trcacvr14)       \
  ETM4_##op##_CASE(val, TRCACVRN(15), trcacvr15)       \
  ETM4_##op##_CASE(val, TRCACATRN(0), trcacatr0)       \
  ETM4_##op##_CASE(val, TRCACATRN(1), trcacatr1)       \
  ETM4_##op##_CASE(val, TRCACATRN(2), trcacatr2)       \
  ETM4_##op##_CASE(val, TRCACATRN(3), trcacatr3)       \
  ETM4_##op##_CASE(val, TRCACATRN(4), trcacatr4)       \
  ETM4_##op##_CASE(val, TRCACATRN(5), trcacatr5)       \
  ETM4_##op##_CASE(val, TRCACATRN(6), trcacatr6)       \
  ETM4_##op##_CASE(val, TRCACATRN(7), trcacatr7)       \
  ETM4_##op##_CASE(val, TRCACATRN(8), trcacatr8)       \
  ETM4_##op##_CASE(val, TRCACATRN(9), trcacatr9)       \
  ETM4_##op##_CASE(val, TRCACATRN(10), trcacatr10)     \
  ETM4_##op##_CASE(val, TRCACATRN(11), trcacatr11)     \
  ETM4_##op##_CASE(val, TRCACATRN(12), trcacatr12)     \
  ETM4_##op##_CASE(val, TRCACATRN(13), trcacatr13)     \
  ETM4_##op##_CASE(val, TRCACATRN(14), trcacatr14)     \
  ETM4_##op##_CASE(val, TRCACATRN(15), trcacatr15)     \
  ETM4_##op##_CASE(val, TRCDVCVRN(0), trcdvcvr0)       \
  ETM4_##op##_CASE(val, TRCDVCVRN(1), trcdvcvr1)       \
  ETM4_##op##_CASE(val, TRCDVCVRN(2), trcdvcvr2)       \
  ETM4_##op##_CASE(val, TRCDVCVRN(3), trcdvcvr3)       \
  ETM4_##op##_CASE(val, TRCDVCVRN(4), trcdvcvr4)       \
  ETM4_##op##_CASE(val, TRCDVCVRN(5), trcdvcvr5)       \
  ETM4_##op##_CASE(val, TRCDVCVRN(6), trcdvcvr6)       \
  ETM4_##op##_CASE(val, TRCDVCVRN(7), trcdvcvr7)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(0), trcdvcmr0)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(1), trcdvcmr1)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(2), trcdvcmr2)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(3), trcdvcmr3)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(4), trcdvcmr4)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(5), trcdvcmr5)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(6), trcdvcmr6)       \
  ETM4_##op##_CASE(val, TRCDVCMRN(7), trcdvcmr7)       \
  ETM4_##op##_CASE(val, TRCCIDCVRN(0), trccidcvr0)     \
  ETM4_##op##_CASE(val, TRCCIDCVRN(1), trccidcvr1)     \
  ETM4_##op##_CASE(val, TRCCIDCVRN(2), trccidcvr2)     \
  ETM4_##op##_CASE(val, TRCCIDCVRN(3), trccidcvr3)     \
  ETM4_##op##_CASE(val, TRCCIDCVRN(4), trccidcvr4)     \
  ETM4_##op##_CASE(val, TRCCIDCVRN(5), trccidcvr5)     \
  ETM4_##op##_CASE(val, TRCCIDCVRN(6), trccidcvr6)     \
  ETM4_##op##_CASE(val, TRCCIDCVRN(7), trccidcvr7)     \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(0), trcvmidcvr0)   \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(1), trcvmidcvr1)   \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(2), trcvmidcvr2)   \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(3), trcvmidcvr3)   \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(4), trcvmidcvr4)   \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(5), trcvmidcvr5)   \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(6), trcvmidcvr6)   \
  ETM4_##op##_CASE(val, TRCVMIDCVRN(7), trcvmidcvr7)   \
  ETM4_##op##_CASE(val, TRCCIDCCTLR0, trccidcctlr0)    \
  ETM4_##op##_CASE(val, TRCCIDCCTLR1, trccidcctlr1)    \
  ETM4_##op##_CASE(val, TRCVMIDCCTLR0, trcvmidcctlr0)  \
  ETM4_##op##_CASE(val, TRCVMIDCCTLR1, trcvmidcctlr1)  \
  ETM4_##op##_CASE(val, TRCCLAIMSET, trcclaimset)      \
  ETM4_##op##_CASE(val, TRCCLAIMCLR, trcclaimclr)      \
  ETM4_##op##_CASE(val, TRCAUTHSTATUS, trcauthstatus)  \
  ETM4_##op##_CASE(val, TRCDEVARCH, trcdevarch)        \
  ETM4_##op##_CASE(val, TRCDEVID, trcdevid)            \
  ETM4_##op##_CASE(val, TRCPROCSELR, trcprocselr)      \
  ETM4_##op##_CASE(val, TRCVDCTLR, trcvdctlr)          \
  ETM4_##op##_CASE(val, TRCVDSACCTLR, trcvdsacctlr)    \
  ETM4_##op##_CASE(val, TRCVDARCCTLR, trcvdarcctlr)

#define ETM4_READ_SYSREG_CASES(res)                    \
        ETM4_READ_WRITE_SYSREG_CASES(READ, (res))      \
        ETM4_READ_ONLY_SYSREG_CASES((res))

#define ETM4_WRITE_SYSREG_CASES(val)                   \
        ETM4_READ_WRITE_SYSREG_CASES(WRITE, (val))     \
        ETM4_WRITE_ONLY_SYSREG_CASES((val))

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int etm4_enable(FAR struct coresight_dev_s *csdev);
static void etm4_disable(FAR struct coresight_dev_s *csdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct coresight_source_ops_s g_etm4_source_ops =
{
  .enable  = etm4_enable,
  .disable = etm4_disable,
};

static const struct coresight_ops_s g_etm4_ops =
{
  .source_ops = &g_etm4_source_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etm4_sysreg_read
 *
 * Description:
 *   Read registers using coprocessor.
 *
 ****************************************************************************/

static uint64_t etm4_sysreg_read(uint32_t offset)
{
  uint64_t res = 0;

  switch (offset)
    {
      ETM4_READ_SYSREG_CASES(res)
      default :
        cserr("etm4x: trying to read unsupported register @%x\n", offset);
    }

  return res;
}

/****************************************************************************
 * Name: etm4_sysreg_write
 *
 * Description:
 *   Write registers using coprocessor.
 *
 ****************************************************************************/

static void etm4_sysreg_write(uint64_t val, uint32_t offset, bool bit_64)
{
  if (!bit_64)
    {
      val &= GENMASK(31, 0);
    }

  switch (offset)
    {
      ETM4_WRITE_SYSREG_CASES(val)
      default :
        cserr("etm4x: trying to write to unsupported register @%x\n",
              offset);
    }
}

/****************************************************************************
 * Name: etm4_devarch_to_arch
 *
 * Description:
 *   Convert the device architecture identifier to an ETMv4
 *   architecture version.
 *
 ****************************************************************************/

static inline uint8_t etm4_devarch_to_arch(uint32_t devarch)
{
  return ETM4_ARCH_VERSION(TRCDEVARCH_ARCHID_ARCH_VER(devarch),
         TRCDEVARCH_REVISION(devarch));
}

/****************************************************************************
 * Name: etm4_is_in_el2
 *
 * Description:
 *   Check if the current execution level is EL2.
 *
 * Returned Value:
 *   Returns true if the current execution level is EL2, false otherwise.
 *
 ****************************************************************************/

static inline bool etm4_is_in_el2(void)
{
  return read_sysreg(currentel) == CURRENTEL_EL2;
}

/****************************************************************************
 * Name: etm4_write_reg32
 *
 * Description:
 *   Writes a 32-bit value to a specified register offset.
 *
 ****************************************************************************/

static inline void etm4_write_reg32(FAR struct coresight_etm4_dev_s *etmdev,
                                    uint32_t val, uint32_t off)
{
#ifdef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
  etm4_sysreg_write(val, off, false);
#else
  coresight_put32(val, etmdev->csdev.addr + off);
#endif
}

/****************************************************************************
 * Name: etm4_write_reg64
 *
 * Description:
 *   Writes a 64-bit value to a specified register offset.
 *
 ****************************************************************************/

static inline void etm4_write_reg64(FAR struct coresight_etm4_dev_s *etmdev,
                                    uint64_t val, uint32_t off)
{
#ifdef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
  etm4_sysreg_write(val, off, true);
#else
  coresight_put64(val, etmdev->csdev.addr + off);
#endif
}

/****************************************************************************
 * Name: etm4_read_reg32
 *
 * Description:
 *   Reads a 32-bit value from a specified register offset.
 *
 ****************************************************************************/

static inline uint32_t etm4_read_reg32(FAR struct coresight_etm4_dev_s
                                       *etmdev, uint32_t off)
{
#ifdef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
  return etm4_sysreg_read(off);
#else
  return coresight_get32(etmdev->csdev.addr + off);
#endif
}

/****************************************************************************
 * Name: etm4_claim_device
 *
 * Description:
 *   Claims the ETMv4 device for exclusive access.
 *
 ****************************************************************************/

int etm4_claim_device(FAR struct coresight_etm4_dev_s *etmdev)
{
  int ret = -EBUSY;

#ifndef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
  return coresight_claim_device(etmdev->csdev.addr);
#endif

  if (etm4_read_reg32(etmdev, CORESIGHT_CLAIMCLR) != 0)
    {
      return ret;
    }

  etm4_write_reg32(etmdev, CORESIGHT_CLAIM_SELF_HOSTED,
                   CORESIGHT_CLAIMSET);
  if (etm4_read_reg32(etmdev, CORESIGHT_CLAIMCLR) ==
        CORESIGHT_CLAIM_SELF_HOSTED)
    {
      ret = 0;
    }
  else
    {
      /* There was a race setting the tags, clean up and fail */

      etm4_write_reg32(etmdev, CORESIGHT_CLAIM_SELF_HOSTED,
                       CORESIGHT_CLAIMCLR);
    }

  return ret;
}

/****************************************************************************
 * Name: etm4_disclaim_device
 *
 * Description:
 *   Releases the ETMv4 device from exclusive access.
 *
 ****************************************************************************/

void etm4_disclaim_device(FAR struct coresight_etm4_dev_s *etmdev)
{
#ifndef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
  return coresight_disclaim_device(etmdev->csdev.addr);
#endif

  if (etm4_read_reg32(etmdev, CORESIGHT_CLAIMCLR) ==
        CORESIGHT_CLAIM_SELF_HOSTED)
    {
      etm4_write_reg32(etmdev, CORESIGHT_CLAIM_SELF_HOSTED,
                       CORESIGHT_CLAIMCLR);
    }
  else
    {
      cserr("current device is not claimed or something wrong happend\n");
    }
}

/****************************************************************************
 * Name: etm4_sspcicrn_present
 *
 * Description:
 *   Check if a specific single-shot comparator (SSPCICRN)
 *   is present in the ETMv4 device.
 *
 ****************************************************************************/

static inline bool
etm4_sspcicrn_present(FAR struct coresight_etm4_dev_s *etmdev, int n)
{
  return (n < etmdev->nr_ss_cmp) && etmdev->nr_pe &&
         (etmdev->cfg.ss_status[n] & TRCSSCSRN_PC);
}

/****************************************************************************
 * Name: etm4_lock
 *
 * Description:
 *   Locks the ETMv4 device to prevent concurrent modifications.
 *
 ****************************************************************************/

static inline void etm4_lock(FAR struct coresight_etm4_dev_s *etmdev)
{
#ifndef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
  coresight_lock(etmdev->csdev.addr);
#endif
}

/****************************************************************************
 * Name: etm4_unlock
 *
 * Description:
 *   Unlocks the ETMv4 device to allow other operations.
 *
 ****************************************************************************/

static inline void etm4_unlock(FAR struct coresight_etm4_dev_s *etmdev)
{
#ifndef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
  coresight_unlock(etmdev->csdev.addr);
#endif
}

/****************************************************************************
 * Name: etm4_os_unlock
 *
 * Description:
 *   Unlocks the TRCOSLAR register of ETMv4 for further configurations.
 *
 ****************************************************************************/

static inline void etm4_os_unlock(FAR struct coresight_etm4_dev_s *etmdev)
{
  etm4_write_reg32(etmdev, 0x0, TRCOSLAR);
}

/****************************************************************************
 * Name: etm4_timeout
 *
 * Description:
 *   Loop until a bitmask of register has changed to a specific value.
 *
 ****************************************************************************/

static int etm4_timeout(FAR struct coresight_etm4_dev_s *etmdev,
                        uint32_t val, uint32_t mask, uint32_t off)
{
  int i;

  for (i = CONFIG_CORESIGHT_TIMEOUT; i > 0; i--)
    {
      uint32_t value = etm4_read_reg32(etmdev, off);
      if ((value & mask) == val)
        {
          return 0;
        }

      up_udelay(1);
    }

  return -EAGAIN;
}

/****************************************************************************
 * Name: etm4_enable_hw
 *
 * Description:
 *   Enable the ETMv4 hardware by configuring various trace registers.
 *
 * Input Parameters:
 *   etmdev - Pointer to the ETMv4 device structure.
 *
 ****************************************************************************/

static void etm4_enable_hw(FAR struct coresight_etm4_dev_s *etmdev)
{
  FAR struct etm4_config_s *config = &etmdev->cfg;
  int i;

  etm4_unlock(etmdev);
  etm4_os_unlock(etmdev);

  /* Disable the trace unit before programming trace registers */

  etm4_write_reg32(etmdev, 0, TRCPRGCTLR);

  /* wait for TRCSTATR.IDLE to go up */

  if (etm4_timeout(etmdev, 1, TRCSTATR_IDLE_BIT, TRCSTATR))
    {
      cserr("timeout while waiting for Idle Trace Status\n");
    }

  if (etmdev->nr_pe)
    {
      etm4_write_reg32(etmdev, config->pe_sel, TRCPROCSELR);
    }

  etm4_write_reg32(etmdev, config->cfg, TRCCONFIGR);

  /* nothing specific implemented */

  etm4_write_reg32(etmdev, 0x0, TRCAUXCTLR);
  etm4_write_reg32(etmdev, config->eventctrl0, TRCEVENTCTL0R);
  etm4_write_reg32(etmdev, config->eventctrl1, TRCEVENTCTL1R);
  if (etmdev->stallctl)
    {
      etm4_write_reg32(etmdev, config->stall_ctrl, TRCSTALLCTLR);
    }

  etm4_write_reg32(etmdev, config->ts_ctrl, TRCTSCTLR);
  etm4_write_reg32(etmdev, config->syncfreq, TRCSYNCPR);
  etm4_write_reg32(etmdev, config->ccctlr, TRCCCCTLR);
  etm4_write_reg32(etmdev, config->bb_ctrl, TRCBBCTLR);
  etm4_write_reg32(etmdev, etmdev->trcid, TRCTRACEIDR);
  etm4_write_reg32(etmdev, config->vinst_ctrl, TRCVICTLR);
  etm4_write_reg32(etmdev, config->viiectlr, TRCVIIECTLR);
  etm4_write_reg32(etmdev, config->vissctlr, TRCVISSCTLR);
  if (etmdev->nr_pe_cmp)
    {
      etm4_write_reg32(etmdev, config->vipcssctlr, TRCVIPCSSCTLR);
    }

  if (etmdev->nrseqstate)
    {
      etm4_write_reg32(etmdev, config->seq_rst, TRCSEQRSTEVR);
      etm4_write_reg32(etmdev, config->seq_state, TRCSEQSTR);

      for (i = 0; i < etmdev->nrseqstate - 1; i++)
        {
          etm4_write_reg32(etmdev, config->seq_ctrl[i], TRCSEQEVRN(i));
        }
    }

  etm4_write_reg32(etmdev, config->ext_inp, TRCEXTINSELR);
  for (i = 0; i < etmdev->nr_cntr; i++)
    {
      etm4_write_reg32(etmdev, config->cntrldvr[i], TRCCNTRLDVRN(i));
      etm4_write_reg32(etmdev, config->cntr_ctrl[i], TRCCNTCTLRN(i));
      etm4_write_reg32(etmdev, config->cntr_val[i], TRCCNTVRN(i));
    }

  /* Resource selector pair 0 is always implemented and reserved.
   * As such start at 2.
   */

  for (i = 2; i < etmdev->nr_resource * 2; i++)
    {
      etm4_write_reg32(etmdev, config->res_ctrl[i], TRCRSCTLRN(i));
    }

  for (i = 0; i < etmdev->nr_ss_cmp; i++)
    {
      /* always clear status bit on restart if using single-shot */

      if (config->ss_ctrl[i] || config->ss_pe_cmp[i])
        {
          config->ss_status[i] &= ~TRCSSCSRN_STATUS;
        }

      etm4_write_reg32(etmdev, config->ss_ctrl[i], TRCSSCCRN(i));
      etm4_write_reg32(etmdev, config->ss_status[i], TRCSSCSRN(i));
      if (etm4_sspcicrn_present(etmdev, i))
        {
          etm4_write_reg32(etmdev, config->ss_pe_cmp[i], TRCSSPCICRN(i));
        }
    }

  for (i = 0; i < etmdev->nr_addr_cmp * 2; i++)
    {
      etm4_write_reg64(etmdev, config->addr_val[i], TRCACVRN(i));
      etm4_write_reg64(etmdev, config->addr_acc[i], TRCACATRN(i));
    }

  for (i = 0; i < etmdev->numcidc; i++)
    {
      etm4_write_reg64(etmdev, config->ctxid_pid[i], TRCCIDCVRN(i));
    }

  etm4_write_reg32(etmdev, config->ctxid_mask0, TRCCIDCCTLR0);
  if (etmdev->numcidc > 4)
    {
      etm4_write_reg32(etmdev, config->ctxid_mask1, TRCCIDCCTLR1);
    }

  for (i = 0; i < etmdev->numvmidc; i++)
    {
      etm4_write_reg64(etmdev, config->vmid_val[i], TRCVMIDCVRN(i));
    }

  etm4_write_reg32(etmdev, config->vmid_mask0, TRCVMIDCCTLR0);
  if (etmdev->numvmidc > 4)
    {
      etm4_write_reg32(etmdev, config->vmid_mask1, TRCVMIDCCTLR1);
    }

  if (!etmdev->skip_power_up)
    {
      uint32_t trcpdcr = etm4_read_reg32(etmdev, TRCPDCR);

      /* Request to keep the trace unit powered and also
       * emulation of powerdown
       */

      etm4_write_reg32(etmdev, trcpdcr | TRCPDCR_PU, TRCPDCR);
    }

  /* Enable the trace unit */

  etm4_write_reg32(etmdev, 1, TRCPRGCTLR);

  /* wait for TRCSTATR.IDLE to go back down to '0' */

  if (etm4_timeout(etmdev, 0, TRCSTATR_IDLE_BIT, TRCSTATR))
    {
      cserr("timeout while waiting for Idle Trace Status\n");
    }

  etm4_lock(etmdev);
}

/****************************************************************************
 * Name: etm4_enable
 *
 * Description:
 *   Enable the ETMv4 device.
 *
 * Input Parameters:
 *   csdev - Pointer to the coresight device structure.
 *
 * Returned Value:
 *   Zero on success; negative error code on failure.
 *
 ****************************************************************************/

static int etm4_enable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_etm4_dev_s *etmdev =
      (FAR struct coresight_etm4_dev_s *)csdev;
  int ret;

  ret = etm4_claim_device(etmdev);

  if (ret < 0)
    {
      return ret;
    }

  etm4_enable_hw(etmdev);
  return ret;
}

/****************************************************************************
 * Name: etm4_disable_hw
 *
 * Description:
 *   Disable the ETMv4 hardware by resetting various trace registers.
 *
 * Input Parameters:
 *   etmdev - Pointer to the ETMv4 device structure.
 *
 ****************************************************************************/

static void etm4_disable_hw(FAR struct coresight_etm4_dev_s *etmdev)
{
  FAR struct etm4_config_s *config = &etmdev->cfg;
  uint32_t control;
  int i;

  etm4_unlock(etmdev);

  if (!etmdev->skip_power_up)
    {
      /* power can be removed from the trace unit now */

      control = etm4_read_reg32(etmdev, TRCPDCR);
      control &= ~TRCPDCR_PU;
      etm4_write_reg32(etmdev, control, TRCPDCR);
    }

  control = etm4_read_reg32(etmdev, TRCPRGCTLR);

  /* EN, bit[0] Trace unit enable bit */

  control &= ~0x1;

  /* If the CPU supports v8.4 Trace filter Control,
   * set the ETM to trace prohibited region.
   */

  etm4_write_reg32(etmdev, control, TRCPRGCTLR);

  /* wait for TRCSTATR.PMSTABLE to go to '1' */

  if (etm4_timeout(etmdev, 1, TRCSTATR_PMSTABLE_BIT, TRCSTATR))
    {
      cserr("timeout while waiting for PM stable Trace Status\n");
    }

  /* read the status of the single shot comparators */

  for (i = 0; i < etmdev->nr_ss_cmp; i++)
    {
      config->ss_status[i] =
        etm4_read_reg32(etmdev, TRCSSCSRN(i));
    }

  /* read back the current counter values */

  for (i = 0; i < etmdev->nr_cntr; i++)
    {
      config->cntr_val[i] =
        etm4_read_reg32(etmdev, TRCCNTVRN(i));
    }

  etm4_lock(etmdev);
}

/****************************************************************************
 * Name: etm4_disable
 *
 * Description:
 *   Disable the ETMv4 device.
 *
 * Input Parameters:
 *   csdev - Pointer to the coresight device structure.
 *
 ****************************************************************************/

static void etm4_disable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_etm4_dev_s *etmdev =
    (FAR struct coresight_etm4_dev_s *)csdev;

  etm4_disable_hw(etmdev);
  etm4_disclaim_device(etmdev);
}

/****************************************************************************
 * Name: etm4_enable_trace_filtering
 *
 * Description:
 *   Configure trace filtering for the ETMv4 device if supported by the CPU.
 *
 * Input Parameters:
 *   etmdev - Pointer to the coresight ETM4 device structure.
 *
 ****************************************************************************/

static void etm4_enable_trace_filtering(struct coresight_etm4_dev_s *etmdev)
{
  uint64_t dfr0 = read_sysreg(id_aa64dfr0_el1);

  if (!ID_AA64DFR0_EL1_TRACEFILT(dfr0))
    {
      cserr("Trace Filter feature is not support");
      return;
    }

  /* If the CPU supports v8.4 SelfHosted Tracing, enable
   * tracing at the kernel EL1 and EL0, forcing to use the
   * virtual time as the timestamp.
   */

  etmdev->trfcr = TRFCR_ELX_TS_VIRTUAL | TRFCR_ELX_E1TRE | TRFCR_ELX_E0TRE;

  /* If we are running at EL2, allow tracing the CONTEXTIDR_EL2. */

  if (etm4_is_in_el2())
    {
      etmdev->trfcr |= TRFCR_EL2_CX;
    }

  write_sysreg(etmdev->trfcr, trfcr_el1);
}

/****************************************************************************
 * Name: etm4_init_arch_data
 *
 * Description:
 *   Initialize the architecture-specific data for the ETMv4 device.
 *
 * Input Parameters:
 *   etmdev - Pointer to the ETMv4 device structure.
 *
 ****************************************************************************/

static void etm4_init_arch_data(FAR struct coresight_etm4_dev_s *etmdev)
{
  uint32_t etmidr;
  uint32_t devarch;
  int i;

  etmdev->skip_power_up = true;

  /* Make sure all registers are accessible */

  etm4_unlock(etmdev);
  etm4_os_unlock(etmdev);

  devarch = read_sysreg(trcdevarch);
  etmdev->arch = etm4_devarch_to_arch(devarch);

  /* find all capabilities of the tracing unit */

  etmidr = etm4_read_reg32(etmdev, TRCIDR0);
  etmdev->instrp0 = (BMVAL(etmidr, 1, 2) == 0x3);
  etmdev->trcbb = etmidr & TRCIDR0_TRCBB;
  etmdev->trccond = etmidr & TRCIDR0_TRCCOND;
  etmdev->trccci = etmidr & TRCIDR0_TRCCCI;
  etmdev->retstack = etmidr & TRCIDR0_RETSTACK;
  etmdev->nr_event = BMVAL(etmidr, 10, 11);
  etmdev->q_support = BMVAL(etmidr, 15, 16);
  etmdev->ts_size = BMVAL(etmidr, 24, 28);

  etmidr = etm4_read_reg32(etmdev, TRCIDR2);
  etmdev->ctxid_size = BMVAL(etmidr, 5, 9);
  etmdev->vmid_size = BMVAL(etmidr, 10, 14);
  etmdev->ccsize = BMVAL(etmidr, 25, 28);

  etmidr = etm4_read_reg32(etmdev, TRCIDR3);
  etmdev->ccitmin = BMVAL(etmidr, 0, 11);

  etmdev->s_ex_level = BMVAL(etmidr, 16, 19);
  etmdev->cfg.s_ex_level = etmdev->s_ex_level;
  etmdev->ns_ex_level = BMVAL(etmidr, 20, 23);
  etmdev->trc_error = etmidr & TRCIDR3_TRCERR;
  etmdev->syncpr = etmidr & TRCIDR3_SYNCPR;
  etmdev->stallctl = etmidr & TRCIDR3_STALLCTL;
  etmdev->sysstall = etmidr & TRCIDR3_SYSSTALL;
  etmdev->nr_pe =  (BMVAL(etmidr, 12, 13) << 3) |
         BMVAL(etmidr, 28, 30);
  etmdev->nooverflow = etmidr & TRCIDR3_NOOVERFLOW;

  etmidr = etm4_read_reg32(etmdev, TRCIDR4);
  etmdev->nr_addr_cmp = BMVAL(etmidr, 0, 3);
  etmdev->nr_pe_cmp = BMVAL(etmidr, 12, 15);
  etmdev->nr_resource = BMVAL(etmidr, 16, 19);
  if (etmdev->arch < ETM4_ARCH_V4_3 || etmdev->nr_resource > 0)
    {
      etmdev->nr_resource += 1;
    }

  etmdev->nr_ss_cmp = BMVAL(etmidr, 20, 23);
  for (i = 0; i < etmdev->nr_ss_cmp; i++)
    {
      etmdev->cfg.ss_status[i] =
        etm4_read_reg32(etmdev, TRCSSCSRN(i));
    }

  etmdev->numcidc = BMVAL(etmidr, 24, 27);
  etmdev->numvmidc = BMVAL(etmidr, 28, 31);

  etmidr = etm4_read_reg32(etmdev, TRCIDR5);
  etmdev->nr_ext_inp = BMVAL(etmidr, 0, 8);
  etmdev->trcid_size = BMVAL(etmidr, 16, 21);
  etmdev->atbtrig = etmidr & TRCIDR5_ATBTRIG;
  etmdev->lpoverride = (etmidr & TRCIDR5_LPOVERRIDE) &&
                       (!etmdev->skip_power_up);
  etmdev->nrseqstate = BMVAL(etmidr, 25, 27);
  etmdev->nr_cntr = BMVAL(etmidr, 28, 30);
  etm4_lock(etmdev);
  etm4_enable_trace_filtering(etmdev);
}

/****************************************************************************
 * Name: etm4_set_default
 *
 * Description:
 *   Set default configuration values for the ETMv4 device.
 *
 * Input Parameters:
 *   config - Pointer to the ETMv4 configuration structure.
 *
 ****************************************************************************/

static void etm4_set_default(FAR struct etm4_config_s *config)
{
  /* enable trace synchronization every 4096 bytes, if available */

  config->syncfreq = 0xc;

  /* TRCVICTLR::EVENT = 0x01, select the always on logic */

  config->vinst_ctrl = BIT(0);

  /* TRCVICTLR::SSSTATUS == 1, the start-stop logic is
   * in the started state
   */

  config->vinst_ctrl |= TRCVICTLR_SSSTATUS;
  config->mode |= ETM4_MODE_VIEWINST_STARTSTOP;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etm4_config
 *
 * Description:
 *   Configure the ETMv4 device based on the provided configuration.
 *
 * Input Parameters:
 *   etmdev - Pointer to the ETMv4 device structure.
 *   config - Pointer to the ETMv4 configuration structure.
 *
 ****************************************************************************/

int etm4_config(FAR struct coresight_etm4_dev_s *etmdev,
                FAR const struct etm4_config_s *config)
{
  etm4_unlock(etmdev);
  etm4_os_unlock(etmdev);

  memcpy(&etmdev->cfg, config, sizeof(struct etm4_config_s));

  etm4_lock(etmdev);

  return 0;
}

/****************************************************************************
 * Name: etm4_register
 *
 * Description:
 *   Register an ETMv4 device with the system.
 *
 * Input Parameters:
 *   desc - Pointer to the description of the coresight device.
 *
 * Returned Value:
 *   Pointer to the ETMv4 device structure on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_etm4_dev_s *
etm4_register(FAR const struct coresight_desc_s *desc)
{
  FAR struct coresight_etm4_dev_s *etmdev;
  FAR struct coresight_dev_s *csdev;
  int ret;

  etmdev = kmm_zalloc(sizeof(struct coresight_etm4_dev_s));
  if (etmdev == NULL)
    {
      cserr("%s:malloc failed!\n", desc->name);
      return NULL;
    }

  etmdev->cpu = desc->cpu;
  etmdev->csdev.addr = desc->addr;
  etm4_init_arch_data(etmdev);

  etmdev->trcid = coresight_get_cpu_trace_id(etmdev->cpu);
  etm4_set_default(&etmdev->cfg);

  csdev = &etmdev->csdev;
  csdev->ops = &g_etm4_ops;
  ret = coresight_register(csdev, desc);
  if (ret < 0)
    {
      kmm_free(etmdev);
      cserr("%s:register failed\n", desc->name);
      return NULL;
    }

  return etmdev;
}

/****************************************************************************
 * Name: etm4_unregister
 *
 * Description:
 *   Unregister an ETMv4 device from the system.
 *
 * Input Parameters:
 *   etmdev - Pointer to the ETMv4 device structure.
 *
 ****************************************************************************/

void etm4_unregister(FAR struct coresight_etm4_dev_s *etmdev)
{
  coresight_unregister(&etmdev->csdev);
  kmm_free(etmdev);
}
