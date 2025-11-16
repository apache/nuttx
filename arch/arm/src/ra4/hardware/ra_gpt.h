/****************************************************************************
 * arch/arm/src/ra4/hardware/ra_gpt.h
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

#ifndef __ARCH_ARM_SRC_RA4_HARDWARE_RA_GPT_H
#define __ARCH_ARM_SRC_RA4_HARDWARE_RA_GPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Register Offsets *********************************************************/

#define R_GPT_GTWP_OFFSET        0x0000  /* Write Protection Register */
#define R_GPT_GTSTR_OFFSET       0x0004  /* Software Start Register */
#define R_GPT_GTSTP_OFFSET       0x0008  /* Software Stop Register */
#define R_GPT_GTCLR_OFFSET       0x000C  /* Software Clear Register */
#define R_GPT_GTSSR_OFFSET       0x0010  /* Start Source Select Register */
#define R_GPT_GTPSR_OFFSET       0x0014  /* Stop Source Select Register */
#define R_GPT_GTCSR_OFFSET       0x0018  /* Clear Source Select Register */
#define R_GPT_GTUPSR_OFFSET      0x001C  /* Up Count Source Select Register */
#define R_GPT_GTDNSR_OFFSET      0x0020  /* Down Count Source Select Register */
#define R_GPT_GTICASR_OFFSET     0x0024  /* Input Capture Source Select Register A */
#define R_GPT_GTICBSR_OFFSET     0x0028  /* Input Capture Source Select Register B */
#define R_GPT_GTCR_OFFSET        0x002C  /* Control Register */
#define R_GPT_GTUDDTYC_OFFSET    0x0030  /* Count Direction and Duty Setting Register */
#define R_GPT_GTIOR_OFFSET       0x0034  /* I/O Control Register */
#define R_GPT_GTINTAD_OFFSET     0x0038  /* Interrupt Output Setting Register */
#define R_GPT_GTST_OFFSET        0x003C  /* Status Register (alias: GTSR) */
#define R_GPT_GTBER_OFFSET       0x0040  /* Buffer Enable Register */
#define R_GPT_GTCNT_OFFSET       0x0048  /* Counter Register */
#define R_GPT_GTCCRA_OFFSET      0x004C  /* Compare Capture Register A */
#define R_GPT_GTCCRB_OFFSET      0x0050  /* Compare Capture Register B */
#define R_GPT_GTCCRC_OFFSET      0x0054  /* Compare Capture Register C */
#define R_GPT_GTCCRE_OFFSET      0x0058  /* Compare Capture Register E */
#define R_GPT_GTCCRD_OFFSET      0x005C  /* Compare Capture Register D */
#define R_GPT_GTCCRF_OFFSET      0x0060  /* Compare Capture Register F */
#define R_GPT_GTPR_OFFSET        0x0064  /* Cycle Setting Register */
#define R_GPT_GTPBR_OFFSET       0x0068  /* Cycle Setting Buffer Register */
#define R_GPT_GTDTCR_OFFSET      0x0088  /* Dead Time Control Register */
#define R_GPT_GTDVU_OFFSET       0x008C  /* Dead Time Value Register U */

/* Register Bitfield Definitions ********************************************/

/* General PWM Timer Write-Protection Register (GTWP) */

#define R_GPT_GTWP_WP                 (1 << 0)                          /* Register Write Disable (b0) */
#define R_GPT_GTWP_PRKEY_SHIFT        (8)
#define R_GPT_GTWP_PRKEY_MASK         (0xFF << R_GPT_GTWP_PRKEY_SHIFT)  /* GTWP Key Code (b15:b8) */
#  define R_GPT_GTWP_PRKEY            (0 << 0xA5)                       /* Key code to enable write access */

/* General PWM Timer Software Start Register (GTSTR) */

#define R_GPT_GTSTR_CSTRT0            (1 << 0)
#define R_GPT_GTSTR_CSTRT7            (1 << 7)

/* General PWM Timer Software Stop Register (GTSTP) */

#define R_GPT_GTSTP_CSTOP0            (1 << 0)
#define R_GPT_GTSTP_CSTOP7            (1 << 7)

/* General PWM Timer Software Clear Register (GTCLR) */

#define R_GPT_GTCLR_CCLR0             (1 << 0)
#define R_GPT_GTCLR_CCLR7             (1 << 7)

/* General PWM Timer Count Direction and Duty Setting Register (GTUDDTYC) */

#define R_GPT_GTUDDTYC_UD                     (1 << 0)                              /* Count Direction Setting (b0) */
#define R_GPT_GTUDDTYC_UDF                    (1 << 1)                              /* Forcible Count Direction Setting (b1) */
#define R_GPT_GTUDDTYC_OADTY_SHIFT            (16)
#define R_GPT_GTUDDTYC_OADTY_MASK             (0x3 << R_GPT_GTUDDTYC_OADTY_SHIFT)   /* GTIOCA Output Duty Setting (b17:b16) */
#  define R_GPT_GTUDDTYC_OADTY_SET0           (0 << R_GPT_GTUDDTYC_OADTY_SHIFT)     /* GTIOCA pin duty depends on compare match */
#  define R_GPT_GTUDDTYC_OADTY_SET2           (2 << R_GPT_GTUDDTYC_OADTY_SHIFT)     /* GTIOCA pin duty 0% */
#  define R_GPT_GTUDDTYC_OADTY_SET3           (3 << R_GPT_GTUDDTYC_OADTY_SHIFT)     /* GTIOCA pin duty 100% */
#define R_GPT_GTUDDTYC_OADTYF                 (1 << 18)
#define R_GPT_GTUDDTYC_OADTYR                 (1 << 19)
#define R_GPT_GTUDDTYC_OBDTY_SHIFT            (24)
#define R_GPT_GTUDDTYC_OBDTY_MASK             (0x3 << R_GPT_GTUDDTYC_OBDTY_SHIFT)   /* GTIOCB Output Duty Setting (b25:b24) */
#  define R_GPT_GTUDDTYC_OBDTY_SET0           (0 << R_GPT_GTUDDTYC_OBDTY_SHIFT)     /* GTIOCB pin duty depends on compare match */
#  define R_GPT_GTUDDTYC_OBDTY_SET2           (2 << R_GPT_GTUDDTYC_OBDTY_SHIFT)     /* GTIOCB pin duty 0% */
#  define R_GPT_GTUDDTYC_OBDTY_SET3           (3 << R_GPT_GTUDDTYC_OBDTY_SHIFT)     /* GTIOCB pin duty 100% */
#define R_GPT_GTUDDTYC_OBDTYF                 (1 << 26)
#define R_GPT_GTUDDTYC_OBDTYR                 (1 << 27)

/* General PWM Timer I/O Control Register (GTIOR) */

#define R_GPT_GTIOR_GTIOA_SHIFT       (0)
#define R_GPT_GTIOR_GTIOA_MASK        (0x1F << R_GPT_GTIOR_GTIOA_SHIFT)           /* GTIOCA Pin Function Select (b4:b0) */
#  define R_GPT_GTIOR_GTIOA_SET6      (0x06 << R_GPT_GTIOR_GTIOA_SHIFT)           /* Initial out: L; cycle end: L; compare match: H */
#  define R_GPT_GTIOR_GTIOA_SET25     (0x19 << R_GPT_GTIOR_GTIOA_SHIFT)           /* Initial out: H; cycle end: H; compare match: L */
#define R_GPT_GTIOR_OADFLT            (1 << 6)                                    /* Output value at count stop */
#define R_GPT_GTIOR_OAHLD             (1 << 7)                                    /* Output value hold enable */
#define R_GPT_GTIOR_OAE               (1 << 8)                                    /* GTIOCA Output Enable */
#define R_GPT_GTIOR_OADF_SHIFT        (9)
#define R_GPT_GTIOR_OADF_MASK         (0x3 << R_GPT_GTIOR_OADF_SHIFT)             /* Output disable source select */
#define R_GPT_GTIOR_NFAEN             (1 << 12)                                   /* Noise filter enable */
#define R_GPT_GTIOR_NFCSA_SHIFT       (13)
#define R_GPT_GTIOR_NFCSA_MASK        (0x3 << R_GPT_GTIOR_NFCSA_SHIFT)            /* Noise filter clock select */
#define R_GPT_GTIOR_GTIOB_SHIFT       (16)
#define R_GPT_GTIOR_GTIOB_MASK        (0x1F << R_GPT_GTIOR_GTIOB_SHIFT)
#  define R_GPT_GTIOR_GTIOB_SET6      (0x06 << R_GPT_GTIOR_GTIOB_SHIFT)           /* Initial out: L; cycle end: L; compare match: H */
#  define R_GPT_GTIOR_GTIOB_SET25     (0x19 << R_GPT_GTIOR_GTIOB_SHIFT)           /* Initial out: H; cycle end: H; compare match: L */
#define R_GPT_GTIOR_OBDFLT            (1 << 22)
#define R_GPT_GTIOR_OBHLD             (1 << 23)
#define R_GPT_GTIOR_OBE               (1 << 24)
#define R_GPT_GTIOR_OBDF_SHIFT        (25)
#define R_GPT_GTIOR_OBDF_MASK         (0x3 << R_GPT_GTIOR_OBDF_SHIFT)
#define R_GPT_GTIOR_NFBEN             (1 << 28)
#define R_GPT_GTIOR_NFCSB_SHIFT       (29)
#define R_GPT_GTIOR_NFCSB_MASK        (0x3 << R_GPT_GTIOR_NFCSB_SHIFT)

/* General PWM Timer Status Register (GTST / GTSR) */

#define R_GPT_GTST_TCFA_SHIFT         (0)
#define R_GPT_GTST_TCFA               (1 << R_GPT_GTST_TCFA_SHIFT)  /* Match Flag A */

#define R_GPT_GTST_TCFB_SHIFT         (1)
#define R_GPT_GTST_TCFB               (1 << R_GPT_GTST_TCFB_SHIFT)  /* Match Flag B */

#define R_GPT_GTST_TCFC_SHIFT         (2)
#define R_GPT_GTST_TCFC               (1 << R_GPT_GTST_TCFC_SHIFT)  /* Match Flag C */

#define R_GPT_GTST_TCFD_SHIFT         (3)
#define R_GPT_GTST_TCFD               (1 << R_GPT_GTST_TCFD_SHIFT)  /* Match Flag D */

#define R_GPT_GTST_TCFE_SHIFT         (4)
#define R_GPT_GTST_TCFE               (1 << R_GPT_GTST_TCFE_SHIFT)  /* Match Flag E */

#define R_GPT_GTST_TCFF_SHIFT         (5)
#define R_GPT_GTST_TCFF               (1 << R_GPT_GTST_TCFF_SHIFT)  /* Match Flag F */

#define R_GPT_GTST_TCFPO_SHIFT        (6)
#define R_GPT_GTST_TCFPO              (1 << R_GPT_GTST_TCFPO_SHIFT) /* Overflow Flag */

#define R_GPT_GTST_TCFPU_SHIFT        (7)
#define R_GPT_GTST_TCFPU              (1 << R_GPT_GTST_TCFPU_SHIFT) /* Underflow Flag */

#define R_GPT_GTST_TUCF_SHIFT         (15)
#define R_GPT_GTST_TUCF               (1 << R_GPT_GTST_TUCF_SHIFT)  /* Count Direction Flag */

#define R_GPT_GTST_ODF_SHIFT          (24)
#define R_GPT_GTST_ODF                (1 << R_GPT_GTST_ODF_SHIFT)   /* Output Disable Flag */

#define R_GPT_GTST_OABHF_SHIFT        (29)
#define R_GPT_GTST_OABHF              (1 << R_GPT_GTST_OABHF_SHIFT) /* Output High Flag */

#define R_GPT_GTST_OABLF_SHIFT        (30)
#define R_GPT_GTST_OABLF              (1 << R_GPT_GTST_OABLF_SHIFT) /* Output Low Flag */

/* General PWM Timer Buffer Enable Register (GTBER) */

#define R_GPT_GTBER_BD0_SHIFT         (0)
#define R_GPT_GTBER_BD0               (1 << R_GPT_GTBER_BD0_SHIFT)  /* GTCCR Buffer Disable */

#define R_GPT_GTBER_BD1_SHIFT         (1)
#define R_GPT_GTBER_BD1               (1 << R_GPT_GTBER_BD1_SHIFT)  /* GTPR Buffer Disable */

#define R_GPT_GTBER_CCRA_SHIFT        (16)
#define R_GPT_GTBER_CCRA_MASK         (0x3 << R_GPT_GTBER_CCRA_SHIFT)

#define R_GPT_GTBER_CCRB_SHIFT        (18)
#define R_GPT_GTBER_CCRB_MASK         (0x3 << R_GPT_GTBER_CCRB_SHIFT)

#define R_GPT_GTBER_PR_SHIFT          (20)
#define R_GPT_GTBER_PR_MASK           (0x3 << R_GPT_GTBER_PR_SHIFT)

#define R_GPT_GTBER_CCRSWT_SHIFT      (22)
#define R_GPT_GTBER_CCRSWT            (1 << R_GPT_GTBER_CCRSWT_SHIFT)

/* General PWM Timer Control Register (GTCR) */

#define R_GPT_GTCR_CST                (1u << 0)                         /* Count Start (b0) */
#define R_GPT_GTCR_MD_SHIFT           (16)
#define R_GPT_GTCR_MD_MASK            (0x7u << R_GPT_GTCR_MD_SHIFT)
#  define R_GPT_GTCR_MD_MOD0          (0 << R_GPT_GTCR_MD_SHIFT)        /* Saw-wave PWM mode (single buffer or double buffer possible) */
#  define R_GPT_GTCR_MD_MOD1          (1 << R_GPT_GTCR_MD_SHIFT)        /* Saw-wave one-shot pulse mode (fixed buffer operation) */
#define R_GPT_GTCR_TPCS_SHIFT         (24)
#define R_GPT_GTCR_TPCS_MASK          (0x7u << R_GPT_GTCR_TPCS_SHIFT)
#  define R_GPT_GTCR_TPCS_DIV1        (0 << R_GPT_GTCR_TPCS_SHIFT)      /*  PCLKD/1 */
#  define R_GPT_GTCR_TPCS_DIV4        (1 << R_GPT_GTCR_TPCS_SHIFT)      /*  PCLKD/4 */
#  define R_GPT_GTCR_TPCS_DIV16       (2 << R_GPT_GTCR_TPCS_SHIFT)      /*  PCLKD/16 */
#  define R_GPT_GTCR_TPCS_DIV64       (3 << R_GPT_GTCR_TPCS_SHIFT)      /*  PCLKD/64 */
#  define R_GPT_GTCR_TPCS_DIV256      (4 << R_GPT_GTCR_TPCS_SHIFT)      /*  PCLKD/256 */
#  define R_GPT_GTCR_TPCS_DIV1024     (5 << R_GPT_GTCR_TPCS_SHIFT)      /*  PCLKD/1024 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA4_HARDWARE_RA_GPT_H */
