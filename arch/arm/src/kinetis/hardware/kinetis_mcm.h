/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_mcm.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MCM_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_MCM_PLASC_OFFSET    0x0008 /* Crossbar switch (AXBS) slave configuration */
#define KINETIS_MCM_PLAMC_OFFSET    0x000a /* Crossbar switch (AXBS) master configuration */
#define KINETIS_MCM_SRAMAP_OFFSET   0x000c /* SRAM arbitration and protection */
#define KINETIS_MCM_ISR_OFFSET      0x0010 /* Interrupt status register */
#define KINETIS_MCM_ETBCC_OFFSET    0x0014 /* ETB counter control register */
#define KINETIS_MCM_ETBRL_OFFSET    0x0018 /* ETB reload register */
#define KINETIS_MCM_ETBCNT_OFFSET   0x001c /* ETB counter value register */
#ifdef KINETIS_K64
#  define KINETIS_MCM_PID_OFFSET    0x0030 /* Process ID register */
#endif

/* Register Addresses *******************************************************/

#define KINETIS_MCM_PLASC           (KINETIS_MCM_BASE+KINETIS_MCM_PLASC_OFFSET)
#define KINETIS_MCM_PLAMC           (KINETIS_MCM_BASE+KINETIS_MCM_PLAMC_OFFSET)
#define KINETIS_MCM_SRAMAP          (KINETIS_MCM_BASE+KINETIS_MCM_SRAMAP_OFFSET)
#define KINETIS_MCM_ISR             (KINETIS_MCM_BASE+KINETIS_MCM_ISR_OFFSET)
#define KINETIS_MCM_ETBCC           (KINETIS_MCM_BASE+KINETIS_MCM_ETBCC_OFFSET)
#define KINETIS_MCM_ETBRL           (KINETIS_MCM_BASE+KINETIS_MCM_ETBRL_OFFSET)
#define KINETIS_MCM_ETBCNT          (KINETIS_MCM_BASE+KINETIS_MCM_ETBCNT_OFFSET)
#ifdef KINETIS_K64
#  define KINETIS_MCM_PID           (KINETIS_MCM_BASE+KINETIS_MCM_PID_OFFSET)
#endif

/* Register Bit Definitions *************************************************/

/* Crossbar switch (AXBS) slave configuration */

#define MCM_PLASC_ASC_SHIFT         (0)       /* Bits 0-7: Each bit in the ASC field
                                               * indicates if there is a corresponding
                                               * connection to the crossbar switch's
                                               * slave input port. */
#define MCM_PLASC_ASC_MASK          (0xff << MCM_PLASC_ASC_SHIFT)
#define MCM_PLASC_ASC(n)            ((1 << (n)) << MCM_PLASC_ASC_SHIFT)
                                              /* Bits 8-15: Reserved */

/* Crossbar switch (AXBS) master configuration */

#define MCM_PLAMC_AMC_SHIFT         (0)       /* Bits 0-7: Each bit in the AMC field
                                               * indicates if there is a corresponding
                                               * connection to the AXBS master input port. */
#define MCM_PLAMC_AMC_MASK          (0xff << MCM_PLAMC_AMC_SHIFT)
#define MCM_PLAMC_AMC(n)            ((1 << (n)) << MCM_PLAMC_AMC_SHIFT)
                                              /* Bits 8-15: Reserved */

/* SRAM arbitration and protection */

                                              /* Bits 0-23: Reserved */
#define MCM_SRAMAP_SRAMUAP_SHIFT    (24)      /* Bits 24-25: SRAM_U arbitration priority */
#define MCM_SRAMAP_SRAMUAP_MASK     (3 << MCM_SRAMAP_SRAMUAP_SHIFT)
#  define MCM_SRAMAP_SRAMUAP_RR     (0 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Round robin */
#  define MCM_SRAMAP_SRAMUAP_SRR    (1 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Special round robin */
#  define MCM_SRAMAP_SRAMUAP_FIXED1 (2 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Fixed pri. Proc highest/backdoor lowest */
#  define MCM_SRAMAP_SRAMUAP_FIXED2 (3 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Fixed pri. Backdoor highest/proc lowest */

#define MCM_SRAMAP_SRAMUWP          (1 << 26) /* Bit 26: SRAM_U write protect */
                                              /* Bit 27: Reserved */
#define MCM_SRAMAP_SRAMLAP_SHIFT    (28)      /* Bits 28-29: SRAM_L arbitration priority */
#define MCM_SRAMAP_SRAMLAP_MASK     (3 << MCM_SRAMAP_SRAMLAP_SHIFT)
#  define MCM_SRAMAP_SRAMLAP_RR     (0 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Round robin */
#  define MCM_SRAMAP_SRAMLAP_SRR    (1 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Special round robin */
#  define MCM_SRAMAP_SRAMLAP_FIXED1 (2 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Fixed pri. Proc highest/backdoor lowest */
#  define MCM_SRAMAP_SRAMLAP_FIXED2 (3 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Fixed pri. Backdoor highest/proc lowest */

#define MCM_SRAMAP_SRAMLWP          (1 << 30) /* Bit 30: SRAM_L write protect */
                                              /* Bit 31: Reserved */

/* Interrupt status register */

                                              /* Bit 0: Reserved */
#define MCM_ISR_IRQ                 (1 << 1)  /* Bit 1:  Normal interrupt pending */
#define MCM_ISR_NMI                 (1 << 2)  /* Bit 2:  Non-maskable interrupt pending */
                                              /* Bits 3-31: Reserved */

/* ETB counter control register */

#define MCM_ETBCC_CNTEN             (1 << 0)  /* Bit 0:  Counter enable */
#define MCM_ETBCC_RSPT_SHIFT        (1)       /* Bits 1-2: Response type */
#define MCM_ETBCC_RSPT_MASK         (3 << MCM_ETBCC_RSPT_SHIFT)
#  define MCM_ETBCC_RSPT_NONE       (0 << MCM_ETBCC_RSPT_SHIFT) /* No response when ETB count expires */
#  define MCM_ETBCC_RSPT_INT        (1 << MCM_ETBCC_RSPT_SHIFT) /* Normal interrupt when ETB count expires */
#  define MCM_ETBCC_RSPT_NMI        (2 << MCM_ETBCC_RSPT_SHIFT) /* NMI when ETB count expires */
#  define MCM_ETBCC_RSPT_HALT       (3 << MCM_ETBCC_RSPT_SHIFT) /* Debug halt when ETB count expires */

#define MCM_ETBCC_RLRQ              (1 << 3)  /* Bit 3:  Reload request */
#define MCM_ETBCC_ETDIS             (1 << 4)  /* Bit 4:  ETM-to-TPIU disable */
#define MCM_ETBCC_ITDIS             (1 << 5)  /* Bit 5:  ITM-to-TPIU disable */
                                              /* Bits 6-31: Reserved */

/* ETB reload register */

#define MCM_ETBRL_RELOAD_SHIFT      (0)       /* Bits 0-10: Byte count reload value */
#define MCM_ETBRL_RELOAD_MASK       (0x7ff << MCM_ETBRL_RELOAD_SHIFT)
                                              /* Bits 11-31: Reserved */

/* ETB counter value register */

#define MCM_ETBCNT_COUNTER_SHIFT    (0)       /* Bits 0-10: Byte count counter value */
#define MCM_ETBCNT_COUNTER_MASK     (0x7ff << MCM_ETBCNT_COUNTER_SHIFT)
                                              /* Bits 11-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MCM_H */
