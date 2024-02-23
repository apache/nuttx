/****************************************************************************
 * arch/x86_64/include/hpet.h
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

#ifndef __ARCH_X86_64_INCLUDE_HPET_H
#define __ARCH_X86_64_INCLUDE_HPET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definitions */

#define HPET_GCAPID_OFFSET          (0x00)
#define HPET_GCONF_OFFSET           (0x10)
#define HPET_GISR_OFFSET            (0x20)
#define HPET_MCNTR_OFFSET           (0xf0)
#define HPET_TCONF_OFFSET(n)        (0x100 + (0x20 * (n)))
#define HPET_TCOMP_OFFSET(n)        (0x108 + (0x20 * (n)))
#define HPET_TFSB_OFFSET(n)         (0x110 + (0x20 * (n)))

/* General Capabilities and ID Register */

#define HPET_GCAPID_REVID_SHIFT     (0ul)     /* Bits 0-7: Revistion */
#  define HPET_GCAPID_REVID_MASK    (0xfful << HPET_GCAPID_REVID_SHIFT)
#define HPET_GCAPID_NUMTIM_SHIFT    (8ul)     /* Bits 8-12: Number of Timers */
#define HPET_GCAPID_NUMTIM_MASK     (0x1ful << HPET_GCAPID_NUMTIM_SHIFT)
#define HPET_GCAPID_COUNTSIZE       (1 << 13) /* Bit 13: Counter size, 0: 32 bit, 1: 64 bit */
                                              /* Bit 14: Reserved */
#define HPET_GCAPID_LEGROUTE        (1 << 15) /* Bit 15: LegacyReplacement Route Capable */
#define HPET_GCAPID_VENDORID_SHIFT  (16ul)    /* Bits 16-31: Vendor ID */
#  define HPET_GCAPID_VENDORID_MASK (0x7ffful << HPET_GCAPID_VENDORID_SHIFT)
#define HPET_GCAPID_CLKPER_SHIFT    (32ul)    /* Bits 32-63: Main Counter Tick Period in ps */
#  define HPET_GCAPID_CLKPER_MASK   (0x7ffffffful << HPET_GCAPID_CLKPER_SHIFT)

/* General Configuration Register */

#define HPET_GCONF_LEGERT           (1 << 0) /* Bit 0: LegacyReplacement Route */
#define HPET_GCONF_ENABLE           (1 << 1) /* Bit 1: Overall Enable */

/* General Interrupt Status Register */

#define HPET_GISR_TINT(n)           (1 << (n)) /* Timer n Interrupt Active */

/* Timer N Configuration and Capabilities Register */

                                             /* Bit 0: Reserved */
#define HPET_TCONF_INTTYPE          (1 << 1) /* Bit 1: Timer n Interrupt Type (0: edge, 1: level) */
#define HPET_TCONF_INTEN            (1 << 2) /* Bit 2: Timer n Interrupt Enable */
#define HPET_TCONF_TYPE             (1 << 3) /* Bit 3: Timer n Type (0: non-periodic, 1: periodic) */
#define HPET_TCONF_PERCAP           (1 << 4) /* Bit 4: Timer n Periodic Interrupt Capable */
#define HPET_TCONF_SIZECAP          (1 << 5) /* Bit 5: Timer n Size */
#define HPET_TCONF_VALSET           (1 << 6) /* Bit 6: Timer n Value Set */
                                             /* Bit 7: Reserved */
#define HPET_TCONF_32MODE           (1 << 8) /* Bit 8: Timer n 32-bit mode */
#define HPET_TCONF_INTROUTE_SHIFT   (9)      /* Bits 9-13: Timer n Interrupt Route */
#  define HPET_TCONF_INTROUTE_MASK  (0x7f << HPET_TCONF_INTROUTE_SHIFT)
#  define HPET_TCONF_INTROUTE(n)    (((n) << HPET_TCONF_INTROUTE_SHIFT) & HPET_TCONF_INTROUTE_MASK)
#define HPET_TCONF_FSBEN            (1 << 14) /* Bit 14: Timer n FSB Interrupt Enable */
#define HPET_TCONF_FSBCAP           (1 << 15) /* Bit 15: Timer n FSB Interrupt Delivery */
                                              /* Bits 16-31: Reserved */
#define HPET_TCONF_ROUTECAP_SHIFT   (32)      /* Bits 32-63: Timer n Interrupt Routing Capability */
#  define HPET_TCONF_ROUTECAP_MASK  (0x7ffffffful << HPET_TCONF_ROUTECAP_SHIFT)

/* Timer N FSB Interrupt Route Register */

#define HPET_TFSB_INT_VAL_SHIFT     (0)
#define HPET_TFSB_INT_VAL_MASK      (0x00000000ffffffff)
#define HPET_TFSB_INT_ADDR_SHIFT    (31)
#define HPET_TFSB_INT_ADDR_MASK     (0xffffffff00000000)

/* HPET register space */

#define HPET_REGION_SIZE            (1024)

#endif /* __ARCH_X86_64_INCLUDE_HPET_H */
