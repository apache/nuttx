/****************************************************************************
 * arch/arm/src/lc823450/lc823450_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_LC823450_LC823450_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_OSCSYS_REGBASE 0x40040000

#define OSCCNT     (LC823450_OSCSYS_REGBASE + 0x00)
#define   OSCCNT_SCKSEL_MASK   (3 << 0)
#define   OSCCNT_SCKSEL_RC     (0 << 0)
#define   OSCCNT_SCKSEL_MAIN   (1 << 0)
#define   OSCCNT_SCKSEL_RTC    (2 << 0)
#define   OSCCNT_MCSEL         (1 << 2)
#define   OSCCNT_XT1EN         (1 << 7)
#define   OSCCNT_MAINDIV_MASK  (7 << 8)
#define   OSCCNT_MAINDIV_1     (0 << 8)
#define   OSCCNT_MAINDIV_2     (1 << 8)
#define   OSCCNT_MAINDIV_4     (2 << 8)
#define   OSCCNT_MAINDIV_8     (3 << 8)
#define   OSCCNT_MAINDIV_16    (4 << 8)
#define PLLREFCNT  (LC823450_OSCSYS_REGBASE + 0x04)
#define PERICLKDIV (LC823450_OSCSYS_REGBASE + 0x08)
#define   PERICLKDIV_HCLKDIV_MASK  (0x3f << 0)
#define FCLKCNT    (LC823450_OSCSYS_REGBASE + 0x0c)
#define   FCLKCNT_SFDIV1      (0 << 24)
#define   FCLKCNT_SFDIV2      (1 << 24)
#define   FCLKCNT_SFDIV4      (2 << 24)
#define   FCLKCNT_SFDIV8      (3 << 24)
#define AUDCLKCNT  (LC823450_OSCSYS_REGBASE + 0x14)
#define IMCNT      (LC823450_OSCSYS_REGBASE + 0x20)
#define CORESTS    (LC823450_OSCSYS_REGBASE + 0x40)

#define LC823450_SYSTEMPLL_BASE  0x40041000
#define PLL1CNT    (LC823450_SYSTEMPLL_BASE + 0x00)
#define   PLL1CNT_STYB        (1 << 1)
#define   PLL1CNT_RSTB        (1 << 0)
#define PLL1MDIV   (LC823450_SYSTEMPLL_BASE + 0x04)
#define PLL1NDIV   (LC823450_SYSTEMPLL_BASE + 0x08)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint32_t lc823450_get_systemfreq(void);

#ifndef CONFIG_DVFS
uint32_t lc823450_get_apb(void);
#endif

uint32_t lc823450_get_ahb(void);
void lc823450_clockconfig(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_CLOCKCONFIG_H */
