/****************************************************************************
 * arch/arm/src/lc823450/lc823450_clockconfig.h
 *
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Public Functions
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
