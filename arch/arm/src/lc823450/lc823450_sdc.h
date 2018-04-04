/****************************************************************************
 * arch/arm/src/lc823450/lc823450_sdc.h
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SDC_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SDC_H

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

int lc823450_sdc_refversion(void);
int lc823450_sdc_clearcardinfo(uint32_t ch);

int lc823450_sdc_initialize(uint32_t ch);
int lc823450_sdc_finalize(uint32_t ch);
int lc823450_sdc_checkcarddetect(uint32_t ch);
int lc823450_sdc_identifycard(uint32_t ch);
int lc823450_sdc_setclock(uint32_t ch, uint32_t limitclk, uint32_t sysclk);
int lc823450_sdc_refmediatype(uint32_t ch);
int lc823450_sdc_getcardsize(uint32_t ch, unsigned long *psecnum,
                             unsigned long *psecsize);
int lc823450_sdc_readsector(uint32_t ch, unsigned long addr, unsigned short cnt,
                            void *pbuf, unsigned long type);
int lc823450_sdc_writesector(uint32_t ch, unsigned long addr, unsigned short cnt,
                             void *pbuf, unsigned long type);
int lc823450_sdc_checktrim(uint32_t ch);
int lc823450_sdc_trimsector(uint32_t ch, unsigned long addr, unsigned short cnt);
int lc823450_sdc_cachectl(uint32_t ch, int ctrl);
int lc823450_sdc_changespeedmode(uint32_t ch, int mode);
int lc823450_sdc_getcid(uint32_t ch, char *cidstr, int length);
int lc823450_sdc_locked(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SDC_H */
