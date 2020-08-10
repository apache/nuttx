/****************************************************************************
 * arch/arm/src/lc823450/lc823450_sdc.h
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
 * Public Function Prototypes
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
int lc823450_sdc_readsector(uint32_t ch, unsigned long addr,
                            unsigned short cnt,
                            void *pbuf, unsigned long type);
int lc823450_sdc_writesector(uint32_t ch, unsigned long addr,
                             unsigned short cnt,
                             void *pbuf, unsigned long type);
int lc823450_sdc_checktrim(uint32_t ch);
int lc823450_sdc_trimsector(uint32_t ch, unsigned long addr,
                            unsigned short cnt);
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
