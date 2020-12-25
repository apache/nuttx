/****************************************************************************
 * arch/arm/src/lc823450/lc823450_dvfs2.h
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_DVFS2_H
#define __ARCH_ARM_SRC_LC823450_LC823450_DVFS2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
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

void lc823450_dvfs_get_idletime(uint64_t idaletime[]);

void lc823450_dvfs_set_min(uint8_t id, uint16_t mhz);
void lc823450_dvfs_enter_idle(void);
void lc823450_dvfs_exit_idle(int irq);
int  lc823450_dvfs_set_freq(int freq);
void lc823450_dvfs_tick_callback(void);
int lc823450_dvfs_oneshot(int irq, uint32_t *regs, FAR void *arg);

int  dvfs_procfs_register(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_DVFS2_H */
