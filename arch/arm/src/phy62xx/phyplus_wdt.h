/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_wdt.h
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

#ifndef __ARCH_ARM_SRC_PHY62XX_PHYPLUS_WDT_H
#define __ARCH_ARM_SRC_PHY62XX_PHYPLUS_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

typedef enum
{
    WDG_2S =   0,
    WDG_4S =   1,
    WDG_8S =   2,
    WDG_16S =  3,
    WDG_32S =  4,
    WDG_64S =  5,
    WDG_128S = 6,
    WDG_256S = 7
} WDG_CYCLE_Type_e;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void phyplus_wdt_initialize(const char *devpath);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_WATCHDOG */
#endif /* __ARCH_ARM_SRC_PHY62XX_PHYPLUS_WDT_H */
