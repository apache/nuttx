/****************************************************************************
 * arch/arm/src/sam34/sam_tc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM_TC_H
#define __ARCH_ARM_SRC_SAM34_SAM_TC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_tc.h"

#ifdef CONFIG_TIMER

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

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tcinitialize
 *
 * Description:
 *   Initialize the timer.  The timer is initialized and
 *   registers as 'devpath.  The initial state of the timer is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the timer.  This should be of the form
 *     /dev/tc0
 *   irq - irq associated with the timer
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_TC0) || defined(CONFIG_SAM34_TC1) || \
    defined(CONFIG_SAM34_TC2) || defined(CONFIG_SAM34_TC3) || \
    defined(CONFIG_SAM34_TC4) || defined(CONFIG_SAM34_TC5)
void sam_tcinitialize(const char *devpath, int irq);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_TIMER */
#endif /* __ARCH_ARM_SRC_SAM34_SAM_TC_H */
