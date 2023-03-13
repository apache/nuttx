/****************************************************************************
 * arch/arm/src/sama5/sam_classd.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_CLASSD_H
#define __ARCH_ARM_SRC_SAMA5_SAM_CLASSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

#if defined(CONFIG_SAMA5D2_CLASSD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif /* __cplusplus */

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

struct audio_lowerhalf_s;
struct audio_lowerhalf_s *sama5_classd_initialize(void);
#undef EXTERN
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMA5D2_CLASSD */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_CLASSD_H */
