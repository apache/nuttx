/****************************************************************************
 * arch/arm/src/sama5/sam_dbgu.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_DBGU_H
#define __ARCH_ARM_SRC_SAMA5_SAM_DBGU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"

#ifdef CONFIG_SAMA5_DBGU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
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
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dbgu_initialize
 *
 * Description:
 *   Performs the low level DBGU initialization early in debug so that the
 *   DBGU console will be available during bootup.
 *
 ****************************************************************************/

void sam_dbgu_initialize(void);

/****************************************************************************
 * Name: sam_dbgu_register
 *
 * Description:
 *   Register DBBU console and serial port.  This assumes that
 *   sam_dbgu_initialize() was called previously.
 *
 ****************************************************************************/

void sam_dbgu_register(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMA5_DBGU */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_DBGU_H */
