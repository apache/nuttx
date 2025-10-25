/****************************************************************************
 * arch/arm/src/samv7/sam_chipid.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_CHIPID_H
#define __ARCH_ARM_SRC_SAMV7_SAM_CHIPID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

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
 * Name: sam_has_revb_periphs
 *
 * Description:
 *   Returns true if revA peripherals are used, false otherwise.
 *.  Note: PIC32CZ CA70 are always revB.
 *
 ****************************************************************************/

bool sam_has_revb_periphs(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_SAMV7_SAM_CHIPID_H */
