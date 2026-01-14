/****************************************************************************
 * arch/arm/src/samv7/sam_systemreset.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_SYSTEMRESET_H
#define __ARCH_ARM_SRC_SAMV7_SAM_SYSTEMRESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAMV7_RESET_PWRUP       1
#define SAMV7_RESET_BACKUP      2
#define SAMV7_RESET_WDOG        3
#define SAMV7_RESET_SWRST       4
#define SAMV7_RESET_NRST        5

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_get_reset_cause
 *
 * Description:
 *   Get cause of the last CPU reset. This is done by reading reset status
 *   registger.
 *
 * Returned Value:
 *   CPU reset cause in form of macros defined in sam_systemreset.h. This is
 *   to avoid passing boardctl dependent structure to architecture layer.
 *   Board level specific code should include sam_systemreset.h and set
 *   boardctl result according to that. -1 is returned in case of invalid
 *   value in status register.
 *
 ****************************************************************************/

int sam_get_reset_cause(void);

#endif /* __ARCH_ARM_SRC_SAMV7_SAM_SYSTEMRESET_H */
