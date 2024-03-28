/****************************************************************************
 * arch/xtensa/src/common/espressif/utils/memory_reserve.h
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

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Region descriptor holds a description for a particular region of
 * memory reserved on this SoC for a particular use (ie not available
 * for stack/heap usage.)
 */

typedef struct
{
  intptr_t start;
  intptr_t end;
} soc_reserved_region_t;

/****************************************************************************
 * Helper Macros/Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: SOC_RESERVE_MEMORY_REGION
 *
 * Description:
 *   Macro to reserve a fixed region of RAM (hardcoded addresses) for a
 *   particular purpose. Usually used to mark out memory addresses needed
 *   for hardware or ROM code purposes. Not intended for user code which
 *   can use normal C static allocation instead.
 *
 * Input Parameters:
 *   START - Start address to be reserved.
 *   END   - One memory address after the address of the last byte to be
 *           reserved.
 *           (ie length of the reserved region is (END - START) in bytes.)
 *   NAME -  Name for the reserved region. Must be a valid variable name,
 *           unique to this source file.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SOC_RESERVE_MEMORY_REGION(START, END, NAME) \
  __attribute__((section(".reserved_memory_address"))) \
  __attribute__((used)) \
  static soc_reserved_region_t reserved_region_##NAME = { START, END };

#ifdef __cplusplus
}
#endif
