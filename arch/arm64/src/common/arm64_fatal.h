/****************************************************************************
 * arch/arm64/src/common/arm64_fatal.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_FATAL_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_FATAL_H

/**
 * @defgroup fatal_apis Fatal error APIs
 * @ingroup kernel_apis
 * @{
 */

#define K_ERR_CPU_EXCEPTION     (0)
#define K_ERR_CPU_MODE32        (1)
#define K_ERR_SPURIOUS_IRQ      (2)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __builtin_unreachable()    \
  do                               \
    {                              \
      sinfo("Unreachable code\n"); \
      PANIC();                     \
    } while (true)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_fatal_error
 *
 * Description:
 *       fatal error handle for arm64
 * Input Parameters:
 *   reason: error reason
 *   reg:    exception stack reg context
 *
 * Returned Value:
 *
 ****************************************************************************/

void arm64_fatal_error(unsigned int reason, struct regs_context * reg);
void arm64_dump_fatal(struct regs_context * reg);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_FATAL_H */
