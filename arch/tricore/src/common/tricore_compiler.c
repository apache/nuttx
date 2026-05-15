/****************************************************************************
 * arch/tricore/src/common/tricore_compiler.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_regs_memcpy(void *dest, void *src, size_t count)
{
  memcpy(dest, src, count);
}

#ifdef CONFIG_TRICORE_TOOLCHAIN_LLVM

void __memcpy_assume_aligned(void *dst, const void *src, size_t n)
{
  if (n == 4)
    {
      *(uint32_t *)dst = *(uint32_t *)src;
      return;
    }

  memcpy(dst, src, n);
}

#endif /* CONFIG_TRICORE_TOOLCHAIN_LLVM */
