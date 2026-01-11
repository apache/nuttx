/****************************************************************************
 * libs/libc/misc/lib_cxx_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <sched.h>
#include <stdlib.h>

#include "libc.h"

/****************************************************************************
 * External References
 ****************************************************************************/

#if defined(CONFIG_ARCH_SIM) && defined(CONFIG_HOST_MACOS)
extern void macho_call_saved_init_funcs(void);
#endif

/* ESP32 platforms have linker scripts that place non-constructor data
 * (soc_reserved_memory_region, esp_system_init_fn_array) between _sinit
 * and _einit. This requires explicit handling to avoid calling invalid
 * function pointers.
 *
 * ESP32 RISC-V chips: Have __init_priority_array and __init_array
 * ESP32 Xtensa chips: Have only __init_array (using .ctors)
 */

#if defined(CONFIG_ARCH_CHIP_ESP32) || defined(CONFIG_ARCH_CHIP_ESP32S2) ||    \
    defined(CONFIG_ARCH_CHIP_ESP32S3)

/* Xtensa ESP32 chips: Only __init_array exists */

extern initializer_t __init_array_start[];
extern initializer_t __init_array_end[];
#define USE_ESP32_XTENSA_INIT_ARRAYS 1
#elif defined(CONFIG_ARCH_CHIP_ESP32C3) ||                                     \
    defined(CONFIG_ARCH_CHIP_ESP32C6) || defined(CONFIG_ARCH_CHIP_ESP32H2)

/* RISC-V ESP32 chips: Both __init_priority_array and __init_array exist */

extern initializer_t __init_priority_array_start[];
extern initializer_t __init_priority_array_end[];
extern initializer_t __init_array_start[];
extern initializer_t __init_array_end[];
#define USE_ESP32_RISCV_INIT_ARRAYS 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_cxx_initialize
 *
 * Description:
 *   If C++ and C++ static constructors are supported, then this function
 *   must be provided by board-specific logic in order to perform
 *   initialization of the static C++ class instances.
 *
 *   This function should then be called in the application-specific
 *   user_start logic in order to perform the C++ initialization.  NOTE
 *   that no component of the core NuttX RTOS logic is involved; this
 *   function definition only provides the 'contract' between application
 *   specific C++ code and platform-specific toolchain support.
 *
 ****************************************************************************/

void lib_cxx_initialize(void)
{
#ifdef CONFIG_HAVE_CXXINITIALIZE
  static int inited = 0;

  if (inited == 0)
    {
#if defined(CONFIG_ARCH_SIM) && defined(CONFIG_HOST_MACOS)
      macho_call_saved_init_funcs();
#elif defined(USE_ESP32_XTENSA_INIT_ARRAYS)
      initializer_t *initp;

      /* ESP32 Xtensa platforms: Only iterate __init_array, skipping
       * non-constructor data after it.
       */

      sinfo("__init_array: %p to %p\n", __init_array_start,
            __init_array_end);

      for (initp = __init_array_start; initp < __init_array_end; initp++)
        {
          initializer_t initializer = *initp;
          sinfo("initp: %p initializer: %p\n", initp, initializer);

          if (initializer)
            {
              sinfo("Calling %p\n", initializer);
              initializer();
            }
        }
#elif defined(USE_ESP32_RISCV_INIT_ARRAYS)
      initializer_t *initp;

      /* ESP32 RISC-V platforms: Iterate through priority array first,
       * then regular array, skipping the non-constructor data sections
       * in between.
       */

      sinfo("__init_priority_array: %p to %p\n",
            __init_priority_array_start, __init_priority_array_end);

      for (initp = __init_priority_array_start;
           initp < __init_priority_array_end; initp++)
        {
          initializer_t initializer = *initp;
          sinfo("priority initp: %p initializer: %p\n", initp, initializer);

          if (initializer)
            {
              sinfo("Calling %p\n", initializer);
              initializer();
            }
        }

      sinfo("__init_array: %p to %p\n", __init_array_start,
            __init_array_end);

      for (initp = __init_array_start; initp < __init_array_end; initp++)
        {
          initializer_t initializer = *initp;
          sinfo("initp: %p initializer: %p\n", initp, initializer);

          if (initializer)
            {
              sinfo("Calling %p\n", initializer);
              initializer();
            }
        }
#else
      initializer_t *initp;

      sinfo("_sinit: %p _einit: %p\n", _sinit, _einit);

      /* Visit each entry in the initialization table */

      for (initp = _sinit; initp < _einit; initp++)
        {
          initializer_t initializer = *initp;
          sinfo("initp: %p initializer: %p\n", initp, initializer);

          /* Make sure that the address is non-NULL. Some toolchains
           * may put NULL values or counts in the initialization table.
           */

          if (initializer)
            {
              sinfo("Calling %p\n", initializer);
              initializer();
            }
        }
#endif

      inited = 1;
    }
#endif
}
