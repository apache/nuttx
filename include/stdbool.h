/****************************************************************************
 * include/stdbool.h
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

#ifndef __INCLUDE_STDBOOL_H
#define __INCLUDE_STDBOOL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* If CONFIG_ARCH_STDBOOL_H is set, then the architecture will provide its
 * own stdbool.h file.  In this case, this header file will simply re-direct
 * to the architecture-specfic stdbool.h header file.
 */

#ifdef CONFIG_ARCH_STDBOOL_H
#  include <arch/stdbool.h>

#else

/* NuttX will insist that the sizeof(bool) is 8-bits.  The sizeof of _Bool
 * used by any specific compiler is implementation specific: It can vary from
 * compiler-to-compiler and even vary between different versions of the same
 * compiler.  Compilers seems to be converging to sizeof(_Bool) == 1. If that
 * is true for your compiler, you should define CONFIG_C99_BOOL in your
 * NuttX configuration for better standards compatibility.
 *
 * CONFIG_C99_BOOL - Means (1) your C++ compiler has sizeof(_Bool) == 8,
 * (2) your C compiler supports the C99 _Bool intrinsic type, and (3) that
 * the C99 _Bool type also has size 1.
 */

/* nuttx/compiler.h define or undefine CONFIG_C99_BOOL */

#  include <nuttx/compiler.h>
#  include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* bool, true, and false must be provided as macros so that they can be
 * redefined by the application if necessary.
 *
 * NOTE: Under C99 'bool' is required to be defined to be the intrinsic type
 * _Bool.  However, in this NuttX context, we need backward compatibility
 * to pre-C99 standards where _Bool is not an intrinsic type.  Hence, we
 * use uint8_t as the underlying type (unless CONFIG_C99_BOOL is defined)
 */

#  ifndef __cplusplus
#    ifdef CONFIG_C99_BOOL
#      define bool _Bool
#    else
#      define bool uint8_t
#    endif

#    define true  (bool)1
#    define false (bool)0

#    define __bool_true_false_are_defined 1
#  endif /* __cplusplus */

#  endif /* CONFIG_ARCH_STDBOOL_H */
#endif /* __INCLUDE_STDBOOL_H */
