/****************************************************************************
 * binfmt/binfmt.h
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

#ifndef __BINFMT_BINFMT_H
#define __BINFMT_BINFMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/binfmt/binfmt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* This is a list of registered handlers for different binary formats.
 * This list should only be accessed by normal user programs.  It should be
 * sufficient protection to simply disable pre-emption when accessing this
 * list.
 */

EXTERN FAR struct binfmt_s *g_binfmts;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_dumpmodule
 *
 * Description:
 *   Dump the contents of struct binary_s.
 *
 * Input Parameters:
 *   bin      - Load structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_BINFMT)
int binfmt_dumpmodule(FAR const struct binary_s *bin);
#else
#  define binfmt_dumpmodule(bin)
#endif

/****************************************************************************
 * Name: binfmt_copyargv
 *
 * Description:
 *   In the kernel build, the argv list will likely lie in the caller's
 *   address environment and, hence, be inaccessible when we switch to the
 *   address environment of the new process address environment.  So we
 *   do not have any real option other than to copy the callers argv[] list.
 *
 * Input Parameters:
 *   argv     - Argument list
 *
 * Returned Value:
 *   A non-zero copy is returned on success.
 *
 ****************************************************************************/

FAR char * const *binfmt_copyargv(FAR char * const *argv);

/****************************************************************************
 * Name: binfmt_freeargv
 *
 * Description:
 *   Release the copied argv[] list.
 *
 * Input Parameters:
 *   argv     - Argument list
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
void binfmt_freeargv(FAR char * const *argv);
#else
#  define binfmt_freeargv(argv)
#endif

/****************************************************************************
 * Name: binfmt_copyenv
 *
 * Description:
 *   In the kernel build, the environment exists in the parent's address
 *   environment and, hence, will be inaccessible when we switch to the
 *   address environment of the new process. So we do not have any real
 *   option other than to copy the parents envp list into an intermediate
 *   buffer that resides in neutral kernel memory.
 *
 * Input Parameters:
 *   envp     - Allocated environment strings
 *
 * Returned Value:
 *   A non-zero copy is returned on success.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
#  define binfmt_copyenv(envp) binfmt_copyargv(envp)
#else
#  define binfmt_copyenv(envp) (envp)
#endif

/****************************************************************************
 * Name: binfmt_freeenv
 *
 * Description:
 *   Release the copied envp[] list.
 *
 * Input Parameters:
 *   envp     - Allocated environment strings
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
#  define binfmt_freeenv(envp) binfmt_freeargv(envp)
#else
#  define binfmt_freeenv(envp)
#endif

/****************************************************************************
 * Name: builtin_initialize
 *
 * Description:
 *   In order to use the builtin binary format, this function must be called
 *   during system initialize to register the builtin binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILTIN
int builtin_initialize(void);
#endif

/****************************************************************************
 * Name: builtin_uninitialize
 *
 * Description:
 *   Unregister the builtin binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_BUILTIN
void builtin_uninitialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BINFMT_BINFMT_H */
