/****************************************************************************
 * include/nuttx/shm.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SHM_H
#define __INCLUDE_NUTTX_SHM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_ADDRENV
#  error CONFIG_ARCH_ADDRENV must be selected with CONFIG_MM_SHM
#endif

#ifndef CONFIG_BUILD_KERNEL
#  error CONFIG_BUILD_KERNEL must be selected with CONFIG_MM_SHM
#endif

#ifndef CONFIG_GRAN
#  error CONFIG_GRAN must be selected with CONFIG_MM_SHM
#endif

#ifdef CONFIG_GRAN_SINGLE
#  error CONFIG_GRAN_SINGLE must NOT be selected with CONFIG_MM_SHM
#endif

#ifndef CONFIG_MM_PGALLOC
#  error CONFIG_MM_PGALLOC must be selected with CONFIG_MM_SHM
#endif

/* Debug */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG_SHM
#    define shmdbg(format, ...)       dbg(format, ##__VA_ARGS__)
#    define shmvdbg(format, ...)      vdbg(format, ##__VA_ARGS__)
#  else
#    define shmdbg(format, ...)       mdbg(format, ##__VA_ARGS__)
#    define shmvdbg(format, ...)      mvdbg(format, ##__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG_SHM
#    define shmdbg                    dbg
#    define shmvdbg                   vdbg
#  else
#    define shmdbg                    (void)
#    define shmvdbg                   (void)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: shm_initialize
 *
 * Description:
 *   Perform one time, start-up initialization of the shared memor logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void shm_initialize(void);

#endif /* CONFIG_MM_SHM */
#endif /* __INCLUDE_NUTTX_SHM_H */
