/****************************************************************************
 * include/nuttx/lib/builtin.h
 *
 * Originally by:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With subsequent updates, modifications, and general maintenance by:
 *
 *   Copyright (C) 2012-2013, 2019 Gregory Nutt.  All rights reserved.
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

#ifndef __INCLUDE_NUTTX_LIB_BUILTIN_H
#define __INCLUDE_NUTTX_LIB_BUILTIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#ifdef CONFIG_BUILTIN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This logic is not usable in the KERNEL build from within the kernel. */

#if !defined(CONFIG_BUILD_KERNEL) || !defined(__KERNEL__)
#  define HAVE_BUILTIN_CONTEXT
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct builtin_s
{
  const char *name;         /* Invocation name and as seen under /sbin/ */
  int         priority;     /* Use: SCHED_PRIORITY_DEFAULT */
  int         stacksize;    /* Desired stack size */
  main_t      main;         /* Entry point: main(int argc, char *argv[]) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_FS_BINFS) && \
    defined(__KERNEL__)
/* In the PROTECTED build, the builtin arrays are only needed by BINFS.
 * in this case, the user-space globals are not accessible and must be
 * provided to the OS via the boardctl(BOARDIOC_BUILTINS) call.  In this
 * case, the PROTECTED kernel will keep its own copy of the user-space
 * array.
 *
 * The call to boardctl(BOARDIOC_BUILTINS) must be provided by the
 * application layer.
 */

EXTERN FAR const struct builtin_s *g_builtins;
EXTERN int g_builtin_count;

#else
/* In the FLAT build, the builtin list is just a global global array and
 * count exported from user space via the backdoor left open by the FLAT
 * address space.  These globals must be provided the application layer.
 */

EXTERN FAR const struct builtin_s g_builtins[];
EXTERN const int g_builtin_count;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: builtin_setlist
 *
 * Description:
 *   Saves the user-space list of built-in applications for use by BINFS in
 *   protected mode.  Normally this is small set of globals provided by
 *   user-space logic.  It provides name-value pairs for associating
 *   built-in application names with user-space entry point addresses.
 *   These globals are only needed for use by BINFS which executes built-in
 *   applications from kernel-space in PROTECTED mode.  In the FLAT build,
 *   the user space globals are readily available.  (BINFS is not
 *   supportable in KERNEL mode since user-space address have no general
 *   meaning that configuration).
 *
 * Input Parameters:
 *   builtins - The list of built-in functions.  Each entry is a name-value
 *              pair that maps a built-in function name to its user-space
 *              entry point address.
 *   count    - The number of name-value pairs in the built-in list.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_FS_BINFS) && \
    defined(__KERNEL__)
void builtin_setlist(FAR const struct builtin_s *builtins, int count);
#endif

/****************************************************************************
 * Name: builtin_isavail
 *
 * Description:
 *   Checks for availability of an application named 'appname' registered
 *   during compile time and, if available, returns the index into the table
 *   of built-in applications.
 *
 * Input Parameters:
 *   filename - Name of the linked-in binary to be started.
 *
 * Returned Value:
 *   This is an internal function, used by by the NuttX binfmt logic and
 *   by the application built-in logic.  It returns a non-negative index to
 *   the application entry in the table of built-in applications on success
 *   or a negated errno value in the event of a failure.
 *
 ****************************************************************************/

int builtin_isavail(FAR const char *appname);

/****************************************************************************
 * Name: builtin_getname
 *
 * Description:
 *   Returns pointer to a name of built-in application pointed by the
 *   index.
 *
 * Input Parameters:
 *   index, from 0 and on ...
 *
 * Returned Value:
 *   Returns valid pointer pointing to the app name if index is valid.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR const char *builtin_getname(int index);

/****************************************************************************
 * Name: builtin_for_index
 *
 * Description:
 *   Returns the builtin_s structure for the selected built-in.
 *   If support for built-in functions is enabled in the NuttX
 *   configuration, then this function must be provided by the application
 *   code.
 *
 * Input Parameters:
 *   index, from 0 and on...
 *
 * Returned Value:
 *   Returns valid pointer pointing to the builtin_s structure if index is
 *   valid.
 *   Otherwise, NULL is returned.
 *
 ****************************************************************************/

FAR const struct builtin_s *builtin_for_index(int index);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_BUILTIN */
#endif /* __INCLUDE_NUTTX_LIB_BUILTIN_H */
