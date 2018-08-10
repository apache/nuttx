/****************************************************************************
 * include/nuttx/environ.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_ENVIRON_H
#define __INCLUDE_NUTTX_ENVIRON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef CONFIG_DISABLE_ENVIRON

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback function used with env_foreach() */

typedef int (*env_foreach_t)(FAR void *arg, FAR const char *pair);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: env_foreach
 *
 * Description:
 *   Search the provided environment structure for the variable of the
 *   specified name.
 *
 *   This is an internal OS function and should not be used by applications.
 *
 * Input Parameters:
 *   group - The task group containing environment array to be searched.
 *   cb    - The callback function to be invoked for each environment
 *           variable.
 *
 * Returned Value:
 *   Zero if the all environment variables have been traversed.  A non-zero
 *   value means that the callback function requested early termination by
 *   returning a nonzero value.
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emption is disabled by caller
 *
 ****************************************************************************/

struct task_group_s;  /* Forward reference */
int env_foreach(FAR struct task_group_s *group, env_foreach_t cb,
                FAR void *arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_ENVIRON */
#endif /* __INCLUDE_NUTTX_ENVIRON_H */
