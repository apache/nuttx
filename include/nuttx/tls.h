/****************************************************************************
 * include/nuttx/tls.h
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

#ifndef __INCLUDE_NUTTX_TLS_H
#define __INCLUDE_NUTTX_TLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_TLS_ALIGNED
#  ifndef CONFIG_TLS_LOG2_MAXSTACK
#    error CONFIG_TLS_LOG2_MAXSTACK is not defined
#  endif
#endif

#ifndef CONFIG_TLS_NELEM
#  warning CONFIG_TLS_NELEM is not defined
#  define CONFIG_TLS_NELEM 0
#endif

/* TLS Definitions **********************************************************/

#ifdef CONFIG_TLS_ALIGNED
#  define TLS_STACK_ALIGN  (1L << CONFIG_TLS_LOG2_MAXSTACK)
#  define TLS_STACK_MASK   (TLS_STACK_ALIGN - 1)
#  define TLS_MAXSTACK     (TLS_STACK_ALIGN)
#  define TLS_INFO(sp)     ((FAR struct tls_info_s *)((sp) & ~TLS_STACK_MASK))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* When TLS is enabled, up_createstack() will align allocated stacks to the
 * TLS_STACK_ALIGN value.  An instance of the following structure will be
 * implicitly positioned at the "lower" end of the stack.  Assuming a
 * "push down" stack, this is at the "far" end of the stack (and can be
 * clobbered if the stack overflows).
 *
 * If an MCU has a "push up" then that TLS structure will lie at the top
 * of the stack and stack allocation and initialization logic must take
 * care to preserve this structure content.
 *
 * The stack memory is fully accessible to user mode threads.  TLS is not
 * available from interrupt handlers (nor from the IDLE thread).
 */

struct tls_info_s
{
#if CONFIG_TLS_NELEM > 0
  uintptr_t tl_elem[CONFIG_TLS_NELEM]; /* TLS elements */
#endif
  int tl_errno;                        /* Per-thread error number */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tls_alloc
 *
 * Description:
 *   Allocate a group-unique TLS data index
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A TLS index that is unique for use within this task group.
 *
 ****************************************************************************/

#if CONFIG_TLS_NELEM > 0
int tls_alloc(void);
#endif

/****************************************************************************
 * Name: tls_free
 *
 * Description:
 *   Release a group-unique TLS data index previous obtained by tls_alloc()
 *
 * Input Parameters:
 *   tlsindex - The previously allocated TLS index to be freed
 *
 * Returned Value:
 *   OK is returned on success; a negated errno value will be returned on
 *   failure:
 *
 *     -EINVAL - the index to be freed is out of range.
 *
 ****************************************************************************/

#if CONFIG_TLS_NELEM > 0
int tls_free(int tlsindex);
#endif

/****************************************************************************
 * Name: tls_get_value
 *
 * Description:
 *   Return an the TLS data value associated with the 'tlsindx'
 *
 * Input Parameters:
 *   tlsindex - Index of TLS data element to return
 *
 * Returned Value:
 *   The value of TLS element associated with 'tlsindex'. Errors are not
 *   reported.  Zero is returned in the event of an error, but zero may also
 *   be valid value and returned when there is no error.  The only possible
 *   error would be if tlsindex < 0 or tlsindex >=CONFIG_TLS_NELEM.
 *
 ****************************************************************************/

#if CONFIG_TLS_NELEM > 0
uintptr_t tls_get_value(int tlsindex);
#endif

/****************************************************************************
 * Name: tls_set_value
 *
 * Description:
 *   Set the TLS element associated with the 'tlsindex' to 'tlsvalue'
 *
 * Input Parameters:
 *   tlsindex - Index of TLS data element to set
 *   tlsvalue - The new value of the TLS data element
 *
 * Returned Value:
 *   Zero is returned on success, a negated errno value is return on
 *   failure:
 *
 *     EINVAL - tlsindex is not in range.
 *
 ****************************************************************************/

#if CONFIG_TLS_NELEM > 0
int tls_set_value(int tlsindex, uintptr_t tlsvalue);
#endif

/****************************************************************************
 * Name: tls_get_info
 *
 * Description:
 *   Return a reference to the tls_info_s structure.  This is used as part
 *   of the internal implementation of tls_get/set_elem() and ONLY for the
 *   where CONFIG_TLS_ALIGNED is *not* defined
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the thread-specific tls_info_s structure is return on
 *   success.  NULL would be returned in the event of any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_TLS_ALIGNED
FAR struct tls_info_s *tls_get_info(void);
#endif

#endif /* __INCLUDE_NUTTX_TLS_H */
