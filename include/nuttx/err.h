/****************************************************************************
 * include/nuttx/err.h
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

#ifndef __INCLUDE_NUTTX_ERR_H
#define __INCLUDE_NUTTX_ERR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Kernel pointers have redundant information, so we can use a
 * scheme where we can return either an error code or a normal
 * pointer with the same return value.
 *
 * This should be a per-architecture thing, to allow different
 * error and pointer decisions.
 ****************************************************************************/

#define MAX_ERRNO 4095

#define IS_ERR_VALUE(x) predict_false((unsigned long)(FAR void *)(x) >= \
                                      (unsigned long) - MAX_ERRNO)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline FAR void *ERR_PTR(long error)
{
  return (FAR void *)error;
}

static inline long PTR_ERR(FAR const void *ptr)
{
  return (long)ptr;
}

static inline bool IS_ERR(FAR const void *ptr)
{
  return IS_ERR_VALUE((unsigned long)ptr);
}

static inline bool IS_ERR_OR_NULL(FAR const void *ptr)
{
  return predict_false(!ptr) || IS_ERR_VALUE((unsigned long)ptr);
}

/****************************************************************************
 * Name: ERR_CAST
 *
 * Description:
 * ERR_CAST - Explicitly cast an error-valued pointer to another pointer type
 *
 * ptr: The pointer to cast.
 *
 * Explicitly cast an error-valued pointer to another pointer type in such a
 * way as to make it clear that's what's going on.
 ****************************************************************************/

static inline FAR void *ERR_CAST(FAR const void *ptr)
{
  /* cast away the const */

  return (FAR void *)ptr;
}

static inline int PTR_ERR_OR_ZERO(FAR const void *ptr)
{
  if (IS_ERR(ptr))
    return PTR_ERR(ptr);
  else
    return 0;
}

#endif /* __INCLUDE_NUTTX_ERR_H */
