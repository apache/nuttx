/****************************************************************************
 * include/nuttx/lib/libvars.h
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

#ifndef __INCLUDE_NUTTX_LIB_LIBVARS_H
#define __INCLUDE_NUTTX_LIB_LIBVARS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/lib/getopt.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
/* This structure encapsulates all task-specific variables needed by the C
 * Library.  This structure is retained at the beginning of the main thread
 * of and is accessed via a reference stored in the TLS of all threads in
 * the task group.
 *
 * NOTE: task-specific variables are not needed in the KERNEL build.  In
 * that build mode, all global variables are inherently process-specific.
 */

struct libvars_s
{
  struct getopt_s lv_getopt; /* Globals used by getopt() */
};
#endif

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_LIBVARS_H */
