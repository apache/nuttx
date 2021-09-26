/****************************************************************************
 * include/glob.h
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

#ifndef __INCLUDE_GLOB_H
#define __INCLUDE_GLOB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following constants shall be provided as values
 * for the flags argument:
 */

#define GLOB_APPEND   0x01 /* Append generated pathnames to
                            * those previously obtained. */
#define GLOB_DOOFFS   0x02 /* Specify how many null pointers
                            * to add to the beginning of gl_pathv. */
#define GLOB_ERR      0x04 /* Cause glob() to return on error. */
#define GLOB_MARK     0x08 /* Each pathname that is a directory that
                            * matches pattern has a slash appended. */
#define GLOB_NOCHECK  0x10 /* If pattern does not match any pathname, then
                            * return a list consisting of only pattern. */
#define GLOB_NOESCAPE 0x20 /* Disable backslash escaping. */
#define GLOB_NOSORT   0x40 /* Do not sort the pathnames returned. */

/* The following constants shall be defined as error return values:
 */

#define GLOB_ABORTED  1 /* The scan was stopped because GLOB_ERR
                         * was set or (*errfunc)() returned non-zero. */
#define GLOB_NOMATCH  2 /* The pattern does not match any existing pathname,
                         * and GLOB_NOCHECK was not set in flags. */
#define GLOB_NOSPACE  3 /* An attempt to allocate memory failed. */
#define GLOB_NOSYS    4 /* Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  size_t gl_pathc;     /* Count of paths matched by pattern. */
  FAR char **gl_pathv; /* Pointer to a list of matched pathnames. */
  size_t gl_offs;      /* Slots to reserve at the beginning of gl_pathv. */
} glob_t;

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
 * Name: glob
 *
 * Description:
 *   find pathnames matching a pattern.
 *
 ****************************************************************************/

int glob(FAR const char *pat, int flags,
         CODE int (*errfunc)(FAR const char *path, int err),
         FAR glob_t *g);

/****************************************************************************
 * Name: globfree
 *
 * Description:
 *   Free memory from glob().
 *
 ****************************************************************************/

void globfree(FAR glob_t *g);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_GLOB_H */
