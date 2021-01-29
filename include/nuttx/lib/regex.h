/****************************************************************************
 * include/nuttx/lib/regex.h
 * Non-standard, pattern-matching APIs available in lib/.
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

#ifndef __INCLUDE_NUTTX_LIB_REGEX_H
#define __INCLUDE_NUTTX_LIB_REGEX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: match
 *
 * Description:
 *   Simple shell-style filename pattern matcher written by Jef Poskanzer
 *   (See copyright notice in lib/lib_match.c).  This pattern matcher only
 *   handles '?', '*' and '**', and  multiple patterns separated by '|'.
 *
 * Returned Value:
 *   Returns 1 (match) or 0 (no-match).
 *
 ****************************************************************************/

int match(FAR const char *pattern, FAR const char *string);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_REGEX_H */
