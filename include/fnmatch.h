/****************************************************************************
 * include/fnmatch.h
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

#ifndef __INCLUDE_FNMATCH_H
#define __INCLUDE_FNMATCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FNM_PATHNAME            0x01
#define FNM_PERIOD              0x02
#define FNM_NOESCAPE            0x04

#define FNM_NOMATCH             1
#define FNM_NOSYS               -1

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
 * Name: fnmatch
 *
 * Description:
 *   The fnmatch() function shall match patterns as described in the Shell
 *   and Utilities volume of IEEE Std 1003.1-2001, Section 2.13.1, Patterns
 *   Matching a Single Character, and Section 2.13.2, Patterns Matching
 *   Multiple Characters. It checks the string specified by the string
 *   argument to see if it matches the pattern specified by the pattern
 *   argument.
 *
 *   The flags argument shall modify the interpretation of pattern and
 *   string. It is the bitwise-inclusive OR of zero or more of the flags
 *   defined in <fnmatch.h>. If the FNM_PATHNAME flag is set in flags,
 *   then a slash character ( '/' ) in string shall be explicitly matched
 *   by a slash in pattern; it shall not be matched by either the asterisk
 *   or question-mark special characters, nor by a bracket expression. If
 *   the FNM_PATHNAME flag is not set, the slash character shall be treated
 *   as an ordinary character.
 *
 *   If FNM_NOESCAPE is not set in flags, a backslash character ( '\' ) in
 *   pattern followed by any other character shall match that second
 *   character in string. In particular, "\\" shall match a backslash in
 *   string. If FNM_NOESCAPE is set, a backslash character shall be treated
 *   as an ordinary character.
 *
 *   If FNM_PERIOD is set in flags, then a leading period ( '.' ) in string
 *   shall match a period in pattern; as described by rule 2 in the Shell and
 *   Utilities volume of IEEE Std 1003.1-2001, Section 2.13.3, Patterns Used
 *   for Filename Expansion where the location of "leading" is indicated by
 *   the value of FNM_PATHNAME:
 *
 *   If FNM_PATHNAME is set, a period is "leading" if it is the first
 *   character in string or if it immediately follows a slash.
 *
 *   If FNM_PATHNAME is not set, a period is "leading" only if it is the
 *   first character of string.
 *
 *   If FNM_PERIOD is not set, then no special restrictions are placed on
 *   matching a period.
 *
 * Returned Value:
 *   If string matches the pattern specified by pattern, then fnmatch()
 *   shall return 0. If there is no match, fnmatch() shall return
 *   FNM_NOMATCH, which is defined in <fnmatch.h>. If an error occurs,
 *   fnmatch() shall return another non-zero value.
 *
 ****************************************************************************/

int fnmatch(FAR const char *pattern, const char *string, int flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_FNMATCH_H */
