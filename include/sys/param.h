/****************************************************************************
 * include/sys/param.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_SYS_PARAM_H
#define __INCLUDE_SYS_PARAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAXHOSTNAMELEN HOST_NAME_MAX

/* Macros for min/max. */

#ifndef MIN
#  define MIN(a,b)      (((a) < (b)) ? (a) : (b))
#endif  /* MIN */

#ifndef MAX
#  define MAX(a,b)      (((a) > (b)) ? (a) : (b))
#endif  /* MAX */

#ifndef CLAMP
/* inclusively clamping a value into a given range */
#  define CLAMP(x, min, max) ((x) <= (min) ? (min) : MIN(x, max))
#endif /* CLAMP */

/* Macros for number of items.
 * (aka. ARRAY_SIZE, ArraySize, Size of an Array)
 */

#ifndef nitems
#  define nitems(_a)    (sizeof(_a) / sizeof(0[(_a)]))
#endif /* nitems */

/* Macros for counting and rounding.
 *
 * These macros accept value to be rounded or checked as 'a' and base as 'b'.
 * They are consistent with those provided by some BSDs.
 *
 * howmany() calculates how many b fit into a.
 *
 * rounddown() and roundup() round a to the next b.
 *
 * rounddown2() and roundup2() are for powers of 2 only.
 *
 * powerof2() returns 1 if a is a power of 2, 0 otherwise.
 *
 * Warning: rounddown2() and roundup2() can be used only if b is power of 2,
 * otherwise they will produce wrong result.
 *
 * In all of these macros, outcome is undefined if b is 0.
 *
 * All parameters are assumed to be unsigned integers.
 */

#ifndef howmany
#  define howmany(a, b)	(((a) + ((b) - 1)) / (b))
#endif /* howmany */

#ifndef rounddown
#  define	rounddown(a, b)	(((a) / (b)) * (b))
#endif /* rounddown */

#ifndef rounddown2
#  define	rounddown2(a, b) ((a) & ~((b) - 1))
#endif /* rounddown2 */

#ifndef roundup
#  define	roundup(a, b)	((((a) + ((b) - 1)) / (b)) * (b))
#endif /* roundup */

#ifndef roundup2
#  define	roundup2(a, b)	(((a) + ((b) - 1)) & ~((b) - 1))
#endif /* roundup2 */

#ifndef powerof2
#  define powerof2(a)	(!(((a) - 1) & (a)))
#endif /* powerof2 */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_SYS_PARAM_H */
