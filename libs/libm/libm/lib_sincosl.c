/****************************************************************************
 * libs/libm/libm/lib_sincosl.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>

/* Disable sincos optimization for all functions in this file,
 * otherwise gcc would generate infinite calls.
 * Refer to gcc PR46926.
 * -fno-builtin-sin or -fno-builtin-cos can disable sincos optimization,
 * but these two options do not work inside optimize pragma in-file.
 * Thus we just enforce -O0 when compiling this file.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE

nooptimiziation_function
void sincosl(long double x, FAR long double *s, FAR long double *c)
{
  *s = sinl(x);
  *c = cosl(x);
}

#endif
