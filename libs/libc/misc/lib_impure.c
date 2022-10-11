/****************************************************************************
 * libs/libc/misc/lib_impure.c
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

#if defined(__has_include)
#  if __has_include(<reent.h>)

#define const
#include <reent.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ATTRIBUTE_IMPURE_PTR__
#  define __ATTRIBUTE_IMPURE_PTR__
#endif

#ifndef __ATTRIBUTE_IMPURE_DATA__
#  define __ATTRIBUTE_IMPURE_DATA__
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(_REENT_SMALL) && \
   (defined(__NEWLIB__) || __NEWLIB__ < 4 || \
    __NEWLIB__ == 4 && __NEWLIB_MINOR__ < 2)
extern struct __sFILE_fake __sf_fake_stdin _ATTRIBUTE((weak));
extern struct __sFILE_fake __sf_fake_stdout _ATTRIBUTE((weak));
extern struct __sFILE_fake __sf_fake_stderr _ATTRIBUTE((weak));
#else
extern __FILE __sf[3] _ATTRIBUTE((weak));
#endif

static struct _reent __ATTRIBUTE_IMPURE_DATA__
impure_data = _REENT_INIT(impure_data);

#ifdef __CYGWIN__
extern struct _reent reent_data __attribute__((alias("impure_data")));
#endif

struct _reent *__ATTRIBUTE_IMPURE_PTR__
_impure_ptr = &impure_data;

struct _reent *__ATTRIBUTE_IMPURE_PTR__
_global_impure_ptr = &impure_data;

#  endif /* __has_include(<reent.h>) */
#endif /* defined(__has_include) */
