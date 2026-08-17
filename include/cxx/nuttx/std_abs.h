/****************************************************************************
 * include/cxx/nuttx/std_abs.h
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

#ifndef __INCLUDE_CXX_BITS_STD_ABS
#define __INCLUDE_CXX_BITS_STD_ABS

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#undef abs

/****************************************************************************
 * External functions
 ****************************************************************************/

extern "C"
{
  extern int abs(int);
  extern long labs(long);
  extern long long llabs(long long);

#ifdef CONFIG_HAVE_FLOAT
  extern float fabsf(float);
#endif
#ifdef CONFIG_HAVE_DOUBLE
  extern double fabs(double);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
  extern long double fabsl(long double);
#endif
}

/****************************************************************************
 * Namespace
 ****************************************************************************/

namespace std
{
    using ::abs;

    inline long abs(long x)
    {
      return ::labs(x);
    }

    inline long long abs(long long x)
    {
      return ::llabs(x);
    }

#ifdef CONFIG_HAVE_FLOAT
    inline float abs(float x)
    {
      return ::fabsf(x);
    }
#endif

#ifdef CONFIG_HAVE_DOUBLE
    inline double abs(double x)
    {
      return ::fabs(x);
    }
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
    inline long double abs(long double x)
    {
      return ::fabsl(x);
    }
#endif
}

#endif /* __INCLUDE_CXX_BITS_STD_ABS */
