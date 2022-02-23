//***************************************************************************
// libs/libxx/libcxxmini/libxx_cxa_guard.cxx
//
// Licensed to the Apache Software Foundation (ASF) under one or more
// contributor license agreements.  See the NOTICE file distributed with
// this work for additional information regarding copyright ownership.  The
// ASF licenses this file to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance with the
// License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
// License for the specific language governing permissions and limitations
// under the License.
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/compiler.h>

//***************************************************************************
// Pre-processor Definitions
//***************************************************************************

//***************************************************************************
// Private Types
//***************************************************************************

#ifdef __ARM_EABI__
// The 32-bit ARM C++ ABI specifies that the guard is a 32-bit
// variable and the least significant bit contains 0 prior to
// initialization, and 1 after.

typedef int __guard;

#else
// The "standard" C++ ABI specifies that the guard is a 64-bit
// variable and the first byte contains 0 prior to initialization, and
// 1 after.

__extension__ typedef int __guard __attribute__((mode(__DI__)));
#endif

//***************************************************************************
// Private Data
//***************************************************************************

//***************************************************************************
// Public Functions
//***************************************************************************

extern "C"
{
  //*************************************************************************
  // Name: __cxa_guard_acquire
  //*************************************************************************

  int __cxa_guard_acquire(FAR __guard *g)
  {
#ifdef __ARM_EABI__
    return !(*g & 1);
#else
    return !*(char *)g;
#endif
  }

  //*************************************************************************
  // Name: __cxa_guard_release
  //*************************************************************************

  void __cxa_guard_release(FAR __guard *g)
  {
#ifdef __ARM_EABI__
    *g = 1;
#else
    *(char *)g = 1;
#endif
  }

  //*************************************************************************
  // Name: __cxa_guard_abort
  //*************************************************************************

  void __cxa_guard_abort(FAR __guard *)
  {
  }
}
