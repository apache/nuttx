//***************************************************************************
// libs/libxx/libxx_eabi_atexit.cxx
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
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>
#include <cstdlib>

#include "libxx.hxx"

//***************************************************************************
// Pre-processor Definitions
//***************************************************************************

//***************************************************************************
// Private Data
//***************************************************************************

//***************************************************************************
// Public Functions
//***************************************************************************

extern "C"
{
  //*************************************************************************
  // Name: __aeabi_atexit
  //
  // Description:
  //   Registers static object destructors.  Normally atexit(f) should call
  //   __aeabi_atexit (NULL, f, NULL).  But in the usage model here, static
  //   constructors are initialized at power up and are never destroyed
  //   because they have global scope and must persist for as long as the
  //   embedded device is powered on.
  //
  // Reference:
  //   http://infocenter.arm.com/help/topic/com.arm.doc.ihi0041c/IHI0041C_cppabi.pdf
  //
  //*************************************************************************

  int __aeabi_atexit(FAR void *object, __cxa_exitfunc_t func, FAR void *dso_handle)
    {
      return __cxa_atexit(func, object, dso_handle); // 0 ? OK; non-0 ? failed
    }
}
