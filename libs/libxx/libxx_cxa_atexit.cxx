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

#include <cassert>

#include "libxx.hxx"

//***************************************************************************
// Pre-processor Definitions
//***************************************************************************

//***************************************************************************
// Private Types
//***************************************************************************

struct __cxa_atexit_s
{
  __cxa_exitfunc_t func;
  FAR void *arg;
};

//***************************************************************************
// Private Data
//***************************************************************************

extern "C"
{
  //*************************************************************************
  // Public Data
  //*************************************************************************

  FAR void *__dso_handle = &__dso_handle;

  //*************************************************************************
  // Private Functions
  //*************************************************************************

  //*************************************************************************
  // Name: __cxa_callback
  //
  // Description:
  //   This is really just an "adaptor" function that matches the form of
  //   the __cxa_exitfunc_t to an onexitfunc_t using an allocated structure
  //   to marshall the call parameters.
  //
  //*************************************************************************

#ifdef CONFIG_SCHED_ONEXIT
  static void __cxa_callback(int exitcode, FAR void *arg)
  {
    FAR struct __cxa_atexit_s *alloc = (FAR struct __cxa_atexit_s *)arg;
    DEBUGASSERT(alloc && alloc->func);

    alloc->func(alloc->arg);
    lib_free(alloc);
  }
#endif

  //*************************************************************************
  // Public Functions
  //*************************************************************************

  //*************************************************************************
  // Name: __cxa_atexit
  //
  // Description:
  //   __cxa_atexit() registers a destructor function to be called by exit().
  //   On a call to exit(), the registered functions should be called with
  //   the single argument 'arg'. Destructor functions shall always be
  //   called in the reverse order to their registration (i.e. the most
  //   recently registered function shall be called first),
  //
  //   If shared libraries were supported, the callbacks should be invoked
  //   when the shared library is unloaded as well.
  //
  // Reference:
  //   Linux base
  //
  //*************************************************************************

  int __cxa_atexit(__cxa_exitfunc_t func, FAR void *arg, FAR void *dso_handle)
    {
#ifdef CONFIG_SCHED_ONEXIT
      // Allocate memory to hold the marshaled __cxa_exitfunc_t call
      // information.

      FAR struct __cxa_atexit_s *alloc =
        (FAR struct __cxa_atexit_s *)lib_malloc(sizeof(struct __cxa_atexit_s));

      if (alloc)
        {
          // Register the function to be called when the task/thread exists.

          alloc->func = func;
          alloc->arg  = arg;

          return on_exit(__cxa_callback, alloc);
        }
      else
#endif
        {
          // What else can we do?

          return 0;
        }
    }
}
