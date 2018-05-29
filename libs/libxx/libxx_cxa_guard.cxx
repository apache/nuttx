//***************************************************************************
// libs/libxx/libxx_cxa_guard.cxx
//
//   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
//   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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
