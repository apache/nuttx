//***************************************************************************
// lib/libxx__gnu_unwind_find_exidx.hxx
//
//   Copyright (C) 2015 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __LIBXX_LIBXX__GNU_UNWIND_FIND_EXIDX_HXX
#define __LIBXX_LIBXX__GNU_UNWIND_FIND_EXIDX_HXX

extern "C"
{
//***************************************************************************
// Included Files
//***************************************************************************

#include <elf32.h>
#include <unwind.h>

//***************************************************************************
// Pre-processor Definitions
//***************************************************************************

//***************************************************************************
// Public Types
//***************************************************************************/

typedef struct __EIT_entry
{
  _uw fnoffset;
  _uw content;
} __EIT_entry;

//***************************************************************************
// Public Variables
//***************************************************************************

extern __EIT_entry __exidx_start;
extern __EIT_entry __exidx_end;

__EIT_entry *__exidx_start_elf;
__EIT_entry *__exidx_end_elf;

//***************************************************************************
// Public Function Prototypes
//***************************************************************************

 _Unwind_Ptr __gnu_Unwind_Find_exidx(_Unwind_Ptr return_address, int *nrecp);

} // extern "C"

#endif // __LIBXX__GNU_UNWIND_FIND_EXIDX_HXX
