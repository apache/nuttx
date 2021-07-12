//***************************************************************************
// libs/libxx/libxx_newa.cxx
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

#include <cstdlib>
#include <debug.h>

//***************************************************************************
// Pre-processor Definitions
//***************************************************************************

//***************************************************************************
// Private Data
//***************************************************************************

//***************************************************************************
// Public Functions
//***************************************************************************

namespace std
{
  void __throw_out_of_range(const char*)
  {
    _err("ERROR: C++: Vector .at() with argument out of range\n");
    abort();
   }

  void __throw_length_error(const char*)
  {
    _err("ERROR: C++: Vector resize to excessive length\n");
    abort();
  }

  void __throw_bad_alloc()
  {
    _err("ERROR: C++: Bad allocation\n");
    abort();
  }

  void __throw_bad_function_call()
  {
    _err("ERROR: C++: Bad function call\n");
    abort();
  }
}
