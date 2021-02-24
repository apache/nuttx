//***************************************************************************
// libs/libxx/libxx_delete_sized.cxx
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

#include <nuttx/compiler.h>

#include <cstddef>

#include "libxx.hxx"

#ifdef CONFIG_HAVE_CXX14

//***************************************************************************
// Operators
//***************************************************************************

//***************************************************************************
// Name: delete
//
// NOTE:
//   This should take a type of size_t.  But size_t has an unknown underlying
//   type.  In the nuttx sys/types.h header file, size_t is typed as uint32_t
//   (which is determined by architecture-specific logic).  But the C++
//   compiler may believe that size_t is of a different type resulting in
//   compilation errors in the operator.  Using the underlying integer type
//   instead of size_t seems to resolve the compilation issues. Need to
//   REVISIT this.
//
//***************************************************************************

void operator delete(FAR void *ptr, std::size_t size)
{
  lib_free(ptr);
}

#endif /* CONFIG_HAVE_CXX14 */
