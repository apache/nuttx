//***************************************************************************
// libs/libxx/libxx.hxx
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

#ifndef __LIBXX_LIBXX_HXX
#define __LIBXX_LIBXX_HXX 1

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>

#include <nuttx/lib/lib.h>

//***************************************************************************
// Public Types
//***************************************************************************/

typedef CODE void (*__cxa_exitfunc_t)(void *arg);

//***************************************************************************
// Public Data
//***************************************************************************

extern "C" FAR void *__dso_handle weak_data;

//***************************************************************************
// Public Function Prototypes
//***************************************************************************

extern "C" int __cxa_atexit(__cxa_exitfunc_t func, void *arg, void *dso_handle);

#endif // __LIBXX_LIBXX_HXX
