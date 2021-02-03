/****************************************************************************
 * include/alloca.h
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

#ifndef __INCLUDE_ALLOCA_H
#define __INCLUDE_ALLOCA_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(__GNUC__) && __GNUC__ >= 3
/* Use GCC >=3 Built-in for alloca */

#  undef  alloca
#  undef  __alloca
#  define alloca(size)   __alloca(size)
#  define __alloca(size) __builtin_alloca(size)
#endif

#endif /* __INCLUDE_ALLOCA_H */
