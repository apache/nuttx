/****************************************************************************
 * include/stdnoreturn.h
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

#ifndef __INCLUDE_STDNORETURN_H
#define __INCLUDE_STDNORETURN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __GNUC__

/* C11 adds _Noreturn keyword that the function will not return. */

#  define noreturn _Noreturn

#elif defined(SDCC) || defined(__SDCC)

/* Current SDCC supports noreturn via C11 _Noreturn keyword. */

#  define noreturn _Noreturn

#else

#  define noreturn

#endif

#endif /* __INCLUDE_STDNORETURN_H */
