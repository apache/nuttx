/****************************************************************************
 * libs/libc/ctype/lib_ctype.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <ctype.h>

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifndef CONFIG_LIBCXXTOOLCHAIN

/* MSVC seems to conflict with theses macro if defined in the public area.
 * As such, they are defined in the private section to let NuttX build
 */

#define _U  01
#define _L  02
#define _N  04
#define _S  010
#define _P  020
#define _C  040
#define _X  0100
#define _B  0200

#else

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GNU libstdc++ is expecting an array _ctype_ to be defined. This array
 * is usually defined in GNU libc or newlib.
 * Here the system support only basic ASCII locale.
 */

const char _ctype_[] =
{
  0,
  _C, _C, _C, _C, _C, _C, _C, _C, \
  _C, _C | _S, _C | _S, _C | _S, _C | _S, _C | _S, _C, _C, \
  _C, _C, _C, _C, _C, _C, _C, _C, \
  _C, _C, _C, _C, _C, _C, _C, _C, \
  _S | _B,  _P, _P, _P, _P, _P, _P, _P, \
  _P, _P, _P, _P, _P, _P, _P, _P, \
  _N, _N, _N, _N, _N, _N, _N, _N, \
  _N, _N, _P, _P, _P, _P, _P, _P, \
  _P, _U | _X, _U | _X, _U | _X, _U | _X, _U | _X, _U | _X, _U, \
  _U, _U, _U, _U, _U, _U, _U, _U, \
  _U, _U, _U, _U, _U, _U, _U, _U, \
  _U, _U, _U, _P, _P, _P, _P, _P, \
  _P, _L | _X, _L | _X, _L | _X, _L | _X, _L | _X, _L | _X, _L, \
  _L, _L, _L, _L, _L, _L, _L, _L, \
  _L, _L, _L, _L, _L, _L, _L, _L, \
  _L, _L, _L, _P, _P, _P, _P, _C, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0, \
  0,  0,  0,  0,  0,  0,  0,  0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#endif

