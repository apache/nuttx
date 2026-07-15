/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/gdwifi_newlib_compat.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The prebuilt libwpa_supplicant.a was compiled against newlib and pulls
 * two of its ABI symbols: the global `errno` cell and the `_ctype_`
 * classification table.  Provide both here.  This file deliberately does
 * not include any NuttX header (errno is a macro there).
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

/* Private errno cell for the prebuilt library (not shared with NuttX's
 * per-thread errno; the supplicant only uses it around its own calls).
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* newlib ctype table: bit flags per character, index biased by one.
 * Layout and values match newlib's ctype_.c (_U upper, _L lower,
 * _N digit, _S space, _P punct, _C control, _X hex, _B blank).
 */

#define _U 0x01
#define _L 0x02
#define _N 0x04
#define _S 0x08
#define _P 0x10
#define _C 0x20
#define _X 0x40
#define _B 0x80

/****************************************************************************
 * Public Data
 ****************************************************************************/

int errno;

const char _ctype_[257] =
{
  0,
  _C,     _C,     _C,     _C,     _C,     _C,     _C,     _C,
  _C,     _C | _S,  _C | _S,  _C | _S,  _C | _S,  _C | _S,  _C,     _C,
  _C,     _C,     _C,     _C,     _C,     _C,     _C,     _C,
  _C,     _C,     _C,     _C,     _C,     _C,     _C,     _C,
  _S | _B,  _P,     _P,     _P,     _P,     _P,     _P,     _P,
  _P,     _P,     _P,     _P,     _P,     _P,     _P,     _P,
  _N,     _N,     _N,     _N,     _N,     _N,     _N,     _N,
  _N,     _N,     _P,     _P,     _P,     _P,     _P,     _P,
  _P,     _U | _X,  _U | _X,  _U | _X,  _U | _X,  _U | _X,  _U | _X,  _U,
  _U,     _U,     _U,     _U,     _U,     _U,     _U,     _U,
  _U,     _U,     _U,     _U,     _U,     _U,     _U,     _U,
  _U,     _U,     _U,     _P,     _P,     _P,     _P,     _P,
  _P,     _L | _X,  _L | _X,  _L | _X,  _L | _X,  _L | _X,  _L | _X,  _L,
  _L,     _L,     _L,     _L,     _L,     _L,     _L,     _L,
  _L,     _L,     _L,     _L,     _L,     _L,     _L,     _L,
  _L,     _L,     _L,     _P,     _P,     _P,     _P,     _C,

  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
