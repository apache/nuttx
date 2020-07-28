/****************************************************************************
 * libs/libc/assert/lib_ctype.c
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

#include <ctype.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

const IOBJ char _ctype_[1 + 256 + 1] =
{
  0,
  _C,      _C,      _C,      _C,      _C,      _C,      _C,      _C,
  _C,      _C | _S, _C | _S, _C | _S, _C | _S, _C | _S, _C,      _C,
  _C,      _C,      _C,      _C,      _C,      _C,      _C,      _C,
  _C,      _C,      _C,      _C,      _C,      _C,      _C,      _C,
  _S | _B, _P,      _P,      _P,      _P,      _P,      _P,      _P,
  _P,      _P,      _P,      _P,      _P,      _P,      _P,      _P,
  _N,      _N,      _N,      _N,      _N,      _N,      _N,      _N,
  _N,      _N,      _P,      _P,      _P,      _P,      _P,      _P,
  _P,      _U | _X, _U | _X, _U | _X, _U | _X, _U | _X, _U | _X, _U,
  _U,      _U,      _U,      _U,      _U,      _U,      _U,      _U,
  _U,      _U,      _U,      _U,      _U,      _U,      _U,      _U,
  _U,      _U,      _U,      _P,      _P,      _P,      _P,      _P,
  _P,      _L | _X, _L | _X, _L | _X, _L | _X, _L | _X, _L | _X, _L,
  _L,      _L,      _L,      _L,      _L,      _L,      _L,      _L,
  _L,      _L,      _L,      _L,      _L,      _L,      _L,      _L,
  _L,      _L,      _L,      _P,      _P,      _P,      _P,      _C,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0,       0,       0,       0,       0,       0,       0,       0,
  0
};

/* +1 because we want for mapping EOF to _ctype_[0] */

const IPTR char *const __ctype_ = _ctype_ + 1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
