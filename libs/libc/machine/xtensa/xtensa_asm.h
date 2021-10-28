/****************************************************************************
 * libs/libc/machine/xtensa/xtensa_asm.h
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

#include <arch/chip/core-isa.h>

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

  .macro  src_b r, w0, w1
#if XCHAL_HAVE_BE
  src \r, \w0, \w1
#else
  src \r, \w1, \w0
#endif
  .endm

  .macro  ssa8  r
#if XCHAL_HAVE_BE
  ssa8b \r
#else
  ssa8l \r
#endif
  .endm

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#if XCHAL_HAVE_BE
#  define MASK0 0xff000000
#  define MASK1 0x00ff0000
#  define MASK2 0x0000ff00
#  define MASK3 0x000000ff
#else
#  define MASK0 0x000000ff
#  define MASK1 0x0000ff00
#  define MASK2 0x00ff0000
#  define MASK3 0xff000000
#endif

