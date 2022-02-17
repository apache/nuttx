/****************************************************************************
 * arch/z80/include/ez80/types.h
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

/* This file should never be included directly but, rather, only indirectly
 * through sys/types.h
 */

#ifndef __ARCH_Z80_INCLUDE_EZ80_TYPES_H
#define __ARCH_Z80_INCLUDE_EZ80_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* These are the sizes of the standard integer types.  NOTE that these type
 * names have a leading underscore character.  This file will be included
 * (indirectly) by include/stdint.h and typedef'ed to the final name without
 * the underscore character.  This roundabout way of doings things allows
 * the stdint.h to be removed from the include/ directory in the event that
 * the user prefers to use the definitions provided by their toolchain header
 * files
 *
 * These are the sizes of the types supported by the ZiLOG compiler:
 *
 *   int    - 24-bits
 *   short  - 16-bits
 *   long   - 32-bits
 *   char   - 8-bits
 *   float  - 32-bits
 *
 * Clang additionally supports:
 *
 *   long long - 64-bits
 */

typedef signed char        _int8_t;
typedef unsigned char      _uint8_t;

typedef signed short       _int16_t;
typedef unsigned short     _uint16_t;

typedef signed int         _int24_t;
typedef unsigned int       _uint24_t;
#define __INT24_DEFINED

typedef signed long        _int32_t;
typedef unsigned long      _uint32_t;

#ifdef __clang__
typedef signed long long   _int64_t;
typedef unsigned long long _uint64_t;
#define __INT64_DEFINED
typedef _int64_t           _intmax_t;
typedef _uint64_t          _uintmax_t;
#else
typedef _int32_t           _intmax_t;
typedef _uint32_t          _uintmax_t;
#endif

#if defined(__WCHAR_TYPE__)
typedef __WCHAR_TYPE__     _wchar_t;
#else
typedef int                _wchar_t;
#endif

/* A pointer is 2 or 3 bytes, depending upon if the ez80 is in z80
 * compatibility mode or not
 *
 *   Z80 mode - 16 bits
 *   ADL mode - 24 bits
 */

#if defined(__SIZE_TYPE__)
/* If __SIZE_TYPE__ is defined we define ssize_t based on size_t.
 * We simply change "unsigned" to "signed" for this single definition
 * to make sure ssize_t and size_t only differ by their signedness.
 */

#define unsigned signed
typedef __SIZE_TYPE__      _ssize_t;
#undef unsigned
typedef __SIZE_TYPE__      _size_t;
#elif defined(CONFIG_EZ80_Z80MODE)
typedef signed short       _ssize_t;
typedef unsigned short     _size_t;
#else
typedef signed int         _ssize_t;
typedef unsigned int       _size_t;
#endif

/* This is the size of the interrupt state save returned by up_irq_save().
 * It holds the AF register pair + a zero pad byte
 */

typedef _uint24_t          irqstate_t;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_Z80_INCLUDE_EZ80_TYPES_H */
