/****************************************************************************
 * arch/arm/src/lc823450/lc823450_sddrv_type.h
 *
 *   Copyright (C) 2014-2015 ON Semiconductor. All rights reserved.
 *   Copyright 2014,2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_TYPE_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_TYPE_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL        ((void * ) 0)
#endif

#define TRUE_T        (1)       /* true */
#define FALSE_T       (0)       /* false */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef SI_8
typedef signed char SI_8;
#endif
#ifndef UI_8
typedef unsigned char UI_8;
#endif

#ifndef SI_16
typedef signed short SI_16;
#endif
#ifndef UI_16
typedef unsigned short UI_16;
#endif

#ifndef SI_32
typedef signed long SI_32;
#endif
#ifndef UI_32
typedef unsigned long UI_32;
#endif

#ifndef SI_64
typedef signed long long SI_64;
#endif
#ifndef UI_64
typedef unsigned long long UI_64;
#endif

#ifndef SINT_T
typedef signed int SINT_T;
#endif
#ifndef UINT_T
typedef unsigned int UINT_T;
#endif

#ifndef CHAR_T
typedef char CHAR_T;
#endif

#ifndef BOOL_T
typedef int BOOL_T;
#endif

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_TYPE_H */
