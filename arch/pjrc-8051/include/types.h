/************************************************************
 * arch/types.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through sys/types.h
 */

#ifndef __ARCH_TYPES_H
#define __ARCH_TYPES_H

/************************************************************
 * Included Files
 ************************************************************/

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Type Declarations
 ************************************************************/

#ifndef __ASSEMBLY__

/* These are the sizes of the standard SDCC types
 *
 * For SDCC, sizeof(int) is 16 and sizeof(long) is 32.
 * long long and double are not supported.
 */

typedef char sbyte;
typedef unsigned char ubyte;
typedef unsigned char uint8;
typedef unsigned char boolean;
typedef int sint16;
typedef unsigned int uint16;
typedef long sint32;
typedef unsigned long uint32;

/* For SDCC, a Generic pointer is 3 bytes in length with the
 * first byte holding data space information.
 */

typedef unsigned long uintptr;

/* This is the size of the interrupt state save returned by
 * irqsave()
 */

typedef unsigned char irqstate_t;

#endif /* __ASSEMBLY__ */

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#endif /* __ARCH_TYPES_H */
