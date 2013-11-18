/****************************************************************************
 * include/nuttx/mtd/hamming.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was taken directly from Atmel sample code with only
 * modifications for better integration with NuttX.  The Atmel sample
 * code has a BSD compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2011, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#ifndef __INCLUDE_NUTTX_HAMMING_H
#define __INCLUDE_NUTTX_HAMMING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand_raw.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* These are the possible errors when trying to verify a block of data
 * encoded using a Hamming code:
 *
 *   HAMMING_SUCCESS            - Block verified without errors
 *   HAMMING_ERROR_SINGLEBIT    - A single bit was incorrect but has been
 *                                recovered
 *   HAMMING_ERROR_ECC          - The original code has been corrupted
 *   HAMMING_ERROR_MULTIPLEBITS - Multiple bits are incorrect in the data
 *                                and they cannot be corrected
 */

#define HAMMING_SUCCESS                 0
#define HAMMING_ERROR_SINGLEBIT         1
#define HAMMING_ERROR_ECC               2
#define HAMMING_ERROR_MULTIPLEBITS      3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_HAMMING_H */
