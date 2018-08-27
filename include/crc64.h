/****************************************************************************
 * include/crc64.h
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#ifndef __INCLUDE_CRC64_H
#define __INCLUDE_CRC64_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_HAVE_LONG_LONG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*
 * CRC64_CHECK is the CRC64 of the string "123456789" without the null byte.
 *
 *   const uint8_t checkbuf[] =
 *   {
 *     '1', '2', '3', '4', '5', '6', '7', '8', '9'
 *   };
 *
 *   assert(crc64(checkbuf, sizeof(checkbuf)) == CRC64_CHECK);
 */

/* CRC-64/WE */

#define CRC64_POLY   ((uint64_t)0x42f0e1eba9ea3693ull)
#define CRC64_INIT   ((uint64_t)0xffffffffffffffffull)
#define CRC64_XOROUT ((uint64_t)0xffffffffffffffffull)
#define CRC64_CHECK  ((uint64_t)0x62ec59e3f1a4f00aull)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: crc64part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 ****************************************************************************/

uint64_t crc64part(FAR const uint8_t *src, size_t len, uint64_t crc64val);

/****************************************************************************
 * Name: crc64
 *
 * Description:
 *   Return a 64-bit CRC of the contents of the 'src' buffer, length 'len'.
 *
 ****************************************************************************/

uint64_t crc64(FAR const uint8_t *src, size_t len);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_HAVE_LONG_LONG */
#endif /* __INCLUDE_CRC64_H */
