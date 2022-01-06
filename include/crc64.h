/****************************************************************************
 * include/crc64.h
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

/* CRC64_CHECK is the CRC64 of the string "123456789" without the null byte.
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
