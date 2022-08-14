/****************************************************************************
 * include/nuttx/crc8.h
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

#ifndef __INCLUDE_NUTTX_CRC8_H
#define __INCLUDE_NUTTX_CRC8_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Append full suffix to avoid the penitential symbol collision */

#define crc8   crc8full

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
 * Name: crc8part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer using the polynomial
 *   x^8+x^6+x^3+x^2+1 (Koopman, et al. "0xA6" poly).
 *
 ****************************************************************************/

uint8_t crc8part(FAR const uint8_t *src, size_t len, uint8_t crc8val);

/****************************************************************************
 * Name: crc8
 *
 * Description:
 *   Return an 8-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using the polynomial x^8+x^6+x^3+x^2+1 (Koopman, et al. "0xA6" poly).
 *
 ****************************************************************************/

uint8_t crc8(FAR const uint8_t *src, size_t len);

/****************************************************************************
 * Name: crc8table
 *
 * Description:
 *   Return a 8-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using a CRC with crc table used for calculation.
 *
 ****************************************************************************/

uint8_t crc8table(const uint8_t table[256], const uint8_t *src,
                  size_t len, uint8_t crc8val);

/****************************************************************************
 * Name: crc8ccitt
 *
 * Description:
 *   Return an 8-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using the polynomial x^8+x^2+x^1+1 (aka "0x07" poly).
 *
 ****************************************************************************/

uint8_t crc8ccitt(FAR const uint8_t *src, size_t len);

/****************************************************************************
 * Name: crc8part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer using the polynomial
 *   x^8+x^2+x^1+1 (aka "0x07" poly).
 *
 ****************************************************************************/

uint8_t crc8ccittpart(FAR const uint8_t *src, size_t len, uint8_t crc8val);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CRC8_H */
