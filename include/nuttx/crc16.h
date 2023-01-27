/****************************************************************************
 * include/nuttx/crc16.h
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

#ifndef __INCLUDE_NUTTX_CRC16_H
#define __INCLUDE_NUTTX_CRC16_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Append full suffix to avoid the penitential symbol collision */

#define crc16   crc16full

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
 * Name: crc16part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 ****************************************************************************/

uint16_t crc16part(FAR const uint8_t *src, size_t len, uint16_t crc16val);

/****************************************************************************
 * Name: crc16
 *
 * Description:
 *   Return a 16-bit CRC of the contents of the 'src' buffer, length 'len'
 *
 ****************************************************************************/

uint16_t crc16(FAR const uint8_t *src, size_t len);

/****************************************************************************
 * Name: crc16ccittpart
 *
 * Description:
 *   Continue 16-bit CRC-CCITT calculation on a part of the buffer using the
 *   polynomial x^16+x^12+x^5+1.
 *
 ****************************************************************************/

uint16_t crc16ccittpart(FAR const uint8_t *src, size_t len,
                        uint16_t crc16val);

/****************************************************************************
 * Name: crc16ccitt
 *
 * Description:
 *   Return a 16-bit CRC-CCITT of the contents of the 'src' buffer, length
 *   'len' using the polynomial x^16+x^12+x^5+1.
 *
 ****************************************************************************/

uint16_t crc16ccitt(FAR const uint8_t *src, size_t len);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CRC16_H */
