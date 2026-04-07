/****************************************************************************
 * include/nuttx/lib/xorsum.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_LIB_XORSUM_H
#define __INCLUDE_NUTTX_LIB_XORSUM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: xorsum16
 *
 * Description:
 *   Return the XOR of the 16-bit little-endian words of a buffer, taken by
 *   their offset from the start; an odd last byte is the low byte of a
 *   final word. The result does not depend on the buffer's alignment.
 *   This is a fast check for framing errors, not a CRC: it misses errors
 *   that flip the same bit in two words and words that are swapped.
 *
 * Input Parameters:
 *   buf - The buffer
 *   len - The buffer length in bytes
 *
 * Returned Value:
 *   The 16-bit XOR checksum.
 *
 ****************************************************************************/

uint16_t xorsum16(FAR const void *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_XORSUM_H */
