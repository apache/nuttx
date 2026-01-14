/****************************************************************************
 * libs/libc/misc/lib_crc16.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/crc16.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc16part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 *   The default polynomial is 0x1021 (x^16 + x^12 + x^5 + 1)
 *   See crc16xmodempart()
 *
 ****************************************************************************/

uint16_t crc16part(FAR const uint8_t *src, size_t len, uint16_t crc16val)
{
  return crc16xmodempart(src, len, crc16val);
}

/****************************************************************************
 * Name: crc16
 *
 * Description:
 *   Return a 16-bit CRC of the contents of the 'src' buffer, length 'len'
 *
 *   The default polynomial is 0x1021 (x^16 + x^12 + x^5 + 1)
 *   See crc16xmodem()
 *
 ****************************************************************************/

uint16_t crc16(FAR const uint8_t *src, size_t len)
{
  return crc16xmodempart(src, len, 0);
}
