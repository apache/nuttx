/****************************************************************************
 * fs/nxffs/nxffs_util.c
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

#include <nuttx/config.h>

#include <string.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_rdle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

uint16_t nxffs_rdle16(FAR const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: nxffs_wrle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxffs_wrle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: nxffs_rdle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint32_t representing the whole 32-bit integer value
 *
 ****************************************************************************/

uint32_t nxffs_rdle32(FAR const uint8_t *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)nxffs_rdle16(&val[2]) << 16 | (uint32_t)nxffs_rdle16(val);
}

/****************************************************************************
 * Name: nxffs_wrle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxffs_wrle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  nxffs_wrle16(dest, (uint16_t)(val & 0xffff));
  nxffs_wrle16(dest + 2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: nxffs_erased
 *
 * Description:
 *   Check if a block of memory is in the erased state.
 *
 * Input Parameters:
 *   buffer - Address of the start of the memory to check.
 *   buflen - The number of bytes to check.
 *
 * Returned Value:
 *   The number of erased bytes found at the beginning of the memory region.
 *
 ****************************************************************************/

size_t nxffs_erased(FAR const uint8_t *buffer, size_t buflen)
{
  size_t nerased = 0;

  for (; nerased < buflen; nerased++)
    {
      if (*buffer != CONFIG_NXFFS_ERASEDSTATE)
        {
          break;
        }

      buffer++;
    }

  return nerased;
}
