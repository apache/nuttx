/****************************************************************************
 * libs/libc/stdlib/lib_checkbase.c
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
#include <ctype.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_checkbase
 *
 * Description:
 *   This is part of the strol() family implementation.  This function checks
 *   the initial part of a string to see if it can determine the numeric
 *   base that is represented.
 *
 * Assumptions:
 *   *ptr points to the first, non-whitespace character in the string.
 *
 * Returned Value:
 *   - if base is valid, the actual base to use, and pptr is updated to point
 *     at the first digit.
 *   - if base is invalid (<2 or >36), return -1.
 *
 ****************************************************************************/

int lib_checkbase(int base, FAR const char **pptr)
{
  FAR const char *ptr = *pptr;

  /* Check for unspecified base */

  if (!base)
    {
      /* Assume base 10 */

      base = 10;

      /* Check for leading '0' - that would signify octal
       * or hex (or binary)
       */

      if (*ptr == '0')
        {
          /* Assume octal */

          base = 8;
          ptr++;

          /* Check for hexadecimal */

          if ((*ptr == 'X' || *ptr == 'x') &&
              lib_isbasedigit(ptr[1], 16, NULL))
            {
              base = 16;
              ptr++;
            }
        }
    }

  /* If it a hexadecimal representation,
   * than discard any leading "0X" or "0x"
   */

  else if (base == 16)
    {
      if (ptr[0] == '0' && (ptr[1] == 'X' || ptr[1] == 'x'))
        {
          ptr += 2;
        }
    }

  /* Check for incorrect bases. */

  else if (base < 2 || base > 26)
    {
      return -1; /* Means incorrect base */
    }

  /* Return the updated pointer and base */

  *pptr = ptr;
  return base;
}
