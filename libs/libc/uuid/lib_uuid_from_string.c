/****************************************************************************
 * libs/libc/uuid/lib_uuid_from_string.c
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

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <uuid.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * uuid_from_string
 *
 * Description:
 *   convert a string representation of an UUID into a binary representation.
 *
 * See also:
 *   http://www.opengroup.org/onlinepubs/009629399/uuid_from_string.htm
 *
 ****************************************************************************/

void uuid_from_string(const char *s, uuid_t *u, uint32_t *status)
{
  int n;

  /* Short-circuit 2 special cases: NULL pointer and empty string. */

  if (s == NULL || *s == '\0')
    {
      uuid_create_nil(u, status);
      return;
    }

  /* Assume the worst. */

  if (status != NULL)
    {
      *status = uuid_s_invalid_string_uuid;
    }

  /* The UUID string representation has a fixed length. */

  if (strnlen(s, UUID_STR_LEN + 1) != UUID_STR_LEN)
    {
      return;
    }

  /* We only work with "new" UUIDs. New UUIDs have the form:
   *  01234567-89ab-cdef-0123-456789abcdef
   * The so called "old" UUIDs, which we don't support, have the form:
   *  0123456789ab.cd.ef.01.23.45.67.89.ab
   */

  if (s[8] != '-')
    {
      return;
    }

  n = sscanf(s,
        "%08" SCNx32 "-%04" SCNx16 "-%04" SCNx16
        "-%02" SCNx8 "%02" SCNx8
        "-%02" SCNx8 "%02" SCNx8 "%02" SCNx8
        "%02" SCNx8 "%02" SCNx8 "%02" SCNx8,
        &u->time_low, &u->time_mid, &u->time_hi_and_version,
        &u->clock_seq_hi_and_reserved, &u->clock_seq_low, &u->node[0],
        &u->node[1], &u->node[2], &u->node[3], &u->node[4], &u->node[5]);

  /* Make sure we have all conversions. */

  if (n != 11)
    {
      return;
    }

  /* We have a successful scan. Check semantics... */

  n = u->clock_seq_hi_and_reserved;
  if ((n & 0x80) != 0x00 && /* variant 0? */
      (n & 0xc0) != 0x80 && /* variant 1? */
      (n & 0xe0) != 0xc0)   /* variant 2? */
    {
      if (status != NULL)
        {
          *status = uuid_s_bad_version;
        }
    }
  else
    {
      if (status != NULL)
        {
          *status = uuid_s_ok;
        }
    }
}
