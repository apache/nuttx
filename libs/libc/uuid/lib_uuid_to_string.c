/****************************************************************************
 * libs/libc/uuid/lib_uuid_to_string.c
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

#include <stdio.h>
#include <string.h>
#include <uuid.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * uuid_to_string
 *
 * Description:
 *   Convert a binary UUID into a string representation.
 *
 * See also:
 *   http://www.opengroup.org/onlinepubs/009629399/uuid_to_string.htm
 *
 ****************************************************************************/

void uuid_to_string(const uuid_t *u, char **s, uint32_t *status)
{
  static const uuid_t nil;
  int c;

  if (status != NULL)
    {
      *status = uuid_s_ok;
    }

  /* Why allow a NULL-pointer here? */

  if (s == NULL)
    {
      return;
    }

  if (u == NULL)
    {
      u = &nil;
    }

  c = asprintf(s,
        "%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        u->time_low, u->time_mid, u->time_hi_and_version,
        u->clock_seq_hi_and_reserved, u->clock_seq_low, u->node[0],
        u->node[1], u->node[2], u->node[3], u->node[4], u->node[5]);
  if (c == -1 && status != NULL)
    {
      *status = uuid_s_no_memory;
    }
}
