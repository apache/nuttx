/****************************************************************************
 * libs/libc/uuid/lib_uuid_equal.c
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

#include <string.h>
#include <uuid.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * uuid_equal
 *
 * Description:
 *   Compare for equality.
 *
 * See also:
 *   http://www.opengroup.org/onlinepubs/009629399/uuid_equal.htm
 *
 ****************************************************************************/

int32_t uuid_equal(const uuid_t *a, const uuid_t *b, uint32_t *status)
{
  if (status != NULL)
    {
      *status = uuid_s_ok;
    }

  /* Deal with equal or NULL pointers. */

  if (a == b)
    {
      return 1;
    }

  if (a == NULL)
    {
      return uuid_is_nil(b, NULL);
    }

  if (b == NULL)
    {
      return uuid_is_nil(a, NULL);
    }

  /* Do a byte for byte comparison. */

  return !memcmp(a, b, sizeof(uuid_t));
}
