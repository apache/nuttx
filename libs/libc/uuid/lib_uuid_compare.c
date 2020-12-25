/****************************************************************************
 * libs/libc/uuid/lib_uuid_compare.c
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
 * Pre-processor definitions
 ****************************************************************************/

/* A macro used to improve the readability of uuid_compare(). */

#define DIFF_RETURN(a, b, field)                   \
  do                                               \
    {                                              \
      if ((a)->field != (b)->field)                \
        {                                          \
          return (a)->field < (b)->field ? -1 : 1; \
        }                                          \
    }                                              \
  while (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * uuid_compare
 *
 * Description:
 *   Compare two UUIDs.
 *
 * See also:
 *   http://www.opengroup.org/onlinepubs/009629399/uuid_compare.htm
 *
 ****************************************************************************/

int32_t uuid_compare(const uuid_t *a, const uuid_t *b, uint32_t *status)
{
  if (status != NULL)
    {
      *status = uuid_s_ok;
    }

  /* Deal with NULL or equal pointers. */

  if (a == b)
    {
      return 0;
    }

  if (a == NULL)
    {
      return uuid_is_nil(b, NULL) ? 0 : -1;
    }

  if (b == NULL)
    {
      return uuid_is_nil(a, NULL) ? 0 : 1;
    }

  /* We have to compare the hard way. */

  DIFF_RETURN(a, b, time_low);
  DIFF_RETURN(a, b, time_mid);
  DIFF_RETURN(a, b, time_hi_and_version);
  DIFF_RETURN(a, b, clock_seq_hi_and_reserved);
  DIFF_RETURN(a, b, clock_seq_low);

  return memcmp(a->node, b->node, sizeof(a->node));
}
