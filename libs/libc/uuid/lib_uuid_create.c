/****************************************************************************
 * libs/libc/uuid/lib_uuid_create.c
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

#include <errno.h>
#include <stdlib.h>
#include <uuid.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * uuid_create_nil
 *
 * Description:
 *   Create an UUID.
 *
 * See also:
 *   http://www.opengroup.org/onlinepubs/009629399/uuid_create.htm
 *
 ****************************************************************************/

void uuid_create(FAR uuid_t *u, FAR uint32_t *status)
{
  arc4random_buf(u, sizeof(uuid_t));

  u->clock_seq_hi_and_reserved &= ~(1 << 6);
  u->clock_seq_hi_and_reserved |= (1 << 7);

  u->time_hi_and_version &= ~(1 << 12);
  u->time_hi_and_version &= ~(1 << 13);
  u->time_hi_and_version |= (1 << 14);
  u->time_hi_and_version &= ~(1 << 15);

  if (status)
    {
      *status = uuid_s_ok;
    }
}
