/****************************************************************************
 * libs/libc/uuid/lib_uuid_create_nil.c
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
 * uuid_create_nil
 *
 * Description:
 *   Create a nil UUID.
 *
 * See also:
 *   http://www.opengroup.org/onlinepubs/009629399/uuid_create_nil.htm
 *
 ****************************************************************************/

void uuid_create_nil(uuid_t *u, uint32_t *status)
{
  memset(u, 0, sizeof(*u));

  if (status)
    {
      *status = uuid_s_ok;
    }
}
