/****************************************************************************
 * libs/libc/uuid/lib_uuid_stream.c
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

#include <endian.h>
#include <string.h>
#include <uuid.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uuid_enc_le(void *buf, const uuid_t *uuid)
{
  uuid_t *temp = buf;

  memcpy(temp, uuid, sizeof(*uuid));
#if BYTE_ORDER == BIG_ENDIAN
  temp->time_low = htole32(temp->time_low);
  temp->time_mid = htole16(temp->time_mid);
  temp->time_hi_and_version = htole16(temp->time_hi_and_version);
#endif
}

void uuid_dec_le(const void *buf, uuid_t *uuid)
{
  const uuid_t *temp = buf;

  memcpy(uuid, temp, sizeof(*uuid));
#if BYTE_ORDER == BIG_ENDIAN
  uuid->time_low = le32toh(uuid->time_low);
  uuid->time_mid = le16toh(uuid->time_mid);
  uuid->time_hi_and_version = le16toh(uuid->time_hi_and_version);
#endif
}

void uuid_enc_be(void *buf, const uuid_t *uuid)
{
  uuid_t *temp = buf;

  memcpy(temp, uuid, sizeof(*uuid));
#if BYTE_ORDER == LITTLE_ENDIAN
  temp->time_low = htobe32(temp->time_low);
  temp->time_mid = htobe16(temp->time_mid);
  temp->time_hi_and_version = htobe16(temp->time_hi_and_version);
#endif
}

void uuid_dec_be(const void *buf, uuid_t *uuid)
{
  const uuid_t *temp = buf;

  memcpy(uuid, temp, sizeof(*uuid));
#if BYTE_ORDER == LITTLE_ENDIAN
  uuid->time_low = be32toh(uuid->time_low);
  uuid->time_mid = be16toh(uuid->time_mid);
  uuid->time_hi_and_version = be16toh(uuid->time_hi_and_version);
#endif
}
