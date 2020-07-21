/****************************************************************************
 * include/uuid.h
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

#ifndef __INCLUDE_UUID_H
#define __INCLUDE_UUID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Status codes returned by the functions. */

#define uuid_s_ok                       0
#define uuid_s_bad_version              1
#define uuid_s_invalid_string_uuid      2
#define uuid_s_no_memory                3

/* Length of a node address (an IEEE 802 address). */

#define UUID_NODE_LEN                   6

/* Length of a UUID. */

#define UUID_STR_LEN                    36

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * A DCE 1.1 compatible source representation of UUIDs.
 * 0                   1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |                          time_low                             |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |       time_mid                |         time_hi_and_version   |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |clk_seq_hi_res |  clk_seq_low  |         node (0-1)            |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |                         node (2-5)                            |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 ****************************************************************************/

struct uuid
{
  uint32_t time_low;
  uint16_t time_mid;
  uint16_t time_hi_and_version;
  uint8_t  clock_seq_hi_and_reserved;
  uint8_t  clock_seq_low;
  uint8_t  node[UUID_NODE_LEN];
};

typedef struct uuid uuid_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int32_t  uuid_compare(const uuid_t *, const uuid_t *, uint32_t *);
void     uuid_create(uuid_t *, uint32_t *);
void     uuid_create_nil(uuid_t *, uint32_t *);
int32_t  uuid_equal(const uuid_t *, const uuid_t *, uint32_t *);
void     uuid_from_string(const char *, uuid_t *, uint32_t *);
uint16_t uuid_hash(const uuid_t *, uint32_t *);
int32_t  uuid_is_nil(const uuid_t *, uint32_t *);
void     uuid_to_string(const uuid_t *, char **, uint32_t *);

void     uuid_enc_le(void *, const uuid_t *);
void     uuid_dec_le(const void *, uuid_t *);
void     uuid_enc_be(void *, const uuid_t *);
void     uuid_dec_be(const void *, uuid_t *);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_UUID_H */
