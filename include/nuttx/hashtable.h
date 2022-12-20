/****************************************************************************
 * include/nuttx/hashtable.h
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

#ifndef __INCLUDE_NUTTX_HASHTABLE_H
#define __INCLUDE_NUTTX_HASHTABLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/lib/math32.h>
#include <nuttx/nuttx.h>
#include <nuttx/queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hash function related definitions. */

#define GOLDEN_RATIO_32 0x61C88647 /* Negative golden ratio, used by Linux. */
#define HASH(val, bits) ((uint32_t)((val) * GOLDEN_RATIO_32) >> (32 - (bits)))

/* Hashtable related definitions. */

#define DECLARE_HASHTABLE(table, bits) hash_head_t table[1 << (bits)]

#define hashtable_size(table) (sizeof(table) / sizeof((table)[0]))
#define hashtable_bits(table) (LOG2_FLOOR(hashtable_size(table)))

#define hashtable_init(table) \
  do \
    { \
      int i; \
      for (i = 0; i < hashtable_size(table); i++) \
        { \
          dq_init(&table[i]); \
        } \
    } \
  while(0)

#define hashtable_add(table, item, key) \
  dq_addfirst(item, &table[HASH(key, hashtable_bits(table))])

#define hashtable_delete(table, item, key) \
  dq_rem(item, &table[HASH(key, hashtable_bits(table))])

/* Iterate over whole hashtable. */

#define hashtable_for_every(table, item, i)         \
  for ((i) = 0; (i) < hashtable_size(table); (i)++) \
    sq_for_every(&table[i], item)

#define hashtable_for_every_safe(table, item, temp, i) \
  for ((i) = 0; (i) < hashtable_size(table); (i)++)    \
    sq_for_every_safe(&table[i], item, temp)

/* Iterate over all possible objects hashing to the same bucket. */

#define hashtable_for_every_possible(table, item, key) \
  sq_for_every(&table[HASH(key, hashtable_bits(table))], item)

#define hashtable_for_every_possible_safe(table, item, temp, key) \
  sq_for_every_safe(&table[HASH(key, hashtable_bits(table))], item, temp)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Note: Our `list` is not suitable for hashtable, because `list` will always
 * access `head` when iterating by `for_every`, but `head` may sometimes be
 * accessed by hash result, then hash function will be called on each
 * iteration. So we use `dq` instead to avoid this.
 *
 * Note: There is an optimized type `hlist` on Linux, which saves one pointer
 * in list head, we can do the same optimization to save memory (but only
 * saves a little compares to the whole hashtable).
 */

typedef dq_queue_t hash_head_t;
typedef dq_entry_t hash_node_t;

#endif /* __INCLUDE_NUTTX_HASHTABLE_H */
