/****************************************************************************
 * libs/libbuiltin/libgcc/profile_write.c
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

#include <stddef.h>
#include <stdlib.h>

#include "gcov_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: write_topn_counters
 *
 * Description:
 *   Store all TOP N counters where each has a dynamic length.
 *
 ****************************************************************************/

void write_topn_counters(FAR void *buffer, FAR size_t *off,
                         FAR const struct gcov_ctr_info *ci_ptr,
                         unsigned t_ix, gcov_unsigned_t n_counts)
{
  size_t i = 0;
  unsigned int disk_size = 0;
  unsigned int pair_total = 0;
  FAR struct gcov_kvp *node = NULL;
  static unsigned int list_size_length = 0;
  FAR static unsigned int *list_sizes = NULL;
  const unsigned int counters = n_counts / GCOV_TOPN_MEM_COUNTERS;

  assert(n_counts % GCOV_TOPN_MEM_COUNTERS == 0);

  /* In multi-threaded environments, the count of counters may not always
   * match the length of their associated linked lists.
   */

#define LIST_SIZE_MIN_LENGTH 4 * 1024

  if (list_sizes == NULL || counters > list_size_length)
    {
      list_size_length = LIST_SIZE_MIN_LENGTH > 2 * counters ?
                         LIST_SIZE_MIN_LENGTH : 2 * counters;
      list_sizes = (FAR unsigned int *)realloc(
        list_sizes, list_size_length * sizeof(unsigned int));
    }

  for (i = 0; i < counters; ++i)
    {
      unsigned int sizes = 0;
      gcov_type start = ci_ptr->values[GCOV_TOPN_MEM_COUNTERS * i + 2];

      node = (FAR struct gcov_kvp *)(__INTPTR_TYPE__)start;

      for (; node != NULL; node = node->next)
        {
          ++sizes;
        }

      pair_total += sizes;
      list_sizes[i] = sizes;
    }

  /* 2 * counters:
   *    Each TOP N counter has two fixed values on disk:
   *      total count and number of key-value pairs.
   * 2 * pair_total:
   *    Each key-value pair has two values on disk: key and count.
   */

  disk_size = 2 * counters + 2 * pair_total;

  dump_unsigned(buffer, off, GCOV_TAG_FOR_COUNTER(t_ix));
  dump_unsigned(buffer, off, GCOV_TAG_COUNTER_LENGTH(disk_size));

  for (i = 0; i < counters; ++i)
    {
      size_t j = 0;
      gcov_type start = ci_ptr->values[GCOV_TOPN_MEM_COUNTERS * i + 2];

      node = (FAR struct gcov_kvp *)(__INTPTR_TYPE__)start;

      dump_counter(buffer, off, ci_ptr->values[GCOV_TOPN_MEM_COUNTERS * i]);
      dump_counter(buffer, off, list_sizes[i]);

      for (; j < list_sizes[i]; node = node->next, ++j)
        {
          dump_counter(buffer, off, node->value);
          dump_counter(buffer, off, node->count);
        }
    }
}
