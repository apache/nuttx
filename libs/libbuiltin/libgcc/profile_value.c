/****************************************************************************
 * libs/libbuiltin/libgcc/profile_value.c
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

#include <stdlib.h>
#include <syslog.h>

#include <stdatomic.h>
#include <nuttx/spinlock.h>

#include "gcov_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct gcov_kvp_list *gcov_kvp_list_pointer = NULL;
static struct gcov_kvp_list gcov_kvp_pool[CONFIG_GCOV_COUNTER_POOL_SIZE];
static spinlock_t gcov_kvp_pool_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* WARNING: This global variable has critical concurrency issues in both SMP
 * and single-CPU environments:
 *
 * 1. SMP Safety Issue:
 *    GCC directly injects code to access this variable without any
 *    protection mechanisms (locks, atomics, etc.), leading to race
 *    conditions in multi-processor systems.
 *    Unless GCC is compiled with --enable-tls, it won't use __thread
 *    qualifier to make this variable thread-local.
 *
 * 2. Interrupt Safety Issue:
 *    Even on single-CPU systems, GCC injects code to access this variable
 *    in interrupt handlers during profiling operations, corrupting the data
 *    when the interrupted context resumes execution. This results in
 *    incomplete or inaccurate performance profiling data.
 *
 * Recommendation:
 *    Consider using Clang/LLVM PGO instead, as Clang does not rely on global
 *    variable for indirect call profiling, avoiding these concurrency issues
 */

struct indirect_call_tuple __gcov_indirect_call;
gcov_type __gcov_time_profiler_counter;

/****************************************************************************
 * Name: gcov_counter_add
 *
 * Description:
 *   Add VALUE to *COUNTER and make it with atomic operation
 *   if USE_ATOMIC is true.
 *
 ****************************************************************************/

static inline void
gcov_counter_add(FAR atomic_ulong *counter, gcov_type value,
                 int use_atomic)
{
  if (use_atomic)
    {
      atomic_fetch_add(counter, value);
    }
  else
    {
      *counter += value;
    }
}

/****************************************************************************
 * Name: allocate_gcov_kvp
 *
 * Description:
 *   Allocate gcov_kvp from statically pre-allocated pool,
 *   or use heap otherwise.
 *
 * Returned Value:
 *   Return pointer to gcov_kvp.
 *
 ****************************************************************************/

static inline struct gcov_kvp *allocate_gcov_kvp(void)
{
  FAR struct gcov_kvp_list *new_node = NULL;
  unsigned kvp_sizeof = sizeof(struct gcov_kvp_list);
  irqstate_t flags;

  flags = spin_lock_irqsave(&gcov_kvp_pool_lock);

  if (gcov_kvp_list_pointer == NULL)
    {
      if (gcov_kvp_pool[0].next == NULL)
        {
          /* Initialize the pool's next pointers */

          for (unsigned int i = 0; i < CONFIG_GCOV_COUNTER_POOL_SIZE; ++i)
            {
              gcov_kvp_pool[i].next =
                (i < CONFIG_GCOV_COUNTER_POOL_SIZE - 1) ?
                &gcov_kvp_pool[i + 1] : NULL;
            }

          gcov_kvp_list_pointer = &gcov_kvp_pool[0];
        }
      else
        {
          /* Allocate memory from heap only if the pool is empty */

          new_node = (FAR struct gcov_kvp_list *)calloc(1, kvp_sizeof);
          if (new_node == NULL)
            {
              spin_unlock_irqrestore(&gcov_kvp_pool_lock, flags);
              syslog(LOG_WARNING, "failed to alloc gcov_kvp_list");
              abort();
            }

          gcov_kvp_list_pointer = new_node;
        }
    }

  new_node = gcov_kvp_list_pointer;
  gcov_kvp_list_pointer = gcov_kvp_list_pointer->next;

  spin_unlock_irqrestore(&gcov_kvp_pool_lock, flags);

  return &(new_node->value);
}

/****************************************************************************
 * Name: gcov_topn_add_value
 *
 * Description:
 *   Add key value pair VALUE:COUNT to a top N COUNTERS.
 *   When INCREMENT_TOTAL is true, add COUNT to total of the TOP counter.
 *   If USE_ATOMIC is true, do it in atomic way.
 *
 * Returned Value:
 *   Return 1 when the counter is full, otherwise return 0.
 *
 ****************************************************************************/

nosanitize_address
static inline unsigned gcov_topn_add_value(FAR atomic_ulong *counters,
                                           gcov_type value,
                                           gcov_type count,
                                           int use_atomic,
                                           int increment_total)
{
  FAR struct gcov_kvp *prev_node = NULL;
  FAR struct gcov_kvp *minimal_node = NULL;
  FAR struct gcov_kvp *current_node = (struct gcov_kvp *)counters[2];

  if (increment_total)
    {
      /* In multi-threaded scenarios, it's possible for a merged profile
       * to end up with a negative total.If this happens,
       * we should exit early.
       */

      if (counters[0] < 0)
        {
          return 0;
        }

      gcov_counter_add(&counters[0], 1, use_atomic);
    }

  while (current_node)
    {
      if (current_node->value == value)
        {
          gcov_counter_add(&current_node->count, count, use_atomic);
          return 0;
        }

      if (minimal_node == NULL || current_node->count < minimal_node->count)
        {
          minimal_node = current_node;
        }

      prev_node = current_node;
      current_node = current_node->next;
    }

  /* 32 is the maximum number of tracked TOP N values.
   * If we have already tracked 32 values, we try to see
   * if the new value's count is larger than the minimal
   * one we have tracked.
   * If yes, we replace the minimal one with the new value.
   */

  if (counters[1] == 32)
    {
      if (--minimal_node->count < count)
        {
          minimal_node->value = value;
          minimal_node->count = count;
        }

      return 1;
    }
  else
    {
      current_node = allocate_gcov_kvp();
      if (current_node == NULL)
        {
          return 0;
        }

      current_node->value = value;
      current_node->count = count;

      int success = 0;
      if (!counters[2])
        {
          if (use_atomic)
            {
              intptr_t expected = 0;
              FAR atomic_intptr_t *ptr = (FAR atomic_intptr_t *)&counters[2];
              success =
                atomic_compare_exchange_strong(ptr, &expected,
                                               (intptr_t)current_node);
            }
          else
            {
              counters[2] = (intptr_t)current_node;
              success = 1;
            }
        }
      else  if (prev_node && !prev_node->next)
        {
          if (use_atomic)
            {
              intptr_t expected = 0;
              FAR atomic_intptr_t *ptr =
                (atomic_intptr_t *)(&prev_node->next);
              success =
                atomic_compare_exchange_strong(ptr, &expected,
                                               (intptr_t)current_node);
            }
          else
            {
              prev_node->next = current_node;
              success = 1;
            }
        }

      /* If insert successful, we should increment number of nodes. */

      if (success)
        {
          gcov_counter_add(&counters[1], 1, use_atomic);
        }
    }

  return 0;
}

static inline void gcov_topn_values_profiler_body(FAR atomic_ulong *counters,
                                                  gcov_type value,
                                                  int use_atomic)
{
  gcov_topn_add_value(counters, value, 1, use_atomic, 1);
}

/****************************************************************************
 * Name: gcov_indirect_call_profiler_body
 *
 * Description:
 *   Profiles indirect function calls by recording the most frequently called
 *   target functions.
 *   If C++ virtual tables use function descriptors, it dereferences
 *   the descriptors to check if they point to the same function.
 *   Updates the profiling counters accordingly.
 *
 * Assumptions:
 *   The __gcov_indirect_call structure is properly initialized
 *   before calling this function.
 *
 ****************************************************************************/

nosanitize_address
static inline void gcov_indirect_call_profiler_body(gcov_type value,
                                                    FAR void *cur_func,
                                                    int use_atomic)
{
  if (cur_func == __gcov_indirect_call.callee)
    {
      gcov_topn_values_profiler_body(__gcov_indirect_call.counters,
                                     value, use_atomic);
    }

#ifdef GCOV_CXX_VTABLE_USES_DESCRIPTORS
  /* If the C++ virtual tables contain function descriptors then one
   * function may have multiple descriptors and we need to dereference
   * the descriptors to see if they point to the same function.
   */

  else if (__gcov_indirect_call.callee &&
          *(FAR void **)cur_func ==
            *(FAR void **)__gcov_indirect_call.callee)
    {
      gcov_topn_values_profiler_body(__gcov_indirect_call.counters,
                                     value, use_atomic);
    }
#endif

  __gcov_indirect_call.callee = NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __gcov_merge_topn(FAR gcov_type *counters, unsigned n_counters)
{
}

void __gcov_merge_ior(FAR gcov_type *counters, unsigned n_counters)
{
}

void __gcov_merge_time_profile(FAR gcov_type *counters, unsigned n_counters)
{
}

void __gcov_indirect_call_profiler_v4(gcov_type value, FAR void *cur_func)
{
  gcov_indirect_call_profiler_body(value, cur_func, 0);
}

void __gcov_indirect_call_profiler_v4_atomic(gcov_type value,
                                             FAR void *cur_func)
{
  gcov_indirect_call_profiler_body(value, cur_func, 1);
}

void __gcov_topn_values_profiler(FAR atomic_ulong *counters, gcov_type value)
{
  gcov_topn_values_profiler_body(counters, value, 0);
}

/****************************************************************************
 * Name: __gcov_topn_values_profiler_atomic
 *
 * Description:
 *   Update one value profilers (COUNTERS) for a given VALUE in atomic way.
 *
 * Note:
 *   This function is not fully atomic.
 *   If it were just an atomic update of `COUNTERS[2]`, it would meet our
 *   expectations. However, the update involves three fields:
 *      `COUNTERS[0]`, `COUNTERS[1]`, and `COUNTERS[2]`.
 *
 *   - COUNTERS[0] represents the sum of all counter values
 *   - COUNTERS[1] represents the number of counters (unique values tracked).
 *   - COUNTERS[2] is a pointer to a linked list of gcov_kvp structures
 *
 *   All three fields are updated together, but this overall update is not
 *   atomic.
 *
 * For more information please read the following email thread:
 *  https://gcc.gnu.org/ml/gcc-patches/2016-08/msg00024.html.
 *
 ****************************************************************************/

void __gcov_topn_values_profiler_atomic(FAR atomic_ulong *counters,
                                        gcov_type value)
{
  gcov_topn_values_profiler_body(counters, value, 1);
}

/****************************************************************************
 * Name: __gcov_pow2_profiler
 *
 * Description:
 *   Increase corresponding COUNTER by VALUE.
 *
 ****************************************************************************/

void __gcov_average_profiler(FAR gcov_type *counters, gcov_type value)
{
  counters[0] += value;
  counters[1] += 1;
}

void __gcov_average_profiler_atomic(FAR atomic_ulong *counters,
                                    gcov_type value)
{
  atomic_fetch_add(&counters[0], value);
  atomic_fetch_add(&counters[1], 1);
}

/****************************************************************************
 * Name: __gcov_pow2_profiler
 *
 * Description:
 *   If VALUE is a power of two, COUNTERS[1] is incremented.
 *   Otherwise COUNTERS[0] is incremented.
 *
 ****************************************************************************/

void __gcov_pow2_profiler(FAR gcov_type *counters, gcov_type value)
{
  if (value == 0 || (value & (value - 1)))
    {
      counters[0]++;
    }
  else
    {
      counters[1]++;
    }
}

void __gcov_pow2_profiler_atomic(FAR atomic_ulong *counters, gcov_type value)
{
  if (value == 0 || (value & (value - 1)))
    {
      atomic_fetch_add((&counters[0]), 1);
    }
  else
    {
      atomic_fetch_add((&counters[1]), 1);
    }
}

/****************************************************************************
 * Name: __gcov_ior_profiler
 *
 * Description:
 *   Bitwise-OR VALUE into COUNTER.
 *
 ****************************************************************************/

void __gcov_ior_profiler(FAR gcov_type *counters, gcov_type value)
{
  *counters |= value;
}

void __gcov_ior_profiler_atomic(FAR atomic_ulong *counters, gcov_type value)
{
  atomic_fetch_or(&counters[0], value);
}

/****************************************************************************
 * Name: __gcov_interval_profiler
 *
 * Description:
 *   If VALUE is in interval <START, START + STEPS - 1>, then increases the
 *   corresponding counter in COUNTERS.  If the VALUE is above or below
 *   the interval, COUNTERS[STEPS] or COUNTERS[STEPS + 1] is increased
 *   instead.
 *
 ****************************************************************************/

void __gcov_interval_profiler(FAR gcov_type *counters, gcov_type value,
                              int start, unsigned steps)
{
  gcov_type delta = value - start;

  if (delta < 0)
    {
      counters[steps + 1]++;
    }
  else if (delta >= steps)
    {
      counters[steps]++;
    }
  else
    {
      counters[delta]++;
    }
}

void __gcov_interval_profiler_atomic(FAR atomic_ulong *counters,
                                     gcov_type value,
                                     int start, unsigned steps)
{
  gcov_type delta = value - start;

  if (delta < 0)
    {
      atomic_fetch_add(&counters[steps + 1], 1);
    }
  else if (delta >= steps)
    {
      atomic_fetch_add(&counters[steps], 1);
    }
  else
    {
      atomic_fetch_add(&counters[delta], 1);
    }
}
