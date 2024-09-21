/****************************************************************************
 * include/nuttx/atomic.h
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

#ifndef __INCLUDE_NUTTX_ATOMIC_H
#define __INCLUDE_NUTTX_ATOMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __has_include
#  if defined(__cplusplus) && __has_include(<atomic>)
extern "C++"
{
#    include <atomic>

#    define ATOMIC_VAR_INIT(value) (value)

  using std::memory_order;
  using std::atomic_bool;
  using std::atomic_char;
  using std::atomic_schar;
  using std::atomic_uchar;
  using std::atomic_short;
  using std::atomic_ushort;
  using std::atomic_int;
  using std::atomic_uint;
  using std::atomic_long;
  using std::atomic_ulong;
  using std::atomic_llong;
  using std::atomic_ullong;

  using std::atomic_load;
  using std::atomic_load_explicit;
  using std::atomic_store;
  using std::atomic_store_explicit;
  using std::atomic_exchange;
  using std::atomic_exchange_explicit;
  using std::atomic_compare_exchange_strong;
  using std::atomic_compare_exchange_strong_explicit;
  using std::atomic_compare_exchange_weak;
  using std::atomic_compare_exchange_weak_explicit;
  using std::atomic_flag_test_and_set;
  using std::atomic_flag_test_and_set_explicit;
  using std::atomic_flag_clear;
  using std::atomic_flag_clear_explicit;
  using std::atomic_fetch_add;
  using std::atomic_fetch_add_explicit;
  using std::atomic_fetch_sub;
  using std::atomic_fetch_sub_explicit;
  using std::atomic_fetch_and;
  using std::atomic_fetch_and_explicit;
  using std::atomic_fetch_or;
  using std::atomic_fetch_or_explicit;
  using std::atomic_fetch_xor;
  using std::atomic_fetch_xor_explicit;
}
#  elif __has_include(<stdatomic.h>) && \
        ((defined(__cplusplus) && __cplusplus >= 201103L) || \
         (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L))
#    if !(__clang__) && defined(__cplusplus)
#      define _Atomic
#    endif
#    include <stdbool.h>
#    include <stdatomic.h>
#    ifndef ATOMIC_VAR_INIT
#      define ATOMIC_VAR_INIT(value) (value)
#    endif
#  else
#    include <nuttx/lib/stdatomic.h>
#  endif
#else
#  include <nuttx/lib/stdatomic.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ATOMIC_H */
