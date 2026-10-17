/****************************************************************************
 * include/nuttx/net/bluetooth/compat/sys/util.h
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_UTIL_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_UTIL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/atomic.h>
#include <nuttx/compiler.h>
#include <nuttx/list.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* IS_ENABLED trick */
#define Z_IS_ENABLED1(config_macro) Z_IS_ENABLED2(_XXXX##config_macro)
#define _XXXX1 _YYYY,
#define Z_IS_ENABLED2(one_or_two_args) Z_IS_ENABLED3(one_or_two_args 1, 0)
#define Z_IS_ENABLED3(ignore, val, ...) val
#define IS_ENABLED(config_macro) Z_IS_ENABLED1(config_macro)

/* Conditional code selection */
#define COND_CODE_1(_flag, _if_1_code, _else_code) __COND_CODE(_flag, _if_1_code, _else_code)
#define __COND_CODE(_flag, _if_1_code, _else_code)                                                 \
___COND_CODE(Z_IS_ENABLED1(_flag), _if_1_code, _else_code)
#define ___COND_CODE(_flag, _if_1_code, _else_code)                                                \
____COND_CODE(UTIL_CAT(_CC1_, _flag), _if_1_code, _else_code)
#define ____COND_CODE(selector, _if_1_code, _else_code) selector(_if_1_code, _else_code)
#define Z_DEBRACKET(...) __VA_ARGS__
#define _CC1_1(_if_1_code, _else_code) Z_DEBRACKET _if_1_code
#define _CC1_0(_if_1_code, _else_code) Z_DEBRACKET _else_code

/* IF_ENABLED - compile code only when config is enabled */
#define IF_ENABLED(_cfg, _code) COND_CODE_1(_cfg, _code, ())

/* Arithmetic - using simple ternary for constant-foldability */

/* Force override toolchain.h statement-expression-compatible versions */
#ifdef Z_MAX
#undef Z_MAX
#endif
#ifndef Z_MAX
#define Z_MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifdef Z_MIN
#undef Z_MIN
#endif
#ifndef Z_MIN
#define Z_MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifdef Z_CLAMP
#undef Z_CLAMP
#endif
#ifndef Z_CLAMP
#define Z_CLAMP(val, low, high) Z_MIN(Z_MAX(val, low), high)
#endif

#ifndef CLAMP
#define CLAMP(val, low, high) Z_CLAMP(val, low, high)
#endif

/* MIN/MAX convenience */
#ifndef MAX
#define MAX(a, b) Z_MAX(a, b)
#endif

#ifndef MIN
#define MIN(a, b) Z_MIN(a, b)
#endif

/* Container of */
#ifndef container_of
#define container_of(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) container_of(ptr, type, member)
#endif

/* List helpers used by the Bluetooth compatibility layer.  Keep these here
 * instead of extending the generic NuttX list API.
 */

#define list_for_every_container(list, entry, member)                                              \
list_for_every_entry(list, entry, __typeof__(*(entry)), member)

#define list_for_every_container_safe(list, entry, temp, member)                                   \
list_for_every_entry_safe(list, entry, temp, __typeof__(*(entry)), member)

#define list_peek_head_container(list, entry, member)                                              \
list_peek_head_type(list, __typeof__(*(entry)), member)

#define list_peek_tail_container(list, entry, member)                                              \
list_peek_tail_type(list, __typeof__(*(entry)), member)

#define list_next_container(list, entry, member)                                                   \
list_next_type(list, &(entry)->member, __typeof__(*(entry)), member)

static inline void bt_list_merge(struct list_node *list,
                                 struct list_node *list_to_append)
{
  struct list_node *node;

  while ((node = list_remove_head(list_to_append)) != NULL)
    {
      list_add_tail(list, node);
    }
}

#define list_merge(list, list_to_append) bt_list_merge(list, list_to_append)

static inline void bt_list_insert_after(
  struct list_node *list, struct list_node *prev, struct list_node *node)
{
  if (prev == NULL)
    {
      list_add_head(list, node);
    }
  else
    {
      list_add_after(prev, node);
    }
}

#define list_insert_after(list, prev, node) bt_list_insert_after(list, prev, node)

static inline bool bt_list_find(
  struct list_node *list, const struct list_node *node,
  struct list_node **prev)
{
  struct list_node *cur;
  struct list_node *last = NULL;

  list_for_every(list, cur)
  {
    if (cur == node)
      {
        if (prev != NULL)
          {
            *prev = last;
          }

        return true;
      }

    last = cur;
  }

  if (prev != NULL)
    {
      *prev = last;
    }

  return false;
}

#define list_find(list, node, prev) bt_list_find(list, node, prev)

static inline bool bt_list_find_and_remove(struct list_node *list,
                                           struct list_node *node)
{
  if (!bt_list_find(list, node, NULL))
    {
      return false;
    }

  list_delete(node);
  return true;
}

#define list_find_and_remove(list, node) bt_list_find_and_remove(list, node)

static inline void bt_list_remove(
  struct list_node *list, struct list_node *prev, struct list_node *node)
{
  (void)list;
  (void)prev;

  list_delete(node);
}

#define list_remove(list, prev, node) bt_list_remove(list, prev, node)

/* Array size */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

#define ARRAY_INDEX(array, ptr) ((ptr) - (array))

#define POINTER_TO_UINT(x) ((uintptr_t)(x))
#define UINT_TO_POINTER(x) ((void *)(uintptr_t)(x))

#define atomic_bits (sizeof(atomic_t) * 8)
#define atomic_bitmap_size(num_bits) (((num_bits) + atomic_bits - 1) / atomic_bits)

#define atomic_define(name, num_bits) atomic_t name[atomic_bitmap_size(num_bits)]

#define atomic_cmpxchg_value(obj, expected, desired)                                               \
({                                                                                               \
  int32_t __expected = (expected);                                                               \
  atomic_cmpxchg(obj, &__expected, desired);                                                     \
})

#define atomic_set_bit(target, bit)                                                                \
((void)atomic_or(&(target)[(bit) / atomic_bits], (int32_t)(1ULL << ((bit) % atomic_bits))))

#define atomic_clear_bit(target, bit)                                                              \
((void)atomic_and(&(target)[(bit) / atomic_bits], ~(int32_t)(1ULL << ((bit) % atomic_bits))))

#define atomic_test_bit(target, bit)                                                               \
((atomic_read(&(target)[(bit) / atomic_bits]) & (int32_t)(1ULL << ((bit) % atomic_bits))) != 0)

#define atomic_test_and_set_bit(target, bit)                                                       \
((atomic_or(&(target)[(bit) / atomic_bits], (int32_t)(1ULL << ((bit) % atomic_bits))) &          \
  (int32_t)(1ULL << ((bit) % atomic_bits))) != 0)

#define atomic_test_and_clear_bit(target, bit)                                                     \
((atomic_and(&(target)[(bit) / atomic_bits], ~(int32_t)(1ULL << ((bit) % atomic_bits))) &        \
  (int32_t)(1ULL << ((bit) % atomic_bits))) != 0)

#define atomic_set_bit_to(target, bit, val)                                                        \
do                                                                                               \
  {                                                                                              \
    if (val)                                                                                     \
      {                                                                                          \
        atomic_set_bit(target, bit);                                                             \
      }                                                                                          \
    else                                                                                         \
      {                                                                                          \
        atomic_clear_bit(target, bit);                                                           \
      }                                                                                          \
  }                                                                                              \
while (0)

#define atomic_ptr_read(target) __atomic_load_n(target, __ATOMIC_RELAXED)

#define atomic_ptr_xchg(target, value) __atomic_exchange_n(target, value, __ATOMIC_SEQ_CST)

#define atomic_ptr_cmpxchg(target, expected, desired)                                              \
({                                                                                               \
  __auto_type __expected = (expected);                                                           \
  __atomic_compare_exchange_n(target, &__expected, desired, false, __ATOMIC_SEQ_CST,             \
                              __ATOMIC_RELAXED);                                                 \
})

/* Round up/down */
#define ROUND_UP(x, align) (((x) + ((align) - 1)) & ~((align) - 1))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

/* Inclusive range check */
#ifndef IN_RANGE
#define IN_RANGE(val, min, max) (((val) >= (min)) && ((val) <= (max)))
#endif

/* Part of array */
#define PART_OF_ARRAY(array, ptr)                                                                  \
((uintptr_t)(ptr) >= (uintptr_t)(array) &&                                                       \
 (uintptr_t)(ptr) < (uintptr_t)(&(array)[ARRAY_SIZE(array)]))

#define IS_ARRAY_ELEMENT(array, ptr) PART_OF_ARRAY(array, ptr)

/* Listify - expand macro F(idx, ...) LEN times. */
#define LISTIFY(LEN, F, SEP, ...) UTIL_CAT(LISTIFY_, LEN)(F, SEP, __VA_ARGS__)
#define LISTIFY_0(F, SEP, ...)
#define LISTIFY_1(F, SEP, ...) F(0, __VA_ARGS__)
#define LISTIFY_2(F, SEP, ...) LISTIFY_1(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(1, __VA_ARGS__)
#define LISTIFY_3(F, SEP, ...) LISTIFY_2(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(2, __VA_ARGS__)
#define LISTIFY_4(F, SEP, ...) LISTIFY_3(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(3, __VA_ARGS__)
#define LISTIFY_5(F, SEP, ...) LISTIFY_4(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(4, __VA_ARGS__)
#define LISTIFY_6(F, SEP, ...) LISTIFY_5(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(5, __VA_ARGS__)
#define LISTIFY_7(F, SEP, ...) LISTIFY_6(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(6, __VA_ARGS__)
#define LISTIFY_8(F, SEP, ...) LISTIFY_7(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(7, __VA_ARGS__)
#define LISTIFY_9(F, SEP, ...) LISTIFY_8(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(8, __VA_ARGS__)
#define LISTIFY_10(F, SEP, ...) LISTIFY_9(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(9, __VA_ARGS__)
#define LISTIFY_11(F, SEP, ...) LISTIFY_10(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(10, __VA_ARGS__)
#define LISTIFY_12(F, SEP, ...) LISTIFY_11(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(11, __VA_ARGS__)
#define LISTIFY_13(F, SEP, ...) LISTIFY_12(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(12, __VA_ARGS__)
#define LISTIFY_14(F, SEP, ...) LISTIFY_13(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(13, __VA_ARGS__)
#define LISTIFY_15(F, SEP, ...) LISTIFY_14(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(14, __VA_ARGS__)
#define LISTIFY_16(F, SEP, ...) LISTIFY_15(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(15, __VA_ARGS__)
#define LISTIFY_17(F, SEP, ...) LISTIFY_16(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(16, __VA_ARGS__)
#define LISTIFY_18(F, SEP, ...) LISTIFY_17(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(17, __VA_ARGS__)
#define LISTIFY_19(F, SEP, ...) LISTIFY_18(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(18, __VA_ARGS__)
#define LISTIFY_20(F, SEP, ...) LISTIFY_19(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(19, __VA_ARGS__)

#define REVERSE_ARGS(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p)                               \
p, o, n, m, l, k, j, i, h, g, f, e, d, c, b, a

#define UTIL_CAT(a, b) UTIL_PRIMITIVE_CAT(a, b)
#define UTIL_PRIMITIVE_CAT(a, b) a##b

#define UTIL_INC(v) UTIL_PRIMITIVE_CAT(UTIL_INC_, v)
#define UTIL_INC_0 1
#define UTIL_INC_1 2
#define UTIL_INC_2 3
#define UTIL_INC_3 4
#define UTIL_INC_4 5
#define UTIL_INC_5 6
#define UTIL_INC_6 7
#define UTIL_INC_7 8
#define UTIL_INC_8 9

/* Array iteration macros */
#define ARRAY_FOR_EACH(array, idx) for (size_t idx = 0; (idx) < ARRAY_SIZE(array); (idx)++)

#define ARRAY_FOR_EACH_PTR(array, ptr)                                                             \
for (__typeof__(*(array)) *ptr = (array); ptr < &(array)[ARRAY_SIZE(array)]; ptr++)

/* BIT macro */
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

/* BIT_MASK */
#define BIT_MASK(n) (BIT(n) - 1)

#ifndef GENMASK
#define GENMASK(h, l)                                                                              \
(((~0UL) >> ((sizeof(unsigned long) * 8) - 1 - MAX(h, l))) & (~0UL << MIN(h, l)))
#endif

#ifndef FIELD_GET
#define FIELD_GET(mask, value) (((value) & (mask)) >> __builtin_ctzl(mask))
#endif

#ifndef FIELD_PREP
#define FIELD_PREP(mask, value) (((value) << __builtin_ctzl(mask)) & (mask))
#endif

#ifndef WRITE_BIT
#define WRITE_BIT(var, bit, set)                                                                   \
do                                                                                               \
  {                                                                                              \
    if (set)                                                                                     \
      {                                                                                          \
        (var) |= BIT(bit);                                                                       \
      }                                                                                          \
    else                                                                                         \
      {                                                                                          \
        (var) &= ~BIT(bit);                                                                      \
      }                                                                                          \
  }                                                                                              \
while (0)
#endif

/**
 * @brief Convert a hex string to binary data.
 *
 * @param hex    Hex string to convert
 * @param hexlen Length of hex string
 * @param buf    Output buffer
 * @param buflen Size of output buffer
 * @return Number of bytes written to buf, or 0 on error
 */

static inline size_t hex2bin(const char *hex,
                             size_t hexlen, uint8_t *buf, size_t buflen)
{
  size_t i;

  if ((hexlen % 2) != 0 || (hexlen / 2) > buflen)
    {
      return 0;
    }

  for (i = 0; i < hexlen / 2; i++)
    {
      char byte[3] =
        {
            hex[i * 2], hex[i * 2 + 1], '\0'
        };
      unsigned int val;

      if (sscanf(byte, "%02x", &val) != 1)
        {
          return 0;
        }

      buf[i] = (uint8_t)val;
    }

  return hexlen / 2;
}

static inline bool util_memeq(const void *a, const void *b, size_t len)
{
  return memcmp(a, b, len) == 0;
}

static inline bool util_eq(const void *a,
                           size_t a_len, const void *b, size_t b_len)
{
  return a_len == b_len && memcmp(a, b, a_len) == 0;
}

static inline void mem_xor_n(uint8_t *dst,
                             const uint8_t *src1, const uint8_t *src2,
                             size_t len)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      dst[i] = src1[i] ^ src2[i];
    }
}

static inline void mem_xor_128(uint8_t dst[16],
                               const uint8_t src1[16],
                                   const uint8_t src2[16])
{
  mem_xor_n(dst, src1, src2, 16);
}

static inline size_t sys_count_bits(const void *mem, size_t size)
{
  const uint8_t *bytes = mem;
  size_t count = 0;
  size_t i;

  for (i = 0; i < size; i++)
    {
      count += __builtin_popcount(bytes[i]);
    }

  return count;
}

/* u8_to_dec - convert uint8_t to decimal string */

static inline uint8_t u8_to_dec(char *buf, uint8_t buflen, uint8_t val)
{
  uint8_t pos = 0;
  if (val >= 100 && pos < buflen)
    buf[pos++] = '0' + val / 100;
  if (val >= 10 && pos < buflen)
    buf[pos++] = '0' + (val / 10) % 10;
  if (pos < buflen)
    buf[pos++] = '0' + val % 10;
  if (pos < buflen)
    buf[pos] = '\0';
  return pos;
}

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_UTIL_H */
