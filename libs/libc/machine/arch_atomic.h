/****************************************************************************
 * libs/libc/machine/arch_atomic.h
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

#ifndef __LIBS_LIBC_MACHINE_ARCH_ATOMIC_H
#define __LIBS_LIBC_MACHINE_ARCH_ATOMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/macro.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STORE(fn, n, type)                                            \
                                                                      \
  void weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value, int memorder)     \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
                                                                      \
    *(FAR type *)ptr = value;                                         \
                                                                      \
    atomic_unlock(irqstate);                                          \
  }

#define LOAD(fn, n, type)                                             \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR const volatile void *ptr, \
                                        int memorder)                 \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
                                                                      \
    type ret = *(FAR type *)ptr;                                      \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define EXCHANGE(fn, n, type)                                         \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value, int memorder)     \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    type ret = *tmp;                                                  \
    *tmp = value;                                                     \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define CMP_EXCHANGE(fn, n, type)                                     \
                                                                      \
  bool weak_function CONCATENATE(fn, n)(FAR volatile void *mem,       \
                                        FAR volatile void *expect,    \
                                        type desired, bool weak,      \
                                        int success, int failure)     \
  {                                                                   \
    bool ret = false;                                                 \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmpmem = (FAR type *)mem;                               \
    FAR type *tmpexp = (FAR type *)expect;                            \
                                                                      \
    if (*tmpmem == *tmpexp)                                           \
      {                                                               \
        ret = true;                                                   \
        *tmpmem = desired;                                            \
      }                                                               \
    else                                                              \
      {                                                               \
        *tmpexp = *tmpmem;                                            \
      }                                                               \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define FLAG_TEST_AND_SET(fn, n, type)                                \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        int memorder)                 \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    *(FAR type *)ptr = 1;                                             \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define FETCH_ADD(fn, n, type)                                        \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value, int memorder)     \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    *tmp = *tmp + value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define FETCH_SUB(fn, n, type)                                        \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value, int memorder)     \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    *tmp = *tmp - value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define FETCH_AND(fn, n, type)                                        \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value, int memorder)     \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    *tmp = *tmp & value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define FETCH_OR(fn, n, type)                                         \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value, int memorder)     \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    *tmp = *tmp | value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define FETCH_XOR(fn, n, type)                                        \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value, int memorder)     \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    *tmp = *tmp ^ value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_ADD_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    *tmp = *tmp + value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return *tmp;                                                      \
  }

#define SYNC_SUB_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    *tmp = *tmp - value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return *tmp;                                                      \
  }

#define SYNC_OR_FETCH(fn, n, type)                                    \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    *tmp = *tmp | value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return *tmp;                                                      \
  }

#define SYNC_AND_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    *tmp = *tmp & value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return *tmp;                                                      \
  }

#define SYNC_XOR_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    *tmp = *tmp ^ value;                                              \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return *tmp;                                                      \
  }

#define SYNC_NAND_FETCH(fn, n, type)                                  \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    *tmp = ~(*tmp & value);                                           \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return *tmp;                                                      \
  }

#define SYNC_BOOL_CMP_SWAP(fn, n, type)                               \
                                                                      \
  bool weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type oldvalue,                \
                                        type newvalue)                \
  {                                                                   \
    bool ret = false;                                                 \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    if (*tmp == oldvalue)                                             \
      {                                                               \
        ret = true;                                                   \
        *tmp = newvalue;                                              \
      }                                                               \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_VAL_CMP_SWAP(fn, n, type)                                \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type oldvalue,                \
                                        type newvalue)                \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    if (*tmp == oldvalue)                                             \
      {                                                               \
        *tmp = newvalue;                                              \
      }                                                               \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#endif /* __LIBS_LIBC_MACHINE_ARCH_ATOMIC_H */
