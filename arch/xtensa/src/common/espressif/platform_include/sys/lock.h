/****************************************************************************
 * arch/xtensa/src/common/espressif/platform_include/sys/lock.h
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

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include_next <sys/lock.h>

#ifdef _RETARGETABLE_LOCKING

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Actual platfrom-specific definition of struct __lock.
 * The size here should be sufficient for a NuttX mutex and recursive mutex.
 * This is checked by a static assertion in <chip>_libc_stubs.c
 */

struct __lock
{
  int reserved[4];
};

typedef _LOCK_T _lock_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: _lock_init
 *
 * Description:
 *   Allocate lock related resources.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_init(_lock_t *plock);

/****************************************************************************
 * Name: _lock_init_recursive
 *
 * Description:
 *   Allocate recursive lock related resources.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_init_recursive(_lock_t *plock);

/****************************************************************************
 * Name: _lock_close
 *
 * Description:
 *   Free lock related resources.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_close(_lock_t *plock);

/****************************************************************************
 * Name: _lock_close_recursive
 *
 * Description:
 *   Free recursive lock related resources.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_close_recursive(_lock_t *plock);

/****************************************************************************
 * Name: _lock_acquire
 *
 * Description:
 *   Acquire lock immediately after the lock object is available.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_acquire(_lock_t *plock);

/****************************************************************************
 * Name: _lock_acquire_recursive
 *
 * Description:
 *   Acquire recursive lock immediately after the lock object is available.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_acquire_recursive(_lock_t *plock);

/****************************************************************************
 * Name: _lock_try_acquire
 *
 * Description:
 *   Acquire lock if the lock object is available.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   Zero for success and non-zero to indicate that the lock cannot be
 *   acquired
 *
 ****************************************************************************/

int _lock_try_acquire(_lock_t *plock);

/****************************************************************************
 * Name: _lock_try_acquire_recursive
 *
 * Description:
 *   Acquire recursive lock if the lock object is available.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   Zero for success and non-zero to indicate that the lock cannot be
 *   acquired
 *
 ****************************************************************************/

int _lock_try_acquire_recursive(_lock_t *plock);

/****************************************************************************
 * Name: _lock_release
 *
 * Description:
 *   Relinquish the lock ownership.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_release(_lock_t *plock);

/****************************************************************************
 * Name: _lock_release_recursive
 *
 * Description:
 *   Relinquish the recursive lock ownership.
 *
 * Input Parameters:
 *   plock - pointer to user defined lock object
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void _lock_release_recursive(_lock_t *plock);

#endif // _RETARGETABLE_LOCKING
