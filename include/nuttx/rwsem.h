/****************************************************************************
 * include/nuttx/rwsem.h
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

#ifndef __INCLUDE_NUTTX_RWSEM_H
#define __INCLUDE_NUTTX_RWSEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef struct
{
  spinlock_t protected;
  sem_t   waiting;
  int     waiter;
  int     writer;
  int     reader;
} rw_semaphore_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: down_read_trylock
 *
 * Description:
 *   Acquire a read lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 * Returned Value:
 *   Return 1 if successful, 0 if failed
 *
 ****************************************************************************/

int down_read_trylock(FAR rw_semaphore_t *rwsem);

/****************************************************************************
 * Name: down_read
 *
 * Description:
 *   Acquire a read lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void down_read(FAR rw_semaphore_t *rwsem);

/****************************************************************************
 * Name: up_read
 *
 * Description:
 *   Unlock a read lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void up_read(FAR rw_semaphore_t *rwsem);

/****************************************************************************
 * Name: down_write_trylock
 *
 * Description:
 *   Acquire a write lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 * Returned Value:
 *   Return 1 if successful, 0 if failed
 *
 ****************************************************************************/

int down_write_trylock(FAR rw_semaphore_t *rwsem);

/****************************************************************************
 * Name: down_write
 *
 * Description:
 *   Acquire a write lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void down_write(FAR rw_semaphore_t *rwsem);

/****************************************************************************
 * Name: up_write
 *
 * Description:
 *   Unlock a write lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void up_write(FAR rw_semaphore_t *rwsem);

/****************************************************************************
 * Name: init_rwsem
 *
 * Description:
 *   Initialize a read-write-lock object, setting its initial state.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy: Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int init_rwsem(FAR rw_semaphore_t *rwsem);

/****************************************************************************
 * Name: destroy_rwsem
 *
 * Description:
 *   Destroy a read-write-lock object, freeing any resources associated with
 *   it.
 *
 * Input Parameters:
 *   rwsem - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void destroy_rwsem(FAR rw_semaphore_t *rwsem);

#endif  /* __INCLUDE_NUTTX_RWSEM_H */
