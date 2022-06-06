/****************************************************************************
 * libs/libc/stdio/lib_libfilesem.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

#include "libc.h"

#ifndef CONFIG_STDIO_DISABLE_BUFFERING

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * lib_sem_initialize
 ****************************************************************************/

void lib_sem_initialize(FAR struct file_struct *stream)
{
  /* Initialize the LIB semaphore to one (to support one-at-a-time access
   * to private data sets.
   */

  _RMUTEX_INIT(&stream->fs_lock);
}

/****************************************************************************
 * lib_take_semaphore
 ****************************************************************************/

void lib_take_semaphore(FAR struct file_struct *stream)
{
  _RMUTEX_LOCK(&stream->fs_lock);
}

/****************************************************************************
 * lib_give_semaphore
 ****************************************************************************/

void lib_give_semaphore(FAR struct file_struct *stream)
{
  _RMUTEX_UNLOCK(&stream->fs_lock);
}

#endif /* CONFIG_STDIO_DISABLE_BUFFERING */
