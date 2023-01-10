/****************************************************************************
 * fs/yaffs/yaffs_osglue.c
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

#include <debug.h>
#include <time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "yportenv.h"
#include "yaffs_trace.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

unsigned int yaffs_trace_mask =
             YAFFS_TRACE_MOUNT |
             YAFFS_TRACE_BAD_BLOCKS |
             YAFFS_TRACE_ALWAYS;

static mutex_t g_yaffs_lock = NXMUTEX_INITIALIZER;
static int g_yaffs_last_error;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void yaffsfs_Lock(void)
{
  nxmutex_lock(&g_yaffs_lock);
}

void yaffsfs_Unlock(void)
{
  nxmutex_unlock(&g_yaffs_lock);
}

u32 yaffsfs_CurrentTime(void)
{
  return time(NULL);
}

void yaffsfs_SetError(int err)
{
  g_yaffs_last_error = err;
}

int yaffsfs_GetLastError(void)
{
  return g_yaffs_last_error;
}

FAR void *yaffsfs_malloc(size_t size)
{
  return kmm_malloc(size);
}

void yaffsfs_free(FAR void *ptr)
{
  return kmm_free(ptr);
}

int yaffsfs_CheckMemRegion(FAR const void *addr, size_t size,
                           int write_request)
{
  return 0;
}

void yaffs_bug_fn(const char *file_name, int line_no)
{
  ferr("yaffs bug detected %s:%d\n", file_name, line_no);
}
