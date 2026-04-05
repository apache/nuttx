/****************************************************************************
 * fs/vfs/fs_profile.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/atomic.h>
#include "vfs.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct vfs_profile_s g_vfs_profile;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void vfs_profile_start(FAR clock_t *start)
{
  *start = perf_gettime();
}

void vfs_profile_stop(FAR clock_t *start, FAR uint64_t *total,
                      FAR uint32_t *count)
{
  clock_t stop = perf_gettime();
  clock_t delta = stop - *start;

  atomic64_fetch_add((FAR atomic64_t *)total, delta);
  atomic_fetch_add((FAR atomic_t *)count, 1);
}
