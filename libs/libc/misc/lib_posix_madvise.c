/****************************************************************************
 * libs/libc/misc/lib_posix_madvise.c
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

#include <sys/mman.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_madvise
 *
 * Description:
 *   Advise the system about the expected use of a memory range.  NuttX
 *   has no other paging strategy to select, so the advice is ignored.
 *
 * Input Parameters:
 *   addr   - The start of the range
 *   len    - The length of the range
 *   advice - The advice (POSIX_MADV_xxx or MADV_xxx)
 *
 * Returned Value:
 *   Zero (always).
 *
 ****************************************************************************/

int posix_madvise(FAR void *addr, size_t len, int advice)
{
  return 0;
}
