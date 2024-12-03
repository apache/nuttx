/****************************************************************************
 * include/nuttx/coredump.h
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

#ifndef __INCLUDE_NUTTX_COREDUMP_H
#define __INCLUDE_NUTTX_COREDUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/utsname.h>
#include <unistd.h>

#include <nuttx/streams.h>
#include <nuttx/memoryregion.h>

#ifdef CONFIG_ARM_COREDUMP_REGION
#  include <nuttx/elf.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COREDUMP_MAGIC    0x434f5245

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Coredump information for block header */

struct coredump_info_s
{
  uint32_t        magic;
  struct utsname  name;
  struct timespec time;
  size_t          size;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: coredump_set_memory_region
 *
 * Description:
 *   Set do coredump memory region.
 *
 ****************************************************************************/

int coredump_set_memory_region(FAR const struct memory_region_s *region);

/****************************************************************************
 * Name: coredump_add_memory_region
 *
 * Description:
 *   Use coredump to dump the memory of the specified area.
 *
 ****************************************************************************/

int coredump_add_memory_region(FAR const void *ptr, size_t size,
                               uint32_t flags);

/****************************************************************************
 * Name: coredump
 *
 * Description:
 *   This function for generating core dump stream.
 *
 ****************************************************************************/

int coredump(FAR const struct memory_region_s *regions,
             FAR struct lib_outstream_s *stream,
             pid_t pid);

#endif /* __INCLUDE_NUTTX_COREDUMP_H */
