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

#include <elf.h>
#include <unistd.h>
#include <sys/utsname.h>

#include <nuttx/nuttx.h>
#include <nuttx/streams.h>
#include <nuttx/memoryregion.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COREDUMP_MAGIC          0x434f5245
#define COREDUMP_INFONAME_SIZE  ALIGN_UP(CONFIG_TASK_NAME_SIZE, 8)

#ifndef CONFIG_COREDUMP_MEMORY_REGION_MAX
#  define CONFIG_COREDUMP_MEMORY_REGION_MAX 8
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Runtime coredump mode */

enum coredump_mode_e
{
  COREDUMP_MODE_DISABLED = 0,
  COREDUMP_MODE_CURRENT_TASK,
  COREDUMP_MODE_ALL_TASKS,
};

struct coredump_config_s
{
  enum coredump_mode_e mode;
  struct memory_region_s regions[CONFIG_COREDUMP_MEMORY_REGION_MAX + 1];
};

/* Coredump information for block header */

struct coredump_info_s
{
  struct utsname  name;
  struct timespec time;
  size_t          size;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: coredump_get_config
 *
 * Description:
 *   Copy the runtime coredump configuration.
 *
 ****************************************************************************/

void coredump_get_config(FAR struct coredump_config_s *config);

/****************************************************************************
 * Name: coredump_set_mode
 *
 * Description:
 *   Set the runtime coredump mode.
 *
 ****************************************************************************/

int coredump_set_mode(enum coredump_mode_e mode);

/****************************************************************************
 * Name: coredump_clear_memory_regions
 *
 * Description:
 *   Clear all runtime coredump memory regions.
 *
 ****************************************************************************/

void coredump_clear_memory_regions(void);

/****************************************************************************
 * Name: coredump_set_memory_regions
 *
 * Description:
 *   Replace all runtime coredump memory regions.
 *
 ****************************************************************************/

int coredump_set_memory_regions(
  FAR const struct memory_region_s *regions, size_t count);

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
