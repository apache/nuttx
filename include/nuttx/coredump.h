/****************************************************************************
 * include/nuttx/coredump.h
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

#include <nuttx/memoryregion.h>
#include <unistd.h>

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

int coredump_set_memory_region(FAR struct memory_region_s *region);

/****************************************************************************
 * Name: coredump_initialize
 *
 * Description:
 *   Initialize the coredump facility.  Called once and only from
 *   nx_start_application.
 *
 ****************************************************************************/

int coredump_initialize(void);

/****************************************************************************
 * Name: coredump_dump
 *
 * Description:
 *   Do coredump of the task specified by pid.
 *
 * Input Parameters:
 *   pid - The task/thread ID of the thread to dump
 *
 ****************************************************************************/

void coredump_dump(pid_t pid);

#endif /* __INCLUDE_NUTTX_COREDUMP_H */
