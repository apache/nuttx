/****************************************************************************
 * mm/kmm_heap/kmm_mallinfo.c
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

#include <malloc.h>

#include <nuttx/mm/mm.h>

#ifdef CONFIG_MM_KERNEL_HEAP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_mallinfo
 *
 * Description:
 *   kmm_mallinfo returns a copy of updated current heap information for the
 *   kernel heap
 *
 ****************************************************************************/

struct mallinfo kmm_mallinfo(void)
{
  struct mallinfo info;
  mm_mallinfo(g_kmmheap, &info);
  return info;
}

/****************************************************************************
 * Name: kmm_mallinfo_task
 *
 * Description:
 *   kmm_mallinfo_task returns a copy of updated current heap information of
 *   task with specified pid for the user heap.
 *
 ****************************************************************************/

struct mallinfo_task kmm_mallinfo_task(pid_t pid)
{
  struct mallinfo_task info;

  info.pid = pid;
  mm_mallinfo_task(g_kmmheap, &info);
  return info;
}
#endif /* CONFIG_MM_KERNEL_HEAP */
