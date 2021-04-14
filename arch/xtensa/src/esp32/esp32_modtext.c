/****************************************************************************
 * arch/xtensa/src/esp32/esp32_modtext.c
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
#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/mm/mm.h>

#include <sys/types.h>
#include <debug.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _smodtext;
extern uint32_t _emodtext;

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct mm_heap_s g_module_text;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_module_text_init
 *
 * Description:
 *   Initialize the module text allocator
 *
 ****************************************************************************/

void up_module_text_init()
{
  mm_initialize(&g_module_text, &_smodtext, &_emodtext - &_smodtext);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  static struct procfs_meminfo_entry_s g_modtext_procfs;

  g_modtext_procfs.name = "modtext";
  g_modtext_procfs.mallinfo = (void *)mm_mallinfo;
  g_modtext_procfs.user_data = &g_module_text;
  procfs_register_meminfo(&g_modtext_procfs);
#endif
}

/****************************************************************************
 * Name: up_module_text_memalign
 *
 * Description:
 *   Allocate memory for module text with the specified alignment.
 *
 ****************************************************************************/

FAR void *up_module_text_memalign(size_t align, size_t size)
{
  return mm_memalign(&g_module_text, align, size);
}

/****************************************************************************
 * Name: up_module_text_free
 *
 * Description:
 *   Free memory for module text.
 *
 ****************************************************************************/

void up_module_text_free(FAR void *p)
{
  return mm_free(&g_module_text, p);
}
