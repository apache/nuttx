/****************************************************************************
 * binfmt/binfmt_debug.c
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

#include <nuttx/binfmt/binfmt.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/compiler.h>
#include "binfmt.h"

static mutex_t g_loadable_module_lock = NXMUTEX_INITIALIZER;
int num_modules = 0;

/**
 * GDB will place breakpoint into this function.
 * To prevent GCC from inlining or removing it we place noinline attribute
 * and inline assembler statement inside.
 */

static void noinline_function __loadable_module_register_code(void)
{
  /* empty implementation. */
}

/**
 * Call __loadable_module_register_code indirectly via global funtione ptr.
 * This gives the debugger an easy way to inject custom code to
 * handle the events.
 */

void (*__loadable_module_register_code_ptr)(void) =
    __loadable_module_register_code;

struct loadable_module_descriptor
    g_loadable_module_descriptor[CONFIG_LOADABLE_MODULE_DESCRIPTOR_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: create_loadable_module_internal
 *
 * Description:
 *   This function for creating loadable module internal.
 *
 ****************************************************************************/

static struct loadable_module_descriptor *
create_loadable_module_internal(FAR const char *symfile_name,
                                FAR void *text_addr, FAR void *data_addr)
{
  struct loadable_module_descriptor *ptr;
  nxmutex_lock(&g_loadable_module_lock);

  /* *
   * return NULL if same module name are already loaded in
   * g_loadable_module_descriptor array.
   */

  for (int i = 0; i < num_modules; i++)
    {
      if (strcmp(g_loadable_module_descriptor[i].symfile_name, symfile_name)
      == 0)
        {
          berr("ERROR: module with same name are already loaded in"
          "g_loadable_module_descriptor array: no need to be loaded again");
          nxmutex_unlock(&g_loadable_module_lock);
          return NULL;
        }
    }

  if (num_modules < CONFIG_LOADABLE_MODULE_DESCRIPTOR_COUNT)
    {
      ptr = &g_loadable_module_descriptor[num_modules++];
      ptr->symfile_name = strdup(symfile_name);
      ptr->text_addr = text_addr;
      ptr->data_addr = data_addr;
      ptr->action_flag = LOADABLE_MODULE_REGISTER_FN;
    }
  else
    {
      berr("ERROR: g_loadable_module_descriptor array size not enough: need "
         "resize array");
      nxmutex_unlock(&g_loadable_module_lock);
      return NULL;
    }

  (*__loadable_module_register_code_ptr)();
  nxmutex_unlock(&g_loadable_module_lock);

  return ptr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loadable_module_create
 *
 * Description:
 *   This function for creating load module.
 *
 ****************************************************************************/

bool loadable_module_create(FAR const char *symfile_name,
                            FAR void *text_addr, FAR void *data_addr)
{
  if (!(create_loadable_module_internal(symfile_name, text_addr, data_addr)))
    {
      berr("ERROR: loadable_module_create error: create loadable module"
           "failed");
      return false;
    }

  return true;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: destroy_loadable_module_internal
 *
 * Description:
 *   This function for destorying loadadble module internal.
 *
 ****************************************************************************/

static void destroy_loadable_module_internal(FAR const char *symfile_name)
{
  nxmutex_lock(&g_loadable_module_lock);
  struct loadable_module_descriptor *ptr;

  if (num_modules == 0 || symfile_name == NULL)
    {
      berr("ERROR: destroy loadable module error: num_modules is null or"
         "symfile_name is none");
      nxmutex_unlock(&g_loadable_module_lock);
      return;
    }

  for (int i = 0; i < num_modules; i++)
    {
      if (strcmp(g_loadable_module_descriptor[i].symfile_name, symfile_name)
      == 0)
        {
          ptr = &g_loadable_module_descriptor[i];
          break;
        }
        else
        {
          berr("ERROR: destroy loadable module error: no symfile_name of "
           "module found in g_loadable_module_descriptor array");
          nxmutex_unlock(&g_loadable_module_lock);
          return;
        }
    }

  ptr->action_flag = LOADABLE_MODULE_UNREGISTER_FN;

  (*__loadable_module_register_code_ptr)();

  if (ptr->symfile_name != NULL)
    {
      kmm_free((FAR void *)ptr->symfile_name);
      ptr->symfile_name = NULL;
    }

  if (symfile_name != NULL)
    {
      kmm_free((FAR void *)symfile_name);
      symfile_name = NULL;
    }

  num_modules--;
  nxmutex_unlock(&g_loadable_module_lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loadable_module_destroy
 *
 * Description:
 *   This function for destorying load module.
 *
 ****************************************************************************/

void loadable_module_destroy(FAR const char *symfile_name)
{
    destroy_loadable_module_internal(symfile_name);
}
