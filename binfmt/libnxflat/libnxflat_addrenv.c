/****************************************************************************
 * binfmt/libnxflat/libnxflat_addrenv.c
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

#include <string.h>
#include <sys/param.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "libnxflat.h"

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_addrenv_alloc
 *
 * Description:
 *   Allocate data memory for the NXFLAT image. If CONFIG_ARCH_ADDRENV=n,
 *   memory will be allocated using kmm_zalloc().  If CONFIG_ARCH_ADDRENV-y,
 *   then memory will be allocated using up_addrenv_create().
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   envsize - The size (in bytes) of the address environment needed for the
 *     ELF image.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nxflat_addrenv_alloc(FAR struct nxflat_loadinfo_s *loadinfo,
                         size_t envsize)
{
  FAR struct dspace_s *dspace;
#ifdef CONFIG_ARCH_ADDRENV
  FAR void *vdata;
  save_addrenv_t oldenv;
  size_t heapsize;
  int ret;
#endif

  DEBUGASSERT(!loadinfo->dspace);

  /* Allocate the struct dspace_s container for the D-Space allocation */

  dspace = (FAR struct dspace_s *)kmm_malloc(sizeof(struct dspace_s));
  if (dspace == 0)
    {
      berr("ERROR: Failed to allocate DSpace\n");
      return -ENOMEM;
    }

#ifdef CONFIG_ARCH_ADDRENV
  /* Determine the heapsize to allocate. If there is no dynamic stack then
   * heapsize must at least as big as the fixed stack size since the stack
   * will be allocated from the heap in that case.
   */

#ifdef CONFIG_ARCH_STACK_DYNAMIC
  heapsize = ARCH_HEAP_SIZE;
#else
  heapsize = MIN(loadinfo->stacksize, ARCH_HEAP_SIZE);
#endif

  /* Create a D-Space address environment for the new NXFLAT task */

  ret = up_addrenv_create(0, envsize, heapsize, &loadinfo->addrenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_create failed: %d\n", ret);
      goto errout_with_dspace;
    }

  /* Get the virtual address associated with the start of the address
   * environment.  This is the base address that we will need to use to
   * access the D-Space region (but only if the address environment has been
   * selected.
   */

  ret = up_addrenv_vdata(&loadinfo->addrenv, 0, &vdata);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_vdata failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Clear all of the allocated D-Space memory.  We have to temporarily
   * selected the D-Space address environment to do this.
   */

  ret = up_addrenv_select(loadinfo->addrenv, &oldenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_select failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  memset(vdata, 0, envsize);

  ret = up_addrenv_restore(oldenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_restore failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Success... save the fruits of our labor */

  loadinfo->dspace = dspace;
  dspace->crefs    = 1;
  dspace->region   = (FAR uint8_t *)vdata;
  return OK;

errout_with_addrenv:
  up_addrenv_destroy(&loadinfo->addrenv);
  loadinfo->addrenv = 0;

errout_with_dspace:
  kmm_free(dspace);
  return ret;
#else
  /* Allocate (and zero) memory to hold the ELF image */

  dspace->region = (FAR uint8_t *)kumm_zalloc(envsize);
  if (!dspace->region)
    {
      kmm_free(dspace);
      return -ENOMEM;
    }

  loadinfo->dspace = dspace;
  dspace->crefs    = 1;
  return OK;
#endif
}

/****************************************************************************
 * Name: nxflat_addrenv_free
 *
 * Description:
 *   Release the address environment previously created by
 *   nxflat_addrenv_create().  This function  is called only under certain
 *   error conditions after the module has been loaded but not yet
 *   started. After the module has been started, the address environment
 *   will automatically be freed when the module exits.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nxflat_addrenv_free(FAR struct nxflat_loadinfo_s *loadinfo)
{
  FAR struct dspace_s *dspace;
#ifdef CONFIG_ARCH_ADDRENV
  int ret;
#endif

  DEBUGASSERT(loadinfo);
  dspace = loadinfo->dspace;

  if (dspace)
    {
#ifdef CONFIG_ARCH_ADDRENV
      /* Destroy the address environment */

      ret = up_addrenv_destroy(loadinfo->addrenv);
      if (ret < 0)
        {
          berr("ERROR: up_addrenv_destroy failed: %d\n", ret);
        }

      loadinfo->addrenv = 0;
#else
      /* Free the allocated D-Space region */

      if (dspace->region)
        {
          kumm_free(dspace->region);
        }
#endif

      /* Now destroy the D-Space container */

      DEBUGASSERT(dspace->crefs == 1);
      kmm_free(dspace);
      loadinfo->dspace = NULL;
    }
}
