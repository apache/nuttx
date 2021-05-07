/****************************************************************************
 * binfmt/libnxflat/libnxflat.h
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

#ifndef __BINFMT_LIBNXFLAT_LIBNXFLAT_H
#define __BINFMT_LIBNXFLAT_LIBNXFLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt/nxflat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
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
                         size_t envsize);

/****************************************************************************
 * Name: nxflat_addrenv_select
 *
 * Description:
 *   Temporarity select the task's address environment.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#  define nxflat_addrenv_select(l) up_addrenv_select(&(l)->addrenv, &(l)->oldenv)
#endif

/****************************************************************************
 * Name: nxflat_addrenv_restore
 *
 * Description:
 *   Restore the address environment before nxflat_addrenv_select() was
 *   called..
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#  define nxflat_addrenv_restore(l) up_addrenv_restore(&(l)->oldenv)
#endif

/****************************************************************************
 * Name: nxflat_addrenv_free
 *
 * Description:
 *   Release the address environment previously created by
 *   nxflat_addrenv_alloc().  This function  is called only under certain
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

void nxflat_addrenv_free(FAR struct nxflat_loadinfo_s *loadinfo);

#endif /* __BINFMT_LIBNXFLAT_LIBNXFLAT_H */
