/****************************************************************************
 * binfmt/binfmt_initialize.c
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
#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/nxflat.h>
#include <nuttx/lib/builtin.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_initialize
 *
 * Description:
 *   initialize binfmt subsystem
 *
 ****************************************************************************/

void binfmt_initialize(void)
{
  int ret;

#ifdef CONFIG_FS_BINFS
  ret = builtin_initialize();
  if (ret < 0)
    {
      berr("ERROR: builtin_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ELF
  ret = elf_initialize();
  if (ret < 0)
    {
      berr("ERROR: elf_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NXFLAT
  ret = nxflat_initialize();
  if (ret < 0)
    {
      berr("ERROR: nxflat_initialize failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
}

#endif /* CONFIG_BINFMT_DISABLE */
