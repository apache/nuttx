/****************************************************************************
 * drivers/misc/mkrd.c
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

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/drivers/ramdisk.h>

#ifdef CONFIG_DRVR_MKRD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkrd
 *
 * Description:
 *   This is a wrapper function around ramdisk_register.  It combines the
 *   necessary operations to create a RAM disk into a single callable
 *   function.  Memory for the RAM disk is allocated, appropriated, from
 *   the kernel heap (in build modes where there is a distinct kernel heap).
 *
 * Input Parameters:
 *   minor:         Selects suffix of device named /dev/ramN, N={1,2,3...}
 *   nsectors:      Number of sectors on device
 *   sectize:       The size of one sector
 *   rdflags:       See RDFLAG_* definitions.  Typically
 *                  RDFLAG_WRENABLED | RDFLAG_FUNLINK
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mkrd(int minor, uint32_t nsectors, uint16_t sectsize, uint8_t rdflags)
{
  FAR uint8_t *buffer;
  int ret;

  /* Allocate the memory backing up the ramdisk from the kernel heap */

  buffer = (FAR uint8_t *)kmm_malloc(sectsize * nsectors);
  if (buffer == NULL)
    {
      ferr("ERROR: kmm_malloc() failed, enable DEBUG_MM for more info!\n");
      return -ENOMEM;
    }

#ifdef CONFIG_DEBUG_INFO
  memset(buffer, 0, sectsize * nsectors);
#endif
  finfo("RAMDISK at %p\n", buffer);

  /* Then register the ramdisk */

  ret = ramdisk_register(minor, buffer, nsectors, sectsize, rdflags);
  if (ret < 0)
    {
      ferr("ERROR: ramdisk_register() failed: %d\n", ret);
      kmm_free(buffer);
    }

  return ret;
}

#endif /* CONFIG_DRVR_MKRD */
