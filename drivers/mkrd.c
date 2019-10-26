/****************************************************************************
 * drivers/mkrd.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
      ferr("ERROR: kmm_malloc() failed: %d\n", ret);
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
      free(buffer);
    }

  return ret;
}

#endif /* CONFIG_DRVR_MKRD */
