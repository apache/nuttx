/****************************************************************************
 * drivers/devicetree/fdt.c
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

#include <stddef.h>
#include <endian.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/compiler.h>
#include <nuttx/fdt.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Location of the fdt data for this system. */

static FAR const char *g_fdt_base = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_register
 *
 * Description:
 *   Store the pointer to the flattened device tree and verify that it at
 *   least appears to be valid. This function will not fully parse the FDT.
 *
 * Return:
 *   Return -EINVAL if the fdt header does not have the expected magic value.
 *   otherwise return OK. If OK is not returned the existing entry for FDT
 *   is not modified.
 *
 ****************************************************************************/

int fdt_register(FAR const char *fdt_base)
{
  struct fdt_header_s *fdt_header;

  DEBUGASSERT(fdt_base);

  fdt_header = (struct fdt_header_s *)fdt_base;
  if (fdt_header->magic != be32toh(FDT_MAGIC))
    {
      return -EINVAL; /* Bad magic byte read */
    }

  g_fdt_base = fdt_base;
  return OK;
}

/****************************************************************************
 * Name: fdt_get
 *
 * Description:
 *   Return the pointer to a raw FDT. NULL is returned if no FDT has been
 *   loaded.
 *
 ****************************************************************************/

FAR const char *fdt_get(void)
{
  return g_fdt_base;
}
