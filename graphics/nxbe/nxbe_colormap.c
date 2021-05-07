/****************************************************************************
 * graphics/nxbe/nxbe_colormap.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "nxbe.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_colormap
 *
 * Description:
 *   Set the hardware color map to the palette expected by NX
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
int nxbe_colormap(FAR NX_DRIVERTYPE *dev)
{
  struct fb_cmap_s cmap;
  FAR uint8_t *alloc;
  FAR uint8_t *red;
  FAR uint8_t *green;
  FAR uint8_t *blue;
  uint8_t rval;
  uint8_t gval;
  int     size;
  int     ndx;
  int     ret;
  int     i;
  int     j;
  int     k;

  /* Allocate the color map tables in one allocation:
   *
   *   size = 3 colors x CONFIG_NX_COLORS each x 8-bits per color
   */

  size  = 3 * CONFIG_NX_NCOLORS * sizeof(uint8_t);
  alloc = (FAR uint8_t *)kmm_malloc(size);
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  memset(alloc, 0xff, size);

  /* Then get pointers to each color table */

  red   = alloc;
  green = &alloc[CONFIG_NX_NCOLORS];
  blue  = &alloc[2*CONFIG_NX_NCOLORS];

  /* Initialize the color map tables. 6*6*6 = 216, the rest
   * are (0xff, 0xfff 0xff)
   */

  ndx = 0;
  for (i = 0; i < 6; i++)
    {
      rval = (i * (CONFIG_NX_NCOLORS - 1) / 5) << 8;
      for (j = 0; j < 6; j++)
        {
          gval = (j * (CONFIG_NX_NCOLORS - 1) / 5) << 8;
          for (k = 0; k < 6; k++)
            {
              red[ndx]   = rval;
              green[ndx] = gval;
              blue[ndx]  = k * (CONFIG_NX_NCOLORS - 1) / 5;
              ndx++;
            }
        }
    }

  /* Now configure the cmap structure */

  cmap.first  = 0;
  cmap.len    = CONFIG_NX_NCOLORS;
  cmap.red    = red;
  cmap.green  = green;
  cmap.blue   = blue;
#ifdef CONFIG_FB_TRANSPARENCY
  cmap.transp = NULL;
#endif

  /* Then set the color map */

  ret = dev->putcmap(dev, &cmap);

  kmm_free(alloc);
  return ret;
}
#endif
