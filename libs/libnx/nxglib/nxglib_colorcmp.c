/****************************************************************************
 * libs/libnx/nxglib/nxglib_colorcmp.c
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

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_colorcmp
 *
 * Description:
 *   This is essentially memcmp for colors.  This does very little for us
 *   other than hide all of the conditional compilation for planar colors
 *   in one place.
 *
 ****************************************************************************/

bool nxgl_colorcmp(const nxgl_mxpixel_t color1[CONFIG_NX_NPLANES],
                   const nxgl_mxpixel_t color2[CONFIG_NX_NPLANES])
{
  int i;

  for (i = 0; i < CONFIG_NX_NPLANES; i++)
    {
      if (color1[i] != color2[i])
        {
          return false;
        }
    }

  return true;
}
