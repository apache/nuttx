/****************************************************************************
 * libs/libnx/nxglib/nxglib_runoffset.c
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

#include <fixedmath.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_runoffset
 *
 * Description:
 *   Offset the run position by the specified (integer) dx, dy values.
 *
 ****************************************************************************/

void nxgl_runoffset(FAR struct nxgl_run_s *dest,
                    FAR const struct nxgl_run_s *src,
                    nxgl_coord_t dx, nxgl_coord_t dy)
{
  b16_t b16dx = itob16(dx);
  dest->x1    = src->x1 + b16dx;
  dest->x2    = src->x2 + b16dx;
  dest->y     = src->y  + dy;
}
