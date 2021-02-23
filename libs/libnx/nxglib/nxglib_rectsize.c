/****************************************************************************
 * libs/libnx/nxglib/nxglib_rectsize.c
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
 * Name: nxgl_rectsize
 *
 * Description:
 *   Return the size of the specified rectangle.
 *
 ****************************************************************************/

void nxgl_rectsize(FAR struct nxgl_size_s *size,
                   FAR const struct nxgl_rect_s *rect)
{
  size->w = rect->pt2.x - rect->pt1.x + 1;
  size->h = rect->pt2.y - rect->pt1.y + 1;
}
