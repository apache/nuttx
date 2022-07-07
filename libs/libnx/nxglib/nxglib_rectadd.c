/****************************************************************************
 * libs/libnx/nxglib/nxglib_rectadd.c
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
 * Name: nxgl_rectadd
 *
 * Description:
 *   Return the rectangle that contains exactly two other rectangles.
 *
 ****************************************************************************/

void nxgl_rectadd(FAR struct nxgl_rect_s *dest,
                  FAR const struct nxgl_rect_s *src1,
                  FAR const struct nxgl_rect_s *src2)
{
  dest->pt1.x = ngl_min(src1->pt1.x, src2->pt1.x);
  dest->pt1.y = ngl_min(src1->pt1.y, src2->pt1.y);
  dest->pt2.x = ngl_max(src1->pt2.x, src2->pt2.x);
  dest->pt2.y = ngl_max(src1->pt2.y, src2->pt2.y);
}
