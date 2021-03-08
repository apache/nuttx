/****************************************************************************
 * libs/libnx/nxglib/nxglib_rectoverlap.c
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

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_rectoverlap
 *
 * Description:
 *   Return true if the two rectangles overlap
 *
 ****************************************************************************/

bool nxgl_rectoverlap(FAR struct nxgl_rect_s *rect1,
                      FAR struct nxgl_rect_s *rect2)
{
  /* The neither is wholly above, below, right, or left of the other, then
   * the two rectangles overlap in some fashion.
   */

  return (rect1->pt1.x <= rect2->pt2.x) &&  /* false: rect1 is wholly to the right */
         (rect2->pt1.x <= rect1->pt2.x) &&  /* false: rect2 is wholly to the right */
         (rect1->pt1.y <= rect2->pt2.y) &&  /* false: rect1 is wholly below rect2 */
         (rect2->pt1.y <= rect1->pt2.y);    /* false: rect2 is wholly below rect1 */
}
