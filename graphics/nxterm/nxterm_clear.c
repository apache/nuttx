/****************************************************************************
 * graphics/nxterm/nxterm_clear.c
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

#include <debug.h>

#include <nuttx/nx/nxglib.h>

#include "nxterm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_clear
 *
 * Description:
 *   Clear the display.
 *
 ****************************************************************************/

void nxterm_clear(FAR struct nxterm_state_s *priv)
{
  struct nxgl_rect_s rect;
  int ret;

  rect.pt1.x = 0;
  rect.pt1.y = 0;
  rect.pt2.x = priv->wndo.wsize.w - 1;
  rect.pt2.y = priv->wndo.wsize.h - 1;

  ret = priv->ops->fill(priv, &rect, priv->wndo.wcolor);
  if (ret < 0)
    {
      gerr("ERROR: Fill failed: %d\n", get_errno());
    }
}
