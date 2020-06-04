/****************************************************************************
 * graphics/nxbe/nxbe_notify_rectangle.c
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

#include "nxbe.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_notify_rectangle
 *
 * Description:
 *   When CONFIG_NX_UPDATE=y, then the graphics system will callout to
 *   inform some external module that the display has been updated.  This
 *   would be useful in a couple for cases.
 *
 *   - When a serial LCD is used, but a framebuffer is used to access the
 *     LCD.  In this case, the update callout can be used to refresh the
 *     affected region of the display.
 *
 *   - When VNC is enabled.  This is case, this callout is necessary to
 *     update the remote frame buffer to match the local framebuffer.
 *
 *   When this feature is enabled, some external logic must provide this
 *   interface.  This is the function that will handle the notification.  It
 *   receives the rectangular region that was updated on the provided plane.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_UPDATE
void nxbe_notify_rectangle(FAR NX_DRIVERTYPE *dev,
                           FAR const struct nxgl_rect_s *rect)
{
  struct fb_area_s area;

  nxgl_rect2area(&area, rect);
  dev->updatearea(dev, &area);
}
#endif
