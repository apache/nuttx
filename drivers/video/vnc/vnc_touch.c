/****************************************************************************
 * drivers/video/vnc/vnc_touch.c
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

#include <assert.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/mouse.h>

#include "vnc_server.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_touch_register
 ****************************************************************************/

int vnc_touch_register(FAR const char *devpath,
                       FAR struct vnc_session_s *session)
{
  int ret;

  ret = touch_register(&session->touch, devpath, 1);

  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: vnc_touch_register
 ****************************************************************************/

void vnc_touch_unregister(FAR struct vnc_session_s *session,
                          FAR const char *devpath)
{
  touch_unregister(&session->touch, devpath);
}

/****************************************************************************
 * Name: vnc_touch_event
 ****************************************************************************/

int vnc_touch_event(FAR void *arg, int16_t x, int16_t y, uint8_t buttons)
{
  struct touch_sample_s sample;
  FAR struct vnc_session_s *session = arg;

  DEBUGASSERT(session);

  sample.npoints = 1;
  sample.point[0].x = x;
  sample.point[0].y = y;

  sample.point[0].flags = TOUCH_ID_VALID | TOUCH_POS_VALID;

  if ((buttons & MOUSE_BUTTON_1) != 0)
    {
      sample.point[0].flags |= TOUCH_DOWN;
    }
  else
    {
      sample.point[0].flags |= TOUCH_UP;
    }

  touch_event(session->touch.priv, &sample);
  return OK;
}

