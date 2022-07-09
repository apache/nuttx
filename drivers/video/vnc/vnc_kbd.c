/****************************************************************************
 * drivers/video/vnc/vnc_kbd.c
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

#include "vnc_server.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_kbd_register
 ****************************************************************************/

int vnc_kbd_register(FAR const char *devpath,
                     FAR struct vnc_session_s *session)
{
  return keyboard_register(&session->kbd, devpath, 1);
}

/****************************************************************************
 * Name: vnc_kbd_register
 ****************************************************************************/

void vnc_kbd_unregister(FAR struct vnc_session_s *session,
                        FAR const char *devpath)
{
  keyboard_unregister(&session->kbd, devpath);
}

/****************************************************************************
 * Name: vnc_kbd_event
 ****************************************************************************/

int vnc_kbd_event(FAR void *arg, uint8_t pressed, FAR const uint8_t *keycode)
{
  FAR struct vnc_session_s *session = arg;
  keyboard_event(&session->kbd, rfb_getbe32(keycode),
                 pressed ? KEYBOARD_PRESS : KEYBOARD_RELEASE);
  return OK;
}
