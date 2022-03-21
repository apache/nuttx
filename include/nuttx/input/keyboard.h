/****************************************************************************
 * include/nuttx/input/keyboard.h
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

#ifndef __INCLUDE_NUTTX_INPUT_KEYBOARD_H
#define __INCLUDE_NUTTX_INPUT_KEYBOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/input/x11_keysym.h>
#include <nuttx/input/x11_xf86keysym.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KEYBOARD_PRESS   0 /* Key press event */
#define KEYBOARD_RELEASE 1 /* Key release event */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct keyboard_event_s
{
  uint32_t type;
  uint32_t code;
};

struct keyboard_lowerhalf_s
{
  FAR void *priv;
  CODE int (*open)(FAR struct keyboard_lowerhalf_s *lower);
  CODE int (*close)(FAR struct keyboard_lowerhalf_s *lower);
  CODE ssize_t (*write)(FAR struct keyboard_lowerhalf_s *lower,
                        FAR const char *buffer, size_t buflen);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: keyboard_event
 ****************************************************************************/

void keyboard_event(FAR struct keyboard_lowerhalf_s *lower, uint32_t keycode,
                    uint32_t type);

/****************************************************************************
 * Name: keyboard_register
 ****************************************************************************/

int keyboard_register(FAR struct keyboard_lowerhalf_s *lower,
                      FAR const char *path, uint8_t nums);

/****************************************************************************
 * Name: keyboard_register
 ****************************************************************************/

int keyboard_unregister(FAR struct keyboard_lowerhalf_s *lower,
                        FAR const char *path);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_KEYBOARD_H */
