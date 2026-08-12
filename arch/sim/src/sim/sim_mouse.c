/****************************************************************************
 * arch/sim/src/sim/sim_mouse.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* A full X11 mouse emulation for the sim architecture, registering
 * /dev/mouseN via the standard mouse_lowerhalf_s interface (see
 * include/nuttx/input/mouse.h). Modeled closely on sim_touchscreen.c
 * (same registration shape, same X11-button-event source via
 * sim_x11eventloop.c), but reports which physical button changed
 * instead of folding every button into one "pen down" contact --
 * sim_touchscreen.c exists to emulate real touchscreen hardware, which
 * has no second or third button to report in the first place. Any
 * consumer that needs to tell a right-click from a left-click (e.g. a
 * GUI toolkit's context-menu handling) needs this driver, not the
 * touchscreen one.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/input/mouse.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEV_FORMAT   "/dev/mouse%d"
#define DEV_NAMELEN  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one simulated mouse driver
 * instance.
 */

struct sim_dev_s
{
  int eventloop;
  uint8_t minor;                     /* Minor device number */
  struct mouse_lowerhalf_s lower;    /* Mouse lowerhalf */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one simulated mouse is supported so the driver state structure
 * may as well be pre-allocated.
 */

static struct sim_dev_s g_simmouse;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_mouse_initialize
 *
 * Description:
 *   Configure the simulated mouse.  This will register the driver as
 *   /dev/mouseN where N is the minor device number
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sim_mouse_initialize(int minor)
{
  struct sim_dev_s *priv = (struct sim_dev_s *)&g_simmouse;
  char devname[DEV_NAMELEN];
  int ret;

  iinfo("minor: %d\n", minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(minor >= 0 && minor < 100);

  /* Initialize the mouse device driver instance */

  memset(priv, 0, sizeof(struct sim_dev_s));

  priv->minor = minor;

  /* Register the device as an input device */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = mouse_register(&priv->lower, devname, 1);
  if (ret < 0)
    {
      ierr("ERROR: mouse_register() failed: %d\n", ret);
      return ret;
    }

  /* Enable X11 event processing from the IDLE loop */

  priv->eventloop = 1;

  /* And return success */

  return OK;
}

/****************************************************************************
 * Name: sim_mouse_uninitialize
 *
 * Description:
 *   Uninitialized the simulated mouse
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Return OK if success or negative value of the error.
 *
 ****************************************************************************/

int sim_mouse_uninitialize(void)
{
  struct sim_dev_s *priv = (struct sim_dev_s *)&g_simmouse;
  char devname[DEV_NAMELEN];

  /* Stop the event loop (Hmm.. the caller must be sure that there are no
   * open references to the mouse driver.  This might better be done in
   * close() using a reference count).
   */

  priv->eventloop = 0;

  /* Un-register the device */

  snprintf(devname, sizeof(devname), DEV_FORMAT, priv->minor);
  iinfo("Un-registering %s\n", devname);

  mouse_unregister(&priv->lower, devname);

  return OK;
}

/****************************************************************************
 * Name: sim_mouseevent
 ****************************************************************************/

void sim_mouseevent(int x, int y, int buttons, int wheel)
{
  struct sim_dev_s *priv = (struct sim_dev_s *)&g_simmouse;
  struct mouse_report_s sample;

  if (priv->eventloop == 0)
    {
      return;
    }

  iinfo("x=%d y=%d buttons=%02x wheel=%d\n", x, y, buttons, wheel);

  memset(&sample, 0, sizeof(sample));
  sample.buttons  = (uint8_t)buttons;
  sample.x        = (int16_t)x;
  sample.y        = (int16_t)y;
  sample.wheel    = (int16_t)wheel;

  /* Report data changes */

  mouse_event(priv->lower.priv, &sample);
}
