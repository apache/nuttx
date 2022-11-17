/****************************************************************************
 * arch/sim/src/sim/sim_keyboard.c
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
#include <sys/types.h>

#include <assert.h>
#include <debug.h>
#include <string.h>

#include <nuttx/input/keyboard.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME "/dev/kbd"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_dev_s
{
  int eventloop;
  struct keyboard_lowerhalf_s lower;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one simulated keyboard is supported so the driver state
 * structure may as well be pre-allocated.
 */

static struct sim_dev_s g_simkeyboard;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_kbd_initialize
 ****************************************************************************/

int sim_kbd_initialize(void)
{
  int              ret;
  struct sim_dev_s *priv = &g_simkeyboard;

  memset(priv, 0, sizeof(*priv));

  /* Register the device as an input device */

  ret = keyboard_register(&priv->lower, DEVNAME,
                          CONFIG_SIM_KEYBOARD_BUFFSIZE);
  if (ret < 0)
    {
      ierr("ERROR: keyboard_register() failed: %d\n", ret);
      return ret;
    }

  /* Enable X11 event processing from the IDLE loop */

  priv->eventloop = 1;
  return OK;
}

/****************************************************************************
 * Name: sim_kbdevent
 ****************************************************************************/

void sim_kbdevent(uint32_t key, bool is_press)
{
  struct sim_dev_s *priv = (struct sim_dev_s *) &g_simkeyboard;
  uint32_t types[2] =
    {
      KEYBOARD_RELEASE, KEYBOARD_PRESS
    };

  if (priv->eventloop == 0)
    {
      return;
    }

  iinfo("key=%04x\n", key);

  /* Report data changes */

  keyboard_event(&priv->lower, key, types[is_press]);
}
