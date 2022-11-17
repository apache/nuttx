/****************************************************************************
 * arch/sim/src/sim/sim_ajoystick.c
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

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/input/ajoystick.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

# define AJOY_SUPPORTED (AJOY_BUTTON_1_BIT | AJOY_BUTTON_2_BIT | \
                         AJOY_BUTTON_3_BIT)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ajoy_buttonset_t ajoy_supported(
  const struct ajoy_lowerhalf_s *lower);
static int ajoy_sample(const struct ajoy_lowerhalf_s *lower,
                       struct ajoy_sample_s *sample);
static ajoy_buttonset_t ajoy_buttons(
  const struct ajoy_lowerhalf_s *lower);
static void ajoy_enable(const struct ajoy_lowerhalf_s *lower,
                        ajoy_buttonset_t press, ajoy_buttonset_t release,
                        ajoy_handler_t handler, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the button joystick lower half driver interface */

static const struct ajoy_lowerhalf_s g_ajoylower =
{
  .al_supported  = ajoy_supported,
  .al_sample     = ajoy_sample,
  .al_buttons    = ajoy_buttons,
  .al_enable     = ajoy_enable,
};

/* Driver state data */

static int g_eventloop;
static bool g_ajoy_valid;                  /* True: Sample data is valid */
static struct ajoy_sample_s g_ajoy_sample; /* Last sample data */
static ajoy_buttonset_t g_ajoy_buttons;    /* Last buttons set */

static ajoy_handler_t g_ajoy_handler;      /* "Interrupt" handler */
static void *g_ajoy_arg;                   /* Handler argument */
static ajoy_buttonset_t g_ajoy_pset;       /* Set of press waited for */
static ajoy_buttonset_t g_ajoy_rset;       /* Set of releases waited for */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ajoy_supported
 *
 * Description:
 *   Return the set of buttons supported on the button joystick device
 *
 ****************************************************************************/

static ajoy_buttonset_t ajoy_supported(
  const struct ajoy_lowerhalf_s *lower)
{
  return (ajoy_buttonset_t)AJOY_SUPPORTED;
}

/****************************************************************************
 * Name: ajoy_sample
 *
 * Description:
 *   Return the current state of all button joystick buttons
 *
 ****************************************************************************/

static int ajoy_sample(const struct ajoy_lowerhalf_s *lower,
                       struct ajoy_sample_s *sample)
{
  memcpy(sample, &g_ajoy_sample, sizeof(struct ajoy_sample_s));
  g_ajoy_buttons = g_ajoy_sample.as_buttons;
  g_ajoy_valid = false;
  return OK;
}

/****************************************************************************
 * Name: ajoy_buttons
 *
 * Description:
 *   Return the current state of button data (only)
 *
 ****************************************************************************/

static ajoy_buttonset_t ajoy_buttons(
  const struct ajoy_lowerhalf_s *lower)
{
  g_ajoy_valid   = false;
  g_ajoy_buttons = g_ajoy_sample.as_buttons;
  return g_ajoy_buttons;
}

/****************************************************************************
 * Name: ajoy_enable
 *
 * Description:
 *   Enable interrupts on the selected set of joystick buttons.  And empty
 *   set will disable all interrupts.
 *
 ****************************************************************************/

static void ajoy_enable(const struct ajoy_lowerhalf_s *lower,
                        ajoy_buttonset_t press, ajoy_buttonset_t release,
                        ajoy_handler_t handler, void *arg)
{
  g_ajoy_handler = NULL;
  g_ajoy_pset    = press;
  g_ajoy_rset    = release;
  g_ajoy_arg     = arg;
  g_ajoy_handler = handler;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_ajoy_initialize
 *
 * Description:
 *   Initialize and register the button joystick driver
 *
 ****************************************************************************/

int sim_ajoy_initialize(void)
{
  int ret;

  /* Register the joystick device as /dev/ajoy0 */

  ret = ajoy_register("/dev/ajoy0", &g_ajoylower);
  if (ret == OK)
    {
      /* Enable X11 event processing from the IDLE loop */

      g_eventloop = 1;
    }

  return ret;
}

/****************************************************************************
 * Name: sim_buttonevent
 ****************************************************************************/

void sim_buttonevent(int x, int y, int buttons)
{
  ajoy_buttonset_t changed;
  ajoy_buttonset_t pressed;
  ajoy_buttonset_t released;

  if (g_eventloop == 0)
    {
      return;
    }

  /* Same the positional data */

  g_ajoy_sample.as_x = x;
  g_ajoy_sample.as_y = y;

  /* Map X11 buttons to joystick buttons */

  g_ajoy_sample.as_buttons = 0;
  if ((buttons & 1) != 0)
    {
      g_ajoy_sample.as_buttons |= AJOY_BUTTON_1_BIT;
    }

  if ((buttons & 2) != 0)
    {
      g_ajoy_sample.as_buttons |= AJOY_BUTTON_2_BIT;
    }

  if ((buttons & 4) != 0)
    {
      g_ajoy_sample.as_buttons |= AJOY_BUTTON_3_BIT;
    }

  /* Sample data is valid */

  g_ajoy_valid = true;

  /* Is there an "interrupt" handler attached? */

  if (g_ajoy_handler)
    {
      /* Check button presses */

      changed  = g_ajoy_buttons ^ g_ajoy_sample.as_buttons;
      if (changed != 0)
        {
          pressed  = changed & (AJOY_SUPPORTED & g_ajoy_pset);
          released = changed & (AJOY_SUPPORTED & ~g_ajoy_rset);
          if ((pressed & g_ajoy_pset) != 0 || (released & g_ajoy_rset) != 0)
            {
              /* Call the interrupt handler */

              g_ajoy_handler(&g_ajoylower, g_ajoy_arg);
            }
        }
    }
}
