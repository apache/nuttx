/****************************************************************************
 * drivers/leds/userled_lower.c
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
#include <inttypes.h>

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#if CONFIG_USERLED_LOWER

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static userled_set_t
userled_supported(FAR const struct userled_lowerhalf_s *lower);
static void userled_setled(FAR const struct userled_lowerhalf_s *lower,
                           int led, bool ledon);
static void userled_setall(FAR const struct userled_lowerhalf_s *lower,
                           userled_set_t ledset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_lednum;

/* This is the user LED lower half driver interface */

static const struct userled_lowerhalf_s g_userled_lower =
{
  .ll_supported = userled_supported,
  .ll_setled    = userled_setled,
  .ll_setall    = userled_setall,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userled_supported
 *
 * Description:
 *   Return the set of LEDs supported by the board
 *
 ****************************************************************************/

static userled_set_t
userled_supported(FAR const struct userled_lowerhalf_s *lower)
{
  ledinfo("BOARD_NLEDS: %02" PRIx32 "\n", g_lednum);
  return (userled_set_t)((1 << g_lednum) - 1);
}

/****************************************************************************
 * Name: userled_setled
 *
 * Description:
 *   Set the current state of one LED
 *
 ****************************************************************************/

static void userled_setled(FAR const struct userled_lowerhalf_s *lower,
                           int led, bool ledon)
{
  board_userled(led, ledon);
}

/****************************************************************************
 * Name: userled_setall
 *
 * Description:
 *   Set the state of all LEDs
 *
 ****************************************************************************/

static void userled_setall(FAR const struct userled_lowerhalf_s *lower,
                           userled_set_t ledset)
{
  board_userled_all(ledset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userled_lower_initialize
 *
 * Description:
 *   Initialize the generic LED lower half driver, bind it and register
 *   it with the upper half LED driver as devname.
 *
 ****************************************************************************/

int userled_lower_initialize(FAR const char *devname)
{
  g_lednum = board_userled_initialize();
  return userled_register(devname, &g_userled_lower);
}

#endif /* CONFIG_USERLED_LOWER */
