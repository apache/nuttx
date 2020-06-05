/****************************************************************************
 * drivers/leds/userled_lower.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#if CONFIG_USERLED_LOWER

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static userled_set_t
userled_supported(FAR const struct userled_lowerhalf_s *lower);
static void userled_led(FAR const struct userled_lowerhalf_s *lower,
                        int led, bool ledon);
static void userled_ledset(FAR const struct userled_lowerhalf_s *lower,
                           userled_set_t ledset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_lednum;

/* This is the user LED lower half driver interface */

static const struct userled_lowerhalf_s g_userled_lower =
{
  .ll_supported = userled_supported,
  .ll_led       = userled_led,
  .ll_ledset    = userled_ledset,
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
  ledinfo("BOARD_NLEDS: %02x\n", g_lednum);
  return (userled_set_t)((1 << g_lednum) - 1);
}

/****************************************************************************
 * Name: userled_led
 *
 * Description:
 *   Set the current state of one LED
 *
 ****************************************************************************/

static void userled_led(FAR const struct userled_lowerhalf_s *lower,
                        int led, bool ledon)
{
  board_userled(led, ledon);
}

/****************************************************************************
 * Name: userled_led
 *
 * Description:
 *   Set the state of all LEDs
 *
 ****************************************************************************/

static void userled_ledset(FAR const struct userled_lowerhalf_s *lower,
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
