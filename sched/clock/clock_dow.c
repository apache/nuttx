/****************************************************************************
 * sched/clock/clock_dow.c
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

#include <nuttx/clock.h>

#include "clock/clock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 23 * (month + 1) / 9, month = 0..11 */

static const uint8_t g_lookup[12] =
{
  2, 5, 7, 10, 12, 15, 17, 20, 23, 25, 28, 30
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_dow
 *
 * Description:
 *   Calculate the day of week (DOW) from they year month and day.  Based on
 *   an algorithm published in 1990 by Michael Keith and Tom Craver with some
 *   tweaks to handle months in the range 0-11.
 *
 * Input Parameters:
 *   year  - year (e.g., 1988)
 *   month - 0 through 11
 *   day   - 1 through 31
 *
 * Returned Value:
 *   The day of the week as days since Sunday: 0 = Sunday, 1 = Monday, etc.
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_dow(int year, int month, int day)
{
  day += month < 2 ? year-- : year - 2;
  return ((int)g_lookup[month] + day + 4 + year / 4 -
           year / 100 + year / 400) % 7;
}
