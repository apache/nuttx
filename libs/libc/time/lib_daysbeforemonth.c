/****************************************************************************
 * libs/libc/time/lib_daysbeforemonth.c
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
#include <stdbool.h>

#include <nuttx/time.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_daysbeforemonth[13] =
{
  0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock_daysbeforemonth
 *
 * Description:
 *    Get the number of days that occurred before the beginning of the
 *    month.
 *
 * Input Parameters:
 *    month    - The month in the form of tm_mon, that is a range of 0-11.
 *    leapyear - True if leap year and there are 29 days in February.
 *               NOTE the month=1 is February.
 *
 * Returned Value:
 *    The number of days that occurred before the month
 *
 ****************************************************************************/

int clock_daysbeforemonth(int month, bool leapyear)
{
  int retval = g_daysbeforemonth[month];
  if (month >= 2 && leapyear)
    {
      retval++;
    }

  return retval;
}
