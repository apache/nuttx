/****************************************************************************
 * libs/libc/time/lib_dayofweek.c
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Constant Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock_dayoftheweek
 *
 * Description:
 *    Get the day of the week
 *
 * Input Parameters:
 *   mday  - The day of the month 1 - 31
 *   month - The month of the year 1 - 12
 *   year  - the year including the 1900
 *
 * Returned Value:
 *   Zero based day of the week 0-6, 0 = Sunday, 1 = Monday... 6 = Saturday
 *
 ****************************************************************************/

int clock_dayoftheweek(int mday, int month, int year)
{
  if (month <= 2)
    {
      year--;
      month += 12;
    }

  month -= 2;
  return (mday + year + year / 4 - year / 100 + year / 400 +
         (31 * month) / 12) % 7;
}
