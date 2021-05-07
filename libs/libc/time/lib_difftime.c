/****************************************************************************
 * libs/libc/time/lib_difftime.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <time.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  difftime
 *
 * Description:
 *   The difftime() function returns the number of seconds elapsed
 *   between time time1 and time time0, represented as a double or a float.
 *   Float is used if the platform does not support double. However, when
 *   using a float, some precision may be lost for big differences.
 *
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double difftime(time_t time1, time_t time0)
{
  return (double)time1 - (double)time0;
}
#else
float difftime(time_t time1, time_t time0)
{
  if (time1 >= time0)
    {
      /* Result will be positive (even though bit 31 may be set on very large
       * differences!)
       */

      return (float)((uint32_t)(time1 - time0));
    }
  else
    {
      /* Result will be negative.
       * REVISIT: Am I missing any case where bit 31
       * might not be set?
       */

      return (float)((int32_t)(time1 - time0));
    }
}
#endif
