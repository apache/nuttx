/****************************************************************************
 * libs/libc/time/lib_asctime.c
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

#include <time.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  asctime
 *
 * Description:
 *   asctime and asctime_r convert the time provided in a struct tm to a
 *   string representation.  asctime is not re-entrant; asctime_r is re-
 *   entrant
 *
 * Input Parameters:
 *   tp - Pointer to the time to be converted.
 *
 * Returned Value:
 *   On success, a pointer to the string is returned; on failure, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR char *asctime(FAR const struct tm *tp)
{
  static char buf[26];
  return asctime_r(tp, buf);
}
