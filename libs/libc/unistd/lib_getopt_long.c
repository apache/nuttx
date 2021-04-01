/****************************************************************************
 * libs/libc/unistd/lib_getopt_long.c
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

#include "unistd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getopt_long
 *
 * Description:
 *   The getopt_long() function works like getopt() except that it also
 *   accepts long options, started with two dashes. (If the program accepts
 *   only long options, then optstring should be specified as an empty
 *   string (""), not NULL.) Long option names may be abbreviated if the
 *   abbreviation is unique or is an exact match for some defined option
 *
 ****************************************************************************/

int getopt_long(int argc, FAR char * const argv[],
                FAR const char *optstring,
                FAR const struct option *longopts,
                FAR int *longindex)
{
  return getopt_common(argc, argv, optstring, longopts, longindex,
                       GETOPT_LONG_MODE);
}
