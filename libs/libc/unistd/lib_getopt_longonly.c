/****************************************************************************
 * libs/libc/unistd/lib_getopt_longonly.c
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
 * Name: getopt_long_only
 *
 * Description:
 *   getopt_long_only() is like getopt_long(), but '-' as well as "--" can
 *   indicate a long option. If an option that starts with '-' (not "--")
 *   doesn't match a long option, but does match a short option, it is
 *   parsed as a short option instead.
 *
 ****************************************************************************/

int getopt_long_only(int argc, FAR char * const argv[],
                     FAR const char *optstring,
                     FAR const struct option *longopts,
                     FAR int *longindex)
{
  return getopt_common(argc, argv, optstring, longopts, longindex,
                       GETOPT_LONG_ONLY_MODE);
}
