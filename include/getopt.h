/****************************************************************************
 * include/getopt.h
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

#ifndef __INCLUDE_GETOPT_H
#define __INCLUDE_GETOPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define no_argument             0
#define required_argument       1
#define optional_argument       2

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct option
{
  FAR const char *name;
  int has_arg;
  FAR int *flag;
  int val;
};

/****************************************************************************
 * Public Function Prototypes
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
                FAR int *longindex);

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
                     FAR int *longindex);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_GETOPT_H */
