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

int getopt_long(int argc, FAR char *const argv[],
                FAR const char *shortopts,
                FAR const struct option *longopts,
                FAR int *longind);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_GETOPT_H */
