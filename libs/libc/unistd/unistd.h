/****************************************************************************
 * libs/libc/unistd/unistd.h
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

#ifndef __LIBS_LIBC_UNISTD_UNISTD_H
#define __LIBS_LIBC_UNISTD_UNISTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/tls.h>

#include <stdbool.h>
#include <getopt.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Mode bit definitions */

#define GETOPT_LONG_BIT         (1 << 0) /* Long options supported */
#define GETOPT_LONGONLY_BIT     (1 << 1) /* Long-Only behavior supported */

#define GETOPT_HAVE_LONG(m)     (((m) & GETOPT_LONG_BIT) != 0)
#define GETOPT_HAVE_LONGONLY(m) (((m) & GETOPT_LONGONLY_BIT) != 0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* The mode determines which of getopt(), getopt_long(), and
 * getopt_long_only() that is being emulated by getopt_common().
 */

enum getopt_mode_e
{
  GETOPT_MODE           = 0,
  GETOPT_LONG_MODE      = GETOPT_LONG_BIT,
  GETOPT_LONG_ONLY_MODE = (GETOPT_LONG_BIT | GETOPT_LONGONLY_BIT)
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: getoptvars
 *
 * Description:
 *   Returns a pointer to to the thread-specific getopt() data.
 *
 ****************************************************************************/

FAR struct getopt_s *getoptvars(void);

/****************************************************************************
 * Name: getopt_common
 *
 * Description:
 *   getopt_common() is the common, internal implementation of getopt(),
 *   getopt_long(), and getopt_long_only().
 *
 ****************************************************************************/

int getopt_common(int argc, FAR char * const argv[],
                  FAR const char *optstring,
                  FAR const struct option *longopts,
                  FAR int *longindex,
                  enum getopt_mode_e mode);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBS_LIBC_UNISTD_UNISTD_H */
