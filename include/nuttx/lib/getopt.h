/****************************************************************************
 * include/nuttx/lib/getopt.h
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

#ifndef __INCLUDE_NUTTX_LIB_GETOPT_H
#define __INCLUDE_NUTTX_LIB_GETOPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure encapsulates all variables associated with getopt(). */

struct getopt_s
{
  /* Part of the implementation of the public getopt() interface */

  FAR char *go_optarg;       /* Optional argument following option */
  int       go_opterr;       /* Print error message */
  int       go_optind;       /* Index into argv */
  int       go_optopt;       /* unrecognized option character */

  /* Internal getopt() state */

  FAR char *go_optptr;       /* Current parsing location */
  bool      go_binitialized; /* true:  getopt() has been initialized */
};

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
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_GETOPT_H */
