/****************************************************************************
 * libs/libc/unistd/lib_getoptpublic.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifdef GETOPT_PUBLIC_VARIABLES

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The public getopt() variables of a kernel-build process.  They are in a
 * file of their own, so that a program that defines its own does not get
 * a second definition from the C library.
 * getopt_common() copies them to and from its internal state.
 */

FAR char *optarg;
int opterr = 1;
int optind = 1;
int optopt = '?';

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* GETOPT_PUBLIC_VARIABLES */
