/****************************************************************************
 * include/ulimit.h
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

#ifndef __INCLUDE_ULIMIT_H
#define __INCLUDE_ULIMIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/resource.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get limit on the size of a file, in units of 512 bytes */

#define UL_GETFSIZE 1

/* Set limit on the size of a file */

#define UL_SETFSIZE 2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

long int ulimit(int cmd, long newlimit);

#endif /* __INCLUDE_ULIMIT_H */
