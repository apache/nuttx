/****************************************************************************
 * include/gcov.h
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

#ifndef __INCLUDE_GCOV_H
#define __INCLUDE_GCOV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: __gcov_reset
 *
 * Description:
 *   Set all counters to zero.
 *
 ****************************************************************************/

extern void __gcov_reset(void);

/****************************************************************************
 * Name: __gcov_dump
 *
 * Description:
 *   Write profile information to a file.
 *
 ****************************************************************************/

extern void __gcov_dump(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_GCOV_H */
