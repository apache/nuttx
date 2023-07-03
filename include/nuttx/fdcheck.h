/****************************************************************************
 * include/nuttx/fdcheck.h
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

#ifndef __INCLUDE_NUTTX_FDCHECK_H
#define __INCLUDE_NUTTX_FDCHECK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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

#ifdef CONFIG_FDCHECK

/****************************************************************************
 * Name: fdcheck_restore
 *
 * Description: Obtain original fd information
 *
 * Val carries the pid and fd information.
 * The original fd information is stored in low bit of val.
 * The pid information is stored in the high bit of val.
 * For ease of understanding, let's give an example where
 * the following information is represented in 32-bit binary format
 *
 *  val       00000000 00000000 01010101 10001010
 *  fd        00000000 00000000 00000000 10001010
 *  pid       00000000 00000000 00000000 01010101
 *
 * In this function, we also check if the pid information is correct.
 * If there is an error, it will panic.
 *
 * Input Parameters:
 *   val - this val carrying pid and original fd information
 *
 * Returned Value: none
 *
 ****************************************************************************/

int fdcheck_restore(int fd);

/****************************************************************************
 * Name: fdcheck_protect
 *
 * Description: Obtain the combined value of fd and pid
 *
 * the return value carries the pid and fd information.
 * The original fd information is stored in low bit of val.
 * The pid information is stored in high bit of val.
 * For ease of understanding, let's give an example where
 * the following information is represented in 32-bit binary format
 *
 *  fd        00000000 00000000 00000000 10001010
 *  pid       00000000 00000000 00000000 01010101
 *  val       00000000 00000000 01010101 10001010
 *
 * Input Parameters:
 *   fd - original fd
 *
 * Returned Value: the combined value of fd and pid
 *
 ****************************************************************************/

int fdcheck_protect(int fd);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif

#endif /* __INCLUDE_NUTTX_FDCHECK_H */
