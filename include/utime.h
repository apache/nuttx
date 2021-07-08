/****************************************************************************
 * include/utime.h
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

#ifndef __INCLUDE_UTIME_H
#define __INCLUDE_UTIME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <time.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct utimbuf
{
  time_t actime;  /* Access time */
  time_t modtime; /* Modification time */
};

/****************************************************************************
 * Public Function Prototypes
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
 * Name: utime
 *
 * Description:
 *   The utime() system call changes the access and modification times of
 *   the inode specified by path to the actime and modtime fields of times
 *   respectively.
 *   If times is NULL, then the access and modification times of the file
 *   are set to the current time.
 *
 * Input Parameters:
 *   path  - Specifies the file to be modified
 *   times - Specifies the time value to set
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall
 *   be returned and errno shall be set to indicate the error, and the file
 *   times shall not be affected.
 *
 ****************************************************************************/

int utime(FAR const char *path, FAR const struct utimbuf *times);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_UTIME_H */
