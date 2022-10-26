/****************************************************************************
 * drivers/misc/rpmsgblk.h
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

#ifndef __DRIVERS_MISC_RPMSGBLK_H
#define __DRIVERS_MISC_RPMSGBLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)         (sizeof(x) / sizeof((x)[0]))
#endif

#define RPMSGBLK_NAME_PREFIX     "rpmsgblk-"
#define RPMSGBLK_NAME_PREFIX_LEN 9

#define RPMSGBLK_OPEN            1
#define RPMSGBLK_CLOSE           2
#define RPMSGBLK_READ            3
#define RPMSGBLK_WRITE           4
#define RPMSGBLK_GEOMETRY        5
#define RPMSGBLK_IOCTL           6
#define RPMSGBLK_UNLINK          7

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct rpmsgblk_header_s
{
  uint32_t                command;
  int32_t                 result;
  uint64_t                cookie;
} end_packed_struct;

begin_packed_struct struct rpmsgblk_open_s
{
  struct rpmsgblk_header_s header;
} end_packed_struct;

#define rpmsgblk_close_s rpmsgblk_open_s

begin_packed_struct struct rpmsgblk_read_s
{
  struct rpmsgblk_header_s header;
  uint32_t                 startsector;
  uint32_t                 nsectors;
  int32_t                  sectorsize;
  uint32_t                 count;
  char                     buf[1];
} end_packed_struct;

#define rpmsgblk_write_s rpmsgblk_read_s

begin_packed_struct struct rpmsgblk_geometry_s
{
  struct rpmsgblk_header_s header;
  uint64_t                 arg;
  uint32_t                 arglen;
  char                     buf[1];
} end_packed_struct;

begin_packed_struct struct rpmsgblk_ioctl_s
{
  struct rpmsgblk_header_s header;
  uint64_t                 arg;
  uint32_t                 arglen;
  int32_t                  request;
  char                     buf[1];
} end_packed_struct;

begin_packed_struct struct rpmsgblk_unlink_s
{
  struct rpmsgblk_header_s header;
} end_packed_struct;

/****************************************************************************
 * Internal function prototypes
 ****************************************************************************/

/****************************************************************************
 * Internal data
 ****************************************************************************/

#endif /* __DRIVERS_MISC_RPMSGBLK_H */
