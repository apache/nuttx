/****************************************************************************
 * drivers/mtd/rpmsgmtd.h
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

#ifndef __DRIVERS_MTD_RPMSGMTD_H
#define __DRIVERS_MTD_RPMSGMTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)         (sizeof(x) / sizeof((x)[0]))
#endif

#define RPMSGMTD_NAME_PREFIX     "rpmsgmtd-"
#define RPMSGMTD_NAME_PREFIX_LEN 9

#define RPMSGMTD_ERASE           1
#define RPMSGMTD_BREAD           2
#define RPMSGMTD_BWRITE          3
#define RPMSGMTD_READ            4
#define RPMSGMTD_WRITE           5
#define RPMSGMTD_IOCTL           6

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct rpmsgmtd_header_s
{
  uint32_t                 command;
  int32_t                  result;
  uint64_t                 cookie;
} end_packed_struct;

begin_packed_struct struct rpmsgmtd_erase_s
{
  struct rpmsgmtd_header_s header;
  int64_t                  startblock;
  uint64_t                 nblocks;
} end_packed_struct;

begin_packed_struct struct rpmsgmtd_bread_s
{
  struct rpmsgmtd_header_s header;
  int64_t                  startblock;
  uint64_t                 nblocks;
  uint32_t                 blocksize;
  uint8_t                  buf[1];
} end_packed_struct;

#define rpmsgmtd_bwrite_s rpmsgmtd_bread_s

begin_packed_struct struct rpmsgmtd_read_s
{
  struct rpmsgmtd_header_s header;
  int64_t                  offset;
  uint64_t                 nbytes;
  uint8_t                  buf[1];
} end_packed_struct;

#define rpmsgmtd_write_s rpmsgmtd_read_s

begin_packed_struct struct rpmsgmtd_ioctl_s
{
  struct rpmsgmtd_header_s header;
  int32_t                  request;
  uint64_t                 arg;
  uint32_t                 arglen;
  uint8_t                  buf[1];
} end_packed_struct;

/****************************************************************************
 * Internal function prototypes
 ****************************************************************************/

/****************************************************************************
 * Internal data
 ****************************************************************************/

#endif /* __DRIVERS_MTD_RPMSGMTD_H */
