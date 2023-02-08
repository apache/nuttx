/****************************************************************************
 * include/nuttx/net/netfilter/x_tables.h
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

#ifndef __INCLUDE_NUTTX_NET_NETFILTER_X_TABLES_H
#define __INCLUDE_NUTTX_NET_NETFILTER_X_TABLES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XT_FUNCTION_MAXNAMELEN  30
#define XT_EXTENSION_MAXNAMELEN 29
#define XT_TABLE_MAXNAMELEN     32

#define XT_STANDARD_TARGET   ""     /* Standard return verdict, or do jump. */
#define XT_ERROR_TARGET      "ERROR"
#define XT_MASQUERADE_TARGET "MASQUERADE"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct xt_entry_target
{
  union
    {
      struct
        {
          uint16_t target_size;

          char name[XT_EXTENSION_MAXNAMELEN];
          uint8_t revision;
        } user;
      struct
        {
          uint16_t target_size;
        } kernel;

      uint16_t target_size; /* Total length */
    } u;

  unsigned char data[1];
};

struct xt_standard_target
{
  struct xt_entry_target target;
  int verdict;
};

struct xt_error_target
{
  struct xt_entry_target target;
  char errorname[XT_FUNCTION_MAXNAMELEN];
};

struct xt_counters
{
  /* Packet and byte counters */

  uint64_t pcnt;
  uint64_t bcnt;
};

#endif /* __INCLUDE_NUTTX_NET_NETFILTER_X_TABLES_H */
