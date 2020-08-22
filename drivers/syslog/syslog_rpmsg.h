/****************************************************************************
 * drivers/syslog/syslog_rpmsg.h
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

#ifndef __DRIVERS_SYSLOG_SYSLOG_RPMSG_H
#define __DRIVERS_SYSLOG_SYSLOG_RPMSG_H

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define SYSLOG_RPMSG_EPT_NAME           "rpmsg-syslog"

#define SYSLOG_RPMSG_TRANSFER           0
#define SYSLOG_RPMSG_TRANSFER_DONE      1
#define SYSLOG_RPMSG_SUSPEND            2
#define SYSLOG_RPMSG_RESUME             3

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct syslog_rpmsg_header_s
{
  uint32_t command;
  int32_t  result;
} end_packed_struct;

begin_packed_struct struct syslog_rpmsg_transfer_s
{
  struct syslog_rpmsg_header_s header;
  int32_t                      count;
  char                         data[0];
} end_packed_struct;

#endif /* __DRIVERS_SYSLOG_SYSLOG_RPMSG_H */
