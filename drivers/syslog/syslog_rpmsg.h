/****************************************************************************
 * drivers/syslog/syslog_rpmsg.h
 * Syslog driver for rpmsg syslog
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Private Types
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
