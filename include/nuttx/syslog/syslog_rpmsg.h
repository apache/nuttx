/****************************************************************************
 * include/nuttx/syslog/syslog_rpmsg.h
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

#ifndef __INCLUDE_NUTTX_SYSLOG_SYSLOG_RPMSG_H
#define __INCLUDE_NUTTX_SYSLOG_SYSLOG_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/syslog/syslog.h>

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

#ifdef CONFIG_SYSLOG_RPMSG
void syslog_rpmsg_init_early(FAR void *buffer, size_t size);
int syslog_rpmsg_init(void);

int syslog_rpmsg_putc(FAR struct syslog_channel_s *channel, int ch);
int syslog_rpmsg_flush(FAR struct syslog_channel_s *channel);
ssize_t syslog_rpmsg_write(FAR struct syslog_channel_s *channel,
                           FAR const char *buffer, size_t buflen);
#endif

#ifdef CONFIG_SYSLOG_RPMSG_SERVER
int syslog_rpmsg_server_init(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SYSLOG_SYSLOG_RPMSG_H */
