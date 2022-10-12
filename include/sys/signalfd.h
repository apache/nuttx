/****************************************************************************
 * include/sys/signalfd.h
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

#ifndef __INCLUDE_SYS_SIGNALFD_H
#define __INCLUDE_SYS_SIGNALFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <signal.h>
#include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SFD_CLOEXEC O_CLOEXEC
#define SFD_NONBLOCK O_NONBLOCK

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct signalfd_siginfo
{
  uint32_t ssi_signo;
  int32_t  ssi_errno;
  int32_t  ssi_code;
  uint32_t ssi_pid;
  uint32_t ssi_uid;
  int32_t  ssi_fd;
  uint32_t ssi_tid;
  uint32_t ssi_band;
  uint32_t ssi_overrun;
  uint32_t ssi_trapno;
  int32_t  ssi_status;
  int32_t  ssi_int;
  uint64_t ssi_ptr;
  uint64_t ssi_utime;
  uint64_t ssi_stime;
  uint64_t ssi_addr;
  uint16_t ssi_addr_lsb;
  uint16_t __pad2;
  int32_t  ssi_syscall;
  uint64_t ssi_call_addr;
  uint32_t ssi_arch;
  uint8_t  __pad[28];
};

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

int signalfd(int fd, FAR const sigset_t *mask, int flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SYS_TIMERFD_H */
