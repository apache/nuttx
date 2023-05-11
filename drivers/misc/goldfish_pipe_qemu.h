/****************************************************************************
 * drivers/misc/goldfish_pipe_qemu.h
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

#ifndef __GOLDFISH_PIPE_QEMU_H
#define __GOLDFISH_PIPE_QEMU_H

/* List of bitflags returned in status of CMD_POLL command */

enum pipepollflags
{
  PIPE_POLL_IN = 1 << 0,
  PIPE_POLL_OUT = 1 << 1,
  PIPE_POLL_HUP = 1 << 2
};

/* Possible status values used to signal errors */

enum pipeerrors
{
  PIPE_ERROR_INVAL = -1,
  PIPE_ERROR_AGAIN = -2,
  PIPE_ERROR_NOMEM = -3,
  PIPE_ERROR_IO = -4
};

/* Bit-flags used to signal events from the emulator */

enum pipewakeflags
{
  /* emulator closed pipe */

  PIPE_WAKE_CLOSED = 1 << 0,

  /* pipe can now be read from */

  PIPE_WAKE_READ = 1 << 1,

  /* pipe can now be written to */

  PIPE_WAKE_WRITE = 1 << 2,

  /* unlock this pipe's DMA buffer */

  PIPE_WAKE_UNLOCK_DMA = 1 << 3,

  /* unlock DMA buffer of the pipe shared to this pipe */

  PIPE_WAKE_UNLOCK_DMA_SHARED = 1 << 4,
};

/* Possible pipe closing reasons */

enum pipeclosereason
{
  /* guest sent a close command */

  PIPE_CLOSE_GRACEFUL = 0,

  /* guest rebooted, we're closing the pipes */

  PIPE_CLOSE_REBOOT = 1,

  /* close old pipes on snapshot load */

  PIPE_CLOSE_LOAD_SNAPSHOT = 2,

  /* some unrecoverable error on the pipe */

  PIPE_CLOSE_ERROR = 3,
};

/* Bit flags for the 'flags' field */

enum pipeflagsbits
{
  BIT_CLOSED_ON_HOST = 0,        /* pipe closed by host */
  BIT_WAKE_ON_WRITE = 1,         /* want to be woken on writes */
  BIT_WAKE_ON_READ = 2,          /* want to be woken on reads */
};

enum piperegs
{
  PIPE_REG_CMD = 0,

  PIPE_REG_SIGNAL_BUFFER_HIGH = 4,
  PIPE_REG_SIGNAL_BUFFER = 8,
  PIPE_REG_SIGNAL_BUFFER_COUNT = 12,

  PIPE_REG_OPEN_BUFFER_HIGH = 20,
  PIPE_REG_OPEN_BUFFER = 24,

  PIPE_REG_VERSION = 36,

  PIPE_REG_GET_SIGNALLED = 48,
};

enum pipecmdcode
{
  /* to be used by the pipe device itself */

  PIPE_CMD_OPEN = 1,

  PIPE_CMD_CLOSE,
  PIPE_CMD_POLL,
  PIPE_CMD_WRITE,
  PIPE_CMD_WAKE_ON_WRITE,
  PIPE_CMD_READ,
  PIPE_CMD_WAKE_ON_READ,

  /**
   * TODO(zyy): implement a deferred read/write execution to allow
   * parallel processing of pipe operations on the host.
   */

  PIPE_CMD_WAKE_ON_DONE_IO,
};

#endif /* __GOLDFISH_PIPE_QEMU_H */
