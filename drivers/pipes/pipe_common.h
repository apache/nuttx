/****************************************************************************
 * drivers/pipes/pipe_common.h
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

#ifndef __DRIVERS_PIPES_PIPE_COMMON_H
#define __DRIVERS_PIPES_PIPE_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <poll.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pipe/FIFO support */

#ifndef CONFIG_PIPES
#  undef CONFIG_DEV_PIPE_MAXSIZE
#  undef CONFIG_DEV_PIPE_SIZE
#  undef CONFIG_DEV_FIFO_SIZE
#  define CONFIG_DEV_PIPE_MAXSIZE 0
#  define CONFIG_DEV_PIPE_SIZE 0
#  define CONFIG_DEV_FIFO_SIZE 0
#endif

/* Pipe/FIFO size */

#ifndef CONFIG_DEV_PIPE_MAXSIZE
#  define CONFIG_DEV_PIPE_MAXSIZE 1024
#endif

#if CONFIG_DEV_PIPE_MAXSIZE <= 0
#  undef CONFIG_PIPES
#  undef CONFIG_DEV_PIPE_SIZE
#  undef CONFIG_DEV_FIFO_SIZE
#  define CONFIG_DEV_PIPE_SIZE 0
#  define CONFIG_DEV_FIFO_SIZE 0
#endif

#ifndef CONFIG_DEV_PIPE_SIZE
#  define CONFIG_DEV_PIPE_SIZE 1024
#endif

#ifndef CONFIG_DEV_FIFO_SIZE
#  define CONFIG_DEV_FIFO_SIZE 1024
#endif

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_DEV_PIPE_NPOLLWAITERS
#  define CONFIG_DEV_PIPE_NPOLLWAITERS 2
#endif

/* Maximum number of open's supported on pipe */

#define CONFIG_DEV_PIPE_MAXUSER 255

/* d_flags values */

#define PIPE_FLAG_POLICY    (1 << 0) /* Bit 0: Policy=Free buffer when empty */
#define PIPE_FLAG_UNLINKED  (1 << 1) /* Bit 1: The driver has been unlinked */

#define PIPE_POLICY_0(f)    do { (f) &= ~PIPE_FLAG_POLICY; } while (0)
#define PIPE_POLICY_1(f)    do { (f) |= PIPE_FLAG_POLICY; } while (0)
#define PIPE_IS_POLICY_0(f) (((f) & PIPE_FLAG_POLICY) == 0)
#define PIPE_IS_POLICY_1(f) (((f) & PIPE_FLAG_POLICY) != 0)

#define PIPE_UNLINK(f)      do { (f) |= PIPE_FLAG_UNLINKED; } while (0)
#define PIPE_IS_UNLINKED(f) (((f) & PIPE_FLAG_UNLINKED) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Make the buffer index as small as possible for the configured pipe size */

#if CONFIG_DEV_PIPE_MAXSIZE > 65535
typedef uint32_t pipe_ndx_t;  /* 32-bit index */
#elif CONFIG_DEV_PIPE_MAXSIZE > 255
typedef uint16_t pipe_ndx_t;  /* 16-bit index */
#else
typedef uint8_t pipe_ndx_t;   /*  8-bit index */
#endif

/* This structure represents the state of one pipe.  A reference to this
 * structure is retained in the i_private field of the inode whenthe
 * pipe/fifo device is registered.
 */

struct pipe_dev_s
{
  sem_t      d_bfsem;       /* Used to serialize access to d_buffer and indices */
  sem_t      d_rdsem;       /* Empty buffer - Reader waits for data write */
  sem_t      d_wrsem;       /* Full buffer - Writer waits for data read */
  pipe_ndx_t d_wrndx;       /* Index in d_buffer to save next byte written */
  pipe_ndx_t d_rdndx;       /* Index in d_buffer to return the next byte read */
  pipe_ndx_t d_bufsize;     /* allocated size of d_buffer in bytes */
  uint8_t    d_nwriters;    /* Number of reference counts for write access */
  uint8_t    d_nreaders;    /* Number of reference counts for read access */
  uint8_t    d_pipeno;      /* Pipe minor number */
  uint8_t    d_flags;       /* See PIPE_FLAG_* definitions */
  uint8_t   *d_buffer;      /* Buffer allocated when device opened */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *d_fds[CONFIG_DEV_PIPE_NPOLLWAITERS];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

struct file;  /* Forward reference */
struct inode; /* Forward reference */

FAR struct pipe_dev_s *pipecommon_allocdev(size_t bufsize);
void    pipecommon_freedev(FAR struct pipe_dev_s *dev);
int     pipecommon_open(FAR struct file *filep);
int     pipecommon_close(FAR struct file *filep);
ssize_t pipecommon_read(FAR struct file *, FAR char *, size_t);
ssize_t pipecommon_write(FAR struct file *, FAR const char *, size_t);
int     pipecommon_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
int     pipecommon_poll(FAR struct file *filep, FAR struct pollfd *fds,
                               bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
int     pipecommon_unlink(FAR struct inode *priv);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_PIPES_PIPE_COMMON_H */
