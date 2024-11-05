/****************************************************************************
 * drivers/misc/goldfish_pipe.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Usage from the guest is simple:
 *
 *    int fd = open("/dev/goldfish_pipe", O_RDWR);
 *    .... write() or read() through the pipe.
 *
 * Connect to a specific qemu service:
 *
 *    // Do this immediately after opening the fd
 *    const char* msg = "<pipename>";
 *    if (write(fd, msg, strlen(msg) + 1) < 0) {
 *       ... could not connect to <pipename> service
 *       close(fd);
 *    }
 *
 * After this, simply read() and write() to communicate with the service.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* List of bitflags returned in status of GOLDFISH_PIPE_CMD_POLL command */

#define GOLDFISH_PIPE_POLL_IN                 (1 << 0)
#define GOLDFISH_PIPE_POLL_OUT                (1 << 1)
#define GOLDFISH_PIPE_POLL_HUP                (1 << 2)

/* Possible status values used to signal errors */

#define GOLDFISH_PIPE_ERROR_INVAL             -1
#define GOLDFISH_PIPE_ERROR_AGAIN             -2
#define GOLDFISH_PIPE_ERROR_NOMEM             -3
#define GOLDFISH_PIPE_ERROR_IO                -4

/* Bit-flags used to signal events from the emulator */

#define GOLDFISH_PIPE_WAKE_CLOSED             (1 << 0) /* Emulator closed pipe */
#define GOLDFISH_PIPE_WAKE_READ               (1 << 1) /* Pipe can now be read from */
#define GOLDFISH_PIPE_WAKE_WRITE              (1 << 2) /* Pipe can now be written to */
#define GOLDFISH_PIPE_WAKE_UNLOCK_DMA         (1 << 3) /* Unlock this pipe's DMA buffer */
#define GOLDFISH_PIPE_WAKE_UNLOCK_DMA_SHARED  (1 << 4) /* Unlock DMA buffer of the pipe shared to this pipe */

/* Possible pipe closing reasons */

#define GOLDFISH_PIPE_CLOSE_GRACEFUL          0 /* Guest sent a close command */
#define GOLDFISH_PIPE_CLOSE_REBOOT            1 /* Guest rebooted, we're closing the pipes */
#define GOLDFISH_PIPE_CLOSE_LOAD_SNAPSHOT     2 /* Close old pipes on snapshot load */
#define GOLDFISH_PIPE_CLOSE_ERROR             3 /* Some unrecoverable error on the pipe */

/* Register offset */

#define GOLDFISH_PIPE_REG_CMD                 0
#define GOLDFISH_PIPE_REG_SIGNAL_BUFFER_HIGH  4
#define GOLDFISH_PIPE_REG_SIGNAL_BUFFER       8
#define GOLDFISH_PIPE_REG_SIGNAL_BUFFER_COUNT 12
#define GOLDFISH_PIPE_REG_OPEN_BUFFER_HIGH    20
#define GOLDFISH_PIPE_REG_OPEN_BUFFER         24
#define GOLDFISH_PIPE_REG_VERSION             36
#define GOLDFISH_PIPE_REG_GET_SIGNALLED       48

/* Possible pipe command */

#define GOLDFISH_PIPE_CMD_OPEN                1
#define GOLDFISH_PIPE_CMD_CLOSE               2
#define GOLDFISH_PIPE_CMD_POLL                3
#define GOLDFISH_PIPE_CMD_WRITE               4
#define GOLDFISH_PIPE_CMD_WAKE_ON_WRITE       5
#define GOLDFISH_PIPE_CMD_READ                6
#define GOLDFISH_PIPE_CMD_WAKE_ON_READ        7
#define GOLDFISH_PIPE_CMD_WAKE_ON_DONE_IO     8

/* Version number */

#define GOLDFISH_PIPE_DRIVER_VERSION          4
#define GOLDFISH_PIPE_CURRENT_DEVICE_VERSION  2

#define GOLDFISH_PIPE_MAX_POLL_WAITERS        2
#define GOLDFISH_PIPE_MAX_COMMAND_BUFFERS     1
#define GOLDFISH_PIPE_MAX_SIGNALLED           16
#define GOLDFISH_PIPE_MAX_PIPES               32

#define goldfish_pipe_putreg32(v, x)          (*(FAR volatile uint32_t *)(x) = (v))
#define goldfish_pipe_getreg32(x)             (*(FAR volatile uint32_t *)(x))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A per-pipe command structure, shared with the host */

struct goldfish_pipe_command_s
{
  int32_t cmd;      /* PipeCmdCode, guest -> host */
  int32_t id;       /* Pipe id, guest -> host */
  int32_t status;   /* Command execution status, host -> guest */
  int32_t reserved; /* To pad to 64-bit boundary */

  struct /* Parameters for GOLDFISH_PIPE_CMD_[READ|WRITE] */
    {
      uint32_t buffers_count; /* Number of buffers, guest -> host */
      int32_t consumed_size;  /* Number of consumed bytes, host -> guest */

      uint64_t ptrs[GOLDFISH_PIPE_MAX_COMMAND_BUFFERS];  /* Buffer pointers, guest -> host */
      uint32_t sizes[GOLDFISH_PIPE_MAX_COMMAND_BUFFERS]; /* Buffer sizes, guest -> host */
    }
  rw_params;
};

/* A single signalled pipe information */

struct goldfish_pipe_signalled_s
{
  uint32_t id;
  uint32_t flags;
};

/* Parameters for the GOLDFISH_PIPE_CMD_OPEN command */

struct goldfish_pipe_open_param_s
{
  uint64_t command_buffer_ptr;
  uint32_t rw_params_max_count;
};

/* This data type models a given pipe instance */

struct goldfish_pipe_dev_s;

struct goldfish_pipe_s
{
  uint32_t id;
  bool closed;
  struct goldfish_pipe_command_s command;
  mutex_t lock;
  sem_t wait_for_write;
  sem_t wait_for_read;
  FAR struct goldfish_pipe_dev_s *dev;
  FAR struct pollfd *fds[GOLDFISH_PIPE_MAX_POLL_WAITERS];
};

struct goldfish_pipe_dev_s
{
  spinlock_t lock;
  FAR struct goldfish_pipe_s *pipes[GOLDFISH_PIPE_MAX_PIPES];
  struct goldfish_pipe_open_param_s open_params;
  struct goldfish_pipe_signalled_s
    pipes_signalled[GOLDFISH_PIPE_MAX_SIGNALLED];
  FAR uint8_t *base;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t goldfish_pipe_read(FAR struct file *filp, FAR char *buffer,
                                  size_t bufflen);
static ssize_t goldfish_pipe_write(FAR struct file *filp,
                                   FAR const char *buffer, size_t bufflen);
static int goldfish_pipe_poll(FAR struct file *filp,
                              FAR struct pollfd *fds, bool setup);
static int goldfish_pipe_open(FAR struct file *filep);
static int goldfish_pipe_close(FAR struct file *filep);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_goldfish_pipe_fops =
{
  .read  = goldfish_pipe_read,
  .write = goldfish_pipe_write,
  .poll  = goldfish_pipe_poll,
  .open  = goldfish_pipe_open,
  .close = goldfish_pipe_close,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int goldfish_pipe_convert_status(int status)
{
  switch (status)
    {
      case GOLDFISH_PIPE_ERROR_INVAL:
        return -EINVAL;
      case GOLDFISH_PIPE_ERROR_AGAIN:
        return -EAGAIN;
      case GOLDFISH_PIPE_ERROR_NOMEM:
        return -ENOMEM;
      case GOLDFISH_PIPE_ERROR_IO:
        return -EIO;
      default:
        return status;
    }
}

static int goldfish_pipe_command_locked(FAR struct goldfish_pipe_s *pipe,
                                        int cmd)
{
  pipe->command.cmd = cmd;
  pipe->command.status = GOLDFISH_PIPE_ERROR_INVAL;
  goldfish_pipe_putreg32(pipe->id, pipe->dev->base + GOLDFISH_PIPE_REG_CMD);
  return goldfish_pipe_convert_status(pipe->command.status);
}

static int goldfish_pipe_command(FAR struct goldfish_pipe_s *pipe, int cmd)
{
  int ret;

  ret = nxmutex_lock(&pipe->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = goldfish_pipe_command_locked(pipe, cmd);
  nxmutex_unlock(&pipe->lock);

  return ret;
}

static int goldfish_pipe_wait(FAR struct goldfish_pipe_s *pipe,
                              bool is_write)
{
  int ret;

  ret = goldfish_pipe_command(pipe, is_write ?
                              GOLDFISH_PIPE_CMD_WAKE_ON_WRITE :
                              GOLDFISH_PIPE_CMD_WAKE_ON_READ);
  if (ret < 0)
    {
      return ret;
    }

  return nxsem_wait(is_write ? &pipe->wait_for_write : &pipe->wait_for_read);
}

static ssize_t goldfish_pipe_transfer_one(FAR struct goldfish_pipe_s *pipe,
                                          FAR char *buffer, size_t buflen,
                                          bool is_write)
{
  ssize_t ret;

  ret = nxmutex_lock(&pipe->lock);
  if (ret < 0)
    {
      return ret;
    }

  pipe->command.rw_params.ptrs[0] = up_addrenv_va_to_pa(buffer);
  pipe->command.rw_params.sizes[0] = buflen;
  pipe->command.rw_params.buffers_count = 1;

  ret = goldfish_pipe_command_locked(pipe,
          is_write ? GOLDFISH_PIPE_CMD_WRITE : GOLDFISH_PIPE_CMD_READ);
  if (ret >= 0)
    {
      ret = pipe->command.rw_params.consumed_size;
    }

  nxmutex_unlock(&pipe->lock);
  return ret;
}

static ssize_t goldfish_pipe_transfer(FAR struct file *filp,
                                      FAR char *buffer, size_t bufflen,
                                      bool is_write)
{
  FAR struct goldfish_pipe_s *pipe = filp->f_priv;
  size_t count = 0;
  int ret = 0;

  while (bufflen > 0)
    {
      /* If the emulator already closed the pipe, no need to go further */

      if (pipe->closed)
        {
          ret = -EIO;
          break;
        }

      ret = goldfish_pipe_transfer_one(pipe, buffer, bufflen, is_write);
      if (ret > 0)
        {
          buffer += ret;
          bufflen -= ret;
          count += ret;
          continue;
        }

      if (ret != -EAGAIN || count || (filp->f_oflags & O_NONBLOCK))
        {
          break;
        }

      ret = goldfish_pipe_wait(pipe, is_write);
      if (ret < 0)
        {
          break;
        }
    }

  return count > 0 ? count : ret;
}

static ssize_t goldfish_pipe_read(FAR struct file *filp, FAR char *buffer,
                                  size_t bufflen)
{
  return goldfish_pipe_transfer(filp, buffer, bufflen, false);
}

static ssize_t goldfish_pipe_write(FAR struct file *filp,
                                   FAR const char *buffer, size_t bufflen)
{
  return goldfish_pipe_transfer(filp, (FAR char *)buffer, bufflen, true);
}

static int goldfish_pipe_poll(FAR struct file *filp,
                              FAR struct pollfd *fds, bool setup)
{
  if (setup)
    {
      FAR struct goldfish_pipe_s *pipe = filp->f_priv;
      FAR struct goldfish_pipe_dev_s *dev = pipe->dev;
      pollevent_t eventset = 0;
      irqstate_t flags;
      int ret;
      int i;

      ret = goldfish_pipe_command(pipe, GOLDFISH_PIPE_CMD_POLL);
      if (ret < 0)
        {
          return ret;
        }

      flags = spin_lock_irqsave(&dev->lock);

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < GOLDFISH_PIPE_MAX_POLL_WAITERS; i++)
        {
          /* Find an available slot */

          if (pipe->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              pipe->fds[i] = fds;
              fds->priv = &pipe->fds[i];
              break;
            }
        }

      spin_unlock_irqrestore(&dev->lock, flags);
      if (i >= GOLDFISH_PIPE_MAX_POLL_WAITERS)
        {
          return -EBUSY;
        }

      if (ret & GOLDFISH_PIPE_POLL_IN)
        {
          eventset |= POLLIN;
        }

      if (ret & GOLDFISH_PIPE_POLL_OUT)
        {
          eventset |= POLLOUT;
        }

      if (ret & GOLDFISH_PIPE_POLL_HUP)
        {
          eventset |= POLLHUP;
        }

      if (pipe->closed)
        {
          eventset |= POLLERR;
        }

      if (eventset)
        {
          poll_notify(&fds, 1, eventset);
        }
      else
        {
          goldfish_pipe_command(pipe, GOLDFISH_PIPE_CMD_WAKE_ON_READ);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      if (slot == NULL)
        {
          return -EINVAL;
        }

      *slot = NULL;
      fds->priv = NULL;
    }

  return 0;
}

static int goldfish_pipe_open(FAR struct file *filep)
{
  FAR struct goldfish_pipe_dev_s *dev = filep->f_inode->i_private;
  FAR struct goldfish_pipe_s *pipe;
  irqstate_t flags;
  int ret = -ENFILE;
  int id;

  pipe = kmm_zalloc(sizeof(*pipe));
  if (pipe == NULL)
    {
      return -ENOMEM;
    }

  pipe->dev = dev;
  nxmutex_init(&pipe->lock);
  nxsem_init(&pipe->wait_for_read, 0, 0);
  nxsem_init(&pipe->wait_for_write, 0, 0);

  flags = spin_lock_irqsave(&dev->lock);

  for (id = 0; id < GOLDFISH_PIPE_MAX_PIPES; id++)
    {
      if (dev->pipes[id] == NULL)
        {
          break;
        }
    }

  if (id >= GOLDFISH_PIPE_MAX_PIPES)
    {
      goto out;
    }

  pipe->id = id;
  pipe->command.id = id;

  /* Now tell the emulator we're opening a new pipe. */

  dev->open_params.command_buffer_ptr = up_addrenv_va_to_pa(&pipe->command);
  dev->open_params.rw_params_max_count = GOLDFISH_PIPE_MAX_COMMAND_BUFFERS;

  ret = goldfish_pipe_command_locked(pipe, GOLDFISH_PIPE_CMD_OPEN);
  if (ret < 0)
    {
      goto out;
    }

  dev->pipes[id] = pipe;
  spin_unlock_irqrestore(&dev->lock, flags);

  filep->f_priv = pipe;
  return 0;

out:
  spin_unlock_irqrestore(&dev->lock, flags);
  nxsem_destroy(&pipe->wait_for_write);
  nxsem_destroy(&pipe->wait_for_read);
  nxmutex_destroy(&pipe->lock);
  kmm_free(pipe);
  return ret;
}

static int goldfish_pipe_close(FAR struct file *filp)
{
  FAR struct goldfish_pipe_s *pipe = filp->f_priv;
  FAR struct goldfish_pipe_dev_s *dev = pipe->dev;
  irqstate_t flags;

  goldfish_pipe_command(pipe, GOLDFISH_PIPE_CMD_CLOSE);

  flags = spin_lock_irqsave(&dev->lock);
  dev->pipes[pipe->id] = NULL;
  spin_unlock_irqrestore(&dev->lock, flags);

  filp->f_priv = NULL;

  nxsem_destroy(&pipe->wait_for_write);
  nxsem_destroy(&pipe->wait_for_read);
  nxmutex_destroy(&pipe->lock);
  kmm_free(pipe);

  return 0;
}

static void goldfish_pipe_wake(FAR struct goldfish_pipe_s *pipe,
                               bool is_write)
{
  FAR sem_t *sem = is_write ? &pipe->wait_for_write : &pipe->wait_for_read;
  int sval;

  while (nxsem_get_value(sem, &sval) >= 0 && sval <= 0)
    {
      nxsem_post(sem);
    }
}

static int goldfish_pipe_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct goldfish_pipe_dev_s *dev = arg;
  irqstate_t irqflags;
  uint32_t count;
  uint32_t i;

  irqflags = spin_lock_irqsave(&dev->lock);

  count = goldfish_pipe_getreg32(dev->base +
                                 GOLDFISH_PIPE_REG_GET_SIGNALLED);
  if (count > GOLDFISH_PIPE_MAX_SIGNALLED)
    {
      count = GOLDFISH_PIPE_MAX_SIGNALLED;
    }

  for (i = 0; i < count; i++)
    {
      uint32_t id = dev->pipes_signalled[i].id;
      uint32_t flags = dev->pipes_signalled[i].flags;
      FAR struct goldfish_pipe_s *pipe = dev->pipes[id];
      pollevent_t eventset = 0;

      if (pipe == NULL)
        {
          continue;
        }

      if (flags & GOLDFISH_PIPE_WAKE_READ)
        {
          eventset |= POLLIN;
        }

      if (flags & GOLDFISH_PIPE_WAKE_WRITE)
        {
          eventset |= POLLOUT;
        }

      if (flags & GOLDFISH_PIPE_WAKE_CLOSED)
        {
          eventset |= POLLERR;
          dev->pipes[id]->closed = true;
        }

      if (eventset & (POLLIN | POLLERR))
        {
          goldfish_pipe_wake(pipe, false);
        }

      if (eventset & (POLLOUT | POLLERR))
        {
          goldfish_pipe_wake(pipe, true);
        }

      if (eventset)
        {
          poll_notify(pipe->fds, GOLDFISH_PIPE_MAX_POLL_WAITERS, eventset);
        }
    }

  spin_unlock_irqrestore(&dev->lock, irqflags);
  return 0;
}

static void goldfish_pipe_write_addr(FAR void *addr,
                                     FAR void *portl, FAR void *porth)
{
  uintptr_t paddr = up_addrenv_va_to_pa(addr);
  goldfish_pipe_putreg32((paddr >> 16) >> 16, porth);
  goldfish_pipe_putreg32(paddr, portl);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_pipe_register
 *
 * Description:
 *   register /dev/goldfish_pipe device
 *
 ****************************************************************************/

int goldfish_pipe_register(FAR void *base, int irq)
{
  FAR struct goldfish_pipe_dev_s *dev;
  uint32_t version;
  int ret = -ENOTSUP;

  /* Allocate and initialize a new device structure instance */

  dev = (FAR struct goldfish_pipe_dev_s *)kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  spin_lock_init(&dev->lock);
  dev->base = (FAR uint8_t *)base;

  /* Exchange the versions with the host device */

  goldfish_pipe_putreg32(GOLDFISH_PIPE_DRIVER_VERSION,
                         dev->base + GOLDFISH_PIPE_REG_VERSION);
  version = goldfish_pipe_getreg32(dev->base + GOLDFISH_PIPE_REG_VERSION);
  if (version < GOLDFISH_PIPE_CURRENT_DEVICE_VERSION)
    {
      goto out;
    }

  ret = irq_attach(irq, goldfish_pipe_interrupt, dev);
  if (ret < 0)
    {
      goto out;
    }

  up_enable_irq(irq);

  /* Send the buffer addresses to the host */

  goldfish_pipe_write_addr(&dev->pipes_signalled,
                           dev->base + GOLDFISH_PIPE_REG_SIGNAL_BUFFER,
                           dev->base + GOLDFISH_PIPE_REG_SIGNAL_BUFFER_HIGH);

  goldfish_pipe_putreg32(GOLDFISH_PIPE_MAX_SIGNALLED,
                         dev->base + GOLDFISH_PIPE_REG_SIGNAL_BUFFER_COUNT);

  goldfish_pipe_write_addr(&dev->open_params,
                           dev->base + GOLDFISH_PIPE_REG_OPEN_BUFFER,
                           dev->base + GOLDFISH_PIPE_REG_OPEN_BUFFER_HIGH);

  /* Register the pipe device */

  return register_driver("/dev/goldfish_pipe",
                         &g_goldfish_pipe_fops, 0666, dev);

out:
  kmm_free(dev);
  return ret;
}
