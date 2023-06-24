/****************************************************************************
 * drivers/misc/goldfish_pipe.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/epoll.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/spinlock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/bits.h>
#include "goldfish_pipe_qemu.h"

#define putreg32(v, x) (*(volatile uint32_t*)(x) = (v))
#define getreg32(x) (*(uint32_t *)(x))

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)(n))

#define BIT(nr) (1 << (nr))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Update this when something changes in the driver's behavior so the host
 * can benefit from knowing it
 *
 ****************************************************************************/

enum
{
  PIPE_DRIVER_VERSION = 4,
  PIPE_CURRENT_DEVICE_VERSION = 2
};

enum
{
  MAX_BUFFERS_PER_COMMAND = 336,
  MAX_SIGNALLED_PIPES = 64,
  INITIAL_PIPES_CAPACITY = 64
};

struct goldfish_pipe_dev;

/* A per-pipe command structure, shared with the host */

struct goldfish_pipe_command
{
  int cmd;        /* PipeCmdCode, guest -> host */
  int id;         /* pipe id, guest -> host */
  int status;     /* command execution status, host -> guest */
  int reserved;   /* to pad to 64-bit boundary */
  union
    {
      /* Parameters for PIPE_CMD_{READ,WRITE} */

      struct
        {
          /* number of buffers, guest -> host */

          uint32_t buffers_count;

          /* number of consumed bytes, host -> guest */

          int consumed_size;

          /* buffer pointers, guest -> host */

          uint64_t ptrs[MAX_BUFFERS_PER_COMMAND];

          /* buffer sizes, guest -> host */

          uint32_t sizes[MAX_BUFFERS_PER_COMMAND];
        } rw_params;
    };
};

/* A single signalled pipe information */

struct signalled_pipe_buffer
{
  uint32_t id;
  uint32_t flags;
};

/* Parameters for the PIPE_CMD_OPEN command */

struct open_command_param
{
  uint64_t command_buffer_ptr;
  uint32_t rw_params_max_count;
};

/* Device-level set of buffers shared with the host */

struct goldfish_pipe_dev_buffers
{
  struct open_command_param open_command_params;
  struct signalled_pipe_buffer
  signalled_pipe_buffers[MAX_SIGNALLED_PIPES];
};

/* This data type models a given pipe instance */

struct goldfish_pipe
{
  /* pipe ID - index into goldfish_pipe_dev::pipes array */

  uint32_t id;

  /* The wake flags pipe is waiting for
   * Note: not protected with any lock, uses atomic operations
   *  and barriers to make it thread-safe.
   */

  unsigned long flags;

  /* wake flags host have signalled,
   *  - protected by goldfish_pipe_dev::lock
   */

  unsigned long signalled_flags;

  /* A pointer to command buffer */

  struct goldfish_pipe_command *command_buffer;

  /* doubly linked list of signalled pipes, protected by
   * goldfish_pipe_dev::lock
   */

  struct goldfish_pipe *prev_signalled;
  struct goldfish_pipe *next_signalled;

  /* A pipe's own lock. Protects the following:
   *  - *command_buffer - makes sure a command can safely write its
   *    parameters to the host and read the results back.
   */

  mutex_t lock;

  /* A wake queue for sleeping until host signals an event */

  sem_t wake_queue;

  /* Pointer to the parent goldfish_pipe_dev instance */

  struct goldfish_pipe_dev *dev;
};

/****************************************************************************
 * The global driver data. Holds a reference to the i/o page used to
 * communicate with the emulator, and a wake queue for blocked tasks
 * waiting to be awoken.
 *
 ****************************************************************************/

struct goldfish_pipe_dev
{
  /* Global device spinlock. Protects the following members:
   *  - pipes, pipes_capacity
   *  - [*pipes, *pipes + pipes_capacity) - array data
   *  - first_signalled_pipe,
   *      goldfish_pipe::prev_signalled,
   *      goldfish_pipe::next_signalled,
   *      goldfish_pipe::signalled_flags - all singnalled-related fields,
   *                                       in all allocated pipes
   *  - open_command_params - PIPE_CMD_OPEN-related buffers
   *
   * It looks like a lot of different fields, but the trick is that
   * the only operation that happens often is the signalled pipes array
   * manipulation. That's why it's OK for now to keep the rest of the
   * fields under the same lock. If we notice too much contention because
   * of PIPE_CMD_OPEN, then we should add a separate lock there.
   */

  spinlock_t lock;
  mutex_t polllock;

  /* Array of the pipes of |pipes_capacity| elements,
   * indexed by goldfish_pipe::id
   */

  struct goldfish_pipe **pipes;
  uint32_t pipes_capacity;

  /* Pointers to the buffers host uses for interaction with this driver */

  struct goldfish_pipe_dev_buffers *buffers;

  /* Head of a doubly linked list of signalled pipes */

  struct goldfish_pipe *first_signalled_pipe;

  /* ptr to platform device's device struct */

  struct device *pdev_dev;

  /* Some device-specific data */

  int irq;
  int version;
  unsigned char *base;

  struct work_s work;

  struct pollfd **fds;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int goldfish_pipe_cmd_locked(struct goldfish_pipe *pipe,
                                    enum pipecmdcode cmd)
{
  pipe->command_buffer->cmd = cmd;

  /* failure by default */

  pipe->command_buffer->status = PIPE_ERROR_INVAL;
  putreg32(pipe->id, pipe->dev->base + PIPE_REG_CMD);
  return pipe->command_buffer->status;
}

static int goldfish_pipe_cmd(struct goldfish_pipe *pipe,
                             enum pipecmdcode cmd)
{
  int status;

  if (nxmutex_lock(&pipe->lock))
    {
      return PIPE_ERROR_IO;
    }

  status = goldfish_pipe_cmd_locked(pipe, cmd);
  nxmutex_unlock(&pipe->lock);

  return status;
}

/****************************************************************************
 *
 * This function converts an error code returned by the emulator through
 * the PIPE_REG_STATUS i/o register into a valid negative errno value.
 *
 ****************************************************************************/

static int goldfish_pipe_error_convert(int status)
{
  switch (status)
    {
      case PIPE_ERROR_AGAIN:
        return -EAGAIN;
      case PIPE_ERROR_NOMEM:
        return -ENOMEM;
      case PIPE_ERROR_IO:
        return -EIO;
      default:
        return -EINVAL;
    }
}

/* Populate the call parameters, merging adjacent pages together */

static void populate_rw_params(unsigned long address,
                               unsigned long address_end,
                               struct goldfish_pipe_command *command)
{
  command->rw_params.ptrs[0] = (uint64_t)address;
  command->rw_params.sizes[0] = address_end - address;
  command->rw_params.buffers_count = 1;
}

static int transfer_max_buffers(struct goldfish_pipe *pipe,
                                unsigned long address,
                                unsigned long address_end,
                                int is_write,
                                int *consumed_size,
                                int *status)
{
  /* Serialize access to the pipe command buffers */

  if (nxmutex_lock(&pipe->lock))
    {
      return -ERESTART;
    }

  populate_rw_params(address, address_end,
                     pipe->command_buffer);

  /* Transfer the data */

  *status = goldfish_pipe_cmd_locked(pipe,
                                     is_write ?
                                     PIPE_CMD_WRITE :
                                     PIPE_CMD_READ);

  *consumed_size = pipe->command_buffer->rw_params.consumed_size;

  nxmutex_unlock(&pipe->lock);

  return 0;
}

static int wait_for_host_signal(struct goldfish_pipe *pipe, int is_write)
{
  uint32_t wake_bit = is_write ? BIT_WAKE_ON_WRITE : BIT_WAKE_ON_READ;

  pipe->flags |= BIT(wake_bit);

  /* Tell the emulator we're going to wait for a wake event */

  goldfish_pipe_cmd(pipe,
                    is_write ?
                    PIPE_CMD_WAKE_ON_WRITE :
                    PIPE_CMD_WAKE_ON_READ);

  while (BIT(wake_bit) & pipe->flags)
    {
      if (nxsem_wait(&pipe->wake_queue))
        {
          return -ERESTART;
        }

      if (BIT(BIT_CLOSED_ON_HOST) & pipe->flags)
        {
          return -EIO;
        }
    }

  return 0;
}

static ssize_t goldfish_pipe_read_write(struct file *filp,
                                        char *buffer,
                                        size_t bufflen,
                                        int is_write)
{
  struct goldfish_pipe *pipe = filp->f_priv;
  int count = 0;
  int ret = -EINVAL;
  unsigned long address;
  unsigned long address_end;

  /* If the emulator already closed the pipe, no need to go further */

  if (BIT(BIT_CLOSED_ON_HOST) & pipe->flags)
    {
      return -EIO;
    }

  /* Null reads or writes succeeds */

  if (bufflen == 0)
    {
      return 0;
    }

  address = (unsigned long)buffer;
  address_end = address + bufflen;

  while (address < address_end)
    {
      int consumed_size;
      int status;

      ret = transfer_max_buffers(pipe, address, address_end, is_write,
                                 &consumed_size, &status);
      if (ret < 0)
        break;

      if (consumed_size > 0)
        {
          /* No matter what's the status, we've transferred something. */

          count += consumed_size;
          address += consumed_size;
        }

      if (status > 0)
        continue;

      if (status == 0)
        {
          /* EOF */

          ret = 0;
          break;
        }

      if (count > 0)
        {
          /* An error occurred, but we already transferred
           * something on one of the previous iterations.
           * Just return what we already copied and log this
           * err.
           */

          if (status != PIPE_ERROR_AGAIN)
            syslog(LOG_INFO, "backend error %d on %s\n",
                   status, is_write ? "write" : "read");
          break;
        }

      /* If the error is not PIPE_ERROR_AGAIN, or if we are in
       * non-blocking mode, just return the error code.
       */

      if (status != PIPE_ERROR_AGAIN || (filp->f_oflags & O_NONBLOCK) != 0)
        {
          ret = goldfish_pipe_error_convert(status);
          break;
        }

      status = wait_for_host_signal(pipe, is_write);

      if (status < 0)
        {
          return status;
        }
    }

  if (count > 0)
    {
      return count;
    }

  return ret;
}

static ssize_t goldfish_pipe_read(FAR struct file *filp, FAR char *buffer,
                                  size_t bufflen)
{
  return goldfish_pipe_read_write(filp, buffer, bufflen,
                                  /* is_write */ 0);
}

static ssize_t goldfish_pipe_write(FAR struct file *filp,
                                   FAR const char *buffer, size_t bufflen)
{
  /* cast away the const */

  char *no_const_buffer = (char *)buffer;

  return goldfish_pipe_read_write(filp, no_const_buffer, bufflen,
                                  /* is_write */ 1);
}

static int goldfish_pipe_poll(FAR struct file *filp,
                              FAR struct pollfd *fds, bool setup)
{
  FAR struct goldfish_pipe *pipe = filp->f_priv;
  FAR struct inode *inode = filp->f_inode;
  FAR struct goldfish_pipe_dev *dev = inode->i_private;

  pollevent_t mask = 0;
  int status;
  int i;
  int ret;

  ret = nxmutex_lock(&dev->polllock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      for (i = 0; i < dev->pipes_capacity; i++)
        {
          if (!dev->fds[i])
            {
              dev->fds[i] = fds;
              fds->priv = &dev->fds[i];
              break;
            }
        }

      if (i >= dev->pipes_capacity)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto errout;
        }

      status = goldfish_pipe_cmd(pipe, PIPE_CMD_POLL);
      if (status < 0)
        {
          return -ERESTART;
        }

      if (status & PIPE_POLL_IN)
        mask |= EPOLLIN | EPOLLRDNORM;
      if (status & PIPE_POLL_OUT)
        mask |= EPOLLOUT | EPOLLWRNORM;
      if (status & PIPE_POLL_HUP)
        mask |= EPOLLHUP;
      if (BIT(BIT_CLOSED_ON_HOST) & pipe->flags)
        mask |= EPOLLERR;

      if (mask)
        poll_notify(dev->fds, 1, mask);
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  nxmutex_unlock(&dev->polllock);
  return ret;
}

static void signalled_pipes_add_locked(struct goldfish_pipe_dev *dev,
                                       uint32_t id, uint32_t flags)
{
  struct goldfish_pipe *pipe;

  if (id >= dev->pipes_capacity)
    {
      return;
    }

  pipe = dev->pipes[id];
  if (!pipe)
    {
      return;
    }

  pipe->signalled_flags |= flags;

  if (pipe->prev_signalled || pipe->next_signalled ||
      dev->first_signalled_pipe == pipe)
    {
      /* already in the list */

      return;
    }

  pipe->next_signalled = dev->first_signalled_pipe;

  if (dev->first_signalled_pipe)
    dev->first_signalled_pipe->prev_signalled = pipe;

  dev->first_signalled_pipe = pipe;
}

static void signalled_pipes_remove_locked(struct goldfish_pipe_dev *dev,
                                          struct goldfish_pipe *pipe)
{
  if (pipe->prev_signalled)
    pipe->prev_signalled->next_signalled = pipe->next_signalled;

  if (pipe->next_signalled)
    pipe->next_signalled->prev_signalled = pipe->prev_signalled;

  if (pipe == dev->first_signalled_pipe)
    dev->first_signalled_pipe = pipe->next_signalled;

  pipe->prev_signalled = NULL;
  pipe->next_signalled = NULL;
}

static struct goldfish_pipe
*signalled_pipes_pop_front(struct goldfish_pipe_dev *dev, int *wakes)
{
  struct goldfish_pipe *pipe;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  pipe = dev->first_signalled_pipe;

  if (pipe)
    {
      *wakes = pipe->signalled_flags;
      pipe->signalled_flags = 0;

      /* This is an optimized version of
       * signalled_pipes_remove_locked()
       * - We want to make it as fast as possible to
       * wake the sleeping pipe operations faster.
       */

      dev->first_signalled_pipe = pipe->next_signalled;
      if (dev->first_signalled_pipe)
        dev->first_signalled_pipe->prev_signalled = NULL;
      pipe->next_signalled = NULL;
    }

  spin_unlock_irqrestore(&dev->lock, flags);

  return pipe;
}

static void goldfish_interrupt_task(FAR void *arg)
{
  /* Iterate over the signalled pipes and wake them one by one */

  struct goldfish_pipe_dev *dev = arg;
  struct goldfish_pipe *pipe;
  int wakes;

  while ((pipe = signalled_pipes_pop_front(dev, &wakes)) != NULL)
    {
      if (wakes & PIPE_WAKE_CLOSED)
        {
          pipe->flags = 1 << BIT_CLOSED_ON_HOST;
        }
      else
        {
          if (wakes & PIPE_WAKE_READ)
            pipe->flags &= ~BIT(BIT_WAKE_ON_READ);
          if (wakes & PIPE_WAKE_WRITE)
            pipe->flags &= ~BIT(BIT_WAKE_ON_WRITE);
        }

      /* wake_up_interruptible() implies a write barrier, so don't
       * explicitly add another one here.
       */

      nxsem_post(&pipe->wake_queue);
    }
}

/****************************************************************************
 * The general idea of the (threaded) interrupt handling:
 *
 * 1.device raises an interrupt if there's at least one signalled pipe
 * 2.IRQ handler reads the signalled pipes and their count from the device
 * 3.device writes them into a shared buffer and returns the count
 *     it only resets the IRQ if it has returned all signalled pipes,
 *     otherwise it leaves it raised, so IRQ handler will be called
 *     again for the next chunk
 * 4.IRQ handler adds all returned pipes to the device's signalled pipes list
 * 5.IRQ handler defers processing the signalled pipes from the list in a
 *     separate context
 *
 ****************************************************************************/

static int goldfish_pipe_interrupt(int irq, void *dev_id, void *arg)
{
  uint32_t count;
  uint32_t i;
  irqstate_t flags;
  uint32_t signalled_id;
  uint32_t signalled_flags;
  struct goldfish_pipe_dev *dev = arg;

  /* Request the signalled pipes from the device */

  flags = spin_lock_irqsave(&dev->lock);

  count = getreg32(dev->base + PIPE_REG_GET_SIGNALLED);

  if (count == 0)
    {
      spin_unlock_irqrestore(&dev->lock, flags);
      return OK;
    }

  if (count > MAX_SIGNALLED_PIPES)
    count = MAX_SIGNALLED_PIPES;

  for (i = 0; i < count; ++i)
    {
      signalled_id = dev->buffers->signalled_pipe_buffers[i].id;
      signalled_flags = dev->buffers->signalled_pipe_buffers[i].flags;
      signalled_pipes_add_locked(dev, signalled_id, signalled_flags);
    }

  spin_unlock_irqrestore(&dev->lock, flags);

  work_queue(HPWORK, &dev->work, goldfish_interrupt_task,
             dev, MSEC2TICK(20));

  return OK;
}

static int get_free_pipe_id_locked(struct goldfish_pipe_dev *dev)
{
  int id;

  for (id = 0; id < dev->pipes_capacity; ++id)
    {
      if (!dev->pipes[id])
        {
          return id;
        }
    }

  /* Reallocate the array.
   * Since get_free_pipe_id_locked runs with interrupts disabled,
   * we don't want to make calls that could lead to sleep.
   */

  uint32_t new_capacity = 2 * dev->pipes_capacity;
  struct goldfish_pipe **pipes = kmm_calloc(new_capacity, sizeof(*pipes));
  struct pollfd **fds = kmm_calloc(new_capacity, sizeof(*fds));
  if (!pipes || !fds)
    {
      return -ENOMEM;
    }

  memcpy(pipes, dev->pipes, sizeof(*pipes) * dev->pipes_capacity);
  kmm_free(dev->pipes);
  dev->pipes = pipes;

  memcpy(fds, dev->fds, sizeof(*fds) * dev->pipes_capacity);
  kmm_free(dev->fds);
  dev->fds = fds;

  id = dev->pipes_capacity;
  dev->pipes_capacity = new_capacity;

  return id;
}

/**
 * goldfish_pipe_open - open a channel to the AVD
 * @inode: inode of device
 * @file: file struct of opener
 *
 * Create a new pipe link between the emulator and the use application.
 * Each new request produces a new pipe.
 *
 * Note: we use the pipe ID as a mux. All goldfish emulations are 32bit
 * right now so this is fine. A move to 64bit will need this addressing
 */

static int goldfish_pipe_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct goldfish_pipe_dev *dev = inode->i_private;
  irqstate_t flags;
  int id;
  int status;

  uint64_t *ptr;
  uint32_t  *count;

  /* Allocate new pipe kernel object */

  struct goldfish_pipe *pipe = kmm_zalloc(sizeof(*pipe));

  if (!pipe)
    {
      return -ENOMEM;
    }

  pipe->dev = dev;
  nxmutex_init(&pipe->lock);
  nxsem_init(&pipe->wake_queue, 0, 0);

  /* Command buffer needs to be allocated on its own page to make sure
   * it is physically contiguous in host's address space.
   */

  pipe->command_buffer = (struct goldfish_pipe_command *)
                          kmm_zalloc(sizeof(struct goldfish_pipe_command));
  if (!pipe->command_buffer)
    {
      status = -ENOMEM;
      goto err_pipe;
    }

  flags = spin_lock_irqsave(&dev->lock);

  id = get_free_pipe_id_locked(dev);
  if (id < 0)
    {
      status = id;
      goto err_id_locked;
    }

  dev->pipes[id] = pipe;
  pipe->id = id;
  pipe->command_buffer->id = id;

  /* Now tell the emulator we're opening a new pipe. */

  count = &dev->buffers->open_command_params.rw_params_max_count;
  ptr = &dev->buffers->open_command_params.command_buffer_ptr;
  *count = MAX_BUFFERS_PER_COMMAND;
  *ptr = (uint64_t)pipe->command_buffer;

  status = goldfish_pipe_cmd_locked(pipe, PIPE_CMD_OPEN);
  spin_unlock_irqrestore(&dev->lock, flags);

  if (status < 0)
    {
      goto err_cmd;
    }

  /* All is done, save the pipe into the file's private data field */

  filep->f_priv = pipe;

  return 0;

err_cmd:
  flags = spin_lock_irqsave(&dev->lock);
  dev->pipes[id] = NULL;

err_id_locked:
  spin_unlock_irqrestore(&dev->lock, flags);
  kmm_free(pipe->command_buffer);

err_pipe:
  kmm_free(pipe);
  return status;
}

static int goldfish_pipe_release(FAR struct file *filp)
{
  irqstate_t flags;
  struct goldfish_pipe *pipe = filp->f_priv;
  struct goldfish_pipe_dev *dev = pipe->dev;

  /* The guest is closing the channel, so tell the emulator right now */

  goldfish_pipe_cmd(pipe, PIPE_CMD_CLOSE);

  flags = spin_lock_irqsave(&dev->lock);
  dev->pipes[pipe->id] = NULL;
  signalled_pipes_remove_locked(dev, pipe);
  spin_unlock_irqrestore(&dev->lock, flags);

  filp->f_priv = NULL;
  kmm_free(pipe->command_buffer);
  kmm_free(pipe);

  return 0;
}

static const struct file_operations g_goldfishpipe_fops =
{
  .read = goldfish_pipe_read,
  .write = goldfish_pipe_write,
  .poll = goldfish_pipe_poll,
  .open = goldfish_pipe_open,
  .close = goldfish_pipe_release,
};

static void write_pa_addr(void *addr, void *portl, void *porth)
{
  putreg32(upper_32_bits((uint64_t)addr), porth);
  putreg32(lower_32_bits((uint64_t)addr), portl);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfishpipe_register
 *
 * Description:
 *   register /dev/goldfish_pipe device
 *
 ****************************************************************************/

int goldfish_pipe_register(void *base, int irq)
{
  FAR struct goldfish_pipe_dev *dev;
  int ret;

  /* Allocate and initialize a new device structure instance */

  dev = (FAR struct goldfish_pipe_dev *)kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->base = (unsigned char *)base;
  dev->irq = irq;

  nxmutex_init(&dev->polllock);
  spin_initialize(&dev->lock, 0);

  putreg32(PIPE_DRIVER_VERSION, dev->base + PIPE_REG_VERSION);
  dev->version = getreg32(dev->base + PIPE_REG_VERSION);
  if (dev->version < PIPE_CURRENT_DEVICE_VERSION)
    {
      return -EINVAL;
    }

  ret = irq_attach(dev->irq, goldfish_pipe_interrupt, dev);
  if (ret < 0)
    {
      syslog(LOG_INFO, "attach irq failed\n");
      return ret;
    }

  up_enable_irq(dev->irq);

  dev->first_signalled_pipe = NULL;
  dev->pipes_capacity = INITIAL_PIPES_CAPACITY;
  dev->pipes = kmm_calloc(dev->pipes_capacity, sizeof(*dev->pipes));

  if (!dev->pipes)
    {
      return -ENOMEM;
    }

  /* We're going to pass two buffers, open_command_params and
   * signalled_pipe_buffers, to the host. This means each of those buffers
   * needs to be contained in a single physical page. The easiest choice
   * is to just allocate a page and place the buffers in it.
   */

  dev->buffers = (struct goldfish_pipe_dev_buffers *)
                  kmm_zalloc(sizeof(struct goldfish_pipe_dev_buffers));

  if (!dev->buffers)
    {
      return -ENOMEM;
    }

  /* Send the buffer addresses to the host */

  write_pa_addr(&dev->buffers->signalled_pipe_buffers,
                dev->base + PIPE_REG_SIGNAL_BUFFER,
                dev->base + PIPE_REG_SIGNAL_BUFFER_HIGH);

  putreg32(MAX_SIGNALLED_PIPES,
           dev->base + PIPE_REG_SIGNAL_BUFFER_COUNT);

  write_pa_addr(&dev->buffers->open_command_params,
                dev->base + PIPE_REG_OPEN_BUFFER,
                dev->base + PIPE_REG_OPEN_BUFFER_HIGH);

  /* Register the pipe device */

  return register_driver("/dev/goldfish_pipe", &g_goldfishpipe_fops,
                         0666,
                         (FAR void *)dev);
}
