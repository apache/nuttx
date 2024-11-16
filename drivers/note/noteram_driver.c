/****************************************************************************
 * drivers/note/noteram_driver.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sched.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <poll.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/kmalloc.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/fs/fs.h>
#include <nuttx/streams.h>

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
#  ifdef CONFIG_LIB_SYSCALL
#    include <syscall.h>
#  else
#    define CONFIG_LIB_SYSCALL
#    include <syscall.h>
#    undef CONFIG_LIB_SYSCALL
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NCPUS CONFIG_SMP_NCPUS

/* Renumber idle task PIDs
 *  In NuttX, PID number less than NCPUS are idle tasks.
 *  In Linux, there is only one idle task of PID 0.
 */

#define get_pid(pid) ((pid) < NCPUS ? 0 : (pid))

#define get_task_state(s)                                                    \
  ((s) == 0 ? 'X' : ((s) <= LAST_READY_TO_RUN_STATE ? 'R' : 'S'))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct noteram_driver_s
{
  struct note_driver_s driver;
  FAR uint8_t *ni_buffer;
  size_t ni_bufsize;
  unsigned int ni_overwrite;
  volatile unsigned int ni_head;
  volatile unsigned int ni_tail;
  volatile unsigned int ni_read;
  spinlock_t lock;
  FAR struct pollfd *pfd;
};

/* The structure to hold the context data of trace dump */

struct noteram_dump_cpu_context_s
{
  int intr_nest;            /* Interrupt nest level */
  bool pendingswitch;       /* sched_switch pending flag */
  int current_state;        /* Task state of the current line */
  pid_t current_pid;        /* Task PID of the current line */
  pid_t next_pid;           /* Task PID of the next line */
  uint8_t current_priority; /* Task Priority of the current line */
  uint8_t next_priority;    /* Task Priority of the next line */
};

struct noteram_dump_context_s
{
  struct noteram_dump_cpu_context_s cpu[NCPUS];
  unsigned int mode;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int noteram_open(FAR struct file *filep);
static int noteram_close(FAR struct file *filep);
static ssize_t noteram_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen);
static int noteram_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int noteram_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
static void noteram_add(FAR struct note_driver_s *drv,
                        FAR const void *note, size_t len);
static void
noteram_dump_init_context(FAR struct noteram_dump_context_s *ctx);
static int noteram_dump_one(FAR uint8_t *p, FAR struct lib_outstream_s *s,
                            FAR struct noteram_dump_context_s *ctx);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_noteram_fops =
{
  noteram_open,  /* open */
  noteram_close, /* close */
  noteram_read,  /* read */
  NULL,          /* write */
  NULL,          /* seek */
  noteram_ioctl, /* ioctl */
  NULL,          /* mmap */
  NULL,          /* truncate */
  noteram_poll,  /* poll */
};

static
#ifdef DRIVERS_NOTERAM_SECTION
locate_data(DRIVERS_NOTERAM_SECTION)
#endif
uint8_t g_ramnote_buffer[CONFIG_DRIVERS_NOTERAM_BUFSIZE];

static const struct note_driver_ops_s g_noteram_ops =
{
  noteram_add
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct noteram_driver_s g_noteram_driver =
{
  {
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
    "ram",
    {
      {
        CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#  ifdef CONFIG_SMP
        CONFIG_SCHED_INSTRUMENTATION_CPUSET
#  endif
      },
    },
#endif
    &g_noteram_ops
  },
  g_ramnote_buffer,
  CONFIG_DRIVERS_NOTERAM_BUFSIZE,
#ifdef CONFIG_DRIVERS_NOTERAM_DEFAULT_NOOVERWRITE
  NOTERAM_MODE_OVERWRITE_DISABLE
#else
  NOTERAM_MODE_OVERWRITE_ENABLE
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noteram_buffer_clear
 *
 * Description:
 *   Clear all contents of the circular buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void noteram_buffer_clear(FAR struct noteram_driver_s *drv)
{
  drv->ni_tail = drv->ni_head;
  drv->ni_read = drv->ni_head;

  if (drv->ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
      drv->ni_overwrite = NOTERAM_MODE_OVERWRITE_DISABLE;
    }
}

/****************************************************************************
 * Name: noteram_next
 *
 * Description:
 *   Return the circular buffer index at offset from the specified index
 *   value, handling wraparound
 *
 * Input Parameters:
 *   ndx - Old circular buffer index
 *
 * Returned Value:
 *   New circular buffer index
 *
 ****************************************************************************/

static inline unsigned int noteram_next(FAR struct noteram_driver_s *drv,
                                        unsigned int ndx,
                                        unsigned int offset)
{
  ndx += offset;
  if (ndx >= drv->ni_bufsize)
    {
      ndx -= drv->ni_bufsize;
    }

  return ndx;
}

/****************************************************************************
 * Name: noteram_length
 *
 * Description:
 *   Length of data currently in circular buffer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Length of data currently in circular buffer.
 *
 ****************************************************************************/

static unsigned int noteram_length(FAR struct noteram_driver_s *drv)
{
  unsigned int head = drv->ni_head;
  unsigned int tail = drv->ni_tail;

  if (tail > head)
    {
      head += drv->ni_bufsize;
    }

  return head - tail;
}

/****************************************************************************
 * Name: noteram_unread_length
 *
 * Description:
 *   Length of unread data currently in circular buffer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Length of unread data currently in circular buffer.
 *
 ****************************************************************************/

static unsigned int noteram_unread_length(FAR struct noteram_driver_s *drv)
{
  unsigned int head = drv->ni_head;
  unsigned int read = drv->ni_read;

  if (read > head)
    {
      head += drv->ni_bufsize;
    }

  return head - read;
}

/****************************************************************************
 * Name: noteram_remove
 *
 * Description:
 *   Remove the variable length note from the tail of the circular buffer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

static void noteram_remove(FAR struct noteram_driver_s *drv)
{
  unsigned int tail;
  unsigned int length;

  /* Get the tail index of the circular buffer */

  tail = drv->ni_tail;
  DEBUGASSERT(tail < drv->ni_bufsize);

  /* Get the length of the note at the tail index */

  length = NOTE_ALIGN(drv->ni_buffer[tail]);
  DEBUGASSERT(length <= noteram_length(drv));

  /* Increment the tail index to remove the entire note from the circular
   * buffer.
   */

  if (drv->ni_read == drv->ni_tail)
    {
      /* The read index also needs increment. */

      drv->ni_read = noteram_next(drv, tail, length);
    }

  drv->ni_tail = noteram_next(drv, tail, length);
}

/****************************************************************************
 * Name: noteram_get
 *
 * Description:
 *   Get the next note from the read index of the circular buffer.
 *
 * Input Parameters:
 *   buffer - Location to return the next note
 *   buflen - The length of the user provided buffer.
 *
 * Returned Value:
 *   On success, the positive, non-zero length of the return note is
 *   provided.  Zero is returned only if the circular buffer is empty.  A
 *   negated errno value is returned in the event of any failure.
 *
 ****************************************************************************/

static ssize_t noteram_get(FAR struct noteram_driver_s *drv,
                           FAR uint8_t *buffer, size_t buflen)
{
  FAR struct note_common_s *note;
  unsigned int remaining;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

  DEBUGASSERT(buffer != NULL);

  /* Verify that the circular buffer is not empty */

  circlen = noteram_unread_length(drv);
  if (circlen <= 0)
    {
      return 0;
    }

  /* Get the read index of the circular buffer */

  read = drv->ni_read;
  DEBUGASSERT(read < drv->ni_bufsize);

  /* Get the length of the note at the read index */

  note = (FAR struct note_common_s *)&drv->ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

  /* Is the user buffer large enough to hold the note? */

  if (buflen < notelen)
    {
      /* Skip the large note so that we do not get constipated. */

      drv->ni_read = noteram_next(drv, read, NOTE_ALIGN(notelen));

      /* and return an error */

      return -EFBIG;
    }

  /* Loop until the note has been transferred to the user buffer */

  remaining = (unsigned int)notelen;
  while (remaining > 0)
    {
      /* Copy the next byte at the read index */

      *buffer++ = drv->ni_buffer[read];

      /* Adjust indices and counts */

      read = noteram_next(drv, read, 1);
      remaining--;
    }

  drv->ni_read = noteram_next(drv, drv->ni_read, NOTE_ALIGN(notelen));

  return notelen;
}

/****************************************************************************
 * Name: noteram_open
 ****************************************************************************/

static int noteram_open(FAR struct file *filep)
{
  FAR struct noteram_dump_context_s *ctx;
  FAR struct noteram_driver_s *drv = (FAR struct noteram_driver_s *)
                                     filep->f_inode->i_private;

  /* Reset the read index of the circular buffer */

  drv->ni_read = drv->ni_tail;
  ctx = kmm_zalloc(sizeof(*ctx));
  if (ctx == NULL)
    {
      return -ENOMEM;
    }

  filep->f_priv = ctx;
  noteram_dump_init_context(ctx);
  return OK;
}

int noteram_close(FAR struct file *filep)
{
  FAR struct noteram_dump_context_s *ctx = filep->f_priv;

  kmm_free(ctx);
  return OK;
}

/****************************************************************************
 * Name: noteram_read
 ****************************************************************************/

static ssize_t noteram_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct noteram_dump_context_s *ctx = filep->f_priv;
  FAR struct noteram_driver_s *drv = filep->f_inode->i_private;
  FAR struct lib_memoutstream_s stream;
  ssize_t ret;
  irqstate_t flags;

  if (ctx->mode == NOTERAM_MODE_READ_BINARY)
    {
      flags = spin_lock_irqsave_wo_note(&drv->lock);
      ret = noteram_get(drv, (FAR uint8_t *)buffer, buflen);
      spin_unlock_irqrestore_wo_note(&drv->lock, flags);
    }
  else
    {
      lib_memoutstream(&stream, buffer, buflen);

      do
        {
          uint8_t note[256];

          /* Get the next note (removing it from the buffer) */

          flags = spin_lock_irqsave_wo_note(&drv->lock);
          ret = noteram_get(drv, note, sizeof(note));
          spin_unlock_irqrestore_wo_note(&drv->lock, flags);
          if (ret <= 0)
            {
              return ret;
            }

          /* Parse notes into text format */

          ret = noteram_dump_one(note, (FAR struct lib_outstream_s *)&stream,
                                 ctx);
        }
      while (ret == 0);
    }

  return ret;
}

/****************************************************************************
 * Name: noteram_ioctl
 ****************************************************************************/

static int noteram_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOSYS;
  FAR struct noteram_driver_s *drv = filep->f_inode->i_private;
  irqstate_t flags = spin_lock_irqsave_wo_note(&drv->lock);

  /* Handle the ioctl commands */

  switch (cmd)
    {
      /* NOTERAM_CLEAR
       *      - Clear all contents of the circular buffer
       *        Argument: Ignored
       */

      case NOTERAM_CLEAR:
        noteram_buffer_clear(drv);
        ret = OK;
        break;

      /* NOTERAM_GETMODE
       *      - Get overwrite mode
       *        Argument: A writable pointer to unsigned int
       */

      case NOTERAM_GETMODE:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            *(FAR unsigned int *)arg = drv->ni_overwrite;
            ret = OK;
          }
        break;

      /* NOTERAM_SETMODE
       *      - Set overwrite mode
       *        Argument: A read-only pointer to unsigned int
       */

      case NOTERAM_SETMODE:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            drv->ni_overwrite = *(FAR unsigned int *)arg;
            ret = OK;
          }
        break;

      /* NOTERAM_GETREADMODE
       *      - Get read mode
       *        Argument: A writable pointer to unsigned int
       */

      case NOTERAM_GETREADMODE:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            FAR struct noteram_dump_context_s *ctx = filep->f_priv;

            *(FAR unsigned int *)arg = ctx->mode;
            ret = OK;
          }
        break;

      /* NOTERAM_SETREADMODE
       *      - Set read mode
       *        Argument: A read-only pointer to unsigned int
       */

      case NOTERAM_SETREADMODE:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            FAR struct noteram_dump_context_s *ctx = filep->f_priv;

            ctx->mode = *(FAR unsigned int *)arg;
            ret = OK;
          }
        break;

      default:
          break;
    }

  spin_unlock_irqrestore_wo_note(&drv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: noteram_poll
 ****************************************************************************/

static int noteram_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  int ret = 0;
  FAR struct inode *inode;
  FAR struct noteram_driver_s *drv;
  irqstate_t flags;

  DEBUGASSERT(filep != NULL && fds != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  drv = inode->i_private;

  flags = spin_lock_irqsave_wo_note(&drv->lock);

  /* Ignore waits that do not include POLLIN */

  if ((fds->events & POLLIN) == 0)
    {
      ret = -EDEADLK;
      goto errout;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Check if we can accept this poll.
       * For now, only one thread can poll the device at any time
       * (shorter / simpler code)
       */

      if (drv->pfd)
        {
          ret = -EBUSY;
          goto errout;
        }

      drv->pfd = fds;

      /* Is there unread data in noteram? then trigger POLLIN now -
       * don't wait for RX.
       */

      if (noteram_unread_length(drv) > 0)
        {
          spin_unlock_irqrestore_wo_note(&drv->lock, flags);
          poll_notify(&drv->pfd, 1, POLLIN);
          return ret;
        }
    }
  else /* Tear it down */
    {
      drv->pfd = NULL;
    }

errout:
  spin_unlock_irqrestore_wo_note(&drv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: noteram_add
 *
 * Description:
 *   Add the variable length note to the transport layer
 *
 * Input Parameters:
 *   note    - The note buffer
 *   notelen - The buffer length
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

static void noteram_add(FAR struct note_driver_s *driver,
                        FAR const void *note, size_t notelen)
{
  FAR const char *buf = note;
  FAR struct noteram_driver_s *drv = (FAR struct noteram_driver_s *)driver;
  unsigned int head;
  unsigned int remain;
  unsigned int space;
  irqstate_t flags;

  flags = spin_lock_irqsave_wo_note(&drv->lock);

  if (drv->ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
      spin_unlock_irqrestore_wo_note(&drv->lock, flags);
      return;
    }

  DEBUGASSERT(note != NULL && notelen < drv->ni_bufsize);
  remain = drv->ni_bufsize - noteram_length(drv);

  if (remain <= NOTE_ALIGN(notelen))
    {
      if (drv->ni_overwrite == NOTERAM_MODE_OVERWRITE_DISABLE)
        {
          /* Stop recording if not in overwrite mode */

          drv->ni_overwrite = NOTERAM_MODE_OVERWRITE_OVERFLOW;
          spin_unlock_irqrestore_wo_note(&drv->lock, flags);
          return;
        }

      /* Remove the note at the tail index , make sure there is enough space
       */

      do
        {
          noteram_remove(drv);
          remain = drv->ni_bufsize - noteram_length(drv);
        }
      while (remain <= NOTE_ALIGN(notelen));
    }

  head = drv->ni_head;
  space = drv->ni_bufsize - head;
  space = space < notelen ? space : notelen;
  memcpy(drv->ni_buffer + head, note, space);
  memcpy(drv->ni_buffer, buf + space, notelen - space);
  drv->ni_head = noteram_next(drv, head, NOTE_ALIGN(notelen));
  spin_unlock_irqrestore_wo_note(&drv->lock, flags);
  poll_notify(&drv->pfd, 1, POLLIN);
}

/****************************************************************************
 * Name: noteram_dump_init_context
 ****************************************************************************/

static void noteram_dump_init_context(FAR struct noteram_dump_context_s *ctx)
{
  int cpu;

  /* Initialize the trace dump context */

  for (cpu = 0; cpu < NCPUS; cpu++)
    {
      ctx->cpu[cpu].intr_nest = 0;
      ctx->cpu[cpu].pendingswitch = false;
      ctx->cpu[cpu].current_state = TSTATE_TASK_RUNNING;
      ctx->cpu[cpu].current_pid = -1;
      ctx->cpu[cpu].next_pid = -1;
      ctx->cpu[cpu].current_priority = -1;
      ctx->cpu[cpu].next_priority = -1;
    }
}

/****************************************************************************
 * Name: get_taskname
 ****************************************************************************/

static const char *get_taskname(pid_t pid)
{
#if CONFIG_DRIVERS_NOTE_TASKNAME_BUFSIZE > 0
  FAR const char *taskname;

  taskname = note_get_taskname(pid);
  if (taskname != NULL)
    {
      return taskname;
    }
#endif

  return "<noname>";
}

/****************************************************************************
 * Name: noteram_dump_header
 ****************************************************************************/

static int noteram_dump_header(FAR struct lib_outstream_s *s,
                               FAR struct note_common_s *note,
                               FAR struct noteram_dump_context_s *ctx)
{
  struct timespec ts;
  pid_t pid;
  int ret;

  perf_convert(note->nc_systime, &ts);
  pid = note->nc_pid;
#ifdef CONFIG_SMP
  int cpu = note->nc_cpu;
#else
  int cpu = 0;
#endif

  ret = lib_sprintf(s, "%8s-%-3u [%d] %3" PRIu64 ".%09lu: ",
                    get_taskname(pid), get_pid(pid), cpu,
                    (uint64_t)ts.tv_sec, ts.tv_nsec);

  return ret;
}

#if (defined CONFIG_SCHED_INSTRUMENTATION_SWITCH)                            \
    || (defined CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER)
/****************************************************************************
 * Name: noteram_dump_sched_switch
 ****************************************************************************/

static int noteram_dump_sched_switch(FAR struct lib_outstream_s *s,
                                     FAR struct note_common_s *note,
                                     FAR struct noteram_dump_context_s *ctx)
{
  FAR struct noteram_dump_cpu_context_s *cctx;
  uint8_t current_priority;
  uint8_t next_priority;
  pid_t current_pid;
  pid_t next_pid;
  int ret;
#ifdef CONFIG_SMP
  int cpu = note->nc_cpu;
#else
  int cpu = 0;
#endif

  cctx = &ctx->cpu[cpu];
  current_pid = cctx->current_pid;
  next_pid = cctx->next_pid;

  current_priority = cctx->current_priority;
  next_priority = cctx->next_priority;

  ret = lib_sprintf(s, "sched_switch: prev_comm=%s prev_pid=%u "
                    "prev_prio=%u prev_state=%c ==> "
                    "next_comm=%s next_pid=%u next_prio=%u\n",
                    get_taskname(current_pid), get_pid(current_pid),
                    current_priority, get_task_state(cctx->current_state),
                    get_taskname(next_pid), get_pid(next_pid),
                    next_priority);

  cctx->current_pid = cctx->next_pid;
  cctx->current_priority = cctx->next_priority;
  cctx->pendingswitch = false;
  return ret;
}
#endif

/****************************************************************************
 * Name: noteram_dump_printf
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
static int noteram_dump_printf(FAR struct lib_outstream_s *s,
                               FAR struct note_printf_s *note)
{
  size_t ret = 0;

  if (note->npt_type == 0)
    {
      ret = lib_bsprintf(s, note->npt_fmt, note->npt_data);
    }
  else
    {
      size_t count = NOTE_PRINTF_GET_COUNT(note->npt_type);
      char fmt[128];
      size_t i;

      fmt[0] = '\0';
      ret += lib_sprintf(s, "%p", note->npt_fmt);
      for (i = 0; i < count; i++)
        {
          int type = NOTE_PRINTF_GET_TYPE(note->npt_type, i);

          switch (type)
            {
              case NOTE_PRINTF_UINT32:
                {
                  strcat(fmt, " %u");
                }
                break;
              case NOTE_PRINTF_UINT64:
                {
                  strcat(fmt, " %llu");
                }
                break;
              case NOTE_PRINTF_STRING:
                {
                  strcat(fmt, " %s");
                }
                break;
              case NOTE_PRINTF_DOUBLE:
                {
                  strcat(fmt, " %f");
                }
            }
        }

        ret += lib_bsprintf(s, fmt, note->npt_data);
        lib_stream_putc(s, '\n');
        ret++;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: noteram_dump_one
 ****************************************************************************/

static int noteram_dump_one(FAR uint8_t *p, FAR struct lib_outstream_s *s,
                            FAR struct noteram_dump_context_s *ctx)
{
  FAR struct note_common_s *note = (FAR struct note_common_s *)p;
  FAR struct noteram_dump_cpu_context_s *cctx;
  int ret = 0;
  pid_t pid;
#ifdef CONFIG_SMP
  int cpu = note->nc_cpu;
#else
  int cpu = 0;
#endif

  cctx = &ctx->cpu[cpu];
  pid = note->nc_pid;

  if (cctx->current_pid < 0)
    {
      cctx->current_pid = pid;
    }

  /* Output one note */

  switch (note->nc_type)
    {
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
    case NOTE_START:
      {
        ret += noteram_dump_header(s, note, ctx);
        ret += lib_sprintf(s, "sched_wakeup_new: comm=%s pid=%d "
                           "target_cpu=%d\n",
                           get_taskname(pid), get_pid(pid), cpu);
      }
      break;

    case NOTE_STOP:
      {
        /* This note informs the task to be stopped.
         * Change current task state for the succeeding NOTE_RESUME.
         */

        cctx->current_state = 0;
      }
      break;

    case NOTE_SUSPEND:
      {
        FAR struct note_suspend_s *nsu = (FAR struct note_suspend_s *)p;

        /* This note informs the task to be suspended.
         * Preserve the information for the succeeding NOTE_RESUME.
         */

        cctx->current_state = nsu->nsu_state;
      }
      break;

    case NOTE_RESUME:
      {
        /* This note informs the task to be resumed.
         * The task switch timing depends on the running context.
         */

        cctx->next_pid = pid;
        cctx->next_priority = note->nc_priority;

        if (cctx->intr_nest == 0)
          {
            /* If not in the interrupt context, the task switch is
             * executed immediately.
             */

            ret += noteram_dump_header(s, note, ctx);
            ret += noteram_dump_sched_switch(s, note, ctx);
          }
        else
          {
            /* If in the interrupt context, the task switch is postponed
             * until leaving the interrupt handler.
             */

            ret += noteram_dump_header(s, note, ctx);
            ret += lib_sprintf(s, "sched_waking: comm=%s "
                               "pid=%d target_cpu=%d\n",
                               get_taskname(cctx->next_pid),
                               get_pid(cctx->next_pid), cpu);
            cctx->pendingswitch = true;
          }
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
    case NOTE_SYSCALL_ENTER:
      {
        FAR struct note_syscall_enter_s *nsc;
        int i;
        uintptr_t arg;

        nsc = (FAR struct note_syscall_enter_s *)p;
        if (nsc->nsc_nr < CONFIG_SYS_RESERVED ||
            nsc->nsc_nr >= SYS_maxsyscall)
          {
            break;
          }

        ret += noteram_dump_header(s, note, ctx);
        ret += lib_sprintf(s, "sys_%s(",
                           g_funcnames[nsc->nsc_nr - CONFIG_SYS_RESERVED]);

        for (i = 0; i < nsc->nsc_argc; i++)
          {
            arg = nsc->nsc_args[i];
            if (i == 0)
              {
                ret += lib_sprintf(s, "arg%d: 0x%" PRIxPTR, i, arg);
              }
            else
              {
                ret += lib_sprintf(s, ", arg%d: 0x%" PRIxPTR, i, arg);
              }
          }

        ret += lib_sprintf(s, ")\n");
      }
      break;

    case NOTE_SYSCALL_LEAVE:
      {
        FAR struct note_syscall_leave_s *nsc;
        uintptr_t result;

        nsc = (FAR struct note_syscall_leave_s *)p;
        if (nsc->nsc_nr < CONFIG_SYS_RESERVED ||
            nsc->nsc_nr >= SYS_maxsyscall)
          {
            break;
          }

        ret += noteram_dump_header(s, note, ctx);
        result = nsc->nsc_result;
        ret += lib_sprintf(s, "sys_%s -> 0x%" PRIxPTR "\n",
                          g_funcnames[nsc->nsc_nr - CONFIG_SYS_RESERVED],
                          result);
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
    case NOTE_IRQ_ENTER:
      {
        FAR struct note_irqhandler_s *nih;

        nih = (FAR struct note_irqhandler_s *)p;
        ret += noteram_dump_header(s, note, ctx);
        ret += lib_sprintf(s, "irq_handler_entry: irq=%u name=%pS\n",
                           nih->nih_irq, (FAR void *)nih->nih_handler);
        cctx->intr_nest++;
      }
      break;

    case NOTE_IRQ_LEAVE:
      {
        FAR struct note_irqhandler_s *nih;

        nih = (FAR struct note_irqhandler_s *)p;
        ret += noteram_dump_header(s, note, ctx);
        ret += lib_sprintf(s, "irq_handler_exit: irq=%u ret=handled\n",
                           nih->nih_irq);
        cctx->intr_nest--;

        if (cctx->intr_nest <= 0)
          {
            cctx->intr_nest = 0;
            if (cctx->pendingswitch)
              {
                /* If the pending task switch exists, it is executed here */

                ret += noteram_dump_header(s, note, ctx);
                ret += noteram_dump_sched_switch(s, note, ctx);
              }
          }
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_WDOG
    case NOTE_WDOG_START:
    case NOTE_WDOG_CANCEL:
    case NOTE_WDOG_ENTER:
    case NOTE_WDOG_LEAVE:
      {
        FAR struct note_wdog_s *nw;
        FAR const char *name[] =
          {
            "start", "cancel", "enter", "leave",
          };

        nw = (FAR struct note_wdog_s *)p;
        ret += noteram_dump_header(s, note, ctx);
        ret += lib_sprintf(s, "tracing_mark_write: I|%d|wdog: %s-%pS %p\n",
                           pid, name[note->nc_type - NOTE_WDOG_START],
                           (FAR void *)nw->handler, (FAR void *)nw->arg);
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
    case NOTE_CSECTION_ENTER:
    case NOTE_CSECTION_LEAVE:
      {
        struct note_csection_s *ncs;
        ncs = (FAR struct note_csection_s *)p;
        ret += noteram_dump_header(s, &ncs->ncs_cmn, ctx);
        ret += lib_sprintf(s, "tracing_mark_write: %c|%d|critical_section\n",
                           note->nc_type == NOTE_CSECTION_ENTER ?
                           'B' : 'E', pid);
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
    case NOTE_PREEMPT_LOCK:
    case NOTE_PREEMPT_UNLOCK:
      {
        struct note_preempt_s *npr;
        int16_t count;
        npr = (FAR struct note_preempt_s *)p;
        count = npr->npr_count;
        ret += noteram_dump_header(s, &npr->npr_cmn, ctx);
        ret += lib_sprintf(s,  "tracing_mark_write: "
                          "%c|%d|sched_lock:%d\n",
                          note->nc_type == NOTE_PREEMPT_LOCK ?
                          'B' : 'E', pid, count);
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
    case NOTE_DUMP_PRINTF:
      {
        FAR struct note_printf_s *npt;

        npt = (FAR struct note_printf_s *)p;
        ret += noteram_dump_header(s, &npt->npt_cmn, ctx);
        ret += lib_sprintf(s, "tracing_mark_write: ");
        ret += noteram_dump_printf(s, npt);
      }
      break;
    case NOTE_DUMP_BEGIN:
    case NOTE_DUMP_END:
      {
        FAR struct note_event_s *nbi = (FAR struct note_event_s *)p;
        char c = note->nc_type == NOTE_DUMP_BEGIN ? 'B' : 'E';
        int len = note->nc_length - SIZEOF_NOTE_EVENT(0);
        uintptr_t ip;

        ip = nbi->nev_ip;
        ret += noteram_dump_header(s, &nbi->nev_cmn, ctx);
        if (len > 0)
          {
            ret += lib_sprintf(s, "tracing_mark_write: %c|%d|%.*s\n",
                               c, pid, len, (FAR const char *)nbi->nev_data);
          }
        else
          {
            ret += lib_sprintf(s, "tracing_mark_write: %c|%d|%pS\n",
                               c, pid, (FAR void *)ip);
          }
      }
      break;
    case NOTE_DUMP_MARK:
      {
        int len = note->nc_length - sizeof(struct note_event_s);
        FAR struct note_event_s *nbi = (FAR struct note_event_s *)p;
        ret += noteram_dump_header(s, &nbi->nev_cmn, ctx);
        ret += lib_sprintf(s, "tracing_mark_write: I|%d|%.*s\n",
                           pid, len, (FAR const char *)nbi->nev_data);
      }
      break;
    case NOTE_DUMP_COUNTER:
      {
        FAR struct note_event_s *nbi = (FAR struct note_event_s *)p;
        FAR struct note_counter_s *counter;
        counter = (FAR struct note_counter_s *)nbi->nev_data;
        ret += noteram_dump_header(s, &nbi->nev_cmn, ctx);
        ret += lib_sprintf(s, "tracing_mark_write: C|%d|%s|%ld\n",
                           pid, counter->name, counter->value);
      }
      break;
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_HEAP
    case NOTE_HEAP_ADD:
    case NOTE_HEAP_REMOVE:
    case NOTE_HEAP_ALLOC:
    case NOTE_HEAP_FREE:
      {
        FAR struct note_heap_s *nmm = (FAR struct note_heap_s *)p;
        FAR const char *name[] =
          {
            "add", "remove", "malloc", "free"
          };

        ret += noteram_dump_header(s, &nmm->nhp_cmn, ctx);
        ret += lib_sprintf(s, "tracing_mark_write: C|%d|Heap Usage|%d|%s"
                           ": heap: %p size:%" PRIiPTR ", address: %p\n",
                           pid, nmm->used,
                           name[note->nc_type - NOTE_HEAP_ADD],
                           nmm->heap, nmm->size, nmm->mem);
      }
      break;
#endif
    default:
      break;
    }

  /* Return the length of the processed note */

  return ret;
}

#ifdef CONFIG_DRIVERS_NOTERAM_CRASH_DUMP

/****************************************************************************
 * Name: noteram_dump
 ****************************************************************************/

static void noteram_dump(FAR struct noteram_driver_s *drv)
{
  struct noteram_dump_context_s ctx;
  struct lib_syslograwstream_s stream;
  uint8_t note[256];

  lib_syslograwstream_open(&stream);
  lib_sprintf(&stream.common, "# tracer:nop\n#\n");

  while (1)
    {
      ssize_t ret;

      ret = noteram_get(drv, note, sizeof(note));
      if (ret <= 0)
        {
          break;
        }

      noteram_dump_one(note, &stream.common, &ctx);
    }

  lib_syslograwstream_close(&stream);
}

/****************************************************************************
 * Name: noteram_crash_dump
 ****************************************************************************/

static int noteram_crash_dump(FAR struct notifier_block *nb,
                              unsigned long action, FAR void *data)
{
  if (action == PANIC_KERNEL)
    {
      noteram_dump(&g_noteram_driver);
    }

  return 0;
}

static void noteram_crash_dump_register(void)
{
  static struct notifier_block nb;
  nb.notifier_call = noteram_crash_dump;
  panic_notifier_chain_register(&nb);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noteram_register
 *
 * Description:
 *   Register a serial driver at /dev/note/ram that can be used by an
 *   application to read data from the circular note buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int noteram_register(void)
{
#ifdef CONFIG_DRIVERS_NOTERAM_CRASH_DUMP
  noteram_crash_dump_register();
#endif
  return register_driver("/dev/note/ram", &g_noteram_fops, 0666,
                         &g_noteram_driver);
}

/****************************************************************************
 * Name: noteram_initialize
 *
 * Description:
 *   Register a serial driver at /dev/note/ram that can be used by an
 *   application to read data from the circular note buffer.
 *
 * Input Parameters:
 *  devpath: The path of the Noteram device
 *  bufsize: The size of the circular buffer
 *  overwrite: The overwrite mode
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

FAR struct note_driver_s *
noteram_initialize(FAR const char *devpath, size_t bufsize, bool overwrite)
{
  FAR struct noteram_driver_s *drv;
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  size_t len = strlen(devpath) + 1;
#else
  size_t len = 0;
#endif
  int ret;

  drv = kmm_malloc(sizeof(*drv) + len + bufsize);
  if (drv == NULL)
    {
      return NULL;
    }
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER

  memcpy(drv + 1, devpath, len);
  drv->driver.name = (FAR const char *)(drv + 1);
  drv->driver.filter.mode.flag =
                      CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE;

#  ifdef CONFIG_SMP
  drv->driver.filter.mode.cpuset =
                      CONFIG_SCHED_INSTRUMENTATION_CPUSET;
#  endif

#endif

  drv->driver.ops = &g_noteram_ops;
  drv->ni_bufsize = bufsize;
  drv->ni_buffer = (FAR uint8_t *)(drv + 1) + len;
  drv->ni_overwrite = overwrite;
  drv->ni_head = 0;
  drv->ni_tail = 0;
  drv->ni_read = 0;
  drv->pfd = NULL;

  ret = note_driver_register(&drv->driver);
  if (ret < 0)
    {
      kmm_free(drv);
      return NULL;
    }

  if (devpath == NULL)
    {
      return &drv->driver;
    }

  ret = register_driver(devpath, &g_noteram_fops, 0666, drv);
  if (ret < 0)
    {
      kmm_free(drv);
      return NULL;
    }

  return &drv->driver;
}
