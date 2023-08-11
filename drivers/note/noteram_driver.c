/****************************************************************************
 * drivers/note/noteram_driver.c
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

#include <nuttx/spinlock.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/kmalloc.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/fs/fs.h>

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

struct noteram_dump_task_context_s
{
  FAR struct noteram_dump_task_context_s *next;
  pid_t pid;                            /* Task PID */
  int syscall_nest;                     /* Syscall nest level */
  char name[CONFIG_TASK_NAME_SIZE + 1]; /* Task name (with NULL terminator) */
};

struct noteram_dump_context_s
{
  int buflen;           /* The length of the dumped data */
  char buffer[256];     /* Buffer to hold the line to be dumped */
  struct noteram_dump_cpu_context_s cpu[NCPUS];
  FAR struct noteram_dump_task_context_s *task;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int noteram_open(FAR struct file *filep);
static int noteram_close(FAR struct file *filep);
static ssize_t noteram_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen);
static int noteram_ioctl(struct file *filep, int cmd, unsigned long arg);
static void noteram_add(FAR struct note_driver_s *drv,
                        FAR const void *note, size_t len);

static void
noteram_dump_init_context(FAR struct noteram_dump_context_s *ctx);
static void
noteram_dump_fini_context(FAR struct noteram_dump_context_s *ctx);
static int noteram_dump_one(FAR uint8_t *p,
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
};

static uint8_t g_ramnote_buffer[CONFIG_DRIVERS_NOTERAM_BUFSIZE];

static const struct note_driver_ops_s g_noteram_ops =
{
  noteram_add
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct noteram_driver_s g_noteram_driver =
{
  {&g_noteram_ops},
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

  length = drv->ni_buffer[tail];
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

      drv->ni_read = noteram_next(drv, read, notelen);

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

  drv->ni_read = read;

  return notelen;
}

/****************************************************************************
 * Name: noteram_size
 *
 * Description:
 *   Return the size of the next note at the read index of the circular
 *   buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero is returned if the circular buffer is empty.  Otherwise, the size
 *   of the next note is returned.
 *
 ****************************************************************************/

static ssize_t noteram_size(FAR struct noteram_driver_s *drv)
{
  FAR struct note_common_s *note;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

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

  note    = (FAR struct note_common_s *)&drv->ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

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
  noteram_dump_fini_context(ctx);
  kmm_free(ctx);
  return OK;
}

/****************************************************************************
 * Name: noteram_read
 ****************************************************************************/

static ssize_t noteram_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  ssize_t ret = -EFBIG;
  ssize_t retlen = 0;
  uint8_t note[32];
  irqstate_t flags;
  FAR struct noteram_dump_context_s *ctx = filep->f_priv;
  FAR struct noteram_driver_s *drv =
    (FAR struct noteram_driver_s *)filep->f_inode->i_private;

  /* If we have parsed but unread data last time,
   * we should save the last data first
   */

  if (ctx->buflen > 0)
    {
      if (ctx->buflen > buflen)
        {
          goto errout;
        }

      strlcpy(buffer, ctx->buffer, buflen);
      retlen = ctx->buflen;
      buffer += ctx->buflen;
      buflen -= ctx->buflen;
    }

  /* Then loop, adding as many notes as possible to the user buffer. */

  do
    {
      /* Get the next note (removing it from the buffer) */

      flags = spin_lock_irqsave_wo_note(&drv->lock);
      ret = noteram_get(drv, note, sizeof(note));
      spin_unlock_irqrestore_wo_note(&drv->lock, flags);
      if (ret <= 0)
        {
          ctx->buflen = 0;
          break;
        }

      /* Parse notes into text format */

      ctx->buflen = noteram_dump_one(note, ctx);

      /* If the remaining space is insufficient,
       * return the read data directly first
       */

      if (ctx->buflen > buflen)
        {
          ret = -EFBIG;
          break;
        }

      /* Update pointers from the note that was transferred */

      strlcpy(buffer, ctx->buffer, buflen);

      retlen += ctx->buflen;
      buffer += ctx->buflen;
      buflen -= ctx->buflen;
      ctx->buflen = 0;

      /* Will the next note fit?  There is a race here and even if the next
       * note will fit, it may fail still when noteram_get() is called.
       *
       * It won't fit (or an error occurred).  Return what we have without
       * trying to get the next note (which would cause it to be deleted).
       */

      ret = noteram_size(drv);
    }
  while (ret > 0);

  /* If no data is read, an error code is returned */

errout:
  return retlen ? retlen : ret;
}

/****************************************************************************
 * Name: noteram_ioctl
 ****************************************************************************/

static int noteram_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOSYS;
  FAR struct noteram_driver_s *drv =
    (FAR struct noteram_driver_s *)filep->f_inode->i_private;
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
            *(unsigned int *)arg = drv->ni_overwrite;
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
            drv->ni_overwrite = *(unsigned int *)arg;
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

  if (remain < notelen)
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
      while (remain < notelen);
    }

  head = drv->ni_head;
  space = drv->ni_bufsize - head;
  space = space < notelen ? space : notelen;
  memcpy(drv->ni_buffer + head, note, space);
  memcpy(drv->ni_buffer, buf + space, notelen - space);
  drv->ni_head = noteram_next(drv, head, notelen);
  spin_unlock_irqrestore_wo_note(&drv->lock, flags);
}

/****************************************************************************
 * Name: noteram_dump_unflatten
 ****************************************************************************/

static void noteram_dump_unflatten(FAR void *dst, FAR uint8_t *src,
                                   size_t len)
{
#ifdef CONFIG_ENDIAN_BIG
  FAR uint8_t *end = (FAR uint8_t *)dst + len - 1;
  while (len-- > 0)
    {
      *end-- = *src++;
    }
#else
  memcpy(dst, src, len);
#endif
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

  ctx->task = NULL;
}

/****************************************************************************
 * Name: noteram_dump_fini_context
 ****************************************************************************/

static void noteram_dump_fini_context(FAR struct noteram_dump_context_s *ctx)
{
  FAR struct noteram_dump_task_context_s *tctx;
  FAR struct noteram_dump_task_context_s *ntctx;

  /* Finalize the trace dump context */

  tctx = ctx->task;
  ctx->task = NULL;
  while (tctx != NULL)
    {
      ntctx = tctx->next;
      free(tctx);
      tctx = ntctx;
    }
}

/****************************************************************************
 * Name: copy_task_name
 ****************************************************************************/

#if CONFIG_TASK_NAME_SIZE > 0
static void copy_task_name(FAR char *dst, FAR const char *src)
{
  char c;
  int i;

  /* Replace space to underline
   * Text trace data format cannot use a space as a task name.
   */

  for (i = 0; i < CONFIG_TASK_NAME_SIZE; i++)
    {
      c = *src++;
      if (c == '\0')
        {
          break;
        }

      *dst++ = (c == ' ') ? '_' : c;
    }

  *dst = '\0';
}
#endif

/****************************************************************************
 * Name: get_task_context
 ****************************************************************************/

FAR static struct noteram_dump_task_context_s *
get_task_context(pid_t pid, FAR struct noteram_dump_context_s *ctx)
{
  FAR struct noteram_dump_task_context_s **tctxp;
  tctxp = &ctx->task;
  while (*tctxp != NULL)
    {
      if ((*tctxp)->pid == pid)
        {
          return *tctxp;
        }

      tctxp = &((*tctxp)->next);
    }

  /* Create new trace dump task context */

  *tctxp = (FAR struct noteram_dump_task_context_s *)malloc(
      sizeof(struct noteram_dump_task_context_s));
  if (*tctxp != NULL)
    {
      (*tctxp)->next = NULL;
      (*tctxp)->pid = pid;
      (*tctxp)->syscall_nest = 0;
      (*tctxp)->name[0] = '\0';

#ifdef CONFIG_DRIVERS_NOTE_TASKNAME_BUFSIZE
        {
          FAR const char *taskname;

          taskname = note_get_taskname(pid);
          if (taskname != NULL)
            {
              copy_task_name((*tctxp)->name, taskname);
            }
        }
#endif
    }

  return *tctxp;
}

/****************************************************************************
 * Name: get_task_name
 ****************************************************************************/

static const char *get_task_name(pid_t pid,
                                 FAR struct noteram_dump_context_s *ctx)
{
  FAR struct noteram_dump_task_context_s *tctx;

  tctx = get_task_context(pid, ctx);
  if (tctx != NULL && tctx->name[0] != '\0')
    {
      return tctx->name;
    }

  return "<noname>";
}

/****************************************************************************
 * Name: noteram_dump_header
 ****************************************************************************/

static int noteram_dump_header(FAR char *offset,
                               FAR struct note_common_s *note,
                               FAR struct noteram_dump_context_s *ctx)
{
  pid_t pid;
  uint32_t nsec;
  uint32_t sec;
  int ret;

  noteram_dump_unflatten(&nsec, note->nc_systime_nsec, sizeof(nsec));
  noteram_dump_unflatten(&sec, note->nc_systime_sec, sizeof(sec));
#ifdef CONFIG_SMP
  int cpu = note->nc_cpu;
#else
  int cpu = 0;
#endif

  noteram_dump_unflatten(&pid, note->nc_pid, sizeof(pid));

  ret = sprintf(offset, "%8s-%-3u [%d] %3" PRIu32 ".%09" PRIu32 ": ",
                get_task_name(pid, ctx), get_pid(pid), cpu, sec, nsec);
  return ret;
}

#if (defined CONFIG_SCHED_INSTRUMENTATION_SWITCH)                            \
    || (defined CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER)
/****************************************************************************
 * Name: noteram_dump_sched_switch
 ****************************************************************************/

static int noteram_dump_sched_switch(FAR char *offset,
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

  ret = sprintf(offset,
                "sched_switch: "
                "prev_comm=%s prev_pid=%u prev_prio=%u prev_state=%c ==> "
                "next_comm=%s next_pid=%u next_prio=%u\n",
                get_task_name(current_pid, ctx), get_pid(current_pid),
                current_priority, get_task_state(cctx->current_state),
                get_task_name(next_pid, ctx), get_pid(next_pid),
                next_priority);

  cctx->current_pid = cctx->next_pid;
  cctx->current_priority = cctx->next_priority;
  cctx->pendingswitch = false;
  return ret;
}
#endif

/****************************************************************************
 * Name: noteram_dump_one
 ****************************************************************************/

static int noteram_dump_one(FAR uint8_t *p,
                            FAR struct noteram_dump_context_s *ctx)
{
  FAR struct note_common_s *note = (FAR struct note_common_s *)p;
  FAR struct noteram_dump_cpu_context_s *cctx;
  FAR char *buffer = (FAR char *)ctx->buffer;
  FAR char *offset = (FAR char *)ctx->buffer;
  pid_t pid;
#ifdef CONFIG_SMP
  int cpu = note->nc_cpu;
#else
  int cpu = 0;
#endif

  cctx = &ctx->cpu[cpu];
  noteram_dump_unflatten(&pid, note->nc_pid, sizeof(pid));

  if (cctx->current_pid < 0)
    {
      cctx->current_pid = pid;
    }

  /* Output one note */

  switch (note->nc_type)
    {
    case NOTE_START:
      {
#if CONFIG_TASK_NAME_SIZE > 0
        FAR struct note_start_s *nst = (FAR struct note_start_s *)p;
        FAR struct noteram_dump_task_context_s *tctx;

        tctx = get_task_context(pid, ctx);
        if (tctx != NULL)
          {
            copy_task_name(tctx->name, nst->nst_name);
          }
#endif

        offset += noteram_dump_header(offset, note, ctx);
        offset += sprintf(offset,
                          "sched_wakeup_new: comm=%s pid=%d target_cpu=%d\n",
                          get_task_name(pid, ctx), get_pid(pid), cpu);
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

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
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

            offset += noteram_dump_header(offset, note, ctx);
            offset += noteram_dump_sched_switch(offset, note, ctx);
          }
        else
          {
            /* If in the interrupt context, the task switch is postponed
             * until leaving the interrupt handler.
             */

            offset += noteram_dump_header(offset, note, ctx);
            offset += sprintf(offset,
                              "sched_waking: comm=%s pid=%d target_cpu=%d\n",
                              get_task_name(cctx->next_pid, ctx),
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
        FAR struct noteram_dump_task_context_s *tctx;
        int i;
        int j;
        uintptr_t arg;

        /* Exclude the case of syscall issued by an interrupt handler and
         * nested syscalls to correct tracecompass display.
         */

        if (cctx->intr_nest > 0)
          {
            break;
          }

        tctx = get_task_context(pid, ctx);
        if (tctx == NULL)
          {
            break;
          }

        tctx->syscall_nest++;
        if (tctx->syscall_nest > 1)
          {
            break;
          }

        nsc = (FAR struct note_syscall_enter_s *)p;
        if (nsc->nsc_nr < CONFIG_SYS_RESERVED ||
            nsc->nsc_nr >= SYS_maxsyscall)
          {
            break;
          }

        offset += noteram_dump_header(offset, note, ctx);
        offset += sprintf(offset, "sys_%s(",
                          g_funcnames[nsc->nsc_nr - CONFIG_SYS_RESERVED]);

        for (i = j = 0; i < nsc->nsc_argc; i++)
          {
            noteram_dump_unflatten(&arg, nsc->nsc_args, sizeof(arg));
            if (i == 0)
              {
                offset += sprintf(offset, "arg%d: 0x%" PRIxPTR, i, arg);
              }
            else
              {
                offset += sprintf(offset, ", arg%d: 0x%" PRIxPTR, i, arg);
              }
          }

        offset += sprintf(offset, ")\n");
      }
      break;

    case NOTE_SYSCALL_LEAVE:
      {
        FAR struct note_syscall_leave_s *nsc;
        FAR struct noteram_dump_task_context_s *tctx;
        uintptr_t result;

        /* Exclude the case of syscall issued by an interrupt handler and
         * nested syscalls to correct tracecompass display.
         */

        if (cctx->intr_nest > 0)
          {
            break;
          }

        tctx = get_task_context(pid, ctx);
        if (tctx == NULL)
          {
            break;
          }

        tctx->syscall_nest--;
        if (tctx->syscall_nest > 0)
          {
            break;
          }

        tctx->syscall_nest = 0;

        nsc = (FAR struct note_syscall_leave_s *)p;
        if (nsc->nsc_nr < CONFIG_SYS_RESERVED ||
            nsc->nsc_nr >= SYS_maxsyscall)
          {
            break;
          }

        offset += noteram_dump_header(offset, note, ctx);
        noteram_dump_unflatten(&result, nsc->nsc_result, sizeof(result));
        offset += sprintf(offset, "sys_%s -> 0x%" PRIxPTR "\n",
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
        offset += noteram_dump_header(offset, note, ctx);
        offset += sprintf(offset, "irq_handler_entry: irq=%u name=%d\n",
                          nih->nih_irq, nih->nih_irq);
        cctx->intr_nest++;
      }
      break;

    case NOTE_IRQ_LEAVE:
      {
        FAR struct note_irqhandler_s *nih;

        nih = (FAR struct note_irqhandler_s *)p;
        offset += noteram_dump_header(offset, note, ctx);
        offset += sprintf(offset, "irq_handler_exit: irq=%u ret=handled\n",
                nih->nih_irq);
        cctx->intr_nest--;

        if (cctx->intr_nest <= 0)
          {
            cctx->intr_nest = 0;
            if (cctx->pendingswitch)
              {
                /* If the pending task switch exists, it is executed here */

                offset += noteram_dump_header(offset, note, ctx);
                offset += noteram_dump_sched_switch(offset, note, ctx);
              }
          }
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
    case NOTE_CSECTION_ENTER:
    case NOTE_CSECTION_LEAVE:
      {
        struct note_csection_s *ncs;
        ncs = (FAR struct note_csection_s *)p;
        offset += noteram_dump_header(offset, &ncs->ncs_cmn, ctx);
        offset += sprintf(offset,
                          "tracing_mark_write: %c|%d|"
                          "critical_section\n",
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
        noteram_dump_unflatten(&count, npr->npr_count, sizeof(count));
        offset += noteram_dump_header(offset, &npr->npr_cmn, ctx);
        offset += sprintf(offset, "tracing_mark_write: "
                          "%c|%d|sched_lock:%d\n",
                          note->nc_type == NOTE_PREEMPT_LOCK ?
                          'B' : 'E', pid, count);
      }
      break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
    case NOTE_DUMP_STRING:
      {
        FAR struct note_string_s *nst;
        uintptr_t ip;

        nst = (FAR struct note_string_s *)p;
        offset += noteram_dump_header(offset, note, ctx);
        noteram_dump_unflatten(&ip, nst->nst_ip, sizeof(ip));

        if (nst->nst_data[1] == '\0' &&
            (nst->nst_data[0] == 'B' || nst->nst_data[0] == 'E'))
          {
            offset += sprintf(offset, "tracing_mark_write: %c|%d|%pS\n",
                              nst->nst_data[0], pid, (FAR void *)ip);
          }
        else
          {
            offset += sprintf(offset, "tracing_mark_write: %s\n",
                    nst->nst_data);
          }
      }
      break;

    case NOTE_DUMP_BINARY:
      {
        FAR struct note_binary_s *nbi;
        uint8_t count;
        uintptr_t ip;
        int i;

        nbi = (FAR struct note_binary_s *)p;
        offset += noteram_dump_header(offset, note, ctx);
        count = note->nc_length - sizeof(struct note_binary_s) + 1;

        noteram_dump_unflatten(&ip, nbi->nbi_ip, sizeof(ip));

        offset += sprintf(offset, "0x%" PRIdPTR ": event=%u count=%u", ip,
                          nbi->nbi_event, count);
        for (i = 0; i < count; i++)
          {
            offset += sprintf(offset, " 0x%x", nbi->nbi_data[i]);
          }

        offset += sprintf(offset, "\n");
      }
      break;
#endif

    default:
      break;
    }

  /* Return the length of the processed note */

  return offset - buffer;
}

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
  int ret;

  drv = (FAR struct noteram_driver_s *)kmm_malloc(sizeof(*drv) + bufsize);
  if (drv == NULL)
    {
      return NULL;
    }

  drv->driver.ops = &g_noteram_ops;
  drv->ni_bufsize = bufsize;
  drv->ni_buffer = (FAR uint8_t *)(drv + 1);
  drv->ni_overwrite = overwrite;
  drv->ni_head = 0;
  drv->ni_tail = 0;
  drv->ni_read = 0;

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
