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
#include <string.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct noteram_info_s
{
  unsigned int ni_overwrite;
  volatile unsigned int ni_head;
  volatile unsigned int ni_tail;
  volatile unsigned int ni_read;
  uint8_t ni_buffer[CONFIG_DRIVER_NOTERAM_BUFSIZE];
};

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
struct noteram_taskname_info_s
{
  uint8_t size;
  uint8_t pid[2];
  char name[1];
};

struct noteram_taskname_s
{
  int buffer_used;
  char buffer[CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE];
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int noteram_open(FAR struct file *filep);
static ssize_t noteram_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen);
static int noteram_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_noteram_fops =
{
  noteram_open,  /* open */
  NULL,          /* close */
  noteram_read,  /* read */
  NULL,          /* write */
  NULL,          /* seek */
  noteram_ioctl, /* ioctl */
  NULL           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

static struct noteram_info_s g_noteram_info =
{
#ifdef CONFIG_DRIVER_NOTERAM_DEFAULT_NOOVERWRITE
  NOTERAM_MODE_OVERWRITE_DISABLE
#else
  NOTERAM_MODE_OVERWRITE_ENABLE
#endif
};

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
static struct noteram_taskname_s g_noteram_taskname;
#endif

#ifdef CONFIG_SMP
static volatile spinlock_t g_noteram_lock;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noteram_find_taskname
 *
 * Description:
 *   Find task name info corresponding to the specified PID
 *
 * Input Parameters:
 *   PID - Task ID
 *
 * Returned Value:
 *   Pointer to the task name info
 *   If the corresponding info doesn't exist in the buffer, NULL is returned.
 *
 ****************************************************************************/

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
static FAR struct noteram_taskname_info_s *noteram_find_taskname(pid_t pid)
{
  int n;
  FAR struct noteram_taskname_info_s *ti;

  for (n = 0; n < g_noteram_taskname.buffer_used; )
    {
      ti = (FAR struct noteram_taskname_info_s *)
            &g_noteram_taskname.buffer[n];
      if (ti->pid[0] + (ti->pid[1] << 8) == pid)
        {
          return ti;
        }

      n += ti->size;
    }

  return NULL;
}
#endif

/****************************************************************************
 * Name: noteram_record_taskname
 *
 * Description:
 *   Record the task name info of the specified task
 *
 * Input Parameters:
 *   PID - Task ID
 *   name - task name
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
static void noteram_record_taskname(pid_t pid, const char *name)
{
  FAR struct noteram_taskname_info_s *ti;
  size_t tilen;
  size_t namelen;

  namelen = strlen(name);
  DEBUGASSERT(namelen <= CONFIG_TASK_NAME_SIZE);
  tilen = sizeof(struct noteram_taskname_info_s) + namelen;
  DEBUGASSERT(tilen <= UCHAR_MAX);

  if (g_noteram_taskname.buffer_used + tilen >
      CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE)
    {
      /* No space in the buffer - ignored */

      return;
    }

  ti = (FAR struct noteram_taskname_info_s *)
        &g_noteram_taskname.buffer[g_noteram_taskname.buffer_used];
  ti->size = tilen;
  ti->pid[0] = pid & 0xff;
  ti->pid[1] = (pid >> 8) & 0xff;
  strlcpy(ti->name, name, namelen + 1);
  g_noteram_taskname.buffer_used += tilen;
}
#endif

/****************************************************************************
 * Name: noteram_get_taskname
 *
 * Description:
 *   Get the task name string of the specified PID
 *
 * Input Parameters:
 *   PID - Task ID
 *
 * Returned Value:
 *   Pointer to the task name string
 *   If the corresponding name doesn't exist in the buffer, NULL is returned.
 *
 ****************************************************************************/

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
static const char *noteram_get_taskname(pid_t pid)
{
  irqstate_t irq_mask;
  const char *ret = NULL;
  FAR struct noteram_taskname_info_s *ti;
  FAR struct tcb_s *tcb;

  irq_mask = enter_critical_section();

  ti = noteram_find_taskname(pid);
  if (ti != NULL)
    {
      ret = ti->name;
    }
  else
    {
      tcb = nxsched_get_tcb(pid);
      if (tcb != NULL)
        {
          noteram_record_taskname(pid, tcb->name);
          ret = tcb->name;
        }
    }

  leave_critical_section(irq_mask);
  return ret;
}
#endif

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

static void noteram_buffer_clear(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  g_noteram_info.ni_tail = g_noteram_info.ni_head;
  g_noteram_info.ni_read = g_noteram_info.ni_head;

  if (g_noteram_info.ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
      g_noteram_info.ni_overwrite = NOTERAM_MODE_OVERWRITE_DISABLE;
    }

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
  g_noteram_taskname.buffer_used = 0;
#endif

  leave_critical_section(flags);
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

static inline unsigned int noteram_next(unsigned int ndx,
                                        unsigned int offset)
{
  ndx += offset;
  if (ndx >= CONFIG_DRIVER_NOTERAM_BUFSIZE)
    {
      ndx -= CONFIG_DRIVER_NOTERAM_BUFSIZE;
    }

  return ndx;
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

static unsigned int noteram_unread_length(void)
{
  unsigned int head = g_noteram_info.ni_head;
  unsigned int read = g_noteram_info.ni_read;

  if (read > head)
    {
      head += CONFIG_DRIVER_NOTERAM_BUFSIZE;
    }

  return head - read;
}

/****************************************************************************
 * Name: noteram_current_available
 *
 * Description:
 *   Get contiguous unused size in circular buffer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   total unused size
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

static inline unsigned int noteram_current_available(unsigned int size)
{
  unsigned int head = g_noteram_info.ni_head;
  head += size;

  if (head >= CONFIG_DRIVER_NOTERAM_BUFSIZE)
    {
      size = head - CONFIG_DRIVER_NOTERAM_BUFSIZE;
    }

  return size;
}

/****************************************************************************
 * Name: noteram_total_unused
 *
 * Description:
 *   Get the total unused size in circular buffer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   total unused size
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

static inline unsigned int noteram_total_unused(void)
{
  return CONFIG_DRIVER_NOTERAM_BUFSIZE - noteram_unread_length();
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

static ssize_t noteram_get(FAR uint8_t *buffer, size_t buflen)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int real;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

  DEBUGASSERT(buffer != NULL);
  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = noteram_unread_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the read index of the circular buffer */

  read    = g_noteram_info.ni_read;
  DEBUGASSERT(read < CONFIG_DRIVER_NOTERAM_BUFSIZE);

  /* Get the length of the note at the read index */

  note    = (FAR struct note_common_s *)&g_noteram_info.ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

  /* Is the user buffer large enough to hold the note? */

  if (buflen < notelen)
    {
      /* Skip the large note so that we do not get constipated. */

      g_noteram_info.ni_read = noteram_next(read, notelen);

      /* and return an error */

      notelen = -EFBIG;
      goto errout_with_csection;
    }

  /* Read the note to the user buffer */

  g_noteram_info.ni_read = noteram_next(read, notelen);
  real = noteram_current_available(notelen);
  memcpy(buffer, &g_noteram_info.ni_buffer[read], real);
  memcpy(buffer + real, g_noteram_info.ni_buffer, notelen - real);

errout_with_csection:
  leave_critical_section(flags);
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

static ssize_t noteram_size(void)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = noteram_unread_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the read index of the circular buffer */

  read = g_noteram_info.ni_read;
  DEBUGASSERT(read < CONFIG_DRIVER_NOTERAM_BUFSIZE);

  /* Get the length of the note at the read index */

  note    = (FAR struct note_common_s *)&g_noteram_info.ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

errout_with_csection:
  leave_critical_section(flags);
  return notelen;
}

/****************************************************************************
 * Name: noteram_open
 ****************************************************************************/

static int noteram_open(FAR struct file *filep)
{
  /* Reset the read index of the circular buffer */

  g_noteram_info.ni_read = g_noteram_info.ni_tail;

  return OK;
}

/****************************************************************************
 * Name: noteram_read
 ****************************************************************************/

static ssize_t noteram_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen)
{
  ssize_t notelen;
  ssize_t retlen ;

  DEBUGASSERT(filep != 0 && buffer != NULL && buflen > 0);

  /* Then loop, adding as many notes as possible to the user buffer. */

  retlen = 0;
  sched_lock();
  do
    {
      /* Get the next note (removing it from the buffer) */

      notelen = noteram_get((FAR uint8_t *)buffer, buflen);
      if (notelen < 0)
        {
          /* We were unable to read the next note, probably because it will
           * not fit into the user buffer.
           */

          if (retlen == 0)
            {
              /* If nothing was read then report the error.  Otherwise,
               * just silently drop the note.
               */

              retlen = notelen;
            }

          break;
        }

      /* Update pointers from the note that was transferred */

      retlen += notelen;
      buffer += notelen;
      buflen -= notelen;

      /* Will the next note fit?  There is a race here and even if the next
       * note will fit, it may fail still when noteram_get() is called.
       *
       * It won't fit (or an error occurred).  Return what we have without
       * trying to get the next note (which would cause it to be deleted).
       */

      notelen = noteram_size();
    }
  while (notelen > 0 && notelen <= buflen);

  sched_unlock();
  return retlen;
}

/****************************************************************************
 * Name: noteram_ioctl
 ****************************************************************************/

static int noteram_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOSYS;

  /* Handle the ioctl commands */

  switch (cmd)
    {
      /* NOTERAM_CLEAR
       *      - Clear all contents of the circular buffer
       *        Argument: Ignored
       */

      case NOTERAM_CLEAR:
        noteram_buffer_clear();
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
            *(unsigned int *)arg = g_noteram_info.ni_overwrite;
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
            g_noteram_info.ni_overwrite = *(unsigned int *)arg;
            ret = OK;
          }
        break;

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
      /* NOTERAM_GETTASKNAME
       *      - Get task name string
       *        Argument: A writable pointer to struct note_get_taskname_s
       *        Result:   If -ESRCH, the corresponding task name doesn't
       *                  exist.
       */

      case NOTERAM_GETTASKNAME:
        {
          struct noteram_get_taskname_s *param;
          const char *taskname;

        if (arg == 0)
          {
            ret = -EINVAL;
            break;
          }

          param = (struct noteram_get_taskname_s *)arg;
          taskname = noteram_get_taskname(param->pid);
          if (taskname != NULL)
            {
              strlcpy(param->taskname, taskname, CONFIG_TASK_NAME_SIZE + 1);
              param->taskname[CONFIG_TASK_NAME_SIZE] = '\0';
              ret = 0;
            }
          else
            {
              param->taskname[0] = '\0';
              ret = -ESRCH;
            }
        }
        break;
#endif

      default:
          break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_add
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

void sched_note_add(FAR const void *note, size_t notelen)
{
  unsigned int head;
  unsigned int real;
  irqstate_t flags;

  flags = up_irq_save();
#ifdef CONFIG_SMP
  spin_lock_wo_note(&g_noteram_lock);
#endif

  if (g_noteram_info.ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
#ifdef CONFIG_SMP
      spin_unlock_wo_note(&g_noteram_lock);
#endif
      up_irq_restore(flags);
      return;
    }

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
  /* Record the name if the new task was created */

    {
      FAR struct note_start_s *note_st;

      note_st = (FAR struct note_start_s *)note;
      if (note_st->nst_cmn.nc_type == NOTE_START)
        {
          noteram_record_taskname(note_st->nst_cmn.nc_pid[0] +
                                  (note_st->nst_cmn.nc_pid[1] << 8),
                                  note_st->nst_name);
        }
    }
#endif

  /* Get the index to the head of the circular buffer */

  DEBUGASSERT(note != NULL && notelen < CONFIG_DRIVER_NOTERAM_BUFSIZE);
  head = g_noteram_info.ni_head;

  if (g_noteram_info.ni_overwrite == NOTERAM_MODE_OVERWRITE_DISABLE)
    {
      unsigned int available = noteram_total_unused();
      if (notelen > available)
        {
          notelen = available;
        }
    }

  /* Read all bytes to the circular buffer */

  g_noteram_info.ni_head = noteram_next(head, notelen);
  real = noteram_current_available(notelen);
  memcpy(&g_noteram_info.ni_buffer[head], note, real);
  memcpy(g_noteram_info.ni_buffer, note + real, notelen - real);

#ifdef CONFIG_SMP
  spin_unlock_wo_note(&g_noteram_lock);
#endif
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: noteram_register
 *
 * Description:
 *   Register a serial driver at /dev/note that can be used by an
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
  return register_driver("/dev/note", &g_noteram_fops, 0666, NULL);
}
