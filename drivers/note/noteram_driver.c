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

#include "sched/sched.h"

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
static void sched_note_add(FAR const void *note, size_t notelen);

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
 * Name: noteram_remove_taskname
 *
 * Description:
 *   Remove the task name info corresponding to the specified PID
 *
 * Input Parameters:
 *   PID - Task ID
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
static void noteram_remove_taskname(pid_t pid)
{
  FAR struct noteram_taskname_info_s *ti;
  size_t tilen;
  char *src;
  int sindex;

  ti = noteram_find_taskname(pid);
  if (ti == NULL)
    {
      return;
    }

  tilen = ti->size;
  src = (char *)ti + tilen;
  sindex = src - g_noteram_taskname.buffer;

  memcpy(ti, src, g_noteram_taskname.buffer_used - sindex);
  g_noteram_taskname.buffer_used -= tilen;
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

static unsigned int noteram_length(void)
{
  unsigned int head = g_noteram_info.ni_head;
  unsigned int tail = g_noteram_info.ni_tail;

  if (tail > head)
    {
      head += CONFIG_DRIVER_NOTERAM_BUFSIZE;
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

static void noteram_remove(void)
{
  unsigned int tail;
  unsigned int length;

  /* Get the tail index of the circular buffer */

  tail = g_noteram_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_DRIVER_NOTERAM_BUFSIZE);

  /* Get the length of the note at the tail index */

  length = g_noteram_info.ni_buffer[tail];
  DEBUGASSERT(length <= noteram_length());

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
  if (g_noteram_info.ni_buffer[noteram_next(tail, 1)] == NOTE_STOP)
    {
      uint8_t nc_pid[2];

      /* The name of the task is no longer needed because the task is deleted
       * and the corresponding notes are lost.
       */

#ifdef CONFIG_SMP
      nc_pid[0] = g_noteram_info.ni_buffer[noteram_next(tail, 4)];
      nc_pid[1] = g_noteram_info.ni_buffer[noteram_next(tail, 5)];
#else
      nc_pid[0] = g_noteram_info.ni_buffer[noteram_next(tail, 3)];
      nc_pid[1] = g_noteram_info.ni_buffer[noteram_next(tail, 4)];
#endif

      noteram_remove_taskname(nc_pid[0] + (nc_pid[1] << 8));
    }
#endif

  /* Increment the tail index to remove the entire note from the circular
   * buffer.
   */

  if (g_noteram_info.ni_read == g_noteram_info.ni_tail)
    {
      /* The read index also needs increment. */

      g_noteram_info.ni_read = noteram_next(tail, length);
    }

  g_noteram_info.ni_tail = noteram_next(tail, length);
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
  unsigned int remaining;
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

  /* Loop until the note has been transferred to the user buffer */

  remaining = (unsigned int)notelen;
  while (remaining > 0)
    {
      /* Copy the next byte at the read index */

      *buffer++ = g_noteram_info.ni_buffer[read];

      /* Adjust indices and counts */

      read = noteram_next(read, 1);
      remaining--;
    }

  g_noteram_info.ni_read = read;

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
 * Name: note_spincommon
 *
 * Description:
 *   Common logic for NOTE_SPINLOCK, NOTE_SPINLOCKED, and NOTE_SPINUNLOCK
 *
 * Input Parameters:
 *   tcb  - The TCB containing the information
 *   note - The common note structure to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
static void note_spincommon(FAR struct tcb_s *tcb,
                            FAR volatile spinlock_t *spinlock,
                            int type)
{
  struct note_spinlock_s note;

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_spinlock_s), type);

  sched_note_flatten(note.nsp_spinlock, &spinlock, sizeof(spinlock));
  note.nsp_value = *(uint8_t *)spinlock;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_spinlock_s));
}
#endif

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

static void sched_note_add(FAR const void *note, size_t notelen)
{
  FAR const char *buf = note;
  unsigned int head;
  unsigned int next;
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

  /* Loop until all bytes have been transferred to the circular buffer */

  while (notelen > 0)
    {
      /* Get the next head index.  Would it collide with the current tail
       * index?
       */

      next = noteram_next(head, 1);
      if (next == g_noteram_info.ni_tail)
        {
          if (g_noteram_info.ni_overwrite == NOTERAM_MODE_OVERWRITE_DISABLE)
            {
              /* Stop recording if not in overwrite mode */

              g_noteram_info.ni_overwrite = NOTERAM_MODE_OVERWRITE_OVERFLOW;

#ifdef CONFIG_SMP
              spin_unlock_wo_note(&g_noteram_lock);
#endif
              up_irq_restore(flags);
              return;
            }

          /* Yes, then remove the note at the tail index */

          noteram_remove();
        }

      /* Save the next byte at the head index */

      g_noteram_info.ni_buffer[head] = *buf++;

      head = next;
      notelen--;
    }

  g_noteram_info.ni_head = head;

#ifdef CONFIG_SMP
  spin_unlock_wo_note(&g_noteram_lock);
#endif
  up_irq_restore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

/****************************************************************************
 * Name: sched_ramnote_*
 *
 * Description:
 *   These are the hooks into the scheduling instrumentation logic.  Each
 *   simply formats the note associated with the schedule event and adds
 *   that note to the circular buffer.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

void sched_ramnote_start(FAR struct tcb_s *tcb)
{
  struct note_startalloc_s note;
  unsigned int length;
#if CONFIG_TASK_NAME_SIZE > 0
  int namelen;
#endif

  /* Copy the task name (if possible) and get the length of the note */

#if CONFIG_TASK_NAME_SIZE > 0
  namelen = strlen(tcb->name);

  DEBUGASSERT(namelen <= CONFIG_TASK_NAME_SIZE);
  strlcpy(note.nsa_name, tcb->name, sizeof(note.nsa_name));

  length = SIZEOF_NOTE_START(namelen + 1);
#else
  length = SIZEOF_NOTE_START(0);
#endif

  /* Finish formatting the note */

  note_common(tcb, &note.nsa_cmn, length, NOTE_START);

  /* Add the note to circular buffer */

  sched_note_add(&note, length);
}

void sched_ramnote_stop(FAR struct tcb_s *tcb)
{
  struct note_stop_s note;

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_stop_s), NOTE_STOP);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_stop_s));
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
void sched_ramnote_suspend(FAR struct tcb_s *tcb)
{
  struct note_suspend_s note;

  /* Format the note */

  note_common(tcb, &note.nsu_cmn, sizeof(struct note_suspend_s),
              NOTE_SUSPEND);
  note.nsu_state           = tcb->task_state;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_suspend_s));
}

void sched_ramnote_resume(FAR struct tcb_s *tcb)
{
  struct note_resume_s note;

  /* Format the note */

  note_common(tcb, &note.nre_cmn, sizeof(struct note_resume_s), NOTE_RESUME);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_resume_s));
}
#endif

#ifdef CONFIG_SMP
void sched_ramnote_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_start_s note;

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_start_s),
              NOTE_CPU_START);
  note.ncs_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_start_s));
}

void sched_ramnote_cpu_started(FAR struct tcb_s *tcb)
{
  struct note_cpu_started_s note;

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_started_s),
              NOTE_CPU_STARTED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_started_s));
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
void sched_ramnote_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_pause_s note;

  /* Format the note */

  note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_pause_s),
              NOTE_CPU_PAUSE);
  note.ncp_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_pause_s));
}

void sched_ramnote_cpu_paused(FAR struct tcb_s *tcb)
{
  struct note_cpu_paused_s note;

  /* Format the note */

  note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_paused_s),
              NOTE_CPU_PAUSED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_paused_s));
}

void sched_ramnote_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_resume_s note;

  /* Format the note */

  note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resume_s),
              NOTE_CPU_RESUME);
  note.ncr_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_resume_s));
}

void sched_ramnote_cpu_resumed(FAR struct tcb_s *tcb)
{
  struct note_cpu_resumed_s note;

  /* Format the note */

  note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resumed_s),
              NOTE_CPU_RESUMED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_resumed_s));
}
#endif
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
void sched_ramnote_premption(FAR struct tcb_s *tcb, bool locked)
{
  struct note_preempt_s note;

  /* Format the note */

  note_common(tcb, &note.npr_cmn, sizeof(struct note_preempt_s),
              locked ? NOTE_PREEMPT_LOCK : NOTE_PREEMPT_UNLOCK);
  sched_note_flatten(note.npr_count,
                     &tcb->lockcount, sizeof(tcb->lockcount));

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_preempt_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
void sched_ramnote_csection(FAR struct tcb_s *tcb, bool enter)
{
  struct note_csection_s note;

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_csection_s),
              enter ? NOTE_CSECTION_ENTER : NOTE_CSECTION_LEAVE);
#ifdef CONFIG_SMP
  sched_note_flatten(note.ncs_count, &tcb->irqcount, sizeof(tcb->irqcount));
#endif

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_csection_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
void sched_ramnote_spinlock(FAR struct tcb_s *tcb,
                            FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCK);
}

void sched_ramnote_spinlocked(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCKED);
}

void sched_ramnote_spinunlock(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_UNLOCK);
}

void sched_ramnote_spinabort(FAR struct tcb_s *tcb,
                             FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_ABORT);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_ramnote_syscall_enter(int nr, int argc, va_list ap)
{
  struct note_syscall_enter_s note;
  FAR struct tcb_s *tcb = this_task();
  unsigned int length;
  uintptr_t arg;
  uint8_t *args;
  int i;

  /* Format the note */

  length = SIZEOF_NOTE_SYSCALL_ENTER(argc);
  note_common(tcb, &note.nsc_cmn, length, NOTE_SYSCALL_ENTER);
  DEBUGASSERT(nr <= UCHAR_MAX);
  note.nsc_nr = nr;
  DEBUGASSERT(argc <= MAX_SYSCALL_ARGS);
  note.nsc_argc = argc;

  /* If needed, retrieve the given syscall arguments */

  args = note.nsc_args;
  for (i = 0; i < argc; i++)
    {
      arg = (uintptr_t)va_arg(ap, uintptr_t);
      sched_note_flatten(args, &arg, sizeof(arg));
      args += sizeof(uintptr_t);
    }

  /* Add the note to circular buffer */

  sched_note_add((FAR const uint8_t *)&note, length);
}

void sched_ramnote_syscall_leave(int nr, uintptr_t result)
{
  struct note_syscall_leave_s note;
  FAR struct tcb_s *tcb = this_task();

  /* Format the note */

  note_common(tcb, &note.nsc_cmn, sizeof(struct note_syscall_leave_s),
              NOTE_SYSCALL_LEAVE);
  DEBUGASSERT(nr <= UCHAR_MAX);
  note.nsc_nr = nr;

  sched_note_flatten(note.nsc_result, &result, sizeof(result));

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_syscall_leave_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_ramnote_irqhandler(int irq, FAR void *handler, bool enter)
{
  struct note_irqhandler_s note;
  FAR struct tcb_s *tcb = this_task();

  /* Format the note */

  note_common(tcb, &note.nih_cmn, sizeof(struct note_irqhandler_s),
              enter ? NOTE_IRQ_ENTER : NOTE_IRQ_LEAVE);
  DEBUGASSERT(irq <= UCHAR_MAX);
  note.nih_irq = irq;

  /* Add the note to circular buffer */

  sched_note_add((FAR const uint8_t *)&note,
                 sizeof(struct note_irqhandler_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
void sched_ramnote_write(FAR const void *data, size_t length)
{
  sched_note_add(data, length);
}
#endif
