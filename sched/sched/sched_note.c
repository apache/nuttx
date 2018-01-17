/****************************************************************************
 * sched/sched/sched_note.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

#ifdef CONFIG_SCHED_INSTRUMENTATION_BUFFER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct note_info_s
{
  volatile unsigned int ni_head;
  volatile unsigned int ni_tail;
  uint8_t ni_buffer[CONFIG_SCHED_NOTE_BUFSIZE];
};

struct note_startalloc_s
{
  struct note_common_s nsa_cmn; /* Common note parameters */
#if CONFIG_TASK_NAME_SIZE > 0
  char nsa_name[CONFIG_TASK_NAME_SIZE + 1];
#endif
};

#if CONFIG_TASK_NAME_SIZE > 0
#  define SIZEOF_NOTE_START(n) (sizeof(struct note_start_s) + (n) - 1)
#else
#  define SIZEOF_NOTE_START(n) (sizeof(struct note_start_s))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void note_add(FAR const uint8_t *note, uint8_t notelen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct note_info_s g_note_info;

#ifdef CONFIG_SMP
static volatile spinlock_t g_note_lock;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_next
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

static inline unsigned int note_next(unsigned int ndx, unsigned int offset)
{
  ndx += offset;
  if (ndx >= CONFIG_SCHED_NOTE_BUFSIZE)
    {
      ndx -= CONFIG_SCHED_NOTE_BUFSIZE;
    }

  return ndx;
}

/****************************************************************************
 * Name: note_common
 *
 * Description:
 *   Fill in some of the common fields in the note structure.
 *
 * Input Parameters:
 *   tcb    - The TCB containing the information
 *   note   - The common note structure to use
 *   length - The total lengthof the note structure
 *   type   - The type of the note
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void note_common(FAR struct tcb_s *tcb, FAR struct note_common_s *note,
                        uint8_t length, uint8_t type)
{
  uint32_t systime    = (uint32_t)clock_systimer();

  /* Save all of the common fields */

  note->nc_length     = length;
  note->nc_type       = type;
  note->nc_priority   = tcb->sched_priority;
#ifdef CONFIG_SMP
  note->nc_cpu        = tcb->cpu;
#endif
  note->nc_pid[0]     = (uint8_t)(tcb->pid & 0xff);
  note->nc_pid[1]     = (uint8_t)((tcb->pid >> 8) & 0xff);

  /* Save the LS 32-bits of the system timer in little endian order */

  note->nc_systime[0] = (uint8_t)( systime        & 0xff);
  note->nc_systime[1] = (uint8_t)((systime >> 8)  & 0xff);
  note->nc_systime[2] = (uint8_t)((systime >> 16) & 0xff);
  note->nc_systime[3] = (uint8_t)((systime >> 24) & 0xff);
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
void note_spincommon(FAR struct tcb_s *tcb, FAR volatile spinlock_t *spinlock,
                     int type)
{
  struct note_spinlock_s note;

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_spinlock_s), type);
  note.nsp_spinlock = (FAR void *)spinlock;
  note.nsp_value    = (uint8_t)*spinlock;

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_spinlock_s));
}
#endif

/****************************************************************************
 * Name: note_length
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

static unsigned int note_length(void)
{
  unsigned int head = g_note_info.ni_head;
  unsigned int tail = g_note_info.ni_tail;

  if (tail > head)
    {
      head += CONFIG_SCHED_NOTE_BUFSIZE;
    }

  return head - tail;
}

/****************************************************************************
 * Name: note_remove
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

static void note_remove(void)
{
  FAR struct note_common_s *note;
  unsigned int tail;
  unsigned int length;

  /* Get the tail index of the circular buffer */

  tail = g_note_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_SCHED_NOTE_BUFSIZE);

  /* Get the length of the note at the tail index */

  note   = (FAR struct note_common_s *)&g_note_info.ni_buffer[tail];
  length = note->nc_length;
  DEBUGASSERT(length <= note_length());

  /* Increment the tail index to remove the entire note from the circular
   * buffer.
   */

  g_note_info.ni_tail = note_next(tail, length);
}

/****************************************************************************
 * Name: note_add
 *
 * Description:
 *   Add the variable length note to the head of the circular buffer
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

static void note_add(FAR const uint8_t *note, uint8_t notelen)
{
  unsigned int head;
  unsigned int next;

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
  spin_lock_wo_note(&g_note_lock);
#endif

  /* Get the index to the head of the circular buffer */

  DEBUGASSERT(note != NULL && notelen < CONFIG_SCHED_NOTE_BUFSIZE);
  head = g_note_info.ni_head;

  /* Loop until all bytes have been transferred to the circular buffer */

  while (notelen > 0)
    {
      /* Get the next head index.  Would it collide with the current tail
       * index?
       */

      next = note_next(head, 1);
      if (next == g_note_info.ni_tail)
        {
          /* Yes, then remove the note at the tail index */

          note_remove();
        }

      /* Save the next byte at the head index */

      g_note_info.ni_buffer[head] = *note++;

      head = next;
      notelen--;
    }

  g_note_info.ni_head = head;

#ifdef CONFIG_SMP
  spin_unlock_wo_note(&g_note_lock);
  up_irq_restore(flags);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_*
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

void sched_note_start(FAR struct tcb_s *tcb)
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
  strncpy(note.nsa_name, tcb->name, CONFIG_TASK_NAME_SIZE + 1);

  length = SIZEOF_NOTE_START(namelen + 1);
#else
  length = SIZEOF_NOTE_START(0)
#endif

  /* Finish formatting the note */

  note_common(tcb, &note.nsa_cmn, length, NOTE_START);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, length);
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
  struct note_stop_s note;

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_stop_s), NOTE_STOP);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_stop_s));
}

void sched_note_suspend(FAR struct tcb_s *tcb)
{
  struct note_suspend_s note;

  /* Format the note */

  note_common(tcb, &note.nsu_cmn, sizeof(struct note_suspend_s), NOTE_SUSPEND);
  note.nsu_state           = tcb->task_state;

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_suspend_s));
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
  struct note_resume_s note;

  /* Format the note */

  note_common(tcb, &note.nre_cmn, sizeof(struct note_resume_s), NOTE_RESUME);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_resume_s));
}

#ifdef CONFIG_SMP
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_start_s note;

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_start_s),
              NOTE_CPU_START);
  note.ncs_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_cpu_start_s));
}

void sched_note_cpu_started(FAR struct tcb_s *tcb)
{
  struct note_cpu_started_s note;

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_started_s),
              NOTE_CPU_STARTED);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_cpu_started_s));
}

void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_pause_s note;

  /* Format the note */

  note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_pause_s),
              NOTE_CPU_PAUSE);
  note.ncp_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_cpu_pause_s));
}

void sched_note_cpu_paused(FAR struct tcb_s *tcb)
{
  struct note_cpu_paused_s note;

  /* Format the note */

  note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_paused_s),
              NOTE_CPU_PAUSED);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_cpu_paused_s));
}

void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_resume_s note;

  /* Format the note */

  note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resume_s),
              NOTE_CPU_RESUME);
  note.ncr_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_cpu_resume_s));
}

void sched_note_cpu_resumed(FAR struct tcb_s *tcb)
{
  struct note_cpu_resumed_s note;

  /* Format the note */

  note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resumed_s),
              NOTE_CPU_RESUMED);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_cpu_resumed_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
void sched_note_premption(FAR struct tcb_s *tcb, bool locked)
{
  struct note_preempt_s note;

  /* Format the note */

  note_common(tcb, &note.npr_cmn, sizeof(struct note_preempt_s),
              locked ? NOTE_PREEMPT_LOCK : NOTE_PREEMPT_UNLOCK);
  note.npr_count[0] = (uint8_t)(tcb->lockcount & 0xff);
  note.npr_count[1] = (uint8_t)((tcb->lockcount >> 8) & 0xff);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_preempt_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
void sched_note_csection(FAR struct tcb_s *tcb, bool enter)
{
  struct note_csection_s note;

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_csection_s),
              enter ? NOTE_CSECTION_ENTER : NOTE_CSECTION_LEAVE);
#ifdef CONFIG_SMP
  note.ncs_count[0] = (uint8_t)(tcb->irqcount & 0xff);
  note.ncs_count[1] = (uint8_t)((tcb->irqcount >> 8) & 0xff);
#endif

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_csection_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
void sched_note_spinlock(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCK);
}

void sched_note_spinlocked(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCKED);
}

void sched_note_spinunlock(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_UNLOCK);
}
void sched_note_spinabort(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_ABORT);
}
#endif

/****************************************************************************
 * Name: sched_note_get
 *
 * Description:
 *   Remove the next note from the tail of the circular buffer.  The note
 *   is also removed from the circular buffer to make room for futher notes.
 *
 * Input Parameters:
 *   buffer - Location to return the next note
 *   buflen - The length of the user provided buffer.
 *
 * Returned Value:
 *   On success, the positive, non-zero length of the return note is
 *   provided.  Zero is returned only if ther circular buffer is empty.  A
 *   negated errno value is returned in the event of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_NOTE_GET
ssize_t sched_note_get(FAR uint8_t *buffer, size_t buflen)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int remaining;
  unsigned int tail;
  ssize_t notelen;
  size_t circlen;

  DEBUGASSERT(buffer != NULL);
  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = note_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the index to the tail of the circular buffer */

  tail    = g_note_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_SCHED_NOTE_BUFSIZE);

  /* Get the length of the note at the tail index */

  note    = (FAR struct note_common_s *)&g_note_info.ni_buffer[tail];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

  /* Is the user buffer large enough to hold the note? */

  if (buflen < notelen)
    {
      /* Remove the large note so that we do not get constipated. */

      note_remove();

      /* and return an error */

      notelen = -EFBIG;
      goto errout_with_csection;
    }

  /* Loop until the note has been transferred to the user buffer */

  remaining = (unsigned int)notelen;
  while (remaining > 0)
    {
      /* Copy the next byte at the tail index */

      *buffer++ = g_note_info.ni_buffer[tail];

      /* Adjust indices and counts */

      tail = note_next(tail, 1);
      remaining--;
    }

  g_note_info.ni_tail = tail;

errout_with_csection:
  leave_critical_section(flags);
  return notelen;
}
#endif

/****************************************************************************
 * Name: sched_note_size
 *
 * Description:
 *   Return the size of the next note at the tail of the circular buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero is returned if the circular buffer is empty.  Otherwise, the size
 *   of the next note is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_NOTE_GET
ssize_t sched_note_size(void)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int tail;
  ssize_t notelen;
  size_t circlen;

  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = note_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the index to the tail of the circular buffer */

  tail = g_note_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_SCHED_NOTE_BUFSIZE);

  /* Get the length of the note at the tail index */

  note    = (FAR struct note_common_s *)&g_note_info.ni_buffer[tail];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

errout_with_csection:
  leave_critical_section(flags);
  return notelen;
}
#endif

#endif /* CONFIG_SCHED_INSTRUMENTATION_BUFFER */
