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
#include <nuttx/sched_note.h>

#ifdef CONFIG_SCHED_INSTRUMENTATION_BUFFER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct note_info_s
{
  unsigned int ni_head;
  unsigned int ni_tail;
  uint8_t ni_buffer[CONFIG_SCHED_NOTE_BUFSIZE];
};

struct note_startalloc_s
{
  uint8_t nsa_length;
  uint8_t nsa_type;
  uint8_t nsa_systime[4];
  uint8_t nsa_pid[2];
#if CONFIG_TASK_NAME_SIZE > 0
  char    nsa_name[CONFIG_TASK_NAME_SIZE + 1];
#endif
};

#if CONFIG_TASK_NAME_SIZE > 0
#  define SIZEOF_NOTE_START(n) (sizeof(struct note_start_s) + (n) - 1)
#else
#  define SIZEOF_NOTE_START(n) (sizeof(struct note_start_s))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct note_info_s g_note_info;

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
 * Name: note_systime
 *
 * Description:
 *   Save the current system time in the note structure as a 32-bit value.
 *
 * Input Parameters:
 *   note - The note structure to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void note_systime(FAR struct note_common_s *note)
{
  uint32_t systime = (uint32_t)clock_systimer();

  /* Save the LS 32-bits of the system timer in little endian order */

  note->nc_systime[0] = (uint8_t)( systime        & 0xff);
  note->nc_systime[1] = (uint8_t)((systime >> 8)  & 0xff);
  note->nc_systime[2] = (uint8_t)((systime >> 16) & 0xff);
  note->nc_systime[3] = (uint8_t)((systime >> 24) & 0xff);
}

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

#ifdef CONFIG_DEBUG
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
#endif

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

  note.nsa_length = length;
  note.nsa_type   = NOTE_START;
  note.nsa_pid[0] = (uint8_t)(tcb->pid & 0xff);
  note.nsa_pid[1] = (uint8_t)((tcb->pid >> 8) & 0xff);

  note_systime((FAR struct note_common_s *)&note);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, length);
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
  struct note_stop_s note;

  /* Format the note */

  note.nsp_length = sizeof(struct note_stop_s);
  note.nsp_type   = NOTE_STOP;
  note.nsp_pid[0] = (uint8_t)(tcb->pid & 0xff);
  note.nsp_pid[1] = (uint8_t)((tcb->pid >> 8) & 0xff);

  note_systime((FAR struct note_common_s *)&note);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_stop_s));
}

void sched_note_switch(FAR struct tcb_s *fromtcb, FAR struct tcb_s *totcb)
{
  struct note_switch_s note;

  /* Format the note */

  note.nsw_length    = sizeof(struct note_switch_s);
  note.nsw_type      = NOTE_SWITCH;
  note.nsw_pidout[0] = (uint8_t)(fromtcb->pid & 0xff);
  note.nsw_pidout[1] = (uint8_t)((fromtcb->pid >> 8) & 0xff);
  note.nsw_pidin[0]  = (uint8_t)(totcb->pid & 0xff);
  note.nsw_pidin[1]  = (uint8_t)((totcb->pid >> 8) & 0xff);

  note_systime((FAR struct note_common_s *)&note);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_switch_s));
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
void sched_note_premption(FAR struct tcb_s *tcb, bool locked)
{
  struct note_preempt_s note;

  /* Format the note */

  note.npr_length    = sizeof(struct note_preempt_s);
  note.npr_type      = locked ? NOTE_PREEMPT_LOCK : NOTE_PREEMPT_UNLOCK;
  note.npr_pid[0]    = (uint8_t)(tcb->pid & 0xff);
  note.npr_pid[1]    = (uint8_t)((tcb->pid >> 8) & 0xff);
  note.npr_count[0]  = (uint8_t)(tcb->lockcount & 0xff);
  note.npr_count[1]  = (uint8_t)((tcb->lockcount >> 8) & 0xff);

  note_systime((FAR struct note_common_s *)&note);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_preempt_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
void sched_note_csection(FAR struct tcb_s *tcb, bool enter)
{
  struct note_preempt_s note;

  /* Format the note */

  note.ncs_length    = sizeof(struct note_preempt_s);
  note.ncs_type      = enter ? NOTE_CSECTION_ENTER : NOTE_CSECTION_LEAVE;
  note.ncs_pid[0]    = (uint8_t)(tcb->pid & 0xff);
  note.ncs_pid[1]    = (uint8_t)((tcb->pid >> 8) & 0xff);
#ifdef CONFIG_SMP
  note.ncs_count[0]  = (uint8_t)(tcb->irqcount & 0xff);
  note.ncs_count[1]  = (uint8_t)((tcb->irqcount >> 8) & 0xff);
#endif

  note_systime((FAR struct note_common_s *)&note);

  /* Add the note to circular buffer */

  note_add((FAR const uint8_t *)&note, sizeof(struct note_preempt_s));
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
 *   None
 *
 * Assumptions:
 *   On success, the length of the return note is provided.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

ssize_t sched_note_get(FAR uint8_t *buffer, size_t buflen)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int remaining;
  unsigned int tail;
  ssize_t notelen;

  DEBUGASSERT(buffer != NULL);
  flags = enter_critical_section();

  /* Get the index to the tail of the circular buffer */

  tail = g_note_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_SCHED_NOTE_BUFSIZE);

  /* Get the length of the note at the tail index */

  note   = (FAR struct note_common_s *)&g_note_info.ni_buffer[tail];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= note_length());

  /* Is the user buffer large enough to hold the note? */

  if (buflen < notelen)
    {
      /* Remove the large note so that we do not get constipated. */

      note_remove();

      /* and return and error */

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

#endif /* CONFIG_SCHED_INSTRUMENTATION_BUFFER */
