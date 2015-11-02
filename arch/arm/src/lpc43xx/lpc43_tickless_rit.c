/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_rit.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 *
 * only controlled resets to 0 are performed, no direct set to counter
 * working counter region is from 0 to to_end
 * all public functions are synchronized with disabled irqs
 *
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <errno.h>
#include <time.h>

#include "up_arch.h"
#include "chip.h"
#include "chip/lpc43_rit.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef min
#  define min(a,b) (a < b ? a : b)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t to_reset = UINT32_MAX / 2;
static uint32_t to_reset_next = UINT32_MAX / 2 + UINT32_MAX / 4;
static uint32_t to_end = UINT32_MAX / 2 + UINT32_MAX / 4 + UINT32_MAX / 8;

static uint32_t reset_ticks = 1000; /* ticks to add to force a reset */

static uint64_t base = 0; /* time base */

static uint64_t alarm_time = 0; /* alarmTime to set on next interrupt, used if not already armed */
static bool call = false; /* true if callback should be called on next interrupt */
static bool forced_int = false; /* true if interrupt was forced with mask, no reset */
static bool armed = false; /* true if alarm is armed for next match */
static uint32_t synch = 0; /* synch all calls, recursion is possible */
static irqstate_t g_flags;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* some timer HW functions */

static inline void lpc43_tl_set_counter (uint32_t value)
{
  putreg32(value, LPC43_RIT_COUNTER);
}

static inline uint32_t lpc43_tl_get_counter (void)
{
  return getreg32(LPC43_RIT_COUNTER);
}

static inline void lpc43_tl_set_compare (uint32_t value)
{
  putreg32(value, LPC43_RIT_COMPVAL);
}

static inline uint32_t lpc43_tl_get_compare (void)
{
  return getreg32(LPC43_RIT_COMPVAL);
}

static inline void lpc43_tl_set_mask (uint32_t value)
{
  putreg32(value, LPC43_RIT_MASK);
}

static inline uint32_t lpc43_tl_get_mask (void)
{
  return getreg32(LPC43_RIT_MASK);
}

static inline void lpc43_tl_set_ctrl_bit (uint32_t bit, bool value)
{
  uint32_t regval;
  regval = getreg32(LPC43_RIT_CTRL);
  if (value)
    {
      regval |= bit;
    }
  else
    {
      regval &= ~bit;
    }
  putreg32(regval, LPC43_RIT_CTRL);
}

static inline bool lpc43_tl_get_ctrl_bit (uint32_t bit)
{
  uint32_t regval;
  regval = getreg32(LPC43_RIT_CTRL);
  regval &= bit;
  if (regval)
    {
      return true;
    }
  else
    {
      return false;
    }
}

static inline void lpc43_tl_set_reset_on_match (bool value)
{
  lpc43_tl_set_ctrl_bit (RIT_CTRL_ENCLR, value);
}

static inline bool lpc43_tl_get_reset_on_match (void)
{
  return lpc43_tl_get_ctrl_bit (RIT_CTRL_ENCLR);
}

static inline void lpc43_tl_set_enable (bool value)
{
  lpc43_tl_set_ctrl_bit (RIT_CTRL_EN, value);
}

static inline bool lpc43_tl_get_enable (void)
{
  return lpc43_tl_get_ctrl_bit (RIT_CTRL_EN);
}

static inline void lpc43_tl_clear_interrupt (void)
{
  lpc43_tl_set_ctrl_bit (RIT_CTRL_INT, true);
}

static inline bool lpc43_tl_get_interrupt (void)
{
  return lpc43_tl_get_ctrl_bit (RIT_CTRL_INT);
}

/* converters */

static inline uint64_t lpc43_tl_ts2tick ( FAR const struct timespec *ts)
  {
    return (  (uint64_t)(ts->tv_sec)*LPC43_CCLK + ( (uint64_t)(ts->tv_nsec)*LPC43_CCLK/NSEC_PER_SEC) );
  }

static inline void lpc43_tl_tick2ts (uint64_t tick, FAR struct timespec *ts)
  {
    ts->tv_sec = tick/LPC43_CCLK;
    ts->tv_nsec = (tick%LPC43_CCLK)*NSEC_PER_SEC/LPC43_CCLK;
  }

/* logic functions */

static void lpc43_tl_sync_up (void) {
  irqstate_t flags;
  flags = irqsave ();

  if ( synch == 0 )
    {
      g_flags = flags;

    }
  synch++;
}

static void lpc43_tl_sync_down (void) {
  synch--;
  if ( synch == 0 )
    {
      irqrestore (g_flags);
    }
}

/* assuming safe timer state, force interrupt, no reset possible */

static void lpc43_tl_force_int (void)
{
  forced_int = true;
  lpc43_tl_set_reset_on_match (false);
  lpc43_tl_set_mask (UINT32_MAX);
  lpc43_tl_set_compare(UINT32_MAX);
}

/* init all vars, forced_int should not be cleared */

static void lpc43_tl_init_timer_vars (void)
{
  alarm_time = 0;
  call = false;
  armed = false;

}

/* calc reset_ticks and set compare to to_reset */

static void lpc43_tl_calibrate_init (void)
{
  uint32_t counter = lpc43_tl_get_counter ();

  uint32_t counter_after = lpc43_tl_get_counter ();
  counter_after = to_reset + counter;
  counter_after = counter_after - counter;

  /*shift to toReset*/

  lpc43_tl_set_compare (counter_after);

  counter_after = lpc43_tl_get_counter ();

  reset_ticks = (counter_after - counter) * 2;

}

/* process current and set timer in default safe state */

static void lpc43_tl_save_timer (void)
{
  if (forced_int) /* special case of forced interrupt by mask*/
    {
	forced_int = false;
	lpc43_tl_set_compare (UINT32_MAX);
	lpc43_tl_set_mask (0);
	lpc43_tl_clear_interrupt ();
   }
  else
   {

    /*process reset if any*/

    uint32_t match = lpc43_tl_get_compare ();

    /*move to end, no resets during processing*/

    lpc43_tl_set_compare (UINT32_MAX);
    lpc43_tl_set_mask (0);

    if (lpc43_tl_get_interrupt ())
      {
	if (lpc43_tl_get_reset_on_match ()) /*was reset ?*/
	  {
	    base = base + match;
	    lpc43_tl_set_reset_on_match (false);
	  }
	lpc43_tl_clear_interrupt ();
      }
   }
}

/* assuming safe timer state, true if set, false - time is in the past */

static bool lpc43_tl_set_safe_compare (uint32_t compare_to_set)
{
  if (compare_to_set < to_reset)
    {
      lpc43_tl_set_reset_on_match (false);
    }
  else
    {
      lpc43_tl_set_reset_on_match (true);
    }

  lpc43_tl_set_compare (compare_to_set);

  //check if ok
  bool reset = lpc43_tl_get_interrupt ();
  uint32_t counter = lpc43_tl_get_counter ();
  bool reset_after = lpc43_tl_get_interrupt ();

  if (reset != reset_after)
    {
      //was a reset get new counter
      counter = lpc43_tl_get_counter ();
    }

  if (reset_after || (!reset_after && compare_to_set > counter))
    {
      return true;
    }
  else
    {
      lpc43_tl_set_reset_on_match (false);
      lpc43_tl_set_compare (UINT32_MAX);

      return false;
    }

}

/* assuming safe timer state, set_safe_compare in loop */

static void lpc43_tl_looped_forced_set_compare (void)
{
  uint32_t i = 1;
  bool result = lpc43_tl_set_safe_compare (
      lpc43_tl_get_counter () + reset_ticks); /* like in calibrateInit */
  while (!result)
    {
      i++;
      result = lpc43_tl_set_safe_compare (
	  lpc43_tl_get_counter () + reset_ticks * i);
    };
}

/* assuming safe timer state, true if set, false - time is in the past */

static bool lpc43_tl_set_calc_arm (uint32_t curr, uint32_t to_set, bool arm)
{

  uint32_t calcTime;

  if (curr < to_reset_next)
    {
      calcTime = min(to_reset_next, to_set);
    }
  else
    {
      if (curr < to_end)
	{
	  calcTime = min(lpc43_tl_get_counter () + reset_ticks, to_set);
	}
      else
	{
	  lpc43_tl_looped_forced_set_compare ();
	  return true;
	}
    }

  bool set = lpc43_tl_set_safe_compare (calcTime);

  if (arm && set && (calcTime == to_set))
    {
      armed = true;
    }

  return set;
}

/* assuming safe timer state, try to set compare for normal operation */

static void lpc43_tl_set_default_compare (void)
{
  bool result = lpc43_tl_set_calc_arm (lpc43_tl_get_counter (), UINT32_MAX,
  false);
  if (!result)
    {
      result = lpc43_tl_set_calc_arm (lpc43_tl_get_counter (), UINT32_MAX,
      false);
      if (!result)
	{
	  lpc43_tl_looped_forced_set_compare ();
	}
    }

}

/* assuming safe timer state, used by isr: sets default compare , calls alarm */
static inline void lpc43_tl_alarm (void)
{
  lpc43_tl_init_timer_vars ();
  lpc43_tl_set_default_compare ();

  struct timespec ts;

  up_timer_gettime (&ts);

#ifdef CONFIG_SCHED_TICKLESS_ALARM
  sched_alarm_expiration (&ts);
#else
  sched_timer_expiration();
#endif
}

/* interrupt handler */

static int lpc43_tl_isr (int irq, FAR void *context)
  {
    lpc43_tl_sync_up();

    lpc43_tl_save_timer();

    if (call)
      {
	lpc43_tl_alarm();
      }
    else
      {
	if (armed)
	  {
	    lpc43_tl_alarm(); /* armed - call alarm */
	  }
	else
	  {
	    if (alarm_time > 0) /* need to set alarm time */
	      {
		int64_t toSet = alarm_time - base;
		uint32_t curr = lpc43_tl_get_counter ();

		if (toSet > curr)
		  {
		    if (toSet > to_end)
		      {
			lpc43_tl_set_default_compare ();
		      }
		    else
		      {
			bool set = lpc43_tl_set_calc_arm (curr, toSet, true);
			if (!set)
			  {
			    lpc43_tl_alarm();
			  }
		      }
		  }
		else
		  {
		    lpc43_tl_alarm();
		  }
	      }
	    else
	      {
		lpc43_tl_set_default_compare ();
	      }
	  }
      }

    lpc43_tl_sync_down();

    return OK;
  }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize (void)
{
  irqstate_t flags;
  flags = irqsave ();

  lpc43_tl_set_enable (false);

  lpc43_tl_set_compare (UINT32_MAX);
  lpc43_tl_set_counter (0);
  lpc43_tl_set_mask (0);

  lpc43_tl_set_reset_on_match (false);
  lpc43_tl_clear_interrupt ();

  irq_attach (LPC43M4_IRQ_RITIMER, lpc43_tl_isr);
  up_enable_irq (LPC43M4_IRQ_RITIMER);

  lpc43_tl_init_timer_vars ();

  lpc43_tl_set_enable (true);

  lpc43_tl_calibrate_init ();

  irqrestore (flags);
}

int up_timer_gettime (FAR struct timespec *ts)
  {
    lpc43_tl_sync_up();

    uint64_t curr;
    /* no reg changes, only processing */

    /* order of calls is important, reset can come during processing */

    bool reset = lpc43_tl_get_interrupt ();
    uint32_t count = lpc43_tl_get_counter ();

    /* not processed reset can exist */
    if (lpc43_tl_get_reset_on_match ())
      {
	bool resetAfter = lpc43_tl_get_interrupt ();

	if (reset != resetAfter) /* was a reset get new counter */
	  {
	    count = lpc43_tl_get_counter ();
	  }

	if (resetAfter)
	  {
	    curr = base + count + lpc43_tl_get_compare ();
	  }
	else
	  {
	    curr = base + count;
	  }
      }
    else
      {
	curr = base + count;
      }

    lpc43_tl_tick2ts (curr, ts);

    lpc43_tl_sync_down();
    return OK;
  }

# ifdef CONFIG_SCHED_TICKLESS_ALARM
int up_alarm_cancel (FAR struct timespec *ts)
  {
    lpc43_tl_sync_up();

    /*no reg changes, only variables logic*/

    if ( ts != NULL ) {
	up_timer_gettime (ts);
    }

    /* let default setup will be done in interrupt handler or up_alarm_start */
    lpc43_tl_init_timer_vars ();

    lpc43_tl_sync_down();
    return OK;
  }

int up_alarm_start (FAR const struct timespec *ts)
  {
    lpc43_tl_sync_up();

    lpc43_tl_save_timer ();

    lpc43_tl_init_timer_vars ();

    uint64_t alarm_time = lpc43_tl_ts2tick (ts);
    int64_t toSet = alarm_time - base;
    uint32_t curr = lpc43_tl_get_counter ();

    if (toSet > curr)
      {
	if (toSet > to_end) /* future set */
	  {
	    lpc43_tl_set_default_compare ();
	  }
	else
	  {
	    bool set = lpc43_tl_set_calc_arm (curr, toSet, true);
	    if (!set)
	      {
		/* signal call, force interrupt handler */

		call = true;
		lpc43_tl_force_int ();
	      }

	  }
      }
    else
      {
	/* signal call, force interrupt handler */

	call = true;
	lpc43_tl_force_int ();
      }

    lpc43_tl_sync_down();

    return OK;
  }

# else

int up_timer_cancel(FAR struct timespec *ts)
  {
    lpc43_tl_sync_up();

    if (ts == NULL)
      {
	up_alarm_cancel(ts);
      }
    else
      {
	uint64_t saved_alarm_time = alarm_time;
	up_alarm_cancel(ts);
	int64_t diff = saved_alarm_time - lpc43_tl_ts2tick(ts);

	if ( diff > 0 )
	  {
	     lpc43_tl_tick2ts(diff, ts);
	  }
	else
	  {
	    ts->tv_sec = 0;
	    ts->tv_nsec = 0;
	  }

      }

    lpc43_tl_sync_down();
    return OK;
  }

int up_timer_start(FAR const struct timespec *ts)
  {

    lpc43_tl_sync_up();

    struct timespec tmp_ts;
    up_timer_gettime(&tmp_ts);

    tmp_ts->tv_sec += ts->tv_sec;
    tmp_ts->tv_nsec += ts->tv_nsec;

    up_alarm_start(tmp_ts);

    lpc43_tl_sync_down();
    return OK;
  }

# endif /* CONFIG_SCHED_TICKLESS_ALARM */

#endif /* CONFIG_SCHED_TICKLESS	 */
