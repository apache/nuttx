/****************************************************************************
 * include/nuttx/timers/clkcnt.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_CLKCNT_H
#define __INCLUDE_NUTTX_TIMERS_CLKCNT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/lib/math32.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLKCNT_MAX UINT64_MAX

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint64_t clkcnt_t;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: clkcnt_best_multshift
 *
 * Description:
 *   Calculate the parameters for converting `x * scale / freq`
 *   to `x * mult >> shift`.
 *
 * Input Parameters:
 *   scale - Time scale per second.
 *   freq  - Clock frequency.
 *   mult  - Pointer to multiply parameter.
 *   shift - Pointer to shift parameter.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
void clkcnt_best_multshift(uint32_t freq, uint32_t scale,
                           FAR uint32_t *mult, FAR uint32_t *shift)
{
  uint32_t logfreq = log2floor(freq);

  /* Be careful of the round-nearest behavior here.
   * It may lead to the converted result is larger than the
   * integer division.
   * In rare cases, this can cause the converted tick to be greater
   * than the actual tick, causing the timer to fire prematurely
   * or tasks be scheduled incorrectly.
   * Therefore, we should perform runtime checks to ensure that the
   * conversion result error is within a acceptable range
   * (e.g., within a few nanoseconds).
   * Due to physical limitations, the execution speed of a single processor
   * cannot exceed 10GHz. Even if the timer fire several nanoseconds earlier,
   * it will not have a substantial impact on the actual result, because this
   * time is at most enough for the CPU to execute a few dozen instructions.
   * Note that this optimization should be disabled in hard real-time
   * systems where execution time is determined and time compensation
   * technique is implemented, as it introduces additional time jitter,
   * affecting the accuracy of time compensation.
   */

  *mult  = (((uint64_t)scale << logfreq) + (freq >> 1)) / freq;
  *shift = logfreq;
}

/****************************************************************************
 * Name: clkcnt_max_tick
 *
 * Description:
 *   Calculate the maximum tick of a timer.
 *
 * Input Parameters:
 *   max_count - Maximum timer count.
 *   freq - Clock frequency.
 *
 * Returned Value:
 *   The maximum tick of the timer.
 *
 ****************************************************************************/

static inline_function
clock_t clkcnt_max_tick(clkcnt_t max_count, uint32_t freq)
{
  clkcnt_t cnt = max_count / freq * TICK_PER_SEC +
                 max_count % freq * TICK_PER_SEC / freq;
  cnt = cnt <= CLOCK_MAX ? cnt : CLOCK_MAX;
  return (clock_t)cnt;
}

/****************************************************************************
 * Name: clkcnt_max_timepsec
 *
 * Description:
 *   Calculate the maximum timespec of a timer.
 *
 * Input Parameters:
 *   max_count - Maximum timer count.
 *   freq - Clock frequency.
 *   ts - The pointer to maximum timespec of the timer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
void clkcnt_max_timespec(clkcnt_t max_count, uint32_t freq,
                         FAR struct timespec *ts)
{
  ts->tv_sec  = max_count / freq;
  ts->tv_nsec = max_count % freq * NSEC_PER_SEC / freq;
}

/****************************************************************************
 * Name: clkcnt_delta_time2cnt
 *
 * Description:
 *   Convert delta time to clock count.
 *
 * Input Parameters:
 *   time - Delta time, required time <= CLKCNT_MAX / freq
 *   freq - Clock frequency.
 *   scale - Time scale per second.
 *
 * Returned Value:
 *   The converted clock count.
 *
 ****************************************************************************/

static inline_function
clkcnt_t clkcnt_delta_time2cnt(uint64_t time, uint32_t freq, uint32_t scale)
{
  /* Be careful of the multiplication-overflow */

  DEBUGASSERT(time <= CLKCNT_MAX / freq);

  return div_const(time * freq, scale);
}

/****************************************************************************
 * Name: clkcnt_time2cnt
 *
 * Description:
 *   Convert absolute time to clock count, this function is slower
 *   than the clkcnt_delta_time2cnt.
 *
 * Input Parameters:
 *   time - Absolute time.
 *   freq - Clock frequency.
 *   scale - Time scale per second.
 *
 * Returned Value:
 *   The converted clock count.
 *
 ****************************************************************************/

static inline_function
clkcnt_t clkcnt_time2cnt(uint64_t time, uint32_t freq, uint32_t scale)
{
  clkcnt_t count;
  clkcnt_t sec;

  /* Fast-path at compile-time. */

  if (is_constexpr(freq) && freq / scale * scale == freq)
    {
      /*  If the freq can be divided by the time scale.
       *  Let cnt_per_timescale = freq / scale,
       *  then cnt = time * cnt_per_timescale.
       */

      DEBUGASSERT(is_constexpr(scale));

      count = time * (freq / scale);
    }
  else
    {
      sec   = div_const(time, scale);
      time -= sec * scale;
      count = sec * freq + clkcnt_delta_time2cnt(time, freq, scale);
    }

  return count;
}

/****************************************************************************
 * Name: clkcnt_nsec2cnt
 *
 * Description:
 *   Convert absolute nano-second to clock count. This function is wrapper
 *   version of the clkcnt_time2cnt.
 *
 * Input Parameters:
 *   nsec - Absolute nano-second.
 *   freq - Clock frequency.
 *
 * Returned Value:
 *   The converted clock count.
 *
 ****************************************************************************/

static inline_function
clkcnt_t clkcnt_nsec2cnt(uint64_t nsec, uint32_t freq)
{
  return clkcnt_time2cnt(nsec, freq, NSEC_PER_SEC);
}

/****************************************************************************
 * Name: clkcnt_tick2cnt
 *
 * Description:
 *   Convert absolute tick to clock count. This function is wrapper
 *   version of the clkcnt_time2cnt.
 *
 * Input Parameters:
 *   tick - Absolute tick.
 *   freq - Clock frequency.
 *
 * Returned Value:
 *   The converted clock count.
 *
 ****************************************************************************/

static inline_function
clkcnt_t clkcnt_tick2cnt(clock_t tick, uint32_t freq)
{
  return clkcnt_time2cnt(tick, freq, TICK_PER_SEC);
}

/****************************************************************************
 * Name: clkcnt_delta_cnt2time
 *
 * Description:
 *   Convert delta clock count to time.
 *
 * Input Parameters:
 *   delta - Delta clock count, requires delta <= CLKCNT_MAX / scale
 *   freq  - Clock frequency.
 *   scale - Time scale per second.
 *
 * Returned Value:
 *   The converted delta time.
 *
 ****************************************************************************/

static inline_function
uint64_t clkcnt_delta_cnt2time(clkcnt_t delta, uint32_t freq, uint32_t scale)
{
  uint64_t time;

  /* Be careful of the multiply-overflow. */

  DEBUGASSERT(delta <= CLKCNT_MAX / scale);

  /* Fast path:
   * If we already knew clkcnt frequency at compile time,
   * we can replace the division by `div_const`.
   */

  if (is_constexpr(freq))
    {
      /*  If the freq can be divided by the time scale.
       *  Let cnt_per_timescale = freq / scale,
       *  then time = cnt / cnt_per_timescale.
       */

      DEBUGASSERT(is_constexpr(scale));

      if (freq / scale * scale == freq)
        {
          time = div_const(delta, freq / scale);
        }
      else
        {
          time = div_const(delta * scale, freq);
        }
    }
  else
    {
      /* Slow path using division */

      time = delta * scale / freq;
    }

  return time;
}

/****************************************************************************
 * Name: clkcnt_delta_cnt2time_invdiv
 *
 * Description:
 *   Convert delta clock count to time using the invariant-divisor division.
 *
 * Input Parameters:
 *   delta - Delta clock count, requires delta <= CLKCNT_MAX / scale
 *   scale - Time scale per second.
 *   invdiv - Initialized parameter of the invariant-divisor division.
 *
 * Returned Value:
 *   The converted delta time.
 *
 ****************************************************************************/

static inline_function
uint64_t clkcnt_delta_cnt2time_invdiv(clkcnt_t delta, uint32_t scale,
                                      FAR const invdiv_param64_t *invdiv)
{
  /* Be careful of the multiply-overflow. */

  DEBUGASSERT(delta <= CLKCNT_MAX / scale);

  return invdiv_u64(delta * scale, invdiv);
}

/****************************************************************************
 * Name: clkcnt_delta_cnt2nsec_fast
 *
 * Description:
 *   Convert delta clock count to nano-second using fast path, which trades
 *   accuracy for performance.
 *   NOTE: This function requires delta <= freq.
 *
 * Input Parameters:
 *   delta - Delta clock count, requires delta <= CLKCNT_MAX / scale
 *   mul   - Multiply parameter.
 *   sh    - Right-shift parameter.
 *
 * Returned Value:
 *   The converted delta time in nano-second.
 *
 ****************************************************************************/

static inline_function
uint64_t clkcnt_delta_cnt2nsec_fast(clkcnt_t delta, uint32_t mul, uint8_t sh)
{
  /* Fast path using multiplication and right-shifting. */

  return delta * mul >> sh;
}

/****************************************************************************
 * Name: clkcnt_delta_cnt2nsec
 *
 * Description:
 *   Convert delta clock count to nano-second. This function is wrapper
 *   version of the clkcnt_delta_cnt2time.
 *
 * Input Parameters:
 *   delta - Delta clock count, requires delta <= CLKCNT_MAX / NSEC_PER_SEC
 *   freq  - Clock frequency.
 *
 * Returned Value:
 *   The converted delta time in nano-second.
 *
 ****************************************************************************/

static inline_function
uint64_t clkcnt_delta_cnt2nsec(clkcnt_t delta, uint32_t freq)
{
  return clkcnt_delta_cnt2time(delta, freq, NSEC_PER_SEC);
}

/****************************************************************************
 * Name: clkcnt_delta_cnt2tick
 *
 * Description:
 *   Convert delta clock count to tick. This function is wrapper
 *   version of the clkcnt_delta_cnt2time.
 *
 * Input Parameters:
 *   delta - Delta clock count, requires delta <= CLKCNT_MAX / TICK_PER_SEC
 *   freq  - Clock frequency.
 *
 * Returned Value:
 *   The converted delta time in tick.
 *
 ****************************************************************************/

static inline_function
clock_t clkcnt_delta_cnt2tick(clkcnt_t delta, uint32_t freq)
{
  clkcnt_t tick = clkcnt_delta_cnt2time(delta, freq, TICK_PER_SEC);

  DEBUGASSERT(tick <= CLOCK_MAX);

  return (clock_t)tick;
}

/****************************************************************************
 * Name: clkcnt_cnt2sec
 *
 * Description:
 *   Convert absolute clock count to sec. This function is wrapper
 *   version of the clkcnt_delta_cnt2time.
 *
 * Input Parameters:
 *   cnt   - absolute clock count.
 *   freq  - Clock frequency.
 *
 * Returned Value:
 *   The converted time in second.
 *
 ****************************************************************************/

static inline_function
uint64_t clkcnt_cnt2sec(clkcnt_t cnt, uint32_t freq)
{
  /* There is no multiply-overflow for clkcnt_delta_cnt2time,
   * since the time_scale == 1.
   */

  return clkcnt_delta_cnt2time(cnt, freq, 1);
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_CLKCNT_H */
