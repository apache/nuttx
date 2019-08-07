/************************************************************************************
 * arch/sim/src/sim/up_critmon.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdint.h>
#include <time.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#undef USE_CLOCK                                   /* Too slow */
#define USE_CLOCK_GETTIME 1                        /* Better */

/* From nuttx/clock.h */
 
#define NSEC_PER_SEC  1000000000

/* From fixedmath.h */

#define b32ONE        0x0000000100000000           /* 1 */
#define b32toi(a)     ((a) >> 32)                  /* Conversion to integer */
#define itob32(i)     (((b32_t)(i)) << 32)         /* Conversion from integer */
#define b32frac(a)    ((a) & 0x00000000ffffffff)   /* Take fractional part */

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* From fixedmath.h */

typedef int64_t b32_t;

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_critmon_gettime
 ************************************************************************************/

#if defined(USE_CLOCK)
uint32_t up_critmon_gettime(void)
{
  return (uint32_t)clock() + 1;  /* Avoid returning zero which means clock-not-ready */
}
#else /* USE_CLOCK_GETTIME */
uint32_t up_critmon_gettime(void)
{
  struct timespec ts;
  (void)clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)ts.tv_nsec;
}
#endif

/************************************************************************************
 * Name: up_critmon_gettime
 ************************************************************************************/

#if defined(USE_CLOCK)
void up_critmon_convert(uint32_t elapsed, struct timespec *ts)
{
  b32_t b32elapsed;

  b32elapsed  = itob32(elapsed) / CLOCKS_PER_SEC;
  ts->tv_sec  = b32toi(b32elapsed);
  ts->tv_nsec = NSEC_PER_SEC * b32frac(b32elapsed) / b32ONE;
}
#else /* USE_CLOCK_GETTIME */
void up_critmon_convert(uint32_t elapsed, struct timespec *ts)
{
  ts->tv_sec  = 0;
  ts->tv_nsec = elapsed;
}
#endif
