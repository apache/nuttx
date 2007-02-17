/************************************************************
 * clock_internal.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __CLOCK_INTERNAL_H
#define __CLOCK_INTERNAL_H

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <nuttx/compiler.h>

/************************************************************
 * Definitions
 ************************************************************/

/* Timing constants */

#define NSEC_PER_SEC          1000000000
#define USEC_PER_SEC             1000000
#define MSEC_PER_SEC                1000
#define NSEC_PER_MSEC            1000000
#define USEC_PER_MSEC               1000
#define NSEC_PER_USEC               1000

#define MSEC_PER_TICK                 10
#define USEC_PER_TICK         (MSEC_PER_TICK * USEC_PER_MSEC)
#define NSEC_PER_TICK         (MSEC_PER_TICK * NSEC_PER_MSEC)
#define TICK_PER_SEC         (MSEC_PER_SEC / MSEC_PER_TICK)

#define MSEC2TICK(msec)       (((msec)+(MSEC_PER_TICK/2))/MSEC_PER_TICK)
#define USEC2TICK(usec)       (((usec)+(USEC_PER_TICK/2))/USEC_PER_TICK)

#define JD_OF_EPOCH   2440588    /* Julian Date of noon, J1970 */

#ifdef CONFIG_JULIAN_TIME
# define GREG_DUTC    -141427    /* Default is October 15, 1582 */
# define GREG_YEAR       1582
# define GREG_MONTH        10
# define GREG_DAY          15
#endif /* CONFIG_JULIAN_TIME */

/************************************************************
 * Public Type Definitions
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

extern volatile uint32 g_system_timer;
extern struct timespec g_basetime;
extern uint32 g_tickbias;

/************************************************************
 * Public Inline Functions
 ************************************************************/

/************************************************************
 * Public Function Prototypes
 ************************************************************/

extern void weak_function clock_initialize(void);
extern void weak_function clock_timer(void);

extern time_t clock_calendar2utc(int year, int month, int day);

#endif /* __CLOCK_INTERNAL_H */
