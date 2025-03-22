/****************************************************************************
 * include/sys/timex.h
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

#ifndef __INCLUDE_SYS_TIMEX_H
#define __INCLUDE_SYS_TIMEX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mode codes (timex.mode) */

#define ADJ_OFFSET            0x0001 /* time offset */
#define ADJ_FREQUENCY         0x0002 /* frequency offset */
#define ADJ_MAXERROR          0x0004 /* maximum time error */
#define ADJ_ESTERROR          0x0008 /* estimated time error */
#define ADJ_STATUS            0x0010 /* clock status */
#define ADJ_TIMECONST         0x0020 /* pll time constant */
#define ADJ_TAI               0x0080 /* set TAI offset */
#define ADJ_SETOFFSET         0x0100 /* add 'time' to current time */
#define ADJ_MICRO             0x1000 /* select microsecond resolution */
#define ADJ_NANO              0x2000 /* select nanosecond resolution */
#define ADJ_TICK              0x4000 /* tick value */
#define ADJ_OFFSET_SINGLESHOT 0x8001 /* old-fashioned adjtime */
#define ADJ_OFFSET_SS_READ    0xa001 /* read-only adjtime */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct timex
{
  unsigned int modes;  /* mode selector */
  long offset;         /* time offset (usec) */
  long freq;           /* frequency offset (scaled ppm) */
  long maxerror;       /* maximum error (usec) */
  long esterror;       /* estimated error (usec) */
  int status;          /* clock command/status */
  long constant;       /* pll time constant */
  long precision;      /* clock precision (usec) (read only) */
  long tolerance;      /* clock frequency tolerance (ppm) (read only) */
  struct timeval time; /* (read only, except for ADJ_SETOFFSET) */
  long tick;           /* (modified) usecs between clock ticks */
  long ppsfreq;        /* pps frequency (scaled ppm) (ro) */
  long jitter;         /* pps jitter (us) (ro) */
  int shift;           /* interval duration (s) (shift) (ro) */
  long stabil;         /* pps stability (scaled ppm) (ro) */
  long jitcnt;         /* jitter limit exceeded (ro) */
  long calcnt;         /* calibration intervals (ro) */
  long errcnt;         /* calibration errors (ro) */
  long stbcnt;         /* stability limit exceeded (ro) */
  int tai;             /* TAI offset (ro) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_TIMEX_H */
