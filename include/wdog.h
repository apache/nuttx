/************************************************************
 * wdog.h
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

#ifndef __WDOG_H
#define __WDOG_H

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sched.h>

/************************************************************
 * Compilations Switches
 ************************************************************/

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Global Type Declarations
 ************************************************************/

/* This is the form of the function that is called when the
 * watchdog function expires.  Up to four parameters may be passed.
 */

typedef void (*wdentry_t)(int arg1, ...);

/* Watchdog 'handle' */

typedef struct wdog_s *WDOG_ID;

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN WDOG_ID wd_create(void);
EXTERN STATUS  wd_delete(WDOG_ID wdId);
EXTERN STATUS  wd_start(WDOG_ID wdId, int delay, wdentry_t wdentry,
			int parm1, int parm2, int parm3, int parm4);
EXTERN STATUS  wd_cancel(WDOG_ID wdId);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* _WDOG_H_ */

