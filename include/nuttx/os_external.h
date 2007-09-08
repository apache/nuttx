/****************************************************************************
 * os_external.h
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
 ****************************************************************************/

#ifndef __OS_EXTERNAL_H
#define __OS_EXTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <pthread.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Timing constants */

#define NSEC_PER_SEC          1000000000
#define USEC_PER_SEC             1000000
#define MSEC_PER_SEC                1000
#define DSEC_PER_SEC                  10
#define NSEC_PER_DSEC          100000000
#define USEC_PER_DSEC             100000
#define MSEC_PER_DSEC                100
#define NSEC_PER_MSEC            1000000
#define USEC_PER_MSEC               1000
#define NSEC_PER_USEC               1000

/* The interrupt interval of the system timer is given by MSEC_PER_TICK.
 * This is the expected number of milliseconds between calls from the
 * processor-specific logic to sched_process_timer().  The default value
 * of MSEC_PER_TICK is 10 milliseconds (100KHz).  However, this default
 * setting can be overridden by defining the interval in milliseconds as
 * CONFIG_MSEC_PER_TICK in the board configuration file.
 *
 * The following calculations are only accurate when (1) there is no
 * truncation involved and (2) the underlying system timer is an even
 * multiple of milliseconds.  If (2) is not true, you will probably want
 * to redefine all of the following.
 */

#ifdef CONFIG_MSEC_PER_TICK
# define MSEC_PER_TICK        (CONFIG_MSEC_PER_TICK)
#else
# define MSEC_PER_TICK        (10)
#endif

#define TICK_PER_SEC          (MSEC_PER_SEC / MSEC_PER_TICK)             /* Truncates! */
#define NSEC_PER_TICK         (MSEC_PER_TICK * NSEC_PER_MSEC)            /* Exact */
#define USEC_PER_TICK         (MSEC_PER_TICK * USEC_PER_MSEC)            /* Exact */

#define NSEC2TICK(nsec)       (((nsec)+(NSEC_PER_TICK/2))/NSEC_PER_TICK) /* Rounds */
#define USEC2TICK(usec)       (((usec)+(USEC_PER_TICK/2))/USEC_PER_TICK) /* Rounds */
#define MSEC2TICK(msec)       (((msec)+(MSEC_PER_TICK/2))/MSEC_PER_TICK) /* Rounds */
#define DSEC2TICK(dsec)       MSEC2TICK((dsec)*DSEC_PER_MSEC)
#define SEC2TICK(sec)         MSEC2TICK((sec)*SEC_PER_MSEC)

/****************************************************************************
 * Global Data
 ****************************************************************************/

/* Access to raw system clock ***********************************************/

#ifndef CONFIG_DISABLE_CLOCK
extern volatile uint32 g_system_timer;
#endif

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* These are functions that must be supplied by the application */

EXTERN void   weak_function user_initialize(void);
EXTERN int    user_start(int argc, char *argv[]);

/* Functions contained in os_task.c *****************************************/

EXTERN void   os_start(void); /* OS entry point called by boot logic */

/* Functions contained in mm_initialize.c ***********************************/

EXTERN void   mm_initialize(FAR void *heap_start, size_t heap_size);
EXTERN void   mm_addregion(FAR void *heapstart, size_t heapsize);

/* Functions contained in mm_sem.c ******************************************/

EXTERN int mm_trysemaphore(void);
EXTERN void mm_givesemaphore(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __OS_EXTERNAL_H */
