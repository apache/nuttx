/****************************************************************************
 * include/nuttx/wdog.h
 *
 *   Copyright (C) 2007-2009, 2014-2015, 2018 Gregory Nutt. All rights
 *     reserved.
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

#ifndef __INCLUDE_NUTTX_WDOG_H
#define __INCLUDE_NUTTX_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ********************************************************/

#ifndef CONFIG_PREALLOC_WDOGS
#  define CONFIG_PREALLOC_WDOGS 32
#endif

#ifndef CONFIG_WDOG_INTRESERVE
#  if CONFIG_PREALLOC_WDOGS > 16
#    define CONFIG_WDOG_INTRESERVE 4
#  elif  CONFIG_PREALLOC_WDOGS > 8
#    define CONFIG_WDOG_INTRESERVE 2
#  else
#    define CONFIG_WDOG_INTRESERVE 1
#  endif
#endif

#if CONFIG_WDOG_INTRESERVE >= CONFIG_PREALLOC_WDOGS
#  error CONFIG_WDOG_INTRESERVE >= CONFIG_PREALLOC_WDOGS
#endif

/* Watchdog Definitions *************************************************/
/* Flag bits for the flags field of struct wdog_s */

#define WDOGF_ACTIVE       (1 << 0) /* Bit 0: 1=Watchdog is actively timing */
#define WDOGF_ALLOCED      (1 << 1) /* Bit 1: 0=Pre-allocated, 1=Allocated */
#define WDOGF_STATIC       (1 << 2) /* Bit 2: 0=[Pre-]allocated, 1=Static */

#define WDOG_SETACTIVE(w)  do { (w)->flags |= WDOGF_ACTIVE; } while (0)
#define WDOG_SETALLOCED(w) do { (w)->flags |= WDOGF_ALLOCED; } while (0)
#define WDOG_SETSTATIC(w)  do { (w)->flags |= WDOGF_STATIC; } while (0)

#define WDOG_CLRACTIVE(w)  do { (w)->flags &= ~WDOGF_ACTIVE; } while (0)
#define WDOG_CLRALLOCED(w) do { (w)->flags &= ~WDOGF_ALLOCED; } while (0)
#define WDOG_CLRSTATIC(w)  do { (w)->flags &= ~WDOGF_STATIC; } while (0)

#define WDOG_ISACTIVE(w)   (((w)->flags & WDOGF_ACTIVE) != 0)
#define WDOG_ISALLOCED(w)  (((w)->flags & WDOGF_ALLOCED) != 0)
#define WDOG_ISSTATIC(w)   (((w)->flags & WDOGF_STATIC) != 0)

/* Initialization of statically allocated timers ****************************/

#define wd_static(w) \
  do { (w)->next = NULL; (w)->flags = WDOGF_STATIC; } while (0)

#ifdef CONFIG_PIC
#  define WDOG_INITIAILIZER { NULL, NULL, NULL, 0, WDOGF_STATIC, 0 }
#else
#  define WDOG_INITIAILIZER { NULL, NULL, 0, WDOGF_STATIC, 0 }
#endif

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/
/* The arguments are passed as scalar wdparm_t values.  For systems where
 * the sizeof(pointer) < sizeof(uint32_t), the following union defines the
 * alignment of the pointer within the uint32_t.  For example, the SDCC
 * MCS51 general pointer is 24-bits, but uint32_t is 32-bits (of course).
 *
 * We always have sizeof(pointer) <= sizeof(uintptr_t) by definition.
 */

union wdparm_u
{
  FAR void     *pvarg; /* The size one generic point */
  uint32_t      dwarg; /* Big enough for a 32-bit value in any case */
  uintptr_t     uiarg; /* sizeof(uintptr_t) >= sizeof(pointer) */
};

#if UINTPTR_MAX >= UINT32_MAX
typedef uintptr_t wdparm_t;
#else
typedef uint32_t  wdparm_t;
#endif

/* This is the form of the function that is called when the
 * watchdog function expires.  Up to four parameters may be passed.
 */

typedef CODE void (*wdentry_t)(int argc, wdparm_t arg1, ...);

/* This is the internal representation of the watchdog timer structure.  The
 * WDOG_ID is a pointer to a watchdog structure.
 */

struct wdog_s
{
  FAR struct wdog_s *next;       /* Support for singly linked lists. */
  wdentry_t          func;       /* Function to execute when delay expires */
#ifdef CONFIG_PIC
  FAR void          *picbase;    /* PIC base address */
#endif
  int                lag;        /* Timer associated with the delay */
  uint8_t            flags;      /* See WDOGF_* definitions above */
  uint8_t            argc;       /* The number of parameters to pass */
  wdparm_t           parm[CONFIG_MAX_WDOGPARMS];
};

/* Watchdog 'handle' */

typedef FAR struct wdog_s *WDOG_ID;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Pubic Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: wd_create
 *
 * Description:
 *   The wd_create function will create a watchdog timer by allocating one
 *   from the list of free watchdog timers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Pointer to watchdog (i.e., the watchdog ID), or NULL if insufficient
 *   watchdogs are available.
 *
 ****************************************************************************/

WDOG_ID wd_create(void);

/****************************************************************************
 * Name: wd_delete
 *
 * Description:
 *   The wd_delete() function will deallocate a watchdog timer by returning
 *   it to the free pool of watchdog timers.  The watchdog timer will be
 *   removed from the active timer queue if had been started.
 *
 * Input Parameters:
 *   wdog - The watchdog ID to delete.  This is actually a pointer to a
 *          watchdog structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The caller has assured that the watchdog is no longer in use.
 *
 ****************************************************************************/

int wd_delete(WDOG_ID wdog);

/****************************************************************************
 * Name: wd_start
 *
 * Description:
 *   This function adds a watchdog timer to the actuve timer queue.  The
 *   specified watchdog function at 'wdentry' will be called from the
 *   interrupt level after the specified number of ticks has elapsed.
 *   Watchdog timers may be started from the interrupt level.
 *
 *   Watchdog timers execute in the address environment that was in effect
 *   when wd_start() is called.
 *
 *   Watchdog timers execute only once.
 *
 *   To replace either the timeout delay or the function to be executed,
 *   call wd_start again with the same wdog; only the most recent wdStart()
 *   on a given watchdog ID has any effect.
 *
 * Input Parameters:
 *   wdog     - watchdog ID
 *   delay    - Delay count in clock ticks
 *   wdentry  - function to call on timeout
 *   parm1..4 - parameters to pass to wdentry
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The watchdog routine runs in the context of the timer interrupt handler
 *   and is subject to all ISR restrictions.
 *
 ****************************************************************************/

int wd_start(WDOG_ID wdog, int32_t delay, wdentry_t wdentry, int argc, ...);

/****************************************************************************
 * Name: wd_cancel
 *
 * Description:
 *   This function cancels a currently running watchdog timer. Watchdog
 *   timers may be cancelled from the interrupt level.
 *
 * Input Parameters:
 *   wdog - ID of the watchdog to cancel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success;  A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int wd_cancel(WDOG_ID wdog);

/****************************************************************************
 * Name: wd_gettime
 *
 * Description:
 *   This function returns the time remaining before the specified watchdog
 *   timer expires.
 *
 * Input Parameters:
 *   wdog - watchdog ID
 *
 * Returned Value:
 *   The time in system ticks remaining until the watchdog time expires.
 *   Zero means either that wdog is not valid or that the wdog has already
 *   expired.
 *
 ****************************************************************************/

int wd_gettime(WDOG_ID wdog);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WDOG_H */
