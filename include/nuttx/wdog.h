/****************************************************************************
 * include/nuttx/wdog.h
 *
 *   Copyright (C) 2007-2009, 2014 Gregory Nutt. All rights reserved.
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
#include <sched.h>

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
/* The arguments are passed as uint32_t values.  For systems where the
 * sizeof(pointer) < sizeof(uint32_t), the following union defines the
 * alignment of the pointer within the uint32_t.  For example, the SDCC
 * MCS51 general pointer is 24-bits, but uint32_t is 32-bits (of course).
 *
 * For systems where sizeof(pointer) > sizeof(uint32_t), we will have to do
 * some redesign.
 */

union wdparm_u
{
  FAR void     *pvarg;
  FAR uint32_t *dwarg;
};

typedef union wdparm_u wdparm_t;

/* This is the form of the function that is called when the
 * watchdog function expires.  Up to four parameters may be passed.
 */

typedef CODE void (*wdentry_t)(int argc, uint32_t arg1, ...);

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
  uint32_t           parm[CONFIG_MAX_WDOGPARMS];
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

WDOG_ID wd_create(void);
int     wd_delete(WDOG_ID wdog);
int     wd_start(WDOG_ID wdog, int delay, wdentry_t wdentry, int argc, ...);
int     wd_cancel(WDOG_ID wdog);
int     wd_gettime(WDOG_ID wdog);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* _WDOG_H_ */
