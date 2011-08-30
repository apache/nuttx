/****************************************************************************
 * include/nuttx/pm.h
 * NuttX Power Management Interfaces
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_PM_H
#define __INCLUDE_NUTTX_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration provides all power management states.  Receipt of the
 * state indication is the state transition event.
 */

enum pm_event_e
{
  PM_IDLE = 0,   /* Drivers will receive periodic idle indications.  The driver
                  * may use these IDLE indications to perform driver-specific
                  * power optimizations.
                  */
  PM_SLEEP_PREP, /* This is a warning that the system is about to enter into
                  * sleep mode.  The driver should begin whatever operations
                  * that may be required to enter sleep mode.  The driver
                  * may abort the sleep mode by returning a non-zero value
                  * from the callback function.
                  */
  PM_STOP_PREP,  /* This is a warning that the system is about to enter into
                  * stop mode.  The driver should begin whatever operations
                  * that may be required to enter stop mode.  The driver
                  * may abort the stop mode by returning a non-zero value
                  * from the callback function.
                  */
  PM_SLEEP,      /* The system is entering sleep mode.  The driver should
                  * already be prepared for this mode.
                  */
  PM_STOP,       /* The system is entering stop mode.  The driver should
                  * already be prepared for this mode.
                  */
  PM_RESUME,     /* The system resuming normal operation.  The driver should
                  * reinitialize for normal operation.
                  */
}

/* This structure contain pointers callback functions in the driver.  These
 * callback functions can be used to provide power management information
 * to the driver.
 */

struct pm_callback_s
{
  struct pm_callback_s *flink;            /* Supports a singly linked list */
  int (*notify)(enum pm_event_e pmevent); /* PM event callback */
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: pm_register
 *
 * Description:
 *   This function is called by a device driver in order to register to
 *   receive power management event callbacks.
 *
 * Input parameters:
 *   callbacks - An instance of struct pm_callback_s providing the driver
 *               callback functions.
 *
 * Returned value:
 *    Zero (OK) on success; otherwise a negater errno value is returned.
 *
 ****************************************************************************/

EXTERN int pm_register(FAR const struct pm_callback_s *callbacks);

/****************************************************************************
 * Name: pm_broadcast
 *
 * Description:
 *   This function is used to platform-specific power managmeent logic.  It
 *   will announce the power management event to all drivers that have
 *   registered for power management event callbacks.
 *
 * 
 *
 ****************************************************************************/

EXTERN int pm_broadcast(enum pm_event_s pmevent);

/****************************************************************************
 * Name: pm_activity
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle).  This will restart a
 *   idle timer and prevent entering reduced power states.
 *
 ****************************************************************************/

EXTERN int pm_activity(void);

/****************************************************************************
 * Name: pm_checkactivity
 *
 * Description:
 *   Return the activity count accumulated since the last time this function
 *   was called.  A count of zero will indicate that no meaningful activity
 *   occurred since the last time this function was called.
 *
 ****************************************************************************/

EXTERN int pm_checkactivity(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_PM_H */
