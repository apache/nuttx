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
/* Definition of terms.  Various "sleep" and low power consumption states
 * have various names and are sometimes used in conflicting ways.  In the
 * PM logic, we will use the following terminology:
 *
 * NORMAL - The normal, full power operating mode.
 * REDUCED - This is still basically normal operational mode, but with some
 *           simple changes to reduce power consumption.  Perhaps this just
 *           means just dimming the backlight.
 * STANDBY - Standby is a very low power consumption mode.  It is the lowest
 *           power from which the system can recover quickly.
 * SLEEP   - The lowest power consumption mode.  It may require some time
 *           to get back to normal operation from SLEEP (some parts may
 *           even require going through reset).
 *
 * State changes always proceed from higher to lower power usage:
 *
 * NORMAL->REDUCED->STANDBY->SLEEP
 *   ^       |         |        |
 *   |       V         V        V
 *   +-------+---------+--------+
 */

#ifndef __INCLUDE_NUTTX_PM_H
#define __INCLUDE_NUTTX_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Time slices */

#ifndef CONFIG_PM_SLICEMS
#  define CONFIG_PM_SLICEMS 100 /* Default is 100 msec */
#endif

#ifndef CONFIG_PM_NREDUCED
#  define CONFIG_PM_NREDUCED 30 /* Thiry IDLE slices to enter reduced mode */
#endif

#ifndef CONFIG_PM_NSTANDBY
#  define CONFIG_PM_NSTANDBY 80 /* Eight IDLE slices to enter standby mode */
#endif

#ifndef CONFIG_PM_NSLEEP
#  define CONFIG_PM_NSLEEP 150 /* 150 IDLE slices to enter standby mode */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration provides all power management states.  Receipt of the
 * state indication is the state transition event.
 */

enum pm_state_e
{
  PM_REDUCED = 0,  /* Drivers will receive periodic this indications if it is
                    * appropriate to enter a simple reduced power state.  This
                    * would include simple things such as displaying display back-
                    * lighting.  The driver should essentially be ready to resume
                    * normal activity instantly.
                    *
                    * PM_REDUCED may be followed by PM_STANDBY or PM_RESUME.
                    */
  PM_STANDBY,      /* The system is entering standby mode.  The driver should
                    * already be prepared for this mode.
                    *
                    * PM_STANDBY may be followed PM_SLEEP or by PM_RESUME
                    */
  PM_SLEEP,        /* The system is entering deep sleep mode.  The driver should
                    * already be prepared for this mode.
                    *
                    * PM_SLEEP may be following by PM_RESUME
                    */
  PM_RESUME,       /* The system is resuming normal operation.  The driver should
                    * reinitialize for normal operation.
                    *
                    * PM_RESUME may be followed by PM_REDUCED.
                    */
}

/* This structure contain pointers callback functions in the driver.  These
 * callback functions can be used to provide power management information
 * to the driver.
 */

struct pm_callback_s
{
  struct pm_callback_s *flink;            /* Supports a singly linked list */

  /**************************************************************************
   * Name: prepare
   *
   * Description:
   *   Notify the driver to prepare for a new power confition  .This is a
   *   warning that the system is about to enter into a new power state.  The
   *   driver should begin whatever operations that may be required to enter
   *   power state.  The driver may abort the state change mode by returning
   *   a non-zero value from the callback function
   *
   * Input Parameters:
   *   cb      - Returned to the driver.  The driver version of the callback
   *             strucure may include additional, driver-specific state
   *             data at the end of the structure.
   *   pmstate - Idenfifies the new PM state
   *
   * Returned Value:
   *   0 (OK) means the event was successfully processed.  Non-zero means
   *   means that the driver is not prepared to perform the tasks needed
   *   achieve this power setting.
   *
   **************************************************************************/

  int (*prepare)(FAR struct pm_callback_s *cb, enum pm_state_e pmstate);

  /**************************************************************************
   * Name: notify
   *
   * Description:
   *   Notify the driver of new power state.  This callback is called after
   *   all drivers have had the opportunity to prepare for the new power
   *   state.
   *
   * Input Parameters:
   *   cb      - Returned to the driver.  The driver version of the callback
   *             strucure may include additional, driver-specific state
   *             data at the end of the structure.
   *   pmstate - Idenfifies the new PM state
   *
   * Returned Value:
   *   0 (OK) means the event was successfully processed.  Non-zero means
   *   means that the driver failed to enter the power mode.
   *
   **************************************************************************/

  int (*notify)(FAR struct pm_callback_s *cb, enum pm_state_e pmstate);
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

EXTERN int pm_register(FAR struct pm_callback_s *callbacks);

/****************************************************************************
 * Name: pm_changestate
 *
 * Description:
 *   This function is used to platform-specific power managmeent logic.  It
 *   will announce the power management power management state change to all
 *   drivers that have registered for power management event callbacks.
 *
 * Input Parameters:
 *   pmstate - Idenfifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means that the callback function for all registered drivers
 *   returned OK (meaning that they accept the state change).
 *
 ****************************************************************************/

EXTERN int pm_changestate(enum pm_event_s pmstate);

/****************************************************************************
 * Name: pm_activity
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle).  This increment an activty
 *   cound and/or will restart a idle timer and prevent entering reduced
 *   power states.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current activity count.
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
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current activity count.
 *
 ****************************************************************************/

EXTERN int pm_checkactivity(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PM */
#endif /* __INCLUDE_NUTTX_PM_H */
