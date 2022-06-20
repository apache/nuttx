/****************************************************************************
 * include/nuttx/power/pm.h
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

/* Definition of terms.  Various "sleep" and low power consumption states
 * have various names and are sometimes used in conflicting ways.  In the
 * PM logic, we will use the following terminology:
 *
 * NORMAL  - The normal, full power operating mode.
 * IDLE    - This is still basically normal operational mode, the system is,
 *           however, IDLE and some simple simple steps to reduce power
 *           consumption provided that they do not interfere with normal
 *           Operation.  Simply dimming a backlight might be an example
 *           something that would be done when the system is idle.
 * STANDBY - Standby is a lower power consumption mode that may involve more
 *           extensive power management steps such has disabling clocking or
 *           setting the processor into reduced power consumption modes. In
 *           this state, the system should still be able to resume normal
 *           activity almost immediately.
 * SLEEP   - The lowest power consumption mode.  The most drastic power
 *           reduction measures possible should be taken in this state. It
 *           may require some time to get back to normal operation from
 *           SLEEP (some MCUs may even require going through reset).
 *
 * State changes always proceed from higher to lower power usage:
 *
 * NORMAL->IDLE->STANDBY->SLEEP
 *   ^       |      |        |
 *   |       V      V        V
 *   +-------+------+--------+
 */

#ifndef __INCLUDE_NUTTX_POWER_PM_H
#define __INCLUDE_NUTTX_POWER_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wdog.h>

#include <stdbool.h>
#include <time.h>
#include <queue.h>

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_PM_NDOMAINS. Defines the number of "domains" that activity may be
 * monitored on.  For example, you may want to separately manage the power
 * from the Network domain, shutting down the network when it is not be used,
 * from the UI domain, shutting down the UI when it is not in use.
 */

#ifndef CONFIG_PM_NDOMAINS
#  define CONFIG_PM_NDOMAINS 1
#endif

#if CONFIG_PM_NDOMAINS < 1
#  error CONFIG_PM_NDOMAINS invalid
#endif

/* CONFIG_IDLE_CUSTOM. Some architectures support this definition.  This,
 * if defined, will allow you replace the default IDLE loop with your
 * own, custom idle loop to support board-specific IDLE time power management
 */

#define PM_WAKELOCK_INITIALIZER(name, domain, state) {name, domain, state}

#define PM_WAKELOCK_DECLARE(var, name, domain, state) \
      struct pm_wakelock_s var = PM_WAKELOCK_INITIALIZER(name, domain, state)

#define PM_WAKELOCK_DECLARE_STATIC(var, name, domain, state) \
static struct pm_wakelock_s var = PM_WAKELOCK_INITIALIZER(name, domain, state)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration provides all power management states.  Receipt of the
 * state indication is the state transition event.
 */

enum pm_state_e
{
  PM_RESTORE = -1, /* PM_RESTORE is not a low power state.
                    *
                    * PM_RESTORE is used to notify for restore from low power
                    * state.
                    */
  PM_NORMAL = 0,   /* Normal full power operating mode.  If the driver is in
                    * a reduced power usage mode, it should immediately re-
                    * initialize for normal operatin.
                    *
                    * PM_NORMAL may be followed by PM_IDLE.
                    */
  PM_IDLE,         /* Drivers will receive this state change if it is
                    * appropriate to enter a simple IDLE power state.  This
                    * would include simple things such as reducing display
                    * back-lighting.  The driver should be ready to resume
                    * normal activity instantly.
                    *
                    * PM_IDLE may be followed by PM_STANDBY or PM_NORMAL.
                    */
  PM_STANDBY,      /* The system is entering standby mode. Standby is a lower
                    * power consumption mode that may involve more extensive
                    * power management steps such has disabling clocking or
                    * setting the processor into reduced power consumption
                    * modes. In this state, the system should still be able
                    * to resume normal activity almost immediately.
                    *
                    * PM_STANDBY may be followed PM_SLEEP or by PM_NORMAL
                    */
  PM_SLEEP,        /* The system is entering deep sleep mode.  The most
                    * drastic power reduction measures possible should be
                    * taken in this state. It may require some time to get
                    * back to normal operation from SLEEP (some MCUs may
                    * even require going through reset).
                    *
                    * PM_SLEEP may be following by PM_NORMAL
                    */
  PM_COUNT,
};

/* This structure contain pointers callback functions in the driver.  These
 * callback functions can be used to provide power management information
 * to the driver.
 */

struct pm_callback_s
{
  struct dq_entry_s entry;   /* Supports a doubly linked list */

  /**************************************************************************
   * Name: prepare
   *
   * Description:
   *   Request the driver to prepare for a new power state. This is a
   *   warning that the system is about to enter into a new power state.  The
   *   driver should begin whatever operations that may be required to enter
   *   power state.  The driver may abort the state change mode by returning
   *   a non-zero value from the callback function
   *
   * Input Parameters:
   *   cb      - Returned to the driver.  The driver version of the callback
   *             structure may include additional, driver-specific state
   *             data at the end of the structure.
   *   domain  - Identifies the activity domain of the state change
   *   pmstate - Identifies the new PM state
   *
   * Returned Value:
   *   0 (OK) means the event was successfully processed and that the driver
   *   is prepared for the PM state change.  Non-zero means that the driver
   *   is not prepared to perform the tasks needed achieve this power setting
   *   and will cause the state change to be aborted.  NOTE:  The prepare
   *   method will also be recalled when reverting from lower back to higher
   *   power consumption modes (say because another driver refused a lower
   *   power state change).  Drivers are not permitted to return non-zero
   *   values when reverting back to higher power consumption modes!
   *
   **************************************************************************/

  CODE int (*prepare)(FAR struct pm_callback_s *cb, int domain,
                      enum pm_state_e pmstate);

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
   *             structure may include additional, driver-specific state
   *             data at the end of the structure.
   *   domain  - Identifies the activity domain of the state change
   *   pmstate - Identifies the new PM state
   *
   * Returned Value:
   *   None.  The driver already agreed to transition to the low power
   *   consumption state when when it returned OK to the prepare() call.
   *   At that time it should have made all preprations necessary to enter
   *   the new state.  Now the driver must make the state transition.
   *
   **************************************************************************/

  CODE void (*notify)(FAR struct pm_callback_s *cb, int domain,
                      enum pm_state_e pmstate);
};

/* An instance of a given PM governor */

struct pm_governor_s
{
  /**************************************************************************
   * Name: initialize
   *
   * Description:
   *   Invoked by the PM system during initialization, to allow the governor
   *   to initialize its internal data. This can be left to NULL if not
   *   needed by the governor.
   *
   *   NOTE: since this will be called from pm_initialize(), the system
   *   is in very early boot state when this callback is invoked. Thus,
   *   only very basic initialization should be performed (e.g. no memory
   *   should be allocated).
   *
   **************************************************************************/

  CODE void (*initialize)(void);

  /**************************************************************************
   * Name: deinitialize
   *
   * Description:
   *   Allow the governor to release its internal data. This can be left to
   *   to NULL if not needed by the governor.
   *
   **************************************************************************/

  CODE void (*deinitialize)(void);

  /**************************************************************************
   * Name: statechanged
   *
   * Description:
   *   Invoked by the PM system when the power state is changed. This can be
   *   left to NULL if this notification is not needed by the governor.
   *
   **************************************************************************/

  CODE void (*statechanged)(int domain, enum pm_state_e newstate);

  /**************************************************************************
   * Name: checkstate
   *
   * Description:
   *   Invoked by the PM system to obtain the governor's suggestion for the
   *   power level to set. Implementing this callback is mandatory for a
   *   governor since recommending power levels is its main responsibility.
   *
   *   NOTE: the governor should consider the "stay" count in order to hold
   *   off the recommendation of a lower-power level.
   *
   **************************************************************************/

  CODE enum pm_state_e (*checkstate)(int domain);

  /**************************************************************************
   * Name: activity
   *
   * Description:
   *   Invoked by the PM system when a driver reports activity via
   *   pm_activity(). This may or may not be useful to the governor to
   *   recommend power levels.
   *   It can be left NULL, in which case it will not be invoked.
   *
   **************************************************************************/

  CODE void (*activity)(int domain, int count);

  /* Private data for governor's internal use */

  FAR void *priv;
};

/* To be used for accessing the user governor via ioctl calls */

struct pm_user_governor_state_s
{
  int domain;
  enum pm_state_e state;
};

struct pm_wakelock_s
{
  char name[32];
  int domain;
  enum pm_state_e state;
  uint32_t count;
  struct dq_entry_s node;
  struct wdog_s wdog;

#ifdef CONFIG_PM_PROCFS
  struct dq_entry_s fsnode;
  struct timespec start;
  struct timespec elapse;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pm_initialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management subsystem.
 *   This function must be called *very* early in the initialization sequence
 *   *before* any other device drivers are initialized (since they may
 *   attempt to register with the power management subsystem).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *    None.
 *
 ****************************************************************************/

void pm_initialize(void);

/****************************************************************************
 * Name: pm_greedy_governor_initialize
 *
 * Description:
 *   Return the greedy governor instance.
 *
 * Returned Value:
 *   A pointer to the governor struct. Otherwise NULL is returned on error.
 *
 ****************************************************************************/

FAR const struct pm_governor_s *pm_greedy_governor_initialize(void);

/****************************************************************************
 * Name: pm_activity_governor_initialize
 *
 * Description:
 *   Return the activity governor instance.
 *
 * Returned Value:
 *   A pointer to the governor struct. Otherwise NULL is returned on error.
 *
 ****************************************************************************/

FAR const struct pm_governor_s *pm_activity_governor_initialize(void);

/****************************************************************************
 * Name: pm_set_governor
 *
 * Description:
 *   This function set the domain with assigned governor
 *
 * Input Parameters:
 *   domain        - The PM domain to Set
 *   gov           - The governor to use
 *
 * Returned Value:
 *  On success - OK
 *  On error   - -EINVAL
 *
 *
 ****************************************************************************/

int pm_set_governor(int domain, FAR const struct pm_governor_s *gov);

/****************************************************************************
 * Name: pm_auto_update
 *
 * Description:
 *   This function set the domain with assign update mode.
 *
 * Input Parameters:
 *   domain        - The PM domain to check
 *   auto_update   - The PM domain auto update or not
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_auto_update(int domain, bool auto_update);

/****************************************************************************
 * Name: pm_register
 *
 * Description:
 *   This function is called by a device driver in order to register to
 *   receive power management event callbacks.
 *
 * Input Parameters:
 *   callbacks - An instance of struct pm_callback_s providing the driver
 *               callback functions.
 *
 * Returned Value:
 *    Zero (OK) on success; otherwise a negated errno value is returned.
 *
 ****************************************************************************/

int pm_register(FAR struct pm_callback_s *callbacks);

/****************************************************************************
 * Name: pm_unregister
 *
 * Description:
 *   This function is called by a device driver in order to unregister
 *   previously registered power management event callbacks.
 *
 * Input parameters:
 *   callbacks - An instance of struct pm_callback_s providing the driver
 *               callback functions.
 *
 * Returned Value:
 *    Zero (OK) on success; otherwise a negated errno value is returned.
 *
 ****************************************************************************/

int pm_unregister(FAR struct pm_callback_s *callbacks);

/****************************************************************************
 * Name: pm_activity
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle).  This is reported to
 *   the PM governor, which may be used to suggest power states.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   priority - Activity priority, range 0-9.  Larger values correspond to
 *     higher priorities.  Higher priority activity can prevent the system
 *     from entering reduced power states for a longer period of time.
 *
 *     As an example, a button press might be higher priority activity
 *     because it means that the user is actively interacting with the
 *     device.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler (this is the ONLY
 *   PM function that may be called from an interrupt handler!).
 *
 ****************************************************************************/

void pm_activity(int domain, int priority);

/****************************************************************************
 * Name: pm_stay
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power kept at
 *   least the specified level.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to stay.
 *
 *     As an example, media player might stay in normal state during
 *     playback.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_stay(int domain, enum pm_state_e state);

/****************************************************************************
 * Name: pm_relax
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   idle now, could relax the previous requested power level.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to relax.
 *
 *     As an example, media player might relax power level after playback.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_relax(int domain, enum pm_state_e state);

/****************************************************************************
 * Name: pm_staytimeout
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power at kept
 *   last the specified level.
 *   And this will be timeout after time (ms), menas auto pm_relax
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to stay.
 *   ms - The timeout value ms
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_staytimeout(int domain, enum pm_state_e state, int ms);

/****************************************************************************
 * Name: pm_staycount
 *
 * Description:
 *   This function is called to get current stay count.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to relax.
 *
 * Returned Value:
 *   Current pm stay count
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

uint32_t pm_staycount(int domain, enum pm_state_e state);

/****************************************************************************
 * Name: pm_wakelock_init
 *
 * Description:
 *   Init wakelock ID with name, domain, state
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *   name     - wakelock name
 *   domain   - the PM domain want to operated
 *   state    - The PM state want to stay/relax
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_wakelock_init(FAR struct pm_wakelock_s *wakelock,
                      FAR const char *name, int domain,
                      enum pm_state_e state);

/****************************************************************************
 * Name: pm_wakelock_uninit
 *
 * Description:
 *   Uninit wakelock ID
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_wakelock_uninit(FAR struct pm_wakelock_s *wakelock);

/****************************************************************************
 * Name: pm_wakelock_stay
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power kept at
 *   least the specified level.
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_wakelock_stay(FAR struct pm_wakelock_s *wakelock);

/****************************************************************************
 * Name: pm_wakelock_relax
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   idle now, could relax the previous requested power level.
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_wakelock_relax(FAR struct pm_wakelock_s *wakelock);

/****************************************************************************
 * Name: pm_wakelock_staytimeout
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power at kept
 *   last the specified level.
 *   And this will be timeout after time (ms), menas auto pm_wakelock_relax
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *   ms       - The timeout value ms
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_wakelock_staytimeout(FAR struct pm_wakelock_s *wakelock, int ms);

/****************************************************************************
 * Name: pm_wakelock_staycount
 *
 * Description:
 *   This function is called to get current stay count in this wakelock ID
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   Current pm stay count in this wakelock ID
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int pm_wakelock_staycount(FAR struct pm_wakelock_s *wakelock);

/****************************************************************************
 * Name: pm_checkstate
 *
 * Description:
 *   This function is called from the MCU-specific IDLE loop to monitor the
 *   the power management conditions.  This function returns the
 *   "recommended" power management state based on the PM configuration and
 *   activity reported in the last sampling periods.  The power management
 *   state is not automatically changed, however.  The IDLE loop must call
 *   pm_changestate() in order to make the state change.
 *
 *   These two steps are separated because the platform-specific IDLE loop
 *   may have additional situational information that is not available to
 *   the the PM sub-system.  For example, the IDLE loop may know that the
 *   battery charge level is very low and may force lower power states
 *   even if there is activity.
 *
 *   NOTE: That these two steps are separated in time and, hence, the IDLE
 *   loop could be suspended for a long period of time between calling
 *   pm_checkstate() and pm_changestate().  The IDLE loop may need to make
 *   these calls atomic by either disabling interrupts until the state change
 *   is completed.
 *
 * Input Parameters:
 *   domain - The PM domain to check
 *
 * Returned Value:
 *   The recommended power management state.
 *
 ****************************************************************************/

enum pm_state_e pm_checkstate(int domain);

/****************************************************************************
 * Name: pm_changestate
 *
 * Description:
 *   This function is used to platform-specific power management logic.  It
 *   will announce the power management power management state change to all
 *   drivers that have registered for power management event callbacks.
 *
 * Input Parameters:
 *   domain - Identifies the domain of the new PM state
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means that the callback function for all registered drivers
 *   returned OK (meaning that they accept the state change).  Non-zero
 *   means that one of the drivers refused the state change.  In this case,
 *   the system will revert to the preceding state.
 *
 * Assumptions:
 *   It is assumed that interrupts are disabled when this function is
 *   called.  This function is probably called from the IDLE loop... the
 *   lowest priority task in the system.  Changing driver power management
 *   states may result in renewed system activity and, as a result, can
 *   suspend the IDLE thread before it completes the entire state change
 *   unless interrupts are disabled throughout the state change.
 *
 ****************************************************************************/

int pm_changestate(int domain, enum pm_state_e newstate);

/****************************************************************************
 * Name: pm_querystate
 *
 * Description:
 *   This function returns the current power management state.
 *
 * Input Parameters:
 *   domain - The PM domain to check
 *
 * Returned Value:
 *   The current power management state.
 *
 ****************************************************************************/

enum pm_state_e pm_querystate(int domain);

/****************************************************************************
 * Name: pm_auto_updatestate
 *
 * Description:
 *   This function update the domain state and notify the power system.
 *
 * Input Parameters:
 *   domain - The PM domain to check
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_auto_updatestate(int domain);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Stubs
 ****************************************************************************/

#else /* CONFIG_PM */

/* Stubbed out versions of all of PM interface functions that may be used to
 * avoid so much conditional compilation in driver code when PM is disabled:
 */

#  define PM_WAKELOCK_INITIALIZER(n,d,s)      {0}
#  define PM_WAKELOCK_DECLARE(v,n,d,s)
#  define PM_WAKELOCK_DECLARE_STATIC(v,n,d,s)
#  define pm_initialize()
#  define pm_register(cb)                     (0)
#  define pm_unregister(cb)                   (0)
#  define pm_activity(domain,prio)
#  define pm_stay(domain,state)
#  define pm_relax(domain,state)
#  define pm_staytimeout(d,state,ms)
#  define pm_staycount(domain, state)         (0)
#  define pm_wakelock_init(w,n,d,s)
#  define pm_wakelock_uninit(w)
#  define pm_wakelock_stay(w)
#  define pm_wakelock_relax(w)
#  define pm_wakelock_staytimeout(w,m)
#  define pm_wakelock_staycount(w)            (0)
#  define pm_checkstate(domain)               (0)
#  define pm_changestate(domain,state)        (0)
#  define pm_querystate(domain)               (0)
#  define pm_auto_updatestate(domain)

#endif /* CONFIG_PM */
#endif /* __INCLUDE_NUTTX_POWER_PM_H */
