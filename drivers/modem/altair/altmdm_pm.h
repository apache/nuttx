/****************************************************************************
 * drivers/modem/altair/altmdm_pm.h
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

#ifndef __DRIVERS_MODEM_ALTAIR_ALTMDM_PM_H
#define __DRIVERS_MODEM_ALTAIR_ALTMDM_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "altmdm_dev.h"
#include "altmdm_sys.h"
#include "altmdm_pm_state.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODEM_PM_WAKEUP_DONE    (0)
#define MODEM_PM_WAKEUP_ALREADY (1)
#define MODEM_PM_WAKEUP_FAIL    (2)

#define MODEM_PM_CB_SLEEP       (0)
#define MODEM_PM_CB_WAKE        (1)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_pm_init
 *
 * Description:
 *   Initialize the ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_init(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_pm_uninit
 *
 * Description:
 *   Uninitialize the ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_uninit(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_pm_wakeup
 *
 * Description:
 *   Make modem wake up.
 *
 ****************************************************************************/

int altmdm_pm_wakeup(FAR struct altmdm_dev_s *priv,
  CODE int (*wait_fn)(FAR struct altmdm_dev_s *priv, uint32_t timeout_ms));

/****************************************************************************
 * Name: altmdm_pm_notify_reset
 *
 * Description:
 *   Notify reset has done.
 *
 ****************************************************************************/

int altmdm_pm_notify_reset(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_pm_registercb
 *
 * Description:
 *   Register callback for ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_registercb(uint32_t type, altmdm_pm_cbfunc_t cb);

/****************************************************************************
 * Name: altmdm_pm_deregistercb
 *
 * Description:
 *   Deregister callback for ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_deregistercb(uint32_t type);

/****************************************************************************
 * Name: altmdm_pm_sleepmodem
 *
 * Description:
 *   Make modem sleep.
 *
 ****************************************************************************/

int altmdm_pm_sleepmodem(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_pm_cansleep
 *
 * Description:
 *   Check if modem can sleep.
 *
 ****************************************************************************/

int altmdm_pm_cansleep(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_pm_initwakelock
 *
 * Description:
 *   Initialize the modem wakelock resource.
 *
 ****************************************************************************/

int altmdm_pm_initwakelock(FAR struct altmdm_pm_wakelock_s *lock);

/****************************************************************************
 * Name: altmdm_pm_acquirewakelock
 *
 * Description:
 *   Acquire the modem wakelock.
 *
 ****************************************************************************/

int altmdm_pm_acquirewakelock(FAR struct altmdm_pm_wakelock_s *lock);

/****************************************************************************
 * Name: altmdm_pm_releasewakelock
 *
 * Description:
 *   Release the modem wakelock.
 *
 ****************************************************************************/

int altmdm_pm_releasewakelock(FAR struct altmdm_pm_wakelock_s *lock);

/****************************************************************************
 * Name: altmdm_pm_getnumofwakelock
 *
 * Description:
 *   Get the lock count of the specified wakelock.
 *
 ****************************************************************************/

int altmdm_pm_getnumofwakelock(FAR struct altmdm_pm_wakelock_s *lock);

/****************************************************************************
 * Name: altmdm_pm_getwakelockstate
 *
 * Description:
 *   Get the wakelock status. If the return value is 0, it means that it is
 *   not locked. Otherwise it means that someone is locking.
 *
 ****************************************************************************/

int altmdm_pm_getwakelockstate(void);

/****************************************************************************
 * Name: altmdm_pm_poweron
 *
 * Description:
 *   Modem power on.
 *
 ****************************************************************************/

int altmdm_pm_poweron(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_pm_poweroff
 *
 * Description:
 *   Modem power off.
 *
 ****************************************************************************/

int altmdm_pm_poweroff(FAR struct altmdm_dev_s *priv);

/****************************************************************************
 * Name: altmdm_pm_set_bootstatus
 *
 * Description:
 *   Set boot status.
 *
 ****************************************************************************/

int altmdm_pm_set_bootstatus(FAR struct altmdm_dev_s *priv, uint32_t status);

#endif
#endif /* __DRIVERS_MODEM_ALTAIR_ALTMDM_PM_H */
