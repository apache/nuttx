/****************************************************************************
 * drivers/modem/altair/altmdm_pm.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
