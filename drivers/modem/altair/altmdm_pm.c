/****************************************************************************
 * drivers/modem/altair/altmdm_pm.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/modem/altmdm.h>

#include "altmdm_sys.h"
#include "altmdm_pm.h"
#include "altmdm_pm_state.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeout is counted in units of millisecond. */

#  define D2H_UP_EVENT_TIMEOUT   (5*1000)

#  define GPIO_LOW                       (false)
#  define GPIO_HIGH                      (true)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct altmdm_dev_s *g_privdata = NULL;
static bool g_is_initdone = false;
static altmdm_pm_cbfunc_t g_pm_callback = NULL;
static altmdm_pm_cbfunc_t g_pm_errcallback = NULL;
static sq_queue_t g_wakelock;
static uint32_t g_boot_stat = MODEM_PM_ERR_RESET_BOOTSTAT_NONE;

/****************************************************************************
 * Name: set_mreq_gpio
 *
 * Description:
 *   Control Master-request GPIO line.
 *
 ****************************************************************************/

static int set_mreq_gpio(FAR struct altmdm_dev_s *priv, bool value)
{
  DEBUGASSERT(priv && priv->lower);
  priv->lower->master_request(value);

  return 0;
}

/****************************************************************************
 * Name: set_h2d_gpio
 *
 * Description:
 *   Control Host to Device GPIO line.
 *
 ****************************************************************************/

static int set_h2d_gpio(FAR struct altmdm_dev_s *priv, bool value)
{
  DEBUGASSERT(priv && priv->lower);
  priv->lower->wakeup(value);

  return 0;
}

/****************************************************************************
 * Name: set_mreq_down
 *
 * Description:
 *   Deassert Master-request GPIO line.
 *
 ****************************************************************************/

static int set_mreq_down(FAR struct altmdm_dev_s *priv)
{
  return set_mreq_gpio(priv, GPIO_LOW);
}

/****************************************************************************
 * Name: set_mreq_up
 *
 * Description:
 *   Assert Master-request GPIO line.
 *
 ****************************************************************************/

static int set_mreq_up(FAR struct altmdm_dev_s *priv)
{
  return set_mreq_gpio(priv, GPIO_HIGH);
}

/****************************************************************************
 * Name: set_h2d_down
 *
 * Description:
 *   Deassert Host to Device GPIO line.
 *
 ****************************************************************************/

static int set_h2d_down(FAR struct altmdm_dev_s *priv)
{
  return set_h2d_gpio(priv, GPIO_LOW);
}

/****************************************************************************
 * Name: set_h2d_up
 *
 * Description:
 *   Assert Host to Device GPIO line.
 *
 ****************************************************************************/

static int set_h2d_up(FAR struct altmdm_dev_s *priv)
{
  return set_h2d_gpio(priv, GPIO_HIGH);
}

/****************************************************************************
 * Name: exe_callback
 *
 * Description:
 *   Execute callback of modem power manager.
 *
 ****************************************************************************/

static int exe_callback(uint32_t type, uint32_t cb_event)
{
  altmdm_pm_cbfunc_t lcallback = NULL;

  if (type == MODEM_PM_CB_TYPE_ERROR)
    {
      lcallback = g_pm_errcallback;
    }
  else
    {
      lcallback = g_pm_callback;
    }

  if (lcallback == NULL)
    {
      return -EPERM;
    }

  lcallback(cb_event);

  return 0;
}

/****************************************************************************
 * Name: sleep_modem_itself
 *
 * Description:
 *   The modem slept on itself, it transitions to the sleep state.
 *
 ****************************************************************************/

static int sleep_modem_itself(FAR struct altmdm_dev_s *priv)
{
  set_h2d_down(priv);

  /* Transitions to the sleep state. */

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_SLEEP);

  /* Perform processing at the time of state transition. */

  exe_callback(MODEM_PM_CB_TYPE_NORMAL, MODEM_PM_CB_SLEEP);

  return 0;
}

/****************************************************************************
 * Name: modem_sleep_procedure
 *
 * Description:
 *   Make the modem transition to sleep.
 *
 ****************************************************************************/

static int modem_sleep_procedure(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = set_h2d_down(priv);
  if (ret == 0)
    {
      /* Transitions to the sleep state. */

      altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_SLEEP);

      /* Perform processing at the time of state transition. */

      exe_callback(MODEM_PM_CB_TYPE_NORMAL, MODEM_PM_CB_SLEEP);
    }

  return ret;
}

/****************************************************************************
 * Name: modem_wakeup_procedure
 *
 * Description:
 *   Make the modem transition to wakeup.
 *
 ****************************************************************************/

static int modem_wakeup_procedure(FAR struct altmdm_dev_s *priv,
  CODE int (*wait_fn)(FAR struct altmdm_dev_s *priv, uint32_t timeout_ms))
{
  int ret = 0;

  if (MODEM_PM_INTERNAL_STATE_WAKE == altmdm_pm_getinternalstate())
    {
      return MODEM_PM_WAKEUP_ALREADY;
    }

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE);

  ret = set_h2d_up(priv);
  if (ret == 0)
    {
      if (wait_fn)
        {
          set_mreq_up(priv);

          ret = wait_fn(priv, D2H_UP_EVENT_TIMEOUT);
          if (ret != 0)
            {
              m_err("ERR:%04d device is not up.\n", __LINE__);

              /* Do rollback. */

              set_mreq_down(priv);

              set_h2d_down(priv);

              altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_SLEEP);

              ret = MODEM_PM_WAKEUP_FAIL;
            }
          else
            {
              /* Transitions to the wake state. */

              altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_WAKE);

              /* Perform processing at the time of state transition. */

              exe_callback(MODEM_PM_CB_TYPE_NORMAL, MODEM_PM_CB_WAKE);

              ret = MODEM_PM_WAKEUP_DONE;
            }
        }
      else
        {
          /* Transitions to the wake state. */

          altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_WAKE);

          /* Perform processing at the time of state transition. */

          exe_callback(MODEM_PM_CB_TYPE_NORMAL, MODEM_PM_CB_WAKE);

          ret = MODEM_PM_WAKEUP_DONE;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sleep_is_possible
 *
 * Description:
 *   Check if the modem can sleep.
 *
 ****************************************************************************/

static int sleep_is_possible(FAR struct altmdm_dev_s *priv)
{
#  ifdef CONFIG_MODEM_ALTMDM_KEEP_WAKE_STATE
  return -EPERM;
#  else
  int num_of_lock;
  uint32_t pm_state;

  pm_state = altmdm_pm_getinternalstate();
  if (pm_state != MODEM_PM_INTERNAL_STATE_WAKE)
    {
      return -EBUSY;
    }

  num_of_lock = altmdm_pm_getwakelockstate();
  if (num_of_lock)
    {
      return -EBUSY;
    }

  if (g_boot_stat != MODEM_PM_ERR_RESET_BOOTSTAT_DONE)
    {
      return -EBUSY;
    }

  return 0;
#  endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_pm_init
 *
 * Description:
 *   Initialize the ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_init(FAR struct altmdm_dev_s *priv)
{
  if (g_is_initdone)
    {
      return -EPERM;
    }

  g_is_initdone = true;

  g_privdata = priv;

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_SLEEP);

  sq_init(&g_wakelock);

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_uninit
 *
 * Description:
 *   Uninitialize the ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_uninit(FAR struct altmdm_dev_s *priv)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  g_is_initdone = false;

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_SLEEP);

  sq_init(&g_wakelock);

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_wakeup
 *
 * Description:
 *   Make modem wake up.
 *
 ****************************************************************************/

int altmdm_pm_wakeup(FAR struct altmdm_dev_s *priv,
  CODE int (*wait_fn)(FAR struct altmdm_dev_s *priv, uint32_t timeout_ms))
{
  int ret = MODEM_PM_WAKEUP_FAIL;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  ret = modem_wakeup_procedure(priv, wait_fn);

  return ret;
}

/****************************************************************************
 * Name: altmdm_pm_notify_reset
 *
 * Description:
 *   Notify reset has done.
 *
 ****************************************************************************/

int altmdm_pm_notify_reset(FAR struct altmdm_dev_s *priv)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  if (MODEM_PM_ERR_RESET_BOOTSTAT_NONE != g_boot_stat)
    {
      exe_callback(MODEM_PM_CB_TYPE_ERROR, g_boot_stat);
    }
  else
    {
      m_err("ERR:%04d Unexpected boot stat:%d.\n",
            __LINE__, g_boot_stat);
    }

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_registercb
 *
 * Description:
 *   Register callback for ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_registercb(uint32_t type, altmdm_pm_cbfunc_t cb)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  if (type == MODEM_PM_CB_TYPE_ERROR)
    {
      if (g_pm_errcallback == NULL)
        {
          g_pm_errcallback = cb;
        }
    }
  else
    {
      if (g_pm_callback == NULL)
        {
          g_pm_callback = cb;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_deregistercb
 *
 * Description:
 *   Deregister callback for ALTMDM power manager driver.
 *
 ****************************************************************************/

int altmdm_pm_deregistercb(uint32_t type)
{
  altmdm_pm_cbfunc_t *lcallback = NULL;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  if (type == MODEM_PM_CB_TYPE_ERROR)
    {
      lcallback = &g_pm_errcallback;
    }
  else
    {
      lcallback = &g_pm_callback;
    }

  if (*lcallback == NULL)
    {
      return -EPERM;
    }

  *lcallback = NULL;

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_sleepmodem
 *
 * Description:
 *   Make modem sleep.
 *
 ****************************************************************************/

int altmdm_pm_sleepmodem(FAR struct altmdm_dev_s *priv)
{
  int ret;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP);
  ret = modem_sleep_procedure(priv);

  return ret;
}

/****************************************************************************
 * Name: altmdm_pm_cansleep
 *
 * Description:
 *   Check if modem can sleep.
 *
 ****************************************************************************/

int altmdm_pm_cansleep(FAR struct altmdm_dev_s *priv)
{
  int ret;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  ret = sleep_is_possible(priv);
  if (ret == 0)
    {
      m_info("possible to sleep.\n");
      ret = 1;
    }
  else
    {
      m_info("impossible to sleep.\n");
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_pm_initwakelock
 *
 * Description:
 *   Initialize the modem wakelock resource.
 *
 ****************************************************************************/

int altmdm_pm_initwakelock(FAR struct altmdm_pm_wakelock_s *lock)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  if (!lock)
    {
      return -EINVAL;
    }

  memset(lock, 0, sizeof(struct altmdm_pm_wakelock_s));

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_acquirewakelock
 *
 * Description:
 *   Acquire the modem wakelock.
 *
 ****************************************************************************/

int altmdm_pm_acquirewakelock(FAR struct altmdm_pm_wakelock_s *lock)
{
  irqstate_t flags;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  if (!lock)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (lock->count == 0)
    {
      sq_addlast((FAR sq_entry_t *) & lock->queue, &g_wakelock);
    }

  lock->count++;

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_releasewakelock
 *
 * Description:
 *   Release the modem wakelock.
 *
 ****************************************************************************/

int altmdm_pm_releasewakelock(FAR struct altmdm_pm_wakelock_s *lock)
{
  irqstate_t flags;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  if (!lock && (lock->count < 1))
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  lock->count--;

  if (lock->count == 0)
    {
      sq_rem((FAR sq_entry_t *) & lock->queue, &g_wakelock);
    }

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_getnumofwakelock
 *
 * Description:
 *   Get the lock count of the specified wakelock.
 *
 ****************************************************************************/

int altmdm_pm_getnumofwakelock(FAR struct altmdm_pm_wakelock_s *lock)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  if (!lock)
    {
      return -EINVAL;
    }

  return lock->count;
}

/****************************************************************************
 * Name: altmdm_pm_getwakelockstate
 *
 * Description:
 *   Get the wakelock status. If the return value is 0, it means that it is
 *   not locked. Otherwise it means that someone is locking.
 *
 ****************************************************************************/

int altmdm_pm_getwakelockstate(void)
{
  int num;
  irqstate_t flags;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  flags = enter_critical_section();

  num = sq_count(&g_wakelock);

  leave_critical_section(flags);

  return num;
}

/****************************************************************************
 * Name: altmdm_pm_poweron
 *
 * Description:
 *   Modem power on.
 *
 ****************************************************************************/

int altmdm_pm_poweron(FAR struct altmdm_dev_s *priv)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  priv->lower->poweron();

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_poweroff
 *
 * Description:
 *   Modem power off.
 *
 ****************************************************************************/

int altmdm_pm_poweroff(FAR struct altmdm_dev_s *priv)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  priv->lower->poweroff();
  sleep_modem_itself(priv);

  return 0;
}

/****************************************************************************
 * Name: altmdm_pm_set_bootstatus
 *
 * Description:
 *   Set boot status.
 *
 ****************************************************************************/

int altmdm_pm_set_bootstatus(FAR struct altmdm_dev_s *priv, uint32_t status)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  g_boot_stat = status;

  return 0;
}

#endif
