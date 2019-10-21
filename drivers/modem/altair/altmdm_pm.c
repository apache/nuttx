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
#include <nuttx/modem/altmdm.h>
#include "altmdm_sys.h"
#include "altmdm_pm.h"
#include "altmdm_pm_state.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#  define EVENT_BIT(b)                     (1<<(b))
#  define EVENT_NONE                       (0)
#  define EVENT_D2H_DOWN                   EVENT_BIT(0)
#  define EVENT_D2H_UP                     EVENT_BIT(1)
#  define EVENT_MODEM_SLEPT_NOTIF          EVENT_BIT(2)
#  define EVENT_MODEM_WAKEUP_NOTIF         EVENT_BIT(3)
#  define EVENT_MODEM_GOING_TO_SLEEP_NOTIF EVENT_BIT(4)
#  define EVENT_MODEM_WAKEUP_REQ           EVENT_BIT(5)
#  define EVENT_MODEM_SLEEP_REQ            EVENT_BIT(6)
#  define EVENT_EXIT                       EVENT_BIT(7)
#  define EVENT_MODEM_RESET_NOTIF          EVENT_BIT(8)
#  define EVENT_MODEM_POWERON_REQ          EVENT_BIT(9)
#  define EVENT_MODEM_POWEROFF_REQ         EVENT_BIT(10)
#  define EVENT_PM_TASK_WAIT               (EVENT_D2H_DOWN | EVENT_D2H_UP | \
                                           EVENT_MODEM_WAKEUP_REQ | \
                                           EVENT_MODEM_RESET_NOTIF | \
                                           EVENT_MODEM_POWERON_REQ | \
                                           EVENT_MODEM_POWEROFF_REQ | \
                                           EVENT_MODEM_SLEEP_REQ | EVENT_EXIT)
#  define EVENT_STATE_CHG_WAIT             (EVENT_MODEM_SLEPT_NOTIF | \
                                           EVENT_MODEM_GOING_TO_SLEEP_NOTIF | \
                                           EVENT_MODEM_WAKEUP_NOTIF)
#  define EVENT_MODEM_SLEEP_REQ_DONE_OK    EVENT_BIT(0)
#  define EVENT_MODEM_SLEEP_REQ_DONE_NG    EVENT_BIT(1)
#  define EVENT_MODEM_SLEEP_REQ_DONE       (EVENT_MODEM_SLEEP_REQ_DONE_OK | \
                                           EVENT_MODEM_SLEEP_REQ_DONE_NG)
#  define EVENT_MODEM_WAKEUP_REQ_DONE_OK   EVENT_BIT(0)
#  define EVENT_MODEM_WAKEUP_REQ_DONE_NG   EVENT_BIT(1)
#  define EVENT_MODEM_WAKEUP_REQ_DONE      (EVENT_MODEM_WAKEUP_REQ_DONE_OK | \
                                           EVENT_MODEM_WAKEUP_REQ_DONE_NG)
#  define EVENT_MODEM_POWER_ON_DONE        EVENT_BIT(0)
#  define EVENT_MODEM_POWER_OFF_DONE       EVENT_BIT(1)

/* Timeout is counted in units of millisecond. */

#  define D2H_UP_EVENT_TIMEOUT   (5*1000)
#  define ST_TRANS_EVENT_TIMEOUT (ALTMDM_SYS_FLAG_TMOFEVR)

#  define PM_TASK_PRI     (90)
#  define PM_TASK_NAME    "altmdm_pm_task"
#  define PM_TASK_STKSIZE (1024)

#  define GPIO_LOW                       (false)
#  define GPIO_HIGH                      (true)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct altmdm_dev_s *g_privdata = NULL;
static struct altmdm_sys_flag_s g_statetrans_flag;
static struct altmdm_sys_flag_s g_pmtask_flag;
static struct altmdm_sys_flag_s g_wakeup_done_flag;
static struct altmdm_sys_flag_s g_sleep_done_flag;
static struct altmdm_sys_flag_s g_power_done_flag;
static bool g_is_initdone = false;
static bool g_is_waitnotif = false;
static bool g_is_notrun;
static int g_taskid;
static altmdm_pm_cbfunc_t g_pm_callback = NULL;
static altmdm_pm_cbfunc_t g_pm_errcallback = NULL;
static sq_queue_t g_wakelock;
static uint32_t g_boot_stat = MODEM_PM_ERR_RESET_BOOTSTAT_NONE;

/****************************************************************************
 * Name: init_h2d_gpio
 *
 * Description:
 *   Initialize Host to Device GPIO line.
 *
 ****************************************************************************/

static int init_h2d_gpio(FAR struct altmdm_dev_s *priv)
{
  return 0;
}

/****************************************************************************
 * Name: uninit_h2d_gpio
 *
 * Description:
 *   Uninitialize Host to Device GPIO line.
 *
 ****************************************************************************/

static int uninit_h2d_gpio(FAR struct altmdm_dev_s *priv)
{
  return 0;
}

/****************************************************************************
 * Name: init_d2h_gpio
 *
 * Description:
 *   Initialize Device to Host GPIO line.
 *
 ****************************************************************************/

static int init_d2h_gpio(FAR struct altmdm_dev_s *priv)
{
  return 0;
}

/****************************************************************************
 * Name: uninit_d2h_gpio
 *
 * Description:
 *   Uninitialize Device to Host GPIO line.
 *
 ****************************************************************************/

static int uninit_d2h_gpio(FAR struct altmdm_dev_s *priv)
{
  return 0;
}

#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1

/****************************************************************************
 * Name: set_mreq_gpio
 *
 * Description:
 *   Control Master-request GPIO line.
 *
 ****************************************************************************/

static int set_mreq_gpio(FAR struct altmdm_dev_s *priv, bool value)
{
  return 0;
}
#  endif

/****************************************************************************
 * Name: set_h2d_gpio
 *
 * Description:
 *   Control Host to Device GPIO line.
 *
 ****************************************************************************/

static int set_h2d_gpio(FAR struct altmdm_dev_s *priv, bool value)
{
  return 0;
}

/****************************************************************************
 * Name: get_d2h_gpio
 *
 * Description:
 *   Get current value of Device to Host GPIO line.
 *
 ****************************************************************************/

static int get_d2h_gpio(FAR struct altmdm_dev_s *priv, FAR bool * value)
{
  return 0;
}

#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1

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
#  endif

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
 * Name: send_d2h_down_notif
 *
 * Description:
 *   Notify that Device to Host GPIO line has been deassertted.
 *
 ****************************************************************************/

static int send_d2h_down_notif(FAR struct altmdm_dev_s *priv)
{
  int ret = 0;

  ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_D2H_DOWN);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_d2h_up_notif
 *
 * Description:
 *   Notify that Device to Host GPIO line has been assertted.
 *
 ****************************************************************************/

static int send_d2h_up_notif(FAR struct altmdm_dev_s *priv)
{
  int ret = 0;
  uint32_t pm_state;

  pm_state = altmdm_pm_getinternalstate();
  if ((pm_state == MODEM_PM_INTERNAL_STATE_SLEEP) ||
      (pm_state == MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE))
    {
      ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_D2H_UP);
      if (ret != 0)
        {
          m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_slept_notif
 *
 * Description:
 *   Notify that the modem slept.
 *
 ****************************************************************************/

static int send_modem_slept_notif(FAR struct altmdm_dev_s *priv)
{
  int ret = 0;

  if (g_is_waitnotif)
    {
      /* Send only when waiting for notification. */

      ret = altmdm_sys_setflag(&g_statetrans_flag, EVENT_MODEM_SLEPT_NOTIF);
      if (ret != 0)
        {
          m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_wakeup_notif
 *
 * Description:
 *   Notify that the modem has woken up.
 *
 ****************************************************************************/

static int send_modem_wakeup_notif(FAR struct altmdm_dev_s *priv)
{
  int ret = 0;

  if (g_is_waitnotif)
    {
      /* Send only when waiting for notification. */

      ret = altmdm_sys_setflag(&g_statetrans_flag, EVENT_MODEM_WAKEUP_NOTIF);
      if (ret != 0)
        {
          m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
        }
    }
#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1
  else
    {
      altmdm_spi_setreceiverready(priv);
    }
#  endif

  return ret;
}

/****************************************************************************
 * Name: send_modem_reset_notif
 *
 * Description:
 *   Notify that the modem is reset.
 *
 ****************************************************************************/

static int send_modem_reset_notif(FAR struct altmdm_dev_s *priv)
{
  int ret = 0;

  ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_MODEM_RESET_NOTIF);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_wakeup_request
 *
 * Description:
 *   Send request to wake up the modem.
 *
 ****************************************************************************/

static int send_modem_wakeup_request(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_MODEM_WAKEUP_REQ);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_sleep_request
 *
 * Description:
 *   Send request to sleep the modem.
 *
 ****************************************************************************/

static int send_modem_sleep_request(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_MODEM_SLEEP_REQ);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_exit_request
 *
 * Description:
 *   Send request to terminate the power management task.
 *
 ****************************************************************************/

static int send_exit_request(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_EXIT);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_poweron_request
 *
 * Description:
 *   Send request to power on the modem.
 *
 ****************************************************************************/

static int send_modem_poweron_request(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_MODEM_POWERON_REQ);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_poweroff_request
 *
 * Description:
 *   Send request to power off the modem.
 *
 ****************************************************************************/

static int send_modem_poweroff_request(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_setflag(&g_pmtask_flag, EVENT_MODEM_POWEROFF_REQ);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_wakeup_done
 *
 * Description:
 *   Notify that the wake up of the modem is complete.
 *
 ****************************************************************************/

static int send_modem_wakeup_done(FAR struct altmdm_dev_s *priv, int result)
{
  int ret;
  uint32_t ptn;

  if (result == 0)
    {
      ptn = EVENT_MODEM_WAKEUP_REQ_DONE_OK;
    }
  else
    {
      ptn = EVENT_MODEM_WAKEUP_REQ_DONE_NG;
    }
  ret = altmdm_sys_setflag(&g_wakeup_done_flag, ptn);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_sleep_done
 *
 * Description:
 *   Notify that the modem slept.
 *
 ****************************************************************************/

static int send_modem_sleep_done(FAR struct altmdm_dev_s *priv, int result)
{
  int ret;
  uint32_t ptn;

  if (result == 0)
    {
      ptn = EVENT_MODEM_SLEEP_REQ_DONE_OK;
    }
  else
    {
      ptn = EVENT_MODEM_SLEEP_REQ_DONE_NG;
    }

  ret = altmdm_sys_setflag(&g_sleep_done_flag, ptn);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_poweron_done
 *
 * Description:
 *   Notify that the modem power on.
 *
 ****************************************************************************/

static int send_modem_poweron_done(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_setflag(&g_power_done_flag, EVENT_MODEM_POWER_ON_DONE);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: send_modem_poweroff_done
 *
 * Description:
 *   Notify that the modem power off.
 *
 ****************************************************************************/

static int send_modem_poweroff_done(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = altmdm_sys_setflag(&g_power_done_flag, EVENT_MODEM_POWER_OFF_DONE);
  if (ret != 0)
    {
      m_err("ERR:%04d Set flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wait_for_d2h_up_complete
 *
 * Description:
 *   Wait until the Device to Host GPIO line is assert.
 *
 ****************************************************************************/

static int wait_for_d2h_up_complete(FAR struct altmdm_dev_s *priv)
{
  int ret;
  uint32_t ptn;

  ret = altmdm_sys_waitflag(&g_pmtask_flag, EVENT_D2H_UP,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            D2H_UP_EVENT_TIMEOUT);

  return ret;
}

/****************************************************************************
 * Name: wait_for_modem_state_change_notif
 *
 * Description:
 *   Wait until the state of the modem changes.
 *
 ****************************************************************************/

static int wait_for_modem_state_change_notif(FAR struct altmdm_dev_s *priv,
                                             FAR uint32_t * state)
{
  int ret;
  uint32_t ptn;

  g_is_waitnotif = true;

  ret = altmdm_sys_waitflag(&g_statetrans_flag, EVENT_STATE_CHG_WAIT,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            ST_TRANS_EVENT_TIMEOUT);
  if (ret != 0)
    {
      m_err("ERR:%04d Wait flag:%d.\n", __LINE__, ret);
    }
  else
    {
      if (ptn & EVENT_MODEM_SLEPT_NOTIF)
        {
          *state = MODEM_PM_INTERNAL_STATE_SLEEP;
        }
      else if (ptn & EVENT_MODEM_WAKEUP_NOTIF)
        {
          *state = MODEM_PM_INTERNAL_STATE_WAKE;
        }
      else if (ptn & EVENT_MODEM_GOING_TO_SLEEP_NOTIF)
        {
          *state = MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP;
        }
      else
        {
          *state = MODEM_PM_INTERNAL_STATE_MAX;
        }
    }

  g_is_waitnotif = false;

  return ret;
}

/****************************************************************************
 * Name: wait_for_wakeup_requeset_done
 *
 * Description:
 *   Wait until the wakeup request is completed.
 *
 ****************************************************************************/

static int wait_for_wakeup_requeset_done(FAR struct altmdm_dev_s *priv)
{
  int ret;
  uint32_t ptn;

  ret = altmdm_sys_waitflag(&g_wakeup_done_flag,
                            EVENT_MODEM_WAKEUP_REQ_DONE,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            ALTMDM_SYS_FLAG_TMOFEVR);
  if (ret != 0)
    {
      m_err("ERR:%04d Wait flag:%d.\n", __LINE__, ret);
    }
  else
    {
      if (ptn & EVENT_MODEM_WAKEUP_REQ_DONE_NG)
        {
          ret = -1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: wait_for_sleep_requeset_done
 *
 * Description:
 *   Wait until the sleep request is completed.
 *
 ****************************************************************************/

static int wait_for_sleep_requeset_done(FAR struct altmdm_dev_s *priv)
{
  int ret;
  uint32_t ptn;

  ret = altmdm_sys_waitflag(&g_sleep_done_flag, EVENT_MODEM_SLEEP_REQ_DONE,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            ALTMDM_SYS_FLAG_TMOFEVR);
  if (ret != 0)
    {
      m_err("ERR:%04d Wait flag:%d.\n", __LINE__, ret);
    }
  else
    {
      if (ptn & EVENT_MODEM_SLEEP_REQ_DONE_NG)
        {
          ret = -1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: wait_for_poweron_done
 *
 * Description:
 *   Wait until the power on is completed.
 *
 ****************************************************************************/

static int wait_for_poweron_done(FAR struct altmdm_dev_s *priv)
{
  int ret;
  uint32_t ptn;

  ret = altmdm_sys_waitflag(&g_power_done_flag, EVENT_MODEM_POWER_ON_DONE,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            ALTMDM_SYS_FLAG_TMOFEVR);
  if (ret != 0)
    {
      m_err("ERR:%04d Wait flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wait_for_poweroff_done
 *
 * Description:
 *   Wait until the power off is completed.
 *
 ****************************************************************************/

static int wait_for_poweroff_done(FAR struct altmdm_dev_s *priv)
{
  int ret;
  uint32_t ptn;

  ret = altmdm_sys_waitflag(&g_power_done_flag, EVENT_MODEM_POWER_OFF_DONE,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            ALTMDM_SYS_FLAG_TMOFEVR);
  if (ret != 0)
    {
      m_err("ERR:%04d Wait flag:%d.\n", __LINE__, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: d2h_gpio_isr
 *
 * Description:
 *   Interrupt handler for Device to Host GPIO line.
 *
 ****************************************************************************/

static void d2h_gpio_isr(FAR struct altmdm_dev_s *priv)
{
  int ret;
  bool val = GPIO_LOW;

  ret = get_d2h_gpio(priv, &val);
  if (ret == 0)
    {
      switch (val)
        {
        case GPIO_LOW:

          ret = send_d2h_down_notif(priv);
          if (ret != 0)
            {
              m_err("ERR:%04d send d2h down notification:%d.\n", __LINE__,
                    ret);
            }
          break;

        case GPIO_HIGH:

          ret = send_d2h_up_notif(priv);
          if (ret != 0)
            {
              m_err("ERR:%04d send device up notification:%d.\n",
                    __LINE__, ret);
            }
          break;

        default:
          m_err("ERR:%04d Invalid D2H GPIO value:%d.\n", __LINE__, val);
          break;
        }
    }
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
 * Name: wakeup_modem_itself
 *
 * Description:
 *   The modem wakes up by itself, it transitions to the wakeup state.
 *
 ****************************************************************************/

static int wakeup_modem_itself(FAR struct altmdm_dev_s *priv, bool wakeupreq)
{
  int ret;

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE);

  ret = set_h2d_up(priv);
  if (ret == 0)
    {
      /* Transitions to the wakeup state. */

      altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_WAKE);

      /* When the EVENT_MODEM_WAKEUP_REQ is simultaneous,
       *  there is no need to notify.
       */

      if (!wakeupreq)
        {
          /* Perform processing at the time of state transition. */

          send_modem_wakeup_notif(priv);
        }

      exe_callback(MODEM_PM_CB_TYPE_NORMAL, MODEM_PM_CB_WAKE);
    }

  return ret;
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

  send_modem_slept_notif(priv);
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

      send_modem_slept_notif(priv);
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

static int modem_wakeup_procedure(FAR struct altmdm_dev_s *priv)
{
  int ret = 0;

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE);

  ret = set_h2d_up(priv);
  if (ret == 0)
    {
#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1
      set_mreq_up(priv);
#  endif
      ret = wait_for_d2h_up_complete(priv);
      if (ret != 0)
        {
          m_err("ERR:%04d device is not up.\n", __LINE__);

          /* Do rollback. */

#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1
          set_mreq_down(priv);
#  endif
          set_h2d_down(priv);

          altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_SLEEP);
        }
      else
        {
          /* Transitions to the wake state. */

          altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_WAKE);

          /* Perform processing at the time of state transition. */

          exe_callback(MODEM_PM_CB_TYPE_NORMAL, MODEM_PM_CB_WAKE);
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
 * Name: modem_sleep_req
 *
 * Description:
 *   Execute the sleep request.
 *
 ****************************************************************************/

static int modem_sleep_req(FAR struct altmdm_dev_s *priv)
{
  int ret;

  altmdm_pm_setinternalstate(MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP);

  ret = send_modem_sleep_request(priv);
  if (ret == 0)
    {
      ret = wait_for_sleep_requeset_done(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: modem_wakeup_req
 *
 * Description:
 *   Execute the wakeup request.
 *
 ****************************************************************************/

static int modem_wakeup_req(FAR struct altmdm_dev_s *priv)
{
  int ret;

  ret = send_modem_wakeup_request(priv);
  if (ret == 0)
    {
      ret = wait_for_wakeup_requeset_done(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: pm_task
 *
 * Description:
 *   ALTMDM power manager driver task.
 *
 ****************************************************************************/

static int pm_task(int argc, FAR char *argv[])
{
  int ret;
  uint32_t ptn;
  FAR struct altmdm_dev_s *priv = g_privdata;

  while (!g_is_notrun)
    {
      ret = altmdm_sys_waitflag(&g_pmtask_flag, EVENT_PM_TASK_WAIT,
                                ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                                ALTMDM_SYS_FLAG_TMOFEVR);
      if (ret != 0)
        {
          m_err("ERR:%04d Wait Flag:%d.\n", __LINE__, ret);
        }
      else
        {
          m_info("pm_task ptn:%x.\n", ptn);

          if (ptn & EVENT_MODEM_WAKEUP_REQ)
            {
              if (ptn & EVENT_D2H_UP)
                {
                  /* Modem already wakeup. */

                  ret = wakeup_modem_itself(priv, true);

#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1
                  set_mreq_up(priv);
#  endif
                }
              else
                {
                  /* Make modem wakeup. */

                  ret = modem_wakeup_procedure(priv);
                }
              send_modem_wakeup_done(priv, ret);

              if (ptn & EVENT_MODEM_SLEEP_REQ)
                {
                  /* This condition does not occur,
                   * but it is due to fail safe.
                   */

                  send_modem_sleep_done(priv, -1);
                }
            }
          else if (ptn & EVENT_MODEM_SLEEP_REQ)
            {
              /* Make modem sleep. */

              ret = modem_sleep_procedure(priv);
              send_modem_sleep_done(priv, ret);
            }
          else if (ptn & EVENT_D2H_UP)
            {
              /* Modem has wake itself. */

              wakeup_modem_itself(priv, false);
            }
          else if (ptn & EVENT_D2H_DOWN)
            {
              /* Modem become sleep itself. */

              sleep_modem_itself(priv);
            }

          if (ptn & EVENT_MODEM_POWERON_REQ)
            {
              /* Modem power on. */

              send_modem_poweron_done(priv);
            }

          if (ptn & EVENT_MODEM_POWEROFF_REQ)
            {
              /* Modem power off. */

              sleep_modem_itself(priv);

              send_modem_poweroff_done(priv);
            }

          if (ptn & EVENT_MODEM_RESET_NOTIF)
            {
              if (MODEM_PM_ERR_RESET_BOOTSTAT_NONE != g_boot_stat)
                {
                  exe_callback(MODEM_PM_CB_TYPE_ERROR, g_boot_stat);
                }
              else
                {
                  m_err("ERR:%04d Unexpected boot stat:%d.\n",
                        __LINE__, g_boot_stat);
                }
            }

          if (ptn & EVENT_EXIT)
            {
              g_is_notrun = true;
            }
        }
    }

  task_delete(0);

  return 0;
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

  /* Create event flags. */

  altmdm_sys_initflag(&g_statetrans_flag);
  altmdm_sys_initflag(&g_pmtask_flag);
  altmdm_sys_initflag(&g_wakeup_done_flag);
  altmdm_sys_initflag(&g_sleep_done_flag);
  altmdm_sys_initflag(&g_power_done_flag);

  sq_init(&g_wakelock);

  /* Initialize GPIO. */

  init_h2d_gpio(priv);
  init_d2h_gpio(priv);

  /* Create Power management task. */

  g_is_notrun = false;

  g_taskid = task_create(PM_TASK_NAME, PM_TASK_PRI, PM_TASK_STKSIZE,
                         (main_t) pm_task, NULL);
  if (g_taskid == ERROR)
    {
      m_err("Failed to create pm task\n");
    }

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

  /* Delete Transfer task. */

  send_exit_request(priv);

  /* Check pm task is deleted or not. */

  while (1)
    {
      if (g_is_notrun)
        {
          break;
        }
      usleep(10);
    }

  /* Delete event flags. */

  altmdm_sys_deleteflag(&g_statetrans_flag);
  altmdm_sys_deleteflag(&g_pmtask_flag);
  altmdm_sys_deleteflag(&g_wakeup_done_flag);
  altmdm_sys_deleteflag(&g_sleep_done_flag);
  altmdm_sys_deleteflag(&g_power_done_flag);

  /* Uinitialize GPIO. */

  uninit_h2d_gpio(priv);
  uninit_d2h_gpio(priv);

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

int altmdm_pm_wakeup(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int retval = MODEM_PM_WAKEUP_FAIL;
  bool is_slept = false;
  uint32_t modem_state;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  modem_state = altmdm_pm_getinternalstate();
  switch (modem_state)
    {
    case MODEM_PM_INTERNAL_STATE_WAKE:

      retval = MODEM_PM_WAKEUP_ALREADY;

#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1
      /* Case where modem spontaneously enters the wake state and already
         notified by altmdm_spi_setreceiverready().
      */

      if (altmdm_spi_isreceiverready(priv))
        {
          altmdm_spi_clearreceiverready(priv);
          retval = MODEM_PM_WAKEUP_DONE;
        }
#  endif
      break;

    case MODEM_PM_INTERNAL_STATE_SLEEP:

      is_slept = true;
      break;

    case MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP:
    case MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE:

      /* Wait for change modem state. */

      wait_for_modem_state_change_notif(priv, &modem_state);

      if (modem_state == MODEM_PM_INTERNAL_STATE_SLEEP)
        {
          is_slept = true;
        }
      else if (modem_state == MODEM_PM_INTERNAL_STATE_WAKE)
        {
          retval = MODEM_PM_WAKEUP_DONE;
        }
      else
        {
          m_err("ERR:%04d unexpected event occurr. state:%d.\n",
                __LINE__, modem_state);
        }

      break;

    default:
      m_err("ERR:%04d unexpected event occurr. state:%d.\n",
            __LINE__, modem_state);
    }

  if (is_slept)
    {
      /* Wakeup request. */

      ret = modem_wakeup_req(priv);
      if (ret == 0)
        {
          retval = MODEM_PM_WAKEUP_DONE;
        }
    }

  return retval;
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

  send_modem_reset_notif(priv);

  return 0;
}

#  ifdef CONFIG_MODEM_ALTMDM_PROTCOL_V2_1

/****************************************************************************
 * Name: altmdm_pm_callgpiohandler
 *
 * Description:
 *   Call Device to Host GPIO interrupt handler.
 *
 ****************************************************************************/

int altmdm_pm_callgpiohandler(FAR struct altmdm_dev_s *priv)
{
  if (!g_is_initdone)
    {
      return -EPERM;
    }

  d2h_gpio_isr(priv);

  return 0;
}
#  endif

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

  ret = modem_sleep_req(priv);

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
  int ret;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  ret = send_modem_poweron_request(priv);
  if (ret == 0)
    {
      ret = wait_for_poweron_done(priv);
    }

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
  int ret;

  if (!g_is_initdone)
    {
      return -EPERM;
    }

  ret = send_modem_poweroff_request(priv);
  if (ret == 0)
    {
      ret = wait_for_poweroff_done(priv);
    }

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
