/****************************************************************************
 * drivers/modem/alt1250/altmdm.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_MODEM_ALT1250)

#include <stdio.h>
#include <unistd.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wireless/lte/lte.h>

#include <nuttx/modem/alt1250.h>  /* for ALTCOM_VERx */

#include "altmdm.h"
#include "altmdm_event.h"
#include "altmdm_spi.h"
#include "altmdm_timer.h"
#include "altcom_pkt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EVENT_POWERON  (1 << 0)
#define EVENT_POWEROFF (1 << 1)
#define EVENT_RESET    (1 << 2)
#define EVENT_WLOCK    (1 << 3)
#define EVENT_TXREQ    (1 << 4)
#define EVENT_RXREQ    (1 << 5)
#define EVENT_TXSUSTO  (1 << 6)
#define EVENT_DESTROY  (1 << 7)

#define TX_DONE        (1 << 0)
#define TX_CANCEL      (1 << 1)

#define RESET_INTERVAL (50*1000)
#define TXSUS_TIMEOUT  (100)
#define TIMEOUT_IDELEWTO_STATE (20) /* Sleep timer */
#define TIMEOUT_HDR_TRX_STATE (5000)
#define TIMEOUT_BODY_TRX_STATE (5)
#define TIMEOUT_NEXT_DELAY (300)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum altmdm_state_e
{
  ALTMDM_STATE_POWEROFF = 0,     /* Modem Power Off state */
  ALTMDM_STATE_SLEEP,            /* Modem Sleep state */
  ALTMDM_STATE_SLEEPWOTX,        /* Modem Sleep with Tx suspend state */
  ALTMDM_STATE_IDLE4RST,         /* Idle for Reset transaction state */
  ALTMDM_STATE_IDLEWTO,          /* Idle with Sleep Time Out state */
  ALTMDM_STATE_IDLEWOTO,         /* Idle without Sleep Time Out state */
  ALTMDM_STATE_IDLEWOTX,         /* Idle with Tx suspend */
  ALTMDM_STATE_V1SET,            /* Altcom version 1 command setting state */
  ALTMDM_STATE_V4SET,            /* Altcom version 4 command setting state */
  ALTMDM_STATE_SLEEPSET,         /* Sleep packet setting state */
  ALTMDM_STATE_TXPREPARE,        /* Normal packet setting state */
  ALTMDM_STATE_TXREQ,            /* TX request signal assertion state */
  ALTMDM_STATE_HDRSREQ,          /* Waiting for Slave request signal state */
  ALTMDM_STATE_HDRTRX,           /* SPI Header transaction state */
  ALTMDM_STATE_SLEEPPKT,         /* Sleep Packet body size adjustment
                                  * state */
  ALTMDM_STATE_BODYSREQ,         /* Waiting for Slave request signal for body
                                  * state */
  ALTMDM_STATE_BODYTRX,          /* SPI body transaction state */
  ALTMDM_STATE_GOTRX,            /* Received normal body state */
  ALTMDM_STATE_GOTRST,           /* Received reset pakcet body state */
  ALTMDM_STATE_GOTSLEEP,         /* Received sleep packet body state */
  ALTMDM_STATE_BACKTOIDLE,       /* Back to Idle state */
  ALTMDM_STATE_RETRECV,          /* Return state */
  ALTMDM_STATE_FORCERST,         /* Modem force reset state */
  ALTMDM_STATE_SLEEPING,         /* Waiting for transition to sleep state */
  ALTMDM_STATE_DECIDEDELAY,      /* Determine if there is a need to delay for
                                  * next TRX */
  ALTMDM_STATE_DELAYNEXT,        /* Delayed state for the next TRX */
  ALTMDM_STATE_SETSUSTIMER,      /* Start TX suspend timer state */
  ALTMDM_STATE_SETSUSTIMERSLEEP, /* Start TX suspend timer when sleep
                                  * state */
  ALTMDM_STATE_DESTORY,          /* State to be destroyed */
} altmdm_state_t;

typedef enum version_phase_e
{
  VP_NO_RESET,          /* Reset packet is not received */
  VP_V1 = ALTCOM_VER1,  /* Confirmed Altcom version is 1 */
  VP_V4 = ALTCOM_VER4,  /* Confirmed Altcom version is 4 */
  VP_UNKNOWN,           /* Altcom version is unknown */
  VP_TRYV1,             /* Try sending version 1 packet */
  VP_NOTV1,             /* Trial of version 1 packet is fail */
  VP_TRYV4,             /* Try sending version 4 packet */
} version_phase_t;

struct state_func_s
{
  int (*goto_next)(altmdm_state_t);
  uint32_t (*wait_event)(void);
  altmdm_state_t (*process_state)(uint32_t, altmdm_state_t);
#ifdef CONFIG_MODEM_ALT1250_DEBUG
  const char *name;
#endif
};

typedef struct altmdm_dev_s
{
  sem_t lock_evt;
  struct altmdm_event_s event;
  sem_t lock_vp;
  version_phase_t vp;
  sem_t lock_counter;
  int wcounter;
  FAR struct spi_dev_s *spidev;
  const struct alt1250_lower_s *lower;
  timer_t txsus_timer;
  altmdm_state_t current_state;
  altmdm_spipkt_t tx_pkt;
  altmdm_spipkt_t rx_pkt;
  int rx_retcode;
  struct altmdm_event_s txdone_event;
  sem_t lock_txreq;
  void *txreq_buff;
  int txreq_size;
  int is_destroy;
  uint32_t reset_reason;
} altmdm_dev_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int next_state_common(altmdm_state_t);

static int next_state_poweroff(altmdm_state_t);
static int next_state_sleep(altmdm_state_t);
static int next_state_sleepwotx(altmdm_state_t);
static int next_state_idle4rst(altmdm_state_t);
static int next_state_idlewto(altmdm_state_t);
static int next_state_idlewoto(altmdm_state_t);
static int next_state_idlewotx(altmdm_state_t);
static int next_state_decidedelay(altmdm_state_t);
static int next_state_destroy(altmdm_state_t);

static uint32_t waitevt_state_common(void);

static uint32_t waitevt_state_poweroff(void);
static uint32_t waitevt_state_sleep(void);
static uint32_t waitevt_state_sleepwotx(void);
static uint32_t waitevt_state_idle4rst(void);
static uint32_t waitevt_state_idlewto(void);
static uint32_t waitevt_state_idlewoto(void);
static uint32_t waitevt_state_idlewotx(void);
static uint32_t waitevt_state_hdrsreq(void);
static uint32_t waitevt_state_bodysreq(void);
static uint32_t waitevt_state_sleeping(void);
static uint32_t waitevt_state_delaynext(void);

static altmdm_state_t process_state_common(uint32_t, altmdm_state_t);

static altmdm_state_t process_state_poweroff(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_sleep(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_sleepwotx(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_idle4rst(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_idlewto(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_idlewoto(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_idlewotx(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_v1set(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_v4set(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_sleepset(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_txprepare(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_txreq(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_hdrsreq(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_hdrtrx(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_sleeppkt(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_bodysreq(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_bodytrx(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_gotrx(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_gotrst(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_gotsleep(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_backtoidle(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_retrecv(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_forcerst(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_sleeping(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_decidedelay(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_delaynext(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_setsustimer(uint32_t, altmdm_state_t);
static altmdm_state_t process_state_setsustimersleep(uint32_t,
  altmdm_state_t);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_MODEM_ALT1250_DEBUG
#  define TABLE_CONTENT(array_name, namea, nameb, namec) \
    [ALTMDM_STATE_##array_name] = \
    { next_state_##namea, waitevt_state_##nameb, process_state_##namec }
#else
#  define TABLE_CONTENT(array_name, namea, nameb, namec) \
    [ALTMDM_STATE_##array_name] = \
    { next_state_##namea, waitevt_state_##nameb, process_state_##namec, \
      #array_name }

static char *g_vp_name[] =
{
  "NO_RESET", "V1      ", "", "",
  "V4      ", "UNKNOWN ", "TRYV1   ", "NOTV1   ",
  "TRYV4   "
};
#endif

/* State functions instance table */

static const struct state_func_s g_state_func[] =
{
  TABLE_CONTENT(POWEROFF,         poweroff,    poweroff,  poweroff),
  TABLE_CONTENT(SLEEP,            sleep,       sleep,     sleep),
  TABLE_CONTENT(SLEEPWOTX,        sleepwotx,   sleepwotx, sleepwotx),
  TABLE_CONTENT(IDLE4RST,         idle4rst,    idle4rst,  idle4rst),
  TABLE_CONTENT(IDLEWTO,          idlewto,     idlewto,   idlewto),
  TABLE_CONTENT(IDLEWOTO,         idlewoto,    idlewoto,  idlewoto),
  TABLE_CONTENT(IDLEWOTX,         idlewotx,    idlewotx,  idlewotx),
  TABLE_CONTENT(V1SET,            common,      common,    v1set),
  TABLE_CONTENT(V4SET,            common,      common,    v4set),
  TABLE_CONTENT(SLEEPSET,         common,      common,    sleepset),
  TABLE_CONTENT(TXPREPARE,        common,      common,    txprepare),
  TABLE_CONTENT(TXREQ,            common,      common,    txreq),
  TABLE_CONTENT(HDRSREQ,          common,      hdrsreq,   hdrsreq),
  TABLE_CONTENT(HDRTRX,           common,      common,    hdrtrx),
  TABLE_CONTENT(SLEEPPKT,         common,      common,    sleeppkt),
  TABLE_CONTENT(BODYSREQ,         common,      bodysreq,  bodysreq),
  TABLE_CONTENT(BODYTRX,          common,      common,    bodytrx),
  TABLE_CONTENT(GOTRX,            common,      common,    gotrx),
  TABLE_CONTENT(GOTRST,           common,      common,    gotrst),
  TABLE_CONTENT(GOTSLEEP,         common,      common,    gotsleep),
  TABLE_CONTENT(BACKTOIDLE,       common,      common,    backtoidle),
  TABLE_CONTENT(RETRECV,          common,      common,    retrecv),
  TABLE_CONTENT(FORCERST,         common,      common,    forcerst),
  TABLE_CONTENT(SLEEPING,         common,      sleeping,  sleeping),
  TABLE_CONTENT(DECIDEDELAY,      decidedelay, common,    decidedelay),
  TABLE_CONTENT(DELAYNEXT,        common,      delaynext, delaynext),
  TABLE_CONTENT(SETSUSTIMER,      common,      common,    setsustimer),
  TABLE_CONTENT(SETSUSTIMERSLEEP, common,      common,    setsustimersleep),
  TABLE_CONTENT(DESTORY,          destroy,     common,    common),
};

static struct altmdm_dev_s g_altmdm_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_MODEM_ALT1250_DEBUG
static void dump_current_all_status(altmdm_dev_t *dev, uint32_t evt,
  altmdm_state_t next, int is_exit)
{
  m_info("state[%s => %s], vp[%s], event out[%08lx]:cur[%08lx], "
    "wcount=%2d, txreq_buf=%s, txreq_sz=%4d, "
    "tx_hdr=%08lx, rx_hdr=%08lx, exit=%s\n",
    g_state_func[dev->current_state].name, g_state_func[next].name,
    g_vp_name[dev->vp], evt, dev->event.event, dev->wcounter,
    dev->txreq_buff == NULL ? "No-req " : "Request", dev->txreq_size,
    dev->tx_pkt.header, dev->rx_pkt.header, is_exit ? "Yes" : "No ");
}
#else
#  define dump_current_all_status(...)
#endif

static int sready_isr(int irq, FAR void *context, FAR void *arg)
{
  altmdm_event_set(&g_altmdm_dev.event, EVENT_RXREQ);

  return 0;
}

static void txsustimer_handler(int signo, FAR siginfo_t *info,
  FAR void *uctx)
{
  FAR struct altmdm_dev_s *priv =
    (FAR struct altmdm_dev_s *)(info->si_value.sival_ptr);

  altmdm_event_set(&priv->event, EVENT_TXSUSTO);
}

/****************************************************************************
 * get wakelock count function
 ****************************************************************************/

static int get_wlock_count(void)
{
  int cnt;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_counter);
  cnt = g_altmdm_dev.wcounter;
  nxsem_post(&g_altmdm_dev.lock_counter);

  return cnt;
}

/****************************************************************************
 * Version Phase access functions
 ****************************************************************************/

static enum version_phase_e get_vp(void)
{
  enum version_phase_e vp;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_vp);
  vp = g_altmdm_dev.vp;
  nxsem_post(&g_altmdm_dev.lock_vp);

  return vp;
}

static void set_vp(enum version_phase_e vp)
{
  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_vp);
  g_altmdm_dev.vp = vp;
  nxsem_post(&g_altmdm_dev.lock_vp);
}

static bool is_vp_valid(void)
{
  enum version_phase_e vp;
  vp = get_vp();
  return (vp == VP_V1 || vp == VP_V4);
}

static bool is_vp_noreset(void)
{
  return (get_vp() == VP_NO_RESET);
}

/****************************************************************************
 * return code setting function
 ****************************************************************************/

static void set_return_code(int code)
{
  g_altmdm_dev.rx_retcode = code;
}

/****************************************************************************
 * reset reason setting/getting functions
 ****************************************************************************/

static void set_reset_reason(uint32_t reason)
{
  g_altmdm_dev.reset_reason = reason;
}

static uint32_t get_reset_reason(void)
{
  return g_altmdm_dev.reset_reason;
}

/****************************************************************************
 * tx done function
 ****************************************************************************/

static void tx_done(uint32_t result)
{
  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_txreq);
  g_altmdm_dev.txreq_buff = NULL;
  g_altmdm_dev.txreq_size = 0;
  altmdm_event_clear(&g_altmdm_dev.event, EVENT_TXREQ);
  altmdm_event_set(&g_altmdm_dev.txdone_event, result);
  nxsem_post(&g_altmdm_dev.lock_txreq);
}

/****************************************************************************
 * converts unit of timer function
 ****************************************************************************/

static void usec2timespec(useconds_t usec, FAR struct timespec *timespec)
{
  time_t sec;

  sec = usec / 1000000;
  timespec->tv_sec = sec;
  timespec->tv_nsec = (usec - (sec * 1000000)) * 1000;
}

static void process_before_poweroff(void)
{
  uint32_t allevt = (uint32_t)(-1);

  set_vp(VP_NO_RESET);
  altmdm_timer_restart(g_altmdm_dev.txsus_timer, 0, 0);

  /* Cancel tx request. */

  tx_done(TX_CANCEL);

  /* clear event without EVENT_POWERON */

  altmdm_event_clear(&g_altmdm_dev.event, (allevt & ~(EVENT_POWERON)));
  g_altmdm_dev.lower->irqenable(false);
  g_altmdm_dev.lower->set_wakeup(false);
  g_altmdm_dev.lower->set_mready(false);
}

/****************************************************************************
 * force reset function
 ****************************************************************************/

static void force_reset(void)
{
  if (is_vp_valid())
    {
      set_reset_reason(LTE_RESTART_MODEM_INITIATED);
    }

  process_before_poweroff();
  g_altmdm_dev.lower->reset();
  g_altmdm_dev.lower->irqenable(true);
}

/****************************************************************************
 * State common functions
 ****************************************************************************/

static int next_state_common(altmdm_state_t state)
{
  return 0;
}

static uint32_t waitevt_state_common(void)
{
  return 0;
}

static altmdm_state_t process_state_common(uint32_t event,
  altmdm_state_t state)
{
  return state;
}

/****************************************************************************
 * On POWEROFF state
 ****************************************************************************/

static int next_state_poweroff(altmdm_state_t state)
{
  process_before_poweroff();
  g_altmdm_dev.lower->poweroff();

  return 0;
}

static uint32_t waitevt_state_poweroff(void)
{
  return altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_POWERON | EVENT_DESTROY, false, 0);
}

static altmdm_state_t process_state_poweroff(uint32_t event,
  altmdm_state_t state)
{
  struct timespec interval;

  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWERON)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWERON);
      usec2timespec(RESET_INTERVAL, &interval);
      nxsig_nanosleep(&interval, NULL);
      g_altmdm_dev.spidev = g_altmdm_dev.lower->poweron();
      g_altmdm_dev.lower->set_mready(false);
      g_altmdm_dev.lower->set_wakeup(false);
      g_altmdm_dev.lower->irqenable(true);
      set_reset_reason(LTE_RESTART_USER_INITIATED);
      state = ALTMDM_STATE_SLEEP;
    }

  return state;
}

/****************************************************************************
 * SLEEP state
 ****************************************************************************/

static int next_state_sleep(altmdm_state_t state)
{
  g_altmdm_dev.lower->set_wakeup(false);

  return 0;
}

static uint32_t waitevt_state_sleep(void)
{
  uint32_t event;

  if (!is_vp_valid())
    {
      return 0;
    }

  event = altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_TXREQ | EVENT_RXREQ | EVENT_WLOCK | EVENT_POWEROFF | EVENT_RESET |
    EVENT_DESTROY, false, 0);

  return event;
}

static altmdm_state_t process_state_sleep(uint32_t event,
  altmdm_state_t state)
{
  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RESET)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RESET);
      state = ALTMDM_STATE_FORCERST;
    }
  else if (!is_vp_valid())
    {
      state = ALTMDM_STATE_IDLE4RST;
    }
  else if (event & (EVENT_TXREQ | EVENT_RXREQ))
    {
      state = ALTMDM_STATE_IDLEWTO;
    }
  else if (event & EVENT_WLOCK)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_WLOCK);
      if (get_wlock_count() != 0)
        {
          state = ALTMDM_STATE_IDLEWOTO;
        }
    }

  return state;
}

/****************************************************************************
 * SLEEPWOTX state
 ****************************************************************************/

static int next_state_sleepwotx(altmdm_state_t state)
{
  g_altmdm_dev.lower->set_wakeup(false);

  return 0;
}

static uint32_t waitevt_state_sleepwotx(void)
{
  uint32_t event;

  event = altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_RXREQ | EVENT_TXSUSTO | EVENT_WLOCK | EVENT_POWEROFF |
    EVENT_RESET | EVENT_DESTROY, false, 0);

  return event;
}

static altmdm_state_t process_state_sleepwotx(uint32_t event,
  altmdm_state_t state)
{
  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RESET)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RESET);
      state = ALTMDM_STATE_FORCERST;
    }
  else if (event & EVENT_RXREQ)
    {
      state = ALTMDM_STATE_IDLEWOTX;
    }
  else if (event & EVENT_WLOCK)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_WLOCK);
      if (get_wlock_count() != 0)
        {
          state = ALTMDM_STATE_IDLEWOTX;
        }
    }
  else if (event & EVENT_TXSUSTO)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_TXSUSTO);
      state = ALTMDM_STATE_SLEEP;
    }

  return state;
}

/****************************************************************************
 * IDLE4RST state
 ****************************************************************************/

static int next_state_idle4rst(altmdm_state_t state)
{
  /* clear TX buffer */

  altmdm_set_spipkt_txbuffer(&g_altmdm_dev.tx_pkt, NULL, 0);

  g_altmdm_dev.lower->set_wakeup(true);

  return 0;
}

static uint32_t waitevt_state_idle4rst(void)
{
  enum version_phase_e vp;
  vp = get_vp();

  if (vp == VP_UNKNOWN)
    {
      return 0;
    }
  else if (vp == VP_NOTV1)
    {
      return 0;
    }

  return altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_RXREQ | EVENT_POWEROFF | EVENT_RESET | EVENT_DESTROY, false, 0);
}

static altmdm_state_t process_state_idle4rst(uint32_t event,
  altmdm_state_t state)
{
  enum version_phase_e vp;
  vp = get_vp();

  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RESET)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RESET);
      state = ALTMDM_STATE_FORCERST;
    }
  else if (vp == VP_UNKNOWN)
    {
      state = ALTMDM_STATE_V1SET;
    }
  else if (vp == VP_NOTV1)
    {
      state = ALTMDM_STATE_V4SET;
    }
  else if (event & EVENT_RXREQ)
    {
      state = ALTMDM_STATE_HDRSREQ;
    }

  return state;
}

/****************************************************************************
 * IDLEWTO state
 ****************************************************************************/

static int next_state_idlewto(altmdm_state_t state)
{
  /* clear TX buffer */

  altmdm_set_spipkt_txbuffer(&g_altmdm_dev.tx_pkt, NULL, 0);

  g_altmdm_dev.lower->set_wakeup(true);

  return 0;
}

static uint32_t waitevt_state_idlewto(void)
{
  return altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_TXREQ | EVENT_RXREQ | EVENT_WLOCK |
    EVENT_POWEROFF | EVENT_RESET | EVENT_DESTROY,
    false, TIMEOUT_IDELEWTO_STATE);
}

static altmdm_state_t process_state_idlewto(uint32_t event,
  altmdm_state_t state)
{
  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RESET)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RESET);
      state = ALTMDM_STATE_FORCERST;
    }
  else if (event & EVENT_TXREQ)
    {
      state = ALTMDM_STATE_TXPREPARE;
    }
  else if (event & EVENT_RXREQ)
    {
      state = ALTMDM_STATE_HDRSREQ;
    }
  else if (event & EVENT_WLOCK)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_WLOCK);
      if (get_wlock_count() != 0)
        {
          state = ALTMDM_STATE_IDLEWOTO;
        }
    }
  else /* Time out case */
    {
      state = ALTMDM_STATE_SLEEPSET;
    }

  return state;
}

/****************************************************************************
 * IDLEWOTO state
 ****************************************************************************/

static int next_state_idlewoto(altmdm_state_t state)
{
  /* clear TX buffer */

  altmdm_set_spipkt_txbuffer(&g_altmdm_dev.tx_pkt, NULL, 0);

  g_altmdm_dev.lower->set_wakeup(true);

  return 0;
}

static uint32_t waitevt_state_idlewoto(void)
{
  return altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_TXREQ | EVENT_RXREQ | EVENT_WLOCK | EVENT_POWEROFF | EVENT_RESET |
    EVENT_DESTROY, false, 0);
}

static altmdm_state_t process_state_idlewoto(uint32_t event,
    altmdm_state_t state)
{
  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RESET)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RESET);
      state = ALTMDM_STATE_FORCERST;
    }
  else if (event & EVENT_TXREQ)
    {
      state = ALTMDM_STATE_TXPREPARE;
    }
  else if (event & EVENT_RXREQ)
    {
      state = ALTMDM_STATE_HDRSREQ;
    }
  else if (event & EVENT_WLOCK)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_WLOCK);
      if (get_wlock_count() == 0)
        {
          state = ALTMDM_STATE_IDLEWTO;
        }
    }

  return state;
}

/****************************************************************************
 * IDLEWOTX state
 ****************************************************************************/

static int next_state_idlewotx(altmdm_state_t state)
{
  /* clear TX buffer */

  altmdm_set_spipkt_txbuffer(&g_altmdm_dev.tx_pkt, NULL, 0);

  g_altmdm_dev.lower->set_wakeup(true);

  return 0;
}

static uint32_t waitevt_state_idlewotx(void)
{
  return altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_RXREQ | EVENT_TXSUSTO | EVENT_POWEROFF | EVENT_RESET |
    EVENT_DESTROY, false, 0);
}

static altmdm_state_t process_state_idlewotx(uint32_t event,
    altmdm_state_t state)
{
  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RESET)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RESET);
      state = ALTMDM_STATE_FORCERST;
    }
  else if (event & EVENT_RXREQ)
    {
      state = ALTMDM_STATE_HDRSREQ;
    }
  else if (event & EVENT_TXSUSTO)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_TXSUSTO);

      if (!is_vp_valid())
        {
          state = ALTMDM_STATE_IDLE4RST;
        }
      else if (get_wlock_count() == 0)
        {
          state = ALTMDM_STATE_IDLEWTO;
        }
      else
        {
          state = ALTMDM_STATE_IDLEWOTO;
        }
    }

  return state;
}

/****************************************************************************
 * V1SET state
 ****************************************************************************/

static altmdm_state_t process_state_v1set(uint32_t event,
  altmdm_state_t state)
{
  int len;
  void *pkt;

  pkt = altcom_make_poweron_cmd_v1(&len);
  set_vp(VP_TRYV1);
  altmdm_set_spipkt_txbuffer(&g_altmdm_dev.tx_pkt, pkt, len);

  return ALTMDM_STATE_TXREQ;
}

/****************************************************************************
 * V4SET state
 ****************************************************************************/

static altmdm_state_t process_state_v4set(uint32_t event,
    altmdm_state_t state)
{
  int len;
  void *pkt;

  pkt = altcom_make_poweron_cmd_v4(&len);
  set_vp(VP_TRYV4);
  altmdm_set_spipkt_txbuffer(&g_altmdm_dev.tx_pkt, pkt, len);

  return ALTMDM_STATE_TXREQ;
}

/****************************************************************************
 * SLEEPSET state
 ****************************************************************************/

static altmdm_state_t process_state_sleepset(uint32_t event,
  altmdm_state_t state)
{
  altmdm_set_sleeppkt(&g_altmdm_dev.tx_pkt);

  return ALTMDM_STATE_TXREQ;
}

/****************************************************************************
 * TXPREPARE state
 ****************************************************************************/

static altmdm_state_t process_state_txprepare(uint32_t event,
  altmdm_state_t state)
{
  void *buff;
  int len;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_txreq);
  buff = g_altmdm_dev.txreq_buff;
  len = g_altmdm_dev.txreq_size;
  nxsem_post(&g_altmdm_dev.lock_txreq);

  altmdm_set_spipkt_txbuffer(&g_altmdm_dev.tx_pkt, buff, len);

  return ALTMDM_STATE_TXREQ;
}

/****************************************************************************
 * TXREQ state
 ****************************************************************************/

static altmdm_state_t process_state_txreq(uint32_t event,
  altmdm_state_t state)
{
  g_altmdm_dev.lower->set_mready(true);

  return ALTMDM_STATE_HDRSREQ;
}

/****************************************************************************
 * HDRSREQ state
 ****************************************************************************/

static uint32_t waitevt_state_hdrsreq(void)
{
  return altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_RXREQ | EVENT_POWEROFF | EVENT_DESTROY,
    false, TIMEOUT_HDR_TRX_STATE);
}

static altmdm_state_t process_state_hdrsreq(uint32_t event,
  altmdm_state_t state)
{
  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RXREQ)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RXREQ);
      state = ALTMDM_STATE_HDRTRX;
    }
  else /* Time out case */
    {
      if (is_vp_noreset())
        {
          state = ALTMDM_STATE_DECIDEDELAY;
        }
      else
        {
          m_err("[altmdm] Time out happened. Current State is %d\n", state);
          state = ALTMDM_STATE_FORCERST;
        }
    }

  return state;
}

/****************************************************************************
 * HDRTRX state
 ****************************************************************************/

static altmdm_state_t process_state_hdrtrx(uint32_t event,
  altmdm_state_t state)
{
  altmdm_do_hdr_transaction(g_altmdm_dev.spidev, g_altmdm_dev.lower,
      &g_altmdm_dev.tx_pkt, &g_altmdm_dev.rx_pkt);

  if (!altmdm_is_valid_spipkt_header(&g_altmdm_dev.rx_pkt))
    {
      if (is_vp_noreset())
        {
          state = ALTMDM_STATE_DECIDEDELAY;
        }
      else
        {
          m_err("[altmdm] Header error. Current State is %d\n", state);
          state = ALTMDM_STATE_FORCERST;
        }
    }
  else if (is_sleep_pkt(&g_altmdm_dev.tx_pkt))
    {
      state = ALTMDM_STATE_SLEEPPKT;
    }
  else if ((pkt_total_size(&g_altmdm_dev.tx_pkt) == 0) &&
      (pkt_total_size(&g_altmdm_dev.rx_pkt) == 0))
    {
      state = ALTMDM_STATE_DECIDEDELAY;
    }
  else
    {
      state = ALTMDM_STATE_BODYSREQ;
    }

  return state;
}

/****************************************************************************
 * SLEEPPKT state
 ****************************************************************************/

static altmdm_state_t process_state_sleeppkt(uint32_t event,
  altmdm_state_t state)
{
  altmdm_overwrite_body_size(&g_altmdm_dev.rx_pkt, 4);

  return ALTMDM_STATE_BODYSREQ;
}

/****************************************************************************
 * BODYSREQ state
 ****************************************************************************/

static uint32_t waitevt_state_bodysreq(void)
{
  return altmdm_event_wait(&g_altmdm_dev.event,
    EVENT_RXREQ | EVENT_POWEROFF | EVENT_DESTROY,
    false, TIMEOUT_BODY_TRX_STATE);
}

static altmdm_state_t process_state_bodysreq(uint32_t event,
    altmdm_state_t state)
{
  if (event & EVENT_DESTROY)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_DESTROY);
      state = ALTMDM_STATE_DESTORY;
    }
  else if (event & EVENT_POWEROFF)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_POWEROFF);
      state = ALTMDM_STATE_POWEROFF;
    }
  else if (event & EVENT_RXREQ)
    {
      altmdm_event_clear(&g_altmdm_dev.event, EVENT_RXREQ);
      state = ALTMDM_STATE_BODYTRX;
    }
  else /* Time out case */
    {
      if (is_vp_noreset())
        {
          state = ALTMDM_STATE_DECIDEDELAY;
        }
      else
        {
          m_err("[altmdm] Timeout happened. Current State is %d\n", state);
          state = ALTMDM_STATE_FORCERST;
        }
    }

  return state;
}

/****************************************************************************
 * BODYTRX state
 ****************************************************************************/

static altmdm_state_t process_state_bodytrx(uint32_t event,
  altmdm_state_t state)
{
  altmdm_do_body_transaction(g_altmdm_dev.spidev, g_altmdm_dev.lower,
    &g_altmdm_dev.tx_pkt, &g_altmdm_dev.rx_pkt);

  g_altmdm_dev.lower->set_mready(false);

  if (is_reset_pkt(&g_altmdm_dev.rx_pkt))
    {
      state = ALTMDM_STATE_GOTRST;
    }
  else if (is_sleep_pkt(&g_altmdm_dev.tx_pkt))
    {
      state = ALTMDM_STATE_GOTSLEEP;
    }
  else if (pkt_total_size(&g_altmdm_dev.rx_pkt) != 0)
    {
      state = ALTMDM_STATE_GOTRX;
    }
  else
    {
      state = ALTMDM_STATE_DECIDEDELAY;
    }

  if (has_sendrequest(&g_altmdm_dev.tx_pkt))
    {
      if (is_reset_pkt(&g_altmdm_dev.rx_pkt))
        {
          tx_done(TX_CANCEL);
        }
      else if (!is_buffer_full(&g_altmdm_dev.rx_pkt))
        {
          tx_done(TX_DONE);
        }
    }

  if (!is_buffer_full(&g_altmdm_dev.rx_pkt))
    {
      altmdm_timer_restart(g_altmdm_dev.txsus_timer, 0, 0);
    }

  return state;
}

/****************************************************************************
 * GOTRX state
 ****************************************************************************/

static altmdm_state_t process_state_gotrx(uint32_t event,
    altmdm_state_t state)
{
  enum version_phase_e vp;
  void *rcv_data;

  state = ALTMDM_STATE_DECIDEDELAY;

  if (is_vp_valid())
    {
      set_return_code(pkt_actual_size(&g_altmdm_dev.rx_pkt));
      state = ALTMDM_STATE_RETRECV;
    }
  else
    {
      rcv_data = get_pkt_buffer(&g_altmdm_dev.rx_pkt);
      vp = get_vp();
      if (vp == VP_TRYV1)
        {
          if (altcom_is_v1pkt_ok((struct altcom_cmdhdr_s *)rcv_data))
            {
              set_vp(VP_V1);
              set_return_code(ALTMDM_RETURN_RESET_V1);
              state = ALTMDM_STATE_RETRECV;
            }
          else
            {
              set_vp(VP_NOTV1);
            }
        }
      else if ((vp == VP_TRYV4)
          && altcom_is_v4pkt_ok((struct altcom_cmdhdr_s *)rcv_data))
        {
          set_vp(VP_V4);
          set_return_code(ALTMDM_RETURN_RESET_V4);
          state = ALTMDM_STATE_RETRECV;
        }
    }

  return state;
}

/****************************************************************************
 * GOTRST state
 ****************************************************************************/

static altmdm_state_t process_state_gotrst(uint32_t event,
  altmdm_state_t state)
{
  if (is_vp_noreset())
    {
      set_return_code(ALTMDM_RETURN_RESET_PKT);
      set_vp(VP_UNKNOWN);
      state = ALTMDM_STATE_RETRECV;
    }
  else
    {
      m_err("[altmdm] Reset pkt received. Current State is %d\n", state);
      state = ALTMDM_STATE_FORCERST;
    }

  return state;
}

/****************************************************************************
 * GOTSLEEP state
 ****************************************************************************/

static altmdm_state_t process_state_gotsleep(uint32_t event,
  altmdm_state_t state)
{
  if (altmdm_is_sleeppkt_ok(&g_altmdm_dev.rx_pkt))
    {
      state = ALTMDM_STATE_SLEEPING;
    }
  else
    {
      state = ALTMDM_STATE_DECIDEDELAY;
    }

  return state;
}

/****************************************************************************
 * BACKTOIDLE state
 ****************************************************************************/

static altmdm_state_t process_state_backtoidle(uint32_t event,
    altmdm_state_t state)
{
  if (is_buffer_full(&g_altmdm_dev.rx_pkt))
    {
      if (altmdm_timer_is_running(g_altmdm_dev.txsus_timer))
        {
          state = ALTMDM_STATE_IDLEWOTX;
        }
      else
        {
          state = ALTMDM_STATE_SETSUSTIMER;
        }
    }
  else if (!is_vp_valid())
    {
      state = ALTMDM_STATE_IDLE4RST;
    }
  else if (get_wlock_count() != 0)
    {
      state = ALTMDM_STATE_IDLEWOTO;
    }
  else
    {
      state = ALTMDM_STATE_IDLEWTO;
    }

  return state;
}

/****************************************************************************
 * RETRECV state
 ****************************************************************************/

static altmdm_state_t process_state_retrecv(uint32_t event,
  altmdm_state_t state)
{
  return ALTMDM_STATE_DECIDEDELAY;
}

/****************************************************************************
 * FORCERST state
 ****************************************************************************/

static altmdm_state_t process_state_forcerst(uint32_t event,
  altmdm_state_t state)
{
  force_reset();

  return ALTMDM_STATE_BACKTOIDLE;
}

/****************************************************************************
 * SLEEPING state
 ****************************************************************************/

static uint32_t waitevt_state_sleeping(void)
{
  up_udelay(TIMEOUT_NEXT_DELAY);

  return 0;
}

static altmdm_state_t process_state_sleeping(uint32_t event,
    altmdm_state_t state)
{
  if (is_buffer_full(&g_altmdm_dev.rx_pkt))
    {
      if (altmdm_timer_is_running(g_altmdm_dev.txsus_timer))
        {
          state = ALTMDM_STATE_SLEEPWOTX;
        }
      else
        {
          state = ALTMDM_STATE_SETSUSTIMERSLEEP;
        }
    }
  else
    {
      state = ALTMDM_STATE_SLEEP;
    }

  return state;
}

/****************************************************************************
 * DECIDEDELAY state
 ****************************************************************************/

static int next_state_decidedelay(altmdm_state_t state)
{
  return (state == ALTMDM_STATE_RETRECV);
}

static altmdm_state_t process_state_decidedelay(uint32_t event,
    altmdm_state_t state)
{
  if (has_sendrequest(&g_altmdm_dev.tx_pkt) ||
      is_sleep_pkt(&g_altmdm_dev.tx_pkt))
    {
      state = ALTMDM_STATE_DELAYNEXT;
    }
  else
    {
      state = ALTMDM_STATE_BACKTOIDLE;
    }

  return state;
}

/****************************************************************************
 * DELAYNEXT state
 ****************************************************************************/

static uint32_t waitevt_state_delaynext(void)
{
  up_udelay(TIMEOUT_NEXT_DELAY);

  return 0;
}

static altmdm_state_t process_state_delaynext(uint32_t event,
    altmdm_state_t state)
{
  state = ALTMDM_STATE_BACKTOIDLE;

  return state;
}

/****************************************************************************
 * SETSUSTIMER state
 ****************************************************************************/

static altmdm_state_t process_state_setsustimer(uint32_t event,
    altmdm_state_t state)
{
  altmdm_timer_restart(g_altmdm_dev.txsus_timer,
    TXSUS_TIMEOUT, 0);

  return ALTMDM_STATE_IDLEWOTX;
}

/****************************************************************************
 * SETSUSTIMERSLEEP state
 ****************************************************************************/

static altmdm_state_t process_state_setsustimersleep(uint32_t event,
    altmdm_state_t state)
{
  altmdm_timer_restart(g_altmdm_dev.txsus_timer,
    TXSUS_TIMEOUT, 0);

  return ALTMDM_STATE_SLEEPWOTX;
}

/****************************************************************************
 * DESTORY state
 ****************************************************************************/

static int next_state_destroy(altmdm_state_t state)
{
  next_state_poweroff(state);
  altmdm_event_clear(&g_altmdm_dev.event, (uint32_t)(-1));
  altmdm_timer_stop(g_altmdm_dev.txsus_timer);
  set_return_code(ALTMDM_RETURN_EXIT);

  return 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int altmdm_init(FAR struct spi_dev_s *spidev,
  FAR const struct alt1250_lower_s *lower)
{
  altmdm_event_init(&g_altmdm_dev.event);
  altmdm_event_init(&g_altmdm_dev.txdone_event);

  nxsem_init(&g_altmdm_dev.lock_evt, 0, 1);
  nxsem_init(&g_altmdm_dev.lock_vp, 0, 1);
  nxsem_init(&g_altmdm_dev.lock_counter, 0, 1);
  nxsem_init(&g_altmdm_dev.lock_txreq, 0, 1);

  altmdm_spipkt_init(&g_altmdm_dev.tx_pkt);
  altmdm_spipkt_init(&g_altmdm_dev.rx_pkt);

  g_altmdm_dev.spidev = spidev;
  g_altmdm_dev.lower = lower;

  g_altmdm_dev.txsus_timer = altmdm_timer_start(0, 0, txsustimer_handler,
    &g_altmdm_dev);
  g_altmdm_dev.wcounter = 0;
  g_altmdm_dev.rx_retcode = 0;
  g_altmdm_dev.txreq_buff = NULL;
  g_altmdm_dev.txreq_size = 0;

  g_altmdm_dev.current_state = ALTMDM_STATE_POWEROFF;
  g_altmdm_dev.vp = VP_NO_RESET;

  lower->irqattach(sready_isr);

  next_state_poweroff(g_altmdm_dev.current_state);

  return 0;
}

int altmdm_fin(void)
{
  int ret = OK;
  uint32_t evt;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_evt);

  evt = altmdm_event_refer(&g_altmdm_dev.event);

  /* Is already accepted DESTROY request? */

  if (evt & EVENT_DESTROY)
    {
      ret = -EALREADY;
    }

  /* Is in DESTROY state? */

  if (g_altmdm_dev.current_state == ALTMDM_STATE_DESTORY)
    {
      ret = -EALREADY;
    }

  if (ret == OK)
    {
      altmdm_event_set(&g_altmdm_dev.event, EVENT_DESTROY);
    }

  nxsem_post(&g_altmdm_dev.lock_evt);

  return ret;
}

int altmdm_poweron(void)
{
  int ret = OK;
  uint32_t evt;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_evt);

  evt = altmdm_event_refer(&g_altmdm_dev.event);

  /* Is already accepted DESTROY request? */

  if (evt & EVENT_DESTROY)
    {
      ret = -EOPNOTSUPP;
    }

  /* Is already accepted EVENT_POWERON request? */

  else if (evt & EVENT_POWERON)
    {
      ret = -EALREADY;
    }

  /* Is in POWERON state? */

  if ((g_altmdm_dev.current_state != ALTMDM_STATE_POWEROFF) &&
      (g_altmdm_dev.current_state != ALTMDM_STATE_DESTORY))
    {
      if (!(evt & EVENT_POWEROFF))
        {
          ret = -EALREADY;
        }
    }

  if (ret == OK)
    {
      altmdm_event_set(&g_altmdm_dev.event, EVENT_POWERON);
    }

  nxsem_post(&g_altmdm_dev.lock_evt);

  return ret;
}

int altmdm_poweroff(void)
{
  int ret = OK;
  uint32_t evt;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_evt);

  evt = altmdm_event_refer(&g_altmdm_dev.event);

  /* Is already accepted DESTROY request? */

  if (evt & EVENT_DESTROY)
    {
      ret = -EOPNOTSUPP;
    }

  /* Is already accepted POWEROFF request? */

  else if (evt & EVENT_POWEROFF)
    {
      ret = -EALREADY;
    }

  /* Is in POWEROFF state? */

  if ((g_altmdm_dev.current_state == ALTMDM_STATE_POWEROFF) ||
      (g_altmdm_dev.current_state == ALTMDM_STATE_DESTORY))
    {
      if (!(evt & EVENT_POWERON))
        {
          ret = -EALREADY;
        }
    }

  if (ret == OK)
    {
      altmdm_event_set(&g_altmdm_dev.event, EVENT_POWEROFF);
    }

  nxsem_post(&g_altmdm_dev.lock_evt);

  return ret;
}

int altmdm_reset(void)
{
  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_evt);

  altmdm_event_set(&g_altmdm_dev.event, EVENT_RESET);

  nxsem_post(&g_altmdm_dev.lock_evt);

  return 0;
}

int altmdm_read(FAR uint8_t *buff, int sz)
{
  int is_exit = 0;
  uint32_t event = 0;
  altmdm_state_t next_state = ALTMDM_STATE_POWEROFF;

  altmdm_set_spipkt_rxbuffer(&g_altmdm_dev.rx_pkt, buff, sz);

  dump_current_all_status(&g_altmdm_dev, event, next_state, is_exit);

  /* State machine loop */

  while (!is_exit)
    {
      /* Waiting events */

      event = g_state_func[g_altmdm_dev.current_state].wait_event();

      /* altmdm_fin(), altmdm_poweron(), and altmdm_poweroff() check
       * the event flag and decide whether or not to send the event.
       * Since this event flag checking process and the process to clear
       * the event flag will cause resource conflicts, so added this function
       * for exclusivity.
       */

      nxsem_wait_uninterruptible(&g_altmdm_dev.lock_evt);

      /* Process on the state */

      next_state = g_state_func[g_altmdm_dev.current_state].process_state(
          event, g_altmdm_dev.current_state);

      /* Going to next state */

      if (next_state != g_altmdm_dev.current_state)
        {
          is_exit = g_state_func[next_state].goto_next(
            g_altmdm_dev.current_state);
        }

      dump_current_all_status(&g_altmdm_dev, event, next_state, is_exit);

      g_altmdm_dev.current_state = next_state;

      nxsem_post(&g_altmdm_dev.lock_evt);
    }

  return g_altmdm_dev.rx_retcode;
}

int altmdm_write(FAR uint8_t *buff, int sz)
{
  bool should_wait = true;
  uint32_t ret;

  sz = (sz > get_spipayload_maxsize()) ? get_spipayload_maxsize() : sz;

  do
    {
      if (!is_vp_valid())
        {
          return ALTMDM_RETURN_NOTREADY;
        }

      nxsem_wait_uninterruptible(&g_altmdm_dev.lock_txreq);
      if (g_altmdm_dev.txreq_buff == NULL)
        {
          g_altmdm_dev.txreq_buff = buff;
          g_altmdm_dev.txreq_size = sz;
          should_wait = false;
          altmdm_event_set(&g_altmdm_dev.event, EVENT_TXREQ);
          altmdm_event_clear(&g_altmdm_dev.txdone_event, (uint32_t)-1);
        }

      nxsem_post(&g_altmdm_dev.lock_txreq);

      if (should_wait)
        {
          ret = altmdm_event_wait(&g_altmdm_dev.txdone_event,
              TX_DONE | TX_CANCEL, true, 0);
          if (ret & TX_CANCEL)
            {
              return ALTMDM_RETURN_CANCELED;
            }
        }
    }
  while (should_wait);

  ret = altmdm_event_wait(&g_altmdm_dev.txdone_event,
      TX_DONE | TX_CANCEL, true, 0);

  if (ret & TX_DONE)
    {
      return sz;
    }
  else
    {
      return ALTMDM_RETURN_CANCELED;
    }
}

int altmdm_take_wlock(void)
{
  int cnt;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_counter);
  cnt = ++g_altmdm_dev.wcounter;
  nxsem_post(&g_altmdm_dev.lock_counter);

  altmdm_event_set(&g_altmdm_dev.event, EVENT_WLOCK);

  return cnt;
}

int altmdm_give_wlock(void)
{
  int cnt;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_counter);
  if (g_altmdm_dev.wcounter != 0)
    {
      g_altmdm_dev.wcounter--;
    }

  cnt = g_altmdm_dev.wcounter;
  nxsem_post(&g_altmdm_dev.lock_counter);

  if (cnt == 0)
    {
      altmdm_event_set(&g_altmdm_dev.event, EVENT_WLOCK);
    }

  return cnt;
}

uint32_t altmdm_get_reset_reason(void)
{
  return get_reset_reason();
}

uint8_t altmdm_get_protoversion(void)
{
  enum version_phase_e vp;

  nxsem_wait_uninterruptible(&g_altmdm_dev.lock_vp);
  vp = g_altmdm_dev.vp;
  vp = ((vp == VP_V1) || (vp == VP_V4)) ? vp : ALTCOM_VERX;
  nxsem_post(&g_altmdm_dev.lock_vp);

  return (uint8_t)vp;
}

#endif
