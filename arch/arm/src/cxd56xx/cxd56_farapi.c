/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_farapi.c
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

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arch/chip/pm.h>

#include "arm_internal.h"
#include "chip.h"
#include "cxd56_icc.h"
#include "cxd56_config.h"
#include "cxd56_farapistub.h"
#include "hardware/cxd5602_backupmem.h"

int fw_pm_wakeupcpu(int cpuid);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPS_CPU_ID (1)

#ifndef CONFIG_CXD56_FARAPI_VERSION_CHECK
#define CONFIG_CXD56_FARAPI_VERSION_CHECK 1
#endif

#define CPU_ID (CXD56_CPU_BASE + 0x40)

/****************************************************************************
 * Private Type
 ****************************************************************************/

typedef int farapicallback(void *data);

struct modulelist_s
{
  void   *mod;
  int     cpuno;
  void   *reserved;
  int16_t mbxid;
};

struct apimsg_s
{
  int     id;
  void   *arg;
  int16_t mbxid;
  int16_t flagid;
  int     flagbitno;
};

struct farcallback_s
{
  int (*cbfunc)(void *);        /* pointer to callback function */
  void *data;                   /* callback data */
  int   flagbitno;              /* callback eventflag bitno */
};

struct farmsghead_s
{
  struct farmsghead_s *next;
};

struct farmsg_s
{
  struct farmsghead_s head;     /* message head */
  int cpuid;                    /* CPU ID of API caller */
  int modid;                    /* module table offset */
  union
  {
    struct apimsg_s api;
    struct farcallback_s cb;
  } u;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern char _image_modlist_base[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_farwait;
static sem_t g_farlock;
static struct pm_cpu_wakelock_s g_wlock =
{
  .count = 0,
  .info  = PM_CPUWAKELOCK_TAG('R', 'M', 0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int farapi_semtake(sem_t *id)
{
  return nxsem_wait_uninterruptible(id);
}

#ifdef CONFIG_CXD56_FARAPI_DEBUG
static void dump_farapi_message(struct farmsg_s *msg)
{
  _info("cpuid : %d\n",    msg->cpuid);
  _info("modid : %d\n",    msg->modid);
  _info("id    : %d\n",    msg->u.api.id);
  _info("arg   : %08x\n",  msg->u.api.arg);
  _info("mbxid : %d\n",    msg->u.api.mbxid);
  _info("flagid: %d\n",    msg->u.api.flagid);
  _info("flagbitno: %d\n", msg->u.api.flagbitno);
}
#  define fainfo(x, ...) _info(x, ##__VA_ARGS__)
#else
#  define dump_farapi_message(x)
#  define fainfo(x, ...)
#endif

static int cxd56_sendmsg(int cpuid, int protoid, int msgtype, uint16_t pdata,
                         uint32_t data)
{
  iccmsg_t msg;

  msg.cpuid = cpuid;
  msg.msgid = msgtype << 4;
  msg.protodata = pdata;
  msg.data = data;
  return cxd56_iccsend(protoid, &msg, 0);
}

static int cxd56_farapidonehandler(int cpuid, int protoid,
                                   uint32_t pdata, uint32_t data,
                                   void *userdata)
{
  /* Receive event flag message as Far API done.
   * We need only far API done event.
   */

  if (protoid == CXD56_PROTO_FLG && (pdata & 0xf) == 0x7)
    {
      /* Send event flag response */

      cxd56_sendmsg(cpuid, CXD56_PROTO_FLG, 5, pdata & 0xff00, 0);
      nxsem_post(&g_farwait);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

used_code
void farapi_main(int id, void *arg, struct modulelist_s *mlist)
{
  struct farmsg_s msg;
  struct apimsg_s *api;
  int ret;

#ifdef CONFIG_SMP
  int cpu = up_cpu_index();
  static cpu_set_t cpuset0;

  if (0 != cpu)
    {
      /* Save the current cpuset */

      sched_getaffinity(getpid(), sizeof(cpu_set_t), &cpuset0);

      /* Assign the current task to cpu0 */

      cpu_set_t cpuset1;
      CPU_ZERO(&cpuset1);
      CPU_SET(0, &cpuset1);
      sched_setaffinity(getpid(), sizeof(cpu_set_t), &cpuset1);

      /* NOTE: a workaround to finish rescheduling */

      nxsig_usleep(10 * 1000);
    }
#endif

#ifdef CONFIG_CXD56_GNSS_HOT_SLEEP
  uint32_t gnscken;

  if (mlist->cpuno == GPS_CPU_ID)
    {
      gnscken = getreg32(CXD56_TOPREG_GNSDSP_CKEN);
      if (((gnscken & GNSDSP_CKEN_P1) != GNSDSP_CKEN_P1) &&
          ((gnscken & GNSDSP_CKEN_COP) != GNSDSP_CKEN_COP))
        {
          fw_pm_wakeupcpu(GPS_CPU_ID);
        }
    }
#endif

  farapi_semtake(&g_farlock);

  api = &msg.u.api;

  msg.cpuid      = getreg32(CPU_ID);
  msg.modid      = mlist - (struct modulelist_s *)&_image_modlist_base;

  api->id        = id;
  api->arg       = arg;
  api->mbxid     = mlist->mbxid;
  api->flagid    = (msg.cpuid + 1) << 8 | 7; /* 7 is a magic. not zero */
  api->flagbitno = 0;                        /* ignore */

  dump_farapi_message(&msg);

  /* Send request by mailbox protocol */

  ret = cxd56_sendmsg(mlist->cpuno, CXD56_PROTO_MBX, 4, 1 << 8 | 1,
                      (uint32_t)(uintptr_t)&msg);
  if (ret)
    {
      _err("Failed far api push\n");
      goto err;
    }

  /* Suppress hot sleep until Far API done */

  up_pm_acquire_wakelock(&g_wlock);

  /* Wait event flag message as Far API done */

  farapi_semtake(&g_farwait);

  /* Permit hot sleep with Far API done */

  up_pm_release_wakelock(&g_wlock);

  dump_farapi_message(&msg);

err:
  nxsem_post(&g_farlock);

#ifdef CONFIG_SMP
  if (0 != cpu)
    {
      /* Restore the cpu affinity */

      sched_setaffinity(getpid(), sizeof(cpu_set_t), &cpuset0);

      /* NOTE: a workaround to finish rescheduling */

      nxsig_usleep(10 * 1000);
    }
#endif
}

void cxd56_farapiinitialize(void)
{
#ifdef CONFIG_CXD56_FARAPI_VERSION_CHECK
  if (GET_SYSFW_VERSION_BUILD() < FARAPISTUB_VERSION)
    {
      _alert("Mismatched version: loader(%d) != Self(%d)\n",
             GET_SYSFW_VERSION_BUILD(), FARAPISTUB_VERSION);
      _alert("Please update loader and gnssfw firmwares!!\n");
#  ifdef CONFIG_CXD56_FARAPI_VERSION_FAILED_PANIC
      PANIC();
#  endif
    }

#endif
  nxsem_init(&g_farlock, 0, 1);
  nxsem_init(&g_farwait, 0, 0);
  nxsem_set_protocol(&g_farwait, SEM_PRIO_NONE);

  cxd56_iccinit(CXD56_PROTO_MBX);
  cxd56_iccinit(CXD56_PROTO_FLG);

  /* Setup CPU FIFO interrupt for SYS and GNSS */

  cxd56_iccregisterhandler(CXD56_PROTO_MBX, cxd56_farapidonehandler, NULL);
  cxd56_iccregisterhandler(CXD56_PROTO_FLG, cxd56_farapidonehandler, NULL);
}
