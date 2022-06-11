/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpu1signal.c
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
#include <stdlib.h>
#include <errno.h>
#include <sched.h>
#include <nuttx/sched.h>
#include <nuttx/kthread.h>
#include <debug.h>

#include "cxd56_icc.h"
#include "cxd56_cpu1signal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56CPU1_WORKER_STACKSIZE
#  define CONFIG_CXD56CPU1_WORKER_STACKSIZE 1024
#endif

#ifndef CONFIG_CXD56CPU1_WORKER_THREAD_PRIORITY
#  define CONFIG_CXD56CPU1_WORKER_THREAD_PRIORITY (SCHED_PRIORITY_MAX)
#endif

#define CXD56CPU1_CPUID          1

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct cxd56_sigtype_s
{
  int                    use;
  cxd56_cpu1sighandler_t handler;
  void                  *data;
};

struct cxd56cpu1_info_s
{
  pid_t                  workerpid;
  int                    ndev;
  struct cxd56_sigtype_s sigtype[CXD56_CPU1_DATA_TYPE_MAX];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56cpu1_info_s g_cpu1_info =
  {
    INVALID_PROCESS_ID,
    0,
    {
      0
    }
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cxd56cpu1_worker(int argc, char *argv[])
{
  struct cxd56cpu1_info_s *priv = &g_cpu1_info;
  iccmsg_t                 msg;
  uint8_t                  sigtype;
  int                      ret;

  msg.cpuid = CXD56CPU1_CPUID;

  while (1)
    {
      ret = cxd56_iccrecvmsg(&msg, 0);
      if (ret < 0)
        {
          continue;
        }

      sigtype = (uint8_t)CXD56_CPU1_GET_DEV(msg.data);
      if (sigtype >= CXD56_CPU1_DATA_TYPE_MAX)
        {
          _info("Caught invalid sigtype %d.\n", sigtype);
          continue;
        }

      if (priv->sigtype[sigtype].handler)
        {
          priv->sigtype[sigtype].handler(msg.data,
                          priv->sigtype[sigtype].data);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_cpu1sigsend(uint8_t sigtype, uint32_t data)
{
  iccmsg_t                 msg;

  msg.cpuid = CXD56CPU1_CPUID;
  msg.msgid = sigtype;
  msg.data  = data;

  return cxd56_iccsend(CXD56_PROTO_GNSS, &msg, 0);
}

void cxd56_cpu1sigregisterhandler(uint8_t sigtype,
                       cxd56_cpu1sighandler_t handler)
{
  struct cxd56cpu1_info_s *priv = &g_cpu1_info;

  if (sigtype >= CXD56_CPU1_DATA_TYPE_MAX)
    {
      return;
    }

  priv->sigtype[sigtype].handler = handler;
}

void cxd56_cpu1sigunregisterhandler(uint8_t sigtype)
{
  struct cxd56cpu1_info_s *priv = &g_cpu1_info;

  if (sigtype >= CXD56_CPU1_DATA_TYPE_MAX)
    {
      return;
    }

  priv->sigtype[sigtype].handler = NULL;
}

int cxd56_cpu1siginit(uint8_t sigtype, void *data)
{
  struct cxd56cpu1_info_s *priv = &g_cpu1_info;
  int                      pid;
  int                      ret;

  if (sigtype >= CXD56_CPU1_DATA_TYPE_MAX)
    {
      return -ENODEV;
    }

  sched_lock();

  if (priv->sigtype[sigtype].use)
    {
      ret = -EBUSY;
      goto err1;
    }

  priv->sigtype[sigtype].use  = true;
  priv->sigtype[sigtype].data = data;

  if (priv->ndev > 0)
    {
      ret = OK;
      goto err1;
    }

  priv->ndev++;

  sched_unlock();

  cxd56_iccinit(CXD56_PROTO_GNSS);

  ret = cxd56_iccinitmsg(CXD56CPU1_CPUID);
  if (ret < 0)
    {
      _err("Failed to initialize ICC for GPS CPU: %d\n", ret);
      goto err0;
    }

  pid = kthread_create("gnss_receiver",
                       CONFIG_CXD56CPU1_WORKER_THREAD_PRIORITY,
                       CONFIG_CXD56CPU1_WORKER_STACKSIZE, cxd56cpu1_worker,
                       NULL);

  if (pid < 0)
    {
      cxd56_iccuninitmsg(CXD56CPU1_CPUID);
      ret = -errno;
      goto err0;
    }

  priv->workerpid = pid;

  return ret;

err0:
  priv->sigtype[sigtype].use  = false;
  priv->sigtype[sigtype].data = NULL;
  return ret;

err1:
  sched_unlock();
  return ret;
}

int cxd56_cpu1siguninit(uint8_t sigtype)
{
  struct cxd56cpu1_info_s *priv = &g_cpu1_info;
  pid_t                    pid;
  int                      ret;

  if (sigtype >= CXD56_CPU1_DATA_TYPE_MAX)
    {
      return -ENODEV;
    }

  sched_lock();

  if (!priv->sigtype[sigtype].use)
    {
      ret = -EBUSY;
      sched_unlock();
      return ret;
    }

  priv->ndev--;
  priv->sigtype[sigtype].use  = false;
  priv->sigtype[sigtype].data = NULL;

  if (priv->ndev > 0)
    {
      ret = OK;
      return ret;
    }

  pid             = priv->workerpid;
  priv->workerpid = INVALID_PROCESS_ID;

  sched_unlock();

  ret = kthread_delete(pid);

  if (ret)
    {
      _err("Failed to delete GNSS receiver task. ret = %d\n", ret);
    }

  cxd56_iccuninit(CXD56CPU1_CPUID);

  return 0;
}
