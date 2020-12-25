/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpu1signal.c
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
#include <stdlib.h>
#include <errno.h>
#include <sched.h>
#include <pthread.h>
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
  FAR void *             data;
};

struct cxd56cpu1_info_s
{
  pthread_t              workertid;
  int                    ndev;
  struct cxd56_sigtype_s sigtype[CXD56_CPU1_DATA_TYPE_MAX];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56cpu1_info_s g_cpu1_info =
  {
    0
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *cxd56cpu1_worker(FAR void *arg)
{
  struct cxd56cpu1_info_s *priv = (struct cxd56cpu1_info_s *)arg;
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

  return arg;
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

int cxd56_cpu1siginit(uint8_t sigtype, FAR void *data)
{
  struct cxd56cpu1_info_s *priv = &g_cpu1_info;
  pthread_attr_t           tattr;
  struct sched_param       param;
  pthread_t                tid;
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

  pthread_attr_init(&tattr);
  tattr.stacksize      = CONFIG_CXD56CPU1_WORKER_STACKSIZE;
  param.sched_priority = CONFIG_CXD56CPU1_WORKER_THREAD_PRIORITY;
  pthread_attr_setschedparam(&tattr, &param);

  ret = pthread_create(&tid, &tattr, cxd56cpu1_worker,
                       (pthread_addr_t)priv);
  if (ret != 0)
    {
      cxd56_iccuninitmsg(CXD56CPU1_CPUID);
      ret = -ret; /* pthread_create does not modify errno. */
      goto err0;
    }

  priv->workertid = tid;

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
  pthread_t                tid;
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

  tid             = priv->workertid;
  priv->workertid = 0;

  sched_unlock();

  pthread_cancel(tid);
  pthread_join(tid, NULL);

  cxd56_iccuninit(CXD56CPU1_CPUID);

  return 0;
}
