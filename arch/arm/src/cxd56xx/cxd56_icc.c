/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_icc.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include "arm_internal.h"
#include "chip.h"
#include "cxd56_cpufifo.h"
#include "cxd56_icc.h"

#ifdef CONFIG_CXD56_ICC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_CPUFIFO_NBUFFERS
#  define NBUFFERS 4
#else
#  define NBUFFERS CONFIG_CXD56_CPUFIFO_NBUFFERS
#endif

#define NPROTOCOLS 16
#define GET_PROTOCOLID(w) (((w)[0] >> 24) & 0xf)

#define NCPUS      8

#define FLAG_TIMEOUT (1 << 0)

#define IS_SIGNAL(w) (((((w)[0]) >> 24) & 0xf) == CXD56_PROTO_SIG)

#ifdef CONFIG_CXD56_ICC_DEBUG
#  define iccerr(fmt, ...)  _err(fmt, ##__VA_ARGS__)
#  define iccinfo(fmt, ...) _info(fmt, ##__VA_ARGS__)
#else
#  define iccerr(fmt, ...)
#  define iccinfo(fmt, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct iccmsg_msg_s
{
  /* Little endian */

  uint16_t pdata;
  uint8_t msgid;
  uint8_t proto: 4;
  uint8_t cpuid: 4;

  uint32_t data;
};

struct iccreq_s
{
  sq_entry_t entry;

  union
  {
    uint32_t word[2];
    struct iccmsg_msg_s msg;
  };
};

struct iccdev_s
{
  union
  {
    cxd56_icchandler_t handler;
    cxd56_iccsighandler_t sighandler;
  } u;

  void *userdata;
  sem_t rxwait;
  struct wdog_s rxtimeout;

  int flags;

  /* for POSIX signal */

  int signo;
  pid_t pid;
  void *sigdata;

  struct sq_queue_s recvq;
  struct sq_queue_s freelist;
  struct iccreq_s pool[NBUFFERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int icc_sighandler(int cpuid, int protoid, uint32_t pdata,
                          uint32_t data, void *userdata);
static int icc_msghandler(int cpuid, int protoid, uint32_t pdata,
                          uint32_t data, void *userdata);
static int icc_irqhandler(int cpuid, uint32_t word[2]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct iccdev_s *g_protocol[NPROTOCOLS];
static struct iccdev_s *g_cpumsg[NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct iccdev_s *icc_getprotocol(int protoid)
{
  if (protoid < 0 || protoid >= NPROTOCOLS)
    {
      return NULL;
    }

  return g_protocol[protoid];
}

static struct iccdev_s *icc_getcpu(int cpuid)
{
  if (cpuid < 0 || cpuid >= NCPUS)
    {
      return NULL;
    }

  return g_cpumsg[cpuid];
}

static int icc_irqhandler(int cpuid, uint32_t word[2])
{
  struct iccdev_s *priv;
  struct iccreq_s *req;
  int protoid;

  protoid = GET_PROTOCOLID(word);
  priv = icc_getprotocol(protoid);
  if (!priv)
    {
      /* Nobody waits this message... */

      iccerr("nobody waits %08x %08x\n", word[0], word[1]);
      return OK;
    }

  /* If handler has been registered, then call it. */

  if (priv->u.handler)
    {
      int ret;

      ret = priv->u.handler(cpuid, protoid, word[0] & 0xffffff, word[1],
                            priv->userdata);
      if (ret == OK)
        {
          return OK;
        }
    }

  /* If MSG protocol, then replace priv to cpu ones. */

  if (protoid == CXD56_PROTO_MSG)
    {
      priv = icc_getcpu(cpuid);
      if (!priv)
        {
          iccerr("nobody waits from CPU %d\n", cpuid);
          return OK;
        }
    }

  req = (struct iccreq_s *)sq_remfirst(&priv->freelist);
  if (!req)
    {
      iccerr("Receive buffer is full.\n");
      PANIC();
    }

  req->word[0] = word[0];
  req->word[1] = word[1];

  sq_addlast((sq_entry_t *)req, &priv->recvq);

  nxsem_post(&priv->rxwait);

  /* If signal registered by cxd56_iccnotify(), then send POSIX signal to
   * process.
   */

  if (priv->pid != INVALID_PROCESS_ID)
    {
      union sigval value;

      value.sival_ptr = priv->sigdata;
      nxsig_queue(priv->pid, priv->signo, value);
    }

  return OK;
}

static int icc_sighandler(int cpuid, int protoid, uint32_t pdata,
                          uint32_t data, void *userdata)
{
  struct iccdev_s *priv = icc_getcpu(cpuid);

  if (!priv)
    {
      /* Nobody waits this message... */

      iccerr("nobody waits %08x %08x\n", word[0], word[1]);
      return OK;
    }

  iccinfo("Caught signal\n");

  if (priv->u.sighandler)
    {
      int8_t signo;
      uint16_t sigdata;

      signo   = (int8_t)((pdata >> 16) & 0xff);
      sigdata = pdata & 0xffff;

      iccinfo("Call signal handler with No %d.\n", signo);
      priv->u.sighandler(signo, sigdata, data, priv->userdata);
    }

  return OK;
}

static int icc_msghandler(int cpuid, int protoid, uint32_t pdata,
                          uint32_t data, void *userdata)
{
  /* Do nothing. This handler used for reserve MSG protocol handler.
   * This handler returns -1 to indicate not consumed the passed
   * message.
   */

  return -1;
}

static void icc_rxtimeout(wdparm_t arg)
{
  struct iccdev_s *priv = (struct iccdev_s *)arg;
  nxsem_post(&priv->rxwait);
}

static int icc_recv(struct iccdev_s *priv, iccmsg_t *msg, int32_t ms)
{
  struct iccreq_s *req;
  irqstate_t flags;
  int ret = OK;

  if (ms == -1)
    {
      /* Try to take the semaphore without waiging. */

      ret = nxsem_trywait(&priv->rxwait);
      if (ret < 0)
        {
          return ret;
        }
    }
  else if (ms == 0)
    {
      nxsem_wait_uninterruptible(&priv->rxwait);
    }
  else
    {
      int32_t timo;
      timo = ms * 1000 / CONFIG_USEC_PER_TICK;
      wd_start(&priv->rxtimeout, timo, icc_rxtimeout, (wdparm_t)priv);

      nxsem_wait_uninterruptible(&priv->rxwait);
      wd_cancel(&priv->rxtimeout);
    }

  flags = enter_critical_section();
  req   = (struct iccreq_s *)sq_remfirst(&priv->recvq);

  if (req)
    {
      msg->msgid = req->msg.msgid;
      msg->data  = req->msg.data;
      msg->cpuid = req->msg.cpuid;
      msg->protodata = req->msg.pdata;
      sq_addlast((sq_entry_t *)req, &priv->freelist);
    }
  else
    {
      ret = -ETIMEDOUT;
    }

  leave_critical_section(flags);

  return ret;
}

static struct iccdev_s *icc_devnew(void)
{
  struct iccdev_s *priv;
  int i;

  priv = (struct iccdev_s *)kmm_malloc(sizeof(struct iccdev_s));
  if (!priv)
    {
      return NULL;
    }

  memset(priv, 0, sizeof(struct iccdev_s));

  nxsem_init(&priv->rxwait, 0, 0);
  nxsem_set_protocol(&priv->rxwait, SEM_PRIO_NONE);

  /* Initialize receive queue and free list */

  sq_init(&priv->recvq);
  sq_init(&priv->freelist);

  for (i = 0; i < NBUFFERS; i++)
    {
      sq_addlast((sq_entry_t *)&priv->pool[i], &priv->freelist);
    }

  priv->pid = INVALID_PROCESS_ID;

  return priv;
}

static void icc_devfree(struct iccdev_s *priv)
{
  wd_cancel(&priv->rxtimeout);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_iccregisterhandler(int protoid, cxd56_icchandler_t handler,
                             void *data)
{
  struct iccdev_s *priv;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();
  priv = icc_getprotocol(protoid);
  if (priv)
    {
      priv->u.handler  = handler;
      priv->userdata = data;
    }
  else
    {
      ret = -EINVAL;
    }

  leave_critical_section(flags);

  return ret;
}

int cxd56_iccregistersighandler(int cpuid, cxd56_iccsighandler_t handler,
                                void *data)
{
  struct iccdev_s *priv;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();
  priv = icc_getcpu(cpuid);
  if (priv)
    {
      priv->u.sighandler  = handler;
      priv->userdata = data;
    }
  else
    {
      ret = -EINVAL;
    }

  leave_critical_section(flags);

  return ret;
}

int cxd56_iccsend(int protoid, iccmsg_t *msg, int32_t ms)
{
  struct iccdev_s *priv;
  struct iccreq_s req;

  if (!msg)
    {
      return -EINVAL;
    }

  priv = icc_getprotocol(protoid);
  if (!priv)
    {
      return -EINVAL;
    }

  req.msg.cpuid = msg->cpuid;
  req.msg.msgid = msg->msgid;
  req.msg.data  = msg->data;
  req.msg.pdata = msg->protodata;
  req.msg.proto = protoid;

  priv->flags = 0;

  return cxd56_cfpush(req.word);
}

int cxd56_iccsendmsg(iccmsg_t *msg, int32_t ms)
{
  return cxd56_iccsend(CXD56_PROTO_MSG, msg, ms);
}

int cxd56_iccrecv(int protoid, iccmsg_t *msg, int32_t ms)
{
  struct iccdev_s *priv;

  if (!msg)
    {
      return -EINVAL;
    }

  priv = icc_getprotocol(protoid);
  if (!priv)
    {
      return -EINVAL;
    }

  return icc_recv(priv, msg, ms);
}

int cxd56_iccrecvmsg(iccmsg_t *msg, int32_t ms)
{
  struct iccdev_s *priv;

  if (!msg)
    {
      return -EINVAL;
    }

  priv = icc_getcpu(msg->cpuid);
  if (!priv)
    {
      return -EINVAL;
    }

  return icc_recv(priv, msg, ms);
}

int cxd56_iccsignal(int8_t cpuid, int8_t signo, int16_t sigdata,
                    uint32_t data)
{
  struct iccreq_s req;

  if (cpuid <= 2 && cpuid >= 7)
    {
      return -EINVAL;
    }

  req.msg.cpuid = cpuid;
  req.msg.proto = CXD56_PROTO_SIG;
  req.msg.msgid = signo;
  req.msg.pdata = sigdata;
  req.msg.data  = data;

  return cxd56_cfpush(req.word);
}

int cxd56_iccnotify(int cpuid, int signo, void *sigdata)
{
  struct iccdev_s *priv;

  priv = icc_getcpu(cpuid);
  if (!priv)
    {
      return -ESRCH;
    }

  priv->pid     = getpid();
  priv->signo   = signo;
  priv->sigdata = sigdata;

  return OK;
}

int cxd56_iccinit(int protoid)
{
  struct iccdev_s *priv;

  if (protoid < 0 || protoid >= NPROTOCOLS)
    {
      return -EINVAL;
    }

  if (g_protocol[protoid])
    {
      return OK;
    }

  priv = icc_devnew();
  if (!priv)
    {
      return -ENOMEM;
    }

  g_protocol[protoid] = priv;

  return OK;
}

int cxd56_iccinitmsg(int cpuid)
{
  struct iccdev_s *priv;

  if (cpuid < 0 || cpuid >= NCPUS)
    {
      return -EINVAL;
    }

  if (g_cpumsg[cpuid])
    {
      return OK;
    }

  priv = icc_devnew();
  if (!priv)
    {
      return -ENOMEM;
    }

  g_cpumsg[cpuid] = priv;

  return OK;
}

void cxd56_iccuninit(int protoid)
{
  struct iccdev_s *priv;
  irqstate_t flags;

  if (protoid < 0 || protoid >= NPROTOCOLS)
    {
      return;
    }

  flags = enter_critical_section();
  priv  = g_protocol[protoid];
  if (priv)
    {
      icc_devfree(priv);
      g_protocol[protoid] = NULL;
    }

  leave_critical_section(flags);
}

void cxd56_iccuninitmsg(int cpuid)
{
  struct iccdev_s *priv;
  irqstate_t flags;

  if (cpuid < 0 || cpuid >= NCPUS)
    {
      return;
    }

  flags = enter_critical_section();
  priv  = g_cpumsg[cpuid];
  if (priv)
    {
      icc_devfree(priv);
      g_cpumsg[cpuid] = NULL;
    }

  leave_critical_section(flags);
}

void cxd56_iccinitialize(void)
{
  int i;

  for (i = 0; i < NPROTOCOLS; i++)
    {
      g_protocol[i] = NULL;
    }

  for (i = 0; i < NCPUS; i++)
    {
      g_cpumsg[i] = NULL;
    }

  /* Protocol MSG and SIG is special, reserved by ICC driver. */

  cxd56_iccinit(CXD56_PROTO_MSG);
  cxd56_iccregisterhandler(CXD56_PROTO_MSG, icc_msghandler, NULL);
  cxd56_iccinit(CXD56_PROTO_SIG);
  cxd56_iccregisterhandler(CXD56_PROTO_SIG, icc_sighandler, NULL);

  cxd56_cfregrxhandler(icc_irqhandler);
}

#endif
