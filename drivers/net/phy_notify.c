/****************************************************************************
 * drivers/net/phy_notify.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/* Force verbose debug on in this file only to support unit-level testing. */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  undef  CONFIG_DEBUG_VERBOSE
#  define CONFIG_DEBUG_VERBOSE 1
#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <signal.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/net/phy.h>

#ifdef CONFIG_ARCH_PHY_INTERRUPT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The current design artificially limits the number of notification client.
 * This is an arbitrary limit.  If you exceed it, simply adjust the affected
 * areas.
 */

#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 4
#  warning Fix me!! Support currently limited to 4 clients
#  undef  CONFIG_PHY_NOTIFICATION_NCLIENTS
#  define CONFIG_PHY_NOTIFICATION_NCLIENTS 4
#endif

/* Debug ********************************************************************/
/* Extra, in-depth debug output that is only available if
 * CONFIG_NETDEV_PHY_DEBUG us defined.
 */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  define phydbg    dbg
#  define phylldbg  lldbg
#else
#  define phydbg(x...)
#  define phylldbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This describes the state of one notification.  There may be up to
 * CONFIG_PHY_NOTIFICATION_NCLIENTS such notifications active simultaneously.
 *
 * There
 */

struct phy_notify_s
{
  bool assigned;
  uint8_t signo;
  uint8_t index;
#ifdef CONFIG_NETDEV_MULTINIC
  char intf[CONFIG_PHY_NOTIFICATION_MAXINTFLEN+1];
#endif
  pid_t pid;
  FAR void *arg;
  phy_enable_t enable;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int phy_handler(FAR struct phy_notify_s *client);
static int phy_handler_0(int irq, FAR void *context);
#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 1
static int phy_handler_1(int irq, FAR void *context);
#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 2
static int phy_handler_2(int irq, FAR void *context);
#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 3
static int phy_handler_3(int irq, FAR void *context);
#endif
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Serializes access to the g_notify_clients array */

static sem_t g_notify_clients_sem = SEM_INITIALIZER(1);

/* This is a array the hold information for each PHY notification client */

static struct phy_notify_s g_notify_clients[CONFIG_PHY_NOTIFICATION_NCLIENTS];

/* Handler addresses accessed with the same index as g_notify_clients[] */

static const xcpt_t g_notify_handler[CONFIG_PHY_NOTIFICATION_NCLIENTS] =
{
    phy_handler_0
#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 1
    , phy_handler_1
#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 2
    , phy_handler_2
#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 3
    , phy_handler_3
#endif
#endif
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_semtake
 ****************************************************************************/

static void phy_semtake(void)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&g_notify_clients_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(errno == EINTR);
    }
}

#define phy_semgive() sem_post(&g_notify_clients_sem);

/****************************************************************************
 * Name: phy_find_unassigned
 ****************************************************************************/

static FAR struct phy_notify_s *phy_find_unassigned(void)
{
  FAR struct phy_notify_s *client;
  int i;

  phy_semtake();
  for (i = 0; i < CONFIG_PHY_NOTIFICATION_NCLIENTS; i++)
    {
      client = &g_notify_clients[i];
      if (!client->assigned)
        {
          /* Assign and re-initialized the entry */

          client->assigned = true;
          client->signo    = 0;
          client->index    = i;
#ifdef CONFIG_NETDEV_MULTINIC
          client->intf  = '\0';
#endif
          client->pid      = -1;
          client->arg      = NULL;
          client->enable   = NULL;

          /* Return the client entry assigned to the caller */

          phy_semgive();
          phydbg("Returning client %d\n", i);
          return client;
        }
    }

  /* Ooops... too many */

  ndbg("ERROR: No free client entries\n");
  phy_semgive();
  return NULL;
}

/****************************************************************************
 * Name: phy_find_assigned
 ****************************************************************************/

static FAR struct phy_notify_s *phy_find_assigned(FAR const char *intf,
                                                  pid_t pid)
{
  FAR struct phy_notify_s *client;
  int i;

  phy_semtake();
  for (i = 0; i < CONFIG_PHY_NOTIFICATION_NCLIENTS; i++)
    {
      client = &g_notify_clients[i];
      if (client->assigned && client->pid == pid
#ifdef CONFIG_NETDEV_MULTINIC
          && strncmp(client->intf, intf, CONFIG_PHY_NOTIFICATION_MAXINTFLEN) == 0
#endif
          )
        {
          /* Return the matching client entry to the caller */

          phy_semgive();
          phydbg("Returning client %d\n", i);
          return client;
        }
    }

  /* Ooops... not found */

  phy_semgive();
  return NULL;
}

/****************************************************************************
 * Name: phy_handler
 ****************************************************************************/

static int phy_handler(FAR struct phy_notify_s *client)
{
#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
#endif
  int ret;

  DEBUGASSERT(client && client->assigned && client->enable);
  phylldbg("Entry client %d, signalling PID=%d with signal %d\n",
           client->index, client->pid, client->signo);

  /* Disable further interrupts */

  client->enable(false);

  /* Signal the client that the PHY has something interesting to say to us */

  #ifdef CONFIG_CAN_PASS_STRUCTS
  value.sival_ptr = client->arg;
  ret = sigqueue(client->pid, client->signo, value);
#else
  ret = sigqueue(client->pid, client->signo, client->arg);
#endif

  if (ret < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode > 0);

      nlldbg("ERROR: sigqueue failed: %d\n", errcode);
      UNUSED(errcode);
    }

  return OK;
}

/****************************************************************************
 * Name: phy_handler_0, phy_handler_1, ...
 ****************************************************************************/

static int phy_handler_0(int irq, FAR void *context)
{
  return phy_handler(&g_notify_clients[0]);
}

#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 1
static int phy_handler_1(int irq, FAR void *context)
{
  return phy_handler(&g_notify_clients[1]);
}
#endif

#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 2
static int phy_handler_2(int irq, FAR void *context)
{
  return phy_handler(&g_notify_clients[2]);
}
#endif

#if CONFIG_PHY_NOTIFICATION_NCLIENTS > 3
static int phy_handler_3(int irq, FAR void *context)
{
  return phy_handler(&g_notify_clients[3]);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: phy_notify_subscribe
 *
 * Description:
 *   Setup up to deliver signals to the task identified by 'pid' when
 *   there is any change indicated by an interrupt from the PHY associated
 *   with 'intf'
 *
 *   NOTE: This function is intended to be called only from an Ethernet
 *   driver in support of the SIOCMIISIG ioctl command.  It should never
 *   by called directly by application logic.
 *
 * Parameters:
 *   intf  - Provides the name of the network interface, for example, "eth0".
 *           The length of intf must not exceed 4 bytes (excluding NULL
 *           terminator).  Configurable with CONFIG_PHY_NOTIFICATION_MAXINTFLEN.
 *   pid   - Identifies the task to receive the signal.  The special value
 *           of zero means to use the pid of the current task.
 *   signo - This is the signal number to use when notifying the task.
 *   arg   - An argument that will accompany the notification signal.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int phy_notify_subscribe(FAR const char *intf, pid_t pid, int signo,
                         FAR void *arg)
{
  FAR struct phy_notify_s *client;
  DEBUGASSERT(intf);

  nvdbg("%s: PID=%d signo=%d arg=%p\n", intf, pid, signo, arg);

  /* The special value pid == 0 means to use the pid of the current task. */

  if (pid == 0)
    {
      pid = getpid();
      phydbg("Actual PID=%d\n", pid);
    }

  /* Check if this client already exists */

  client = phy_find_assigned(intf, pid);
  if (client)
    {
      /* Yes.. update the signal number and argument */

      client->signo = signo;
      client->arg   = arg;
    }
  else
    {
      /* No, allocate a new slot in the client notification table */

      client = phy_find_unassigned();
      if (!client)
        {
          ndbg("ERROR: Failed to allocate a client entry\n");
          return -ENOMEM;
        }

      /* Initialize the new client entry */

      client->signo = signo;
      client->pid   = pid;
      client->arg   = arg;
#ifdef CONFIG_NETDEV_MULTINIC
      snprintf(client->intf, CONFIG_PHY_NOTIFICATION_MAXINTFLEN+1, intf);
      client->intf[CONFIG_PHY_NOTIFICATION_MAXINTFLEN] = '\0';
#endif

      /* Attach/re-attach the PHY interrupt */

      (void)arch_phy_irq(intf, g_notify_handler[client->index], &client->enable);
    }

  /* Enable/re-enable the PH interrupt */

  DEBUGASSERT(client->enable);
  client->enable(true);
  return OK;
}

/****************************************************************************
 * Function: phy_notify_unsubscribe
 *
 * Description:
 *   Stop the deliver of signals for events from the PHY associated with
 *   'intf' to the task identified by 'pid'
 *
 *   NOTE: This function is intended to be called only from an Ethernet
 *   driver in support of the SIOCMIISIG ioctl command.  It should never
 *   by called directly by application logic.
 *
 * Parameters:
 *   intf  - Provides the name of the network interface, for example, "eth0".
 *           The length of 'intf' must not exceed 4 bytes (excluding NULL
 *           terminator).  Configurable with CONFIG_PHY_NOTIFICATION_MAXINTFLEN.
 *   pid   - Identifies the task that was receiving notifications.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int phy_notify_unsubscribe(FAR const char *intf, pid_t pid)
{
  FAR struct phy_notify_s *client;

  nvdbg("%s: PID=%d\n", intf, pid);

  /* Find the client entry for this interface */

  client = phy_find_assigned(intf, pid);
  if (!client)
    {
      ndbg("ERROR: No such client\n");
      return -ENOENT;
    }

  /* Detach and disable the PHY interrupt */

  phy_semtake();
  (void)arch_phy_irq(intf, NULL, NULL);

  /* Un-initialize the client entry */

  client->assigned = false;
  client->signo    = 0;
#ifdef CONFIG_NETDEV_MULTINIC
  client->intf[0]  = '\0';
#endif
  client->pid      = -1;
  client->arg      = NULL;

  phy_semgive();
  return OK;
}

#endif /* CONFIG_ARCH_PHY_INTERRUPT */
