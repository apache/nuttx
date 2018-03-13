/****************************************************************************
 * drivers/net/phy_notify.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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
#  undef  CONFIG_DEBUG_INFO
#  define CONFIG_DEBUG_INFO 1
#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <signal.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
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
#  define phyinfo   _info
#  define phyerr    _err
#else
#  define phyinfo(x...)
#  define phyerr(x...)
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
  char intf[CONFIG_PHY_NOTIFICATION_MAXINTFLEN+1];
  pid_t pid;
  FAR void *arg;
  phy_enable_t enable;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void phy_semtake(void);
static FAR struct phy_notify_s *phy_find_unassigned(void);
static FAR struct phy_notify_s *phy_find_assigned(FAR const char *intf,
                                                  pid_t pid);
static int phy_handler(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Serializes access to the g_notify_clients array */

static sem_t g_notify_clients_sem = SEM_INITIALIZER(1);

/* This is a array the hold information for each PHY notification client */

static struct phy_notify_s g_notify_clients[CONFIG_PHY_NOTIFICATION_NCLIENTS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_semtake
 ****************************************************************************/

static void phy_semtake(void)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_notify_clients_sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

#define phy_semgive() nxsem_post(&g_notify_clients_sem);

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
          client->intf[0]  = '\0';
          client->pid      = -1;
          client->arg      = NULL;
          client->enable   = NULL;

          /* Return the client entry assigned to the caller */

          phy_semgive();
          phyinfo("Returning client %d\n", i);
          return client;
        }
    }

  /* Ooops... too many */

  phyerr("ERROR: No free client entries\n");
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
      if (client->assigned && client->pid == pid &&
          strncmp(client->intf, intf, CONFIG_PHY_NOTIFICATION_MAXINTFLEN) == 0)
        {
          /* Return the matching client entry to the caller */

          phy_semgive();
          phyinfo("Returning client %d\n", i);
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

static int phy_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct phy_notify_s *client = (FAR struct phy_notify_s *)arg;
#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
#endif
  int ret;

  DEBUGASSERT(client != NULL && client->assigned && client->enable);
  phyinfo("Signalling PID=%d with signal %d\n", client->pid, client->signo);

  /* Disable further interrupts */

  client->enable(false);

  /* Signal the client that the PHY has something interesting to say to us */

#ifdef CONFIG_CAN_PASS_STRUCTS
  value.sival_ptr = client->arg;
  ret = nxsig_queue(client->pid, client->signo, value);
#else
  ret = nxsig_queue(client->pid, client->signo, client->arg);
#endif

  if (ret < 0)
    {
      phyerr("ERROR: nxsig_queue failed: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_notify_subscribe
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
 * Input Parameters:
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
  int ret = OK;

  DEBUGASSERT(intf);

  phyinfo("%s: PID=%d signo=%d arg=%p\n", intf, pid, signo, arg);

  /* The special value pid == 0 means to use the pid of the current task. */

  if (pid == 0)
    {
      pid = getpid();
      phyinfo("Actual PID=%d\n", pid);
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
          phyerr("ERROR: Failed to allocate a client entry\n");
          return -ENOMEM;
        }

      /* Initialize the new client entry */

      client->signo = signo;
      client->pid   = pid;
      client->arg   = arg;
      snprintf(client->intf, CONFIG_PHY_NOTIFICATION_MAXINTFLEN+1, intf);
      client->intf[CONFIG_PHY_NOTIFICATION_MAXINTFLEN] = '\0';

      /* Attach/re-attach the PHY interrupt */

      ret = arch_phy_irq(intf, phy_handler, client, &client->enable);
    }

  /* Enable/re-enable the PH interrupt */

  DEBUGASSERT(client->enable);
  client->enable(true);
  return ret;
}

/****************************************************************************
 * Name: phy_notify_unsubscribe
 *
 * Description:
 *   Stop the deliver of signals for events from the PHY associated with
 *   'intf' to the task identified by 'pid'
 *
 *   NOTE: This function is intended to be called only from an Ethernet
 *   driver in support of the SIOCMIISIG ioctl command.  It should never
 *   by called directly by application logic.
 *
 * Input Parameters:
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

  phyinfo("%s: PID=%d\n", intf, pid);

  /* Find the client entry for this interface */

  client = phy_find_assigned(intf, pid);
  if (!client)
    {
      phyerr("ERROR: No such client\n");
      return -ENOENT;
    }

  /* Detach and disable the PHY interrupt */

  phy_semtake();
  (void)arch_phy_irq(intf, NULL, NULL, NULL);

  /* Un-initialize the client entry */

  client->assigned = false;
  client->signo    = 0;
  client->intf[0]  = '\0';
  client->pid      = -1;
  client->arg      = NULL;

  phy_semgive();
  return OK;
}

#endif /* CONFIG_ARCH_PHY_INTERRUPT */
