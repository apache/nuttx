/****************************************************************************
 * drivers/net/phy_notify.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <net/if.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
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
  char intf[IFNAMSIZ + 1];
  pid_t pid;
  struct sigevent event;
  struct sigwork_s work;
  phy_enable_t enable;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int phy_semtake(void);
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

static struct phy_notify_s
  g_notify_clients[CONFIG_PHY_NOTIFICATION_NCLIENTS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_semtake
 ****************************************************************************/

static int phy_semtake(void)
{
  return nxsem_wait_uninterruptible(&g_notify_clients_sem);
}

#define phy_semgive() nxsem_post(&g_notify_clients_sem);

/****************************************************************************
 * Name: phy_find_unassigned
 ****************************************************************************/

static FAR struct phy_notify_s *phy_find_unassigned(void)
{
  FAR struct phy_notify_s *client;
  int ret;
  int i;

  ret = phy_semtake();
  if (ret < 0)
    {
      phyerr("ERROR: phy_semtake failed: %d\n", ret);
      return NULL;
    }

  for (i = 0; i < CONFIG_PHY_NOTIFICATION_NCLIENTS; i++)
    {
      client = &g_notify_clients[i];
      if (!client->assigned)
        {
          /* Assign and re-initialized the entry */

          client->assigned = true;
          client->intf[0]  = '\0';
          client->pid      = INVALID_PROCESS_ID;
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
  int ret;
  int i;

  ret = phy_semtake();
  if (ret < 0)
    {
      phyerr("ERROR: phy_semtake failed: %d\n", ret);
      return NULL;
    }

  for (i = 0; i < CONFIG_PHY_NOTIFICATION_NCLIENTS; i++)
    {
      client = &g_notify_clients[i];
      if (client->assigned && client->pid == pid &&
          strncmp(client->intf, intf, IFNAMSIZ) == 0)
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
  int ret;

  DEBUGASSERT(client != NULL && client->assigned && client->enable);
  phyinfo("Signaling PID=%d with event %p\n", client->pid, &client->event);

  /* Disable further interrupts */

  client->enable(false);

  /* Signal the client that the PHY has something interesting to say to us */

  ret = nxsig_notification(client->pid, &client->event,
                           SI_QUEUE, &client->work);
  if (ret < 0)
    {
      phyerr("ERROR: nxsig_notification failed: %d\n", ret);
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
 *   pid   - Identifies the task to receive the signal.  The special value
 *           of zero means to use the pid of the current task.
 *   event - Describes the way a task is to be notified
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int phy_notify_subscribe(FAR const char *intf, pid_t pid,
                         FAR struct sigevent *event)
{
  FAR struct phy_notify_s *client;
  int ret = OK;

  DEBUGASSERT(intf);

  phyinfo("%s: PID=%d event=%p\n", intf, pid, event);

  /* The special value pid == 0 means to use the pid of the current task. */

  if (pid == 0)
    {
      pid = getpid();
      phyinfo("Actual PID=%d\n", pid);
    }

  /* Unsubscribe if sigev_notify field equals SIGEV_NONE */

  if (event->sigev_notify == SIGEV_NONE)
    {
      return phy_notify_unsubscribe(intf, pid);
    }

  /* Check if this client already exists */

  client = phy_find_assigned(intf, pid);
  if (client != NULL)
    {
      /* Yes.. update the signal number and argument */

      client->event = *event;
    }
  else
    {
      /* No, allocate a new slot in the client notification table */

      client = phy_find_unassigned();
      if (client == NULL)
        {
          phyerr("ERROR: Failed to allocate a client entry\n");
          return -ENOMEM;
        }

      /* Initialize the new client entry */

      client->pid   = pid;
      client->event = *event;
      strncpy(client->intf, intf, IFNAMSIZ + 1);
      client->intf[IFNAMSIZ] = '\0';

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
 *   pid   - Identifies the task that was receiving notifications.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int phy_notify_unsubscribe(FAR const char *intf, pid_t pid)
{
  FAR struct phy_notify_s *client;
  int ret;

  phyinfo("%s: PID=%d\n", intf, pid);

  /* Find the client entry for this interface */

  client = phy_find_assigned(intf, pid);
  if (client == NULL)
    {
      phyerr("ERROR: No such client\n");
      return -ENOENT;
    }

  /* Detach and disable the PHY interrupt */

  ret = phy_semtake();
  if (ret >= 0)
    {
      arch_phy_irq(intf, NULL, NULL, NULL);

      /* Cancel any pending notification */

      nxsig_cancel_notification(&client->work);

      /* Un-initialize the client entry */

      client->assigned = false;
      client->intf[0]  = '\0';
      client->pid      = INVALID_PROCESS_ID;

      phy_semgive();
    }

  return OK;
}

#endif /* CONFIG_ARCH_PHY_INTERRUPT */
