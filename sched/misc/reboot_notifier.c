/****************************************************************************
 * sched/misc/reboot_notifier.c
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

#include <nuttx/arch.h>
#include <nuttx/notifier.h>
#include <nuttx/reboot_notifier.h>

#include <sys/types.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ATOMIC_NOTIFIER_HEAD(g_reboot_notifier_list);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  register_reboot_notifier
 *
 * Description:
 *   Add notifier to the reboot notifier chain
 *
 * Input Parameters:
 *    nb - New entry in notifier chain
 *
 ****************************************************************************/

void register_reboot_notifier(FAR struct notifier_block *nb)
{
  atomic_notifier_chain_register(&g_reboot_notifier_list, nb);
}

/****************************************************************************
 * Name:  unregister_reboot_notifier
 *
 * Description:
 *   Remove notifier from the reboot notifier chain
 *
 * Input Parameters:
 *    nb - Entry to remove from notifier chain
 *
 ****************************************************************************/

void unregister_reboot_notifier(FAR struct notifier_block *nb)
{
  atomic_notifier_chain_unregister(&g_reboot_notifier_list, nb);
}

/****************************************************************************
 * Name:  reboot_notifier_call_chain
 *
 * Description:
 *   Call functions in the reboot notifier chain.
 *
 * Input Parameters:
 *    action - Value passed unmodified to notifier function
 *    data   - Pointer passed unmodified to notifier function
 *
 ****************************************************************************/

void reboot_notifier_call_chain(unsigned long action, FAR void *data)
{
  atomic_notifier_call_chain(&g_reboot_notifier_list, action, data);
}
