/****************************************************************************
 * sched/clock/clock_notifier.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/clock_notifier.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ATOMIC_NOTIFIER_HEAD(g_clock_notifier_list);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  register_clock_notifier
 *
 * Description:
 *   Add notifier to the clock_notifier chain
 *
 * Input Parameters:
 *    nb - New entry in notifier chain
 *
 ****************************************************************************/

void register_clock_notifier(FAR struct notifier_block *nb)
{
  atomic_notifier_chain_register(&g_clock_notifier_list, nb);
}

/****************************************************************************
 * Name:  unregister_clock_notifier
 *
 * Description:
 *   Remove notifier from the clock_notifier chain
 *
 * Input Parameters:
 *    nb - Entry to remove from notifier chain
 *
 ****************************************************************************/

void unregister_clock_notifier(FAR struct notifier_block *nb)
{
  atomic_notifier_chain_unregister(&g_clock_notifier_list, nb);
}

/****************************************************************************
 * Name:  clock_notifier_call_chain
 *
 * Description:
 *   Call functions in the clock_notifier chain.
 *
 * Input Parameters:
 *    action - Value passed unmodified to notifier function
 *    tp - Pointer of current timespec passed unmodified to notifier function
 *
 ****************************************************************************/

void clock_notifier_call_chain(unsigned long action,
                               FAR const struct timespec *tp)
{
  atomic_notifier_call_chain(&g_clock_notifier_list, action, (FAR void *)tp);
}
