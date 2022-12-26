/****************************************************************************
 * include/nuttx/panic_notifier.h
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

#ifndef __INCLUDE_NUTTX_PANIC_NOTIFIER_H
#define __INCLUDE_NUTTX_PANIC_NOTIFIER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/notifier.h>

#include <sys/types.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum panic_type_e
{
  PANIC_KERNEL         =  0,
  PANIC_TASK           =  1,
};

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name:  panic_notifier_chain_register
 *
 * Description:
 *   Add notifier to the panic notifier chain
 *
 * Input Parameters:
 *    nb - New entry in notifier chain
 *
 ****************************************************************************/

void panic_notifier_chain_register(FAR struct notifier_block *nb);

/****************************************************************************
 * Name:  panic_notifier_chain_unregister
 *
 * Description:
 *   Remove notifier from the panic notifier chain
 *
 * Input Parameters:
 *    nb - Entry to remove from notifier chain
 *
 ****************************************************************************/

void panic_notifier_chain_unregister(FAR struct notifier_block *nb);

/****************************************************************************
 * Name:  panic_notifier_call_chain
 *
 * Description:
 *   Call functions in the panic notifier chain.
 *
 * Input Parameters:
 *    action - Value passed unmodified to notifier function
 *    data   - Pointer passed unmodified to notifier function
 *
 ****************************************************************************/

void panic_notifier_call_chain(unsigned long action, FAR void *data);

#endif /* __INCLUDE_NUTTX_PANIC_NOTIFIER_H */
