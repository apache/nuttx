/****************************************************************************
 * include/nuttx/reboot_notifier.h
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

#ifndef __INCLUDE_NUTTX_REBOOT_NOTIFIER_H
#define __INCLUDE_NUTTX_REBOOT_NOTIFIER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/notifier.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYS_DOWN        0x0001     /* Notify of system down */
#define SYS_RESTART     SYS_DOWN
#define SYS_HALT        0x0002     /* Notify of system halt */
#define SYS_POWER_OFF   0x0003     /* Notify of system power off */

/****************************************************************************
 * Public Function
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

void register_reboot_notifier(FAR struct notifier_block *nb);

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

void unregister_reboot_notifier(FAR struct notifier_block *nb);

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

void reboot_notifier_call_chain(unsigned long action, FAR void *data);

#endif /* __INCLUDE_NUTTX_REBOOT_NOTIFIER_H */

