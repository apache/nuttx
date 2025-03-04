/****************************************************************************
 * drivers/note/noterpmsg.h
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

#ifndef __DRIVERS_NOTE_NOTERPMSG_H
#define __DRIVERS_NOTE_NOTERPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define NOTERPMSG_EPT_NAME           "rpmsg-note"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTERPMSG
extern struct noterpmsg_driver_s g_noterpmsg_driver;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTERPMSG_SERVER
int noterpmsg_server_init(void);
#endif

#ifdef CONFIG_DRIVERS_NOTERPMSG
int noterpmsg_init(void);
#endif

#endif /* __DRIVERS_NOTE_NOTERPMSG_H */
