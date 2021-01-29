/****************************************************************************
 * include/nuttx/modem/ioctl.h
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

#ifndef __INCLUDE_NUTTX_MODEM_IOCTL_H
#define __INCLUDE_NUTTX_MODEM_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All modem-related IOCTL commands are defined here to assure that they are
 * globally unique.
 */

/* U-Blox Modem IOCTL commands */

#define MODEM_IOC_POWERON    _MODEMIOC(1)
#define MODEM_IOC_POWEROFF   _MODEMIOC(2)
#define MODEM_IOC_RESET      _MODEMIOC(3)
#define MODEM_IOC_GETSTATUS  _MODEMIOC(4)

/* Unrecognized IOCTL commands are forwarded to the lower half driver.  These
 * may include the modem commands from include/nuttx/serial/ioctl.h such as
 * the following:
 *
 *   TIOCMGET: Get modem status bits: FAR int
 *   TIOCMSET: Set modem status bits: FAR const int
 *   TIOCMBIC: Clear modem bits: FAR const int
 *   TIOCMBIS: Set modem bits: FAR const int
 */

#endif /* __INCLUDE_NUTTX_MODEM_IOCTL_H */
