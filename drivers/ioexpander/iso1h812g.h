/****************************************************************************
 * drivers/ioexpander/iso1h812g.h
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

#ifndef __DRIVERS_IOEXPANDER_ISO1H812G_H
#define __DRIVERS_IOEXPANDER_ISO1H812G_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_ISO1H812G)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_SPI
 *     SPI support is required
 *   CONFIG_IOEXPANDER
 *     Enables I/O expander support
 *
 * CONFIG_IOEXPANDER_ISO1H812G
 *   Enables support for the ISO1H812G driver (Needs CONFIG_INPUT)
 * CONFIG_ISO1H812G_MULTIPLE
 *   Can be defined to support multiple ISO1H812G devices on board.
 */

#ifndef CONFIG_SPI
#  error "CONFIG_SPI is required by ISO1H812G"
#endif

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_ISO1H812G */
#endif /* __DRIVERS_IOEXPANDER_ISO1H812G_H */
