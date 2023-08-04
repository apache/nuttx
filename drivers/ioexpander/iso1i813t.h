/****************************************************************************
 * drivers/ioexpander/iso1i813t.h
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

#ifndef __DRIVERS_IOEXPANDER_ISO1I813T_H
#define __DRIVERS_IOEXPANDER_ISO1I813T_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_ISO1I813T)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers Definition */

#define ISO1I813T_DIAG      0x00 /* Collective Diagnostics Register */
#define ISO1I813T_INPDATA   0x02 /* Input Data Register */
#define ISO1I813T_GLERR     0x04 /* Global Error Register */
#define ISO1I813T_COEFIL    0x06 /* Filter Time for the Data and the Diagnostics */
#define ISO1I813T_INTERR    0x16 /* Internal Error Register */
#define ISO1I813T_GLCFG     0x18 /* Global Configuration Register */

#define ISO1I813T_WRITE_OPS 1 << 7

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_SPI
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables I/O expander support
 *
 * CONFIG_IOEXPANDER_ISO1I813T
 *   Enables support for the ISO1I813T driver (Needs CONFIG_INPUT)
 * CONFIG_ISO1I813T_MULTIPLE
 *   Can be defined to support multiple ISO1I813T devices on board.
 */

#ifndef CONFIG_SPI
#  error "CONFIG_SPI is required by ISO1I813T"
#endif

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_ISO1I813T */
#endif /* __DRIVERS_IOEXPANDER_ISO1I813T_H */
