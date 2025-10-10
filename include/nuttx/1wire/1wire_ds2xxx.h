/****************************************************************************
 * include/nuttx/1wire/1wire_ds2xxx.h
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
 * Author: Stepan Pressl <pressl.stepan@gmail.com>
 *                       <pressste@fel.cvut.cz>
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_1WIRE_1WIRE_DS2XXX_H
#define __INCLUDE_NUTTX_1WIRE_1WIRE_DS2XXX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/1wire/1wire.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* All device specific commands for the scratchpad are luckily the same
 * for all the devices.
 */

#define DS2XXX_WRITE_SCRATCHPAD 0x0f
#define DS2XXX_READ_SCRATCHPAD  0xaa
#define DS2XXX_COPY_SCRATCHPAD  0x55
#define DS2XXX_READ_MEMORY      0xf0

/* There are other commands, also. For example, the common commands
 * have a different code than specified in drivers/1wire/1wire.c.
 */

#define DS2XXX_MATCH_ROM 0x55

/* DS28E05 has a different architecture,
 * as it does not use a scratchpad. Let's keep this device out of here.
 */

enum ds2xxx_eeproms_e
{
    EEPROM_DS2430 = 0,
    EEPROM_DS2431,
    EEPROM_DS2432,
    EEPROM_DS2433,
    EEPROM_DS28E04,
    EEPROM_DS28E07,
    EEPROM_DS28EC20,
    EEPROM_DS_COUNT
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ds2xxx_initialize
 *
 * Description: Bind a onewire_dev_s struct to this driver,
 * capable of interfacing DS2XXX 1Wire EEPROMs. The user must specify
 * the device type and also the name of the device (e.g. /dev/ds2xxx).
 *
 ****************************************************************************/

int ds2xxx_initialize(FAR struct onewire_dev_s *dev,
                      enum ds2xxx_eeproms_e devtype, FAR char *devname);

#endif /* __INCLUDE_NUTTX_1WIRE_1WIRE_DS2XXX_H */
