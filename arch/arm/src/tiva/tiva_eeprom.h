/****************************************************************************
 * arch/arm/src/tiva/tiva_eeprom.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_EEPROM_H
#define __ARCH_ARM_SRC_TIVA_TIVA_EEPROM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/tiva_eeprom.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_eeprom_initialize
 *
 * Description:
 *  Performs any necessary recovery in case of power failures during write.
 *
 *  This function must be called after tiva_eeprom_enable() and before the
 *  EEPROM is accessed.  It is used to check for errors in the EEPROM state
 *  such as from power failure during a previous write operation.  The
 *  function detects these errors and performs as much recovery as possible.
 *
 *  If -ENODEV is returned, the EEPROM was unable to recover its state.  If
 *  power is stable when this occurs, this indicates a fatal error and is
 *  likely an indication that the EEPROM memory has exceeded its specified
 *  lifetime write/erase specification.  If the supply voltage is unstable
 *  when this return code is observed, retrying the operation once the
 *  voltage is stabilized may clear the error.
 *
 *  Failure to call this function after a reset may lead to incorrect
 *  operation or permanent data loss if the EEPROM is later written.
 *
 * Returned Value:
 *   Returns OK if no errors were detected or -ENODEV if the EEPROM
 *   peripheral cannot currently recover from an interrupted write or erase
 *   operation.
 *
 ****************************************************************************/

int tiva_eeprom_initialize(void);

/****************************************************************************
 * Name: tiva_eeprom_instance
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can be
 *   bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

struct mtd_dev_s; /* Forward reference */
struct mtd_dev_s *tiva_eeprom_instance(void);

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_EEPROM_H */
