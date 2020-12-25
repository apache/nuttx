/****************************************************************************
 * arch/arm/src/tiva/tiva_eeprom.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Shirshak Sengupta <sgshirshak@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
FAR struct mtd_dev_s *tiva_eeprom_instance(void);

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_EEPROM_H */
