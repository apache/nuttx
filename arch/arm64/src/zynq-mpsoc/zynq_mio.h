/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/zynq_mio.h
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

#ifndef __ARCH_ARM64_ZYNQ_MPSOC_ZYNQ_MIO_H
#define __ARCH_ARM64_ZYNQ_MPSOC_ZYNQ_MIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm64_internal.h"
#include "hardware/zynq_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ZYNQ_MIO_MAX_BANK        (6)        /*  MIO+EMIO MAX bank number  */
#define ZYNQ_MIO_PIN_MAX         (174)      /*  MIO+EMIO MAX pin number   */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 *
 * This function resets the GPIO module by writing reset values to all
 * registers
 *
 ****************************************************************************/

void zynq_mio_initialize(void);

/****************************************************************************
 *
 * read the data register of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *     Valid values are 0-5 in Zynq Ultrascale+ MP.
 *
 * return Current value of the data register.
 *
 * note: This function is used for reading the state of all the GPIO pins
 * of specified bank.
 *
 ****************************************************************************/

uint32_t zynq_mio_read(uint32_t bank);

/****************************************************************************
 *
 * read data from the specified pin.
 *
 * pin is the pin number for which the data has to be read.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 *
 * return Current value of the pin (0 or 1).
 *
 * note: This function is used for reading the state of the specified
 *    GPIO pin.
 *
 ****************************************************************************/

bool zynq_mio_readpin(uint32_t pin);

/****************************************************************************
 *
 * write to the data register of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 * data is the value to be written to the data register.
 *
 * note: This function is used for writing to all the GPIO pins of
 *    the bank. The previous state of the pins is not maintained.
 *
 ****************************************************************************/

void zynq_mio_write(uint32_t bank, uint32_t data);

/****************************************************************************
 *
 * write data to the specified pin.
 *
 * pin is the pin number to which the data is to be written.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 * data is the data to be written to the specified pin (0 or 1).
 *
 * note: This function does a masked write to the specified pin of
 *    the specified GPIO bank. The previous state of other pins
 *    is maintained.
 *
 ****************************************************************************/

void zynq_mio_writepin(uint32_t pin, bool data);

/****************************************************************************
 *
 * Set the dir of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 * dir is the 32 bit mask of the pin direction to be set for
 *    all the pins in the bank. Bits with 0 are set to Input mode,
 *    bits with 1 are set to Output Mode.
 *
 * note: This function is used for setting the direction of all the pins
 *    in the specified bank. The previous state of the pins is
 *    not maintained.
 *
 ****************************************************************************/

void zynq_mio_setdir(uint32_t bank, uint32_t dir);

/****************************************************************************
 *
 * Set the dir of the specified pin.
 *
 * pin is the pin number to which the data is to be written.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 * dir is the direction to be set for the specified pin.
 *    Valid values are 0 for Input dir, 1 for Output dir.
 *
 ****************************************************************************/

void zynq_mio_setdirpin(uint32_t pin, bool dir);

/****************************************************************************
 *
 * Get the dir of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 *
 * return Returns a 32 bit mask of the dir register. Bits with 0 are
 *     in Input mode, bits with 1 are in Output Mode.
 *
 ****************************************************************************/

uint32_t zynq_mio_getdir(uint32_t bank);

/****************************************************************************
 *
 * Get the dir of the specified pin.
 *
 * pin is the pin number for which the dir is to be
 *    retrieved.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 *
 * return dir of the specified pin.
 *    - 0 for Input dir
 *    - 1 for Output dir
 *
 ****************************************************************************/

uint32_t zynq_mio_getdirpin(uint32_t pin);

/****************************************************************************
 *
 * Set the Output Enable of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 * outen is the 32 bit mask of the Output Enables to be set for
 *    all the pins in the bank. The Output Enable of bits with 0 are
 *    disabled, the Output Enable of bits with 1 are enabled.
 *
 * return None.
 *
 * note: This function is used for setting the Output Enables of all the
 *    pins in the specified bank. The previous state of the Output
 *    Enables is not maintained.
 *
 ****************************************************************************/

void zynq_mio_setouten(uint32_t bank, uint32_t outen);

/****************************************************************************
 *
 * Set the Output Enable of the specified pin.
 *
 * pin is the pin number to which the data is to be written.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 * outen specifies whether the Output Enable for the specified
 *    pin should be enabled.
 *    Valid values are 0 for Disabling Output Enable,
 *    1 for Enabling Output Enable.
 *
 ****************************************************************************/

void zynq_mio_setoutenpin(uint32_t pin, bool outen);

/****************************************************************************
 *
 * Get the Output Enable status of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 *
 * return Returns a a 32 bit mask of the Output Enable register.
 *    Bits with 0 are in Disabled state, bits with 1 are in
 *    Enabled State.
 *
 ****************************************************************************/

uint32_t zynq_mio_getouten(uint32_t bank);

/****************************************************************************
 *
 * Get the Output Enable status of the specified pin.
 *
 * pin is the pin number for which the Output Enable status is to
 *    be retrieved.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 *
 * return Output Enable of the specified pin.
 *    - 0 if Output Enable is disabled for this pin
 *    - 1 if Output Enable is enabled for this pin
 *
 ****************************************************************************/

uint32_t zynq_mio_getoutenpin(uint32_t pin);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_ZYNQ_MPSOC_ZYNQ_MIO_H */
