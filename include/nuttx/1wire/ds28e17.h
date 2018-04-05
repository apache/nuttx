/****************************************************************************
 * include/nuttx/1wire/ds28e17.h
 *
 *   Copyright (C) 2018 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_1WIRE_DS28E17_H
#define __INCLUDE_NUTTX_1WIRE_DS28E17_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/drivers/1wire.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default how many converters in a bus. */

#define DS_DEFAULT_MAXSLAVES         10

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ds28e17_dev_s;
struct i2c_master_s;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ds28e17_search
 *
 * Description:
 *   Search all DS28E17 devices from a 1-wire network.
 *
 * Input Parameters:
 *   priv      - Pointer to the associated DS28E17
 *   cb_search - Callback to call on each device found
 *   arg       - Argument passed to cb_search
 *
 * Return Value:
 *   Number of DS28E17 devices present.
 *
 ****************************************************************************/

int ds28e17_search(FAR struct ds28e17_dev_s *priv,
                   CODE void (*cb_search)(int family, uint64_t romcode, FAR void *arg),
                   FAR void *arg);

/****************************************************************************
 * Name: ds28e17_lower_half
 *
 * Description:
 *   Initialize the lower half of the DS28E17 by creating a i2c_master_s
 *   for the virtual i2c master and link it to the associated DS28E17 and
 *   its port.
 *
 * Input Parameters:
 *   priv    - Pointer to the associated DS28E17
 *   romcode - The unique 64-bit address in 1-wire network.
 *             Use zero for skip-ROM mode.
 *
 * Returned Value:
 *   i2c device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *ds28e17_lower_half(FAR struct ds28e17_dev_s *priv,
                                            uint64_t romcode);

/****************************************************************************
 * Name: ds28e17_lower_half_unregister
 *
 * Description:
 *   Put back the lower half of the DS28E17.
 *
 * Input Parameters:
 *   priv    - Pointer to the associated DS28E17
 *   i2cdev  - i2c device instance from ds28e17_lower_half()
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ds28e17_lower_half_unregister(FAR struct ds28e17_dev_s *priv,
                                  FAR struct i2c_master_s *i2cdev);

/****************************************************************************
 * Name: ds28e17_initialize
 *
 * Description:
 *   Returns a common DS28E17 device from 1-wire lower half device
 *
 * Input Parameters:
 *   dev - The allocated 1-wire lower half
 *
 ****************************************************************************/

FAR struct ds28e17_dev_s *ds28e17_initialize(FAR struct onewire_dev_s *dev);

#endif /* __INCLUDE_NUTTX_1WIRE_DS28E17_H */
