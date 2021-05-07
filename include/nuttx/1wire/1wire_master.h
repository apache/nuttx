/****************************************************************************
 * include/nuttx/1wire/1wire_master.h
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

#ifndef __INCLUDE_NUTTX_1WIRE_1WIRE_MASTER_H
#define __INCLUDE_NUTTX_1WIRE_1WIRE_MASTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct onewire_dev_s;
struct onewire_master_s;

struct onewire_config_s
{
  uint64_t romcode;                 /* Unique device identifier */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: onewire_write
 *
 * Description:
 *   Send a block of data on 1WIRE. Each write operation will be an 'atomic'
 *   operation in the sense that any other 1WIRE actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   master - Device-specific state data
 *   config - Described the 1WIRE configuration
 *   buffer - A pointer to the read-only buffer of data to be written to
 *            device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int onewire_write(FAR struct onewire_master_s *master,
                  FAR const struct onewire_config_s *config,
                  FAR const uint8_t *buffer, int buflen);

/****************************************************************************
 * Name: onewire_read
 *
 * Description:
 *   Receive a block of data from 1WIRE. Each read operation will be an
 *   'atomic' operation in the sense that any other 1WIRE actions will be
 *   serialized and pend until this read completes.
 *
 * Input Parameters:
 *   master - Device-specific state data
 *   config - Described the 1WIRE configuration
 *   buffer - A pointer to a buffer of data to receive the data from the
 *            device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int onewire_read(FAR struct onewire_master_s *master,
                 FAR const struct onewire_config_s *config,
                 FAR uint8_t *buffer, int buflen);

/****************************************************************************
 * Name: onewire_writeread
 *
 * Description:
 *   Receive a block of data from 1WIRE. Each read operation will be an
 *   'atomic' operation in the sense that any other 1WIRE actions will be
 *   serialized and pend until this read completes.
 *
 * Input Parameters:
 *   master  - Device-specific state data
 *   config  - Described the 1WIRE configuration
 *   wbuffer - A pointer to the read-only buffer of data to be written to
 *             device
 *   wbuflen - The number of bytes to send from the buffer
 *   rbuffer - A pointer to a buffer of data to receive the data from the
 *             device
 *   rbuflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int onewire_writeread(FAR struct onewire_master_s *master,
                      FAR const struct onewire_config_s *config,
                      FAR const uint8_t *wbuffer, int wbuflen,
                      FAR uint8_t *rbuffer, int rbuflen);

/****************************************************************************
 * Name: onewire_search
 *
 * Description:
 *   Search all devices from a 1-wire network. This is the 1-wire search
 *   algorithm from Maxim Application Note 187. Note! This is an atomic
 *   operation. The callback 'cb_search' can't execute any function that will
 *   lock this bus, because of the locked state as long the search is active.
 *
 * Input Parameters:
 *   master    - Pointer to the allocated 1-wire interface
 *   family    - Limit search to devices of matching family
 *   alarmonly - Limit search to devices on alarm state
 *   cb_search - Callback to call on each device found
 *   arg       - Argument passed to cb_search
 *
 * Return Value:
 *   Number of slaves present and matching family.
 *
 ****************************************************************************/

int onewire_search(FAR struct onewire_master_s *master,
                   int family,
                   bool alarmonly,
                   CODE void (*cb_search)(int family,
                                          uint64_t romcode,
                                          FAR void *arg),
                   FAR void *arg);

/****************************************************************************
 * Name: onewire_isalarm
 *
 * Description:
 *   Check if a 1wire devices has set the alarm flag in an atomic operation.
 *
 * Input Parameters:
 *   master  - Pointer to the allocated 1-wire interface
 *   config  - Described the 1WIRE configuration
 *
 * Return Value:
 *   0   - no alarm flag is set
 *   1   - alaram flag is set
 *   < 0 - in case of error
 *
 ****************************************************************************/

int onewire_isalarm(FAR struct onewire_master_s *master,
                    FAR const struct onewire_config_s *config);

/****************************************************************************
 * Name: onewire_initialize
 *
 * Description:
 *   Return 1-wire bus master from 1-wire lower half device.
 *
 * Input Parameters:
 *   dev       - Pointer to the allocated 1-wire lower half
 *   maxslaves - Maximum number of 1-wire slave devices
 *
 ****************************************************************************/

FAR struct onewire_master_s *
onewire_initialize(FAR struct onewire_dev_s *dev, int maxslaves);

/****************************************************************************
 * Name: onewire_uninitialize
 *
 * Description:
 *   Release 1-wire bus master.
 *
 * Input Parameters:
 *   master    - Pointer to the allocated 1-wire master
 *
 ****************************************************************************/

int onewire_uninitialize(FAR struct onewire_master_s *master);

#endif /* __INCLUDE_NUTTX_1WIRE_1WIRE_MASTER_H */
