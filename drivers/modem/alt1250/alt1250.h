/****************************************************************************
 * drivers/modem/alt1250/alt1250.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALT1250_H
#define __DRIVERS_MODEM_ALT1250_ALT1250_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/lte/lte_ioctl.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: get_event_lapibuffer
 *
 * Description:
 *   Gets the buffer used in the LAPI event.
 *
 * Input Parameters:
 *   dev         - An device instance of the ALT11250 driver.
 *   lapicmdid   - ID of the LAPI command.
 *   inst        - A pointer to an instance of the LAPI event buffer.
 *
 * Returned Value:
 *   Returns a bitmap indicating the location of the LAPI event buffer.
 *   If the buffer associated with the specified lapicmdid does not exist,
 *   NULL is returned.
 *
 ****************************************************************************/

uint64_t get_event_lapibuffer(FAR struct alt1250_dev_s *dev,
  uint32_t lapicmdid, alt_evtbuf_inst_t **inst);

#endif  /* __DRIVERS_MODEM_ALT1250_ALT1250_H */
