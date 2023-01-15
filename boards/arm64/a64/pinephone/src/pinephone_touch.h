/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_touch.h
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

#ifndef __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_TOUCH_H
#define __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_TOUCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_touch_panel_register
 *
 * Description:
 *   Register the driver for Goodix GT9XX Touch Panel.  Attach the
 *   Interrupt Handler for the Touch Panel and disable Touch Interrupts.
 *
 * Input Parameters:
 *   devpath - Device Path (e.g. "/dev/input0")
 *   i2c     - I2C Bus
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_touch_panel_register(const char *devpath,
                                   struct i2c_master_s *i2c);

#endif /* __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_TOUCH_H */
