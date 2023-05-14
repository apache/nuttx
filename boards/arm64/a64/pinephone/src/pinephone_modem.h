/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_modem.h
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

#ifndef __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_MODEM_H
#define __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_MODEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_modem_init
 *
 * Description:
 *   Initialize the Quectel EG25-G LTE Modem.  Power up the modem, deassert
 *   the Reset, toggle the Power Key and wait for the modem to be ready
 *   (3 seconds). Note that the Reset and Power Key Pins are High-Low
 *   Inverted for PinePhone.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_modem_init(void);

#endif /* __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_MODEM_H */
