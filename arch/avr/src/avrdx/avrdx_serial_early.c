/****************************************************************************
 * arch/avr/src/avrdx/avrdx_serial_early.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "avrdx_config.h"

#include <stdint.h>
#include <avr/io.h>

#include "avrdx.h"
#include "avrdx_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_DEV_CONSOLE

/****************************************************************************
 * Name: avr_earlyserialinit
 *
 * Description:
 *   Not doing anything but common code expects this to exist. (Serial code
 *   is modeled after ATmega and that family has call to up_consoleinit
 *   which calls usartN_configure ifdef HAVE_SERIAL_CONSOLE. Then, there is
 *   avr_earlyserialinit() which disables interrupts ("early" - they are
 *   not enabled) and calls usartN_setup, which in turns calls
 *   usartN_configure. Does not seem necessary to do it twice so that
 *   part of ATmega code's behaviour is not mirrored.)
 *
 ****************************************************************************/

void avr_earlyserialinit(void)
{
}

#endif
