/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_serial.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_SERIAL_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "lpc54_config.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before lpc54_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in lpc54_lowsetup() and main clock
 *   initialization performed in lpc54_clockconfig().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void lpc54_earlyserialinit(void);
#endif

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_SERIAL_H */
