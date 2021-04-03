/****************************************************************************
 * arch/z80/src/z180/z180_serial.h
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

#ifndef __ARCH_Z80_SRC_Z180_Z180_SERIAL_H
#define __ARCH_Z80_SRC_Z180_Z180_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "z80_internal.h"
#include "z180_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: z180_putc
 *
 * Description:
 *   Low-level character output
 *
 ****************************************************************************/

void z180_putc(uint8_t ch) __naked;

/****************************************************************************
 * Name: up_putc/up_lowputc
 *
 * Description:
 *   Low-level console output
 *
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
int up_lowputc(int ch);
#else
int up_putc(int ch);
#  define up_lowputc(ch) up_putc(ch)
#endif

#endif /* __ARCH_Z80_SRC_Z180_Z180_SERIAL_H */
