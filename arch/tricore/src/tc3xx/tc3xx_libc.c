/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_libc.c
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
#include <stdint.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* TODO:
 * The data copy from flash to ram reuses the implementation in tricore sdk.
 * The next update will reimplement _c_init() to abandon the copy code.
 *
 * Usage: ltc [options] files
 * -i --user-provided-initialization-code
 *
 *         the user provides his own initialization
 *         routine, do not emit the copytable
 *
 * void _c_init(void)
 * {
 * }
 */

/* The implementation of libc is introduced by default in the Tricore
 * toolchain, in nuttx we made a fake libc_fpu.a library to bypass
 * this issue, but the linker will still generate markup code,
 * add a few definitions to fool the linker.
 */

void __printf_float(void)
{
}

void __printf_int(void)
{
}

void __printf_llong(void)
{
}

void _main(void)
{
}

void _doexit(void)
{
}

/* BUG, Workaroud for tasking compiler:
 *
 * ltc E106: unresolved external: regulator_gpio_init -
 *                                (drivers_initialize.o)
 * ltc F019: unrecoverable error: fatal link error
 *
 */

int regulator_gpio_init(void *iodev, void *desc)
{
  return 0;
}
