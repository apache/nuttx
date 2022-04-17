/****************************************************************************
 * arch/sim/src/sim/up_romgetc.c
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

#include <nuttx/arch.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_romgetc
 *
 * Description:
 *   In Harvard architectures, data accesses and instruction accesses occur
 *   on different buses, perhaps concurrently.  All data accesses are
 *   performed on the data bus unless special  machine instructions are
 *   used to read data from the instruction address space.  Also, in the
 *   typical MCU, the available SRAM data memory is much smaller that the
 *   non-volatile FLASH instruction memory.  So if the application requires
 *   many constant strings, the only practical solution may be to store
 *   those constant strings in FLASH memory where they can only be accessed
 *   using architecture-specific machine instructions.
 *
 *   A similar case is where strings are retained in "external" memory such
 *   as EEPROM or serial FLASH.  This case is similar only in that again
 *   special operations are required to obtain the string data; it cannot
 *   be accessed directly from a string pointer.
 *
 *   If CONFIG_ARCH_ROMGETC is defined, then the architecture logic must
 *   export the function up_romgetc().  up_romgetc() will simply read one
 *   byte of data from the instruction space.
 *
 *   If CONFIG_ARCH_ROMGETC, certain C stdio functions are effected: (1)
 *   All format strings in printf, fprintf, sprintf, etc. are assumed to
 *   lie in FLASH (string arguments for %s are still assumed to reside in
 *   SRAM). And (2), the string argument to puts and fputs is assumed to
 *   reside in FLASH.  Clearly, these assumptions may have to modified for
 *   the particular needs of your environment.  There is no
 *   "one-size-fits-all" solution for this problem.
 *
 ****************************************************************************/

char up_romgetc(const char *ptr)
{
  /* This is basically a No-Op if enabled in the simulation environment */

  return *ptr;
}
