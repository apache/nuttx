/****************************************************************************
 * boards/z80/z80/z80sim/src/z80_lowputc.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include "z80_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
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

/****************************************************************************
 * Name: z80_lowputc
 *
 * Data sent to port 0xbe are echoed on stdout by the simulation
 *
 ****************************************************************************/

void z80_lowputc(char ch) __naked
{
  __asm__ (
  "\tld hl, #2\n"
  "\tadd hl, sp\n"
  "\tld a, (hl)\n"
  "\tout (0xbe), a\n"
  "\tret\n"
  );
}

/****************************************************************************
 * Name: z80_lowgetc
 *
 * Data from stdin can be received on port 0xbe in the simulation
 *
 ****************************************************************************/

char z80_lowgetc(void) __naked
{
  __asm__ (
  "\tin a, (0xbe)\n"
  "\tld l, a\n"
  "\tld h, #0\n"
  "\tret\n"
  );
}
