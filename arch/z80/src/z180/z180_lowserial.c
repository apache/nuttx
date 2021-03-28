/****************************************************************************
 * arch/z80/src/z180/z180_lowserial.c
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

#include "common/z80_internal.h"

#include "z180_config.h"
#include "z180_serial.h"

#ifdef HAVE_SERIAL

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: up_putc/up_lowputc
 *
 * Description:
 *   Low-level console output
 *
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
int up_lowputc(int ch)
#else
int up_putc(int ch)
#endif
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Output CR before LF */

      z180_putc('\r');
    }

  /* Output the character */

  z180_putc(ch);
  return ch;
}

#endif /* HAVE_SERIAL */
