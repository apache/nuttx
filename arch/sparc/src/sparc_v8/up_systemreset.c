/****************************************************************************
 * arch/sparc/src/sparc_v8/up_systemreset.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "up_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal, sparc reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  asm("st  %g0,[%g1+0x90]"); /* ÐŽÈëÖÐ¶ÏŒ¶±ðºÍÓÅÏÈ¿ØÖÆŒÄŽæÆ÷£¬ÆÁ±ÎËùÓÐÖÐ¶Ï */
  asm("st  %g0,[%g1+0x94]"); /* ÐŽÈëÖÐ¶ÏÇëÇóŒÄŽæÆ÷,Çå³ýËùÓÐµÄÖÐ¶Ï */
  asm("st  %g0,[%g1+0x98]"); /* ÐŽÈëÇ¿ÖÆÖÐ¶ÏŒÄŽæÆ÷£¬Çå³ýËùÓÐÖÐ¶Ï */

  /* Çå³ýËùÓÐ±»×èÈûµÄÖÐ¶Ï */

  asm("set  0xfffe,%g2");    /* ÉèÖÃÖÐ¶ÏÇå³ýŒÄŽæÆ÷µÄÖµ */
  asm("st  %g2,[%g1+0x9c]"); /* ÐŽÈëÖÐ¶ÏÇå³ýŒÄŽæÆ÷£¬Çå³ýËùÓÐµÄÖÐ¶Ï */

  asm("set 0x1024, %l4");
  asm("jmp %l4");
  asm("nop");
  asm("nop");

  /* Wait for the reset */

  for (; ; );
}
