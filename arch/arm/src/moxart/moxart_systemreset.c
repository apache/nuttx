/****************************************************************************
 * arch/arm/src/moxart/moxart_systemreset.c
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

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FTWDT010_LOAD    0x98500004
#define FTWDT010_RESTART 0x98500008
#define FTWDT010_CR      0x9850000C

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal, reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  putreg32(0, FTWDT010_CR);
  putreg32(0, FTWDT010_LOAD);
  putreg32(0x5ab9, FTWDT010_RESTART); /* Magic */

  putreg32(0x11, FTWDT010_CR);
  putreg32(0x13, FTWDT010_CR);

  /* Wait for the reset */

  for (; ; );
}
