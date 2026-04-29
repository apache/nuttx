/****************************************************************************
 * boards/x86/qemu/qemu-i486/src/qemu_vga_lcd.c
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

#include <stdio.h>
#include <stdbool.h>
#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>

#include "qemu_vga.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lcd_dev_s *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  g_lcddev = qemu_vga_initialize();
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to get the VGA LCD device\n");
    }
  else
    {
      lcdinfo("VGA LCD Initialized\n");
    }

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (!g_lcddev)
    {
      lcderr("ERROR: VGA LCD was not initialized\n");
    }
  else
    {
      lcdinfo("Returning the lcd_dev\n");

      return g_lcddev;
    }

  return NULL;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* TO-FIX */
}

