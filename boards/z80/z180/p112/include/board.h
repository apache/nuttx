/****************************************************************************
 * boards/z80/z180/p112/include/board.h
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

#ifndef __BOARDS_Z80_Z180_P112_INCLUDE_BOARD_H
#define __BOARDS_Z80_Z180_P112_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Z180 is driven by a 16MHz crystal.  The system clock
 * is equal to the crystal frequency.
 */

#define Z180_BOARD_XTAL  16000000        /* 16 MHz */
#define Z180_SYSCLOCK    Z180_BOARD_XTAL /* 16 MHz */

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_Z80_Z180_P112_INCLUDE_BOARD_H */
