/****************************************************************************
 * boards/renesas/rx65n/rx65n-grrose/src/rx65n_grrose.h
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

#ifndef __BOARDS_RENESAS_RX65N_GRROSE_SRC_RX65N_GRROSE_H
#define __BOARDS_RENESAS_RX65N_GRROSE_SRC_RX65N_GRROSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: rx65n_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int rx65n_bringup(void);

/****************************************************************************
 * Name: rx65n_sbsram_int
 *
 * Description:
 *   Initialize SBRAM Driver
 *
 *
 ****************************************************************************/
#ifdef CONFIG_RX65N_SBRAM
int rx65n_sbram_int(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RENESAS_RX65N_GRROSE_SRC_RX65N_GRROSE_H */
