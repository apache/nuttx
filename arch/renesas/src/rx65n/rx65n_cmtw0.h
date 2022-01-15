/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cmtw0.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_CMTW0_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_CMTW0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "rx65n_cmtw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Compare Match Constant Register (CMWCOR) */

#define _000000BB_CMTW0_CMWCOR_VALUE              (0xfffffffful)

/* Output Compare Registers 0 (CMWOCR0) */

#define _0000000A_CMTW0_CMWOCR0_VALUE             (0x0000000aul)

/* Output Compare Registers 1 (CMWOCR1) */

#define _0000000A_CMTW0_CMWOCR1_VALUE             (0x0000000aul)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_cmtw0_create
 *
 * Description:
 *   Initializes CMTW0 Timer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rx65n_cmtw0_create(uint32_t txpoll_time, uint32_t txtimeout_time);

/****************************************************************************
 * Name: rx65n_cmtw0_start
 *
 * Description:
 *   Start CMTW0 Timer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rx65n_cmtw0_start(uint8_t type, uint32_t timeout);

/****************************************************************************
 * Name: rx65n_cmtw0_stop
 *
 * Description:
 *   Stop CMTW0 Timer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rx65n_cmtw0_stop(uint8_t type);

#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_CMTW0_H */
