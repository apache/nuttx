/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cmtw0.h
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Surya Prakash <surya.prakash@tataelxsi.co.in>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#ifndef __ARCH_RENESAS_SRC_RX65N_CMTW0_H
#define __ARCH_RENESAS_SRC_RX65N_CMTW0_H

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
 ***************************************************************************/

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

#endif /* __ARCH_RENESAS_SRC_RX65N_CMTW0_H */
