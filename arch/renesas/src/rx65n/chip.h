/****************************************************************************
 * arch/renesas/src/rx65n/chip.h
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Anjana <anjana@tataelxsi.co.in>
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

#ifndef __ARCH_RENESAS_SRC_RX65N_CHIP_H
#define __ARCH_RENESAS_SRC_RX65N_CHIP_H
#ifndef __ASSEMBLY__

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_CHIP_R5F565NEDDF
#  include "rx65n_definitions.h"
#endif
#include "rx65n_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/****************************************************************************
 * Public Types
 ***************************************************************************/

/***************************************************************************
 * Public Data
 **************************************************************************/

#ifndef __ASSEMBLY__
extern uint16_t ebss;
#endif

/**************************************************************************
 * Public Functions
 *************************************************************************/

#endif
#endif /* __ARCH_RENESAS_SRC_SH1_CHIP_H */
