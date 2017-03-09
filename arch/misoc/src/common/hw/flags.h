/****************************************************************************
 *  arch/misoc/src/common/hw/emac_mem.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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
 ****************************************************************************/

#ifndef __ARCH_MISOC_SRC_COMMON_HW_FLAGS_H
#define __ARCH_MISOC_SRC_COMMON_HW_FLAGS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_EV_TX                      0x1
#define UART_EV_RX                      0x2

#define DFII_CONTROL_SEL                0x01
#define DFII_CONTROL_CKE                0x02
#define DFII_CONTROL_ODT                0x04
#define DFII_CONTROL_RESET_N            0x08

#define DFII_COMMAND_CS                 0x01
#define DFII_COMMAND_WE                 0x02
#define DFII_COMMAND_CAS                0x04
#define DFII_COMMAND_RAS                0x08
#define DFII_COMMAND_WRDATA             0x10
#define DFII_COMMAND_RDDATA             0x20

#define ETHMAC_EV_SRAM_WRITER           0x1
#define ETHMAC_EV_SRAM_READER           0x1

#define CLKGEN_STATUS_BUSY              0x1
#define CLKGEN_STATUS_PROGDONE          0x2
#define CLKGEN_STATUS_LOCKED            0x4

#define DVISAMPLER_TOO_LATE             0x1
#define DVISAMPLER_TOO_EARLY            0x2

#define DVISAMPLER_DELAY_MASTER_CAL     0x01
#define DVISAMPLER_DELAY_MASTER_RST     0x02
#define DVISAMPLER_DELAY_SLAVE_CAL      0x04
#define DVISAMPLER_DELAY_SLAVE_RST      0x08
#define DVISAMPLER_DELAY_INC            0x10
#define DVISAMPLER_DELAY_DEC            0x20

#define DVISAMPLER_SLOT_EMPTY           0
#define DVISAMPLER_SLOT_LOADED          1
#define DVISAMPLER_SLOT_PENDING         2

#endif /* __ARCH_MISOC_SRC_COMMON_HW_FLAGS_H */
