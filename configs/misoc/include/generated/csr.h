/****************************************************************************
 * configs/misoc/include/generated/csr.h
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

#ifndef __CONFIGS_MISOC_INCLUDE_GENERATED_CSR_H
#define __CONFIGS_MISOC_INCLUDE_GENERATED_CSR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hw/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDRAM */

#define CSR_SDRAM_BASE 0xe0004000
#define CSR_SDRAM_DFII_CONTROL_ADDR 0xe0004000
#define CSR_SDRAM_DFII_CONTROL_SIZE 1

#define CSR_SDRAM_DFII_PI0_COMMAND_ADDR 0xe0004004
#define CSR_SDRAM_DFII_PI0_COMMAND_SIZE 1

#define CSR_SDRAM_DFII_PI0_COMMAND_ISSUE_ADDR 0xe0004008
#define CSR_SDRAM_DFII_PI0_COMMAND_ISSUE_SIZE 1

#define CSR_SDRAM_DFII_PI0_ADDRESS_ADDR 0xe000400c
#define CSR_SDRAM_DFII_PI0_ADDRESS_SIZE 2

#define CSR_SDRAM_DFII_PI0_BADDRESS_ADDR 0xe0004014
#define CSR_SDRAM_DFII_PI0_BADDRESS_SIZE 1

#define CSR_SDRAM_DFII_PI0_WRDATA_ADDR 0xe0004018
#define CSR_SDRAM_DFII_PI0_WRDATA_SIZE 2

#define CSR_SDRAM_DFII_PI0_RDDATA_ADDR 0xe0004020
#define CSR_SDRAM_DFII_PI0_RDDATA_SIZE 2

#define CSR_TIMER0_BASE 0xe0002000
#define CSR_TIMER0_LOAD_ADDR 0xe0002000
#define CSR_TIMER0_LOAD_SIZE 4

#define CSR_TIMER0_RELOAD_ADDR 0xe0002010
#define CSR_TIMER0_RELOAD_SIZE 4

#define CSR_TIMER0_EN_ADDR 0xe0002020
#define CSR_TIMER0_EN_SIZE 1

#define CSR_TIMER0_UPDATE_VALUE_ADDR 0xe0002024
#define CSR_TIMER0_UPDATE_VALUE_SIZE 1

#define CSR_TIMER0_VALUE_ADDR 0xe0002028
#define CSR_TIMER0_VALUE_SIZE 4

#define CSR_TIMER0_EV_STATUS_ADDR 0xe0002038
#define CSR_TIMER0_EV_STATUS_SIZE 1

#define CSR_TIMER0_EV_PENDING_ADDR 0xe000203c
#define CSR_TIMER0_EV_PENDING_SIZE 1

#define CSR_TIMER0_EV_ENABLE_ADDR 0xe0002040
#define CSR_TIMER0_EV_ENABLE_SIZE 1

#define CSR_UART_TXFULL_ADDR 0xe0001004
#define CSR_UART_TXFULL_SIZE 1

#define CSR_UART_RXEMPTY_ADDR 0xe0001008
#define CSR_UART_RXEMPTY_SIZE 1

#define CSR_UART_EV_STATUS_ADDR 0xe000100c
#define CSR_UART_EV_STATUS_SIZE 1

#define CSR_UART_EV_PENDING_ADDR 0xe0001010
#define CSR_UART_EV_PENDING_SIZE 1

#define CSR_UART_EV_ENABLE_ADDR 0xe0001014
#define CSR_UART_EV_ENABLE_SIZE 1

#define CSR_UART_PHY_BASE 0xe0000800
#define CSR_UART_PHY_TUNING_WORD_ADDR 0xe0000800
#define CSR_UART_PHY_TUNING_WORD_SIZE 4

/* Constants */

#define UART_INTERRUPT 0
#define TIMER0_INTERRUPT 1
#define SYSTEM_CLOCK_FREQUENCY 80000000
#define L2_SIZE 8192

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline unsigned char sdram_dfii_control_read(void)
{
  unsigned char r = MMPTR(0xe0004000);
  return r;
}

static inline void sdram_dfii_control_write(unsigned char value)
{
  MMPTR(0xe0004000) = value;
}

static inline unsigned char sdram_dfii_pi0_command_read(void)
{
  unsigned char r = MMPTR(0xe0004004);
  return r;
}

static inline void sdram_dfii_pi0_command_write(unsigned char value)
{
  MMPTR(0xe0004004) = value;
}

static inline unsigned char sdram_dfii_pi0_command_issue_read(void)
{
  unsigned char r = MMPTR(0xe0004008);
  return r;
}

static inline void sdram_dfii_pi0_command_issue_write(unsigned char value)
{
  MMPTR(0xe0004008) = value;
}

static inline unsigned short int sdram_dfii_pi0_address_read(void)
{
  unsigned short int r = MMPTR(0xe000400c);
  r <<= 8;
  r |= MMPTR(0xe0004010);
  return r;
}

static inline void sdram_dfii_pi0_address_write(unsigned short int value)
{
  MMPTR(0xe000400c) = value >> 8;
  MMPTR(0xe0004010) = value;
}

static inline unsigned char sdram_dfii_pi0_baddress_read(void)
{
  unsigned char r = MMPTR(0xe0004014);
  return r;
}

static inline void sdram_dfii_pi0_baddress_write(unsigned char value)
{
  MMPTR(0xe0004014) = value;
}

static inline unsigned short int sdram_dfii_pi0_wrdata_read(void)
{
  unsigned short int r = MMPTR(0xe0004018);
  r <<= 8;
  r |= MMPTR(0xe000401c);
  return r;
}

static inline void sdram_dfii_pi0_wrdata_write(unsigned short int value)
{
  MMPTR(0xe0004018) = value >> 8;
  MMPTR(0xe000401c) = value;
}

static inline unsigned short int sdram_dfii_pi0_rddata_read(void)
{
  unsigned short int r = MMPTR(0xe0004020);
  r <<= 8;
  r |= MMPTR(0xe0004024);
  return r;
}

/* Timer0 */

static inline unsigned int timer0_load_read(void)
{
  unsigned int r = MMPTR(0xe0002000);
  r <<= 8;
  r |= MMPTR(0xe0002004);
  r <<= 8;
  r |= MMPTR(0xe0002008);
  r <<= 8;
  r |= MMPTR(0xe000200c);
  return r;
}

static inline void timer0_load_write(unsigned int value)
{
  MMPTR(0xe0002000) = value >> 24;
  MMPTR(0xe0002004) = value >> 16;
  MMPTR(0xe0002008) = value >> 8;
  MMPTR(0xe000200c) = value;
}

static inline unsigned int timer0_reload_read(void)
{
  unsigned int r = MMPTR(0xe0002010);
  r <<= 8;
  r |= MMPTR(0xe0002014);
  r <<= 8;
  r |= MMPTR(0xe0002018);
  r <<= 8;
  r |= MMPTR(0xe000201c);
  return r;
}

static inline void timer0_reload_write(unsigned int value)
{
  MMPTR(0xe0002010) = value >> 24;
  MMPTR(0xe0002014) = value >> 16;
  MMPTR(0xe0002018) = value >> 8;
  MMPTR(0xe000201c) = value;
}

static inline unsigned char timer0_en_read(void)
{
  unsigned char r = MMPTR(0xe0002020);
  return r;
}

static inline void timer0_en_write(unsigned char value)
{
  MMPTR(0xe0002020) = value;
}

static inline unsigned char timer0_update_value_read(void)
{
  unsigned char r = MMPTR(0xe0002024);
  return r;
}

static inline void timer0_update_value_write(unsigned char value)
{
  MMPTR(0xe0002024) = value;
}

static inline unsigned int timer0_value_read(void)
{
  unsigned int r = MMPTR(0xe0002028);
  r <<= 8;
  r |= MMPTR(0xe000202c);
  r <<= 8;
  r |= MMPTR(0xe0002030);
  r <<= 8;
  r |= MMPTR(0xe0002034);
  return r;
}

static inline unsigned char timer0_ev_status_read(void)
{
  unsigned char r = MMPTR(0xe0002038);
  return r;
}

static inline void timer0_ev_status_write(unsigned char value)
{
  MMPTR(0xe0002038) = value;
}

static inline unsigned char timer0_ev_pending_read(void)
{
  unsigned char r = MMPTR(0xe000203c);
  return r;
}

static inline void timer0_ev_pending_write(unsigned char value)
{
  MMPTR(0xe000203c) = value;
}

static inline unsigned char timer0_ev_enable_read(void)
{
  unsigned char r = MMPTR(0xe0002040);
  return r;
}

static inline void timer0_ev_enable_write(unsigned char value)
{
  MMPTR(0xe0002040) = value;
}

/* UART */

static inline unsigned char uart_rxtx_read(void)
{
  unsigned char r = MMPTR(0xe0001000);
  return r;
}

static inline void uart_rxtx_write(unsigned char value)
{
  MMPTR(0xe0001000) = value;
}

static inline unsigned char uart_txfull_read(void)
{
  unsigned char r = MMPTR(0xe0001004);
  return r;
}

static inline unsigned char uart_rxempty_read(void)
{
  unsigned char r = MMPTR(0xe0001008);
  return r;
}

static inline unsigned char uart_ev_status_read(void)
{
  unsigned char r = MMPTR(0xe000100c);
  return r;
}

static inline void uart_ev_status_write(unsigned char value)
{
  MMPTR(0xe000100c) = value;
}

static inline unsigned char uart_ev_pending_read(void)
{
  unsigned char r = MMPTR(0xe0001010);
  return r;
}

static inline void uart_ev_pending_write(unsigned char value)
{
  MMPTR(0xe0001010) = value;
}

static inline unsigned char uart_ev_enable_read(void)
{
  unsigned char r = MMPTR(0xe0001014);
  return r;
}

static inline void uart_ev_enable_write(unsigned char value)
{
  MMPTR(0xe0001014) = value;
}

/* uart_phy */

static inline unsigned int uart_phy_tuning_word_read(void)
{
  unsigned int r = MMPTR(0xe0000800);
  r <<= 8;
  r |= MMPTR(0xe0000804);
  r <<= 8;
  r |= MMPTR(0xe0000808);
  r <<= 8;
  r |= MMPTR(0xe000080c);
  return r;
}

static inline void uart_phy_tuning_word_write(unsigned int value)
{
  MMPTR(0xe0000800) = value >> 24;
  MMPTR(0xe0000804) = value >> 16;
  MMPTR(0xe0000808) = value >> 8;
  MMPTR(0xe000080c) = value;
}

#endif /* __CONFIGS_MISOC_INCLUDE_GENERATED_CSR_H */
