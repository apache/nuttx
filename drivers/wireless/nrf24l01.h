/****************************************************************************
 * include/nuttx/drivers/wireless/nrf24l01.h
 *
 *   Copyright (C) 2013 Laurent Latil
 *   Author: Laurent Latil <laurent@latil.nom.fr>
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

#ifndef __DRIVERS_WIRELESS_NRF24L01_H
#define __DRIVERS_WIRELESS_NRF24L01_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* nRF24L01 hardware definitions */

/* Commands */

#define NRF24L01_R_REGISTER             0x00
#define NRF24L01_W_REGISTER             0x20
#define NRF24L01_R_RX_PAYLOAD           0x61
#define NRF24L01_W_TX_PAYLOAD           0xA0
#define NRF24L01_FLUSH_TX               0xE1
#define NRF24L01_FLUSH_RX               0xE2
#define NRF24L01_REUSE_TX_PL            0xE3

#define NRF24L01_ACTIVATE               0x50
#define NRF24L01_R_RX_PL_WID            0x60
#define NRF24L01_W_TX_PAYLOAD_NOACK     0xB0
#define NRF24L01_W_ACK_PAYLOAD          0xA8
#define NRF24L01_NOP                    0xFF

/* Registers */

#define NRF24L01_CONFIG         0x00
#define NRF24L01_EN_AA          0x01
#define NRF24L01_EN_RXADDR      0x02
#define NRF24L01_SETUP_AW       0x03
#define NRF24L01_SETUP_RETR     0x04
#define NRF24L01_RF_CH          0x05
#define NRF24L01_RF_SETUP       0x06
#define NRF24L01_STATUS         0x07
#define NRF24L01_OBSERVE_TX     0x08
#define NRF24L01_CD             0x09
#define NRF24L01_RX_ADDR_P0     0x0A
#define NRF24L01_RX_ADDR_P1     0x0B
#define NRF24L01_RX_ADDR_P2     0x0C
#define NRF24L01_RX_ADDR_P3     0x0D
#define NRF24L01_RX_ADDR_P4     0x0E
#define NRF24L01_RX_ADDR_P5     0x0F
#define NRF24L01_TX_ADDR        0x10
#define NRF24L01_RX_PW_P0       0x11
#define NRF24L01_RX_PW_P1       0x12
#define NRF24L01_RX_PW_P2       0x13
#define NRF24L01_RX_PW_P3       0x14
#define NRF24L01_RX_PW_P4       0x15
#define NRF24L01_RX_PW_P5       0x16
#define NRF24L01_FIFO_STATUS    0x17
#define NRF24L01_DYNPD          0x1C
#define NRF24L01_FEATURE        0x1D

/* STATUS register definitions */

#define NRF24L01_RX_DR          (1 << 6)
#define NRF24L01_TX_DS          (1 << 5)
#define NRF24L01_MAX_RT         (1 << 4)
#define NRF24L01_RX_P_NO_SHIFT  1
#define NRF24L01_RX_P_NO_MASK   (7 << NRF24L01_RX_P_NO_SHIFT)
#define NRF24L01_STAT_TX_FULL   (1 << 0)

/* CONFIG register definitions */

#define NRF24L01_MASK_RX_DR     (1 << 6)
#define NRF24L01_MASK_TX_DS     (1 << 5)
#define NRF24L01_MASK_MAX_RT    (1 << 4)
#define NRF24L01_EN_CRC         (1 << 3)
#define NRF24L01_CRCO           (1 << 2)
#define NRF24L01_PWR_UP         (1 << 1)
#define NRF24L01_PRIM_RX        (1 << 0)

/* RF_SETUP register definition */

#define NRF24L01_CONT_WAVE      (1 << 7)
#define NRF24L01_RF_DR_LOW      (1 << 5)
#define NRF24L01_PLL_LOCK       (1 << 4)
#define NRF24L01_RF_DR_HIGH     (1 << 3)

#define NRF24L01_RF_PWR_SHIFT   1
#define NRF24L01_RF_PWR_MASK    (3 << NRF24L01_RF_PWR_SHIFT)

/* FIFO STATUS register definitions */

#define NRF24L01_TX_REUSE       (1 << 6)
#define NRF24L01_TX_FULL        (1 << 5)
#define NRF24L01_TX_EMPTY       (1 << 4)
#define NRF24L01_RX_FULL        (1 << 1)
#define NRF24L01_RX_EMPTY       (1 << 0)

/* SETUP_RETR */

#define NRF24L01_ARC_SHIFT       0
#define NRF24L01_ARC_MASK       (0xF << NRF24L01_ARC_SHIFT)

#define NRF24L01_ARD_SHIFT      4
#define NRF24L01_ARD_MASK       (0xF << NRF24L01_ARD_SHIFT)

/* OBSERVE_TX register definitions */
#define NRF24L01_PLOS_CNT_SHIFT 4
#define NRF24L01_PLOS_CNT_MASK  (0xF << NRF24L01_PLOS_CNT_SHIFT)
#define NRF24L01_ARC_CNT_SHIFT  0
#define NRF24L01_ARC_CNT_MASK   (0xF << NRF24L01_ARC_CNT_SHIFT)
#define NRF24L01_RX_P_NO_SHIFT  1
#define NRF24L01_RX_P_NO_MASK   (7 << NRF24L01_RX_P_NO_SHIFT)

/* FEATURE register definitions */

#define NRF24L01_EN_DPL          (1 << 2)
#define NRF24L01_EN_ACK_PAY      (1 << 1)
#define NRF24L01_EN_DYN_ACK      (1 << 0)

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
  {
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __DRIVERS_WIRELESS_NRF24L01_H */
