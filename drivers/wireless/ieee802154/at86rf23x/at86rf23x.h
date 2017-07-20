/****************************************************************************
 * drivers/wireless/ieee802154/at86rf23x/at86rf23x.h
 *
 *   Copyright (C) 2016 Matt Poppe. All rights reserved.
 *   Author: Matt Poppe <matt@poppe.me>
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_AT86RF23X_AT86RF23X_H
#define __DRIVERS_WIRELESS_IEEE802154_AT86RF23X_AT86RF23X_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RF23X_SPI_REG_READ                 0x80
#define RF23X_SPI_REG_WRITE                0xc0
#define RF23X_SPI_FRAME_WRITE              0x60
#define RF23X_SPI_FRAME_READ               0x20
#define RF23X_SPI_SRAM_READ                0x00
#define RF23X_SPI_SRAM_WRITE               0x40

/* US Times Constants for the RF233 */

#define RF23X_TIME_RESET_BOOT              510
#define RF23X_TIME_FORCE_TRXOFF            100
#define RF23X_TIME_P_ON_TO_TRXOFF          510
#define RF23X_TIME_SLEEP_TO_TRXOFF         1200
#define RF23X_TIME_RESET                   6
#define RF23X_TIME_ED_MEASUREMENT          140
#define RF23X_TIME_CCA                     140
#define RF23X_TIME_PLL_LOCK                150
#define RF23X_TIME_FTN_TUNNING             25
#define RF23X_TIME_NOCLK_TO_WAKE           6
#define RF23X_TIME_CMD_FORCE_TRX_OFF       1
#define RF23X_TIME_TRXOFF_TO_PLL           180
#define RF23X_TIME_TRANSITION_PLL_ACTIVE   1
#define RF23X_TIME_TRXOFF_TO_SLEEP         1200

#define RF23X_MAX_RETRY_RESET_TO_TRX_OFF   5

#define RF23X_REG_TRXSTATUS                0x01
#define RF23X_REG_TRXSTATE                 0x02
#define RF23X_REG_TRXCTRL0                 0x03
#define RF23X_REG_TRXCTRL1                 0x04
#define RF23X_REG_TXPWR                    0x05
#define RF23X_REG_RSSI                     0x06
#define RF23X_REG_EDLEVEL                  0x07
#define RF23X_REG_CCA                      0x08
#define RF23X_REG_THRES                    0x09
#define RF23X_REG_RXCTRL                   0x0a
#define RF23X_REG_SFD                      0x0b
#define RF23X_REG_TRXCTRL2                 0x0c
#define RF23X_REG_ANT_DIV                  0x0d
#define RF23X_REG_IRQ_MASK                 0x0e
#define RF23X_REG_IRQ_STATUS               0x0f
#define RF23X_REG_VREG_CTRL                0x10
#define RF23X_REG_BATMON                   0x11
#define RF23X_REG_XOSC_CTRL                0x12
#define RF23X_REG_CCCTRL0                  0x13
#define RF23X_REG_CCCTRL1                  0x14
#define RF23X_REG_RXSYN                    0x15
#define RF23X_REG_TRXRPC                   0x16
#define RF23X_REG_XAHCTRL1                 0x17
#define RF23X_REG_FTNCTRL                  0x18
#define RF23X_REG_XAHCTRL2                 0x19
#define RF23X_REG_PLLCF                    0x1a
#define RF23X_REG_PLLDCU                   0x1b
#define RF23X_REG_PART                     0x1c
#define RF23X_REG_VERSION                  0x1d
#define RF23X_REG_MANID0                   0x1e
#define RF23X_REG_MANID1                   0x1f
#define RF23X_REG_SADDR0                   0x20
#define RF23X_REG_SADDR1                   0x21
#define RF23X_REG_PANID0                   0x22
#define RF23X_REG_PANID1                   0x23
#define RF23X_REG_IEEEADDR0                0x24
#define RF23X_REG_IEEEADDR1                0x25
#define RF23X_REG_IEEEADDR2                0x26
#define RF23X_REG_IEEEADDR3                0x27
#define RF23X_REG_IEEEADDR4                0x28
#define RF23X_REG_IEEEADDR5                0x29
#define RF23X_REG_IEEEADDR6                0x2a
#define RF23X_REG_IEEEADDR7                0x2b
#define RF23X_REG_XAHCTRL0                 0x2c
#define RF23X_REG_CSMASEED0                0x2d
#define RF23X_REG_CSMASEED1                0x2e
#define RF23X_REG_CSMABE                   0x2f
#define RF23X_REG_TSTCTRLDIGI              0x36
#define RF23X_REG_TSTAGC                   0x3c
#define RF23X_REG_SDM                      0x3d
#define RF23X_REG_PHYTXTIME                0x3b
#define RF23X_REG_PHYPMUVALUE              0x3b

#define RF23X_TRXSTATUS_POS                0
#define RF23X_TRXSTATUS_MASK               0x1f
#define RF23X_TRXSTATUS_STATUS             RF23X_REG_TRXSTATUS, RF23X_TRXSTATUS_POS, RF23X_TRXSTATUS_MASK

#define RF23X_TRXCMD_POS                   0
#define RF23X_TRXCMD_MASK                  0x1f
#define RF23X_TRXCMD_STATE                 RF23X_REG_TRXSTATE, RF23X_TRXCMD_POS, RF23X_TRXCMD_MASK

#define RF23X_TXPWR_POS_4                  0x00
#define RF23X_TXPWR_POS_3_7                0x01
#define RF23X_TXPWR_POS_3_4                0x02
#define RF23X_TXPWR_POS_3                  0x03
#define RF23X_TXPWR_POS_2_5                0x04
#define RF23X_TXPWR_POS_2                  0x05
#define RF23X_TXPWR_POS_1                  0x06
#define RF23X_TXPWR_0                      0x07
#define RF23X_TXPWR_NEG_1                  0x08
#define RF23X_TXPWR_NEG_2                  0x09
#define RF23X_TXPWR_NEG_3                  0x0a
#define RF23X_TXPWR_NEG_4                  0x0b
#define RF23X_TXPWR_NEG_6                  0x0c
#define RF23X_TXPWR_NEG_8                  0x0d
#define RF23X_TXPWR_NEG_12                 0x0e
#define RF23X_TXPWR_NEG_17                 0x0f

/* CCA_STATUS */

#define RF23X_CCA_MODE_CS_OR_ED            0x00
#define RF23X_CCA_MODE_ED                  0x01
#define RF23X_CCA_MODE_CS                  0x02
#define RF23X_CCA_MODE_CS_AND_ED           0x03

#define RF23X_CCA_CHANNEL_POS              0
#define RF23X_CCA_CHANNEL_MASK             0x1f
#define RF23X_CCA_BITS_CHANNEL             RF23X_REG_CCA, RF23X_CCA_CHANNEL_POS, RF23X_CCA_CHANNEL_MASK
#define RF23X_CCA_MODE_POS                 5
#define RF23X_CCA_MODE_MASK                0x03
#define RF23X_CCA_BITS_MODE                RF23X_REG_CCA, RF23X_CCA_MODE_POS, RF23X_CCA_MODE_MASK

/* XAH CTRL 1 */

#define RF23X_XAHCTRL1_PROM_MODE_POS       1
#define RF23X_XAHCTRL1_PROM_MODE_MASK      0x01
#define RF23X_XAHCTRL1_BITS_PROM_MODE      RF23X_REG_XAHCTRL1, RF23X_XAHCTRL1_PROM_MODE_POS, RF23X_XAHCTRL1_PROM_MODE_MASK

/* CSMA SEED 0 */

/* CSMA SEED 1 */

#define RF23X_CSMASEED1_IAMCOORD_POS       3
#define RF23X_CSMASEED1_IAMCOORD_MASK      0x1
#define RF23X_CSMASEED1_IAMCOORD_BITS      RF23X_REG_CSMASEED1, RF23X_CSMASEED1_IAMCOORD_POS, RF23X_CSMASEED1_IAMCOORD_MASK

#define RF23X_CSMASEED1_AACK_DIS_ACK_POS
#define RF23X_CSMASEED1_AACK_SET_PD_POS
#define RF23X_CSMASEED1_AACK_FVN_MODE_POS

/* TRX Status */

#define TRX_STATUS_PON                     0x00
#define TRX_STATUS_BUSYRX                  0x01
#define TRX_STATUS_BUSYTX                  0x02
#define TRX_STATUS_RXON                    0x06
#define TRX_STATUS_TRXOFF                  0x08
#define TRX_STATUS_PLLON                   0x09
#define TRX_STATUS_SLEEP                   0x0f
#define TRX_STATUS_DEEPSLEEP               0x10
#define TRX_STATUS_BUSYRXACK               0x11
#define TRX_STATUS_BUSYTXARET               0x12
#define TRX_STATUS_RXAACKON                0x16
#define TRX_STATUS_TXARETON                0x19
#define TRX_STATUS_STATEINTRANS            0x1f

/* TRX Command */

#define TRX_CMD_NOP                        0x00
#define TRX_CMD_TX                         0x02
#define TRX_CMD_FORCETRXOFF                0x03
#define TRX_CMD_FORCE_PLLON                0x04
#define TRX_CMD_RX_ON                      0x06
#define TRX_CMD_TRXOFF                     0x08
#define TRX_CMD_PLL_ON                     0x09
#define TRX_CMD_PREP_DEEPSLEEP             0x10
#define TRX_CMD_RX_AACK_ON                 0x16
#define TRX_CMD_TX_ARET_ON                 0x19

/* IRQ MASK 0x0e */

#define RF23X_IRQ_MASK_LOCK_PLL            (1 << 0)
#define RF23X_IRQ_MASK_UNLOCK_PLL          (1 << 1)
#define RF23X_IRQ_MASK_RX_START            (1 << 2)
#define RF23X_IRQ_MASK_TRX_END             (1 << 3)
#define RF23X_IRQ_MASK_CCA_ED_DONE         (1 << 4)
#define RF23X_IRQ_MASK_AMI                 (1 << 5)
#define RF23X_IRQ_MASK_TRX_UR              (1 << 6)
#define RF23X_IRQ_MASK_BAT_LOW             (1 << 7)

#define RF23X_IRQ_MASK_DEFAULT             (RF23X_IRQ_MASK_TRX_END)

#endif /* __DRIVERS_WIRELESS_IEEE802154_AT86RF23X_AT86RF23X_H */
