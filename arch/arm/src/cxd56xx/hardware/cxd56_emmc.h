/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_emmc.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_EMMC_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_EMMC_H

#define EMMC_CTRL           (CXD56_EMMC_BASE + 0x00u)
#define EMMC_PWREN          (CXD56_EMMC_BASE + 0x04u)
#define EMMC_CLKDIV         (CXD56_EMMC_BASE + 0x08u)
#define EMMC_CLKSRC         (CXD56_EMMC_BASE + 0x0Cu)
#define EMMC_CLKENA         (CXD56_EMMC_BASE + 0x10u)
#define EMMC_TMOUT          (CXD56_EMMC_BASE + 0x14u)
#define EMMC_CTYPE          (CXD56_EMMC_BASE + 0x18u)
#define EMMC_BLKSIZ         (CXD56_EMMC_BASE + 0x1Cu)
#define EMMC_BYTCNT         (CXD56_EMMC_BASE + 0x20u)
#define EMMC_INTMASK        (CXD56_EMMC_BASE + 0x24u)
#define EMMC_CMDARG         (CXD56_EMMC_BASE + 0x28u)
#define EMMC_CMD            (CXD56_EMMC_BASE + 0x2Cu)
#define EMMC_RESP0          (CXD56_EMMC_BASE + 0x30u)
#define EMMC_RESP1          (CXD56_EMMC_BASE + 0x34u)
#define EMMC_RESP2          (CXD56_EMMC_BASE + 0x38u)
#define EMMC_RESP3          (CXD56_EMMC_BASE + 0x3Cu)
#define EMMC_MINTSTS        (CXD56_EMMC_BASE + 0x40u)
#define EMMC_RINTSTS        (CXD56_EMMC_BASE + 0x44u)
#define EMMC_STATUS         (CXD56_EMMC_BASE + 0x48u)
#define EMMC_FIFOTH         (CXD56_EMMC_BASE + 0x4Cu)
#define EMMC_CDETECT        (CXD56_EMMC_BASE + 0x50u)
#define EMMC_WRTPRT         (CXD56_EMMC_BASE + 0x54u)
#define EMMC_GPIO           (CXD56_EMMC_BASE + 0x58u)
#define EMMC_TCBCNT         (CXD56_EMMC_BASE + 0x5Cu)
#define EMMC_TBBCNT         (CXD56_EMMC_BASE + 0x60u)
#define EMMC_DEBNCE         (CXD56_EMMC_BASE + 0x64u)
#define EMMC_USRID          (CXD56_EMMC_BASE + 0x68u)
#define EMMC_VERID          (CXD56_EMMC_BASE + 0x6Cu)
#define EMMC_HCON           (CXD56_EMMC_BASE + 0x70u)
#define EMMC_UHS_REG        (CXD56_EMMC_BASE + 0x74u)
#define EMMC_RST_N          (CXD56_EMMC_BASE + 0x78u)
#define EMMC_BMOD           (CXD56_EMMC_BASE + 0x80u)
#define EMMC_PLDMND         (CXD56_EMMC_BASE + 0x84u)
#define EMMC_DBADDR         (CXD56_EMMC_BASE + 0x88u)
#define EMMC_IDSTS          (CXD56_EMMC_BASE + 0x8Cu)
#define EMMC_IDINTEN        (CXD56_EMMC_BASE + 0x90u)
#define EMMC_DSCADDR        (CXD56_EMMC_BASE + 0x94u)
#define EMMC_BUFADDR        (CXD56_EMMC_BASE + 0x98u)
#define EMMC_CARDTHRCTL     (CXD56_EMMC_BASE + 0x100u)
#define EMMC_BACK_END_POWER (CXD56_EMMC_BASE + 0x104u)
#define EMMC_UHS_REG_EXT    (CXD56_EMMC_BASE + 0x108u)
#define EMMC_EMMC_DDR_REG   (CXD56_EMMC_BASE + 0x10Cu)
#define EMMC_ENABLE_SHIFT   (CXD56_EMMC_BASE + 0x110u)
#define EMMC_FIFO_DATA_BASE (CXD56_EMMC_BASE + 0x200u)

/* EMMC_CTRL */

#define EMMC_CTRL_USE_IDMAC  (1u << 25)
#define EMMC_CTRL_OD_PULLUP  (1u << 24)
#define EMMC_CTRL_DMA_ENABLE (1u <<  5)
#define EMMC_CTRL_INT_ENABLE (1u <<  4)
#define EMMC_CTRL_DMA_RESET  (1u <<  2)
#define EMMC_CTRL_FIFO_RESET (1u <<  1)
#define EMMC_CTRL_CTRL_RESET (1u <<  0)

/* EMMC_CLKENA */

#define EMMC_CLKENA_LOWPWR   (1u << 16)
#define EMMC_CLKENA_ENABLE   (1u <<  0)
#define EMMC_CLKENA_DIS      (EMMC_CLKENA_LOWPWR)
#define EMMC_CLKENA_ENA      (EMMC_CLKENA_LOWPWR | EMMC_CLKENA_ENABLE)

/* EMMC_TMOUT */

#define EMMC_TMOUT_DATA_SHIFT (8)
#define EMMC_TMOUT_RESP_SHIFT (0)

/* EMMC_CTYPE */

#define EMMC_CTYPE_4BIT_MODE  (1u <<  0)

/* EMMC_INTMASK */

#define EMMC_INTMASK_ALLMASK   (0)
#define EMMC_INTMASK_NO_MASK   (0x0000FFFEu)
#define EMMC_INTMASK_RXDR_MASK (~(1u <<  5) & (EMMC_INTMASK_NO_MASK))
#define EMMC_INTMASK_TXDR_MASK (~(1u <<  4) & (EMMC_INTMASK_NO_MASK))
#define EMMC_INTMASK_DTO_MASK  (~(1u <<  3) & (EMMC_INTMASK_NO_MASK))
#define EMMC_INTMASK_CD_MASK   (~(1u <<  2) & (EMMC_INTMASK_NO_MASK))

/* EMMC_CMD */

#define EMMC_CMD_START_CMD     (1u << 31)
#define EMMC_CMD_USE_HOLDREG   (1u << 29)
#define EMMC_CMD_UPDATE_CLKREG (1u << 21)
#define EMMC_CMD_SEND_INIT     (1u << 15)
#define EMMC_CMD_STOP_CMD      (1u << 14)
#define EMMC_CMD_WAIT_PRE_DATA (1u << 13)
#define EMMC_CMD_WRITE         (1u << 10)
#define EMMC_CMD_READ          (0u << 10)
#define EMMC_CMD_DATA_EXPECTED (1u <<  9)
#define EMMC_CMD_CHK_RESP_CRC  (1u <<  8)
#define EMMC_CMD_RESP_LENGTH   (1u <<  7)
#define EMMC_CMD_RESP_EXPECTED (1u <<  6)

/* EMMC_RINTSTS */

#define EMMC_INTSTS_INT_CLEAR (0xFFFFFFFFu)
#define EMMC_INTSTS_SDIO      (0xFFFF0000u)
#define EMMC_INTSTS_EBE       (1u << 15) /* End-bit error/write no CRC */
#define EMMC_INTSTS_ACD       (1u << 14) /* Auto command done */
#define EMMC_INTSTS_BCI       (1u << 13) /* Start-bit error/Busy Clear Interrupt */
#define EMMC_INTSTS_HLE       (1u << 12) /* Hardware locked write error */
#define EMMC_INTSTS_FRUN      (1u << 11) /* FIFO underrun/overrun error */
#define EMMC_INTSTS_HTO       (1u << 10) /* Data starvation-by-host timeout/Volt_switch_int */
#define EMMC_INTSTS_DRTO      (1u <<  9) /* Data read timeout/Boot Data Start */
#define EMMC_INTSTS_RTO       (1u <<  8) /* Response timeout/Boot Ack Received */
#define EMMC_INTSTS_DCRC      (1u <<  7) /* Data CRC error */
#define EMMC_INTSTS_RCRC      (1u <<  6) /* Response CRC error */
#define EMMC_INTSTS_RXDR      (1u <<  5) /* Receive FIFO data request */
#define EMMC_INTSTS_TXDR      (1u <<  4) /* Transmit FIFO data request */
#define EMMC_INTSTS_DTO       (1u <<  3) /* Data transfer over */
#define EMMC_INTSTS_CD        (1u <<  2) /* Command done */
#define EMMC_INTSTS_RE        (1u <<  1) /* Response error */
#define EMMC_INTSTS_CDET      (1u <<  0) /* Card detect */

/* EMMC_STATUS */

#define EMMC_STATUS_DATA_BUSY    (1u <<  9)

/* EMMC_FIFOTH */

#define EMMC_FIFOTH_MSIZE_SHIFT (28)
#define EMMC_FIFOTH_RX_SHIFT    (16)
#define EMMC_FIFOTH_TX_SHIFT    (0)

/* EMMC_UHS_REG */

#define EMMC_UHSREG_DDR_MODE (1u << 16)
#define EMMC_UHSREG_18_VOLT  (1u <<  0)

/* EMMC_RST_N */

#define EMMC_RST_N_HW_RESET  (0)

/* EMMC_BMOD */

#define EMMC_BMOD_IDMAC_EN    (1u <<  7)
#define EMMC_BMOD_FIXED_BURST (1u <<  1)
#define EMMC_BMOD_SW_RESET    (1u <<  0)

/* EMMC_IDSTS */

#define EMMC_IDSTS_INT_CLEAR (0x000003FFu)
#define EMMC_IDSTS_FSM       (0xFu << 13) /* DMAC FSM present state */
#define EMMC_IDSTS_EB_RESP   (1u << 11) /* This bit valids when EMMC_IDSTS_FBE */
#define EMMC_IDSTS_EB_TRANS  (1u << 10) /* This bit valids when EMMC_IDSTS_FBE */
#define EMMC_IDSTS_AIS       (1u <<  9) /* Abnormal Interrupt Summary */
#define EMMC_IDSTS_NIS       (1u <<  8) /* Normal Interrupt Summary */
#define EMMC_IDSTS_CES       (1u <<  5) /* Card Error Summary */
#define EMMC_IDSTS_DU        (1u <<  4) /* Descriptor Unavailable Interrupt */
#define EMMC_IDSTS_FBE       (1u <<  2) /* Fatal Bus Error Interrupt */
#define EMMC_IDSTS_RI        (1u <<  1) /* Receive Interrupt */
#define EMMC_IDSTS_TI        (1u <<  0) /* Transmit Interrupt */

/* EMMC_IDINTEN */

#define EMMC_IDINTEN_ALLDIS  (0)
#define EMMC_IDINTEN_ALLEN   (EMMC_IDSTS_INT_CLEAR)

/* EMMC_CARDTHRCTL */

#define EMMC_CARD_RD_THR_SHIFT (16)
#define EMMC_BSYCLR_INT_EN     (1u <<  1)
#define EMMC_CARD_RD_THE_EN    (1u <<  0)

/* EMMC_UHS_REG_EXT */

#define EMMC_EXT_CLK_MUX_CTRL_SHIFT    (30)
#define EMMC_CLK_DRV_PHASE_CTRL_SHIFT  (23)
#define EMMC_CLK_SMPL_PHASE_CTRL_SHIFT (16)

/* EMMC_EMMC_DDR_REG */

#define EMMC_DDRREG_HALF_START (1u << 0)

/* EMMC_ENABLE_SHIFT */

#define EMMC_SHIFT_NEG_EDGE  (1u <<  1)
#define EMMC_SHIFT_POS_EDGE  (1u <<  0)
#define EMMC_SHIFT_DEFAULT   (0)

/* IDMAC descriptor */

#define EMMC_IDMAC_TRANS_SIZE   (4 * 1024) /* Data trans size per chain desc */
#define EMMC_IDMAC_DES0_OWN     (1u << 31) /* Owner bit */
#define EMMC_IDMAC_DES0_CES     (1u << 30) /* Card Error Summary */
#define EMMC_IDMAC_DES0_CH      (1u <<  4) /* Second Address Chained */
#define EMMC_IDMAC_DES0_FD      (1u <<  3) /* First Descriptor */
#define EMMC_IDMAC_DES0_LD      (1u <<  2) /* Last Descriptor */
#define EMMC_IDMAC_DES0_DIC     (1u <<  1) /* Disable Interrupt on Completion */

/* Command Code */

#define GO_IDLE_STATE         0
#define SEND_OP_COND          1
#define ALL_SEND_CID          2
#define SET_RELATIVE_ADDR     3
#define SLEEP_AWAKE           5
#define SWITCH                6
#define SELECT_DESELECT       7
#define SEND_EXT_CSD          8
#define SEND_CSD              9
#define STOP_TRANS           12
#define SEND_STATUS          13
#define READ_MULTIPLE_BLOCK  18
#define SET_BLOCK_COUNT      23
#define WRITE_MULTIPLE_BLOCK 25

/* OCR */

#define OCR_1_8VOLT   (1u <<  7)
#define OCR_3_3VOLT   (0x1FFu << 15)
#define OCR_DUAL_VOLT (OCR_1_8VOLT | OCR_3_3VOLT)
#define OCR_SECTOR_MODE  (2u << 29)
#define OCR_POWER_UP (1u << 31)

/* Device Status */

#define R1STATUS_ADDRESS_OUT_OF_RANGE (1u << 31)
#define R1STATUS_ADDRESS_MISALIGE     (1u << 30)
#define R1STATUS_BLOCK_LEN_ERROR      (1u << 29)
#define R1STATUS_ERASE_SEQ_ERROR      (1u << 28)
#define R1STATUS_ERASE_PARAM          (1u << 27)
#define R1STATUS_WP_VIOLATION         (1u << 26)
#define R1STATUS_DEVICE_IS_LOCKED     (1u << 25)
#define R1STATUS_LOCK_UNLOCK_FAILED   (1u << 24)
#define R1STATUS_COM_CRC_ERROR        (1u << 23)
#define R1STATUS_ILLEGAL_COMMAND      (1u << 22)
#define R1STATUS_DEVICE_ECC_FAILED    (1u << 21)
#define R1STATUS_CC_ERROR             (1u << 20)
#define R1STATUS_ERROR                (1u << 19)

/* 18bit: Hosts should ignore this bit
 * 17bit: Hosts should ignore this bit
 */

#define R1STATUS_CID_CSD_OVERWRITE    (1u << 16)
#define R1STATUS_WP_ERASE_SKIP        (1u << 15)

/* 14bit: Reserved. Value must be 0 */

#define R1STATUS_ERASE_RESET          (1u << 13)
#define R1STATUS_CURRENT_STATE        (0xFu << 9)
#define R1STATUS_READY_FOR_DATA       (1u <<  8)
#define R1STATUS_SWITCH_ERROR         (1u <<  7)
#define R1STATUS_EXCEPTION_EVENT      (1u <<  6)
#define R1STATUS_APP_CMD              (1u <<  5)

/* 4bit: Reserved.
 * 3-2bit: Reserved for application specific command
 * 1-0bit: Reserved for manufacturer testmode
 */

#define R1STATUS_ALL_ERR                        \
  (R1STATUS_ADDRESS_OUT_OF_RANGE |              \
   R1STATUS_ADDRESS_MISALIGE      |             \
   R1STATUS_BLOCK_LEN_ERROR       |             \
   R1STATUS_ERASE_SEQ_ERROR       |             \
   R1STATUS_ERASE_PARAM           |             \
   R1STATUS_WP_VIOLATION          |             \
   R1STATUS_LOCK_UNLOCK_FAILED    |             \
   R1STATUS_COM_CRC_ERROR         |             \
   R1STATUS_ILLEGAL_COMMAND       |             \
   R1STATUS_DEVICE_ECC_FAILED     |             \
   R1STATUS_CC_ERROR              |             \
   R1STATUS_ERROR                 |             \
   R1STATUS_CID_CSD_OVERWRITE     |             \
   R1STATUS_WP_ERASE_SKIP         |             \
   R1STATUS_ERASE_RESET           |             \
   R1STATUS_SWITCH_ERROR)

#define R1STATUS_ALL_STAT                       \
  (R1STATUS_DEVICE_IS_LOCKED |                  \
   R1STATUS_CURRENT_STATE     |                 \
   R1STATUS_EXCEPTION_EVENT   |                 \
   R1STATUS_APP_CMD)

/* EMMC_R1STATUS_CURRENT_STATE */

#define R1STATUS_CURSTAT_IDLE         (0 << 9)
#define R1STATUS_CURSTAT_READY        (1 << 9)
#define R1STATUS_CURSTAT_IDENT        (2 << 9)
#define R1STATUS_CURSTAT_STDY         (3 << 9)
#define R1STATUS_CURSTAT_TRAN         (4 << 9)
#define R1STATUS_CURSTAT_DATA         (5 << 9)
#define R1STATUS_CURSTAT_RCV          (6 << 9)
#define R1STATUS_CURSTAT_PRG          (7 << 9)
#define R1STATUS_CURSTAT_DIS          (8 << 9)
#define R1STATUS_CURSTAT_BTST         (9 << 9)
#define R1STATUS_CURSTAT_SLP          (10 << 9)

/* Extended CSD */

#define EXTCSD_SIZE                 (SECTOR_SIZE)
#define EXTCSD_SECTORS              (EXTCSD_SIZE / SECTOR_SIZE)
#define EXTCSD_HC_ERASE_GRP_SIZE    (224u)
#define EXTCSD_SEC_COUNT            (212u)
#define EXTCSD_HS_TIMING            (185u) /* High Speed Timing */
#define EXTCSD_HS_TIMING_HIGH_SPEED (0x01u)
#define EXTCSD_BUS_WIDTH            (183u)
#define EXTCSD_BUS_WIDTH_4BIT_SDR   (0x01u)
#define EXTCSD_BUS_WIDTH_4BIT_DDR   (0x05u)
#define EXTCSD_PON                  (34u) /* Power Off Notification */
#define EXTCSD_PON_POWERED_ON       (0x01u)
#define EXTCSD_PON_POWERED_OFF_LONG (0x03u)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_EMMC_H */
