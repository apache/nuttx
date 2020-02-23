/****************************************************************************
 * drivers/contactless/mfrc522.h
 *
 *   Copyright(C) 2016 Uniquix Ltda. All rights reserved.
 *   Authors: Alan Carvalho de Assis <acassis@gmail.com>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_CONTACTLESS_MFRC522_H
#define __DRIVERS_CONTACTLESS_MFRC522_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* The commands used by the PCD to manage communication with several PICCs
 * (ISO 14443-3, Type A, section 6.4)
 */

#define PICC_CMD_REQA    0x26 /* REQuest command, Type A */
#define PICC_CMD_WUPA    0x52 /* Wake-UP command, Type A */
#define PICC_CMD_CT      0x88 /* Cascade Tag, used during anti collision. */
#define PICC_CMD_SEL_CL1 0x93 /* Anti collision/Select, Cascade Level 1 */
#define PICC_CMD_SEL_CL2 0x95 /* Anti collision/Select, Cascade Level 2 */
#define PICC_CMD_SEL_CL3 0x97 /* Anti collision/Select, Cascade Level 3 */
#define PICC_CMD_HLTA    0x50 /* HaLT command, Type A */

/* The commands used for MIFARE Classic
 * (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
 * Use PCD_MFAuthent to authenticate access to a sector, then use these
 * commands to read/write/modify the blocks on the sector.
 * The read/write commands can also be used for MIFARE Ultralight.
 */

#define PICC_CMD_MF_AUTH_KEY_A  0x60 /* Perform authentication with Key A */
#define PICC_CMD_MF_AUTH_KEY_B  0x61 /* Perform authentication with Key B */
#define PICC_CMD_MF_READ        0x30 /* Reads one 16 byte block from auth sector */
#define PICC_CMD_MF_WRITE       0xA0 /* Writes one 16 byte block to auth senctor */
#define PICC_CMD_MF_DECREMENT   0xC0 /* Decrements contents of a block */
#define PICC_CMD_MF_INCREMENT   0xC1 /* Increments contents of a block  */
#define PICC_CMD_MF_RESTORE     0xC2 /* Reads the contents of a block */
#define PICC_CMD_MF_TRANSFER    0xB0 /* Writes the contents of a block */

/* The commands used for MIFARE Ultralight
 * (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
 * The PICC_CMD_MF_READ/_MF_WRITE can also be used for MIFARE Ultralight.
 */

#define PICC_CMD_UL_WRITE       0xA2 /* Writes one 4 byte page to the PICC. */

/* MFRC522 Registers */

/* NOTE: All SPI addresses are shifted one bit left in the SPI address byte
 * See section 8.1.2.3 from MFRC522 datasheet
 */

/* Page 0: Commands and status */
                                             /* 0x00 - reserved for future use */
#define MFRC522_COMMAND_REG      (0x01 << 1) /* starts/stops command execution */
#define MFRC522_COM_IEN_REG      (0x02 << 1) /* dis/enable int. req. ctrl bits */
#define MFRC522_DIV_IEN_REG      (0x03 << 1) /* dis/enable int. req. ctrl bits */
#define MFRC522_COM_IRQ_REG      (0x04 << 1) /* interrupt request bits */
#define MFRC522_DIV_IRQ_REG      (0x05 << 1) /* interrupt request bits */
#define MFRC522_ERROR_REG        (0x06 << 1) /* error bits status of last cmd */
#define MFRC522_STATUS1_REG      (0x07 << 1) /* communication status bits */
#define MFRC522_STATUS2_REG      (0x08 << 1) /* rcvr and transmitter status */
#define MFRC522_FIFO_DATA_REG    (0x09 << 1) /* input/output of FIFO buffer */
#define MFRC522_FIFO_LEVEL_REG   (0x0A << 1) /* number of bytes stored in the FIFO */
#define MFRC522_WATER_LEVEL_REG  (0x0B << 1) /* level for FIFO under/overflow */
#define MFRC522_CONTROL_REG      (0x0C << 1) /* miscellaneos control register */
#define MFRC522_BIT_FRAMING_REG  (0x0D << 1) /* adjustments for bit-oriented frames */
#define MFRC522_COLL_REG         (0x0E << 1) /* bit position of first bit-collision detected */
                                             /* 0x0F - reserved for future use */
/* Page 1: Commands */
                                             /* 0x10 - reserved for future use */
#define MFRC522_MODE_REG         (0x11 << 1) /* defines general modes for transmit/receive */
#define MFRC522_TX_MODE_REG      (0x12 << 1) /* defines transmission data rate and framing */
#define MFRC522_RX_MODE_REG      (0x13 << 1) /* defines reception data rate and framing */
#define MFRC522_TX_CTRL_REG      (0x14 << 1) /* controls antenna driver pins TX1 and TX2 */
#define MFRC522_TX_ASK_REG       (0x15 << 1) /* controls the setting of transmission modulation */
#define MFRC522_TX_SEL_REG       (0x16 << 1) /* selects the internal sources for antenna driver */
#define MFRC522_RX_SEL_REG       (0x17 << 1) /* selects the internal receiver settings */
#define MFRC522_RX_THLD_REG      (0x18 << 1) /* selects the thresholds for bit decoder */
#define MFRC522_DEMOD_REG        (0x19 << 1) /* defines demodulator settings */
                                             /* 0x1A - reserved for future use */
                                             /* 0x1B - reserved for future use */
#define MFRC522_MF_TX_REG        (0x1C << 1) /* controls some MIFARE comm tx param */
#define MFRC522_MF_RX_REG        (0x1D << 1) /* controls some MIFARE comm rx param */
                                             /* 0x1E - reserved for future use */
#define MFRC522_SERIAL_SPD_REG   (0x1F << 1) /* selects the speed of the serial UART */

/* Page 2: Configuration */
                                             /* 0x20 - reserved for future use */
#define MFRC522_CRC_RESULT_REGH  (0x21 << 1) /* shows the MSB values of CRC calc. */
#define MFRC522_CRC_RESULT_REGL  (0x22 << 1) /* shows the LSB values of CRC calc. */
                                             /* 0x23 - reserved for future use */
#define MFRC522_MOD_WIDTH_REG    (0x24 << 1) /* controls the ModWidth setting */
                                             /* 0x25 - reserved for future use */
#define MFRC522_RF_CFG_REG       (0x26 << 1) /* configures the receiver gain */
#define MFRC522_GSN_REG          (0x27 << 1) /* selects the conductance of n-driver TX1/2 */
#define MFRC522_CW_GSP_REG       (0x28 << 1) /* defines the conductance of p-driver during no modulation */
#define MFRC522_MOD_GSP_REG      (0x29 << 1) /* defines the conductance of p-driver during modulation */
#define MFRC522_TMODE_REG        (0x2A << 1) /* defines settings for the internal timer */
#define MFRC522_TPRESCALER_REG   (0x2B << 1) /* the lower 8 bits of TPrescaler value */
#define MFRC522_TRELOAD_REGH     (0x2C << 1) /* defines the 16-bit timer reload value */
#define MFRC522_TRELOAD_REGL     (0x2D << 1) /* defines the 16-bit timer reload value */
#define MFRC522_TCOUNT_VAL_REGH  (0x2E << 1) /* shows the 16-bit timer value */
#define MFRC522_TCOUNT_VAL_REGL  (0x2F << 1) /* shows the 16-bit timer value */

/* Page 3: Test Registers */
                                             /* 0x30 - reserved for future use */
#define MFRC522_TEST_SEL1_REG    (0x31 << 1) /* general test signal configuration */
#define MFRC522_TEST_SEL2_REG    (0x32 << 1) /* general test signal configuration */
#define MFRC522_TEST_PIN_EN_REG  (0x33 << 1) /* enables pin output driver on pins D1 to D7 */
#define MFRC522_TEST_PIN_VAL_REG (0x34 << 1) /* defines the values to D1 to D7 */
#define MFRC522_TEST_BUS_REG     (0x35 << 1) /* shows the status of the internal test bus */
#define MFRC522_AUTOTEST_REG     (0x36 << 1) /* controls the digital self test */
#define MFRC522_VERSION_REG      (0x37 << 1) /* shows the software version */
#define MFRC522_ANALOG_TEST_REG  (0x38 << 1) /* controls the pins AUX1 and AUX2 */
#define MFRC522_TEST_DAC1_REG    (0x39 << 1) /* defines the test value for TestDAC1 */
#define MFRC522_TEST_DAC2_REG    (0x3A << 1) /* defines the test value for TestDAC2 */
#define MFRC522_TEST_ADC_REG     (0x3B << 1) /* show the value of ADC I and Q channels */

/* Section 9.3.1.2: MFRC522 Command Register */

#define MFRC522_CMD_MASK           0x0F
#  define MFRC522_IDLE_CMD         0x00 /* no action, cancels current command execution */
#  define MFRC522_MEM_CMD          0x01 /* stores 25 bytes into the internal buffer */
#  define MFRC522_GEN_RND_ID_CMD   0x02 /* generates a 10-bytes random ID number */
#  define MFRC522_CALC_CRC_CMD     0x03 /* activates the CRC coprocessor or self test */
#  define MFRC522_TRANSMIT_CMD     0x04 /* transmits data from the FIFO buffer */
#  define MFRC522_NO_CHANGE_CMD    0x07 /* no command change, used to modify CommandReg */
#  define MFRC522_RECEIVE_CMD      0x08 /* activates the receiver circuits */
#  define MFRC522_TRANSCV_CMD      0x0C /* transmits data from FIFO and receive automatically */
#  define MFRC522_MF_AUTH_CMD      0x0E /* performs the MIFARE stand authentication as a reader */
#  define MFRC522_SOFTRST_CMD      0x0F /* resets the MFRC522 */
#define MFRC522_POWER_DOWN         (1 << 4) /* soft power-down mode entered */
#define MFRC522_RCV_OFF            (1 << 5) /* turns off analog part of receiver */

/* Section 9.3.1.3: ComIEnReg register */

#define MFRC522_TIMER_IEN          (1 << 0) /* allows the timer interrupt request on pin IRQ */
#define MFRC522_ERR_IEN            (1 << 1) /* allows the error interrupt request on pin IRQ */
#define MFRC522_LO_ALERT_IEN       (1 << 2) /* allows the low alert interrupt request on pin IRQ */
#define MFRC522_HI_ALERT_IEN       (1 << 3) /* allows the high alert interrupt request on pin IRQ */
#define MFRC522_IDLE_IEN           (1 << 4) /* allows the idle interrupt request on pin IRQ */
#define MFRC522_RX_IEN             (1 << 5) /* allows the receiver interrupt request on pin IRQ */
#define MFRC522_TX_IEN             (1 << 6) /* allows the transmitter interrupt request on pin IRQ */
#define MFRC522_IRQ_INV            (1 << 7) /* signal on pin IRQ is inverse of IRq bit from Status1Reg */

/* Section 9.3.1.4: DivIEnReg register */

#define MFRC522_CRC_IEN            (1 << 2) /* allows the CRC interrupt request on pin IRQ */
#define MFRC522_MFIN_ACT_IEN       (1 << 4) /* allows the MFIN active interrupt request on pin IRQ */
#define MFRC522_IRQ_PUSH_PULL      (1 << 7) /* 1 = IRQ pin is a standard CMOS output pin, 0 = open-drain */

/* Section 9.3.1.5: ComIrqReg register */

#define MFRC522_COM_IRQ_MASK       (0x7F)
#define MFRC522_TIMER_IRQ          (1 << 0) /* enabled when TCounterValReg reaches value 0 */
#define MFRC522_ERR_IRQ            (1 << 1) /* any error bit in the ErrorReg register is set */
#define MFRC522_LO_ALERT_IRQ       (1 << 2) /* Status1Reg register’s LoAlert bit is set */
#define MFRC522_HI_ALERT_IRQ       (1 << 3) /* Status1Reg register’s HiAlert bit is set */
#define MFRC522_IDLE_IRQ           (1 << 4) /* if a command terminates this bit is set */
#define MFRC522_RX_IRQ             (1 << 5) /* receiver has detected the end of a valid data stream */
#define MFRC522_TX_IRQ             (1 << 6) /* set immediately after the last data bit was transmitted */
#define MFRC522_SET1               (1 << 7) /* indicate the status of ComIrqReg bits */

/* Section 9.3.1.6: DivIrqReg register */

#define MFRC522_CRC_IRQ            (1 << 2) /* the CalcCRC command is active and all data is processed */
#define MFRC522_MFIN_ACT_IRQ       (1 << 4) /* MFIN is active, int is set on rising/falling signal edge */
#define MFRC522_SET2               (1 << 7) /* indicates the status of the marked bits in the DivIrqReg */

/* Section 9.3.1.7: ErrorReg register */

#define MFRC522_PROTO_ERR       (1 << 0) /* set if the SOF is incorrect or during MFAuthent if data is incorrect */
#define MFRC522_PARITY_ERR         (1 << 1) /* parity check failed */
#define MFRC522_CRC_ERR            (1 << 2) /* the RxCRCEn bit is set and the CRC calculation fails */
#define MFRC522_COLL_ERR           (1 << 3) /* a bit-collision is detected */
#define MFRC522_BUF_OVFL_ERR    (1 << 4) /* FIFO is full and the host or internal state machine try to write data */
#define MFRC522_TEMP_ERR           (1 << 6) /* internal temperature sensor detects overheating */
#define MFRC522_WR_ERR             (1 << 7) /* data write error in the FIFO, host writing to FIFO at the wrong time */

/* Section 9.3.1.8: Status1Reg register */

#define MFRC522_LO_ALERT           (1 << 0) /* number of bytes on FIFO lower than the water-mark */
#define MFRC522_HI_ALERT           (1 << 1) /* number of bytes on FIFO higher than the water-mark */
#define MFRC522_TRUNNING           (1 << 3) /* timer is running */
#define MFRC522_IRQ                (1 << 4) /* indicates if any interrupt source requests attention */
#define MFRC522_CRC_READY          (1 << 5) /* the CRC calculation has finished */
#define MFRC522_CRC_OK             (1 << 6) /* when the calculation is done correctly this bit change to 1 */

/* Section 9.3.1.9: Status2Reg register */

#define MFRC522_MODEM_STATE_MASK   (7 << 0) /* shows the state of the transmitter and receiver state machine */
#define MFRC522_MODEM_IDLE         (0)      /* idle */
#define MFRC522_MODEM_WAIT_BFR     (1)      /* wait for the BitFramingReg register’s StartSend bit */
#define MFRC522_MODEM_TXWAIT       (2)      /* wait until RF field is present if TxWaitRF bit is set to 1 */
#define MFRC522_MODEM_TXING        (3)      /* transmitting */
#define MFRC522_MODEM_RXWAIT       (4)      /* wait until RF field is present if TxWaitRF bit is set to 1 */
#define MFRC522_MODEM_WAIT_DATA    (5)      /* wait for data */
#define MFRC522_MODEM_RXING        (6)      /* receiving */
#define MFRC522_MF_CRYPTO1_ON      (1 << 3) /* MIFARE Crypto1 unit is switched on */
#define MFRC522_I2C_FORCE_HS       (1 << 6) /* set the I2C to high-speed mode (R/W bit) */
#define MFRC522_TEMP_SENS_CLEAR    (1 << 7) /* clears the temperature error if it is below 125C (R/W bit) */

/* Section 9.3.1.10: FIFODataReg register */

#define MFRC522_FIFO_DATA_MASK     (0xFF)   /* Input and output of 64 byte FIFO buffer */

/* Section 9.3.1.11: FIFOLevelReg register */

#define MFRC522_FIFOLEVEL_MASK     (0x7F)   /* indicates the number of bytes stored in the FIFO buffer */
#define MFRC522_FLUSH_BUFFER       (1 << 7) /* immediately clears the internal FIFO buffer */

/* Section 9.3.1.12: WaterLevelReg register */

#define MFRC522_WATER_LEVEL_MASK   (0x3F)   /* level for FIFO under- and overflow warning */

/* Section 9.3.1.13: ControlReg register */

#define MFRC522_RX_LAST_BITS_MASK  (7 << 0) /* indicates the number of valid bits in the last received byte */
#define MFRC522_TSTART_NOW         (1 << 6) /* timer starts immediately */
#define MFRC522_TSTOP_NOW          (1 << 7) /* timer stops immediately */

/* Section 9.3.1.14: BitFramingReg register */

#define MFRC522_TX_LAST_BITS_MASK  (7 << 0) /* defines the number of bits of the last byte that will be transmitted */
#define MFRC522_RX_ALIGN_MASK      (7 << 4) /* used for reception of bit-oriented frames */
#define MFRC522_START_SEND         (1 << 7) /* starts the transmission of data */

/* Section 9.3.1.15: CollReg register */

#define MFRC522_COLL_POS_MASK      (0x1F)   /* shows the bit position of the first detected collision */
#define MFRC522_COLL_POS_NOT_VALID (1 << 5) /* no collision detected or it is out of the range of CollPos[4:0] */
#define MFRC522_VALUES_AFTER_COLL  (1 << 7) /* 0 means: all received bits will be cleared after a collision */

/* Section 9.3.2.2: ModeReg register */

#define MFRC522_CRC_PRESET_MASK    (0x3) /* defines the preset value for the CalcCRC */
#define MFRC522_CRC_PRESET_0000    (0x0) /* 0000h CRC preset value */
#define MFRC522_CRC_PRESET_6363    (0x1) /* 6363h CRC preset value */
#define MFRC522_CRC_PRESET_A671    (0x2) /* A671h CRC preset value */
#define MFRC522_CRC_PRESET_FFFF    (0x3) /* FFFFh CRC preset value */
#define MFRC522_POL_MFIN           (1 << 3) /* defines the polarity of pin MFIN */
#define MFRC522_TX_WAIT_RF         (1 << 5) /* transmitter can only be started if an RF field is generated */
#define MFRC522_MSB_FIRST          (1 << 7) /* CRC coprocessor calculates the CRC with MSB first */

/* Section 9.3.2.3: TxModeReg register */

#define MFRC522_INV_MOD             (1 << 3) /* modulation of transmitted data is inverted */
#define MFRC522_TX_SPEED_MASK       (7 << 4) /* defines the bit rate during data transmission */
#define MFRC522_TX_106KBD           (0 << 4) /* 106 kBd */
#define MFRC522_TX_212KBD           (1 << 4) /* 212 kBd */
#define MFRC522_TX_424KBD           (2 << 4) /* 424 kBd */
#define MFRC522_TX_848KBD           (3 << 4) /* 848 kBd */
                                             /* 4-7 << 4 - reserved */
#define MFRC522_TX_CRC_EN           (1 << 7) /* enables CRC generation during data transmission */

/* Section 9.3.2.4: RxModeReg register */

#define MFRC522_RX_MULTIPLE         (1 << 2) /* enable to receive more than one data frame, only at 106kBd */
#define MFRC522_RX_NO_ERR           (1 << 3) /* ignore invalid data stream error (less than 4 bits received) */
#define MFRC522_RX_SPEED_MASK       (7 << 4) /* defines the bit rate during data reception */
#define MFRC522_RX_106KBD           (0 << 4) /* 106 kBd */
#define MFRC522_RX_212KBD           (1 << 4) /* 212 kBd */
#define MFRC522_RX_424KBD           (2 << 4) /* 424 kBd */
#define MFRC522_RX_848KBD           (3 << 4) /* 848 kBd */
                                             /* 4-7 << 4 - reserved */
#define MFRC522_RX_CRC_EN           (1 << 7) /* enables CRC generation during data reception */

/* Section 9.3.2.5: TxControlReg register */

#define MFRC522_TX1_RF_EN          (1 << 0) /* output signal on pin TX1 delivers 13.56MHz */
#define MFRC522_TX2_RF_EN          (1 << 1) /* output signal on pin TX2 delivers 13.56MHz */
                                            /* bit 2 - reserved */
#define MFRC522_TX2_CW             (1 << 3) /* output signal on pin TX2 delivers (un)modulated 13.56MHz */
#define MFRC522_INV_TX1_RF_OFF     (1 << 4) /* output signal on pin TX1 is inverted when driver TX1 is disabled */
#define MFRC522_INV_TX2_RF_OFF     (1 << 5) /* output signal on pin TX2 is inverted when driver TX2 is disabled */
#define MFRC522_INV_TX1_RF_ON      (1 << 6) /* output signal on pin TX1 is inverted when driver TX1 is enabled */
#define MFRC522_INV_TX2_RF_ON      (1 << 7) /* output signal on pin TX2 is inverted when driver TX2 is enabled */

/* Section 9.3.2.6: TxASKReg register */

#define MFRC522_FORCE_100ASK       (1 << 6) /* forces a 100% ASK modulation independent of the ModGsPReg setting */

/* Section 9.3.2.7: TxSelReg register */

#define MFRC522_MFOUT_SEL_MASK     (0xF)    /* selects the input for pin MFOUT */
#define MFRC522_MFOUT_3STATE       (0)      /* 3-state */
#define MFRC522_MFOUT_LOW          (1)      /* constant Low */
#define MFRC522_MFOUT_HIGH         (2)      /* constant High */
#define MFRC522_MFOUT_TEST_BUS     (3)      /* test bus signal as defined by the TstBusBitSel[2:0] value */
#define MFRC522_MFOUT_INT_ENV      (4)      /* modulation signal (envelope) from the internal encoder */
#define MFRC522_MFOUT_TX_STREAM    (5)      /* serial data stream to be transmitted, data stream before Miller encoder */
                                            /* 6 - reserved */
#define MFRC522_MFOUT_RX_STREAM    (7)      /* serial data stream received, data stream after Manchester decoder */
                                            /* 8-15 - reserved */
#define MFRC522_DRV_SEL_MASK       (3 << 4) /* selects the input of drivers TX1 and TX2 */
#define MFRC522_DRV_SEL_3STATE     (0 << 4) /* 3-state */
#define MFRC522_DRV_SEL_INT_ENV    (1 << 4) /* modulation signal (envelope) from the internal encoder */
#define MFRC522_DVR_SEL_ENV_MFIN   (2 << 4) /* modulation signal (envelope) from pin MFIN */
#define MFRC522_DVR_SEL_HIGH       (3 << 4) /* High: depends on InvTx1RFOn/InvTx1RFOff and InvTx2RFOn/InvTx2RFOff */

/* Section 9.3.2.8: RxSelReg register */

#define MFRC522_RX_WAIT_MASK       (0x3F)   /* delay the receiver RxWait bit-clocks after transmission */
#define MFRC522_UART_SEL_MASK      (3 << 6) /* selects the input of the contactless UART */
#define MFRC522_UART_LOW           (0 << 6) /* constant Low */
#define MFRC522_UART_MANCHESTER    (1 << 6) /* Manchester with subcarrier from pin MFIN */
#define MFRC522_UART_INT_MOD       (2 << 6) /* modulated signal from the internal analog module, default */
#define MFRC522_UART_NRZ_CODE      (3 << 6) /* NRZ coding without subcarrier from pin MFIN */

/* Section 9.3.2.9: RxThresholdReg register */

#define MFRC522_COLL_LEVEL_MASK    (7) /* the minimum signal strength to generate a bit-collision */
#define MFRC522_MIN_LEVEL_MASK     (0xF << 4) /* the minimum signal strength that will be accepted */

/* Section 9.3.2.10: DemodReg register */

#define MFRC522_TAU_SYNC_MASK      (3 << 0) /* changes the time-constant of the internal PLL during burst */
#define MFRC522_TAU_RCV_MASK       (3 << 2) /* changes the time-constant of the internal PLL during data reception */
#define MFRC522_TPRESCAL_EVEN      (1 << 4) /* defines the Timer Prescaler formula to use */
#define MFRC522_FIX_IQ             (1 << 5) /* defines if reception will be fixed at channel I or Q based on AddIQ[1:0] */
#define MFRC522_ADD_IQ_MASK        (3 << 6) /* defines the use of I and Q channel during reception */

/* Section 9.3.2.13: MfTxReg register */

#define MFRC522_MF_TX_WAIT_MASK    (3 << 0) /* defines the additional response time */

/* Section 9.3.2.14 MfRxReg register */

#define MFRC522_MF_RX_PARITY_DIS   (1 << 4 ) /* disable parity bit to transmission and reception */

/* Section 9.3.2.16: SerialSpeedReg register */

#define MFRC522_BR_T1_MASK         (0x1F)   /* factor BR_T1 adjusts the transfer speed */
#define MFRC522_BR_T0_MASK         (7 << 5) /* factor BR_T0 adjusts the transfer speed */

/* Section 9.3.3.6: RFCfgReg register */

#define MFRC522_RX_GAIN_MASK       (0x7 << 4)
#define MFRC522_RX_GAIN_18DB       (0x0 << 4)
#define MFRC522_RX_GAIN_23DB       (0x1 << 4)
#define MFRC522_RX_GAIN_18DB_2     (0x2 << 4)
#define MFRC522_RX_GAIN_23DB_2     (0x3 << 4)
#define MFRC522_RX_GAIN_33DB       (0x4 << 4)
#define MFRC522_RX_GAIN_38DB       (0x5 << 4)
#define MFRC522_RX_GAIN_43DB       (0x6 << 4)
#define MFRC522_RX_GAIN_48DB       (0x7 << 4)

/* MFRC522 TModeReg and TPrescalerReg registers */

#define MFRC522_TPRESCALER_HI_MASK (0xF)
#define MFRC522_TAUTO_RESTART      (1 << 4)
#define MFRC522_TGATED_MASK        (3 << 5)
#define MFRC522_TGATED_NONGATED    (0 << 5) /* non-gated mode */
#define MFRC522_TGATED_MFIN        (1 << 5) /* gated by pin MFIN */
#define MFRC522_TGATED_AUX1        (2 << 5) /* gated by pin AUX1 */
#define MFRC522_TAUTO              (1 << 7) /* timer starts automatically at the end of the transmission */

/* MFRC522 AutoTestReg register */

#define MFRC522_SELFTEST_MASK      (0xF)    /* for default operation the self test must be disabled by value 0000b */
#define MFRC522_RFT_MASK           (3 << 4) /* reserved for production tests */
#define MFRC522_AMP_RCV            (1 << 6) /* non-linear signal processing mode, increase range distance at 106kBd */

#define MFRC522_SELFTEST_EN        9        /* the self test is enabled by value 1001b */

#ifndef CONFIG_MFRC522_SPI_FREQ
#  define CONFIG_MFRC522_SPI_FREQ  (5000000)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mfrc522_dev_s
{
  uint8_t state;
  FAR struct spi_dev_s *spi;          /* SPI interface */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool mfrc522_set_config(struct mfrc522_dev_s *dev, uint8_t flags);

#endif /* __DRIVERS_CONTACTLESS_MFRC522_H */
