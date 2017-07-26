/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_commands.h
 *  Management of SPIRIT Commands.
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_COMMANDS_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_COMMANDS_H

/* In this module can be found all the API used to strobe commands to
 * Spirit.  Every command strobe is an SPI transaction with a specific command
 * code.
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in debug assertions */

#define IS_SPIRIT_CMD(cmd)  (cmd == CMD_TX || \
                             cmd == CMD_RX || \
                             cmd == CMD_READY || \
                             cmd == CMD_STANDBY || \
                             cmd == CMD_SLEEP || \
                             cmd == CMD_LOCKRX || \
                             cmd == CMD_LOCKTX || \
                             cmd == CMD_SABORT || \
                             cmd == CMD_LDC_RELOAD || \
                             cmd == CMD_SEQUENCE_UPDATE || \
                             cmd == CMD_AES_ENC || \
                             cmd == CMD_AES_KEY || \
                             cmd == CMD_AES_DEC || \
                             cmd == CMD_AES_KEY_DEC || \
                             cmd == CMD_SRES || \
                             cmd == CMD_FLUSHRXFIFO || \
                             cmd == CMD_FLUSHTXFIFO \
                            )

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* SPIRIT Commands codes enumeration */

enum spirit_cmd_e
{
  CMD_TX         = COMMAND_TX,            /* Start to transmit; valid only
                                           * from READY */
  CMD_RX         = COMMAND_RX,            /* Start to receive; valid only
                                           * from READY */
  CMD_READY      = COMMAND_READY,         /* Go to READY; valid only from
                                           * STANDBY or SLEEP or LOCK */
  CMD_STANDBY    = COMMAND_STANDBY,       /* Go to STANDBY; valid only from
                                           * READY */
  CMD_SLEEP      = COMMAND_SLEEP,         /* Go to SLEEP; valid only from
                                           * READY */
  CMD_LOCKRX     = COMMAND_LOCKRX,        /* Go to LOCK state by using the
                                           * RX configuration of the synth;
                                           * valid only from READY */
  CMD_LOCKTX     = COMMAND_LOCKTX,        /* Go to LOCK state by using the
                                           * TX configuration of the synth;
                                           * valid only from READY */
  CMD_SABORT     = COMMAND_SABORT,        /* Force exit form TX or RX states
                                           * and go to READY state; valid
                                           * only from TX or RX */
  CMD_LDC_RELOAD = COMMAND_LDC_RELOAD,    /* LDC Mode: Reload the LDC
                                           * timer with the value stored
                                           * in the LDC_PRESCALER /
                                           * COUNTER registers; valid
                                           * from all states */
  CMD_SEQUENCE_UPDATE = COMMAND_SEQUENCE_UPDATE,
                                          /* Autoretransmission: Reload the
                                           * Packet sequence counter with the
                                           * value stored in the PROTOCOL[2]
                                           * register valid from all states */
  CMD_AES_ENC     = COMMAND_AES_ENC,      /* Commands: Start the encryption
                                           * routine; valid from all states;
                                           * valid from all states */
  CMD_AES_KEY     = COMMAND_AES_KEY,      /* Commands: Start the procedure
                                           * to compute the key for the
                                           * decryption; valid from all states */
  CMD_AES_DEC     = COMMAND_AES_DEC,      /* Commands: Start the decryption
                                           * routine using the current key;
                                           * valid  from all states */
  CMD_AES_KEY_DEC = COMMAND_AES_KEY_DEC,  /* Commands: Compute the key and
                                           * start the decryption; valid
                                           * from all states */
  CMD_SRES        = COMMAND_SRES,         /* Reset of all digital part,
                                           * xcept SPI registers */
  CMD_FLUSHRXFIFO = COMMAND_FLUSHRXFIFO,  /* Clean the RX FIFO; valid from
                                           * all states */
  CMD_FLUSHTXFIFO = COMMAND_FLUSHTXFIFO,  /* Clean the TX FIFO; valid from
                                           * all states */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_COMMANDS_H */
