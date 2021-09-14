/******************************************************************************
 * drivers/wireless/spirit/include/spirit_directrf.h
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_DIRECTRF_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_DIRECTRF_H

/* This module contains functions to manage the direct Tx/Rx mode.
 * The user can choose the way to send data to Spirit through the
 * enumerative types DirectTx/DirectRx.
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used for range-checking in assertions */

#define IS_DIRECT_TX(mode) \
  (((mode) == NORMAL_TX_MODE)      || ((mode) == DIRECT_TX_FIFO_MODE) || \
   ((mode) == DIRECT_TX_GPIO_MODE) || ((mode) == PN9_TX_MODE))
#define IS_DIRECT_RX(mode) \
  (((mode) == NORMAL_RX_MODE)      || ((mode) == DIRECT_RX_FIFO_MODE) || \
   ((mode) == DIRECT_RX_GPIO_MODE))

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Direct transmission mode enumeration for SPIRIT. */

enum spirit_directtx_e
{
  NORMAL_TX_MODE      = 0x00,  /* Normal mode, no direct transmission is
                                * used */
  DIRECT_TX_FIFO_MODE = 0x04,  /* Source is FIFO: payload bits are
                                * continuously read from the TX FIFO */
  DIRECT_TX_GPIO_MODE = 0x08,  /* Source is GPIO: payload bits are
                                * continuously read from one of the GPIO
                                * ports and transmitted without any
                                * processing */
  PN9_TX_MODE         = 0x0c   /* A pseudorandom binary sequence is
                                * generated internally */
};

/* Direct receive mode enumeration for SPIRIT. */

enum spirit_directrx_e
{
  NORMAL_RX_MODE      = 0x00,  /* Normal mode, no direct reception is used */
  DIRECT_RX_FIFO_MODE = 0x10,  /* Destination is FIFO: payload bits are
                                * continuously written to the RX FIFO and
                                * not subjected to any* processing */
  DIRECT_RX_GPIO_MODE = 0x20   /* Destination is GPIO: payload bits  are
                                * continuously written to one of the GPIO
                                * ports and not subjected to any processing */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_directrf_set_rxmode
 *
 * Description:
 *   Sets the DirectRF RX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   directrx - Code of the desired mode.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_directrf_set_rxmode(FAR struct spirit_library_s *spirit,
                               enum spirit_directrx_e directrx);

/******************************************************************************
 * Name: spirit_directrf_get_rxmode
 *
 * Description:
 *   Returns the DirectRF RX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Direct Rx mode.
 *
 ******************************************************************************/

enum spirit_directrx_e
  spirit_directrf_get_rxmode(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_directrf_set_txmode
 *
 * Description:
 *   Sets the TX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   directtx - Code of the desired source.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_directrf_set_txmode(FAR struct spirit_library_s *spirit,
                               enum spirit_directtx_e directtx);

/******************************************************************************
 * Name: spirit_directrf_get_txmode
 *
 * Description:
 *   Returns the DirectRF TX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Direct Tx mode.
 *
 ******************************************************************************/

enum spirit_directtx_e
  spirit_directrf_get_txmode(FAR struct spirit_library_s *spirit);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_DIRECTRF_H */
