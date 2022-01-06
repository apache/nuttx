/******************************************************************************
 * drivers/wireless/spirit/include/spirit_pktmbus.h
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

#ifndef __DRVIERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTMBUS_H
#define __DRVIERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTMBUS_H

/* This module can be used to manage the configuration of Spirit MBUS packets.
 * The user can obtain a packet configuration filling the structure
 * struct spirit_pktmbus_init_s, defining in it some general parameters for the
 * Spirit MBUS packet format.  Since the MBUS protocol is a standard, the
 * configuration of a MBUS* packet is very simple to do.
 *
 * Example:
 *
 * struct spirit_pktmbus_init_s mbusInit =
 * {
 *   MBUS_SUBMODE_S1_S2_LONG_HEADER,    // MBUS submode selection
 *   36,                                // added "01" chips on preamble
 *   16                                 // postamble length in "01" chips
 * };
 *
 * ...
 *
 * SpiritPktMbusInit(&mbusInit);
 *
 * ...
 *
 * The module provides some other functions that can be used to modify
 * or read only some configuration parameters.
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"
#include "spirit_pktcommon.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in assertions */

#define IS_MBUS_SUBMODE(mode) \
  (((mode) == MBUS_SUBMODE_S1_S2_LONG_HEADER) || \
   ((mode) == MBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER) || \
   ((mode) == MBUS_SUBMODE_T1_T2_METER_TO_OTHER) || \
   ((mode) == MBUS_SUBMODE_R2_SHORT_HEADER))

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Configuration and management of SPIRIT MBUS packets. */

/* Pkt MBUS Exported Types */

/* MBUS submode enumeration. */

enum spirit_mbus_submode_e
{
  /* MBUS submode S1, S2 (long header):
   * Header length = mbus_prmbl_ctrl + 279 (in "01"  bit pairs)
   * Sync word = 0x7696 (length 18 bits)
   */

  MBUS_SUBMODE_S1_S2_LONG_HEADER = MBUS_CTRL_MBUS_SUBMODE_S1_S2L,

  /* MBUS submode S1-m, S2, T2 (other to meter):
   * Header length = mbus_prmbl_ctrl + 15 (in "01" bit pairs)
   * Sync word = 0x7696 (length 18 bits)
   */

  MBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER =
                                      MBUS_CTRL_MBUS_SUBMODE_S2_S1M_T2_OTHER,

  /* MBUS submode T1, T2 (meter to other):
   * Header length = mbus_prmbl_ctrl + 19 (in "01" bit pairs):
   * Sync word = 0x3d (length 10 bits)
   */

  MBUS_SUBMODE_T1_T2_METER_TO_OTHER = MBUS_CTRL_MBUS_SUBMODE_T1_T2_METER,

  /* MBUS submode R2, short header:
   * Header length = mbus_prmbl_ctrl + 39 (in "01" bit pairs)
   * Sync word = 0x7696 (length 18 bits)
   */

  MBUS_SUBMODE_R2_SHORT_HEADER = MBUS_CTRL_MBUS_SUBMODE_R2,
};

/* SPIRIT MBUS Packet Init structure definition */

struct spirit_pktmbus_init_s
{
  enum spirit_mbus_submode_e submode; /* Specifies the SUBMODE to be
                                       * configured.  This field can be any
                                       * value from enum spirit_mbus_submode_e */
  uint8_t preamblen;                  /* Specifies the PREAMBLE length. This
                                       * parameter can be any value between
                                       * 0 and 255 chip sequence '01' */
  uint8_t postamblen;                 /* Specifies the POSTAMBLE length. This
                                       * parameter can be any value between 0
                                       * and 255  chip sequence '01' */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_pktmbus_initialize
 *
 * Description:
 *   Initializes the SPIRIT MBUS packet according to the specified parameters
 *   in the struct spirit_pktmbus_init_s structure.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   mbusinit - Pointer to a struct spirit_pktmbus_init_s structure that
 *              contains the configuration information for the specified
 *              SPIRIT MBUS PACKET FORMAT.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_initialize(FAR struct spirit_library_s *spirit,
                          FAR const struct spirit_pktmbus_init_s *mbusinit);

/******************************************************************************
 * Name: spirit_pktmbus_get_setup
 *
 * Description:
 *   Returns the SPIRIT MBUS packet structure according to the specified
 *   parameters in the registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   mbusinit - MBUS packet init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_get_setup(FAR struct spirit_library_s *spirit,
                             FAR struct spirit_pktmbus_init_s *mbusinit);

/******************************************************************************
 * Name: spirit_pktmbus_set_format
 *
 * Description:
 *   Configures the MBUS packet format as the one used by SPIRIT.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_format(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktmbus_set_preamble
 *
 * Description:
 *   Sets how many chip sequence "02" shall be added in the preamble respect
 *   to the minimum value as defined according to the specified sub-mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   preamble - The number of chip sequence.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_preamble(FAR struct spirit_library_s *spirit,
                                uint8_t preamble);

/******************************************************************************
 * Name: spirit_pktmbus_get_preamble
 *
 * Description:
 *   Returns how many chip sequence "01" are added in the preamble respect to
 *   the minimum value as defined according to the specified sub-mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Preable in number of "01" chip sequences.
 *
 ******************************************************************************/

uint8_t spirit_pktmbus_get_preamble(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktmbus_set_postamble
 *
 * Description:
 *   Sets how many chip sequence “01” will be used in postamble
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   postamble - The number of chip sequence.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_postamble(FAR struct spirit_library_s *spirit,
                                 uint8_t postamble);

/******************************************************************************
 * Name: spirit_pktmbus_get_postamble
 *
 * Description:
 *   Returns how many chip sequence "01" are used in the postamble
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Postamble in number of "01" chip sequences.
 *
 ******************************************************************************/

uint8_t spirit_pktmbus_get_postamble(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktmbus_set_submode
 *
 * Description:
 *   Sets the MBUS submode used.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   submode - The submode used.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_submode(FAR struct spirit_library_s *spirit,
                               enum spirit_mbus_submode_e submode);

/******************************************************************************
 * Name: spirit_pktmbus_get_submode
 *
 * Description:
 *   Returns the MBUS submode used.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   MBUS submode.
 *
 ******************************************************************************/

enum spirit_mbus_submode_e
  spirit_pktmbus_get_submode(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktmbus_set_payloadlen
 *
 * Description:
 *   Sets the payload length for SPIRIT MBUS packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   payloadlen - Payload length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_payloadlen(FAR struct spirit_library_s *spirit,
                                  uint16_t payloadlen);

/******************************************************************************
 * Name: spirit_pktmbus_get_payloadlen
 *
 * Description:
 *   Returns the payload length for SPIRIT MBUS packets.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Payload length in bytes.
 *
 ******************************************************************************/

uint16_t spirit_pktmbus_get_payloadlen(FAR struct spirit_library_s *spirit);

#ifdef __cplusplus
}
#endif

#endif /* __DRVIERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTMBUS_H */
