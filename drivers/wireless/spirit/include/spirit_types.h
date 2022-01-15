/******************************************************************************
 * drivers/wireless/spirit/include/spirit_types.h
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author:  VMA division - AMS
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_TYPES_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_TYPES_H

/* This module provide some types definitions which will be used in
 * all the modules of this library. Here is defined also the global
 * variable spirit->state which contains the status of Spirit and
 * is updated every time an SPI transaction occurs.
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "spirit_regs.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in assertions */

#define IS_SPIRIT_FUNCTIONAL_STATE(STATE) \
  (STATE == S_DISABLE || STATE == S_ENABLE)

#define IS_SPIRIT_FLAG_STATUS(STATUS) \
  (STATUS == S_RESET || STATUS == S_SET)

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Spirit Functional state. Used to enable or disable a specific option. */

enum spirit_functional_state_e
{
  S_DISABLE = 0,
  S_ENABLE  = 1
};

/* Spirit Flag status. Used to control the state of a flag. */

enum spirit_flag_status_e
{
  S_RESET = 0,
  S_SET   = 1
};

/* SPIRIT States enumeration. */

enum spirit_state_e
{
  MC_STATE_STANDBY           = 0x40,  /* STANDBY */
  MC_STATE_SLEEP             = 0x36,  /* SLEEP */
  MC_STATE_READY             = 0x03,  /* READY */
  MC_STATE_PM_SETUP          = 0x3d,  /* PM_SETUP */
  MC_STATE_XO_SETTLING       = 0x23,  /* XO_SETTLING */
  MC_STATE_SYNTH_SETUP       = 0x53,  /* SYNT_SETUP */
  MC_STATE_PROTOCOL          = 0x1f,  /* PROTOCOL */
  MC_STATE_SYNTH_CALIBRATION = 0x4f,  /* SYNTH */
  MC_STATE_LOCK              = 0x0f,  /* LOCK */
  MC_STATE_RX                = 0x33,  /* RX */
  MC_STATE_TX                = 0x5f   /* TX */
};

/* SPIRIT Status. This definition represents the single field of the SPIRIT
 *        status returned on each SPI transaction, equal also to the MC_STATE
 *        registers.
 *        This field-oriented structure allows user to address in simple way
 *        the single field of the SPIRIT status.
 *        The user shall define a variable of SpiritStatus type to access on
 *        SPIRIT status fields.
 * NOTE:  The fields order in the structure depends on used endianness
 *        (little or big endian). The actual definition is valid ONLY for
 *        LITTLE ENDIAN mode. Be sure to change opportunely the fields order
 *        when use a different endianness.
 */

#ifndef CONFIG_ENDIAN_BIG
struct spirit_status_s
{
  uint8_t XO_ON         : 1; /* Notifies if XO is operating (XO_ON is
                              * 1) or not (XO_On is 0) */
  uint8_t MC_STATE      : 7; /* Indicates the state of the Main Controller
                              * of SPIRIT. The possible states and their
                              * corresponding values are defined in enum
                              * spirit_state_e */
  uint8_t ERROR_LOCK    : 1; /* Notifies if there is an error on RCO
                              * calibration (ERROR_LOCK is 1) or not
                              * (ERROR_LOCK is 0) */
  uint8_t RX_FIFO_EMPTY : 1; /* Notifies if RX FIFO is empty (RX_FIFO_EMPTY
                              * is 1) or not (RX_FIFO_EMPTY is 0) */
  uint8_t TX_FIFO_FULL  : 1; /* Notifies if TX FIFO is full (TX_FIFO_FULL
                              * is 1) or not (TX_FIFO_FULL is 0) */
  uint8_t ANT_SELECT    : 1; /* Notifies the currently selected antenna */
  uint8_t reserved      : 4; /* Reserved and equal to 5 */
};
#endif

/* One instance of this structure represents the overall state of one Spirit
 * device from the standpoint of the library.  Multiple spirit devices may be
 * supported by the library with multiple instances of this structure.
 *
 * The caller allocates memory for the instance of struct spirit_library_s
 * and must provide these values:
 *
 *   spi            - The SPI instance to use to interface with the spirit chip
 *   xtal_frequency - The frequency of the crystal driving the spirit chip.
 *
 * All other fields should be set to zero.
 */

struct spirit_library_s
{
  FAR struct spi_dev_s *spi;         /* SPI:   Contained SPI device instance */
  uint32_t xtal_frequency;           /* RADIO: Crystal frequency */
  uint32_t commfrequency;            /* MANAGEMENT: Desired communication frequency */

  union spirit_struct_u
  {
    struct spirit_status_s state;    /* State of the Spirit device */
    uint16_t u16;                    /* For alternative accesses */
  } u;

  uint8_t commstate;                 /* MANAGEMENT: Communication state */
  uint8_t vcocalib;                  /* RADIO: Enable VCO calibration WA */
};

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_TYPES_H */
