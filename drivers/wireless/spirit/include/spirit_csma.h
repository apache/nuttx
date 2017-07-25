/******************************************************************************
 * include/nuttx/wireless/spirit/spirit_csma.h
 * Configuration and management of SPIRIT CSMA.
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

#ifndef __DRUVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_CSMA_H
#define __DRUVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_CSMA_H

/* The Spirit CSMA feature, when configured and enabled, is transparent
 * for the user. It means the user has only to call the spirit_csma_initialize()
 * function on a filled structure and then enable the CSMA policy using the
 * SpiritCsma() function.
 *
 * Example:
 *
 * struct spirit_csma_init_s g_csma_init =
 * {
 *   0xFA21,            # BU counter seed
 *   S_DISABLE,         # persistent mode
 *   TBIT_TIME_64,      # Tbit time
 *   TCCA_TIME_3,       # Tcca time
 *   5,                 # max number of backoffs
 *   32                 # CU prescaler
 * };
 *
 * ...
 *
 * spirit_csma_initialize(spirit, &g_csma_init);
 * spirit_csma(spirit, S_ENABLE);
 *
 * NOTE: The CS status depends of the RSSI threshold set. Please see
 * spirit_qi.c for details.
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"
#include "spirit_regs.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Csma_Parameters */

#define IS_BU_COUNTER_SEED(seed)   (seed!=0)
#define IS_BU_PRESCALER(prescaler) (prescaler < 64)
#define IS_CMAX_NB(nb)             (nb < 8)

/* Macros used in assertions */

#define IS_CCA_PERIOD(period) \
  ((period) == TBIT_TIME_64  || (period) == TBIT_TIME_128 || \
   (period) == TBIT_TIME_256 || (period) == TBIT_TIME_512)
#define IS_CSMA_LENGTH(len) \
  ((len) == TCCA_TIME_0      || (len) == TCCA_TIME_1      || \
   (len) == TCCA_TIME_2      || (len) == TCCA_TIME_3      || \
   (len) == TCCA_TIME_4      || (len) == TCCA_TIME_5      || \
   (len) == TCCA_TIME_6      || (len) == TCCA_TIME_7      || \
   (len) == TCCA_TIME_8      || (len) == TCCA_TIME_9      || \
   (len) == TCCA_TIME_10     || (len) == TCCA_TIME_11     || \
   (len) == TCCA_TIME_12     || (len) == TCCA_TIME_13     || \
   (len) == TCCA_TIME_14     || (len) == TCCA_TIME_15)

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Multiplier for Tcca time enumeration (Tcca = Multiplier*Tbit). */

enum spirit_cca_period_e
{
  TBIT_TIME_64  = CSMA_CCA_PERIOD_64TBIT,    /* CSMA/CA: Sets CCA period
                                             * to 64*TBIT */
  TBIT_TIME_128 = CSMA_CCA_PERIOD_128TBIT,  /* CSMA/CA: Sets CCA period
                                             * to 128*TBIT */
  TBIT_TIME_256 = CSMA_CCA_PERIOD_256TBIT,  /* CSMA/CA: Sets CCA period
                                             * to 256*TBIT */
  TBIT_TIME_512 = CSMA_CCA_PERIOD_512TBIT,  /* CSMA/CA: Sets CCA period
                                             * to 512*TBIT */
};

/* Multiplier of Tcca time enumeration to obtain Tlisten (Tlisten = [1...15]*Tcca). */

enum spirit_csmalen_e
{
  TCCA_TIME_0  = 0x00,      /* CSMA/CA: Sets CCA length to 0 */
  TCCA_TIME_1  = 0x10,      /* CSMA/CA: Sets CCA length to 1*TLISTEN */
  TCCA_TIME_2  = 0x20,      /* CSMA/CA: Sets CCA length to 2*TLISTEN */
  TCCA_TIME_3  = 0x30,      /* CSMA/CA: Sets CCA length to 3*TLISTEN */
  TCCA_TIME_4  = 0x40,      /* CSMA/CA: Sets CCA length to 4*TLISTEN */
  TCCA_TIME_5  = 0x50,      /* CSMA/CA: Sets CCA length to 5*TLISTEN */
  TCCA_TIME_6  = 0x60,      /* CSMA/CA: Sets CCA length to 6*TLISTEN */
  TCCA_TIME_7  = 0x70,      /* CSMA/CA: Sets CCA length to 7*TLISTEN */
  TCCA_TIME_8  = 0x80,      /* CSMA/CA: Sets CCA length to 8*TLISTEN */
  TCCA_TIME_9  = 0x90,      /* CSMA/CA: Sets CCA length to 9*TLISTEN */
  TCCA_TIME_10 = 0xa0,      /* CSMA/CA: Sets CCA length to 10*TLISTEN */
  TCCA_TIME_11 = 0xb0,      /* CSMA/CA: Sets CCA length to 11*TLISTEN */
  TCCA_TIME_12 = 0xc0,      /* CSMA/CA: Sets CCA length to 12*TLISTEN */
  TCCA_TIME_13 = 0xd0,      /* CSMA/CA: Sets CCA length to 13*TLISTEN */
  TCCA_TIME_14 = 0xe0,      /* CSMA/CA: Sets CCA length to 14*TLISTEN */
  TCCA_TIME_15 = 0xf0,      /* CSMA/CA: Sets CCA length to 15*TLISTEN */
};

/* SPIRIT CSMA Init structure definition */

struct spirit_csma_init_s
{
  uint16_t seed;            /* Specifies the BU counter seed. Not used
                              * in persistent mode. */
  uint8_t csmapersistent;   /* Specifies if the CSMA persistent mode
                             * has to be on or off. This parameter can be
                             * S_ENABLE or S_DISABLE */
  uint8_t multbit;          /* Specifies the Tbit multiplier to obtain the
                             * Tcca. This parameter can be a value from
                             * enum spirit_cca_period_e */
  uint8_t ccalen;           /* Specifies the Tcca multiplier to determine
                             * the Tlisten. This parameter can be a value
                             * from enum spirit_csmalen_e. */
  uint8_t maxnb;            /* Specifies the max number of backoff
                             * cycles. Not used in persistent mode. */
  uint8_t prescaler;        /* Specifies the BU prescaler. Not used in
                              * persistent mode. */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __DRUVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_CSMA_H */
