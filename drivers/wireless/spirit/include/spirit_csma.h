/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_csma.h
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

/******************************************************************************
 * Name: spirit_csma_initialize
 *
 * Description:
 *   Initializes the SPIRIT CSMA according to the specified parameters in the
 *   struct spirit_csma_init_s.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   csmainit - Reference to the Csma init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_initialize(FAR struct spirit_library_s *spirit,
                           FAR const struct spirit_csma_init_s *csmainit);

/******************************************************************************
 * Name: spirit_csma_getinfo
 *
 * Description:
 *   Returns the fitted structure struct spirit_csma_init_s starting from the
 *   registers values.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   csmainit - Csma structure to be fitted.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_getinfo(FAR struct spirit_library_s *spirit,
                        FAR struct spirit_csma_init_s *csmainit);

/******************************************************************************
 * Name: spirit_csma_enable
 *
 * Description:
 *   Enables or Disables the CSMA.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - The state of the CSMA mode.  This parameter can be: S_ENABLE
 *              or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_enable(FAR struct spirit_library_s *spirit,
                       enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_csma_getstate
 *
 * Description:
 *   Gets the CSMA mode. Says if it is enabled or disabled.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CSMA mode.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_csma_getstate(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_csma_set_persistentmode
 *
 * Description:
 *   Enables or Disables the persistent CSMA mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - The state of the persistent CSMA mode.  This parameter can
 *              be: S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_persistentmode(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_csma_get_persistentmode
 *
 * Description:
 *   Gets the persistent CSMA mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CSMA persistent mode.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_csma_get_persistentmode(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_csma_set_seedreload
 *
 * Description:
 *   Enables or Disables the seed reload mode (if enabled it reloads the back-
 *   off generator seed using the value written in the BU_COUNTER_SEED register).
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - The state of the seed reload mode.  This parameter can be:
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_seedreload(FAR struct spirit_library_s *spirit,
                               enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_csma_get_seedreload
 *
 * Description:
 *   Gets the seed reload state.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CSMA seed reload mode.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_csma_get_seedreload(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_csma_set_bucounterseed
 *
 * Description:
 *   Sets the BU counter seed (BU_COUNTER_SEED register). The CSMA back off
 *   time is given by the formula: BO = rand(2^NB)*BU.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   seed   - Seed of the random number generator used to apply the BBE
 *            algorithm.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_bucounterseed(FAR struct spirit_library_s *spirit,
                                  uint16_t seed);

/******************************************************************************
 * Name: spirit_csma_get_bucounterseed
 *
 * Description:
 *   Returns the BU counter seed (BU_COUNTER_SEED register).
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Seed of the random number generator used to apply the BBE algorithm.
 *
 ******************************************************************************/

uint16_t spirit_csma_get_bucounterseed(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_csma_set_buprescaler
 *
 * Description:
 *   Sets the BU prescaler. The CSMA back off time is given by the formula:
 *   BO = rand(2^NB)*BU.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   prescaler - Used to program the back-off unit BU.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_buprescaler(FAR struct spirit_library_s *spirit,
                                uint8_t prescaler);

/******************************************************************************
 * Name: spirit_csma_get_buprescaler
 *
 * Description:
 *   Returns the BU prescaler.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Value back-off unit (BU).
 *
 ******************************************************************************/

uint8_t spirit_csma_get_buprescaler(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_csma_set_ccaperiod
 *
 * Description:
 *   Sets the CCA period.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   multbit - Value of CCA period to store.  This parameter can be a value
 *             of enum spirit_cca_period_e.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_ccaperiod(FAR struct spirit_library_s *spirit,
                              enum spirit_cca_period_e multbit);

/******************************************************************************
 * Name: spirit_csma_get_ccaperiod
 *
 * Description:
 *   Returns the CCA period.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CCA period.
 *
 ******************************************************************************/

enum spirit_cca_period_e
  spirit_csma_get_ccaperiod(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_csma_set_ccalen
 *
 * Description:
 *   Sets the CCA length.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ccalen - The CCA length (a value between 1 and 15 that multiplies the
 *            CCA period).  This parameter can be any value of enum
 *            spirit_csmalen_e.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_ccalen(FAR struct spirit_library_s *spirit,
                           enum spirit_csmalen_e ccalen);

/******************************************************************************
 * Name: spirit_csma_get_ccalen
 *
 * Description:
 *   Returns the CCA length.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CCA length.
 *
 ******************************************************************************/

uint8_t spirit_csma_get_ccalen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_csma_set_maxbackoff
 *
 * Description:
 *   Sets the max number of back-off. If reached Spirit stops the transmission.
 *
 * Input Parameters:
 *   maxnb the max number of back-off.
 *         This parameter is an uint8_t.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_maxbackoff(FAR struct spirit_library_s *spirit,
                               uint8_t maxnb);

/******************************************************************************
 * Name: spirit_csma_get_maxbackoff
 *
 * Description:
 *   Returns the max number of back-off.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Max number of back-off.
 *
 ******************************************************************************/

uint8_t spirit_csma_get_maxbackoff(FAR struct spirit_library_s *spirit);

#ifdef __cplusplus
}
#endif

#endif /* __DRUVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_CSMA_H */
