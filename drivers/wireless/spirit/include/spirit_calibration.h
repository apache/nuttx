/******************************************************************************
 * include/nuttx/wireless/spirit/spirit_calibration.h
 * Configuration and management of SPIRIT VCO-RCO calibration.
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

#ifndef __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_CALIBRAITON_H
#define __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_CALIBRAITON_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_config.h"
#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in assertions */

#define IS_VCO_SEL(ref)  ((ref) == VCO_L  || (ref) == VCO_H)

/******************************************************************************
 * Public Types
 ******************************************************************************/

/* VCO_H / VCO_L selection. */

enum spirit_vcoselect_e
{
  VCO_L = 0,             /* VCO lower */
  VCO_H                  /* VCO higher */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_calib_enable_rco
 *
 * Description:
 *   Enables or Disables the RCO calibration.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for RCO calibration.  This parameter can be
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_enable_rco(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_calib_enable_vco
 *
 * Description:
 *   Enables or Disables the VCO calibration.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for VCO calibration. This parameter can be
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_enable_vco(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_calib_get_vcocal
 *
 * Description:
 *   Returns the VCO calibration data from internal VCO calibrator.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   VCO calibration data byte.
 *
 ******************************************************************************/

uint8_t spirit_calib_get_vcocal(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_calib_set_vcotxcal
 *
 * Description:
 *   Sets the VCO calibration data to be used in TX mode.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   caldata - Calibration data word to be set.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_set_vcotxcal(FAR struct spirit_library_s *spirit,
                               uint8_t caldata);

/******************************************************************************
 * Name: spirit_calib_get_vcotxcal
 *
 * Description:
 *   Returns the actual VCO calibration data used in TX mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   VCO calibration data used in TX mode
 *
 ******************************************************************************/

uint8_t spirit_calib_get_vcotxcal(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_calib_set_vcorxcal
 *
 * Description:
 *   Sets the VCO calibration data to be used in RX mode.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   caldata - Calibration data word to be set.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_set_vcorxcal(FAR struct spirit_library_s *spirit,
                              uint8_t caldata);

/******************************************************************************
 * Name: spirit_calib_get_vcorxcal
 *
 * Description:
 *   Returns the actual VCO calibration data used in RX mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Calibration data word used in RX mode.
 *
 ******************************************************************************/

uint8_t spirit_calib_get_vcorxcal(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_calib_select_vco
 *
 * Description:
 *   Selects a VCO.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   vco    - Can be VCO_H or VCO_L according to which VCO select.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_select_vco(FAR struct spirit_library_s *spirit,
                            enum spirit_vcoselect_e vco);

/******************************************************************************
 * Name: spirit_calib_get_vco
 *
 * Description:
 *   Returns the VCO selected.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   VCO_H or VCO_L according to which VCO selected.
 *
 ******************************************************************************/

enum spirit_vcoselect_e spirit_calib_get_vco(FAR struct spirit_library_s *spirit);

#endif /* __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_CALIBRAITON_H */
