/******************************************************************************
 * drivers/wireless/spirit/include/spirit_management.h
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
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_MANAGEMENT_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_MANAGEMENT_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_config.h"
#include "spirit_types.h"

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_managment_wavco_calibration
 *
 * Description:
 *   Perform VCO calbration WA.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

uint8_t spirit_managment_wavco_calibration(
                                    FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_management_txstrobe
 *
 * Description:
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_management_txstrobe(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_management_rxstrobe
 *
 * Description:
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_management_rxstrobe(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_management_waextracurrent
 *
 * Description:
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_management_waextracurrent(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_management_initcommstate
 *
 * Description:
 *   Initialize communication state
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   frequency - Desired communication frequency
 *
 * Returned Value:
 *
 ******************************************************************************/

void spirit_management_initcommstate(FAR struct spirit_library_s *spirit,
                                     uint32_t frequency);

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_MANAGEMENT_H */
