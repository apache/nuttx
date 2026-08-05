/****************************************************************************
 * include/nuttx/wireless/lpwan/sx1301.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_SX1301_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_SX1301_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/lpwan/lora_gw.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SX1301 is a LoRa concentrator (gateway baseband processor) with 8
 * multi-SF demodulators, one LoRa standard demodulator (IF8) and one FSK
 * demodulator (IF9), driven by two SX125x radio front-ends accessed
 * indirectly through the SX1301 internal SPI bridge.
 *
 * Only the board glue is here: everything an application deals with is the
 * gateway interface of nuttx/wireless/lpwan/lora_gw.h.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board specific hooks.  The SX1301 needs a reset line and, on the
 * LRWAN_GS_HF1 class of shields, a pair of GPIOs selecting the RF band of
 * the front-end filters.
 */

struct sx1301_lower_s
{
  /* Drive the reset line.  'assert' true holds the chip in reset. */

  CODE void (*reset)(FAR const struct sx1301_lower_s *lower, bool assert);

  /* Select the RF band of the shield, in MHz (868 or 915).  May be NULL on
   * boards with a single band.
   */

  CODE void (*band_select)(FAR const struct sx1301_lower_s *lower,
                           int band_mhz);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: sx1301_register
 *
 * Description:
 *   Register the SX1301 concentrator character driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g. "/dev/lora0"
 *   spi     - An instance of the SPI interface wired to the SX1301
 *   lower   - Board specific reset and band selection hooks
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sx1301_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                    FAR const struct sx1301_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_SX1301_H */
