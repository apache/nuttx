/****************************************************************************
 * drivers/wireless/ieee80211/morse/morse.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_MORSE_MORSE_H
#define __DRIVERS_WIRELESS_IEEE80211_MORSE_MORSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wireless/morse.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_IEEE80211_MORSE_SPI_FREQUENCY
#  define CONFIG_IEEE80211_MORSE_SPI_FREQUENCY 20000000
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: morse_hal_initialize
 *
 * Description:
 *   Bind the mmhal implementation (used by libmorse) to the SPI bus and to
 *   the board lower half.  Must be called before mmwlan_init().
 *
 ****************************************************************************/

int morse_hal_initialize(FAR struct spi_dev_s *spi,
                         FAR const struct morse_lower_s *lower);

/****************************************************************************
 * Name: morse_hal_has_powersave
 *
 * Description:
 *   Return true if the board connects both the WAKE and the BUSY lines,
 *   which are required to put the chip in power save.
 *
 ****************************************************************************/

bool morse_hal_has_powersave(void);

#endif /* __DRIVERS_WIRELESS_IEEE80211_MORSE_MORSE_H */
