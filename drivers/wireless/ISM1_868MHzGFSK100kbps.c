/****************************************************************************
 * drivers/wireless/ISM1_868MHzGFSK100kbps.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/cc1101.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Settings for 868 MHz, GFSK at 100kbps
 *
 *  ISM Region 1 (Europe) only, Band 868–870 MHz
 *
 * Frequency          ERP         Duty Cycle  Bandwidth  Remarks
 * 868 – 868.6 MHz    +14 dBm     < 1%        No limits
 * 868.7 – 869.2 MHz  +14 dBm     < 0.1%      No limits
 * 869.3 – 869.4 MHz  +10 dBm     No limits   < 25 kHz   Appropriate access
 *                                                       protocol required
 * 869.4 – 869.65 MHz +27 dBm     < 10%       < 25 kHz   Channels may be
 *                                                       combined to one
 *                                                       high speed channel
 * 869.7 -870 MHz     +7 dBm      No limits   No limits
 *
 *   Deviation = 46.142578
 *   Base frequency = 867.999985
 *   Carrier frequency = 867.999985
 *   Channel number = 0
 *   Carrier frequency = 867.999985
 *   Modulated = true
 *   Modulation format = GFSK
 *   Manchester enable = false
 *   Sync word qualifier mode = 30/32 sync word bits detected
 *   Preamble count = 4
 *   Channel spacing = 199.813843
 *   Carrier frequency = 867.999985
 *   Data rate = 99.9069
 *   RX filter BW = 210.937500
 *   Data format = Normal mode
 *   Length config = Fixed packet length mode.
 *                   Length configured in PKTLEN register
 *   CRC enable = true
 *   Packet length = 62
 *   Device address = 00
 *   Address config = NO Address check, no broadcast
 *   CRC autoflush = true
 *   PA ramping = false
 *   TX power = 0
 */

const struct c1101_rfsettings_s cc1101_rfsettings_ISM1_868MHzGFSK100kbps =
{
  .FSCTRL1  = 0x08,    /* FSCTRL1       Frequency Synthesizer Control */
  .FSCTRL0  = 0x00,    /* FSCTRL0       Frequency Synthesizer Control */

  .FREQ2    = 0x20,    /* FREQ2         Frequency Control Word, High Byte */
  .FREQ1    = 0x25,    /* FREQ1         Frequency Control Word, Middle Byte */
  .FREQ0    = 0xed,    /* FREQ0         Frequency Control Word, Low Byte */

  .MDMCFG4  = 0x8b,    /* MDMCFG4       Modem Configuration */
  .MDMCFG3  = 0xe5,    /* MDMCFG3       Modem Configuration */
  .MDMCFG2  = 0x13,    /* MDMCFG2       Modem Configuration */
  .MDMCFG1  = 0x22,    /* MDMCFG1       Modem Configuration */
  .MDMCFG0  = 0xe5,    /* MDMCFG0       Modem Configuration */

  .DEVIATN  = 0x46,    /* DEVIATN       Modem Deviation Setting */

  .FOCCFG   = 0x1d,    /* FOCCFG        Frequency Offset Compensation Configuration */

  .BSCFG    = 0x1c,    /* BSCFG         Bit Synchronization Configuration */

  .AGCCTRL2 = 0xc7,    /* AGCCTRL2      AGC Control */
  .AGCCTRL1 = 0x00,    /* AGCCTRL1      AGC Control */
  .AGCCTRL0 = 0xb2,    /* AGCCTRL0      AGC Control */

  .FREND1   = 0xb6,    /* FREND1        Front End RX Configuration */
  .FREND0   = 0x10,    /* FREND0        Front End TX Configuration */

  .FSCAL3   = 0xea,    /* FSCAL3        Frequency Synthesizer Calibration */
  .FSCAL2   = 0x2a,    /* FSCAL2        Frequency Synthesizer Calibration */
  .FSCAL1   = 0x00,    /* FSCAL1        Frequency Synthesizer Calibration */
  .FSCAL0   = 0x1f,    /* FSCAL0        Frequency Synthesizer Calibration */

  .CHMIN    = 0,       /* Fix at 9th channel: 869.80 MHz +- 100 kHz RF Bandwidth */
  .CHMAX    = 9,       /* single channel */

  .PAMAX    = 8,       /* 0 means power OFF, 8 represents PA[7] */
  .PA       =
  {
    0x03, 0x0f, 0x1e, 0x27, 0x67, 0x50, 0x81, 0xc2
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
