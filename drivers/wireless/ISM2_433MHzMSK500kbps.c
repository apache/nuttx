/****************************************************************************
 * drivers/wireless/ISM2_433MHzMSK500kbps.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2011 Ales Verbic.  All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Ales Verbic <ales.verbic@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/cc1101.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Settings for 905 MHz, GFSK at 250kbps
 *
 * ISM Region 2 (America) only, Band 902–928 MHz
 *
 *   Cordless phones          1 W
 *   Microwave ovens        750 W
 *   Industrial heaters     100 kW
 *   Military radar        1000 kW
 *
 *   Deviation = 126.953125
 *   Base frequency = 901.999969
 *   Carrier frequency = 905.998993
 *   Channel number = 20
 *   Carrier frequency = 905.998993
 *   Modulated = true
 *   Modulation format = GFSK
 *   Manchester enable = false
 *   Sync word qualifier mode = 30/32 sync word bits detected
 *   Preamble count = 4
 *   Channel spacing = 199.951172
 *   Carrier frequency = 905.998993
 *   Data rate = 249.939
 *   RX filter BW = 541.666667
 *   Data format = Normal mode
 *   Length config = Variable packet length mode. Packet length configured
 *     by the first byte after sync word
 *   CRC enable = true
 *   Packet length = 61
 *   Device address = 0
 *   Address config = No address check
 *   CRC autoflush = false
 *   PA ramping = false
 *   TX power = 0
 */

const struct c1101_rfsettings_s cc1101_rfsettings_ISM2_433MHzMSK500kbps =
{
  .FIFOTHR  = 0x07,  /* FIFOTHR */
  .SYNC1    = 0x9b,  /* SYNC1 */
  .SYNC0    = 0xad,  /* SYNC0 */
  .PKTLEN   = 0xff,  /* PKTLEN */
  .PKTCTRL1 = 0x04,  /* Packet Automation Control */
  .PKTCTRL0 = 0x05,  /* Packet Automation Control */
  .ADDR     = 0xff,  /* ADDR */
  .CHANNR   = 0x00,  /* CHANNR */

  .FSCTRL1  = 0x0f,  /* FSCTRL1  Frequency Synthesizer Control */
  .FSCTRL0  = 0x00,  /* FSCTRL0  Frequency Synthesizer Control */

  .FREQ2    = 0x10,  /* FREQ2    Frequency Control Word, High Byte */
  .FREQ1    = 0xa7,  /* FREQ1    Frequency Control Word, Middle Byte */
  .FREQ0    = 0x62,  /* FREQ0    Frequency Control Word, Low Byte */

  .MDMCFG4  = 0x1e,  /* MDMCFG4  Modem Configuration */
  .MDMCFG3  = 0x3b,  /* MDMCFG3  Modem Configuration */
  .MDMCFG2  = 0x73,  /* MDMCFG2  Modem Configuration */
  .MDMCFG1  = 0x42,  /* MDMCFG1  Modem Configuration */
  .MDMCFG0  = 0xf8,  /* MDMCFG0  Modem Configuration */

  .DEVIATN  = 0x44,  /* DEVIATN  Modem Deviation Setting */
  .FOCCFG   = 0x16,  /* FOCCFG   Frequency Offset Compensation Configuration */

  .BSCFG    = 0x6c,  /* BSCFG    Bit Synchronization Configuration */

  .AGCCTRL2 = 0x45,  /* AGCCTRL2 AGC Control */
  .AGCCTRL1 = 0x40,  /* AGCCTRL1 AGC Control */
  .AGCCTRL0 = 0x91,  /* AGCCTRL0 AGC Control */

  .FREND1   = 0x56,  /* FREND1   Front End RX Configuration */
  .FREND0   = 0x10,  /* FREND0   Front End TX Configuration */

  .FSCAL3   = 0xea,  /* FSCAL3   Frequency Synthesizer Calibration */
  .FSCAL2   = 0x2a,  /* FSCAL2   Frequency Synthesizer Calibration */
  .FSCAL1   = 0x00,  /* FSCAL1   Frequency Synthesizer Calibration */
  .FSCAL0   = 0x1f,  /* FSCAL0   Frequency Synthesizer Calibration */

  .CHMIN    = 0,     /* VERIFY REGULATIONS! */
  .CHMAX    = 0xff,

  .PAMAX    = 8,     /* 0 means power OFF, 8 represents PA[7] */
  .PA       =
  {
    0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  }
};
