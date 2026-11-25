/****************************************************************************
 * arch/arm/src/stm32n6/stm32_csi.c
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

/* MIPI CSI-2 host and D-PHY receiver of the STM32N6.  The D-PHY set-up
 * (frequency ranges and test interface writes) follows the sequence of
 * the ST HAL DCMIPP driver.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_csi.h"
#include "stm32_csi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* D-PHY configuration clock frequency range for the 20 MHz IC18 clock,
 * as programmed by the ST HAL.
 */

#define CSI_PHY_CCFR             0x28

/* Physical data lanes of logical lanes 0 and 1 (no swap) */

#define CSI_LANE0                1
#define CSI_LANE1                2

/* Receive errors corrupting a frame (a corrected ECC error does not) */

#define CSI_ERR0                 (CSI_INT0_CRCERR | CSI_INT0_ECCERR | \
                                  CSI_INT0_IDERR | CSI_INT0_SPKTERR | \
                                  CSI_INT0_WDERR | CSI_INT0_SYNCERR)
#define CSI_ERR1                 (CSI_INT1_ESOTDL(0) | \
                                  CSI_INT1_ESOTSYNCDL(0) | \
                                  CSI_INT1_ECTRLDL(0) | \
                                  CSI_INT1_ESOTDL(1) | \
                                  CSI_INT1_ESOTSYNCDL(1) | \
                                  CSI_INT1_ECTRLDL(1))

/* Timeout waiting for the virtual channel state, in milliseconds */

#define CSI_VC_TIMEOUT           100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct csi_phyfreq_s
{
  uint16_t rate;                 /* Upper bit rate of the range (Mbit/s) */
  uint8_t  hsfreqrange;          /* D-PHY hsfreqrange */
  uint16_t osc;                  /* DLL target oscillation frequency */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct csi_phyfreq_s g_csi_phyfreq[] =
{
  {   80, 0x00, 460 },
  {   90, 0x10, 460 },
  {  100, 0x20, 460 },
  {  110, 0x30, 460 },
  {  120, 0x01, 460 },
  {  130, 0x11, 460 },
  {  140, 0x21, 460 },
  {  150, 0x31, 460 },
  {  160, 0x02, 460 },
  {  170, 0x12, 460 },
  {  180, 0x22, 460 },
  {  190, 0x32, 460 },
  {  205, 0x03, 460 },
  {  220, 0x13, 460 },
  {  235, 0x23, 460 },
  {  250, 0x33, 460 },
  {  275, 0x04, 460 },
  {  300, 0x14, 460 },
  {  325, 0x25, 460 },
  {  350, 0x35, 460 },
  {  400, 0x05, 460 },
  {  450, 0x16, 460 },
  {  500, 0x26, 460 },
  {  550, 0x37, 460 },
  {  600, 0x07, 460 },
  {  650, 0x18, 460 },
  {  700, 0x28, 460 },
  {  750, 0x39, 460 },
  {  800, 0x09, 460 },
  {  850, 0x19, 460 },
  {  900, 0x29, 460 },
  {  950, 0x3a, 460 },
  { 1000, 0x0a, 460 },
  { 1050, 0x1a, 460 },
  { 1100, 0x2a, 460 },
  { 1150, 0x3b, 460 },
  { 1200, 0x0b, 460 },
  { 1250, 0x1b, 460 },
  { 1300, 0x2b, 460 },
  { 1350, 0x3c, 460 },
  { 1400, 0x0c, 460 },
  { 1450, 0x1c, 460 },
  { 1500, 0x2c, 460 },
  { 1550, 0x3d, 285 },
  { 1600, 0x0d, 295 },
  { 1650, 0x1d, 304 },
  { 1700, 0x2e, 313 },
  { 1750, 0x3e, 322 },
  { 1800, 0x0e, 331 },
  { 1850, 0x1e, 341 },
  { 1900, 0x2f, 350 },
  { 1950, 0x3f, 359 },
  { 2000, 0x0f, 368 },
  { 2050, 0x40, 377 },
  { 2100, 0x41, 387 },
  { 2150, 0x42, 396 },
  { 2200, 0x43, 405 },
  { 2250, 0x44, 414 },
  { 2300, 0x45, 423 },
  { 2350, 0x46, 432 },
  { 2400, 0x47, 442 },
  { 2450, 0x48, 451 },
  { 2500, 0x49, 460 },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: csi_phywrite
 *
 * Description:
 *   Write an 8-bit D-PHY register through the test interface: clock in the
 *   address (MSB then LSB with TWM set) and then the data.
 *
 ****************************************************************************/

static void csi_phywrite(uint8_t msb, uint8_t lsb, uint8_t val)
{
  modifyreg32(STM32_CSI_PTCR1, 0, CSI_PTCR1_TWM);
  modifyreg32(STM32_CSI_PTCR0, 0, CSI_PTCR0_TCKEN);
  modifyreg32(STM32_CSI_PTCR1, 0, CSI_PTCR1_TWM);
  putreg32(0, STM32_CSI_PTCR0);
  putreg32(0, STM32_CSI_PTCR1);
  putreg32(CSI_PTCR1_TDI(msb), STM32_CSI_PTCR1);
  putreg32(CSI_PTCR0_TCKEN, STM32_CSI_PTCR0);
  putreg32(0, STM32_CSI_PTCR0);
  modifyreg32(STM32_CSI_PTCR1, 0, CSI_PTCR1_TWM);
  putreg32(CSI_PTCR0_TCKEN, STM32_CSI_PTCR0);
  modifyreg32(STM32_CSI_PTCR1, 0, CSI_PTCR1_TWM | CSI_PTCR1_TDI(lsb));
  putreg32(0, STM32_CSI_PTCR0);
  putreg32(0, STM32_CSI_PTCR1);
  putreg32(CSI_PTCR1_TDI(val), STM32_CSI_PTCR1);
  putreg32(CSI_PTCR0_TCKEN, STM32_CSI_PTCR0);
  putreg32(0, STM32_CSI_PTCR0);
}

/****************************************************************************
 * Name: csi_waitvc
 ****************************************************************************/

static int csi_waitvc(bool running)
{
  int i;

  for (i = 0; i <= CSI_VC_TIMEOUT; i++)
    {
      if (((getreg32(STM32_CSI_SR0) & CSI_SR0_VC0STATEF) != 0) == running)
        {
          return OK;
        }

      nxsig_usleep(1000);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_csi_start
 ****************************************************************************/

int stm32_csi_start(unsigned int lanes, unsigned int bitrate,
                    unsigned int bpp)
{
  FAR const struct csi_phyfreq_s *freq = NULL;
  uint32_t regval;
  int ret;
  int i;

  if (lanes < 1 || lanes > 2 || bpp < 8 || bpp > 14 || (bpp & 1) != 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < nitems(g_csi_phyfreq); i++)
    {
      if (g_csi_phyfreq[i].rate >= bitrate)
        {
          freq = &g_csi_phyfreq[i];
          break;
        }
    }

  if (freq == NULL)
    {
      return -EINVAL;
    }

  /* Lane merger: logical lanes map to the physical lanes in order */

  putreg32(0, STM32_CSI_CR);
  putreg32(CSI_LMCFGR_LANENB(lanes) | CSI_LMCFGR_DL0MAP(CSI_LANE0) |
           CSI_LMCFGR_DL1MAP(CSI_LANE1), STM32_CSI_LMCFGR);
  putreg32(CSI_CR_CSIEN, STM32_CSI_CR);

  /* Stop the D-PHY with all lanes disabled and pulse its test clock */

  putreg32(0, STM32_CSI_PRCR);
  putreg32(0, STM32_CSI_PCR);
  putreg32(CSI_PTCR0_TCKEN, STM32_CSI_PTCR0);
  nxsig_usleep(1000);
  putreg32(0, STM32_CSI_PTCR0);

  /* Frequency ranges, deskew polarity and the DLL oscillation target */

  regval = CSI_PFCR_CCFR(CSI_PHY_CCFR) | CSI_PFCR_HSFR(freq->hsfreqrange);
  putreg32(regval, STM32_CSI_PFCR);

  csi_phywrite(0x00, 0x08, 0x38);
  csi_phywrite(0x00, 0xe4, 0x11);
  csi_phywrite(0x00, 0xe3, freq->osc >> 8);
  csi_phywrite(0x00, 0xe2, freq->osc & 0xff);

  putreg32(regval | CSI_PFCR_DLD, STM32_CSI_PFCR);

  /* Enable the clock lane and the data lanes, then the D-PHY */

  regval = CSI_PCR_NPWRDOWN | CSI_PCR_CLEN | CSI_PCR_DL0EN;
  if (lanes == 2)
    {
      regval |= CSI_PCR_DL1EN;
    }

  putreg32(regval, STM32_CSI_PCR);
  putreg32(CSI_PRCR_PEN, STM32_CSI_PRCR);
  putreg32(0, STM32_CSI_PMCR);

  /* Forward every data type of virtual channel 0, all with the same
   * pixel size (CDTFT: 8 bpp = 2, 10 bpp = 3, ...).
   */

  putreg32(CSI_VC0CFGR1_CDTFT(bpp / 2 - 2) | CSI_VC0CFGR1_ALLDT,
           STM32_CSI_VC0CFGR1);
  modifyreg32(STM32_CSI_CR, 0, CSI_CR_VC0START);

  ret = csi_waitvc(true);
  if (ret < 0)
    {
      verr("ERROR: CSI virtual channel 0 did not start\n");
      stm32_csi_stop();
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_csi_errors
 ****************************************************************************/

bool stm32_csi_errors(void)
{
  uint32_t err0 = getreg32(STM32_CSI_SR0) & CSI_ERR0;
  uint32_t err1 = getreg32(STM32_CSI_SR1) & CSI_ERR1;

  putreg32(err0, STM32_CSI_FCR0);
  putreg32(err1, STM32_CSI_FCR1);
  return err0 != 0 || err1 != 0;
}

/****************************************************************************
 * Name: stm32_csi_stop
 ****************************************************************************/

void stm32_csi_stop(void)
{
  if ((getreg32(STM32_CSI_SR0) & CSI_SR0_VC0STATEF) != 0)
    {
      modifyreg32(STM32_CSI_CR, 0, CSI_CR_VC0STOP);
      csi_waitvc(false);
    }

  putreg32(0, STM32_CSI_PCR);
  putreg32(0, STM32_CSI_PRCR);
  putreg32(0, STM32_CSI_CR);
}
