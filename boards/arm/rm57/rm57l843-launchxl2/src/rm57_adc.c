/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_adc.c
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

/* MibADC bring-up for the LAUNCHXL2-RM57L.  Every enabled conversion group
 * is registered as its own character device, numbered /dev/adc0 upwards in
 * the order the groups are enabled.
 *
 * Board wiring (schematic SPRR397).  ADREFHI is tied to VCCAD through R1 and
 * ADREFLO to VSSAD through R2/R3, all zero ohms, so the conversion range is
 * 0V to 3.3V and full scale reads 0xfff in 12-bit mode.  R4 is not fitted;
 * the schematic warns that R3 must come off before it is populated.
 *
 * The board also fits a 1k trim potentiometer, U10, across +3V3 and GND with
 * its wiper on AD1IN[6] (ball T19) and a 4700pF capacitor to ground at the
 * pin.  The silkscreen calls it "replaces temp sensor".  It gives the ADC an
 * adjustable full-range input with no external wiring, which makes AD1IN[6]
 * the natural channel for a first bring-up test.  AD1IN[6] has no AD2IN
 * function, so it belongs to ADC1 alone.
 *
 * That wiper presents up to 250 ohms, which with the 4700pF gives an RC of
 * about 1.2us.  Converting AD1IN[6] on its own is unaffected, but in a
 * sequence with channels at very different voltages the sampling window
 * needs to cover a few time constants for the settling error to stay near
 * one LSB; the adc configuration uses an ACQ of 30, about 3.4us, for that.
 *
 * The analog inputs reachable on the BoosterPack headers are:
 *
 *   J1-2   AD1IN[16] / AD2IN[0]      J3-3   AD1IN[0]
 *   J1-6   AD1IN[17] / AD2IN[1]      J3-4   AD1IN[7]
 *                                    J3-5   AD1IN[18] / AD2IN[2]
 *   J1-1   +3V3                      J3-6   AD1IN[19] / AD2IN[3]
 *   J3-2   GND                       J3-7   AD1IN[20] / AD2IN[4]
 *                                    J3-8   AD1IN[21] / AD2IN[5]
 *
 * The remaining AD1IN/AD2IN balls come out on the J-headers along the board
 * edges.  All of them are dedicated analog pins, so no pin muxing is needed;
 * only the AD1EVT/AD2EVT trigger inputs are muxed, and AD1EVT (ball N19)
 * collides with the Ethernet MII_RX_ER signal.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include "rm57_adc.h"
#include "rm57l843-launchxl2.h"

#ifdef CONFIG_RM57_ADC

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_adc_group_setup
 *
 * Description:
 *   Register one conversion group at the next free /dev/adcN minor.
 *
 ****************************************************************************/

static int rm57_adc_group_setup(int module, int group, int *minor)
{
  char devpath[16];
  int ret;

  snprintf(devpath, sizeof(devpath), "/dev/adc%d", *minor);

  ret = rm57_adc_initialize(devpath, module, group);
  if (ret < 0)
    {
      aerr("ERROR: ADC%d group %d init failed: %d\n", module + 1, group,
           ret);
      return ret;
    }

  (*minor)++;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_adc_setup
 *
 * Description:
 *   Register every enabled MibADC conversion group.
 *
 ****************************************************************************/

int rm57_adc_setup(void)
{
  int minor = 0;
#ifdef CONFIG_RM57_ADC_TEMPSENSOR
  int devno = CONFIG_RM57_ADC_TEMP_DEVNO;
#endif
  int ret;

  UNUSED(ret);

#ifdef CONFIG_RM57_ADC1_EV
  ret = rm57_adc_group_setup(RM57_ADC_MODULE1, RM57_ADC_GROUP_EV, &minor);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_ADC1_G1
  ret = rm57_adc_group_setup(RM57_ADC_MODULE1, RM57_ADC_GROUP_G1, &minor);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_ADC1_G2
  ret = rm57_adc_group_setup(RM57_ADC_MODULE1, RM57_ADC_GROUP_G2, &minor);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_ADC2_EV
  ret = rm57_adc_group_setup(RM57_ADC_MODULE2, RM57_ADC_GROUP_EV, &minor);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_ADC2_G1
  ret = rm57_adc_group_setup(RM57_ADC_MODULE2, RM57_ADC_GROUP_G1, &minor);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_ADC2_G2
  ret = rm57_adc_group_setup(RM57_ADC_MODULE2, RM57_ADC_GROUP_G2, &minor);
  if (ret < 0)
    {
      return ret;
    }
#endif

  UNUSED(minor);

#ifdef CONFIG_RM57_ADC_TEMPSENSOR

#ifdef CONFIG_RM57_ADC_TEMP1
  ret = rm57_adc_temp_register(0, devno);
  if (ret < 0)
    {
      return ret;
    }

  devno++;
#endif

#ifdef CONFIG_RM57_ADC_TEMP2
  ret = rm57_adc_temp_register(1, devno);
  if (ret < 0)
    {
      return ret;
    }

  devno++;
#endif

#ifdef CONFIG_RM57_ADC_TEMP3
  ret = rm57_adc_temp_register(2, devno);
  if (ret < 0)
    {
      return ret;
    }

  devno++;
#endif

  UNUSED(devno);
#endif

  return OK;
}

#endif /* CONFIG_RM57_ADC */
