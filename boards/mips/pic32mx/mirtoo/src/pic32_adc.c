/****************************************************************************
 * boards/mips/pic32mx/mirtoo/src/pic32_adc.c
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "pic32mx.h"
#include "mirtoo.h"

#ifdef CONFIG_PIC32MX_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The Mirtoo features a PGA117 amplifier/multipexer that can be configured
 * to bring any analog signal from PORT0,.. PORT7 to pin 19 of the PIC32MX:
 *
 * --- ------------------------------------------------ ---------------------
 * PIN PIC32 SIGNAL(s)                                  BOARD SIGNAL/USAGE
 * --- ------------------------------------------------ ---------------------
 * 19  PGED3/VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/PMD7/RA0 AIN PGA117 Vout
 * --- ------------------------------------------------ ---------------------
 *
 * The PGA117 driver can be enabled by setting the following the nsh
 * configuration:
 *
 *   CONFIG_ADC=y         : Enable support for analog input devices
 *   CONFIG_PIC32MX_ADC=y : Enable support the PIC32 ADC driver
 *   CONFIG_ADC_PGA11X=y  : Enable support for the PGA117
 *
 * When CONFIG_PIC32MX_ADC=y is defined, the Mirtoo boot up logic will
 * automatically configure pin 18 (AN0) as an analog input.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_adcinitialize
 *
 * Description:
 *   Perform architecture specific ADC initialization
 *
 ****************************************************************************/

#if 0 /* Not used */
int pic32mx_adcinitialize(void)
{
  /* Configure the pin 19 as an analog input */

#warning "Missing logic"

  /* Initialize the PGA117 amplifier multiplexer */

#warning "Missing logic"

  /* Register the ADC device driver */

#warning "Missing logic"

  return OK;
}
#endif

#endif /* CONFIG_PIC32MX_ADC */
