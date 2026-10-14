/****************************************************************************
 * arch/arm/include/rm57/rm57_adc_ioctl.h
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

#ifndef __ARCH_ARM_INCLUDE_RM57_RM57_ADC_IOCTL_H
#define __ARCH_ARM_INCLUDE_RM57_RM57_ADC_IOCTL_H

/* The MibADC's application-visible interface: the RM57-specific ioctl
 * commands accepted by any /dev/adcN, the types they take, and the constants
 * needed to fill those types in.  This is the header an application
 * includes, as <arch/chip/rm57_adc_ioctl.h>; the driver's own
 * arch/arm/src/rm57/rm57_adc.h includes it and adds the in-kernel entry
 * points on top.
 *
 * The generic ANIOC_* commands work too.  ANIOC_TRIGGER starts a conversion
 * sequence, ANIOC_GET_NCHANNELS returns the number of channels selected in
 * the group, and ANIOC_WDOG_UPPER/LOWER map onto magnitude comparators 0
 * and 1 of the owning module when CONFIG_RM57_ADC_MAGCOMP is enabled.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Module identifiers */

#define RM57_ADC_MODULE1        0
#define RM57_ADC_MODULE2        1
#define RM57_ADC_NMODULES       2

/* Conversion groups.  Note that arch/arm/include/rm57/rm57l843_irq.h calls
 * the Event group "GROUP0".
 */

#define RM57_ADC_GROUP_EV       0      /* Event group */
#define RM57_ADC_GROUP_G1       1      /* Group1 */
#define RM57_ADC_GROUP_G2       2      /* Group2 */
#define RM57_ADC_NGROUPS        3

/* Magnitude comparators per module */

#define RM57_ADC_NMAGCOMP       3

/* Look-up table entries per group in the enhanced channel selection mode */

#define RM57_ADC_LUT_NENTRIES   32

/* On-die temperature sensors, and the number of factory calibration points
 * recorded for each of them.
 */

#define RM57_ADC_NTEMPSENSORS   3
#define RM57_ADC_TEMPCAL_NPOINTS 3

/* Hardware trigger sources for the default trigger set (SPNS215).  The
 * alternate set is selected by the PINMUX_ALT_ADC_TRIGGER control.
 */

#define ADC_TRIGSRC_EVT         0      /* ADxEVT pin */
#define ADC_TRIGSRC_N2HET1_8    1
#define ADC_TRIGSRC_N2HET1_10   2
#define ADC_TRIGSRC_RTI_COMP0   3
#define ADC_TRIGSRC_N2HET1_12   4
#define ADC_TRIGSRC_N2HET1_14   5
#define ADC_TRIGSRC_GIOB_0      6
#define ADC_TRIGSRC_GIOB_1      7

/* RM57-specific ioctl commands.  The command range is reserved in
 * include/nuttx/analog/ioctl.h as AN_RM57_FIRST/AN_RM57_NCMDS.
 */

/* Start/stop conversions on this group.  IN: None. */

#define ANIOC_RM57_START        _ANIOC(AN_RM57_FIRST + 0)
#define ANIOC_RM57_STOP         _ANIOC(AN_RM57_FIRST + 1)

/* Set the group's channel selection mask.
 * IN: uint32_t bit mask of channels
 */

#define ANIOC_RM57_SET_CHANMASK _ANIOC(AN_RM57_FIRST + 2)

/* Configure the group's conversion mode.
 * IN: const struct rm57_adc_grpcfg_s *
 */

#define ANIOC_RM57_SET_GRPCFG   _ANIOC(AN_RM57_FIRST + 3)

/* Run the offset error calibration sequence on the owning module.
 * IN: None, or a struct rm57_adc_calib_s * to receive the raw results.
 */

#define ANIOC_RM57_CALIBRATE    _ANIOC(AN_RM57_FIRST + 4)

/* Run the differential mid-point calibration sequence.
 * IN: struct rm57_adc_calib_s * (may be NULL)
 */

#define ANIOC_RM57_MIDPOINTCAL  _ANIOC(AN_RM57_FIRST + 5)

/* Read back or override the module's offset correction value, the quantity
 * the hardware adds to every conversion result.  Both directions use the
 * same signed convention as struct rm57_adc_calib_s.correction: the get
 * sign extends ADCALR out of the module's resolution, and the set truncates
 * back into it.
 *
 * IN: int32_t * (get) / int32_t (set)
 */

#define ANIOC_RM57_GET_OFFSET   _ANIOC(AN_RM57_FIRST + 6)
#define ANIOC_RM57_SET_OFFSET   _ANIOC(AN_RM57_FIRST + 7)

/* Run the input channel open/short self test.
 * IN: struct rm57_adc_selftest_s *
 */

#define ANIOC_RM57_SELFTEST     _ANIOC(AN_RM57_FIRST + 8)

/* Configure one magnitude comparator.
 * IN: const struct rm57_adc_magcomp_s *
 */

#define ANIOC_RM57_SET_MAGCOMP  _ANIOC(AN_RM57_FIRST + 9)

/* Load a free-order channel conversion sequence into this group's look-up
 * table and enable the enhanced channel selection mode (ADC1 only).
 * IN: const struct rm57_adc_chnseq_s *, or NULL for sequential mode.
 */

#define ANIOC_RM57_SET_CHNSEQ   _ANIOC(AN_RM57_FIRST + 10)

/* Access the module's ADEVT pin when it is used as a GPIO.
 * IN: struct rm57_adc_evtpin_s *
 */

#define ANIOC_RM57_EVTPIN       _ANIOC(AN_RM57_FIRST + 11)

/* Convert one on-die temperature sensor, reporting raw and calibrated value.
 * IN/OUT: struct rm57_adc_temp_s * (caller sets .sensor)
 */

#define ANIOC_RM57_GET_TEMP     _ANIOC(AN_RM57_FIRST + 12)

/* Self test verdicts, reported in struct rm57_adc_selftest_s.status */

#define RM57_ADC_SELFTEST_OK        0  /* Channel behaves normally */
#define RM57_ADC_SELFTEST_OPEN      1  /* Channel appears open */
#define RM57_ADC_SELFTEST_SHORT_GND 2  /* Channel shorted to ground */
#define RM57_ADC_SELFTEST_SHORT_PWR 3  /* Channel shorted to supply */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Runtime configuration of one conversion group */

struct rm57_adc_grpcfg_s
{
  uint32_t chanmask;     /* Channels to convert (bit n = AD<m>IN[n]) */
  uint16_t acq;          /* Sampling window: SW = acq + 2 ADCLK cycles */
  uint16_t threshold;    /* Results to accumulate before interrupting */
  uint8_t  trigsrc;      /* ADC_TRIGSRC_* hardware trigger source */
  bool     hwtrig;       /* True: hardware triggered, false: software */
  bool     rising;       /* Trigger edge when hwtrig is set */
  bool     bothedges;    /* Trigger on either edge when hwtrig is set */
  bool     continuous;   /* Convert continuously rather than once */
  bool     freeze;       /* Allow this group to be frozen by another group */
  bool     ovrwrite;     /* Overwrite results memory on overrun */
  bool     dischargeen;  /* Enable sample capacitor discharge */
  uint8_t  dischargecyc; /* Discharge duration in ADCLK cycles */
};

/* Raw calibration conversion results.  The four entries correspond to the
 * BRIDGE_EN/HILO combinations of TRM Table 22-2, in that order.
 */

struct rm57_adc_calib_s
{
  uint16_t raw[4];      /* Raw conversions of each reference voltage */
  int16_t  offset;      /* Computed offset error (pre-negation) */
  int16_t  correction;  /* Value written to ADCALR (2's complement) */
};

/* Self test of a single input channel.  'lo' and 'hi' are the conversions
 * obtained with ADREFLO and ADREFHI driven onto the channel through the
 * internal test resistor; 'status' summarises them.
 */

struct rm57_adc_selftest_s
{
  uint8_t  channel;     /* IN:  Channel to test */
  uint16_t lo;          /* OUT: Conversion with ADREFLO applied */
  uint16_t hi;          /* OUT: Conversion with ADREFHI applied */
  uint8_t  status;      /* OUT: RM57_ADC_SELFTEST_* */
};

/* One magnitude comparator.  When 'usechannel' is set the monitored channel
 * is compared against the last conversion of 'cmpchannel' instead of the
 * fixed 'threshold'.
 */

struct rm57_adc_magcomp_s
{
  uint8_t  index;       /* Comparator to configure, 0-2 */
  bool     enable;      /* False disables the comparator */
  uint8_t  channel;     /* Channel whose result is monitored */
  uint16_t threshold;   /* Compare value */
  bool     greater;     /* True: interrupt when result >= threshold */
  bool     usechannel;  /* Compare against cmpchannel, not threshold */
  uint8_t  cmpchannel;  /* Channel supplying the compare value */
};

/* A free-order channel conversion sequence for the enhanced channel
 * selection mode.  'nentries' may not exceed RM57_ADC_LUT_NENTRIES.
 */

struct rm57_adc_chnseq_s
{
  uint8_t  nentries;                        /* Number of LUT entries used */
  uint8_t  channel[RM57_ADC_LUT_NENTRIES];  /* Internal mux selection */
  uint8_t  extsel[RM57_ADC_LUT_NENTRIES];   /* External mux selection */
};

/* One reading from an on-die temperature sensor.  'millicel' is only
 * meaningful when 'calibrated' is set; without usable factory calibration in
 * OTP the driver reports the raw conversion and leaves the temperature zero
 * rather than inventing one.
 */

struct rm57_adc_temp_s
{
  uint8_t  sensor;      /* IN:  0, 1 or 2 */
  uint16_t raw;         /* OUT: raw conversion result */
  int32_t  millicel;    /* OUT: temperature in milli-Celsius */
  bool     calibrated;  /* OUT: OTP calibration was usable */
};

/* ADEVT pin access.  Setting 'output' configures the pin as an output and
 * drives 'value'; otherwise the pin is an input and 'value' is updated with
 * the level read back.
 */

struct rm57_adc_evtpin_s
{
  bool     output;      /* IN:     Drive the pin rather than read it */
  bool     value;       /* IN/OUT: Level to drive, or level read */
  bool     opendrain;   /* IN:     Enable open drain output */
  bool     pullen;      /* IN:     Enable the pull resistor (input only) */
  bool     pullup;      /* IN:     Pull up rather than down */
};

#endif /* __ARCH_ARM_INCLUDE_RM57_RM57_ADC_IOCTL_H */
