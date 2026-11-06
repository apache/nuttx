/****************************************************************************
 * include/nuttx/power/mpq8785.h
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

#ifndef __INCLUDE_NUTTX_POWER_MPQ8785_H
#define __INCLUDE_NUTTX_POWER_MPQ8785_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_REGULATOR_MPQ8785

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* What the part itself can produce, which is not the same question as what
 * the rail it is fitted to can survive.  See struct mpq8785_config_s.
 *
 * The reference DAC runs from 0.35V to 1.55V, from the electrical table in
 * the datasheet under "VID Digital-to-Analog Converter".  Note that the
 * command register is wider than the DAC and will accept values outside
 * this, so these bounds come from the specification rather than from what
 * the register happens to hold.
 */

#define MPQ8785_PART_MIN_UV   350000
#define MPQ8785_PART_MAX_UV   1550000

/* The DAC steps 1.5625mV, which is not a whole number of microvolts, so a
 * selector here moves two of its steps.  Every voltage this driver can be
 * asked for is then exactly representable, at the cost of half the
 * available resolution, which no user of a rail like this will notice.
 */

#define MPQ8785_STEP_UV       3125

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* How a board describes the part to this driver.
 *
 * The limits are the board's, and they are the important field.  This
 * driver knows what the chip can do and cannot know what the rail it feeds
 * will tolerate, because that is a property of the board rather than of the
 * part: the same regulator supplying a core rail and supplying a peripheral
 * has entirely different bounds.  Whatever is given here is intersected
 * with what the part can produce, so a board can narrow the range and never
 * widen it.
 *
 * Leave both zero to get the part's own range.  That is the right answer on
 * a bench setup, and the wrong one on a board where somebody knows what the
 * rail is for.  Anything driving a processor should say so here.
 */

struct mpq8785_config_s
{
  FAR const char *name;    /* Regulator name that consumers ask for       */
  uint8_t         addr;    /* Bus address                                 */
  uint32_t        min_uv;  /* Board's floor, or zero for the part's       */
  uint32_t        max_uv;  /* Board's ceiling, or zero for the part's     */

  /* Where the readings are published, as uORB topic numbers.
   *
   * The part reports what it is producing and what it is being fed, and
   * those are both voltages, so they cannot share a topic number: one
   * number carries one topic of each type.  devno takes the output side,
   * its voltage, current, power and the part's own temperature, and
   * vin_devno takes the input voltage on its own.
   *
   * Set nosensor to leave all of it unpublished, for a board that wants
   * the rail controlled and does not care to watch it.
   */

  int             devno;
  int             vin_devno;
  bool            nosensor;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mpq8785_initialize
 *
 * Description:
 *   Bind the part to an I2C bus and register it as a regulator that
 *   in-kernel consumers reach by name.
 *
 *   The regulator is registered as already enabled and nothing is applied
 *   to it, so bringing this up observes the rail rather than moving it.
 *
 * Input Parameters:
 *   i2c    - The bus the part is on
 *   config - Naming, address and the board's voltage limits
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int mpq8785_initialize(FAR struct i2c_master_s *i2c,
                       FAR const struct mpq8785_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_REGULATOR_MPQ8785 */
#endif /* __INCLUDE_NUTTX_POWER_MPQ8785_H */
