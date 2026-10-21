/****************************************************************************
 * include/nuttx/power/ltc4013.h
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

#ifndef __INCLUDE_NUTTX_POWER_LTC4013_H
#define __INCLUDE_NUTTX_POWER_LTC4013_H

/* Lower half driver for the Analog Devices LTC4013 multi-chemistry battery
 * charger controller, on top of the battery charger upper half
 * (include/nuttx/power/battery_charger.h).
 *
 * The LTC4013 has no digital interface.  The board wires its pins as
 * follows and provides the access to them through the callbacks of
 * struct ltc4013_config_s:
 *
 *   ENAB          Enable (also usable as an input UVLO).
 *   MODE1, MODE2  Three-state pins that select the charge algorithm
 *                 (Table 1 of the datasheet): low, high or floating.
 *   STAT0, STAT1  Open drain status outputs (Table 2 of the datasheet).
 *   FB            Feedback of the battery voltage.  The divider from the
 *                 battery to FB sets the charge voltage.
 *
 * The application chooses the charge profile: it selects the algorithm
 * with BATIOC_OPERATE (LTC4013_OPRTN_ALGO) and the voltage with
 * BATIOC_VOLTAGE.  The voltage is the one of the VFLOAT column of Table 1,
 * the voltage of the last stage of the algorithm (for the Li-Ion
 * algorithms it is the charge voltage).  The absorption and the
 * equalization voltages follow from the same divider, in the proportions
 * of the datasheet.  The charge current is set by the sense resistor of the
 * board and the input current by its input stage, so BATIOC_CURRENT and
 * BATIOC_INPUT_CURRENT are not supported.
 *
 * How the voltage is programmed depends on the board.  If the board
 * provides the voltage callback, it programs its own feedback network (a
 * digital potentiometer, a DAC, ...) for a battery voltage and the
 * regulation voltage of the FB pin that the driver passes, and the voltage
 * is set by software.  Without it the divider is fixed by the hardware:
 * BATIOC_VOLTAGE is not supported, and if the board gives the two
 * resistors of the divider (r_top from the battery to FB and r_bottom from
 * FB to ground) BATIOC_GET_VOLTAGE reports the voltage that it programs
 * with the selected algorithm.
 *
 * The driver drives ENAB low when it registers.  When the voltage is set by
 * software the charger cannot be enabled until a voltage has been set.
 * Changing the algorithm is refused while the charger is enabled, since
 * the regulation voltage of the FB pin changes with it; the driver then
 * programs the same battery voltage again, with the new regulation voltage.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_LTC4013

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device specific operation of BATIOC_OPERATE.  The values from 0x80 on
 * are reserved for the lower half drivers.
 *
 * LTC4013_OPRTN_ALGO - Select the charge algorithm.
 *   batio_operate_msg_s.u8[0] holds an enum ltc4013_algo_e.
 *
 * The common BATIO_OPRTN_CHARGE is supported too: u8[0] is nonzero to
 * enable the charger and zero to disable it.
 */

#define LTC4013_OPRTN_ALGO  0x80

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Charge algorithms, the rows of Table 1 of the datasheet.  The Li-Ion
 * and the 2-stage algorithms have no absorption stage.  "Wide" is the
 * wide voltage spread: the absorption (and the equalization) voltage
 * is further above the float voltage.
 */

enum ltc4013_algo_e
{
  LTC4013_ALGO_LIION_NORMAL = 0,  /* MODE1 H, MODE2 M: Vfb 2.367 V */
  LTC4013_ALGO_LIION_WIDE,        /* MODE1 M, MODE2 M: Vfb 2.400 V */
  LTC4013_ALGO_2STAGE,            /* MODE1 L, MODE2 L: Vfb 2.267 V */
  LTC4013_ALGO_2STAGE_HIGH,       /* MODE1 L, MODE2 M: Vfb 2.333 V */
  LTC4013_ALGO_3STAGE_NORMAL,     /* MODE1 H, MODE2 L: float 2.267 V */
  LTC4013_ALGO_3STAGE_WIDE,       /* MODE1 M, MODE2 L: float 2.200 V */
  LTC4013_ALGO_4STAGE_NORMAL_T4,  /* MODE1 L, MODE2 H: tEQ = tEOC / 4 */
  LTC4013_ALGO_4STAGE_NORMAL,     /* MODE1 H, MODE2 H: tEQ = tEOC / 8 */
  LTC4013_ALGO_4STAGE_WIDE,       /* MODE1 M, MODE2 H: float 2.200 V */
  LTC4013_ALGO_NALGOS
};

/* Level of a MODE pin */

enum ltc4013_level_e
{
  LTC4013_LEVEL_LOW = 0,          /* Driven low */
  LTC4013_LEVEL_HIGH,             /* Driven high, at least 2.5 V */
  LTC4013_LEVEL_FLOAT             /* High impedance, the chip biases it */
};

/* Board configuration */

struct ltc4013_config_s
{
  /* Programs the feedback network for a battery voltage of mv millivolts,
   * when the regulation voltage of the FB pin is vfb_mv millivolts.  It
   * returns a negated errno value, for example -EINVAL if the network
   * cannot reach the voltage, and then it must leave the hardware as it
   * was.  It is called again with the same voltage and the new Vfb when
   * the algorithm changes.  Optional: without it the voltage is fixed by
   * the hardware.
   */

  CODE int (*voltage)(FAR void *arg, int mv, int vfb_mv);

  /* Fixed divider, in ohms: r_top between the battery and FB and r_bottom
   * between FB and ground.  Optional, only used without the voltage
   * callback to report the voltage in BATIOC_GET_VOLTAGE; 0 for r_bottom
   * if it is not known.  The sum of the two must stay below 1.7 Mohm.
   */

  uint32_t        r_top;
  uint32_t        r_bottom;

  /* Limits of the voltage that the application may request, in mV, 0 for
   * none.  They apply to the voltage callback; set them from the battery of
   * the board (cells and chemistry).
   */

  int             v_min;
  int             v_max;

  /* Algorithm selected when the charger is registered */

  enum ltc4013_algo_e algo;

  /* Pin access, all required.  They are called from the context of the
   * caller of the upper half, never from an interrupt.
   *
   * enable - Drive ENAB: true enables the charger.
   * mode   - Set MODE1 and MODE2.
   * stat   - Read STAT0 and STAT1: true when the pin is pulled low (ON in
   *          Table 2 of the datasheet).
   */

  CODE int (*enable)(FAR void *arg, bool enable);
  CODE int (*mode)(FAR void *arg, enum ltc4013_level_e mode1,
                   enum ltc4013_level_e mode2);
  CODE int (*stat)(FAR void *arg, FAR bool *stat0, FAR bool *stat1);
  FAR void       *arg;                /* Argument of the callbacks */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ltc4013_register
 *
 * Description:
 *   Register a LTC4013 as a battery charger.  The charger is left
 *   disabled with the algorithm of the configuration; the application sets
 *   the voltage and enables it.
 *
 * Input Parameters:
 *   devpath - Device path, for example "/dev/bat0".
 *   config  - Board configuration.  The structure is copied.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltc4013_register(FAR const char *devpath,
                     FAR const struct ltc4013_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LTC4013 */
#endif /* __INCLUDE_NUTTX_POWER_LTC4013_H */
