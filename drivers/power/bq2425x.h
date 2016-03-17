/****************************************************************************
 * drivers/power/bq2425x.h
 * Lower half driver for BQ2425x battery charger
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __DRIVERS_POWER_BQ2425X_H
#define __DRIVERS_POWER_BQ2425X_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Auxiliary Definitions */

#define BQ2425X_VOLT_MIN  3500
#define BQ2425X_VOLT_MAX  4440

#define BQ2425X_CURR_MIN  500
#define BQ2425X_CURR_MAX  2000

/* BQ2425x Register Definitions ********************************************/

#define BQ2425X_REG_1     0x00
#define BQ2425X_REG_2     0x01
#define BQ2425X_REG_3     0x02
#define BQ2425X_REG_4     0x03
#define BQ2425X_REG_5     0x04
#define BQ2425X_REG_6     0x05
#define BQ2425X_REG_7     0x06

/* REG 1 */

#define BQ2425X_WD_FAULT             (1 << 7) /* If 1 means WD timeout if WD is enabled */
#define BQ2425X_WD_EN                (1 << 6) /* Set 1 will enable and reset timeout */
#define BQ2425X_STAT_SHIFT           4        /* Battery charger status */
#define BQ2425X_STAT_MASK            (3 << BQ2425X_STAT_SHIFT)
#  define BQ2425X_STAT_READY         (0 << BQ2425X_STAT_SHIFT)
#  define BQ2425X_STAT_CHG_PROGRESS  (1 << BQ2425X_STAT_SHIFT)
#  define BQ2425X_STAT_CHG_DONE      (2 << BQ2425X_STAT_SHIFT)
#  define BQ2425X_STAT_FAULT         (3 << BQ2425X_STAT_SHIFT)
#define BQ2425X_FAULT_SHIFT          0        /* Battery Fault report */
#define BQ2425X_FAULT_MASK           (15 << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_NORMAL       (0  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_INP_OVP      (1  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_INP_UVLO     (2  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_SLEEP        (3  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_BAT_TEMP     (4  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_BAT_OVP      (5  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_THERM_SHUT   (6  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_TIMER        (7  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_NO_BATTERY   (8  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_ISET_SHORT   (9  << BQ2425X_FAULT_SHIFT)
#  define BQ2425X_FAULT_INP_LDO_LOW  (10 << BQ2425X_FAULT_SHIFT)

/* REG 2 */

#define BQ2425X_RESET                 (1 << 7) /* Write 1 to Reset all register to default values */
#define BQ2425X_INP_CURR_LIM_SHIFT    4        /* Input Current Limit */
#define BQ2425X_INP_CURR_LIM_MASK     (7 << BQ2425X_INP_CURR_LIM_SHIFT)
#  define BQ2425X_INP_CURR_LIM_100MA  (0 << BQ2425X_INP_CURR_LIM_SHIFT) /* USB2.0 host with 100mA current limit */
#  define BQ2425X_INP_CURR_LIM_150MA  (1 << BQ2425X_INP_CURR_LIM_SHIFT) /* USB3.0 host with 150mA current limit */
#  define BQ2425X_INP_CURR_LIM_500MA  (2 << BQ2425X_INP_CURR_LIM_SHIFT) /* USB2.0 host with 500mA current limit */
#  define BQ2425X_INP_CURR_LIM_900MA  (3 << BQ2425X_INP_CURR_LIM_SHIFT) /* USB3.0 host with 900mA current limit */
#  define BQ2425X_INP_CURR_LIM_1500MA (4 << BQ2425X_INP_CURR_LIM_SHIFT) /* Charger with 1500mA current limit */
#  define BQ2425X_INP_CURR_LIM_2000MA (5 << BQ2425X_INP_CURR_LIM_SHIFT) /* Charger with 2000mA current limit */
#  define BQ2425X_INP_CURR_EXT_ILIM   (6 << BQ2425X_INP_CURR_LIM_SHIFT) /* External ILIM current limit */
#  define BQ2425X_INP_CURR_NO_LIMIT   (7 << BQ2425X_INP_CURR_LIM_SHIFT) /* No input current limit with internal clamp at 3A */
#define BQ2425X_EN_STAT               (1 << 3) /* Enable/Disable STAT pin */
#define BQ2425X_EN_TERM               (1 << 2) /* Enable/Disable charge termination */
#define BQ2425X_CE                    (1 << 1) /* Enable/Disable the Charger, inverted logic, 0 enables */
#define BQ2425X_HZ_MODE               (1 << 0) /* Sets the charger IC into low power standby mode, but keep BAT FET ON */

/* REG 3 */

#define BQ2425X_BAT_VOLT_SHIFT        2
#define BQ2425X_BAT_VOLT_MASK         (0x3F << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3500MV     (0 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3520MV     (1 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3540MV     (2 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3560MV     (3 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3580MV     (4 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3600MV     (5 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3620MV     (6 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3640MV     (7 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3660MV     (8 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3680MV     (9 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3700MV     (10 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3720MV     (11 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3740MV     (12 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3760MV     (13 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3780MV     (14 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3800MV     (15 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3820MV     (16 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3840MV     (17 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3860MV     (18 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3880MV     (19 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3900MV     (20 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3920MV     (21 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3940MV     (22 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3960MV     (23 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_3980MV     (24 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4000MV     (25 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4020MV     (26 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4040MV     (27 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4060MV     (28 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4080MV     (29 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4100MV     (30 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4120MV     (31 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4140MV     (32 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4160MV     (33 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4180MV     (34 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4200MV     (35 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4220MV     (36 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4240MV     (37 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4260MV     (38 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4280MV     (39 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4300MV     (40 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4320MV     (41 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4340MV     (42 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4360MV     (43 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4380MV     (44 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4400MV     (45 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4420MV     (46 << BQ2425X_BAT_VOLT_SHIFT)
#  define BQ2425X_BAT_VOLT_4440MV     (47 << BQ2425X_BAT_VOLT_SHIFT)
#define BQ2425X_USB_DET_EN_SHIFT      0
#define BQ2425X_USB_DET_EN_MASK       (3 << BQ2425X_USB_DET_EN_SHIFT)
#  define BQ2425X_DCP_EN2_0_EN1_0     (0 << BQ2425X_USB_DET_EN_SHIFT)
#  define BQ2425X_CDP_EN2_0_EN1_1     (1 << BQ2425X_USB_DET_EN_SHIFT)
#  define BQ2425X_SDP_EN2_1_EN1_0     (2 << BQ2425X_USB_DET_EN_SHIFT)
#  define BQ2425X_APPLE_EN2_1_EN1_1   (3 << BQ2425X_USB_DET_EN_SHIFT)

/* REG 4 */

#define BQ2425X_CHG_CURRENT_SHIFT     2
#define BQ2425X_CHG_CURRENT_MASK      (0x1F << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_500MA   (0 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_550MA   (1 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_600MA   (2 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_650MA   (3 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_700MA   (4 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_750MA   (5 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_800MA   (6 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_850MA   (7 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_900MA   (8 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_950MA   (9 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1000MA  (10 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1050MA  (11 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1100MA  (12 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1150MA  (13 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1200MA  (14 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1250MA  (15 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1300MA  (16 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1350MA  (17 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1400MA  (18 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1450MA  (19 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1500MA  (20 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1550MA  (21 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1600MA  (22 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1650MA  (23 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1700MA  (24 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1750MA  (25 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1800MA  (26 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1850MA  (27 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1900MA  (28 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_1950MA  (29 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_CURRENT_2000MA  (30 << BQ2425X_CHG_CURRENT_SHIFT)
#  define BQ2425X_CHG_EXT_ISET_MODE   (31 << BQ2425X_CHG_CURRENT_SHIFT)
#define BQ2425X_CHG_CURR_TERM_SHIFT   0
#define BQ2425X_CHG_CURR_TERM_MASK    (7 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_50MA  (0 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_75MA  (1 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_100MA (2 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_125MA (3 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_150MA (4 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_175MA (5 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_200MA (6 << BQ2425X_CHG_CURR_TERM_SHIFT)
#  define BQ2425X_CHG_CURR_TERM_225MA (7 << BQ2425X_CHG_CURR_TERM_SHIFT)

/* REG 5 */

#define BQ2425X_LOOP_STATUS_SHIFT     6        /* Show if there is some active loop that slow down safety timer */
#define BQ2425X_LOOP_STATUS_MASK      (3 << BQ2425X_LOOP_STATUS_SHIFT)
#  define BQ2425X_NO_LOOP             (0 << BQ2425X_LOOP_STATUS_SHIFT)
#  define BQ2425X_VIN_DPM_LOOP        (1 << BQ2425X_LOOP_STATUS_SHIFT)
#  define BQ2425X_INP_CURR_LIM_LOOP   (2 << BQ2425X_LOOP_STATUS_SHIFT)
#  define BQ2425X_THERM_REG_LOOP      (3 << BQ2425X_LOOP_STATUS_SHIFT)
#define BQ2425X_LOW_CHG               (1 << 5) /* 0 = REG 4 defines current; 1 = Low curr 330mA */
#define BQ2425X_DPDM_EN               (1 << 4) /* Force D+/D- detection */
#define BQ2425X_CE_STATUS             (1 << 3) /* 0 = CE low ; 1 = CE high */
#define BQ2425X_VIN_DPM_SHIFT         0        /* Sets the input VDPM level */
#define BQ2425X_VIN_DPM_MASK          (7 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4200MV      (0 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4280MV      (1 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4360MV      (2 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4440MV      (3 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4520MV      (4 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4600MV      (5 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4680MV      (5 << BQ2425X_VIN_DPM_SHIFT)
#  define BQ2425X_VIN_DPM_4760MV      (5 << BQ2425X_VIN_DPM_SHIFT)

/* REG 6 */

#define BQ2425X_2XTMR_EN              (1 << 7) /* Timer slowed 2x when in thermal reg., Vin_dpm or DPPM */
#define BQ2425X_TIMER_SHIFT           6 /* Safety timer time limit */
#define BQ2425X_TIMER_MASK            (3 << BQ2425X_TIMER_SHIFT)
#  define BQ2425X_TIMER_0p74H         (0 << BQ2425X_TIMER_SHIFT) /* 0.75 hour fast charge */
#  define BQ2425X_TIMER_6H            (1 << BQ2425X_TIMER_SHIFT) /* 6 hour fast charge (default 01) */
#  define BQ2425X_TIMER_9H            (2 << BQ2425X_TIMER_SHIFT) /* 9 hour fast charge */
#  define BQ2425X_TIMER_DISABLED      (3 << BQ2425X_TIMER_SHIFT) /* Disable safety timers */
#define BQ2425X_SYSOFF                (1 << 4) /* 0 = SYSOFF disabled ; 1 = SYSOFF enabled */
#define BQ2425X_TS_EN                 (1 << 3) /* 0 = TS function disabled ; 1 = TS function enabled */
#define BQ2425X_TS_STATUS_SHIFT       0        /* TS Fault Mode */
#define BQ2425X_TS_STATUS_MASK        (7 << BQ2425X_TS_STATUS_SHIFT)
#  define BQ2425X_TS_NORMAL           (0 << BQ2425X_TS_STATUS_SHIFT) /* Normal, No TS fault */
#  define BQ2425X_TS_TEMP_HOT         (1 << BQ2425X_TS_STATUS_SHIFT) /* TS_temp > T_hot */
#  define BQ2425X_TS_TEMP_WARM        (2 << BQ2425X_TS_STATUS_SHIFT) /* T_warm < TS_temp < T_hot */
#  define BQ2425X_TS_TEMP_COOL        (3 << BQ2425X_TS_STATUS_SHIFT) /* T_cold < TS_temp < T_cool*/
#  define BQ2425X_TS_TEMP_COLD        (4 << BQ2425X_TS_STATUS_SHIFT) /* TS_temp < T_cold */
#  define BQ2425X_TS_TEMP_VERY_COLD   (5 << BQ2425X_TS_STATUS_SHIFT) /* T_freeze < TS_temp < T_cold */
#  define BQ2425X_TS_TEMP_FREEZE      (6 << BQ2425X_TS_STATUS_SHIFT) /* TS_temp < T_freeze */
#  define BQ2425X_TS_OPEN_DISABLED    (7 << BQ2425X_TS_STATUS_SHIFT) /* TS open (TS disabled) */

/* REG 7 */

#define BQ2425X_VOLT_OVP_SHIFT        5        /* OVP voltage */
#define BQ2425X_VOLT_OVP_MASK         (7 << BQ2425X_VOLT_OVP_SHIFT)
#  define BQ2425X_VOLT_OVP_6p0V       (0 << BQ2425X_VOLT_OVP_SHIFT) /* 6.0V */
#  define BQ2425X_VOLT_OVP_6p5V       (1 << BQ2425X_VOLT_OVP_SHIFT) /* 6.5V */
#  define BQ2425X_VOLT_OVP_7p0V       (2 << BQ2425X_VOLT_OVP_SHIFT) /* 7.0V */
#  define BQ2425X_VOLT_OVP_8p0V       (3 << BQ2425X_VOLT_OVP_SHIFT) /* 8.0V */
#  define BQ2425X_VOLT_OVP_9p0V       (4 << BQ2425X_VOLT_OVP_SHIFT) /* 9.0V */
#  define BQ2425X_VOLT_OVP_9p5V       (5 << BQ2425X_VOLT_OVP_SHIFT) /* 9.5V */
#  define BQ2425X_VOLT_OVP_10p0V      (6 << BQ2425X_VOLT_OVP_SHIFT) /* 10.0V */
#  define BQ2425X_VOLT_OVP_10p5V      (7 << BQ2425X_VOLT_OVP_SHIFT) /* 10.5V */
#define BQ2425X_CLR_VDP               (1 << 4) /* 0 = Keep D+ voltage ; 1 = Turn off D+ voltage */
#define BQ2425X_FORCE_BAT_DET         (1 << 3) /* Enter the battery detection routine */
#define BQ2425X_FORCE_PTM             (1 << 2) /* PTM mode enable */
                                      /* bit 1: reserved */
                                      /* bit 0: reserved */

#endif /* __DRIVERS_POWER_BQ2425X_H */
