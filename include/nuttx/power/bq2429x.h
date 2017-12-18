/****************************************************************************
 * drivers/power/bq2429x.h
 * Lower half driver for BQ2429X battery charger
 *
 *   Copyright (C) 2017 Neil Hancock. All rights reserved.
 *
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#ifndef __DRIVERS_POWER_BQ2429X_H
#define __DRIVERS_POWER_BQ2429X_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Auxiliary Definitions */

#define BQ2429X_VOLTCHG_MIN  3504
#define BQ2429X_VOLTCHG_MAX  4400

#define BQ2429X_CURRCHG_MIN  512
#define BQ2429X_CURRCHG_MAX  3008

/* BQ2429X Register Definitions ********************************************/
#define BQ2429X_REG00     0x00
#define BQ2429X_REG01     0x01
#define BQ2429X_REG02     0x02
#define BQ2429X_REG03     0x03
#define BQ2429X_REG04     0x04
#define BQ2429X_REG05     0x05
#define BQ2429X_REG06     0x06
#define BQ2429X_REG07     0x07
#define BQ2429X_REG08     0x08
#define BQ2429X_REG09     0x09
#define BQ2429X_REG0A     0x0a

/* REG00 Input Source Control Register */

/* For enabling Device Shutdown for shipping - EN_HIZ=1 until QON pressed*/
#define BQ2429XR1_EN_HIZ               (1 << 7) /* 0 Disable (default) 1 Enable HighZ on battery, powerdown */

/* Dynamic Power Management - Indicated in StatusReg DPM_STAT REG08[3]
 VINDPM - Input Voltage threshold (a drop below  5V) that triggers DPM
 INLIM - Input current threshold that tiggers DPM */

#define BQ2429XR0_VINDPM_SHIFT          3   /* VIN DPM Offset 5V? Range*/
#define BQ2429XR0_VINDPM_MASK          (0xf << BQ2429XR0_VINDPM_SHIFT)
#  define BQ2429XR0_VINDPM3_080mV      (1 << BQ2429XR0_VINDPM_SHIFT)
#  define BQ2429XR0_VINDPM2_160mV      (2 << BQ2429XR0_VINDPM_SHIFT)
#  define BQ2429XR0_VINDPM1_320mV      (4 << BQ2429XR0_VINDPM_SHIFT)
#  define BQ2429XR0_VINDPM0_640mV      (8 << BQ2429XR0_VINDPM_SHIFT)
#define BQ2429XR0_INLIM_SHIFT           0   /* Input Current Limit - lower of I2C & ILIM */
#define BQ2429XR0_INLIM_MASK           (0x7 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_0100mA       (0x0 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_0150mA       (0x1 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_0500mA       (0x2 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_0900mA       (0x3 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_1000mA       (0x4 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_1500mA       (0x5 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_2000mA       (0x6 << BQ2429XR0_INLIM_SHIFT)
#  define BQ2429XR0_INLIM_3000mA       (0x7 << BQ2429XR0_INLIM_SHIFT)

/* REG01 Power-On Configuration Register */

#define BQ2429XR1_REG_RESET            (1 << 7) /* Write 1 to Reset all registers to default values */
#define BQ2429XR1_DOG_RESET            (1 << 6) /* Write 1 for watchdog timer reset */
#define BQ2429XR1_OTG_CONFIG           (1 << 5) /* =0 Disable (default) =1 Enable See description */
#define BQ2429XR1_CHG_CONFIG           (1 << 4) /* =0 Disable =1 Enable (default) See description */
#define BQ2429XR1_SYS_MINV_SHIFT        1    /* Min Sys Voltage Limit. Offset 3.0V Range 3-3.7V */
#define BQ2429XR1_SYS_MINV_MASK        (7 << BQ2429XR1_SYS_MINV_SHIFT)
#define BQ2429XR1_SYS_MINV0_0_1V       (1 << BQ2429XR1_SYS_MINV_SHIFT)
#define BQ2429XR1_SYS_MINV0_0_2V       (2 << BQ2429XR1_SYS_MINV_SHIFT)
#define BQ2429XR1_SYS_MINV0_0_4V       (4 << BQ2429XR1_SYS_MINV_SHIFT)

#define BQ2429XR2_BOOST_LIM            (1 << 0) /* 0=1A, 1=1.5A (default) Vout Boost Current Limit */

/* REG02 Charge Current Control */

#define BQ2429XR2_ICHG_SHIFT         2
#define BQ2429XR2_ICHG_MASK          (0x3f << BQ2429XR2_ICHG_SHIFT)

#define BQ2429XR2_BCOLD              (1 << 1) /* Boost Mode temperature threshold config for boost disable 0=76% 1=79% */
#define BQ2429XR2_FORCE_20PCT        (1 << 0) /* Charge type 0=Fast (default), 1=Trickle charge with 20% current */

/* REG03 Pre-charge Termination Control Register */

#define BQ2429XR3_IPRECHG_SHIFT     4 /* Precharge I Limit. Offset 128mA Range 128-2048 mA */
#define BQ2429XR3_IPRECHG_MASK      (0xf << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_0128mA      (0x00 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_0128mA      (0x01 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_0256mA      (0x02 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_0384mA      (0x03 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_0512mA      (0x04 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_0768mA      (0x05 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_0896mA      (0x06 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1024mA      (0x07 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1152mA      (0x10 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1280mA      (0x11 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1408mA      (0x12 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1536mA      (0x13 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1664mA      (0x14 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1792mA      (0x15 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_1920mA      (0x16 << BQ2429XR3_IPRECHG_SHIFT)
#define BQ2429XR3_IPRECHG_2048mA      (0x17 << BQ2429XR3_IPRECHG_SHIFT)

#define BQ2429XR3_ITERM_SHIFT     0 /* Offset 128mA Range 128-2048 mA (128-1024 mA in BQ24296M )*/
#define BQ2429XR3_ITERM_MASK      (0xf << BQ2429XR3_ITERM_SHIFT)
#define BQ2429XR3_ITERM0_128mA    (1 << BQ2429XR3_ITERM_SHIFT)
#define BQ2429XR3_ITERM0_256mA    (2 << BQ2429XR3_ITERM_SHIFT)
#define BQ2429XR3_ITERM0_512mA    (4 << BQ2429XR3_ITERM_SHIFT)
#define BQ2429XR3_ITERM0_1024mA   (8 << BQ2429XR3_ITERM_SHIFT) /* Reserved in BQ24296M */

/* REG04 Charge Voltage Control Register */
#define BQ2429XR4_VREG_SHIFT     2 /* Offset 3.504V Range 3.504-4.400V Default 4.208V */
#define BQ2429XR4_VREG_MASK      (0x3f<< BQ2429XR4_VREG_SHIFT)

#define BQ2429XR4_BATLOWV        (1 << 1) /* 0=2.8V  1=3.0V Pre-charge to fast Charge */
#define BQ2429XR4_VRECHG         (1 << 0) /* 0=100mV 1=300mV */

/* REG05 Charge Termination Timer Control Register */

#define BQ2429XR5_EN_TERM           (1 << 7) /* 0=Disable 1=Enable(default) terminate of charge */
#define BQ2429XR5_RESERVED6         (1 << 6)
#define BQ2429XR5_WATCHDOG_SHIFT     4        /* Watchdog Timer Settings */
#define BQ2429XR5_WATCHDOG_MASK     (3 << BQ2429XR5_WATCHDOG_SHIFT)
#  define BQ2429XR5_WATCHDOG_DIS    (0 << BQ2429XR5_WATCHDOG_SHIFT)
#  define BQ2429XR5_WATCHDOG_040Sec (1 << BQ2429XR5_WATCHDOG_SHIFT)
#  define BQ2429XR5_WATCHDOG_080Sec (2 << BQ2429XR5_WATCHDOG_SHIFT)
#  define BQ2429XR5_WATCHDOG_160Sec (3 << BQ2429XR5_WATCHDOG_SHIFT)

#define BQ2429XR5_EN_TIMER           (1 << 3) /* 0=Disable 1=Enable(default) */
/* Fast Charge Timer Settings */
#define BQ2429XR5_CHG_TIMER_SHIFT     1
#define BQ2429XR5_CHG_TIMER_MASK     (3 << BQ2429XR5_CHG_TIMER_SHIFT)
# define BQ2429XR5_CHG_TIMER_05hrs   (0 << BQ2429XR5_CHG_TIMER_SHIFT)
# define BQ2429XR5_CHG_TIMER_08hrs   (1 << BQ2429XR5_CHG_TIMER_SHIFT)
# define BQ2429XR5_CHG_TIMER_12hrs   (2 << BQ2429XR5_CHG_TIMER_SHIFT)
# define BQ2429XR5_CHG_TIMER_20hrs   (3 << BQ2429XR5_CHG_TIMER_SHIFT)
#define BQ2429XR5_RESERVED0          (1 << 0)

/* REG06 Boost Voltage/Thermal Regulation Control register */

#define BQ2429XR6_BOOSTV_SHIFT       4    /* Offset 4.55V Range 4.55-5.51A Dev 4.998V(0111) */
#define BQ2429XR6_BOOSTV_MASK       (0xf << BQ2429XR6_BOOSTV_SHIFT)
#  define BQ2429XR6_BOOSTV_064mV    (1 << BQ2429XR6_BOOSTV_SHIFT)
#  define BQ2429XR6_BOOSTV_128mV    (2 << BQ2429XR6_BOOSTV_SHIFT)
#  define BQ2429XR6_BOOSTV_256mV    (4 << BQ2429XR6_BOOSTV_SHIFT)
#  define BQ2429XR6_BOOSTV_512mV    (8 << BQ2429XR6_BOOSTV_SHIFT)
#define BQ2429XR6_BHOT_SHIFT         2   /* Boost Mode temp threshold */
#define BQ2429XR6_BHOT_MASK         (3 << BQ2429XR6_BHOT_SHIFT)
#  define BQ2429XR6_BHOT_55C        (0 << BQ2429XR6_BHOT_SHIFT)
#  define BQ2429XR6_BHOT_60C        (1 << BQ2429XR6_BHOT_SHIFT)
#  define BQ2429XR6_BHOT_65C        (2 << BQ2429XR6_BHOT_SHIFT)
#  define BQ2429XR6_BHOT_DISABLE    (3 << BQ2429XR6_BHOT_SHIFT)
#define BQ2429XR6_TREG_SHIFT         0   /* Thermal Regulation  */
#define BQ2429XR6_TREG_MASK         (3 << BQ2429XR6_TREG_SHIFT)
#  define BQ2429XR6_TREG_060C       (0 << BQ2429XR6_TREG_SHIFT)
#  define BQ2429XR6_TREG_080C       (1 << BQ2429XR6_TREG_SHIFT)
#  define BQ2429XR6_TREG_100C       (2 << BQ2429XR6_TREG_SHIFT)
#  define BQ2429XR6_TREG_110C       (3 << BQ2429XR6_TREG_SHIFT)

/* REG07 Misc Operation Control Register */

#define BQ2429XR7_DPDM_EN             (1 << 7) /* 1=Force Detection when VBUS power is present */
#define BQ2429XR7_TMR2X_EN            (1 << 6) /* 1=Safety Timer slowed by 2X during DPM/Thermal regulation */
#define BQ2429XR7_BATFET_DISABLE      (1 << 5) /* 1=BATFET (Q4) turn off */
#define BQ2429XR7_RESERVED4           (1 << 4)
#define BQ2429XR7_RESERVED3           (1 << 3)
#define BQ2429XR7_RESERVED2           (1 << 2)
#define BQ2429XR7_INT_MASK1           (1 << 1) /* =1 (default) INT on CHRG_FAULT */
#define BQ2429XR7_INT_MASK0           (1 << 0) /* =1 (default) INT on BAT_FAULT */

/* REG08 Systems Status Register */

#define BQ2429XR8_VBUS_STAT_SHIFT       6    /* VBUS Connection Type */
#define BQ2429XR8_VBUS_STAT_MASK       (3 << BQ2429XR8_VBUS_STAT_SHIFT)
#  define BQ2429XR8_VBUS_STAT_UNKNOWN  (0 << BQ2429XR8_VBUS_STAT_SHIFT)
#  define BQ2429XR8_VBUS_STAT_USBH     (1 << BQ2429XR8_VBUS_STAT_SHIFT)
#  define BQ2429XR8_VBUS_STAT_ADAPTER  (2 << BQ2429XR8_VBUS_STAT_SHIFT)
#  define BQ2429XR8_VBUS_STAT_OTG      (3 << BQ2429XR8_VBUS_STAT_SHIFT)
#define BQ2429XR8_CHRG_STAT_SHIFT       4    /* Charging Status */
#define BQ2429XR8_CHRG_STAT_MASK       (3 << BQ2429XR8_CHRG_STAT_SHIFT)
#  define BQ2429XR8_CHRG_STAT_NONE     (0 << BQ2429XR8_CHRG_STAT_SHIFT)
#  define BQ2429XR8_CHRG_STAT_PRECHG   (1 << BQ2429XR8_CHRG_STAT_SHIFT)
#  define BQ2429XR8_CHRG_STAT_FASTCHG  (2 << BQ2429XR8_CHRG_STAT_SHIFT)
#  define BQ2429XR8_CHRG_STAT_DONE     (3 << BQ2429XR8_CHRG_STAT_SHIFT)
#define BQ2429XR8_DPM_STAT             (1 << 3) /* 0= NotDPM 1=VINDPM or INDPM */
#define BQ2429XR8_PG_STAT              (1 << 2) /* 0= Not 1=Power Good */
#define BQ2429XR8_THERM_STAT           (1 << 1) /* 0= Normal 1=In Thermal Regulation */
#define BQ2429XR8_VSYS_STAT            (1 << 0) /* 0= Not 1=In VSYSMIN regulation BAT < VSYSMIN */

/* REG09 New Fault Register */

#define BQ2429XR9_WATCHDOG_FAULT        (1 << 7) /* 1=Watchdog Timer expired */
#define BQ2429XR9_OTG_FAULT             (1 << 6) /* 1=Bus overloaded in OTG, or VBUS OVP or battery low */
#define BQ2429XR9_CHRG_FAULT_SHIFT       4    /* Charging Status */
#define BQ2429XR9_CHRG_FAULT_MASK       (3 << BQ2429XR9_CHRG_FAULT_SHIFT)
#  define BQ2429XR9_CHRG_FAULT_NORMAL   (0 << BQ2429XR9_CHRG_FAULT_SHIFT)
#  define BQ2429XR9_CHRG_FAULT_INPUT    (1 << BQ2429XR9_CHRG_FAULT_SHIFT)
#  define BQ2429XR9_CHRG_FAULT_THERMAL  (2 << BQ2429XR9_CHRG_FAULT_SHIFT)
#  define BQ2429XR9_CHRG_FAULT_TIMER    (3 << BQ2429XR9_CHRG_FAULT_SHIFT)
#define BQ2429XR9_BAT_FAULT             (1 << 3) /* 1=Battery OVP */
#define BQ2429XR9_RESERVED2             (1 << 2)

#define BQ2429XR9_NTC_FAULT1_COLD       (1 << 1) /* Cold temperature */
#define BQ2429XR9_NTC_FAULT2_HOT        (1 << 0) /* Hot temperature */

/* REG0A Vendor Part Revision Info */

#define BQ24296_VENDOR_ID               0x20 /* BQ24296 */
#define BQ24296M_VENDOR_ID              0x20 /* BQ24296M */
#define BQ24297_VENDOR_ID               0x60 /* BQ24297 */

#endif /* __DRIVERS_POWER_BQ2429X_H */
