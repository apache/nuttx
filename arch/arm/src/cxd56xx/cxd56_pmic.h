/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_pmic.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_PMIC_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_PMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMIC Register Definitions */

#define PMIC_REG_DDC_ANA1   (0x26)
#define PMIC_REG_CNT_USB2   (0x81)

/* PMIC DDC_ANA1 Definitions */

#define PMIC_PM_HIZ     (2u << 4)
#define PMIC_PM_DEF     (0u << 4)
#define PMIC_IOST_DEF   (2u << 2)
#define PMIC_IOMAX_DEF  (2u << 0)

/* PMIC CNT_USB2 Definitions */

#define PMIC_SET_CHGOFF (1u << 2)

/* PMIC Interrupt Status Definitions */

#define PMIC_INT_ALARM  (1u << 0)
#define PMIC_INT_WKUPS  (1u << 1)
#define PMIC_INT_WKUPL  (1u << 2)
#define PMIC_INT_VSYS   (1u << 3)

/* PMIC GPO Channel Definitions */

#define PMIC_GPO0       (1u << 0)
#define PMIC_GPO1       (1u << 1)
#define PMIC_GPO2       (1u << 2)
#define PMIC_GPO3       (1u << 3)
#define PMIC_GPO4       (1u << 4)
#define PMIC_GPO5       (1u << 5)
#define PMIC_GPO6       (1u << 6)
#define PMIC_GPO7       (1u << 7)

/* PMIC LoadSwitch Channel Definitions */

#define PMIC_LOADSW2    (1u << 2)
#define PMIC_LOADSW3    (1u << 3)
#define PMIC_LOADSW4    (1u << 4)

/* PMIC DDC/LDO Channel Definitions */

#define PMIC_DDC_IO     (1u << 0)
#define PMIC_LDO_EMMC   (1u << 1)
#define PMIC_DDC_ANA    (1u << 2)
#define PMIC_LDO_ANA    (1u << 3)
#define PMIC_DDC_CORE   (1u << 4)
#define PMIC_LDO_PERI   (1u << 5)

/* Charge mode for both of low/high temperature */

#define PMIC_CHGMODE_ON   0x00
#define PMIC_CHGMODE_OFF  0x01
#define PMIC_CHGMODE_WEAK 0x02

/* Charge status */

#define PMIC_STAT_INIT_RST       0
#define PMIC_STAT_INIT_WAIT      1
#define PMIC_STAT_INIT_CHK       2
#define PMIC_STAT_DBP_START      3
#define PMIC_STAT_DB_INICHARGE   4
#define PMIC_STAT_DB_PRECHARGE   5
#define PMIC_STAT_DCON_WAIT      6
#define PMIC_STAT_PD_START       7
#define PMIC_STAT_DM_COMPARE     8
#define PMIC_STAT_PD_END         9
#define PMIC_STAT_BAT_WAIT       10
#define PMIC_STAT_CHG_STOP       11
#define PMIC_STAT_GB_PRECHARGE   12
#define PMIC_STAT_GB_QCKCHARGE   13
#define PMIC_STAT_GB_LOWCHARGE   14
#define PMIC_STAT_GB_HIGHCHARGE  15
#define PMIC_STAT_CHG_JUDGE      16
#define PMIC_STAT_CHG_COMPLETE   17
#define PMIC_STAT_GB_CONWAIT     18
#define PMIC_STAT_GB_CPTEMPWAIT1 19
#define PMIC_STAT_GB_CPTEMPWAIT2 20
#define PMIC_STAT_GB_TEMPWAIT    21
#define PMIC_STAT_DB_TEMPWAIT    22
#define PMIC_STAT_DB_CONWAIT     23
#define PMIC_STAT_BAT_UNUSUAL    24
#define PMIC_STAT_BAT_DISCON     25

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pmic_gauge_s
{
  int voltage;
  int current;
  int temp;
};

struct pmic_temp_table_s
{
  int T60;
  int T45;
  int T10;
  int T00;
};

struct pmic_mon_s
{
  int on;
  int interval;
  int threshold_volt;
  int threshold_current;
};

struct pmic_mon_status_s
{
  int brun;
  int index;
  int latest;
  int total_watt;
  int total_time;
};

struct pmic_mon_set_s
{
  int clearbuf;
  int clearsum;
};

struct pmic_mon_rec_s
{
  uint16_t index;
  uint16_t timestamp;
  uint16_t voltage;
  int16_t  current;
};

struct pmic_mon_log_s
{
  FAR struct pmic_monitor_rec_s *rec;
  int index;
  int size;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_pmic_get_interrupt_status
 *
 * Description:
 *   Get Raw Interrupt Status register. And furthermore, if status is set,
 *   then clear the interrupt. Register's description is below:
 *
 *     7   6   5   4    3     2     1     0
 *   +---------------+-----+-----+-----+-----+
 *   | x   x   x   x |VSYS |WKUPL|WKUPS|ALARM| target of status
 *   +---------------+-----+-----+-----+-----+
 *
 * Input Parameter:
 *   status - a pointer to the polled value of interrupt status
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   status - return the value of interrupt status register
 *
 ****************************************************************************/

int cxd56_pmic_get_interrupt_status(FAR uint8_t *status);

/****************************************************************************
 * Name: cxd56_pmic_set_gpo_reg
 *
 * Description:
 *   Set GPO register. Register's description is below:
 *
 *     7   6   5   4   3   2   1   0
 *   +---+---+---+---+---+---+---+---+
 *   |CH3|CH2|CH1|CH0|CH3|CH2|CH1|CH0| target of setbit0/clrbit0
 *   +---+---+---+---+---+---+---+---+
 *   +---+---+---+---+---+---+---+---+
 *   |CH7|CH6|CH5|CH4|CH7|CH6|CH5|CH4| target of setbit1/clrbit1
 *   +---+---+---+---+---+---+---+---+
 *   |<- 0: Hi-Z   ->|<- 0: Low    ->|
 *   |<- 1: Output ->|<- 1: High   ->|
 *
 * Input Parameter:
 *   setbitX - set bit that 1 is set
 *   clrbitX - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbitX - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo_reg(FAR uint8_t *setbit0, FAR uint8_t *clrbit0,
                           FAR uint8_t *setbit1, FAR uint8_t *clrbit1);

/****************************************************************************
 * Name: cxd56_pmic_set_gpo
 *
 * Description:
 *   Set High/Low to the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset - GPO Channel number(s)
 *   value - true if output high, false if output low.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo(uint8_t chset, bool value);

/****************************************************************************
 * Name: cxd56_pmic_set_gpo_hiz
 *
 * Description:
 *   Set Hi-Z to the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset - GPO Channel number(s)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo_hiz(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_get_gpo
 *
 * Description:
 *   Get the value from the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset : GPO Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are high. Else, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_gpo(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_get_gpo_hiz
 *
 * Description:
 *   Get the tristate value from the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset : GPO Channel number(s)
 *
 * Returned Value:
 *   Return 0(off), 1(on) or -1(HiZ)
 *
 ****************************************************************************/

int cxd56_pmic_get_gpo_hiz(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_set_loadswitch_reg
 *
 * Description:
 *   Set LoadSwitch register. Register's description is below:
 *
 *     7   6   5   4   3   2   1   0
 *   +---+---+---+---+---+---+---+---+
 *   | - | - | - |CH4|CH3|CH2| 1 | 1 | target of setbit/clrbit
 *   +---+---+---+---+---+---+---+---+
 *               |<- 0: Off->|
 *               |<- 1: On ->|
 *
 * Input Parameter:
 *   setbit - set bit that 1 is set
 *   clrbit - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbit - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_loadswitch_reg(FAR uint8_t *setbit, FAR uint8_t *clrbit);

/****************************************************************************
 * Name: cxd56_pmic_set_loadswitch
 *
 * Description:
 *   Set On/Off to the specified LoadSwitch channel(s)
 *
 * Input Parameter:
 *   chset - LoadSwitch Channel number(s)
 *   value - true if set on, false if set off.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_loadswitch(uint8_t chset, bool value);

/****************************************************************************
 * Name: cxd56_pmic_get_loadswitch
 *
 * Description:
 *   Get the value from the specified LoadSwitch channel(s)
 *
 * Input Parameter:
 *   chset - LoadSwitch Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are on.
 *   Otherwise, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_loadswitch(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_set_ddc_ldo_reg
 *
 * Description:
 *   Set DDC/LDO register. Register's description is below:
 *
 *      7    6    5    4    3    2    1    0
 *   +----+----+----+----+----+----+----+----+
 *   |    |    |LDO |DDC |LDO |DDC |LDO |DDC | target of setbit/clrbit
 *   | -  | -  |PERI|CORE|ANA |ANA |EMMC|IO  |
 *   +----+----+----+----+----+----+----+----+
 *             |<-  0: Off                 ->|
 *             |<-  1: On                  ->|
 *
 * Input Parameter:
 *   setbit - set bit that 1 is set
 *   clrbit - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbit - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_ddc_ldo_reg(FAR uint8_t *setbit, FAR uint8_t *clrbit);

/****************************************************************************
 * Name: cxd56_pmic_set_ddc_ldo
 *
 * Description:
 *   Set On/Off to the specified DDC/LDO channel(s)
 *
 * Input Parameter:
 *   chset - DDC/LO Channel number(s)
 *   value - true if set on, false if set off.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_ddc_ldo(uint8_t chset, bool value);

/****************************************************************************
 * Name: cxd56_pmic_get_ddc_ldo
 *
 * Description:
 *   Get the value from the specified DDC/LDO channel(s)
 *
 * Input Parameter:
 *   chset - DDC/LDO Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are on.
 *   Otherwise, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_ddc_ldo(uint8_t chset);

/****************************************************************************
 * Name: cxd56_pmic_get_gauge
 *
 * Description:
 *   Get the set of values (gauge, voltage, current and temperature) from
 *   PMIC.
 *
 * Input Parameter:
 *   gauge - Set of gauge values
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_get_gauge(FAR struct pmic_gauge_s *gauge);

/****************************************************************************
 * Name: cxd56_pmic_getlowervol
 *
 * Description:
 *   Get lower limit of voltage for system to be running.
 *
 * Input Parameter:
 *   voltage - Lower limit voltage (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getlowervol(FAR int *voltage);

/****************************************************************************
 * Name: cxd56_pmic_setlowervol
 *
 * Description:
 *   Set lower limit of voltage for system to be running.
 *
 * Input Parameter:
 *   voltage - Lower limit voltage (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setlowervol(int voltage);

/****************************************************************************
 * Name: cxd56_pmic_getnotifyvol
 *
 * Description:
 *   Get voltage for the low battery notification
 *
 * Input Parameter:
 *   voltage - Low battery voltage (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getnotifyvol(FAR int *voltage);

/****************************************************************************
 * Name: cxd56_pmic_setnotifyvol
 *
 * Description:
 *   Set voltage for the low battery notification
 *
 * Input Parameter:
 *   voltage - Low battery voltage (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setnotifyvol(int voltage);

/****************************************************************************
 * Name: cxd56_pmic_getchargevol
 *
 * Description:
 *   Get charge voltage
 *
 * Input Parameter:
 *   voltage - Possible values are every 50 between 4000 to 4400 (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargevol(FAR int *voltage);

/****************************************************************************
 * Name: cxd56_pmic_setchargevol
 *
 * Description:
 *   Set charge voltage
 *
 * Input Parameter:
 *   voltage - Available values are every 50 between 4000 to 4400 (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargevol(int voltage);

/****************************************************************************
 * Name: cxd56_pmic_getchargecurrent
 *
 * Description:
 *   Get charge current value
 *
 * Input Parameter:
 *   current - Possible values are 2, 100 and 500 (mA). However,
 *             2 means 2.5 mA actually.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargecurrent(FAR int *current);

/****************************************************************************
 * Name: cxd56_pmic_setchargecurrent
 *
 * Description:
 *   Set charge current value
 *
 * Input Parameter:
 *   current - Available values are 2, 100 and 500 (mA). However, 2 means
 *             2.5 mA actually.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargecurrent(int current);

/****************************************************************************
 * Name: cxd56_pmic_getporttype
 *
 * Description:
 *   Get USB port type
 *
 * Input Parameter:
 *   porttype - PMIC_PORTTYPE_SDP or PMIC_PORTTYPE_DCPCDP
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getporttype(FAR int *porttype);

/****************************************************************************
 * Name: cxd56_pmic_getchargestate
 *
 * Description:
 *   Read charging status
 *
 * Input Parameter:
 *   state - Charging status (see PMIC_STAT_* definitions)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargestate(FAR uint8_t *state);

/****************************************************************************
 * Name: cxd56_pmic_setrechargevol
 *
 * Description:
 *   Set threshold voltage against full charge for automatic restart
 *   charging.
 *
 * Input Parameter:
 *   mV - Available values are -400, -350, -300 and -250 (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setrechargevol(int mv);

/****************************************************************************
 * Name: cxd56_pmic_getrechargevol
 *
 * Description:
 *   Get threshold voltage against full charge for automatic restart
 *   charging.
 *
 * Input Parameter:
 *   mV - Possible values are -400, -350, -300 and -250 (mV)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getrechargevol(FAR int *mv);

/****************************************************************************
 * Name: cxd56_pmic_setchargecompcurrent
 *
 * Description:
 *   Set current value setting for determine fully charged.
 *
 * Input Parameter:
 *   current - Possible values are 50, 40, 30, 20 and 10 (mA)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargecompcurrent(int current);

/****************************************************************************
 * Name: cxd56_pmic_getchargecompcurrent
 *
 * Description:
 *   Get current value setting for determine fully charged.
 *
 * Input Parameter:
 *   current - Available values are 50, 40, 30, 20 and 10 (mA)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargecompcurrent(FAR int *current);

/****************************************************************************
 * Name: cxd56_pmic_gettemptable
 *
 * Description:
 *   Get temperature detect resistance table
 *
 * Input Parameter:
 *   table - Settings values for temperature detecting (see CXD5247GF
 *           specification for more detail)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_gettemptable(FAR struct pmic_temp_table_s *table);

/****************************************************************************
 * Name: cxd56_pmic_settemptable
 *
 * Description:
 *   Set temperature detect resistance table
 *
 * Input Parameter:
 *   table - Settings values for temperature detecting (see CXD5247GF
 *           specification for more detail)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_settemptable(FAR struct pmic_temp_table_s *table);

/****************************************************************************
 * Name: cxd56_pmic_setchargemode
 *
 * Description:
 *   Set charging mode in each low/high temperatures.
 *   In lower than 10 degrees Celsius, charging mode will be changed on/off
 *   and weak (half of charge current) according to setting.
 *   In higher than 45 degrees Celsius, charging mode will be charged on/off
 *   and weak (-0.15V from charge voltage) according to setting.
 *
 * Input Parameter:
 *   low  - Charging mode in low temperature (see PMIC_CHGMODE_*)
 *   high - Charging mode in high temperature (see PMIC_CHGMODE_*)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargemode(int low, int high);

/****************************************************************************
 * Name: cxd56_pmic_getchargemode
 *
 * Description:
 *   Get charging mode in each low/high temperatures.
 *   In lower than 10 degrees Celsius, charging mode will be changed on/off
 *   and weak (half of charge current) according to setting.
 *   In higher than 45 degrees Celsius, charging mode will be charged on/off
 *   and weak (-0.15V from charge voltage) according to setting.
 *
 * Input Parameter:
 *   low  - Charging mode in low temperature (see PMIC_CHG_*)
 *   high - Charging mode in high temperature (see PMIC_CHG_*)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargemode(FAR int *low, FAR int *high);

/****************************************************************************
 * Name: cxd56_pmic_read
 *
 * Description:
 *   Read the value from the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to read buffer
 *   size - byte count of read
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_read(uint8_t addr, FAR void *buf, uint32_t size);

/****************************************************************************
 * Name: cxd56_pmic_write
 *
 * Description:
 *   Write the value to the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to write buffer
 *   size - byte count of write
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_write(uint8_t addr, FAR void *buf, uint32_t size);

/****************************************************************************
 * Battery monitor for debug
 ****************************************************************************/

#ifdef CONFIG_CXD56_PMIC_BATMONITOR
int cxd56_pmic_monitor_enable(FAR struct pmic_mon_s *ptr);
int cxd56_pmic_monitor_status(FAR struct pmic_mon_status_s *ptr);
int cxd56_pmic_monitor_set(FAR struct pmic_mon_set_s *ptr);
int cxd56_pmic_monitor_get(FAR struct pmic_mon_log_s *ptr);
#else
#define cxd56_pmic_monitor_enable(ptr)
#define cxd56_pmic_monitor_status(ptr)
#define cxd56_pmic_monitor_set(ptr)
#define cxd56_pmic_monitor_get(ptr)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_PMIC_H */
