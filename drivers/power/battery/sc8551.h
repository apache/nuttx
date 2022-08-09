/****************************************************************************
 * drivers/power/battery/sc8551.h
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

#ifndef __INCLUDE_NUTTX_POWER_SC8551_H
#define __INCLUDE_NUTTX_POWER_SC8551_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************

 * Pre-processor Definitions
 ****************************************************************************/

#define SC8551_HEALTH_TEMP_MAX  80
#define SC8551_HEALTH_TEMP_MIN  10

#define SC8551_HEALTH_UNKNOWN   -1
#define SC8551_HEALTH_GOOD      0
#define SC8551_HEALTH_OVERHEAT  1
#define SC8551_HEALTH_OVERCOLD  2

#define SC_IIC_RETRY_NUM        3

#define SC8551_IBUSUCP_DELAY    (500*1000U)

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct sc8551_cfg_s
{
  bool bat_ovp_disable;
  bool bat_ocp_disable;
  bool bat_ovp_alm_disable;
  bool bat_ocp_alm_disable;

  int bat_ovp_th;
  int bat_ovp_alm_th;
  int bat_ocp_th;
  int bat_ocp_alm_th;

  bool bus_ovp_alm_disable;
  bool bus_ocp_disable;
  bool bus_ocp_alm_disable;

  int bus_ovp_th;
  int bus_ovp_alm_th;
  int bus_ocp_th;
  int bus_ocp_alm_th;

  bool bat_ucp_alm_disable;

  int bat_ucp_alm_th;
  int ac_ovp_th;

  bool bat_therm_disable;
  bool bus_therm_disable;
  bool die_therm_disable;

  int bat_therm_th; /* in % */
  int bus_therm_th; /* in % */
  int die_therm_th; /* in degC */

  int sense_r_mohm;
};

struct sc8551_stat_s
{
  int part_no;
  int revision;

  int mode;

  bool batt_present;
  bool vbus_present;

  bool usb_present;
  bool charge_enabled; /* Register bit status */

  /* ADC reading */

  int vbat_volt;
  int vbus_volt;
  int vout_volt;
  int vac_volt;

  int ibat_curr;
  int ibus_curr;

  int bat_temp;
  int bus_temp;
  int die_temp;

  /* alarm/fault status */

  bool bat_ovp_fault;
  bool bat_ocp_fault;
  bool bus_ovp_fault;
  bool bus_ocp_fault;

  bool bat_ovp_alarm;
  bool bat_ocp_alarm;
  bool bus_ovp_alarm;
  bool bus_ocp_alarm;

  bool bat_ucp_alarm;

  bool bat_therm_alarm;
  bool bus_therm_alarm;
  bool die_therm_alarm;

  bool bat_therm_fault;
  bool bus_therm_fault;
  bool die_therm_fault;

  bool therm_shutdown_flag;
  bool therm_shutdown_stat;

  bool vbat_reg;
  bool ibat_reg;

  int  prev_alarm;
  int  prev_fault;

  int chg_ma;
  int chg_mv;

  int charge_state;
};

typedef enum
{
  ADC_IBUS,
  ADC_VBUS,
  ADC_VAC,
  ADC_VOUT,
  ADC_VBAT,
  ADC_IBAT,
  ADC_TBUS,
  ADC_TBAT,
  ADC_TDIE,
  ADC_MAX_NUM,
}ADC_CH;

FAR struct sc8551_key_state_s
{
  uint16_t vbus;
  uint16_t vout;
  bool     vbat_ovp;
  bool     ibat_ocp;
  bool     vbus_ovp;
  bool     ibus_ocp;
  bool     ibus_ucp;
  bool     adapter_insert;
  bool     vbat_insert;
  bool     adc_done;
  bool     vbus_errorlo_stat;
  bool     vbus_errorhi_stat;
  bool     cp_switching_stat;
  bool     charge_en_stat;
};

#endif
