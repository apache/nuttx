/********************************************************************************************************
 * arch/arm/src/xmc4/xmc4_vadc.h
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
 ********************************************************************************************************/

/********************************************************************************************************
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ********************************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_VADC_H
#define __ARCH_ARM_SRC_XMC4_XMC4_VADC_H

/********************************************************************************************************
 * Included Files
 ********************************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "xmc4_config.h"

#include "hardware/xmc4_vadc.h"

/********************************************************************************************************
 * Public Types
 ********************************************************************************************************/

/**
 *  Structure to handle the global and groups Input Class configurations.
 *  Configured parameters are sample time and conversion Mode.
 */

typedef struct
{
  /* GLOBICLASSx register config */

  union
  {
    struct
    {
      uint32_t sample_time_std_conv         : 5;  /* Sample time for channels.
                                                   * Range:[0x0 to 0x1F] */
      uint32_t                              : 3;
      uint32_t conversion_mode_standard     : 3;  /* Conversion mode for channels.
                                                   * Uses vadc_convmode_t */
      uint32_t                              : 5;
      uint32_t sampling_phase_emux_channel  : 5;  /* Sample time for EMUX channels.
                                                   * Range:[0x0 to 0x1F] */
      uint32_t                              : 3;
      uint32_t conversion_mode_emux         : 3;  /* Conversion mode for EMUX.
                                                   * Uses vadc_convmode_t */
      uint32_t                              : 5;
    };
    uint32_t iclass;
  };
} vadc_input_class_config_t;

/**
 *   Structure to handle the VADC Global registers configurations
 */

typedef struct
{
  /* GLOBBOUND register config */

  union
  {
    struct
    {
      uint32_t boundary0        : 12;  /* Boundary value for results comparison */
      uint32_t                  : 4;
      uint32_t boundary1        : 12;  /* Boundary value for results comparison */
      uint32_t                  : 4;
    };
    uint32_t globbound;
  };

  /* GLOBCFG register config */

  union
  {
    struct
    {
      uint32_t analog_clock_divider   : 5;   /* Clock for the converter. Range: [0x0 to 0x1F] */
      uint32_t                        : 2;
      uint32_t msb_conversion_clock   : 1;   /* Additional clock cycle for analog converter */
      uint32_t arbiter_clock_divider  : 2;   /* Request source arbiter clock divider.
                                              * Range: [0x0 to 0x3] */
      uint32_t                        : 5;
      uint32_t                        : 17;
    };
    uint32_t globcfg;
  };

  /* GLOBICLASS0 register config */

  vadc_input_class_config_t class0;

  /* GLOBICLASS1 register config */

  vadc_input_class_config_t class1;

  /* GLOBRCR register config */

  union
  {
    struct
    {
      uint32_t                        : 16;
      uint32_t data_reduction_control : 4;   /* Data reduction stages */
      uint32_t                        : 4;
      uint32_t wait_for_read_mode     : 1;   /* Results of the next conversion will not be overwritten. */
      uint32_t                        : 6;
      uint32_t event_gen_enable       : 1;   /* Generates an event on availability of new result. */
    };
    uint32_t globrcr;
  };

  /* CLC register config */

  union
  {
    struct
    {
      uint32_t module_disable             : 1;   /* Disables the module clock. */
      uint32_t                            : 2;
      uint32_t disable_sleep_mode_control : 1;   /* Set it to true in order to disable the Sleep mode */
      uint32_t                            : 28;
    };
    uint32_t clc;
  };
} vadc_global_config_t;

/**
 *   Structure to handle the VADC Groups registers configurations
 */

typedef struct
{
  /* GxICLASS0 register config */

  vadc_input_class_config_t   class0;

  /* GxICLASS1 register config */

  vadc_input_class_config_t   class1;

  /* GxBOUND register config */

  union
  {
    struct
    {
      uint32_t boundary0    : 12;  /* Boundary value for results comparison */
      uint32_t              : 4;
      uint32_t boundary1    : 12;  /* Boundary value for results comparison */
      uint32_t              : 4;
    };
    uint32_t g_bound;
  };

  /* GxARBCFG register config */

  union
  {
    struct
    {
      uint32_t                            : 4;
      uint32_t arbitration_round_length   : 2;  /* Number of arbiter slots to be considered */
      uint32_t                            : 1;
      uint32_t arbiter_mode               : 1;  /* Arbiter mode, either Continuous mode or Demand based. */
      uint32_t                            : 24;
    };
    uint32_t g_arbcfg;
  };
} vadc_group_config_t;

/**
 *   Structure to handle the VADC Groups Channel registers configurations
 */

typedef struct
{
  /* GxCHCTRy register config */

  union
  {
    struct
    {
      uint32_t input_class                : 2;  /* Input conversion class selection.
                                                 * Uses vadc_channel_conv_t */
      uint32_t                            : 2;
      uint32_t lower_boundary_select      : 2;  /* Which boundary register serves as lower bound? */
      uint32_t upper_boundary_select      : 2;  /* Which boundary register serves as upper bound? */
      uint32_t event_gen_criteria         : 2;  /* When should an event be generated? */
      uint32_t sync_conversion            : 1;  /* Enables synchronous conversion for the configured channel */
      uint32_t alternate_reference        : 1;  /* Input reference voltage selection either VARef or CH-0. */
      uint32_t                            : 4;
      uint32_t result_reg_number          : 4;  /* Group result register number */
      uint32_t use_global_result          : 1;  /* Use global result register for background request source channels */
      uint32_t result_alignment           : 1;  /* Alignment of the results read in the result register. */
      uint32_t                            : 6;
      uint32_t broken_wire_detect_channel : 2;  /* Source to be used to charge the capacitor for BWD feature. */
      uint32_t broken_wire_detect         : 1;  /* Configures extra phase before the capacitor is sampled. */
    };
    uint32_t chctr;
  };

  /* GxBFL register config */

  union
  {
    struct
    {
      uint32_t                           : 8;
#if (XMC_VADC_BOUNDARY_FLAG_SELECT == 1U)
      uint32_t flag_output_condition_ch0 : 1;    /* Condition for which the boundary flag should change. */
      uint32_t flag_output_condition_ch1 : 1;    /* Condition for which the boundary flag should change. */
      uint32_t flag_output_condition_ch2 : 1;    /* Condition for which the boundary flag should change. */
      uint32_t flag_output_condition_ch3 : 1;    /* Condition for which the boundary flag should change. */
#else
      uint32_t                           : 4;
#endif
      uint32_t                           : 4;
#if (XMC_VADC_BOUNDARY_FLAG_SELECT == 1U)
      uint32_t invert_boundary_flag_ch0  : 1;    /* Inverts boundary flag output. */
      uint32_t invert_boundary_flag_ch1  : 1;    /* Inverts boundary flag output. */
      uint32_t invert_boundary_flag_ch2  : 1;    /* Inverts boundary flag output. */
      uint32_t invert_boundary_flag_ch3  : 1;    /* Inverts boundary flag output. */
#else
      uint32_t boundary_flag_output_ch0  : 1;    /* Enable the boundary flag output on the specific channel. */
      uint32_t boundary_flag_output_ch1  : 1;    /* Enable the boundary flag output on the specific channel. */
      uint32_t boundary_flag_output_ch2  : 1;    /* Enable the boundary flag output on the specific channel. */
      uint32_t boundary_flag_output_ch3  : 1;    /* Enable the boundary flag output on the specific channel. */
#endif
      uint32_t                           : 12;
    };
    uint32_t bfl;
  };

#if (XMC_VADC_BOUNDARY_FLAG_SELECT == 1U)
  /* GxBFLC register config */

  union
  {
    struct
    {
      uint32_t boundary_flag_mode_ch0    : 4;     /* Specify the basic operation of boundary flag 0 */
      uint32_t boundary_flag_mode_ch1    : 4;     /* Specify the basic operation of boundary flag 1 */
      uint32_t boundary_flag_mode_ch2    : 4;     /* Specify the basic operation of boundary flag 2 */
      uint32_t boundary_flag_mode_ch3    : 4;     /* Specify the basic operation of boundary flag 3 */
      uint32_t                           : 16;
    };
    uint32_t bflc;
  };

#endif

  bool    channel_priority;              /* Only non priority channels can be converted by Background Request Source */
  int8_t  alias_channel;                 /* Specifies the channel which has to be aliased with CH0/CH1.
                                          * Uses vadc_channel_alias_t */
} vadc_channel_config_t;

/**
 *  Structure handle the VADC scan request source.
 *  It can be either Channel Scan (0) or Background Scan (2).
 */

typedef struct
{
  uint32_t conv_start_mode        : 2;   /* This field determines how scan request source would request for
                                          * Uses vadc_startmode_t */
  uint32_t req_src_priority       : 2;   /* Request source priority for the arbiter. */

  /* BRSCTRL (Background Request) or GxASCTRL (Autoscan Source) register config */

  union
  {
    struct
    {
#if(XMC_VADC_GROUP_SRCREG_AVAILABLE == (1U))
      uint32_t src_specific_result_reg     : 4;     /* Use any one Group related result register as the destination
                                                     * for all conversions results. To use the  individual result register
                                                     * from each channel configuration, configure this field with 0x0 */
#else
      uint32_t                    : 4;
#endif
      uint32_t                    : 4;
      uint32_t trigger_signal     : 4;   /* Select one of the 16 possibilities for trigger. */
      uint32_t                    : 1;
      uint32_t trigger_edge       : 2;   /* Edge selection for trigger signal. */
      uint32_t                    : 1;
      uint32_t gate_signal        : 4;   /* Select one of the 16 possibilities for gating. */
      uint32_t                    : 8;
      uint32_t timer_mode         : 1;   /* Decides whether timer mode for equi-distant sampling shall be activated or not. */
      uint32_t                    : 3;
    };
    uint32_t asctrl;
  };

  /* BRSMR (Background Request) or GxASMR (Autoscan Source) register config */

  union
  {
    struct
    {
      uint32_t                    : 2;
      uint32_t external_trigger   : 1;   /* Conversions be initiated by external hardware trigger */
      uint32_t req_src_interrupt  : 1;   /* Request source event can be generated after a conversion sequence. */
      uint32_t enable_auto_scan   : 1;   /* Enables the continuous conversion mode. */
      uint32_t load_mode          : 1;   /* Selects load event mode. */
      uint32_t                    : 26;
    };
    uint32_t asmr;
  };
} vadc_scan_config_t;

typedef vadc_scan_config_t vadc_background_config_t;

/* Defines the power mode that can be selected for each group. */

typedef enum
{
  XMC_VADC_GROUP_POWERMODE_OFF       = 0,   /* Group is powered down */
  XMC_VADC_GROUP_POWERMODE_RESERVED1,       /* Reserved */
  XMC_VADC_GROUP_POWERMODE_RESERVED2,       /* Reserved */
  XMC_VADC_GROUP_POWERMODE_NORMAL    = 3    /* Group is powered up */
} vadc_group_powermode_t;

/* Defines the conversion input classes that can be selected for each channel. */

typedef enum
{
  XMC_VADC_CHANNEL_CONV_GROUP_CLASS0 = 0, /* Iclass0 specific to the group */
  XMC_VADC_CHANNEL_CONV_GROUP_CLASS1,     /* IClass1 specific to the group */
  XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0,    /* Iclass0 Module wide */
  XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS1     /* IClass1 Module wide */
} vadc_channel_conv_t;

/* Defines channel alias for channels 0 and 1.
 * Other Channels can accept only XMC_VADC_CHANNEL_ALIAS_DISABLED.
 */

typedef enum
{
  XMC_VADC_CHANNEL_ALIAS_DISABLED = -1,
  XMC_VADC_CHANNEL_ALIAS_CH0 = 0,
  XMC_VADC_CHANNEL_ALIAS_CH1 = 1,
  XMC_VADC_CHANNEL_ALIAS_CH2 = 2,
  XMC_VADC_CHANNEL_ALIAS_CH3 = 3,
  XMC_VADC_CHANNEL_ALIAS_CH4 = 4,
  XMC_VADC_CHANNEL_ALIAS_CH5 = 5,
  XMC_VADC_CHANNEL_ALIAS_CH6 = 6,
  XMC_VADC_CHANNEL_ALIAS_CH7 = 7
} vadc_channel_alias_t;

/* Defines the conversion mode. It defines the resolution of conversion. */

typedef enum
{
  XMC_VADC_CONVMODE_12BIT       = 0,  /* Results of conversion are 12bits wide */
  XMC_VADC_CONVMODE_10BIT       = 1,  /* Results of conversion are 10bits wide */
  XMC_VADC_CONVMODE_8BIT        = 2,  /* Results of conversion are 8bits wide */
  XMC_VADC_CONVMODE_FASTCOMPARE = 5   /* Input signal compared with a preset range */
} vadc_convmode_t;

/* Defines the mode of operation of a channel, when an ongoing conversion gets interrupted in between. */

typedef enum
{
  XMC_VADC_STARTMODE_WFS = 0, /* An ongoing conversion completes without interruption */
  XMC_VADC_STARTMODE_CIR,     /* An ongoing conversion can be interrupted and resumed later */
  XMC_VADC_STARTMODE_CNR      /* An ongoing conversion can be interrupted and never resumed, check GxASMR.RPTDIS */
} vadc_startmode_t;

/* Defines the condition for gating the conversion requests. It can be used to set the ENGT field
 * of ASMR/BSMR/QMR register respectively for auto_scan/background_scan/queue request sources.
 */

typedef enum
{
  XMC_VADC_GATEMODE_BLOCK = 0,  /* External triggers are permanently blocked */
  XMC_VADC_GATEMODE_IGNORE,     /* External triggers are unconditionally passed */
  XMC_VADC_GATEMODE_ACTIVEHIGH, /* External trigger is passed only if the gate signal is high */
  XMC_VADC_GATEMODE_ACTIVELOW   /* External trigger is passed only if the gate signal is low */
} vadc_gatemode_t;

/********************************************************************************************************
 * Private Function
 ********************************************************************************************************/

static inline bool xmc_vadc_check_group_ptr(vadc_group_t *const group_ptr)
{
  return ((group_ptr == VADC_G0) || (group_ptr == VADC_G1)
            || (group_ptr == VADC_G2) || (group_ptr == VADC_G3));
}

/********************************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************************/

/********************************************************************************************************
 * Name: xmc4_vadc_global_enable
 *
 * Description:
 *   Ungate the clock to the VADC module (if applicable) and bring the VADC
 *   module out of reset state.
 *   Called in xmc4_vadc_global_initialize.
 *
 ********************************************************************************************************/

void xmc4_vadc_global_enable(void);

/********************************************************************************************************
 * Name: xmc4_vadc_global_disable
 *
 * Description:
 *   Gate the clock to the VADC module (if applicable) and put the VADC
 *   module into the reset state
 *
 ********************************************************************************************************/

void xmc4_vadc_global_disable(void);

/********************************************************************************************************
 * Name: xmc4_vadc_global_initialize
 *
 * Description:
 *   Initializes the VADC global module with the associated
 *   configuration structure pointed by config. It initializes global input
 *   classes, boundaries , result resources by setting
 *   GLOBICLASS,GLOBBOUND,GLOBRCR registers. It also configures the global
 *   analog and digital clock dividers by setting GLOBCFG register.
 *
 ********************************************************************************************************/

void xmc4_vadc_global_initialize(const vadc_global_config_t *config);

/********************************************************************************************************
 * Name: xmc4_vadc_group_initialize
 *
 * Description:
 *   Initializes the VADC group module with the associated
 *   configuration structure pointed by config. It initializes group conversion
 *   class, arbiter configuration, boundary configuration by setting
 *   GxICLASS,GxARBCFG,GxBOUND, registers.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_group_initialize(vadc_group_t *const group_ptr, const vadc_group_config_t *config);

/********************************************************************************************************
 * Name: xmc4_vadc_group_set_powermode
 *
 * Description:
 *   Configures the power mode of a VADC group. For a VADC group to
 *   actually convert an analog signal, its analog converter must be turned on.
 *   Configure the register bit field GxARBCFG.ANONC
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_group_set_powermode(vadc_group_t *const group_ptr,
                                    const vadc_group_powermode_t power_mode);

/********************************************************************************************************
 * Name: xmc4_vadc_global_start_calibration
 *
 * Description:
 *   Start the calibration process and loops until all active groups
 *   finish calibration. Call xmc4_vadc_global_enable and
 *   xmc4_vadc_global_initialize before calibration.
 *   Configures the register bit field GLOBCFG.SUCAL.
 *
 ********************************************************************************************************/

void xmc4_vadc_global_start_calibration(void);

/********************************************************************************************************
 * Name: xmc4_vadc_group_background_enable_arbitrationslot
 *
 * Description:
 *   Enables arbitration slot of the Background request source to
 *   participate in the arbitration round. Even if a load event occurs the
 *   Background channel can only be converted when the arbiter comes to the
 *   Background slot. Thus this must be enabled if any conversion need to take
 *   place.
 *   Configure the register bit field GxARBPR.ASEN2.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_group_background_enable_arbitrationslot(vadc_group_t *const group_ptr);

/********************************************************************************************************
 * Name: xmc4_vadc_group_background_disable_arbitrationslot
 *
 * Description:
 *   Disables arbitration slot of the Background request source to
 *   participate in the arbitration round. Even if a load event occurs the
 *   Background channel can only be converted when the arbiter comes to the
 *   Background slot. Thus this must be enabled if any conversion need to take
 *   place.
 *   Configure the register bit field GxARBPR.ASEN2.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_group_background_disable_arbitrationslot(vadc_group_t *const group_ptr);

/********************************************************************************************************
 * Name: xmc4_vadc_global_background_initialize
 *
 * Description:
 *   Initializes the Background scan functional block. The BACKGROUND
 *   SCAN request source functional block converts channels of all VADC groups
 *   that have not been assigned as a priority channel (priority channels can be
 *   converted only by queue and scan). Related arbitration slot must be
 *   disabled to configure background request, then re-enabled.
 *
 ********************************************************************************************************/

void xmc4_vadc_global_background_initialize(const vadc_background_config_t *config);

/********************************************************************************************************
 * Name: xmc4_vadc_group_channel_initialize
 *
 * Description:
 *   Initializes the ADC channel for conversion. Must be called after
 *   request source initialization for each channel to enable their conversion.
 *   Configures registers GxCHCTRy and boundary flag GxBFL, GxBFLC and GxCHASS.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_group_channel_initialize(vadc_group_t *const group_ptr,
                                        const uint32_t ch_num,
                                        const vadc_channel_config_t *config);

/********************************************************************************************************
 * Name: xmc4_vadc_global_background_add_channel_to_sequence
 *
 * Description:
 *   Adds a channel to the background scan sequence. The pending
 *   register are updated only after a new load event occured.
 *   Configures the register bit fields of BRSSEL.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_global_background_add_channel_to_sequence(const uint32_t grp_num,
                                                        const uint32_t ch_num);

/********************************************************************************************************
 * Name: xmc4_vadc_global_background_enable_autoscan
 *
 * Description:
 *   Enables continuous conversion mode (autoscan). Once all channels
 *   belonging to a Background request source have been converted, a new  load
 *   event occurs.
 *   Configures the register bit field BRSMR.SCAN.
 *
 ********************************************************************************************************/

void xmc4_vadc_global_background_enable_autoscan(void);

/********************************************************************************************************
 * Name: xmc4_vadc_global_background_start_conversion
 *
 * Description:
 *   Generates conversion request (Software initiated conversion).
 *   The background scan must been init.
 *   Configures the register bit field BRSMR.LDEV.
 *
 ********************************************************************************************************/

void xmc4_vadc_global_background_start_conversion(void);

/********************************************************************************************************
 * Name: xmc4_vadc_group_get_result
 *
 * Description:
 *   Returns the result of the conversion in the given group result register.
 *   Get the register bit field GxRES.RESULT.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_group_get_result(vadc_group_t *const group_ptr,
                                const uint32_t res_reg,
                                uint16_t *result_ptr);

/********************************************************************************************************
 * Name: xmc4_vadc_group_channel_get_result
 *
 * Description:
 *   Returns the result of the conversion of the given group channel.
 *   Get the register bit field GxRES.RESULT.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 ********************************************************************************************************/

int xmc4_vadc_group_get_channel_result(vadc_group_t *const group_ptr,
                                        const uint32_t ch_num,
                                        uint16_t *result_ptr);

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_VADC_H */
