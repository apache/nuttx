/****************************************************************************
 * arch/risc-v/src/bl602/bl602_rtc.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_RTC_H
#define __ARCH_RISCV_SRC_BL602_BL602_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL602_HBN_32K_RC               0         /* HBN use rc 32k */
#define BL602_HBN_32K_XTAL             1         /* HBN use xtal 32k */
#define BL602_HBN_32K_DIG              3         /* HBN use dig 32k */

#define BL602_HBN_RTC_INT_DELAY_32T    0         /* HBN RTC int delay 32T */
#define BL602_HBN_RTC_INT_DELAY_0T     1         /* HBN RTC int delay 0T */

#define BL602_HBN_RTC_COMP_BIT0_39     0x01      /* RTC COMP mode bit0-39 */
#define BL602_HBN_RTC_COMP_BIT0_23     0x02      /* RTC COMP mode bit0-23 */
#define BL602_HBN_RTC_COMP_BIT13_39    0x04      /* RTC COMP mode bit13-39 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_hbn_sel
 *
 * Description:
 *   HBN select 32K
 *
 * Input Parameters:
 *   clk_type: HBN 32k clock type
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_sel(uint8_t clk_type);

/****************************************************************************
 * Name: bl602_hbn_clear_rtc_int
 *
 * Description:
 *   HBN clear RTC timer interrupt,this function must be called to clear
 *   delayed rtc IRQ
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_clear_rtc_int(void);

/****************************************************************************
 * Name: bl602_hbn_set_rtc_timer
 *
 * Description:
 *   HBN set RTC timer configuration
 *
 * Input Parameters:
 *   delay: RTC interrupt delay 32 clocks
 *   compval_low: RTC interrupt commpare value low 32 bits
 *   compval_high: RTC interrupt commpare value high 32 bits
 *   comp_mode: RTC interrupt commpare
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_set_rtc_timer(uint8_t delay_type, uint32_t compval_low,
                                 uint32_t compval_high,  uint8_t comp_mode);

/****************************************************************************
 * Name: bl602_hbn_clear_rtc_counter
 *
 * Description:
 *   HBN set RTC timer configuration
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_clear_rtc_counter(void);

/****************************************************************************
 * Name: bl602_hbn_enable_rtc_counter
 *
 * Description:
 *   HBN clear RTC timer counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_enable_rtc_counter(void);

/****************************************************************************
 * Name: bl602_hbn_get_rtc_timer_val
 *
 * Description:
 *   HBN get RTC timer count value
 *
 * Input Parameters:
 *   val_low: RTC count value pointer for low 32 bits
 *   val_high: RTC count value pointer for high 8 bits
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_get_rtc_timer_val(uint32_t *val_low, uint32_t *val_high);

/****************************************************************************
 * Name: bl602_rtc_lowerhalf_initialize
 *
 * Description:
 *   None.
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the BL602 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *bl602_rtc_lowerhalf_initialize(void);

#endif /* __ARCH_RISCV_SRC_BL602_BL602_RTC_H */
