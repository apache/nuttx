/****************************************************************************
 * arch/arm/src/nrf52/sdc/nrf.h
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

#ifndef __ARCH_ARM_SRC_NRF52_SDC_NRF_H
#define __ARCH_ARM_SRC_NRF52_SDC_NRF_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the only definition we need from NRFX. We could simply do a
 * typedef to an integer but we cannot ensure it will end up being the right
 * size, so we replicate all values here.
 */

typedef enum
{
  Reset_IRQn                             = -15,
  NonMaskableInt_IRQn                    = -14,
  HardFault_IRQn                         = -13,
  MemoryManagement_IRQn                  = -12,
  BusFault_IRQn                          = -11,
  UsageFault_IRQn                        = -10,
  SVCall_IRQn                            =  -5,
  DebugMonitor_IRQn                      =  -4,
  PendSV_IRQn                            =  -2,
  SysTick_IRQn                           =  -1,
  POWER_CLOCK_IRQn                       =   0,
  RADIO_IRQn                             =   1,
  UARTE0_UART0_IRQn                      =   2,
  SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn =   3,
  SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn =   4,
  NFCT_IRQn                              =   5,
  GPIOTE_IRQn                            =   6,
  SAADC_IRQn                             =   7,
  TIMER0_IRQn                            =   8,
  TIMER1_IRQn                            =   9,
  TIMER2_IRQn                            =  10,
  RTC0_IRQn                              =  11,
  TEMP_IRQn                              =  12,
  RNG_IRQn                               =  13,
  ECB_IRQn                               =  14,
  CCM_AAR_IRQn                           =  15,
  WDT_IRQn                               =  16,
  RTC1_IRQn                              =  17,
  QDEC_IRQn                              =  18,
  COMP_LPCOMP_IRQn                       =  19,
  SWI0_EGU0_IRQn                         =  20,
  SWI1_EGU1_IRQn                         =  21,
  SWI2_EGU2_IRQn                         =  22,
  SWI3_EGU3_IRQn                         =  23,
  SWI4_EGU4_IRQn                         =  24,
  SWI5_EGU5_IRQn                         =  25,
  TIMER3_IRQn                            =  26,
  TIMER4_IRQn                            =  27,
  PWM0_IRQn                              =  28,
  PDM_IRQn                               =  29,
  MWU_IRQn                               =  32,
  PWM1_IRQn                              =  33,
  PWM2_IRQn                              =  34,
  SPIM2_SPIS2_SPI2_IRQn                  =  35,
  RTC2_IRQn                              =  36,
  I2S_IRQn                               =  37,
  FPU_IRQn                               =  38
} IRQn_Type;

#endif /* __ARCH_ARM_SRC_NRF52_SDC_NRF_H */
