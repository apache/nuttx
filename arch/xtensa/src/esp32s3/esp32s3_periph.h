/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_periph.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PERIPH_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PERIPH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  PERIPH_LEDC_MODULE = 0,
  PERIPH_UART0_MODULE,
  PERIPH_UART1_MODULE,
  PERIPH_UART2_MODULE,
  PERIPH_USB_MODULE,
  PERIPH_I2C0_MODULE,
  PERIPH_I2C1_MODULE,
  PERIPH_I2S0_MODULE,
  PERIPH_I2S1_MODULE,
  PERIPH_LCD_CAM_MODULE,
  PERIPH_TIMG0_MODULE,
  PERIPH_TIMG1_MODULE,
  PERIPH_PWM0_MODULE,
  PERIPH_PWM1_MODULE,
  PERIPH_PWM2_MODULE,
  PERIPH_PWM3_MODULE,
  PERIPH_UHCI0_MODULE,
  PERIPH_UHCI1_MODULE,
  PERIPH_RMT_MODULE,
  PERIPH_PCNT_MODULE,
  PERIPH_SPI_MODULE,
  PERIPH_SPI2_MODULE,
  PERIPH_SPI3_MODULE,
  PERIPH_SDMMC_MODULE,
  PERIPH_TWAI_MODULE,
  PERIPH_RNG_MODULE,
  PERIPH_WIFI_MODULE,
  PERIPH_BT_MODULE,
  PERIPH_WIFI_BT_COMMON_MODULE,
  PERIPH_BT_BASEBAND_MODULE,
  PERIPH_BT_LC_MODULE,
  PERIPH_AES_MODULE,
  PERIPH_SHA_MODULE,
  PERIPH_HMAC_MODULE,
  PERIPH_DS_MODULE,
  PERIPH_RSA_MODULE,
  PERIPH_SYSTIMER_MODULE,
  PERIPH_GDMA_MODULE,
  PERIPH_DEDIC_GPIO_MODULE,
  PERIPH_SARADC_MODULE,
  PERIPH_TEMPSENSOR_MODULE,
  PERIPH_MODULE_MAX
} esp32s3_periph_module_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_periph_module_enable
 *
 * Description:
 *   Enable peripheral module
 *
 * Input Parameters:
 *   periph - Periph module (one of enum esp32s3_periph_module_t values)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_periph_module_enable(esp32s3_periph_module_t periph);

/****************************************************************************
 * Name: esp32s3_periph_module_disable
 *
 * Description:
 *   Disable peripheral module
 *
 * Input Parameters:
 *   periph - Periph module (one of enum esp32s3_periph_module_t values)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_periph_module_disable(esp32s3_periph_module_t periph);

/****************************************************************************
 * Name: esp32s3_periph_module_reset
 *
 * Description:
 *   Reset peripheral module by asserting and de-asserting the reset signal.
 *
 * Input Parameters:
 *   periph - Periph module (one of the esp32s3_periph_module_t values)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_periph_module_reset(esp32s3_periph_module_t periph);

/****************************************************************************
 * Name: esp32s3_periph_wifi_bt_common_module_enable
 *
 * Description:
 *   Enable Wi-Fi and BT common module.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_periph_wifi_bt_common_module_enable(void);

/****************************************************************************
 * Name: esp32s3_periph_wifi_bt_common_module_disable
 *
 * Description:
 *   Disable Wi-Fi and BT common module.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_periph_wifi_bt_common_module_disable(void);

/****************************************************************************
 * Name:  esp32s3_perip_clk_init
 *
 * Description:
 *   This function disables clock of useless peripherals when cpu starts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_perip_clk_init(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PERIPH_H */
