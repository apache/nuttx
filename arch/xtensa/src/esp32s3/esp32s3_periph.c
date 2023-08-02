/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_periph.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "hardware/esp32s3_syscon.h"
#include "hardware/esp32s3_system.h"

#include "esp32s3_reset_reasons.h"

#include "xtensa.h"

#include "esp32s3_periph.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t ref_counts[PERIPH_MODULE_MAX];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get the reset reason for CPU. */

extern esp32s3_periph_module_t esp_rom_get_reset_reason(int cpu_no);

/****************************************************************************
 * Name: esp32s3_periph_ll_get_clk_en_reg
 *
 * Description:
 *   Get module clock register through periph module
 *
 * Input Parameters:
 *   periph - Periph module (one of the esp32s3_periph_module_t values)
 *
 * Returned Value:
 *   Module clock register
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp32s3_periph_ll_get_clk_en_reg(
                              esp32s3_periph_module_t periph)
{
  switch (periph)
    {
      case PERIPH_DEDIC_GPIO_MODULE:
        return SYSTEM_CPU_PERI_CLK_EN_REG;
      case PERIPH_RNG_MODULE:
      case PERIPH_WIFI_MODULE:
      case PERIPH_BT_MODULE:
      case PERIPH_WIFI_BT_COMMON_MODULE:
      case PERIPH_BT_BASEBAND_MODULE:
      case PERIPH_BT_LC_MODULE:
        return SYSTEM_WIFI_CLK_EN_REG;
      case PERIPH_UART2_MODULE:
      case PERIPH_SDMMC_MODULE:
      case PERIPH_LCD_CAM_MODULE:
      case PERIPH_GDMA_MODULE:
      case PERIPH_HMAC_MODULE:
      case PERIPH_DS_MODULE:
      case PERIPH_AES_MODULE:
      case PERIPH_SHA_MODULE:
      case PERIPH_RSA_MODULE:
        return SYSTEM_PERIP_CLK_EN1_REG;
      default:
        return SYSTEM_PERIP_CLK_EN0_REG;
    }
}

/****************************************************************************
 * Name: esp32s3_periph_ll_get_clk_en_mask
 *
 * Description:
 *   Get module clock bit through periph module
 *
 * Input Parameters:
 *   periph - Periph module (one of the esp32s3_periph_module_t values)
 *
 * Returned Value:
 *   Module clock bit
 *
 ****************************************************************************/

static inline uint32_t IRAM_ATTR esp32s3_periph_ll_get_clk_en_mask(
                                      esp32s3_periph_module_t periph)
{
  switch (periph)
    {
      case PERIPH_SARADC_MODULE:
        return SYSTEM_APB_SARADC_CLK_EN;
      case PERIPH_RMT_MODULE:
        return SYSTEM_RMT_CLK_EN;
      case PERIPH_LEDC_MODULE:
        return SYSTEM_LEDC_CLK_EN;
      case PERIPH_UART0_MODULE:
        return SYSTEM_UART_CLK_EN;
      case PERIPH_UART1_MODULE:
        return SYSTEM_UART1_CLK_EN;
      case PERIPH_UART2_MODULE:
        return SYSTEM_UART2_CLK_EN;
      case PERIPH_USB_MODULE:
        return SYSTEM_USB_CLK_EN;
      case PERIPH_I2C0_MODULE:
        return SYSTEM_I2C_EXT0_CLK_EN;
      case PERIPH_I2C1_MODULE:
        return SYSTEM_I2C_EXT1_CLK_EN;
      case PERIPH_I2S0_MODULE:
        return SYSTEM_I2S0_CLK_EN;
      case PERIPH_I2S1_MODULE:
        return SYSTEM_I2S1_CLK_EN;
      case PERIPH_LCD_CAM_MODULE:
        return SYSTEM_LCD_CAM_CLK_EN;
      case PERIPH_TIMG0_MODULE:
        return SYSTEM_TIMERGROUP_CLK_EN;
      case PERIPH_TIMG1_MODULE:
        return SYSTEM_TIMERGROUP1_CLK_EN;
      case PERIPH_PWM0_MODULE:
        return SYSTEM_PWM0_CLK_EN;
      case PERIPH_PWM1_MODULE:
        return SYSTEM_PWM1_CLK_EN;
      case PERIPH_UHCI0_MODULE:
        return SYSTEM_UHCI0_CLK_EN;
      case PERIPH_UHCI1_MODULE:
        return SYSTEM_UHCI1_CLK_EN;
      case PERIPH_PCNT_MODULE:
        return SYSTEM_PCNT_CLK_EN;
      case PERIPH_SPI_MODULE:
        return SYSTEM_SPI01_CLK_EN;
      case PERIPH_SPI2_MODULE:
        return SYSTEM_SPI2_CLK_EN;
      case PERIPH_SPI3_MODULE:
        return SYSTEM_SPI3_CLK_EN;
      case PERIPH_SDMMC_MODULE:
        return SYSTEM_SDIO_HOST_CLK_EN;
      case PERIPH_TWAI_MODULE:
        return SYSTEM_TWAI_CLK_EN;
      case PERIPH_RNG_MODULE:
        return SYSTEM_WIFI_CLK_RNG_EN;
      case PERIPH_WIFI_MODULE:
        return SYSTEM_WIFI_CLK_WIFI_EN_M;
      case PERIPH_BT_MODULE:
        return SYSTEM_WIFI_CLK_BT_EN_M;
      case PERIPH_WIFI_BT_COMMON_MODULE:
        return SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M;
      case PERIPH_BT_BASEBAND_MODULE:
        return SYSTEM_BT_BASEBAND_EN;
      case PERIPH_BT_LC_MODULE:
        return SYSTEM_BT_LC_EN;
      case PERIPH_SYSTIMER_MODULE:
        return SYSTEM_SYSTIMER_CLK_EN;
      case PERIPH_DEDIC_GPIO_MODULE:
        return SYSTEM_CLK_EN_DEDICATED_GPIO;
      case PERIPH_GDMA_MODULE:
        return SYSTEM_DMA_CLK_EN;
      case PERIPH_AES_MODULE:
        return SYSTEM_CRYPTO_AES_CLK_EN;
      case PERIPH_SHA_MODULE:
        return SYSTEM_CRYPTO_SHA_CLK_EN;
      case PERIPH_RSA_MODULE:
        return SYSTEM_CRYPTO_RSA_CLK_EN;
      case PERIPH_HMAC_MODULE:
        return SYSTEM_CRYPTO_HMAC_CLK_EN;
      case PERIPH_DS_MODULE:
        return SYSTEM_CRYPTO_DS_CLK_EN;
      default:
        return 0;
    }
}

/****************************************************************************
 * Name: esp32s3_periph_ll_get_rst_en_reg
 *
 * Description:
 *   Get system reset register through periph module
 *
 * Input Parameters:
 *   periph - Periph module (one of the esp32s3_periph_module_t values)
 *
 * Returned Value:
 *   System reset register
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp32s3_periph_ll_get_rst_en_reg(
                              esp32s3_periph_module_t periph)
{
  switch (periph)
    {
      case PERIPH_DEDIC_GPIO_MODULE:
        return SYSTEM_CPU_PERI_RST_EN_REG;
      case PERIPH_RNG_MODULE:
      case PERIPH_WIFI_MODULE:
      case PERIPH_BT_MODULE:
      case PERIPH_WIFI_BT_COMMON_MODULE:
      case PERIPH_BT_BASEBAND_MODULE:
      case PERIPH_BT_LC_MODULE:
        return SYSTEM_CORE_RST_EN_REG;
      case PERIPH_UART2_MODULE:
      case PERIPH_SDMMC_MODULE:
      case PERIPH_LCD_CAM_MODULE:
      case PERIPH_GDMA_MODULE:
      case PERIPH_HMAC_MODULE:
      case PERIPH_DS_MODULE:
      case PERIPH_AES_MODULE:
      case PERIPH_SHA_MODULE:
      case PERIPH_RSA_MODULE:
        return SYSTEM_PERIP_RST_EN1_REG;
      default:
        return SYSTEM_PERIP_RST_EN0_REG;
    }
}

/****************************************************************************
 * Name: esp32s3_periph_ll_get_rst_en_mask
 *
 * Description:
 *   Get system reset bit through periph module
 *
 * Input Parameters:
 *   periph - Periph module (one of the esp32s3_periph_module_t values)
 *   enable - Whether hardware acceleration is enabled
 *
 * Returned Value:
 *   System reset bit
 *
 ****************************************************************************/

static inline uint32_t IRAM_ATTR esp32s3_periph_ll_get_rst_en_mask(
                                    esp32s3_periph_module_t periph,
                                    bool enable)
{
  switch (periph)
    {
      case PERIPH_SARADC_MODULE:
        return SYSTEM_APB_SARADC_RST;
      case PERIPH_RMT_MODULE:
        return SYSTEM_RMT_RST;
      case PERIPH_LEDC_MODULE:
        return SYSTEM_LEDC_RST;
      case PERIPH_WIFI_MODULE:
        return SYSTEM_WIFIMAC_RST;
      case PERIPH_BT_MODULE:
        return (SYSTEM_BTBB_RST | SYSTEM_BTBB_REG_RST |
                SYSTEM_RW_BTMAC_RST | SYSTEM_RW_BTLP_RST |
                SYSTEM_RW_BTMAC_REG_RST | SYSTEM_RW_BTLP_REG_RST);
      case PERIPH_UART0_MODULE:
        return SYSTEM_UART_RST;
      case PERIPH_UART1_MODULE:
        return SYSTEM_UART1_RST;
      case PERIPH_UART2_MODULE:
        return SYSTEM_UART2_RST;
      case PERIPH_USB_MODULE:
        return SYSTEM_USB_RST;
      case PERIPH_I2C0_MODULE:
        return SYSTEM_I2C_EXT0_RST;
      case PERIPH_I2C1_MODULE:
        return SYSTEM_I2C_EXT1_RST;
      case PERIPH_I2S0_MODULE:
        return SYSTEM_I2S0_RST;
      case PERIPH_I2S1_MODULE:
        return SYSTEM_I2S1_RST;
      case PERIPH_LCD_CAM_MODULE:
        return SYSTEM_LCD_CAM_RST;
      case PERIPH_TIMG0_MODULE:
        return SYSTEM_TIMERGROUP_RST;
      case PERIPH_TIMG1_MODULE:
        return SYSTEM_TIMERGROUP1_RST;
      case PERIPH_PWM0_MODULE:
        return SYSTEM_PWM0_RST;
      case PERIPH_PWM1_MODULE:
        return SYSTEM_PWM1_RST;
      case PERIPH_UHCI0_MODULE:
        return SYSTEM_UHCI0_RST;
      case PERIPH_UHCI1_MODULE:
        return SYSTEM_UHCI1_RST;
      case PERIPH_PCNT_MODULE:
        return SYSTEM_PCNT_RST;
      case PERIPH_SPI_MODULE:
        return SYSTEM_SPI01_RST;
      case PERIPH_SPI2_MODULE:
        return SYSTEM_SPI2_RST;
      case PERIPH_SPI3_MODULE:
        return SYSTEM_SPI3_RST;
      case PERIPH_SDMMC_MODULE:
        return SYSTEM_SDIO_HOST_RST;
      case PERIPH_TWAI_MODULE:
        return SYSTEM_TWAI_RST;
      case PERIPH_SYSTIMER_MODULE:
        return SYSTEM_SYSTIMER_RST;
      case PERIPH_DEDIC_GPIO_MODULE:
        return SYSTEM_RST_EN_DEDICATED_GPIO;
      case PERIPH_GDMA_MODULE:
        return SYSTEM_DMA_RST;
      case PERIPH_HMAC_MODULE:
        return SYSTEM_CRYPTO_HMAC_RST;
      case PERIPH_DS_MODULE:
        return SYSTEM_CRYPTO_DS_RST;
      case PERIPH_AES_MODULE:
        if (enable)
          {
            /* Clear reset on digital signature, otherwise AES unit is
             * held in reset also.
             */

            return (SYSTEM_CRYPTO_AES_RST | SYSTEM_CRYPTO_DS_RST);
          }
        else
          {
            /* Don't return other units to reset, as this pulls reset
             * on RSA & SHA units, respectively.
             */

            return SYSTEM_CRYPTO_AES_RST;
          }

      case PERIPH_SHA_MODULE:
        if (enable)
          {
            /* Clear reset on digital signature and HMAC, otherwise SHA is
             * held in reset
             */

            return (SYSTEM_CRYPTO_SHA_RST | SYSTEM_CRYPTO_DS_RST |
                    SYSTEM_CRYPTO_HMAC_RST) ;
          }
        else
          {
            /* Don't assert reset on secure boot, otherwise AES is
             * held in reset
             */

            return SYSTEM_CRYPTO_SHA_RST;
          }

      case PERIPH_RSA_MODULE:
        if (enable)
          {
            /* also clear reset on digital signature, otherwise RSA is
             * held in reset
             */

            return (SYSTEM_CRYPTO_RSA_RST | SYSTEM_CRYPTO_DS_RST);
          }
        else
          {
            /* don't reset digital signature unit, as this resets AES also */

            return SYSTEM_CRYPTO_RSA_RST;
          }

      default:
        return 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_periph_module_enable
 *
 * Description:
 *   Enable peripheral module
 *
 * Input Parameters:
 *   periph - Periph module (one of the esp32s3_periph_module_t values)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_periph_module_enable(esp32s3_periph_module_t periph)
{
  irqstate_t flags = enter_critical_section();

  ASSERT(periph < PERIPH_MODULE_MAX);
  if (ref_counts[periph] == 0)
    {
      modifyreg32(esp32s3_periph_ll_get_clk_en_reg(periph), 0,
                  esp32s3_periph_ll_get_clk_en_mask(periph));
      modifyreg32(esp32s3_periph_ll_get_rst_en_reg(periph),
                  esp32s3_periph_ll_get_rst_en_mask(periph, true), 0);
    }

  ref_counts[periph]++;

  leave_critical_section(flags);
}

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

void esp32s3_periph_module_disable(esp32s3_periph_module_t periph)
{
  irqstate_t flags = enter_critical_section();

  ASSERT(periph < PERIPH_MODULE_MAX);
  ref_counts[periph]--;
  if (ref_counts[periph] == 0)
    {
      modifyreg32(esp32s3_periph_ll_get_clk_en_reg(periph),
                  esp32s3_periph_ll_get_clk_en_mask(periph), 0);
      modifyreg32(esp32s3_periph_ll_get_rst_en_reg(periph), 0,
                  esp32s3_periph_ll_get_rst_en_mask(periph, false));
    }

  leave_critical_section(flags);
}

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

void esp32s3_periph_module_reset(esp32s3_periph_module_t periph)
{
  irqstate_t flags = enter_critical_section();

  ASSERT(periph < PERIPH_MODULE_MAX);

  modifyreg32(esp32s3_periph_ll_get_rst_en_reg(periph), 0,
              esp32s3_periph_ll_get_rst_en_mask(periph, false));
  modifyreg32(esp32s3_periph_ll_get_rst_en_reg(periph),
              esp32s3_periph_ll_get_rst_en_mask(periph, false), 0);

  leave_critical_section(flags);
}

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

void IRAM_ATTR esp32s3_periph_wifi_bt_common_module_enable(void)
{
  irqstate_t flags = enter_critical_section();

  if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0)
    {
      modifyreg32(SYSTEM_WIFI_CLK_EN_REG,
                  0, SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M);
      modifyreg32(SYSTEM_CORE_RST_EN_REG, 0, 0);
    }

  ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]++;

  leave_critical_section(flags);
}

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

void esp32s3_periph_wifi_bt_common_module_disable(void)
{
  irqstate_t flags = enter_critical_section();

  ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]--;

  if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0)
    {
      modifyreg32(SYSTEM_WIFI_CLK_EN_REG,
                  SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M, 0);
      modifyreg32(SYSTEM_CORE_RST_EN_REG, 0, 0);
    }

  leave_critical_section(flags);
}

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

void IRAM_ATTR esp32s3_perip_clk_init(void)
{
  uint32_t common_perip_clk;
  uint32_t common_perip_clk1 = 0;
  uint32_t hwcrypto_perip_clk;
  uint32_t wifi_bt_sdio_clk;

#ifndef CONFIG_SMP
  soc_reset_reason_t rst_reas[1];
#else
  soc_reset_reason_t rst_reas[2];
#endif

  rst_reas[0] = esp_rom_get_reset_reason(0);
#ifdef CONFIG_SMP
  rst_reas[1] = esp_rom_get_reset_reason(1);
#endif

  /* For reason that only reset CPU, do not disable the clocks
   * that have been enabled before reset.
   */

  if ((rst_reas[0] == RESET_REASON_CPU0_MWDT0 ||
       rst_reas[0] == RESET_REASON_CPU0_SW ||
       rst_reas[0] == RESET_REASON_CPU0_RTC_WDT ||
       rst_reas[0] == RESET_REASON_CPU0_MWDT1)
#ifdef CONFIG_SMP
      || (rst_reas[1] == RESET_REASON_CPU1_MWDT0 ||
          rst_reas[1] == RESET_REASON_CPU1_SW ||
          rst_reas[1] == RESET_REASON_CPU1_RTC_WDT ||
          rst_reas[1] == RESET_REASON_CPU1_MWDT1)
#endif
      )
    {
      common_perip_clk = ~getreg32(SYSTEM_PERIP_CLK_EN0_REG);
      hwcrypto_perip_clk = ~getreg32(SYSTEM_PERIP_CLK_EN1_REG);
      wifi_bt_sdio_clk = ~getreg32(SYSTEM_WIFI_CLK_EN_REG);
    }
  else
    {
      common_perip_clk = SYSTEM_WDG_CLK_EN |
                         SYSTEM_I2S0_CLK_EN |
#ifndef CONFIG_UART0_SERIAL_CONSOLE
                         SYSTEM_UART_CLK_EN |
#endif
#ifndef CONFIG_UART1_SERIAL_CONSOLE
                         SYSTEM_UART1_CLK_EN |
#endif
#ifndef CONFIG_UART2_SERIAL_CONSOLE
                         SYSTEM_UART2_CLK_EN |
#endif
                         SYSTEM_USB_CLK_EN |
                         SYSTEM_SPI2_CLK_EN |
                         SYSTEM_I2C_EXT0_CLK_EN |
                         SYSTEM_UHCI0_CLK_EN |
                         SYSTEM_RMT_CLK_EN |
                         SYSTEM_PCNT_CLK_EN |
                         SYSTEM_LEDC_CLK_EN |
                         SYSTEM_TIMERGROUP1_CLK_EN |
                         SYSTEM_SPI3_CLK_EN |
                         SYSTEM_SPI4_CLK_EN |
                         SYSTEM_PWM0_CLK_EN |
                         SYSTEM_TWAI_CLK_EN |
                         SYSTEM_PWM1_CLK_EN |
                         SYSTEM_I2S1_CLK_EN |
                         SYSTEM_SPI2_DMA_CLK_EN |
                         SYSTEM_SPI3_DMA_CLK_EN |
                         SYSTEM_PWM2_CLK_EN |
                         SYSTEM_PWM3_CLK_EN;
      common_perip_clk1 = 0;
      hwcrypto_perip_clk = SYSTEM_CRYPTO_AES_CLK_EN |
                           SYSTEM_CRYPTO_SHA_CLK_EN |
                           SYSTEM_CRYPTO_RSA_CLK_EN;
      wifi_bt_sdio_clk = SYSTEM_WIFI_CLK_WIFI_EN |
                         SYSTEM_WIFI_CLK_BT_EN_M |
                         SYSTEM_WIFI_CLK_I2C_CLK_EN |
                         SYSTEM_WIFI_CLK_UNUSED_BIT12 |
                         SYSTEM_WIFI_CLK_SDIO_HOST_EN;
    }

  /* Reset the communication peripherals like I2C, SPI, UART, I2S
   * and bring them to known state.
   */

  common_perip_clk |= SYSTEM_I2S0_CLK_EN |
#ifndef CONFIG_UART0_SERIAL_CONSOLE
                      SYSTEM_UART_CLK_EN |
#endif
#ifndef CONFIG_UART1_SERIAL_CONSOLE
                      SYSTEM_UART1_CLK_EN |
#endif
#ifndef CONFIG_UART2_SERIAL_CONSOLE
                      SYSTEM_UART2_CLK_EN |
#endif
                      SYSTEM_USB_CLK_EN |
                      SYSTEM_SPI2_CLK_EN |
                      SYSTEM_I2C_EXT0_CLK_EN |
                      SYSTEM_UHCI0_CLK_EN |
                      SYSTEM_RMT_CLK_EN |
                      SYSTEM_UHCI1_CLK_EN |
                      SYSTEM_SPI3_CLK_EN |
                      SYSTEM_SPI4_CLK_EN |
                      SYSTEM_I2C_EXT1_CLK_EN |
                      SYSTEM_I2S1_CLK_EN |
                      SYSTEM_SPI2_DMA_CLK_EN |
                      SYSTEM_SPI3_DMA_CLK_EN;
  common_perip_clk1 = 0;

  /* Disable some peripheral clocks. */

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, common_perip_clk, 0);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, common_perip_clk);

  modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, common_perip_clk1, 0);
  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, 0, common_perip_clk1);

  /* Disable hardware crypto clocks. */

  modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, hwcrypto_perip_clk, 0);
  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, 0, hwcrypto_perip_clk);

  /* Force clear backup dma reset signal. This is a fix to the backup dma
   * implementation in the ROM, the reset signal was not cleared when the
   * backup dma was started, which caused the backup dma operation to fail.
   */

  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_PERI_BACKUP_RST, 0);

  /* Disable WiFi/BT/SDIO clocks. */

  modifyreg32(SYSTEM_WIFI_CLK_EN_REG, wifi_bt_sdio_clk, 0);
  modifyreg32(SYSTEM_WIFI_CLK_EN_REG, 0, SYSTEM_WIFI_CLK_EN);

  /* Set WiFi light sleep clock source to RTC slow clock */

  REG_SET_FIELD(SYSTEM_BT_LPCK_DIV_INT_REG, SYSTEM_BT_LPCK_DIV_NUM, 0);
  modifyreg32(SYSTEM_BT_LPCK_DIV_FRAC_REG, SYSTEM_LPCLK_SEL_8M, 0);
  modifyreg32(SYSTEM_BT_LPCK_DIV_FRAC_REG, 0, SYSTEM_LPCLK_SEL_RTC_SLOW);

  /* Enable RNG clock. */

  esp32s3_periph_module_enable(PERIPH_RNG_MODULE);

  /* Enable TimerGroup 0 clock to ensure its reference counter will never
   * be decremented to 0 during normal operation and preventing it from
   * being disabled.
   * If the TimerGroup 0 clock is disabled and then reenabled, the watchdog
   * registers (Flashboot protection included) will be reenabled, and some
   * seconds later, will trigger an unintended reset.
   */

  esp32s3_periph_module_enable(PERIPH_TIMG0_MODULE);
}
