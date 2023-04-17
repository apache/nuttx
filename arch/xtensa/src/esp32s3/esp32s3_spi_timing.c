/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spi_timing.c
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

#include <nuttx/config.h>

#include <sys/param.h>

#include "xtensa.h"
#include "esp32s3_gpio.h"
#include "esp32s3_psram.h"
#include "esp32s3_spi_timing.h"
#include "hardware/esp32s3_spi_mem_reg.h"
#include "hardware/esp32s3_iomux.h"
#include "hardware/esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "rom/esp32s3_spiflash.h"
#include "rom/esp32s3_opi_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CORE_CLK_REG_SEL_80M    0
#define CORE_CLK_REG_SEL_120M   1
#define CORE_CLK_REG_SEL_160M   2
#define CORE_CLK_REG_SEL_240M   3

#if defined(CONFIG_ESP32S3_FLASH_SAMPLE_MODE_DTR) || \
    defined(CONFIG_ESP32S3_SPIRAM_MODE_OCT)
#  define ESP32S3_SPI_TIMING_CORE_CLOCK_DIV         2
#else
#  define ESP32S3_SPI_TIMING_CORE_CLOCK_DIV         1
#endif

#if defined(CONFIG_ESP32S3_FLASH_SAMPLE_MODE_DTR)
#  if defined(CONFIG_ESP32S3_FLASH_FREQ_80M) 
#    define ESP32S3_SPI_TIMING_FLASH_CORE_CLK       160
#  elif defined(CONFIG_ESP32S3_FLASH_FREQ_120M)
#    define ESP32S3_SPI_TIMING_FLASH_CORE_CLK       240
#  endif
#elif defined(CONFIG_ESP32S3_FLASH_SAMPLE_MODE_STR)
#  if defined(CONFIG_ESP32S3_FLASH_FREQ_120M)
#    if ESP32S3_SPI_TIMING_CORE_CLOCK_DIV == 1
#      define ESP32S3_SPI_TIMING_FLASH_CORE_CLK     120
#    elif ESP32S3_SPI_TIMING_CORE_CLOCK_DIV == 2
#      define ESP32S3_SPI_TIMING_FLASH_CORE_CLK     240
#    endif
#  endif
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_MODE_OCT)
#  if defined(CONFIG_ESP32S3_SPIRAM_SPEED_80M)
#    define ESP32S3_SPI_TIMING_PSRAM_CORE_CLK       160
#  endif
#elif defined(ESP32S3_SPI_TIMING_PSRAM_STR_MODE)
#  if defined(CONFIG_ESP32S3_SPIRAM_SPEED_120M)
#    if ESP32S3_SPI_TIMING_CORE_CLOCK_DIV == 1
#      define ESP32S3_SPI_TIMING_PSRAM_CORE_CLK     120
#    elif ESP32S3_SPI_TIMING_CORE_CLOCK_DIV == 2
#      define ESP32S3_SPI_TIMING_PSRAM_CORE_CLK     240
#    endif
#  endif
#endif

#if ESP32S3_SPI_TIMING_FLASH_TUNING
#  if ESP32S3_SPI_TIMING_PSRAM_TUNING
#    if ESP32S3_SPI_TIMING_FLASH_CORE_CLK != ESP32S3_SPI_TIMING_PSRAM_CORE_CLK
#      error "FLASH and PSRAM Mode configuration are not supported"
#    endif
#    define ESP32S3_SPI_TIMING_CORE_CLK     ESP32S3_SPI_TIMING_FLASH_CORE_CLK
#  else
#    if ESP32S3_SPI_TIMING_FLASH_CORE_CLK % ESP32S3_SPI_TIMING_PSRAM_CLOCK != 0
#      error "FLASH and PSRAM Mode configuration are not supported"
#    endif
#    define ESP32S3_SPI_TIMING_CORE_CLK     ESP32S3_SPI_TIMING_FLASH_CORE_CLK
#  endif
#else
#  if ESP32S3_SPI_TIMING_PSRAM_TUNING
#    if ESP32S3_SPI_TIMING_PSRAM_CORE_CLK % ESP32S3_SPI_TIMING_FLASH_CLOCK != 0
#      error "FLASH and PSRAM Mode configuration are not supported"
#    endif
#    define ESP32S3_SPI_TIMING_CORE_CLK     ESP32S3_SPI_TIMING_PSRAM_CORE_CLK
#  else
#    define ESP32S3_SPI_TIMING_CORE_CLK     80
#  endif
#endif

#define ESP32S3_CHECK_POWER_OF_2(n)         ((((n) & ((~(n)) + 1))) == (n))

#ifdef CONFIG_ESP32S3_FLASH_SAMPLE_MODE_DTR
#  if ESP32S3_CHECK_POWER_OF_2(ESP32S3_SPI_TIMING_CORE_CLK / ESP32S3_SPI_TIMING_FLASH_CLOCK) == 0
#    error "FLASH and PSRAM Mode configuration are not supported"
#  endif
#endif

#ifdef CONFIG_ESP32S3_SPIRAM_MODE_OCT
#  if ESP32S3_CHECK_POWER_OF_2(ESP32S3_SPI_TIMING_CORE_CLK / ESP32S3_SPI_TIMING_PSRAM_CLOCK) == 0
#    error "FLASH and PSRAM Mode configuration are not supported"
#  endif
#endif

#if ESP32S3_SPI_TIMING_CORE_CLK == 80
#  define DEFAULT_CORE_CLOCK    CORE_CLOCK_80M
#  define DEFAULT_CORE_CLK_REG  CORE_CLK_REG_SEL_80M
#elif ESP32S3_SPI_TIMING_CORE_CLK == 120
#  define DEFAULT_CORE_CLOCK    CORE_CLOCK_120M
#  define DEFAULT_CORE_CLK_REG  CORE_CLK_REG_SEL_120M
#elif ESP32S3_SPI_TIMING_CORE_CLK == 160
#  define DEFAULT_CORE_CLOCK    CORE_CLOCK_160M
#  define DEFAULT_CORE_CLK_REG  CORE_CLK_REG_SEL_160M
#elif ESP32S3_SPI_TIMING_CORE_CLK == 240
#  define DEFAULT_CORE_CLOCK    CORE_CLOCK_240M
#  define DEFAULT_CORE_CLK_REG  CORE_CLK_REG_SEL_240M
#else
#  error "SPI timing core clock is invalid"
#endif

#if defined(CONFIG_ESP32S3_FLASH_FREQ_20M)
#  define FLASH_CLOCK_DIVIDER (ESP32S3_SPI_TIMING_CORE_CLK / 20)
#elif defined(CONFIG_ESP32S3_FLASH_FREQ_40M)
#  define FLASH_CLOCK_DIVIDER (ESP32S3_SPI_TIMING_CORE_CLK / 40)
#elif defined(CONFIG_ESP32S3_FLASH_FREQ_80M)
#  define FLASH_CLOCK_DIVIDER (ESP32S3_SPI_TIMING_CORE_CLK / 80)
#elif defined(CONFIG_ESP32S3_FLASH_FREQ_120M)
#  define FLASH_CLOCK_DIVIDER (ESP32S3_SPI_TIMING_CORE_CLK / 120)
#else
#  error "SPI timing flash clock is invalid"
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_SPEED_40M)
#  define PSRAM_CLOCK_DIVIDER (ESP32S3_SPI_TIMING_CORE_CLK / 40)
#elif defined(CONFIG_ESP32S3_SPIRAM_SPEED_80M)
#  define PSRAM_CLOCK_DIVIDER (ESP32S3_SPI_TIMING_CORE_CLK / 80)
#elif defined(CONFIG_ESP32S3_SPIRAM_SPEED_120M)
#  define PSRAM_CLOCK_DIVIDER (ESP32S3_SPI_TIMING_CORE_CLK / 120)
#else
#  define PSRAM_CLOCK_DIVIDER 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum core_clock_e
{
  CORE_CLOCK_80M  = 0,
  CORE_CLOCK_120M = 1,
  CORE_CLOCK_160M = 2,
  CORE_CLOCK_240M = 3
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: set_psram_clock
 *
 * Description:
 *   Set PSRAM clock.
 *
 * Input Parameters:
 *   spi_num - SPI port
 *   freqdiv - SPI clock divideor
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_psram_clock(uint8_t spi_num, uint32_t freqdiv)
{
  if (freqdiv == 1)
    {
      WRITE_PERI_REG(SPI_MEM_SRAM_CLK_REG(spi_num), SPI_MEM_SCLK_EQU_SYSCLK);
    }
  else
    {
      uint32_t freqbits = ((freqdiv - 1) << SPI_MEM_SCLKCNT_N_S) |
                          ((freqdiv / 2 - 1) << SPI_MEM_SCLKCNT_H_S) |
                          ((freqdiv - 1) << SPI_MEM_SCLKCNT_L_S);

      WRITE_PERI_REG(SPI_MEM_SRAM_CLK_REG(spi_num), freqbits);
    }
}

/****************************************************************************
 * Name: set_flash_clock
 *
 * Description:
 *   Set flash clock.
 *
 * Input Parameters:
 *   spi_num - SPI port
 *   freqdiv - SPI clock divideor
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR set_flash_clock(uint8_t spi_num, uint32_t freqdiv)
{
  DEBUGASSERT(freqdiv > 0);

  if (freqdiv == 1)
    {
      WRITE_PERI_REG(SPI_MEM_CLOCK_REG(spi_num), SPI_MEM_CLK_EQU_SYSCLK);
    }
  else
    {
      uint32_t freqbits = ((freqdiv - 1) << SPI_MEM_CLKCNT_N_S) |
                          ((freqdiv / 2 - 1) << SPI_MEM_CLKCNT_H_S) |
                          ((freqdiv - 1) << SPI_MEM_CLKCNT_L_S);

      WRITE_PERI_REG(SPI_MEM_CLOCK_REG(spi_num), freqbits);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_spi_timing_set_pin_drive_strength
 *
 * Description:
 *   Make SPI all GPIO strength to be 3 under default clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_spi_timing_set_pin_drive_strength(void)
{
  const uint32_t regs[] =
    {
      IO_MUX_GPIO27_REG,
      IO_MUX_GPIO28_REG,
      IO_MUX_GPIO31_REG,
      IO_MUX_GPIO32_REG,
      IO_MUX_GPIO33_REG,
      IO_MUX_GPIO34_REG,
      IO_MUX_GPIO35_REG,
      IO_MUX_GPIO36_REG,
      IO_MUX_GPIO37_REG
    };

  /* Set default clock */

  SET_PERI_REG_MASK(SPI_MEM_DATE_REG(0), SPI_MEM_SPICLK_PAD_DRV_CTL_EN);
  REG_SET_FIELD(SPI_MEM_DATE_REG(0), SPI_MEM_SPI_SMEM_SPICLK_FUN_DRV, 3);
  REG_SET_FIELD(SPI_MEM_DATE_REG(0), SPI_MEM_SPI_FMEM_SPICLK_FUN_DRV, 3);

  /* Set default mspi d0 ~ d7, dqs pin drive strength */

  for (int i = 0; i < nitems(regs); i++)
    {
      PIN_SET_DRV(regs[i], 3);
    }
}

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_high_speed
 *
 * Description:
 *   Make MSPI work under the frequency as users set, may add certain
 *   delays to MSPI RX direction to meet timing requirements.
 *
 * Input Parameters:
 *   spi1 - Select whether to control SPI1
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_spi_timing_set_mspi_high_speed(bool spi1)
{
  uint32_t flash_div = FLASH_CLOCK_DIVIDER;
  uint32_t psram_div = PSRAM_CLOCK_DIVIDER;

  /* Set SPI0 & 1 core clock */

  REG_SET_FIELD(SPI_MEM_CORE_CLK_SEL_REG(0),
                SPI_MEM_CORE_CLK_SEL,
                DEFAULT_CORE_CLK_REG);

  set_flash_clock(0, flash_div);
  if (spi1)
    {
      set_flash_clock(1, flash_div);
    }

  set_psram_clock(0, psram_div);
}

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_low_speed
 *
 * Description:
 *   Make MSPI work under 20MHz and remove the timing tuning required delays.
 *
 * Input Parameters:
 *   spi1 - Select whether to control SPI1
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_spi_timing_set_mspi_low_speed(bool spi1)
{
  /**
   * Here we are going to set the SPI1 frequency to be 20MHz,
   * so we need to set SPI1 din_num and din_mode regs.
   *
   * Because SPI0 and SPI1 share the din_num and din_mode regs,
   * but if we clear SPI1 din_num and din_mode to 0 and SPI0 flash
   * module clock is still in high freq, it may not work correctly.
   *
   * Therefore, we need to set both the SPI0 and SPI1 and related
   * timing tuning regs to be 20MHz.
   */

  /* Set SPIMEM core clock as 80MHz, and set SPI1 and SPI0 clock
   * to be 20MHz by setting clock division as 4
   */

  REG_SET_FIELD(SPI_MEM_CORE_CLK_SEL_REG(0),
                SPI_MEM_CORE_CLK_SEL,
                CORE_CLK_REG_SEL_80M);

  set_flash_clock(0, 4);
  if (spi1)
    {
      /* After tuning, won't touch SPI1 again */

      set_flash_clock(1, 4);
    }
}
