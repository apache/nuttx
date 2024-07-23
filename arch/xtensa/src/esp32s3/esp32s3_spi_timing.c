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
#include <debug.h>
#include <sys/param.h>

#include "xtensa.h"
#include "esp32s3_gpio.h"
#include "esp32s3_psram.h"
#include "esp32s3_spi_timing.h"
#include "hardware/esp32s3_iomux.h"
#include "hardware/esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "rom/esp32s3_spiflash.h"
#include "rom/opi_flash.h"

#include "soc/spi_mem_reg.h"

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

#define MSPI_TIMING_LL_FLASH_OCT_MASK        (SPI_MEM_FCMD_OCT | SPI_MEM_FADDR_OCT | SPI_MEM_FDIN_OCT | SPI_MEM_FDOUT_OCT)
#define MSPI_TIMING_LL_FLASH_QUAD_MASK       (SPI_MEM_FASTRD_MODE | SPI_MEM_FREAD_DUAL | SPI_MEM_FREAD_DIO | SPI_MEM_FREAD_QUAD | SPI_MEM_FREAD_QIO)
#define MSPI_TIMING_LL_FLASH_QIO_MODE_MASK   (SPI_MEM_FREAD_QIO | SPI_MEM_FASTRD_MODE)
#define MSPI_TIMING_LL_FLASH_QUAD_MODE_MASK  (SPI_MEM_FREAD_QUAD | SPI_MEM_FASTRD_MODE)
#define MSPI_TIMING_LL_FLASH_DIO_MODE_MASK   (SPI_MEM_FREAD_DIO | SPI_MEM_FASTRD_MODE)
#define MSPI_TIMING_LL_FLASH_DUAL_MODE_MASK  (SPI_MEM_FREAD_DUAL | SPI_MEM_FASTRD_MODE)
#define MSPI_TIMING_LL_FLASH_FAST_MODE_MASK  (SPI_MEM_FASTRD_MODE)

#define MSPI_TIMING_PSRAM_DTR_MODE           CONFIG_ESP32S3_SPIRAM_MODE_OCT
#define MSPI_TIMING_PSRAM_STR_MODE           !CONFIG_ESP32S3_SPIRAM_MODE_OCT
#define MSPI_TIMING_FLASH_DTR_MODE           CONFIG_ESP32S3_FLASH_SAMPLE_MODE_DTR
#define MSPI_TIMING_FLASH_STR_MODE           CONFIG_ESP32S3_FLASH_SAMPLE_MODE_STR

#define MSPI_TIMING_TEST_DATA_LEN            1024
#define MSPI_TIMING_PSRAM_TEST_DATA_ADDR     0
#define MSPI_TIMING_FLASH_TEST_DATA_ADDR     0

#define OPI_PSRAM_SYNC_READ        0x0000
#define OPI_PSRAM_SYNC_WRITE       0x8080
#define OCT_PSRAM_RD_DUMMY_NUM     (2*(10-1))
#define OCT_PSRAM_WR_DUMMY_NUM     (2*(5-1))

#define QPI_PSRAM_FAST_READ        0XEB
#define QPI_PSRAM_WRITE            0X38
#define QPI_PSRAM_FAST_READ_DUMMY  6
#define NOT_INIT_INT               127

#define g_spiflash_dummy_len_plus  (rom_spiflash_legacy_data->dummy_len_plus)

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
 * Private Functions Prototypes
 ****************************************************************************/

static void IRAM_ATTR set_psram_clock(uint8_t spi_num, uint32_t freqdiv);
static void IRAM_ATTR set_flash_clock(uint8_t spi_num, uint32_t freqdiv);
#if ESP32S3_SPI_TIMING_FLASH_TUNING || ESP32S3_SPI_TIMING_PSRAM_TUNING
static void init_spi1_for_tuning(bool is_flash);
static uint32_t get_dummy(void);
static void set_flash_extra_dummy(uint8_t spi_num, uint8_t extra_dummy);
static void set_psram_extra_dummy(uint8_t spi_num, uint8_t extra_dummy);
static void set_flash_din_mode_num(uint8_t spi_num, uint8_t din_mode,
                                   uint8_t din_num);
static void set_psram_din_mode_num(uint8_t spi_num, uint8_t din_mode,
                                   uint8_t din_num);
#if ESP32S3_SPI_TIMING_FLASH_TUNING
static void config_flash_read_data(uint8_t *buf, uint32_t addr,
                                   uint32_t len);
static void config_flash_set_tuning_regs(const tuning_param_s *params);
static void get_flash_tuning_configs(tuning_config_s *config);
#endif
#if ESP32S3_SPI_TIMING_PSRAM_TUNING
static void psram_read_data(uint8_t *buf, uint32_t addr, uint32_t len);
static void psram_write_data(uint8_t *buf, uint32_t addr, uint32_t len);
static void config_psram_read_write_data(uint8_t *buf, uint32_t addr,
                                         uint32_t len, bool is_read);
static void get_psram_tuning_configs(tuning_config_s *config);
static void config_psram_set_tuning_regs(const tuning_param_s *params);
#endif
static void sweep_for_success_sample_points(const uint8_t *reference_data,
        const tuning_config_s *config, bool is_flash, uint8_t *out_array);
static void find_max_consecutive_success_points(uint8_t *array,
            uint32_t size, uint32_t *out_length, uint32_t *out_end_index);
#if MSPI_TIMING_FLASH_STR_MODE || MSPI_TIMING_PSRAM_STR_MODE
static uint32_t select_best_tuning_config_str(tuning_config_s *config,
                          uint32_t consecutive_length, uint32_t end);
#endif
#if MSPI_TIMING_FLASH_DTR_MODE || MSPI_TIMING_PSRAM_DTR_MODE
static uint32_t select_best_tuning_config_dtr(tuning_config_s *config,
                           uint32_t consecutive_length, uint32_t end);
#endif
static void select_best_tuning_config(tuning_config_s *config,
                    uint32_t consecutive_length, uint32_t end,
                    const uint8_t *reference_data, bool is_flash);
static void set_timing_tuning_regs(bool control_spi1);
static void clear_timing_tuning_regs(bool control_spi1);
static void do_tuning(const uint8_t *reference_data,
                tuning_config_s *timing_config, bool is_flash);
#endif

/****************************************************************************
 * Extern Functions declaration
 ****************************************************************************/

#if CONFIG_ESP32S3_SPIRAM_MODE_QUAD
extern void psram_exec_cmd(int spi_num, int mode,
  uint32_t cmd, int cmd_bit_len,
  uint32_t addr, int addr_bit_len,
  int dummy_bits, uint8_t *mosi_data,
  int mosi_bit_len, uint8_t *miso_data,
  int miso_bit_len, uint32_t cs_mask,
  bool is_write_erase_operation);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if ESP32S3_SPI_TIMING_FLASH_TUNING || ESP32S3_SPI_TIMING_PSRAM_TUNING
#if CONFIG_ESP32S3_SPIRAM_MODE_QUAD
static uint8_t g_psram_extra_dummy;
#endif
static uint8_t g_flash_extra_dummy[2] =
{
  NOT_INIT_INT,
  NOT_INIT_INT
};

static tuning_param_s g_flash_timing_tuning_config;
static tuning_param_s g_psram_timing_tuning_config;
#endif

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

static void IRAM_ATTR set_psram_clock(uint8_t spi_num, uint32_t freqdiv)
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

#if ESP32S3_SPI_TIMING_FLASH_TUNING || ESP32S3_SPI_TIMING_PSRAM_TUNING

/****************************************************************************
 * Name: init_spi1_for_tuning
 *
 * Description:
 *   Initialize SPI1 for timing tuning.
 *
 * Input Parameters:
 *   is_flash - is flash or not
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void init_spi1_for_tuning(bool is_flash)
{
  /* Set SPI1 core clock. SPI0 and SPI1 share the register for core clock.
   * So we only set SPI0 here.
   */

  REG_SET_FIELD(SPI_MEM_CORE_CLK_SEL_REG(0),
                SPI_MEM_CORE_CLK_SEL,
                DEFAULT_CORE_CLK_REG);

  /* Set SPI1 module clock as required */

  if (is_flash)
    {
      set_flash_clock(1, FLASH_CLOCK_DIVIDER);

      /* Enable Flash HCLK */

      REG_SET_BIT(SPI_MEM_TIMING_CALI_REG(0), SPI_MEM_TIMING_CLK_ENA);
    }
  else
    {
      /* We use SPI1 Flash to tune PSRAM, PSRAM timing related regs
       * do nothing on SPI1
       */

      set_flash_clock(1, PSRAM_CLOCK_DIVIDER);

      /* Enable PSRAM HCLK */

      REG_SET_BIT(SPI_MEM_SPI_SMEM_TIMING_CALI_REG(0),
                  SPI_MEM_SPI_SMEM_TIMING_CLK_ENA);
    }
}

/****************************************************************************
 * Name: get_dummy
 *
 * Description:
 *   Get dummy cycle length
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Dummy cycle length.
 *
 ****************************************************************************/

static uint32_t get_dummy(void)
{
  uint32_t ctrl_reg = READ_PERI_REG(SPI_MEM_CTRL_REG(0));
  if (ctrl_reg & MSPI_TIMING_LL_FLASH_OCT_MASK)
    {
      DEBUGPANIC();
      return 0;
    }

  switch (ctrl_reg & MSPI_TIMING_LL_FLASH_QUAD_MASK)
    {
      case MSPI_TIMING_LL_FLASH_QIO_MODE_MASK:
        return SPI1_R_QIO_DUMMY_CYCLELEN;
      case MSPI_TIMING_LL_FLASH_DIO_MODE_MASK:
        return SPI1_R_DIO_DUMMY_CYCLELEN;
      case MSPI_TIMING_LL_FLASH_QUAD_MODE_MASK:
      case MSPI_TIMING_LL_FLASH_DUAL_MODE_MASK:
      case MSPI_TIMING_LL_FLASH_FAST_MODE_MASK:
        return SPI1_R_FAST_DUMMY_CYCLELEN;
      default:
        DEBUGPANIC();
        return 0;
    }
}

/****************************************************************************
 * Name: set_flash_extra_dummy
 *
 * Description:
 *   Set MSPI Flash extra dummy
 *
 * Input Parameters:
 *   spi_num     - SPI0 / 1
 *   extra_dummy - extra dummy
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_flash_extra_dummy(uint8_t spi_num, uint8_t extra_dummy)
{
#ifdef CONFIG_ESP32S3_FLASH_MODE_OCT
  if (extra_dummy > 0)
    {
      SET_PERI_REG_MASK(SPI_MEM_TIMING_CALI_REG(spi_num),
                        SPI_MEM_TIMING_CALI_M);
      SET_PERI_REG_BITS(SPI_MEM_TIMING_CALI_REG(spi_num),
                        SPI_MEM_EXTRA_DUMMY_CYCLELEN_V,
                        extra_dummy,
                        SPI_MEM_EXTRA_DUMMY_CYCLELEN_S);
    }
  else
    {
      CLEAR_PERI_REG_MASK(SPI_MEM_TIMING_CALI_REG(spi_num),
                          SPI_MEM_TIMING_CALI_M);
      SET_PERI_REG_BITS(SPI_MEM_TIMING_CALI_REG(spi_num),
                        SPI_MEM_EXTRA_DUMMY_CYCLELEN_V, 0,
                        SPI_MEM_EXTRA_DUMMY_CYCLELEN_S);
    }

  return;
#endif
  if (g_flash_extra_dummy[spi_num] == NOT_INIT_INT)
    {
      g_flash_extra_dummy[spi_num] = g_spiflash_dummy_len_plus[spi_num];
    }

  g_spiflash_dummy_len_plus[spi_num] = g_flash_extra_dummy[(int)spi_num] +
                                       extra_dummy;

  /* Only Quad Flash will run into this branch. */

  uint32_t dummy = get_dummy();

  /* Set MSPI Quad Flash dummy */

  SET_PERI_REG_MASK(SPI_MEM_USER_REG(spi_num), SPI_MEM_USR_DUMMY);
  SET_PERI_REG_BITS(SPI_MEM_USER1_REG(spi_num), SPI_MEM_USR_DUMMY_CYCLELEN_V,
                                  dummy + g_spiflash_dummy_len_plus[spi_num],
                                  SPI_MEM_USR_DUMMY_CYCLELEN_S);
}

/****************************************************************************
 * Name: set_psram_extra_dummy
 *
 * Description:
 *   Set MSPI PSRAM extra dummy
 *
 * Input Parameters:
 *   spi_num     - SPI0 / 1
 *   extra_dummy - extra dummy
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_psram_extra_dummy(uint8_t spi_num, uint8_t extra_dummy)
{
#if CONFIG_ESP32S3_SPIRAM_MODE_OCT

  /* Set MSPI Octal PSRAM extra dummy */

  if (extra_dummy > 0)
    {
      SET_PERI_REG_MASK(SPI_MEM_SPI_SMEM_TIMING_CALI_REG(spi_num),
                        SPI_MEM_SPI_SMEM_TIMING_CALI_M);
      SET_PERI_REG_BITS(SPI_MEM_SPI_SMEM_TIMING_CALI_REG(spi_num),
                        SPI_MEM_SPI_SMEM_EXTRA_DUMMY_CYCLELEN_V,
                        extra_dummy,
                        SPI_MEM_SPI_SMEM_EXTRA_DUMMY_CYCLELEN_S);
    }
  else
    {
      CLEAR_PERI_REG_MASK(SPI_MEM_SPI_SMEM_TIMING_CALI_REG(spi_num),
                          SPI_MEM_SPI_SMEM_TIMING_CALI_M);
      SET_PERI_REG_BITS(SPI_MEM_SPI_SMEM_TIMING_CALI_REG(spi_num),
                        SPI_MEM_SPI_SMEM_EXTRA_DUMMY_CYCLELEN_V, 0,
                        SPI_MEM_SPI_SMEM_EXTRA_DUMMY_CYCLELEN_S);
    }

#elif CONFIG_ESP32S3_SPIRAM_MODE_QUAD

  /* Set MSPI QUAD PSRAM dummy.
   * HW workaround: Use normal dummy register to set extra dummy, the
   * calibration dedicated extra dummy register doesn't work for quad mode
   */

  SET_PERI_REG_MASK(SPI_MEM_CACHE_SCTRL_REG(spi_num),
                    SPI_MEM_USR_RD_SRAM_DUMMY_M);
  SET_PERI_REG_BITS(SPI_MEM_CACHE_SCTRL_REG(spi_num),
                    SPI_MEM_SRAM_RDUMMY_CYCLELEN_V,
                   (QPI_PSRAM_FAST_READ_DUMMY + extra_dummy - 1),
                    SPI_MEM_SRAM_RDUMMY_CYCLELEN_S);
#endif
}

/****************************************************************************
 * Name: set_flash_din_mode_num
 *
 * Description:
 *   Set MSPI Flash Din Mode and Din Num
 *
 * Input Parameters:
 *   spi_num  - SPI0 / 1
 *   din_mode - Din mode
 *   din_num  - Din num
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_flash_din_mode_num(uint8_t spi_num, uint8_t din_mode,
                                   uint8_t din_num)
{
  /* Set MSPI Flash din mode */

  uint32_t reg_val = (getreg32(SPI_MEM_DIN_MODE_REG(spi_num)) &
      (~(SPI_MEM_DIN0_MODE_M | SPI_MEM_DIN1_MODE_M | SPI_MEM_DIN2_MODE_M |
         SPI_MEM_DIN3_MODE_M | SPI_MEM_DIN4_MODE_M | SPI_MEM_DIN5_MODE_M |
         SPI_MEM_DIN6_MODE_M | SPI_MEM_DIN7_MODE_M | SPI_MEM_DINS_MODE_M))) |
      (din_mode << SPI_MEM_DIN0_MODE_S) | (din_mode << SPI_MEM_DIN1_MODE_S) |
      (din_mode << SPI_MEM_DIN2_MODE_S) | (din_mode << SPI_MEM_DIN3_MODE_S) |
      (din_mode << SPI_MEM_DIN4_MODE_S) | (din_mode << SPI_MEM_DIN5_MODE_S) |
      (din_mode << SPI_MEM_DIN6_MODE_S) | (din_mode << SPI_MEM_DIN7_MODE_S) |
      (din_mode << SPI_MEM_DINS_MODE_S);
  putreg32(reg_val, SPI_MEM_DIN_MODE_REG(spi_num));

  /* Set MSPI Flash din num */

  reg_val = (getreg32(SPI_MEM_DIN_NUM_REG(spi_num)) &
      (~(SPI_MEM_DIN0_NUM_M | SPI_MEM_DIN1_NUM_M | SPI_MEM_DIN2_NUM_M |
         SPI_MEM_DIN3_NUM_M | SPI_MEM_DIN4_NUM_M | SPI_MEM_DIN5_NUM_M |
         SPI_MEM_DIN6_NUM_M | SPI_MEM_DIN7_NUM_M | SPI_MEM_DINS_NUM_M))) |
      (din_num << SPI_MEM_DIN0_NUM_S) | (din_num << SPI_MEM_DIN1_NUM_S) |
      (din_num << SPI_MEM_DIN2_NUM_S) | (din_num << SPI_MEM_DIN3_NUM_S) |
      (din_num << SPI_MEM_DIN4_NUM_S) | (din_num << SPI_MEM_DIN5_NUM_S) |
      (din_num << SPI_MEM_DIN6_NUM_S) | (din_num << SPI_MEM_DIN7_NUM_S) |
      (din_num << SPI_MEM_DINS_NUM_S);
  putreg32(reg_val, SPI_MEM_DIN_NUM_REG(spi_num));
}

/****************************************************************************
 * Name: set_psram_din_mode_num
 *
 * Description:
 *   Set MSPI PSRAM Din Mode and Din Num
 *
 * Input Parameters:
 *   spi_num  - SPI0 / 1
 *   din_mode - Din mode
 *   din_num  - Din num
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_psram_din_mode_num(uint8_t spi_num, uint8_t din_mode,
                                   uint8_t din_num)
{
  /* Set MSPI PSRAM din mode */

  uint32_t reg_val = (getreg32(SPI_MEM_SPI_SMEM_DIN_MODE_REG(spi_num)) &
      (~(SPI_MEM_SPI_SMEM_DIN0_MODE_M | SPI_MEM_SPI_SMEM_DIN1_MODE_M |
         SPI_MEM_SPI_SMEM_DIN2_MODE_M | SPI_MEM_SPI_SMEM_DIN3_MODE_M |
         SPI_MEM_SPI_SMEM_DIN4_MODE_M | SPI_MEM_SPI_SMEM_DIN5_MODE_M |
         SPI_MEM_SPI_SMEM_DIN6_MODE_M | SPI_MEM_SPI_SMEM_DIN7_MODE_M |
         SPI_MEM_SPI_SMEM_DINS_MODE_M))) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN0_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN1_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN2_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN3_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN4_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN5_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN6_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DIN7_MODE_S) |
         (din_mode << SPI_MEM_SPI_SMEM_DINS_MODE_S);
  putreg32(reg_val, SPI_MEM_SPI_SMEM_DIN_MODE_REG(spi_num));

  /* Set MSPI PSRAM din num */

  reg_val = (getreg32(SPI_MEM_SPI_SMEM_DIN_NUM_REG(spi_num)) &
      (~(SPI_MEM_SPI_SMEM_DIN0_NUM_M | SPI_MEM_SPI_SMEM_DIN1_NUM_M |
         SPI_MEM_SPI_SMEM_DIN2_NUM_M | SPI_MEM_SPI_SMEM_DIN3_NUM_M |
         SPI_MEM_SPI_SMEM_DIN4_NUM_M | SPI_MEM_SPI_SMEM_DIN5_NUM_M |
         SPI_MEM_SPI_SMEM_DIN6_NUM_M | SPI_MEM_SPI_SMEM_DIN7_NUM_M |
         SPI_MEM_SPI_SMEM_DINS_NUM_M))) |
         (din_num << SPI_MEM_SPI_SMEM_DIN0_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DIN1_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DIN2_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DIN3_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DIN4_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DIN5_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DIN6_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DIN7_NUM_S) |
         (din_num << SPI_MEM_SPI_SMEM_DINS_NUM_S);
  putreg32(reg_val, SPI_MEM_SPI_SMEM_DIN_NUM_REG(spi_num));
}

#if ESP32S3_SPI_TIMING_FLASH_TUNING

/****************************************************************************
 * Name: config_flash_read_data
 *
 * Description:
 *   Configure Flash to read data via SPI1h
 *
 * Input Parameters:
 *   buf  - data buffer pointer
 *   addr - target address value
 *   len  - data length
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void config_flash_read_data(uint8_t *buf, uint32_t addr, uint32_t len)
{
#ifdef CONFIG_ESP32S3_FLASH_MODE_OCT
  /* Clear MSPI hw fifo */

  for (int i = 0; i < 16; i++)
    {
      putreg32(0, SPI_MEM_W0_REG(1) + i * 4);
    }

  esp_rom_opiflash_read_raw(addr, buf, len);
#else
  esp_rom_spiflash_read(addr, (uint32_t *)buf, len);
#endif
}

/****************************************************************************
 * Name: config_flash_set_tuning_regs
 *
 * Description:
 *   Tune Flash timing registers for SPI1 accessing Flash
 *
 * Input Parameters:
 *   params - tuning timing parameters pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void config_flash_set_tuning_regs(const tuning_param_s *params)
{
  /* SPI_MEM_DINx_MODE(1), SPI_MEM_DINx_NUM(1) are meaningless SPI0 and SPI1
   * share SPI_MEM_DINx_MODE(0), SPI_MEM_DINx_NUM(0) for FLASH timing tuning
   * We use SPI1 to get the best Flash timing tuning (mode and num) config
   */

  set_flash_din_mode_num(0, params->spi_din_mode, params->spi_din_num);
  set_flash_extra_dummy(1, params->extra_dummy_len);
}

/****************************************************************************
 * Name: get_flash_tuning_configs
 *
 * Description:
 *   Get FLASH tuning configuration.
 *
 * Input Parameters:
 *   config - tuning timing configuration pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void get_flash_tuning_configs(tuning_config_s *config)
{
#if MSPI_TIMING_FLASH_DTR_MODE
#  define FLASH_MODE  DTR_MODE
#else
#  define FLASH_MODE  STR_MODE
#endif

#if CONFIG_ESP32S3_FLASH_FREQ_20M
    *config = MSPI_TIMING_FLASH_GET_TUNING_CONFIG(
                ESP32S3_SPI_TIMING_CORE_CLK, 20, FLASH_MODE);
#elif CONFIG_ESP32S3_FLASH_FREQ_40M
    *config = MSPI_TIMING_FLASH_GET_TUNING_CONFIG(
                ESP32S3_SPI_TIMING_CORE_CLK, 40, FLASH_MODE);
#elif CONFIG_ESP32S3_FLASH_FREQ_80M
    *config = MSPI_TIMING_FLASH_GET_TUNING_CONFIG(
                ESP32S3_SPI_TIMING_CORE_CLK, 80, FLASH_MODE);
#elif CONFIG_ESP32S3_FLASH_FREQ_120M
    *config = MSPI_TIMING_FLASH_GET_TUNING_CONFIG(
                ESP32S3_SPI_TIMING_CORE_CLK, 120, FLASH_MODE);
#endif

#undef FLASH_MODE
}
#endif

#if ESP32S3_SPI_TIMING_PSRAM_TUNING

/****************************************************************************
 * Name: psram_read_data
 *
 * Description:
 *   Configure PSRAM to read data via SPI1.
 *
 * Input Parameters:
 *   buf  - data buffer pointer
 *   addr - target address value
 *   len  - data length
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void psram_read_data(uint8_t *buf, uint32_t addr, uint32_t len)
{
#if CONFIG_ESP32S3_SPIRAM_MODE_OCT

  /* Clear MSPI hw fifo */

  for (int i = 0; i < 16; i++)
    {
      putreg32(0, SPI_MEM_W0_REG(1) + i * 4);
    }

  esp_rom_opiflash_exec_cmd(1, ESP_ROM_SPIFLASH_OPI_DTR_MODE,
                            OPI_PSRAM_SYNC_READ, 16,
                            addr, 32,
                            OCT_PSRAM_RD_DUMMY_NUM,
                            NULL, 0,
                            buf, len * 8,
                            BIT(1),
                            false);
#elif CONFIG_ESP32S3_SPIRAM_MODE_QUAD
  psram_exec_cmd(1, 0,
                 QPI_PSRAM_FAST_READ, 8,
                 addr, 24,
                 QPI_PSRAM_FAST_READ_DUMMY + g_psram_extra_dummy,
                 NULL, 0,
                 buf, len * 8,
                 SPI_MEM_CS1_DIS_M,
                 false);
#endif
}

/****************************************************************************
 * Name: psram_write_data
 *
 * Description:
 *   Configure PSRAM to write data via SPI1.
 *
 * Input Parameters:
 *   buf  - data buffer pointer
 *   addr - target address value
 *   len  - data length
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void psram_write_data(uint8_t *buf, uint32_t addr, uint32_t len)
{
#if CONFIG_ESP32S3_SPIRAM_MODE_OCT
  esp_rom_opiflash_exec_cmd(1, ESP_ROM_SPIFLASH_OPI_DTR_MODE,
                            OPI_PSRAM_SYNC_WRITE, 16,
                            addr, 32,
                            OCT_PSRAM_WR_DUMMY_NUM,
                            buf, len * 8,
                            NULL, 0,
                            BIT(1),
                            false);
#elif CONFIG_ESP32S3_SPIRAM_MODE_QUAD
  psram_exec_cmd(1, 0,
                 QPI_PSRAM_WRITE, 8,  /* command and command bit len */
                 addr, 24,            /* address and address bit len */
                 0,                   /* dummy bit len */
                 buf, len * 8,        /* tx data and tx bit len */
                 NULL, 0,             /* rx data and rx bit len */
                 SPI_MEM_CS1_DIS_M,   /* cs bit mask */
                 false);              /* whether is program/erase operation */
#endif
}

/****************************************************************************
 * Name: config_psram_read_write_data
 *
 * Description:
 *   Configure PSRAM to read or write data via SPI1.
 *
 * Input Parameters:
 *   buf     - data buffer pointer
 *   addr    - target address value
 *   len     - data length
 *   is_read - is read or write
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void config_psram_read_write_data(uint8_t *buf, uint32_t addr,
                                         uint32_t len, bool is_read)
{
  while (len > 0)
    {
      uint32_t length = MIN(len, 32);
      if (is_read)
        {
          psram_read_data(buf, addr, length);
        }
      else
        {
          psram_write_data(buf, addr, length);
        }

      addr += length;
      buf += length;
      len -= length;
    }
}

/****************************************************************************
 * Name: get_psram_tuning_configs
 *
 * Description:
 *   Get PSRAM tuning configuration.
 *
 * Input Parameters:
 *   config - tuning timing configuration pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void get_psram_tuning_configs(tuning_config_s *config)
{
#if MSPI_TIMING_PSRAM_DTR_MODE
#  define PSRAM_MODE  DTR_MODE
#else
#  define PSRAM_MODE  STR_MODE
#endif

#if CONFIG_ESP32S3_SPIRAM_SPEED_40M
    *config = MSPI_TIMING_PSRAM_GET_TUNING_CONFIG(
                ESP32S3_SPI_TIMING_CORE_CLK, 40, PSRAM_MODE);
#elif CONFIG_ESP32S3_SPIRAM_SPEED_80M
    *config = MSPI_TIMING_PSRAM_GET_TUNING_CONFIG(
                ESP32S3_SPI_TIMING_CORE_CLK, 80, PSRAM_MODE);
#elif CONFIG_ESP32S3_SPIRAM_SPEED_120M
    *config = MSPI_TIMING_PSRAM_GET_TUNING_CONFIG(
                ESP32S3_SPI_TIMING_CORE_CLK, 120, PSRAM_MODE);
#endif

#undef PSRAM_MODE
}

/****************************************************************************
 * Name: config_psram_set_tuning_regs
 *
 * Description:
 *   Tune PSRAM timing registers for SPI1 accessing PSRAM
 *
 * Input Parameters:
 *   params - tuning timing parameters pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void config_psram_set_tuning_regs(const tuning_param_s *params)
{
  /* 1. SPI_MEM_SPI_SMEM_DINx_MODE(1), SPI_MEM_SPI_SMEM_DINx_NUM(1) are
   * meaningless SPI0 and SPI1 share the SPI_MEM_SPI_SMEM_DINx_MODE(0),
   * SPI_MEM_SPI_SMEM_DINx_NUM(0) for PSRAM timing tuning
   * 2. We use SPI1 to get the best PSRAM timing tuning (mode and num) config
   */

  set_psram_din_mode_num(0, params->spi_din_mode, params->spi_din_num);
#if CONFIG_ESP32S3_SPIRAM_MODE_OCT

  /* On 728, for SPI1, flash and psram share the extra dummy register */

  set_flash_extra_dummy(1, params->extra_dummy_len);
#elif CONFIG_ESP32S3_SPIRAM_MODE_QUAD

  /* Update this `g_psram_extra_dummy`, the `psram_read_data` will
   * set dummy according to this `g_psram_extra_dummy`
   */

  g_psram_extra_dummy = params->extra_dummy_len;

  /* Set MSPI Quad Flash dummy */

  SET_PERI_REG_MASK(SPI_MEM_USER_REG(1), SPI_MEM_USR_DUMMY);
  SET_PERI_REG_BITS(SPI_MEM_USER1_REG(1), SPI_MEM_USR_DUMMY_CYCLELEN_V,
                    g_psram_extra_dummy - 1, SPI_MEM_USR_DUMMY_CYCLELEN_S);
#endif
}
#endif

/****************************************************************************
 * Name: sweep_for_success_sample_points
 *
 * Description:
 *   Use different SPI1 timing tuning config to read data to see if current
 *   MSPI sampling is successful.The sampling result will be stored in an
 *   array. In this array, successful item will be 1, failed item will be 0.
 *
 * Input Parameters:
 *   reference_data - reference data pointer
 *   config         - tuning timing configuration pointer
 *   is_flash       - is flash or not
 *   out_array      - last success point pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sweep_for_success_sample_points(const uint8_t *reference_data,
            const tuning_config_s *config, bool is_flash, uint8_t *out_array)
{
  uint32_t config_idx = 0;
  uint8_t read_data[MSPI_TIMING_TEST_DATA_LEN];

  for (config_idx = 0; config_idx < config->config_num; config_idx++)
    {
      memset(read_data, 0, MSPI_TIMING_TEST_DATA_LEN);
#if ESP32S3_SPI_TIMING_FLASH_TUNING
      if (is_flash)
        {
          config_flash_set_tuning_regs(&(config->config_table[config_idx]));
          config_flash_read_data(read_data, MSPI_TIMING_FLASH_TEST_DATA_ADDR,
                                 sizeof(read_data));
        }
#endif

#if ESP32S3_SPI_TIMING_PSRAM_TUNING
      if (!is_flash)
        {
          config_psram_set_tuning_regs(&(config->config_table[config_idx]));
          config_psram_read_write_data(read_data,
                                       MSPI_TIMING_PSRAM_TEST_DATA_ADDR,
                                       MSPI_TIMING_TEST_DATA_LEN, true);
        }

#endif
      if (memcmp(reference_data, read_data, sizeof(read_data)) == 0)
        {
          out_array[config_idx] = 1;
          minfo("%d, good\n", config_idx);
        }
      else
        {
          minfo("%d, bad\n", config_idx);
        }
    }
}

/****************************************************************************
 * Name: find_max_consecutive_success_points
 *
 * Description:
 *   Find max consecutive success points.
 *
 * Input Parameters:
 *   array         - sampling results pointer
 *   size          - sampling results length
 *   out_length    - consecutive length pointer
 *   out_end_index - last success point pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void find_max_consecutive_success_points(uint8_t *array,
                uint32_t size, uint32_t *out_length, uint32_t *out_end_index)
{
  uint32_t max = 0;
  uint32_t match_num = 0;
  uint32_t i = 0;
  uint32_t end = 0;
  while (i < size)
    {
      if (array[i])
        {
          match_num++;
        }
      else
        {
          if (match_num > max)
            {
              max = match_num;
              end = i - 1;
            }

          match_num = 0;
        }

      i++;
    }

  *out_length = match_num > max ? match_num : max;
  *out_end_index = match_num == size ? size : end;
}

/****************************************************************************
 * Name: select_best_tuning_config_str
 *
 * Description:
 *   Select the STR best point
 *
 * Input Parameters:
 *   config             - tuning timing configuration pointer
 *   consecutive_length - consecutive length
 *   end                - last success point
 *
 * Returned Value:
 *   Returns the STR best point.
 *
 ****************************************************************************/

#if MSPI_TIMING_FLASH_STR_MODE || MSPI_TIMING_PSRAM_STR_MODE
static uint32_t select_best_tuning_config_str(tuning_config_s *config,
                          uint32_t consecutive_length, uint32_t end)
{
#if (ESP32S3_SPI_TIMING_CORE_CLK == 120 || ESP32S3_SPI_TIMING_CORE_CLK == 240)
  minfo("DO NOT USE FOR MASS PRODUCTION! Timing parameters may be updated");

  /* STR best point scheme */

  uint32_t best_point;
  if (consecutive_length <= 2 || consecutive_length >= 5)
    {
      /* tuning is FAIL, select default point, and generate a warning */

      best_point = config->default_config_id;
      minfo("tuning fail, best point is fallen back to index %d\n",
            best_point);
    }
  else
    {
      /* consecutive length :  3 or 4 */

      best_point = end - consecutive_length / 2;
      minfo("tuning success, best point is index %d\n", best_point);
    }

  return best_point;
#else
  /* won't reach here */

  abort();
#endif
}
#endif

/****************************************************************************
 * Name: select_best_tuning_config_dtr
 *
 * Description:
 *   Select the DTR best point
 *
 * Input Parameters:
 *   config             - tuning timing configuration pointer
 *   consecutive_length - consecutive length
 *   end                - last success point
 *
 * Returned Value:
 *   Returns the DTR best point.
 *
 ****************************************************************************/

#if MSPI_TIMING_FLASH_DTR_MODE || MSPI_TIMING_PSRAM_DTR_MODE
static uint32_t select_best_tuning_config_dtr(tuning_config_s *config,
                           uint32_t consecutive_length, uint32_t end)
{
#if (ESP32S3_SPI_TIMING_CORE_CLK == 160)

  /* Core clock 160M DTR best point scheme */

  uint32_t best_point;

  /* These numbers will probably be same on other chips,
   * if this version of algorithm is utilised
   */

  if (consecutive_length <= 2 || consecutive_length >= 6)
    {
      /* tuning is FAIL, select default point, and generate a warning */

      best_point = config->default_config_id;
      minfo("tuning fail, best point is fallen back to index %d\n",
            best_point);
    }
  else if (consecutive_length <= 4)
    {
      /* consecutive length :  3 or 4 */

      best_point = end - 1;
      minfo("tuning success, best point is index %d\n", best_point);
    }
  else
    {
      /* consecutive point list length equals 5 */

      best_point = end - 2;
      minfo("tuning success, best point is index %d\n", best_point);
    }

  return best_point;
#else
  /* won't reach here */

  abort();
#endif
}
#endif

/****************************************************************************
 * Name: select_best_tuning_config
 *
 * Description:
 *   Select the best timing tuning configuration
 *
 * Input Parameters:
 *   config             - tuning timing configuration pointer
 *   consecutive_length - consecutive length
 *   end                - last success point
 *   reference_data     - reference data pointer
 *   is_flash           - is flash or not
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void select_best_tuning_config(tuning_config_s *config,
                    uint32_t consecutive_length, uint32_t end,
                    const uint8_t *reference_data, bool is_flash)
{
  uint32_t best_point = 0;
  if (is_flash)
    {
#if MSPI_TIMING_FLASH_DTR_MODE
      best_point = select_best_tuning_config_dtr(config, consecutive_length,
                                                 end);
#elif MSPI_TIMING_FLASH_STR_MODE
      best_point = select_best_tuning_config_str(config, consecutive_length,
                                                 end);
#endif
      g_flash_timing_tuning_config = config->config_table[best_point];
      minfo("Flash timing tuning index: %d\n", best_point);
    }
  else
    {
#if MSPI_TIMING_PSRAM_DTR_MODE
      best_point = select_best_tuning_config_dtr(config, consecutive_length,
                                                 end);
#elif MSPI_TIMING_PSRAM_STR_MODE
      best_point = select_best_tuning_config_str(config, consecutive_length,
                                                 end);
#endif
      g_psram_timing_tuning_config = config->config_table[best_point];
      minfo("PSRAM timing tuning index: %d\n", best_point);
    }
}

/****************************************************************************
 * Name: set_timing_tuning_regs
 *
 * Description:
 *   Set timing tuning registers
 *
 * Input Parameters:
 *   spi1 - Select whether to control SPI1
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_timing_tuning_regs(bool control_spi1)
{
  /* SPI0 and SPI1 share the registers for flash din mode and num setting,
   * so we only set SPI0's reg
   */

  set_flash_din_mode_num(0, g_flash_timing_tuning_config.spi_din_mode,
                          g_flash_timing_tuning_config.spi_din_num);
  set_flash_extra_dummy(0, g_flash_timing_tuning_config.extra_dummy_len);
  if (control_spi1)
    {
      set_flash_extra_dummy(1, g_flash_timing_tuning_config.extra_dummy_len);
    }

  set_psram_din_mode_num(0, g_psram_timing_tuning_config.spi_din_mode,
                         g_psram_timing_tuning_config.spi_din_num);
  set_psram_extra_dummy(0, g_psram_timing_tuning_config.extra_dummy_len);
}

/****************************************************************************
 * Name: clear_timing_tuning_regs
 *
 * Description:
 *   Clear timing tuning registers
 *
 * Input Parameters:
 *   spi1 - Select whether to control SPI1
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void clear_timing_tuning_regs(bool control_spi1)
{
  /* SPI0 and SPI1 share the registers for flash din mode and num setting,
   * so we only set SPI0's reg
   */

  set_flash_din_mode_num(0, 0, 0);
  set_flash_extra_dummy(0, 0);
  if (control_spi1)
    {
      set_flash_extra_dummy(1, 0);
    }

  set_psram_din_mode_num(0, 0, 0);
  set_psram_extra_dummy(0, 0);
}

/****************************************************************************
 * Name: do_tuning
 *
 * Description:
 *   Start to tune the timing:
 *
 * Input Parameters:
 *   reference_data - reference data pointer
 *   timing_config  - tuning timing configuration pointer
 *   is_flash       - is flash or not
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void do_tuning(const uint8_t *reference_data,
                tuning_config_s *timing_config, bool is_flash)
{
  /* We use SPI1 to tune the timing:
   * 1. Get all SPI1 sampling results.
   * 2. Find the longest consecutive successful sampling points.
   * 3. The middle one will be the best sampling point.
   */

  uint32_t consecutive_length = 0;
  uint32_t last_success_point = 0;
  uint8_t sample_result[MSPI_TIMING_CONFIG_NUM_DEFAULT] =
    {
      0
    };

  init_spi1_for_tuning(is_flash);
  sweep_for_success_sample_points(reference_data, timing_config, is_flash,
                                  sample_result);
  find_max_consecutive_success_points(sample_result,
  MSPI_TIMING_CONFIG_NUM_DEFAULT, &consecutive_length, &last_success_point);
  select_best_tuning_config(timing_config, consecutive_length,
                            last_success_point, reference_data, is_flash);
}

#endif

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

void IRAM_ATTR esp32s3_spi_timing_set_pin_drive_strength(void)
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

#if ESP32S3_SPI_TIMING_FLASH_TUNING || ESP32S3_SPI_TIMING_PSRAM_TUNING
  set_timing_tuning_regs(true);
#endif
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
  /* Here we are going to set the SPI1 frequency to be 20MHz,
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

  set_psram_clock(0, 4);

#if ESP32S3_SPI_TIMING_FLASH_TUNING || ESP32S3_SPI_TIMING_PSRAM_TUNING
  clear_timing_tuning_regs(spi1);
#endif
}

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_psram_tuning
 *
 * Description:
 *   Tune MSPI psram timing to make it work under high frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_spi_timing_set_mspi_psram_tuning(void)
{
#if ESP32S3_SPI_TIMING_PSRAM_TUNING
  tuning_config_s timing_configs =
    {
      0
    };

  /* set SPI01 related regs to 20mhz configuration, to write reference data
   * to PSRAM.
   */

  esp32s3_spi_timing_set_mspi_low_speed(true);

  /* write data into psram, used to do timing tuning test. */

  uint8_t reference_data[MSPI_TIMING_TEST_DATA_LEN];
  for (int i = 0; i < MSPI_TIMING_TEST_DATA_LEN / 4; i++)
    {
      ((uint32_t *)reference_data)[i] = 0xa5ff005a;
    }

  config_psram_read_write_data(reference_data,
  MSPI_TIMING_PSRAM_TEST_DATA_ADDR, MSPI_TIMING_TEST_DATA_LEN, false);

  get_psram_tuning_configs(&timing_configs);

  /* Disable the variable dummy mode when doing timing tuning */

  REG_SET_FIELD(SPI_MEM_DDR_REG(1), SPI_MEM_SPI_FMEM_VAR_DUMMY, false);

  /* Get required config, and set them to PSRAM related registers */

  do_tuning(reference_data, &timing_configs, false);
  esp32s3_spi_timing_set_mspi_high_speed(true);
#endif
}

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_flash_tuning
 *
 * Description:
 *   Tune MSPI flash timing to make it work under high frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_spi_timing_set_mspi_flash_tuning(void)
{
#if ESP32S3_SPI_TIMING_FLASH_TUNING
  tuning_config_s timing_configs =
    {
      0
    };

  /* set SPI01 related regs to 20mhz configuration, to get reference data
   * from FLASH.
   */

  esp32s3_spi_timing_set_mspi_low_speed(true);

  /* Disable the variable dummy mode when doing timing tuning. */

  REG_SET_FIELD(SPI_MEM_DDR_REG(1), SPI_MEM_SPI_FMEM_VAR_DUMMY, false);
  uint8_t reference_data[MSPI_TIMING_TEST_DATA_LEN] =
    {
      0
    };

  config_flash_read_data(reference_data, MSPI_TIMING_FLASH_TEST_DATA_ADDR,
                         sizeof(reference_data));
  get_flash_tuning_configs(&timing_configs);
  do_tuning(reference_data, &timing_configs, true);
  esp32s3_spi_timing_set_mspi_high_speed(true);
#endif
}
