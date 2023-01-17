/****************************************************************************
 * arch/xtensa/src/esp32/esp32_psram.c
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
#include <stdio.h>
#include <stdbool.h>
#include <strings.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/signal.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp32_rtc.h"
#include "esp32_gpio.h"
#include "esp32_psram.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_iomux.h"
#include "hardware/esp32_rtccntl.h"
#include "hardware/esp32_gpio_sigmap.h"

#include "rom/esp32_efuse.h"
#include "rom/esp32_spiflash.h"
#include "hardware/efuse_reg.h"

#ifdef CONFIG_ESP32_SPIRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTC_VDDSDIO_TIEH_1_8V      0  /* TIEH field value for 1.8V VDDSDIO */
#define RTC_VDDSDIO_TIEH_3_3V      1  /* TIEH field value for 3.3V VDDSDIO */

/* Commands for PSRAM chip */

#define PSRAM_READ                 0x03
#define PSRAM_FAST_READ            0x0b
#define PSRAM_FAST_READ_DUMMY      0x03
#define PSRAM_FAST_READ_QUAD       0xeb
#define PSRAM_FAST_READ_QUAD_DUMMY 0x05
#define PSRAM_WRITE                0x02
#define PSRAM_QUAD_WRITE           0x38
#define PSRAM_ENTER_QMODE          0x35
#define PSRAM_EXIT_QMODE           0xf5
#define PSRAM_RESET_EN             0x66
#define PSRAM_RESET                0x99
#define PSRAM_SET_BURST_LEN        0xc0
#define PSRAM_DEVICE_ID            0x9f

#define PSRAM_ID_KGD_M             0xff
#define PSRAM_ID_KGD_S             8
#define PSRAM_ID_KGD               0x5d
#define PSRAM_ID_EID_M             0xff
#define PSRAM_ID_EID_S             16

/* Use the [7:5](bit7~bit5) of EID to distinguish the psram size:
 *
 *   BIT7  |  BIT6  |  BIT5  |  SIZE(MBIT)
 *   -------------------------------------
 *    0    |   0    |   0    |     16
 *    0    |   0    |   1    |     32
 *    0    |   1    |   0    |     64
 */

#define PSRAM_EID_SIZE_M           0x07
#define PSRAM_EID_SIZE_S           5

#define PSRAM_KGD(id)         (((id) >> PSRAM_ID_KGD_S) & PSRAM_ID_KGD_M)
#define PSRAM_EID(id)         (((id) >> PSRAM_ID_EID_S) & PSRAM_ID_EID_M)
#define PSRAM_SIZE_ID(id)     ((PSRAM_EID(id) >> PSRAM_EID_SIZE_S) & PSRAM_EID_SIZE_M)
#define PSRAM_IS_VALID(id)    (PSRAM_KGD(id) == PSRAM_ID_KGD)

/* For the old version 32Mbit psram, using the special driver */

#define PSRAM_IS_32MBIT_VER0(id)  (PSRAM_EID(id) == 0x20)
#define PSRAM_IS_64MBIT_TRIAL(id) (PSRAM_EID(id) == 0x26)

/* IO-pins for PSRAM.
 * WARNING: PSRAM shares all but the CS and CLK pins with the flash, so these
 * defines hardcode the flash pins as well, making this code incompatible
 * with either a setup that has the flash on non-standard pins or ESP32s with
 * built-in flash.
 */

#define SPI_IOMUX_PIN_NUM_CLK      6
#define SPI_IOMUX_PIN_NUM_CS       11

#define PSRAM_SPIQ_SD0_IO          7
#define PSRAM_SPID_SD1_IO          8
#define PSRAM_SPIWP_SD3_IO         10
#define PSRAM_SPIHD_SD2_IO         9

#define FLASH_HSPI_CLK_IO          14
#define FLASH_HSPI_CS_IO           15
#define PSRAM_HSPI_SPIQ_SD0_IO     12
#define PSRAM_HSPI_SPID_SD1_IO     13
#define PSRAM_HSPI_SPIWP_SD3_IO    2
#define PSRAM_HSPI_SPIHD_SD2_IO    4

/* PSRAM clock and cs IO should be configured based on hardware design.
 * For ESP32-WROVER or ESP32-WROVER-B module, the clock IO is IO17, the CS IO
 * is IO16, they are the default value for these two configs.
 */

#ifndef CONFIG_D0WD_PSRAM_CLK_IO   /* Default is 17 */
#  define CONFIG_D0WD_PSRAM_CLK_IO 17
#endif

#ifndef CONFIG_D0WD_PSRAM_CS_IO    /* Default is 16 */
#  define CONFIG_D0WD_PSRAM_CS_IO  16
#endif

#ifndef CONFIG_D2WD_PSRAM_CLK_IO   /* Default is 9 */
#  define CONFIG_D2WD_PSRAM_CLK_IO 9
#endif

#ifndef CONFIG_D2WD_PSRAM_CS_IO    /* Default is 10 */
#  define CONFIG_D2WD_PSRAM_CS_IO  10
#endif

/* For ESP32-PICO chip, the psram share clock with flash. The flash clock
 * pin is fixed, which is IO6.
 */

#define PICO_PSRAM_CLK_IO          6

#ifndef CONFIG_PICO_PSRAM_CS_IO    /* Default is 10 */
#  define CONFIG_PICO_PSRAM_CS_IO  10
#endif

#ifndef CONFIG_ESP32_SPIRAM_SPIWP_SD3_PIN /* Default is 7 */
#  define CONFIG_ESP32_SPIRAM_SPIWP_SD3_PIN 7
#endif

#define PSRAM_INTERNAL_IO_28       28
#define PSRAM_INTERNAL_IO_29       29
#define PSRAM_IO_MATRIX_DUMMY_40M  ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_40M
#define PSRAM_IO_MATRIX_DUMMY_80M  ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_80M

#define _SPI_CACHE_PORT            0
#define _SPI_FLASH_PORT            1
#define _SPI_80M_CLK_DIV           1
#define _SPI_40M_CLK_DIV           2

/* For 4MB PSRAM, we need one more SPI host, select which one to use by
 * kconfig
 */

#ifdef CONFIG_ESP32_SPIRAM_OCCUPY_HSPI_HOST
#  define PSRAM_SPI_MODULE    PERIPH_HSPI_MODULE
#  define PSRAM_SPI_HOST      HSPI_HOST
#  define PSRAM_CLK_SIGNAL    HSPICLK_OUT_IDX
#  define PSRAM_SPI_NUM       PSRAM_SPI_2
#  define PSRAM_SPICLKEN      DPORT_SPI2_CLK_EN
#elif defined CONFIG_ESP32_SPIRAM_OCCUPY_VSPI_HOST
#  define PSRAM_SPI_MODULE    PERIPH_VSPI_MODULE
#  define PSRAM_SPI_HOST      VSPI_HOST
#  define PSRAM_CLK_SIGNAL    VSPICLK_OUT_IDX
#  define PSRAM_SPI_NUM       PSRAM_SPI_3
#  define PSRAM_SPICLKEN      DPORT_SPI3_CLK_EN
#else   /* set to SPI avoid HSPI and VSPI being used */
#  define PSRAM_SPI_MODULE    PERIPH_SPI_MODULE
#  define PSRAM_SPI_HOST      SPI_HOST
#  define PSRAM_CLK_SIGNAL    SPICLK_OUT_IDX
#  define PSRAM_SPI_NUM       PSRAM_SPI_1
#  define PSRAM_SPICLKEN      DPORT_SPI01_CLK_EN
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  PSRAM_CLK_MODE_NORM = 0,    /* Normal SPI mode */
  PSRAM_CLK_MODE_DCLK = 1,    /* 2 clock cycles after CS is high level */
} psram_clk_mode_t;

typedef enum
{
  PSRAM_EID_SIZE_16MBITS = 0,
  PSRAM_EID_SIZE_32MBITS = 1,
  PSRAM_EID_SIZE_64MBITS = 2,
} psram_eid_size_t;

typedef struct
{
  uint8_t flash_clk_io;
  uint8_t flash_cs_io;
  uint8_t psram_clk_io;
  uint8_t psram_cs_io;
  uint8_t psram_spiq_sd0_io;
  uint8_t psram_spid_sd1_io;
  uint8_t psram_spiwp_sd3_io;
  uint8_t psram_spihd_sd2_io;
} psram_io_t;

typedef enum
{
  PSRAM_SPI_1  = 0x1,
  PSRAM_SPI_2,
  PSRAM_SPI_3,
  PSRAM_SPI_MAX ,
} psram_spi_num_t;

typedef enum
{
  PSRAM_CMD_QPI,
  PSRAM_CMD_SPI,
} psram_cmd_mode_t;

typedef struct
{
  uint16_t cmd;                /* Command value */
  uint16_t cmd_bit_len;        /* Command byte length */
  uint32_t *addr;              /* Point to address value */
  uint16_t addr_bit_len;       /* Address byte length */
  uint32_t *tx_data;           /* Point to send data buffer */
  uint16_t tx_data_bit_len;    /* Send data byte length. */
  uint32_t *rx_data;           /* Point to receive data buffer */
  uint16_t rx_data_bit_len;    /* Receive Data byte length. */
  uint32_t dummy_bit_len;
} psram_cmd_t;

/* Structure describing vddsdio configuration. */

/* Note: this struct is also defined on esp32_pm.c */

struct rtc_vddsdio_config_s
{
  uint32_t force : 1;     /* If 1, use configuration from RTC registers;
                           * if 0, use EFUSE/bootstrapping pins.
                           */
  uint32_t enable : 1;    /* Enable VDDSDIO regulator */
  uint32_t tieh  : 1;     /* Select VDDSDIO voltage. One of
                           * RTC_VDDSDIO_TIEH_1_8V, RTC_VDDSDIO_TIEH_3_3V
                           */
  uint32_t drefh : 2;     /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefm : 2;     /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefl : 2;     /* Tuning parameter for VDDSDIO regulator */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int s_psram_mode = PSRAM_CACHE_MAX;
static psram_clk_mode_t s_clk_mode = PSRAM_CLK_MODE_DCLK;
static uint64_t s_psram_id = 0;
static bool s_2t_mode_enabled = false;
static int extra_dummy = 0;

/****************************************************************************
 * ROM Data
 ****************************************************************************/

/* dummy_len_plus values defined in ROM for SPI flash configuration */

extern uint8_t g_rom_spiflash_dummy_len_plus[];

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int IRAM_ATTR esp32_get_vddsdio_config(
                     struct rtc_vddsdio_config_s *config);

static void IRAM_ATTR
psram_cache_init(int psram_cache_mode,
                 int vaddrmode);

static void psram_clear_spi_fifo(psram_spi_num_t spi_num);

static void psram_set_basic_write_mode(psram_spi_num_t spi_num);

static void psram_set_qio_write_mode(psram_spi_num_t spi_num);

static void psram_set_qio_read_mode(psram_spi_num_t spi_num);

static void psram_set_basic_read_mode(psram_spi_num_t spi_num);

static void IRAM_ATTR
psram_cmd_recv_start(psram_spi_num_t spi_num,
                     uint32_t *data,
                     uint16_t rx_byte_len,
                     psram_cmd_mode_t cmd_mode);

static int psram_cmd_config(psram_spi_num_t spi_num, psram_cmd_t *data);

static void psram_cmd_end(int spi_num);

static void psram_disable_qio_mode(psram_spi_num_t spi_num);

static void psram_read_id(uint64_t *dev_id);

static int IRAM_ATTR
psram_enable_qio_mode(psram_spi_num_t spi_num);

#if defined(CONFIG_ESP32_SPIRAM_2T_MODE)
static void spi_user_psram_write(psram_spi_num_t spi_num, uint32_t address,
                                 uint32_t *data_buffer, uint32_t data_len);

static void spi_user_psram_read(psram_spi_num_t spi_num, uint32_t address,
                                uint32_t *data_buffer, uint32_t data_len);

static int IRAM_ATTR
psram_2t_mode_enable(psram_spi_num_t spi_num);

static int psram_2t_mode_check(psram_spi_num_t spi_num);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_get_vddsdio_config
 *
 * Description:
 *   Get current VDDSDIO configuration.
 *
 * Input Parameters:
 *   Incoming parameter address of VDDSDIO configuration to be saved
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_get_vddsdio_config(
                     struct rtc_vddsdio_config_s *config)
{
  struct rtc_vddsdio_config_s *result = config;
  uint32_t efuse_reg;
  uint32_t strap_reg;
  uint32_t sdio_conf_reg = getreg32(RTC_CNTL_SDIO_CONF_REG);

  result->drefh = (sdio_conf_reg & RTC_CNTL_DREFH_SDIO_M)
                                 >> RTC_CNTL_DREFH_SDIO_S;
  result->drefm = (sdio_conf_reg & RTC_CNTL_DREFM_SDIO_M)
                                 >> RTC_CNTL_DREFM_SDIO_S;
  result->drefl = (sdio_conf_reg & RTC_CNTL_DREFL_SDIO_M)
                                 >> RTC_CNTL_DREFL_SDIO_S;

  if (sdio_conf_reg & RTC_CNTL_SDIO_FORCE)
    {
      /* Get configuration from RTC */

      result->force = 1;
      result->enable = (sdio_conf_reg & RTC_CNTL_XPD_SDIO_REG_M)
                                      >> RTC_CNTL_XPD_SDIO_REG_S;
      result->tieh = (sdio_conf_reg & RTC_CNTL_SDIO_TIEH_M)
                                      >> RTC_CNTL_SDIO_TIEH_S;

      return OK;
    }

  efuse_reg = getreg32(EFUSE_BLK0_RDATA4_REG);

  if (efuse_reg & EFUSE_RD_SDIO_FORCE)
    {
      /* Get configuration from EFUSE */

      result->force = 0;
      result->enable = (efuse_reg & EFUSE_RD_XPD_SDIO_REG_M)
                                  >> EFUSE_RD_XPD_SDIO_REG_S;
      result->tieh = (efuse_reg & EFUSE_RD_SDIO_TIEH_M)
                                >> EFUSE_RD_SDIO_TIEH_S;

      if (REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG,
              EFUSE_RD_BLK3_PART_RESERVE) == 0)
        {
          result->drefh = (efuse_reg & EFUSE_RD_SDIO_DREFH_M)
                                     >> EFUSE_RD_SDIO_DREFH_S;
          result->drefm = (efuse_reg & EFUSE_RD_SDIO_DREFM_M)
                                     >> EFUSE_RD_SDIO_DREFM_S;
          result->drefl = (efuse_reg & EFUSE_RD_SDIO_DREFL_M)
                                     >> EFUSE_RD_SDIO_DREFL_S;
        }

      return OK;
    }

  /* Otherwise, VDD_SDIO is controlled by bootstrapping pin */

  strap_reg = getreg32(GPIO_STRAP_REG);
  result->force = 0;
  result->tieh = (strap_reg & BIT(5)) ? RTC_VDDSDIO_TIEH_1_8V
                                      : RTC_VDDSDIO_TIEH_3_3V;
  result->enable = 1;

  return OK;
}

/* register initialization for sram cache params and r/w commands */

static void IRAM_ATTR
  psram_cache_init(int psram_cache_mode, int vaddrmode)
{
  uint32_t regval;

  switch (psram_cache_mode)
    {
      case PSRAM_CACHE_F80M_S80M:

        /* flash 1 div clk,80+40; */

        modifyreg32(SPI_DATE_REG(0), BIT(31), 0);

        /* pre clk div , ONLY IF SPI/SRAM@ DIFFERENT SPEED,JUST FOR SPI0.
         * FLASH DIV 2+SRAM DIV4
         */

        modifyreg32(SPI_DATE_REG(0), BIT(30), 0);
        break;

      case PSRAM_CACHE_F80M_S40M:
        modifyreg32(SPI_CLOCK_REG(0), SPI_CLK_EQU_SYSCLK_M, 0);
        SET_PERI_REG_BITS(SPI_CLOCK_REG(0), SPI_CLKDIV_PRE_V, 0,
                          SPI_CLKDIV_PRE_S);
        SET_PERI_REG_BITS(SPI_CLOCK_REG(0), SPI_CLKCNT_N, 1, SPI_CLKCNT_N_S);
        SET_PERI_REG_BITS(SPI_CLOCK_REG(0), SPI_CLKCNT_H, 0, SPI_CLKCNT_H_S);
        SET_PERI_REG_BITS(SPI_CLOCK_REG(0), SPI_CLKCNT_L, 1, SPI_CLKCNT_L_S);
        modifyreg32(SPI_DATE_REG(0), BIT(31), 0); /* flash 1 div clk */

        /* pre clk div , ONLY IF SPI/SRAM@ DIFFERENT SPEED,JUST FOR SPI0. */

        modifyreg32(SPI_DATE_REG(0), BIT(30), 0);
        break;
      case PSRAM_CACHE_F40M_S40M:
      default:

        /* flash 1 div clk */

        modifyreg32(SPI_DATE_REG(0), BIT(31), 0);

        /* pre clk div */

        modifyreg32(SPI_DATE_REG(0), BIT(30), 0);
        break;
    }

  /* disable dio mode for cache command */

  modifyreg32(SPI_CACHE_SCTRL_REG(0), SPI_USR_SRAM_DIO_M, 0);

  /* enable qio mode for cache command */

  modifyreg32(SPI_CACHE_SCTRL_REG(0), 0, SPI_USR_SRAM_QIO_M);

  /* enable cache read command */

  modifyreg32(SPI_CACHE_SCTRL_REG(0), 0, SPI_CACHE_SRAM_USR_RCMD_M);

  /* enable cache write command */

  modifyreg32(SPI_CACHE_SCTRL_REG(0), 0, SPI_CACHE_SRAM_USR_WCMD_M);

  /* write address for cache command */

  SET_PERI_REG_BITS(SPI_CACHE_SCTRL_REG(0), SPI_SRAM_ADDR_BITLEN_V, 23,
                    SPI_SRAM_ADDR_BITLEN_S);

  /* enable cache read dummy */

  modifyreg32(SPI_CACHE_SCTRL_REG(0), 0, SPI_USR_RD_SRAM_DUMMY_M);

  /* config sram cache r/w command */

  SET_PERI_REG_BITS(SPI_SRAM_DRD_CMD_REG(0),
                    SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_V, 7,
                    SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_S);

  SET_PERI_REG_BITS(SPI_SRAM_DRD_CMD_REG(0),
                    SPI_CACHE_SRAM_USR_RD_CMD_VALUE_V,
                    PSRAM_FAST_READ_QUAD,
                    SPI_CACHE_SRAM_USR_RD_CMD_VALUE_S); /* 0xEB */

  SET_PERI_REG_BITS(SPI_SRAM_DWR_CMD_REG(0),
                    SPI_CACHE_SRAM_USR_WR_CMD_BITLEN, 7,
                    SPI_CACHE_SRAM_USR_WR_CMD_BITLEN_S);

  SET_PERI_REG_BITS(SPI_SRAM_DWR_CMD_REG(0),
                    SPI_CACHE_SRAM_USR_WR_CMD_VALUE,
                    PSRAM_QUAD_WRITE,
                    SPI_CACHE_SRAM_USR_WR_CMD_VALUE_S); /* 0x38 */

  /* dummy, psram cache : 40m--+1dummy; 80m--+2dummy */

  SET_PERI_REG_BITS(SPI_CACHE_SCTRL_REG(0), SPI_SRAM_DUMMY_CYCLELEN_V,
                    PSRAM_FAST_READ_QUAD_DUMMY + extra_dummy,
                    SPI_SRAM_DUMMY_CYCLELEN_S);

  switch (psram_cache_mode)
    {
      /* in this mode , no delay is needed */

      case PSRAM_CACHE_F80M_S80M:
        break;

      /* if sram is @40M, need 2 cycles of delay */

      case PSRAM_CACHE_F80M_S40M:
      case PSRAM_CACHE_F40M_S40M:
      default:
        if (s_clk_mode == PSRAM_CLK_MODE_DCLK)
          {
            /* read command length, 2 bytes(1byte for delay), sending in qio
             * mode in cache
             */

            SET_PERI_REG_BITS(SPI_SRAM_DRD_CMD_REG(0),
                              SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_V, 15,
                              SPI_CACHE_SRAM_USR_RD_CMD_BITLEN_S);

            /* 0xEB, read command value,(0x00 for delay,0xeb for cmd) */

            SET_PERI_REG_BITS(SPI_SRAM_DRD_CMD_REG(0),
                              SPI_CACHE_SRAM_USR_RD_CMD_VALUE_V,
                              ((PSRAM_FAST_READ_QUAD) << 8),
                              SPI_CACHE_SRAM_USR_RD_CMD_VALUE_S);

            /* write command length,2 bytes(1byte for delay,send in qio mode
             * in cache)
             */

            SET_PERI_REG_BITS(SPI_SRAM_DWR_CMD_REG(0),
                              SPI_CACHE_SRAM_USR_WR_CMD_BITLEN, 15,
                              SPI_CACHE_SRAM_USR_WR_CMD_BITLEN_S);

            /* 0x38, write command value,(0x00 for delay) */

            SET_PERI_REG_BITS(SPI_SRAM_DWR_CMD_REG(0),
                              SPI_CACHE_SRAM_USR_WR_CMD_VALUE,
                              ((PSRAM_QUAD_WRITE) << 8),
                              SPI_CACHE_SRAM_USR_WR_CMD_VALUE_S);

            /* dummy, psram cache : 40m--+1dummy; 80m--+2dummy */

            SET_PERI_REG_BITS(SPI_CACHE_SCTRL_REG(0),
                              SPI_SRAM_DUMMY_CYCLELEN_V,
                              PSRAM_FAST_READ_QUAD_DUMMY + extra_dummy,
                              SPI_SRAM_DUMMY_CYCLELEN_S);
          }
        break;
    }

  modifyreg32(DPORT_PRO_CACHE_CTRL_REG,
              DPORT_PRO_DRAM_HL | DPORT_PRO_DRAM_SPLIT, 0);
  modifyreg32(DPORT_APP_CACHE_CTRL_REG,
              DPORT_APP_DRAM_HL | DPORT_APP_DRAM_SPLIT, 0);
  if (vaddrmode == PSRAM_VADDR_MODE_LOWHIGH)
    {
      modifyreg32(DPORT_PRO_CACHE_CTRL_REG, 0, DPORT_PRO_DRAM_HL);
      modifyreg32(DPORT_APP_CACHE_CTRL_REG, 0, DPORT_APP_DRAM_HL);
    }
  else
    {
      if (vaddrmode == PSRAM_VADDR_MODE_EVENODD)
        {
          modifyreg32(DPORT_PRO_CACHE_CTRL_REG, 0, DPORT_PRO_DRAM_SPLIT);
          modifyreg32(DPORT_APP_CACHE_CTRL_REG, 0, DPORT_APP_DRAM_SPLIT);
        }
    }

  /* use Dram1 to visit ext sram. */

  modifyreg32(DPORT_PRO_CACHE_CTRL1_REG,
              DPORT_PRO_CACHE_MASK_DRAM1 | DPORT_PRO_CACHE_MASK_OPSDRAM, 0);

  /* cache page mode :
   * 1 -->16k
   * 4 -->2k
   * 0 -->32k,(accord with the settings in cache_sram_mmu_set)
   */

  /* get into unknown exception if not comment */

  regval  = getreg32(DPORT_PRO_CACHE_CTRL1_REG);
  regval &= ~(DPORT_PRO_CMMU_SRAM_PAGE_MODE <<
              DPORT_PRO_CMMU_SRAM_PAGE_MODE_S);
  putreg32(regval, DPORT_PRO_CACHE_CTRL1_REG);

  /* use DRAM1 to visit ext sram. */

  modifyreg32(DPORT_APP_CACHE_CTRL1_REG,
              DPORT_APP_CACHE_MASK_DRAM1 |
              DPORT_APP_CACHE_MASK_OPSDRAM, 0);

  /* cache page mode :
   * 1 -->16k
   * 4 -->2k
   * 0 -->32k, (accord with the settings in cache_sram_mmu_set)
   */

  regval  = getreg32(DPORT_APP_CACHE_CTRL1_REG);
  regval &= ~(DPORT_APP_CMMU_SRAM_PAGE_MODE <<
              DPORT_APP_CMMU_SRAM_PAGE_MODE_S);
  putreg32(regval, DPORT_APP_CACHE_CTRL1_REG);

  /* ENABLE SPI0 CS1 TO PSRAM(CS0--FLASH; CS1--SRAM) */

  modifyreg32(SPI_PIN_REG(0), SPI_CS1_DIS_M, 0);
}

static void psram_clear_spi_fifo(psram_spi_num_t spi_num)
{
  int i;

  for (i = 0; i < 16; i++)
    {
      putreg32(0, SPI_W0_REG(spi_num) + i * 4);
    }
}

/* set basic SPI write mode */

static void psram_set_basic_write_mode(psram_spi_num_t spi_num)
{
  modifyreg32(SPI_USER_REG(spi_num), SPI_FWRITE_QIO, 0);
  modifyreg32(SPI_USER_REG(spi_num), SPI_FWRITE_DIO, 0);
  modifyreg32(SPI_USER_REG(spi_num), SPI_FWRITE_QUAD, 0);
  modifyreg32(SPI_USER_REG(spi_num), SPI_FWRITE_DUAL, 0);
}

/* set QPI write mode */

static void psram_set_qio_write_mode(psram_spi_num_t spi_num)
{
  modifyreg32(SPI_USER_REG(spi_num), 0, SPI_FWRITE_QIO);
  modifyreg32(SPI_USER_REG(spi_num), SPI_FWRITE_DIO, 0);
  modifyreg32(SPI_USER_REG(spi_num), SPI_FWRITE_QUAD, 0);
  modifyreg32(SPI_USER_REG(spi_num), SPI_FWRITE_DUAL, 0);
}

/* set QPI read mode */

static void psram_set_qio_read_mode(psram_spi_num_t spi_num)
{
  modifyreg32(SPI_CTRL_REG(spi_num), 0, SPI_FREAD_QIO);
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_FREAD_QUAD, 0);
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_FREAD_DUAL, 0);
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_FREAD_DIO, 0);
}

/* set SPI read mode */

static void psram_set_basic_read_mode(psram_spi_num_t spi_num)
{
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_FREAD_QIO, 0);
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_FREAD_QUAD, 0);
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_FREAD_DUAL, 0);
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_FREAD_DIO, 0);
}

/* start sending cmd/addr and optionally, receiving data */

static void IRAM_ATTR
psram_cmd_recv_start(psram_spi_num_t spi_num,
                     uint32_t *data,
                     uint16_t rx_byte_len,
                     psram_cmd_mode_t cmd_mode)
{
  /* Get CS1 */

  modifyreg32(SPI_PIN_REG(PSRAM_SPI_1), SPI_CS1_DIS_M, 0);
  modifyreg32(SPI_PIN_REG(PSRAM_SPI_1), 0, SPI_CS0_DIS_M);

  uint32_t mode_backup = (getreg32(SPI_USER_REG(spi_num)) >>
                          SPI_FWRITE_DUAL_S) & 0xf;

  uint32_t rd_mode_backup = getreg32(SPI_CTRL_REG(spi_num)) &
                            (SPI_FREAD_DIO_M | SPI_FREAD_DUAL_M |
                             SPI_FREAD_QUAD_M | SPI_FREAD_QIO_M);

  if (cmd_mode == PSRAM_CMD_SPI)
    {
      psram_set_basic_write_mode(spi_num);
      psram_set_basic_read_mode(spi_num);
    }
  else
    {
      if (cmd_mode == PSRAM_CMD_QPI)
        {
          psram_set_qio_write_mode(spi_num);
          psram_set_qio_read_mode(spi_num);
        }
    }

  /* Wait for SPI0 to idle */

  while (getreg32(SPI_EXT2_REG(0)) != 0);

#ifndef CONFIG_ESP32_PID
  modifyreg32(DPORT_HOST_INF_SEL_REG, 0, 1 << 14);
#endif

  /* Start send data */

  modifyreg32(SPI_CMD_REG(spi_num), 0, SPI_USR);

  while ((getreg32(SPI_CMD_REG(spi_num)) & SPI_USR));

#ifndef CONFIG_ESP32_PID
  modifyreg32(DPORT_HOST_INF_SEL_REG, 1 << 14, 0);
#endif

  /* recover spi mode */

  SET_PERI_REG_BITS(SPI_USER_REG(spi_num),
                    (data?SPI_FWRITE_DUAL_M:0xf),
                    mode_backup, SPI_FWRITE_DUAL_S);

  modifyreg32(SPI_CTRL_REG(spi_num),
                      (SPI_FREAD_DIO_M | SPI_FREAD_DUAL_M |
                      SPI_FREAD_QUAD_M | SPI_FREAD_QIO_M), 0);

  modifyreg32(SPI_CTRL_REG(spi_num), 0, rd_mode_backup);

  /* return cs to cs0 */

  modifyreg32(SPI_PIN_REG(PSRAM_SPI_1), 0, SPI_CS1_DIS_M);
  modifyreg32(SPI_PIN_REG(PSRAM_SPI_1), SPI_CS0_DIS_M, 0);

  if (data)
    {
      int idx = 0;

      /* Read data out */

      do
        {
          *data++ = getreg32(SPI_W0_REG(spi_num) + (idx << 2));
        }
      while (++idx < ((rx_byte_len / 4) + ((rx_byte_len % 4) ? 1 : 0)));
    }
}

static uint32_t backup_usr[3];
static uint32_t backup_usr1[3];
static uint32_t backup_usr2[3];

/* setup spi command/addr/data/dummy in user mode */

static int psram_cmd_config(psram_spi_num_t spi_num, psram_cmd_t *data)
{
  while (getreg32(SPI_CMD_REG(spi_num)) & SPI_USR);

  backup_usr[spi_num]  = getreg32(SPI_USER_REG(spi_num));
  backup_usr1[spi_num] = getreg32(SPI_USER1_REG(spi_num));
  backup_usr2[spi_num] = getreg32(SPI_USER2_REG(spi_num));

  /* Set command by user. */

  if (data->cmd_bit_len != 0)
    {
      /* Max command length 16 bits. */

      SET_PERI_REG_BITS(SPI_USER2_REG(spi_num),
                        SPI_USR_COMMAND_BITLEN,
                        data->cmd_bit_len - 1,
                        SPI_USR_COMMAND_BITLEN_S);

      /* Enable command */

      modifyreg32(SPI_USER_REG(spi_num), 0, SPI_USR_COMMAND);

      /* Load command,bit15-0 is cmd value. */

      SET_PERI_REG_BITS(SPI_USER2_REG(spi_num),
                        SPI_USR_COMMAND_VALUE,
                        data->cmd,
                        SPI_USR_COMMAND_VALUE_S);
    }
  else
    {
      modifyreg32(SPI_USER_REG(spi_num), SPI_USR_COMMAND, 0);
      SET_PERI_REG_BITS(SPI_USER2_REG(spi_num), SPI_USR_COMMAND_BITLEN,
                        0, SPI_USR_COMMAND_BITLEN_S);
    }

  /* Set Address by user. */

  if (data->addr_bit_len != 0)
    {
      SET_PERI_REG_BITS(SPI_USER1_REG(spi_num), SPI_USR_ADDR_BITLEN,
                        (data->addr_bit_len - 1),
                        SPI_USR_ADDR_BITLEN_S);

      /* Enable address */

      modifyreg32(SPI_USER_REG(spi_num), 0, SPI_USR_ADDR);

      /* Set address */

      putreg32(*data->addr, SPI_ADDR_REG(spi_num));
    }
  else
    {
      modifyreg32(SPI_USER_REG(spi_num), SPI_USR_ADDR, 0);
      SET_PERI_REG_BITS(SPI_USER1_REG(spi_num), SPI_USR_ADDR_BITLEN,
                        0, SPI_USR_ADDR_BITLEN_S);
    }

  /* Set data by user. */

  uint32_t *p_tx_val = data->tx_data;

  if (data->tx_data_bit_len != 0)
    {
      /* Enable MOSI */

      modifyreg32(SPI_USER_REG(spi_num), 0, SPI_USR_MOSI);

      /* Load send buffer */

      int len = (data->tx_data_bit_len + 31) / 32;
      if (p_tx_val != NULL)
        {
          memcpy((void *) SPI_W0_REG(spi_num), p_tx_val, len * 4);
        }

      /* Set data send buffer length.Max data length 64 bytes. */

      SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(spi_num), SPI_USR_MOSI_DBITLEN,
                        (data->tx_data_bit_len - 1),
                        SPI_USR_MOSI_DBITLEN_S);
    }
  else
    {
      modifyreg32(SPI_USER_REG(spi_num), SPI_USR_MOSI, 0);
      SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(spi_num), SPI_USR_MOSI_DBITLEN,
                        0, SPI_USR_MOSI_DBITLEN_S);
    }

  /* Set rx data by user. */

  if (data->rx_data_bit_len != 0)
    {
      /* Enable MOSI */

      modifyreg32(SPI_USER_REG(spi_num), 0, SPI_USR_MISO);

      /* Set data send buffer length.Max data length 64 bytes. */

      SET_PERI_REG_BITS(SPI_MISO_DLEN_REG(spi_num), SPI_USR_MISO_DBITLEN,
                        (data->rx_data_bit_len - 1),
                        SPI_USR_MISO_DBITLEN_S);
    }
  else
    {
      modifyreg32(SPI_USER_REG(spi_num), SPI_USR_MISO, 0);
      SET_PERI_REG_BITS(SPI_MISO_DLEN_REG(spi_num), SPI_USR_MISO_DBITLEN,
                        0, SPI_USR_MISO_DBITLEN_S);
    }

  if (data->dummy_bit_len != 0)
    {
      modifyreg32(SPI_USER_REG(PSRAM_SPI_1), 0, SPI_USR_DUMMY); /* Dummy EN */
      SET_PERI_REG_BITS(SPI_USER1_REG(PSRAM_SPI_1), SPI_USR_DUMMY_CYCLELEN_V,
                        data->dummy_bit_len - 1,
                        SPI_USR_DUMMY_CYCLELEN_S);  /* DUMMY */
    }
  else
    {
      modifyreg32(SPI_USER_REG(PSRAM_SPI_1), SPI_USR_DUMMY, 0); /* dummy en */
      SET_PERI_REG_BITS(SPI_USER1_REG(PSRAM_SPI_1), SPI_USR_DUMMY_CYCLELEN_V,
                        0, SPI_USR_DUMMY_CYCLELEN_S);  /* DUMMY */
    }

  return 0;
}

static void psram_cmd_end(int spi_num)
{
  while (getreg32(SPI_CMD_REG(spi_num)) & SPI_USR);

  putreg32(backup_usr[spi_num], SPI_USER_REG(spi_num));
  putreg32(backup_usr1[spi_num], SPI_USER1_REG(spi_num));
  putreg32(backup_usr2[spi_num], SPI_USER2_REG(spi_num));
}

/* exit QPI mode(set back to SPI mode) */

static void psram_disable_qio_mode(psram_spi_num_t spi_num)
{
  psram_cmd_t ps_cmd;
  uint32_t cmd_exit_qpi;

  cmd_exit_qpi = PSRAM_EXIT_QMODE;
  ps_cmd.tx_data_bit_len = 8;
  if (s_clk_mode == PSRAM_CLK_MODE_DCLK)
    {
      switch (s_psram_mode)
        {
          case PSRAM_CACHE_F80M_S80M:
            break;
          case PSRAM_CACHE_F80M_S40M:
          case PSRAM_CACHE_F40M_S40M:
          default:
            cmd_exit_qpi = PSRAM_EXIT_QMODE << 8;
            ps_cmd.tx_data_bit_len = 16;
            break;
        }
    }

  ps_cmd.tx_data = &cmd_exit_qpi;
  ps_cmd.cmd = 0;
  ps_cmd.cmd_bit_len = 0;
  ps_cmd.addr = 0;
  ps_cmd.addr_bit_len = 0;
  ps_cmd.rx_data = NULL;
  ps_cmd.rx_data_bit_len = 0;
  ps_cmd.dummy_bit_len = 0;
  psram_cmd_config(spi_num, &ps_cmd);
  psram_cmd_recv_start(spi_num, NULL, 0, PSRAM_CMD_QPI);
  psram_cmd_end(spi_num);
}

/* read psram id */

static void psram_read_id(uint64_t *dev_id)
{
  psram_spi_num_t spi_num = PSRAM_SPI_1;
  psram_disable_qio_mode(spi_num);
  uint32_t dummy_bits = 0 + extra_dummy;
  uint32_t psram_id[2] =
                          {
                            0
                          };

  psram_cmd_t ps_cmd;

  uint32_t addr = 0;
  ps_cmd.addr_bit_len = 3 * 8;
  ps_cmd.cmd = PSRAM_DEVICE_ID;
  ps_cmd.cmd_bit_len = 8;

  if (s_clk_mode == PSRAM_CLK_MODE_DCLK)
    {
      switch (s_psram_mode)
        {
          case PSRAM_CACHE_F80M_S80M:
            break;
          case PSRAM_CACHE_F80M_S40M:
          case PSRAM_CACHE_F40M_S40M:
          default:
            ps_cmd.cmd_bit_len = 2;   /* this 2 bits is used to delay 2 clk cycle */
            ps_cmd.cmd = 0;
            addr = (PSRAM_DEVICE_ID << 24) | 0;
            ps_cmd.addr_bit_len = 4 * 8;
            break;
        }
    }

  ps_cmd.addr = &addr;
  ps_cmd.tx_data_bit_len = 0;
  ps_cmd.tx_data = NULL;
  ps_cmd.rx_data_bit_len = 8 * 8;
  ps_cmd.rx_data = psram_id;
  ps_cmd.dummy_bit_len = dummy_bits;

  psram_cmd_config(spi_num, &ps_cmd);
  psram_clear_spi_fifo(spi_num);
  psram_cmd_recv_start(spi_num, ps_cmd.rx_data,
                       ps_cmd.rx_data_bit_len / 8, PSRAM_CMD_SPI);
  psram_cmd_end(spi_num);
  *dev_id = (uint64_t)(((uint64_t)psram_id[1] << 32) | psram_id[0]);
}

/* enter QPI mode */

static int IRAM_ATTR
psram_enable_qio_mode(psram_spi_num_t spi_num)
{
  psram_cmd_t ps_cmd;
  uint32_t addr = (PSRAM_ENTER_QMODE << 24) | 0;

  ps_cmd.cmd_bit_len = 0;

  if (s_clk_mode == PSRAM_CLK_MODE_DCLK)
    {
      switch (s_psram_mode)
        {
          case PSRAM_CACHE_F80M_S80M:
            break;
          case PSRAM_CACHE_F80M_S40M:
          case PSRAM_CACHE_F40M_S40M:
          default:
            ps_cmd.cmd_bit_len = 2;
            break;
        }
    }

  ps_cmd.cmd = 0;
  ps_cmd.addr = &addr;
  ps_cmd.addr_bit_len = 8;
  ps_cmd.tx_data = NULL;
  ps_cmd.tx_data_bit_len = 0;
  ps_cmd.rx_data = NULL;
  ps_cmd.rx_data_bit_len = 0;
  ps_cmd.dummy_bit_len = 0;
  psram_cmd_config(spi_num, &ps_cmd);
  psram_cmd_recv_start(spi_num, NULL, 0, PSRAM_CMD_SPI);
  psram_cmd_end(spi_num);
  return OK;
}

#if defined(CONFIG_ESP32_SPIRAM_2T_MODE)
/* use SPI user mode to write psram */

static void spi_user_psram_write(psram_spi_num_t spi_num, uint32_t address,
                                 uint32_t *data_buffer, uint32_t data_len)
{
  uint32_t addr = (PSRAM_QUAD_WRITE << 24) | (address & 0x7fffff);
  psram_cmd_t ps_cmd;
  ps_cmd.cmd_bit_len = 0;
  ps_cmd.cmd = 0;
  ps_cmd.addr = &addr;
  ps_cmd.addr_bit_len = 4 * 8;
  ps_cmd.tx_data_bit_len = 32 * 8;
  ps_cmd.tx_data = NULL;
  ps_cmd.rx_data_bit_len = 0;
  ps_cmd.rx_data = NULL;
  ps_cmd.dummy_bit_len = 0;

  for (uint32_t i = 0; i < data_len; i += 32)
    {
      psram_clear_spi_fifo(spi_num);
      addr = (PSRAM_QUAD_WRITE << 24) | ((address & 0x7fffff) + i);
      ps_cmd.tx_data = data_buffer + (i / 4);
      psram_cmd_config(spi_num, &ps_cmd);
      psram_cmd_recv_start(spi_num, ps_cmd.rx_data,
                           ps_cmd.rx_data_bit_len / 8, PSRAM_CMD_QPI);
    }

  psram_cmd_end(spi_num);
}

/* use SPI user mode to read psram */

static void spi_user_psram_read(psram_spi_num_t spi_num, uint32_t address,
                                uint32_t *data_buffer, uint32_t data_len)
{
  uint32_t addr = (PSRAM_FAST_READ_QUAD << 24) | (address & 0x7fffff);
  uint32_t dummy_bits = PSRAM_FAST_READ_QUAD_DUMMY + 1;

  psram_cmd_t ps_cmd;
  ps_cmd.cmd_bit_len = 0;
  ps_cmd.cmd = 0;
  ps_cmd.addr = &addr;
  ps_cmd.addr_bit_len = 4 * 8;
  ps_cmd.tx_data_bit_len = 0;
  ps_cmd.tx_data = NULL;
  ps_cmd.rx_data_bit_len = 32 * 8;
  ps_cmd.dummy_bit_len = dummy_bits + extra_dummy;

  for (uint32_t i = 0; i < data_len; i += 32)
    {
      psram_clear_spi_fifo(spi_num);
      addr = (PSRAM_FAST_READ_QUAD << 24) | ((address & 0x7fffff) + i);
      ps_cmd.rx_data = data_buffer + (i / 4);
      psram_cmd_config(spi_num, &ps_cmd);
      psram_cmd_recv_start(spi_num, ps_cmd.rx_data,
                           ps_cmd.rx_data_bit_len / 8, PSRAM_CMD_QPI);
    }

  psram_cmd_end(spi_num);
}

/* enable psram 2T mode */

static int IRAM_ATTR
psram_2t_mode_enable(psram_spi_num_t spi_num)
{
  psram_disable_qio_mode(spi_num);

  /* configure psram clock as 5 MHz */

  uint32_t div = rtc_clk_apb_freq_get() / 5000000;
  esp_rom_spiflash_config_clk(div, spi_num);

  psram_cmd_t ps_cmd;

  /* setp1: send cmd 0x5e
   *        send one more bit clock after send cmd
   */

  ps_cmd.cmd = 0x5e;
  ps_cmd.cmd_bit_len = 8;
  ps_cmd.addr_bit_len = 0;
  ps_cmd.addr = 0;
  ps_cmd.tx_data_bit_len = 0;
  ps_cmd.tx_data = NULL;
  ps_cmd.rx_data_bit_len = 0;
  ps_cmd.rx_data = NULL;
  ps_cmd.dummy_bit_len = 1;
  psram_cmd_config(spi_num, &ps_cmd);
  psram_clear_spi_fifo(spi_num);
  psram_cmd_recv_start(spi_num, NULL, 0, PSRAM_CMD_SPI);
  psram_cmd_end(spi_num);

  /* setp2: send cmd 0x5f
   *        send one more bit clock after send cmd
   */

  ps_cmd.cmd = 0x5f;
  psram_cmd_config(spi_num, &ps_cmd);
  psram_clear_spi_fifo(spi_num);
  psram_cmd_recv_start(spi_num, NULL, 0, PSRAM_CMD_SPI);
  psram_cmd_end(spi_num);

  /* setp3: keep cs as high level
   *        send 128 cycles clock
   *        send 1 bit high level in ninth clock from the back to PSRAM SIO1
   */

  GPIO_OUTPUT_SET(CONFIG_D0WD_PSRAM_CS_IO, 1);
  esp32_gpio_matrix_out(CONFIG_D0WD_PSRAM_CS_IO, SIG_GPIO_OUT_IDX, 0, 0);

  esp32_gpio_matrix_out(PSRAM_SPID_SD1_IO, SPIQ_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(PSRAM_SPID_SD1_IO, SPIQ_IN_IDX, 0);
  esp32_gpio_matrix_out(PSRAM_SPIQ_SD0_IO, SPID_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(PSRAM_SPIQ_SD0_IO, SPID_IN_IDX, 0);

  uint32_t w_data_2t[4] =
                          {
                            0x0, 0x0, 0x0, 0x00010000
                          };

  ps_cmd.cmd = 0;
  ps_cmd.cmd_bit_len = 0;
  ps_cmd.tx_data_bit_len = 128;
  ps_cmd.tx_data = w_data_2t;
  ps_cmd.dummy_bit_len = 0;
  psram_clear_spi_fifo(spi_num);
  psram_cmd_config(spi_num, &ps_cmd);
  psram_cmd_recv_start(spi_num, NULL, 0, PSRAM_CMD_SPI);
  psram_cmd_end(spi_num);

  esp32_gpio_matrix_out(PSRAM_SPIQ_SD0_IO, SPIQ_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(PSRAM_SPIQ_SD0_IO, SPIQ_IN_IDX, 0);
  esp32_gpio_matrix_out(PSRAM_SPID_SD1_IO, SPID_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(PSRAM_SPID_SD1_IO, SPID_IN_IDX, 0);

  esp32_gpio_matrix_out(CONFIG_D0WD_PSRAM_CS_IO, SPICS1_OUT_IDX, 0, 0);

  /* setp4: send cmd 0x5f
   *        send one more bit clock after send cmd
   */

  ps_cmd.cmd = 0x5f;
  ps_cmd.cmd_bit_len = 8;
  ps_cmd.tx_data_bit_len = 0;
  ps_cmd.tx_data = NULL;
  ps_cmd.dummy_bit_len = 1;
  psram_cmd_config(spi_num, &ps_cmd);
  psram_clear_spi_fifo(spi_num);
  psram_cmd_recv_start(spi_num, NULL, 0, PSRAM_CMD_SPI);
  psram_cmd_end(spi_num);

  /* configure psram clock back to the default value */

  switch (s_psram_mode)
    {
      case PSRAM_CACHE_F80M_S40M:
      case PSRAM_CACHE_F40M_S40M:
        esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, spi_num);
        break;
      case PSRAM_CACHE_F80M_S80M:
        esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, spi_num);
        break;
      default:
        break;
    }

  psram_enable_qio_mode(spi_num);
  return OK;
}

#define CHECK_DATA_LEN   (1024)
#define CHECK_ADDR_STEP  (0x100000)
#define SIZE_32MBIT      (0x400000)
#define SIZE_64MBIT      (0x800000)

static int psram_2t_mode_check(psram_spi_num_t spi_num)
{
  uint32_t addr;
  uint8_t w_check_data[CHECK_DATA_LEN] =
                                          {
                                            0
                                          };

  uint8_t r_check_data[CHECK_DATA_LEN] =
                                          {
                                            0
                                          };

  for (uint32_t addr = 0; addr < SIZE_32MBIT; addr += CHECK_ADDR_STEP)
    {
      spi_user_psram_write(spi_num, addr, (uint32_t *)w_check_data,
                           CHECK_DATA_LEN);
    }

  memset(w_check_data, 0xff, sizeof(w_check_data));

  for (addr = SIZE_32MBIT; addr < SIZE_64MBIT; addr += CHECK_ADDR_STEP)
    {
      spi_user_psram_write(spi_num, addr, (uint32_t *)w_check_data,
                           CHECK_DATA_LEN);
    }

  for (uint32_t addr = 0; addr < SIZE_32MBIT; addr += CHECK_ADDR_STEP)
    {
      spi_user_psram_read(spi_num, addr, (uint32_t *)r_check_data,
                          CHECK_DATA_LEN);
      for (uint32_t j = 0; j < CHECK_DATA_LEN; j++)
        {
          if (r_check_data[j] != 0xff)
            {
              return -EFAULT;
            }
        }
    }

  return ESP_OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void psram_set_cs_timing(psram_spi_num_t spi_num, psram_clk_mode_t clk_mode)
{
  if (clk_mode == PSRAM_CLK_MODE_NORM)
    {
      modifyreg32(SPI_USER_REG(spi_num), 0, SPI_CS_HOLD_M | SPI_CS_SETUP_M);

      /* Set cs time. */

      SET_PERI_REG_BITS(SPI_CTRL2_REG(spi_num), SPI_HOLD_TIME_V, 1,
                        SPI_HOLD_TIME_S);
      SET_PERI_REG_BITS(SPI_CTRL2_REG(spi_num), SPI_SETUP_TIME_V, 0,
                        SPI_SETUP_TIME_S);
    }
  else
    {
      modifyreg32(SPI_USER_REG(spi_num),
                          SPI_CS_HOLD_M | SPI_CS_SETUP_M, 0);
    }
}

/* spi param init for psram */

void IRAM_ATTR
psram_spi_init(psram_spi_num_t spi_num, int mode)
{
  modifyreg32(SPI_SLAVE_REG(spi_num), SPI_TRANS_DONE << 5, 0);

  /* SPI_CPOL & SPI_CPHA */

  modifyreg32(SPI_PIN_REG(spi_num), SPI_CK_IDLE_EDGE, 0);
  modifyreg32(SPI_USER_REG(spi_num), SPI_CK_OUT_EDGE, 0);

  /* SPI bit order */

  modifyreg32(SPI_CTRL_REG(spi_num), SPI_WR_BIT_ORDER, 0);
  modifyreg32(SPI_CTRL_REG(spi_num), SPI_RD_BIT_ORDER, 0);

  /* SPI bit order */

  modifyreg32(SPI_USER_REG(spi_num), SPI_DOUTDIN, 0);

  /* May be not must to do. */

  putreg32(0, SPI_USER1_REG(spi_num));

  /* SPI mode type */

  modifyreg32(SPI_SLAVE_REG(spi_num), SPI_SLAVE_MODE, 0);
  memset((void *)SPI_W0_REG(spi_num), 0, 16 * 4);
  psram_set_cs_timing(spi_num, s_clk_mode);
}

/* psram gpio init , different working frequency we have different
 * solutions
 */

static void IRAM_ATTR psram_gpio_config(psram_io_t *psram_io,
                                        int mode)
{
  int spi_cache_dummy = 0;
  uint32_t rd_mode_reg = getreg32(SPI_CTRL_REG(0));

  if (rd_mode_reg & SPI_FREAD_QIO_M)
    {
      spi_cache_dummy = SPI0_R_QIO_DUMMY_CYCLELEN;
    }
  else
    {
      if (rd_mode_reg & SPI_FREAD_DIO_M)
        {
          spi_cache_dummy = SPI0_R_DIO_DUMMY_CYCLELEN;
          SET_PERI_REG_BITS(SPI_USER1_REG(0), SPI_USR_ADDR_BITLEN_V,
                          SPI0_R_DIO_ADDR_BITSLEN, SPI_USR_ADDR_BITLEN_S);
        }
      else
        {
          if (rd_mode_reg & (SPI_FREAD_QUAD_M | SPI_FREAD_DUAL_M))
            {
              spi_cache_dummy = SPI0_R_FAST_DUMMY_CYCLELEN;
            }
          else
            {
              spi_cache_dummy = SPI0_R_FAST_DUMMY_CYCLELEN;
            }
        }
    }

  switch (mode)
    {
      case PSRAM_CACHE_F80M_S40M:
        extra_dummy = PSRAM_IO_MATRIX_DUMMY_40M;
        g_rom_spiflash_dummy_len_plus[_SPI_CACHE_PORT] =
                                     PSRAM_IO_MATRIX_DUMMY_80M;
        g_rom_spiflash_dummy_len_plus[_SPI_FLASH_PORT] =
                                     PSRAM_IO_MATRIX_DUMMY_40M;
        SET_PERI_REG_BITS(SPI_USER1_REG(_SPI_CACHE_PORT),
                          SPI_USR_DUMMY_CYCLELEN_V,
                          spi_cache_dummy + PSRAM_IO_MATRIX_DUMMY_80M,
                          SPI_USR_DUMMY_CYCLELEN_S);  /* DUMMY */
        esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_CACHE_PORT);
        esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_FLASH_PORT);
        break;

      case PSRAM_CACHE_F80M_S80M:
        extra_dummy = PSRAM_IO_MATRIX_DUMMY_80M;
        g_rom_spiflash_dummy_len_plus[_SPI_CACHE_PORT] =
                                     PSRAM_IO_MATRIX_DUMMY_80M;
        g_rom_spiflash_dummy_len_plus[_SPI_FLASH_PORT] =
                                     PSRAM_IO_MATRIX_DUMMY_80M;
        SET_PERI_REG_BITS(SPI_USER1_REG(_SPI_CACHE_PORT),
                          SPI_USR_DUMMY_CYCLELEN_V,
                          spi_cache_dummy + PSRAM_IO_MATRIX_DUMMY_80M,
                          SPI_USR_DUMMY_CYCLELEN_S);  /* DUMMY */
        esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_CACHE_PORT);
        esp_rom_spiflash_config_clk(_SPI_80M_CLK_DIV, _SPI_FLASH_PORT);
        break;

      case PSRAM_CACHE_F40M_S40M:
        extra_dummy = PSRAM_IO_MATRIX_DUMMY_40M;
        g_rom_spiflash_dummy_len_plus[_SPI_CACHE_PORT] =
                                     PSRAM_IO_MATRIX_DUMMY_40M;
        g_rom_spiflash_dummy_len_plus[_SPI_FLASH_PORT] =
                                     PSRAM_IO_MATRIX_DUMMY_40M;
        SET_PERI_REG_BITS(SPI_USER1_REG(_SPI_CACHE_PORT),
                          SPI_USR_DUMMY_CYCLELEN_V,
                          spi_cache_dummy + PSRAM_IO_MATRIX_DUMMY_40M,
                          SPI_USR_DUMMY_CYCLELEN_S);  /* DUMMY */
        esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_CACHE_PORT);
        esp_rom_spiflash_config_clk(_SPI_40M_CLK_DIV, _SPI_FLASH_PORT);

        break;

      default:
        break;
    }

  SET_PERI_REG_MASK(SPI_USER_REG(0), SPI_USR_DUMMY); /* dummy enable */

  /* In bootloader, all the signals are already configured */

  /* We keep the following code in case the bootloader is some older
   * version.
   */

  esp32_gpio_matrix_out(psram_io->flash_cs_io, SPICS0_OUT_IDX, 0, 0);
  esp32_gpio_matrix_out(psram_io->psram_cs_io, SPICS1_OUT_IDX, 0, 0);
  esp32_gpio_matrix_out(psram_io->psram_spiq_sd0_io, SPIQ_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(psram_io->psram_spiq_sd0_io, SPIQ_IN_IDX, 0);
  esp32_gpio_matrix_out(psram_io->psram_spid_sd1_io, SPID_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(psram_io->psram_spid_sd1_io, SPID_IN_IDX, 0);
  esp32_gpio_matrix_out(psram_io->psram_spiwp_sd3_io, SPIWP_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(psram_io->psram_spiwp_sd3_io, SPIWP_IN_IDX, 0);
  esp32_gpio_matrix_out(psram_io->psram_spihd_sd2_io, SPIHD_OUT_IDX, 0, 0);
  esp32_gpio_matrix_in(psram_io->psram_spihd_sd2_io, SPIHD_IN_IDX, 0);

  /* select pin function gpio */

  if ((psram_io->flash_clk_io == SPI_IOMUX_PIN_NUM_CLK) &&
      (psram_io->flash_clk_io != psram_io->psram_clk_io))
    {
      /* flash clock signal should come from IO MUX. */

      esp32_configgpio(psram_io->flash_clk_io, OUTPUT_FUNCTION_2);
    }
  else
    {
      /* flash clock signal should come from GPIO matrix. */

      esp32_configgpio(psram_io->flash_clk_io, OUTPUT_FUNCTION_3);
    }

  esp32_configgpio(psram_io->flash_cs_io, OUTPUT | FUNCTION_3);
  esp32_configgpio(psram_io->psram_cs_io, OUTPUT | FUNCTION_3);
  esp32_configgpio(psram_io->psram_clk_io, OUTPUT | FUNCTION_3);
  esp32_configgpio(psram_io->psram_spiq_sd0_io, OUTPUT | INPUT | FUNCTION_3);
  esp32_configgpio(psram_io->psram_spid_sd1_io, OUTPUT | INPUT | FUNCTION_3);
  esp32_configgpio(psram_io->psram_spihd_sd2_io,
                   OUTPUT | INPUT | FUNCTION_3);
  esp32_configgpio(psram_io->psram_spiwp_sd3_io,
                   OUTPUT | INPUT | FUNCTION_3);

#if 0
  uint32_t flash_id = g_rom_flashchip.device_id;
  if (flash_id == FLASH_ID_GD25LQ32C)
    {
      /* Set drive ability for 1.8v flash in 80Mhz. */

      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->flash_cs_io], FUN_DRV_V,
                        3, FUN_DRV_S);
      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->flash_clk_io], FUN_DRV_V,
                        3, FUN_DRV_S);
      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->psram_cs_io], FUN_DRV_V,
                        3, FUN_DRV_S);
      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->psram_clk_io], FUN_DRV_V,
                        3, FUN_DRV_S);
      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->psram_spiq_sd0_io],
                        FUN_DRV_V, 3, FUN_DRV_S);
      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->psram_spid_sd1_io],
                        FUN_DRV_V, 3, FUN_DRV_S);
      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->psram_spihd_sd2_io],
                        FUN_DRV_V, 3, FUN_DRV_S);
      SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[psram_io->psram_spiwp_sd3_io],
                        FUN_DRV_V, 3, FUN_DRV_S);
    }
#endif
}

int psram_get_size(void)
{
  if ((PSRAM_SIZE_ID(s_psram_id) == PSRAM_EID_SIZE_64MBITS) ||
      PSRAM_IS_64MBIT_TRIAL(s_psram_id))
    {
      return s_2t_mode_enabled ? PSRAM_SIZE_32MBITS : PSRAM_SIZE_64MBITS;
    }
  else
    {
      if (PSRAM_SIZE_ID(s_psram_id) == PSRAM_EID_SIZE_32MBITS)
        {
          return PSRAM_SIZE_32MBITS;
        }
      else
        {
          if (PSRAM_SIZE_ID(s_psram_id) == PSRAM_EID_SIZE_16MBITS)
            {
              return PSRAM_SIZE_16MBITS;
            }
          else
            {
              return PSRAM_SIZE_MAX;
            }
        }
    }
}

/* used in UT only */

bool psram_is_32mbit_ver0(void)
{
  return PSRAM_IS_32MBIT_VER0(s_psram_id);
}

/* PSRAM mode init will overwrite original flash speed mode, so that it is
 * possible to change psram and flash speed after OTA.
 * Flash read mode(QIO/QOUT/DIO/DOUT) will not be changed in app bin.
 * It is decided by bootloader, OTA can not change this mode.
 */

int IRAM_ATTR
psram_enable(int mode, int vaddrmode)   /* psram init */
{
  struct rtc_vddsdio_config_s cfg;

  /* Let's assume we are not worring about OTA issue and ignore for now */

  psram_io_t psram_io =
                        {
                          0
                        };

  uint32_t chip_ver = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG,
                                    EFUSE_RD_CHIP_VER_PKG);
  uint32_t pkg_ver = chip_ver & 0x7;
  uint32_t spiconfig;

  if (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32D2WDQ5)
    {
      minfo("This chip is ESP32-D2WD\n");

      esp32_get_vddsdio_config(&cfg);
      if (cfg.tieh != RTC_VDDSDIO_TIEH_1_8V)
        {
          merr("VDDSDIO is not 1.8V");
          return -EFAULT;
        }

      psram_io.psram_clk_io = CONFIG_D2WD_PSRAM_CLK_IO;
      psram_io.psram_cs_io  = CONFIG_D2WD_PSRAM_CS_IO;
    }
  else
    {
      if ((pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOD2) ||
          (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4))
        {
          minfo("This chip is ESP32-PICO");

          esp32_get_vddsdio_config(&cfg);
          if (cfg.tieh != RTC_VDDSDIO_TIEH_3_3V)
            {
              merr("VDDSDIO is not 3.3V");
              return -EFAULT;
            }

          s_clk_mode = PSRAM_CLK_MODE_NORM;
          psram_io.psram_clk_io = PICO_PSRAM_CLK_IO;
          psram_io.psram_cs_io  = CONFIG_PICO_PSRAM_CS_IO;
        }
      else
        {
          if ((pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ6) ||
              (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ5))
            {
              minfo("This chip is ESP32-D0WD\n");
              psram_io.psram_clk_io = CONFIG_D0WD_PSRAM_CLK_IO;
              psram_io.psram_cs_io  = CONFIG_D0WD_PSRAM_CS_IO;
            }
          else
            {
              merr("Not a valid or known package id: %d", pkg_ver);
              PANIC();
            }
        }
    }

  spiconfig = ets_efuse_get_spiconfig();
  if (spiconfig == EFUSE_SPICFG_SPI_DEFAULTS)
    {
      minfo("SPI Defaults\n");
      psram_io.flash_clk_io       = SPI_IOMUX_PIN_NUM_CLK;
      psram_io.flash_cs_io        = SPI_IOMUX_PIN_NUM_CS;
      psram_io.psram_spiq_sd0_io  = PSRAM_SPIQ_SD0_IO;
      psram_io.psram_spid_sd1_io  = PSRAM_SPID_SD1_IO;
      psram_io.psram_spiwp_sd3_io = PSRAM_SPIWP_SD3_IO;
      psram_io.psram_spihd_sd2_io = PSRAM_SPIHD_SD2_IO;
    }
  else
    {
      if (spiconfig == EFUSE_SPICFG_HSPI_DEFAULTS)
        {
          minfo("HSPI Defaults\n");
          psram_io.flash_clk_io       = FLASH_HSPI_CLK_IO;
          psram_io.flash_cs_io        = FLASH_HSPI_CS_IO;
          psram_io.psram_spiq_sd0_io  = PSRAM_HSPI_SPIQ_SD0_IO;
          psram_io.psram_spid_sd1_io  = PSRAM_HSPI_SPID_SD1_IO;
          psram_io.psram_spiwp_sd3_io = PSRAM_HSPI_SPIWP_SD3_IO;
          psram_io.psram_spihd_sd2_io = PSRAM_HSPI_SPIHD_SD2_IO;
        }
      else
        {
          minfo("SPI EFUSE Config\n");
          psram_io.flash_clk_io       = EFUSE_SPICFG_RET_SPICLK(spiconfig);
          psram_io.flash_cs_io        = EFUSE_SPICFG_RET_SPICS0(spiconfig);
          psram_io.psram_spiq_sd0_io  = EFUSE_SPICFG_RET_SPIQ(spiconfig);
          psram_io.psram_spid_sd1_io  = EFUSE_SPICFG_RET_SPID(spiconfig);
          psram_io.psram_spihd_sd2_io = EFUSE_SPICFG_RET_SPIHD(spiconfig);

          /* If flash mode is set to QIO or QOUT, the WP pin is equal the
           * value configured in bootloader. If flash mode is set to DIO or
           * DOUT, the WP pin should config it via menuconfig.
           */

          /* Since the default value is the same from the bootloader
           * Let's use it for now.
           */

          psram_io.psram_spiwp_sd3_io = CONFIG_ESP32_SPIRAM_SPIWP_SD3_PIN;
        }
    }

  assert(mode < PSRAM_CACHE_MAX && "we don't support any other mode.");
  s_psram_mode = mode;

  putreg32(0x1, SPI_EXT3_REG(0));
  modifyreg32(SPI_USER_REG(PSRAM_SPI_1), SPI_USR_PREP_HOLD_M, 0);

  psram_spi_init(PSRAM_SPI_1, mode);

  switch (mode)
    {
      case PSRAM_CACHE_F80M_S80M:
        esp32_gpio_matrix_out(psram_io.psram_clk_io, SPICLK_OUT_IDX, 0, 0);
        break;
      case PSRAM_CACHE_F80M_S40M:
      case PSRAM_CACHE_F40M_S40M:
      default:
        if (s_clk_mode == PSRAM_CLK_MODE_DCLK)
          {
            /* We need to delay CLK to the PSRAM with respect to the clock
             * signal as output by the SPI peripheral. We do this by routing
             * it signal to signal 224/225, which are used as a loopback; the
             * extra run through the GPIO matrix causes the delay. We use
             * GPIO20 (which is not in any package but has pad logic in
             * silicon) as a temporary pad for this. So the signal path is:
             * SPI CLK --> GPIO28 --> signal224(in then out) -->
             * internal GPIO29 --> signal225(in then out) -->
             * GPIO17(PSRAM CLK)
             */

            minfo("clk_mode == PSRAM_CLK_MODE_DCLK\n");
            esp32_gpio_matrix_out(PSRAM_INTERNAL_IO_28,
                                  SPICLK_OUT_IDX, 0, 0);
            esp32_gpio_matrix_in(PSRAM_INTERNAL_IO_28,
                                 SIG_IN_FUNC224_IDX, 0);
            esp32_gpio_matrix_out(PSRAM_INTERNAL_IO_29,
                                  SIG_IN_FUNC224_IDX, 0, 0);
            esp32_gpio_matrix_in(PSRAM_INTERNAL_IO_29,
                                 SIG_IN_FUNC225_IDX, 0);
            esp32_gpio_matrix_out(psram_io.psram_clk_io,
                                  SIG_IN_FUNC225_IDX, 0, 0);
          }
        else
          {
            minfo("clk_io == OUT_IDX\n");
            esp32_gpio_matrix_out(psram_io.psram_clk_io,
                                  SPICLK_OUT_IDX, 0, 0);
          }
        break;
    }

  /* Rise VDDSIO for 1.8V psram. */

  /* bootloader_common_vddsdio_configure(); */

  /* GPIO related settings */

  psram_gpio_config(&psram_io, mode);
  psram_read_id(&s_psram_id);

  minfo("psram ID = 0x%x\n", (uint32_t)s_psram_id);

  if (!PSRAM_IS_VALID(s_psram_id))
    {
      minfo("PSRAM ID NOT VALID\n");
      return -ENODEV;
    }

  if (psram_is_32mbit_ver0())
    {
      s_clk_mode = PSRAM_CLK_MODE_DCLK;
      if (mode == PSRAM_CACHE_F80M_S80M)
        {
#ifdef CONFIG_ESP32_SPIRAM_OCCUPY_NO_HOST
          merr("This version of PSRAM needs to claim an extra SPI"
               "peripheral at 80MHz. Please either: choose lower"
               "frequency by SPIRAM_SPEED_, or select one SPI peripheral"
               "it by SPIRAM_OCCUPY_*SPI_HOST in the menuconfig.");
          abort();
#else
          /* note: If the third mode(80Mhz+80Mhz) is enabled for 32MBit 1V8
           * psram, one of HSPI/VSPI port will be occupied by the system
           * (according to kconfig). Application code should never touch
           * HSPI/VSPI hardware in this case.  We try to stop applications
           * from doing this using the drivers by claiming the port for
           * ourselves
           */

          esp32_gpio_matrix_out(psram_io.psram_clk_io,
                                PSRAM_CLK_SIGNAL, 0, 0);

          /* use spi3 clock,but use spi1 data/cs wires
           * We get a solid 80MHz clock from SPI3 by setting it up, starting
           * a transaction, waiting until it is in progress, then cutting the
           * clock (but not the reset!) to that peripheral.
           */

          putreg32(32 << 24, SPI_ADDR_REG(PSRAM_SPI_NUM));
          modifyreg32(SPI_CMD_REG(PSRAM_SPI_NUM), 0, SPI_FLASH_READ_M);
          uint32_t spi_status;
          while (1)
            {
              spi_status = getreg32(SPI_EXT2_REG(PSRAM_SPI_NUM));
              if (spi_status != 0 && spi_status != 1)
                {
                  modifyreg32(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN_1, 0);
                  break;
                }
            }
#endif
        }
    }
  else
    {
      /* For other psram, we don't need any extra clock cycles after cs get
       * back to high level
       */

      s_clk_mode = PSRAM_CLK_MODE_NORM;
      esp32_gpio_matrix_out(PSRAM_INTERNAL_IO_28, SIG_GPIO_OUT_IDX, 0, 0);
      esp32_gpio_matrix_out(PSRAM_INTERNAL_IO_29, SIG_GPIO_OUT_IDX, 0, 0);
      esp32_gpio_matrix_out(psram_io.psram_clk_io, SPICLK_OUT_IDX, 0, 0);
    }

  /* Update cs timing according to psram driving method. */

  psram_set_cs_timing(PSRAM_SPI_1, s_clk_mode);
  psram_set_cs_timing(_SPI_CACHE_PORT, s_clk_mode);
  psram_enable_qio_mode(PSRAM_SPI_1);

  if (((PSRAM_SIZE_ID(s_psram_id) == PSRAM_EID_SIZE_64MBITS) ||
        PSRAM_IS_64MBIT_TRIAL(s_psram_id)))
    {
#if defined(CONFIG_ESP32_SPIRAM_2T_MODE)
#  if CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE
      merr("PSRAM 2T mode and SPIRAM bank switching can not enabled\
            meanwhile. Please read the help text for SPIRAM_2T_MODE in the\
            project configuration menu.");
      abort();
#  endif
      /* Note: 2T mode command should not be sent twice,
       * otherwise psram would get back to normal mode.
       */

      if (psram_2t_mode_check(PSRAM_SPI_1) != OK)
        {
          psram_2t_mode_enable(PSRAM_SPI_1);
          if (psram_2t_mode_check(PSRAM_SPI_1) != OK)
            {
              merr("PSRAM 2T mode enable fail!\n");
              return -EINVAL;
            }
        }

      s_2t_mode_enabled = true;
      minfo("PSRAM is in 2T mode");
#endif
    }

  psram_cache_init(mode, vaddrmode);

  return OK;
}

#endif /* CONFIG_ESP32_SPIRAM */
