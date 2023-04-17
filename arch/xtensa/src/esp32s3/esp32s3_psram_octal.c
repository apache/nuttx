/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_psram_octal.c
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
#include <stddef.h>
#include <errno.h>
#include <debug.h>

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

#define OPI_PSRAM_SYNC_READ             0x0000
#define OPI_PSRAM_SYNC_WRITE            0x8080
#define OPI_PSRAM_REG_READ              0x4040
#define OPI_PSRAM_REG_WRITE             0xC0C0
#define OCT_PSRAM_RD_CMD_BITLEN         16
#define OCT_PSRAM_WR_CMD_BITLEN         16
#define OCT_PSRAM_ADDR_BITLEN           32
#define OCT_PSRAM_RD_DUMMY_BITLEN       (2 * (10 - 1))
#define OCT_PSRAM_WR_DUMMY_BITLEN       (2 * (5 - 1))

#define OCT_PSRAM_CS_SETUP_TIME         3
#define OCT_PSRAM_CS_HOLD_TIME          3
#define OCT_PSRAM_CS_HOLD_DELAY         2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct opi_psram_reg
{
    union mr0_u
      {
        struct
          {
            uint8_t drive_str: 2;
            uint8_t read_latency: 3;
            uint8_t lt: 1;
            uint8_t rsvd0_1: 2;
          };

        uint8_t val;
      }
    mr0;

    union mr1_u
      {
        struct
          {
            uint8_t vendor_id: 5;
            uint8_t rsvd0_2: 3;
          };

        uint8_t val;
      }
    mr1;

    union mr2_u
      {
        struct
          {
            uint8_t density: 3;
            uint8_t dev_id: 2;
            uint8_t rsvd1_2: 2;
            uint8_t gb: 1;
          };

        uint8_t val;
      }
    mr2;

    union mr3_u
      {
        struct
          {
            uint8_t rsvd3_7: 5;
            uint8_t srf: 1;
            uint8_t vcc: 1;
            uint8_t rsvd0: 1;
          };

        uint8_t val;
      }
    mr3;

    union mr4_u
      {
        struct
          {
            uint8_t pasr: 3;
            uint8_t rf: 1;
            uint8_t rsvd3: 1;
            uint8_t wr_latency: 3;
          };

        uint8_t val;
      }
    mr4;

    union mr8_u
      {
        struct
          {
            uint8_t bl: 2;
            uint8_t bt: 1;
            uint8_t rsvd0_4: 5;
          };

        uint8_t val;
      }
    mr8;
};

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern void cache_resume_dcache(uint32_t val);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static size_t g_psram_size;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: set_psram_reg
 *
 * Description:
 *   Set PSRAM registers' value
 *
 * Input Parameters:
 *   spi_num - SPI port
 *   in_reg - PSRAM registers' value
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_psram_reg(int spi_num, const struct opi_psram_reg *in_reg)
{
  esp_rom_spiflash_read_mode_t mode = ESP_ROM_SPIFLASH_OPI_DTR_MODE;
  int cmd_len = 16;
  uint32_t addr = 0x0;
  int addr_bit_len = 32;
  int dummy = OCT_PSRAM_RD_DUMMY_BITLEN;
  int data_bit_len = 16;
  struct opi_psram_reg psram_reg =
    {
      0
    };

  esp_rom_opiflash_exec_cmd(spi_num, mode,
                            OPI_PSRAM_REG_READ,
                            cmd_len,
                            addr,
                            addr_bit_len,
                            dummy,
                            NULL, 0,
                            &psram_reg.mr0.val,
                            data_bit_len,
                            BIT(1),
                            false);

  psram_reg.mr0.lt = in_reg->mr0.lt;
  psram_reg.mr0.read_latency = in_reg->mr0.read_latency;
  psram_reg.mr0.drive_str = in_reg->mr0.drive_str;

  esp_rom_opiflash_exec_cmd(spi_num, mode,
                            OPI_PSRAM_REG_WRITE,
                            cmd_len,
                            addr,
                            addr_bit_len,
                            0,
                            &psram_reg.mr0.val,
                            16,
                            NULL, 0,
                            BIT(1),
                            false);
}

/****************************************************************************
 * Name: get_psram_reg
 *
 * Description:
 *   Get PSRAM registers' value
 *
 * Input Parameters:
 *   spi_num - SPI port
 *   out_reg - PSRAM registers' value
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void get_psram_reg(int spi_num, struct opi_psram_reg *out_reg)
{
  esp_rom_spiflash_read_mode_t mode = ESP_ROM_SPIFLASH_OPI_DTR_MODE;
  int cmd_len = 16;
  int addr_bit_len = 32;
  int dummy = OCT_PSRAM_RD_DUMMY_BITLEN;
  int data_bit_len = 16;

  /** Read MR0~1 register */

  esp_rom_opiflash_exec_cmd(spi_num, mode,
                            OPI_PSRAM_REG_READ,
                            cmd_len,
                            0x0,
                            addr_bit_len,
                            dummy,
                            NULL, 0,
                            &out_reg->mr0.val,
                            data_bit_len,
                            BIT(1),
                            false);

  /** Read MR2~3 register */

  esp_rom_opiflash_exec_cmd(spi_num, mode,
                          OPI_PSRAM_REG_READ,
                          cmd_len,
                          0x2,
                          addr_bit_len,
                          dummy,
                          NULL, 0,
                          &out_reg->mr2.val,
                          data_bit_len,
                          BIT(1),
                          false);

  /** Read MR4 register */

  data_bit_len = 8;
  esp_rom_opiflash_exec_cmd(spi_num, mode,
                          OPI_PSRAM_REG_READ,
                          cmd_len,
                          0x4,
                          addr_bit_len,
                          dummy,
                          NULL, 0,
                          &out_reg->mr4.val,
                          data_bit_len,
                          BIT(1),
                          false);

  /** Read MR8 register */

  esp_rom_opiflash_exec_cmd(spi_num, mode,
                          OPI_PSRAM_REG_READ,
                          cmd_len,
                          0x8,
                          addr_bit_len,
                          dummy,
                          NULL, 0,
                          &out_reg->mr8.val,
                          data_bit_len,
                          BIT(1),
                          false);
}

/****************************************************************************
 * Name: print_psram_reg
 *
 * Description:
 *   Print PSRAM information.
 *
 * Input Parameters:
 *   psram_reg - PSRAM registers' value
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void print_psram_reg(const struct opi_psram_reg *psram_reg)
{
  minfo("vendor id : 0x%02x (%s)\n", psram_reg->mr1.vendor_id,
        psram_reg->mr1.vendor_id == 0x0d ? "AP" : "UNKNOWN");
  minfo("dev id    : 0x%02x (generation %d)\n", psram_reg->mr2.dev_id,
        psram_reg->mr2.dev_id + 1);
  minfo("density   : 0x%02x (%d Mbit)\n", psram_reg->mr2.density,
        psram_reg->mr2.density == 0x1 ? 32 :
        psram_reg->mr2.density == 0X3 ? 64 :
        psram_reg->mr2.density == 0x5 ? 128 :
        psram_reg->mr2.density == 0x7 ? 256 : 0);
  minfo("good-die  : 0x%02x (%s)\n", psram_reg->mr2.gb,
        psram_reg->mr2.gb == 1 ? "Pass" : "Fail");
  minfo("Latency   : 0x%02x (%s)\n", psram_reg->mr0.lt,
        psram_reg->mr0.lt == 1 ? "Fixed" : "Variable");
  minfo("VCC       : 0x%02x (%s)\n", psram_reg->mr3.vcc,
        psram_reg->mr3.vcc == 1 ? "3V" : "1.8V");
  minfo("SRF       : 0x%02x (%s Refresh)\n", psram_reg->mr3.srf,
        psram_reg->mr3.srf == 0x1 ? "Fast" : "Slow");
  minfo("BurstType : 0x%02x (%s Wrap)\n", psram_reg->mr8.bt,
        psram_reg->mr8.bt == 1 && psram_reg->mr8.bl != 3 ? "Hybrid" : "");
  minfo("BurstLen  : 0x%02x (%d Byte)\n", psram_reg->mr8.bl,
        psram_reg->mr8.bl == 0x00 ? 16 :
        psram_reg->mr8.bl == 0x01 ? 32 :
        psram_reg->mr8.bl == 0x10 ? 64 : 1024);
  minfo("Readlatency  : 0x%02x (%d cycles@%s)\n",
        psram_reg->mr0.read_latency,
        psram_reg->mr0.read_latency * 2 + 6,
        psram_reg->mr0.lt == 1 ? "Fixed" : "Variable");
  minfo("DriveStrength: 0x%02x (1/%d)\n", psram_reg->mr0.drive_str,
        psram_reg->mr0.drive_str == 0x00 ? 1 :
        psram_reg->mr0.drive_str == 0x01 ? 2 :
        psram_reg->mr0.drive_str == 0x02 ? 4 : 8);
}

/****************************************************************************
 * Name: init_cs_timing
 *
 * Description:
 *   Initialize SPI CS timing.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void init_cs_timing(void)
{
  /**
   * SPI0/1 share the cs_hold / cs_setup, cd_hold_time / cd_setup_time,
   * cs_hold_delay registers for PSRAM, so we only need to set SPI0
   * related registers here.
   */

  SET_PERI_REG_MASK(SPI_MEM_SPI_SMEM_AC_REG(0),
                    SPI_MEM_SPI_SMEM_CS_HOLD_M |
                    SPI_MEM_SPI_SMEM_CS_SETUP_M);
  SET_PERI_REG_BITS(SPI_MEM_SPI_SMEM_AC_REG(0),
                    SPI_MEM_SPI_SMEM_CS_HOLD_TIME_V,
                    OCT_PSRAM_CS_HOLD_TIME,
                    SPI_MEM_SPI_SMEM_CS_HOLD_TIME_S);
  SET_PERI_REG_BITS(SPI_MEM_SPI_SMEM_AC_REG(0),
                    SPI_MEM_SPI_SMEM_CS_SETUP_TIME_V,
                    OCT_PSRAM_CS_SETUP_TIME,
                    SPI_MEM_SPI_SMEM_CS_SETUP_TIME_S);

  /** CS1 high time */

  SET_PERI_REG_BITS(SPI_MEM_SPI_SMEM_AC_REG(0),
                    SPI_MEM_SPI_SMEM_CS_HOLD_DELAY_V,
                    OCT_PSRAM_CS_HOLD_DELAY,
                    SPI_MEM_SPI_SMEM_CS_HOLD_DELAY_S);
}

/****************************************************************************
 * Name: init_psram_pins
 *
 * Description:
 *   Initialize PSRAM pins.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void init_psram_pins(void)
{
  uint32_t reg = REG_IO_MUX_BASE +
                 (CONFIG_ESP32S3_DEFAULT_PSRAM_CS_IO + 1) * 4;

  PIN_FUNC_SELECT(reg,  FUNC_SPICS1_SPICS1);
  PIN_SET_DRV(reg, 3);
  REG_SET_FIELD(SPI_MEM_DATE_REG(0), SPI_MEM_SPI_SMEM_SPICLK_FUN_DRV, 3);
}

/****************************************************************************
 * Name: config_psram_spi_phases
 *
 * Description:
 *   Configure PSRAM SPI0 phase related registers here according to
 *   the PSRAM chip requirement.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void config_psram_spi_phases(void)
{
  /** Config Write CMD phase for SPI0 to access PSRAM */

  SET_PERI_REG_MASK(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_CACHE_SRAM_USR_WCMD_M);
  SET_PERI_REG_BITS(SPI_MEM_SRAM_DWR_CMD_REG(0),
                    SPI_MEM_CACHE_SRAM_USR_WR_CMD_BITLEN,
                    OCT_PSRAM_WR_CMD_BITLEN - 1,
                    SPI_MEM_CACHE_SRAM_USR_WR_CMD_BITLEN_S);
  SET_PERI_REG_BITS(SPI_MEM_SRAM_DWR_CMD_REG(0),
                    SPI_MEM_CACHE_SRAM_USR_WR_CMD_VALUE,
                    OPI_PSRAM_SYNC_WRITE,
                    SPI_MEM_CACHE_SRAM_USR_WR_CMD_VALUE_S);

  /** Config Read CMD phase for SPI0 to access PSRAM */

  SET_PERI_REG_MASK(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_CACHE_SRAM_USR_RCMD_M);
  SET_PERI_REG_BITS(SPI_MEM_SRAM_DRD_CMD_REG(0),
                    SPI_MEM_CACHE_SRAM_USR_RD_CMD_BITLEN_V,
                    OCT_PSRAM_RD_CMD_BITLEN - 1,
                    SPI_MEM_CACHE_SRAM_USR_RD_CMD_BITLEN_S);
  SET_PERI_REG_BITS(SPI_MEM_SRAM_DRD_CMD_REG(0),
                    SPI_MEM_CACHE_SRAM_USR_RD_CMD_VALUE_V,
                    OPI_PSRAM_SYNC_READ,
                    SPI_MEM_CACHE_SRAM_USR_RD_CMD_VALUE_S);

  /** Config ADDR phase */

  SET_PERI_REG_BITS(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_SRAM_ADDR_BITLEN_V,
                    OCT_PSRAM_ADDR_BITLEN - 1,
                    SPI_MEM_SRAM_ADDR_BITLEN_S);
  SET_PERI_REG_MASK(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_CACHE_USR_SCMD_4BYTE_M);

  /** Config RD/WR Dummy phase */

  SET_PERI_REG_MASK(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_USR_RD_SRAM_DUMMY_M |
                    SPI_MEM_USR_WR_SRAM_DUMMY_M);
  SET_PERI_REG_BITS(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_SRAM_RDUMMY_CYCLELEN_V,
                    OCT_PSRAM_RD_DUMMY_BITLEN - 1,
                    SPI_MEM_SRAM_RDUMMY_CYCLELEN_S);
  SET_PERI_REG_MASK(SPI_MEM_SPI_SMEM_DDR_REG(0),
                    SPI_MEM_SPI_SMEM_VAR_DUMMY_M);
  SET_PERI_REG_BITS(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_SRAM_WDUMMY_CYCLELEN_V,
                    OCT_PSRAM_WR_DUMMY_BITLEN - 1,
                    SPI_MEM_SRAM_WDUMMY_CYCLELEN_S);

  CLEAR_PERI_REG_MASK(SPI_MEM_SPI_SMEM_DDR_REG(0),
                      SPI_MEM_SPI_SMEM_DDR_WDAT_SWP_M |
                      SPI_MEM_SPI_SMEM_DDR_RDAT_SWP_M);
  SET_PERI_REG_MASK(SPI_MEM_SPI_SMEM_DDR_REG(0),
                    SPI_MEM_SPI_SMEM_DDR_EN_M);

  SET_PERI_REG_MASK(SPI_MEM_SRAM_CMD_REG(0),
                    SPI_MEM_SDUMMY_OUT_M |
                    SPI_MEM_SCMD_OCT_M |
                    SPI_MEM_SADDR_OCT_M |
                    SPI_MEM_SDOUT_OCT_M |
                    SPI_MEM_SDIN_OCT_M);
  SET_PERI_REG_MASK(SPI_MEM_CACHE_SCTRL_REG(0),
                    SPI_MEM_SRAM_OCT_M);

  cache_resume_dcache(0);
}

/****************************************************************************
 * Name: spi_flash_set_rom_required_regs
 *
 * Description:
 *   Set flash ROM required register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void spi_flash_set_rom_required_regs(void)
{
#ifdef CONFIG_ESP32S3_FLASH_MODE_OCT
  /**
   * Disable the variable dummy mode when doing timing tuning.
   *
   * STR /DTR mode setting is done every time when
   * "esp_rom_opiflash_exec_cmd" is called.
   *
   * Add any registers that are not set in ROM SPI flash functions here
   * in the future.
   */

  CLEAR_PERI_REG_MASK(SPI_MEM_DDR_REG(1), SPI_MEM_SPI_FMEM_VAR_DUMMY);
#endif
}

/****************************************************************************
 * Name: flash_set_vendor_required_regs
 *
 * Description:
 *   Set flash vendor required register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void flash_set_vendor_required_regs(void)
{
#ifdef CONFIG_ESP32S3_FLASH_MODE_OCT
  /**
   * Set MSPI specifical configuration,
   * "esp32s3_bsp_opiflash_set_required_regs" is board defined function.
   */

  esp32s3_bsp_opiflash_set_required_regs();

  SET_PERI_REG_BITS(SPI_MEM_CACHE_FCTRL_REG(1),
                    SPI_MEM_CACHE_USR_CMD_4BYTE_V,
                    1,
                    SPI_MEM_CACHE_USR_CMD_4BYTE_S);
#else
  /** Restore MSPI registers after Octal PSRAM initialization. */

  SET_PERI_REG_BITS(SPI_MEM_CACHE_FCTRL_REG(1),
                    SPI_MEM_CACHE_USR_CMD_4BYTE_V,
                    0,
                    SPI_MEM_CACHE_USR_CMD_4BYTE_S);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int IRAM_ATTR psram_enable(int mode, int vaddrmode)
{
  struct opi_psram_reg psram_reg =
    {
      0
    };

  init_psram_pins();
  init_cs_timing();

  /** enter MSPI slow mode to init PSRAM device registers */

  esp32s3_spi_timing_set_mspi_low_speed(true);

  /** set to variable dummy mode */

  SET_PERI_REG_MASK(SPI_MEM_DDR_REG(1), SPI_MEM_SPI_FMEM_VAR_DUMMY);
  esp_rom_spi_set_dtr_swap_mode(1, false, false);

  /** Set PSRAM read latency and drive strength */

  psram_reg.mr0.lt = 1;
  psram_reg.mr0.read_latency = 2;
  psram_reg.mr0.drive_str = 0;
  set_psram_reg(1, &psram_reg);
  get_psram_reg(1, &psram_reg);
  print_psram_reg(&psram_reg);

  g_psram_size = psram_reg.mr2.density == 0x1 ? PSRAM_SIZE_4MB  :
                 psram_reg.mr2.density == 0X3 ? PSRAM_SIZE_8MB  :
                 psram_reg.mr2.density == 0x5 ? PSRAM_SIZE_16MB :
                 psram_reg.mr2.density == 0x7 ? PSRAM_SIZE_32MB : 0;

  /* Back to the high speed mode. Flash/PSRAM clocks are set to the clock
   * that user selected. SPI0/1 registers are all set correctly.
   */

  esp32s3_spi_timing_set_mspi_high_speed(true);

  /**
   * Tuning may change SPI1 regs, whereas legacy spi_flash APIs rely on
   * these regs. This function is to restore SPI1 init state.
   */

  spi_flash_set_rom_required_regs();

  /**
   * Flash chip requires MSPI specifically, call this function to set them
   */

  flash_set_vendor_required_regs();

  config_psram_spi_phases();

  return 0;
}

int psram_get_physical_size(uint32_t *out_size_bytes)
{
  *out_size_bytes = g_psram_size;
  return g_psram_size > 0 ? OK : -EINVAL;
}

/* This function is to get the available physical psram size in bytes.
 */

int psram_get_available_size(uint32_t *out_size_bytes)
{
  *out_size_bytes = g_psram_size;
  return (g_psram_size ? OK : -EINVAL);
}
