/*****************************************************************************
 * arch/xtensa/src/esp32s2/rom/esp32s2_opi_flash.h
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
 *****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ROM_ESP32S2_OPI_FLASH_H
#define __ARCH_XTENSA_SRC_ESP32S2_ROM_ESP32S2_OPI_FLASH_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
  uint16_t cmd;                /* !< Command value */
  uint16_t cmd_bit_len;        /* !< Command byte length */
  uint32_t *addr;              /* !< Point to address value */
  uint32_t addr_bit_len;       /* !< Address byte length */
  uint32_t *tx_data;           /* !< Point to send data buffer */
  uint32_t tx_data_bit_len;    /* !< Send data byte length. */
  uint32_t *rx_data;           /* !< Point to recevie data buffer */
  uint32_t rx_data_bit_len;    /* !< Recevie Data byte length. */
  uint32_t dummy_bit_len;
} esp_rom_spi_cmd_t;

#define ESP_ROM_OPIFLASH_MUX_TAKE()
#define ESP_ROM_OPIFLASH_MUX_GIVE()
#define ESP_ROM_OPIFLASH_SEL_CS0     (BIT(0))
#define ESP_ROM_OPIFLASH_SEL_CS1     (BIT(1))

/* Definition of MX25UM25645G Octa Flash
 * SPI status register
 */

#define ESP_ROM_SPIFLASH_BUSY_FLAG     BIT0
#define ESP_ROM_SPIFLASH_WRENABLE_FLAG BIT1
#define ESP_ROM_SPIFLASH_BP0           BIT2
#define ESP_ROM_SPIFLASH_BP1           BIT3
#define ESP_ROM_SPIFLASH_BP2           BIT4
#define ESP_ROM_SPIFLASH_QE            BIT9

#define FLASH_OP_MODE_RDCMD_DOUT       0x3B
#define ESP_ROM_FLASH_SECTOR_SIZE      0x1000
#define ESP_ROM_FLASH_BLOCK_SIZE_64K   0x10000
#define ESP_ROM_FLASH_PAGE_SIZE        256

/* FLASH commands */

#define ROM_FLASH_CMD_RDID             0x9F
#define ROM_FLASH_CMD_WRSR             0x01
#define ROM_FLASH_CMD_WRSR2            0x31 /* Not all SPI flash uses this command */
#define ROM_FLASH_CMD_WREN             0x06
#define ROM_FLASH_CMD_WRDI             0x04
#define ROM_FLASH_CMD_RDSR             0x05
#define ROM_FLASH_CMD_RDSR2            0x35 /* Not all SPI flash uses this command */
#define ROM_FLASH_CMD_ERASE_SEC        0x20
#define ROM_FLASH_CMD_ERASE_BLK_32K    0x52
#define ROM_FLASH_CMD_ERASE_BLK_64K    0xD8
#define ROM_FLASH_CMD_OTPEN            0x3A /* Enable OTP mode, not all SPI flash uses this command */
#define ROM_FLASH_CMD_RSTEN            0x66
#define ROM_FLASH_CMD_RST              0x99

#define ROM_FLASH_CMD_SE4B             0x21
#define ROM_FLASH_CMD_SE4B_OCT         0xDE21
#define ROM_FLASH_CMD_BE4B             0xDC
#define ROM_FLASH_CMD_BE4B_OCT         0x23DC
#define ROM_FLASH_CMD_RSTEN_OCT        0x9966
#define ROM_FLASH_CMD_RST_OCT          0x6699

#define ROM_FLASH_CMD_FSTRD4B_STR      0x13EC
#define ROM_FLASH_CMD_FSTRD4B_DTR      0x11EE
#define ROM_FLASH_CMD_FSTRD4B          0x0C
#define ROM_FLASH_CMD_PP4B             0x12
#define ROM_FLASH_CMD_PP4B_OCT         0xED12

#define ROM_FLASH_CMD_RDID_OCT         0x609F
#define ROM_FLASH_CMD_WREN_OCT         0xF906
#define ROM_FLASH_CMD_RDSR_OCT         0xFA05
#define ROM_FLASH_CMD_RDCR2            0x71
#define ROM_FLASH_CMD_RDCR2_OCT        0x8E71
#define ROM_FLASH_CMD_WRCR2            0x72
#define ROM_FLASH_CMD_WRCR2_OCT        0x8D72

/* Definitions for GigaDevice GD25LX256E Flash */

#define ROM_FLASH_CMD_RDFSR_GD            0x70
#define ROM_FLASH_CMD_RD_GD               0x03
#define ROM_FLASH_CMD_RD4B_GD             0x13
#define ROM_FLASH_CMD_FSTRD_GD            0x0B
#define ROM_FLASH_CMD_FSTRD4B_GD          0x0C
#define ROM_FLASH_CMD_FSTRD_OOUT_GD       0x8B
#define ROM_FLASH_CMD_FSTRD4B_OOUT_GD     0x7C
#define ROM_FLASH_CMD_FSTRD_OIOSTR_GD     0xCB
#define ROM_FLASH_CMD_FSTRD4B_OIOSTR_GD   0xCC
#define ROM_FLASH_CMD_FSTRD4B_OIODTR_GD   0xFD

#define ROM_FLASH_CMD_PP_GD               0x02
#define ROM_FLASH_CMD_PP4B_GD             0x12
#define ROM_FLASH_CMD_PP_OOUT_GD          0x82
#define ROM_FLASH_CMD_PP4B_OOUT_GD        0x84
#define ROM_FLASH_CMD_PP_OIO_GD           0xC2
#define ROM_FLASH_CMD_PP4B_OIOSTR_GD      0x8E

#define ROM_FLASH_CMD_SE_GD               0x20
#define ROM_FLASH_CMD_SE4B_GD             0x21
#define ROM_FLASH_CMD_BE32K_GD            0x52
#define ROM_FLASH_CMD_BE32K4B_GD          0x5C
#define ROM_FLASH_CMD_BE64K_GD            0xD8
#define ROM_FLASH_CMD_BE64K4B_GD          0xDC

#define ROM_FLASH_CMD_EN4B_GD             0xB7
#define ROM_FLASH_CMD_DIS4B_GD            0xE9

/* spi user mode command config */

/* Config the spi user command
 * spi_num spi port
 * pcmd pointer to accept the spi command struct
 */

void esp_rom_spi_cmd_config(int spi_num, esp_rom_spi_cmd_t *pcmd);

/* Start a spi user command sequence
 * spi_num spi port
 * rx_buf buffer pointer to receive data
 * rx_len receive data length in byte
 * cs_en_mask decide which cs to use, 0 for cs0, 1 for cs1
 * is_write_erase to indicate whether this is a write or erase
 * operation, since the CPU would check permission
 */

void esp_rom_spi_cmd_start(int spi_num, uint8_t *rx_buf, uint16_t rx_len,
                           uint8_t cs_en_mask, bool is_write_erase);

/* Config opi flash pads according to efuse settings. */

void esp_rom_opiflash_pin_config(void);

/* Set SPI operation mode
 * spi_num spi port
 * mode Flash Read Mode
 */

void esp_rom_spi_set_op_mode(int spi_num, esp_rom_spiflash_read_mode_t mode);

/* Set data swap mode in DTR(DDR) mode
 * spi_num spi port
 * wr_swap to decide whether to swap fifo data in dtr write operation
 * rd_swap to decide whether to swap fifo data in dtr read operation
 */

void esp_rom_spi_set_dtr_swap_mode(int spi, bool wr_swap, bool rd_swap);

/* To send reset command in spi/opi-str/opi-dtr mode(for MX25UM25645G)
 * spi_num spi port
 */

void esp_rom_opiflash_mode_reset(int spi_num);

#if 0

/* MX25UM25645G opi flash interface */

/* To execute a flash operation command
 * spi_num spi port
 * mode Flash Read Mode
 * cmd data to send in command field
 * cmd_bit_len bit length of command field
 * addr data to send in address field
 * addr_bit_len bit length of address field
 * dummy_bits bit length of dummy field
 * mosi_data data buffer to be sent in mosi field
 * mosi_bit_len bit length of data buffer to be sent in mosi field
 * miso_data data buffer to accept data in miso field
 * miso_bit_len bit length of data buffer to accept data in miso field
 * cs_mark decide which cs pin to use. 0: cs0, 1: cs1
 * is_write_erase_operation to indicate whether this a write or erase
 * flash operation
 */

void esp_rom_opiflash_exec_cmd(int spi_num, esp_rom_spiflash_read_mode_t mode,
                               uint32_t cmd, int cmd_bit_len,
                               uint32_t addr, int addr_bit_len,
                               int dummy_bits,
                               uint8_t *mosi_data, int mosi_bit_len,
                               uint8_t *miso_data, int miso_bit_len,
                               uint32_t cs_mask,
                               bool is_write_erase_operation);

/* end reset command to opi flash
 * spi_num spi port
 * mode Flash Operation Mode
 */

void esp_rom_opiflash_soft_reset(int spi_num,
                                 esp_rom_spiflash_read_mode_t mode);

/* To read opi flash ID
 * Note command format would be defined in initialization
 *   out_id buffer to accept id
 * Return flash operation result
 */

uint32_t esp_rom_opiflash_read_id(int spi_num,
                                  esp_rom_spiflash_read_mode_t mode);

/* To read opi flash status register(for MX25UM25645G)
 * spi_num spi port
 * mode Flash Operation Mode
 * Return opi flash status value
 */

uint8_t esp_rom_opiflash_rdsr(int spi_num, esp_rom_spiflash_read_mode_t mode);

/* Wait opi flash status register to be idle
 * spi_num spi port
 * mode Flash Operation Mode
 */

void esp_rom_opiflash_wait_idle(int spi_num,
                                esp_rom_spiflash_read_mode_t mode);

/* To read the config register2(for MX25UM25645G)
 * spi_num spi port
 * mode Flash Operation Mode
 * addr the address of configure register
 * Return value of config register2
 */

uint8_t esp_rom_opiflash_rdcr2(int spi_num,
                               esp_rom_spiflash_read_mode_t mode,
                               uint32_t addr);

/* To write the config register2(for MX25UM25645G)
 * spi_num spi port
 * mode Flash Operation Mode
 * addr the address of config register
 * val the value to write
 */

void esp_rom_opiflash_wrcr2(int spi_num, esp_rom_spiflash_read_mode_t mode,
                            uint32_t addr, uint8_t val);

/* To erase flash sector(for MX25UM25645G)
 * spi_num spi port
 * address the sector address to be erased
 * mode Flash operation mode
 * Return flash operation result
 */

esp_rom_spiflash_result_t
esp_rom_opiflash_erase_sector(int spi_num, uint32_t address,
                              esp_rom_spiflash_read_mode_t mode);

/* To erase flash block(for MX25UM25645G)
 * spi_num spi port
 * address the block address to be erased
 * mode Flash operation mode
 * Return flash operation result
 */

esp_rom_spiflash_result_t
esp_rom_opiflash_erase_block_64k(int spi_num, uint32_t address,
                                 esp_rom_spiflash_read_mode_t mode);

/* To erase a flash area define by start address and length
 * (for MX25UM25645G)
 * spi_num spi port
 * start_addr the start address to be erased
 * area_len the erea length to be erased
 * mode flash operation mode
 * Return flash operation result
 */

esp_rom_spiflash_result_t
esp_rom_opiflash_erase_area(int spi_num, uint32_t start_addr,
                            uint32_t area_len,
                            esp_rom_spiflash_read_mode_t mode);

/* T read data from opi flash(for MX25UM25645G)
 * spi_num spi port
 * mode flash operation mode
 * flash_addr flash address to read data from
 * data_addr data buffer to accept the data
 * len data length to be read
 * Return flash operation result
 */

esp_rom_spiflash_result_t
esp_rom_opiflash_read(int spi_num, esp_rom_spiflash_read_mode_t mode,
                      uint32_t flash_addr, uint8_t *data_addr, int len);

/* To write data to opi flash(for MX25UM25645G)
 * spi_num spi port
 * mode flash operation mode
 * flash_addr flash address to write data to
 * data_addr data buffer to write to flash
 * len data length to write
 * Return flash operation result
 */

esp_rom_spiflash_result_t
esp_rom_opiflash_write(int spi_num, esp_rom_spiflash_read_mode_t mode,
                       uint32_t flash_addr, uint8_t *data_addr,
                       uint32_t len);

/* To set opi flash operation mode(for MX25UM25645G)
 * spi_num spi port
 * cur_mode current operation mode
 * target the target operation mode to be set
 */

void esp_rom_opiflash_set_mode(int spi_num,
                               esp_rom_spiflash_read_mode_t cur_mode,
                               esp_rom_spiflash_read_mode_t target_mode);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32S2_ROM_ESP32S2_OPI_FLASH_H */
