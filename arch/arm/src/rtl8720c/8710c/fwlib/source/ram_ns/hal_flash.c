/**************************************************************************//**
 * @file     hal_flash.c
 * @brief    Implement flash commands and features.
 * @version  1.00
 * @date     2017-08-22
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "hal_flash.h"
#include "hal_spic.h"
#include "rtl8710c_spic_type.h"
#include "hal_sce.h"
#include "hal_efuse.h"

extern const hal_flash_func_stubs_t hal_flash_stubs;

flash_cmd_t new_flash_cmd = {
    0x06, // wren,
    0x04, // wrdi,
    0x05, // rdsr,
    0x01, // wrsr,
    0x35, // rdsr2,
    0x31, // wrsr2,
    0x15, // rdsr3,
    0x11, // wrsr3,
    0xC8, // rear, Read extended address register
    0xC5, // wear, Write extended address register
    0x20, // se,
    0x52, // be_32k,
    0xD8, // be_64k,
    0xC7, // ce,
    0x9F, // rdid,
    0x03, // read,
    0x0B, // fread,
    0x3B, // dread,
    0xBB, // str_2read,
    0xBD, // dtr_2read,
    0x6B, // qread,
    0xEB, // str_4read,
    0xED, // dtr_4read
    0x02, // pp,
    0xB9, // dp,
    0x00, // udp,
    0xAB, // rdp,
    0xFF, // en_spi,
    0x38, // en_qpi,
    0x36, // block_lock,
    0x39, // block_unlock,
    0x7E, // global_lock,
    0x98, // global_unlock,
    0x3D, // read_block_lock,
    0x66, // en_reset,
    0x99, // reset,
    0x00 // read_sfdp,
};


// TODO: modification for 8710C: pin mux register/control
#if CONFIG_SPI_FLASH_EN

/**

        \addtogroup hs_hal_flash FLASH
        @{
*/

/**

        \addtogroup hs_hal_gdma_flash_func FLASH HAL RAM APIs
        \ingroup hs_hal_flash
        \brief The FLASH HAL APIs. Functions become an interface between API functions and ROM codes.
        @{
*/


/** \brief Description of hal_flash_read_unique_id
 *
 *    hal_flash_read_unique_id is used to read unique ID for the flash.
 *    This function is only valid for Winbond Flash.
 *    The ID is unique for each flash sample even if the part number is the name.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param uint8_t *buf:      The buff to store ID.
 *   \param uint8_t *len:      The lenght of ID.
 *
 *   \return void.
 */
void hal_flash_read_unique_id (phal_spic_adaptor_t phal_spic_adaptor, uint8_t *buf, uint8_t len)
{
    hal_flash_stubs.hal_flash_read_unique_id(phal_spic_adaptor, buf, len);
}

/** \brief Description of hal_flash_read_id
 *
 *    hal_flash_read_id is used to read ID for the flash.
 *    Flash with the same part number has the same flash ID.
 *    ID can be used to identify the type of flash.
 *    The ID information is stored in the flash adaptor.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_flash_read_id (phal_spic_adaptor_t phal_spic_adaptor)
{
    return hal_flash_stubs.hal_flash_read_id(phal_spic_adaptor);
}

/** \brief Description of hal_flash_set_write_enable
 *
 *    hal_flash_set_write_enable is used to send write enable command.
 *    After the command is sent, SPIC checks if the write enable bit is set or not.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_set_write_enable (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_set_write_enable(phal_spic_adaptor);
}

/** \brief Description of hal_flash_set_status
 *
 *    hal_flash_set_status is used to send write status register command.
 *    Write status register command may not be the same for different status registers or different flash types.
 *    Users can input an instruction and a value according to flash data sheet to set the status register.
 *    The function will check if flash is ready after setting the status register.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 cmd:      The instruction of write status register command.
 *   \param u8 data:     The value is going to be set in status register.
 *
 *   \return void.
 */
void hal_flash_set_status (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data)
{
    hal_flash_stubs.hal_flash_set_status(phal_spic_adaptor, cmd, data);
}

/** \brief Description of hal_flash_set_status_no_check
 *
 *    hal_flash_set_status_no_check is used to send write status register command.
 *    Write status register command may not be the same for different status registers or different flash types.
 *    Users can input an instruction and a value according to flash data sheet to set the status register.
 *    The function will NOT check if flash is ready after setting the status register.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 cmd:      The instruction of write status register command.
 *   \param u8 data:     The value is going to be set in status register.
 *
 *   \return void.
 */
void hal_flash_set_status_no_check (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data)
{
    hal_flash_stubs.hal_flash_set_status_no_check(phal_spic_adaptor, cmd, data);
}

/** \brief Description of hal_flash_set_status_with_addr
 *
 *    hal_flash_set_status_with_addr is used to send write status register with address command.
 *    Write status register command may not be the same for different status registers or different flash types.
 *    Some write status commands allow to carry one address byte to identify which status register.
 *    Users can input an instruction, an address and a value according to flash data sheet to set the status register.
 *    The function will check if flash is ready after setting the status register.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 cmd:      The instruction of write status register command.
 *   \param u8 addr:     The address of write status register command.
 *   \param u8 data:     The value is going to be set in status register.
 *
 *   \return void.
 */
void hal_flash_set_status_with_addr (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 addr, u8 data)
{
    hal_flash_stubs.hal_flash_set_status_with_addr(phal_spic_adaptor, cmd, addr, data);
}

/** \brief Description of hal_flash_set_extended_addr
 *
 *    hal_flash_set_extended_addr is used to send set extended address command.
 *    Flash access 128 Mbit region at most with three address bytes.
 *    With this command, the value followed by the command presents the fourth address byte.
 *    Flash accesses beyond 128 Mbit region by setting different value of the fourth address byte to switch among different 128 Mbit bank.
 *    Some flashes cannot support this feature, please refer to flash data sheets.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 data:     The fourth address byte.
 *
 *   \return void.
 */
void hal_flash_set_extended_addr (phal_spic_adaptor_t phal_spic_adaptor, u8 data)
{
    hal_flash_stubs.hal_flash_set_extended_addr(phal_spic_adaptor, data);
}

/** \brief Description of hal_flash_set_write_protect_mode
 *
 *    hal_flash_set_write_protect_mode is used to send set write protect mode command.
 *    Write protect feature can be either by multiple blocks or by individual block.
 *    Only some of Winbond flashes support this feature.
 *    The block protect control byte is written to one of the status register.
 *    Please refer to flash data sheet for more details
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 mode:     The block protect control parameter.
 *
 *   \return void.
 */
void hal_flash_set_write_protect_mode (phal_spic_adaptor_t phal_spic_adaptor, u8 mode)
{
    hal_flash_stubs.hal_flash_set_write_protect_mode(phal_spic_adaptor, mode);
}

/** \brief Description of hal_flash_get_status
 *
 *    hal_flash_get_status is used to send read status register command.
 *    Each flash may have more than one status registers that can be accessed with different instructions.
 *    Please refer to flash data sheet for more details
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 cmd:     The instruction of read status register command.
 *
 *   \return u8: the value of the status register.
 */
u8 hal_flash_get_status (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd)
{
    return hal_flash_stubs.hal_flash_get_status(phal_spic_adaptor, cmd);
}

/** \brief Description of hal_flash_get_status_with_addr
 *
 *    hal_flash_get_status_with_addr is used to send read status register command.
 *    Each flash may have more than one status registers that can be accessed with different instructions.
 *    A address byte followed by the instruction is sent to identify which status register.
 *    Please refer to flash data sheet for more details
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 cmd:     The instruction of read status register command.
 *   \param u8 addr:    The address byte of read status register command.
 *
 *   \return u8: the value of the status register.
 */
u8 hal_flash_get_status_with_addr (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 addr)
{
    return hal_flash_stubs.hal_flash_get_status_with_addr(phal_spic_adaptor, cmd, addr);
}

/** \brief Description of hal_flash_get_extended_addr
 *
 *    hal_flash_get_extended_addr is used to send read extended address register command.
 *    The value of the extended address register presents the fourth byte of the address phase.
 *    Some flashes cannot support this feature, please refer to flash data sheets.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return u8: the value of the extended addr register.
 */
u8 hal_flash_get_extended_addr (phal_spic_adaptor_t phal_spic_adaptor)
{
    return hal_flash_stubs.hal_flash_get_extended_addr(phal_spic_adaptor);
}

/** \brief Description of hal_flash_wait_ready
 *
 *    hal_flash_wait_ready is used to check the BUSY/WIP bit of the status register.
 *    This function polls the BUSY/WIP of the status register until flash returns to ready state.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_wait_ready (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_wait_ready(phal_spic_adaptor);
}

/** \brief Description of hal_flash_chip_erase
 *
 *    hal_flash_chip_erase is used to send chip erase command.
 *    This function aims to erase whole flash chip.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_chip_erase (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_chip_erase(phal_spic_adaptor);
}

/** \brief Description of hal_flash_64k_block_erase
 *
 *    hal_flash_64k_block_erase is used to send 64K block erase command.
 *    This function aims to erase 64K byte blocks.
 *    The address is aligned to 64K byte boundaries.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 address:      The target address to be erased.
 *
 *   \return void.
 */
void hal_flash_64k_block_erase (phal_spic_adaptor_t phal_spic_adaptor, u32 address)
{
    hal_flash_stubs.hal_flash_64k_block_erase(phal_spic_adaptor, address);
}

/** \brief Description of hal_flash_32k_block_erase
 *
 *    hal_flash_32k_block_erase is used to send 32K block erase command.
 *    This function aims to erase 32K byte blocks.
 *    The address is aligned to 32K byte boundaries.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 address:      The target address to be erased.
 *
 *   \return void.
 */
void hal_flash_32k_block_erase (phal_spic_adaptor_t phal_spic_adaptor, u32 address)
{
    hal_flash_stubs.hal_flash_32k_block_erase(phal_spic_adaptor, address);
}

/** \brief Description of hal_flash_sector_erase
 *
 *    hal_flash_sector_erase is used to send sector erase command.
 *    This function aims to erase 4K byte sectors.
 *    The address is aligned to 4K byte boundaries.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 address:      The target address to be erased.
 *
 *   \return void.
 */
void hal_flash_sector_erase (phal_spic_adaptor_t phal_spic_adaptor, u32 address)
{
    hal_flash_stubs.hal_flash_sector_erase(phal_spic_adaptor, address);
}

/** \brief Description of hal_flash_query_sector_protect_state
 *
 *    hal_flash_query_sector_protect_state is used to query the sector's write protect status.
 *    This function is only valid for flashes supporting individual block protect or sector protect feature.
 *    Please refer to flash data sheets for more details.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 address:      The target address to query write protect state.
 *
 *   \return u8: the write protect status.
 */
u8 hal_flash_query_sector_protect_state(phal_spic_adaptor_t phal_spic_adaptor, u32 address)
{
    return hal_flash_stubs.hal_flash_query_sector_protect_state(phal_spic_adaptor, address);
}

/** \brief Description of hal_flash_protect_sector
 *
 *    hal_flash_protect_sector is used to send individual write protect command.
 *    This function is only valid for flashes supporting individual block protect or sector protect feature.
 *    For actual protect regions depending on flash types.
 *    Please refer to flash data sheets for more details.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 address:      The target address to enable write protect.
 *
 *   \return void.
 */
void hal_flash_protect_sector (phal_spic_adaptor_t phal_spic_adaptor, u32 address)
{
    hal_flash_stubs.hal_flash_protect_sector(phal_spic_adaptor, address);
}

/** \brief Description of hal_flash_unprotect_sector
 *
 *    hal_flash_unprotect_sector is used to send individual write unprotect command.
 *    This function is only valid for flashes supporting individual block protect or sector protect feature.
 *    For actual protect regions depending on flash types.
 *    Please refer to flash data sheets for more details.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 address:      The target address to disable write protect.
 *
 *   \return void.
 */
void hal_flash_unprotect_sector (phal_spic_adaptor_t phal_spic_adaptor, u32 address)
{
    hal_flash_stubs.hal_flash_unprotect_sector(phal_spic_adaptor, address);
}

/** \brief Description of hal_flash_global_lock
 *
 *    hal_flash_global_lock is used to send whole chip write protect command.
 *    This function is only valid some of Winbond flash.
 *    Please refer to flash data sheets for more details.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_global_lock (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_global_lock(phal_spic_adaptor);
}

/** \brief Description of hal_flash_global_unlock
 *
 *    hal_flash_global_unlock is used to send whole chip write unprotect command.
 *    This function is only valid some of Winbond flash.
 *    Please refer to flash data sheets for more details.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_global_unlock (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_global_unlock(phal_spic_adaptor);
}

/** \brief Description of hal_flash_set_dummy_cycle
 *
 *    hal_flash_set_dummy_cycle is used to configure flash dummy cycle.
 *    Flash allows users to adjust dummy cycles with various operting frequency under multiple IO modes.
 *    Higher speed requires more dummy cycles.
 *    We take into account the SPIC clock source and performance to derive optimal operating frequnecy of flash.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_set_dummy_cycle (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_set_dummy_cycle(phal_spic_adaptor);
}

/** \brief Description of hal_flash_set_quad_enable
 *
 *    hal_flash_set_quad_enable is used to set quad enable bit of flash.
 *    Some flash types require flash to set quad enable bit before switching to Quad IO mode.
 *    Only Winbond and MXIC supports this function.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_set_quad_enable (phal_spic_adaptor_t phal_spic_adaptor)
{
    pflash_cmd_t cmd = phal_spic_adaptor->cmd;
    u8 flash_type = phal_spic_adaptor->flash_type;
    u8 status_value = 0;
    u8 data[2];

    switch (flash_type) {
        case FLASH_TYPE_WINBOND:
        case FLASH_TYPE_GD32:
        case FLASH_TYPE_BOYA:
        case FLASH_TYPE_XMC:
        case FLASH_TYPE_ZBIT:
            status_value = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr2);
            hal_flash_set_status(phal_spic_adaptor, cmd->wrsr2, 0x2 | status_value);
            break;

        case FLASH_TYPE_MXIC:
            status_value = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr);
            hal_flash_set_status(phal_spic_adaptor, cmd->wrsr, 0x40 | status_value);
            break;

        case FLASH_TYPE_GD:
        case FLASH_TYPE_XTX:
            data[0] = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr);
            data[1] = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr2) | 0x2;
            hal_flash_set_write_enable(phal_spic_adaptor);
            spic_tx_cmd(phal_spic_adaptor, cmd->wrsr, 2, data);
            break;

        case FLASH_TYPE_EON:
            break;

        default:
            DBG_SPIF_ERR("Unknown flash type, cannot set QE bit!\n");
    }
}

/** \brief Description of hal_flash_unset_quad_enable
 *
 *    hal_flash_unset_quad_enable is used to clear quad enable bit of flash.
 *    Quad IO bit should be cleaned before switching to other non-Quad IO mode.
 *    Only Winbond and MXIC supports this function.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_unset_quad_enable (phal_spic_adaptor_t phal_spic_adaptor)
{
    pflash_cmd_t cmd = phal_spic_adaptor->cmd;
    u8 flash_type = phal_spic_adaptor->flash_type;
    u8 status_value = 0;
    u8 data[2];

    switch (flash_type) {
        case FLASH_TYPE_WINBOND:         
        case FLASH_TYPE_GD32:
        case FLASH_TYPE_BOYA:
        case FLASH_TYPE_XMC:
        case FLASH_TYPE_ZBIT:
            status_value = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr2);
            hal_flash_set_status(phal_spic_adaptor, cmd->wrsr2, ~0x2 & status_value);
            break;

        case FLASH_TYPE_MXIC:
            status_value = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr);
            hal_flash_set_status(phal_spic_adaptor, cmd->wrsr, ~0x40 & status_value);
            break;

        case FLASH_TYPE_GD:
        case FLASH_TYPE_XTX:
            data[0] = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr);
            data[1] = hal_flash_get_status(phal_spic_adaptor, cmd->rdsr2) & (~0x2);
            hal_flash_set_write_enable(phal_spic_adaptor);
            spic_tx_cmd(phal_spic_adaptor, cmd->wrsr, 2, data);
            break;
            
        case FLASH_TYPE_EON:
        case FLASH_TYPE_MICRON:
            break;

        default:
            DBG_SPIF_ERR("Unknown flash type, cannot reset QE bit!\n");
    }
}

/** \brief Description of hal_flash_enable_qpi
 *
 *    hal_flash_enable_qpi is used to switch flash to QPI mode (4-4-4).
 *    Quad enable bit is set before SPIC sends enter QPI mode command.
 *    Only Winbond and MXIC supports this function.
 *    Must ensure flash is under SPI mode before CPU enters this function.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_enable_qpi (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_enable_qpi(phal_spic_adaptor);
}

/** \brief Description of hal_flash_return_spi
 *
 *    hal_flash_return_spi is used to switch flash back to SPI mode.
 *    Unlike flash_reset_to_spi_rtl8195bhp function recovering flash back to SPI mode from abnormal state,
 *    this funciton is a bridge to switch between different IO mode when flash is under control.
 *    Every IO mode should call this function to switch back to SPI mode before flash enters the other IO mode.
 *    Only Winbond and MXIC support this function.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_return_spi (phal_spic_adaptor_t phal_spic_adaptor)
{
    pflash_cmd_t cmd = phal_spic_adaptor->cmd;
    spic_valid_cmd_t valid_cmd;
    spic_init_para_t spic_init_data;
    pspic_init_para_t pspic_init_data = &spic_init_data;
    SPIC_Type *spic_dev  = phal_spic_adaptor->spic_dev;
    u8 spic_send_cmd_mode = phal_spic_adaptor->spic_send_cmd_mode;
    u8 spic_bit_mode = phal_spic_adaptor->spic_bit_mode;
    u8 cpu_type = spic_query_system_clk();

    if (phal_spic_adaptor->spic_init_data[SpicOneIOMode][cpu_type].valid) {
        pspic_init_data = &(phal_spic_adaptor->spic_init_data[SpicOneIOMode][cpu_type]);
    } else {
        spic_load_default_setting(pspic_init_data);
    }

    /*Disable SPIC to set control register*/
    spic_disable_rtl8710c(spic_dev);

    /*Send cmd to switch from QPI to SPI mode*/
    if ((spic_send_cmd_mode == QuadChnl)) {
        spic_tx_cmd_no_check(phal_spic_adaptor, cmd->en_spi, 0, 0);
    }

    phal_spic_adaptor->spic_send_cmd_mode = SingleChnl;
    phal_spic_adaptor->cmd_chnl = SingleChnl;
    phal_spic_adaptor->addr_chnl = SingleChnl;
    phal_spic_adaptor->data_chnl = SingleChnl;
    spic_set_chnl_num(phal_spic_adaptor);
    hal_flash_wait_ready(phal_spic_adaptor);

    if ((spic_bit_mode == SpicQuadIOMode) || (spic_bit_mode == SpicQpiMode)) {
        hal_flash_unset_quad_enable(phal_spic_adaptor);
    }

    /*Restore back to one IO auto mode*/
    valid_cmd.w = 0;
    valid_cmd.b.wr_blocking = ENABLE;
    spic_dev->valid_cmd = valid_cmd.w;

    /*Resotre back to one IO user mode*/
    phal_spic_adaptor->spic_bit_mode = SpicOneIOMode;
    phal_spic_adaptor->read_cmd = cmd->read;
    phal_spic_adaptor->addr_byte_num = ThreeBytesLength;
    spic_dev->auto_length_b.auto_addr_length = phal_spic_adaptor->addr_byte_num;
    spic_dev->addr_length_b.addr_phase_length = phal_spic_adaptor->addr_byte_num;

    spic_set_dummy_cycle_rtl8710c(spic_dev, pspic_init_data->rd_dummy_cycle);
    spic_set_baudr_rtl8710c(spic_dev, pspic_init_data->baud_rate);
}

/** \brief Description of hal_flash_enter_power_down
 *
 *    hal_flash_enter_power_down is used to send deep power down command.
 *    Flash enters deep power down state after this function.
 *    Any access except few commands is ignored by flash.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_flash_enter_power_down (phal_spic_adaptor_t phal_spic_adaptor)
{
    return hal_flash_stubs.hal_flash_enter_power_down(phal_spic_adaptor);
}

/** \brief Description of hal_flash_release_from_power_down
 *
 *    hal_flash_release_from_power_down is used to send release from deep power down command.
 *    Flash returns from deep power down state after this function.
 *    It can accept any command normally.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_flash_release_from_power_down (phal_spic_adaptor_t phal_spic_adaptor)
{
    return hal_flash_stubs.hal_flash_release_from_power_down(phal_spic_adaptor);
}

/** \brief Description of hal_flash_stream_read
 *
 *    hal_flash_stream_read is used to read sequential data with auto mode.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 length:      The transfer length.
 *   \param u32 addr:      The starting read address in flash.
 *   \param u8 *data:      The destination address in memory to store data.
 *
 *   \return void.
 */
void hal_flash_stream_read (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data)
{
    hal_flash_stubs.hal_flash_stream_read(phal_spic_adaptor, length, addr, data);
}

/** \brief Description of hal_flash_stream_write
 *
 *    hal_flash_stream_write is used to program sequential data with auto mode.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 length:      The transfer length.
 *   \param u32 addr:      The starting program address in flash.
 *   \param u8 *data:      The address of data. Data stored in ram is about to be programmed to the flash.
 *
 *   \return void.
 */
void hal_flash_stream_write (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data)
{
    hal_flash_stubs.hal_flash_stream_write(phal_spic_adaptor, length, addr, data);
}

/** \brief Description of hal_flash_burst_read
 *
 *    hal_flash_burst_read is used to read sequential data with user mode.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 length:      The transfer length.
 *   \param u32 addr:      The starting read address in flash.
 *   \param u8 *data:      The destination address in memory to store data.
 *
 *   \return void.
 */
void hal_flash_burst_read (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data)
{
    hal_flash_stubs.hal_flash_burst_read(phal_spic_adaptor, length, addr, data);
}

/** \brief Description of hal_flash_burst_write
 *
 *    hal_flash_burst_write is used to program sequential data with user mode.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 length:      The transfer length.
 *   \param u32 addr:      The starting program address in flash.
 *   \param u8 *data:      The address of data. Data stored in ram or flash is about to be programmed to the flash.
 *
 *   \return void.
 */
void hal_flash_burst_write (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data)
{
    u32 existed_data_size;
    u32 program_size;
    u32 page_size;
    u32 is_encry;
    u32 phy_src;

    if ((((u32)(data)) >> 24) == 0x9B) {
        hal_xip_get_phy_addr((u32) data, (u32 *)&phy_src, &is_encry);
        
        if (is_encry) {
            DBG_SPIF_ERR("Source address should not be on the encryted remapping region!\r\n");
            return;
        } else {
            data = (u8 *)phy_src;
        }
    }

    if (((u32)data >> 24) == 0x98) {       
        data = (u8*)((u32)data & 0xFFFFFF);
        page_size = 256;
        existed_data_size = addr & 0xFF;
        
        if((length >= page_size)
            ||((length + existed_data_size) >= page_size)) {
            program_size = page_size - existed_data_size;
        } else {
            program_size = length;
        }
        
        while(length > 0){
            if(existed_data_size){
                hal_flash_read_write_flash(phal_spic_adaptor, program_size, addr, data);
                addr += program_size;
                data += program_size;
                length -= program_size;
                existed_data_size = 0;
            } else {
                while (length >= page_size) {
                    hal_flash_read_write_flash(phal_spic_adaptor, page_size, addr, data);
                    addr += page_size;
                    data += page_size;
                    length -= page_size;
                }
        
                if (length > 0) {
                    hal_flash_read_write_flash(phal_spic_adaptor, length, addr, data);
                    break;
                }
            }
        }
    } else {       
        hal_flash_stubs.hal_flash_burst_write(phal_spic_adaptor, length, addr, data);
    }
}

/** \brief Description of hal_flash_page_program
 *
 *    hal_flash_page_program is used to divide total transfer length into several pages.
 *    Each page program command can program 256 bytes, a page size, at most.
 *    Multiple program commands should be sent if the transfer length is larger than the page size.
 *    Please refer to flash data sheets about page program command for more details.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 length:      The transfer length.
 *   \param u32 addr:      The starting program address in flash.
 *   \param u8 *data:      The address of data. Data stored in ram is about to be programmed to the flash.
 *
 *   \return void.
 */
void hal_flash_page_program (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data)
{
    hal_flash_stubs.hal_flash_page_program(phal_spic_adaptor, length, addr, data);
}

/** \brief Description of hal_flash_read_write_flash
 *
 *    hal_flash_read_write_flash is used to program sequential data with user mode.
 *    The source address and the destination address must be in the flash.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u32 length:      The transfer length.
 *   \param u32 addr:      The starting program address in flash.
 *   \param u8 *data:      The address of data. Data stored in flash is about to be programmed to the other address in flash.
 *
 *   \return void.
 */
void hal_flash_read_write_flash (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data)
{
    SPIC_Type *spic_dev  = phal_spic_adaptor->spic_dev;
    pflash_cmd_t cmd = phal_spic_adaptor->cmd;
    spic_ctrlr0_t ctrlr0;
    u32 occu_size;
    u32 program_length;
    u32 page_size;
    u32 tmp;
    u8 addr_byte[4];
    u8 index;
    u8 spic_send_cmd_mode;
    u8 addr_byte_num;
    u8 buffer[256];
    u8* ptr = &buffer[0];

    hal_flash_stream_read(phal_spic_adaptor, length, (u32)data, buffer);
    
    addr_byte_num = phal_spic_adaptor->addr_byte_num;
    spic_send_cmd_mode = phal_spic_adaptor->spic_send_cmd_mode;
    program_length = 0;
    page_size = 256;
    occu_size = addr & 0xFF;

    /*Load ctrlr0 value to configure*/
    ctrlr0.w = spic_dev->ctrlr0;

    if(occu_size){
        if ((length >= page_size)
            || ((occu_size + length) >= page_size)) {
            program_length = page_size - occu_size;
        } else {
            program_length = length;
        }
    } else {//program from the beginning of the page
        if (length >= page_size) {
            program_length = page_size;
        } else {
            program_length = length;
        }
    }

    /* Set writen enable bit */
    hal_flash_set_write_enable(phal_spic_adaptor);

    /* Disable spic in user mode */
    spic_disable_rtl8710c(spic_dev);

    spic_dev->valid_cmd_b.prm_en = DISABLE;

    addr_byte[3] = (addr & 0xFF000000) >> 24;
    addr_byte[2] = (addr & 0xFF0000) >>16;
    addr_byte[1] = (addr & 0xFF00)>>8;
    addr_byte[0] = addr & 0xFF;

    if (addr_byte_num == 3) {
        tmp = (cmd->pp) | (addr_byte[2] << 8)|(addr_byte[1] << 16)|(addr_byte[0] << 24);
        spic_dev->dr_word = tmp;
    } else {
        spic_dev->dr_byte = cmd->pp;
        tmp = addr_byte[3] | (addr_byte[2] << 8) | (addr_byte[1] << 16) | (addr_byte[0] << 24);
        spic_dev->dr_word = tmp;
    }

    /* Set tx mode*/
    ctrlr0.b.tmod = TxMode;

    /* Set length of address byte */
    spic_dev->addr_length_b.addr_phase_length = addr_byte_num;

    /*Program can be executed only with 1-1-1, 4-4-4, 8-8-8 mode*/
    switch (spic_send_cmd_mode) {
        case SingleChnl:
            ctrlr0.b.cmd_ch = SingleChnl;
            ctrlr0.b.addr_ch = SingleChnl;
            ctrlr0.b.data_ch = SingleChnl;
            break;

        case QuadChnl:
            ctrlr0.b.cmd_ch = QuadChnl;
            ctrlr0.b.addr_ch = QuadChnl;
            ctrlr0.b.data_ch = QuadChnl;
            break;

        default:
            DBG_SPIF_ERR("Wrong IO Mode to program!\n");
    }

    spic_dev->ctrlr0 = ctrlr0.w;

    /*Set Ctrl2 to enable data split program*/
    spic_dev->ctrlr2_b.seq_en = ENABLE;

    //pre-load data before enabling
    index = 0;

    while (program_length > 4) {
        spic_dev->dr_word = (u32)*((u32 *)ptr);
        program_length -= 4;
        ptr += 4;
        index += 4;

        if (index >= (SPIC_FIFO_DEPTH - 8)) {
            break;
        }
    }

    /* Enable spic in user mode */
    spic_enable_rtl8710c(spic_dev);

    while (program_length >= 4) {
        spic_dev->dr_word = (u32)*((u32 *)ptr);
        program_length -= 4;
        ptr += 4;
    }

    while (program_length > 0) {
        spic_dev->dr_byte = (u8) (*ptr);
        program_length--;
        ptr++;
    }

    /*Poll SPIC to get ready*/
    spic_wait_ready(spic_dev);

    /*Poll Flash to get ready*/
    hal_flash_wait_ready(phal_spic_adaptor);

    /*Disable spic in user mode*/
    spic_disable_rtl8710c(spic_dev);

    spic_dev->ctrlr2_b.seq_en = DISABLE;

    spic_dev->valid_cmd_b.prm_en = ENABLE;
}


/** \brief Description of hal_flash_reset_to_spi
 *
 *    hal_flash_reset_to_spi is used to reset flash back to SPI mode.
 *    This function is called when SPIC cannot read correct ID.
 *    It would be two scenarios that ID cannot be read.
 *    First, flash may enter unknown state that SPIC cannot access the flash with the correct IO mode.
 *    Second, flash may be in deep power down state so that commands sent by SPIC cannot be recognized.
 *    This function aims to wake up flash and reset it to default SPI mode.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_reset_to_spi (phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_flash_stubs.hal_flash_reset_to_spi(phal_spic_adaptor);
}

/** \brief Description of hal_flash_support_new_type
 *
 *    hal_flash_support_new_type is used to modify some parameters to support new flash types.
 *    
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void hal_flash_support_new_type (phal_spic_adaptor_t phal_spic_adaptor)
{
    u8 flash_id = phal_spic_adaptor->flash_id[0];

    switch (flash_id) {
        case 0xC8:
            phal_spic_adaptor->cmd = &new_flash_cmd;
            if (phal_spic_adaptor->flash_id[2] >= 0x16) {
                phal_spic_adaptor->flash_type = FLASH_TYPE_GD32;
            } else {
                phal_spic_adaptor->flash_type = FLASH_TYPE_GD;
            }
            break;

        case 0x0B:
            phal_spic_adaptor->cmd = &new_flash_cmd;
            phal_spic_adaptor->flash_type = FLASH_TYPE_XTX;
            break;
            
        case 0x68:
            phal_spic_adaptor->cmd = &new_flash_cmd;
            phal_spic_adaptor->flash_type = FLASH_TYPE_BOYA;
            break;

        case 0x46:
            phal_spic_adaptor->cmd = &new_flash_cmd;
            phal_spic_adaptor->flash_type = FLASH_TYPE_XMC;
            break;
            
        case 0x1C:
            phal_spic_adaptor->flash_type = FLASH_TYPE_EON;
            break;
                   
        case 0x5E:
           phal_spic_adaptor->cmd = &new_flash_cmd;
           phal_spic_adaptor->flash_type = FLASH_TYPE_ZBIT;
           break;
           
        default:
            break;
    }
}

/** \brief Description of hal_flash_get_size
 *
 *    hal_flash_get_size returns the density of the flash.
 *    
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *
 *   \return u8: the density of the flash, unit is Mbit.
 */
u8 hal_flash_get_size (phal_spic_adaptor_t phal_spic_adaptor)
{
    u8 efuse_value;
    u8 size_id = phal_spic_adaptor->flash_id[2];
    u32 size = 0;

    size = ((1 << size_id) >> 20) * 8;

    /*Read efuse to get the package type*/
    hal_efuse_read(0x1F8, &efuse_value, 0);
    
    /*CF returns 16Mbit, others return real density*/
    if (efuse_value == 0xFE) {
        return 16;
    } else {
        return size;
    }
}


/** *@} */ /* End of group hs_hal_flash_ram_func */

/** *@} */ /* End of group hs_hal_flash */
#endif      // end of "#if CONFIG_SPI_FLASH_EN"
