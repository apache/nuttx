/**************************************************************************//**
 * @file     hal_spic.c
 * @brief    Functions to implement the flash controller operation.
 * @version  1.00
 * @date     2019-12-13
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

#include "hal_spic.h"
#include "hal_flash.h"
#include "hal_pinmux.h"
#include "rtl8710c_spic_type.h"
//#include <arm_cmse.h>
#include "memory.h"

// TODO: modification for 8710c: pin mux register & control, IO mode selection
#if defined(CONFIG_SPI_FLASH_EN) && (CONFIG_SPI_FLASH_EN == 1)

extern void hal_syson_spic_dev_ctrl(BOOL en);
extern int32_t spic_init_from_resume (void);

extern const hal_spic_func_stubs_t hal_spic_stubs;

RAM_BSS_NOINIT_SECTION hal_spic_adaptor_t hal_spic_adaptor;
hal_spic_adaptor_t *pglob_spic_adaptor;

const flash_pin_sel_t spic_flash_pins[3] = {
  /*  CS,      CLK,     D0,      D1,       D2,      D3 */
    { PIN_A7,  PIN_A8,  PIN_A11, PIN_A10,  PIN_A9,  PIN_A12},
    { PIN_B8,  PIN_B11, PIN_B12, PIN_B7,   PIN_B6,  PIN_B10},
    { PIN_A15, PIN_A16, PIN_A19, PIN_A20,  PIN_A17, PIN_A18}
};

/**

        \addtogroup hs_hal_spic Flash Controller
        @{
*/


/**

        \addtogroup hs_hal_spic_ram_func Flash Controller HAL RAM APIs
        \ingroup hs_hal_spic
        \brief The flash controller HAL APIs. Functions become an interface between API functions and ROM codes.
        @{
*/


/** \brief Description of spic_load_default_setting
 *
 *    spic_load_default_setting is used to load default setting for One IO mode at low speed without calibration.
 *    The calibration includes baud rate, delay line and dummy cycles(associated with path delay).
 *
 *   \param pspic_init_para_t pspic_init_data:      The pointer of struct storing calibation settings.
 *
 *   \return void.
 */
void spic_load_default_setting(pspic_init_para_t pspic_init_data)
{
    hal_spic_stubs.spic_load_default_setting(pspic_init_data);
}

/** \brief Description of spic_query_system_clk
 *
 *    spic_query_system_clk is used to query current CPU clock.
 *    Different CPU speed will apply different calibration parameters to optimize the flash controller.
 *
 *   \param void.
 *
 *   \return u8: The macro definition of CPU clock.
 */
u8 spic_query_system_clk(void)
{
    return hal_spic_stubs.spic_query_system_clk();
}

/** \brief Description of spic_clock_ctrl
 *
 *    spic_clock_ctrl is used to enable or disable the flash controller clock and function enable.
 *
 *   \param u8 ctl:      The control parameter.
 *
 *   \return void.
 */
void spic_clock_ctrl(u8 ctl)
{
    hal_spic_stubs.spic_clock_ctrl(ctl);
}

/** \brief Description of spic_init_setting
 *
 *    spic_init_setting is used to initialize the flash controller.
 *    The function and clock are enbaled at the first. Some parameters and registers are initialized afte that.
 *    Flash controller is set with One IO mode with low speed to ensure it can operate flash correctly without calibration procedure.
 *    If flash ID cannot be identified, a release from deep power down command is executed to wake the flash.
 *    A return to SPI command is executed followed by the release from deep power down command to ensure flash is under SPI mode.
 *    After these commands, a read ID command is issued to identify the flash type.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash controller adaptor.
 *   \param u8 spic_bit_mode:      The target IO mode the flash controller is going to operate.
 *
 *   \return hal_status_t.
 */
hal_status_t spic_init_setting(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode)
{
    return hal_spic_stubs.spic_init_setting(phal_spic_adaptor, spic_bit_mode);
}

/** \brief Description of spic_config_auto_mode
 *
 *    spic_config_auto_mode is used to configurate auto mode of flash controller.
 *    In the meantime, the flash controller sends commands to switch flash to the target IO mode.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash controller adaptor.
 *
 *   \return void.
 */
void spic_config_auto_mode(phal_spic_adaptor_t phal_spic_adaptor)
{
    SPIC_Type *spic_dev  = phal_spic_adaptor->spic_dev;
    pflash_cmd_t cmd = phal_spic_adaptor->cmd;
    spic_valid_cmd_t valid_cmd;
    u8 spic_bit_mode = phal_spic_adaptor->spic_bit_mode;
    u8 flash_type = phal_spic_adaptor->flash_type;


    /*Set default send cmd channel mode*/
    phal_spic_adaptor->spic_send_cmd_mode = SingleChnl;

    spic_disable_rtl8710c(spic_dev);
    valid_cmd.w = 0;
    valid_cmd.b.wr_blocking = ENABLE;
    valid_cmd.b.prm_en = ENABLE;

    spic_dev->valid_cmd = valid_cmd.w;

    switch (spic_bit_mode) {
        case SpicOneIOMode:
            spic_dev->read_fast_single = cmd->fread;
            break;

        case SpicDualOutputMode:        /*Enable 1-1-2*/
            spic_dev->read_dual_data = cmd->dread;
            spic_dev->valid_cmd_b.rd_dual_i = ENABLE;
            break;

        case SpicDualIOMode:            /*Enable 1-2-2*/
            spic_dev->read_dual_addr_data = cmd->str_2read;
            spic_dev->valid_cmd_b.rd_dual_io = ENABLE;
            break;

        case SpicQuadOutputMode:        /*Enable 1-1-4*/
            spic_dev->read_quad_data = cmd->qread;
            spic_dev->valid_cmd_b.rd_quad_o = ENABLE;
            if ((flash_type == FLASH_TYPE_WINBOND) 
                || (flash_type == FLASH_TYPE_GD)
                || (flash_type == FLASH_TYPE_GD32)
                || (flash_type == FLASH_TYPE_ZBIT)
                || (flash_type == FLASH_TYPE_XTX)) {
                hal_flash_set_quad_enable(phal_spic_adaptor);
            }
            break;

        case SpicQuadIOMode:            /*Enable 1-4-4*/
            spic_dev->read_quad_addr_data_b.rd_quad_io_cmd = cmd->str_4read;
            spic_dev->valid_cmd_b.rd_quad_io = ENABLE;
            hal_flash_set_quad_enable(phal_spic_adaptor);
            break;

        case SpicQpiMode:               /*Enable 4-4-4*/
            spic_dev->read_quad_addr_data_b.rd_quad_io_cmd = cmd->str_4read;
            spic_dev->valid_cmd_b.rd_quad_io = ENABLE;
            hal_flash_enable_qpi(phal_spic_adaptor);
            break;

        default:
            DBG_SPIF_ERR("spic_config_auto_mode_rtl8710c : Invalid Bit Mode\r\n");
    }
    
    phal_spic_adaptor->addr_byte_num = ThreeBytesLength;

    spic_dev->auto_length_b.auto_addr_length = phal_spic_adaptor->addr_byte_num;
    spic_enable_rtl8710c(spic_dev);
}

/** \brief Description of spic_config_user_mode
 *
 *    spic_config_user_mode is used to configurate user mode of flash controller.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash controller adaptor.
 *
 *   \return void.
 */
void spic_config_user_mode(phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_spic_stubs.spic_config_user_mode(phal_spic_adaptor);
}

/** \brief Description of spic_verify_calibration_para
 *
 *    spic_verify_calibration_para is used to verify calibration pattern for calibration procedure.
 *
 *   \param void.
 *
 *   \return BOOL: _TRUE: data is correct, _FALSE: data is wrong.
 */
BOOL spic_verify_calibration_para(void)
{
    return hal_spic_stubs.spic_verify_calibration_para();
}

/** \brief Description of spic_set_chnl_num
 *
 *    spic_set_chnl_num is used to set channel numbers of command, address and data phases to the control register.
 *    The flash controller should be disabled so that the values can be correctly written.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash controller adaptor.
 *
 *   \return void.
 */
void spic_set_chnl_num(phal_spic_adaptor_t phal_spic_adaptor)
{
    hal_spic_stubs.spic_set_chnl_num(phal_spic_adaptor);
}

/** \brief Description of spic_set_delay_line
 *
 *    spic_set_delay_line is used to fine-tune data receive timing with digital PHY.
 *    100 levels of delay line can be used.
 *
 *   \param u8 delay_line:      The level of the delay line, can be 0~99.
 *
 *   \return void.
 */
void spic_set_delay_line(u8 delay_line)
{
    hal_spic_stubs.spic_set_delay_line(delay_line);
}

/** \brief Description of spic_rx_cmd
 *
 *    spic_rx_cmd is an entry function to switch between 8IO or non-8IO rx_cmd functions depending on flash types.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash controller adaptor.
 *   \param u8 cmd:      The command byte.
 *   \param u8 data_phase_len:      The length of data followed by command phase.
 *   \param u8 *pdata:      The data followed by command phase.
 *
 *   \return void.
 */
void spic_rx_cmd(phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data_phase_len, u8 *pdata)
{
    hal_spic_stubs.spic_rx_cmd(phal_spic_adaptor, cmd, data_phase_len, pdata);
}

/** \brief Description of spic_tx_cmd_no_check
 *
 *    spic_tx_cmd_no_check is an entry function to switch between 8IO or non-8IO tx_cmd_no_check functions depending on flash types.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash controller adaptor.
 *   \param u8 cmd:      The command byte.
 *   \param u8 data_phase_len:      The length of data followed by command phase.
 *   \param u8 *pdata:      The data followed by command phase.
 *
 *   \return void.
 */
void spic_tx_cmd_no_check(phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data_phase_len, u8 *pdata)
{
    hal_spic_stubs.spic_tx_cmd_no_check(phal_spic_adaptor, cmd, data_phase_len, pdata);
}

/** \brief Description of spic_tx_cmd
 *
 *    spic_tx_cmd is an entry function to switch between 8IO or non-8IO tx_cmd functions depending on flash types.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash controller adaptor.
 *   \param u8 cmd:      The command byte.
 *   \param u8 data_phase_len:      The length of data followed by command phase.
 *   \param u8 *pdata:      The data followed by command phase.
 *
 *   \return void.
 */
void spic_tx_cmd(phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data_phase_len, u8 *pdata)
{
    hal_spic_stubs.spic_tx_cmd(phal_spic_adaptor, cmd, data_phase_len,pdata);
}

/** \brief Description of spic_wait_ready
 *
 *    spic_wait_ready is used to check if the flash controller is ready.
 *
 *   \param SPIC_Type *spic_dev:      The pointer of the flash controller register base.
 *
 *   \return void.
 */
void spic_wait_ready(SPIC_Type *spic_dev)
{
    hal_spic_stubs.spic_wait_ready(spic_dev);
}

/** \brief Description of spic_flush_fifo
 *
 *    spic_flush_fifo is used to flush FIFO of the flash controller.
 *
 *   \param SPIC_Type *spic_dev:      The pointer of the flash controller register base.
 *
 *   \return void.
 */
void spic_flush_fifo(SPIC_Type *spic_dev)
{
    hal_spic_stubs.spic_flush_fifo(spic_dev);
}

/** \brief Description of spic_pinmux_init
 *
 *    spic_pinmux_init is used to select pinmux to operate flash.
 *    The pinmux selection is registered and checked if the relevant pins are occpuied by other devices.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 ctl:      The flash pins control bit.
 *
 *   \return hal_status_t.
 */
hal_status_t spic_pinmux_ctl(phal_spic_adaptor_t phal_spic_adaptor, u8 ctl)
{   
    hal_status_t ret;
    u8 quad_pin_sel = phal_spic_adaptor->quad_pin_sel;
    pflash_pin_sel_t pflash_pin_sel = &(phal_spic_adaptor->flash_pin_sel);
#if !defined(CONFIG_BUILD_NONSECURE)
#define spic_pinmux_register        hal_pinmux_register
#define spic_pinmux_unregister      hal_pinmux_unregister
#else
#undef hal_pinmux_register
#undef hal_pinmux_unregister
    const hal_pin_manag_func_stubs_t *phal_pin_manag_stubs = (hal_pin_manag_func_stubs_t *)0x160;
    typedef hal_status_t (*spic_pinmux_register_t) (uint8_t pin_name, uint8_t periphl_id);
    typedef hal_status_t (*spic_pinmux_unregister_t) (uint8_t pin_name, uint8_t periphl_id);
    spic_pinmux_register_t spic_pinmux_register;
    spic_pinmux_unregister_t spic_pinmux_unregister;
    int state = spic_init_from_resume();

    if (!state) {
        // called from non-secure
        spic_pinmux_register = hal_pinmux_register_nsc;
        spic_pinmux_unregister = hal_pinmux_unregister_nsc;
    } else {
        // called from within secure
        spic_pinmux_register = phal_pin_manag_stubs->hal_pinmux_register;
        spic_pinmux_unregister = phal_pin_manag_stubs->hal_pinmux_unregister;
    }
#endif

    if (ctl == ENABLE) {    
        ret = spic_pinmux_register(pflash_pin_sel->pin_cs, PID_FLASH);
        ret |= spic_pinmux_register(pflash_pin_sel->pin_clk, PID_FLASH);
        ret |= spic_pinmux_register(pflash_pin_sel->pin_d0, PID_FLASH);
        ret |= spic_pinmux_register(pflash_pin_sel->pin_d1, PID_FLASH);

        if (quad_pin_sel) {
            ret |= spic_pinmux_register(pflash_pin_sel->pin_d2, PID_FLASH);
            ret |= spic_pinmux_register(pflash_pin_sel->pin_d3, PID_FLASH);
        }
    } else {
        ret = spic_pinmux_unregister(pflash_pin_sel->pin_cs, PID_FLASH);
        ret |= spic_pinmux_unregister(pflash_pin_sel->pin_clk, PID_FLASH);
        ret |= spic_pinmux_unregister(pflash_pin_sel->pin_d0, PID_FLASH);
        ret |= spic_pinmux_unregister(pflash_pin_sel->pin_d1, PID_FLASH);
        
        if (quad_pin_sel) {
            ret |= spic_pinmux_unregister(pflash_pin_sel->pin_d2, PID_FLASH);
            ret |= spic_pinmux_unregister(pflash_pin_sel->pin_d3, PID_FLASH);
            phal_spic_adaptor->quad_pin_sel = 0;
        }
    }

    return ret;
}

/** \brief Description of spic_init
 *
 *    spic_init is used to initialize flash controller setting.
 *    Flash can be accessed freely with API functions after this function.
 *    Flash access path delay is calibrated if no available setting is stored in flash before.
 *    After that, the flash controller initializes user mode and auto mode registers for the current IO mode.
 *    At the end of the function, the sequential transfer mode is also calibrated for future use.
 *
 *   \param void *adaptor:      The pointer of the flash adaptor.
 *   \param u8 spic_bit_mode:   The flash IO mode.
 *   \param pflash_pin_sel_t pflash_pin_sel:      The GPIO pin selections for flash.
 *
 *   \return hal_status_t.
 */
hal_status_t spic_init(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode, pflash_pin_sel_t pflash_pin_sel)
{
    pflash_dummy_cycle_t dummy_cycle;
    SPIC_Type *spic_dev;
    u32 default_dummy_cycle;
    pspic_init_para_t pspic_data;
    flash_pin_sel_t flash_pin_sel;
    u8 cpu_type = spic_query_system_clk();

    phal_spic_adaptor->spic_bit_mode = spic_bit_mode;

    /*Assign each pin to the adaptor*/
    flash_pin_sel.pin_cs = pflash_pin_sel->pin_cs;
    flash_pin_sel.pin_clk = pflash_pin_sel->pin_clk;
    flash_pin_sel.pin_d0 = pflash_pin_sel->pin_d0;
    flash_pin_sel.pin_d1 = pflash_pin_sel->pin_d1;
    flash_pin_sel.pin_d2 = pflash_pin_sel->pin_d2;
    flash_pin_sel.pin_d3 = pflash_pin_sel->pin_d3;
    //(phal_spic_adaptor->flash_pin_sel) = flash_pin_sel;
    rt_memcpy(&phal_spic_adaptor->flash_pin_sel, &flash_pin_sel, sizeof(flash_pin_sel_t));

    if ((spic_bit_mode == SpicQuadOutputMode) 
        || (spic_bit_mode == SpicQuadIOMode) 
        || (spic_bit_mode == SpicQpiMode)) {
        phal_spic_adaptor->quad_pin_sel = 1;
    }
    
    if (spic_pinmux_ctl(phal_spic_adaptor, ENABLE) == HAL_OK) {
        if (spic_init_setting(phal_spic_adaptor, spic_bit_mode) != HAL_OK) {
            return HAL_ERR_HW;
        }
    } else {
        return HAL_ERR_PARA;
    }

    hal_flash_support_new_type(phal_spic_adaptor);
    spic_load_calibration_setting(phal_spic_adaptor);

    pspic_data = &phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type];
    dummy_cycle = phal_spic_adaptor->dummy_cycle;
    default_dummy_cycle = *(((u8 *)dummy_cycle) + spic_bit_mode);

    if (pspic_data->valid == 0) {
        spic_config_auto_mode(phal_spic_adaptor);
    }

    /*STR TX do not have to tx calibration*/
    if (spic_calibration(phal_spic_adaptor, default_dummy_cycle) != _TRUE) {
        DBG_SPIF_ERR("spic_init : Calibration Fail, switch back to one bit mode!\r\n");
        hal_flash_return_spi(phal_spic_adaptor);
    }

    /*Disable auto mode write relevant commands*/
    spic_dev = phal_spic_adaptor->spic_dev;
    spic_disable_rtl8710c(spic_dev);
    spic_dev->write_single = 0x0;
    spic_dev->write_dual_data = 0x0;
    spic_dev->write_dual_addr_data = 0x0;
    spic_dev->write_quad_data = 0x0;
    spic_dev->write_quad_addr_data = 0x0;
    spic_dev->write_enable = 0x0;
    spic_enable_rtl8710c(spic_dev);

    /*Set user relevant parameters according to bit mode*/
    spic_config_user_mode(phal_spic_adaptor);
    pglob_spic_adaptor = phal_spic_adaptor;

    return HAL_OK;
}

/** \brief Description of spic_deinit
 *
 *    spic_deinit is used to de-initialize flash controller setting when flash is no longer used.
 *    Pinmux, function enable and clock are disabled in the function.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t spic_deinit(phal_spic_adaptor_t phal_spic_adaptor)
{
    spic_pinmux_ctl(phal_spic_adaptor, DISABLE);

    /*Disable Flash clock*/
    spic_clock_ctrl(DISABLE);

    return HAL_OK;
}

/** \brief Description of spic_calibration
 *
 *    spic_calibration is used to calibrate the path delay of flash data access path.
 *    It ensures the flash controller can read correct data by inserting few delay.
 *    The level of delays depends on the operating frequency and flash IO mode.
 *    If an optimal available window is found, the setting is stored in flash so that flash controller directly load the setting next time.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash adaptor.
 *   \param u32 default_dummy_cycle:      The default dummy cycle defined by flash spec.
 *
 *   \return BOOL: _TURE: Success, _FALSE: No available window, calibration unsuccessful.
 */
BOOL spic_calibration(phal_spic_adaptor_t phal_spic_adaptor, u32 default_dummy_cycle)
{
    SPIC_Type *spic_dev  = phal_spic_adaptor->spic_dev;
    pspic_init_para_t pspic_init_para;
    u32 rd_data = 0;
    u32 auto_len = 0;
    u32 tmp_str_pt = 0;
    u32 tmp_end_pt = 0;
    u32 tmp_max_wd = 0;
    u32 tmp_max_str = 0;
    u32 tmp_max_end = 0;
    u32 last_pass = 0;
    u8 baudr = 0;
    u8 min_baud_rate = 0;
    u8 cpu_type = spic_query_system_clk();
    u8 spic_bit_mode = phal_spic_adaptor->spic_bit_mode;

    pspic_init_para = &(phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type]);

    if (pspic_init_para->valid) {
        spic_disable_rtl8710c(spic_dev);
        spic_set_baudr_rtl8710c(spic_dev, pspic_init_para->baud_rate);
        spic_set_fbaudr_rtl8710c(spic_dev, pspic_init_para->baud_rate);
        spic_set_dummy_cycle_rtl8710c(spic_dev,pspic_init_para->rd_dummy_cycle);
        spic_enable_rtl8710c(spic_dev);

        if (spic_verify_calibration_para() == _TRUE) {
            return _TRUE;
        }
    }

    DBG_SPIF_INFO("Start Flash Calibration\r\n");

    if (spic_bit_mode == SpicOneIOMode) {
        min_baud_rate = 0x4;
    } else {
        min_baud_rate = MIN_BAUD_RATE;
    }
    
    for (baudr = min_baud_rate; baudr <= MAX_BAUD_RATE; baudr++) {
        spic_disable_rtl8710c(spic_dev);
        spic_set_baudr_rtl8710c(spic_dev, baudr);
        spic_set_fbaudr_rtl8710c(spic_dev, baudr);
        spic_enable_rtl8710c(spic_dev);

        for (auto_len = (default_dummy_cycle*baudr*2); auto_len < (default_dummy_cycle*baudr*2 + MAX_AUTO_LENGTH); auto_len++) {
            spic_disable_rtl8710c(spic_dev);
            spic_set_dummy_cycle_rtl8710c(spic_dev,auto_len);
            spic_enable_rtl8710c(spic_dev);

            if (spic_verify_calibration_para() == _TRUE) {
                if (last_pass == 0) {
                    tmp_str_pt = auto_len;
                } else {
                    if (auto_len == (default_dummy_cycle*baudr*2 + MAX_AUTO_LENGTH) - 1) {
                        tmp_end_pt = auto_len;
                        if (tmp_max_wd < (tmp_end_pt - tmp_str_pt)) {
                            tmp_max_str = tmp_str_pt;
                            tmp_max_end = tmp_end_pt;
                            tmp_max_wd = tmp_end_pt - tmp_str_pt;
                            DBG_SPIF_WARN("tmp_max_str = %x, tmp_max_end = %x\r\n", tmp_max_str, tmp_max_end);
                        }
                    }
                }
                
                last_pass = 1;
            } else {
                if (last_pass) {
                    tmp_end_pt = auto_len;
                    if (tmp_max_wd == 0) {
                        tmp_max_str = tmp_str_pt;
                        tmp_max_end = tmp_end_pt;
                        tmp_max_wd = tmp_max_end - tmp_max_str;
                        DBG_SPIF_WARN("tmp_max_str = %x, tmp_max_end = %x\r\n", tmp_max_str, tmp_max_end);
                    } else {
                        if (tmp_max_wd < (tmp_end_pt - tmp_str_pt)) {
                            tmp_max_str = tmp_str_pt;
                            tmp_max_end = tmp_end_pt;
                            tmp_max_wd = tmp_end_pt - tmp_str_pt;
                            DBG_SPIF_WARN("tmp_max_str = %x, tmp_max_end = %x\r\n", tmp_max_str, tmp_max_end);
                        }
                    }
                }
                
                last_pass = 0;
            }

        }

        if (tmp_max_wd) {
            DBG_SPIF_INFO("Find the avaiable window\r\n");
            break;
        }
    }

    if (tmp_max_wd == 0) {
        return _FALSE;
    } else {
        DBG_SPIF_WARN("Baud:%x; dc start:%x; dc end:%x\n",baudr, tmp_max_str, tmp_max_end);

        /* Disable SPI_FLASH User Mode */
        spic_disable_rtl8710c(spic_dev);

        /* Set Baud Rate*/
        spic_set_baudr_rtl8710c(spic_dev, baudr);
        spic_set_fbaudr_rtl8710c(spic_dev, baudr);
        pspic_init_para->baud_rate = baudr;

        /* Set Dummy Cycle*/
        rd_data = (tmp_max_str + tmp_max_end) >> 1;
        spic_set_dummy_cycle_rtl8710c(spic_dev, rd_data);
        pspic_init_para->rd_dummy_cycle = rd_data;

        /* Mark as valid*/
        pspic_init_para->valid = 1;

        /* Enable SPI_FLASH User Mode */
        spic_enable_rtl8710c(spic_dev);

        spic_store_calibration_setting(phal_spic_adaptor);

        return _TRUE;
    }
}

#if 0
BOOL spic_calibration(phal_spic_adaptor_t phal_spic_adaptor, u32 default_dummy_cycle)
{
    SPIC_Type *spic_dev  = phal_spic_adaptor->spic_dev;
    pspic_init_para_t pspic_init_para;
    valid_windows_t max_wd;
    u32 rd_data = 0;
    u32 auto_len = 0;
    u32 dly_line = 0;
    u32 total_ava_wds = 0;
    u32 tmp_str_pt = 0;
    u32 tmp_end_pt = 0;
    u32 last_pass = 0;
    u8  baudr = 0;
    u8 min_baud_rate = 0;
    u8 cpu_type = spic_query_system_clk();
    u8 spic_bit_mode = phal_spic_adaptor->spic_bit_mode;

    pspic_init_para = &(phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type]);

    if (pspic_init_para->valid) {
        spic_disable_rtl8710c(spic_dev);
        spic_set_baudr_rtl8710c(spic_dev, pspic_init_para->baud_rate);
        spic_set_fbaudr_rtl8710c(spic_dev, pspic_init_para->baud_rate);
        spic_set_dummy_cycle_rtl8710c(spic_dev,pspic_init_para->rd_dummy_cycle);
        spic_set_delay_line(pspic_init_para->delay_line);
        spic_enable_rtl8710c(spic_dev);

        if (spic_verify_calibration_para() == _TRUE) {
            return _TRUE;
        }
    }

    DBG_SPIF_INFO("Start Flash Calibration\r\n");

    max_wd.auto_length = 0;
    max_wd.baud_rate = 0;
    max_wd.dly_line_ep = 0;
    max_wd.dly_line_sp = 0;

    if (spic_bit_mode == SpicOneIOMode) {
        min_baud_rate = 0x4;
    } else {
        min_baud_rate = MIN_BAUD_RATE;
    }
    
    for (baudr = min_baud_rate; baudr <= MAX_BAUD_RATE; baudr++) {
        spic_disable_rtl8710c(spic_dev);
        spic_set_baudr_rtl8710c(spic_dev, baudr);
        spic_set_fbaudr_rtl8710c(spic_dev, baudr);
        spic_enable_rtl8710c(spic_dev);

        for (auto_len = (default_dummy_cycle*baudr*2) + 2; auto_len < (default_dummy_cycle*baudr*2 + MAX_AUTO_LENGTH); auto_len++) {
            spic_disable_rtl8710c(spic_dev);
            spic_set_dummy_cycle_rtl8710c(spic_dev,auto_len);
            spic_enable_rtl8710c(spic_dev);
            tmp_str_pt = MAX_DELAY_LINE;
            tmp_end_pt = 0;
            last_pass = 0;

            for (dly_line = 0; dly_line <= MAX_DELAY_LINE; dly_line++) {
                spic_set_delay_line(dly_line);

                if (spic_verify_calibration_para() == _TRUE) {
                    if (last_pass == 0) {
                        tmp_str_pt = dly_line;
                        total_ava_wds++;
                    }

                    if (dly_line == MAX_DELAY_LINE) {
                        tmp_end_pt = dly_line;

                        if (total_ava_wds == 1) {
                            max_wd.baud_rate = baudr;
                            max_wd.auto_length = auto_len;
                            max_wd.dly_line_sp = tmp_str_pt;
                            max_wd.dly_line_ep = tmp_end_pt;
                        } else {
                            if ((tmp_end_pt - tmp_str_pt) > (max_wd.dly_line_ep - max_wd.dly_line_sp)) {
                                max_wd.baud_rate = baudr;
                                max_wd.auto_length = auto_len;
                                max_wd.dly_line_sp = tmp_str_pt;
                                max_wd.dly_line_ep = tmp_end_pt;
                            }
                        }
                    }

                    last_pass = 1;
                } else {
                    if (last_pass == 1) {
                        tmp_end_pt = dly_line;

                        if (total_ava_wds == 1) {
                            max_wd.baud_rate = baudr;
                            max_wd.auto_length = auto_len;
                            max_wd.dly_line_sp = tmp_str_pt;
                            max_wd.dly_line_ep = tmp_end_pt;
                        } else {
                            if((tmp_end_pt - tmp_str_pt) > (max_wd.dly_line_ep - max_wd.dly_line_sp)) {
                                max_wd.baud_rate = baudr;
                                max_wd.auto_length = auto_len;
                                max_wd.dly_line_sp = tmp_str_pt;
                                max_wd.dly_line_ep = tmp_end_pt;
                            }
                        }
                    }

                    last_pass = 0;
                }
            }

            if ((max_wd.dly_line_ep - max_wd.dly_line_sp) == MAX_DELAY_LINE) {
                break;
            }

            DBG_SPIF_INFO("total wds: %d\r\n",total_ava_wds);
            DBG_SPIF_INFO("Baud:%x; auto_length:%x; Delay start:%x; Delay end:%x\n",max_wd.baud_rate, max_wd.auto_length,max_wd.dly_line_sp, max_wd.dly_line_ep);
        }

        if (total_ava_wds) {
            DBG_SPIF_INFO("Find the avaiable window\r\n");
            break;
        }
    }

    if (total_ava_wds == 0) {
        return _FALSE;
    } else {
        DBG_SPIF_WARN("Baud:%x; auto_length:%x; Delay start:%x; Delay end:%x\n",max_wd.baud_rate, max_wd.auto_length,max_wd.dly_line_sp, max_wd.dly_line_ep);

        /* Disable SPI_FLASH User Mode */
        spic_disable_rtl8710c(spic_dev);

        /* Set Baud Rate*/
        spic_set_baudr_rtl8710c(spic_dev, max_wd.baud_rate);
        spic_set_fbaudr_rtl8710c(spic_dev, max_wd.baud_rate);
        pspic_init_para->baud_rate = max_wd.baud_rate;

        /* Set Dummy Cycle*/
        spic_set_dummy_cycle_rtl8710c(spic_dev, max_wd.auto_length);
        pspic_init_para->rd_dummy_cycle = max_wd.auto_length;

        /* Set Delay Line*/
        rd_data = ((max_wd.dly_line_sp + max_wd.dly_line_ep) >> 1);
        spic_set_delay_line(rd_data);
        pspic_init_para->delay_line = rd_data;

        /* Mark as valid*/
        pspic_init_para->valid = 1;

        /* Enable SPI_FLASH User Mode */
        spic_enable_rtl8710c(spic_dev);

        spic_store_calibration_setting(phal_spic_adaptor);

        return _TRUE;
    }
}
#endif

/** \brief Description of spic_load_calibration_setting
 *
 *    spic_load_calibration_setting is used to load calibration settings from flash to the adaptor.
 *    Flash controller will try to load available calibration setting so that it does not have to calibrate.
 *    It takes less time to boot up system without flash calibration process.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void spic_load_calibration_setting(phal_spic_adaptor_t phal_spic_adaptor)
{
    u8 spic_bit_mode = phal_spic_adaptor->spic_bit_mode;
    u8 cpu_type = spic_query_system_clk();
    SPIC_Type *spic_dev  = phal_spic_adaptor->spic_dev;
    pspic_init_para_t pspic_data;
    u32 spic_data;
    u32 spic_data_inv;
    u32 data_offset;

    /*The last 4 byte is used to check the integrity of data*/
    data_offset = cpu_type * 48 + spic_bit_mode * 8;

    phal_spic_adaptor->addr_byte_num = ThreeBytesLength;

    spic_disable_rtl8710c(spic_dev);
    spic_dev->auto_length_b.auto_addr_length = phal_spic_adaptor->addr_byte_num;
    spic_data = HAL_READ32(SPI_FLASH_BASE, SPIC_DATA_BASE + data_offset);
    spic_data_inv = HAL_READ32(SPI_FLASH_BASE, SPIC_DATA_BASE + data_offset + 4);

    if (spic_data != 0xFFFFFFFF) {
        if (0xFFFFFFFF == (spic_data^spic_data_inv)) {
            pspic_data = (pspic_init_para_t)&spic_data;
            spic_disable_rtl8710c(spic_dev);
            spic_set_baudr_rtl8710c(spic_dev, pspic_data->baud_rate);
            spic_set_fbaudr_rtl8710c(spic_dev, pspic_data->baud_rate);
            spic_set_dummy_cycle_rtl8710c(spic_dev, pspic_data->rd_dummy_cycle);
            spic_set_delay_line(pspic_data->delay_line);
            spic_config_auto_mode(phal_spic_adaptor);

            if (spic_verify_calibration_para() == _TRUE) {
                phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].baud_rate = pspic_data->baud_rate;
                phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].rd_dummy_cycle = pspic_data->rd_dummy_cycle;
                phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].delay_line = pspic_data->delay_line;
                phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].valid = pspic_data->valid;
                DBG_SPIF_WARN("Bit mode %d, Calibration setting loaded from flash. 0x%x\r\n", spic_bit_mode, spic_data);
            } else {
                hal_flash_return_spi(phal_spic_adaptor);
                DBG_SPIF_ERR("Bit mode %d setting cannot work, switch back to one IO mode. 0x%x\n",spic_bit_mode, spic_data);
            }
        } else {
            DBG_SPIF_WARN("spic_load_calibration_setting: Data in Flash(@ 0x%x = 0x%x 0x%x) is Invalid\r\n",
                (SPIC_DATA_BASE+data_offset), spic_data, spic_data_inv);
        }
    }
}

/** \brief Description of spic_store_calibration_setting
 *
 *    spic_store_calibration_setting is used to store valid calibration setting to flash after calibration process.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash adaptor.
 *
 *   \return void.
 */
void spic_store_calibration_setting(phal_spic_adaptor_t phal_spic_adaptor)
{
    u8 spic_bit_mode = phal_spic_adaptor->spic_bit_mode;
    u8 cpu_type = spic_query_system_clk();
    pspic_init_para_t pspic_data;
    u32 data_offset;
    u32 spic_data[2];

    data_offset = cpu_type * 48 + spic_bit_mode * 8;

    spic_data[0] = HAL_READ32(SPI_FLASH_BASE, SPIC_DATA_BASE + data_offset);

    if (spic_data[0] == 0xFFFFFFFF) {
        pspic_data = (pspic_init_para_t) &spic_data[0];
        pspic_data->baud_rate = phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].baud_rate;
        pspic_data->rd_dummy_cycle = phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].rd_dummy_cycle;
        pspic_data->delay_line = phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].delay_line;
        pspic_data->valid = phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type].valid;
        spic_data[1] = ~spic_data[0];
        hal_flash_page_program(phal_spic_adaptor, 8, (u32)(SPIC_DATA_BASE + data_offset), (u8 *)&spic_data[0]);
        DBG_SPIF_WARN("spic_store_calibration_setting: Wr=%x\r\n", spic_data[0]);
    } else {
        DBG_SPIF_ERR("spic_store_calibration_setting: The flash memory(@0x%x = 0x%x) cannot be programmed, Erase it first!!\r\n",
            (SPIC_DATA_BASE + data_offset), spic_data[0]);
    }
}


/** \brief Description of spic_store_setting
 *
 *    spic_store_setting is used to store flash adaptor setting.
 *    The information is used to recover flash from reset state or system sleep.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash adaptor.
 *   \param phal_spic_restore_setting_t phal_spic_setting:      The pointer of data structure storing recovery information.
 *
 *   \return void.
 */
void spic_store_setting(phal_spic_adaptor_t phal_spic_adaptor, phal_spic_restore_setting_t phal_spic_setting)
{
    u8 cpu_type = spic_query_system_clk();
    u8 spic_bit_mode = phal_spic_adaptor->spic_bit_mode;

    phal_spic_setting->spic_bit_mode = spic_bit_mode;
    //phal_spic_setting->flash_pin_sel = phal_spic_adaptor->flash_pin_sel;
    rt_memcpy(&phal_spic_setting->flash_pin_sel, &phal_spic_adaptor->flash_pin_sel, sizeof(phal_spic_setting->flash_pin_sel));
    phal_spic_setting->flash_type = phal_spic_adaptor->flash_type;
    //phal_spic_setting->spic_init_data = phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type];
    rt_memcpy(&phal_spic_setting->spic_init_data, &phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type], sizeof(phal_spic_setting->spic_init_data));
    phal_spic_setting->cmd = phal_spic_adaptor->cmd;
    phal_spic_setting->quad_pin_sel = phal_spic_adaptor->quad_pin_sel;
    phal_spic_setting->recored = SPIC_RESTORE_VALID;
}

/** \brief Description of spic_recover_setting
 *
 *    spic_recover_setting is used to load flash adaptor setting.
 *    After the syetem wakes up from sleep state or recovers from reset, the flash controller registers are also reseted.
 *    This function re-initialize the flash controller with the information stored before.
 *
 *   \param phal_spic_adaptor_t phal_spic_adaptor:      The pointer of the flash adaptor.
 *   \param phal_spic_restore_setting_t phal_spic_setting:      The pointer of data structure storing recovery information.
 *
 *   \return void.
 */
void spic_recover_setting(phal_spic_adaptor_t phal_spic_adaptor, phal_spic_restore_setting_t phal_spic_setting)
{
    SPIC_Type *spic_dev;
    pspic_init_para_t pspic_init_data = NULL;
    u8 cpu_type = spic_query_system_clk();
    u8 spic_bit_mode = phal_spic_setting->spic_bit_mode;

    phal_spic_adaptor->spic_bit_mode = spic_bit_mode;
    //phal_spic_adaptor->flash_pin_sel = phal_spic_setting->flash_pin_sel;
    rt_memcpy(&phal_spic_adaptor->flash_pin_sel, &phal_spic_setting->flash_pin_sel, sizeof(phal_spic_setting->flash_pin_sel));
    phal_spic_adaptor->flash_type = phal_spic_setting->flash_type;
    //phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type] = (phal_spic_setting->spic_init_data);
    rt_memcpy(&phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type], &phal_spic_setting->spic_init_data, sizeof(phal_spic_setting->spic_init_data));
    phal_spic_adaptor->cmd = phal_spic_setting->cmd;
    phal_spic_adaptor->quad_pin_sel = phal_spic_setting->quad_pin_sel;

    /*Turn ON spic clk*/
    spic_clock_ctrl(ENABLE);

    /*Enable Spic Rx after Spic is ready*/
    hal_syson_spic_dev_ctrl(ENABLE);

    spic_pinmux_ctl(phal_spic_adaptor, ENABLE);

    phal_spic_adaptor->spic_dev = SPIC;
    spic_dev = phal_spic_adaptor->spic_dev;

    /*Initial Chnl mode*/
    spic_dev->ctrlr0 = 0;

    /*Set slave select*/
    spic_dev->ser = 1;

    /*Set dma mode*/
    spic_dev->dmacr = 0;

    /*Disable Interrupt*/
    spic_dev->imr = 0;
    
    /*Disable PRM mode to prevent from flash entering this mode unexpectedly*/
    spic_dev->read_quad_addr_data_b.prm_value = 0x00;

    pspic_init_data = &(phal_spic_adaptor->spic_init_data[spic_bit_mode][cpu_type]);

    spic_set_dummy_cycle_rtl8710c(spic_dev, pspic_init_data->rd_dummy_cycle);
    spic_set_baudr_rtl8710c(spic_dev, pspic_init_data->baud_rate);
    spic_set_delay_line(pspic_init_data->delay_line);

    spic_config_user_mode(phal_spic_adaptor);
    hal_flash_release_from_power_down(phal_spic_adaptor);
    hal_flash_return_spi(phal_spic_adaptor);
    spic_pinmux_ctl(phal_spic_adaptor, DISABLE);
    spic_init(phal_spic_adaptor, spic_bit_mode, (pflash_pin_sel_t)&(phal_spic_setting->flash_pin_sel));
}

/** *@} */ /* End of group hs_hal_spic_ram_func */

/** *@} */ /* End of group hs_hal_spic */
#endif  // end of "#if defined(CONFIG_SPI_FLASH_EN) && (CONFIG_SPI_FLASH_EN == 1)"
