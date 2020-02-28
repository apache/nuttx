/************************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13x2_cc26xx2_v1_rom.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of TI's rom.h file which has a fully compatible BSD license:
 *
 *    Copyright (c) 2015-2017, Texas Instruments Incorporated
 *    All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_CC13XX_CC13X2_CC26X2_V1_ROM_H
#define __ARCH_ARM_SRC_TIVA_CC13XX_CC13X2_CC26X2_V1_ROM_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Start address of the ROM hard API access table (located after the ROM FW rev
 * field)
 */

#define ROM_HAPI_TABLE_ADDR 0x10000048

/* Pointer to the ROM HAPI table */

#define P_HARD_API                     ((hard_api_t *)ROM_HAPI_TABLE_ADDR)

#define hapi_crc32(a,b,c)              P_HARD_API->crc32(a,b,c)
#define hapi_get_flashsize()           P_HARD_API->get_flashsize()
#define hapi_get_chipid()              P_HARD_API->get_chipid()
#define hapi_reset_device()            P_HARD_API->reset_device()
#define hapi_fletcher32(a,b,c)         P_HARD_API->fletcher32(a,b,c)
#define hapi_min(a,b)                  P_HARD_API->min(a,b)
#define hapi_max(a,b)                  P_HARD_API->max(a,b)
#define hapi_mean(a,b)                 P_HARD_API->mean(a,b)
#define hapi_standard_devation(a,b)    P_HARD_API->standard_devation(a,b)
#define hapi_source_safeswitch()       P_HARD_API->hf_source_safeswitch()
#define hapi_select_compa_input(a)     P_HARD_API->select_compa_input(a)
#define hapi_select_compa_ref(a)       P_HARD_API->select_compa_ref(a)
#define hapi_select_adc_compb_input(a) P_HARD_API->select_adc_compb_input(a)
#define hapi_select_dac_vref(a)        P_HARD_API->select_dac_vref(a)

/* Defines for input parameter to the hapi_select_compa_input function.
 * The define values can not be changed!
 */

#define COMPA_IN_NC            0x00
#define COMPA_IN_AUXIO7        0x09
#define COMPA_IN_AUXIO6        0x0a
#define COMPA_IN_AUXIO5        0x0b
#define COMPA_IN_AUXIO4        0x0c
#define COMPA_IN_AUXIO3        0x0d
#define COMPA_IN_AUXIO2        0x0e
#define COMPA_IN_AUXIO1        0x0f
#define COMPA_IN_AUXIO0        0x10

/* Defines for input parameter to the hapi_select_compa_ref function.
 * The define values can not be changed!
 */

#define COMPA_REF_NC           0x00
#define COMPA_REF_DCOUPL       0x01
#define COMPA_REF_VSS          0x02
#define COMPA_REF_VDDS         0x03
#define COMPA_REF_ADCVREFP     0x04
#define COMPA_REF_AUXIO7       0x09
#define COMPA_REF_AUXIO6       0x0a
#define COMPA_REF_AUXIO5       0x0b
#define COMPA_REF_AUXIO4       0x0c
#define COMPA_REF_AUXIO3       0x0d
#define COMPA_REF_AUXIO2       0x0e
#define COMPA_REF_AUXIO1       0x0f
#define COMPA_REF_AUXIO0       0x10

/* Defines for input parameter to the hapi_select_adc_compb_input function.
 * The define values can not be changed!
 */

#define ADC_COMPB_IN_NC        0x00
#define ADC_COMPB_IN_DCOUPL    0x03
#define ADC_COMPB_IN_VSS       0x04
#define ADC_COMPB_IN_VDDS      0x05
#define ADC_COMPB_IN_AUXIO7    0x09
#define ADC_COMPB_IN_AUXIO6    0x0a
#define ADC_COMPB_IN_AUXIO5    0x0b
#define ADC_COMPB_IN_AUXIO4    0x0c
#define ADC_COMPB_IN_AUXIO3    0x0d
#define ADC_COMPB_IN_AUXIO2    0x0e
#define ADC_COMPB_IN_AUXIO1    0x0f
#define ADC_COMPB_IN_AUXIO0    0x10

/* Defines for input parameter to the hapi_select_dac_vref function.
 * The define values can not be changed!
 */

#define DAC_REF_NC             0x00
#define DAC_REF_DCOUPL         0x01
#define DAC_REF_VSS            0x02
#define DAC_REF_VDDS           0x03

/* Pointers to the main API tables */

#define ROM_API_TABLE           ((uint32_t *) 0x10000180)
#define ROM_VERSION             (ROM_API_TABLE[0])

#define ROM_API_AON_EVENT_TABLE  ((uint32_t *)(ROM_API_TABLE[1]))
#define ROM_API_AON_IOC_TABLE    ((uint32_t *)(ROM_API_TABLE[2]))
#define ROM_API_AON_RTC_TABLE    ((uint32_t *)(ROM_API_TABLE[3]))
#define ROM_API_AUX_CTRL_TABLE   ((uint32_t *)(ROM_API_TABLE[5]))
#define ROM_API_AUX_TDC_TABLE    ((uint32_t *)(ROM_API_TABLE[6]))
#define ROM_API_DDI_TABLE        ((uint32_t *)(ROM_API_TABLE[9]))
#define ROM_API_FLASH_TABLE      ((uint32_t *)(ROM_API_TABLE[10]))
#define ROM_API_I2C_TABLE        ((uint32_t *)(ROM_API_TABLE[11]))
#define ROM_API_INTERRUPT_TABLE  ((uint32_t *)(ROM_API_TABLE[12]))
#define ROM_API_IOC_TABLE        ((uint32_t *)(ROM_API_TABLE[13]))
#define ROM_API_PRCM_TABLE       ((uint32_t *)(ROM_API_TABLE[14]))
#define ROM_API_SMPH_TABLE       ((uint32_t *)(ROM_API_TABLE[15]))
#define ROM_API_SSI_TABLE        ((uint32_t *)(ROM_API_TABLE[17]))
#define ROM_API_TIMER_TABLE      ((uint32_t *)(ROM_API_TABLE[18]))
#define ROM_API_TRNG_TABLE       ((uint32_t *)(ROM_API_TABLE[19]))
#define ROM_API_UART_TABLE       ((uint32_t *)(ROM_API_TABLE[20]))
#define ROM_API_UDMA_TABLE       ((uint32_t *)(ROM_API_TABLE[21]))
#define ROM_API_VIMS_TABLE       ((uint32_t *)(ROM_API_TABLE[22]))
#define ROM_API_CRYPTO_TABLE     ((uint32_t *)(ROM_API_TABLE[23]))
#define ROM_API_OSC_TABLE        ((uint32_t *)(ROM_API_TABLE[24]))
#define ROM_API_AUX_ADC_TABLE    ((uint32_t *)(ROM_API_TABLE[25]))
#define ROM_API_SYS_CTRL_TABLE   ((uint32_t *)(ROM_API_TABLE[26]))
#define ROM_API_AON_BATMON_TABLE ((uint32_t *)(ROM_API_TABLE[27]))
#define ROM_API_SETUP_ROM_TABLE  ((uint32_t *)(ROM_API_TABLE[28]))
#define ROM_API_I2S_TABLE        ((uint32_t *)(ROM_API_TABLE[29]))
#define ROM_API_PWR_CTRL_TABLE   ((uint32_t *)(ROM_API_TABLE[30]))

/* AON_EVENT FUNCTIONS */

#define rom_aon_set_mcuwakeup_event \
    ((void (*)(uint32_t mcuwuevent, uint32_t eventsrc)) \
    ROM_API_AON_EVENT_TABLE[0])

#define rom_aon_get_mcuwakeup_event \
    ((uint32_t (*)(uint32_t mcuwuevent)) \
    ROM_API_AON_EVENT_TABLE[1])

#define rom_aon_set_mcu_event \
    ((void (*)(uint32_t mcuevent, uint32_t eventsrc)) \
    ROM_API_AON_EVENT_TABLE[4])

#define rom_aon_get_mcu_event \
    ((uint32_t (*)(uint32_t mcuevent)) \
    ROM_API_AON_EVENT_TABLE[5])

/* AON_RTC FUNCTIONS */

#define rom_aon_rtc_get64 \
    ((uint64_t (*)(void)) \
    ROM_API_AON_RTC_TABLE[12])

/* AUX_TDC FUNCTIONS */

#define rom_aux_set_tdc_config \
    ((void (*)(uint32_t base, uint32_t startcondition, uint32_t stopcondition)) \
    ROM_API_AUX_TDC_TABLE[0])

#define rom_aux_tcd_measurement_done \
    ((uint32_t (*)(uint32_t base)) \
    ROM_API_AUX_TDC_TABLE[1])

/* DDI FUNCTIONS */

#define rom_ddi_write16 \
    ((void (*)(uint32_t base, uint32_t regoffset, uint32_t mask, uint32_t wrdata)) \
    ROM_API_DDI_TABLE[0])

#define rom_ddi_bitfield_write16 \
    ((void (*)(uint32_t base, uint32_t regoffset, uint32_t mask, uint32_t shift, uint16_t data)) \
    ROM_API_DDI_TABLE[1])

#define rom_ddi_read16 \
    ((uint16_t (*)(uint32_t base, uint32_t regoffset, uint32_t mask)) \
    ROM_API_DDI_TABLE[2])

#define rom_ddi_bitfield_read16 \
    ((uint16_t (*)(uint32_t base, uint32_t regoffset, uint32_t mask, uint32_t shift)) \
    ROM_API_DDI_TABLE[3])

#define rom_ddi_write32 \
    ((void (*)(uint32_t base, uint32_t regoffset, uint32_t regval)) \
    ROM_API_DDI_TABLE[4])

/* FLASH FUNCTIONS */

#define rom_flash_set_powermode \
    ((void (*)(uint32_t powermode, uint32_t bank_graceperiod, uint32_t pump_graceperiod)) \
    ROM_API_FLASH_TABLE[0])

#define rom_flash_get_powermode \
    ((uint32_t (*)(void)) \
    ROM_API_FLASH_TABLE[1])

#define rom_set_protection \
    ((void (*)(uint32_t sectoraddress, uint32_t protectmode)) \
    ROM_API_FLASH_TABLE[2])

#define rom_get_protection \
    ((uint32_t (*)(uint32_t sectoraddress)) \
    ROM_API_FLASH_TABLE[3])

#define rom_save_protection \
    ((uint32_t (*)(uint32_t sectoraddress)) \
    ROM_API_FLASH_TABLE[4])

#define rom_read_efuserow \
    ((bool (*)(uint32_t *efusedata, uint32_t rowaddress)) \
    ROM_API_FLASH_TABLE[8])

#define rom_disable_writesectors \
    ((void (*)(void)) \
    ROM_API_FLASH_TABLE[9])

/* I2C FUNCTIONS */

#define rom_i2cmaster_init_expclk \
    ((void (*)(uint32_t base, uint32_t i2cclk, bool fast)) \
    ROM_API_I2C_TABLE[0])

#define rom_i2cmaster_err \
    ((uint32_t (*)(uint32_t base)) \
    ROM_API_I2C_TABLE[1])

/* INTERRUPT FUNCTIONS */

#define rom_int_set_prioritygrouping \
    ((void (*)(uint32_t bits)) \
    ROM_API_INTERRUPT_TABLE[0])

#define rom_int_get_prioritygrouping \
    ((uint32_t (*)(void)) \
    ROM_API_INTERRUPT_TABLE[1])

#define rom_int_set_priority \
    ((void (*)(uint32_t interrupt, uint8_t priority)) \
    ROM_API_INTERRUPT_TABLE[2])

#define rom_int_get_priority \
    ((int32_t (*)(uint32_t interrupt)) \
    ROM_API_INTERRUPT_TABLE[3])

#define rom_int_enable \
    ((void (*)(uint32_t interrupt)) \
    ROM_API_INTERRUPT_TABLE[4])

#define rom_int_disable \
    ((void (*)(uint32_t interrupt)) \
    ROM_API_INTERRUPT_TABLE[5])

#define rom_int_set_pending \
    ((void (*)(uint32_t interrupt)) \
    ROM_API_INTERRUPT_TABLE[6])

#define rom_int_get_pending \
    ((bool (*)(uint32_t interrupt)) \
    ROM_API_INTERRUPT_TABLE[7])

#define rom_int_clear_pending \
    ((void (*)(uint32_t interrupt)) \
    ROM_API_INTERRUPT_TABLE[8])

/* IOC FUNCTIONS */

#define rom_iocport_set_configuration \
    ((void (*)(uint32_t ioid, uint32_t portid, uint32_t ioconfig)) \
    ROM_API_IOC_TABLE[0])

#define rom_iocport_get_configuration \
    ((uint32_t (*)(uint32_t ioid)) \
    ROM_API_IOC_TABLE[1])

#define rom_iocio_set_shutdown \
    ((void (*)(uint32_t ioid, uint32_t shutdown)) \
    ROM_API_IOC_TABLE[2])

#define rom_iocio_set_mode \
    ((void (*)(uint32_t ioid, uint32_t iomode)) \
    ROM_API_IOC_TABLE[4])

#define rom_iocio_set_int \
    ((void (*)(uint32_t ioid, uint32_t int, uint32_t edgedet)) \
    ROM_API_IOC_TABLE[5])

#define rom_iocio_set_portpullset \
    ((void (*)(uint32_t ioid, uint32_t pull)) \
    ROM_API_IOC_TABLE[6])

#define rom_iocio_set_hyst \
    ((void (*)(uint32_t ioid, uint32_t hysteresis)) \
    ROM_API_IOC_TABLE[7])

#define rom_iocio_set_input \
    ((void (*)(uint32_t ioid, uint32_t input)) \
    ROM_API_IOC_TABLE[8])

#define rom_iocio_set_slewctrl \
    ((void (*)(uint32_t ioid, uint32_t slewenable)) \
    ROM_API_IOC_TABLE[9])

#define rom_iocio_set_drvstrength \
    ((void (*)(uint32_t ioid, uint32_t iocurrent, uint32_t drvstrength)) \
    ROM_API_IOC_TABLE[10])

#define rom_iocio_set_portid \
    ((void (*)(uint32_t ioid, uint32_t portid)) \
    ROM_API_IOC_TABLE[11])

#define rom_iocint_enable \
    ((void (*)(uint32_t ioid)) \
    ROM_API_IOC_TABLE[12])

#define rom_iocint_disable \
    ((void (*)(uint32_t ioid)) \
    ROM_API_IOC_TABLE[13])

#define rom_iocpintype_gpioinput \
    ((void (*)(uint32_t ioid)) \
    ROM_API_IOC_TABLE[14])

#define rom_iocpintype_gpiooutput \
    ((void (*)(uint32_t ioid)) \
    ROM_API_IOC_TABLE[15])

#define rom_iocpintype_uart \
    ((void (*)(uint32_t base, uint32_t rx, uint32_t tx, uint32_t cts, uint32_t rts)) \
    ROM_API_IOC_TABLE[16])

#define rom_iocpintype_ssimaster \
    ((void (*)(uint32_t base, uint32_t rx, uint32_t tx, uint32_t fss, uint32_t ioid)) \
    ROM_API_IOC_TABLE[17])

#define rom_iocpintype_ssislave \
    ((void (*)(uint32_t base, uint32_t rx, uint32_t tx, uint32_t fss, uint32_t ioid)) \
    ROM_API_IOC_TABLE[18])

#define rom_iocpintype_i2c \
    ((void (*)(uint32_t base, uint32_t data, uint32_t ioid)) \
    ROM_API_IOC_TABLE[19])

#define rom_iocpintype_aux \
    ((void (*)(uint32_t ioid)) \
    ROM_API_IOC_TABLE[21])

/* PRCM FUNCTIONS */

#define rom_prcm_set_clockconfig \
    ((void (*)(uint32_t ui32ClkDiv, uint32_t powermode)) \
    ROM_API_PRCM_TABLE[0])

#define rom_prcm_get_clockconfig \
    ((uint32_t (*)(uint32_t powermode)) \
    ROM_API_PRCM_TABLE[1])

#define rom_prcm_set_audioclockconfig \
    ((void (*)(uint32_t clkconfig, uint32_t samplerate)) \
    ROM_API_PRCM_TABLE[4])

#define rom_prcm_powerdomain_on \
    ((void (*)(uint32_t domains)) \
    ROM_API_PRCM_TABLE[5])

#define rom_prcm_powerdomain_off \
    ((void (*)(uint32_t domains)) \
    ROM_API_PRCM_TABLE[6])

#define rom_prcm_enable_periphrun \
    ((void (*)(uint32_t peripheral)) \
    ROM_API_PRCM_TABLE[7])

#define rom_prcm_disable_periphrun \
    ((void (*)(uint32_t peripheral)) \
    ROM_API_PRCM_TABLE[8])

#define rom_prcm_enable_periphsleep \
    ((void (*)(uint32_t peripheral)) \
    ROM_API_PRCM_TABLE[9])

#define rom_disable_periphsleep \
    ((void (*)(uint32_t peripheral)) \
    ROM_API_PRCM_TABLE[10])

#define rom_prcm_enable_periphdeepsleep \
    ((void (*)(uint32_t peripheral)) \
    ROM_API_PRCM_TABLE[11])

#define rom_prcm_disable_periphdeepsleep \
    ((void (*)(uint32_t peripheral)) \
    ROM_API_PRCM_TABLE[12])

#define rom_prcm_powerdomain_staus \
    ((uint32_t (*)(uint32_t domains)) \
    ROM_API_PRCM_TABLE[13])

#define rom_prcm_deepsleep \
    ((void (*)(void)) \
    ROM_API_PRCM_TABLE[14])

#define rom_prcm_set_audioclockconfigOverride \
    ((void (*)(uint32_t clkconfig, uint32_t mstdiv, uint32_t bitdiv, uint32_t worddiv)) \
    ROM_API_PRCM_TABLE[17])

/* SMPH FUNCTIONS */

#define rom_smph_acquire \
    ((void (*)(uint32_t semaphore)) \
    ROM_API_SMPH_TABLE[0])

/* SSI FUNCTIONS */

#define rom_ssi_set_expclkconfig \
    ((void (*)(uint32_t base, uint32_t ssiclk, uint32_t protocol, uint32_t mode, uint32_t bitrate, uint32_t datawidth)) \
    ROM_API_SSI_TABLE[0])

#define rom_ssi_put_data \
    ((void (*)(uint32_t base, uint32_t data)) \
    ROM_API_SSI_TABLE[1])

#define rom_ssi_put_dataNonBlocking \
    ((int32_t (*)(uint32_t base, uint32_t data)) \
    ROM_API_SSI_TABLE[2])

#define rom_ssi_get \
    ((void (*)(uint32_t base, uint32_t *data)) \
    ROM_API_SSI_TABLE[3])

#define rom_ssi_getNonBlocking \
    ((int32_t (*)(uint32_t base, uint32_t *data)) \
    ROM_API_SSI_TABLE[4])

/* TIMER FUNCTIONS */

#define rom_timer_configure \
    ((void (*)(uint32_t base, uint32_t config)) \
    ROM_API_TIMER_TABLE[0])

#define rom_timer_levelcontrol \
    ((void (*)(uint32_t base, uint32_t timer, bool invert)) \
    ROM_API_TIMER_TABLE[1])

#define rom_timer_stallcontrol \
    ((void (*)(uint32_t base, uint32_t timer, bool stall)) \
    ROM_API_TIMER_TABLE[3])

#define rom_timer_wait_trigcontrol \
    ((void (*)(uint32_t base, uint32_t timer, bool wait)) \
    ROM_API_TIMER_TABLE[4])

#define rom_timer_interval_loadmode \
    ((void (*)(uint32_t base, uint32_t timer, uint32_t mode)) \
    ROM_API_TIMER_TABLE[5])

#define rom_timer_update_matchmode \
    ((void (*)(uint32_t base, uint32_t timer, uint32_t mode)) \
    ROM_API_TIMER_TABLE[6])

/* TRNG FUNCTIONS */

#define rom_trng_configure \
    ((void (*)(uint32_t min_sampespercycle, uint32_t max_sampespercycle, uint32_t clockspercycle)) \
    ROM_API_TRNG_TABLE[0])

#define rom_trng_get_number \
    ((uint32_t (*)(uint32_t word)) \
    ROM_API_TRNG_TABLE[1])

/* UART FUNCTIONS */

#define rom_uart_get_fifolevel \
    ((void (*)(uint32_t base, uint32_t *txlevel, uint32_t *rxlevel)) \
    ROM_API_UART_TABLE[0])

#define rom_uart_set_expclk \
    ((void (*)(uint32_t base, uint32_t uartclk, uint32_t baud, uint32_t config)) \
    ROM_API_UART_TABLE[1])

#define rom_uart_get_expclk \
    ((void (*)(uint32_t base, uint32_t uartclk, uint32_t *baud, uint32_t *config)) \
    ROM_API_UART_TABLE[2])

#define rom_uart_disable \
    ((void (*)(uint32_t base)) \
    ROM_API_UART_TABLE[3])

#define rom_uart_getchar_nonblocking \
    ((int32_t (*)(uint32_t base)) \
    ROM_API_UART_TABLE[4])

#define rom_uart_getchar \
    ((int32_t (*)(uint32_t base)) \
    ROM_API_UART_TABLE[5])

#define rom_uart_putchar_nonblocking \
    ((bool (*)(uint32_t base, uint8_t data)) \
    ROM_API_UART_TABLE[6])

#define rom_uart_putchar \
    ((void (*)(uint32_t base, uint8_t data)) \
    ROM_API_UART_TABLE[7])

/* UDMA FUNCTIONS */

#define rom_udmach_enable_attribute \
    ((void (*)(uint32_t base, uint32_t channum, uint32_t attr)) \
    ROM_API_UDMA_TABLE[0])

#define rom_udmach_disable_attribute \
    ((void (*)(uint32_t base, uint32_t channum, uint32_t attr)) \
    ROM_API_UDMA_TABLE[1])

#define rom_udmach_get_attribute \
    ((uint32_t (*)(uint32_t base, uint32_t channum)) \
    ROM_API_UDMA_TABLE[2])

#define rom_udmach_set_control \
    ((void (*)(uint32_t base, uint32_t channdx, uint32_t control)) \
    ROM_API_UDMA_TABLE[3])

#define rom_udmach_set_transfer \
    ((void (*)(uint32_t base, uint32_t channdx, uint32_t mode, void *srcaddr, void *destaddr, uint32_t xfrsize)) \
    ROM_API_UDMA_TABLE[4])

#define rom_udmach_set_scattergather \
    ((void (*)(uint32_t base, uint32_t channum, uint32_t taskcount, void *tasklist, uint32_t periphsg)) \
    ROM_API_UDMA_TABLE[5])

#define rom_udmach_set_size \
    ((uint32_t (*)(uint32_t base, uint32_t channdx)) \
    ROM_API_UDMA_TABLE[6])

#define rom_udmach_get_mode \
    ((uint32_t (*)(uint32_t base, uint32_t channdx)) \
    ROM_API_UDMA_TABLE[7])

/* VIMS FUNCTIONS */

#define rom_vims_configure \
    ((void (*)(uint32_t base, bool roundrobin, bool prefetch)) \
    ROM_API_VIMS_TABLE[0])

#define rom_vims_set_mode \
    ((void (*)(uint32_t base, uint32_t mode)) \
    ROM_API_VIMS_TABLE[1])

#define rom_vims_get_mode \
    ((uint32_t (*)(uint32_t base)) \
    ROM_API_VIMS_TABLE[2])

#define rom_vims_set_modesafe \
    ((void (*)(uint32_t base, uint32_t newmode, bool blocking)) \
    ROM_API_VIMS_TABLE[3])

/* CRYPTO FUNCTIONS */

#define rom_crypto_aesecb \
    ((uint32_t (*)(uint32_t *msgin, uint32_t *msgout, uint32_t keylocation, bool encrypt, bool brom_int_enable)) \
    ROM_API_CRYPTO_TABLE[0])

#define rom_crypto_aesecbStatus \
    ((uint32_t (*)(void)) \
    ROM_API_CRYPTO_TABLE[1])

#define rom_crypto_encrypt_ccmauth \
    ((uint32_t (*)(bool encrypt, uint32_t authlen, uint32_t *nonce, uint32_t *plaintext, uint32_t plaintextlen, uint32_t *header, uint32_t headerlen, uint32_t keylocation, uint32_t fieldlen, bool brom_int_enable)) \
    ROM_API_CRYPTO_TABLE[3])

#define rom_crypto_encrypt_ccmauthResultGet \
    ((uint32_t (*)(uint32_t ui32TagLength, uint32_t *ccmtag)) \
    ROM_API_CRYPTO_TABLE[4])

#define rom_crypto_encrypt_ccmauthStatus \
    ((uint32_t (*)(void)) \
    ROM_API_CRYPTO_TABLE[5])

#define rom_crypto_decrypt_ccminvauth \
    ((uint32_t (*)(bool decrypt, uint32_t authlen, uint32_t *nonce, uint32_t *ciphertext, uint32_t ciphertextlen, uint32_t *header, uint32_t headerlen, uint32_t keylocation, uint32_t fieldlen, bool brom_int_enable)) \
    ROM_API_CRYPTO_TABLE[6])

#define rom_crypto_decrypt_ccminvauthResultGet \
    ((uint32_t (*)(uint32_t authlen, uint32_t *ciphertext, uint32_t ciphertextlen, uint32_t *ccmtag)) \
    ROM_API_CRYPTO_TABLE[7])

#define rom_crypto_decrypt_ccminvauthStatus \
    ((uint32_t (*)(void)) \
    ROM_API_CRYPTO_TABLE[8])

#define rom_crypto_aescbc \
    ((uint32_t (*)(uint32_t *msgin, uint32_t *msgout, uint32_t msglen, uint32_t *nonce, uint32_t keylocation, bool encrypt, bool brom_int_enable)) \
    ROM_API_CRYPTO_TABLE[9])

#define rom_crypto_aescbcStatus \
    ((uint32_t (*)(void)) \
    ROM_API_CRYPTO_TABLE[10])

#define rom_crypto_disable_dma \
    ((void (*)(uint32_t channels)) \
    ROM_API_CRYPTO_TABLE[11])

#define rom_crypto_enable_dma \
    ((void (*)(uint32_t channels)) \
    ROM_API_CRYPTO_TABLE[12])

/* OSC FUNCTIONS */

#define rom_osc_get_clocksource \
    ((uint32_t (*)(uint32_t srcclk)) \
    ROM_API_OSC_TABLE[0])

#define rom_osc_set_clocksource \
    ((void (*)(uint32_t srcclk, uint32_t osc)) \
    ROM_API_OSC_TABLE[1])

#define rom_hposc_get_relfrequencyoffset \
    ((int32_t (*)(int32_t tempdegC)) \
    ROM_API_OSC_TABLE[2])

#define rom_hposc_convert_relfrequencyoffset_to_rfcoreformat \
    ((int16_t (*)(int32_t hposc_relfreqoffset)) \
    ROM_API_OSC_TABLE[3])

/* AUX_ADC FUNCTIONS */

#define rom_auxadc_adjust_gainoffset \
    ((int32_t (*)(int32_t adcvalue, int32_t gain, int32_t offset)) \
    ROM_API_AUX_ADC_TABLE[0])

#define rom_auxadc_disable \
    ((void (*)(void)) \
    ROM_API_AUX_ADC_TABLE[1])

#define rom_auxadc_disableInputScaling \
    ((void (*)(void)) \
    ROM_API_AUX_ADC_TABLE[2])

#define rom_auxadc_enable_async \
    ((void (*)(uint32_t refsource, uint32_t trigger)) \
    ROM_API_AUX_ADC_TABLE[3])

#define rom_auxadc_enable_sync \
    ((void (*)(uint32_t refsource, uint32_t sampletime, uint32_t trigger)) \
    ROM_API_AUX_ADC_TABLE[4])

#define rom_auxadc_flush_fifo \
    ((void (*)(void)) \
    ROM_API_AUX_ADC_TABLE[5])

#define rom_auxadc_get_adjustmentgain \
    ((int32_t (*)(uint32_t refsource)) \
    ROM_API_AUX_ADC_TABLE[6])

#define rom_auxadc_get_adjustoffset \
    ((int32_t (*)(uint32_t refsource)) \
    ROM_API_AUX_ADC_TABLE[7])

#define rom_auxadc_microvolts_to_value \
    ((int32_t (*)(int32_t fixedrefvoltage, int32_t microvolts)) \
    ROM_API_AUX_ADC_TABLE[8])

#define rom_auxadc_pop_fifo \
    ((uint32_t (*)(void)) \
    ROM_API_AUX_ADC_TABLE[9])

#define rom_auxadc_read_fifo \
    ((uint32_t (*)(void)) \
    ROM_API_AUX_ADC_TABLE[10])

#define rom_auxadc_unadjust_gainoffset \
    ((int32_t (*)(int32_t adcvalue, int32_t gain, int32_t offset)) \
    ROM_API_AUX_ADC_TABLE[11])

#define rom_auxadc_value_to_microvolts \
    ((int32_t (*)(int32_t fixedrefvoltage, int32_t adcvalue)) \
    ROM_API_AUX_ADC_TABLE[12])

/* SYS_CTRL FUNCTIONS */

#define rom_sysctrl_get_resetsource \
    ((uint32_t (*)(void)) \
    ROM_API_SYS_CTRL_TABLE[0])

#define rom_sysctrl_dcdc_voltagecondcontrol \
    ((void (*)(void)) \
    ROM_API_SYS_CTRL_TABLE[1])

/* AON_BATMON FUNCTIONS */

#define rom_aonbatmaon_get_temperatureC \
    ((int32_t (*)(void)) \
    ROM_API_AON_BATMON_TABLE[0])

/* SETUP_ROM FUNCTIONS */

#define rom_setup_coldreset_from_shutdown_cfg2 \
    ((void (*)(uint32_t fcfg1_revision, uint32_t ccfg_modeconf)) \
    ROM_API_SETUP_ROM_TABLE[1])

#define rom_setup_coldreset_from_shutdown_cfg3 \
    ((void (*)(uint32_t ccfg_modeconf)) \
    ROM_API_SETUP_ROM_TABLE[2])

#define rom_setup_get_trim_adcshmodeen \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[3])

#define rom_setup_get_trim_adcshvbufen \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[4])

#define rom_setup_get_trim_ampcompctrl \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[5])

#define rom_setup_get_trim_ampcompth1 \
    ((uint32_t (*)(void)) \
    ROM_API_SETUP_ROM_TABLE[6])

#define rom_setup_get_trim_ampcompth2 \
    ((uint32_t (*)(void)) \
    ROM_API_SETUP_ROM_TABLE[7])

#define rom_setup_get_trim_anabypass_value1 \
    ((uint32_t (*)(uint32_t ccfg_modeconf)) \
    ROM_API_SETUP_ROM_TABLE[8])

#define rom_setup_get_trim_dblrloopfilter_resetvoltage \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[9])

#define rom_setup_get_trim_radc_extcfg \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[10])

#define rom_setup_get_trim_rcosc_lfibiastrim \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[11])

#define rom_setup_get_trim_rcosc_lfrtunectuntrim \
    ((uint32_t (*)(void)) \
    ROM_API_SETUP_ROM_TABLE[12])

#define rom_setup_get_trim_xosc_hfctrl \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[13])

#define rom_setup_get_trim_xosc_hffaststart \
    ((uint32_t (*)(void)) \
    ROM_API_SETUP_ROM_TABLE[14])

#define rom_setup_get_trim_xosc_hfibiastherm \
    ((uint32_t (*)(void)) \
    ROM_API_SETUP_ROM_TABLE[15])

#define rom_setup_get_trim_lfregulator_cmirrwr_ratio \
    ((uint32_t (*)(uint32_t fcfg1_revision)) \
    ROM_API_SETUP_ROM_TABLE[16])

#define rom_setup_aonrtc_subsecinc \
    ((void (*)(uint32_t subsecinc)) \
    ROM_API_SETUP_ROM_TABLE[17])

#define rom_setup_cachemode \
    ((void (*)(void)) \
    ROM_API_SETUP_ROM_TABLE[18])

/* I2S FUNCTIONS */

#define rom_i2s_set_pointer \
    ((void (*)(uint32_t base, bool input, void *nextpointer)) \
    ROM_API_I2S_TABLE[0])

#define rom_i2s_get_samplestamp \
    ((uint32_t (*)(uint32_t base, uint32_t channel)) \
    ROM_API_I2S_TABLE[1])

/* PWR_CTRL FUNCTIONS */

#define rom_powerctrl_set_source \
    ((void (*)(uint32_t powerconfig)) \
    ROM_API_PWR_CTRL_TABLE[0])

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* ROM Hard-API function interface types */

typedef uint32_t (*fptr_crc32_t)              (uint8_t *     /* data        */,\
                                               uint32_t      /* bytecount   */,\
                                               uint32_t      /* repeatcount */);

typedef uint32_t (*fptr_getflsize_t)          (void);

typedef uint32_t (*fptr_getchipid_t)          (void);

typedef uint32_t (*fptr_reserved1_t)          (uint32_t);

typedef uint32_t (*fptr_reserved2_t)          (void);

typedef uint32_t (*fptr_reserved3_t)          (uint8_t *,\
                                               uint32_t,\
                                               uint32_t);

typedef void     (*fptr_resetdev_t)           (void);

typedef uint32_t (*fptr_fletcher32_t)         (uint16_t *    /* data        */,\
                                               uint16_t      /* wordcount   */,\
                                               uint16_t      /* repeatcount */);

typedef uint32_t (*fptr_minval_t)             (uint32_t *    /* data   */,\
                                               uint32_t      /* count  */);

typedef uint32_t (*fptr_maxval_t)             (uint32_t *    /* buffer */,\
                                               uint32_t      /* count  */);

typedef uint32_t (*fptr_meanval_t)            (uint32_t *    /* buffer */,\
                                               uint32_t      /* count  */);

typedef uint32_t (*fptr_stddval_t)            (uint32_t *    /* buffer */,\
                                               uint32_t      /* count  */);

typedef void     (*fptr_hfsourcesafeswitch_t) (void);

typedef void     (*fptr_reserved4_t)          (uint32_t);

typedef void     (*fptr_reserved5_t)          (uint32_t);

typedef void     (*fptr_compain_t)            (uint8_t       /* signal */);

typedef void     (*fptr_comparef_t)           (uint8_t       /* signal */);

typedef void     (*fptr_adccompbin_t)         (uint8_t       /* signal */);

typedef void     (*fptr_dacvref_t)            (uint8_t       /* signal */);

/* ROM Hard-API access table type */

struct hard_api_s
{
  fptr_crc32_t               crc32;
  fptr_getflsize_t           FlashGetSize;
  fptr_getchipid_t           get_chipid;
  fptr_reserved1_t           reserved1;
  fptr_reserved2_t           reserved2;
  fptr_reserved3_t           reserved3;
  fptr_resetdev_t            reset_device;
  fptr_fletcher32_t          fletcher32;
  fptr_minval_t              min;
  fptr_maxval_t              max;
  fptr_meanval_t             mean;
  fptr_stddval_t             standard_devation;
  fptr_reserved4_t           reserved4;
  fptr_reserved5_t           reserved5;
  fptr_hfsourcesafeswitch_t  hf_source_safeswitch;
  fptr_compain_t             select_compa_input;
  fptr_comparef_t            select_compa_ref;
  fptr_adccompbin_t          select_adc_compb_input;
  fptr_dacvref_t             select_dac_vref;
};

typedef struct hard_api_s hard_api_t;

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Name: rom_signextend_vddrtrim
 *
 * Description:
 *   Sign extend the VDDR_TRIM setting (special format ranging from -10 to +21)
 *
 * Input Parameters
 *   vddrtrim - VDDR_TRIM setting
 *
 * Returned Value:
 *  Returns sign extended VDDR_TRIM setting.
 *
 ************************************************************************************/

static inline int32_t rom_signextend_vddrtrim(uint32_t vddrtrim)
{
  /* The VDDR trim value is 5 bits representing the range from -10 to +21
   * (where -10=0x16, -1=0x1F, 0=0x00, 1=0x01 and +21=0x15)
   */

  int32_t signed_vaddrtrim = vddrtrim;
  if (signed_vaddrtrim > 0x15)
    {
      signed_vaddrtrim -= 0x20;
    }

  return signed_vaddrtrim;
}

/************************************************************************************
 * Global Function Prototypes
 ************************************************************************************/

/* ROM functions implemented in FLASH */

void rom_setup_coldreset_from_shutdown_cfg1(uint32_t ccfg_modeconf);
void rom_setup_coldreset_from_shutdown_cfg1(uint32_t ccfg_modeconf);
void rom_setup_stepvaddrtrimto(uint32_t tocode);

#endif /* __ARCH_ARM_SRC_TIVA_CC13XX_CC13X2_CC26X2_V1_ROM_H */
