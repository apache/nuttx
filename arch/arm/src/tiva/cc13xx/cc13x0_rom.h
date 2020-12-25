/************************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13x0_rom.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of TI's setup_rom.h file which has a fully compatible BSD license:
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

#ifndef __ARCH_ARM_SRC_TIVA_CC13XX_CC13X0_ROM_H
#define __ARCH_ARM_SRC_TIVA_CC13XX_CC13X0_ROM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdint.h>
#include <nuttx/irq.h>

#include "hardware/tiva_aux_smph.h"

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

/* REVISIT:  In the TI Driverlib, the following go through a "Safe" layer */

#define hapi_source_safeswitch() \
   rom_hapi_void(P_HARD_API->hf_source_safe_switch)
#define hapi_select_compa_input(a) \
   rom_hapi_auxadiselect(P_HARD_API->select_compa_input,(a))
#define hapi_select_compa_ref(a) \
   rom_hapi_auxadiselect(P_HARD_API->select_compa_ref,(a))
#define hapi_select_adc_compb_input(a) \
   rom_hapi_auxadiselect(P_HARD_API->select_adc_compb_input,(a))
#define hapi_select_adc_compb_ref(a) \
   rom_hapi_auxadiselect(P_HARD_API->select_adc_compb_iref,(a))

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

/* Defines for input parameter to the hapi_select_adc_compb_ref function.
 * The define values can not be changed!
 */

#define COMPB_REF_NC           0x00
#define COMPB_REF_DCOUPL       0x01
#define COMPB_REF_VSS          0x02
#define COMPB_REF_VDDS         0x03

/* Pointers to the main API tables */

#define ROM_API_TABLE           ((uint32_t *) 0x10000180)
#define ROM_VERSION             (ROM_API_TABLE[0])

#define ROM_API_AON_EVENT_TABLE ((uint32_t *) (ROM_API_TABLE[1]))
#define ROM_API_AON_IOC_TABLE   ((uint32_t *) (ROM_API_TABLE[2]))
#define ROM_API_AON_RTC_TABLE   ((uint32_t *) (ROM_API_TABLE[3]))
#define ROM_API_AON_WUC_TABLE   ((uint32_t *) (ROM_API_TABLE[4]))
#define ROM_API_AUX_CTRL_TABLE  ((uint32_t *) (ROM_API_TABLE[5]))
#define ROM_API_AUX_TDC_TABLE   ((uint32_t *) (ROM_API_TABLE[6]))
#define ROM_API_AUX_TIMER_TABLE ((uint32_t *) (ROM_API_TABLE[7]))
#define ROM_API_AUX_WUC_TABLE   ((uint32_t *) (ROM_API_TABLE[8]))
#define ROM_API_DDI_TABLE       ((uint32_t *) (ROM_API_TABLE[9]))
#define ROM_API_FLASH_TABLE     ((uint32_t *) (ROM_API_TABLE[10]))
#define ROM_API_I2C_TABLE       ((uint32_t *) (ROM_API_TABLE[11]))
#define ROM_API_INTERRUPT_TABLE ((uint32_t *) (ROM_API_TABLE[12]))
#define ROM_API_IOC_TABLE       ((uint32_t *) (ROM_API_TABLE[13]))
#define ROM_API_PRCM_TABLE      ((uint32_t *) (ROM_API_TABLE[14]))
#define ROM_API_SMPH_TABLE      ((uint32_t *) (ROM_API_TABLE[15]))
#define ROM_API_SSI_TABLE       ((uint32_t *) (ROM_API_TABLE[17]))
#define ROM_API_TIMER_TABLE     ((uint32_t *) (ROM_API_TABLE[18]))
#define ROM_API_TRNG_TABLE      ((uint32_t *) (ROM_API_TABLE[19]))
#define ROM_API_UART_TABLE      ((uint32_t *) (ROM_API_TABLE[20]))
#define ROM_API_UDMA_TABLE      ((uint32_t *) (ROM_API_TABLE[21]))
#define ROM_API_VIMS_TABLE      ((uint32_t *) (ROM_API_TABLE[22]))

/* AON_EVENT FUNCTIONS */

#define rom_aon_set_mcuwakeup_event \
    ((void (*)(uint32_t mcuwuevent, uint32_t eventsrc)) \
    ROM_API_AON_EVENT_TABLE[0])

#define rom_aon_get_mcuwakeup_event \
    ((uint32_t (*)(uint32_t mcuwuevent)) \
    ROM_API_AON_EVENT_TABLE[1])

#define rom_aonevent_set_auxwakeup \
    ((void (*)(uint32_t auxwuevent, uint32_t eventsrc)) \
    ROM_API_AON_EVENT_TABLE[2])

#define rom_aonevent_get_auxwakeup \
    ((uint32_t (*)(uint32_t auxwuevent)) \
    ROM_API_AON_EVENT_TABLE[3])

#define rom_aon_set_mcu_event \
    ((void (*)(uint32_t mcuevent, uint32_t eventsrc)) \
    ROM_API_AON_EVENT_TABLE[4])

#define rom_aon_get_mcu_event \
    ((uint32_t (*)(uint32_t mcuevent)) \
    ROM_API_AON_EVENT_TABLE[5])

/* AON_WUC FUNCTIONS */

#define rom_aonwuc_reset_aux \
    ((void (*)(void)) \
    ROM_API_AON_WUC_TABLE[3])

#define rom_aonwuc_set_rechargectrl_config \
    ((void (*)(bool adaptenable, uint32_t adaptrate, uint32_t period,\
               uint32_t maxperiod)) \
    ROM_API_AON_WUC_TABLE[4])

#define rom_aonwuc_oscconfig \
    ((void (*)(uint32_t period)) \
    ROM_API_AON_WUC_TABLE[5])

/* AUX_TDC FUNCTIONS */

#define rom_aux_set_tdc_config \
    ((void (*)(uint32_t base, uint32_t startcondition, uint32_t stopcondition)) \
    ROM_API_AUX_TDC_TABLE[0])

#define rom_aux_tcd_measurement_done \
    ((uint32_t (*)(uint32_t base)) \
    ROM_API_AUX_TDC_TABLE[1])

/* AUX_WUC FUNCTIONS */

#define rom_aonwuc_enable_clock \
    ((void (*)(uint32_t clocks)) \
    ROM_API_AUX_WUC_TABLE[0])

#define rom_aonwuc_disable_clock \
    ((void (*)(uint32_t clocks)) \
    ROM_API_AUX_WUC_TABLE[1])

#define rom_aonwuc_status_clock \
    ((uint32_t (*)(uint32_t clocks)) \
    ROM_API_AUX_WUC_TABLE[2])

#define rom_aonwuc_powerctrl \
    ((void (*)(uint32_t powermode)) \
    ROM_API_AUX_WUC_TABLE[3])

/* DDI FUNCTIONS */

#define rom_ddi_write16 \
    ((void (*)(uint32_t base, uint32_t regoffset, uint32_t mask, uint32_t wrdata)) \
    ROM_API_DDI_TABLE[0])

#define rom_ddi_bitfield_write16 \
    ((void (*)(uint32_t base, uint32_t regoffset, uint32_t mask, uint32_t shift, \
               uint16_t data)) \
    ROM_API_DDI_TABLE[1])

#define rom_ddi_read16 \
    ((uint16_t (*)(uint32_t base, uint32_t regoffset, uint32_t mask)) \
    ROM_API_DDI_TABLE[2])

#define rom_ddi_bitfield_read16 \
    ((uint16_t (*)(uint32_t base, uint32_t regoffset, uint32_t mask, uint32_t shift)) \
    ROM_API_DDI_TABLE[3])

/* FLASH FUNCTIONS */

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
    ((void (*)(uint32_t base, uint32_t rx, uint32_t tx, uint32_t fss, uint32_t clk)) \
    ROM_API_IOC_TABLE[17])

#define rom_iocpintype_ssislave \
    ((void (*)(uint32_t base, uint32_t rx, uint32_t tx, uint32_t fss, uint32_t clk)) \
    ROM_API_IOC_TABLE[18])

#define rom_iocpintype_i2c \
    ((void (*)(uint32_t base, uint32_t data, uint32_t clk)) \
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

/* SMPH FUNCTIONS */

#define rom_smph_acquire \
    ((void (*)(uint32_t semaphore)) \
    ROM_API_SMPH_TABLE[0])

/* SSI FUNCTIONS */

#define rom_ssi_set_expclkconfig \
    ((void (*)(uint32_t base, uint32_t ssiclk, uint32_t protocol, uint32_t mode, uint32_t \
               bitrate, uint32_t datawidth)) \
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

/* TRNG FUNCTIONS */

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

/* Defines for the AUX power control.  Inputs to rom_aonwuc_powerctrl.
 * NOTE: These come from the file aux_wuc.h in the TI DriverLib
 */

#define AUX_WUC_POWER_OFF     0x00000001
#define AUX_WUC_POWER_DOWN    0x00000002
#define AUX_WUC_POWER_ACTIVE  0x00000004

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* ROM Hard-API function interface types */

typedef uint32_t (*fptr_crc32_t)              (uint8_t *     /* data        */,
                                               uint32_t      /* bytecount   */,
                                               uint32_t      /* repeatcount */);

typedef uint32_t (*fptr_getflsize_t)          (void);

typedef uint32_t (*fptr_getchipid_t)          (void);

typedef uint32_t (*fptr_reserved1_t)          (uint32_t);

typedef uint32_t (*fptr_reserved2_t)          (void);

typedef uint32_t (*fptr_reserved3_t)          (uint8_t *,
                                               uint32_t ,
                                               uint32_t);

typedef void     (*fptr_resetdev_t)           (void);

typedef uint32_t (*fptr_fletcher32_t)         (uint16_t *    /* data        */,
                                               uint16_t      /* wordcount   */,
                                               uint16_t      /* repeatcount */);

typedef uint32_t (*fptr_minval_t)             (uint32_t *    /* data   */,
                                               uint32_t      /* count  */);

typedef uint32_t (*fptr_maxval_t)             (uint32_t *    /* buffer */,
                                               uint32_t      /* count  */);

typedef uint32_t (*fptr_meanval_t)            (uint32_t *    /* buffer */,
                                               uint32_t      /* count  */);

typedef uint32_t (*fptr_stddval_t)            (uint32_t *    /* buffer */,
                                               uint32_t      /* count  */);

typedef void     (*fptr_hfsourcesafeswitch_t) (void);

typedef void     (*fptr_reserved4_t)          (uint32_t);

typedef void     (*fptr_reserved5_t)          (uint32_t);

typedef void     (*fptr_compain_t)            (uint8_t       /* signal */);

typedef void     (*fptr_comparef_t)           (uint8_t       /* signal */);

typedef void     (*fptr_adccompbin_t)         (uint8_t       /* signal */);

typedef void     (*fptr_compbref_t)           (uint8_t       /* signal */);

/* Types used in the "Safe" interfaces taken from the TI DriverLib hw_types.h */

typedef void     (*fptr_void_void_t)          (void);
typedef void     (*fptr_void_uint8_t)         (uint8_t);

/* ROM Hard-API access table type */

struct hard_api_s
{
  fptr_crc32_t               crc32;
  fptr_getflsize_t           get_flashsize;
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
  fptr_compbref_t            select_adc_compb_ref;
};

typedef struct hard_api_s hard_api_t;

/************************************************************************************
 * Global Function Prototypes
 ************************************************************************************/

/* ROM functions implemented in FLASH */

void     rom_setup_coldreset_from_shutdown_cfg1(uint32_t ccfg_modeconf);
void     rom_setup_coldreset_from_shutdown_cfg2(uint32_t fcfg1_revision,
                                                uint32_t ccfg_modeconf);
void     rom_setup_coldreset_from_shutdown_cfg3(uint32_t ccfg_modeconf);
uint32_t rom_setup_get_trim_anabypass_value1(uint32_t ccfg_modeconf);
uint32_t rom_setup_get_trim_anabypass_value1(uint32_t ccfg_modeconf);
uint32_t rom_setup_get_trim_rcosc_lfrtunectuntrim(void);
uint32_t rom_setup_get_trim_xosc_hfibiastherm(void);
uint32_t rom_setup_get_trim_ampcompth1(void);
uint32_t rom_setup_get_trim_ampcompth2(void);
uint32_t rom_setup_get_trim_ampcompctrl(uint32_t fcfg1_revision);
uint32_t rom_setup_get_trim_dblrloopfilter_resetvoltage(uint32_t fcfg1_revision);
uint32_t rom_setup_get_trim_adcshmodeen(uint32_t fcfg1_revision);
uint32_t rom_setup_get_trim_adcshvbufen(uint32_t fcfg1_revision);
uint32_t rom_setup_get_trim_xosc_hfctrl(uint32_t fcfg1_revision);
uint32_t rom_setup_get_trim_xosc_hffaststart(void);
uint32_t rom_setup_get_trim_radc_extcfg(uint32_t fcfg1_revision);
uint32_t rom_setup_get_trim_rcosc_lfibiastrim(uint32_t fcfg1_revision);
uint32_t rom_setup_get_trim_lfregulator_cmirrwr_ratio(uint32_t fcfg1_revision);
void     rom_setup_cachemode(void);
void     rom_setup_aonrtc_subsecinc(uint32_t subsecinc);

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
 *   Returns sign extended VDDR_TRIM setting.
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
 * Name: rom_hapi_void and rom_hapi_auxadiselect
 *
 * Description:
 *   Work-arounds for bus arbitration issue.
 *   REVISIT:  Originally for the adi.h header file in the TI DriverLib
 *
 * Input Parameters
 *   fptr - Function pointer
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

inline static void rom_hapi_void(fptr_void_void_t fptr)
{
  irqstate_t flags = enter_critical_section();
  while (getreg32(TIVA_AUX_SMPH_SMPH0) == 0)
    {
    }

  fptr();
  putreg32(1, TIVA_AUX_SMPH_SMPH0);
  leave_critical_section(flags);
}

inline static void rom_hapi_auxadiselect(fptr_void_uint8_t fptr, uint8_t signal)
{
  irqstate_t flags = enter_critical_section();
  while (getreg32(TIVA_AUX_SMPH_SMPH0) == 0)
    {
    }

  fptr(signal);
  putreg32(1, TIVA_AUX_SMPH_SMPH0);
  leave_critical_section(flags);
}

#endif /* __ARCH_ARM_SRC_TIVA_CC13XX_CC13X0_ROM_H */
