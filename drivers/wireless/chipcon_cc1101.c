/****************************************************************************
 * drivers/wireless/chipcon_cc1101.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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
 ****************************************************************************/

/** \file
 *  \author Uros Platise
 *  \brief Chipcon CC1101 Device Driver
 * 
 * 
 **/ 
 
#include <nuttx/config.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/chipcon_cc1101.h>


/************************************************************************************
 * Chipcon CC1101 Internal Registers
 ************************************************************************************/

/* Configuration Registers */

#define CC1101_IOCFG2           0x00        /* GDO2 output pin configuration */
#define CC1101_IOCFG1           0x01        /* GDO1 output pin configuration */
#define CC1101_IOCFG0           0x02        /* GDO0 output pin configuration */
#define CC1101_FIFOTHR          0x03        /* RX FIFO and TX FIFO thresholds */
#define CC1101_SYNC1            0x04        /* Sync word, high byte */
#define CC1101_SYNC0            0x05        /* Sync word, low byte */
#define CC1101_PKTLEN           0x06        /* Packet length */
#define CC1101_PKTCTRL1         0x07        /* Packet automation control */
#define CC1101_PKTCTRL0         0x08        /* Packet automation control */
#define CC1101_ADDR             0x09        /* Device address */
#define CC1101_CHANNR           0x0A        /* Channel number */
#define CC1101_FSCTRL1          0x0B        /* Frequency synthesizer control */
#define CC1101_FSCTRL0          0x0C        /* Frequency synthesizer control */
#define CC1101_FREQ2            0x0D        /* Frequency control word, high byte */
#define CC1101_FREQ1            0x0E        /* Frequency control word, middle byte */
#define CC1101_FREQ0            0x0F        /* Frequency control word, low byte */
#define CC1101_MDMCFG4          0x10        /* Modem configuration */
#define CC1101_MDMCFG3          0x11        /* Modem configuration */
#define CC1101_MDMCFG2          0x12        /* Modem configuration */
#define CC1101_MDMCFG1          0x13        /* Modem configuration */
#define CC1101_MDMCFG0          0x14        /* Modem configuration */
#define CC1101_DEVIATN          0x15        /* Modem deviation setting */
#define CC1101_MCSM2            0x16        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM1            0x17        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM0            0x18        /* Main Radio Cntrl State Machine config */
#define CC1101_FOCCFG           0x19        /* Frequency Offset Compensation config */
#define CC1101_BSCFG            0x1A        /* Bit Synchronization configuration */
#define CC1101_AGCCTRL2         0x1B        /* AGC control */
#define CC1101_AGCCTRL1         0x1C        /* AGC control */
#define CC1101_AGCCTRL0         0x1D        /* AGC control */
#define CC1101_WOREVT1          0x1E        /* High byte Event 0 timeout */
#define CC1101_WOREVT0          0x1F        /* Low byte Event 0 timeout */
#define CC1101_WORCTRL          0x20        /* Wake On Radio control */
#define CC1101_FREND1           0x21        /* Front end RX configuration */
#define CC1101_FREND0           0x22        /* Front end TX configuration */
#define CC1101_FSCAL3           0x23        /* Frequency synthesizer calibration */
#define CC1101_FSCAL2           0x24        /* Frequency synthesizer calibration */
#define CC1101_FSCAL1           0x25        /* Frequency synthesizer calibration */
#define CC1101_FSCAL0           0x26        /* Frequency synthesizer calibration */
#define CC1101_RCCTRL1          0x27        /* RC oscillator configuration */
#define CC1101_RCCTRL0          0x28        /* RC oscillator configuration */
#define CC1101_FSTEST           0x29        /* Frequency synthesizer cal control */
#define CC1101_PTEST            0x2A        /* Production test */
#define CC1101_AGCTEST          0x2B        /* AGC test */
#define CC1101_TEST2            0x2C        /* Various test settings */
#define CC1101_TEST1            0x2D        /* Various test settings */
#define CC1101_TEST0            0x2E        /* Various test settings */

/* Status registers */

#define CC1101_PARTNUM          (0x30 | 0xc0)   /* Part number */
#define CC1101_VERSION          (0x31 | 0xc0)   /* Current version number */
#define CC1101_FREQEST          (0x32 | 0xc0)   /* Frequency offset estimate */
#define CC1101_LQI              (0x33 | 0xc0)   /* Demodulator estimate for link quality */
#define CC1101_RSSI             (0x34 | 0xc0)   /* Received signal strength indication */
#define CC1101_MARCSTATE        (0x35 | 0xc0)   /* Control state machine state */
#define CC1101_WORTIME1         (0x36 | 0xc0)   /* High byte of WOR timer */
#define CC1101_WORTIME0         (0x37 | 0xc0)   /* Low byte of WOR timer */
#define CC1101_PKTSTATUS        (0x38 | 0xc0)   /* Current GDOx status and packet status */
#define CC1101_VCO_VC_DAC       (0x39 | 0xc0)   /* Current setting from PLL cal module */
#define CC1101_TXBYTES          (0x3A | 0xc0)   /* Underflow and # of bytes in TXFIFO */
#define CC1101_RXBYTES          (0x3B | 0xc0)   /* Overflow and # of bytes in RXFIFO */
#define CC1101_RCCTRL1_STATUS   (0x3E | 0xc0)   /* Last RC oscilator calibration results */
#define CC1101_RCCTRL0_STATUS   (0x3F | 0xc0)   /* Last RC oscilator calibration results */

/* Multi byte memory locations */

#define CC1101_PATABLE          0x3E
#define CC1101_TXFIFO           0x3F
#define CC1101_RXFIFO           0x3F

/* Definitions for burst/single access to registers */

#define CC1101_WRITE_BURST      0x40
#define CC1101_READ_SINGLE      0x80
#define CC1101_READ_BURST       0xC0

/* Strobe commands */

#define CC1101_SRES             0x30        /* Reset chip. */
#define CC1101_SFSTXON          0x31        /* Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). */
#define CC1101_SXOFF            0x32        /* Turn off crystal oscillator. */
#define CC1101_SCAL             0x33        /* Calibrate frequency synthesizer and turn it off */
#define CC1101_SRX              0x34        /* Enable RX. Perform calibration first if switching from IDLE and MCSM0.FS_AUTOCAL=1. */
#define CC1101_STX              0x35        /* Enable TX. Perform calibration first if IDLE and MCSM0.FS_AUTOCAL=1.  */
                                            /* If switching from RX state and CCA is enabled then go directly to TX if channel is clear. */
#define CC1101_SIDLE            0x36        /* Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. */
#define CC1101_SAFC             0x37        /* Perform AFC adjustment of the frequency synthesizer */
#define CC1101_SWOR             0x38        /* Start automatic RX polling sequence (Wake-on-Radio) */
#define CC1101_SPWD             0x39        /* Enter power down mode when CSn goes high. */
#define CC1101_SFRX             0x3A        /* Flush the RX FIFO buffer. */
#define CC1101_SFTX             0x3B        /* Flush the TX FIFO buffer. */
#define CC1101_SWORRST          0x3C        /* Reset real time clock. */
#define CC1101_SNOP             0x3D        /* No operation. */

/* 
 * Chip Status Byte 
 */

/* Bit fields in the chip status byte */

#define CC1101_STATUS_CHIP_RDYn_BM              0x80
#define CC1101_STATUS_STATE_BM                  0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM   0x0F

/* Chip states */

#define CC1101_STATE_IDLE                       0x00
#define CC1101_STATE_RX                         0x10
#define CC1101_STATE_TX                         0x20
#define CC1101_STATE_FSTXON                     0x30
#define CC1101_STATE_CALIBRATE                  0x40
#define CC1101_STATE_SETTLING                   0x50
#define CC1101_STATE_RX_OVERFLOW                0x60
#define CC1101_STATE_TX_UNDERFLOW               0x70

/* Values of the MACRSTATE register */

#define CC1101_MARCSTATE_SLEEP                  0x00
#define CC1101_MARCSTATE_IDLE                   0x01
#define CC1101_MARCSTATE_XOFF                   0x02
#define CC1101_MARCSTATE_VCOON_MC               0x03
#define CC1101_MARCSTATE_REGON_MC               0x04
#define CC1101_MARCSTATE_MANCAL                 0x05
#define CC1101_MARCSTATE_VCOON                  0x06
#define CC1101_MARCSTATE_REGON                  0x07
#define CC1101_MARCSTATE_STARTCAL               0x08
#define CC1101_MARCSTATE_BWBOOST                0x09
#define CC1101_MARCSTATE_FS_LOCK                0x0A
#define CC1101_MARCSTATE_IFADCON                0x0B
#define CC1101_MARCSTATE_ENDCAL                 0x0C
#define CC1101_MARCSTATE_RX                     0x0D
#define CC1101_MARCSTATE_RX_END                 0x0E
#define CC1101_MARCSTATE_RX_RST                 0x0F
#define CC1101_MARCSTATE_TXRX_SWITCH            0x10
#define CC1101_MARCSTATE_RXFIFO_OVERFLOW        0x11
#define CC1101_MARCSTATE_FSTXON                 0x12
#define CC1101_MARCSTATE_TX                     0x13
#define CC1101_MARCSTATE_TX_END                 0x14
#define CC1101_MARCSTATE_RXTX_SWITCH            0x15
#define CC1101_MARCSTATE_TXFIFO_UNDERFLOW       0x16


/*
 * General Purpose, Test Output Pin Options
 */
 
/* Associated to the RX FIFO: Asserts when RX FIFO is filled at or above 
 * the RX FIFO threshold. De-asserts when RX FIFO is drained below the 
 * same threshold. */
#define CC1101_GDO_RXFIFO_THR                   0x00

/* Associated to the RX FIFO: Asserts when RX FIFO is filled at or above 
 * the RX FIFO threshold or the end of packet is reached. De-asserts when 
 * the RX FIFO is empty. */
#define CC1101_GDO_RXFIFO_THREND                0x01   

/* Associated to the TX FIFO: Asserts when the TX FIFO is filled at or 
 * above the TX FIFO threshold. De-asserts when the TX FIFO is below the 
 * same threshold. */
#define CC1101_GDO_TXFIFO_THR                   0x02

/* Associated to the TX FIFO: Asserts when TX FIFO is full. De-asserts
 * when the TX FIFO is drained below theTX FIFO threshold. */
#define CC1101_GDO_TXFIFO_FULL                  0x03

/* Asserts when the RX FIFO has overflowed. De-asserts when the FIFO has 
 * been flushed. */
#define CC1101_GDO_RXFIFO_OVR                   0x04

/* Asserts when the TX FIFO has underflowed. De-asserts when the FIFO is 
 * flushed. */
#define CC1101_GDO_TXFIFO_UNR                   0x05

/* Asserts when sync word has been sent / received, and de-asserts at the
 * end of the packet. In RX, the pin will de-assert when the optional 
 * address check fails or the RX FIFO overflows. In TX the pin will 
 * de-assert if the TX FIFO underflows. */
#define CC1101_GDO_SYNC                         0x06

/* Asserts when a packet has been received with CRC OK. De-asserts when 
 * the first byte is read from the RX FIFO. */
#define CC1101_GDO_CRCOK                        0x07

/* Preamble Quality Reached. Asserts when the PQI is above the programmed 
 * PQT value. */
#define CC1101_GDO_PREAMBLE                     0x08

/* Clear channel assessment. High when RSSI level is below threshold 
 * (dependent on the current CCA_MODE setting). */
#define CC1101_GDO_CHCLEAR                      0x09

/* Lock detector output. The PLL is in lock if the lock detector output 
 * has a positive transition or is constantly logic high. To check for 
 * PLL lock the lock detector output should be used as an interrupt for 
 * the MCU. */
#define CC1101_GDO_LOCK                         0x0A

/* Serial Clock. Synchronous to the data in synchronous serial mode.
 * In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0.
 * In TX mode, data is sampled by CC1101 on the rising edge of the serial 
 * clock when GDOx_INV=0. */
#define CC1101_GDO_SSCLK                        0x0B

/* Serial Synchronous Data Output. Used for synchronous serial mode. */
#define CC1101_GDO_SSDO                         0x0C

/* Serial Data Output. Used for asynchronous serial mode. */
#define CC1101_GDO_ASDO                         0x0D

/* Carrier sense. High if RSSI level is above threshold. */
#define CC1101_GDO_CARRIER                      0x0E

/* CRC_OK. The last CRC comparison matched. Cleared when entering or 
 * restarting RX mode. */
#define CC1101_GDO_CRCOK2                       0x0F

/* RX_HARD_DATA[1]. Can be used together with RX_SYMBOL_TICK for 
 * alternative serial RX output. */
#define CC1101_GDO_RXOUT1                       0x16
 
/* RX_HARD_DATA[0]. Can be used together with RX_SYMBOL_TICK for 
 * alternative serial RX output. */
#define CC1101_GDO_RXOUT0                       0x17

/* PA_PD. Note: PA_PD will have the same signal level in SLEEP and TX 
 * states. To control an external PA or RX/TX switch in applications 
 * where the SLEEP state is used it is recommended to use GDOx_CFGx=0x2F 
 * instead. */
#define CC1101_GDO_PA_PD                        0x1B

/* LNA_PD. Note: LNA_PD will have the same signal level in SLEEP and RX 
 * states. To control an external LNA or RX/TX switch in applications 
 * where the SLEEP state is used it is recommended to use GDOx_CFGx=0x2F 
 * instead. */
#define CC1101_GDO_LNA_PD                       0x1C

/* RX_SYMBOL_TICK. Can be used together with RX_HARD_DATA for alternative 
 * serial RX output. */
#define CC1101_GDO_RXSYMTICK                    0x1D

#define CC1101_GDO_WOR_EVNT0                    0x24
#define CC1101_GDO_WOR_EVNT1                    0x25
#define CC1101_GDO_CLK32K                       0x27
#define CC1101_GDO_CHIP_RDYn                    0x29
#define CC1101_GDO_XOSC_STABLE                  0x2B

/* GDO0_Z_EN_N. When this output is 0, GDO0 is configured as input 
 * (for serial TX data). */
#define CC1101_GDO_GDO0_Z_EN_N                  0x2D

/* High impedance (3-state). */
#define CC1101_GDO_HIZ                          0x2E

/* HW to 0 (HW1 achieved by setting GDOx_INV=1). Can be used to control 
 * an external LNA/PA or RX/TX switch. */
#define CC1101_GDO_HW                           0x2F

/* There are 3 GDO pins, but only one CLK_XOSC/n can be selected as an 
 * output at any time. If CLK_XOSC/n is to be monitored on one of the 
 * GDO pins, the other two GDO pins must be configured to values less 
 * than 0x30. The GDO0 default value is CLK_XOSC/192. To optimize RF 
 * performance, these signals should not be used while the radio is 
 * in RX or TX mode. */
#define CC1101_GDO_CLK_XOSC1                    0x30
#define CC1101_GDO_CLK_XOSC1_5                  0x31
#define CC1101_GDO_CLK_XOSC2                    0x32
#define CC1101_GDO_CLK_XOSC3                    0x33
#define CC1101_GDO_CLK_XOSC4                    0x34
#define CC1101_GDO_CLK_XOSC6                    0x35
#define CC1101_GDO_CLK_XOSC8                    0x36
#define CC1101_GDO_CLK_XOSC12                   0x37
#define CC1101_GDO_CLK_XOSC16                   0x38
#define CC1101_GDO_CLK_XOSC24                   0x39
#define CC1101_GDO_CLK_XOSC32                   0x3A
#define CC1101_GDO_CLK_XOSC48                   0x3B
#define CC1101_GDO_CLK_XOSC64                   0x3C
#define CC1101_GDO_CLK_XOSC96                   0x3D
#define CC1101_GDO_CLK_XOSC128                  0x3E
#define CC1101_GDO_CLK_XOSC192                  0x3F

/*
 *  Others ... 
 */

#define CC1101_LQI_CRC_OK_BM                    0x80
#define CC1101_LQI_EST_BM                       0x7F

 
/************************************************************************************
 * Private Data Types
 ************************************************************************************/

struct chipcon_c1101_rfsettings_s {
	uint8_t FSCTRL1;    /* Frequency synthesizer control. */
	uint8_t IOCFG0;     /* GDO0 output pin configuration */
	uint8_t FSCTRL0;    /* Frequency synthesizer control. */
	uint8_t FREQ2;      /* Frequency control word, high byte. */
	uint8_t FREQ1;      /* Frequency control word, middle byte. */
	uint8_t FREQ0;      /* Frequency control word, low byte. */
	uint8_t MDMCFG4;    /* Modem configuration. */
	uint8_t MDMCFG3;    /* Modem configuration. */
	uint8_t MDMCFG2;    /* Modem configuration. */
	uint8_t MDMCFG1;    /* Modem configuration. */
	uint8_t MDMCFG0;    /* Modem configuration. */
	uint8_t CHANNR;     /* Channel number. */
	uint8_t DEVIATN;    /* Modem deviation setting (when FSK modulation is enabled). */
	uint8_t FREND1;     /* Front end RX configuration. */
	uint8_t FREND0;     /* Front end RX configuration. */
	uint8_t MCSM0;      /* Main Radio Control State Machine configuration. */
	uint8_t FOCCFG;     /* Frequency Offset Compensation Configuration. */
	uint8_t BSCFG;      /* Bit synchronization Configuration. */
	uint8_t AGCCTRL2;   /* AGC control. */
	uint8_t AGCCTRL1;   /* AGC control. */
	uint8_t AGCCTRL0;   /* AGC control. */
	uint8_t FSCAL3;     /* Frequency synthesizer calibration. */
	uint8_t FSCAL2;     /* Frequency synthesizer calibration. */
	uint8_t FSCAL1;     /* Frequency synthesizer calibration. */
	uint8_t FSCAL0;     /* Frequency synthesizer calibration. */
	uint8_t FSTEST;     /* Frequency synthesizer calibration control */
	uint8_t TEST2;      /* Various test settings. */
	uint8_t TEST1;      /* Various test settings. */
	uint8_t TEST0;      /* Various test settings. */
	uint8_t FIFOTHR;    /* RXFIFO and TXFIFO thresholds. */
	uint8_t IOCFG2;     /* GDO2 output pin configuration */
	uint8_t PKTCTRL1;   /* Packet automation control. */
	uint8_t PKTCTRL0;   /* Packet automation control. */
	uint8_t ADDR;       /* Device address. */
	uint8_t PKTLEN;     /* Packet length. */
};

struct chipcon_cc1101_dev_s {
    struct spi_dev_s *                  spi;
    struct chipcon_c1101_rfsettings_s * settings;
    
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/** LIS331DL Access with range check
 * 
 * \param dev LIS331 DL Private Structure
 * \param subaddr LIS331 Sub Address
 * \param buf Pointer to buffer, either for read or write access
 * \param length when >0 it denotes read access, when <0 it denotes write access of -length
 * \return OK on success or errno is set.
 **/
int chipcon_cc1101_access(struct chipcon_cc1101_dev_s * dev, uint8_t subaddr, uint8_t *buf, int length)
{
    return 0;
}


/****************************************************************************
 * Callbacks
 ****************************************************************************/
 
/** External line triggers this callback 
 * 
 * The concept todo is:
 *  - GPIO provides EXTI Interrupt
 *  - It should handle EXTI Interrupts in ISR, to which chipcon can
 *    register a callback (and others). The ISR then foreach() calls a 
 *    its callback, and it is up to peripheral to find, whether the cause
 *    of EXTI ISR was itself.
 **/
void chipcon_cc1101_eventcb(void)
{
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct chipcon_cc1101_dev_s * chipcon_cc1101_init(struct spi_dev_s * spi)
{
    /* Powerup or wake-up, if it was previously set into some power-down state */

    /* Inquiry part number and version */
    
    /* Reset chip, check status bytes */
    
    /* Load it with default configuration */
    
    /* Bind to external interrupt line */

    return NULL;
}


int chipcon_cc1101_deinit(struct chipcon_cc1101_dev_s * dev)
{
    /* Power down chip, possibly in some inactive state */

    return 0;
}


int chipcon_cc1101_setgdo(struct chipcon_cc1101_dev_s * dev, uint8_t pin, uint8_t function)
{
    return 0;
}


int chipcon_cc1101_setrf(struct chipcon_cc1101_dev_s * dev, uint32_t frequency, int modulation)
{
    return 0;
}


int chipcon_cc1101_read(struct chipcon_cc1101_dev_s * dev, uint8_t * data, size_t size)
{
    return 0;
}


int chipcon_cc1101_write(struct chipcon_cc1101_dev_s * dev, const uint8_t * data, size_t size)
{
    return 0;
}


int chipcon_cc1101_listen(struct chipcon_cc1101_dev_s * dev)
{
    return 0;
}


int chipcon_cc1101_send(struct chipcon_cc1101_dev_s * dev)
{
    return 0;
}


int chipcon_cc1101_idle(struct chipcon_cc1101_dev_s * dev)
{
    return 0;
}


int chipcon_cc1101_powerup(struct chipcon_cc1101_dev_s * dev)
{
    return 0;
}


int chipcon_cc1101_powerdown(struct chipcon_cc1101_dev_s * dev)
{
    return 0;
}
