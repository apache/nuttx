/*******************************************************************************************
 * include/nuttx/wireless/nrf24l01.h
 *
 *   Copyright (C) 2013 Laurent Latil
 *   Author: Laurent Latil <laurent@latil.nom.fr>
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_NRF24L01_H
#define __INCLUDE_NUTTX_NRF24L01_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

#include <stdint.h>
#include <stdbool.h>

/********************************************************************************************
 * Pre-Processor Declarations
 ********************************************************************************************/

#define NRF24L01_MIN_ADDR_LEN    3      /* Minimal length (in bytes) of a pipe address */
#define NRF24L01_MAX_ADDR_LEN    5      /* Maximum length (in bytes) of a pipe address */
#define NRF24L01_MAX_PAYLOAD_LEN 32     /* Maximum length (in bytes) of a payload */
#define NRF24L01_MAX_XMIT_RETR   15     /* Maximum auto retransmit count (for AA
                                         * transmissions) */
#define NRF24L01_PIPE_COUNT      6      /* Number of available pipes */

#define NRF24L01_MIN_FREQ        2400   /* Lower bound for RF frequency */
#define NRF24L01_MAX_FREQ        2525   /* Upper bound for RF frequency */

#define NRF24L01_DYN_LENGTH      33     /* Specific length value to use to enable dynamic
                                         * packet length */
#define NRF24L01_XMIT_MAXRT      255    /* Specific value returned by Number of available
                                         * pipes */

/* IOCTL commands
 *
 * COMMAND                       ARGUMENT
 * NRF24L01IOC_SETRETRCFG        Pointer to nrf24l01_retrcfg_t structure
 * NRF24L01IOC_GETRETRCFG        Pointer to nrf24l01_retrcfg_t structure
 * NRF24L01IOC_SETPIPESCFG       Pointer to an array of nrf24l01_pipecfg_t pointers
 * NRF24L01IOC_GETPIPESCFG       Pointer to an array of nrf24l01_pipecfg_t pointers
 * NRF24L01IOC_SETPIPESENABLED   Pointer to a uint8_t value, bit field of enabled /
 *                               disabled pipes
 * NRF24L01IOC_GETPIPESENABLED   Pointer to a uint8_t value, bit field of enabled /
 *                               disabled pipes
 * NRF24L01IOC_SETDATARATE       Pointer to a nrf24l01_datarate_t value
 * NRF24L01IOC_GETDATARATE       Pointer to a nrf24l01_datarate_t value
 * NRF24L01IOC_SETADDRWIDTH      Pointer to an uint32_t value, width of the address
 * NRF24L01IOC_GETADDRWIDTH      Pointer to an uint32_t value, width of the address
 * NRF24L01IOC_SETSTATE          Pointer to a nrf24l01_state_t value
 * NRF24L01IOC_GETSTATE          Pointer to a nrf24l01_state_t value
 * NRF24L01IOC_GETLASTXMITCOUNT  Pointer to an uint32_t value, retransmission count of the
 *                               last send operation (NRF24L01_XMIT_MAXRT if no ACK
 *                               received)
 * NRF24L01IOC_GETLASTPIPENO     Pointer to an uint32_t value, pipe # of the last received
 *                               packet
 * NRF24L01IOC_SETTXPAYLOADNOACK Pointer to an uint32_t, interpreted as bool
 * NRF24L01IOC_GETTXPAYLOADNOACK Pointer to an uint32_t, interpreted as bool
 */

#define NRF24L01IOC_SETRETRCFG        _WLCIOC(NRF24L01_FIRST + 0)
#define NRF24L01IOC_GETRETRCFG        _WLCIOC(NRF24L01_FIRST + 1)
#define NRF24L01IOC_SETPIPESCFG       _WLCIOC(NRF24L01_FIRST + 2)
#define NRF24L01IOC_GETPIPESCFG       _WLCIOC(NRF24L01_FIRST + 3)
#define NRF24L01IOC_SETPIPESENABLED   _WLCIOC(NRF24L01_FIRST + 4)
#define NRF24L01IOC_GETPIPESENABLED   _WLCIOC(NRF24L01_FIRST + 5)
#define NRF24L01IOC_SETDATARATE       _WLCIOC(NRF24L01_FIRST + 6)
#define NRF24L01IOC_GETDATARATE       _WLCIOC(NRF24L01_FIRST + 7)
#define NRF24L01IOC_SETADDRWIDTH      _WLCIOC(NRF24L01_FIRST + 8)
#define NRF24L01IOC_GETADDRWIDTH      _WLCIOC(NRF24L01_FIRST + 9)
#define NRF24L01IOC_SETSTATE          _WLCIOC(NRF24L01_FIRST + 10)
#define NRF24L01IOC_GETSTATE          _WLCIOC(NRF24L01_FIRST + 11)
#define NRF24L01IOC_GETLASTXMITCOUNT  _WLCIOC(NRF24L01_FIRST + 12)
#define NRF24L01IOC_GETLASTPIPENO     _WLCIOC(NRF24L01_FIRST + 13)
#define NRF24L01IOC_SETTXPAYLOADNOACK _WLCIOC(NRF24L01_FIRST + 14)
#define NRF24L01IOC_GETTXPAYLOADNOACK _WLCIOC(NRF24L01_FIRST + 15)

/* Aliased name for these commands */

#define NRF24L01IOC_SETTXADDR        WLIOC_SETADDR
#define NRF24L01IOC_GETTXADDR        WLIOC_GETADDR

/********************************************************************************************
 * Public Data Types
 ********************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
  {
#else
#  define EXTERN extern
#endif

/* Data rates available for transmission */

typedef enum
{
  RATE_1Mbps,
  RATE_2Mbps,
  RATE_250kbps
} nrf24l01_datarate_t;

/* Definition of the different possible states of the module */

typedef enum
{
  ST_UNKNOWN,
  ST_POWER_DOWN,
  ST_STANDBY,
#ifdef CONFIG_WL_NRF24L01_RXSUPPORT
  ST_RX
#endif
} nrf24l01_state_t;

/* Re-transmission delay */

typedef enum
{
  DELAY_250us,
  DELAY_500us,
  DELAY_750us,
  DELAY_1000us,
  DELAY_1250us,
  DELAY_1500us,
  DELAY_1750us,
  DELAY_2000us,
  DELAY_2250us,
  DELAY_2500us,
  DELAY_2750us,
  DELAY_3000us,
  DELAY_3250us,
  DELAY_3500us,
  DELAY_3750us,
  DELAY_4000us
} nrf24l01_retransmit_delay_t;

/* Opaque definition of a nRF24L01 device */

struct nrf24l01_dev_s;

/* Configuration info for a RX pipe */

struct nrf24l01_pipecfg_s
{
  bool en_aa;                              /* Enable auto-acknowledge */
  uint8_t rx_addr[NRF24L01_MAX_ADDR_LEN];  /* Receive address for the data pipe (LSB first) */
  uint8_t payload_length;                  /* Define packet size  (NRF24L01_DYN_LENGTH :
                                            * dynamic length payload ) */
};
typedef struct nrf24l01_pipecfg_s nrf24l01_pipecfg_t;

/* Configuration of the retransmission parameters  (used when AA is enabled) */

struct nrf24l01_retrcfg_s
{
  nrf24l01_retransmit_delay_t delay;    /* Delay before retransmitting  (when no ACK received) */
  uint8_t count;                        /* Retransmit retries count */
};
typedef struct nrf24l01_retrcfg_s nrf24l01_retrcfg_t;

/* A reference to a structure of this type must be passed to the initialization
 * method  (nrf24l01_init() ). It provides some board-specific hooks used
 * by driver to manage the GPIO lines  (IRQ and CE).
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct nrf24l01_config_s
{
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the ADS7843E driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * irqattach  - Attach the driver interrupt handler to the GPIO interrupt
   * chipenable - Enable or disable the chip  (CE line)
   */

  int  (*irqattach)(xcpt_t isr, FAR void *arg);
  void (*chipenable)(bool enable);
};

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

/********************************************************************************************
 * Register the nRF24L01+ device.
 *
 * Input Parameters:
 *   spi - SPI Device structure
 *   cfg Board specific configuration info
 *
 * Returned Value:
 *   Pointer to newly allocated nrf24l01 device structure or NULL on error
 *   (errno is set accordingly in this case).
 *
 * Possible errors:
 *  - ENOMEM: Out of kernel memory to allocate the device
 *
 ********************************************************************************************/

int nrf24l01_register(FAR struct spi_dev_s *spi, FAR struct nrf24l01_config_s *cfg);

/********************************************************************************************
 * Initialize the nRF24L01+ chip to a default initial state.
 *
 * Input Parameters:
 *   dev Pointer to a registered nRF24L01 device structure
 *
 ********************************************************************************************/

int nrf24l01_init(FAR struct nrf24l01_dev_s *dev);

/********************************************************************************************
 * Set the default TX address.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   addr TX address  (LSByte first)
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_settxaddr(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *addr);

/********************************************************************************************
 * Get the default TX address.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   addr TX address  (LSByte first)
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_gettxaddr(FAR struct nrf24l01_dev_s *dev, FAR uint8_t *addr);

/********************************************************************************************
 * Configure auto-retransmit
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   retrdelay  Auto-retransmit delay
 *   retrcount  Auto-retransmit count  (0 - 15)
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_setretransmit(FAR struct nrf24l01_dev_s *dev,
                           nrf24l01_retransmit_delay_t retrdelay, uint8_t retrcount);

/********************************************************************************************
 * Configure a RX pipe.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   pipeno Pipe number to configure
 *   pipecfg Pointer to configuration data
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_setpipeconfig(FAR struct nrf24l01_dev_s *dev, unsigned int pipeno,
                           FAR const nrf24l01_pipecfg_t *pipecfg);

/********************************************************************************************
 * Get pipe configuration.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   pipeno Pipe number to configure
 *   pipecfg Pointer to configuration data used to store the config
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_getpipeconfig(FAR struct nrf24l01_dev_s *dev, unsigned int pipeno,
                           FAR nrf24l01_pipecfg_t *pipecfg);

/********************************************************************************************
 * Enable a RX pipe.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   pipeno Pipe number
 *   enable true to enable the pipe, false to disable it
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_enablepipe(FAR struct nrf24l01_dev_s *dev, unsigned int pipeno,
                        bool enable);

/********************************************************************************************
 * Configure RF.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   datarate Datarate
 *   outpower Output power
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_setuprf(FAR struct nrf24l01_dev_s *dev, nrf24l01_datarate_t datarate,
                     int outpower);

/********************************************************************************************
 * Configure the TX output power.
 *
 * Note that hardware supports only -18, -12, -6 and 0 dBm values.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   outpower Output power (in dBm).
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_settxpower(FAR struct nrf24l01_dev_s *dev, int outpower);

/********************************************************************************************
 * Get the current TX output power.
 *
 * Note that hardware supports only -18, -12, -6 and 0 dBm values.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *
 * Returned Value:
 *   outpower Output power (in dBm)
 *
 ********************************************************************************************/

int nrf24l01_gettxpower(FAR struct nrf24l01_dev_s *dev);

/********************************************************************************************
 * Set transmission data rate
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *
 * Returned Value:
 *   datarate Data rate
 *
 ********************************************************************************************/

int nrf24l01_setdatarate(FAR struct nrf24l01_dev_s *dev,
                         nrf24l01_datarate_t datarate);

/********************************************************************************************
 * Set radio frequency.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   freq New frequency value  (in MHz: 2400 to 2525)
 *
 * Returned Value:
 *   OK
 *
 ********************************************************************************************/

int nrf24l01_setradiofreq(FAR struct nrf24l01_dev_s *dev, uint32_t freq);

/********************************************************************************************
 * Get current radio frequency.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *
 * Returned Value:
 *   Radio frequency  (in MHz)
 *
 ********************************************************************************************/

uint32_t nrf24l01_getradiofreq(FAR struct nrf24l01_dev_s *dev);

/********************************************************************************************
 * Configure address length.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   width Address width to use (3-5)
 *
 * Returned Value:
 *   0
 *
 ********************************************************************************************/

int nrf24l01_setaddrwidth(FAR struct nrf24l01_dev_s *dev, uint32_t width);

/********************************************************************************************
 * Change the current lifecycle state of the nRF24L01+ chip.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   state New state to put the nRF24L01 module into
 *
 ********************************************************************************************/

int nrf24l01_changestate(FAR struct nrf24l01_dev_s *dev, nrf24l01_state_t state);

/********************************************************************************************
 * Send data to the default address.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   data Pointer on the data buffer
 *   datalen Length of the buffer (in bytes)
 *
 * Returned Value:
 *
 ********************************************************************************************/

int nrf24l01_send(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *data,
                  size_t datalen);

/********************************************************************************************
 * Send data to the specified address.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *   data Pointer on the data buffer
 *   datalen Length of the buffer (in bytes)
 *   destaddr Destination address
 *
 * Returned Value:
 *
 ********************************************************************************************/

int nrf24l01_sendto(FAR struct nrf24l01_dev_s *dev, FAR const uint8_t *data,
                    size_t datalen, FAR const uint8_t *destaddr);

/********************************************************************************************
 * Get the retransmits count of the last transmission.
 * This value is meaningful only if auto-acknowledge is enabled.
 *
 * Input Parameters:
 *   dev Pointer to an nRF24L01 device structure
 *
 * Returned Value:
 *   Retransmit count, or NRF24L01_XMIT_MAXRT if no ACK received (transmission
 *   failure)
 *
 ********************************************************************************************/

int nrf24l01_xmitcount(FAR struct nrf24l01_dev_s *dev);

#ifdef CONFIG_WL_NRF24L01_RXSUPPORT

/********************************************************************************************
 * Read the next received frame.
 *
 *   dev Pointer to an nRF24L01 device structure
 *   buffer Pointer on buffer used to store the received frame bytes
 *   buflen Length of the buffer (in bytes)
 *   recvpipe Pointer to a byte value used to store the pipe number of the frame
 *     (use NULL if the pipe number info is not required)
 *
 * Returned Value:
 *   Length of the actual data
 *
 ********************************************************************************************/

ssize_t nrf24l01_recv(struct nrf24l01_dev_s *dev, FAR uint8_t *buffer,
                      size_t buflen, FAR uint8_t *recvpipe);

#endif

#ifdef CONFIG_DEBUG_WIRELESS
void nrf24l01_dumpregs(FAR struct nrf24l01_dev_s *dev);
void nrf24l01_dumprxfifo(FAR struct nrf24l01_dev_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NRF24L01_H */
