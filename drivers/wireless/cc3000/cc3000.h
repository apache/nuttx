/****************************************************************************
 * drivers//wireless/cc3000.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * References:
 *   CC30000 from Texas Instruments http://processors.wiki.ti.com/index.php/CC3000
 *
 * See also:
 * 		http://processors.wiki.ti.com/index.php/CC3000_Host_Driver_Porting_Guide
 * 		http://processors.wiki.ti.com/index.php/CC3000_Host_Programming_Guide
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

#ifndef __DRIVERS_WIRELESS_WIRELESS_CC3000_H
#define __DRIVERS_WIRELESS_WIRELESS_CC3000_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdint.h>
#include <mqueue.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/cc3000.h>
#include "spi.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* CC3000 Interfaces *********************************************************************/

/* Driver support **************************************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */


/********************************************************************************************
 * Public Types
 ********************************************************************************************/
#define READ                    3
#define READ_COMMAND		       {READ, 0 , 0 , 0 , 0}
#define READ_OFFSET_TO_LENGTH   3 //cmd  dmy dmy lh  ll
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define SPI_HEADER_SIZE         (5)


/* This structure describes the state of one CC3000 driver instance */
typedef enum {
		eSPI_STATE_POWERUP = 0,
		eSPI_STATE_INITIALIZED,
		eSPI_STATE_IDLE,
		eSPI_STATE_WRITE_WAIT_IRQ,
		eSPI_STATE_WRITE_PROCEED,
		eSPI_STATE_WRITE_DONE,
		eSPI_STATE_READ_IRQ,
		eSPI_STATE_READ_PROCEED,
		eSPI_STATE_READ_READY,

} eDeviceStates;

struct cc3000_dev_s
{
#ifdef CONFIG_CC3000_MULTIPLE
  FAR struct cc3000_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif
  uint8_t crefs;                        /* Number of times the device has been opened */
  uint8_t nwaiters;                     /* Number of threads waiting for CC3000 data */
  uint8_t minor;                        /* minor */
  sem_t devsem;                         /* Manages exclusive access to this structure */
  sem_t waitsem;                        /* Used to wait for the availability of data */
  sem_t readysem;                        /* Used to wait for Ready Condition from the cc3000 */

  FAR struct cc3000_config_s *config; 	/* Board configuration data */
  FAR struct spi_dev_s *spi;            /* Saved SPI driver instance */
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */
  mqd_t	 queue;							/* For unsolicited data delivery */
  eDeviceStates state;					/* The device state */
  uint8_t rx_buffer[CC3000_RX_BUFFER_SIZE];
  ssize_t rx_buffer_len;

  uint8_t tx_buffer[CC3000_TX_BUFFER_SIZE];
  ssize_t tx_buffer_len;

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_CC3000_NPOLLWAITERS];
#endif
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_WIRELESS_CC3000_H */
