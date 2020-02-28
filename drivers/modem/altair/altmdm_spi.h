/****************************************************************************
 * drivers/modem/altmdm/altmdm_spi.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __DRIVERS_MODEM_ALTMDM_ALTMDM_ALTMDM_SPI_H
#define __DRIVERS_MODEM_ALTMDM_ALTMDM_ALTMDM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "altmdm_dev.h"
#include "altmdm_sys.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the transfer header. */

struct altmdm_spi_xferhdr_s
{
  uint8_t header[4];                     /* Transfer header. */
};

/* This structure describes the buffer for data receive. */

struct altmdm_spi_rxbuff_s
{
  FAR char *buff_addr;                   /* Receive buffer address. */
  uint32_t buff_size;                    /* Size of this buffer. */
  uint32_t rx_size;                      /* Received data size. */
  FAR struct altmdm_spi_rxbuff_s *next;  /* Link for next buffer. */
};

/* This structure describes the fifo for received buffer. */

struct altmdm_spi_rxbufffifo_s
{
  FAR struct altmdm_spi_rxbuff_s *head; /* Point to the head of fifo */
  FAR struct altmdm_spi_rxbuff_s *tail; /* Point to the tail of fifo */
  struct altmdm_sys_csem_s csem;        /* It is used for notification when
                                         * data is put in fifo.
                                         */
};

/* This structure describes the parameters for receive buffer information. */

struct altmdm_spi_rxbuffinfo_s
{
  FAR struct altmdm_spi_rxbuff_s *free_buff;  /* Free receive buffer address. */
  struct altmdm_spi_rxbufffifo_s fifo;        /* Receive buffer fifo. */
};

/* This structure describes the parameters for send data. */

struct altmdm_spi_tx_s
{
  struct altmdm_sys_lock_s lock;      /* Lock on accessing the following
                                       * parameters.
                                       */
  struct altmdm_sys_flag_s done_flag; /* Notify that tx request has been
                                       * completed.
                                       */
  struct altmdm_spi_xferhdr_s header; /* Tx header. */
  FAR char *buff_addr;                /* Buffer address for data transmission
                                       * specified by the user.
                                       */
  int32_t actual_size;                /* Actual data size. */
  int32_t total_size;                 /* Data size of 4byte alignment. */
  int32_t result;                     /* Result of transfer. */
  int32_t is_bufful;                  /* Indicates the slave is buffer full status. */
};

/* This structure describes the parameters for receive data. */

struct altmdm_spi_rx_s
{
  struct altmdm_sys_lock_s lock;      /* Lock on accessing the following
                                       * parameters.
                                       */
  struct altmdm_spi_xferhdr_s header; /* Rx header. */
  int8_t status_info;                 /* Header status information */
  int32_t actual_size;                /* Actual data size */
  int32_t total_size;                 /* Data size of 4byte alignment. */
  FAR struct altmdm_spi_rxbuff_s *rxbuff; /* Current receive buffer. */
  bool rxabort;                       /* Indicates whether the rx process is aborted. */
};

/* This structure describes the parameters for sleep modem. */

struct altmdm_spi_sleepmodem_s
{
  struct altmdm_sys_lock_s lock;      /* Lock on accessing the following
                                       * parameters.
                                       */

  struct altmdm_sys_flag_s done_flag; /* Notify that sleep request has been
                                       * completed.
                                       */
  int32_t result;                     /* Result of sleep request. */
  bool requested;                     /* Indicates that sleep request has been requested. */
  timer_t sv_timerid;                 /* Supervisor timer. */
};

/* This structure describes the resource of the ALTMDM spi driver */

struct altmdm_spi_dev_s
{
  /* Common fields */

  bool is_not_run;                    /* Indicates xfer task is not run. */
  int32_t task_id;                    /* xfer task ID. */
  bool is_xferready;                  /* Indicates whether the modem is ready to xfer. */
  struct altmdm_sys_flag_s xferready_flag;    /* Used for waiting ready to
                                               * xfer.
                                               */
  struct altmdm_sys_flag_s xfer_flag; /* Used for event handling of xfer
                                       * task.
                                       */
  struct altmdm_sys_flag_s dma_done_flag; /* Notify that DMA transfer has
                                           * been completed.
                                           */

  /* Parameter for receive buffer */

  struct altmdm_spi_rxbuffinfo_s rxbuffinfo;

  /* Parameter for send data */

  struct altmdm_spi_tx_s tx_param;

  /* Parameter for receive data */

  struct altmdm_spi_rx_s rx_param;

  /* Parameters for sleep modem */

  struct altmdm_spi_sleepmodem_s sleep_param;
};

#endif
#endif /* __DRIVERS_MODEM_ALTMDM_ALTMDM_SPI_H */
