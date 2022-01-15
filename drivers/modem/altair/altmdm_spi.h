/****************************************************************************
 * drivers/modem/altair/altmdm_spi.h
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

#ifndef __DRIVERS_MODEM_ALTAIR_ALTMDM_SPI_H
#define __DRIVERS_MODEM_ALTAIR_ALTMDM_SPI_H

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
  bool rxabort;                           /* Indicates whether the rx process
                                           * is aborted.
                                           */
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
#endif /* __DRIVERS_MODEM_ALTAIR_ALTMDM_SPI_H */
