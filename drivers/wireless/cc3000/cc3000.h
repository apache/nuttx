/****************************************************************************
 * drivers//wireless/cc3000.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * References:
 *   CC30000 from Texas Instruments http://processors.wiki.ti.com/index.php/CC3000
 *
 * See also:
 *     http://processors.wiki.ti.com/index.php/CC3000_Host_Driver_Porting_Guide
 *     http://processors.wiki.ti.com/index.php/CC3000_Host_Programming_Guide
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
#include <pthread.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/cc3000_common.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define CONFIG_CC3000_MT              /* Indicate multi threaded version */

#ifdef CONFIG_CC3000_MT
#  define CONFIG_WL_MAX_SOCKETS 5
#endif

/* CC3000 Interfaces ********************************************************/

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define READ                    3
#define READ_COMMAND           {READ, 0 , 0 , 0 , 0}
#define READ_OFFSET_TO_LENGTH   3 //cmd  dmy dmy lh  ll
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define SPI_HEADER_SIZE         (5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_CC3000_MT
/* lock to serialize access to driver (SPI protocol is window size 1) */

extern pthread_mutex_t g_cc3000_mut;

/* This structure describes the state of one CC3000 driver instance */

typedef struct cc3000_socket_ent
{
  volatile int sd;
  long status;
  bool received_closed_event:1;
  bool emptied_and_remotely_closed:1;
  sem_t semwait;
} cc3000_socket_ent;

typedef struct cc3000_accept_ent
{
  cc3000_socket_ent acc;
  struct sockaddr addr;
  socklen_t addrlen;
} cc3000_accept_ent;
#endif

typedef enum
{
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
  FAR struct cc3000_dev_s *flink;       /* Supports a singly linked list of drivers */
#endif
  pthread_t workertid;                  /* Handle for the worker thread */
  uint8_t crefs;                        /* Number of times the device has been opened */
  uint8_t nwaiters;                     /* Number of threads waiting for CC3000 data */
  uint8_t minor;                        /* minor */
  sem_t devsem;                         /* Manages exclusive access to this structure */
  sem_t *wrkwaitsem;                    /* Suspend and resume the delivery of messages */
  sem_t waitsem;                        /* Used to wait for the availability of data */
  sem_t irqsem;                         /* Used to signal irq from cc3000 */
  sem_t readysem;                       /* Used to wait for Ready Condition from the cc3000 */

  FAR struct cc3000_config_s *config;   /* Board configuration data */
  FAR struct spi_dev_s *spi;            /* Saved SPI driver instance */
  mqd_t queue;                          /* For unsolicited data delivery */
  eDeviceStates state;                  /* The device state */
  cc3000_buffer_desc rx_buffer;
  ssize_t rx_buffer_max_len;

  /* The following is a list if socket structures of threads waiting
   * long operations to finish;
   */
#ifdef CONFIG_CC3000_MT
  pthread_t selecttid;                  /* Handle for the select thread */
  sem_t selectsem;                      /* Used to sleep the select thread */
  cc3000_socket_ent sockets[CONFIG_WL_MAX_SOCKETS];
  cc3000_accept_ent accepting_socket;
#endif

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_CC3000_NPOLLWAITERS];
#endif

#if defined(CONFIG_DEBUG) && defined(CONFIG_CC3000_PROBES)
 long guard;
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

static inline void cc3000_lib_lock(void)
{
#ifdef CONFIG_CC3000_MT
  int status = pthread_mutex_lock(&g_cc3000_mut);
  DEBUGASSERT(status == 0);
  UNUSED(status);
#endif
}

static inline void cc3000_lib_unlock(void)
{
#ifdef CONFIG_CC3000_MT
  int status = pthread_mutex_unlock(&g_cc3000_mut);
  DEBUGASSERT(status == 0);
  UNUSED(status);
#endif
}

int cc3000_do_accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_WIRELESS_CC3000_H */
