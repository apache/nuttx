/****************************************************************************
 * include/nuttx/ipcc.h
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

#ifndef __INCLUDE_NUTTX_IPCC_H
#define __INCLUDE_NUTTX_IPCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/semaphore.h>

#include <poll.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IPCC is a two-part driver:
 *
 * 1) A common upper half driver, that provides the character device
 *    interface for the user
 * 2) Platform-specific lower half drivers that provide interface
 *    between upper and lower layer. Lower half also contains
 *    common objects like semaphore to inform
 */

/* This structure defines all of the operations provided by the
 * architecture specific logic. All fields must be provided with
 * non-NULL function pointers.
 */

struct ipcc_lower_s;
struct ipcc_ops_s
{
  /**************************************************************************
   * Name: read
   *
   * Description:
   *   Reads data directly from IPCC mbox, function shall not block.
   *   If no data is available, 0 shall be returned. Partial read
   *   is possible. In that case function can return value less
   *   than buflen.
   *
   * Input Parameters:
   *   ipcc - ipcc channel to read data from
   *   buffer - location where data shall be stored
   *   buflen - size of the buffer
   *
   * Returned Value:
   *   Number of bytes actually written to buffer.
   *
   **************************************************************************/

  CODE ssize_t (*read)(FAR struct ipcc_lower_s *ipcc,
                       FAR char *buffer, size_t buflen);

  /**************************************************************************
   * Name: write
   *
   * Description:
   *   Writes data directly to IPCC mbox, function shall not block.
   *   If IPCC memory is full, or busy or otherwise unavailable for
   *   write, 0 shall be returned. Partial write is possible. In
   *   that case function can return value less then buflen.
   *
   * Input Parameters:
   *   ipcc - ipcc channel to write data to
   *   buffer - location to read data from
   *   buflen - number of bytes to write from buffer
   *
   * Returned Value:
   *   Number of actually written data to IPCC channel.
   *
   **************************************************************************/

  CODE ssize_t (*write)(FAR struct ipcc_lower_s *ipcc,
                        FAR const char *buffer, size_t buflen);

  /**************************************************************************
   * Name: buffer_data
   *
   * Description:
   *   Lower driver shall copy data from IPCC to cirbuf on demand.
   *   It does not have to copy all data (in case rxbuf is full), but
   *   it must keep internal pointer to track uncopied data, and copy
   *   what is left on the next call to this function.
   *
   * Input Parameters:
   *   ipcc - channel to buffer data
   *   rxbuf - circural buffer where data should be written to
   *
   * Returned Value:
   *   Number of bytes that were successfully written to rxbuf.
   *
   **************************************************************************/

  CODE ssize_t (*buffer_data)(FAR struct ipcc_lower_s *ipcc,
                              FAR struct circbuf_s *rxbuf);

  /**************************************************************************
   * Name: write_notify
   *
   * Description:
   *   Upper half driver notifies lower half driver that new data has been
   *   copied to circ buffer. This can be used by lower half to transfer
   *   data from buffer to IPCC memory.
   *
   * Input Parameters:
   *   ipcc - ipcc channel that wrote data.
   *
   * Returned Value:
   *   0 on success, or negated errno otherwise
   *
   **************************************************************************/

  CODE ssize_t (*write_notify)(FAR struct ipcc_lower_s *ipcc);

  /**************************************************************************
   * Name: cleanup
   *
   * Description:
   *   Cleans up resources initialized by <arch>_ipcc_init(). If arch code
   *   malloc()ed any memory, this funciton is responsible of freeing it.
   *   After this function is called, ipcc driver will not access lower
   *   half pointer anymore.
   *
   * Input Parameters:
   *   ipcc - ipcc channel to cleanup
   *
   * Returned Value:
   *   Always OK
   *
   **************************************************************************/

  CODE int (*cleanup)(FAR struct ipcc_lower_s *ipcc);
};

/* This structure defines the interface between upper and lower halves.
 * Such instance is passed to ipcc_register() function after driver
 * has been initialized, binding the upper and lower halves into one
 * driver.
 */

struct ipcc_lower_s
{
  /* Pointer to upper half part of the driver. This should be used only
   * when calling ipcc_* upper half functions from lower half code.
   * There should be no need to access upper half driver's fields from
   * lower half context.
   */

  FAR struct ipcc_driver_s *upper;
  struct ipcc_ops_s         ops;      /* Arch specific functions */
  int                       chan;     /* IPCC channel */

#ifdef CONFIG_IPCC_BUFFERED
  struct circbuf_s          rxbuf;    /* Receive buffer */
  struct circbuf_s          txbuf;    /* Transmit buffer */

  /* 1 - not all data could fit into buffer, some unread data is still
   * in IPCC mbox. State of this variable is maintained by lower half,
   * upper half only reads it to know when to inform lower driver that
   * buffer got free.
   */

  int                       overflow;
#endif /* CONFIG_IPCC_BUFFERED */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void ipcc_txfree_notify(FAR struct ipcc_driver_s *priv);
void ipcc_rxfree_notify(FAR struct ipcc_driver_s *priv);

#ifdef CONFIG_IPCC_BUFFERED
int ipcc_register(FAR struct ipcc_lower_s *ipcc, size_t rxbuflen,
                  size_t txbuflen);
#else
int ipcc_register(FAR struct ipcc_lower_s *ipcc);
#endif

#endif /* __INCLUDE_NUTTX_IPCC_H */
