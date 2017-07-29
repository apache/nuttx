/****************************************************************************
 * include/nuttx/wireless/pktradio.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some definitions that a compatible with the LGPL GNU C Library
 * header file of the same name.
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

#ifndef __INCLUDE_NUTTX_WIRELESS_PKTRADIO_H
#define __INCLUDE_NUTTX_WIRELESS_PKTRADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Pools */

#define PKTRADIO_POOL_PREALLOCATED  0
#define PKTRADIO_POOL_DYNAMIC       1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes an address used by the packet radio.  There is no standard
 * size for such an address.  Hence, it is represented simply as a arry of
 * bytes.
 */

struct pktradio_addr_s
{
  uint8_t pa_addrlen;                   /* Length of the following address */
  uint8_t pa_addr[CONFIG_PKTRADIO_ADDRLEN];
};

/* This is the form of the meta data that provides the radio-specific
 * information necessary to send and receive packets to and from the radio.
 */

struct pktradio_metadata_s
{
  struct pktradio_metadata_s *pm_flink; /* Supports a singly linked list */
  struct pktradio_addr_s pm_src;        /* Source of the packet */
  struct pktradio_addr_s pm_dest;       /* Destination of the packet */
  uint8_t pm_pool;                      /* See PKTRADIO_POOL_* definitions */
};

/* Different packet radios may have different properties.  If there are
 * multiple packet radios, then those properties have to be queried at
 * run time.  This information is provided to the 6LoWPAN network via the
 * following structure.
 */

struct pktradio_properties_s
{
  uint8_t pp_addrlen;                 /* Length of an address */
  uint8_t pp_pktlen;                  /* Fixed packet/frame size (up to 255) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pktradio_metadata_initialize
 *
 * Description:
 *   This function initializes the meta-data allocator.  This function must
 *   be called early in the initialization sequence before any radios
 *   begin operation.
 *
 *   This function is idempotent:  It may be called numerous times.  The
 *   initialization will be performed only on the first time that it is
 *   called.  Therefore, it may be called during packet radio driver
 *   initialization, even if there are multiple packet radio drivers.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void pktradio_metadata_initialize(void);

/****************************************************************************
 * Name: pktradio_metadata_allocate
 *
 * Description:
 *   The pktradio_metadata_allocate function will get a free meta-data
 *   structure for use by the packet radio.
 *
 *   This function will first attempt to allocate from the g_free_metadata
 *   list.  If that the list is empty, then the meta-data structure will be
 *   allocated from the dynamic memory pool.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   A reference to the allocated metadata structure.  All user fields in this
 *   structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR struct pktradio_metadata_s *pktradio_metadata_allocate(void);

/****************************************************************************
 * Name: pktradio_metadata_free
 *
 * Description:
 *   The pktradio_metadata_free function will return a metadata structure
 *   to the free list of  messages if it was a pre-allocated metadata
 *   structure. If the metadata structure was allocated dynamically it will
 *   be deallocated.
 *
 * Inputs:
 *   metadata - metadata structure to free
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void pktradio_metadata_free(FAR struct pktradio_metadata_s *metadata);

#endif /* __INCLUDE_NUTTX_WIRELESS_PKTRADIO_H */
