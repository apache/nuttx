/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_uart.h
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UART_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define H4_HEADER_SIZE  1

#define H4_CMD           0x01
#define H4_ACL           0x02
#define H4_SCO           0x03
#define H4_EVT           0x04
#define H4_ISO           0x05

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the Bluetooth UART upper-half driver interrupt
 * callback used with the struct btuart_lowerhalf_s attach() method.
 */

struct btuart_lowerhalf_s;
typedef CODE void (*btuart_rxcallback_t)
  (FAR const struct btuart_lowerhalf_s *lower, FAR void *arg);

/* The Bluetooth UART driver is a two-part driver:
 *
 * 1) A common upper half driver that provides the common Bluetooth stack
 *    interface to UART.
 * 2) Platform-specific lower half drivers that provide the interface
 *    between the common upper half and the platform-specific UARTs.
 *
 * This structure defines the interface between an instance of the lower
 * half driver and the common upper half driver.  Such an instance is
 * passed to the upper half driver when the driver is initialized, binding
 * the upper and lower halves into one driver.
 */

struct btuart_lowerhalf_s
{
  /* Attach/enable the upper half Rx interrupt callback.
   *
   * rxattach() allows the upper half logic to attach a callback function
   *   that will be used to inform the upper half that an Rx frame is
   *   available.  This callback will, most likely, be invoked in the
   *   context of an interrupt callback.  The callback function should
   *   defer processing to the (high priority) work queue.  The receive()
   *   method should then be invoked from the work queue logic in order to
   *   receive the obtain the Rx frame data.
   * rxenable() may be used to enable or disable callback events.  This
   *   probably translates to enabling and disabled Rx interrupts at
   *   the UART.  NOTE:  Disabling Rx event notification should be done
   *   sparingly:  Rx data overrun may occur when Rx events are disabled!
   */

  CODE void (*rxattach)(FAR const struct btuart_lowerhalf_s *lower,
                        btuart_rxcallback_t callback, FAR void *arg);
  CODE void (*rxenable)(FAR const struct btuart_lowerhalf_s *lower,
                        bool enable);

  /* Change the HCI UART BAUD
   *
   * The HCI UART comes up with some initial BAUD rate.  Some support
   * auto-BAUD detection, some support writing a configuration file to
   * select the initial BAUD.  The simplest strategy, however, is simply
   * to use the HCI UART's default initial BAUD to perform the basic
   * bring up, then send a vendor-specific command to increase the HCI
   * UARTs BAUD.  This method then may be used to adjust the lower half
   * driver to the new HCI UART BAUD.
   */

  CODE int (*setbaud)(FAR const struct btuart_lowerhalf_s *lower,
                      uint32_t baud);

  /* Read/write UART frames
   *
   * read() after receipt of a callback notifying the upper half of the
   *   availability of Rx frame, the upper half may call the receive()
   *   method in order to obtain the buffered Rx frame data.
   * write() will add the outgoing frame to the Tx buffer and will return
   *   immediately.  This function may block only in the event that there
   *   is insufficient buffer space to hold the Tx frame data.  In that
   *   case the lower half will block until there is sufficient to buffer
   *   the entire outgoing packet.
   */

  CODE ssize_t (*read)(FAR const struct btuart_lowerhalf_s *lower,
                       FAR void *buffer, size_t buflen);
  CODE ssize_t (*write)(FAR const struct btuart_lowerhalf_s *lower,
                        FAR const void *buffer, size_t buflen);

  /* Flush/drain all buffered RX data */

  CODE ssize_t (*rxdrain)(FAR const struct btuart_lowerhalf_s *lower);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: btuart_register
 *
 * Description:
 *   Create the generic UART-based Bluetooth device and register it with the
 *   Bluetooth stack.
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int btuart_register(FAR const struct btuart_lowerhalf_s *lower);

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UART_H */
