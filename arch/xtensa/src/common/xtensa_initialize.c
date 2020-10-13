/****************************************************************************
 * arch/xtensa/src/common/xtensa_initialize.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/loop.h>
#include <nuttx/net/loopback.h>
#include <nuttx/net/tun.h>
#include <nuttx/net/telnet.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/syslog/syslog_console.h>
#include <nuttx/serial/pty.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.  The architecture specific
 *   details of initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the clock, and
 *   registering device drivers are some of the things that are different
 *   for each processor and hardware platform.
 *
 *   up_initialize is called after the OS initialized but before the user
 *   initialization logic has been started and before the libraries have
 *   been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
#ifdef CONFIG_SMP
  int i;

  /* Initialize global variables */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      g_current_regs[i] = NULL;
    }
#else
  CURRENT_REGS = NULL;
#endif

  /* Add any extra memory fragments to the memory manager */

  xtensa_add_region();

#ifdef CONFIG_PM
  /* Initialize the power management subsystem.  This MCU-specific function
   * must be called *very* early in the initialization sequence *before* any
   * other device drivers are initialized (since they may attempt to register
   * with the power management subsystem).
   */

  xtensa_pminitialize();
#endif

  /* Initialize the internal heap */

#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  xtensa_imm_initialize();
#endif

#ifdef CONFIG_ARCH_DMA
  /* Initialize the DMA subsystem if the weak function xtensa_dma_initialize
   * has been brought into the build
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (xtensa_dma_initialize)
#endif
    {
      xtensa_dma_initialize();
    }
#endif

  /* Register devices */

#if defined(CONFIG_DEV_NULL)
  devnull_register();   /* Standard /dev/null */
#endif

#if defined(CONFIG_DEV_RANDOM)
  devrandom_register(); /* Standard /dev/random */
#endif

#if defined(CONFIG_DEV_URANDOM)
  devurandom_register();   /* Standard /dev/urandom */
#endif

#if defined(CONFIG_DEV_ZERO)
  devzero_register();   /* Standard /dev/zero */
#endif

#if defined(CONFIG_DEV_LOOP)
  loop_register();      /* Standard /dev/loop */
#endif

#if defined(CONFIG_DRIVER_NOTE)
  note_register();      /* Non-standard /dev/note */
#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  xtensa_serial_initialize();
#endif

#ifdef CONFIG_RPMSG_UART
  rpmsg_serialinit();
#endif

  /* Initialize the console device driver (if it is other than the standard
   * serial driver).
   */

#if defined(CONFIG_CONSOLE_SYSLOG)
  syslog_console_init();
#endif

#ifdef CONFIG_PSEUDOTERM_SUSV1
  /* Register the master pseudo-terminal multiplexor device */

  ptmx_register();
#endif

#if defined(CONFIG_CRYPTO)
  /* Initialize the HW crypto and /dev/crypto */

  up_cryptoinitialize();
#endif

#ifdef CONFIG_CRYPTO_CRYPTODEV
  devcrypto_register();
#endif

#ifndef CONFIG_NETDEV_LATEINIT
  /* Initialize the network */

  up_netinitialize();
#endif

#ifdef CONFIG_NET_LOOPBACK
  /* Initialize the local loopback device */

  localhost_initialize();
#endif

#ifdef CONFIG_NET_TUN
  /* Initialize the TUN device */

  tun_initialize();
#endif

#ifdef CONFIG_NETDEV_TELNET
  /* Initialize the Telnet session factory */

  telnet_initialize();
#endif

  /* Initialize USB -- device and/or host */

  up_usbinitialize();

  board_autoled_on(LED_IRQSENABLED);
}
