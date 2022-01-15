/****************************************************************************
 * arch/sparc/src/common/up_initialize.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched_note.h>
#include <nuttx/mm/iob.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/loop.h>
#include <nuttx/net/loopback.h>
#include <nuttx/net/tun.h>
#include <nuttx/net/telnet.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_console.h>
#include <nuttx/serial/pty.h>
#include <nuttx/crypto/crypto.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine which (if any) console driver to use.  This will probably cause
 * up_serialinit to be incorrectly called if there is no USART configured to
 * be an RS-232 device (see as an example arch/sparc/src/at32uc23/at32uc3
 * config.h) This will probably have to be revisited someday.
 *
 * If a console is enabled and no other console device is specified, then a
 * serial console is
 * assumed.
 */

#ifndef CONFIG_DEV_CONSOLE
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_DEV_LOWCONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  else
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
#  endif
#endif

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
static inline void up_color_intstack(void)
{
  uint8_t *ptr = (uint8_t *)&g_intstackalloc;
  ssize_t size;

  for (size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
       size > 0;
       size -= sizeof(uint8_t))
    {
      *ptr++ = INTSTACK_COLOR;
    }
}
#else
#  define up_color_intstack()
#endif

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
  /* Initialize global variables */

  g_current_regs = NULL;

  /* Colorize the interrupt stack */

  up_color_intstack();

  /* Add any extra memory fragments to the memory manager */

  up_addregion();

  /* Initialize the interrupt subsystem */

  up_irqinitialize();

  /* Initialize the system timer interrupt */

#if !defined(CONFIG_SUPPRESS_INTERRUPTS) && \
             !defined(CONFIG_SUPPRESS_TIMER_INTS)
  up_timer_initialize();
#endif

#ifdef CONFIG_ARCH_DMA
  /* Initialize the DMA subsystem if the weak function up_dma_initialize has
   * been brought into the build
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (up_dma_initialize)
#endif
    {
      up_dma_initialize();
    }
#endif

#ifdef CONFIG_MM_IOB
  /* Initialize IO buffering */

  iob_initialize();
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

#if defined(CONFIG_SCHED_INSTRUMENTATION_BUFFER) && \
    defined(CONFIG_DRIVER_NOTE)
  note_register();      /* Non-standard /dev/note */
#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  up_serialinit();
#endif

#ifdef CONFIG_RPMSG_UART
  rpmsg_serialinit();
#endif

  /* Initialize the console device driver (if it is other than the standard
   * serial driver).
   */

#if defined(CONFIG_DEV_LOWCONSOLE)
  lowconsole_init();
#elif defined(CONFIG_CONSOLE_SYSLOG)
  syslog_console_init();
#elif defined(CONFIG_RAMLOG_CONSOLE)
  ramlog_consoleinit();
#endif

#ifdef CONFIG_PSEUDOTERM_SUSV1
  /* Register the master pseudo-terminal multiplexor device */

  (void)ptmx_register();
#endif

  /* Initialize the console device driver (if it is other than the standard
   * serial driver).
   */

#if defined(CONFIG_CONSOLE_SYSLOG)
  syslog_console_init();
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

#ifdef CONFIG_NETDEV_LOOPBACK
  /* Initialize the local loopback device */

  (void)localhost_initialize();
#endif

#ifdef CONFIG_NET_TUN
  /* Initialize the TUN device */

  (void)tun_initialize();
#endif

#ifdef CONFIG_NETDEV_TELNET
  /* Initialize the Telnet session factory */

  (void)telnet_initialize();
#endif

  /* Initialize USB */

  up_usbinitialize();

#if defined(ARCH_HAVE_LEDS)
  board_autoled_on(LED_IRQSENABLED);
#endif
}
