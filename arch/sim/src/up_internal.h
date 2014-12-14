/**************************************************************************
 * arch/sim/src/up_internal.h
 *
 *   Copyright (C) 2007, 2009, 2011-2012, 2014 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

#ifndef __ARCH_SIM_SRC_UP_INTERNAL_H
#define __ARCH_SIM_SRC_UP_INTERNAL_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdbool.h>

#  include <nuttx/irq.h>
#  include <arch/irq.h>
#endif

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/
/* Configuration **********************************************************/

#ifndef CONFIG_SIM_X11FB
#  ifdef CONFIG_SIM_TOUCHSCREEN
#    error "CONFIG_SIM_TOUCHSCREEN depends on CONFIG_SIM_X11FB"
#    undef CONFIG_SIM_TOUCHSCREEN
#  endif
#  ifdef CONFIG_SIM_AJOYSTICK
#    error "CONFIG_SIM_AJOYSTICK depends on CONFIG_SIM_X11FB"
#    undef CONFIG_SIM_AJOYSTICK
#  endif
#endif

#ifndef CONFIG_INPUT
#  ifdef CONFIG_SIM_TOUCHSCREEN
#    error "CONFIG_SIM_TOUCHSCREEN depends on CONFIG_INPUT"
#    undef CONFIG_SIM_TOUCHSCREEN
#  endif
#  ifdef CONFIG_SIM_AJOYSTICK
#    error "CONFIG_SIM_AJOYSTICK depends on CONFIG_INPUT"
#    undef CONFIG_SIM_AJOYSTICK
#  endif
#endif

/* Determine which (if any) console driver to use */

#if !defined(CONFIG_DEV_CONSOLE) || CONFIG_NFILE_DESCRIPTORS == 0
#  undef USE_DEVCONSOLE
#  undef CONFIG_RAMLOG_CONSOLE
#else
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef USE_DEVCONSOLE
#  else
#    define USE_DEVCONSOLE 1
#  endif
#endif

/* Determine which device to use as the system logging device */

#ifndef CONFIG_SYSLOG
#  undef CONFIG_SYSLOG_CHAR
#  undef CONFIG_RAMLOG_SYSLOG
#endif

/* The design for how we signal UART data availability is up in the air */

#undef CONFIG_SIM_UART_DATAPOST

/* Context Switching Definitions ******************************************/

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
   /* Storage order: %rbx, %rsp, %rbp, %r12, %r13, %r14, %r15, %rip */

#  ifdef __ASSEMBLY__
#    define JB_RBX (0*8)
#    define JB_RSP (1*8)
#    define JB_RBP (2*8)
#    define JB_R12 (3*8)
#    define JB_R13 (4*8)
#    define JB_R14 (5*8)
#    define JB_R15 (6*8)
#    define JB_RSI (7*8)

#  else
#    define JB_RBX (0)
#    define JB_RSP (1)
#    define JB_RBP (2)
#    define JB_R12 (3)
#    define JB_R13 (4)
#    define JB_R14 (5)
#    define JB_R15 (6)
#    define JB_RSI (7)

#  endif /* __ASSEMBLY__ */

/* Compatibility definitions */

#  define JB_SP    JB_RSP
#  define JB_PC    JB_RSI

#else
/* Storage order: %ebx, $esi, %edi, %ebp, sp, and return PC */

#  ifdef __ASSEMBLY__
#    define JB_EBX (0*4)
#    define JB_ESI (1*4)
#    define JB_EDI (2*4)
#    define JB_EBP (3*4)
#    define JB_SP  (4*4)
#    define JB_PC  (5*4)

#  else
#    define JB_EBX (0)
#    define JB_ESI (1)
#    define JB_EDI (2)
#    define JB_EBP (3)
#    define JB_SP  (4)
#    define JB_PC  (5)

#  endif /* __ASSEMBLY__ */
#endif /* CONFIG_HOST_X86_64 && !CONFIG_SIM_M32 */

/* Simulated Heap Definitions **********************************************/
/* Size of the simulated heap */

#if CONFIG_MM_SMALL
#  define SIM_HEAP_SIZE (64*1024)
#else
#  define SIM_HEAP_SIZE (4*1024*1024)
#endif

/* File System Definitions **************************************************/
/* These definitions characterize the compressed filesystem image */

#define BLOCK_COUNT         1024
#define SECTOR_OF_BACKUPT   6
#define NUMBER_OF_FATS      2
#define FAT_SIZE            32
#define NUM_HIDDEN_SECTORS  0
#define VOLUME_NAME         "NuttXTestVol"
#define USE_WHOLE_DEVICE    1
#define ROOT_DIR_ENTRIES    512
#define RESERVED_SECTORS    32
#define SECTORS_PER_CLUSTER 4
#define LOGICAL_SECTOR_SIZE 512

/**************************************************************************
 * Public Types
 **************************************************************************/

/**************************************************************************
 * Public Data
 **************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_SIM_X11FB
extern int g_x11initialized;
#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK)
extern volatile int g_eventloop;
#endif
#endif

#if defined(CONFIG_DEV_CONSOLE) && !defined(CONFIG_SIM_UART_DATAPOST)
extern volatile int g_uart_data_available;
#endif

/**************************************************************************
 * Public Function Prototypes
 **************************************************************************/

/* up_setjmp32.S **********************************************************/

int  up_setjmp(xcpt_reg_t *jb);
void up_longjmp(xcpt_reg_t *jb, int val) noreturn_function;

/* up_tickless.c **********************************************************/

#ifdef CONFIG_SCHED_TICKLESS
void up_timer_update(void);
#endif

/* up_devconsole.c ********************************************************/

void up_devconsole(void);
void up_registerblockdevice(void);

/* up_simuart.c ***********************************************************/

void simuart_start(void);
int simuart_putc(int ch);
int simuart_getc(void);
bool simuart_checkc(void);

/* up_uartwait.c **********************************************************/

void simuart_initialize(void);
void simuart_post(void);
void simuart_wait(void);

/* up_deviceimage.c *******************************************************/

char *up_deviceimage(void);

/* up_netdev.c ************************************************************/

#ifdef CONFIG_NET
unsigned long up_getwalltime( void );
#endif

/* up_x11framebuffer.c ****************************************************/

#ifdef CONFIG_SIM_X11FB
int up_x11initialize(unsigned short width, unsigned short height,
                     void **fbmem, unsigned int *fblen, unsigned char *bpp,
                     unsigned short *stride);
#ifdef CONFIG_FB_CMAP
int up_x11cmap(unsigned short first, unsigned short len,
               unsigned char *red, unsigned char *green,
               unsigned char *blue, unsigned char  *transp);
#endif
#endif

/* up_eventloop.c *********************************************************/

#if defined(CONFIG_SIM_X11FB) && \
   (defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK))
void up_x11events(void);
#endif

/* up_eventloop.c *********************************************************/

#if defined(CONFIG_SIM_X11FB) && \
   (defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK))
int up_buttonevent(int x, int y, int buttons);
#endif

/* up_ajoystick.c *********************************************************/

#ifdef CONFIG_SIM_AJOYSTICK
int sim_ajoy_initialize(void);
#endif

/* up_tapdev.c ************************************************************/

#if defined(CONFIG_NET) && !defined(__CYGWIN__)
void tapdev_init(void);
unsigned int tapdev_read(unsigned char *buf, unsigned int buflen);
void tapdev_send(unsigned char *buf, unsigned int buflen);

#define netdev_init()           tapdev_init()
#define netdev_read(buf,buflen) tapdev_read(buf,buflen)
#define netdev_send(buf,buflen) tapdev_send(buf,buflen)
#endif

/* up_wpcap.c *************************************************************/

#if defined(CONFIG_NET) && defined(__CYGWIN__)
void wpcap_init(void);
unsigned int wpcap_read(unsigned char *buf, unsigned int buflen);
void wpcap_send(unsigned char *buf, unsigned int buflen);

#define netdev_init()           wpcap_init()
#define netdev_read(buf,buflen) wpcap_read(buf,buflen)
#define netdev_send(buf,buflen) wpcap_send(buf,buflen)
#endif

/* up_netdriver.c *********************************************************/

#ifdef CONFIG_NET
int netdriver_init(void);
int netdriver_setmacaddr(unsigned char *macaddr);
void netdriver_loop(void);
#endif

#ifdef CONFIG_SIM_SPIFLASH
struct spi_dev_s;
struct spi_dev_s *up_spiflashinitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SIM_SRC_UP_INTERNAL_H */
