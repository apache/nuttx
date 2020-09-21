/****************************************************************************
 * arch/sim/src/up_internal.h
 *
 *   Copyright (C) 2007, 2009, 2011-2012, 2014, 2016-2017 Gregory Nutt.
 *     All rights reserved.
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

#ifndef __ARCH_SIM_SRC_UP_INTERNAL_H
#define __ARCH_SIM_SRC_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __ASSEMBLY__
#  include <nuttx/config.h>
#else
#  include <sys/types.h>
#  include <stdbool.h>
#  include <netinet/in.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine which (if any) console driver to use */

#ifndef CONFIG_DEV_CONSOLE
#  undef USE_DEVCONSOLE
#else
#  ifdef CONFIG_SYSLOG_RPMSG
#    undef USE_DEVCONSOLE
#  else
#    define USE_DEVCONSOLE 1
#  endif
#endif

/* Context Switching Definitions ********************************************/

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

#elif defined(CONFIG_HOST_X86) || defined(CONFIG_SIM_M32)
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
#elif defined(CONFIG_HOST_ARM)
#  define JB_SP 8
#  define JB_PC 9
#endif

/* Simulated Heap Definitions ***********************************************/

/* Size of the simulated heap */

#define SIM_HEAP_SIZE (64*1024*1024)

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

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR         0xdeadbeef

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct tcb_s;
struct spi_dev_s;
struct qspi_dev_s;
struct ioexpander_dev_s;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

int up_cpu_index(void); /* See include/nuttx/arch.h */
extern volatile void *g_current_regs[CONFIG_SMP_NCPUS];
#  define CURRENT_REGS (g_current_regs[up_cpu_index()])

#else

extern volatile void *g_current_regs[1];
#  define CURRENT_REGS (g_current_regs[0])

#endif

#ifdef CONFIG_SMP
/* These spinlocks are used in the SMP configuration in order to implement
 * up_cpu_pause().  The protocol for CPUn to pause CPUm is as follows
 *
 * 1. The up_cpu_pause() implementation on CPUn locks both g_cpu_wait[m]
 *    and g_cpu_paused[m].  CPUn then waits spinning on g_cpu_paused[m].
 * 2. CPUm receives the interrupt it (1) unlocks g_cpu_paused[m] and
 *    (2) locks g_cpu_wait[m].  The first unblocks CPUn and the second
 *    blocks CPUm in the interrupt handler.
 *
 * When CPUm resumes, CPUn unlocks g_cpu_wait[m] and the interrupt handler
 * on CPUm continues.  CPUm must, of course, also then unlock g_cpu_wait[m]
 * so that it will be ready for the next pause operation.
 */

extern volatile uint8_t g_cpu_wait[CONFIG_SMP_NCPUS];
extern volatile uint8_t g_cpu_paused[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void *up_doirq(int irq, void *regs);

/* up_setjmp32.S ************************************************************/

int  up_setjmp(void *jb);
void up_longjmp(void *jb, int val);

/* up_hostmemory.c **********************************************************/

void *host_alloc_heap(size_t sz);
void *host_alloc_shmem(const char *name, size_t size, int master);
void  host_free_shmem(void *mem);

/* up_hosttime.c ************************************************************/

uint64_t host_gettime(bool rtc);
void host_sleep(uint64_t nsec);
void host_sleepuntil(uint64_t nsec);
int host_settimer(int *irq);

/* up_simsmp.c **************************************************************/

#ifdef CONFIG_SMP
void sim_cpu0_start(void);
#endif

/* up_smpsignal.c ***********************************************************/

#ifdef CONFIG_SMP
void up_cpu_started(void);
int up_cpu_paused(int cpu);
struct tcb_s *up_this_task(void);
#endif

/* up_oneshot.c *************************************************************/

#ifdef CONFIG_ONESHOT
void up_timer_update(void);
#endif

/* rpmsg_serialinit *********************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#endif

/* up_uart.c ****************************************************************/

void up_uartinit(void);
void up_uartloop(void);

/* up_simuart.c *************************************************************/

void simuart_start(void);
int  simuart_open(const char *pathname);
void simuart_close(int fd);
int  simuart_putc(int fd, int ch);
int  simuart_getc(int fd);
bool simuart_checkc(int fd);
int  simuart_setcflag(int fd, unsigned int cflag);
int  simuart_getcflag(int fd, unsigned int *cflag);

/* up_deviceimage.c *********************************************************/

char *up_deviceimage(void);
void up_registerblockdevice(void);

/* up_x11framebuffer.c ******************************************************/

#ifdef CONFIG_SIM_X11FB
int up_x11initialize(unsigned short width, unsigned short height,
                     void **fbmem, size_t *fblen, unsigned char *bpp,
                     unsigned short *stride);
void up_x11update(void);
#ifdef CONFIG_FB_CMAP
int up_x11cmap(unsigned short first, unsigned short len,
               unsigned char *red, unsigned char *green,
               unsigned char *blue, unsigned char  *transp);
#endif
#endif

/* up_touchscreen.c *********************************************************/

#ifdef CONFIG_SIM_TOUCHSCREEN
int sim_tsc_initialize(int minor);
int sim_tsc_uninitialize(void);
#endif

/* up_eventloop.c ***********************************************************/

#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK)
void up_x11events(void);
void up_buttonevent(int x, int y, int buttons);
#endif

/* up_ajoystick.c ***********************************************************/

#ifdef CONFIG_SIM_AJOYSTICK
int sim_ajoy_initialize(void);
#endif

/* up_ioexpander.c **********************************************************/

#ifdef CONFIG_SIM_IOEXPANDER
struct ioexpander_dev_s *sim_ioexpander_initialize(void);
#endif

/* up_tapdev.c **************************************************************/

#if defined(CONFIG_SIM_NETDEV_TAP) && !defined(__CYGWIN__)
void tapdev_init(void);
int tapdev_avail(void);
unsigned int tapdev_read(unsigned char *buf, unsigned int buflen);
void tapdev_send(unsigned char *buf, unsigned int buflen);
void tapdev_ifup(in_addr_t ifaddr);
void tapdev_ifdown(void);

#  define netdev_init()           tapdev_init()
#  define netdev_avail()          tapdev_avail()
#  define netdev_read(buf,buflen) tapdev_read(buf,buflen)
#  define netdev_send(buf,buflen) tapdev_send(buf,buflen)
#  define netdev_ifup(ifaddr)     tapdev_ifup(ifaddr)
#  define netdev_ifdown()         tapdev_ifdown()
#endif

/* up_wpcap.c ***************************************************************/

#if defined(CONFIG_SIM_NETDEV_TAP) && defined(__CYGWIN__)
void wpcap_init(void);
unsigned int wpcap_read(unsigned char *buf, unsigned int buflen);
void wpcap_send(unsigned char *buf, unsigned int buflen);

#  define netdev_init()           wpcap_init()
#  define netdev_avail()          1
#  define netdev_read(buf,buflen) wpcap_read(buf,buflen)
#  define netdev_send(buf,buflen) wpcap_send(buf,buflen)
#  define netdev_ifup(ifaddr)     {}
#  define netdev_ifdown()         {}
#endif

/* up_vpnkit.c **************************************************************/

#if defined(CONFIG_SIM_NETDEV_VPNKIT)
void vpnkit_init(void);
int vpnkit_avail(void);
unsigned int vpnkit_read(unsigned char *buf, unsigned int buflen);
void vpnkit_send(unsigned char *buf, unsigned int buflen);
void vpnkit_ifup(in_addr_t ifaddr);
void vpnkit_ifdown(void);

#  define netdev_init()           vpnkit_init()
#  define netdev_avail()          vpnkit_avail()
#  define netdev_read(buf,buflen) vpnkit_read(buf,buflen)
#  define netdev_send(buf,buflen) vpnkit_send(buf,buflen)
#  define netdev_ifup(ifaddr)     vpnkit_ifup(ifaddr)
#  define netdev_ifdown()         vpnkit_ifdown()
#endif

/* up_netdriver.c ***********************************************************/

int netdriver_init(void);
void netdriver_setmacaddr(unsigned char *macaddr);
void netdriver_setmtu(int mtu);
void netdriver_loop(void);

/* up_rptun.c ***************************************************************/

#ifdef CONFIG_RPTUN
int up_rptun_init(void);
void up_rptun_loop(void);
#endif

#ifdef CONFIG_SIM_SPIFLASH
struct spi_dev_s *up_spiflashinitialize(const char *name);
#endif

#ifdef CONFIG_SIM_QSPIFLASH
struct qspi_dev_s *up_qspiflashinitialize(void);
#endif

/* up_hcisocket.c ***********************************************************/

#ifdef CONFIG_SIM_HCISOCKET
int bthcisock_register(int dev_id);
int bthcisock_loop(void);
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
void up_stack_color(void *stackbase, size_t nbytes);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SIM_SRC_UP_INTERNAL_H */
