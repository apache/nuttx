/****************************************************************************
 * arch/sim/src/sim/up_internal.h
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

#ifndef __ARCH_SIM_SRC_SIM_UP_INTERNAL_H
#define __ARCH_SIM_SRC_SIM_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __SIM__
#  include "config.h"
#endif

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdbool.h>
#  include <stdint.h>
#  if defined(CONFIG_SIM_NETDEV_TAP)
#    include <netinet/in.h>
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SMP_NCPUS
#  define CONFIG_SMP_NCPUS 1
#endif

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

/* Use a stack alignment of 16 bytes.  If necessary frame_size must be
 * rounded up to the next boundary
 */

#define STACK_ALIGNMENT     16

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* Simulated Heap Definitions ***********************************************/

/* Size of the simulated heap */

#define SIM_HEAP_SIZE (64*1024*1024)

/* Macros to handle saving and restoring interrupt state ********************/

#define up_savestate(regs) up_copyfullstate(regs, (xcpt_reg_t *)CURRENT_REGS)
#define up_restorestate(regs) (CURRENT_REGS = regs)

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
struct i2c_master_s;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

extern volatile void *g_current_regs[CONFIG_SMP_NCPUS];
#define CURRENT_REGS (g_current_regs[up_cpu_index()])

/* The command line  arguments passed to simulator */

extern int g_argc;
extern char **g_argv;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Context switching */

void up_copyfullstate(unsigned long *dest, unsigned long *src);
void *up_doirq(int irq, void *regs);

/* up_hostmisc.c ************************************************************/

void host_abort(int status);
int  host_backtrace(void** array, int size);

/* up_hostmemory.c **********************************************************/

void *host_alloc_heap(size_t sz);
void *host_alloc_shmem(const char *name, size_t size, int master);
void  host_free_shmem(void *mem);

size_t host_malloc_size(void *mem);
void *host_memalign(size_t alignment, size_t size);
void host_free(void *mem);
void *host_realloc(void *oldmem, size_t size);
void host_mallinfo(int *aordblks, int *uordblks);

/* up_hosttime.c ************************************************************/

uint64_t host_gettime(bool rtc);
void host_sleep(uint64_t nsec);
void host_sleepuntil(uint64_t nsec);
int host_settimer(int *irq);

/* up_sigdeliver.c **********************************************************/

void sim_sigdeliver(void);

/* up_simsmp.c **************************************************************/

#ifdef CONFIG_SMP
void sim_cpu0_start(void);
int sim_cpu_start(int cpu, void *stack, size_t size);
void sim_send_ipi(int cpu);
#endif

/* up_smpsignal.c ***********************************************************/

#ifdef CONFIG_SMP
void up_cpu_started(void);
int up_init_ipi(int irq);
#endif

/* up_oneshot.c *************************************************************/

#ifdef CONFIG_ONESHOT
void up_timer_update(void);
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

/* up_keyboard.c ************************************************************/

#ifdef CONFIG_SIM_KEYBOARD
int sim_kbd_initialize(void);
void up_kbdevent(uint32_t key, bool is_press);
#endif

/* up_eventloop.c ***********************************************************/

#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK) || \
    defined(CONFIG_ARCH_BUTTONS) || defined(CONFING_SIM_KEYBOARD)
void up_x11events(void);
void up_buttonevent(int x, int y, int buttons);
#endif

/* up_ajoystick.c ***********************************************************/

#ifdef CONFIG_SIM_AJOYSTICK
int sim_ajoy_initialize(void);
#endif

/* up_tapdev.c **************************************************************/

#if defined(CONFIG_SIM_NETDEV_TAP) && !defined(__CYGWIN__)
void tapdev_init(void *priv,
                 void (*tx_done_intr_cb)(void *priv),
                 void (*rx_ready_intr_cb)(void *priv));
int tapdev_avail(void);
unsigned int tapdev_read(unsigned char *buf, unsigned int buflen);
void tapdev_send(unsigned char *buf, unsigned int buflen);
void tapdev_ifup(in_addr_t ifaddr);
void tapdev_ifdown(void);

#  define netdev_init(priv,txcb,rxcb) tapdev_init(priv,txcb,rxcb)
#  define netdev_avail()              tapdev_avail()
#  define netdev_read(buf,buflen)     tapdev_read(buf,buflen)
#  define netdev_send(buf,buflen)     tapdev_send(buf,buflen)
#  define netdev_ifup(ifaddr)         tapdev_ifup(ifaddr)
#  define netdev_ifdown()             tapdev_ifdown()
#endif

/* up_wpcap.c ***************************************************************/

#if defined(CONFIG_SIM_NETDEV_TAP) && defined(__CYGWIN__)
void wpcap_init(void *priv,
                void (*tx_done_intr_cb)(void *priv),
                void (*rx_ready_intr_cb)(void *priv));
unsigned int wpcap_read(unsigned char *buf, unsigned int buflen);
void wpcap_send(unsigned char *buf, unsigned int buflen);

#  define netdev_init(priv,txcb,rxcb) wpcap_init(priv,txcb,rxcb)
#  define netdev_avail()              1
#  define netdev_read(buf,buflen)     wpcap_read(buf,buflen)
#  define netdev_send(buf,buflen)     wpcap_send(buf,buflen)
#  define netdev_ifup(ifaddr)         {}
#  define netdev_ifdown()             {}
#endif

/* up_vpnkit.c **************************************************************/

#if defined(CONFIG_SIM_NETDEV_VPNKIT)
void vpnkit_init(void *priv,
                 void (*tx_done_intr_cb)(void *priv),
                 void (*rx_ready_intr_cb)(void *priv));
int vpnkit_avail(void);
unsigned int vpnkit_read(unsigned char *buf, unsigned int buflen);
void vpnkit_send(unsigned char *buf, unsigned int buflen);

#  define netdev_init(priv,txcb,rxcb) vpnkit_init(priv,txcb,rxcb)
#  define netdev_avail()              vpnkit_avail()
#  define netdev_read(buf,buflen)     vpnkit_read(buf,buflen)
#  define netdev_send(buf,buflen)     vpnkit_send(buf,buflen)
#  define netdev_ifup(ifaddr)         {}
#  define netdev_ifdown()             {}
#endif

/* up_netdriver.c ***********************************************************/

int netdriver_init(void);
void netdriver_setmacaddr(unsigned char *macaddr);
void netdriver_setmtu(int mtu);
void netdriver_loop(void);

/* up_rptun.c ***************************************************************/

#ifdef CONFIG_RPTUN
int up_rptun_init(const char *shmemname, const char *cpuname, bool master);
void up_rptun_loop(void);
#endif

/* up_hcisocket.c ***********************************************************/

#ifdef CONFIG_SIM_HCISOCKET
int bthcisock_register(int dev_id);
int bthcisock_loop(void);
#endif

/* up_audio.c ***************************************************************/

#ifdef CONFIG_SIM_SOUND
struct audio_lowerhalf_s *sim_audio_initialize(bool playback);
void sim_audio_loop(void);
#endif

/* up_i2cbus*.c *************************************************************/

#ifdef CONFIG_SIM_I2CBUS
struct i2c_master_s *sim_i2cbus_initialize(int bus);
int sim_i2cbus_uninitialize(struct i2c_master_s *dev);
#endif

/* up_spi*.c ****************************************************************/

#ifdef CONFIG_SIM_SPI
struct spi_dev_s *sim_spi_initialize(const char *filename);
int sim_spi_uninitialize(struct spi_dev_s *dev);
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
size_t sim_stack_check(void *alloc, size_t size);
void up_stack_color(void *stackbase, size_t nbytes);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SIM_SRC_SIM_UP_INTERNAL_H */
