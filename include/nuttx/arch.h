/****************************************************************************
 * include/nuttx/arch.h
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

#ifndef __INCLUDE_NUTTX_ARCH_H
#define __INCLUDE_NUTTX_ARCH_H

/* This header file contains function prototypes for the interfaces between
 * (1) the nuttx core-code, (2) the microprocessor specific logic that
 * resides under the arch/ sub-directory, and (3) the board-specific logic
 * that resides under boards/
 *
 * Naming conventions:
 *
 * 1. Common Microprocessor Interfaces.
 *
 *    Any interface that is common across all microprocessors should be
 *    prefixed with up_ and prototyped in this header file. These
 *    definitions provide the common interface between NuttX and the
 *    architecture-specific implementation in arch/
 *
 *    This chip related declarations are retained in this header file.
 *
 *    NOTE: up_ is supposed to stand for microprocessor; the u is like the
 *    Greek letter micron.
 *
 * 2. Microprocessor-Specific Interfaces.
 *
 *    An interface which is unique to a certain microprocessor should be
 *    prefixed with the name of the microprocessor, for example stm32_,
 *    and be prototyped in some header file in the arch/ directories.
 *
 *    There is also a arch/<architecture>/include/<chip>/chip.h header file
 *    that can be used to communicate other microprocessor-specific
 *    information between the board logic and even application logic.
 *    Application logic may, for example, need to know specific capabilities
 *    of the chip.  Prototypes in that chip.h header file should follow the
 *    microprocessor specific naming convention.
 *
 * 3. Common Board Interfaces.
 *
 *    Any interface that is common across all boards should be prefixed
 *    with board_ and should be prototyped in the board.h header file.
 *    These board_ definitions provide the interface between the board-level
 *    logic and the architecture-specific logic.
 *
 * 4. Board-Specific Interfaces.
 *
 *    Any interface that is unique to a board should be prefixed with
 *    the board name, for example stm32f4discovery_.  Sometimes the board
 *    name is too long so stm32_ would be okay too.  These should be
 *    prototyped in boards/<arch>/<chip><board>/src/<board>.h and should
 *    not be used outside of that board directory since board-specific
 *    definitions have no meaning outside of the board directory.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/arch.h>
#include <arch/types.h>

#include <nuttx/compiler.h>
#include <nuttx/cache.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*sig_deliver_t)(FAR struct tcb_s *tcb);
typedef CODE void (*phy_enable_t)(bool enable);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
/* By default, the RTOS tickless logic assumes that the range of times that
 * can be represented by the underlying hardware timer is so large that no
 * special precautions need to be taken.  That is not always the case.  If
 * there is a limit to the maximum timing interval that can be represented by
 * the timer, then that limit must be respected.
 *
 * If CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP is defined, then use a 32-bit
 * global variable called g_oneshot_maxticks variable is enabled. This
 * variable is initialized by platform-specific logic at runtime to the
 * maximum delay that the timer can wait (in configured clock ticks).
 * The RTOS tickless logic will then limit all requested delays to this
 * value (in ticks).
 */

EXTERN uint32_t g_oneshot_maxticks;
#endif

#ifdef CONFIG_RTC
/* Variable determines the state of the RTC module.
 *
 * After initialization value is set to 'true' if RTC starts successfully.
 * The value can be changed to false also during operation if RTC for
 * some reason fails.
 */

EXTERN volatile bool g_rtc_enabled;
#endif

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
/* This is the interrupt vector mapping table.  This must be provided by
 * architecture specific logic if CONFIG_ARCH_MINIMAL_VECTORTABLE is define
 * in the configuration.  See declaration in include/nuttx/irq.h
 */

/* EXTERN const irq_mapped_t g_irqmap[NR_IRQS]; */

#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * These are standard interfaces that must be exported to the base RTOS
 * logic from architecture-specific code.
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
 *   up_initialize is called after the OS initialized but before the initial
 *   application has been started and before the libraries have been
 *   initialized. OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void);

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   The function up_systemreset() will reset the MCU.  Optional!
 *   Availability of this function is dependent upon the architecture
 *   support.
 *
 ****************************************************************************/

void up_systemreset(void) noreturn_function;

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g.,
 *   this is where power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void);

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created.
 *   This function is called to initialize the processor specific portions
 *   of the new TCB.
 *
 *   This function must setup the initial architecture registers and/or
 *   stack so that execution will begin at tcb->start on the next context
 *   switch.
 *
 ****************************************************************************/

void up_initial_state(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: up_create_stack
 *
 * Description:
 *   Allocate a stack for a new thread and setup up stack-related information
 *   in the TCB.
 *
 *   The following TCB fields must be initialized by this function:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware, processor,
 *     etc.  This value is retained only for debug purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - stack_base_ptr: Adjusted stack base pointer after the TLS Data and
 *     Arguments has been removed from the stack allocation.
 *
 * Input Parameters:
 *   - tcb: The TCB of new task
 *   - stack_size:  The requested stack size.  At least this much
 *     must be allocated.
 *   - ttype:  The thread type.  This may be one of following (defined in
 *     include/nuttx/sched.h):
 *
 *       TCB_FLAG_TTYPE_TASK     Normal user task
 *       TCB_FLAG_TTYPE_PTHREAD  User pthread
 *       TCB_FLAG_TTYPE_KERNEL   Kernel thread
 *
 *     This thread type is normally available in the flags field of the TCB,
 *     however, there are certain contexts where the TCB may not be fully
 *     initialized when up_create_stack is called.
 *
 *     If CONFIG_BUILD_PROTECTED or CONFIG_BUILD_KERNEL are is defined, then
 *     this thread type may affect how the stack is allocated.  For example,
 *     kernel thread stacks should be allocated from protected kernel memory.
 *     Stacks for user tasks and threads must come from memory that is
 *     accessible to user code.
 *
 ****************************************************************************/

int up_create_stack(FAR struct tcb_s *tcb, size_t stack_size, uint8_t ttype);

/****************************************************************************
 * Name: up_use_stack
 *
 * Description:
 *   Setup stack-related information in the TCB using pre-allocated stack
 *   memory.  This function is called only from nxtask_init() when a task or
 *   kernel thread is started (never for pthreads).
 *
 *   The following TCB fields must be initialized:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - stack_base_ptr: Adjusted stack base pointer after the TLS Data and
 *     Arguments has been removed from the stack allocation.
 *
 * Input Parameters:
 *   - tcb:  The TCB of new task
 *   - stack:  The new stack to be used.
 *   - stack_size:  The allocated stack size.
 *
 *   NOTE:  Unlike up_stack_create() and up_stack_release, this function
 *   does not require the task type (ttype) parameter.  The TCB flags will
 *   always be set to provide the task type to up_use_stack() if it needs
 *   that information.
 *
 ****************************************************************************/

int up_use_stack(FAR struct tcb_s *tcb, FAR void *stack, size_t stack_size);

/****************************************************************************
 * Name: up_stack_frame
 *
 * Description:
 *   Allocate a stack frame in the TCB's stack to hold thread-specific data.
 *   This function may be called any time after up_create_stack() or
 *   up_use_stack() have been called but before the task has been started.
 *
 *   Thread data may be kept in the stack (instead of in the TCB) if it is
 *   accessed by the user code directly.  This includes such things as
 *   argv[].  The stack memory is guaranteed to be in the same protection
 *   domain as the thread.
 *
 *   The following TCB fields will be re-initialized:
 *
 *   - adj_stack_size: Stack size after removal of the stack frame from
 *     the stack
 *   - stack_base_ptr: Adjusted stack base pointer after the TLS Data and
 *     Arguments has been removed from the stack allocation.
 *
 *   Here is the diagram after some allocation(tls, arg):
 *
 *                   +-------------+ <-stack_alloc_ptr(lowest)
 *                   |  TLS Data   |
 *                   +-------------+
 *                   |  Arguments  |
 *  stack_base_ptr-> +-------------+\
 *                   |  Available  | +
 *                   |    Stack    | |
 *                |  |             | |
 *                |  |             | +->adj_stack_size
 *                v  |             | |
 *                   |             | |
 *                   |             | +
 *                   +-------------+/
 *
 * Input Parameters:
 *   - tcb:  The TCB of new task
 *   - frame_size:  The size of the stack frame to allocate.
 *
 *  Returned Value:
 *   - A pointer to bottom of the allocated stack frame.  NULL will be
 *     returned on any failures.  The alignment of the returned value is
 *     the same as the alignment of the stack itself.
 *
 ****************************************************************************/

FAR void *up_stack_frame(FAR struct tcb_s *tcb, size_t frame_size);

/****************************************************************************
 * Name: up_release_stack
 *
 * Description:
 *   A task has been stopped. Free all stack related resources retained in
 *   the defunct TCB.
 *
 * Input Parameters:
 *   - dtcb:  The TCB containing information about the stack to be released
 *   - ttype:  The thread type.  This may be one of following (defined in
 *     include/nuttx/sched.h):
 *
 *       TCB_FLAG_TTYPE_TASK     Normal user task
 *       TCB_FLAG_TTYPE_PTHREAD  User pthread
 *       TCB_FLAG_TTYPE_KERNEL   Kernel thread
 *
 *     This thread type is normally available in the flags field of the TCB,
 *     however, there are certain error recovery contexts where the TCB may
 *     not be fully initialized when up_release_stack is called.
 *
 *     If CONFIG_BUILD_PROTECTED or CONFIG_BUILD_KERNEL are defined, then
 *     this thread type may affect how the stack is freed.  For example,
 *     kernel thread stacks may have been allocated from protected kernel
 *     memory.  Stacks for user tasks and threads must have come from memory
 *     that is accessible to user code.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_release_stack(FAR struct tcb_s *dtcb, uint8_t ttype);

/****************************************************************************
 * Name: up_unblock_task
 *
 * Description:
 *   A task is currently in an inactive task list
 *   but has been prepped to execute.  Move the TCB to the
 *   ready-to-run list, restore its context, and start execution.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Input Parameters:
 *   tcb: Refers to the tcb to be unblocked.  This tcb is
 *     in one of the waiting tasks lists.  It must be moved to
 *     the ready-to-run list and, if it is the highest priority
 *     ready to run task, executed.
 *
 ****************************************************************************/

void up_unblock_task(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: up_block_task
 *
 * Description:
 *   The currently executing task at the head of the ready to run list must
 *   be stopped.  Save its context and move it to the inactive list
 *   specified by task_state.
 *
 *   This function is called only from the NuttX scheduling logic.
 *   Interrupts will always be disabled when this function is called.
 *
 * Input Parameters:
 *   tcb: Refers to a task in the ready-to-run list (normally the task at
 *     the head of the list).  It must be stopped, its context saved and
 *     moved into one of the waiting task lists.  If it was the task at the
 *     head of the ready-to-run list, then a context switch to the new ready
 *     to run task must be performed.
 *   task_state: Specifies which waiting task list should be
 *     hold the blocked task TCB.
 *
 ****************************************************************************/

void up_block_task(FAR struct tcb_s *tcb, tstate_t task_state);

/****************************************************************************
 * Name: up_release_pending
 *
 * Description:
 *   When tasks become ready-to-run but cannot run because
 *   pre-emption is disabled, they are placed into a pending
 *   task list.  This function releases and makes ready-to-run
 *   all of the tasks that have collected in the pending task
 *   list.  This can cause a context switch if a new task is
 *   placed at the head of the ready to run list.
 *
 *   This function is called only from the NuttX scheduling
 *   logic when pre-emptioni is re-enabled.  Interrupts will
 *   always be disabled when this function is called.
 *
 ****************************************************************************/

void up_release_pending(void);

/****************************************************************************
 * Name: up_reprioritize_rtr
 *
 * Description:
 *   Called when the priority of a running or
 *   ready-to-run task changes and the reprioritization will
 *   cause a context switch.  Two cases:
 *
 *   1) The priority of the currently running task drops and the next
 *      task in the ready to run list has priority.
 *   2) An idle, ready to run task's priority has been raised above the
 *      priority of the current, running task and it now has the priority.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Input Parameters:
 *   tcb: The TCB of the task that has been reprioritized
 *   priority: The new task priority
 *
 ****************************************************************************/

void up_reprioritize_rtr(FAR struct tcb_s *tcb, uint8_t priority);

/****************************************************************************
 * Name: up_exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete() where the task to
 *   be deleted is the currently executing task.  It is more complex because
 *   a context switch must be perform to the next ready to run task.
 *
 *   Unlike other UP APIs, this function may be called directly from user
 *   programs in various states.  The implementation of this function should
 *   disable interrupts before performing scheduling operations.
 *
 ****************************************************************************/

void up_exit() noreturn_function;

/* Prototype is in unistd.h */

/****************************************************************************
 * Name: up_assert
 *
 * Description:
 *   Assertions may be handled in an architecture-specific way.
 *
 ****************************************************************************/

void up_assert(FAR const char *filename, int linenum);

#ifdef CONFIG_ARCH_HAVE_BACKTRACE

/****************************************************************************
 * Name: up_backtrace
 *
 * Description:
 *  up_backtrace()  returns  a backtrace for the TCB, in the array
 *  pointed to by buffer.  A backtrace is the series of currently active
 *  function calls for the program.  Each item in the array pointed to by
 *  buffer is of type void *, and is the return address from the
 *  corresponding stack frame.  The size argument specifies the maximum
 *  number of addresses that can be stored in buffer.   If  the backtrace is
 *  larger than size, then the addresses corresponding to the size most
 *  recent function calls are returned; to obtain the complete backtrace,
 *  make sure that buffer and size are large enough.
 *
 * Input Parameters:
 *   tcb    - Address of the task's TCB, NULL means dump the running task
 *   buffer - Return address from the corresponding stack frame
 *   size   - Maximum number of addresses that can be stored in buffer
 *   skip   - number of addresses to be skipped
 *
 * Returned Value:
 *   up_backtrace() returns the number of addresses returned in buffer
 *
 ****************************************************************************/

int up_backtrace(FAR struct tcb_s *tcb,
                 FAR void **buffer, int size, int skip);
#endif /* CONFIG_ARCH_HAVE_BACKTRACE */

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'sigdeliver' callback is executed on the thread
 *   specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked
 *   nor should it cause any immediate execution of sigdeliver.
 *   Typically, a few cases need to be considered:
 *
 *   (1) This function may be called from an interrupt handler
 *       During interrupt processing, all xcptcontext structures
 *       should be valid for all tasks.  That structure should
 *       be modified to invoke sigdeliver() either on return
 *       from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT
 *       the currently executing task, then again just modify
 *       the saved xcptcontext structure for the recipient
 *       task so it will invoke sigdeliver when that task is
 *       later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the
 *       currently executing task -- just call the signal
 *       handler now.
 *
 ****************************************************************************/

void up_schedule_sigaction(FAR struct tcb_s *tcb, sig_deliver_t sigdeliver);

/****************************************************************************
 * Name: up_task_start
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   task in user-space.  When the task is first started, a kernel-mode
 *   stub will first run to perform some housekeeping functions.  This
 *   kernel-mode stub will then be called transfer control to the user-mode
 *   task by calling this function.
 *
 *   Normally the a user-mode start-up stub will also execute before the
 *   task actually starts.  See libc/sched/task_startup.c
 *
 * Input Parameters:
 *   taskentry - The user-space entry point of the task.
 *   argc - The number of parameters being passed.
 *   argv - The parameters being passed. These lie in kernel-space memory
 *     and will have to be reallocated  in user-space memory.
 *
 * Returned Value:
 *   This function should not return.  It should call the user-mode start-up
 *   stub and that stub should call exit if/when the user task terminates.
 *
 ****************************************************************************/

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
void up_task_start(main_t taskentry, int argc, FAR char *argv[])
       noreturn_function;
#endif

/****************************************************************************
 * Name: up_pthread_start
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   pthread in user-space.  When the pthread is first started, a kernel-mode
 *   stub will first run to perform some housekeeping functions.  This
 *   kernel-mode stub will then be called transfer control to the user-mode
 *   pthread by calling this function.
 *
 *   Normally the a user-mode start-up stub will also execute before the
 *   pthread actually starts.  See libc/pthread/pthread_create.c
 *
 * Input Parameters:
 *   startup - The user-space pthread startup function
 *   entrypt - The user-space address of the pthread entry point
 *   arg     - Standard argument for the pthread entry point
 *
 * Returned Value:
 *   This function should not return.  It should call the user-mode start-up
 *   stub and that stub should call pthread_exit if/when the user pthread
 *   terminates.
 *
 ****************************************************************************/

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__) && \
    !defined(CONFIG_DISABLE_PTHREAD)
void up_pthread_start(pthread_trampoline_t startup,
                      pthread_startroutine_t entrypt, pthread_addr_t arg)
       noreturn_function;
#endif

/****************************************************************************
 * Name: up_signal_dispatch
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   a signal handler in user-space.  When the signal is delivered, a
 *   kernel-mode stub will first run to perform some housekeeping functions.
 *   This kernel-mode stub will then be called transfer control to the user
 *   mode signal handler by calling this function.
 *
 *   Normally the a architecture, user-mode signal handling stub will also
 *   execute before the ultimate signal handler is called.  That stub
 *   function is the user-space, signal handler trampoline function.  It is
 *   called from up_signal_dispatch() in user-mode.
 *
 * Input Parameters:
 *   sighand - The address user-space signal handling function
 *   signo, info, and ucontext - Standard arguments to be passed to the
 *     signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via an architecture specific system call made by up_signal_handler()
 *   (see below).  However, this will look like a normal return by the
 *   caller of up_signal_dispatch.
 *
 ****************************************************************************/

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
void up_signal_dispatch(_sa_sigaction_t sighand, int signo,
                        FAR siginfo_t *info, FAR void *ucontext);
#endif

/****************************************************************************
 * Name: up_signal_handler
 *
 * Description:
 *   This function is the user-space, signal handler trampoline function that
 *   must be provided by architecture-specific logic.  It is called from
 *   up_signal_dispatch() in user-mode.
 *
 * Input Parameters:
 *   sighand - The address user-space signal handling function
 *   signo, info, and ucontext - Standard arguments to be passed to the
 *     signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via an architecture specific system call.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)
void up_signal_handler(_sa_sigaction_t sighand, int signo,
                       FAR siginfo_t *info, FAR void *ucontext)
       noreturn_function;
#endif

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size);

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel builds (CONFIG_BUILD_PROTECTED=y or
 *   CONFIG_BUILD_KERNEL=y) there may be both kernel- and user-space heaps
 *   as determined by CONFIG_MM_KERNEL_HEAP=y.  This function allocates (and
 *   protects) the kernel-space heap.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size);
#endif

/****************************************************************************
 * Name: up_allocate_pgheap
 *
 * Description:
 *   If there is a page allocator in the configuration, then this function
 *   must be provided by the platform-specific code.  The OS initialization
 *   logic will call this function early in the initialization sequence to
 *   get the page heap information needed to configure the page allocator.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_PGALLOC
void up_allocate_pgheap(FAR void **heap_start, size_t *heap_size);
#endif

/****************************************************************************
 * Name: pgalloc
 *
 * Description:
 *   If there is a page allocator in the configuration and if an MMU is
 *   available to map physical addresses to virtual address, then this
 *   function must be provided by the platform-specific code.  This is part
 *   of the implementation of sbrk().  This function will allocate the
 *   requested number of pages using the page allocator and map them into
 *   consecutive virtual addresses beginning with 'brkaddr'
 *
 *   NOTE:  This function does not use the up_ naming standard because it
 *   is indirectly callable from user-space code via a system trap.
 *   Therefore, it is a system interface and follows a different naming
 *   convention.
 *
 * Input Parameters:
 *   brkaddr - The heap break address.  The next page will be allocated and
 *     mapped to this address.  Must be page aligned.  If the memory manager
 *     has not yet been initialized and this is the first block requested for
 *     the heap, then brkaddr should be zero.  pgalloc will then assigned the
 *     well-known virtual address of the beginning of the heap.
 *   npages - The number of pages to allocate and map.  Mapping of pages
 *     will be contiguous beginning beginning at 'brkaddr'
 *
 * Returned Value:
 *   The (virtual) base address of the mapped page will be returned on
 *   success.
 *   Normally this will be the same as the 'brkaddr' input. However, if
 *   the 'brkaddr' input was zero, this will be the virtual address of the
 *   beginning of the heap.  Zero is returned on any failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_MM_PGALLOC) && \
    defined(CONFIG_ARCH_USE_MMU)
uintptr_t pgalloc(uintptr_t brkaddr, unsigned int npages);
#endif

/****************************************************************************
 * Name: up_extraheaps_init
 *
 * Description:
 *   Initialize any extra heap.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
void up_extraheaps_init(void);
#endif

/****************************************************************************
 * Name: up_textheap_memalign
 *
 * Description:
 *   Allocate memory for text sections with the specified alignment.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_USE_TEXT_HEAP)
FAR void *up_textheap_memalign(size_t align, size_t size);
#endif

/****************************************************************************
 * Name: up_textheap_free
 *
 * Description:
 *   Free memory allocated for text sections.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_USE_TEXT_HEAP)
void up_textheap_free(FAR void *p);
#endif

/****************************************************************************
 * Name: up_setpicbase and up_getpicbase
 *
 * Description:
 *   It NXFLAT external modules (or any other binary format that requires
 *   PIC) are supported, then these macros must defined to (1) set or get
 *   the PIC base register value.  These must be implemented with in-line
 *   assembly.
 *
 ****************************************************************************/

#ifndef CONFIG_PIC
#  define up_setpicbase(picbase)
#  define up_getpicbase(ppicbase)
#endif

/****************************************************************************
 * Address Environment Interfaces
 *
 * Low-level interfaces used in binfmt/ to instantiate tasks with address
 * environments.  These interfaces all operate on type group_addrenv_t which
 * is an abstract representation of a task group's address environment and
 * must be defined in arch/arch.h if CONFIG_ARCH_ADDRENV is defined.
 *
 *   up_addrenv_create   - Create an address environment
 *   up_addrenv_destroy  - Destroy an address environment.
 *   up_addrenv_vtext    - Returns the virtual base address of the .text
 *                         address environment
 *   up_addrenv_vdata    - Returns the virtual base address of the .bss/.data
 *                         address environment
 *   up_addrenv_vheap    - Returns the virtual base address of the heap
 *                         address environment
 *   up_addrenv_heapsize - Returns the size of the initial heap allocation.
 *   up_addrenv_select   - Instantiate an address environment
 *   up_addrenv_restore  - Restore an address environment
 *   up_addrenv_clone    - Copy an address environment from one location to
 *                         another.
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the thread which whose
 * group been assigned an address environment by up_addrenv_clone().
 *
 *   up_addrenv_attach   - Clone the address environment assigned to one TCB
 *                         to another.  This operation is done when a pthread
 *                         is created that share's the same address
 *                         environment.
 *   up_addrenv_detach   - Release the threads reference to an address
 *                         environment when a task/thread exits.
 *
 * CONFIG_ARCH_STACK_DYNAMIC=y indicates that the user process stack resides
 * in its own address space.  This options is also *required* if
 * CONFIG_BUILD_KERNEL and CONFIG_LIBC_EXECFUNCS are selected.  Why?
 * Because the caller's stack must be preserved in its own address space
 * when we instantiate the environment of the new process in order to
 * initialize it.
 *
 * NOTE: The naming of the CONFIG_ARCH_STACK_DYNAMIC selection implies that
 * dynamic stack allocation is supported.  Certainly this option must be set
 * if dynamic stack allocation is supported by a platform.  But the more
 * general meaning of this configuration environment is simply that the
 * stack has its own address space.
 *
 * If CONFIG_ARCH_STACK_DYNAMIC=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_ustackalloc  - Create a stack address environment
 *   up_addrenv_ustackfree   - Destroy a stack address environment.
 *   up_addrenv_vustack      - Returns the virtual base address of the stack
 *   up_addrenv_ustackselect - Instantiate a stack address environment
 *
 * If CONFIG_ARCH_KERNEL_STACK is selected, then each user process will have
 * two stacks:  (1) a large (and possibly dynamic) user stack and (2) a
 * smaller kernel stack.  However, this option is *required* if both
 * CONFIG_BUILD_KERNEL and CONFIG_LIBC_EXECFUNCS are selected.  Why?  Because
 * when we instantiate and initialize the address environment of the new
 * user process, we will temporarily lose the address environment of the old
 * user process, including its stack contents.  The kernel C logic will crash
 * immediately with no valid stack in place.
 *
 * If CONFIG_ARCH_KERNEL_STACK=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_kstackalloc  - Create a stack in the kernel address
 *                             environment
 *   up_addrenv_kstackfree   - Destroy the kernel stack.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called when a new task is created in order to
 *   instantiate an address environment for the new task group.
 *   up_addrenv_create() is essentially the allocator of the physical
 *   memory for the new task.
 *
 * Input Parameters:
 *   textsize - The size (in bytes) of the .text address environment needed
 *     by the task.  This region may be read/execute only.
 *   datasize - The size (in bytes) of the .data/.bss address environment
 *     needed by the task.  This region may be read/write only.  NOTE: The
 *     actual size of the data region that is allocated will include a
 *     OS private reserved region at the beginning.  The size of the
 *     private, reserved region is give by ARCH_DATA_RESERVE_SIZE.
 *   heapsize - The initial size (in bytes) of the heap address environment
 *     needed by the task.  This region may be read/write only.
 *   addrenv - The location to return the representation of the task address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_create(size_t textsize, size_t datasize, size_t heapsize,
                      FAR group_addrenv_t *addrenv);
#endif

/****************************************************************************
 * Name: up_addrenv_destroy
 *
 * Description:
 *   This function is called when a final thread leaves the task group and
 *   the task group is destroyed.  This function then destroys the defunct
 *   address environment, releasing the underlying physical memory.
 *
 * Input Parameters:
 *   addrenv - The address environment to be destroyed.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_destroy(FAR group_addrenv_t *addrenv);
#endif

/****************************************************************************
 * Name: up_addrenv_vtext
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vtext - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_vtext(FAR group_addrenv_t *addrenv, FAR void **vtext);
#endif

/****************************************************************************
 * Name: up_addrenv_vdata
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   textsize - For some implementations, the text and data will be saved
 *      in the same memory region (read/write/execute) and, in this case,
 *      the virtual address of the data just lies at this offset into the
 *      common region.
 *   vdata - The location to return the virtual address.  NOTE that the
 *      beginning of the data region is reserved for use by the OS.  The
 *      returned address will be at a offset from the actual allocated base
 *      address to account for the OS private region.  The size of that
 *      offset is given by ARCH_DATA_RESERVE_SIZE
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_vdata(FAR group_addrenv_t *addrenv, uintptr_t textsize,
                     FAR void **vdata);
#endif

/****************************************************************************
 * Name: up_addrenv_vheap
 *
 * Description:
 *   Return the heap virtual address associated with the newly created
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vheap - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
int up_addrenv_vheap(FAR const group_addrenv_t *addrenv, FAR void **vheap);
#endif

/****************************************************************************
 * Name: up_addrenv_heapsize
 *
 * Description:
 *   Return the initial heap allocation size.  That is the amount of memory
 *   allocated by up_addrenv_create() when the heap memory region was first
 *   created.  This may or may not differ from the heapsize parameter that
 *   was passed to up_addrenv_create()
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   The initial heap size allocated is returned on success; a negated
 *   errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
ssize_t up_addrenv_heapsize(FAR const group_addrenv_t *addrenv);
#endif

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task group (via
 *   up_addrenv_create().  This function may be called to instantiate
 *   that address environment in the virtual address space.  This might be
 *   necessary, for example, to load the code for the task group from a file
 *   or to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *   oldenv
 *     The address environment that was in place before up_addrenv_select().
 *     This may be used with up_addrenv_restore() to restore the original
 *     address environment that was in place before up_addrenv_select() was
 *     called.  Note that this may be a task agnostic, platform-specific
 *     representation that may or may not be different from group_addrenv_t.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_select(FAR const group_addrenv_t *addrenv,
                      FAR save_addrenv_t *oldenv);
#endif

/****************************************************************************
 * Name: up_addrenv_restore
 *
 * Description:
 *   After an address environment has been temporarily instantiated by
 *   up_addrenv_select(), this function may be called to restore the
 *   original address environment.
 *
 * Input Parameters:
 *   oldenv - The platform-specific representation of the address environment
 *     previously returned by up_addrenv_select.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_restore(FAR const save_addrenv_t *oldenv);
#endif

/****************************************************************************
 * Name: up_addrenv_coherent
 *
 * Description:
 *   Flush D-Cache and invalidate I-Cache in preparation for a change in
 *   address environments.  This should immediately precede a call to
 *   up_addrenv_select();
 *
 * Input Parameters:
 *   addrenv - Describes the address environment to be made coherent.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_coherent(FAR const group_addrenv_t *addrenv);
#endif

/****************************************************************************
 * Name: up_addrenv_clone
 *
 * Description:
 *   Duplicate an address environment.  This does not copy the underlying
 *   memory, only the representation that can be used to instantiate that
 *   memory as an address environment.
 *
 * Input Parameters:
 *   src - The address environment to be copied.
 *   dest - The location to receive the copied address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_clone(FAR const group_addrenv_t *src,
                     FAR group_addrenv_t *dest);
#endif

/****************************************************************************
 * Name: up_addrenv_attach
 *
 * Description:
 *   This function is called from the core scheduler logic when a thread
 *   is created that needs to share the address environment of its task
 *   group.
 *
 *   NOTE: In some platforms, nothing will need to be done in this case.
 *   Simply being a member of the group that has the address environment
 *   may be sufficient.
 *
 * Input Parameters:
 *   group - The task group to which the new thread belongs.
 *   tcb   - The TCB of the thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_attach(FAR struct task_group_s *group, FAR struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_detach
 *
 * Description:
 *   This function is called when a task or thread exits in order to release
 *   its reference to an address environment.  The address environment,
 *   however, should persist until up_addrenv_destroy() is called when the
 *   task group is itself destroyed.  Any resources unique to this thread
 *   may be destroyed now.
 *
 *   NOTE: In some platforms, nothing will need to be done in this case.
 *   Simply being a member of the group that has the address environment
 *   may be sufficient.
 *
 * Input Parameters:
 *   group - The group to which the thread belonged.
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_detach(FAR struct task_group_s *group, FAR struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_mprot
 *
 * Description:
 *   Modify access rights to an address range.
 *
 * Input Parameters:
 *   addrenv - The address environment to be modified.
 *   addr - Base address of the region.
 *   len - Size of the region.
 *   prot - Access right flags.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_mprot(FAR group_addrenv_t *addrenv, uintptr_t addr,
                     size_t len, int prot);
#endif

/****************************************************************************
 * Name: up_addrenv_ustackalloc
 *
 * Description:
 *   This function is called when a new thread is created in order to
 *   instantiate an address environment for the new thread's stack.
 *   up_addrenv_ustackalloc() is essentially the allocator of the physical
 *   memory for the new task's stack.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that requires the stack address environment.
 *   stacksize - The size (in bytes) of the initial stack address
 *     environment needed by the task.  This region may be read/write only.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_STACK_DYNAMIC)
int up_addrenv_ustackalloc(FAR struct tcb_s *tcb, size_t stacksize);
#endif

/****************************************************************************
 * Name: up_addrenv_ustackfree
 *
 * Description:
 *   This function is called when any thread exits.  This function then
 *   destroys the defunct address environment for the thread's stack,
 *   releasing the underlying physical memory.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that no longer requires the stack address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_STACK_DYNAMIC)
int up_addrenv_ustackfree(FAR struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_vustack
 *
 * Description:
 *   Return the virtual address associated with the newly create stack
 *   address environment.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread with the stack address environment of
 *     interest.
 *   vstack - The location to return the stack virtual base address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_STACK_DYNAMIC)
int up_addrenv_vustack(FAR const struct tcb_s *tcb, FAR void **vstack);
#endif

/****************************************************************************
 * Name: up_addrenv_ustackselect
 *
 * Description:
 *   After an address environment has been established for a task's stack
 *   (via up_addrenv_ustackalloc().  This function may be called to
 *   instantiate that address environment in the virtual address space.
 *   This is a necessary step before each context switch to the newly created
 *   thread (including the initial thread startup).
 *
 * Input Parameters:
 *   tcb - The TCB of the thread with the stack address environment to be
 *     instantiated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_STACK_DYNAMIC)
int up_addrenv_ustackselect(FAR const struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_kstackalloc
 *
 * Description:
 *   This function is called when a new thread is created to allocate
 *   the new thread's kernel stack.   This function may be called for certain
 *   terminating threads which have no kernel stack.  It must be tolerant of
 *   that case.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that requires the kernel stack.
 *   stacksize - The size (in bytes) of the kernel stack needed by the
 *     thread.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)
int up_addrenv_kstackalloc(FAR struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_kstackfree
 *
 * Description:
 *   This function is called when any thread exits.  This function frees
 *   the kernel stack.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that no longer requires the kernel stack.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)
int up_addrenv_kstackfree(FAR struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_pa_to_va
 *
 * Description:
 *   Map phy address to virtual address.  Not supported by all architectures.
 *
 *   REVISIT:  Should this not then be conditional on having that
 *   architecture-specific support?
 *
 * Input Parameters:
 *   pa - The phy address to be mapped.
 *
 * Returned Value:
 *   Virtual address on success; NULL on failure.
 *
 ****************************************************************************/

FAR void *up_addrenv_pa_to_va(uintptr_t pa);

/****************************************************************************
 * Name: up_addrenv_va_to_pa
 *
 * Description:
 *   Map virtual address to phy address.  Not supported by all architectures.
 *
 *   REVISIT:  Should this not then be conditional on having that
 *   architecture-specific support?
 *
 * Input Parameters:
 *   va - The virtual address to be mapped.  Not supported by all
 *        architectures.
 *
 * Returned Value:
 *   Phy address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_va_to_pa(FAR void *va);

/****************************************************************************
 * Name: up_shmat
 *
 * Description:
 *   Attach, i.e, map, on shared memory region to a user virtual address
 *
 * Input Parameters:
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_SHM
int up_shmat(FAR uintptr_t *pages, unsigned int npages, uintptr_t vaddr);
#endif

/****************************************************************************
 * Name: up_shmdt
 *
 * Description:
 *   Detach, i.e, unmap, on shared memory region from a user virtual address
 *
 * Input Parameters:
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_SHM
int up_shmdt(uintptr_t vaddr, unsigned int npages);
#endif

/****************************************************************************
 * Interfaces required for ELF module support
 *
 *   up_checkarch   - Perform architecture-specific ELF check
 *   up_relocate    - Perform architecture-specific ELF relocation
 *   up_relocateadd - Perform architecture-specific ELF relocation
 *
 ****************************************************************************/

/* See prototype in include/nuttx/elf.h */

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void);

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in
 *   the interrupt handler context.
 *
 ****************************************************************************/

bool up_interrupt_context(void);

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_restore() supports the global level, the device level is
 *   hardware specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_NOINTC
void up_enable_irq(int irq);
#endif

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_NOINTC
void up_disable_irq(int irq);
#endif

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software. May not be supported by all architectures.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
void up_trigger_irq(int irq);
#endif

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority);
#endif

#ifdef CONFIG_ARCH_HAVE_TRUSTZONE

/****************************************************************************
 * Name: up_set_secure_irq
 *
 * Description:
 *   Secure an IRQ
 *
 ****************************************************************************/

void up_secure_irq(int irq, bool secure);

/****************************************************************************
 * Name: up_secure_irq_all
 *
 * Description:
 *   Secure all IRQ
 *
 ****************************************************************************/

void up_secure_irq_all(bool secure);

#endif

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer hardware.
 *
 ****************************************************************************/

void up_timer_initialize(void);

/****************************************************************************
 * Tickless OS Support.
 *
 * When CONFIG_SCHED_TICKLESS is enabled, all support for timer interrupts
 * is suppressed and the platform specific code is expected to provide the
 * following custom functions.
 *
 *   Architecture specific timer initialization logic initializes the timer
 *     facilities.  This happens early in the initialization sequence (via
 *     up_initialize()).
 *   int up_timer_gettime(FAR struct timespec *ts):  Returns the current
 *     time from the platform specific time source.
 *
 * The tickless option can be supported either via a simple interval timer
 * (plus elapsed time) or via an alarm.  The interval timer allows
 * programming events to occur after an interval.  With the alarm, you can
 * set a time in the future and get an event when that alarm goes off.
 *
 *   int up_alarm_cancel(void):  Cancel the alarm.
 *   int up_alarm_start(FAR const struct timespec *ts): Enable (or re-anable
 *     the alarm.
 * #else
 *   int up_timer_cancel(void):  Cancels the interval timer.
 *   int up_timer_start(FAR const struct timespec *ts): Start (or re-starts)
 *     the interval timer.
 * #endif
 *
 * The RTOS will provide the following interfaces for use by the platform-
 * specific interval timer implementation:
 *
 * #ifdef CONFIG_SCHED_TICKLESS_ALARM
 *   void nxsched_alarm_expiration(FAR const struct timespec *ts):  Called
 *     by the platform-specific logic when the alarm expires.
 * #else
 *   void nxsched_timer_expiration(void):  Called by the platform-specific
 *     logic when the interval timer expires.
 * #endif
 *
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   the archtecture-specific timer was initialized).  This function is
 *   functionally equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
 *   time calculations.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS)
int up_timer_gettime(FAR struct timespec *ts);
#endif

#ifdef CONFIG_CLOCK_TIMEKEEPING
int up_timer_getcounter(FAR uint64_t *cycles);
void up_timer_getmask(FAR uint64_t *mask);
#endif

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Cancel the alarm and return the time of cancellation of the alarm.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_alarm_expiration() will not be called unless the alarm is
 *   restarted with up_alarm_start().
 *
 *   If, as a race condition, the alarm has already expired when this
 *   function is called, then time returned is the current time.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the expiration time.  The current time should
 *        returned if the alarm is not active.  ts may be NULL in which
 *        case the time is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_alarm_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS) && defined(CONFIG_SCHED_TICKLESS_ALARM)
int up_alarm_cancel(FAR struct timespec *ts);
#endif

/****************************************************************************
 * Name: up_alarm_start
 *
 * Description:
 *   Start the alarm.  nxsched_alarm_expiration() will be called when the
 *   alarm occurs (unless up_alaram_cancel is called to stop it).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - The time in the future at the alarm is expected to occur.  When
 *        the alarm occurs the timer logic will call
 *        nxsched_alarm_expiration().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS) && defined(CONFIG_SCHED_TICKLESS_ALARM)
int up_alarm_start(FAR const struct timespec *ts);
#endif

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining on the timer.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_timer_expiration() will not be called unless the timer is
 *   restarted with up_timer_start().
 *
 *   If, as a race condition, the timer has already expired when this
 *   function is called, then that pending interrupt must be cleared so
 *   that up_timer_start() and the remaining time of zero should be
 *   returned.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the remaining time.  Zero should be returned
 *        if the timer is not active.  ts may be zero in which case the
 *        time remaining is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS) && !defined(CONFIG_SCHED_TICKLESS_ALARM)
int up_timer_cancel(FAR struct timespec *ts);
#endif

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.  nxsched_timer_expiration() will be called at
 *   the completion of the timeout (unless up_timer_cancel is called to stop
 *   the timing.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until nxsched_timer_expiration() is
 *        called.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS) && !defined(CONFIG_SCHED_TICKLESS_ALARM)
int up_timer_start(FAR const struct timespec *ts);
#endif

/****************************************************************************
 * Name: up_getsp
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Current stack pointer.
 *
 ****************************************************************************/

/* uintptr_t up_getsp(void);
 *
 * The actual declaration or definition is provided in arch/arch.h.
 * The actual implementation may be a MACRO or an inline function.
 */

/****************************************************************************
 * TLS support
 ****************************************************************************/

/****************************************************************************
 * Name: up_tls_info
 *
 * Description:
 *   Return the TLS information structure for the currently executing thread.
 *   When TLS is enabled, up_create_stack() will align allocated stacks to
 *   the TLS_STACK_ALIGN value.  An instance of the following structure will
 *   be implicitly positioned at the "lower" end of the stack.  Assuming a
 *   "push down" stack, this is at the "far" end of the stack (and can be
 *   clobbered if the stack overflows).
 *
 *   If an MCU has a "push up" then that TLS structure will lie at the top
 *   of the stack and stack allocation and initialization logic must take
 *   care to preserve this structure content.
 *
 *   The stack memory is fully accessible to user mode threads.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A pointer to TLS info structure at the beginning of the STACK memory
 *   allocation.  This is essentially an application of the TLS_INFO(sp)
 *   macro and has a platform dependency only in the manner in which the
 *   stack pointer (sp) is obtained and interpreted.
 *
 ****************************************************************************/

/* struct tls_info_s;
 * FAR struct tls_info_s *up_tls_info(void);
 *
 * The actual definition is provided in arch/arch.h as a macro. The default
 * implementation provided here assume the arch has a "push down" stack.
 */

/****************************************************************************
 * Name: up_tls_size
 *
 * Description:
 *   Get TLS (sizeof(struct tls_info_s) + tdata + tbss) section size.
 *
 * Returned Value:
 *   Size of (sizeof(struct tls_info_s) + tdata + tbss).
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_THREAD_LOCAL
int up_tls_size(void);
#else
#define up_tls_size() sizeof(struct tls_info_s)
#endif

/****************************************************************************
 * Name: up_tls_initialize
 *
 * Description:
 *   Initialize thread local region
 *
 * Input Parameters:
 *   tls_data - The memory region to initialize
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_THREAD_LOCAL
void up_tls_initialize(FAR struct tls_info_s *info);
#else
#define up_tls_initialize(x)
#endif

/****************************************************************************
 * Multiple CPU support
 ****************************************************************************/

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform an atomic test and set operation on the provided spinlock.
 *
 * Input Parameters:
 *   lock - The address of spinlock object.
 *
 * Returned Value:
 *   The spinlock is always locked upon return.  The value of previous value
 *   of the spinlock variable is returned, either SP_LOCKED if the spinlock
 *   was previously locked (meaning that the test-and-set operation failed to
 *   obtain the lock) or SP_UNLOCKED if the spinlock was previously unlocked
 *   (meaning that we successfully obtained the lock)
 *
 ****************************************************************************/

/* See prototype in include/nuttx/spinlock.h */

/****************************************************************************
 * Name: up_fetchadd8, up_fetchadd16, and up_fetchadd32
 *
 * Description:
 *   Perform an atomic fetch add operation on the provided 8-, 16-, or 32-
 *   bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of value to be incremented.
 *   value - The addend
 *
 * Returned Value:
 *   The incremented value (volatile!)
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_FETCHADD
int32_t up_fetchadd32(FAR volatile int32_t *addr, int32_t value);
int16_t up_fetchadd16(FAR volatile int16_t *addr, int16_t value);
int8_t up_fetchadd8(FAR volatile int8_t *addr, int8_t value);
#endif

/****************************************************************************
 * Name: up_fetchsub8
 *
 * Description:
 *   Perform an atomic fetch subtract operation on the provided 8-, 16-, or
 *   32-bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of value to be decremented.
 *   value - The subtrahend
 *
 * Returned Value:
 *   The decremented value (volatile!)
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_FETCHADD
int32_t up_fetchsub32(FAR volatile int32_t *addr, int32_t value);
int16_t up_fetchsub16(FAR volatile int16_t *addr, int16_t value);
int8_t up_fetchsub8(FAR volatile int8_t *addr, int8_t value);
#endif

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_index(void);
#else
#  define up_cpu_index() (0)
#endif

/****************************************************************************
 * Name: up_cpu_idlestack
 *
 * Description:
 *   Allocate a stack for the CPU[n] IDLE task (n > 0) if appropriate and
 *   setup up stack-related information in the IDLE task's TCB.  This
 *   function is always called before up_cpu_start().  This function is
 *   only called for the CPU's initial IDLE task; up_create_task is used for
 *   all normal tasks, pthreads, and kernel threads for all CPUs.
 *
 *   The initial IDLE task is a special case because the CPUs can be started
 *   in different wans in different environments:
 *
 *   1. The CPU may already have been started and waiting in a low power
 *      state for up_cpu_start().  In this case, the IDLE thread's stack
 *      has already been allocated and is already in use.  Here
 *      up_cpu_idlestack() only has to provide information about the
 *      already allocated stack.
 *
 *   2. The CPU may be disabled but started when up_cpu_start() is called.
 *      In this case, a new stack will need to be created for the IDLE
 *      thread and this function is then equivalent to:
 *
 *      return up_create_stack(tcb, stack_size, TCB_FLAG_TTYPE_KERNEL);
 *
 *   The following TCB fields must be initialized by this function:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware, processor,
 *     etc.  This value is retained only for debug purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - stack_base_ptr: Adjusted stack base pointer after the TLS Data and
 *     Arguments has been removed from the stack allocation.
 *
 * Input Parameters:
 *   - cpu:         CPU index that indicates which CPU the IDLE task is
 *                  being created for.
 *   - tcb:         The TCB of new CPU IDLE task
 *   - stack_size:  The requested stack size for the IDLE task.  At least
 *                  this much must be allocated.  This should be
 *                  CONFIG_IDLETHREAD_STACKSIZE.
 *
 ****************************************************************************/

int up_cpu_idlestack(int cpu, FAR struct tcb_s *tcb, size_t stack_size);

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configuration, only one CPU is initially active (CPU 0).
 *   System initialization occurs on that single thread. At the completion of
 *   the initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to its IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  No stack has been allocated or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of one to (CONFIG_SMP_NCPUS-1).
 *         (CPU 0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_start(int cpu);
#endif

/****************************************************************************
 * Name: up_cpu_pause
 *
 * Description:
 *   Save the state of the current task at the head of the
 *   g_assignedtasks[cpu] task list and then pause task execution on the
 *   CPU.
 *
 *   This function is called by the OS when the logic executing on one CPU
 *   needs to modify the state of the g_assignedtasks[cpu] list for another
 *   CPU.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be paused.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called from within a critical section; up_cpu_resume() must be called
 *   later while still within the same critical section.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_pause(int cpu);
#endif

/****************************************************************************
 * Name: up_cpu_pausereq
 *
 * Description:
 *   Return true if a pause request is pending for this CPU.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be queried
 *
 * Returned Value:
 *   true   = a pause request is pending.
 *   false = no pasue request is pending.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
bool up_cpu_pausereq(int cpu);
#endif

/****************************************************************************
 * Name: up_cpu_paused
 *
 * Description:
 *   Handle a pause request from another CPU.  Normally, this logic is
 *   executed from interrupt handling logic within the architecture-specific
 *   However, it is sometimes necessary to perform the pending pause
 *   operation in other contexts where the interrupt cannot be taken
 *   in order to avoid deadlocks.
 *
 *   This function performs the following operations:
 *
 *   1. It saves the current task state at the head of the current assigned
 *      task list.
 *   2. It waits on a spinlock, then
 *   3. Returns from interrupt, restoring the state of the new task at the
 *      head of the ready to run list.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be paused
 *
 * Returned Value:
 *   On success, OK is returned.  Otherwise, a negated errno value indicating
 *   the nature of the failure is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_paused(int cpu);
#endif

/****************************************************************************
 * Name: up_cpu_resume
 *
 * Description:
 *   Restart the cpu after it was paused via up_cpu_pause(), restoring the
 *   state of the task at the head of the g_assignedtasks[cpu] list, and
 *   resume normal tasking.
 *
 *   This function is called after up_cpu_pause in order ot resume operation
 *   of the CPU after modifying its g_assignedtasks[cpu] list.
 *
 * Input Parameters:
 *   cpu - The index of the CPU being resumed.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called from within a critical section; up_cpu_pause() must have
 *   previously been called within the same critical section.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_resume(int cpu);
#endif

/****************************************************************************
 * Name: up_romgetc
 *
 * Description:
 *   In Harvard architectures, data accesses and instruction accesses occur
 *   on different buses, perhaps concurrently.  All data accesses are
 *   performed on the data bus unless special  machine instructions are
 *   used to read data from the instruction address space.  Also, in the
 *   typical MCU, the available SRAM data memory is much smaller that the
 *   non-volatile FLASH instruction memory.  So if the application requires
 *   many constant strings, the only practical solution may be to store
 *   those constant strings in FLASH memory where they can only be accessed
 *   using architecture-specific machine instructions.
 *
 *   A similar case is where strings are retained in "external" memory such
 *   as EEPROM or serial FLASH.  This case is similar only in that again
 *   special operations are required to obtain the string data; it cannot
 *   be accessed directly from a string pointer.
 *
 *   If CONFIG_ARCH_ROMGETC is defined, then the architecture logic must
 *   export the function up_romgetc().  up_romgetc() will simply read one
 *   byte of data from the instruction space.
 *
 *   If CONFIG_ARCH_ROMGETC, certain C stdio functions are effected: (1)
 *   All format strings in printf, fprintf, sprintf, etc. are assumed to
 *   lie in FLASH (string arguments for %s are still assumed to reside in
 *   SRAM). And (2), the string argument to puts and fputs is assumed to
 *   reside in FLASH.  Clearly, these assumptions may have to modified for
 *   the particular needs of your environment.  There is no
 *   "one-size-fits-all" solution for this problem.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ROMGETC
char up_romgetc(FAR const char *ptr);
#else
#  define up_romgetc(ptr) (*ptr)
#endif

/****************************************************************************
 * Name: up_mdelay and up_udelay
 *
 * Description:
 *   Some device drivers may require that the plaform-specific logic
 *   provides these timing loops for short delays.
 *
 ****************************************************************************/

void up_mdelay(unsigned int milliseconds);
void up_udelay(useconds_t microseconds);

/****************************************************************************
 * These are standard interfaces that are exported by the OS for use by the
 * architecture specific logic.
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_process_timer
 *
 * Description:
 *   This function handles system timer events (only when
 *   CONFIG_SCHED_TICKLESS is *not* defined).  The timer interrupt logic
 *   itself is implemented in the architecture specific code, but must call
 *   the following OS function periodically -- the calling interval must
 *   be CONFIG_USEC_PER_TICK.
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
void nxsched_process_timer(void);
#endif

/****************************************************************************
 * Name:  nxsched_timer_expiration
 *
 * Description:
 *   If CONFIG_SCHED_TICKLESS is defined, then this function is provided by
 *   the RTOS base code and called from platform-specific code when the
 *   interval timer used to implement the tick-less OS expires.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   Base code implementation assumes that this function is called from
 *   interrupt handling logic with interrupts disabled.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS) && !defined(CONFIG_SCHED_TICKLESS_ALARM)
void nxsched_timer_expiration(void);
#endif

/****************************************************************************
 * Name:  nxsched_alarm_expiration
 *
 * Description:
 *   if CONFIG_SCHED_TICKLESS is defined, then this function is provided by
 *   the RTOS base code and called from platform-specific code when the
 *   alarm used to implement the tick-less OS expires.
 *
 * Input Parameters:
 *   ts - The time that the alarm expired
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   Base code implementation assumes that this function is called from
 *   interrupt handling logic with interrupts disabled.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS) && defined(CONFIG_SCHED_TICKLESS_ALARM)
void nxsched_alarm_expiration(FAR const struct timespec *ts);
#endif

/****************************************************************************
 * Name: nxsched_process_cpuload
 *
 * Description:
 *   Collect data that can be used for CPU load measurements.  When
 *   CONFIG_SCHED_CPULOAD_EXTCLK is defined, this is an exported interface,
 *   use the the external clock logic.  Otherwise, it is an OS internal
 *   interface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function is called from a timer interrupt handler with all
 *   interrupts disabled.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_CPULOAD) && defined(CONFIG_SCHED_CPULOAD_EXTCLK)
void weak_function nxsched_process_cpuload(void);
#endif

/****************************************************************************
 * Name: nxsched_process_cpuload_ticks
 *
 * Description:
 *   Collect data that can be used for CPU load measurements.  When
 *   CONFIG_SCHED_CPULOAD_EXTCLK is defined, this is an exported interface,
 *   use the the external clock logic.  Otherwise, it is an OS internal
 *   interface.
 *
 * Input Parameters:
 *   ticks - The ticks that we increment in this cpuload
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function is called from a timer interrupt handler with all
 *   interrupts disabled.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_CPULOAD) && defined(CONFIG_SCHED_CPULOAD_EXTCLK)
void weak_function nxsched_process_cpuload_ticks(uint32_t ticks);
#endif

/****************************************************************************
 * Name: irq_dispatch
 *
 * Description:
 *   This function must be called from the achitecture-specific logic in
 *   order to dispatch an interrupt to the appropriate, registered handling
 *   logic.
 *
 ****************************************************************************/

void irq_dispatch(int irq, FAR void *context);

/****************************************************************************
 * Name: up_check_stack and friends
 *
 * Description:
 *   Determine (approximately) how much stack has been used be searching the
 *   stack memory for a high water mark.  That is, the deepest level of the
 *   stack that clobbered some recognizable marker in the stack memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The estimated amount of stack space used.
 *
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
struct tcb_s;
size_t  up_check_tcbstack(FAR struct tcb_s *tcb);
ssize_t up_check_tcbstack_remain(FAR struct tcb_s *tcb);
size_t  up_check_stack(void);
ssize_t up_check_stack_remain(void);
#if defined(CONFIG_ARCH_INTERRUPTSTACK) && CONFIG_ARCH_INTERRUPTSTACK > 3
size_t  up_check_intstack(void);
size_t  up_check_intstack_remain(void);
#endif
#endif

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the builtin, MCU hardware RTC per the selected
 *   configuration.  This function is called once very early in the OS
 *   initialization sequence.
 *
 *   NOTE that initialization of external RTC hardware that depends on the
 *   availability of OS resources (such as SPI or I2C) must be deferred
 *   until the system has fully booted.  Other, RTC-specific initialization
 *   functions are used in that case.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_RTC)
int up_rtc_initialize(void);
#endif

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation is selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC is
 *   set but neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC) && !defined(CONFIG_RTC_HIRES)
time_t up_rtc_time(void);
#endif

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation.
 *   It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC) && defined(CONFIG_RTC_HIRES)
int up_rtc_gettime(FAR struct timespec *tp);
#endif

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface.  However, since the
 *   system time is reinitialized on each power-up/reset, there will be no
 *   timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC) && defined(CONFIG_RTC_DATETIME)
int up_rtc_getdatetime(FAR struct tm *tp);
#endif

/****************************************************************************
 * Name: up_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: This interface exposes sub-second accuracy capability of RTC
 *   hardware. This interface allow maintaining timing accuracy when system
 *   time needs constant resynchronization with RTC, for example on MCU with
 *   low-power state that stop system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_RTC) && defined(CONFIG_RTC_DATETIME) && \
    defined(CONFIG_ARCH_HAVE_RTC_SUBSECONDS)
int up_rtc_getdatetime_with_subseconds(FAR struct tm *tp, FAR long *nsec);
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
int up_rtc_settime(FAR const struct timespec *tp);
#endif

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a PHY interrupt occurs.  This function both attaches
 *   the interrupt handler and enables the interrupt if 'handler' is non-
 *   NULL.  If handler is NULL, then the interrupt is detached and disabled
 *   instead.
 *
 *   The PHY interrupt is always disabled upon return.  The caller must
 *   call back through the enable function point to control the state of
 *   the interrupt.
 *
 *   This interrupt may or may not be available on a given platform depending
 *   on how the network hardware architecture is implemented.  In a typical
 *   case, the PHY interrupt is provided to board-level logic as a GPIO
 *   interrupt (in which case this is a board-specific interface and really
 *   should be called board_phy_irq()); In other cases, the PHY interrupt
 *   may be cause by the chip's MAC logic (in which case arch_phy_irq()) is
 *   an appropriate name.  Other other boards, there may be no PHY interrupts
 *   available at all.  If client attachable PHY interrupts are available
 *   from the board or from the chip, then CONFIG_ARCH_PHY_INTERRUPT should
 *   be defined to indicate that fact.
 *
 *   Typical usage:
 *   a. OS service logic (not application logic*) attaches to the PHY
 *      PHY interrupt and enables the PHY interrupt.
 *   b. When the PHY interrupt occurs:  (1) the interrupt should be
 *      disabled and () work should be scheduled on the worker thread (or
 *      perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *   d. It should then interact the PHY to clear any pending
 *      interrupts, then re-enable the PHY interrupt.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.  Rather applications should use the SIOCMIISIG
 *      ioctl to receive a signal when a PHY event occurs.
 *   ** This interrupt is really of no use if the Ethernet MAC driver
 *      does not support these ioctl calls.
 *
 * Input Parameters:
 *   intf    - Identifies the network interface.  For example "eth0".  Only
 *             useful on platforms that support multiple Ethernet interfaces
 *             and, hence, multiple PHYs and PHY interrupts.
 *   handler - The client interrupt handler to be invoked when the PHY
 *             asserts an interrupt.  Must reside in OS space, but can
 *             signal tasks in user space.  A value of NULL can be passed
 *             in order to detach and disable the PHY interrupt.
 *   arg     - The argument that will accompany the interrupt
 *   enable  - A function pointer that be unused to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
int arch_phy_irq(FAR const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable);
#endif

/****************************************************************************
 * Debug interfaces exported by the architecture-specific logic
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Output one character on the console
 *
 ****************************************************************************/

int up_putc(int ch);

/****************************************************************************
 * Name: up_puts
 *
 * Description:
 *   Output a string on the console
 *
 ****************************************************************************/

void up_puts(FAR const char *str);

/****************************************************************************
 * Name: arch_sporadic_*
 *
 * Description:
 *   Hooks that can be enabled to monitor the behavior of the sporadic
 *   scheduler.  These are call outs from the OS and must be provided by
 *   architecture-specific logic.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread to be restarted.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
void arch_sporadic_start(FAR struct tcb_s *tcb);
void arch_sporadic_lowpriority(FAR struct tcb_s *tcb);
void arch_sporadic_suspend(FAR struct tcb_s *tcb);
void arch_sporadic_resume(FAR struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_perf_*
 *
 * Description:
 *   The first interface simply provides the current time value in unknown
 *   units.  NOTE:  This function may be called early before the timer has
 *   been initialized.  In that event, the function should just return a
 *   start time of zero.
 *
 *   Nothing is assumed about the units of this time value.  The following
 *   are assumed, however: (1) The time is an unsigned integer value, (2)
 *   the time is monotonically increasing, and (3) the elapsed time (also
 *   in unknown units) can be obtained by subtracting a start time from
 *   the current time.
 *
 *   The second interface simple converts an elapsed time into well known
 *   units.
 *
 ****************************************************************************/

void up_perf_init(FAR void *arg);
uint32_t up_perf_gettime(void);
uint32_t up_perf_getfreq(void);
void up_perf_convert(uint32_t elapsed, FAR struct timespec *ts);

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current thread context
 *
 ****************************************************************************/

int up_saveusercontext(FAR void *saveregs);

/****************************************************************************
 * Name: up_fpucmp
 *
 * Description:
 *   compare FPU areas from thread context
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
bool up_fpucmp(FAR const void *saveregs1, FAR const void *saveregs2);
#else
#define up_fpucmp(r1, r2) (true)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ARCH_H */
