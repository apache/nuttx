/****************************************************************************
 * include/nuttx/arch.h
 *
 *   Copyright (C) 2007-2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_ARCH_H
#define __INCLUDE_NUTTX_ARCH_H

/* This header file contains function prototypes for the interfaces between
 * (1) the nuttx core-code, (2) the microprocessor specific logic that
 * resides under the arch/ sub-directory, and (3) the board-specific logic
 * that resides under configs/
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
 *    NOTE: up_ is supposed to stand for microprocessor; the u is like the
 *    Greek letter micron: µ. So it would be µP which is a common shortening
 *    of the word microprocessor.
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
 *    with board_ and should be prototyped in this header file. These
 *    board_ definitions provide the interface between the board-level
 *    logic and the architecture-specific logic.
 *
 *    There is also a configs/<board>/include/board.h header file that
 *    can be used to communicate other board-specific information between
 *    the architecture logic and even application logic.  Any definitions
 *    which are common between a single architecture and several boards
 *    should go in this board.h header file; this file is reserved for
 *    board-related definitions common to all architectures.
 *
 * 4. Board-Specific Interfaces.
 *
 *    Any interface which is unique to a board should be prefixed with
 *    the board name, for example stm32f4discovery_. Sometimes the board
 *    name is too long so stm32_ would be okay too. These should be
 *    prototyped in configs/<board>/src/<board>.h and should not be used
 *    outside of that board directory since board-specific definitions
 *    have no meaning outside of the board directory.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>

#include <arch/arch.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*sig_deliver_t)(FAR struct tcb_s *tcb);

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

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
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void);
#endif

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed
 *   when their is no other ready-to-run task.  This is processor
 *   idle time and will continue until some interrupt occurs to
 *   cause a context switch from the idle task.
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
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the intial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
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
 *   - adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The initial value of
 *     the stack pointer.
 *
 * Inputs:
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
 *     If CONFIG_NUTTX_KERNEL is defined, then this thread type may affect
 *     how the stack is allocated.  For example, kernel thread stacks should
 *     be allocated from protected kernel memory.  Stacks for user tasks and
 *     threads must come from memory that is accessible to user code.
 *
 ****************************************************************************/

#ifndef CONFIG_CUSTOM_STACK
int up_create_stack(FAR struct tcb_s *tcb, size_t stack_size, uint8_t ttype);
#endif

/****************************************************************************
 * Name: up_use_stack
 *
 * Description:
 *   Setup up stack-related information in the TCB using pre-allocated stack
 *   memory.  This function is called only from task_init() when a task or
 *   kernel thread is started (never for pthreads).
 *
 *   The following TCB fields must be initialized:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
 *
 * Inputs:
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

#ifndef CONFIG_CUSTOM_STACK
int up_use_stack(FAR struct tcb_s *tcb, FAR void *stack, size_t stack_size);
#endif

/****************************************************************************
 * Name: up_stack_frame
 *
 * Description:
 *   Allocate a stack frame in the TCB's stack to hold thread-specific data.
 *   This function may be called anytime after up_create_stack() or
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
 *   - adj_stack_ptr: Adjusted initial stack pointer after the frame has
 *     been removed from the stack.  This will still be the initial value
 *     of the stack pointer when the task is started.
 *
 * Inputs:
 *   - tcb:  The TCB of new task
 *   - frame_size:  The size of the stack frame to allocate.
 *
 *  Returned Value:
 *   - A pointer to bottom of the allocated stack frame.  NULL will be
 *     returned on any failures.  The alignment of the returned value is
 *     the same as the alignment of the stack itself.
 *
 ****************************************************************************/

#if !defined(CONFIG_CUSTOM_STACK) && defined(CONFIG_NUTTX_KERNEL)
FAR void *up_stack_frame(FAR struct tcb_s *tcb, size_t frame_size);
#endif

/****************************************************************************
 * Name: up_release_stack
 *
 * Description:
 *   A task has been stopped. Free all stack related resources retained in
 *   the defunct TCB.
 *
 * Input Parmeters
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
 *     If CONFIG_NUTTX_KERNEL is defined, then this thread type may affect
 *     how the stack is freed.  For example, kernel thread stacks may have
 *     been allocated from protected kernel memory.  Stacks for user tasks
 *     and threads must have come from memory that is accessible to user
 *     code.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_CUSTOM_STACK
void up_release_stack(FAR struct tcb_s *dtcb, uint8_t ttype);
#endif

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
 * Inputs:
 *   tcb: Refers to the tcb to be unblocked.  This tcb is
 *     in one of the waiting tasks lists.  It must be moved to
 *     the ready-to-run list and, if it is the highest priority
 *     ready to run taks, executed.
 *
 ****************************************************************************/

void up_unblock_task(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: up_block_task
 *
 * Description:
 *   The currently executing task at the head of
 *   the ready to run list must be stopped.  Save its context
 *   and move it to the inactive list specified by task_state.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Inputs:
 *   tcb: Refers to a task in the ready-to-run list (normally
 *     the task at the head of the list).  It most be
 *     stopped, its context saved and moved into one of the
 *     waiting task lists.  It it was the task at the head
 *     of the ready-to-run list, then a context to the new
 *     ready to run task must be performed.
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
 *      the priority of the current, running task and it now has the
 *      priority.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Inputs:
 *   tcb: The TCB of the task that has been reprioritized
 *   priority: The new task priority
 *
 ****************************************************************************/

void up_reprioritize_rtr(FAR struct tcb_s *tcb, uint8_t priority);

/****************************************************************************
 * Name: _exit
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
/* Prototype is in unistd.h */

/****************************************************************************
 * Name: up_assert
 *
 * Description:
 *   Assertions may be handled in an architecture-specific way.
 *
 ****************************************************************************/
/* Prototype is in assert.h */

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'igdeliver' callback is executed on the thread
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

#ifndef CONFIG_DISABLE_SIGNALS
void up_schedule_sigaction(FAR struct tcb_s *tcb, sig_deliver_t sigdeliver);
#endif

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

#if defined(CONFIG_NUTTX_KERNEL) && defined(__KERNEL__)
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
 *   pthread actually starts.  See libc/pthread/pthread_startup.c
 *
 * Input Parameters:
 *   entrypt - The user-space address of the pthread entry point
 *   arg     - Standard argument for the pthread entry point
 *
 * Returned Value:
 *   This function should not return.  It should call the user-mode start-up
 *   stub and that stub should call pthread_exit if/when the user pthread
 *   terminates.
 *
 ****************************************************************************/

#if defined(CONFIG_NUTTX_KERNEL) && defined(__KERNEL__) && !defined(CONFIG_DISABLE_PTHREAD)
void up_pthread_start(pthread_startroutine_t entrypt, pthread_addr_t arg)
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
 * Inputs:
 *   sighand - The address user-space signal handling function
 *   signo, info, and ucontext - Standard arguments to be passed to the
 *     signal handling function.
 *
 * Return:
 *   None.  This function does not return in the normal sense.  It returns
 *   via an architecture specific system call made by up_signal_handler()
 *   (see below).  However, this will look like a normal return by the
 *   caller of up_signal_dispatch.
 *
 ****************************************************************************/

#if defined(CONFIG_NUTTX_KERNEL) && defined(__KERNEL__) && !defined(CONFIG_DISABLE_SIGNALS)
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
 * Inputs:
 *   sighand - The address user-space signal handling function
 *   signo, info, and ucontext - Standard arguments to be passed to the
 *     signal handling function.
 *
 * Return:
 *   None.  This function does not return in the normal sense.  It returns
 *   via an architecture specific system call.
 *
 ****************************************************************************/

#if defined(CONFIG_NUTTX_KERNEL) && !defined(__KERNEL__) && !defined(CONFIG_DISABLE_SIGNALS)
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
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
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
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel-space heap.
 *
 ****************************************************************************/

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size);
#endif

/****************************************************************************
 * Name: up_setpicbase, up_getpicbase
 *
 * Description:
 *   It NXFLAT external modules (or any other binary format that requires)
 *   PIC) are supported, then these macros must defined to (1) get or get
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
 * environments.  These interfaces all operate on type task_addrenv_t which
 * is an abstract representation of a task's address environment and must be
 * defined in arch/arch.h if CONFIG_ADDRENV is defined.
 *
 *   up_addrenv_create  - Create an address environment
 *   up_addrenv_vaddr   - Returns the virtual base address of the address
 *                        environment
 *   up_addrenv_select  - Instantiate an address environment
 *   up_addrenv_restore - Restore an address environment
 *   up_addrenv_destroy - Destroy an address environment.
 *   up_addrenv_assign  - Assign an address environment to a TCB
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the TCB which as been
 * assigned an address environment by up_addrenv_assign().
 *
 *   up_addrenv_share   - Clone the address environment assigned to one TCB
 *                        to another.  This operation is done when a pthread
 *                        is created that share's the same address
 *                        environment.
 *   up_addrenv_release - Release the TCBs reference to an address
 *                        environment when a task/thread exits.
 *
 ****************************************************************************/
/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called from the binary loader logic when a new
 *   task is created in order to instantiate an address environment for the
 *   task.  up_addrenv_create is essentially the allocator of the physical
 *   memory for the new task.
 *
 * Input Parameters:
 *   envsize - The size (in bytes) of the address environment needed by the
 *     task.
 *   addrenv - The location to return the representation of the task address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_create(size_t envsize, FAR task_addrenv_t *addrenv);
#endif

/****************************************************************************
 * Name: up_addrenv_vaddr
 *
 * Description:
 *   Return the virtual address associated with the newly create address
 *   environment.  This function is used by the binary loaders in order
 *   get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vaddr - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_vaddr(FAR task_addrenv_t addrenv, FAR void **vaddr);
#endif

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task (via
 *   up_addrenv_create().  This function may be called to to instantiate
 *   that address environment in the virtual address space.  this might be
 *   necessary, for example, to load the code for the task from a file or
 *   to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *   oldenv
 *     The address environment that was in place before up_addrenv_select().
 *     This may be used with up_addrenv_restore() to restore the original
 *     address environment that was in place before up_addrenv_select() was
 *     called.  Note that this may be a task agnostic, hardware
 *     representation that is different from task_addrenv_t.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_select(task_addrenv_t addrenv, hw_addrenv_t *oldenv);
#endif

/****************************************************************************
 * Name: up_addrenv_restore
 *
 * Description:
 *   After an address environment has been temporarilty instantiated by
 *   up_addrenv_select, this function may be called to to restore the
 *   original address environment.
 *
 * Input Parameters:
 *   oldenv - The hardware representation of the address environment
 *     previously returned by up_addrenv_select.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_restore(hw_addrenv_t oldenv);
#endif

/****************************************************************************
 * Name: up_addrenv_destroy
 *
 * Description:
 *   Called from the binary loader loader during error handling to destroy
 *   the address environment previously created by up_addrenv_create().
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_destroy(task_addrenv_t addrenv);
#endif

/****************************************************************************
 * Name: up_addrenv_assign
 *
 * Description:
 *   Assign an address environment to a TCB.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *   tcb - The TCB of the task to receive the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_assign(task_addrenv_t addrenv, FAR struct tcb_s *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_share
 *
 * Description:
 *   This function is called from the core scheduler logic when a thread
 *   is created that needs to share the address ennvironment of its parent
 *   task.  In this case, the parent's address environment needs to be
 *   "cloned" for the child.
 *
 * Input Parameters:
 *   ptcb - The TCB of the parent task that has the address environment.
 *   ctcb - The TCB of the child thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_share(FAR const struct tcb_s *ptcb, FAR struct tcb_s *ctcb);
#endif

/****************************************************************************
 * Name: up_addrenv_release
 *
 * Description:
 *   This function is called when a task or thread exits in order to release
 *   its reference to an address environment.  When there are no further
 *   references to an address environment, that address environment should
 *   be destroyed.
 *
 * Input Parameters:
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_release(FAR struct tcb_s *tcb);
#endif

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
 *   (irqrestore() supports the global level, the device level is hardware
 *   specific).
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
 *   (irqsave() supports the global level, the device level is hardware
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

/****************************************************************************
 * Tickless OS Support.
 *
 * When CONFIG_SCHED_TICKLESS is enabled, all support for timer interrupts
 * is suppressed and the platform specific code is expected to provide the
 * following custom functions.
 *
 *   void up_timer_initialize(void): Initializes the timer facilities.  Called
 *     early in the intialization sequence (by up_intialize()).
 *   int up_timer_gettime(FAR struct timespec *ts):  Returns the current
 *     time from the platform specific time source.
 *
 * The tickless option can be supported either via a simple interval timer
 * (plus elapsed time) or via an alarm.  The interval timer allows programming
 * events to occur after an interval.  With the alarm, you can set a time in
 * the future and get an event when that alarm goes off.
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
 *   void sched_alarm_expiration(FAR const struct timespec *ts):  Called
 *     by the platform-specific logic when the alarm expires.
 * #else
 *   void sched_timer_expiration(void):  Called by the platform-specific
 *     logic when the interval timer expires.
 * #endif
 *
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.  This function is
 *   called early in the initialization sequence by up_intialize().
 *   On return, the current up-time should be available from
 *   up_timer_gettime() and the interval timer is ready for use (but not
 *   actively timing).
 *
 *   Provided by platform-specific code and called from the architecture-
 *   specific logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence before any special
 *   concurrency protections are required.
 *
 ****************************************************************************/

#if 0 /* Prototyped in up_internal.h in all cases. */
void up_timer_initialize(void);
#endif

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   up_timer_initialize() was called).  This function is functionally
 *   equivalent to:
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
 *   Called from the the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int up_timer_gettime(FAR struct timespec *ts);
#endif

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Cancel the alarm and return the time of cancellation of the alarm.
 *   These two steps need to be as nearly atomic as possible.
 *   sched_alarm_expiration() will not be called unless the alarm is
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
 *   Start the alarm.  sched_alarm_expiration() will be called when the
 *   alarm occurs (unless up_alaram_cancel is called to stop it).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - The time in the future at the alarm is expected to occur.  When
 *        the alarm occurs the timer logic will call sched_alarm_expiration().
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
 *   sched_timer_expiration() will not be called unless the timer is
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
 *   Start the interval timer.  sched_timer_expiration() will be called at
 *   the completion of the timeout (unless up_timer_cancel is called to stop
 *   the timing.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until sched_timer_expiration() is
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
 * Name: up_romgetc
 *
 * Description:
 *   In Harvard architectures, data accesses and instruction accesses occur
 *   on different busses, perhaps concurrently.  All data accesses are
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
 *   the particular needs of your environment.  There is no "one-size-fits-all"
 *   solution for this problem.
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
 *   provide these timing loops for short delays.
 *
 ***************************************************************************/

void up_mdelay(unsigned int milliseconds);
void up_udelay(useconds_t microseconds);

/****************************************************************************
 * Name: up_cxxinitialize
 *
 * Description:
 *   If C++ and C++ static constructors are supported, then this function
 *   must be provided by board-specific logic in order to perform
 *   initialization of the static C++ class instances.
 *
 *   This function should then be called in the application-specific
 *   user_start logic in order to perform the C++ initialization.  NOTE
 *   that no component of the core NuttX RTOS logic is involved; This
 *   function definition only provides the 'contract' between application
 *   specific C++ code and platform-specific toolchain support
 *
 ***************************************************************************/

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
void up_cxxinitialize(void);
#endif

/****************************************************************************
 * These are standard interfaces that are exported by the OS for use by the
 * architecture specific logic
 ****************************************************************************/

/****************************************************************************
 * Name: sched_process_timer
 *
 * Description:
 *   This function handles system timer events (only when
 *   CONFIG_SCHED_TICKLESS is *not* defined).  The timer interrupt logic
 *   itself is implemented in the architecture specific code, but must call
 *   the following OS function periodically -- the calling interval must
 *   be MSEC_PER_TICK.
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
void sched_process_timer(void);
#endif

/****************************************************************************
 * Name:  sched_timer_expiration
 *
 * Description:
 *   if CONFIG_SCHED_TICKLESS is defined, then this function is provided by
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
void sched_timer_expiration(void);
#endif

/****************************************************************************
 * Name:  sched_alarm_expiration
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
void sched_alarm_expiration(FAR const struct *ts);
#endif

/************************************************************************
 * Name: sched_process_cpuload
 *
 * Description:
 *   Collect data that can be used for CPU load measurements.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function is called from a timer interrupt handler with all
 *   interrupts disabled.
 *
 ************************************************************************/

#if defined(CONFIG_SCHED_CPULOAD) && defined(CONFIG_SCHED_CPULOAD_EXTCLK)
void weak_function sched_process_cpuload(void);
#endif

/****************************************************************************
 * Name: irq_dispatch
 *
 * Description:
 *   This function must be called from the achitecture-specific logic in
 *   order to dispatch an interrupt to the appropriate, registered handling
 *   logic.
 *
 ***************************************************************************/

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
 * Returned value:
 *   The estimated amount of stack space used.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_STACK)
struct tcb_s;
size_t  up_check_tcbstack(FAR struct tcb_s *tcb);
ssize_t up_check_tcbstack_remain(FAR struct tcb_s *tcb);
size_t  up_check_stack(void);
ssize_t up_check_stack_remain(void);
#if CONFIG_ARCH_INTERRUPTSTACK > 3
size_t  up_check_intstack(void);
size_t  up_check_intstack_remain(void);
#endif
#endif

/****************************************************************************
 * Board-specific button interfaces exported by the board-specific logic
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state of
 *   all buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports button interfaces, then CONFIG_ARCH_BUTTONS
 *   will be defined.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void board_button_initialize(void);
#endif

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  A bit set to
 *   "1" means that the button is depressed; a bit set to "0" means that
 *   the button is released.  The correspondence of the each button bit
 *   and physical buttons is board-specific.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports button interfaces, then
 *   CONFIG_ARCH_BUTTONS will be defined
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
uint8_t board_buttons(void);
#endif

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource.
 *   The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports any button interfaces, then
 *   CONFIG_ARCH_BUTTONS will be defined; If the board supports interrupt
 *   buttons, then CONFIG_ARCH_IRQBUTTONS will also be defined.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
xcpt_t board_button_irq(int id, xcpt_t irqhandler);
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
 *      PHY interrupt.
 *   b. When the PHY interrupt occurs, work should be scheduled on the
 *      worker thread (or perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.
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
 *
 * Returned Value:
 *   The previous PHY interrupt handler address is returned.  This allows you
 *   to temporarily replace an interrupt handler, then restore the original
 *   interrupt handler.  NULL is returned if there is was not handler in
 *   place when the call was made.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
xcpt_t arch_phy_irq(FAR const char *intf, xcpt_t handler);
#endif

/************************************************************************************
 * Relay control functions
 *
 * Description:
 *   Non-standard functions for relay control.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_RELAYS
void up_relaysinit(void);
void relays_setstat(int relays, bool stat);
bool relays_getstat(int relays);
void relays_setstats(uint32_t relays_stat);
uint32_t relays_getstats(void);
void relays_onoff(int relays, uint32_t mdelay);
void relays_onoffs(uint32_t relays_stat, uint32_t mdelay);
void relays_resetmode(int relays);
void relays_powermode(int relays);
void relays_resetmodes(uint32_t relays_stat);
void relays_powermodes(uint32_t relays_stat);
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
 * Name: up_getc
 *
 * Description:
 *   Get one character on the console
 *
 ****************************************************************************/

int up_getc(void);

/****************************************************************************
 * Name: up_puts
 *
 * Description:
 *   Output a string on the console
 *
 ****************************************************************************/

void up_puts(FAR const char *str);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ARCH_H */

