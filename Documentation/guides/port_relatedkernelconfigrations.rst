The list of related kernel configurations
=========================================

The premise of this list : NuttX12.4.0, cxd56xx, non-SMP and Flat build.

I found following kernel configurations by analyzing the boot sequence. I think it is good to consider them, although almost of them might be set as default.

+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+
| Category        | Item                          | Comment                                                                                                             |
+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+
| Memory Map      | CONFIG_RAM_START              | see : https://github.com/apache/nuttx/blob/master/arch/Kconfig                                                      |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_RAM_SIZE               |                                                                                                                     |
+                 +-------------------------------+---------------------------------------------------------------------------------------------------------------------+
|                 | CONFIG_IDLETHREAD_STACKSIZE   | see : https://github.com/apache/nuttx/blob/master/sched/Kconfig                                                     |
+                 +-------------------------------+---------------------------------------------------------------------------------------------------------------------+
|                 | CONFIG_MM_REGIONS             | see : https://github.com/apache/nuttx/blob/master/mm/Kconfig                                                        |
+                 +-------------------------------+---------------------------------------------------------------------------------------------------------------------+
|                 | CONFIG_ARCH_HAVE_EXTRA_HEAPS  | see : https://github.com/apache/nuttx/blob/master/arch/Kconfig                                                      |
+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+
| Interrupt       | CONFIG_ARCH_RAMVECTORS        | see : https://github.com/apache/nuttx/blob/master/arch/Kconfig                                                      |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_ARCH_IRQPRIO           |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_ARCH_INTERRUPTSTACK    |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_SUPPRESS_INTERRUPTS    |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_SUPPRESS_TIMER_INTS    |                                                                                                                     |
+                 +-------------------------------+---------------------------------------------------------------------------------------------------------------------+
|                 | CONFIG_ARMV7M_USEBASEPRI      | see : https://github.com/apache/nuttx/blob/master/arch/arm/src/armv7-m/Kconfig                                      |
+                 +-------------------------------+---------------------------------------------------------------------------------------------------------------------+
|                 | CONFIG_IRQCHAIN               | see : https://github.com/apache/nuttx/blob/master/sched/Kconfig                                                     |
+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+
| Timer           | CONFIG_SYSTEMTICK_EXTCLK      | see : https://github.com/apache/nuttx/blob/master/sched/Kconfig                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_SCHED_TICKLESS         |                                                                                                                     |
+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+
| Serial          | CONFIG_STANDARD_SERIAL        | see : https://github.com/apache/nuttx/blob/master/drivers/serial/Kconfig                                            |
+                 +-------------------------------+---------------------------------------------------------------------------------------------------------------------+
|                 | CONFIG_DEV_CONSOLE            | see : https://github.com/apache/nuttx/blob/master/sched/Kconfig                                                     |
+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+
| Board           | CONFIG_BOARD_EARLY_INITIALIZE | see : https://github.com/apache/nuttx/blob/master/sched/Kconfig                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_BOARD_LATE_INITIALIZE  |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_INIT_ENTRYPOINT        |                                                                                                                     |
+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+
| POSIX API       | CONFIG_PRIORITY_INHERITANCE   | see : https://github.com/apache/nuttx/blob/master/sched/Kconfig                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_SEM_PREALLOCHOLDERS    |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_DISABLE_MQUEUE         |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_DISABLE_MQUEUE_SYSV    |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_PREALLOC_MQ_MSGS       |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_PREALLOC_MQ_IRQ_MSGS   |                                                                                                                     |
+                 +-------------------------------+                                                                                                                     +
|                 | CONFIG_MQ_MAXMSGSIZE          |                                                                                                                     |
+-----------------+-------------------------------+---------------------------------------------------------------------------------------------------------------------+



