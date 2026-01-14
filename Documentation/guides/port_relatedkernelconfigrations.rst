The list of related kernel configurations
=========================================

The premise of this list : NuttX12.4.0, cxd56xx, non-SMP and Flat build.

I found following kernel configurations by analyzing the boot sequence.
I think it is good to consider them,
although almost of them might be set as default.

+-----------------+-------------------------------+------------------+
| Category        | Item                          | Comment          |
+-----------------+-------------------------------+------------------+
| Memory Map      | CONFIG_RAM_START              | `arch/Kconfig`_  |
+                 +-------------------------------+                  +
|                 | CONFIG_RAM_SIZE               |                  |
+                 +-------------------------------+------------------+
|                 | CONFIG_IDLETHREAD_STACKSIZE   | `sched/Kconfig`_ |
+                 +-------------------------------+------------------+
|                 | CONFIG_MM_REGIONS             | `mm/Kconfig`_    |
+                 +-------------------------------+------------------+
|                 | CONFIG_ARCH_HAVE_EXTRA_HEAPS  | `arch/Kconfig`_  |
+-----------------+-------------------------------+------------------+
| Interrupt       | CONFIG_ARCH_RAMVECTORS        | `arch/Kconfig`_  |
+                 +-------------------------------+                  +
|                 | CONFIG_ARCH_IRQPRIO           |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_ARCH_INTERRUPTSTACK    |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_SUPPRESS_INTERRUPTS    |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_SUPPRESS_TIMER_INTS    |                  |
+                 +-------------------------------+------------------+
|                 | CONFIG_IRQCHAIN               | `sched/Kconfig`_ |
+-----------------+-------------------------------+------------------+
| Timer           | CONFIG_SYSTEMTICK_EXTCLK      | `sched/Kconfig`_ |
+                 +-------------------------------+                  +
|                 | CONFIG_SCHED_TICKLESS         |                  |
+-----------------+-------------------------------+------------------+
| Serial          | CONFIG_STANDARD_SERIAL        | `serial/Kconfig`_|
+                 +-------------------------------+------------------+
|                 | CONFIG_DEV_CONSOLE            | `sched/Kconfig`_ |
+-----------------+-------------------------------+------------------+
| Board           | CONFIG_BOARD_EARLY_INITIALIZE | `sched/Kconfig`_ |
+                 +-------------------------------+                  +
|                 | CONFIG_BOARD_LATE_INITIALIZE  |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_INIT_ENTRYPOINT        |                  |
+-----------------+-------------------------------+------------------+
| POSIX API       | CONFIG_PRIORITY_INHERITANCE   | `sched/Kconfig`_ |
+                 +-------------------------------+                  +
|                 | CONFIG_SEM_PREALLOCHOLDERS    |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_DISABLE_MQUEUE         |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_DISABLE_MQUEUE_SYSV    |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_PREALLOC_MQ_MSGS       |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_PREALLOC_MQ_IRQ_MSGS   |                  |
+                 +-------------------------------+                  +
|                 | CONFIG_MQ_MAXMSGSIZE          |                  |
+-----------------+-------------------------------+------------------+

.. _arch/Kconfig: https://github.com/apache/nuttx/blob/master/arch/Kconfig
.. _sched/Kconfig: https://github.com/apache/nuttx/blob/master/sched/Kconfig
.. _mm/Kconfig: https://github.com/apache/nuttx/blob/master/mm/Kconfig
.. _serial/Kconfig: https://github.com/apache/nuttx/blob/master/drivers/serial/Kconfig
