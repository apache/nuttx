=======
irqinfo
=======

``irqinfo`` is a custom GDB command that prints information about the IRQs in the system.
The output includes the IRQ number, the number of times the IRQ has been triggered,
the total time spent in the IRQ handler, the rate of the IRQ, the IRQ handler function,
and the handler's argument.

The argument is displayed as function if possible.

It's similar to nsh command ``irqinfo`` but works in GDB. See :ref:`cmdirqinfo` for more information.

The ``RATE`` column is not available.

.. tip::
    To show the ``COUNT`` column, you need to enable the ``CONFIG_SCHED_IRQMONITOR`` option in the NuttX configuration.

Syntax
------

  ``irqinfo``


Example
-------
.. code-block:: bash

    (gdb) irqinfo
    IRQ  COUNT      TIME   RATE   HANDLER                                          ARGUMENT
    0    0          0      N/A    mps_reserved                             0x0 <sensor_unregister>
    2    0          0      N/A    mps_nmi                                  0x0 <sensor_unregister>
    3    0          0      N/A    arm_hardfault                            0x0 <sensor_unregister>
    4    0          0      N/A    arm_memfault                             0x0 <sensor_unregister>
    5    0          0      N/A    arm_busfault                             0x0 <sensor_unregister>
    6    0          0      N/A    arm_usagefault                           0x0 <sensor_unregister>
    11   1          0      N/A    arm_svcall                               0x0 <sensor_unregister>
    12   0          0      N/A    arm_dbgmonitor                           0x0 <up_debugpoint_remove>
    14   0          0      N/A    mps_pendsv                               0x0 <up_debugpoint_remove>
    15   6581421    0      N/A    systick_interrupt                        0x100010c <g_systick_lower>
    49   2          0      N/A    uart_cmsdk_tx_interrupt                  0x1000010 <g_uart0port>
    50   0          0      N/A    uart_cmsdk_rx_interrupt                  0x1000010 <g_uart0port>
    59   2          0      N/A    uart_cmsdk_ov_interrupt                  0x1000010 <g_uart0port>
    (gdb)
