The diagram of boot sequence
============================

The premise of this sequence diagram is NuttX12.4.0, cxd56xx, non-SMP and Flat build.

The beginning of boot sequence is ``__start()``.

``__start()`` will call ``nx_start()``. ``nx_start()`` will initialize the
kernel and call ``nsh_main()``. ``nsh_main()`` will execute NuttShell(NSH).
``nx_start()`` is NuttX standard function, but the behavior depends on some
kernel configurations. For example, the calling ``nsh_main()`` is also
configurable. About related kernel configurations, see
:doc:`/guides/port_relatedkernelconfigrations`.

.. uml::

   bootloader --> cxd56_start.c : Jump to __start by function call or de-assert the reset signal of CM4F

   note over cxd56_start.c : Initialize SP
   note over cxd56_start.c : Disable IRQ @processor level
   note over cxd56_start.c : Initialize .data/.bss
   note over cxd56_start.c : cxd56_board_initialize() as <arch>_board_initialize()

   cxd56_start.c --> nx_start.c : nx_start()

   note over nx_start.c : Initialize IDLE task TCB
   note over nx_start.c : nxsem_initialize()
   note over nx_start.c : Initialize HEAP
   note over nx_start.c : Create and initialize IDLE group instance
   note over nx_start.c : sched_lock()
   note over nx_start.c : fs_initialize()
   note over nx_start.c : irq_initialize()
   note over nx_start.c : clock_initialize()
   note over nx_start.c : timer_initialize()
   note over nx_start.c : nxsig_initialize()
   note over nx_start.c : nxmq_initialize()
   note over nx_start.c : net_initialize()
   note over nx_start.c : binfmt_initialize()
   note over nx_start.c : up_initialize()
   note over nx_start.c : drivers_initialize()
   note over nx_start.c : board_early_initialize()
   note over nx_start.c : Initialize SDIO for IDLE task

   nx_start.c --> nx_bringup.c : nx_bringup()

   note over nx_bringup.c : Initialize environment variables (PWD, PATH, LD_LIBRARY_PATH)
   note over nx_bringup.c : Start work queues

   nx_bringup.c --> nx_start_application : nx_start_task()

   note over nx_start_application : board_late_initialize()
   note over nx_start_application : coredump_initialize()
   note over nx_start_application : Start nsh_main() as the application initialization task

   note over nx_start.c : up_idle()
