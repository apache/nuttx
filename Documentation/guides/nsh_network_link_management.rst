===========================
NSH Network Link Management
===========================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/NSH+Network+Link+Management

In the past, if the network was not connected when NuttX started, two problems
could arise:

1. It could take a very long time for the NSH prompt to appear because of the
   sequential initialization (for example, when the network cable is not
   connected).
2. After NuttX came up, installing the network cable would not enable the
   network; the only way to recover networking was to connect the cable and
   reset the board.

Network link management capability has now been added to NSH to address these
problems.

Configuration Options
=====================

A simple feature can be enabled to move network initialization to a separate
thread so that it is no longer performed sequentially. With this feature, the
network bringup occurs asynchronously, and the NSH prompt appears immediately
(although the network may not be available until some time later). That feature
is enabled with the following setting and is the first prerequisite for the full
NSH link management feature:

- ``CONFIG_NSH_NETINIT_THREAD``. Refer to the help text in the Kconfig file for
  this option (``apps/nshlib/Kconfig``). Additional information about this
  setting is included there.

The logic that implements NSH network management is provided in
``apps/nshlib/nsh_netinit.c``. The behavior of that logic depends on multiple
configuration settings. First, there are some additional prerequisites that
must be satisfied:

- ``CONFIG_NETDEV_PHY_IOCTL``  
  Enable PHY IOCTL commands in the Ethernet device driver. Special IOCTL
  commands must be provided by the Ethernet driver to support certain PHY
  operations needed for link management. These operations are not complex and
  are implemented for Atmel SAM4/4, SAMA5 families, and for the STMicro STM32.
  See ``nuttx/arch/arm/src/sam34/sam_emac.c``,
  ``nuttx/arch/arm/src/sam34/sam_emaca.c``, ``sam_emacb.c``, and ``sam_gmac.c``,
  and ``nuttx/arch/arm/src/stm32/stm32_eth.c``.
- ``CONFIG_ARCH_PHY_INTERRUPT``  
  This is not a user-selectable option. Rather, it is set when selecting a board
  that supports PHY interrupts. In most architectures, the PHY interrupt is not
  directly associated with the Ethernet driver. Instead, the PHY interrupt is
  provided through some board-specific GPIO, and the board-specific logic must
  provide support for that GPIO interrupt. Specifically, the board logic must:
  
  1. Provide the function ``arch_phy_irq()`` as described and prototyped in
     ``nuttx/include/nuttx/arch.h``.
  2. Select ``CONFIG_ARCH_PHY_INTERRUPT`` in the board configuration file to
     advertise that ``arch_phy_irq()`` is supported.
  
  Examples can be found at:
  
  - ``nuttx/boards/arm/sama5/sama5d3x-ek/src/sam_ethernet.c``
  - ``nuttx/boards/arm/sama5/sama5d3-xplained/src/sam_ethernet.c``
  - ``nuttx/boards/arm/sama5/sama5d4-ek/src/sam_ethernet.c``
- Other requirements: UDP support must be enabled (``CONFIG_NET_UDP``), and
  signals must not be disabled (``CONFIG_DISABLE_SIGNALS``).

With all these prerequisites in place, NSH network management can be enabled on
the NSH network initialization thread by selecting these additional options:

- ``CONFIG_NSH_NETINIT_MONITOR``  
  By default, the network initialization thread brings up the network (or
  fails while trying) then exits, freeing all resources it used. If this option
  is selected, however, the network initialization thread will persist
  indefinitely to monitor the network status. Should the network go down (for
  example, if the cable is removed), the thread will monitor the link status
  and attempt to bring the network back up. In this scenario, the resources
  required for network initialization are never released.

If the network monitor is selected, additional options control its behavior:

- ``CONFIG_NSH_NETINIT_SIGNO``  
  The network monitor logic receives signals when there is a change in link
  status. This setting can be used to customize the signal number to avoid
  conflicts.
- ``CONFIG_NSH_NETINIT_RETRYMSEC``  
  When the network is down, the initialization thread will periodically attempt
  to bring the network back up. Because this can be time-consuming, the retry
  operation is performed only at the interval specified by this value, in
  milliseconds.
- ``CONFIG_NSH_NETINIT_THREAD_STACKSIZE``  
  The stack size for the network initialization thread.
- ``CONFIG_NSH_NETINIT_THREAD_PRIORITY``  
  The network initialization thread priority.

Overview of the Operation
=========================

Below is a summary of how the NSH management thread operates:

1. During initialization, the thread opens a UDP socket for IOCTL operations
   and connects a signal handler.
2. It enters a loop. At the beginning of each loop iteration, the thread
   uses an IOCTL command to register (or re-register) with the Ethernet device
   to receive a signal whenever the PHY reports a link up or link down
   interrupt. Re-registration is necessary because the notification disarms
   after each PHY interrupt.
3. The thread reads the link status from both the PHY and the Ethernet device.
   If they disagree, the network monitor uses an IOCTL command to bring the
   Ethernet driver up or down to match the current state of the network. If the
   network is lost, the monitor brings the Ethernet driver down; if the network
   is regained, the monitor brings the Ethernet driver back up and re-establishes
   the connection.
4. If the PHY and the Ethernet driver agree on the link state, no action is
   performed.
5. At the end of the loop, the network monitor waits for a PHY interrupt or a
   timeout. When either occurs, control returns to the top of the loop, and the
   process repeats.
6. If a PHY interrupt happens, a signal is delivered to the task and handled by
   the network monitor’s signal handler, which posts a semaphore to immediately
   re-awaken the network monitor from its wait.
