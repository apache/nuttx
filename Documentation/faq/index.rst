.. todo::
  Create new subsections as needed.

===
FAQ
===

NSH Tricks
==========

How to interrupt an NSH Application using Ctrl^C ?
--------------------------------------------------

You need to enable these options in the menuconfig:

    RTOS Features --->
        Signal Configuration --->
            [*] Default signal actions --->
                [*] SIGINT and SIGKILL
    Device Drivers --->
        Serial Driver Support --->
            [*] Serial TERMIOS support
            [*]   Support SIGINT
            (0x03)  Serial parse SIGINT characters

Network
=======

How to detect Ethernet cable connection/disconnection?
------------------------------------------------------

NuttX supports Ethernet connection/disconnection events from Ethernet PHY
using signals (see ``nuttx/drivers/net/phy_notify.c``).
The network monitor thread at apps/netutils/netinit (see
``CONFIG_NETINIT_MONITOR``) will handle taking the network down if the cable
is unplugged and bringing it back up when the cable is restored.
It is important to note that your Ethernet controller driver of your
MCU needs to support CONFIG_ARCH_PHY_INTERRUPT (and implement
``arch_phy_irq()``).

