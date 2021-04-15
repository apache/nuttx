.. todo::
  Create new subsections as needed.

===
FAQ
===

NSH Tricks
==========

How to increase the command line length?
----------------------------------------

You can increase the Max command line length from 64 to other value,
this way:

Application Configuration  --->
        NSH Library  --->
            Command Line Configuration  --->
                (64) Max command line length

How do I enable editing support on the command line?
----------------------------------------------------

You need to change Command Line Editor from "Minimal readline" to
"Command Line Editor", this way:

    Application Configuration  --->
        NSH Library  --->
            Command Line Configuration  --->
                Command Line Editor (Command Line Editor)  --->

How to enable command line history?
-----------------------------------

You need to enable these options in the menuconfig:

    Application Configuration  --->
        System Libraries and NSH Add-Ons  --->
            -*- readline() Support  --->
                [*]     Command line history
                (80)      Command line history length
                (16)      Command line history records

Note: If you are using the "Command Line Editor" instead of the "readline"
then you need to use this other option:

    Application Configuration  --->
        System Libraries and NSH Add-Ons  --->
            -*- EMACS-like Command Line Editor  --->
                	[*]   Command line history
                	(80)    Command line history length
                	(16)    Command line history records

How to enable autocomplete on the command line?
-----------------------------------------------

You need to enable these options in the menuconfig:

    Application Configuration  --->
        System Libraries and NSH Add-Ons  --->
            -*- readline() Support  --->
                [*]     Tab completion
                (64)      Maximum built-in matches
                (64)      Maximum external command matches

Note: autocomplete is not enabled when "Command Line Editor" instead of the
"readline".

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

