README
======

  This is the README file for a port of NuttX to the TM4C1294 Connected Launchpad
  (more correctly, the EK-TM4C1294XL).  For more information about this board,
  see http://www.ti.com/tool/ek-tm4c1294xl

DK-TM4C129X
===========

  This board configuration derives from the DK-T4C129X.  Refer to the README
  file at nuttx/boards/dk-tm4c129x for additional information that may be
  relevant to this board as well.

Serial Console
==============

  These configurations use UART0 for the serial console.  UART0 is connected
  to the on-board TM4C123G-based debugger and is forwarded through the ICDI
  virtual UART.

Configurations
==============

Each EK-TM4C1294XL configuration is maintained in a
sub-directory and can be selected as follow:

    tools/configure.sh tm4c1294-launchpad:<subdir>

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    configuration enables the serial ICDI virtual UART on UART0.  Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the ARM EABI toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARM_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary

    3. Default stack sizes are large and should really be tuned to reduce
       the RAM footprint:

         CONFIG_SCHED_HPWORKSTACKSIZE=2048
         CONFIG_IDLETHREAD_STACKSIZE=1024
         CONFIG_INIT_STACKSIZE=2048
         CONFIG_PTHREAD_STACK_DEFAULT=2048
         CONFIG_POSIX_SPAWN_PROXY_STACKSIZE=1024
         CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE=2048
         CONFIG_NSH_TELNETD_DAEMONSTACKSIZE=2048
         CONFIG_NSH_TELNETD_CLIENTSTACKSIZE=2048

    4. This configuration has the network enabled by default.  See the
       paragraph "Using the network with NSH" in the DK-TM4C129X README).

       Networking can easily be disabled or reconfigured (See see the
       network related configuration settings in the section entitled
       "Networking" in the DK-TM4C129X README).

       By default, this configuration assumes a 10.0.0.xx network.  It
       uses a fixed IP address of 10.0.0.2 and assumes that the host is
       at 10.0.0.1 and that the host provides the default router.  The
       network mask is 255.255.255.0.  These address can be changed by
       modifying the settings in the configuration.  DHCPC can be enabled
       be modifying this default configuration (See the "Networking"
       section in the DK-TM4C129X README).

       The network initialization thread is enabled in this example.  NSH
       will create a separate thread when it starts to initialize the
       network.  This eliminates start-up delays to bring the network.  This
       feature may be disabled by reverting the configuration as described
       under "Network Initialization Thread" in the DK-TM4C129X README.

       The persistent network monitor thread is also available in this
       configuration.  The network monitor will monitor changes in the
       link status and gracefully take the network down when the link is
       lost (for example, if the cable is disconnected) and bring the
       network back up when the link becomes available again (for example,
       if the cable is reconnected).  See the paragraph "Network Monitor"
       in the DK-TM4C129X README for additional information.

  ipv6:
  ----
    This is another version of the NuttShell configuration.  It is very
    similar to the nsh configuration except that it has IPv6 enabled and
    IPv4 disabled.  Several network utilities that are not yet available
    under IPv6 are disabled.

    NOTES:

    1. As of 2015-02-12, this configuration was identical to the nsh
       configuration other than using IPv6.  So all of the notes above
       regarding the nsh configuration apply.

       Telnet does work with IPv6 but is not enabled in this
       configuration (but could be).

    2. This configuration can be modified so that both IPv4 and IPv6
       are supported.  Here is a summary of the additional configuration
       settings required to support both IPv4 and IPv6:

         CONFIG_NET_IPv4=y
         CONFIG_NET_ARP=y
         CONFIG_NET_ARP_SEND=y (optional)
         CONFIG_NET_ICMP=y
         CONFIG_NET_ICMP_SOCKET=y

         CONFIG_NETDB_DNSCLIENT=y
         CONFIG_NETUTILS_TELNETD=y

         CONFIG_NSH_IPADDR=0x0a000002
         CONFIG_NSH_DRIPADDR=0x0a000001
         CONFIG_NSH_NETMASK=0xffffff00
         CONFIG_NSH_TELNET=y

       Then from NSH, you have both ping and ping6 commands:

         nsh> ping 10.0.0.1
         nsh> ping6 fc00::1

       And from the host you can do similar:

         ping 10.0.0.2
         ping6 fc00::2   (Linux)
         ping -6 fc00::2 (Windows cmd)

       and Telnet is now enabled and works from the host... but only using
       IPv6 addressing:

         telnet fc00::2

       That is because the Telnet daemon will default to IPv6 and there is
       no Telnet option to let you select which if both IPv4 and IPv6 are
       enabled.

    3. You can enable IPv6 autonomous address configuration with the
       following changes to the configuration:

       + CONFIG_NET_ICMPv6_AUTOCONF=y
       + CONFIG_ICMPv6_AUTOCONF_DELAYMSEC=100
       + CONFIG_ICMPv6_AUTOCONF_MAXTRIES=5

       - CONFIG_NSH_DRIPv6ADDR_1=0xfc00
       - CONFIG_NSH_DRIPv6ADDR_2=0x0000
       - CONFIG_NSH_DRIPv6ADDR_3=0x0000
       - CONFIG_NSH_DRIPv6ADDR_4=0x0000
       - CONFIG_NSH_DRIPv6ADDR_5=0x0000
       - CONFIG_NSH_DRIPv6ADDR_6=0x0000
       - CONFIG_NSH_DRIPv6ADDR_7=0x0000
       - CONFIG_NSH_DRIPv6ADDR_8=0x0001

       - CONFIG_NSH_IPv6ADDR_1=0xfc00
       - CONFIG_NSH_IPv6ADDR_2=0x0000
       - CONFIG_NSH_IPv6ADDR_3=0x0000
       - CONFIG_NSH_IPv6ADDR_4=0x0000
       - CONFIG_NSH_IPv6ADDR_5=0x0000
       - CONFIG_NSH_IPv6ADDR_6=0x0000
       - CONFIG_NSH_IPv6ADDR_7=0x0000
       - CONFIG_NSH_IPv6ADDR_8=0x0002
       - CONFIG_NSH_IPv6NETMASK_1=0xffff
       - CONFIG_NSH_IPv6NETMASK_2=0xffff
       - CONFIG_NSH_IPv6NETMASK_3=0xffff
       - CONFIG_NSH_IPv6NETMASK_4=0xffff
       - CONFIG_NSH_IPv6NETMASK_5=0xffff
       - CONFIG_NSH_IPv6NETMASK_6=0xffff
       - CONFIG_NSH_IPv6NETMASK_7=0xffff
       - CONFIG_NSH_IPv6NETMASK_8=0xff80
