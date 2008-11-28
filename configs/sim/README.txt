README
^^^^^^

mount
  Configures to use examples/mount.  This configuration may be
  selected as follows:

    cd <nuttx-directory>/tools
    ./Configure.sh sim/mount

nettest

  Configures to use examples/nettest.  This configuration
  enables networking using the network TAP device.  It may
  be selected via:

    cd <nuttx-directory>/tools
    ./Configure.sh sim/nettest

  NOTE: The NuttX network is not, however, functional on the TAP
  device yet.

nsh
  Configures to use the NuttShell at examples/nsh.  This configuration
  may be selected as follows:

    cd <nuttx-directory>/tools
    ./Configure.sh sim/nsh

nx
  Configures to use examples/nx.  This configuration may be
  selected as follows:

    cd <nuttx-directory>/tools
    ./Configure.sh sim/nx

ostest

  The "standard" NuttX examples/ostest configuration.  This
  configuration may be selected as follows:

    cd <nuttx-directory>/tools
    ./Configure.sh sim/ostest

pashello

  Configures to use examples/pashello.  This configuration may
  by selected as follows:

    cd <nuttx-directory>/tools
    ./Configure.sh sim/pashello
