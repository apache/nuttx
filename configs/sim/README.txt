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

  Special simulated framebuffer configuration options:

  CONFIG_SIM_X11FB    - Use X11 window for framebuffer
  CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
  CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
  CONFIG_SIM_FBBPP    - Pixel depth in bits

  NOTES:
  - If CONFIG_SIM_X11FB is selected then CONFIG_SIM_FBBPP must
    match the resolution of the display.
  - For whatever value of CONFIG_SIM_FBBPP is selected, then
    the corresponing CONFIG_NXGLIB_DISABLE_*BPP setting must
    not be disabled.
  - The default in defconfig is to use a generic memory buffer
    for the framebuffer.  defconfig-x11 is an example with X11
    support enabled.

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
