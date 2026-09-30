=========
QEMU Tips
=========

Netowrking with TAP device
==========================

Step 1: Configure NuttX network with ``NETUTILS_NETINIT``::

  CONFIG_NETUTILS_NETINIT=y
  CONFIG_NETINIT_IPADDR=0xc0a80868   # Target: 192.168.8.104
  CONFIG_NETINIT_DRIPADDR=0xc0a80801 # Router: 192.168.8.1
  CONFIG_NETINIT_NETMASK=0xffffff00  # Mask:   255.255.255.0

Step 2: Create and configure a TAP device on the host::

  # Create the bridge
  sudo ip link add name br0 type bridge
  sudo ip link set br0 up

  # Create the tap interface
  sudo ip tuntap add dev tap0 mode tap
  sudo ip link set tap0 master br0
  sudo ip link set tap0 up

  # (optional) also attach your real NIC if you want LAN access
  # sudo ip link set enp3s0 master br0

  # Assign IP to the *bridge*
  sudo ip addr add 192.168.8.1/24 dev br0

Step 3: Launch QEMU using the TAP interface::

  qemu-system-x86_64 -m 2G -smp 4 -cpu host -enable-kvm \
  -kernel nuttx -nographic -serial mon:stdio \
  -device e1000,netdev=mynet0 \
  -netdev tap,id=mynet0,ifname=tap0,script=no,downscript=no

Step 4: Clean up::

  sudo ip link set tap0 down
  sudo ip link set br0 down
  sudo ip tuntap del dev tap0 mode tap
  sudo ip link del br0
