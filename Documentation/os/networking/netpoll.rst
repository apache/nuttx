=======
Netpoll
=======

Netpoll is a tool inspired by a similar tool in Linux that allows you to use
network interfaces in places where the network stack is normally inaccessible,
such as interrupts.

It allows sending and receiving UDP packets with minimal kernel involvement,
which can be used to implement kernel debugging features over network interface.

The NuttX implementation is based on the ``struct netdev_lowerhalf_s`` which
means that the network interface using netpoll must be based on
the :ref:`Netdev Lowerhalf <netdriver>` implementation.
