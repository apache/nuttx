===========
About NuttX
===========

Goals
=====

NuttX is a real time embedded operating system (RTOS). Its goals are:

* **Small Footprint**
  Usable in all but the tightest micro-controller environments, the focus is on the
  tiny-to-small, deeply embedded environment.

* **Rich Feature OS Set**
  The goal is to provide implementations of most standard POSIX OS interfaces to support a
  rich, multi-threaded development environment for deeply embedded processors.

  NON-GOALS: It is not a goal to provide the level of OS features like those provided by Linux. In order to work with
  smaller MCUs, small footprint must be more important than an extensive feature set. But standard compliance is more
  important than small footprint. Surely a smaller RTOS could be produced by ignoring standards. Think of NuttX is a
  tiny Linux work-alike with a much reduced feature set.

* **Highly Scalable**
  Fully scalable from tiny (8-bit) to moderate embedded (64-bit). Scalability with rich feature set
  is accomplished with: Many tiny source files, link from static libraries, highly configurable,
  use of weak symbols when available.

* **Standards Compliance**
  NuttX strives to achieve a high degree of standards compliance. The primary governing standards
  are POSIX and ANSI standards. Additional standard APIs from Unix and other common RTOS's are
  adopted for functionality not available under these standards or for functionality that is not
  appropriate for the deeply-embedded RTOS (such as ``fork()``).

  Because of this standards conformance, software developed under other standard OSs (such as
  Linux) should port easily to NuttX.

* **Real-Time**
  Fully pre-emptible; fixed priority, round-robin, and "sporadic" scheduling.

* **Totally Open**
  Non-restrictive Apache license.

* **GNU Toolchains**
  Compatible GNU toolchains based on `buildroot <http://buildroot.uclibc.org/>`__ available for `download <https://bitbucket.org/nuttx/buildroot/downloads/>`__
  to provide a complete development environment for many architectures.

Feature Set
===========

Key features of NuttX include:

* **Standards Compliant Core Task Management**

  * Fully pre-emptible.
  * Naturally scalable.
  * Highly configurable.
  * Easily extensible to new processor architectures, SoC architecture, or board architectures. :doc:`/reference/os/index` is available.
  * FIFO and round-robin scheduling.
  * Realtime, deterministic, with support for priority inheritance
  * Tickless Operation
  * POSIX/ANSI-like task controls, named message queues, counting semaphores, clocks/timers,
    signals, pthreads, robust mutexes, cancellation points, environment variables, filesystem.
  * Standard default signal actions (optional).
  * VxWorks-like task management and watchdog timers.
  * BSD socket interface.
  * Extensions to manage pre-emption.
  * Optional tasks with address environments (*Processes*).
  * Loadable kernel modules; lightweight, embedded shared libraries.
  * Memory Configurations: (1) Flat embedded build, (2) Protected build with MPU, and (3) Kernel build with MMU.
  * Memory Allocators: (1) standard heap memory allocation, (2) granule allocator, (3) shared memory, and
    (4) dynamically sized, per-process heaps.
  * Inheritable "controlling terminals" and I/O re-direction.
  * Pseudo-terminals
  * On-demand paging.
  * System logging.
  * May be built either as an open, flat embedded RTOS or as a separately built, secure, monolithic kernel with a
    system call interface.
  * Built-in, per-thread CPU load measurements.
  * Well documented in the NuttX User Guide.

* **File system**

  * Tiny, in-memory, root pseudo-file-system.
  * Virtual file system (VFS) supports drivers and mountpoints.
  * Mount-able volumes. Bind mountpoint, filesystem, and block device driver.
  * Generic system logging (SYSLOG) support.
  * FAT12/16/32 filesystem support with optional FAT long file name support1.
  * NFS Client. Client side support for a Network File System (NFS, version 3, UDP).
  * NXFFS. The tiny NuttX wear-leveling FLASH file system.
  * SMART. FLASH file system from Ken Pettit.
  * SPIFFS. FLASH file system, originally by Peter Anderson.
  * LittleFS. FLASH file system from ARM mbed..
  * ROMFS filesystem support (XIP capable).
  * CROMFS (Compressed ROMFS) filesystem support.
  * TMPFS RAM filesystem support.
  * BINFS pseudo-filesystem support.
  * HOSTFS filesystem support (simulation only).
  * Union filesystem - Supports combining and overlaying file systems.
  * UserFS - User application file system.
  * ``procfs/`` pseudo-filesystem support.
  * :doc:`/components/binfmt` with support for the following formats:

    - Separately linked ELF modules.
    - Separately linked :doc:`/components/nxflat` modules. NXFLAT is a binary format that can be XIP from a
      file system.
    - "Built-In" applications.

  * PATH variable support.
  * File transfers via TFTP and FTP (``get`` and ``put``), HTML (``wget``), and Zmodem (``sz``
    and ``rz``). Intel HEX file conversions.

    * FAT long file name support may be subject to certain Microsoft patent restrictions if enabled.
      See the top-level ``COPYING`` file for details.

* **Device Drivers**

  * Supports character and block drivers as well as specialized driver interfaces.
  * Full VFS integration. Asynchronous I/O (AIO)
  * Network, USB (host), USB (device), serial, I2C, I2S, NAND, CAN, ADC, DAC, PWM, Quadrature Encoder, I/O Expander, Wireless,
    generic timer, and watchdog timer driver architectures.
  * RAMDISK, pipes, FIFO, ``/dev/null``, ``/dev/zero``, ``/dev/random``, and loop drivers.
  * Generic driver for SPI-based or SDIO-based MMC/SD/SDH cards.
  * Graphics: framebuffer drivers, graphic- and segment-LCD drivers. VNC server.
  * Audio subsystem: CODECs, audio input and output drivers. Command line and graphic media player applications.
  * Cryptographic subsystem.
  * :doc:`/components/power` sub-system.
  * ModBus support provided by built-in `FreeModBus <https://www.embedded-experts.at/en/freemodbus/>`__ version 1.5.0.

* **C/C++ Libraries**

  * Standard C Library Fully integrated into the OS.
  * Includes floating point support via a Standard Math Library.
  * Add-on `uClibc++ <http://cxx.uclibc.org/>`__ module provides Standard C++ Library (LGPL).

* **Networking**

  * Multiple network interface support; multiple network link layer support.
  * IPv4, IPv6, TCP/IP, UDP, ICMP, ICMPv6, IGMPv2 and MLDv1/v2 (client) stacks.
  * IP Forwarding (routing) support.
  * User space stacks.
  * Stream and datagram sockets.
  * Address Families: IPv4/IPv6 (``AF_INET``/``AF_INET6``), Raw socket (``AF_PACKET``), raw IEEE
    802.15.4 (``AF_IEEE802154``), raw Bluetooth (``AF_BLUETOOTH``), and local, Unix domain socket support (``AF_LOCAL``).
  * Special ``INET`` protocol sockets: Raw ICMP and ICMPv6 protocol ping sockets (``IPPROTO_ICMP``/``IPPROTO_ICMP6``).
  * Custom user sockets.
  * IP Forwarding.
  * DNS name resolution / NetDB
  * IEEE 802.11 FullMac
  * Radio Network Drivers: IEEE 802.15.4 MAC, Generic Packet Radio, Bluetooth LE
  * 6LoWPAN for radio network drivers (IEEE 802.15.4 MAC and generic packet radios)
  * SLIP, TUN/PPP, Local loopback devices
  * A port cJSON
  * Small footprint.
  * BSD compatible socket layer.
  * Networking utilities (DHCP server and client, SMTP client, Telnet server and client, FTP server and
    client, TFTP client, HTTP server and client, PPPD, NTP client). Inheritable TELNET server sessions (as "controlling
    terminal"). VNC server.
  * ICMPv6 autonomous auto-configuration
  * NFS Client. Client side support for a Network File System (NFS, version 3, UDP).
  * A NuttX port of Jeff Poskanzer's `THTTPD <http://acme.com/software/thttpd>`__
    HTTP server integrated with the NuttX :doc:`/components/binfmt` to provide true, embedded CGI.
  * PHY Link Status Management.
  * UDP Network Discovery (Contributed by Richard Cochran).
  * XML RPC Server (Contributed by Richard Cochran).
  * Support for networking modules (e.g., ESP8266).

* **FLASH Support**

  * *MTD*\ -inspired interface for *M*\ emory *T*\ echnology *D*\ evices.
  * NAND support.
  * *FTL*. Simple *F*\ lash *T*\ ranslation *L*\ ayer support file systems on FLASH.
  * Wear-Leveling FLASH File Systems: NXFFS, SmartFS, SPIFFS.
  * Support for SPI-based FLASH and FRAM devices.

* **USB Host Support**

  * USB host architecture for USB host controller drivers and device-dependent USB class drivers.
  * USB host controller drivers available for the Atmel SAMA5Dx, NXP LPC17xx, LPC31xx, and STmicro STM32
  * Device-dependent USB class drivers available for USB mass storage, CDC/ACM serial, HID keyboard, and HID mouse.
  * Seam-less support for USB hubs.

* **USB Device Support**

  * *Gadget*-like architecture for USB device controller drivers and device-dependent USB class drivers.
  * USB device controller drivers available for the most MCU architectures includeing PIC32,
    Atmel AVR, SAM3, SAM4, SAMv7, and SAMA5Dx, NXP/Freescale LPC17xx, LPC214x, LPC313x, LPC43xx, and
    Kinetis, Silicon Laboraties EFM32, STMicro STM32 F1, F2, F3, F4, and F7, TI DM320, and others.
  * Device-dependent USB class drivers available for USB serial (CDC/ACM and a PL2303 emulation),
    for USB mass storage, for USB networking (RNDIS and CDC/ECM), DFU, and for a dynamically
    configurable, composite USB devices.
  * Built-in :doc:`/guides/usbtrace` and USB host trace functionality for non-invasive USB debug.

* **Graphics Support**

  * Framebuffer drivers.
  * Graphic LCD drivers for both parallel and SPI LCDs and OLEDs.
  * Segment LCD drivers.
  * VNC Server.
  * ``mmap``-able, framebuffer character driver.
  * NX: A graphics library, tiny windowing system and tiny font support that works with either
    framebuffer or LCD drivers. Documented in the :doc:`/components/nxgraphics/index` manual.
  * Font management sub-system.
  * :doc:`/components/nxwidgets`: NXWidgets is library of graphic objects, or "widgets," (labels,
    buttons, text boxes, images, sliders, progress bars, etc.). NXWidgets is written in C++ and
    integrates seamlessly with the NuttX NX graphics and font management subsystems.
  * NxWM is the tiny NuttX window manager based on NX and NxWidgets.

* **Input Devices**

  * Touchscreen, USB keyboard, GPIO-based buttons and keypads.

* **Analog Devices**

  * Support for Analog-to-Digital conversion (ADC), Digital-to-Analog conversion (DAC), multiplexers, and
    amplifiers.

* **Motor Control**

  * Pulse width modulation (PWM) / Pulse count modulation.

* **NuttX Add-Ons**.
  The following packages are available to extend the basic NuttX feature set:

  * **NuttShell (NSH)**
    A small, scalable, bash-like shell for NuttX with rich feature set and small footprint. See the :doc:`/applications/nsh/index`.
  * **BAS 2.4**
    Seamless integration of Michael Haardt's BAS 2.4: "Bas is an interpreter for the classic dialect of the programming language BASIC. It is pretty compatible to typical BASIC interpreters of the 1980s, unlike some other UNIX BASIC interpreters, that implement a different syntax, breaking compatibility to existing programs. Bas offers many ANSI BASIC statements for structured programming, such as procedures, local variables and various loop types. Further there are matrix operations, automatic LIST indentation and many statements and functions found in specific classic dialects. Line numbers are not required."

Look at all those files and features... How can it be a tiny OS?
================================================================

The NuttX feature list (above) is fairly long and if you
look at the NuttX source tree, you will see that there are hundreds of source files comprising NuttX. How can NuttX be a tiny
OS with all of that?

  * **Lots of Features -- More can be smaller!**

    The philosophy behind that NuttX is that lots of features are great... *BUT* also that if you
    don't use those features, then you should not have to pay a penalty for the unused features.
    And, with NuttX, you don't! If you don't use a feature, it will not be included in the final
    executable binary. You only have to pay the penalty of increased footprint for the features that
    you actually use.

    Using a variety of technologies, NuttX can scale from the very tiny to the moderate-size system.
    I have executed NuttX with some simple applications in as little as 32K *total* memory (code and
    data). On the other hand, typical, richly featured NuttX builds require more like 64K (and
    if all of the features are used, this can push 100K).

  * **Many, many files -- More really is smaller!**

   One may be intimidated by the size NuttX source tree. There are hundreds of source files! How can
   that be a tiny OS? Actually, the large number of files is one of the tricks to keep NuttX small
   and as scalable as possible. Most files contain only a single function. Sometimes just one tiny
   function with only a few lines of code. Why?

     - **Static Libraries**.
       Because in the NuttX build processed, objects are compiled and saved into *static libraries*
       (*archives*). Then, when the file executable is linked, only the object files that are
       needed are extracted from the archive and added to the final executable. By having many,
       many tiny source files, you can assure that no code that you do not execute is ever
       included in the link. And by having many, tiny source files you have better granularity --
       if you don't use that tiny function of even just a few lines of code, it will not be
       included in the binary.

* **Other Tricks**

  As mentioned above, the use of many, tiny source files and linking from static libraries
  keeps the size of NuttX down. Other tricks used in NuttX include:

  - **Configuration Files**.

    Before you build NuttX, you must provide a configuration file that specifies what features you plan to use and
    which features you do not. This configuration file contains a long list of settings that control what is
    built into NuttX and what is not. There are hundreds of such settings (see the
    `Configuration Variable Documentation <https://cwiki.apache.org/confluence/display/NUTTX/Configuration+Variables?src=contextnavpagetreemode>`__
    for a partial list that excludes platform specific settings). These many, many configuration options allow
    NuttX to be highly tuned to meet size requirements. The downside to all of these configuration options is that
    it greatly complicates the maintenance of NuttX -- but that is my problem, not yours. -

  - **Weak Symbols**
    The GNU toolchain supports *weak* symbols and these also help to keep the size of NuttX down.
    Weak symbols prevent object files from being drawn into the link even if they are accessed from source code.
    Careful use of weak symbols is another trick for keep unused code out of the final binary.

