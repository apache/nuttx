===============
``iperf`` iperf
===============

Overview
--------

This is a NuttX port of the ESP-IDF iperf example. [1]

It doesn't support all features in standard iperf.
It's supposed to be compatible with iperf version 2.x. [2]

[1] https://github.com/espressif/esp-idf/tree/master/examples/wifi/iperf
[2] https://sourceforge.net/projects/iperf2/

Configuring NuttX to use your Wireless Router (aka Access Point)
----------------------------------------------------------------

Since you are already in the root of NuttX/ repository, execute
make menuconfig to define your Wireless Router and your password::

    $ make menuconfig

    Browser the menus this way:

    Application Configuration  --->
        Network Utilities  --->
            Networking Configuration  --->
                WAPI Configuration  --->
                    (myApSSID) SSID
                    (mySSIDpassphrase) Passprhase

Replace the SSID from myApSSID with your wireless router name and
the Passprhase with your WiFi password.

Exit and save your configuration.

iperf Test Example
------------------

To set up, do ``make menuconfig`` and select the Apps > netutils > iperf example. By default, NuttX will the be the client
which sends data; and the host computer (Linux, macOS, or Windows) will be the server.

Set up networking so the NuttX computer can ping the host, and the host can ping NuttX. Now you are ready to run the
test.

If you are using a wireless network card, you must first connect to the router:

On host::

    $ iperf -s -p 5471 -i 1 -w 416K
    ------------------------------------------------------------
    Server listening on TCP port 5471
    TCP window size:  416 KByte
    ------------------------------------------------------------

On NuttX::

    nsh> iperf -c 192.168.1.181 -p 5471 -i 1 -t 10
    mode=tcp-client sip=192.168.1.198:5001, dip=192.168.1.181:5471, interval=1, time=10

            Interval Bandwidth

    0-   1 sec,  0.39 Mbits/sec
    1-   2 sec,  0.26 Mbits/sec
    2-   3 sec,  0.39 Mbits/sec
    3-   4 sec,  0.26 Mbits/sec
    4-   5 sec,  0.26 Mbits/sec
    5-   6 sec,  0.26 Mbits/sec
    6-   7 sec,  0.26 Mbits/sec
    7-   8 sec,  0.26 Mbits/sec
    8-   9 sec,  0.26 Mbits/sec
    9-  10 sec,  0.26 Mbits/sec
    0-  10 sec,  0.28 Mbits/sec

Now on the host you should see something like::

    $ iperf -s -p 5471 -i 1 -w 416K
    ------------------------------------------------------------
    Server listening on TCP port 5471
    TCP window size:  416 KByte
    ------------------------------------------------------------
    [  5] local 192.168.1.181 port 5471 connected with 192.168.1.198 port 4210
    [  5]  0.0- 1.0 sec  60.8 KBytes   498 Kbits/sec
    [  5]  1.0- 2.0 sec  34.9 KBytes   286 Kbits/sec
    [  5]  2.0- 3.0 sec  33.7 KBytes   276 Kbits/sec
    [  5]  3.0- 4.0 sec  33.4 KBytes   274 Kbits/sec
    [  5]  4.0- 5.0 sec  32.0 KBytes   262 Kbits/sec
    [  5]  5.0- 6.0 sec  32.0 KBytes   262 Kbits/sec
    [  5]  6.0- 7.0 sec  33.4 KBytes   274 Kbits/sec
    [  5]  7.0- 8.0 sec  32.0 KBytes   262 Kbits/sec
    [  5]  8.0- 9.0 sec  32.0 KBytes   262 Kbits/sec
    [  5]  9.0-10.0 sec  33.4 KBytes   274 Kbits/sec
    [  5]  0.0-10.3 sec   368 KBytes   292 Kbits/sec


This will tell you the link speed in Kbits/sec – kilobits per second. If you want kilobytes, divide by 8.

