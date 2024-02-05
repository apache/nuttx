=================
``ymodem`` YMODEM
=================

This is `ymodem protocol <http://pauillac.inria.fr/~doligez/zmodem/ymodem.txt>`_.
According to it, the sb rb application is realized, which is used to send files and receive files respectively

Usage
-----

Common Usage
~~~~~~~~~~~~

In the ubuntu system, lszrz needs to be installed, can use ``sudo apt install lszrz``.
Use minicom to communicate with the board.

Advanced Usage
~~~~~~~~~~~~~~

In order to achieve a faster transmission speed,
I added a specific HEADER ``STC`` to the YMODEM protocol to represent the custom length.
Using the ``sb`` and ``rb`` commands on the board, you can use the ``-k`` option to set the length
of the custom packet, and the unit is KB. Therefore, you need to use ``sbrb.py`` for file transfer,
and you need ``sbrb.py`` -k to set the same length as the board. According to my test,
when using -k 32, it can reach 93% of the baud rate,
and is fully compatible with the original ymodem protocol.
First, you need to add a soft link to sbrb.py, for example ``sudo ln -s /home/<name>/.../<nuttxwork>/apps/system/ymodem/sbrb.py /usr/bin``
and then sbrb.py can be configured into minicom.``<Ctrl + a> z o`` then chose ``File transfer protocols`` and create two option cmd is 'sbrb.py -k 32'. like this

=========== ============= ==== === ======= ======= =====
Name        Program       Name U/D FullScr IO-Red. Multi
=========== ============= ==== === ======= ======= =====
ymodem-k    sbrb.py -k 32 Y    U   N       Y       Y 
ymodem-k    sbrb.py -k 32 N    D   N       Y       Y 
=========== ============= ==== === ======= ======= =====

usb ``sb -k 32`` or ``rb -k 32`` for file transfer on board.

Sendfile to pc
--------------

use sb command like this ``nsh> sb /tmp/test.c ...``, this command support send multiple files together
then use ``<Ctrl + a> , r`` chose ``ymodem`` to receive board file.

Sendfile to board
-----------------

use rb cmd like this ``nsh> rb``, this command support receive multiple files together
then use ``<Ctrl + a> , s`` chose ``ymodem``, then chose what file need to send.

help
~~~~

can use ``sb -h`` or ``rb -h`` get help.

Debug
-----

Because the serial port is used for communication, the log is printed to the debug file
you can use ``CONFIG_SYSTEM_YMODEM_DEBUGFILE_PATH`` set debug file path.
