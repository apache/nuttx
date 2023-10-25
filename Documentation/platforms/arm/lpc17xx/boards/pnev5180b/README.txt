README
^^^^^^

README for NuttX port to NXP's PNEV5180B, which is a development board
containing a NXP-LPC1769 MCU and a PN5180 NFC Frontend.

Contents
^^^^^^^^

  NXP's PNEV5180B Board
  Configurations

NXP's PNEV5180B Board
^^^^^^^^^^^^^^^^^^^^^

  Pin Description                      On Board       Connector
  -------------------------------- ---------------- -------------
  P0.2/TXD0/AD0.7                    TX               J201
  P0.3/RXD0/AD0.6                    RX
  P0.22/RTS1/TD1                     LD200            ORANGE LED
  P0.15/TXD1/SCK0/SCK                PN5180-SCK
  P0.16/RXD1/SSEL0/SSEL              PN5180-SSEL      PN5180
  P0.17/CTS1/MISO0/MISO              PN5180-MISO
  P0.18/DCD1/M0SI0/MOSI              PN5180-MOSI
  P0.19/DSR1/SDA1                    EEPROM           (Not Assembled)
  P0.20/DTR1/SCL1                    EEPROM
  P0.21/RI1/RD1                      PN5180-AUX2      PN5180
  P0.29/USB_D+                       USB-D+           USB
  P0.30/USB_D-                       USB-D-
  P2.0/PWM1.1/TXD1                   LD201            RED LED
  P2.5/PWM1.6/DTR1/TRACEDATA0        PN5180-nPN_RST
  P2.9/USB_CONNECT/RXD2              USB_CONNECT      USB
  P2.11/nEINT1/I2STX_CLK             PN5180-BUSY      PN5180
  P2.12/nEINT2/I2STX_WS              PN5180-IRQ
  P3.25/MAT0.0/PWM1.2                LD203            GREEN LED
  P3.26/STCLK/MAT0.1/PWM1.3          LD202            BLUE LED

NXP's PNEV5180B Board
^^^^^^^^^^^^^^^^^^^^^

UART
----

  Board             LPC1769
  Signal            Pin
  ----------------- -----------------
  TX                P0.2  TXD0
  RX                P0.3  RXD0

USB
---

  Board             LPC1769
  Signal            Pin
  ----------------- -----------------
  USB_CONNECT       P2.9  USB_CONNECT
  USB_DM            P0.29 USB_D-
  USB_DP            P0.30 USB_D+

PN5180
------

  Board             LPC1769
  Signal            Pin
  ----------------- -----------------
  PN5180_SCK        P0.15 SCK
  PN5180_SSEL       P0.16 SSEL
  PN5180_MISO       P0.17 MISO
  PN5180_MOSI       P0.18 MOSI
  PN5180_AUX2       P0.21 GPIO I
  PN5180_nPN_RST    P2.5  GPIO O
  PN5180_BUSY       P2.11 GPIO I
  PN5180_IRQ        P2.12 GPIO I

Configurations
^^^^^^^^^^^^^^

Each PNEV5180B configuration is maintained in a sub-directory and can be
selected as follow:

    cd tools
    ./configure.sh pnev5180b:<subdir>
    cd -

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at apps/system/nsh.  The
    Configuration enables the serial NSH interface.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

  usbnsh:
    Configures the NuttShell (nsh) located at apps/system/nsh.  The
    Configuration enables the CDC/ACM based NSH interface on /dev/ttyACM0.

  nsh-cdcecm:
    Configures the NuttShell (nsh) to provide a terminal on UART0.  The
    Configuration also provides network interface eth0 via CDC/ECM.

  usbnsh-cdcecm:
    This configuration includes a USB Composite Device with both CDC/ACM and
    CDC/ECM.  NuttShell provides a terminal via CDC/ACM.  A network interface
    (eth0) is also provided via CDC/ECM.

  knsh:
    This is identical to the nsh configuratio above except that NuttX
    is built as a kernel-mode, monolithic module and the user applications
    are built separately.
