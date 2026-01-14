==================
I3C Device Drivers
==================

The I3C (Improved Inter-Integrated Circuit) driver is a comprehensive software framework designed to
support the advanced features of the I3C protocol. It comprises a system framework and underlying IP
drivers, both of which are critical components for enabling seamless communication between I3C devices.
The system framework, provided by vela os, offers user-level device nodes and kernel-level driver
interfaces. Meanwhile, the underlying IP drivers, developed specifically for the I3C driver framework,
handle the implementation of IP functionalities and the adaptation of framework-level interfaces.

- Key Components of the I3C Driver Framework:

 #. **Dynamic Addressing and Access**: One of the primary advancements of I3C over I2C is its support
    for dynamic addressing. This allows for more flexible device management and addressing schemes.
 #. **CCC (Common Command Codes) Support**: The I3C driver framework enables the transmission of CCC commands,
    which extend the functionality and capabilities of I3C devices. Dynamic address allocation and
    release commands are mandatory, while other CCC commands are optional and can be implemented as
    needed.
 #. **Data Transmission**: The framework supports both I3C and I2C device data transmission, ensuring
    compatibility with existing I2C devices while leveraging the enhanced features of I3C.
 #. **Interface Callbacks**: The I3C driver framework provides a set of callback interfaces that enable
    IP drivers to interact with the framework layer. These include interfaces for dynamic address
    configuration and release, CCC command transmission, I3C device data transmission, and I2C device
    data transmission.

- Data Structures and Interfaces:

 #. **I3C Master Controller (struct i3c_master_controller)**: Represents an I3C controller and manages
    basic software information for the I3C bus.
 #. **I3C Bus Operation Callbacks (struct i3c_master_controller_ops)**: Enables application-level access
    to I3C for data transmission and other operations. Key functions include bus_init for bus initialization,
    attach_i3c_dev and attach_i2c_dev for device attachment, ccc_xfer for CCC command transmission, priv_xfers
    for private data transmission, and i2c_xfers for I2C device data transmission.
 #. **Data Structures for I2C and I3C Transmissions**: Separate structures (struct i2c_msg_s and struct i3c_priv_xfer)
    are defined for encapsulating data transmitted to I2C and I3C devices, respectively.
 #. **CCC Command Transmission (struct i3c_ccc_cmd)**: A dedicated structure for encapsulating CCC commands
    transmitted to I3C devices.

- Application Usage of I3C

Applications interact with I3C devices through device nodes (/dev/i3cX for I3C and /dev/i2cX for I2C devices,
where X represents the specific bus number). Standard file operations such as open, close, read, write, and ioctl are supported.

- IOCTL Commands:

  #. For I2C devices, IOCTL commands like I2C_TRANSFER and I2C_RESET are available, allowing applications to transmit
     data and reset I2C devices.
  #. For I3C devices, a range of IOCTL commands are provided, including I3CIOC_PRIV_XFERS for data transmission,
     I3CIOC_EN_IBI and I3CIOC_DIS_IBI for enabling and disabling IBI (In-Band Interrupt) commands, I3CIOC_REQ_IBI and
     I3CIOC_FREE_IBI for requesting and releasing IBI commands, and I3CIOC_GET_DEVINFO for retrieving device information.

- Data Transmission Format:

When transmitting data to I3C slave devices, applications must encapsulate their data in a struct i3c_transfer_s
format. This structure includes fields such as target_addr for the slave device address, nxfers for the number of
data frames to be transmitted, xfers for the data frame format, and additional fields for IBI operation requests and
device information retrieval.

- Provisional ID Implementation:

Due to the use of dynamic addressing in I3C, devices may be identified using Provisional IDs (PIDs). These PIDs
are encoded in the reg[3] array of the boardinfo structure, with specific rules for encoding manufacturer ID, part ID,
instance ID, and extra information. This encoding scheme ensures unique identifiability of I3C devices, even when static
addressing is not
