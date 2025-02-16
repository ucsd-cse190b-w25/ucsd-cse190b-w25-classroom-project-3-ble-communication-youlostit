# CSE-190 WI25 Project 3

This project adds low-energy radio communication (using BLE) to your project. It has 3 sections

1. Trying out the starter code
2. Adding Disconnect and Non-discoverable functionality
3. Integrating your application

While the 1st and the 3rd step might be straight forward, this document acts as a hints to step 2. 

Before you jump to the hints, I highly recommend Section 1 and Section 2 of the [The BlueNRG-MS Bluetooth® LE stack application command interface (ACI)](https://cseweb.ucsd.edu/classes/wi20/cse190-d/docs/bluenrgms-aci_datasheet.pdf). The first two sections explain the entire architecture of the how BLE communication works with respect to our board and how commands and event packets are formed. Its short (relatively), simple, and easy to understand and it would make things a lot more simpler to understand. 

Now that you are done reading about the architecture (and hopefully understand how HCI packets are constructed), your task is to fill in `DISCONNECT` and `ACI_GAP_SET_NON_DISCOVERABLE` commands as well as the `ACI_GAP_SET_NON_DISCOVERABLE_COMPLETE` event in `ble_commands.h`

## Command Packet Structure


| Bytes     | Description                                                                                     | Value                                   |
|-----------|-------------------------------------------------------------------------------------------------|-----------------------------------------|
| 0         | HCI Packet Type                                                                                 | Varies (0x01 => Indicates that this is a command) |
| 1-2       | OpCode - The code for the command as per the BlueNRG-MS ACI Manual                          | Varies                                  |
| 3         | Parameter Length - The number of command parameters to follow this byte                             | Varies                                  |
| 4 onwards | Parameters - The parameters for that command described as per the BlueNRG-MS ACI Manual | Varies                                  |


## Event Packet Structure

| Bytes     | Description                                                                                                                                                | Value                                                                                                                      |
|-----------|------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| 0         | HCI Packet Type                                                                                                                                            | Varies (0x04 => Indicates that this is a event)                                                                            |
| 1         | Event Code - The code for the <br>event as per the BlueNRG-MS ACI Manual                                                                                   | Varies, for example: Command Complete<br>is 0x0E                                                                           |
| 2         | Parameter Length - The number of<br>parameters to follow this byte                                                                                         | Varies - should be minimum 3, since <br>the next 3 bytes count as well                                                     |
| 3         | Number of HCI Command Packets Allowed <br>before the interrupt pin is set again <br>(When BLE_INT_Pin is set to 1, it implies that an event was generated) | Varies - usually 1                                                                                                         |
| 4-5       | Opcode of Command sent (Little Endian format) in the case that this was a `command_complete` event                                                                                                             | Varies                                                                                                                     |
| 6         | Status of the command                                                                                                                                      | 0x00 => Success, anything else implies failure,<br>the reason for the failure is found in <br>Bluetooth Core specification |
| 7 onwards | Parameters to follow                                                                                                                                       | Varies                                                                                                                     |


## Hints
- ACI Commands are found in the BlueNRG-MS ACI Manual
- HCI Events will be found in the Bluetooth Core specification
- The Bluetooth Core specification version 6.0 is really helpful for this part
- Disconnect Command is a Link Control Command
- Every command will also have return parameters which will be passed in the respective `command_complete` event
