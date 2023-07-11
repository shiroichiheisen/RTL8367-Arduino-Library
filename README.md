#The library its on development, so no readme.

To use this library, you will need to connect the sda and sck pins to the arduino, !important! you must mantain the rtl8367 eeprom for him to start, and after that you can use the library to configure the switch. !important! the rtl8367 series is 3.3v! so you will need to use a logic level converter to use it with the arduino if you are using a 5v arduino.

# Important:

You need to wait 1 second after the switch is powered on to use the library, because the switch needs time to start and read the eeprom, if your creating a board and dont have the eeprom data, you can use the .bin file on the eeprom folder to program the eeprom, the file is a dump of the eeprom of a rtl8367s switch, so you can use it to program your eeprom.

I only have the rtl8367s eeprom file using the 24c32, i dont have other rtl8367 eeprom files, so if you have other rtl8367 eeprom files, please send them to this repository.

This library is based on the realteck original library, theres a programming guide from realtech on this library.

# Functions Added:

- [x] Read the port status - getPortStatus - Workin
- [x] Probe the ic to see if its connected - probeIc - Working
- [x] Init Vlan - rtk_svlan_init - Working
- [x] Set Vlan - rtk_svlan_set
- [x] Get Vlan - rtk_svlan_get

# tested on:

- [x] RTL8367S

# used the library on:

- [x] Esp32
- [x] Esp32-s2