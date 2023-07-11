#The library its on development, so no readme.

To use this library, you will need to connect the sda and sck pins to the arduino, !important! you must mantain the rtl8367 eeprom for him to start, and after that you can use the library to configure the switch. !important! the rtl8367 series is 3.3v! so you will need to use a logic level converter to use it with the arduino if you are using a 5v arduino.

# Working:

- [x] Read the port status - getPortStatus
- [x] Probe the ic to see if its connected - probeIc

# tested on:

- [x] RTL8367S

# used the library on:

- [x] Esp32
- [x] Esp32-s2