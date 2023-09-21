# I will create a doc about how to use the library later.

To use this library, you will need to connect the sda and sck pins to the Arduino, !important! you must maintain the rtl8367 eeprom for him to start, and after that, you can use the library to configure the switch. !important! the rtl8367 series is 3.3v! so you will need to use a logic-level converter to use it with the Arduino if you are using a 5v Arduino.

You can't use the library if the eeprom is not present, on my board with the rtl8367s I need the eeprom to start the switch, I don't know if it's the same for the other switches, but I think it's the same for all of them. After some searching, I found that some switches with the rtl8367 are programmed internally to boot without the eeprom, but I don't know how to do that, so I will use the eeprom for now.

This library doesn't use the Arduino wire library, it's using its own software i2c, so you can use almost any pin for sda and sck, the pins just need to have digital input and output capabilities.

# Important:

You need to wait for 2 second after the switch is powered on to use the library because the switch needs time to start and read the eeprom. If you're creating a board and don't have the eeprom data, you can use the .bin file on the eeprom folder to program the eeprom, the file is a dump of the eeprom of an rtl8367 switch, so you can use it to program your eeprom.
This library is based on the Realtek original library, there's a programming guide from Realtek on this library.

This library is based on Realtek API 1.3.12, I don't have converted all of the functions from the API, but with some time I will convert them all, but now we have all the functions on the programming guide converted to this library.

# Tested on:

- [x] RTL8367S

# Used the library on:

- [x] Esp32
- [x] Esp32-s2

# Work with other rtl chips, just need to change the halCtrl to the correct one from the api on the programming guide