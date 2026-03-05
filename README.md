By: Jack Varagas

Initial firmware version. Not sure about versioning but definitley needs more work.

TODO:
 - Need to double check ADC and DAC conversions as well as account for the voltage divider on ADC inputs
 - Want to move away from Emulated EEPROM and just write to flash if possible. (Can index entries and erase sector once full rather than erase every time we need to write)
