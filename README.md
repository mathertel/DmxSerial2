[![Arduino Library Checks](https://github.com/mathertel/DmxSerial2/actions/workflows/arduino-checks.yml/badge.svg)](https://github.com/mathertel/DmxSerial2/actions/workflows/arduino-checks.yml)[![codespell](https://github.com/mathertel/DmxSerial2/actions/workflows/codespell.yml/badge.svg)](https://github.com/mathertel/DmxSerial2/actions/workflows/codespell.yml)

DmxSerial2
==========

An Arduino library for sending and receiving DMX RDM packets.

You can find more detail on this library at http://www.mathertel.de/Arduino/DMXSerial2.aspx.

A suitable hardware is the Arduino platform plus a shield for the DMX/RDM physical protocol implementation.
You can find such a shield at: http://www.mathertel.de/Arduino/DMXShield.aspx.

N.B. This library writes to the EEPROM to store your DMX start address and other parameters during a repower.
