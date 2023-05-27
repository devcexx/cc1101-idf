# Sanity Check

This example just resets the CC1101 and prints all the internal CC1101
registers to the ESP-IDF console, so you can check if your device is
properly working and it is correctly connected to the MCU. If
something goes wrong during this check, like an abort(), or all the
registers set to 00 or ff, probably you should check your connections.
