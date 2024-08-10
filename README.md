ESP32 BLE client for RaceBox Micro, RaceBox Mini and RaceBox Mini S. 
https://www.racebox.pro

The code is not yet finished, please contribute to further development.

Please do not fork the repository solely for the pirpose of bookmarking. That is a bad practise. Only create forks if you implement own work and ideally plan to issue pull requests, so that your improvements can be implemented to this original repository.

Currently, Serial output of ESP32 at 115200 BAUD and connection a 1.5" SPI full color OLED to display live data is supported.

You may #define TARGET_DEVICE_ADDRESS "aa:bb:cc:dd:ee:ff in the code to only connect to a distinguished BLE address - if commented out, the code will connect as a client to any RaceBox.

Checksum check and message identification are implemented, data parsing is implemented for RaceBox Data Messages (=live data like GPS speed, coordinates, accelerations, heading, date/time, voltage etc.)
parsing of other message types is prepared but not yet implemented.

The code is commented in detail to allow better understanding.
