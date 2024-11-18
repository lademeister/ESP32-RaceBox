#A quick CYD port for the ESP32-RaceBox project from https://github.com/lademeister/ESP32-RaceBox 

## RaceBox BLE blutooth client for ESP32 for RaceBox Micro, RaceBox Mini and RaceBox Mini S. 
## for RaceBox devices from https://www.racebox.pro

# The code allows connecting an ESP32 microcontroller to a RaceBox device.
## It displays live data from RaceBox and allows data parsing of the payload sent from RaceBox using bluetooth low energy (BLE).
## It implements checksum and data validity checks and provides a framework to handle different types of messages send from the RaceBox device, as well as sending data to it.

### It also features displaying live Data from RaceBox on an OLED display.

### The code can connect to any RaceBox in range - by adding the bluetooth device address (Bluetooth MAC address) the code will only connect to a certain device, e.g. if more than one RaceBoxes are in range.
#### In any case (i.e. also in open scan mode) upon connection, the code verifies that a device indeed is a RaceBox, by checking the advertised name, as well as the advertised bluetooth services (UUIDs).

Not all possibilities of RaceBox are yet implemented, please contribute to further development.

Please do not fork the repository solely for the purpose of bookmarking. That is a bad practise. Only create forks if you implement own work and ideally plan to issue pull requests, so that your improvements can be implemented to this original repository.

Currently, Serial output of ESP32 at 115200 BAUD and connection a 1.5" SPI full color OLED to display live data is supported.

You may #define TARGET_DEVICE_ADDRESS "aa:bb:cc:dd:ee:ff in the code to only connect to a distinguished BLE address - if commented out, the code will connect as a client to any RaceBox.

Checksum check and message identification are implemented, data parsing is implemented for RaceBox Data Messages (=live data like GPS speed, coordinates, accelerations, heading, date/time, voltage etc.)
parsing of other message types is prepared but not yet implemented.

Automatic reconnect is implemented.
For voltage readings, an automatic device recognition between RaceBox Mini/Mini S and RaceBox micro is implemented,

The code is commented in detail to allow better understanding.
