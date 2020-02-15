# SX126x
This is an Arduino library for LoRa Communication using the radio transceiver chips [SX1268](https://www.semtech.com) and [SX1262](https://www.semtech.com).
I developed it because I want to extend my bunch of [homeautomation devices](https://github.com/tinytronix/homeautomation). 
to be able to speak LoRa. 

Most of the hardware driver software is taken from [RadioLib](https://github.com/jgromes/RadioLib) a universal wireless communication library for Arduino (great project!) and slightly adapted.

## Installing
Download this repo as zip. Then in the Arduino IDE go to Sketch->Add library->add .zip library.
Or please refer to the offical Arduino howto.

## First Steps
The first place to start may be the [examples](https://github.com/tinytronix/SX126x/tree/master/examples) folder. There is a simple rx tx example which sends data (TX) and echos them back (rx). 

## Hardware compatibility
This library was tested with LoRa module [DRF1268T](http://www.dorji.com) and works with the provided [schematic](https://github.com/tinytronix/homeautomation/blob/master/Hardware/LoraGateway/Schematic.pdf).  It is as well usable for DRF1262T. Please see the schematic for wiring. The library should be able to handle other
LoRa modules as long as they use the SX1262 or SX1268. In this case take a look at the DRF1268T datasheet and check if DIO1, DIO2 and DIO3 are wired to different IO-Pins. You can easily change the used IO-Pins when calling the [constructor](https://github.com/tinytronix/SX126x/blob/master/examples/LoRaRX.ino).

## FAQ
Q: Why is the SW pin not supported by this library? <br>
A: Currently (in my hardware setup) the SW Pin is connected to 3,3V permanently, so RF is always on. In one of the next versions it might be a good idea to add a 5th parameter to the constructor (bool true/false) in order to let the SX126x DIO2 output control the RF switch. 5th Param TRUE: DIO2 switches RF, 5th Param FALSE: RF controlled externally. See SX126x datasheet, section "SetDio2AsRfSwitchCtrl" for details.<br><br>
Q: Does the lib support interrupts? <br>
A: No. Sending and receiving data will be handled in the arduino main loop.<br>
<br>
Q: Does the lib support LoRaWAN?<br>
A: It is a bare metal driver library for SX126x chipset and implements clean LoRa data send and receive functons according to the OSI reference model. Therefore any LoRaWAN library may use this hardware driver library.<br> 
<br> 
Q: Is FSK Mode available.<br>
A: The SX126x chip implements FSK but it is not supported by this driver library.

## Hardware
Here is my homeautomation gateway running with this Adruino driver library and [my peer-to-peer LoRa communication protocol](https://github.com/tinytronix/LoRa):
![lt](https://raw.githubusercontent.com/tinytronix/SX126x/master/pcb/LoRa2.JPG)

