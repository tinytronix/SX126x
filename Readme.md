# SX126x
This is an Arduino library for LoRa Communication using the radio transceiver chip SX126x (https://www.semtech.com).
I developed it because I want to extend my bunch of homeautomation devices (https://github.com/tinytronix/homeautomation). 
to be able to speak LoRa.

FSK Mode is currently not supported in this lib.

Here is my homeautomation gateway running with this Adruino driver library:
![lt](https://raw.githubusercontent.com/tinytronix/SX126x/master/pcb/LoRa2.JPG)
## Installing
Download this repo as zip. Then in the Arduino IDE go to Sketch->Add library->add .zip library.
Or please refer to the offical Arduino howto.

## First Steps
The first place to start may be the examples folder. There is a simple rx tx example
which sends data (TX) and echos them back (rx). 

## Hardware compatibility
This library was tested with LoRa module DRF1268T (http://www.dorji.com) and works with the attached schematic.  It is as well usable for DRF1262T. Please see the attached schematic for wiring. The library should be able to handle other
LoRa modules as long as they use the SX1262 or SX1268. In this case take a look at the DRF1268T datasheet and check if DIO1, DIO2 and DIO3 are wired differently.

## FAQ
Q: Why is the SW pin not supported by this library? <br>
A: Currently (in my hardware setup) the SW Pin is connected to 3,3V permanently, so RF is always on. In one of the next versions it might be a good idea to add a 5th parameter to the constructor (bool true/false) in order to let the SX126x DIO2 output control the RF switch. 5th Param TRUE: DIO2 switches RF, 5th Param FALSE: RF controlled externally. See SX126x datasheet, section "SetDio2AsRfSwitchCtrl" for details.<br><br>
Q: Does the lib support interrupts? <br>
A: No. Sending and receiving data will be handled in the arduino main loop.<br>
Q: Does the lib support LoRaWAN?<br>
A: No. It is a bare metal driver for SX126xchipset. 


