# SX126x
This is an Arduino library for LoRa Communication using the radio transceiver chip SX126x (https://www.semtech.com).
I developed it because I want to extend my bunch of homeautomation devices (https://github.com/tinytronix/homeautomation). 
to be able to speak LoRa.

FSK Mode is currently not supported in this lib.

## Installing
Download this repo as zip. Then in the Arduino IDE go to Sketch->Add library->add .zip library.
Or please refer to the offical Arduino howto.

## First Steps
The first place to start may be the examples folder. There is a simple rx tx example
which sends data (TX) and echos them back (rx). 

## Hardware compatibility
The examle works with the attached schematic. This library was tested with LoRa module DRF1268T (http://www.dorji.com), as well usable for DRF1262T. Please see the attached schematic for wiring. 

