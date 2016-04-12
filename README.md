Single Channel LoRaWAN Gateway
==============================
This repository contains a proof-of-concept implementation of a single
channel LoRaWAN gateway.

It has been tested on the Wemos D1 Mini, using a Semtech SX1276
transceiver (HopeRF RFM95W).

The code is for testing and development purposes only, and is not meant 
for production usage. 

Source Code is based on code base of Single Channel gateway for RaspberryPI
which is developed by Thomas Telkamp.

Maintained by Maarten Westenberg (mw12554@hotmail.com)

Features
--------
- listen on configurable frequency and spreading factor
- SF7 to SF12
- status updates
- can forward to two servers

Not (yet) supported:
- PACKET_PUSH_ACK processing
- SF7BW250 modulation
- FSK modulation
- downstream messages (tx)

Dependencies
------------
- SPI needs to be enabled on the Raspberry Pi (use raspi-config)
- gBase64 library, pubsubclient-2.4

Connections
-----------
See http://things4u.github.io in the hardware section for building
and connection instructions

Configuration
-------------

Defaults:

- LoRa:   SF7 at 868.1 Mhz
- Server: 54.229.214.112, port 1700  (The Things Network: croft.thethings.girovito.nl)
  or directly croft.thethings.girovito.nl

Edit .h file (ESP-sc-gway.h) to change configuration (look for: "Configure these values!").

Please set location, email and description.

License
-------
The source files in this repository are made available under the Eclipse
Public License v1.0, except for the base64 implementation, that has been
copied from the Semtech Packet Forwader.
