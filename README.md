
Synopsis
=================
This little firmware is written for the ESP8266 [1].
The goal is to transmit MIDI data [2] e.g. from an instrument (Device) to e.g. to a synthesizer (Host).
For this, two variants are supported:
 - Host: Opens a WIFI network and TCP-server socket
 - Device: Connects to the WIFI network and establishes a TCP/IP connection to the host's server socket.

This firmware is only tested on a ESP-01, but may be compatible to other boards.

Preparation
=================
See [3]

Build
=================

Host:
```
make BOOT=none APP=0 SPI_SPEED=26.7 SPI_MODE=QIO SPI_SIZE_MAP=2 USER_DEFINES='-DNETWORK_MODE=NETWORK_MODE_OWN -DMIDI_VARIANT=MIDI_VARIANT_HOST -DMIDI2WIFI_SSID=\"add_desired_SSID_here\" -DMIDI2WIFI_PASS=\"add_a_pass_here\"'
```

Device:
```
make BOOT=none APP=0 SPI_SPEED=26.7 SPI_MODE=QIO SPI_SIZE_MAP=2 USER_DEFINES='-DNETWORK_MODE=NETWORK_MODE_OWN -DMIDI_VARIANT=MIDI_VARIANT_DEVICE -DMIDI2WIFI_SSID=\"add_desired_SSID_here\" -DMIDI2WIFI_PASS=\"add_a_pass_here\"'
```

Debug-Builds
-------------
For debugging purposes, it is possible to create a host and device variant, which are using an existing WIFI infrastructure:

Host:
```
make BOOT=none APP=0 SPI_SPEED=26.7 SPI_MODE=QIO SPI_SIZE_MAP=2 USER_DEFINES='-DNETWORK_MODE=NETWORK_MODE_INFRASTRUCTURE -DMIDI_VARIANT=MIDI_VARIANT_HOST -DNETWORK_INFRASTRUCTURE_PASS=\"add_pass_here\" -DNETWORK_INFRASTRUCTURE_SSID=\"existing_SSID\"'
```

Device:
```
make BOOT=none APP=0 SPI_SPEED=26.7 SPI_MODE=QIO SPI_SIZE_MAP=2 USER_DEFINES='-DNETWORK_MODE=NETWORK_MODE_INFRASTRUCTURE -DMIDI_VARIANT=MIDI_VARIANT_DEVICE -DNETWORK_INFRASTRUCTURE_PASS=\"add_pass_here\" -DNETWORK_INFRASTRUCTURE_SSID=\"existing_SSID\"'
```


Flashing
=================
esptool is the way to go:
```
esptool.py --port <PORT> write_flash --flash_freq 26m --flash_mode qio --flash_size 8m 0x00000 output/eagle.flash.bin 0x20000 output/eagle.irom0text.bin
```

References
=================

[1] https://en.wikipedia.org/wiki/ESP8266
[2] https://www.midi.org/
[3] https://github.com/espressif/ESP8266_RTOS_SDK

License
================

Copyright (c) 2018 Christian Haettich [feddischson@gmail.com]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

