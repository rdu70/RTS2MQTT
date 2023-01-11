# RTS2MQTT

## Goal
This project provide code to emulate an RTS remote (used by Somfy cover).  It can send RTS frame (emulate remotes) and also receive those frames and then simulate the cover moves to get a pretty accurate feedback.  This allow to keep in sync the cover status by using at the same time real hardware remotes and simulated one without messing the rolling codes.

MQTT interface allow to easily interface with domotic solution like Home Assistant.

## Functionalities
You can record your real hardware remote (40 memory slots for remotes) and link them to the virtual covers (10 slots).  Finaly define additionnal 'soft' remote and link them to the covers as well (a virtual cover can be linked to 10 remote slots).  This allow to control the cover using either the hardware and the software remotes (throught MQTT and Home Assistant for instance) without messing with the rolling codes and get a status of the cover.  As the RTS protocol does not feedback the cover position, this is emulated by software and should kept in sync provided that the reception area is good and that the cover speed has be correctly define (time for full open to full close).

Data are stored on ESP flash to keep definition and rolling codes when power lost.

## Hardware
RX/TX are done usign OOK modulation at 433.42 MHz.  This is slightly different that usual 433 transmitter.  For best coverage and good signal, it's important to use the correct frequency, so a CC1101 module with external antenna is used.
Tested on a LOLIN D1 Mini v3.1 (https://www.wemos.cc/en/latest/d1/d1_mini_3.1.0.html).

![This is an image](doc/CC1101.jpeg)

| WeMOS D1 mini v3.1 PIN (ESP8266 PIN) | CC1101 PIN (8 PIN Header) | (10 PIN Header) | LED |
| --- | --- | --- | --- |
| GND | GND (1) | (9 + 10) ||
| 3V3 | VCC (2) | (1 + 2) ||
| D1 (GPIO5) | GDO0 IN (3) | (8) ||
| D8  (GPIO15) | CSN (4) | (7) ||
| D5 (GPIO14/SCK) | SCK (5) | (4) ||
| D7 (GPIO13/MOSI) | MOSI (6) | (3) ||
| D6 (GPIO12/MISO) | MISO (7) | (5) ||
| D2 (GPIO4) | GDO2 OUT (8) | (6) ||
| D3 (GPIO0) | --- | --- | 330R + Green LED to 3V3 |
| D4 (GPIO2) | --- | --- | 330R + Red LED to 3V3 |


## Configuration
Currentlty WiFi and MQTT configuration is done at compile time only (see cred.h file).
Once flashed, connect to the serial console and the following commands are available:
- show (or sh)
- copy (or cp)
- roll (or rl)
- link (or ln)
- unlink (or ul)
- rcedit (or rced)
- devedit (or ded)
- send
- save
- load

Remote Memory configuration
| memory slot | meaning |
| --- | --- |
| rx | last valid RTS frame received |
| tx | working slot and RTS frame to be transmitted by next 'send' command |
| 0..39 | Remote memory slots |

Remote Parameters
| Parameter | meaning |
| --- | --- |
| rid | remote ID |
| cid | channel ID |
| rc | Rolling code |
| ac | Action code |
| enc | Encryption key |
| proto | protocol (56 or 80 bits)


Closer/Shutter Memory configuration
| memory slot | meaning |
| --- | --- |
| 0..9 | Closer/Shutter memory slots |

Closer/Shutter parameter
| Parameter | meaning |
| --- | --- |
| cmd remote ID | virtual command to be use to control this closer |
| Aux remote ID | other remote to listen to and update closer status |
| opt | open time (seconds) |
| pos | current position (0..100) |


## MQTT
/ home-assistant / cover / cover number (0..9) / { set - position - state - availability }

## Sample commands
```
> cp rx tx
> rl
> send
> sh rx
> sh rc 1
Remote idx     : 01
Remote ID      : 0113
Channel ID     : 21
Encryption key : A7
Protocol       : 56 bits
Timing         : 2 - 7 preambles
Rolling code   : 149D
Action code    : 04
Action         : Dn
> sh dev 1
Shutter idx    : 01
Cmd remote idx : 31
Aux remote idx : 01 11 15 21 25 255 255 255 255 255
Relay Aux->Cmd : Yes
Open/Close time: 18
Action         : 00
Position       : 100
Command        : 000
> ln 1 1
> save
> sh uptime
Uptime         :    0d 02h:46m:16s
```

# Credit / Links
This project has be done by reverse engineering some real frame generated with real hardware remote and also by some informations found on the Internet about the RTS protocol.

More the protocol itself is already provided at https://pushstack.wordpress.com/somfy-rts-protocol/ and at https://github.com/henrythasler/sdr/tree/master/somfy so I'll not explain more here.  It seems that there is 2 versions, one based on 56 bits frame wich I have device to test and wich is working very well here.  A second version with 80 bits frame seems to exist but I don't have any device to test, so the 80 bit code is based on information found on the web but not tested on real hardware.  If you have those feel free to contact me.
