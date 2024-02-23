# Reference library for Waveshare serial bus servos

This library is designed to be used with Waveshare's [Servo Driver with ESP32](https://www.waveshare.com/wiki/Servo_Driver_with_ESP32) board, 
which converts asynchronous UART to a single DATA line.

The message protocol is as follows:
```
Host -> Servo
cmd   [0xff 0xff ID MSG_LEN 0x01 CMD      CHECKSUM          ]
read  [0xff 0xff ID MSG_LEN 0x02 MEMADDR  LEN      CHECKSUM ]
write [0xff 0xff ID MSG_LEN 0x03 MEMADDR  <DATA>   CHECKSUM ]
 
Servo -> Host
ack   [0xff 0xff ID MSG_LEN ERR CHECKSUM          ]
resp  [0xff 0xff ID MSG_LEN ERR <DATA>   CHECKSUM ]
 
Checksum is calculated as: ~(ID + MSG_LEN + <DATA>)
```

Initially, the servos use a UART baudrate of 1M and ID of 1.

The library has been tested with the following circuit:
<img width="689" alt="image" src="https://github.com/nickyu42/waveshare-st-servo-lib/assets/10641355/d6f66d56-4932-429e-b306-077aefef49a1">

Alternatively, a simple diode and pull-up resistor can also be used:
![R1 4 7k](https://github.com/nickyu42/waveshare-st-servo-lib/assets/10641355/29791667-253b-40f2-a6f7-bb0a9be2d70f)

Taken from: https://github.com/dword1511/onewire-over-uart 

Note that with the above circuit, the RX signal will equal the TX signal in a transmission, so an MCU will read its own sent data, but that can be ignored in software.

Another approach could be to connect UART TX, RX, and DATA signals together with a pull-up resistor and use the MCUs TX line in an open-drain configuration, but this has not been tested.

