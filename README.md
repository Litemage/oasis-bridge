# Purpose

The purpose of the O.A.S.I.S bridge service is to poll a target i2c device 
address which is expected to have sensor data at particular addresses. The 
master device running this service will then post that data to a redis server 
running locally.

# Notes

Due to hardware limitations on the raspberry pi, the pi cannot act as an i2c
slave, so the plan is to access the co-processor (arduino, in our case) which
will be the i2c slave, and we can implement an interrupt line between the 
co-processor and the device running this service as a "data ready" line.

# Slave Expectations

The I2C slave, also called the "target" is expected to have the following 
parameters:

|======================|===================|
| I2C ADDRESS (TARGET) | 0x40 (64 decimal) |
| I2C ADDRESS SPEED    | 100khz            |
|======================|===================|

## Communication

The master (device running this service) shall access the slave through it's 
address, and then it's register address(es).

See [this appnote from ti](https://www.ti.com/lit/an/slva704/slva704.pdf?ts=1700006656921) for some more i2c information

The I2C Exchange should go something like this: ("\*" indicates slave or "target" is in control of SDA line)

Read operation:
|=============================================================================================================|
| S | SLAVE ADDR + WRITE | A\* | REGISTER ADDRESS | A\* | Sr | SLAVE ADDR + READ | A\* | REGISTER DATA\* | NA | P |
|=============================================================================================================|

Write Operation:
|==============================================================================|
| S | SLAVE ADDR + WRITE | A\* | REGISTER ADDRESS | A\* | REGISTER DATA | A\* | P |
|==============================================================================|

**LEGEND**

|=====================|
| START          | S  |
| REPEATED START | Sr |
| STOP           | P  | 
| ACK            | A  | 
| NACK           | NA | 
|=====================|

## Registers (Target)

O.A.S.I.S bridge service expects the following registers to be implemented on the target device:

I don't think we'll need more than 255 registers, so 8-bit reg address...

|==========================|
| ADDRESS | DESCRIPTION    |
|==========================|
| 0x10    | Left Sensor    |
| 0x11    | Front Sensor   |
| 0x12    | Right Sensor   |
| 0x13    | Down Sensor    |
|==========================|

The target device needs to implement the above registers and should have the following data at these registers:

**Left Sensor** -> The distance that the left sensor on the robot detects, in cm

**Front Sensor** -> Same as left sensor, but in front, in cm

**Right Sensor** -> Same as above, but right, in cm

**Down Sensor** -> Same as above, but downward facing, in cm
