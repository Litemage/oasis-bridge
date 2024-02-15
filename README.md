# Purpose

The purpose of the O.A.S.I.S bridge service is to poll a target device 
which is expected to respond with sensor data. The device running this service 
will then post the received data to a redis server running locally.

This service was developed for the AGV ECE 398 Project of the FA23 semester. 

# Installation & Quick-Start

Follow these steps to get up and running with the oasis-bridge program.

## Dependencies

The oasis-bridge project only has one dependency. Otherwise, this will compile theoretically on any device implementing the POSIX standard.

- [hiredis](https://github.com/redis/hiredis), which is used for accessing the redis database

I'm going to assume that you're running either on an RPi or another similar OS. 

### Make sure you have git installed:

```
sudo apt install git
```

### Installing hiredis

To install hiredis, clone the repo:

```
git clone git@github.com:redis/hiredis.git
```

Ensure you have build tools (Cmake and the like):

```
sudo apt install build-essential
```

Navigate to the folder that was just cloned, created a build folder, and run the build

```
mkdir build
cd build
cmake ..
```

After that build succeeds, run:

```
make install
```

This should complete your install of hiredis

### Make sure you have Redis installed

Follow the Linux installation instructions on the [Redis website](https://redis.io/docs/install/install-redis/install-redis-on-linux/)

### Building oasis-bridge

First, clone the repo:

```
git clone http://github.com/Litemage/oasis-bridge.git
```

Navigate to the oasis-bridge repo that was just cloned, and run the following commands:

```
cd OasisBridge
source ./build.sh
```

### Running oasis-bridge

First, determine what serial interface the co-processor is connected to. (something like: `/dev/tty*`). You can look for serial devices by using the command:

```
ls -la /dev/tty*
```

For example, how some development boards' uart bridge chip show up:

- Arduino: `/dev/ttyACMx`
- ESP32: `/dev/ttyUSBx`

Next, make sure that the redis server is running locally:
```
sudo systemctl start redis-server.service 
```

Finally, navigate to the folder inside oasis-bridge repo: `(REPO FOLDER)/OasisBridge/build` and run:

```
oasis-bridge /dev/ttyACM0
```

there is a good chance the `/dev/ACM0` file may be different for you.

**This should be a complete set-up, data should now be bridging from serial to redis :)**

# Slave Expectations

The coprocessor should have the following RS2332 configuration to work with oasis-bridge:

| Configuration Parameter | Value |
| --- | --- |
| Baud Rate | 115200 |
| Data bits | 8 |
| Parity    | None |
| Stop bits | 1 |

## Communication

The following sequence diagram should be implemented on the coprocessor oasis-bridge is accessing:

![oasis-bridge-sequence](https://github.com/Litemage/oasis-bridge/assets/77081880/425c4bbf-c970-4456-83ab-1eb9ff74beca)

**SensorVals_t frame format:**

<table>
  <tr>
    <th colspan="9">Data Frame Format</th>
  </tr>
  <tr>
    <th>Offset:</th>
    <th>0</th>
    <th>1</th>
    <th>2</th>
    <th>3</th>
    <th>4</th>
    <th>5</th>
    <th>6</th>
    <th>7</th>
  </tr>
  <tr>
    <th>Value:</th>
    <td colspan="2">Left Distance Cm</td>
    <td colspan="2">Forward Distance Cm</td>
    <td colspan="2">Right Distance Cm</td>
    <td colspan="2">Servo Angle Deg</td>
  </tr>
</table>

**NOTE: All values are little endian in the packet**
