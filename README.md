# Custom user-space driver for the Xreal Air, Xreal Air 2 and Xreal Air 2 Pro to use it on Linux

## Information before use

The code is provided as is and it's free to use. However the contributors can neither guarantee that 
it will work or that it won't damage your device since all of it is based on reverse-engineering 
instead of public documentation. The contributors are not responsible for proper or even official 
support. So use it at your own risk!

## Inspiration and motivation

Because I've written a user-space driver before for a [graphics tablet](https://gitlab.com/TheJackiMonster/HuionGT191LinuxDriver), 
I thought I might look into it. To my surprise video output, audio output (stereo) and audio input (microphone) already 
worked using drivers from the [Linux kernel](https://linux-hardware.org/?id=usb:3318-0424). So the only piece missing 
for full use was the IMU sensor data in theory.

A big help for implementing this piece of software was the source code and feedback from a custom 
driver for Windows [here](https://github.com/MSmithDev/AirAPI_Windows/). Without that I would have 
needed to find out payloads my own. So big thanks to such projects being open-source!

Another huge help was the reverse engineering [here](https://github.com/edwatt/real_utilities/) to 
send different payloads to the device and even read calibration data from the local storage. Without 
that calibrating would be quite a hassle for end-users as well as efforts from developers tweaking 
the values manually. So big thanks as well!

## Features

The driver will read, parse and interpret sensor data from two HID interfaces to feed custom event 
callbacks with data which can be used in user-space applications (for example whether buttons have 
been pressed, the current brightness level and the orientation quaternion/matrix/euler angles).

It's still a work-in-progress project since the goal would be to wire such events up into a 
compositor to render whole screens virtually depending on your 6-DoF orientation (without position).

Also keep in mind that this software will only run on Linux including devices like the Steam Deck, 
Pinephone or other mobile Linux devices.

## Dependencies

You can build the binary using `cmake` and there are only three dependencies for now:
 - [hidapi](https://github.com/libusb/hidapi)
 - [json-c](https://github.com/json-c/json-c/)
 - [Fusion](https://github.com/xioTechnologies/Fusion)

Fusion is a sensor fusion library which is integrated as git submodule. So when you checkout the 
repository, just update the submodules to get it. The libraries `hidapi` and `json-c` should be 
pretty common in most Linux distributions repositories.

## Build

The build process should be straight forward:

```
mkdir build
cd build
cmake ..
make
```

## Run

It's easiest to run the software with root privileges.

```
sudo xrealAirLinuxDriver
```

Alternatively, you can copy the xreal_air.rules to /etc/udev/rules.d:

```
sudo cp udev/xreal_air.rules /etc/udev/rules.d/xreal_air.rules
```
