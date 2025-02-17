# Custom user-space driver for the Xreal Air, Xreal Air 2 and Xreal Air 2 Pro to use it on MacOS

This is a macOS port of the [nrealAirLinuxDriver](https://gitlab.com/wheaney/nrealAirLinuxDriver). The original Linux implementation has been adapted to work on macOS while maintaining the core functionality.

## Information before use

The code is provided as is and it's free to use. However the contributors can neither guarantee that
it will work or that it won't damage your device since all of it is based on reverse-engineering
instead of public documentation. The contributors are not responsible for proper or even official
support. So use it at your own risk!

## Inspiration and motivation

This macOS port aims to bring the functionality of the Linux driver to macOS users. The original Linux driver was instrumental in understanding the device's communication protocol and behavior.

Because the original author wrote a user-space driver before for a [graphics tablet](https://gitlab.com/TheJackiMonster/HuionGT191LinuxDriver), they looked into implementing support for the Xreal Air. On macOS, like Linux, the video output, audio output (stereo) and audio input (microphone) already work using built-in drivers. The only piece missing for full use was the IMU sensor data.

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

Also keep in mind that this software will only run on macOS. For Linux support, please refer to the original [nrealAirLinuxDriver](https://gitlab.com/wheaney/nrealAirLinuxDriver).

## Dependencies

You can build the binary using `cmake`. The following dependencies are included as submodules and get statically linked during build:

- [hidapi](https://github.com/libusb/hidapi)
- [Fusion](https://github.com/xioTechnologies/Fusion)
- [magnetometer-calibrate](https://github.com/wheaney/magnetometer-calibrate)
- [json-c](https://github.com/json-c/json-c/)
- [gsl](https://www.gnu.org/software/gsl/) (dependency of magnetometer-calibrate)

When you checkout the repository, be sure to update the submodules.

## Build

The build process should be straight forward:

```
mkdir build
cd build
cmake ..
make
```

## Run

On macOS, you'll need to run the software with sudo:

```
sudo xrealAirMacDriver
```

Note: Unlike Linux, macOS handles device permissions differently, so udev rules are not applicable here. The driver needs to run with root privileges to access the HID interfaces.
