# Capra SpaceMouse Package

This ROS 2 package provides a node that publishes SpaceMouse input as a `Joy` message. It continuously reads the state of the SpaceMouse and publishes the data, and can handle the SpaceMouse being disconnected.

## Overview

The main node in this package is `spacemouse_joy.py`, which handles the connection to the SpaceMouse, reads its state, and publishes the data as a `Joy` message. If the SpaceMouse is disconnected, the node will attempt to reconnect and continue publishing zero values for all axes until the connection is restored.

The outputs represent the axes as follows:
- `float` - Translation X (left/right)
- `float` - Translation Y (forward/backward)
- `float` - Translation Z (up/down)
- `float` - Rotation Pitch (forward/backward)
- `float` - Rotation Roll (twist)
- `float` - Rotation Yaw (left/right)
- `int` - Left button
- `int` - Right button

### Files

- **spacemouse_joy.py**: This file contains the `SpaceMouseJoy` class, which is responsible for connecting to the SpaceMouse, reading its state, and publishing the data as a `Joy` message. It also handles reconnection attempts if the SpaceMouse is disconnected.

- **joy_listener.py**: This file contains the `JoyListener` class, which subscribes to the `Joy` messages published by the `spacemouse_joy.py` node. It prints the axes and button states to the console, providing a clean output to verify that the publisher is working correctly.

## Setup

This setup is simplified for Ubuntu 22.04. See the original documentation for a complete setup.

### Part 1: Installing PySpaceMouse

Source: [PySpaceMouse GitHub](https://github.com/JakubAndrysek/PySpaceMouse)

1. Install the package:
    ```sh
    pip install pyspacemouse
    ```

2. Install dependencies:
    1. The library uses `hidapi` as a low-level interface to the device and `easyhid` as a Python abstraction for easier use.
        ```sh
        sudo apt-get install libhidapi-dev
        ```

    2. Add rules for permissions:
        ```sh
        sudo echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' > /etc/udev/rules.d/99-hidraw-permissions.rules
        sudo usermod -aG plugdev $USER
        newgrp plugdev
        ```

    3. `easyhid` is a `hidapi` interface for Python - required on all platforms:
        ```sh
        pip install git+https://github.com/bglopez/python-easyhid.git
        ```

### Part 2: Give Permission to Access HID Devices

By default, ordinary users may not have permission to access HID devices.

Source (Chinese): [CSDN Blog](https://blog.csdn.net/qq_40081208/article/details/144306644)

1. Run the following command to add the `idVendor` and `idProduct` of the 3D SpaceMouse in udev rules:
    ```sh
    sudo tee /etc/udev/rules.d/99-spacemouse.rules > /dev/null <<EOF
    SUBSYSTEM=="input", GROUP="input", MODE="0660"
    KERNEL=="hidraw*", ATTRS{idVendor}=="256f", ATTRS{idProduct}=="c635", MODE="0666"
    EOF
    ```
    *These IDs are for CAPRA's SpaceMouse. See the original setup for other devices.*

2. Reload udev rules:
    ```sh
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

3. Disconnect and reconnect the SpaceMouse.

4. Make sure the current user belongs to the input group (expect no output):
    ```sh
    sudo usermod -a -G input $USER
    ```

5. Log out and log back in to Ubuntu.