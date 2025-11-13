# Phoenix Drivers
Phoenix 5/6 drivers for ROS2. Note that currently these are hard-configured for the LANCE-1.5 motor setup.

# Dependencies
1. Ensure you have patchelf installed. Without it some build commands will silently fail and the `phx5_driver` will exit with a linking error.
    - Run:
        ```
        sudo apt update
        sudo apt install patchelf
        ```
2. Install the [CTRE Phoenix 6](https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-nonfrc.html) library:
    - Run (configure `YEAR` with the current year):
        ```
        YEAR=2025
        sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
        sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr${YEAR}.list"
        ```
        *If you are installing on an arm machine, see the official documentation for changing the target distribution.*
    - Run:
        ```
        sudo apt update
        sudo apt install phoenix6
        ```

# Usage
The package exposes the nodes `phx5_driver` and `phx6_driver`, as well as launchfiles for each, although it is recommended to run the automated scripts to handle CAN startup/shutdown.

Run the phoenix 5 driver:
```
./scripts/launch_phoenix5_standalone.sh
```
Run the phoenix 6 driver:
```
./scripts/launch_phoenix6_standalone.sh
```

## Notes
### Phoenix 5 Driver
* There is a chance that motors will fail to initialize when the diagnostics server is disabled. For reliability, always configure the node with a valid diagnostics server port (>0) so that it is enabled.
### Phoenix 6 Driver
* `TalonInfo` messages `status` field should be decoded as follows (uint8_t):
    1. FIRST BIT: **Set if the device is enabled in hardware**
    2. SECOND BIT: **Set if the device is connected**
    3. THIRD BIT: **Set if a reset has occurred (triggered when the motor first connects)**
    4. FOURTH BIT: **Set if the phoenix API enabled**
    5. FIFTH BIT: **Set if the watchdog in an enabled state (ROS)**
    6. SIXTH BIT: **Set if NO blocks are in place signifying that configuration is required (motor config and set position)**
    7. SEVENTH BIT: **Set if the device has been marked as successfully configured**
    8. EIGHT BIT: **Set if the device doesn't need to have it's sensor position set**
