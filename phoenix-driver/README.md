# PhoenixMotorInterface
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
