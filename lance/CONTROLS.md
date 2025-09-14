## Controls

This node subscribes to the `/joy` ros topic for user input, and is configured for the logitech (xbox layout) bindings. Controls are listed below:

* **LEFT STICK**: When in manual mode, controls the tracks (arcade style).
* **RIGHT STICK**: When in manual mode, controls the hopper level. Pressing up will raise the hopper and pressing down will lower it.
* **LEFT TRIGGER**: When in manual mode, controls the trencher. Pressing **LB** (above the trigger) will reverse the direction.
* **RIGHT TRIGGER**: When in manual mode, controls the hopper belt. Pressing **RB** (above the trigger) will reverse the direction.
* **X**: Set the track speed (multiplier) to HIGH.
* **Y**: Set the track speed (multiplier) to MEDIUM - this is the default value.
* **B**: Set the track speed (multiplier) to LOW.
* **A**: Disable any running commands and stop all motors (soft e-stop).

* **LEFT STICK BUTTON**: Enables "Assisted Mining Mode". This will lower the hopper, spin the trencher, and begin driving forwards while incrementing the hopper belt. While in this mode, the **LEFT STICK** increases/decreases the driving speed (offset from base value), the **RIGHT STICK** controls the trencher depth (offset from base value), and the **LEFT TRIGGER** slows down the trenching speed. Pressing the stick button again will raise the trencher such that the robot is safe to begin driving, and pressing A cancels the command.
* **RIGHT STICK BUTTON**: Enables "Assisted Offload Mode". This will raise the hopper, spin the hopper belt for a preset duration, and lower the hopper again. While in this mode, the **LEFT STICK** can be used to drive the robot forward/backwards (turning is disabled). Pressing the stick while running will shutdown the command early and pressing A will cancel the command without shutting it down.

* The **DPAD** buttons run preconfigured semi-autonomous routines. Pressing **DPAD UP** runs a timed mining cycle (**DPAD DOWN** to cancel), and **DPAD RIGHT** runs a timed offload cycle (**DPAD LEFT** to cancel).