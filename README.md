## LANCE-2025
This repo houses all client and robot code used for running LANCE "1.5" (2025). If setting up from scratch, you will want to read this entire document. For instructions on building/running, see the relevant sections: [BUILDING](#building) | [RUNNING](#running).

## Workspace Setup
Create a new workspace directory
```
mkdir lance-ws && cd lance-ws
```
Clone this branch into the `src` directory.
```bash
git clone --recurse-submodules -b main https://github.com/Cardinal-Space-Mining/lance-2025 src
```
If you forgot to clone recursively:
```bash
git submodule update --init --recursive
```

## Dependencies
1. Install [ROS2](https://docs.ros.org/en/jazzy/Installation.html) if not already done (we are using Jazzy for 2024-2025).

2. Use rosdep to install dependencies.
    - Initialize rosdep if not already done:
        ```bash
        sudo rosdep init
        ```
    - Update and install:
        ```bash
        rosdep update
        rosdep install --ignore-src --from-paths . -r -y
        ```

3. Install submodule dependencies.
    - Add phoenix6 apt sources:
        ```bash
        YEAR=2025
        sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
        sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr${YEAR}.list"
        ```
    - Install apt packages:
        ```bash
        sudo apt update
        sudo apt install libpcl-dev libopencv-dev python3-netifaces phoenix6 patchelf
        ```

## Building
The included build script builds all included packages. Run the following when inside the top-level workspace directory **(same for client AND robot)**:
```bash
./src/build.sh
source install/setup.bash
```
The "frappepanda" [lattepanda] needs minimal packages to run, so to speed up build times, use the following argument:
```bash
./src/build.sh --frappe-only
source install/setup.bash
```

## Running
Each target platform/machine has it's own script:

### 1. Client Laptop
Run:
```bash
./src/run_client.sh
```
If foxglove-bridge and robot-state-publisher are to be run on the robot, use:
```bash
./src/run_local.sh --remote-bridge
```

### 2. "Mochapanda"
Run:
```bash
./src/run_mocha.sh --client-bridge <perception={true/false}> <lidar-logging={true/false}>
```
To run foxglove-bridge and robot-state-publisher on the robot, remove the `--client-bridge` argument:
```bash
./src/run_mocha.sh <perception={true/false}> <lidar-logging={true/false}>
```

### 3. "Frappepanda"
Run:
```bash
./src/run_frappe.sh
```
Note that this runs a simplified script. If logging or other utility nodes need to be run (via script configuration parameters), run (this requires having built all packages):
```bash
./src/run_frappe.sh --full
```

## Foxglove Studio
A foxglove studio layout configuration (`foxglove_layout.json`) is included which provides a main control dashboard as well as tabs for each perception stage and motor status info. This can be loaded by clicking the **"LAYOUT"** dropdown in the top right corner of foxglove studio, then clicking **"Import from file..."** and navigating to the json.

## Simulation
Simulation assets (Gazebo and Nvidia Isaac) and launch utilities are encapsulated in a separate repo since including them here by default would make the repo quite bloated. Conveniently, the repo just needs to be cloned alongside the other packages to be built and used (see included readme for dependencies!):
```bash
pushd src && git clone https://gitlab.com/csm2.0/csm-sim && popd
```

## VSCode
The build script exports compile commands which can help VSCode's C/C++ extension resolve correct syntax highlighting. To ensure this is working, paste the following code into the `c_cpp_properties.json` file (under .vscode directory in a workspace):
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "intelliSenseMode": "linux-gcc-x64",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "compileCommands": [
                "build/compile_commands.json"
            ]
        }
    ],
    "version": 4
}
```
__*Last updated: 9/13/25*__
