## Branches
Different branches contain different deployments, as indicated by branch name. PLEASE DO NOT MERGE DEPLOYMENT-SPECIFIC BRANCHES INTO EACH OTHER! They are separated to simplify the workspace complexity for each deployment (as well as reduce clone/pull overhead), so doing this would defeat the purpose.

## Setup
Create a new workspace directory
```
mkdir ws && cd ws
```
Clone this branch into the `src` directory.
```bash
git clone --recurse-submodules -b remote https://github.com/Cardinal-Space-Mining/lance-2025 src
```
If you forgot to clone recursively:
```bash
git submodule update --init --recursive
```

## Build
1. Install [ROS2](https://docs.ros.org/en/jazzy/Installation.html) if not already done (we are using Jazzy for 2024-2025)

2. Use rosdep to install dependencies
    - Initialize rosdep if not already done:
        ```bash
        sudo rosdep init
        ```
    - Update and install:
        ```bash
        rosdep update
        rosdep install --ignore-src --from-paths . -r -y
        ```
    - Check submodule READMEs to install any other necessary deps

3. Build with colcon
    ```bash
    colcon build --symlink-install <--executor parallel> <--event-handlers console_direct+>
    source install/setup.bash
    ```

## VSCode
See the [Cardinal Perception README](https://github.com/Cardinal-Space-Mining/Cardinal-Perception) for details on fixing vscode.