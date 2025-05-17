#!/bin/bash

colcon build --symlink-install --executor parallel --event-handlers console_direct+ --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
