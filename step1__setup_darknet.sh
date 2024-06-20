#!/bin/bash

# Determine the directory of the current script, that is, the tiny_gestures directory
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Add darknet submodule
git submodule add https://github.com/AlexeyAB/darknet.git

# Initialize and update submodules
git submodule update --init --recursive

# Modify the Makefile
DARKNET_DIR="$TINY_GESTURES_DIR/darknet"
MAKEFILE="$DARKNET_DIR/Makefile"

# Use the sed command to modify the first four lines of the Makefile file
sed -i '1s/.*/GPU=1/' "$MAKEFILE"
sed -i '2s/.*/CUDNN=1/' "$MAKEFILE"
sed -i '3s/.*/CUDNN_HALF=1/' "$MAKEFILE"
sed -i '4s/.*/OPENCV=1/' "$MAKEFILE"

# Enter the darknet directory and run make
cd "$DARKNET_DIR"
make

echo "The darknet repository has been added as a submodule, the Makefile has been modified, and the compilation is completed."
