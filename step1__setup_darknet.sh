#!/bin/bash

# Determine the directory of the current script, that is, the tiny_gestures directory
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Initialize and update submodules
git submodule update --init --recursive
if [ $? -ne 0 ]; then
    echo "Failed to initialize and update submodules"
    exit 1
fi

# Modify the Makefile
DARKNET_DIR="$TINY_GESTURES_DIR/darknet"
MAKEFILE="$DARKNET_DIR/Makefile"

# Use the sed command to modify the first four lines of the Makefile file
sed -i '1s/.*/GPU=1/' "$MAKEFILE"
if [ $? -ne 0 ]; then
    echo "Failed to modify GPU option in Makefile"
    exit 1
fi

sed -i '2s/.*/CUDNN=1/' "$MAKEFILE"
if [ $? -ne 0 ]; then
    echo "Failed to modify CUDNN option in Makefile"
    exit 1
fi

sed -i '3s/.*/CUDNN_HALF=1/' "$MAKEFILE"
if [ $? -ne 0 ]; then
    echo "Failed to modify CUDNN_HALF option in Makefile"
    exit 1
fi

sed -i '4s/.*/OPENCV=1/' "$MAKEFILE"
if [ $? -ne 0 ]; then
    echo "Failed to modify OPENCV option in Makefile"
    exit 1
fi

# Enter the darknet directory and run make
cd "$DARKNET_DIR"
if [ $? -ne 0 ]; then
    echo "Failed to enter darknet directory"
    exit 1
fi

make
if [ $? -ne 0 ]; then
    echo "Failed to compile darknet"
    exit 1
fi

echo "The darknet repository has been added as a submodule, the Makefile has been modified, and the compilation is completed."
