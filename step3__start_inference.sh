#!/bin/bash

# Determine the directory of the current script, that is, the tiny_gestures directory
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Get the necessary file path
OBJ_DATA_PATH="$TINY_GESTURES_DIR/obj.data"
CFG_PATH="$TINY_GESTURES_DIR/yolov4-tiny-custom.cfg"
WEIGHTS_PATH="$TINY_GESTURES_DIR/backup/yolov4-tiny-custom_best.weights"
IMAGE_PATH="$TINY_GESTURES_DIR/images_val/palm_001.jpg"

# Check if a file exists
if [ ! -f "$OBJ_DATA_PATH" ]; then
    echo "file $OBJ_DATA_PATH does not exist"
    exit 1
fi

if [ ! -f "$CFG_PATH" ]; then
    echo "file $CFG_PATH does not exist"
    exit 1
fi

if [ ! -f "$WEIGHTS_PATH" ]; then
    echo "file $WEIGHTS_PATH does not exist"
    exit 1
fi

if [ ! -f "$IMAGE_PATH" ]; then
    echo "file $IMAGE_PATH does not exist"
    exit 1
fi

# Enter the darknet directory and run the inference command
DARKNET_DIR="$(dirname "$TINY_GESTURES_DIR")/darknet"
cd "$DARKNET_DIR"

# Run the inference command and enter the image path
./darknet detector test "$OBJ_DATA_PATH" "$CFG_PATH" "$WEIGHTS_PATH" <<< "$IMAGE_PATH"

echo "inference is completed"
