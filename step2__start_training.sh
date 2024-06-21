#!/bin/bash

# Determine the directory of the current script, that is, the tiny_gestures directory
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Get sample path
python3 "$TINY_GESTURES_DIR/read_images_path.py"
if [ $? -ne 0 ]; then
    echo "Failed to run read_images_path.py"
    exit 1
fi

classes_file="$TINY_GESTURES_DIR/classes.txt"

if [ -f "$classes_file" ]; then
    num_categories=$(grep -v '^[[:space:]]*$' "$classes_file" | wc -l)
    {
    echo -e "classes = $num_categories"
    echo -e "train = $TINY_GESTURES_DIR/images_train.txt"
    echo -e "valid = $TINY_GESTURES_DIR/images_val.txt"
    echo -e "names = $TINY_GESTURES_DIR/classes.txt"
    echo -e "backup = $TINY_GESTURES_DIR/backup/"
    } > "$TINY_GESTURES_DIR/training_control_file.txt"
fi

# Get the necessary file path
OBJ_DATA_PATH="$TINY_GESTURES_DIR/training_control_file.txt"
CFG_PATH="$TINY_GESTURES_DIR/yolov4-tiny-custom.cfg"
WEIGHTS_PATH="$TINY_GESTURES_DIR/yolov4-tiny.conv.29"

# Enter the darknet directory and run the training command
DARKNET_DIR="$TINY_GESTURES_DIR/darknet"

# Run training command with nohup and redirect output to a log file
nohup $DARKNET_DIR/darknet detector train "$OBJ_DATA_PATH" "$CFG_PATH" "$WEIGHTS_PATH" -map > "$TINY_GESTURES_DIR/training.log" 2>&1 &

echo "Training has started. Check training.log for progress."
