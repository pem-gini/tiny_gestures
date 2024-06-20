#!/bin/bash

# Determine the directory of the current script, that is, the tiny_gestures directory
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# find the path of obj.data
OBJ_DATA_FILE="$TINY_GESTURES_DIR/obj.data"

# Use the sed command to modify the path in the obj.data file
sed -i "s|^train.*|train = $TINY_GESTURES_DIR/images_train.txt|" "$OBJ_DATA_FILE"
sed -i "s|^valid.*|valid = $TINY_GESTURES_DIR/images_val.txt|" "$OBJ_DATA_FILE"
sed -i "s|^names.*|names = $TINY_GESTURES_DIR/obj.name|" "$OBJ_DATA_FILE"
sed -i "s|^backup.*|backup = $TINY_GESTURES_DIR/backup/|" "$OBJ_DATA_FILE"

# download zip file of the dataset
TRAIN_ZIP_URL="https://m365rwthaachende.sharepoint.com/:u:/r/sites/GINI/Freigegebene%20Dokumente/Software%20Development/ai/gesture_recognition/tinyyolov4/dataset/images_train.zip?csf=1&web=1&e=1C1vKA"
VAL_ZIP_URL="https://m365rwthaachende.sharepoint.com/:u:/r/sites/GINI/Freigegebene%20Dokumente/Software%20Development/ai/gesture_recognition/tinyyolov4/dataset/images_val.zip?csf=1&web=1&e=0KxbV6"

wget -O "$TINY_GESTURES_DIR/images_train.zip" "$TRAIN_ZIP_URL"
wget -O "$TINY_GESTURES_DIR/images_val.zip" "$VAL_ZIP_URL"

# decompress
unzip "$TINY_GESTURES_DIR/images_train.zip" -d "$TINY_GESTURES_DIR"
unzip "$TINY_GESTURES_DIR/images_val.zip" -d "$TINY_GESTURES_DIR"

# run read_images_path.py
python3 "$TINY_GESTURES_DIR/read_images_path.py"

# Get the necessary file path
OBJ_DATA_PATH="$TINY_GESTURES_DIR/obj.data"
CFG_PATH="$TINY_GESTURES_DIR/yolov4-tiny-custom.cfg"
WEIGHTS_PATH="$TINY_GESTURES_DIR/yolov4-tiny.conv.29"

# Enter the darknet directory and run the training command
DARKNET_DIR="$(dirname "$TINY_GESTURES_DIR")/darknet"
cd "$DARKNET_DIR"

./darknet detector train "$OBJ_DATA_PATH" "$CFG_PATH" "$WEIGHTS_PATH" -map

echo "training has started"
