#!/bin/bash

# Determine the directory of the current script, that is, the tiny_gestures directory
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Ensure gdown is installed
pip install gdown
if [ $? -ne 0 ]; then
    echo "Failed to install gdown"
    exit 1
fi

# Google Drive file IDs
TRAIN_ZIP_ID="1_Wb7LixfWEZI7jdkVTUy3BSjYM9ALtN2"
VAL_ZIP_ID="1_STF27fqk5-DEM5U-akvKZsq8Z6elqZ2"

# Download train zip file
gdown --id "$TRAIN_ZIP_ID" -O "$TINY_GESTURES_DIR/images_train.zip"
if [ $? -ne 0 ]; then
    echo "Failed to download images_train.zip"
    exit 1
fi

# Download validation zip file
gdown --id "$VAL_ZIP_ID" -O "$TINY_GESTURES_DIR/images_val.zip"
if [ $? -ne 0 ]; then
    echo "Failed to download images_val.zip"
    exit 1
fi

# Ensure the files are fully downloaded
if [ ! -f "$TINY_GESTURES_DIR/images_train.zip" ] || [ ! -s "$TINY_GESTURES_DIR/images_train.zip" ]; then
    echo "images_train.zip is not fully downloaded"
    exit 1
fi

if [ ! -f "$TINY_GESTURES_DIR/images_val.zip" ] || [ ! -s "$TINY_GESTURES_DIR/images_val.zip" ]; then
    echo "images_val.zip is not fully downloaded"
    exit 1
fi

# Decompress train zip file
unzip "$TINY_GESTURES_DIR/images_train.zip" -d "$TINY_GESTURES_DIR"
if [ $? -ne 0 ]; then
    echo "Failed to decompress images_train.zip"
    exit 1
fi

# Decompress validation zip file
unzip "$TINY_GESTURES_DIR/images_val.zip" -d "$TINY_GESTURES_DIR"
if [ $? -ne 0 ]; then
    echo "Failed to decompress images_val.zip"
    exit 1
fi

# Run read_images_path.py
python3 "$TINY_GESTURES_DIR/read_images_path.py"
if [ $? -ne 0 ]; then
    echo "Failed to run read_images_path.py"
    exit 1
fi

echo "All steps completed successfully."
