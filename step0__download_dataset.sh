#!/bin/bash

# Determine the directory of the current script, that is, the tiny_gestures directory
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# download zip file of the dataset
TRAIN_ZIP_URL="https://drive.google.com/file/d/1_Wb7LixfWEZI7jdkVTUy3BSjYM9ALtN2/view?usp=sharing"
VAL_ZIP_URL="https://drive.google.com/file/d/1_STF27fqk5-DEM5U-akvKZsq8Z6elqZ2/view?usp=sharing"

# Download train zip file
wget -O "$TINY_GESTURES_DIR/images_train.zip" "$TRAIN_ZIP_URL"
if [ $? -ne 0 ]; then
    echo "Failed to download images_train.zip"
    exit 1
fi

# Download validation zip file
wget -O "$TINY_GESTURES_DIR/images_val.zip" "$VAL_ZIP_URL"
if [ $? -ne 0 ]; then
    echo "Failed to download images_val.zip"
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
