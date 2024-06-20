#!/bin/bash

# Get the absolute path of the directory where the script is located
TINY_GESTURES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define the name of the conda environment
ENV_NAME="tiny_gestures"

# Navigate to the root directory of the tiny_gestures project
cd "$TINY_GESTURES_DIR"

# Create a conda virtual environment named tiny_gestures
conda create -n $ENV_NAME python=3.8

# Activate the conda environment
conda activate $ENV_NAME

# Check if the requirements.txt file exists
if [ -f "requirements.txt" ]; then
    # Install all the packages listed in the requirements.txt file
    while read requirement; do
        # Skip empty lines and comments
        if [[ ! $requirement =~ ^#.*$ ]] && [[ ! -z $requirement ]]; then
            # Install the package with a specific version if a version number is specified
            conda install -c conda-forge "$requirement"
        fi
    done < requirements.txt
else
    echo "The requirements.txt file does not exist."
    exit 1
fi

# Deactivate the conda environment
conda deactivate

echo "Environment creation and dependency installation are complete."