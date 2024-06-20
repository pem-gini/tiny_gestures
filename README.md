# Install Package Independence

If you already have corresponding PyTorch virtual conda environments, you can just ignore this step.

If not ,  you can run the script `set_virtual_environment.sh` to install independence.

The steps is below:

first: `cd` to the root path of the `tiny_gestures`repositiory in the terminal.

second: run`chmod +x set_virtual_environment.sh` in the terminal.

third: run `set_virtual_environment.sh` to install the dependence in the terminal



# download the dataset

`cd` to the root path of the `tiny_gestures`repositiory in the terminal.

run `chmod +x step0__download_dataset.sh` in the terminal.

run `./step0__download_dataset.sh` to run the script to download the dataset.



# Preparation for Training

`cd` to the root path of the `tiny_gestures`repositiory in the terminal.

run `chmod +x step1__setup_darknet.sh` in the terminal.

run `./step1__setup_darknet.sh` to run the script to make file of the original repository "darknet" for yolov4-tiny training.





# Train the Model

`cd` to the root path of the `tiny_gestures`repositiory in the terminal.

run `chmod +x step2__start_training.sh` in the terminal.

run `./step2__start_training.sh` to run the script to train.





# Inference

`cd` to the root path of the `tiny_gestures`repositiory in the terminal.

run `chmod +x step3__start_inference.sh` in the terminal.

run `./step3__start_inference.sh` to run the script to inference.


# More Details

You can read the "description.md" to see more training details, if you want to make your own modification.