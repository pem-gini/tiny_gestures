# Install Package Independence

If you already have corresponding PyTorch virtual conda environments, you can just ignore this step.

If not ,  you can run the script `set_virtual_environment.sh` to install independence.

The steps is below:

first: `cd` to the root path of the `tiny_gestures`repositiory in the terminal.

second: run`chmod +x set_virtual_environment.sh` in the terminal.

third: run `./set_virtual_environment.sh` to install the dependence in the terminal



# download the dataset

Using link downloads in scripts may not be stable, so I recommend you to download the dataset by yourself and decompress the downloaded zip file by hands.

Just download the dataset and put the zip file in the root path of the `tiny_gestures`repositiory. And decompress the dataset, extract the folders in the root path of the `tiny_gestures`repositiory as well.

The download link from onedrive is here: 

images_train.zip:
https://m365rwthaachende.sharepoint.com/:u:/r/sites/GINI/Freigegebene%20Dokumente/Software%20Development/ai/gesture_recognition/tinyyolov4/dataset/images_train.zip?csf=1&web=1&e=EDLzcl

images_val.zip:
https://m365rwthaachende.sharepoint.com/:u:/r/sites/GINI/Freigegebene%20Dokumente/Software%20Development/ai/gesture_recognition/tinyyolov4/dataset/images_val.zip?csf=1&web=1&e=dVnggP

the download link from google drive is here:

images_train.zip:
https://drive.google.com/file/d/1_Wb7LixfWEZI7jdkVTUy3BSjYM9ALtN2/view?usp=sharing

images_val.zip:
https://drive.google.com/file/d/1_STF27fqk5-DEM5U-akvKZsq8Z6elqZ2/view?usp=sharing





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