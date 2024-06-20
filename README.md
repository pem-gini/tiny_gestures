# Install Package Independence

If you already have corresponding PyTorch virtual conda environments, you can just ignore this step.

If not ,  you can run the script `set_virtual_environment.sh` to install independence.

The steps is below:

first: `cd` to the root path of the `tiny_gestures`repositiory in the terminal.

second: run`chmod +x set_virtual_environment.sh` in the terminal.

third: run `./set_virtual_environment.sh` to install the dependence in the terminal



# download the dataset

Using scripts to download the zip file from onedrive & google drive may be unstable, so I recommend you to download the dataset by yourself and unzip the downloaded zip file by hands.

Just download the dataset and put the dataset zip file in the root path of the `tiny_gestures`repositiory. Then unzip the dataset, extract the folders in the root path of the `tiny_gestures`repositiory as well. The extracted folders have the same name with its zip file. And all the samples & labels files are in the extracted folders.

There are two dataset compressed file, respectively names as "images_train.zip" & "images_val.zip" .

---
The download link from onedrive is here: 

images_train.zip:
https://m365rwthaachende.sharepoint.com/:u:/r/sites/GINI/Freigegebene%20Dokumente/Software%20Development/ai/gesture_recognition/tinyyolov4/dataset/images_train.zip?csf=1&web=1&e=EDLzcl

images_val.zip:
https://m365rwthaachende.sharepoint.com/:u:/r/sites/GINI/Freigegebene%20Dokumente/Software%20Development/ai/gesture_recognition/tinyyolov4/dataset/images_val.zip?csf=1&web=1&e=dVnggP

---

the download link from google drive is here:

images_train.zip:
https://drive.google.com/file/d/1_Wb7LixfWEZI7jdkVTUy3BSjYM9ALtN2/view?usp=sharing

images_val.zip:
https://drive.google.com/file/d/1_STF27fqk5-DEM5U-akvKZsq8Z6elqZ2/view?usp=sharing


---

After unzip the dataset, the structure of this project folder(tiny_gestures) is like: 

```bash
├── backup
├── description.md
├── detect
├── images_train
├── images_val
├── obj.data
├── obj.name
├── read_images_path.py
├── README.md
├── requirements.txt
├── set_virtual_environment.sh
├── step1__setup_darknet.sh
├── step2__start_training.sh
├── step3__start_inference.sh
├── yolov4-tiny.conv.29
└── yolov4-tiny-custom.cfg
```



# Preparation for Training

`cd` to the root path of the `tiny_gestures`repositiory in the terminal.

run `chmod +x step1__setup_darknet.sh` in the terminal.

run `./step1__setup_darknet.sh` to run the script to make file of the original repository "darknet" for yolov4-tiny training.





# Train the Model

`cd` to the root path of the `tiny_gestures`repositiory in the terminal.

run `chmod +x step2__start_training.sh` in the terminal.

run `./step2__start_training.sh` to run the script to train.

After execute the `step2__start_training.sh` script, you will get another two txt files under the root path of tiny_gestures: `images_train.txt` and `images_val.txt`. They are designed to capture the image samples' path.

If you want to stop the training process(training process for this dataset at least should run for  6000 steps, it is terminated by hand not automatically). 

The training time is decided by your GPU & you configuration in yolov4-tiny-custom.cfg, which is descriped in "description.md" file. RTX 4060 will take about 40 minutes to train. While mx150 will takes 5 hours to train.

run this in the terminal:`ps aux | grep darknet` to check the process id of the training process.

and run `pkill darknet` or `kill 12345`to end the training process. 12345 is just an example, it represents the process id of the training process.








# Inference

`cd` to the root path of the `tiny_gestures`repositiory in the terminal.

run `chmod +x step3__start_inference.sh` in the terminal.

run `./step3__start_inference.sh` to run the script to inference.


# More Details

You can read the "description.md" to see more training details, if you want to make your own modification.