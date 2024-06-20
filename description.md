# How to train your own custom yoloV4-tiny model

# Dataset selection

Here we use HaGRID - HAnd Gesture Recognition Image Dataset as our target datasets. The repository link is here: [hagrid](https://github.com/hukenovs/hagrid). 

In the readme file of hagrid, u can easily find the dataset download links.



# Make your own dataset(labeling)

The HaGRID's dataset does not provide you with labels. It seems only containing the original JPG pictures. U can use the software "labelimg" to make your own yolo label. 

Remember that before u mark the label, u have to set "labelimg" in the yolo marking mode. Otherwise, the default marking mode is VOC format. 

The link of the software "labelimg" is here: [labelimg](https://github.com/HumanSignal/labelImg) 

The instruction of this software is presented in the readme file of the labelimg repository.



# Create your own yoloV4-tiny project work

## The structure of the project work

U need to create a project folder, and put some specified subfolders and files in this project folder. Here, I will name this project folder as "tiny_gestures".

The structure of this project folder is like: 

```bash
.
├── backup
├── detect
├── images_train
├── images_val
├── obj.data
├── obj.name
├── read_images_path.py
├── yolov4-tiny.conv.29
└── yolov4-tiny-custom.cfg
```

👆 These subfolders and files are in the project folder "tiny_gestures".

Among them, 

- There are subfolders: 
  - backup
  - detect 
  - images_train
  - images_val
- There are files: 
  - obj.data
  - obj.name 
  - read_images_path.py 
  - yolov4-tiny.conv.29 
  - yolov4-tiny-custom.cfg



## Subfolders

All of these subfolders are created by yourself. U just create them, which are empty now.

"backup" is used to store your custom trained yoloV4-tiny model after your own training.

"detect" will store nothing, if u do not need to do inference test.

"imgae_train" stores all the jpg images used for training, and all the corresponding txt labels files (yolo format). U just put them together into this subfolder. 

Do not hesitate, the images & corresponding labels file are put together stored in the same folder, which is different from other YOLO repository, and different from standard yolo dataset format.

The function of "image_val" is the same with "image_train". The only difference is it stores the jpg images used for validation, and all the corresponding txt labels files (yolo format).



## Files

### obj.data

The content in it is like: 

```txt
classes = 7
train  = /home/user/Tasks__Gestures_Classification/tiny_gestures/images_train.txt
valid  = /home/user/Tasks__Gestures_Classification/tiny_gestures/images_val.txt
names = /home/user/Tasks__Gestures_Classification/tiny_gestures/obj.name
backup = /home/user/Tasks__Gestures_Classification/tiny_gestures/backup/
```

You need to change these variable and the path according to your own condition.(absolute path)

You must be curious about where "images_train.txt" and "images_val.txt" comes from.

Do not worry, just run the script "read_images_path.py" and then u will get these two files in the project root path.

This file is used to define the classes that your task needs. And the "images_train.txt" will record the path of all your training data (images).

obj.name is another file under the project root path.

backup is a subfolder which has been mentioned above.



### obj.name

This file stores the class information, and use yolo format, like:

```txt
ok
stop
palm
fist
like
peace
peace_inv
```

These are all the class's label.



### yolov4-tiny-custom.cfg

[yolov4-tiny-custom.cfg](https://github.com/AlexeyAB/darknet/blob/master/cfg/yolov4-tiny-custom.cfg) is the link of this file. It comes from darknet repository [darknet](https://github.com/AlexeyAB/darknet?tab=readme-ov-file#when-should-i-stop-training) .

Just copy this file and modify some lines of them according to your requirements. The full description can be found at [here](https://github.com/AlexeyAB/darknet?tab=readme-ov-file#how-to-train-to-detect-your-custom-objects) .

1. Line 6 and line 7 need to be modified. Bigger "batch" leads to fast training, while bigger "subdivision" leads to slower training. For example, bacth = 64, subdivision = 4, means every step will take $64\div 4=16 $ images. U need to adjust these two parameters to adjust to your own GPU device. This example will take about 3 G video ram.
2. Line 20 and 22 need to be modified. **"max_batches" = number of your classes × 2000**.      **"steps" =  80% and 90% of "max_batches"**. For example, max_batches = 14000, steps=11200,12600.
3. Line 212 and Line 263 need to be modified. "filters" which actually is the last filters title before 2 [yolo] marks, should be modified **filters=(classes + 5)x3)**. So here we use 36 to substitute the old one. Do forget there are 2 "filters" title need to be modified, not one.

Then, enough, at least for your custom training.



### yolov4-tiny.conv.29

U can find this file's link in darknet repository as well. Here is the link: [yolov4-tiny.conv.29](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.conv.29)

Just download it and put it into the project root path





# Clone darknet repository

Clone the darknet repository in another place (not under the project folder u just created)

Here is the link: [darknet](https://github.com/AlexeyAB/darknet?tab=readme-ov-file#how-to-train-to-detect-your-custom-objects) 



## Modify the Makefile

Makefile is just under the root path of the darknet repository.

Change the first four lines from 0 to 1. 

```bash
GPU=1
CUDNN=1
CUDNN_HALF=1
OPENCV=1
```

Input "make" in the terminal to make the file, then it will generate a target file called "darknet".

Then we can start training.



## Training

Input this command into the terminal(under the root path of darknet repository):

```bash
./darknet detector train /home/user/Tasks__Gestures_Classification/tiny_gestures/obj.data /home/user/Tasks__Gestures_Classification/tiny_gestures/yolov4-tiny-custom.cfg /home/user/Tasks__Gestures_Classification/tiny_gestures/yolov4-tiny.conv.29 -map
```

The structure of this command is : 

`./darknet detector train` 

+

path of `obj.data`in your own project folder 

+

path of `yolov4-tiny-custom.cfg`in your own project folder  

+

path of `yolov4-tiny.conv.29`in your own project folder.

+

-map



## Inference

Almost the same with training.

```bash
./darknet detector test /home/user/Tasks__Gestures_Classification/tiny_gestures/obj.data /home/user/Tasks__Gestures_Classification/tiny_gestures/yolov4-tiny-custom.cfg /home/user/Tasks__Gestures_Classification/tiny_gestures/backup/yolov4-tiny-custom_best.weights
```

input an image to inference: `/home/user/Tasks__Gestures_Classification/tiny_gestures/images_val/stop_001.jpg`

succeed.



# Over



