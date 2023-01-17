# ORBSLAM3 on Visual SLAM
## Final project for EECE5554 @Northeastern University
## Team8: Yao Zhou, Shuchong Wang, Zhiyu Zhu, Tianrun Yan, Qiming Huang
 


## Installation
1. Install ORBSLAM3 and its dependencies

ORBSLAM3 and droidcam repositories have already been forked into our github repo. Please refer to the installation links mentioned above to download these necessary components. It is highly recommended to check the official github repo. We also get a modified version monoVO-Python [here](https://github.com/SaulBatman/monoVO-python) as our baseline, where we also add out kitti toolkits(dataprocessing directory). 

Please follow the installation guidance [here](https://github.com/UZ-SLAMLab/ORB_SLAM3).
Follow the instructions provided in ORB SLAM3 github and install the listed softwares and their dependencies

    1. Pangolin 
    2. OpenCV 4.4.0
    3. ORB SLAM3

2. Install ROS driver for steaming
  On your Ubuntu PC, follow [this](https://github.com/ros-drivers/video_stream_opencv).


## Download the Dataset
  1. EuRoc Dataset [link](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip).

  1. TUM Dataset [link](http://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room4_512_16.tar).

  1. Kitti Dataset [link](https://drive.google.com/drive/folders/1K5SzPNkahGHBhbs4ry2Aajzvn2mHcwQI?usp=sharing). 
  Download the kitti odometry dataset (grayscale 22GB) which contains 22 sequences of left and right stereo images along with their timestamps and calibration files.

## Run the Dataset
The EuRoc Dataset could be run with final Command:
 
    ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi
   

The TUM Dataset could be run with final Command:

    ./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM_512.yaml ~/Datasets/TUM_VI"/dataset-magistrale4_512_16/mav0/cam0/data Examples/Monocular-Inertial/TUM_TimeStamps/dataset-magistrale4_512.txt Examples/Monocular-Inertial/TUM_IMU/dataset-magistrale4_512.txt dataset-magistrale4_512_monoi

As For The Kitti Dataset:
Please create a single folder for every dataset and store the video in it. Use our KITTI toolkit in this folder to convert video into frames and timeStamps.

For example, create a folder named "DATA00" under /dataprocessing, and store the video in /dataprocessing/DATA00. Then, modify the corresponding path in /dataprocessing/video_process.py

Datasets we collect for our own use [link](https://drive.google.com/drive/folders/13_z45tFl9xYl8y7eHGY1H6qEW9LOxmUA?usp=sharing).

## Video
Three shortened videos from the presentation are located in the videos directory since the videos cannot be displayed properly in a PDF version of the Final Presentation.

