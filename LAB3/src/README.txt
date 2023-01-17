This is a README file.
Yao Zhou
EECE 5554 LAB 3

Thanks for your reading my lab3 report!

For commands to run the GPS driver.
0. Make sure you connected IMU sensor with your computer, If you have a different port address/location, please change it inside the python script,
    otherwise it will keep generating error "No such file or directory: '/dev/ttyUSB0'"

1. in the terminal, go to the LAB directory
	"cd LAB3"

2. since there is only a src directory, do a 
	"catkin_make" 
    under LAB2 directory, the make should success without any problems.

3. then do 
	"source ./devel/setup.bash"

4. to run the python driver.py into with launch file
	"roslaunch imu_driver driver.launch port:="/dev/ttyUSB0" 

5. to start recorded with
	"rosbag record -a" 

6. then after you recorded, the bagfile would be generated into your dirctionary, then you need to check the bag information "rogbag info <bagfile name>" there are 40msg/s, and the only topic is IMU toipic.

7. you can use Matlab to draw the map, there are 2 file, first one is analysis noise characteristics of each variable, and the other is analysis Allan Variance. You can just used the matlab file to generate map.

*Additional Support"
If you didn't grant permission to gps port, it might fail to compile. Do the following:
	"sudo chmod 666 /dev/ttyUSB0"
typing the password for your Ubuntu, then everything should be fine afterwards.

-------------------------------------------------------------------
----The remaining parts of README file is lab explanation----
-------------------------------------------------------------------




