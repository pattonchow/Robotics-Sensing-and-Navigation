This is a README file.
Yao Zhou
EECE 5554 LAB 2

For commands to run the GPS driver.
0. Make sure you have a "roscore" running, and gps port plug in with address "/dev/ttyACM0"
    If you have a different port address/location, please change it inside the python script,
    otherwise it will keep generating error
	"No such file or directory: '/dev/ttyACM0'"

1. in another terminal, go to the LAB2 directory
	"cd LAB2"

2. since there is only a src directory, do a 
	"catkin_make" 
    under LAB2 directory, the make should success without any problems.

3. then do 
	"source ./devel/setup.bash"

4. dive into the ros package
	"roscd gnss_driver"

5. to run the python script (the gps driver)
	"rosrun gnss_driver gnss_reader.py"

*Additional Support"
If you didn't grant permission to gps port, it might fail to compile. Do the following:
	"sudo chmod 666 /dev/ttyACM0"
typing the password for your Ubuntu, then everything should be fine afterwards.

-------------------------------------------------------------------
----The remaining parts of README file is lab explanation----
-------------------------------------------------------------------

Two datasets are collected from different locations. 
One is from MacDonald Stadium next to Malden Center MBTA station.
This is the data for completely clear spot.
The bagfile is stationary.bag and walk_base.bag and walk_rover.bag
I also tranform these bagfiles into yaml with same name.

The other one is from Northeastern University next to the West Village Hall.
This spot is for data that is partially occlusion and reflection nearby.
The bagfile is stationary2.bag and walk_rover2.bag


The data are all recorded using "rosbag record -a" commands.
Afterwards, we firstly check the bag information "rogbag info <bagfile name>"
Then we convert our bagfile into yaml format.
The python can read yaml file directly without any issue.
Inside analysis scripts, I commented out many lines.
If you want to see different plots, you may want to uncomment some line, choose the correct file you desired and corrent plot you wanted.

In analysis folder, I create analysis.py which generate the figures in my report, you can open my analysis folder in Pycharm or Vs code to generate the figure I used in report, I mainly use 2D scatter plot and 3D scatter Plot with matplotlib.pyplot.