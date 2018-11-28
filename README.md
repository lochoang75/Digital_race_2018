# Digital\_race\_2018
Source code for digital race 2018
## Include
1. Source code for lane detect.
2. Model for sign detect.
3. Algorithm for car control (update soon).
4. Bug and many bug.
## How to use ?
1. Install ROS.
2. Clone or pull newest version.
3. Copy to src folder of catkin\_ws.
4. Fix any static link (ex: cascade.xml...).
5. Run
	- Run Dytqet team node
```
catkin_make
source devel/setup.bash
roslaunch dytqet_node dytqet_node.launch
```
	- Run lane - detect node (with some test function from Thanh)
```
catkin_make
source devel/setup.bash
roslaunch lane_detect lane_detect.launch
```
6. Run FPT app, if you just want to run lane\_detect node you only need to select Manual or Auto in app.
7. Enjoy
## Author
This project belong to Dyqet team_ Digital race 2018

