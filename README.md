*不强制每人一个dev，但是在push到主分支上时请谨慎，push之前先pull本地解决冲突。*

## 路径组织形式

创建一个ros的ws，然后所有的功能包放在src/purple-bee-icuas2022-uav路径下，和uav_nav同级，暂时想到的需要添加有{识别二维码：arucode}，{控制无人机摇摆扔出小球:drop}。之后新创建的功能包在下面创建一个二级标题说明使用方法，仿照uav_nav。

```shell
$ pwd
.../uav_icuas2022_ws/src
$ tree
.
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
└── purple-bee-icuas2022-uav
    ├── README.md
    └── uav_nav
        ├── CMakeLists.txt
        ├── include
        │   └── icuas_solution
        ├── launch
        │   └── keyboard_ctr.launch
        ├── package.xml
        ├── scripts
        │   └── cmd_twist.py
        └── src

7 directories, 6 files
```



## uav_nav

通过使用交键盘控制无人机的位置。

+ 安装键盘rospackage

    ```powershell
     sudo apt-get install ros-noetic-teleop-twist-keyboard
    ```

+ 使用

    放到自己的catkin_ws里面然后catkin_make

    ```shell
    roslaunch uav_nav keyboard_ctr.launch
    ```


## uav_controller

一些收发信息的接口，封成一个类了，主要就是`scripts/PSuber.py`



## uav_map

建图功能包，使用cartographer，现在还不能用。



## depthimage_to_laserscan

将深度相机的信息转换为二维激光雷达信息

+ `catkin_make`

+ 运行`roslaunch`即可获取到`/scan`话题下的激光雷达信息，`test.launch`可更改一些参数，可以参考[ROS Wiki](http://wiki.ros.org/depthimage_to_laserscan)，也可以看看`./cfg/Depth.cfg`

  ```
  roslaunch depthimage_to_laserscan test.launch
  ```

+ 读取`/scan`下`sensor_msgs/LaserScan`类型信息



## ar_track_alvar

识别并发布AR tag位置信息

+ `catkin_make`
+ 运行`roslaunch`可订阅`/visualization_marker`话题下的`visualization_msgs/Marker`类型信息，其有一成员pose（`geometry_msgs/Pose`类型），可直接读取

  ```sh
  roslaunch ar_track_alvar test.launch
  ```

+ 同样的，可更改launch文件的参数
+ **TODO**： 将launch文件串起来