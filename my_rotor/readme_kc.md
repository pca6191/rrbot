### 啟動程式

    $ roslaunch my_rotor rotor.launch  

在 gazebo 會看到：一長立柱、一短旋臂。

### dependency

要裝 ros-kinetic-gazebo-ros-control

### 轉動旋臂

ex.  

    rostopic pub -1 /rotor/joint1_velocity_controller/command    std_msgs/Float64 "data: 1.0"
