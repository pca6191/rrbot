### 啟動程式

 * $ roslaunch my_reachtruck reachtruck.launch  
 * 以簡單的關節，搭配 parser 接收 ROS basecontrol 傳來的叉車封包，模擬 reach 叉車做動。

### dependency

* 技巧：

```
To make Gazebo die sooner, edit the file at /opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py:

Near line 57, change the timeouts to be:

_TIMEOUT_SIGINT  = 0.5 #seconds
_TIMEOUT_SIGTERM = 0.5 #seconds
This will cause ROS to send a kill signal much sooner.
```

### 轉動旋臂


