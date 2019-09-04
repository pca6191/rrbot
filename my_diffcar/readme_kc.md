### 說明
本 pcackage 示範：  

* 使用 xacro 巨集
* 純粹使用 gazebo 的 pluging: libgazebo_ros_diff_drive.so，來控制兩輪車。 

### 使用

帶起 launch file

```
    $ roslaunch my_diffcar diffcar.launch
```

其中同時帶起 

    teleop_twist_keyboard node
    
依據 consol 畫面訊息，可用鍵盤控制車子運動。