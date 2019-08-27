### 出處：
gazebo 內使用 ROS control 教學：  

    http://gazebosim.org/tutorials/?tut=ros_control

教學程式碼下載處：
  
    https://github.com/ros-simulation/gazebo_ros_demos.git

### 使用
參考 README.md  

Rviz 顯示:

    roslaunch rrbot_description rrbot_rviz.launch

Gazebo 顯示:

    roslaunch rrbot_gazebo rrbot_world.launch

ROS Control 加入控制能力:

    roslaunch rrbot_control rrbot_control.launch

控制轉臂 2, example:

    rostopic pub /rrbot/joint2_position_controller/command std_msgs/Float64 "data: 1.5"
    
### 修改與說明
模型計有
 * 立柱
 * 轉臂 1
 * 轉臂 2
 
修改為
 * 立柱
 * 滑臂 1 ：練習貨叉升降控制
 * 轉臂 2
 
 修改處 (rrbot.xacro)：
 
 原為 (轉臂 1)  
 
 ```
   <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 ${width} ${height1 - axel_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.7"/>
  </joint>
  ```
  
 改為 (滑臂 1)
 
 ```
  <joint name="joint1" type="prismatic">
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="1000.0" velocity="0.1" lower="-1.0" upper="1.0"/>
    <origin xyz="0 ${width} ${height1/2}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  ```
  
  其中：
  
  * <origin> 表示活動軸起點位置，相對於 parent link.
  * <axis xyz="0 0 1"> 表示沿著 z 軸，上下滑動
  * velocity 表示上推速度，lower / upper 表示滑動上下限，相對於 parent link.
  * 下達下列指令，可以將滑臂升到上限 ... -1.0 為下限...其餘類推
  
  ```
   rostopic pub -1 /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 1.0" 
  ```
   
   
  
 