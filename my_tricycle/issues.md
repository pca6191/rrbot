### issues

* gazebo pid 有設值，所有 joints 會崩壞、集中在 footprint 處

    # for gazebol plug-in libgazebo_ros_control.so ---------------
    /gazebo_ros_control:   
      pid_gains:
        steer_wheel_joint:
          p: 0.0
          i: 0.0 
          d: 0.0
        disk_link_joint:
          p: 0.0
          i: 0.0 
          d: 0.0
          
  tricycle.yaml 加入述，launch 起來不再有以下 error
  
      [ERROR] [1568077992.025833959, 0.333000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/disk_link_joint
      [ERROR] [1568077992.028633026, 0.333000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/steer_wheel_joint
      
  但是 gazebo 內所有 joints 會崩壞、集中在 footprint 處，原因不明？
      
      
### 解法
* 使用 libgazebo_ros_control.so，不要設置 /gazebo_ros_control: ... pid_gains
* 雖然會出現上述 error，但表示 DefaultRobotHWSim::writeSim 直接收 controller command,
  並且透傳設入 gazebo model (PID 計算由 controller 處理， writeSim 不再計算一次)：
  
      DefaultRobotHWSim::writeSim(...) {
       ...
      case VELOCITY:
        sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
      }