#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control agv!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1, 0, 0),
        'o':(1, 0, -1),
        'j':(0, 1, 0),
        'l':(0, -1, 0),
        'u':(1, 0, 1),
        ',':(-1,0, 0),
        '.':(-1, 0, 1),
        'm':(-1, 0, -1),
           }

speedBindings={
        'q':(1.1, 1.1),
        'z':(.9, .9),
        'w':(1.1, 1),
        'x':(.9, 1),
        'e':(1, 1.1),
        'c':(1, .9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    node_name = 'mecanum_teleop'
    
    rospy.init_node(node_name)

    twist_topic = rospy.get_param('/mecanum_teleop/twist_topic', '/cmd_vel')
    rospy.loginfo("[%s] twist_topic: %s", node_name, twist_topic)

    pub = rospy.Publisher(twist_topic, Twist, queue_size=5)

    x = 0
    y = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed_x = 0
    target_speed_y = 0
    target_turn = 0
    control_speed_x = 0
    control_speed_y = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key == 'k' :
                x = 0
                y = 0
                th = 0
                control_speed_x = 0
                control_speed_y = 0 
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0;
                    th = 0
                if (key == '\x03'):
                    break

            # 目标速度=速度值*方向值
            target_speed_x = speed * x
            target_speed_y = speed * y
            target_turn = turn * th

            # 速度限位，防止速度增减过快
            if target_speed_x > control_speed_x:
                control_speed_x = min( target_speed_x, control_speed_x + 0.02 )
            elif target_speed_x < control_speed_x:
                control_speed_x = max( target_speed_x, control_speed_x - 0.02 )
            else:
                control_speed_x = target_speed_x

            if target_speed_y > control_speed_y:
                control_speed_y = min( target_speed_y, control_speed_y + 0.02 )
            elif target_speed_y < control_speed_y:
                control_speed_y = max( target_speed_y, control_speed_y - 0.02 )
            else:
                control_speed_y = target_speed_y

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = control_speed_x; 
            twist.linear.y = control_speed_y; 
            twist.linear.z = 0
            twist.angular.x = 0; 
            twist.angular.y = 0; 
            twist.angular.z = control_turn
            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
