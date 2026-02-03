# source ~/kxr/bin/activate
# source ~/ros_ws/devel/setup.bash
# ipython -i -- swing.py

import argparse
import time

import IPython
from kxr_controller.check_ros_master import is_ros_master_local
from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np  # NOQA
import rospy
from skrobot.model import RobotModel
from skrobot.viewers import PyrenderViewer

rospy.init_node("kxr_interface", anonymous=True)
robot_model = RobotModel()
robot_description = "/robot_description"
robot_model.load_urdf_from_robot_description(robot_description)
viewer = PyrenderViewer(resolution=(640, 480))
viewer.add(robot_model)
viewer.show()

namespace = None
ri = KXRROSRobotInterface(
    robot_model, namespace, controller_timeout=10.0
)


# servo onは
#ri.servo_on()

# ri.servo_off()
# 今の姿勢を作りたいときはri.angle_vector()で実機の姿勢が来る
# ri.servo_off()
# ri.angle_vector()
""""
Warn: old version
# -90,-90,-90 -> 1.777749, 0.52    , 2.35
# 0,0,0 ->  0.06361743, -0.06184993,  1.063822 
# 90,90,90 -> -1.5774721, -1.5338825, -0.5843361
Warn: old version
"""
 
# -90,-90,-90  右利きでラケット引く方向 ->  
#   0,  0,  0  すべてのサーボまっすぐ   ->  
#  90, 90, 90  右利きでラケット水ing   -> 

# 1.5=90度
# servo[1]はバグってるので-2.35~0.52のみなので-0.6をまっすぐとする
# servo[1]は-2.35~2.35のみなので0.25をまっすぐとする

# 0.688009  ,  0.21015227, -0.3753621 いいかんじ  11V 0.01
# 0.45121145, 0.52      , 0.02768546 やばい
before_array= [0.688009  ,  0.21015227, -0.3753621 ]
robot_model.rarm_joint0.joint_angle(before_array[0])
robot_model.rarm_joint1.joint_angle(before_array[1])
robot_model.rarm_joint2.joint_angle(before_array[2])
before_swing = robot_model.angle_vector()

# 0.13090725, -0.10779573, -0.8765042　いいかんじ　11V 0.01
# 0.24268822, -0.1849611 , -0.7333654   やばい
after_array= [  0.13090725, -0.10779573, -0.8765042 ]
robot_model.rarm_joint0.joint_angle(after_array[0])
robot_model.rarm_joint1.joint_angle(after_array[1])
robot_model.rarm_joint2.joint_angle(after_array[2])
after_swing = robot_model.angle_vector()



ri.servo_on()

speed = 0.01
# swing speed time

for i in range(1):
    
    robot_model.angle_vector(before_swing)
    ri.angle_vector(robot_model.angle_vector(), 5)
    ri.wait_interpolation()
    
    robot_model.angle_vector(after_swing)
    ri.angle_vector(robot_model.angle_vector(), speed)
    ri.wait_interpolation()
    
    time.sleep(3.0)

ri.servo_off()