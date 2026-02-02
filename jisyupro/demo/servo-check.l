# terminal 0 (pre-confirmation)
source ~/work/jisyupro/servo-check/rcb4-venv/bin/activate
python3 -i -m rcb4.armh7interface

interface.search_servo_ids()
interface.angle_vector()


# terminal 1
source ~/ros_ws/devel/setup.bash
roslaunch jedy_ros1_bridge jedy_bridge.launch

# terminal 2
source ~/ros_ws/devel/setup.bash
roscd jedyeus/euslisp
roseus jedy-interface.l

## うまくいかない
(jedy-init) 
(ros::rate 10) ;; ループの周期を10Hzに設定する
(do-until-key ;; Enterキーが押されるまでループする
            (send *ri* :spin-once) ;; subscribeしているトピックの更新が行われる．
            (send *jedy* :angle-vector (send *ri* :state :potentio-vector))
            (ros::ros-info (format nil "Current head pitch joint angle ~A [deg]" (send *jedy* :rarm_joint1)))
            (ros::sleep))

##  rarm_joint0: {id: 0, direction: 1, offset: 0} -> servo-0 staright-direction 0  from
##  rarm_joint1: {id: 2, direction: 1, offset: 0} -> servo-1 staright-direction 0  from -90 to 30
##  rarm_joint2: {id: 4, direction: 1, offset: 0} -> servo-2 staright-direction 55 from

## example
(send *ri* :servo-on :names (list "head_joint0" "head_joint1"))
(send *ri* :servo-on :names (list "rarm_joint1" "rarm_joint2"))

## move joint1
(jedy-init)


(send *ri* :servo-on :names (list "rarm_joint1" ))
(send *jedy* :rarm_joint1 :joint-angle 0) ;dammy
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *jedy* :rarm_joint1 :joint-angle -90)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *jedy* :rarm_joint1 :joint-angle 0)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *ri* :servo-off)
(send *ri* :servo-off)

(send *ri* :servo-on :names (list "rarm_joint2" ))
(send *jedy* :rarm_joint2 :joint-angle 50) ;dammy
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *jedy* :rarm_joint2 :joint-angle 140)
(send *ri* :angle-vector (send *jedy* :angle-vector) 1000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *jedy* :rarm_joint2 :joint-angle 50)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *ri* :servo-off)
(send *ri* :servo-off)

(send *ri* :servo-on :names (list "rarm_joint0" ))
(send *jedy* :rarm_joint0 :joint-angle 0) ;dammy
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *jedy* :rarm_joint0 :joint-angle 90)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *jedy* :rarm_joint0 :joint-angle 0)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :default-controller 0 :min-time 0.1)
(send *ri* :wait-interpolation)
(send *ri* :servo-off)
(send *ri* :servo-off)


(send *ri* :servo-off)