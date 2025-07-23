import mujoco
import mujoco.viewer
import numpy as np
import time

import rospy
from std_msgs.msg import Float32MultiArray

arm_pose = np.array([0, 0, 0, 0, 0, 0, 0])
def rightArmCmdCallback(msg):
    global arm_pose
    arm_pose = np.array(msg.data)

if __name__ == "__main__":
    rospy.init_node("mujoco_sim")
    state_publisher = rospy.Publisher("/mujoco/right_arm/pose", Float32MultiArray, queue_size=10)
    cmd_subscriber = rospy.Subscriber("/mujoco/right_arm/cmd", Float32MultiArray, rightArmCmdCallback)

    right_jaka_xml = "/home/tengxun/lab/pinocchio_ws/src/MoveTo/description/jaka/right_jaka.xml"
    model = mujoco.MjModel.from_xml_path(right_jaka_xml)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.sync()
        
        rate = rospy.Rate(100)
        while viewer.is_running() and rospy.is_shutdown() == False:
            state_publisher.publish(Float32MultiArray(data=data.qpos))        
            rate.sleep()

            data.ctrl[:] = arm_pose
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)