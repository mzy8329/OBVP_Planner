import mujoco
import os

model = mujoco.MjModel.from_xml_path("/home/user/lab/pinocchio_ws/src/OBVP_Planner/description/jaka/right_jaka.urdf")
model.save("/home/user/lab/pinocchio_ws/src/OBVP_Planner/description/jaka/right_jaka.xml")