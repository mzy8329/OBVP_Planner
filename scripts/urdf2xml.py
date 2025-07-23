import mujoco
import os

model = mujoco.MjModel.from_xml_path("/home/tengxun/lab/pinocchio_ws/src/MoveTo/description/jaka/right_jaka.urdf")
model.save("/home/tengxun/lab/pinocchio_ws/src/MoveTo/description/jaka/right_jaka.xml")