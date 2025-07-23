import os
import re

PKG_PATH = "/home/tengxun/lab/pinocchio_ws/src/MoveTo"

# 读取URDF
with open("/home/tengxun/lab/pinocchio_ws/src/MoveTo/description/jaka/right_jaka.urdf") as f:
    urdf = f.read()

# 替换路径：package://pkg/xxx -> /abs_path/xxx
fixed_urdf = re.sub(
    r'package://([\w_]+)/(.*?)"',
    lambda m: f'{os.path.join(PKG_PATH, m.group(2))}"',
    urdf
)

# 保存处理后的URDF
with open("/home/tengxun/lab/pinocchio_ws/src/MoveTo/description/jaka/right_jaka_renamed.urdf", "w") as f:
    f.write(fixed_urdf)

print("URDF路径修复完成！")