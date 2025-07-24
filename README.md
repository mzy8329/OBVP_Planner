# MoveTo
## 启动mujoco
需要先启动roscore
```shell
pip install mujoco-py
python3 ./scripts/start_mujoco.py
```
如果pip的时候提示找不到#include <Eigen/Core>
可以通过
```shell
sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
```
添加软链接

## obvp使用指南
### C语言
可见src/MoveTo.cpp

### Python
运行
```shell
rm -r ./include/obvp_planner/build && rm -r ./include/obvp_planner/obvp_planner.egg-info &&
pip install ./include/obvp_planner --verbose
```
即可将obvp_planner安装到python中

具体使用可见scripts/MoveTo.py
