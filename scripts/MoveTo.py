import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

from obvp_planner import ObvpPlanner

def draw(t_list, q_list, v_list, a_list, cmd_list):
    fig, axes = plt.subplots(3, 7, figsize=(21, 9))
    plt.subplots_adjust(wspace=0.3, hspace=0.4)

    for j in range(7):
        px = axes[0, j]
        vx = axes[1, j]
        ax = axes[2, j]

        px.set_title(f't-q({j}), t-cmd_q({j})', fontsize=10)


        px.plot(t_list, q_list[:, j], label="q")
        px.plot(t_list, cmd_list[:, j], label="cmd_q")
        vx.plot(t_list, v_list[:, j], label="v")
        ax.plot(t_list, a_list[:, j], label="a")

        px.grid(alpha=0.3)
        vx.grid(alpha=0.3)
        ax.grid(alpha=0.3)
    
    plt.suptitle('3x7 Subplot Layout Demonstration', fontsize=18, y=0.98)
    # plt.savefig('3x7_subplots.png', dpi=150, bbox_inches='tight')
    plt.tight_layout(pad=3.0)
    plt.show()

if __name__ == "__main__":
    rospy.init_node("MoveTo")
    state_publisher = rospy.Publisher("/mujoco/right_arm/cmd", Float32MultiArray, queue_size=10)

    dof = 7
    initial_state = np.zeros((3, dof), dtype=np.float64)
    max_vel = np.array([3.0] * dof, dtype=np.float64)
    max_acc = np.array([5.0] * dof, dtype=np.float64)
    weight_T = 0.1

    planner = ObvpPlanner(initial_state, dof, max_vel, max_acc, weight_T)

    t_list = []
    p_list = []
    v_list = []
    a_list = []
    cmd_list = []

    t = 0
    dt = 0.05
    rate = rospy.Rate(1/dt)
    temp_tar_state = np.random.uniform(low=-np.pi, high=np.pi, size=(1, dof))

    tar_state = np.zeros((3, dof))
    tar_state[0, :] = temp_tar_state
    i = 0
    pts = 0
    while rospy.is_shutdown() == False:
        if dt*i >= planner.getT():
            i = 0
            pts += 1

            temp_tar_state = np.random.uniform(low=-0.5*np.pi, high=0.5*np.pi, size=(1, dof))
            if pts > 3:
                break                
            else:
                tar_state[0, :] = np.random.uniform(low=-0.5*np.pi, high=0.5*np.pi, size=(1, dof))
        
        output_q = planner.getCurrentOutput_EPVA(tar_state, dt)
        current_state = planner.getCurrentState()
        t_list.append(t)
        p_list.append(np.array(current_state[0, :]))
        v_list.append(np.array(current_state[1, :]))
        a_list.append(np.array(current_state[2, :]))
        cmd_list.append(np.array(tar_state[0, :]))
        
        output_q = output_q.astype(np.float32)
        msg = Float32MultiArray(data=output_q)
        state_publisher.publish(msg)
        i += 1
        t += dt
        rate.sleep()

    draw(np.array(t_list), np.array(p_list), np.array(v_list), np.array(a_list), np.array(cmd_list))