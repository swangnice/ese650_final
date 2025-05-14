#run in conda(dialmpc)

import time
import numpy as np
import mujoco
import mujoco.viewer
import glfw

import matplotlib.pyplot as plt

import os
os.environ["MUJOCO_GL"] = "egl"

# 1. Load model and data
model = mujoco.MjModel.from_xml_path(
    "/home/swang/Documents/ese650_final/forked/dial-mpc/"
    "dial_mpc/models/unitree_h1/h1_loco.xml"
)
data = mujoco.MjData(model)

# 2. Load state sequence
#    Each row: [step, qpos (nq=18), qvel (nv=17), ctrl (nu=11)]
states = np.load(
    "/home/swang/Documents/ese650_final/forked/dial-mpc/"
    "unitree_h1_loco/20250513-180312_states.npy"
)
nq, nv, nu = model.nq, model.nv, model.nu

# 3. Launch passive viewer (non-blocking, customizable timing)
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Iterate through each frame
    for frame in states:
        if not viewer.is_running():
            break

        with viewer.lock():
            data.qpos[:] = frame[1 : 1 + nq]  # Set qpos
            data.qvel[:] = frame[1 + nq : 1 + nq + nv]  # Set qvel
            data.ctrl[:] = frame[1 + nq + nv : 1 + nq + nv + nu]  # Set ctrl

        # Compute new geometric positions
        mujoco.mj_forward(model, data)

        viewer.sync()

        time.sleep(1 / 50)  # 50 FPS

# 4. Close viewer
viewer.close()
glfw.terminate()

print("Viewer closed. Plotting data...")

steps = states[:, 0]
nv = 17  
nu = 11   

# Extract control and velocity data
ctrl = states[:, -nu:]                             # Last 11 columns (control)
qvel = states[:, -(nv+nu):-nu]                     # Velocity columns

# 1. Compute control cost = sum(ctrl**2) and plot
control_cost = np.sum(ctrl**2, axis=1)
plt.figure()
plt.plot(steps, control_cost)
plt.xlabel("Step")
plt.ylabel("Control cost\n(∑ u_i²)")
plt.title("Control Cost vs Step")

vx = qvel[:, 0]
vy = qvel[:, 1]
vz = qvel[:, 2]

# 2. Plot linear velocity components
plt.figure()
plt.plot(steps, vx, label='v_x')
plt.plot(steps, vy, label='v_y')
plt.plot(steps, vz, label='v_z')
plt.xlabel("Step")
plt.ylabel("Linear Velocity (m/s)")
plt.title("Linear Velocity Components vs Step")
plt.legend()

# 3. Compute and plot jerk in x-direction
ax = np.diff(vx)  # Acceleration in x
jerk_x = np.diff(ax)  # Jerk in x
jerk_steps = steps[2:]  # Adjust steps for jerk

plt.figure()
plt.plot(jerk_steps, jerk_x)
plt.xlabel("Step")
plt.ylabel("Jerk in x (m/s³)")
plt.title("X-direction Jerk vs Step")
plt.show()
