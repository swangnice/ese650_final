#run in conda(dialmpc)

import time
import numpy as np
import mujoco
import mujoco.viewer
import glfw
import pandas as pd

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
states_0_5 = np.load(
    "/home/swang/Documents/ese650_final/forked/dial-mpc/unitree_h1_loco/20250515-192611_states_0_5.npy"
)

states_1_0 = np.load(
    "/home/swang/Documents/ese650_final/forked/dial-mpc/unitree_h1_loco/20250515-192829_states.npy"
)

states_1_5 = np.load(
    "/home/swang/Documents/ese650_final/forked/dial-mpc/unitree_h1_loco/20250515-193129_states.npy"
)

states_2_0 = np.load(
    "/home/swang/Documents/ese650_final/forked/dial-mpc/unitree_h1_loco/20250515-193524_states.npy"
)

protomotions_cc = pd.read_csv("/home/swang/Documents/ese650_final/forked/ProtoMotions/protomotions_cost.csv")  
protomotions_vel = pd.read_csv("/home/swang/Documents/ese650_final/forked/ProtoMotions/protomotions_vel.csv")  
metrics          = pd.read_csv("/home/swang/Documents/ese650_final/forked/unitree_rl_gym/metrics.csv")  
nq, nv, nu = model.nq, model.nv, model.nu

# 3. Launch passive viewer (non-blocking, customizable timing)
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Iterate through each frame
    for frame in states_2_0:
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


datasets = {
    "ALMI_RL": states_0_5,
    "DIAL MPC V=1m/s": states_1_0,
    "HOVER": states_1_5,
    "hb": states_2_0
}

nq, nv, nu = model.nq, model.nv, model.nu

# Pre-create two figures
plt.figure(figsize=(8, 5))
for label, states in datasets.items():
    steps = states[:, 0]
    # Control inputs are in the last nu columns
    ctrl = states[:, -nu:]
    control_cost = np.sum(ctrl**2, axis=1)
    plt.plot(steps, control_cost, label=label)

plt.plot(protomotions_cc["step"], protomotions_cc["control_cost"], 
         linestyle='--', label="Protomotions Control Cost")
plt.plot(metrics["step"], metrics["control_cost"],
         linestyle='-.', label="unitree_rl_gym Control Cost")
plt.xlabel("Step")
plt.ylabel("Control Cost (∑ u_i²)")
plt.title("Control Cost Comparison")
plt.legend()
plt.tight_layout()

plt.figure(figsize=(8, 5))
for label, states in datasets.items():
    steps = states[:, 0]
    # qvel is located between the last (nv+nu) and the last nu columns
    qvel = states[:, -(nv+nu):-nu]
    vx = qvel[:, 0]
    vy = qvel[:, 1]
    # Compute horizontal velocity magnitude
    vel_mag = np.sqrt(vx**2 + vy**2)
    plt.plot(steps, vel_mag, label=label)

vel_mag_csv = np.sqrt(protomotions_vel["vel_x"]**2 + protomotions_vel["vel_y"]**2)
plt.plot(protomotions_vel["step"], vel_mag_csv, 
         linestyle='--', label="Protomotions Speed")

vel_m = np.sqrt(metrics["vel_x"]**2 + metrics["vel_y"]**2)
plt.plot(metrics["step"], vel_m,
         linestyle='-.', label="unitree_rl_gym Speed")


plt.xlabel("Step")
plt.ylabel("Horizontal Speed (m/s)")
plt.title("Horizontal Velocity Magnitude Comparison")
plt.legend()
plt.tight_layout()


plt.figure(figsize=(8, 5))
for label, states in datasets.items():
    steps = states[:, 0]
    # Extract linear velocity vx
    qvel = states[:, -(nv+nu):-nu]
    vx = qvel[:, 0]
    # Compute acceleration ax and jerk
    ax = np.diff(vx)              # Δvx → ax
    jerk_x = np.diff(ax)          # Δax → jerk
    jerk_steps = steps[2:]        # Steps corresponding to jerk (drop the first two)

    plt.plot(jerk_steps, jerk_x, label=label)

vx_pm = protomotions_vel["vel_x"].to_numpy()
ax_pm = np.diff(vx_pm)
jerk_csv = np.diff(ax_pm)
jerk_steps_csv = protomotions_vel["step"].to_numpy()[2:]

plt.plot(jerk_steps_csv, jerk_csv,
         linestyle='--', label="Protomotions Jerk")    

vx_m    = metrics["vel_x"].to_numpy()
ax_m    = np.diff(vx_m)
jerk_m  = np.diff(ax_m)
jerk_s_m = metrics["step"].to_numpy()[2:]
plt.plot(jerk_s_m, jerk_m,
         linestyle='-.', label="unitree_rl_gym Jerk")

plt.xlabel("Step")
plt.ylabel("Jerk in x (m/s³)")
plt.title("X-direction Jerk Comparison")
plt.legend()
plt.tight_layout()
plt.show()



# steps = states_0_5[:, 0]
# nv = 17  
# nu = 11   

# # Extract control and velocity data
# ctrl = states_0_5[:, -nu:]                             # Last 11 columns (control)
# qvel = states_0_5[:, -(nv+nu):-nu]                     # Velocity columns

# # 1. Compute control cost = sum(ctrl**2) and plot
# control_cost = np.sum(ctrl**2, axis=1)
# plt.figure()
# plt.plot(steps, control_cost)
# plt.xlabel("Step")
# plt.ylabel("Control cost\n(∑ u_i²)")
# plt.title("Control Cost vs Step")

# vx = qvel[:, 0]
# vy = qvel[:, 1]
# vz = qvel[:, 2]

# # 2. Plot linear velocity components
# plt.figure()
# plt.plot(steps, vx, label='v_x')
# plt.plot(steps, vy, label='v_y')
# plt.plot(steps, vz, label='v_z')
# plt.xlabel("Step")
# plt.ylabel("Linear Velocity (m/s)")
# plt.title("Linear Velocity Components vs Step")
# plt.legend()

# # 3. Compute and plot jerk in x-direction
# ax = np.diff(vx)  # Acceleration in x
# jerk_x = np.diff(ax)  # Jerk in x
# jerk_steps = steps[2:]  # Adjust steps for jerk

# plt.figure()
# plt.plot(jerk_steps, jerk_x)
# plt.xlabel("Step")
# plt.ylabel("Jerk in x (m/s³)")
# plt.title("X-direction Jerk vs Step")
# plt.show()
