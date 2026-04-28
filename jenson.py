import mujoco
import numpy as np
import matplotlib.pyplot as plt
import imageio

# 1. Highly Tuned Physics XML
# - Internal collisions disabled (contype="0" conaffinity="0")
# - High solver iterations (200) for strict constraint adherence
# - Critical damping on joints and constraints
xml_string = """
<mujoco model="Smooth Dynamic Strandbeest">
    <compiler angle="degree" coordinate="local"/>
    <option timestep="0.002" iterations="200" tolerance="1e-6" gravity="0 0 -9.81"/>
    
    <default>
        <geom contype="0" conaffinity="0" friction="0 0 0"/>
        <joint damping="0.5" limited="false"/>
    </default>

    <asset>
        <material name="bone" rgba="0.8 0.8 0.7 1"/>
        <material name="crank_mat" rgba="0.8 0.2 0.2 1"/>
        <material name="joint_mat" rgba="0.2 0.2 0.2 1"/>
    </asset>

    <worldbody>
        <geom type="plane" size="4 4 0.1" rgba="0.9 0.9 0.9 1" contype="1" conaffinity="1"/>
        <light pos="0 0 5" dir="0 0 -1"/>
        
        <body name="suspension_base" pos="0 0 1.5">
            <geom type="cylinder" size="0.03 0.1" material="joint_mat" euler="90 0 0"/>
            
            <body name="crank" pos="0 0 0">
                <joint name="crank_motor_joint" type="hinge" axis="0 1 0" damping="1.0"/>
                <geom type="capsule" fromto="0 0 0  0.15 0 0" size="0.015" material="crank_mat"/>
                
                <body name="link_j" pos="0.15 0 0">
                    <joint name="j_pivot" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" fromto="0 0 0  -0.3 0 0.4" size="0.015" material="bone"/>
                    <site name="loop1_A" pos="-0.3 0 0.4"/>
                </body>
                
                <body name="link_k" pos="0.15 0 0">
                    <joint name="k_pivot" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" fromto="0 0 0  -0.4 0 -0.47" size="0.015" material="bone"/>
                    <site name="loop2_A" pos="-0.4 0 -0.47"/>
                </body>
            </body>

            <body name="pivot_B" pos="-0.38 0 -0.078">
                <geom type="cylinder" size="0.03 0.1" material="joint_mat" euler="90 0 0"/>
                
                <body name="link_b" pos="0 0 0">
                    <joint name="b_pivot" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" fromto="0 0 0  0.25 0 0.33" size="0.015" material="bone"/>
                    <site name="loop1_B" pos="0.25 0 0.33"/>
                    
                    <body name="link_e" pos="0.25 0 0.33">
                        <joint name="e_pivot" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" fromto="0 0 0  -0.558 0 0" size="0.015" material="bone"/>
                        <site name="loop3_A" pos="-0.558 0 0"/>
                        
                        <body name="link_f" pos="-0.558 0 0">
                            <joint name="f_pivot" type="hinge" axis="0 1 0"/>
                            <geom type="capsule" fromto="0 0 0  0.2 0 -0.34" size="0.015" material="bone"/>
                            <site name="loop4_A" pos="0.2 0 -0.34"/>
                            
                            <body name="link_h" pos="0.2 0 -0.34">
                                <joint name="h_pivot" type="hinge" axis="0 1 0"/>
                                <geom type="capsule" fromto="0 0 0  0.2 0 -0.62" size="0.015" material="bone"/>
                                <geom name="foot_contact" type="sphere" pos="0.2 0 -0.62" size="0.03" rgba="0 1 0 1" contype="1" conaffinity="1" friction="1 0.005 0.0001"/>
                                <site name="foot_G" pos="0.2 0 -0.62"/>
                            </body>
                        </body>
                    </body>
                </body>
                
                <body name="link_c" pos="0 0 0">
                    <joint name="c_pivot" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" fromto="0 0 0  0.1 0 -0.38" size="0.015" material="bone"/>
                    <site name="loop2_B" pos="0.1 0 -0.38"/>
                    <site name="loop4_B" pos="0.1 0 -0.38"/>
                    
                    <body name="link_i" pos="0.1 0 -0.38">
                        <joint name="i_pivot" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" fromto="0 0 0  -0.1 0 -0.48" size="0.015" material="bone"/>
                        <site name="loop_foot" pos="-0.1 0 -0.48"/>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <equality>
        <connect site1="loop1_A" site2="loop1_B" solref="0.01 1"/>
        <connect site1="loop2_A" site2="loop2_B" solref="0.01 1"/>
        <connect site1="loop4_A" site2="loop4_B" solref="0.01 1"/> 
        <connect site1="foot_G" site2="loop_foot" solref="0.01 1"/>
    </equality>

    <actuator>
        <velocity name="motor" joint="crank_motor_joint" kv="15" ctrlrange="-10 10"/>
    </actuator>
</mujoco>
"""

# 2. Setup and Engine Initialization
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, height=480, width=640)

# Allow the physics engine to settle the constraints before we start
print("Settling physics constraints to prevent snap...")
for _ in range(500):
    mujoco.mj_step(model, data)

# --- THE CHANGES ARE HERE ---
duration = 12.0  # Increased to 12 seconds for ~6 full walking cycles
framerate = 60
frames = []
traj_x, traj_z = [], []

site_id = model.site('foot_G').id

# Drive the crank at a steady 3 rad/s
data.ctrl[0] = 3.0 

print("Running extended smooth dynamic simulation (12 seconds)...")

# 3. Core Simulation Loop
while data.time < duration:
    mujoco.mj_step(model, data)
    
    foot_pos = data.site_xpos[site_id]
    traj_x.append(foot_pos[0])
    traj_z.append(foot_pos[2])
    
    if len(frames) < (data.time * framerate):
        renderer.update_scene(data, camera=-1)
        frames.append(renderer.render())

print("Simulation complete. Writing files...")

# 4. Export Video
video_file = "perfect_smooth_jansen_long.mp4"
imageio.mimsave(video_file, frames, fps=framerate)
print(f"Long video saved: {video_file}")

# 5. Export Dense Plot
plt.figure(figsize=(8, 5))
# Added alpha=0.5 so overlapping repetitions create a darker, richer line
plt.plot(traj_x, traj_z, color='#2c3e50', linewidth=1.5, alpha=0.5)
plt.title("Theo Jansen Point G Trajectory (6 Cycles)")
plt.xlabel("X Position (m)")
plt.ylabel("Z Position (m)")
plt.axis('equal')
plt.grid(True, linestyle='--', alpha=0.7)
plt.savefig("perfect_smooth_trajectory_long.png", dpi=300)
print("Data-rich trajectory plot saved: perfect_smooth_trajectory_long.png")