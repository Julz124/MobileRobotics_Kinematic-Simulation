import numpy as np
import matplotlib.pyplot as plt
from roboter import Roboter

robot = Roboter()

base_x = robot.l / 2
base_z = robot.h

beta_1_range = np.radians(np.linspace(robot.beta_1_limits[0], robot.beta_1_limits[1], 100))
beta_2_range = np.radians(np.linspace(robot.beta_2_limits[0], robot.beta_2_limits[1], 100))

def forward_kinematics(l1, l2, beta_1, beta_2):
    x = l1 * np.cos(beta_1) + l2 * np.cos(beta_1 + beta_2)
    z = l1 * np.sin(beta_1) + l2 * np.sin(beta_1 + beta_2)
    return x, z

def calculate_workspace(robot, beta_1_range, beta_2_range):
    workspace = []
    for beta_1 in beta_1_range:
        for beta_2 in beta_2_range:
            x, z = forward_kinematics(robot.l1, robot.l2, beta_1, beta_2)
            x += base_x
            z += base_z
            workspace.append((x, z))
    return workspace

workspace = calculate_workspace(robot, beta_1_range, beta_2_range)

x_vals = [pos[0] for pos in workspace]
z_vals = [pos[1] for pos in workspace]

plt.figure(figsize=(8, 8))
plt.scatter(x_vals, z_vals, s=1, color='b', label='Workspace')

plt.plot(base_x, base_z, 'go', label=f'Base ({base_x:.2f}, {base_z:.2f})')

extreme_positions = []
extreme_angles = [
    (np.radians(robot.beta_1_limits[0]), np.radians(robot.beta_2_limits[0])),
    (np.radians(robot.beta_1_limits[1]), np.radians(robot.beta_2_limits[1]))
]

for beta_1, beta_2 in extreme_angles:
    x, z = forward_kinematics(robot.l1, robot.l2, beta_1, beta_2)
    x += base_x
    z += base_z
    extreme_positions.append((x, z))

    plt.plot([base_x, base_x + robot.l1 * np.cos(beta_1)], [base_z, base_z + robot.l1 * np.sin(beta_1)], 'r-', label=f'Arm Part 1 @{np.degrees(beta_1):.1f}°')
    plt.plot([base_x + robot.l1 * np.cos(beta_1), x], [base_z + robot.l1 * np.sin(beta_1), z], 'g-', label=f'Arm Part 2 @{np.degrees(beta_2):.1f}°')

plt.title('Workspace of the Robotic Arm (Side View) with Arm Extremes')
plt.xlabel('X Position')
plt.ylabel('Z Position')
plt.grid(True)
plt.axis('equal')

plt.legend()
plt.show()