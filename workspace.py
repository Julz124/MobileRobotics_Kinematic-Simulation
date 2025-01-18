import numpy as np
import matplotlib.pyplot as plt
from roboter import Roboter
import transformations as tf

class Workspace:
    '''
    @staticmethod
    def forward_kinematics(l1, l2, beta_1, beta_2):
        x = l1 * np.cos(beta_1) + l2 * np.cos(beta_1 + beta_2)
        z = l1 * np.sin(beta_1) + l2 * np.sin(beta_1 + beta_2)
        return x, z
    '''

    @staticmethod
    def forward_kinematics(l1, l2, beta_1, beta_2):
        T0 = tf.trans([0, 0])
        R1 = tf.rot2trans(tf.rot(beta_1))
        TR0 = np.matmul(T0, R1)

        T1 = tf.trans([l1, 0])
        TR1 = np.matmul(TR0, T1)

        R2 = tf.rot2trans(tf.rot(beta_2))
        TR2 = np.matmul(TR1, R2)

        T2 = tf.trans([l2, 0])
        TR3 = np.matmul(TR2, T2)

        x, z = TR3[0, 2], TR3[1, 2]
        
        return x, z


    @staticmethod
    def calculate_workspace(self, robot, res):

        base_x = robot.l / 2
        base_z = robot.h

        beta_1_range = np.radians(np.linspace(robot.beta_1_limits[0], robot.beta_1_limits[1], res))
        beta_2_range = np.radians(np.linspace(robot.beta_2_limits[0], robot.beta_2_limits[1], res))

        with open("output/beta_1.txt", "a") as f:
            print(beta_1_range, file=f)
        with open("output/beta_2.txt", "a") as f:
            print(beta_2_range, file=f)

        workspace = []
        
        for beta_1 in beta_1_range:
            for beta_2 in beta_2_range:
                x, z = self.forward_kinematics(robot.l1, robot.l2, beta_1, beta_2)
                x += base_x
                z += base_z
                workspace.append((x, z))
        return workspace, base_x, base_z

def plot():
    robot = Roboter()
    ws = Workspace()

    workspace, base_x, base_z = ws.calculate_workspace(ws, robot, 100)

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
        x, z = ws.forward_kinematics(robot.l1, robot.l2, beta_1, beta_2)
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

if __name__ == "__main__":
    plot()

